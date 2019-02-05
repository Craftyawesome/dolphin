// Copyright 2018 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#include "InputCommon/ControllerEmu/StickGate.h"

#include <cmath>

#include "Common/Common.h"
#include "Common/MathUtil.h"
#include "Common/Matrix.h"
#include "Common/StringUtil.h"
#include "InputCommon/ControllerEmu/Control/Control.h"
#include "InputCommon/ControllerEmu/Setting/NumericSetting.h"

constexpr auto CALIBRATION_CONFIG_NAME = "Calibration";
constexpr auto CALIBRATION_DEFAULT_VALUE = 1.0;
constexpr auto CALIBRATION_CONFIG_SCALE = 100;

// This is the number of samples we generate but any number could be loaded from config.
constexpr int CALIBRATION_SAMPLE_COUNT = 32;

namespace
{
// Calculate distance to intersection of a ray with a line defined by two points.
double GetRayLineIntersection(Common::DVec2 ray, Common::DVec2 point1, Common::DVec2 point2)
{
  const auto diff = point2 - point1;

  const auto dot = diff.Dot({-ray.y, ray.x});
  if (std::abs(dot) < 0.00001)
  {
    // Handle situation where both points are on top of eachother.
    // This could occur if the user configures a single "input radius" calibration value
    // or when updating calibration.
    return point1.Length();
  }

  return diff.Cross(-point1) / dot;
}

Common::DVec2 GetPointFromAngleAndLength(double ang, double length)
{
  return Common::DVec2{std::cos(ang), std::sin(ang)} * length;
}
}  // namespace

namespace ControllerEmu
{
std::optional<u32> StickGate::GetIdealCalibrationSampleCount() const
{
  return {};
}

OctagonStickGate::OctagonStickGate(ControlState radius) : m_radius(radius)
{
}

ControlState OctagonStickGate::GetRadiusAtAngle(double ang) const
{
  constexpr int sides = 8;
  constexpr double sum_int_angles = (sides - 2) * MathUtil::PI;
  constexpr double half_int_angle = sum_int_angles / sides / 2;

  ang = std::fmod(ang, MathUtil::TAU / sides);
  // Solve ASA triangle using The Law of Sines:
  return m_radius / std::sin(MathUtil::PI - ang - half_int_angle) * std::sin(half_int_angle);
}

std::optional<u32> OctagonStickGate::GetIdealCalibrationSampleCount() const
{
  return 8;
}

RoundStickGate::RoundStickGate(ControlState radius) : m_radius(radius)
{
}

ControlState RoundStickGate::GetRadiusAtAngle(double) const
{
  return m_radius;
}

SquareStickGate::SquareStickGate(ControlState half_width) : m_half_width(half_width)
{
}

ControlState SquareStickGate::GetRadiusAtAngle(double ang) const
{
  constexpr double section_ang = MathUtil::TAU / 4;
  return m_half_width / std::cos(std::fmod(ang + section_ang / 2, section_ang) - section_ang / 2);
}

std::optional<u32> SquareStickGate::GetIdealCalibrationSampleCount() const
{
  // Because angle:0 points to the right we must use 8 samples for our square.
  return 8;
}

ReshapableInput::ReshapableInput(std::string name, std::string ui_name, GroupType type)
    : ControlGroup(std::move(name), std::move(ui_name), type)
{
  numeric_settings.emplace_back(std::make_unique<NumericSetting>(_trans("Dead Zone"), 0, 0, 50));
}

ControlState ReshapableInput::GetDeadzoneRadiusAtAngle(double ang) const
{
  // FYI: deadzone is scaled by input radius which allows the shape to match.
  return GetInputRadiusAtAngle(ang) * numeric_settings[SETTING_DEADZONE]->GetValue();
}

ControlState ReshapableInput::GetInputRadiusAtAngle(double ang) const
{
  // Handle the "default" state.
  if (m_calibration.empty())
  {
    return GetDefaultInputRadiusAtAngle(ang);
  }

  const auto sample_pos = ang / MathUtil::TAU * m_calibration.size();
  // Interpolate the radius between 2 calibration samples.
  const u32 sample1_index = u32(sample_pos) % m_calibration.size();
  const u32 sample2_index = (sample1_index + 1) % m_calibration.size();
  const double sample1_ang = sample1_index * MathUtil::TAU / m_calibration.size();
  const double sample2_ang = sample2_index * MathUtil::TAU / m_calibration.size();

  return GetRayLineIntersection(
      GetPointFromAngleAndLength(ang, 1.0),
      GetPointFromAngleAndLength(sample1_ang, m_calibration[sample1_index]),
      GetPointFromAngleAndLength(sample2_ang, m_calibration[sample2_index]));
}

ControlState ReshapableInput::GetDefaultInputRadiusAtAngle(double ang) const
{
  // This will normally be the same as the gate radius.
  // Unless a sub-class is doing weird things with the gate radius (e.g. Tilt)
  return GetGateRadiusAtAngle(ang);
}

void ReshapableInput::SetCalibrationToDefault()
{
  m_calibration.clear();
}

void ReshapableInput::SetCalibrationToZero()
{
  m_calibration.assign(CALIBRATION_SAMPLE_COUNT, 0.0);
}

void ReshapableInput::SetCalibrationFromGate(const StickGate& gate)
{
  m_calibration.resize(gate.GetIdealCalibrationSampleCount().value_or(CALIBRATION_SAMPLE_COUNT));

  u32 i = 0;
  for (auto& val : m_calibration)
    val = gate.GetRadiusAtAngle(MathUtil::TAU * i++ / m_calibration.size());
}

void ReshapableInput::UpdateCalibration(ControlState x, ControlState y)
{
  const auto ang_scale = MathUtil::TAU / m_calibration.size();

  const u32 calibration_index =
      std::lround((std::atan2(y, x) + MathUtil::TAU) / ang_scale) % m_calibration.size();
  const double calibration_ang = calibration_index * ang_scale;
  auto& calibration_sample = m_calibration[calibration_index];

  // Update closest sample from provided x,y.
  calibration_sample = std::max(calibration_sample, Common::DVec2(x, y).Length());

  // Here we update all other samples in our calibration vector to maintain
  // a convex polygon containing our new calibration point.
  // This is required to properly fill in angles that cannot be gotten.
  // (e.g. Keyboard input only has 8 possible angles)

  // Note: Loop assumes an even sample count, which should not be a problem.
  for (auto sample_offset = u32(m_calibration.size() / 2 - 1); sample_offset > 1; --sample_offset)
  {
    const auto update_at_offet = [&](u32 offset1, u32 offset2) {
      const u32 sample1_index = (calibration_index + offset1) % m_calibration.size();
      const double sample1_ang = sample1_index * ang_scale;
      auto& sample1 = m_calibration[sample1_index];

      const u32 sample2_index = (calibration_index + offset2) % m_calibration.size();
      const double sample2_ang = sample2_index * ang_scale;
      auto& sample2 = m_calibration[sample2_index];

      const double intersection =
          GetRayLineIntersection(GetPointFromAngleAndLength(sample2_ang, 1.0),
                                 GetPointFromAngleAndLength(sample1_ang, sample1),
                                 GetPointFromAngleAndLength(calibration_ang, calibration_sample));

      sample2 = std::max(sample2, intersection);
    };

    update_at_offet(sample_offset, sample_offset - 1);
    update_at_offet(u32(m_calibration.size() - sample_offset),
                    u32(m_calibration.size() - sample_offset + 1));
  }
}

void ReshapableInput::LoadConfig(IniFile::Section* sec, const std::string& defdev,
                                 const std::string& base)
{
  ControlGroup::LoadConfig(sec, defdev, base);

  const std::string group(base + name + "/");
  std::string load_str;
  sec->Get(group + CALIBRATION_CONFIG_NAME, &load_str, "");
  const auto load_data = SplitString(load_str, ' ');

  m_calibration.assign(load_data.size(), CALIBRATION_DEFAULT_VALUE);

  auto it = load_data.begin();
  for (auto& sample : m_calibration)
  {
    if (TryParse(*(it++), &sample))
      sample /= CALIBRATION_CONFIG_SCALE;
  }
}

void ReshapableInput::SaveConfig(IniFile::Section* sec, const std::string& defdev,
                                 const std::string& base)
{
  ControlGroup::SaveConfig(sec, defdev, base);

  const std::string group(base + name + "/");
  std::vector<std::string> save_data(m_calibration.size());
  std::transform(
      m_calibration.begin(), m_calibration.end(), save_data.begin(),
      [](ControlState val) { return StringFromFormat("%.2f", val * CALIBRATION_CONFIG_SCALE); });
  sec->Set(group + CALIBRATION_CONFIG_NAME, JoinStrings(save_data, " "), "");
}

ReshapableInput::ReshapeData ReshapableInput::Reshape(ControlState x, ControlState y,
                                                      ControlState modifier)
{
  // TODO: make the AtAngle functions work with negative angles:
  const ControlState ang = std::atan2(y, x) + MathUtil::TAU;

  const ControlState gate_max_dist = GetGateRadiusAtAngle(ang);
  const ControlState input_max_dist = GetInputRadiusAtAngle(ang);

  // If input radius is zero we apply no scaling.
  // This is useful when mapping native controllers without knowing intimate radius details.
  const ControlState max_dist = input_max_dist ? input_max_dist : gate_max_dist;

  ControlState dist = std::sqrt(x * x + y * y) / max_dist;

  // If the modifier is pressed, scale the distance by the modifier's value.
  // This is affected by the modifier's "range" setting which defaults to 50%.
  if (modifier)
  {
    // TODO: Modifier's range setting gets reset to 100% when the clear button is clicked.
    // This causes the modifier to not behave how a user might suspect.
    // Retaining the old scale-by-50% behavior until range is fixed to clear to 50%.
    dist *= 0.5;
    // dist *= modifier;
  }

  // Apply deadzone as a percentage of the user-defined radius/shape:
  const ControlState deadzone = GetDeadzoneRadiusAtAngle(ang);
  dist = std::max(0.0, dist - deadzone) / (1.0 - deadzone);

  // Scale to the gate shape/radius:
  dist = dist *= gate_max_dist;

  x = MathUtil::Clamp(std::cos(ang) * dist, -1.0, 1.0);
  y = MathUtil::Clamp(std::sin(ang) * dist, -1.0, 1.0);
  return {x, y};
}

}  // namespace ControllerEmu
