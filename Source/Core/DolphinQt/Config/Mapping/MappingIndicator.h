// Copyright 2018 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#pragma once

#include <QToolButton>
#include <QWidget>

namespace ControllerEmu
{
class Control;
class ControlGroup;
class Cursor;
class NumericSetting;
class ReshapableInput;
}  // namespace ControllerEmu

class QPaintEvent;
class QTimer;
class QPainter;

class MappingIndicator : public QWidget
{
public:
  explicit MappingIndicator(ControllerEmu::ControlGroup* group);

  bool is_calibrating = {};

private:
  void DrawCursor(ControllerEmu::Cursor& cursor);
  void DrawReshapableInput(ControllerEmu::ReshapableInput& stick);
  void DrawMixedTriggers();
  void DrawCalibration(QPainter& p, double x, double y);

  void paintEvent(QPaintEvent*) override;

  ControllerEmu::ControlGroup* m_group;

  QTimer* m_timer;
};

class CalibrationWidget : public QToolButton
{
public:
  CalibrationWidget(ControllerEmu::ReshapableInput& input, MappingIndicator& indicator);

private:
  void StartCalibration();
  void SetupActions();

  ControllerEmu::ReshapableInput& m_input;
  MappingIndicator& m_indicator;
};
