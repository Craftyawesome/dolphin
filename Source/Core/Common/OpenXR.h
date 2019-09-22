// Copyright 2019 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#pragma once

#include <openxr/openxr.h>

#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "Common/MathUtil.h"
#include "Common/Matrix.h"

namespace OpenXR
{
bool IsInit();

bool Init();
bool Shutdown();

void EventThreadFunc();

class Session
{
  friend void EventThreadFunc();

public:
  Session() = default;
  ~Session();

  Session(const Session&) = delete;
  Session& operator=(const Session&) = delete;

  bool Create(const std::vector<std::string_view>& required_extensions,
              const void* graphics_binding);

  bool Destroy();

  bool IsValid() const;

  bool CreateSwapchain(const std::vector<int64_t>& supported_formats);

  template <typename T>
  std::vector<T> GetSwapchainImages(const T& image_type)
  {
    uint32_t chain_length;
    EnumerateSwapchainImages(0, &chain_length, nullptr);

    std::vector<T> images(chain_length, image_type);
    EnumerateSwapchainImages(chain_length, &chain_length, images.data());

    return images;
  }

  bool RequestExit();

  bool BeginFrame();
  bool EndFrame();

  uint32_t AcquireAndWaitForSwapchainImage();
  bool ReleaseSwapchainImage();
  int64_t GetSwapchainFormat();
  XrExtent2Di GetSwapchainSize();

  // 0:Left, 1:Right
  Common::Matrix44 GetEyeViewMatrix(int eye_index, float near, float far);

  XrSessionState GetState() const;

private:
  // Updates predicted display time and eye matrices.
  // Values are marked dirty on frame end and thus updated once per frame on first request.
  void UpdateValuesIfDirty();
  bool AreValuesDirty();
  void MarkValuesDirty();

  bool WaitFrame();
  bool Begin();
  bool End();

  void OnChangeState(XrSessionState);
  void StopWaitFrameThread();

  bool EnumerateSwapchainImages(uint32_t count, uint32_t* capacity, void* data);

  XrSession m_session = XR_NULL_HANDLE;
  std::atomic<XrSessionState> m_session_state = XR_SESSION_STATE_IDLE;

  XrSwapchain m_swapchain = XR_NULL_HANDLE;
  int64_t m_swapchain_format;
  XrExtent2Di m_swapchain_size;

  XrSpace m_view_space = XR_NULL_HANDLE;
  XrSpace m_local_space = XR_NULL_HANDLE;
  bool m_is_headless_session = false;

  std::atomic<bool> m_run_wait_frame_thread;
  std::thread m_wait_frame_thread;

  // Constantly updated via wait-frame thread.
  std::atomic<XrTime> m_predicted_display_time = 0;

  // Copied from above and used to update eye views.
  // The display time we request should match the time of our views.
  XrTime m_display_time = 0;

  static constexpr uint32_t VIEW_COUNT = 2;

  std::array<XrView, VIEW_COUNT> m_eye_views = {};
};

std::unique_ptr<Session> CreateSession(const std::vector<std::string_view>& required_extensions,
                                       const void* graphics_binding,
                                       const std::vector<int64_t>& swapchain_formats);

}  // namespace OpenXR
