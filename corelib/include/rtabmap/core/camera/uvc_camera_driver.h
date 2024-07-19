/*********************************************************************
 * Software License Agreement (BSD License)
 *  Copyright (C) 2012 Ken Tossell
 *  Copyright (c) 2022 Orbbec 3D Technology, Inc
 *  Copyright (c) 2024 Guillaume Ricard <guillaume.ricard@polymtl.ca>
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the author nor other contributors may be
 *     used to endorse or promote products derived from this software
 *     without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#pragma once

#include <atomic>

#include <libuvc/libuvc.h>
#include <openni2/OpenNI.h>

#include <boost/optional.hpp>

namespace astra_camera {
struct UVCCameraConfig {
    int vendor_id = 0;
    int product_id = 0;
    int width = 0;
    int height = 0;
    int fps = 0;
    std::string serial_number;
    std::string format;
    std::string frame_id;
    std::string optical_frame_id;
    int retry_count = 0;
    UVCCameraConfig() = default;
    UVCCameraConfig(const UVCCameraConfig&) = default;
    UVCCameraConfig(UVCCameraConfig&&) = default;
    UVCCameraConfig& operator=(const UVCCameraConfig&) = default;
    UVCCameraConfig& operator=(UVCCameraConfig&&) = default;
};

std::ostream& operator<<(std::ostream& os, const UVCCameraConfig& config);

class UVCCameraDriver {
 public:
  explicit UVCCameraDriver(const UVCCameraConfig& camera_info,
                           const std::string& serial_number = "");

  ~UVCCameraDriver();

#if 0
  void setupCameraParams();

  void updateConfig(const UVCCameraConfig& config);

  void setVideoMode();

  void imageSubscribedCallback();

  void imageUnsubscribedCallback();

  void startStreaming();

  void stopStreaming() noexcept;

  int getResolutionX() const;

  int getResolutionY() const;
#if defined(USE_RK_MPP)
  void mppInit();
  void mppDeInit();
  void convertFrameToRGB(MppFrame frame, uint8_t* rgb_data);
  bool MPPDecodeFrame(uvc_frame_t* frame, uint8_t* rgb_data);
#endif

 private:
  void setupCameraControlService();

  sensor_msgs::CameraInfo getCameraInfo();

  static enum uvc_frame_format UVCFrameFormatString(const std::string& format);

  static void frameCallbackWrapper(uvc_frame_t* frame, void* ptr);

  void frameCallback(uvc_frame_t* frame);

  void autoControlsCallback(enum uvc_status_class status_class, int event, int selector,
                            enum uvc_status_attribute status_attribute, void* data,
                            size_t data_len);
  static void autoControlsCallbackWrapper(enum uvc_status_class status_class, int event,
                                          int selector, enum uvc_status_attribute status_attribute,
                                          void* data, size_t data_len, void* ptr);

  void openCamera();

  bool getUVCExposureCb(GetInt32Request& request, GetInt32Response& response);

#endif
  bool setUVCExposure(int value);
#if 0

  bool getUVCGainCb(GetInt32Request& request, GetInt32Response& response);

#endif
  bool setUVCGain(int value);
#if 0

  bool getUVCWhiteBalanceCb(GetInt32Request& request, GetInt32Response& response);

  bool setUVCWhiteBalanceCb(SetInt32Request& request, SetInt32Response& response);

#endif
  bool setUVCAutoExposure(bool enable);

  bool setUVCAutoWhiteBalance(bool enable);
#if 0
  bool getUVCMirrorCb(GetInt32Request& request, GetInt32Response& response);

#endif
  bool setUVCMirror(bool enable);
#if 0

  bool toggleUVCCamera(std_srvs::SetBoolRequest& request, std_srvs::SetBoolResponse& response);

  bool saveImageCallback(std_srvs::EmptyRequest& request, std_srvs::EmptyResponse& response);

  int UVCGetControl(int control, int unit, int len, uvc_req_code req_code);

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
#endif
  UVCCameraConfig config_;
#if 0
  std::string camera_name_ = "camera";
  std::string frame_id_;
  std::string color_info_uri_;
  ImageROI roi_;
  uvc_context_t* ctx_ = nullptr;
  uvc_device_t* device_ = nullptr;
#endif
  uvc_device_handle_t* device_handle_ = nullptr;
#if 0
  uvc_frame_t* frame_buffer_ = nullptr;
  uvc_stream_ctrl_t ctrl_{};
#endif
  std::atomic_bool uvc_flip_{false};
#if 0
  std::atomic_bool is_streaming_started{false};
  std::atomic_bool save_image_{false};
  std::atomic_bool is_camera_opened_{false};
  bool flip_color_ = false;

  ros::ServiceServer get_uvc_exposure_srv_;
  ros::ServiceServer set_uvc_exposure_srv_;
  ros::ServiceServer get_uvc_gain_srv_;
  ros::ServiceServer set_uvc_gain_srv_;
  ros::ServiceServer get_uvc_white_balance_srv_;
  ros::ServiceServer set_uvc_white_balance_srv_;
  ros::ServiceServer set_uvc_auto_exposure_srv_;
  ros::ServiceServer set_uvc_auto_white_balance_srv_;
  ros::ServiceServer get_uvc_mirror_srv_;
  ros::ServiceServer set_uvc_mirror_srv_;
  ros::ServiceServer toggle_uvc_camera_srv_;
  ros::ServiceServer save_image_srv_;
  ros::Publisher image_publisher_;
  ros::Publisher camera_info_publisher_;
  sensor_msgs::CameraInfo camera_info_;
  std::shared_ptr<camera_info_manager::CameraInfoManager> color_info_manager_ = nullptr;
  int device_num_ = 1;
  bool enable_color_auto_exposure_ = true;
  int exposure_ = -1;
  int gain_ = -1;
  int white_balance_ = -1;
#if defined(USE_RK_MPP)
  MppCtx mpp_ctx_ = nullptr;
  MppApi* mpp_api_ = nullptr;
  MppPacket mpp_packet_ = nullptr;
  MppFrame mpp_frame_ = nullptr;
  uint8_t* rgb_data_ = nullptr;
  MppDecCfg mpp_dec_cfg_ = nullptr;
  MppBuffer mpp_frame_buffer_ = nullptr;
  MppBuffer mpp_packet_buffer_ = nullptr;
  uint8_t* data_buffer_ = nullptr;
  MppBufferGroup mpp_frame_group_ = nullptr;
  MppBufferGroup mpp_packet_group_ = nullptr;
  MppTask mpp_task_ = nullptr;
  uint32_t need_split_ = 0;
#endif
#endif
};
}  // namespace astra_camera
