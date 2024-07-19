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

#include <rtabmap/core/camera/uvc_camera_driver.h>
#include <rtabmap/utilite/ULogger.h>

#include <boost/optional.hpp>
#include <opencv2/opencv.hpp>

#define libuvc_VERSION \
  (libuvc_VERSION_MAJOR * 10000 + libuvc_VERSION_MINOR * 100 + libuvc_VERSION_PATCH)
/** Converts an unaligned four-byte little-endian integer into an int32 */
#define DW_TO_INT(p) ((p)[0] | ((p)[1] << 8) | ((p)[2] << 16) | ((p)[3] << 24))
/** Converts an unaligned two-byte little-endian integer into an int16 */
#define SW_TO_SHORT(p) ((p)[0] | ((p)[1] << 8))
/** Converts an int16 into an unaligned two-byte little-endian integer */
#define SHORT_TO_SW(s, p) \
  (p)[0] = (s);           \
  (p)[1] = (s) >> 8;
/** Converts an int32 into an unaligned four-byte little-endian integer */
#define INT_TO_DW(i, p) \
  (p)[0] = (i);         \
  (p)[1] = (i) >> 8;    \
  (p)[2] = (i) >> 16;   \
  (p)[3] = (i) >> 24;

/** Selects the nth item in a doubly linked list. n=-1 selects the last item. */
#define DL_NTH(head, out, n)               \
  do {                                     \
    int dl_nth_i = 0;                      \
    LDECLTYPE(head) dl_nth_p = (head);     \
    if ((n) < 0) {                         \
      while (dl_nth_p && dl_nth_i > (n)) { \
        dl_nth_p = dl_nth_p->prev;         \
        dl_nth_i--;                        \
      }                                    \
    } else {                               \
      while (dl_nth_p && dl_nth_i < (n)) { \
        dl_nth_p = dl_nth_p->next;         \
        dl_nth_i++;                        \
      }                                    \
    }                                      \
    (out) = dl_nth_p;                      \
  } while (0);

namespace astra_camera {

std::ostream &operator<<(std::ostream &os, const UVCCameraConfig &config) {
  os << "vendor_id: " << std::hex << config.vendor_id << std::endl;
  os << "product_id: " << std::hex << config.product_id << std::endl;
  os << "width: " << std::dec << config.width << std::endl;
  os << "height: " << config.height << std::endl;
  os << "fps: " << config.fps << std::endl;
  os << "serial_number: " << config.serial_number << std::endl;
  os << "format: " << config.format << std::endl;
  return os;
}

void UVCCameraDriver::setupCameraParams() {
  // auto exposure
  if (enable_color_auto_exposure_) {
    UINFO("enable color auto exposure");
    uvc_set_ae_mode(device_handle_, 8);
  } else {
    UINFO("disable color auto exposure");
    uvc_set_ae_mode(device_handle_, 1);
  }
  if (exposure_ != -1) {
    uint32_t max_expo, min_expo;
    uvc_get_exposure_abs(device_handle_, &max_expo, UVC_GET_MAX);
    uvc_get_exposure_abs(device_handle_, &min_expo, UVC_GET_MIN);
    if (exposure_ < static_cast<int>(min_expo) || exposure_ > static_cast<int>(max_expo)) {
      UWARN("exposure value %d is out of range [%d, %d], set to auto mode", exposure_, min_expo, max_expo);
      uvc_set_ae_mode(device_handle_, 1);
    } else {
      uvc_set_ae_mode(device_handle_,
                      1);  // mode 1: manual mode; 2: auto mode; 4: shutter priority mode; 8:
      // aperture priority mode
      uvc_set_exposure_abs(device_handle_, static_cast<uint32_t>(exposure_));
    }
  }
  if (gain_ != -1) {
    uint16_t min_gain, max_gain;
    uvc_get_gain(device_handle_, &min_gain, UVC_GET_MIN);
    uvc_get_gain(device_handle_, &max_gain, UVC_GET_MAX);
    if (gain_ < min_gain || gain_ > max_gain) {
      UWARN("gain value %d is out of range [%d, %d], set to auto mode", gain_, min_gain, max_gain);
      uvc_set_ae_mode(device_handle_, 1);
    } else {
      uvc_set_ae_mode(device_handle_, 1);
      uvc_set_gain(device_handle_, static_cast<uint16_t>(gain_));
    }
  }
}

UVCCameraDriver::UVCCameraDriver(const UVCCameraConfig &camera_info,
                                 const std::string &serial_number)
    : config_(camera_info) {
  auto err = uvc_init(&ctx_, nullptr);
  if (err != UVC_SUCCESS) {
    uvc_perror(err, "ERROR: uvc_init");
    UERROR("init uvc context failed, exit");
    throw std::runtime_error("init uvc context failed");
  }
  config_.serial_number = serial_number;
  device_num_ = 1;
  uvc_flip_ = false;
  flip_color_ = false;
  config_.vendor_id = 0x2bc5;
  config_.product_id = 0x050e;
  camera_name_ = "camera";
  enable_color_auto_exposure_ = true;
  gain_ = -1;
  exposure_ = -1;
  frame_buffer_ = uvc_allocate_frame(config_.width * config_.height * 3);
  if (frame_buffer_ == nullptr) {
      UERROR("Null pointer received from uvc_allocate_frame");
      throw std::runtime_error("allocate uvc frame failed");
  }

  frame_data_ = (char*) malloc(3 * config_.width * config_.height);

  openCamera();
  setupCameraParams();
#if defined(USE_RK_MPP)
  mppInit();
#endif
}

UVCCameraDriver::~UVCCameraDriver() {
  stopStreaming();
  if (frame_data_) {
    free(frame_data_);
    frame_data_ = nullptr;
  }
  if (device_handle_) {
    UINFO("uvc close device");
    uvc_close(device_handle_);
    UINFO("uvc close device done");
    device_handle_ = nullptr;
  }
  if (device_) {
    UINFO("uvc unref device");
    uvc_unref_device(device_);
    UINFO("uvc unref device done");
    device_ = nullptr;
  }
  if (ctx_) {
    UINFO("uvc exit");
    uvc_exit(ctx_);
    UINFO("uvc exit done");
    ctx_ = nullptr;
  }
  if (frame_buffer_) {
    UINFO("uvc free frame");
    uvc_free_frame(frame_buffer_);
    frame_buffer_ = nullptr;
    UINFO("uvc free frame done");
  }
#if defined(USE_RK_MPP)
  mppDeInit();
#endif
}

void UVCCameraDriver::openCamera() {
  UINFO("open uvc camera");
  uvc_error_t err;
  auto serial_number = config_.serial_number.empty() ? nullptr : config_.serial_number.c_str();
  if(device_ == nullptr)
  {
      UERROR("Check failed libuvc device is NULL");
      return;
  }
  err = uvc_find_device(ctx_, &device_, config_.vendor_id, config_.product_id, serial_number);
  if (err != UVC_SUCCESS && device_num_ == 1) {
    // retry serial number == nullptr
    err = uvc_find_device(ctx_, &device_, config_.vendor_id, config_.product_id, nullptr);
  }
  if (err != UVC_SUCCESS) {
    uvc_perror(err, "ERROR: uvc_find_device");
    UERROR("find uvc device failed, retry %d times", config_.retry_count);
    for (int i = 0; i < config_.retry_count; i++) {
      err = uvc_find_device(ctx_, &device_, config_.vendor_id, config_.product_id, serial_number);
      if (err == UVC_SUCCESS) {
        break;
      }
      usleep(100 * i);
    }
  }
  std::stringstream ss;
  ss << "uvc config: " << config_;
  UERROR("%s", ss.str().c_str());
  if (err != UVC_SUCCESS) {
    UERROR("Find device error %s process will be exit", uvc_strerror(err));
    if (device_ != nullptr) {
      uvc_unref_device(device_);
    }
    device_ = nullptr;
    throw std::runtime_error(ss.str());
  }
  if(device_handle_ == nullptr) {
      UERROR("Check failed libuvc device handle is NULL");
      return;
  }
  err = uvc_open(device_, &device_handle_);
  if (err != UVC_SUCCESS) {
    std::stringstream ss;
    if (UVC_ERROR_ACCESS == err) {
      UERROR("Permission denied opening /dev/bus/usb/%d/", uvc_get_bus_number(device_));
    } else {
      UERROR("Can't open /dev/bus/usb/%d/%d: %s (%d)", uvc_get_bus_number(device_),
          uvc_get_device_address(device_), uvc_strerror(err), err);
    }
    uvc_unref_device(device_);
    device_ = nullptr;
    throw std::runtime_error("Cannot open libuvc device");
  }
  uvc_set_status_callback(device_handle_, &UVCCameraDriver::autoControlsCallbackWrapper, this);
  if (device_handle_ == nullptr) {
      UERROR("Check failed device_handle_ is NULL");
  }
  if (device_ == nullptr) {
      UERROR("Check failed device_ is NULL");
  }
  UINFO("open camera success");
  is_camera_opened_ = true;
}

#if 0
void UVCCameraDriver::updateConfig(const UVCCameraConfig &config) { config_ = config; }

void UVCCameraDriver::setVideoMode() {
  auto uvc_format = UVCFrameFormatString(config_.format);
  int width = config_.width;
  int height = config_.height;
  int fps = config_.fps;
  uvc_error_t err;
  CHECK_NOTNULL(device_);
  CHECK_NOTNULL(device_handle_);
  for (int i = 0; i < 5; i++) {
    err = uvc_get_stream_ctrl_format_size(device_handle_, &ctrl_, uvc_format, width, height, fps);
    if (err == UVC_SUCCESS) {
      break;
    }
  }
  ROS_INFO_STREAM("set uvc mode " << width << "x" << height << "@" << fps << " format "
                                  << config_.format);
  if (err != UVC_SUCCESS) {
    ROS_ERROR_STREAM("set uvc ctrl error " << uvc_strerror(err));
    uvc_close(device_handle_);
    device_handle_ = nullptr;
    uvc_unref_device(device_);
    device_ = nullptr;
    return;
  }
}

void UVCCameraDriver::imageSubscribedCallback() {
  ROS_INFO_STREAM("UVCCameraDriver image subscribed");
  startStreaming();
}

void UVCCameraDriver::imageUnsubscribedCallback() {
  ROS_INFO_STREAM("UVCCameraDriver image unsubscribed");
  auto subscriber_count = image_publisher_.getNumSubscribers();
  if (subscriber_count == 0) {
    stopStreaming();
  }
}
#endif

void UVCCameraDriver::startStreaming() {
#if 0
  if (is_streaming_started) {
    ROS_WARN_STREAM("UVCCameraDriver streaming is already started");
    return;
  }
  if (!is_camera_opened_) {
    ROS_WARN_STREAM("UVCCameraDriver camera is not opened");
    return;
  }
  CHECK_NOTNULL(device_);
  CHECK_NOTNULL(device_handle_);
  setVideoMode();
  uvc_error_t stream_err =
      uvc_start_streaming(device_handle_, &ctrl_, &UVCCameraDriver::frameCallbackWrapper, this, 0);
  if (stream_err != UVC_SUCCESS) {
    ROS_ERROR_STREAM("uvc start streaming error " << uvc_strerror(stream_err) << " retry "
                                                  << config_.retry_count << " times");
    for (int i = 0; i < config_.retry_count; i++) {
      stream_err = uvc_start_streaming(device_handle_, &ctrl_,
                                       &UVCCameraDriver::frameCallbackWrapper, this, 0);
      if (stream_err == UVC_SUCCESS) {
        break;
      }
      usleep(100 * i);
    }
  }
  if (stream_err != UVC_SUCCESS) {
    uvc_perror(stream_err, "uvc_start_streaming");
    uvc_close(device_handle_);
    device_handle_ = nullptr;
    uvc_unref_device(device_);
    device_ = nullptr;
    return;
  }
  is_streaming_started.store(true);
#endif
}

void UVCCameraDriver::stopStreaming() noexcept {
  if (!is_streaming_started) {
    UWARN("streaming is already stopped");
    return;
  }
  UINFO("stop uvc streaming");
  uvc_stop_streaming(device_handle_);
  is_streaming_started.store(false);
}

int UVCCameraDriver::getResolutionX() const { return config_.width; }

int UVCCameraDriver::getResolutionY() const { return config_.height; }

#if defined(USE_RK_MPP)

void UVCCameraDriver::mppInit() {
  MPP_RET ret = mpp_create(&mpp_ctx_, &mpp_api_);
  if (ret != MPP_OK) {
    ROS_ERROR_STREAM("mpp_create failed, ret = " << ret);
    throw std::runtime_error("mpp_create failed");
  }
  MpiCmd mpi_cmd = MPP_CMD_BASE;
  MppParam mpp_param = nullptr;

  mpi_cmd = MPP_DEC_SET_PARSER_SPLIT_MODE;
  mpp_param = &need_split_;
  ret = mpp_api_->control(mpp_ctx_, mpi_cmd, mpp_param);
  if (ret != MPP_OK) {
    ROS_ERROR_STREAM("mpp_api_->control failed, ret = " << ret);
    throw std::runtime_error("mpp_api_->control failed");
  }
  ret = mpp_init(mpp_ctx_, MPP_CTX_DEC, MPP_VIDEO_CodingMJPEG);
  if (ret != MPP_OK) {
    ROS_ERROR_STREAM("mpp_init failed, ret = " << ret);
    throw std::runtime_error("mpp_init failed");
  }
  MppFrameFormat fmt = MPP_FMT_YUV420SP_VU;
  mpp_param = &fmt;
  ret = mpp_api_->control(mpp_ctx_, MPP_DEC_SET_OUTPUT_FORMAT, mpp_param);
  ret = mpp_frame_init(&mpp_frame_);
  if (ret != MPP_OK) {
    ROS_ERROR_STREAM("mpp_frame_init failed, ret = " << ret);
    throw std::runtime_error("mpp_frame_init failed");
  }
  ret = mpp_buffer_group_get_internal(&mpp_frame_group_, MPP_BUFFER_TYPE_ION);
  if (ret != MPP_OK) {
    ROS_ERROR_STREAM("mpp_buffer_group_get_internal failed, ret = " << ret);
    throw std::runtime_error("mpp_buffer_group_get_internal failed");
  }
  ret = mpp_buffer_group_get_internal(&mpp_packet_group_, MPP_BUFFER_TYPE_ION);
  if (ret != MPP_OK) {
    ROS_ERROR_STREAM("mpp_buffer_group_get_internal failed, ret = " << ret);
    throw std::runtime_error("mpp_buffer_group_get_internal failed");
  }
  RK_U32 hor_stride = MPP_ALIGN((config_.width), 16);
  RK_U32 ver_stride = MPP_ALIGN((config_.height), 16);
  ret = mpp_buffer_get(mpp_frame_group_, &mpp_frame_buffer_, hor_stride * ver_stride * 4);
  if (ret != MPP_OK) {
    ROS_ERROR_STREAM("mpp_buffer_get failed, ret = " << ret);
    throw std::runtime_error("mpp_buffer_get failed");
  }
  mpp_frame_set_buffer(mpp_frame_, mpp_frame_buffer_);
  ret = mpp_buffer_get(mpp_packet_group_, &mpp_packet_buffer_, config_.width * config_.height * 3);
  if (ret != MPP_OK) {
    ROS_ERROR_STREAM("mpp_buffer_get failed, ret = " << ret);
    throw std::runtime_error("mpp_buffer_get failed");
  }
  mpp_packet_init_with_buffer(&mpp_packet_, mpp_packet_buffer_);
  data_buffer_ = (uint8_t *)mpp_buffer_get_ptr(mpp_packet_buffer_);
  rgb_data_ = new uint8_t[config_.width * config_.height * 3];
}
void UVCCameraDriver::mppDeInit() {
  if (mpp_frame_buffer_) {
    mpp_buffer_put(mpp_frame_buffer_);
    mpp_frame_buffer_ = nullptr;
  }
  if (mpp_packet_buffer_) {
    mpp_buffer_put(mpp_packet_buffer_);
    mpp_packet_buffer_ = nullptr;
  }
  if (mpp_frame_group_) {
    mpp_buffer_group_put(mpp_frame_group_);
    mpp_frame_group_ = nullptr;
  }
  if (mpp_packet_group_) {
    mpp_buffer_group_put(mpp_packet_group_);
    mpp_packet_group_ = nullptr;
  }
  if (mpp_frame_) {
    mpp_frame_deinit(&mpp_frame_);
    mpp_frame_ = nullptr;
  }
  if (mpp_packet_) {
    mpp_packet_deinit(&mpp_packet_);
    mpp_packet_ = nullptr;
  }
  if (mpp_ctx_) {
    mpp_destroy(mpp_ctx_);
    mpp_ctx_ = nullptr;
  }
  delete[] rgb_data_;
  rgb_data_ = nullptr;
}

void UVCCameraDriver::convertFrameToRGB(MppFrame frame, uint8_t *rgb_data) {
  rga_info_t src_info = {0};
  rga_info_t dst_info = {0};
  size_t width = mpp_frame_get_width(frame);
  size_t height = mpp_frame_get_height(frame);
  int format = mpp_frame_get_fmt(frame);
  MppBuffer buffer = mpp_frame_get_buffer(frame);
  memset(rgb_data, 0, width * height * 3);
  auto data = mpp_buffer_get_ptr(buffer);
  src_info.fd = -1;
  src_info.mmuFlag = 1;
  src_info.virAddr = data;
  src_info.format = RK_FORMAT_YCbCr_420_SP;
  dst_info.fd = -1;
  dst_info.mmuFlag = 1;
  dst_info.virAddr = rgb_data;
  dst_info.format = RK_FORMAT_RGB_888;
  rga_set_rect(&src_info.rect, 0, 0, width, height, width, height, RK_FORMAT_YCbCr_420_SP);
  rga_set_rect(&dst_info.rect, 0, 0, width, height, width, height, RK_FORMAT_RGB_888);
  int ret = c_RkRgaBlit(&src_info, &dst_info, NULL);
  if (ret) {
    ROS_ERROR_STREAM("c_RkRgaBlit error " << ret);
  }
}

bool UVCCameraDriver::MPPDecodeFrame(uvc_frame_t *frame, uint8_t *rgb_data) {
  MPP_RET ret = MPP_OK;
  memset(data_buffer_, 0, config_.width * config_.height * 3);
  memcpy(data_buffer_, frame->data, frame->data_bytes);
  mpp_packet_set_pos(mpp_packet_, data_buffer_);
  mpp_packet_set_length(mpp_packet_, frame->data_bytes);
  mpp_packet_set_eos(mpp_packet_);
  CHECK_NOTNULL(mpp_ctx_);
  ret = mpp_api_->poll(mpp_ctx_, MPP_PORT_INPUT, MPP_POLL_BLOCK);
  if (ret != MPP_OK) {
    ROS_ERROR("mpp poll failed %d", ret);
    return false;
  }
  ret = mpp_api_->dequeue(mpp_ctx_, MPP_PORT_INPUT, &mpp_task_);
  if (ret != MPP_OK) {
    ROS_ERROR("mpp dequeue failed %d", ret);
    return false;
  }
  mpp_task_meta_set_packet(mpp_task_, KEY_INPUT_PACKET, mpp_packet_);
  mpp_task_meta_set_frame(mpp_task_, KEY_OUTPUT_FRAME, mpp_frame_);
  ret = mpp_api_->enqueue(mpp_ctx_, MPP_PORT_INPUT, mpp_task_);
  if (ret != MPP_OK) {
    ROS_ERROR("mpp enqueue failed %d", ret);
    return false;
  }
  ret = mpp_api_->poll(mpp_ctx_, MPP_PORT_OUTPUT, MPP_POLL_BLOCK);
  if (ret != MPP_OK) {
    ROS_ERROR("mpp poll failed %d", ret);
    return false;
  }
  ret = mpp_api_->dequeue(mpp_ctx_, MPP_PORT_OUTPUT, &mpp_task_);
  if (ret != MPP_OK) {
    ROS_ERROR("mpp dequeue failed %d", ret);
    return false;
  }
  if (mpp_task_) {
    MppFrame output_frame = nullptr;
    mpp_task_meta_get_frame(mpp_task_, KEY_OUTPUT_FRAME, &output_frame);
    if (mpp_frame_) {
      convertFrameToRGB(mpp_frame_, rgb_data);
      if (mpp_frame_get_eos(output_frame)) {
        ROS_INFO_STREAM("mpp frame get eos");
      }
    }
    ret = mpp_api_->enqueue(mpp_ctx_, MPP_PORT_OUTPUT, mpp_task_);
    if (ret != MPP_OK) {
      ROS_ERROR("mpp enqueue failed %d", ret);
      return false;
    }
  }
  return true;
}

#endif

enum uvc_frame_format UVCCameraDriver::UVCFrameFormatString(const std::string &format) {
  if (format == "uncompressed") {
    return UVC_COLOR_FORMAT_UNCOMPRESSED;
  } else if (format == "compressed") {
    return UVC_COLOR_FORMAT_COMPRESSED;
  } else if (format == "yuyv") {
    return UVC_COLOR_FORMAT_YUYV;
  } else if (format == "uyvy") {
    return UVC_COLOR_FORMAT_UYVY;
  } else if (format == "rgb") {
    return UVC_COLOR_FORMAT_RGB;
  } else if (format == "bgr") {
    return UVC_COLOR_FORMAT_BGR;
  } else if (format == "mjpeg") {
    return UVC_COLOR_FORMAT_MJPEG;
  } else if (format == "gray8") {
    return UVC_COLOR_FORMAT_GRAY8;
  } else {
    UWARN("Invalid Video Mode: %s", format.c_str());
    UWARN("Continue using video mode: uncompressed");
    return UVC_COLOR_FORMAT_UNCOMPRESSED;
  }
}

void UVCCameraDriver::frameCallbackWrapper(uvc_frame_t *frame, void *ptr) {
  if (ptr == nullptr) {
      UERROR("Check failed libuvc user data is NULL");
      return;
  }
  auto driver = static_cast<UVCCameraDriver *>(ptr);
  driver->frameCallback(frame);
}

void UVCCameraDriver::frameCallback(uvc_frame_t *frame) {
  if (frame == NULL) {
      UERROR("Check failed libuvc frame is NULL");
      return;
  }
  if (frame_buffer_ == NULL) {
      UERROR("Check failed libuvc frame buffer is NULL");
      return;
  }
  static constexpr int unit_step = 3;
  if ((frame->frame_format == UVC_FRAME_FORMAT_BGR) ||
      (frame->frame_format == UVC_FRAME_FORMAT_RGB) ||
      (frame->frame_format == UVC_FRAME_FORMAT_UYVY)) {
    memcpy(frame_data_, frame->data, frame->data_bytes);
  } else if (frame->frame_format == UVC_FRAME_FORMAT_YUYV) {
    // FIXME: uvc_any2bgr does not work on "yuyv" format, so use uvc_yuyv2bgr directly.
    uvc_error_t conv_ret = uvc_yuyv2bgr(frame, frame_buffer_);
    if (conv_ret != UVC_SUCCESS) {
      uvc_perror(conv_ret, "Couldn't convert frame to RGB");
      return;
    }
    memcpy(frame_data_, frame_buffer_->data, frame_buffer_->data_bytes);
  } else if (frame->frame_format == UVC_FRAME_FORMAT_MJPEG) {
    // Enable mjpeg support despite uvs_any2bgr shortcoming
    //  https://github.com/ros-drivers/libuvc_ros/commit/7508a09f
#if defined(USE_RK_MPP)
    bool ret = MPPDecodeFrame(frame, rgb_data_);
    if (!ret) {
      UERROR("MPPDecodeFrame failed");
      return;
    }
    image.encoding = "bgr8";
    if (mpp_frame_ == nullptr) {
        return;
    }
    if (rgb_data_ == nullptr) {
        return;
    }
    memcpy(&(image.data[0]), rgb_data_, frame->width * frame->height * 3);
#else

    uvc_error_t conv_ret = uvc_mjpeg2rgb(frame, frame_buffer_);
    if (conv_ret != UVC_SUCCESS) {
      uvc_perror(conv_ret, "Couldn't convert frame to RGB");
      return;
    }
    memcpy(frame_data_, frame_buffer_->data, frame_buffer_->data_bytes);
#endif
  } else {
    uvc_error_t conv_ret = uvc_any2bgr(frame, frame_buffer_);
    if (conv_ret != UVC_SUCCESS) {
      uvc_perror(conv_ret, "Couldn't convert frame to RGB");
      return;
    }
    memcpy(frame_data_, frame_buffer_->data, frame_buffer_->data_bytes);
  }
}

void UVCCameraDriver::autoControlsCallback(enum uvc_status_class status_class, int event,
                                           int selector, enum uvc_status_attribute status_attribute,
                                           void *data, size_t data_len) {
  char buff[256];
  if (data_len < 256) {
      UERROR("Check failed libuvc control data is less than 256 bytes");
      return;
  }
  (void)data;
  sprintf(buff, "Controls callback. class: %d, event: %d, selector: %d, attr: %d, data_len: %zu\n",
          status_class, event, selector, status_attribute, data_len);
  UINFO("%s", buff);
}

void UVCCameraDriver::autoControlsCallbackWrapper(enum uvc_status_class status_class, int event,
                                                  int selector,
                                                  enum uvc_status_attribute status_attribute,
                                                  void *data, size_t data_len, void *ptr) {
  if (ptr == nullptr) {
    UERROR("Check failed libuvc control user data is NULL");
    return;
  }
  auto driver = static_cast<UVCCameraDriver *>(ptr);
  driver->autoControlsCallback(status_class, event, selector, status_attribute, data, data_len);
}

int UVCCameraDriver::getUVCExposure() {
  uint32_t data;
  uvc_error_t err = uvc_get_exposure_abs(device_handle_, &data, UVC_GET_CUR);
  if (err != UVC_SUCCESS) {
    auto msg = uvc_strerror(err);
    UERROR("getUVCExposure %s", msg);
    return 0;
  }
  return static_cast<int>(data);
}

bool UVCCameraDriver::setUVCExposure(int value) {
  if (value == 0) {
    UERROR("set auto mode");
    uvc_error_t err = uvc_set_ae_mode(device_handle_, 8);  // 8才是自动8: aperture priority mode
    UERROR("ret :%d", (int)err);
    return true;
  }
  uint32_t max_expo, min_expo;
  uvc_get_exposure_abs(device_handle_, &max_expo, UVC_GET_MAX);
  uvc_get_exposure_abs(device_handle_, &min_expo, UVC_GET_MIN);
  if (value < static_cast<int>(min_expo) || value > static_cast<int>(max_expo)) {
    UERROR("Exposure value out of range. Min: %d, Max: %d", (int)min_expo, (int)max_expo);
    return false;
  }
  uvc_set_ae_mode(
      device_handle_,
      1);  // mode 1: manual mode; 2: auto mode; 4: shutter priority mode; 8: aperture priority mode

  uvc_error_t err = uvc_set_exposure_abs(device_handle_, value);
  if (err != UVC_SUCCESS) {
    auto msg = uvc_strerror(err);
    UERROR("setUVCExposure %s", msg);
    return false;
  }
  return true;
}

int UVCCameraDriver::getUVCGain() {
  uint16_t gain;
  uvc_error_t err = uvc_get_gain(device_handle_, &gain, UVC_GET_CUR);
  if (err != UVC_SUCCESS) {
    auto msg = uvc_strerror(err);
    UERROR("getUVCGain %s", msg);
    return false;
  }
  return gain;
}

bool UVCCameraDriver::setUVCGain(int value) {
  uint16_t min_gain, max_gain;
  uvc_get_gain(device_handle_, &min_gain, UVC_GET_MIN);
  uvc_get_gain(device_handle_, &max_gain, UVC_GET_MAX);
  if (value < min_gain || value > max_gain) {
    UERROR("Gain must be between %d and %d", (int)min_gain, (int)max_gain);
    return false;
  }
  uvc_error_t err = uvc_set_gain(device_handle_, value);
  if (err != UVC_SUCCESS) {
    auto msg = uvc_strerror(err);
    UERROR("setUVCGain %s", msg);
    return false;
  }
  return true;
}

#if 0
bool UVCCameraDriver::getUVCWhiteBalanceCb(GetInt32Request &request, GetInt32Response &response) {
  (void)request;
  uint16_t data;
  uvc_error_t err = uvc_get_white_balance_temperature(device_handle_, &data, UVC_GET_CUR);
  response.data = data;
  if (err != UVC_SUCCESS) {
    auto msg = uvc_strerror(err);
    ROS_ERROR_STREAM("getUVCWhiteBalanceCb " << msg);
    response.message = msg;
    return false;
  }
  return true;
}

bool UVCCameraDriver::setUVCWhiteBalanceCb(SetInt32Request &request, SetInt32Response &response) {
  if (request.data == 0) {
    uvc_set_white_balance_temperature_auto(device_handle_, 1);
    return true;
  }
  uvc_set_white_balance_temperature_auto(device_handle_, 0);  // 0: manual, 1: auto
  uint8_t data[4];
  INT_TO_DW(request.data, data);
  int unit = uvc_get_processing_units(device_handle_)->bUnitID;
  int control = UVC_PU_WHITE_BALANCE_TEMPERATURE_CONTROL;
  int min_white_balance = UVCGetControl(control, unit, sizeof(int16_t), UVC_GET_MIN);
  int max_white_balance = UVCGetControl(control, unit, sizeof(int16_t), UVC_GET_MAX);
  if (request.data < min_white_balance || request.data > max_white_balance) {
    std::stringstream ss;
    ss << "Please set white balance between " << min_white_balance << " and " << max_white_balance;
    response.message = ss.str();
    ROS_ERROR_STREAM(ss.str());
    return false;
  }
  int ret = uvc_set_ctrl(device_handle_, unit, control, data, sizeof(int32_t));
  if (ret != sizeof(int32_t)) {
    auto err = static_cast<uvc_error_t>(ret);
    std::stringstream ss;
    ss << "set white balance failed " << uvc_strerror(err);
    ROS_ERROR_STREAM(ss.str());
    response.message = ss.str();
    return false;
  }
  return true;
}

#endif
bool UVCCameraDriver::setUVCAutoExposure(bool enable) {
  if (enable) {
    uvc_set_ae_mode(device_handle_, 8);
  } else {
    uvc_set_ae_mode(device_handle_, 1);
  }
  return true;
}

bool UVCCameraDriver::setUVCAutoWhiteBalance(bool enabled) {
  if (enabled) {
    uvc_set_white_balance_temperature_auto(device_handle_, 1);
  } else {
    uvc_set_white_balance_temperature_auto(device_handle_, 0);
  }
  return true;
}
#if 0

bool UVCCameraDriver::getUVCMirrorCb(GetInt32Request &request, GetInt32Response &response) {
  (void)request;
  int16_t mirror;
  uvc_error_t err = uvc_get_roll_abs(device_handle_, &mirror, UVC_GET_CUR);
  response.data = mirror;
  if (err != UVC_SUCCESS) {
    auto msg = uvc_strerror(err);
    ROS_ERROR_STREAM("getUVCMirrorCb " << msg);
    response.message = msg;
    return false;
  }
  return true;
}
#endif

bool UVCCameraDriver::setUVCMirror(bool enable) {
  uvc_flip_ = enable;
  return true;
}

#if 0
bool UVCCameraDriver::toggleUVCCamera(std_srvs::SetBoolRequest &request,
                                      std_srvs::SetBoolResponse &response) {
  (void)response;
  if (request.data) {
    startStreaming();
  } else {
    stopStreaming();
  }
  return true;
}

int UVCCameraDriver::UVCGetControl(int control, int unit, int len, uvc_req_code req_code) {
  uint8_t data[4];
  int ret = uvc_get_ctrl(device_handle_, unit, control, data, len, req_code);
  if (ret < 0) {
    auto err = static_cast<uvc_error>(ret);
    ROS_ERROR("Failed to get control %d on unit %d: %s", control, unit, uvc_strerror(err));
    return -1;
  }
  return SW_TO_SHORT(data);
}
#endif

}  // namespace astra_camera
