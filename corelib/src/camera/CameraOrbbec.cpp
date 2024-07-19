/*
Copyright (c) 2024, Guillaume Ricard - MISTLab - Polytechnique Montreal
Copyright (c) 2010-2016, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Universite de Sherbrooke nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <rtabmap/core/camera/CameraOrbbec.h>
#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/utilite/UFile.h>
#include <rtabmap/utilite/UThreadC.h>
#include <rtabmap/core/util2d.h>
#include <opencv2/imgproc/types_c.h>

#ifdef RTABMAP_OPENNI2
#include <OniVersion.h>
#include <OpenNI.h>
#endif

namespace rtabmap
{

CameraOrbbec::CameraOrbbec(
		const std::string & deviceId,
		Type type,
		float imageRate,
		const rtabmap::Transform & localTransform) :
	CameraOpenNI2(deviceId, type, imageRate, localTransform)
#ifdef RTABMAP_OPENNI2
    ,
	_type(type),
	_device(new openni::Device()),
	_color_driver(nullptr),
	_color(new openni::VideoStream()),
	_depth(new openni::VideoStream()),
	_depthFx(0.0f),
	_depthFy(0.0f),
	_deviceId(deviceId),
	_openNI2StampsAndIDsUsed(false),
	_depthHShift(0),
	_depthVShift(0),
	_depthDecimation(1)
#endif
{
}

CameraOrbbec::~CameraOrbbec()
{
    if (_color_driver != nullptr) {
        delete _color_driver;
        _color_driver = nullptr;
    }
#ifdef RTABMAP_OPENNI2
	_color->stop();
	_color->destroy();
	_depth->stop();
	_depth->destroy();
	_device->close();
	openni::OpenNI::shutdown();

	delete _device;
	delete _color;
	delete _depth;
#endif
}

bool CameraOrbbec::setAutoWhiteBalance(bool enabled)
{
#ifdef RTABMAP_LIBUVC
    if (_color_driver)
    {
        return _color_driver->setUVCAutoWhiteBalance(enabled);
    }
#else
	UERROR("CameraOrbbec: RTAB-Map is not built with libuvc support!");
#endif
	return false;
}

bool CameraOrbbec::setAutoExposure(bool enabled)
{
#ifdef RTABMAP_LIBUVC
	if(_color_driver)
	{
    	return _color_driver->setUVCAutoExposure(enabled);
	}
#else
	UERROR("CameraOrbbec: RTAB-Map is not built with libuvc support!");
#endif
	return false;
}

bool CameraOrbbec::setExposure(int value)
{
#ifdef RTABMAP_LIBUVC
    if(_color_driver)
    {
        return _color_driver->setUVCExposure(value);
    }
#else
	UERROR("CameraOrbbec: RTAB-Map is not built with libuvc support!");
#endif
	return false;
}

bool CameraOrbbec::setGain(int value)
{
#ifdef RTABMAP_LIBUVC
    if(_color_driver)
    {
        return _color_driver->setUVCGain(value);
    }
#else
	UERROR("CameraOrbbec: RTAB-Map is not built with libuvc support!");
#endif
	return false;
}

bool CameraOrbbec::setMirroring(bool enabled)
{
    bool ok = false;
#ifdef RTABMAP_OPENNI2
	if(_depth->isValid())
	{
		ok &= _depth->setMirroringEnabled(enabled) == openni::STATUS_OK;
	}
#endif
#ifdef RTABMAP_LIBUVC
    if(_color_driver)
    {
        ok &= _color_driver->setUVCMirror(enabled);
    }
#endif
	return ok;
}

void CameraOrbbec::setOpenNI2StampsAndIDsUsed(bool used)
{
#ifdef RTABMAP_OPENNI2
	_openNI2StampsAndIDsUsed = used;
#endif
}

void CameraOrbbec::setIRDepthShift(int horizontal, int vertical)
{
#ifdef RTABMAP_OPENNI2
	_depthHShift = horizontal;
	_depthVShift = vertical;
#endif
}

void CameraOrbbec::setDepthDecimation(int decimation)
{
#ifdef RTABMAP_OPENNI2
	UASSERT(decimation >= 1);
	_depthDecimation = decimation;
#endif
}

bool CameraOrbbec::init(const std::string & calibrationFolder, const std::string & cameraName)
{
#ifdef RTABMAP_OPENNI2
	openni::OpenNI::initialize();

	openni::Array<openni::DeviceInfo> devices;
	openni::OpenNI::enumerateDevices(&devices);
	for(int i=0; i<devices.getSize(); ++i)
	{
		UINFO("Device %d: Name=%s URI=%s Vendor=%s",
				i,
				devices[i].getName(),
				devices[i].getUri(),
				devices[i].getVendor());
	}
	if(_deviceId.empty() && devices.getSize() == 0)
	{
		UERROR("CameraOrbbec: No device detected!");
		return false;
	}

	openni::Status error = _device->open(_deviceId.empty()?openni::ANY_DEVICE:_deviceId.c_str());
	if(error != openni::STATUS_OK)
	{
		if(!_deviceId.empty())
		{
			UERROR("CameraOrbbec: Cannot open device \"%s\" (error=%d).", _deviceId.c_str(), error);
		}
		else
		{
#ifdef _WIN32
			UERROR("CameraOrbbec: Cannot open device \"%s\" (error=%d).", devices[0].getName(), error);
#else
			UERROR("CameraOrbbec: Cannot open device \"%s\" (error=%d). Verify if \"%s\" is in udev rules: \"/lib/udev/rules.d/40-libopenni2-0.rules\". If not, add it and reboot.", devices[0].getName(), error, devices[0].getUri());
#endif
		}

		_device->close();
		openni::OpenNI::shutdown();
		return false;
	}

	// look for calibration files
	_stereoModel = StereoCameraModel();
	bool hardwareRegistration = true;
	if(!calibrationFolder.empty())
	{
		// we need the serial
		std::string calibrationName = _device->getDeviceInfo().getName();
		if(!cameraName.empty())
		{
			calibrationName = cameraName;
		}
		_stereoModel.setName(calibrationName, "depth", "rgb");
		hardwareRegistration = !_stereoModel.load(calibrationFolder, calibrationName, false);

		if(_type != kTypeColorDepth)
		{
			hardwareRegistration = false;
		}


		if((_type != kTypeColorDepth && !_stereoModel.left().isValidForRectification()) ||
		   (_type == kTypeColorDepth && !_stereoModel.right().isValidForRectification()))
		{
			UWARN("Missing calibration files for camera \"%s\" in \"%s\" folder, default calibration used.",
					calibrationName.c_str(), calibrationFolder.c_str());
		}
		else if(_type == kTypeColorDepth && _stereoModel.right().isValidForRectification() && hardwareRegistration)
		{
			UWARN("Missing extrinsic calibration file for camera \"%s\" in \"%s\" folder, default registration is used even if rgb is rectified!",
					calibrationName.c_str(), calibrationFolder.c_str());
		}
		else if(_type == kTypeColorDepth && _stereoModel.right().isValidForRectification() && !hardwareRegistration)
		{
			UINFO("Custom calibration files for \"%s\" were found in \"%s\" folder. To use "
				  "factory calibration, remove the corresponding files from that directory.", calibrationName.c_str(), calibrationFolder.c_str());
		}
	}

	if(UFile::getExtension(_deviceId).compare("oni")==0)
	{
		if(_device->getPlaybackControl() &&
		   _device->getPlaybackControl()->setRepeatEnabled(false) != openni::STATUS_OK)
		{
			UERROR("CameraOrbbec: Cannot set repeat mode to false.");
			_device->close();
			openni::OpenNI::shutdown();
			return false;
		}
	}
	else if(_type==kTypeColorDepth && hardwareRegistration &&
			!_device->isImageRegistrationModeSupported(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR))
	{
		UERROR("CameraOrbbec: Device doesn't support depth/color registration.");
		_device->close();
		openni::OpenNI::shutdown();
		return false;
	}

	if(_device->getSensorInfo(openni::SENSOR_DEPTH) == NULL)
	{
		UERROR("CameraOrbbec: Cannot get sensor info for depth");
		_device->close();
		openni::OpenNI::shutdown();
		return false;
	}

	if(_depth->create(*_device, openni::SENSOR_DEPTH) != openni::STATUS_OK)
	{
		UERROR("CameraOrbbec: Cannot create depth stream.");
		_device->close();
		openni::OpenNI::shutdown();
		return false;
	}

	astra_camera::UVCCameraConfig uvc_config;
	uvc_config.width = 640;
	uvc_config.height = 480;
	uvc_config.fps = 30;
	uvc_config.format = "rgb";
	uvc_config.retry_count = 500;
    _color_driver = new astra_camera::UVCCameraDriver(uvc_config);
    if (_color_driver->device_handle_ == nullptr) {
        UERROR("CameraOrbbec: Cannot create color stream.");
        _depth->destroy();
        _device->close();
        openni::OpenNI::shutdown();
        return false;
    }

	if(_type==kTypeColorDepth && hardwareRegistration &&
	   _device->setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR ) != openni::STATUS_OK)
	{
		UERROR("CameraOrbbec: Failed to set depth/color registration.");
	}

	if (_device->setDepthColorSyncEnabled(true) != openni::STATUS_OK)
	{
		UERROR("CameraOrbbec: Failed to set depth color sync");
	}

	_depth->setMirroringEnabled(false);
	_color_driver->setUVCMirror(false);

	const openni::Array<openni::VideoMode>& depthVideoModes = _depth->getSensorInfo().getSupportedVideoModes();
	for(int i=0; i<depthVideoModes.getSize(); ++i)
	{
		UINFO("CameraOrbbec: Depth video mode %d: fps=%d, pixel=%d, w=%d, h=%d",
				i,
				depthVideoModes[i].getFps(),
				depthVideoModes[i].getPixelFormat(),
				depthVideoModes[i].getResolutionX(),
				depthVideoModes[i].getResolutionY());
	}

	openni::VideoMode mMode;
	mMode.setFps(30);
	mMode.setResolution(640,480);
	mMode.setPixelFormat(openni::PIXEL_FORMAT_DEPTH_1_MM);
	_depth->setVideoMode(mMode);

	UINFO("CameraOrbbec: Using depth video mode: fps=%d, pixel=%d, w=%d, h=%d, H-FOV=%f rad, V-FOV=%f rad",
			_depth->getVideoMode().getFps(),
			_depth->getVideoMode().getPixelFormat(),
			_depth->getVideoMode().getResolutionX(),
			_depth->getVideoMode().getResolutionY(),
			_depth->getHorizontalFieldOfView(),
			_depth->getVerticalFieldOfView());
	UINFO("CameraOrbbec: Using color video mode: fps=30, pixel=rgb, w=640, h=480, H-FOV=1.23918 rad, V-FOV=0.76270 rad");

	if(_depth->getVideoMode().getResolutionX() != 640 ||
		_depth->getVideoMode().getResolutionY() != 480 ||
		_depth->getVideoMode().getPixelFormat() != openni::PIXEL_FORMAT_DEPTH_1_MM)
	{
		UERROR("Could not set depth format to 640x480 pixel=%d(mm)!",
				openni::PIXEL_FORMAT_DEPTH_1_MM);
		_depth->destroy();
		_device->close();
		openni::OpenNI::shutdown();
		return false;
	}

	UINFO("CameraOrbbec: AutoWhiteBalanceEnabled = 0");
	UINFO("CameraOrbbec: AutoExposureEnabled = 1");
	UINFO("CameraOrbbec: Exposure = %d", _color_driver->getUVCExposure());
	UINFO("CameraOrbbec: GAIN = %d", _color_driver->getUVCGain());

	if(_type==kTypeColorDepth && hardwareRegistration)
	{
		_depthFx = float(640.0f/2.0f) / std::tan(1.23918f/2.0f);
		_depthFy = float(480.0f/2.0f) / std::tan(0.76270/2.0f);
	}
	else
	{
		_depthFx = float(_depth->getVideoMode().getResolutionX()/2) / std::tan(_depth->getHorizontalFieldOfView()/2.0f);
		_depthFy = float(_depth->getVideoMode().getResolutionY()/2) / std::tan(_depth->getVerticalFieldOfView()/2.0f);
	}
	UINFO("depth fx=%f fy=%f", _depthFx, _depthFy);

	if(_type == kTypeIR)
	{
		UWARN("With type IR-only, depth stream will not be started");
	}

	if(_type != kTypeIR && _depth->start() != openni::STATUS_OK)
	{
		UERROR("CameraOrbbec: Cannot start depth and/or color streams.");
		_depth->stop();
		_color_driver->stopStreaming();
		_depth->destroy();
		_device->close();
		openni::OpenNI::shutdown();
		return false;
	}
	_color_driver->startStreaming();

	uSleep(3000); // just to make sure the sensor is correctly initialized and exposure is set

	return true;
#else
	UERROR("CameraOrbbec: RTAB-Map is not built with OpenNI2 support!");
	return false;
#endif
}

bool CameraOrbbec::isCalibrated() const
{
	return true;
}

std::string CameraOrbbec::getSerial() const
{
#ifdef RTABMAP_OPENNI2
	if(_device)
	{
		return _device->getDeviceInfo().getName();
	}
#endif
	return "";
}

SensorData CameraOrbbec::captureImage(SensorCaptureInfo * info)
{
	SensorData data;
#ifdef RTABMAP_OPENNI2
	int readyStream = -1;
	if(_device->isValid() &&
		_depth->isValid() &&
		_color_driver != nullptr &&
		_device->getSensorInfo(openni::SENSOR_DEPTH) != NULL)
	{
		openni::VideoStream* depthStream[] = {_depth};
		if(_type != kTypeIR && openni::OpenNI::waitForAnyStream(depthStream, 1, &readyStream, 5000) != openni::STATUS_OK)
		{
			UWARN("No frames received since the last 5 seconds, end of stream is reached!");
		}
		else
		{
			openni::VideoFrameRef depthFrame;
			if(_type != kTypeIR)
			{
				_depth->readFrame(&depthFrame);
			}
			cv::Mat depth, rgb;
			if(_type == kTypeIR || depthFrame.isValid())
			{
				int h,w;
				if(_type != kTypeIR)
				{
					h=depthFrame.getHeight();
					w=depthFrame.getWidth();
					depth = cv::Mat(h, w, CV_16U, (void*)depthFrame.getData()).clone();
				}
				h=_color_driver->getResolutionY();
				w=_color_driver->getResolutionX();
				cv::Mat tmp(h, w, CV_8UC3, _color_driver->getData());
				if(_type==kTypeColorDepth)
				{
					cv::cvtColor(tmp, rgb, CV_RGB2BGR);
				}
				else // IR
				{
					rgb = tmp.clone();
				}
			}
			UASSERT(_depthFx != 0.0f && _depthFy != 0.0f);
			if(!rgb.empty() && (_type == kTypeIR || !depth.empty()))
			{
				// default calibration
				CameraModel model(
						_depthFx, //fx
						_depthFy, //fy
						float(rgb.cols/2) - 0.5f,  //cx
						float(rgb.rows/2) - 0.5f,  //cy
						this->getLocalTransform(),
						0,
						rgb.size());

				if(_type==kTypeColorDepth)
				{
					if (_depthHShift != 0 || _depthVShift != 0)
					{
						cv::Mat out = cv::Mat::zeros(depth.size(), depth.type());
						depth(cv::Rect(
								_depthHShift>0?_depthHShift:0,
								_depthVShift>0?_depthVShift:0,
								depth.cols - abs(_depthHShift),
								depth.rows - abs(_depthVShift))).copyTo(
							   out(cv::Rect(
								_depthHShift<0?-_depthHShift:0,
								_depthVShift<0?-_depthVShift:0,
								depth.cols - abs(_depthHShift),
								depth.rows - abs(_depthVShift))));
						depth = out;
					}

					if(_stereoModel.right().isValidForRectification())
					{
						rgb = _stereoModel.right().rectifyImage(rgb);
						model = _stereoModel.right();

						if(_stereoModel.left().isValidForRectification() && !_stereoModel.stereoTransform().isNull())
						{
							depth = _stereoModel.left().rectifyImage(depth, 0);
							CameraModel depthModel = _stereoModel.left().scaled(1.0 / double(_depthDecimation));
							depth = util2d::decimate(depth, _depthDecimation);
							depth = util2d::registerDepth(depth, depthModel.K(), rgb.size()/_depthDecimation, _stereoModel.right().scaled(1.0/double(_depthDecimation)).K(), _stereoModel.stereoTransform());
						}
						else if (_depthDecimation > 1)
						{
							depth = util2d::decimate(depth, _depthDecimation);
						}
					}
					else if (_depthDecimation > 1)
					{
						depth = util2d::decimate(depth, _depthDecimation);
					}
				}
				else // IR
				{
					if(_stereoModel.left().isValidForRectification())
					{
						rgb = _stereoModel.left().rectifyImage(rgb);
						if(_type!=kTypeIR)
						{
							depth = _stereoModel.left().rectifyImage(depth, 0);
						}
						model = _stereoModel.left();
					}
				}
				model.setLocalTransform(this->getLocalTransform());

				if(_openNI2StampsAndIDsUsed)
				{
					data = SensorData(rgb, depth, model, depthFrame.getFrameIndex(), double(depthFrame.getTimestamp()) / 1000000.0);
				}
				else
				{
					data = SensorData(rgb, depth, model, this->getNextSeqID(), UTimer::now());
				}
			}
		}
	}
	else
	{
		ULOGGER_WARN("The camera must be initialized before requesting an image.");
	}
#else
	UERROR("CameraOrbbec: RTAB-Map is not built with OpenNI2 support!");
#endif
	return data;
}

} // namespace rtabmap
