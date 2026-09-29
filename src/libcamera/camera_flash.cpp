/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2026, Osilvfe
 *
 * A camera flash controller
 */

#include "libcamera/internal/camera_flash.h"

#include <algorithm>
#include <array>
#include <errno.h>

#include <libcamera/base/utils.h>

#include "libcamera/internal/v4l2_subdevice.h"

namespace libcamera {

LOG_DEFINE_CATEGORY(CameraFlash)

CameraFlash::CameraFlash(const MediaEntity *entity)
	: entity_(entity)
{
}

CameraFlash::~CameraFlash() = default;

int CameraFlash::init()
{
	if (entity_->function() != MEDIA_ENT_F_FLASH) {
		LOG(CameraFlash, Error)
			<< "Invalid flash function "
			<< utils::hex(entity_->function());
		return -EINVAL;
	}

	subdev_ = std::make_unique<V4L2Subdevice>(entity_);
	int ret = subdev_->open();
	if (ret < 0)
		return ret;

	ret = validateFlashDriver();
	if (ret)
		return ret;

	model_ = subdev_->model();
	return stop();
}

int CameraFlash::setControl(uint32_t id, int32_t value)
{
	ControlList controls(subdev_->controls());
	controls.set(id, value);
	return subdev_->setControls(&controls);
}

int CameraFlash::setMode(int32_t mode)
{
	return setControl(V4L2_CID_FLASH_LED_MODE, mode);
}

int32_t CameraFlash::normalize(uint32_t id, int32_t value) const
{
	const struct v4l2_query_ext_ctrl *info = subdev_->controlInfo(id);
	if (!info)
		return value;

	value = std::clamp<int64_t>(value, info->minimum, info->maximum);
	if (info->step > 1)
		value = info->minimum +
			((value - info->minimum) / info->step) * info->step;

	return value;
}

int CameraFlash::setTorch(int32_t intensity)
{
	int ret = setControl(V4L2_CID_FLASH_TORCH_INTENSITY,
			     normalize(V4L2_CID_FLASH_TORCH_INTENSITY,
				       intensity));
	if (ret)
		return ret;

	/* The LED mode is set separately to accommodate the PM8550 driver. */
	ret = setMode(V4L2_FLASH_LED_MODE_TORCH);
	if (ret)
		stop();

	return ret;
}

int CameraFlash::prepareFlash(int32_t intensity, int32_t timeout)
{
	int ret = stop();
	if (ret)
		return ret;

	ret = setControl(V4L2_CID_FLASH_INTENSITY,
			 normalize(V4L2_CID_FLASH_INTENSITY, intensity));
	if (ret)
		return ret;

	ret = setControl(V4L2_CID_FLASH_TIMEOUT,
			 normalize(V4L2_CID_FLASH_TIMEOUT, timeout));
	if (ret)
		return ret;

	if (subdev_->controls().count(V4L2_CID_FLASH_STROBE_SOURCE)) {
		ret = setControl(V4L2_CID_FLASH_STROBE_SOURCE,
				 V4L2_FLASH_STROBE_SOURCE_SOFTWARE);
		if (ret)
			return ret;
	}

	/* Configure Flash only after all intensity and timing controls. */
	ret = setMode(V4L2_FLASH_LED_MODE_FLASH);
	if (ret)
		stop();

	return ret;
}

int CameraFlash::strobe()
{
	return setControl(V4L2_CID_FLASH_STROBE, 1);
}

int CameraFlash::stop()
{
	bool strobing = false;
	int32_t fault;
	int statusRet = status(&strobing, &fault);

	if (statusRet || strobing)
		setControl(V4L2_CID_FLASH_STROBE_STOP, 1);

	/* LED_MODE_NONE is sufficient when no strobe is currently active. */
	return setMode(V4L2_FLASH_LED_MODE_NONE);
}

int CameraFlash::status(bool *strobing, int32_t *fault)
{
	static constexpr std::array<uint32_t, 2> ids = {
		V4L2_CID_FLASH_STROBE_STATUS,
		V4L2_CID_FLASH_FAULT,
	};
	ControlList controls = subdev_->getControls(ids);
	if (controls.empty())
		return -EIO;

	/* V4L2Device currently stores scalar boolean reads as int32_t. */
	*strobing = controls.get(V4L2_CID_FLASH_STROBE_STATUS).get<int32_t>() != 0;
	*fault = controls.get(V4L2_CID_FLASH_FAULT).get<int32_t>();
	return 0;
}

int CameraFlash::validateFlashDriver()
{
	static constexpr std::array<uint32_t, 8> mandatoryControls = {
		V4L2_CID_FLASH_LED_MODE,
		V4L2_CID_FLASH_STROBE,
		V4L2_CID_FLASH_STROBE_STOP,
		V4L2_CID_FLASH_STROBE_STATUS,
		V4L2_CID_FLASH_TIMEOUT,
		V4L2_CID_FLASH_INTENSITY,
		V4L2_CID_FLASH_TORCH_INTENSITY,
		V4L2_CID_FLASH_FAULT,
	};

	const ControlInfoMap &controls = subdev_->controls();
	for (uint32_t id : mandatoryControls) {
		if (controls.count(id))
			continue;

		LOG(CameraFlash, Error)
			<< "Mandatory V4L2 control " << utils::hex(id)
			<< " not available";
		return -EINVAL;
	}

	return 0;
}

std::string CameraFlash::logPrefix() const
{
	return "'" + entity_->name() + "'";
}

const ControlInfoMap &CameraFlash::controls() const
{
	return subdev_->controls();
}

} /* namespace libcamera */
