/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2026, Osilvfe
 *
 * A camera flash controller
 */
#pragma once

#include <memory>
#include <stdint.h>
#include <string>

#include <libcamera/base/class.h>
#include <libcamera/base/log.h>

#include <libcamera/controls.h>

namespace libcamera {

class MediaEntity;
class V4L2Subdevice;

class CameraFlash : protected Loggable
{
public:
	explicit CameraFlash(const MediaEntity *entity);
	~CameraFlash();

	int init();

	int setTorch(int32_t intensity);
	int prepareFlash(int32_t intensity, int32_t timeout);
	int strobe();
	int stop();
	int status(bool *strobing, int32_t *fault);

	const std::string &model() const { return model_; }
	const ControlInfoMap &controls() const;

protected:
	std::string logPrefix() const override;

private:
	LIBCAMERA_DISABLE_COPY_AND_MOVE(CameraFlash)

	int setControl(uint32_t id, int32_t value);
	int setMode(int32_t mode);
	int validateFlashDriver();
	int32_t normalize(uint32_t id, int32_t value) const;

	const MediaEntity *entity_;
	std::unique_ptr<V4L2Subdevice> subdev_;
	std::string model_;
};

} /* namespace libcamera */
