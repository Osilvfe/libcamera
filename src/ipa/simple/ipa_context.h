/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024-2026 Red Hat, Inc.
 *
 * Simple pipeline IPA Context
 */

#pragma once

#include <array>
#include <optional>
#include <stdint.h>

#include <libcamera/controls.h>

#include "libcamera/internal/matrix.h"
#include "libcamera/internal/vector.h"

#include <libipa/fc_queue.h>

#include "core_ipa_interface.h"

namespace libcamera {

namespace ipa::soft {

struct IPASessionConfiguration {
	struct {
		int32_t exposureMin, exposureMax;
		double againMin, againMax, again10, againMinStep;
		utils::Duration lineDuration;
	} agc;
	struct {
		std::optional<uint8_t> level;
	} black;
};

struct IPAActiveState {
	struct {
		int32_t exposure;
		double again;
		int32_t manualExposure;
		double manualGain;
		float exposureValue;
		bool autoExposure;
		bool autoGain;
		unsigned int stableFrames;
		bool valid;
	} agc;

	struct {
		uint8_t level;
		int32_t lastExposure;
		double lastGain;
	} blc;

	struct {
		RGB<float> gains;
		RGB<float> manualGains;
		unsigned int temperatureK;
		unsigned int manualTemperatureK;
		int32_t mode;
		unsigned int stableFrames;
		bool autoEnabled;
		bool locked;
	} awb;

	struct {
		bool enabled;
	} lensShading;

	struct {
		float noiseReduction;
		float sharpness;
		float manualSharpness;
		int32_t mode;
		bool manualSharpnessEnabled;
	} detail;

	Matrix<float, 3, 3> combinedMatrix;

	struct {
		float gamma;
		/* 0..2 range, 1.0 = normal */
		std::optional<float> contrast;
		std::optional<float> saturation;
	} knobs;
};

struct IPAFrameContext : public FrameContext {
	Matrix<float, 3, 3> ccm;

	struct {
		int32_t exposure;
		double gain;
		int32_t manualExposure;
		double manualGain;
		float exposureValue;
		bool autoExposure;
		bool autoGain;
	} sensor;

	RGB<float> gains;
	unsigned int colourTemperatureK;
	int32_t awbMode;
	bool awbAutoEnabled;
	bool awbLocked;

	float gamma;
	std::optional<float> contrast;
	std::optional<float> saturation;
	bool lensShadingEnabled;
	float noiseReduction;
	float sharpness;
	int32_t noiseReductionMode;
};

struct IPAContext {
	IPAContext(unsigned int frameContextSize)
		: frameContexts(frameContextSize)
	{
	}

	IPACameraSensorInfo sensorInfo;
	IPASessionConfiguration configuration;
	IPAActiveState activeState;
	FCQueue<IPAFrameContext> frameContexts;
	ControlInfoMap::Map ctrlMap;
	bool ccmEnabled = false;
};

} /* namespace ipa::soft */

} /* namespace libcamera */
