/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2026 Osilvfe
 */

#include "noise_reduction.h"

#include <algorithm>
#include <errno.h>

#include <libcamera/base/log.h>
#include <libcamera/control_ids.h>

namespace libcamera {

namespace ipa::soft::algorithms {

LOG_DEFINE_CATEGORY(IPASoftNoiseReduction)

int NoiseReduction::init(IPAContext &context, const ValueNode &tuningData)
{
	defaultSharpness_ = tuningData["sharpness"].get<float>(1.0f);
	lowLightSharpness_ = tuningData["lowLightSharpness"].get<float>(0.45f);
	defaultDenoise_ = tuningData["denoise"].get<float>(0.08f);
	lowLightDenoise_ = tuningData["lowLightDenoise"].get<float>(0.55f);
	gainStart_ = tuningData["gainStart"].get<float>(2.0f);
	gainEnd_ = tuningData["gainEnd"].get<float>(10.0f);
	exposureStartMs_ = tuningData["exposureStartMs"].get<float>(15.0f);
	exposureEndMs_ = tuningData["exposureEndMs"].get<float>(80.0f);
	smoothing_ = tuningData["smoothing"].get<float>(0.15f);

	if (defaultSharpness_ < 0.0f || defaultSharpness_ > 2.0f ||
	    lowLightSharpness_ < 0.0f || lowLightSharpness_ > 2.0f ||
	    defaultDenoise_ < 0.0f || defaultDenoise_ > 1.0f ||
	    lowLightDenoise_ < 0.0f || lowLightDenoise_ > 1.0f ||
	    gainStart_ < 1.0f || gainStart_ >= gainEnd_ ||
	    exposureStartMs_ < 0.0f || exposureStartMs_ >= exposureEndMs_ ||
	    smoothing_ <= 0.0f || smoothing_ > 1.0f)
		return -EINVAL;

	context.ctrlMap[&controls::Sharpness] =
		ControlInfo(0.0f, 2.0f, defaultSharpness_);
	context.ctrlMap[&controls::draft::NoiseReductionMode] = ControlInfo(
		controls::draft::NoiseReductionModeValues,
		ControlValue(controls::draft::NoiseReductionModeFast));

	return 0;
}

int NoiseReduction::configure(IPAContext &context,
			      [[maybe_unused]] const IPAConfigInfo &configInfo)
{
	auto &detail = context.activeState.detail;
	detail.noiseReduction = defaultDenoise_;
	detail.sharpness = defaultSharpness_;
	detail.manualSharpness = defaultSharpness_;
	detail.mode = controls::draft::NoiseReductionModeFast;
	detail.manualSharpnessEnabled = false;

	return 0;
}

void NoiseReduction::queueRequest(IPAContext &context,
				  [[maybe_unused]] const uint32_t frame,
				  [[maybe_unused]] IPAFrameContext &frameContext,
				  const ControlList &controls)
{
	auto &detail = context.activeState.detail;
	const auto &mode = controls.get(controls::draft::NoiseReductionMode);
	if (mode)
		detail.mode = *mode;

	const auto &sharpness = controls.get(controls::Sharpness);
	if (sharpness) {
		detail.manualSharpness = std::clamp(*sharpness, 0.0f, 2.0f);
		detail.manualSharpnessEnabled = true;
	}
}

void NoiseReduction::prepare(IPAContext &context,
			     [[maybe_unused]] const uint32_t frame,
			     IPAFrameContext &frameContext,
			     DebayerParams *params)
{
	auto &detail = context.activeState.detail;
	const auto &agc = context.activeState.agc;
	const float exposureMs =
		context.configuration.agc.lineDuration.get<std::milli>() * agc.exposure;
	const float gainWeight = std::clamp(
		(static_cast<float>(agc.again) - gainStart_) / (gainEnd_ - gainStart_),
		0.0f, 1.0f);
	const float exposureWeight = std::clamp(
		(exposureMs - exposureStartMs_) / (exposureEndMs_ - exposureStartMs_),
		0.0f, 1.0f);
	const float lowLight = std::max(gainWeight, exposureWeight);

	float denoise = defaultDenoise_ +
			lowLight * (lowLightDenoise_ - defaultDenoise_);
	switch (detail.mode) {
	case controls::draft::NoiseReductionModeOff:
		denoise = 0.0f;
		break;
	case controls::draft::NoiseReductionModeMinimal:
		denoise *= 0.5f;
		break;
	case controls::draft::NoiseReductionModeHighQuality:
		denoise = std::min(1.0f, denoise * 1.25f);
		break;
	default:
		break;
	}

	const float sharpness = defaultSharpness_ +
				lowLight * (lowLightSharpness_ - defaultSharpness_);
	detail.noiseReduction += smoothing_ * (denoise - detail.noiseReduction);
	if (detail.manualSharpnessEnabled)
		detail.sharpness = detail.manualSharpness;
	else
		detail.sharpness += smoothing_ * (sharpness - detail.sharpness);

	frameContext.noiseReduction = detail.noiseReduction;
	frameContext.sharpness = detail.sharpness;
	frameContext.noiseReductionMode = detail.mode;
	params->noiseReduction = detail.noiseReduction;
	params->sharpness = detail.sharpness;
}

void NoiseReduction::process([[maybe_unused]] IPAContext &context,
			     [[maybe_unused]] const uint32_t frame,
			     IPAFrameContext &frameContext,
			     [[maybe_unused]] const SwIspStats *stats,
			     ControlList &metadata)
{
	metadata.set(controls::Sharpness, frameContext.sharpness);
	metadata.set(controls::draft::NoiseReductionMode,
		     frameContext.noiseReductionMode);
}

REGISTER_IPA_ALGORITHM(NoiseReduction, "NoiseReduction")

} /* namespace ipa::soft::algorithms */

} /* namespace libcamera */
