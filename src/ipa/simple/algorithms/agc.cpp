/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Red Hat Inc.
 *
 * Exposure and gain
 */

#include "agc.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdint.h>

#include <libcamera/base/log.h>
#include <libcamera/base/utils.h>

#include <libcamera/control_ids.h>

#include "control_ids.h"

namespace libcamera {

using namespace std::literals::chrono_literals;

LOG_DEFINE_CATEGORY(IPASoftExposure)

namespace ipa::soft::algorithms {

/*
 * The number of bins to use for the optimal exposure calculations.
 */
static constexpr unsigned int kExposureBinsCount = 5;

/*
 * The exposure is optimal when the mean sample value of the histogram is
 * in the middle of the range.
 */
static constexpr float kExposureOptimal = kExposureBinsCount / 2.0;

/*
 * This implements the hysteresis for the exposure adjustment.
 * It is small enough to have the exposure close to the optimal, and is big
 * enough to prevent the exposure from wobbling around the optimal value.
 */
static constexpr float kExposureSatisfactory = 0.2;

/*
 * Proportional gain for exposure/gain adjustment. Maps the MSV error to a
 * multiplicative correction factor:
 *
 *   factor = 1.0 + kExpProportionalGain * error
 *
 * With kExpProportionalGain = 0.04:
 *   - max error ~2.5 -> factor 1.10 (~10% step, same as before)
 *   - error 1.0      -> factor 1.04 (~4% step)
 *   - error 0.3      -> factor 1.012 (~1.2% step)
 *
 * This replaces the fixed 10% bang-bang step with a proportional correction
 * that converges smoothly and avoids overshooting near the target.
 */
static constexpr float kExpProportionalGain = 0.04;

/*
 * Maximum multiplicative step per frame, to bound the correction when the
 * scene changes dramatically.
 */
static constexpr float kExpMaxStep = 0.15;

Agc::Agc()
{
}

int Agc::init(IPAContext &context, [[maybe_unused]] const ValueNode &tuningData)
{
	context.ctrlMap[&controls::ExposureTimeMode] =
		ControlInfo({ { ControlValue(controls::ExposureTimeModeAuto),
				ControlValue(controls::ExposureTimeModeManual) } },
			    ControlValue(controls::ExposureTimeModeAuto));
	context.ctrlMap[&controls::AnalogueGainMode] =
		ControlInfo({ { ControlValue(controls::AnalogueGainModeAuto),
				ControlValue(controls::AnalogueGainModeManual) } },
			    ControlValue(controls::AnalogueGainModeAuto));
	context.ctrlMap[&controls::AeEnable] = ControlInfo(false, true, true);
	context.ctrlMap[&controls::ExposureValue] = ControlInfo(-2.0f, 2.0f, 0.0f);
	context.ctrlMap[&controls::AeState] =
		ControlInfo({ { ControlValue(controls::AeStateIdle),
				ControlValue(controls::AeStateSearching),
				ControlValue(controls::AeStateConverged) } },
			    ControlValue(controls::AeStateSearching));

	const IPACameraSensorInfo &sensorInfo = context.sensorInfo;
	if (!sensorInfo.pixelRate || !sensorInfo.minLineLength) {
		LOG(IPASoftExposure, Warning)
			<< "Missing sensor timing information, "
			<< "FrameDurationLimits not exposed";
		return 0;
	}

	utils::Duration lineDuration =
		sensorInfo.minLineLength * 1.0s / sensorInfo.pixelRate;
	utils::Duration minDuration = lineDuration * sensorInfo.minFrameLength;
	utils::Duration maxDuration = lineDuration * sensorInfo.maxFrameLength;
	int64_t minFrameDuration = minDuration.get<std::micro>();
	int64_t maxFrameDuration = maxDuration.get<std::micro>();
	context.ctrlMap[&controls::FrameDurationLimits] = ControlInfo(
		minFrameDuration, maxFrameDuration, minFrameDuration);

	return 0;
}

int Agc::configure(IPAContext &context,
		   [[maybe_unused]] const IPAConfigInfo &configInfo)
{
	auto &agc = context.activeState.agc;

	agc.exposure = std::clamp<int32_t>(
		static_cast<int32_t>(10ms / context.configuration.agc.lineDuration),
		context.configuration.agc.exposureMin,
		context.configuration.agc.exposureMax);
	agc.again = context.configuration.agc.againMin;
	agc.manualExposure = agc.exposure;
	agc.manualGain = agc.again;
	agc.exposureValue = 0.0f;
	agc.autoExposure = true;
	agc.autoGain = true;
	agc.stableFrames = 0;
	agc.valid = false;

	const auto it = context.ctrlMap.find(&controls::FrameDurationLimits);
	const auto &cfg = context.configuration.agc;
	if (it != context.ctrlMap.end() && cfg.vblankSupported) {
		agc.minFrameDuration =
			std::chrono::microseconds(it->second.min().get<int64_t>());
		agc.maxFrameDuration =
			std::chrono::microseconds(it->second.max().get<int64_t>());
	} else {
		agc.minFrameDuration = cfg.lineDuration *
				       (cfg.frameHeight + cfg.vblankDef);
		agc.maxFrameDuration = agc.minFrameDuration;
	}
	agc.vblank = cfg.vblankDef;

	return 0;
}

void Agc::queueRequest(IPAContext &context,
		       [[maybe_unused]] const uint32_t frame,
		       IPAFrameContext &frameContext,
		       const ControlList &controls)
{
	auto &agc = context.activeState.agc;

	const auto &frameDurationLimits =
		controls.get(controls::FrameDurationLimits);
	if (frameDurationLimits && context.configuration.agc.vblankSupported) {
		const auto it = context.ctrlMap.find(&controls::FrameDurationLimits);
		if (it != context.ctrlMap.end()) {
			const ControlInfo &limits = it->second;
			int64_t minFrameDuration = std::clamp(
				frameDurationLimits->front(),
				limits.min().get<int64_t>(),
				limits.max().get<int64_t>());
			int64_t maxFrameDuration = std::clamp(
				frameDurationLimits->back(),
				limits.min().get<int64_t>(),
				limits.max().get<int64_t>());
			if (maxFrameDuration < minFrameDuration)
				maxFrameDuration = minFrameDuration;

			agc.minFrameDuration =
				std::chrono::microseconds(minFrameDuration);
			agc.maxFrameDuration =
				std::chrono::microseconds(maxFrameDuration);
		}
	}

	frameContext.agc.minFrameDuration = agc.minFrameDuration;
	frameContext.agc.maxFrameDuration = agc.maxFrameDuration;

	const auto &exposureMode = controls.get(controls::ExposureTimeMode);
	if (exposureMode) {
		const bool automatic = *exposureMode == controls::ExposureTimeModeAuto;
		if (automatic != agc.autoExposure) {
			agc.autoExposure = automatic;
			agc.stableFrames = 0;
			if (!automatic)
				agc.manualExposure = agc.exposure;
		}
	}

	const auto &gainMode = controls.get(controls::AnalogueGainMode);
	if (gainMode) {
		const bool automatic = *gainMode == controls::AnalogueGainModeAuto;
		if (automatic != agc.autoGain) {
			agc.autoGain = automatic;
			agc.stableFrames = 0;
			if (!automatic)
				agc.manualGain = agc.again;
		}
	}

	const auto &exposureTime = controls.get(controls::ExposureTime);
	if (exposureTime && !agc.autoExposure) {
		int32_t vblankLo;
		int32_t vblankHi;
		vblankRange(context, frameContext, vblankLo, vblankHi);
		agc.manualExposure = std::clamp<int32_t>(
			std::chrono::microseconds(*exposureTime) /
				context.configuration.agc.lineDuration,
			context.configuration.agc.exposureMin,
			exposureMaxForVblank(context, vblankHi));
	}

	const auto &gain = controls.get(controls::AnalogueGain);
	if (gain && !agc.autoGain)
		agc.manualGain = std::clamp<double>(*gain,
						    context.configuration.agc.againMin,
						    context.configuration.agc.againMax);

	const auto &exposureValue = controls.get(controls::ExposureValue);
	if (exposureValue && *exposureValue != agc.exposureValue) {
		agc.exposureValue = std::clamp(*exposureValue, -2.0f, 2.0f);
		agc.stableFrames = 0;
	}

	frameContext.sensor.autoExposure = agc.autoExposure;
	frameContext.sensor.autoGain = agc.autoGain;
	frameContext.sensor.manualExposure = agc.manualExposure;
	frameContext.sensor.manualGain = agc.manualGain;
	frameContext.sensor.exposureValue = agc.exposureValue;
}

void Agc::vblankRange(const IPAContext &context,
		      const IPAFrameContext &frameContext,
		      int32_t &vblankLo, int32_t &vblankHi) const
{
	const auto &cfg = context.configuration.agc;

	if (!cfg.vblankSupported) {
		vblankLo = vblankHi = cfg.vblankDef;
		return;
	}

	const double minLines =
		std::round(frameContext.agc.minFrameDuration / cfg.lineDuration);
	const double maxLines =
		std::round(frameContext.agc.maxFrameDuration / cfg.lineDuration);
	const int64_t height = cfg.frameHeight;

	vblankLo = static_cast<int32_t>(std::clamp<int64_t>(
		static_cast<int64_t>(minLines) - height,
		cfg.vblankMin, cfg.vblankMax));
	vblankHi = static_cast<int32_t>(std::clamp<int64_t>(
		static_cast<int64_t>(maxLines) - height,
		cfg.vblankMin, cfg.vblankMax));
	if (vblankHi < vblankLo)
		vblankHi = vblankLo;
}

int32_t Agc::exposureMaxForVblank(const IPAContext &context,
				  int32_t vblank) const
{
	const auto &cfg = context.configuration.agc;

	if (!cfg.vblankSupported)
		return cfg.exposureMax;

	return std::max(cfg.exposureMin,
			static_cast<int32_t>(cfg.frameHeight) + vblank -
				cfg.exposureMargin);
}

void Agc::updateVblank(const IPAContext &context,
		       IPAFrameContext &frameContext) const
{
	const auto &cfg = context.configuration.agc;
	int32_t vblankLo;
	int32_t vblankHi;
	vblankRange(context, frameContext, vblankLo, vblankHi);

	frameContext.sensor.exposure = std::clamp(
		frameContext.sensor.exposure, cfg.exposureMin,
		exposureMaxForVblank(context, vblankHi));
	frameContext.sensor.vblank = std::clamp(
		frameContext.sensor.exposure + cfg.exposureMargin -
			static_cast<int32_t>(cfg.frameHeight),
		vblankLo, vblankHi);
}

void Agc::updateExposure(IPAContext &context, IPAFrameContext &frameContext, double exposureMSV)
{
	int32_t &exposure = frameContext.sensor.exposure;
	double &again = frameContext.sensor.gain;
	int32_t vblankLo;
	int32_t vblankHi;
	vblankRange(context, frameContext, vblankLo, vblankHi);
	const int32_t exposureMax = exposureMaxForVblank(context, vblankHi);

	const double compensatedMSV =
		exposureMSV / std::exp2(frameContext.sensor.exposureValue);
	double error = kExposureOptimal - compensatedMSV;

	if (std::abs(error) <= kExposureSatisfactory) {
		context.activeState.agc.stableFrames++;
		updateVblank(context, frameContext);
		context.activeState.agc.exposure = exposure;
		context.activeState.agc.again = again;
		context.activeState.agc.vblank = frameContext.sensor.vblank;
		return;
	}

	context.activeState.agc.stableFrames = 0;

	/*
	 * Compute a proportional correction factor. The sign of the error
	 * determines the direction: positive error means too dark (increase),
	 * negative means too bright (decrease).
	 */
	float step = std::clamp(static_cast<float>(error) * kExpProportionalGain,
				-kExpMaxStep, kExpMaxStep);
	float factor = 1.0f + step;

	if (factor > 1.0f) {
		/* Scene too dark: increase exposure first, then gain. */
		if (frameContext.sensor.autoExposure &&
		    exposure < exposureMax) {
			int32_t next = static_cast<int32_t>(exposure * factor);
			exposure = std::max(next, exposure + 1);
		} else if (frameContext.sensor.autoGain) {
			double next = again * factor;
			if (next - again < context.configuration.agc.againMinStep)
				again += context.configuration.agc.againMinStep;
			else
				again = next;
		}
	} else {
		/* Scene too bright: decrease gain first, then exposure. */
		if (frameContext.sensor.autoGain &&
		    again > context.configuration.agc.again10) {
			double next = again * factor;
			if (again - next < context.configuration.agc.againMinStep)
				again -= context.configuration.agc.againMinStep;
			else
				again = next;
		} else if (frameContext.sensor.autoExposure) {
			int32_t next = static_cast<int32_t>(exposure * factor);
			exposure = std::min(next, exposure - 1);
		}
	}

	exposure = std::clamp(exposure, context.configuration.agc.exposureMin,
			      exposureMax);
	again = std::clamp(again, context.configuration.agc.againMin,
			   context.configuration.agc.againMax);

	updateVblank(context, frameContext);
	context.activeState.agc.exposure = exposure;
	context.activeState.agc.again = again;
	context.activeState.agc.vblank = frameContext.sensor.vblank;

	LOG(IPASoftExposure, Debug)
		<< "exposureMSV " << exposureMSV << " EV "
		<< frameContext.sensor.exposureValue
		<< " error " << error << " factor " << factor
		<< " exp " << exposure << " again " << again
		<< " vblank " << frameContext.sensor.vblank
		<< " (" << vblankLo << "-" << vblankHi << ")";
}

void Agc::process(IPAContext &context,
		  [[maybe_unused]] const uint32_t frame,
		  IPAFrameContext &frameContext,
		  const SwIspStats *stats,
		  ControlList &metadata)
{
	utils::Duration exposureTime =
		context.configuration.agc.lineDuration * frameContext.sensor.exposure;
	metadata.set(controls::ExposureTime, exposureTime.get<std::micro>());
	metadata.set(controls::AnalogueGain, frameContext.sensor.gain);
	const auto &cfg = context.configuration.agc;
	if (cfg.vblankSupported) {
		frameContext.agc.frameDuration = cfg.lineDuration *
						 (cfg.frameHeight + frameContext.sensor.vblank);
		metadata.set(controls::FrameDuration,
			     frameContext.agc.frameDuration.get<std::micro>());
	}
	metadata.set(controls::ExposureTimeMode,
		     frameContext.sensor.autoExposure
			     ? controls::ExposureTimeModeAuto
			     : controls::ExposureTimeModeManual);
	metadata.set(controls::AnalogueGainMode,
		     frameContext.sensor.autoGain
			     ? controls::AnalogueGainModeAuto
			     : controls::AnalogueGainModeManual);
	metadata.set(controls::ExposureValue, frameContext.sensor.exposureValue);
	metadata.set(controls::AeState,
		     !frameContext.sensor.autoExposure && !frameContext.sensor.autoGain
			     ? controls::AeStateIdle
		     : context.activeState.agc.stableFrames >= 3
			     ? controls::AeStateConverged
			     : controls::AeStateSearching);

	if (!context.activeState.agc.valid) {
		/*
		 * Init active-state from sensor values in case updateExposure()
		 * does not run for the first frame.
		 */
		context.activeState.agc.exposure = frameContext.sensor.exposure;
		context.activeState.agc.again = frameContext.sensor.gain;
		context.activeState.agc.vblank = frameContext.sensor.vblank;
		context.activeState.agc.valid = true;
	}

	if (!stats->valid) {
		/*
		 * Use the new exposure and gain values calculated the last time
		 * there were valid stats.
		 */
		frameContext.sensor.exposure = frameContext.sensor.autoExposure
						       ? context.activeState.agc.exposure
						       : frameContext.sensor.manualExposure;
		frameContext.sensor.gain = frameContext.sensor.autoGain
						   ? context.activeState.agc.again
						   : frameContext.sensor.manualGain;
		updateVblank(context, frameContext);
		context.activeState.agc.exposure = frameContext.sensor.exposure;
		context.activeState.agc.again = frameContext.sensor.gain;
		context.activeState.agc.vblank = frameContext.sensor.vblank;
		return;
	}

	if (!frameContext.sensor.autoExposure)
		frameContext.sensor.exposure = frameContext.sensor.manualExposure;
	if (!frameContext.sensor.autoGain)
		frameContext.sensor.gain = frameContext.sensor.manualGain;

	if (!frameContext.sensor.autoExposure && !frameContext.sensor.autoGain) {
		updateVblank(context, frameContext);
		context.activeState.agc.exposure = frameContext.sensor.exposure;
		context.activeState.agc.again = frameContext.sensor.gain;
		context.activeState.agc.vblank = frameContext.sensor.vblank;
		return;
	}

	/*
	 * Calculate Mean Sample Value (MSV) according to formula from:
	 * https://www.araa.asn.au/acra/acra2007/papers/paper84final.pdf
	 */
	const auto &histogram = stats->yHistogram;
	const unsigned int blackLevelHistIdx =
		context.activeState.blc.level / (256 / SwIspStats::kYHistogramSize);
	const unsigned int histogramSize =
		SwIspStats::kYHistogramSize - blackLevelHistIdx;
	const unsigned int yHistValsPerBin = histogramSize / kExposureBinsCount;
	const unsigned int yHistValsPerBinMod =
		histogramSize / (histogramSize % kExposureBinsCount + 1);
	int exposureBins[kExposureBinsCount] = {};
	unsigned int denom = 0;
	unsigned int num = 0;

	if (yHistValsPerBin == 0) {
		LOG(IPASoftExposure, Debug)
			<< "Not adjusting exposure due to insufficient histogram data";
		return;
	}

	for (unsigned int i = 0; i < histogramSize; i++) {
		unsigned int idx = (i - (i / yHistValsPerBinMod)) / yHistValsPerBin;
		exposureBins[idx] += histogram[blackLevelHistIdx + i];
	}

	for (unsigned int i = 0; i < kExposureBinsCount; i++) {
		LOG(IPASoftExposure, Debug) << i << ": " << exposureBins[i];
		denom += exposureBins[i];
		num += exposureBins[i] * (i + 1);
	}

	float exposureMSV = (denom == 0 ? 0 : static_cast<float>(num) / denom);
	updateExposure(context, frameContext, exposureMSV);
}

REGISTER_IPA_ALGORITHM(Agc, "Agc")

} /* namespace ipa::soft::algorithms */

} /* namespace libcamera */
