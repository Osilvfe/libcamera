/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024-2026 Red Hat Inc.
 *
 * Auto white balance
 */

#include "awb.h"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <stdint.h>

#include <libcamera/base/log.h>

#include <libcamera/control_ids.h>

#include "libipa/colours.h"
#include "simple/ipa_context.h"

namespace libcamera {

LOG_DEFINE_CATEGORY(IPASoftAwb)

namespace ipa::soft::algorithms {

namespace {

constexpr unsigned int kMinColourTemperature = 2500;
constexpr unsigned int kMaxColourTemperature = 8000;
constexpr unsigned int kDefaultColourTemperature = 5000;

}

int Awb::init(IPAContext &context, const ValueNode &tuningData)
{
	int ret = colourGains_.readYaml(tuningData["colourGains"], "ct", "gains");
	if (ret < 0) {
		LOG(IPASoftAwb, Error) << "Failed to parse AWB colour gain curve";
		return ret;
	}

	smoothing_ = tuningData["smoothing"].get<float>(0.18f);
	if (smoothing_ <= 0.0f || smoothing_ > 1.0f)
		return -EINVAL;

	auto &cmap = context.ctrlMap;
	cmap[&controls::AwbEnable] = ControlInfo(false, true, true);
	cmap[&controls::AwbMode] = ControlInfo(
		{ { ControlValue(controls::AwbAuto),
		    ControlValue(controls::AwbIncandescent),
		    ControlValue(controls::AwbTungsten),
		    ControlValue(controls::AwbFluorescent),
		    ControlValue(controls::AwbIndoor),
		    ControlValue(controls::AwbDaylight),
		    ControlValue(controls::AwbCloudy),
		    ControlValue(controls::AwbCustom) } },
		ControlValue(controls::AwbAuto));
	cmap[&controls::AwbLocked] = ControlInfo(false, true, false);
	cmap[&controls::ColourGains] = ControlInfo(
		0.1f, 4.0f, Span<const float, 2>{ { 1.0f, 1.0f } });
	cmap[&controls::ColourTemperature] =
		ControlInfo(static_cast<int32_t>(kMinColourTemperature),
			    static_cast<int32_t>(kMaxColourTemperature),
			    static_cast<int32_t>(kDefaultColourTemperature));

	return 0;
}

int Awb::configure(IPAContext &context,
		   [[maybe_unused]] const IPAConfigInfo &configInfo)
{
	auto &gains = context.activeState.awb.gains;
	gains = { { 1.0, 1.0, 1.0 } };
	context.activeState.awb.manualGains = gains;
	context.activeState.awb.temperatureK = kDefaultColourTemperature;
	context.activeState.awb.manualTemperatureK = kDefaultColourTemperature;
	context.activeState.awb.mode = controls::AwbAuto;
	context.activeState.awb.stableFrames = 0;
	context.activeState.awb.autoEnabled = true;
	context.activeState.awb.locked = false;

	return 0;
}

std::optional<RGB<float>> Awb::gainsFromTemperature(unsigned int temperatureK)
{
	const Vector<double, 2> gains = colourGains_.getInterpolated(
		std::clamp(temperatureK, kMinColourTemperature, kMaxColourTemperature));
	return RGB<float>{ { static_cast<float>(gains[0]), 1.0f,
			    static_cast<float>(gains[1]) } };
}

unsigned int Awb::modeTemperature(int32_t mode) const
{
	switch (mode) {
	case controls::AwbIncandescent:
		return 2850;
	case controls::AwbTungsten:
		return 3200;
	case controls::AwbFluorescent:
		return 4000;
	case controls::AwbIndoor:
		return 4200;
	case controls::AwbDaylight:
		return 5500;
	case controls::AwbCloudy:
		return 6500;
	default:
		return 0;
	}
}

void Awb::queueRequest(IPAContext &context,
		       [[maybe_unused]] const uint32_t frame,
		       IPAFrameContext &frameContext,
		       const ControlList &controls)
{
	auto &awb = context.activeState.awb;

	const auto &enable = controls.get(controls::AwbEnable);
	if (enable && *enable != awb.autoEnabled) {
		awb.autoEnabled = *enable;
		awb.stableFrames = 0;
		awb.locked = false;
		if (!*enable) {
			awb.manualGains = awb.gains;
			awb.manualTemperatureK = awb.temperatureK;
		}
	}

	const auto &mode = controls.get(controls::AwbMode);
	if (mode && *mode != awb.mode) {
		awb.mode = *mode;
		awb.stableFrames = 0;
		awb.locked = false;
	}

	if (!awb.autoEnabled || awb.mode == controls::AwbCustom) {
		const auto &gains = controls.get(controls::ColourGains);
		if (gains) {
			awb.manualGains = { {
				std::clamp((*gains)[0], 0.1f, 4.0f), 1.0f,
				std::clamp((*gains)[1], 0.1f, 4.0f),
			} };
			const RGB<double> inverse{ {
				1.0 / awb.manualGains.r(), 1.0,
				1.0 / awb.manualGains.b(),
			} };
			awb.manualTemperatureK = std::clamp<unsigned int>(
				estimateCCT(inverse), kMinColourTemperature,
				kMaxColourTemperature);
		}

		const auto &temperature = controls.get(controls::ColourTemperature);
		if (temperature) {
			awb.manualTemperatureK = std::clamp<unsigned int>(
				*temperature, kMinColourTemperature, kMaxColourTemperature);
			awb.manualGains = *gainsFromTemperature(awb.manualTemperatureK);
		}
	}

	const unsigned int presetTemperature = modeTemperature(awb.mode);
	if (awb.autoEnabled && presetTemperature) {
		awb.gains = *gainsFromTemperature(presetTemperature);
		awb.temperatureK = presetTemperature;
		awb.locked = true;
	}

	if (!awb.autoEnabled || awb.mode == controls::AwbCustom) {
		awb.gains = awb.manualGains;
		awb.temperatureK = awb.manualTemperatureK;
		awb.locked = true;
	}

	frameContext.gains = awb.gains;
	frameContext.colourTemperatureK = awb.temperatureK;
	frameContext.awbMode = awb.mode;
	frameContext.awbAutoEnabled = awb.autoEnabled;
	frameContext.awbLocked = awb.locked;
}

void Awb::prepare(IPAContext &context,
		  [[maybe_unused]] const uint32_t frame,
		  IPAFrameContext &frameContext,
		  DebayerParams *params)
{
	auto &gains = context.activeState.awb.gains;

	frameContext.gains = gains;
	frameContext.colourTemperatureK = context.activeState.awb.temperatureK;
	frameContext.awbMode = context.activeState.awb.mode;
	frameContext.awbAutoEnabled = context.activeState.awb.autoEnabled;
	frameContext.awbLocked = context.activeState.awb.locked;
	params->gains = gains;
}

void Awb::process(IPAContext &context,
		  [[maybe_unused]] const uint32_t frame,
		  IPAFrameContext &frameContext,
		  const SwIspStats *stats,
		  ControlList &metadata)
{
	const SwIspStats::Histogram &histogram = stats->yHistogram;
	const uint8_t blackLevel = context.activeState.blc.level;

	metadata.set(controls::ColourGains, { frameContext.gains.r(),
					      frameContext.gains.b() });
	metadata.set(controls::ColourTemperature, frameContext.colourTemperatureK);
	metadata.set(controls::AwbEnable, frameContext.awbAutoEnabled);
	metadata.set(controls::AwbMode, frameContext.awbMode);
	if (frameContext.awbAutoEnabled)
		metadata.set(controls::AwbLocked, frameContext.awbLocked);

	if (!frameContext.awbAutoEnabled || frameContext.awbMode != controls::AwbAuto)
		return;

	if (!stats->valid)
		return;

	/*
	 * Black level must be subtracted to get the correct AWB ratios, they
	 * would be off if they were computed from the whole brightness range
	 * rather than from the sensor range.
	 */
	const uint64_t nPixels = std::accumulate(
		histogram.begin(), histogram.end(), uint64_t(0));
	const uint64_t offset = blackLevel * nPixels;
	const uint64_t minValid = 1;
	/*
	 * Make sure the sums are at least minValid, while preventing unsigned
	 * integer underflow.
	 */
	const RGB<uint64_t> sum = stats->sum_.max(offset + minValid) - offset;

	/*
	 * Calculate red and blue gains for AWB.
	 * Clamp max gain at 4.0, this also avoids 0 division.
	 */
	const RGB<float> target{ {
		sum.r() <= sum.g() / 4 ? 4.0f : static_cast<float>(sum.g()) / sum.r(),
		1.0,
		sum.b() <= sum.g() / 4 ? 4.0f : static_cast<float>(sum.g()) / sum.b(),
} };
	auto &gains = context.activeState.awb.gains;
	const float maxDelta = std::max(std::abs(target.r() - gains.r()),
					std::abs(target.b() - gains.b()));
	gains.r() += smoothing_ * (target.r() - gains.r());
	gains.b() += smoothing_ * (target.b() - gains.b());

	if (maxDelta < 0.01f)
		context.activeState.awb.stableFrames++;
	else
		context.activeState.awb.stableFrames = 0;
	context.activeState.awb.locked = context.activeState.awb.stableFrames >= 3;

	RGB<double> rgbGains{ { 1 / gains.r(), 1 / gains.g(), 1 / gains.b() } };
	context.activeState.awb.temperatureK = estimateCCT(rgbGains);

	LOG(IPASoftAwb, Debug)
		<< "target R/B: " << target << "; filtered: " << gains
		<< "; temperature: "
		<< context.activeState.awb.temperatureK;
}

REGISTER_IPA_ALGORITHM(Awb, "Awb")

} /* namespace ipa::soft::algorithms */

} /* namespace libcamera */
