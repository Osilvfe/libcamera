/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2026 Osilvfe
 *
 * Software ISP lens shading correction
 */

#include "lens_shading.h"

#include <algorithm>
#include <cmath>
#include <errno.h>

#include <libcamera/base/log.h>
#include <libcamera/base/utils.h>

#include <libcamera/control_ids.h>

namespace libcamera {

namespace ipa::soft::algorithms {

LOG_DEFINE_CATEGORY(IPASoftLensShading)

int LensShading::init(IPAContext &context, const ValueNode &tuningData)
{
	defaultEnabled_ = tuningData["enabled"].get<bool>(true);
	smoothing_ = tuningData["smoothing"].get<double>(0.15);
	if (smoothing_ <= 0.0 || smoothing_ > 1.0) {
		LOG(IPASoftLensShading, Error)
			<< "'smoothing' must be in the (0, 1] range";
		return -EINVAL;
	}

	const ValueNode &sets = tuningData["sets"];
	if (!sets.isList()) {
		LOG(IPASoftLensShading, Error) << "'sets' must be a list";
		return -EINVAL;
	}

	for (const ValueNode &node : sets.asList()) {
		auto exposureMs = node["exposure-ms"].get<double>();
		const ValueNode &tables = node["tables"];
		if (!exposureMs || *exposureMs <= 0.0 || !tables.isList()) {
			LOG(IPASoftLensShading, Error)
				<< "Each set needs a positive 'exposure-ms' and a 'tables' list";
			return -EINVAL;
		}

		ExposureSet set{ *exposureMs, {} };
		for (const ValueNode &entry : tables.asList()) {
			auto ct = entry["ct"].get<uint32_t>();
			auto gains = entry["gains"].get<std::vector<float>>();
			if (!ct || !gains || gains->size() != Table{}.size()) {
				LOG(IPASoftLensShading, Error)
					<< "Each table needs 'ct' and " << Table{}.size()
					<< " gain values";
				return -EINVAL;
			}

			Table table;
			std::copy(gains->begin(), gains->end(), table.begin());
			if (std::any_of(table.begin(), table.end(), [](float gain) {
				    return !std::isfinite(gain) || gain <= 0.0f || gain > 8.0f;
			    })) {
				LOG(IPASoftLensShading, Error)
					<< "Lens shading gains must be finite and in the (0, 8] range";
				return -EINVAL;
			}

			if (!set.tables.emplace(*ct, std::move(table)).second) {
				LOG(IPASoftLensShading, Error)
					<< "Duplicate colour temperature " << *ct;
				return -EINVAL;
			}
		}

		if (set.tables.empty()) {
			LOG(IPASoftLensShading, Error) << "Lens shading set has no tables";
			return -EINVAL;
		}

		sets_.push_back(std::move(set));
	}

	std::sort(sets_.begin(), sets_.end(),
		  [](const ExposureSet &a, const ExposureSet &b) {
			  return a.exposureMs < b.exposureMs;
		  });
	if (sets_.empty()) {
		LOG(IPASoftLensShading, Error) << "No lens shading sets configured";
		return -EINVAL;
	}
	for (unsigned int i = 1; i < sets_.size(); ++i) {
		if (sets_[i - 1].exposureMs == sets_[i].exposureMs) {
			LOG(IPASoftLensShading, Error)
				<< "Duplicate exposure point " << sets_[i].exposureMs;
			return -EINVAL;
		}
	}

	context.ctrlMap[&controls::LensShadingCorrectionEnable] =
		ControlInfo(false, true, defaultEnabled_);

	return 0;
}

int LensShading::configure(IPAContext &context,
			   [[maybe_unused]] const IPAConfigInfo &configInfo)
{
	context.activeState.lensShading.enabled = defaultEnabled_;
	filteredExposureMs_.reset();
	return 0;
}

void LensShading::queueRequest(IPAContext &context,
			       [[maybe_unused]] const uint32_t frame,
			       [[maybe_unused]] IPAFrameContext &frameContext,
			       const ControlList &controls)
{
	const auto &enabled = controls.get(controls::LensShadingCorrectionEnable);
	if (enabled)
		context.activeState.lensShading.enabled = *enabled;
}

void LensShading::interpolateCct(const ExposureSet &set, unsigned int ct,
				 Table *table) const
{
	auto upper = set.tables.lower_bound(ct);
	if (upper == set.tables.begin()) {
		*table = upper->second;
		return;
	}
	if (upper == set.tables.end()) {
		*table = std::prev(upper)->second;
		return;
	}
	if (upper->first == ct) {
		*table = upper->second;
		return;
	}

	auto lower = std::prev(upper);
	const float lambda = static_cast<float>(ct - lower->first) /
			     (upper->first - lower->first);
	for (unsigned int i = 0; i < table->size(); ++i)
		(*table)[i] = lower->second[i] * (1.0f - lambda) +
			      upper->second[i] * lambda;
}

void LensShading::prepare(IPAContext &context,
			  [[maybe_unused]] const uint32_t frame,
			  IPAFrameContext &frameContext,
			  DebayerParams *params)
{
	const bool enabled = context.activeState.lensShading.enabled;
	frameContext.lensShadingEnabled = enabled;
	params->lensShadingEnabled = enabled;
	if (!enabled)
		return;

	if (!context.activeState.agc.valid) {
		params->lensShading.fill(1.0f);
		LOG(IPASoftLensShading, Debug)
			<< "Waiting for a valid exposure, using unity gains";
		return;
	}

	double exposureMs = context.configuration.agc.lineDuration.get<std::milli>() *
			    context.activeState.agc.exposure *
			    context.activeState.agc.again;

	if (!filteredExposureMs_)
		filteredExposureMs_ = exposureMs;
	else
		*filteredExposureMs_ += smoothing_ * (exposureMs - *filteredExposureMs_);

	const unsigned int ct = context.activeState.awb.temperatureK;
	LOG(IPASoftLensShading, Debug)
		<< "exposure " << exposureMs << " ms, filtered "
		<< *filteredExposureMs_ << " ms, colour temperature " << ct << " K";
	auto upper = std::lower_bound(sets_.begin(), sets_.end(), *filteredExposureMs_,
				      [](const ExposureSet &set, double value) {
					      return set.exposureMs < value;
				      });
	if (upper == sets_.begin()) {
		interpolateCct(*upper, ct, &params->lensShading);
		return;
	}
	if (upper == sets_.end()) {
		interpolateCct(sets_.back(), ct, &params->lensShading);
		return;
	}

	const ExposureSet &lower = *std::prev(upper);
	Table lowerTable;
	Table upperTable;
	interpolateCct(lower, ct, &lowerTable);
	interpolateCct(*upper, ct, &upperTable);
	const float lambda = (*filteredExposureMs_ - lower.exposureMs) /
			     (upper->exposureMs - lower.exposureMs);
	for (unsigned int i = 0; i < params->lensShading.size(); ++i)
		params->lensShading[i] = lowerTable[i] * (1.0f - lambda) +
					 upperTable[i] * lambda;
}

void LensShading::process([[maybe_unused]] IPAContext &context,
			  [[maybe_unused]] const uint32_t frame,
			  IPAFrameContext &frameContext,
			  [[maybe_unused]] const SwIspStats *stats,
			  ControlList &metadata)
{
	metadata.set(controls::LensShadingCorrectionEnable,
		     frameContext.lensShadingEnabled);
}

REGISTER_IPA_ALGORITHM(LensShading, "LensShading")

} /* namespace ipa::soft::algorithms */

} /* namespace libcamera */
