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
#include <string>

#include <libcamera/base/file.h>
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

	const ValueNode &calibration = tuningData["calibration"];
	if (calibration) {
		if (!calibration.isDictionary()) {
			LOG(IPASoftLensShading, Error)
				<< "'calibration' must be a dictionary";
			return -EINVAL;
		}

		auto nvmem = calibration["nvmem"].get<std::string>();
		auto flagOffset = calibration["flag-offset"].get<uint32_t>();
		auto dataOffset = calibration["data-offset"].get<uint32_t>();
		auto checksumOffset = calibration["checksum-offset"].get<uint32_t>();
		auto referenceExposureMs =
			calibration["reference-exposure-ms"].get<double>();
		auto referenceCt = calibration["reference-ct"].get<uint32_t>();
		auto maximum = calibration["maximum"].get<double>();
		if (!nvmem || !flagOffset || !dataOffset || !checksumOffset ||
		    !referenceExposureMs || !referenceCt || !maximum ||
		    nvmem->empty() || nvmem->find('/') != std::string::npos ||
		    !std::isfinite(*referenceExposureMs) || *referenceExposureMs <= 0.0 ||
		    !std::isfinite(*maximum) || *maximum <= 0.0 || *maximum > 65535.0 ||
		    *flagOffset >= *dataOffset || *checksumOffset > 1024 * 1024) {
			LOG(IPASoftLensShading, Error)
				<< "Incomplete or invalid lens shading calibration configuration";
			return -EINVAL;
		}

		CalibrationConfig config{
			*nvmem,
			*flagOffset,
			*dataOffset,
			*checksumOffset,
			*referenceExposureMs,
			*referenceCt,
			static_cast<float>(*maximum),
			calibration["flip-x"].get<bool>(false),
		};
		if (!applyCalibration(config))
			LOG(IPASoftLensShading, Warning)
				<< "Using generic lens shading tables for " << config.nvmem;
	}

	context.ctrlMap[&controls::LensShadingCorrectionEnable] =
		ControlInfo(false, true, defaultEnabled_);

	return 0;
}

bool LensShading::applyCalibration(const CalibrationConfig &config)
{
	constexpr unsigned int kBytesPerValue = 2;
	if (config.checksumOffset <= config.dataOffset ||
	    config.flagOffset >= config.checksumOffset) {
		LOG(IPASoftLensShading, Warning)
			<< "Invalid calibration offsets for " << config.nvmem;
		return false;
	}
	const unsigned int payloadSize = config.checksumOffset - config.dataOffset;
	if (payloadSize != Table{}.size() * kBytesPerValue) {
		LOG(IPASoftLensShading, Warning)
			<< "Invalid calibration payload size for " << config.nvmem;
		return false;
	}

	const std::string path = "/sys/bus/nvmem/devices/" + config.nvmem + "/nvmem";
	File file(path);
	if (!file.open(File::OpenModeFlag::ReadOnly)) {
		LOG(IPASoftLensShading, Warning)
			<< "Unable to open " << path << ": " << file.error();
		return false;
	}

	std::vector<uint8_t> data(config.checksumOffset + 1);
	ssize_t bytes = file.read({ data.data(), data.size() });
	if (bytes != static_cast<ssize_t>(data.size())) {
		LOG(IPASoftLensShading, Warning)
			<< "Short calibration read from " << path << ": " << bytes
			<< " of " << data.size() << " bytes";
		return false;
	}

	unsigned int sum = data[config.flagOffset];
	for (unsigned int offset = config.dataOffset;
	     offset < config.checksumOffset; ++offset)
		sum += data[offset];
	const uint8_t expectedChecksum = sum % 255 + 1;
	if (data[config.flagOffset] != 1 ||
	    data[config.checksumOffset] != expectedChecksum) {
		LOG(IPASoftLensShading, Warning)
			<< "Invalid calibration checksum for " << config.nvmem
			<< ": stored " << static_cast<unsigned int>(data[config.checksumOffset])
			<< ", expected " << static_cast<unsigned int>(expectedChecksum);
		return false;
	}

	auto referenceSet = std::find_if(sets_.begin(), sets_.end(),
					[&config](const ExposureSet &set) {
						return std::abs(set.exposureMs -
								config.referenceExposureMs) < 0.001;
					});
	if (referenceSet == sets_.end()) {
		LOG(IPASoftLensShading, Warning)
			<< "Calibration reference exposure "
			<< config.referenceExposureMs << " ms is unavailable";
		return false;
	}
	auto referenceTable = referenceSet->tables.find(config.referenceCt);
	if (referenceTable == referenceSet->tables.end()) {
		LOG(IPASoftLensShading, Warning)
			<< "Calibration reference colour temperature "
			<< config.referenceCt << " K is unavailable";
		return false;
	}

	Table factors;
	for (unsigned int y = 0; y < DebayerParams::kLensShadingHeight; ++y) {
		for (unsigned int x = 0; x < DebayerParams::kLensShadingWidth; ++x) {
			const unsigned int sourceX = config.flipX
				? DebayerParams::kLensShadingWidth - 1 - x : x;
			for (unsigned int channel = 0;
			     channel < DebayerParams::kLensShadingChannels; ++channel) {
				const unsigned int outputIndex =
					(y * DebayerParams::kLensShadingWidth + x) *
					DebayerParams::kLensShadingChannels + channel;
				const unsigned int sourceIndex =
					(y * DebayerParams::kLensShadingWidth + sourceX) *
					DebayerParams::kLensShadingChannels + channel;
				const unsigned int offset = config.dataOffset +
					sourceIndex * kBytesPerValue;
				const unsigned int raw = data[offset] | data[offset + 1] << 8;
				const float reference = referenceTable->second[outputIndex];
				if (!raw || raw > config.maximum || !std::isfinite(reference) ||
				    reference <= 0.0f) {
					LOG(IPASoftLensShading, Warning)
						<< "Unsafe calibration value at mesh index "
						<< outputIndex << " for " << config.nvmem;
					return false;
				}
				factors[outputIndex] = config.maximum / raw / reference;
			}
		}
	}

	for (const ExposureSet &set : sets_) {
		for (const auto &[ct, table] : set.tables) {
			for (unsigned int i = 0; i < table.size(); ++i) {
				const float corrected = table[i] * factors[i];
				if (!std::isfinite(corrected) || corrected <= 0.0f ||
				    corrected > 8.0f) {
					LOG(IPASoftLensShading, Warning)
						<< "Calibrated gain is outside (0, 8] at "
						<< set.exposureMs << " ms, " << ct
						<< " K, mesh index " << i;
					return false;
				}
			}
		}
	}

	for (ExposureSet &set : sets_)
		for (auto &entry : set.tables) {
			Table &table = entry.second;
			for (unsigned int i = 0; i < table.size(); ++i)
				table[i] *= factors[i];
		}

	const auto [minimum, maximum] =
		std::minmax_element(factors.begin(), factors.end());
	LOG(IPASoftLensShading, Info)
		<< "Applied " << config.nvmem << " calibration; factor range "
		<< *minimum << ".." << *maximum;
	return true;
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
