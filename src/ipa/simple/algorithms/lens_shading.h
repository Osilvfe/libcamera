/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2026 Osilvfe
 *
 * Software ISP lens shading correction
 */

#pragma once

#include <array>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "algorithm.h"

namespace libcamera {

namespace ipa::soft::algorithms {

class LensShading : public Algorithm
{
public:
	LensShading() = default;
	~LensShading() = default;

	int init(IPAContext &context, const ValueNode &tuningData) override;
	int configure(IPAContext &context, const IPAConfigInfo &configInfo) override;
	void queueRequest(IPAContext &context, const uint32_t frame,
			  IPAFrameContext &frameContext,
			  const ControlList &controls) override;
	void prepare(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     DebayerParams *params) override;
	void process(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     const SwIspStats *stats,
		     ControlList &metadata) override;

private:
	using Table = std::array<float, DebayerParams::kLensShadingSize>;

	struct ExposureSet {
		double exposureMs;
		std::map<unsigned int, Table> tables;
	};

	struct CalibrationConfig {
		std::string nvmem;
		unsigned int flagOffset;
		unsigned int dataOffset;
		unsigned int checksumOffset;
		double referenceExposureMs;
		unsigned int referenceCt;
		float maximum;
		bool flipX;
	};

	bool applyCalibration(const CalibrationConfig &config);
	void interpolateCct(const ExposureSet &set, unsigned int ct,
			    Table *table) const;

	std::vector<ExposureSet> sets_;
	std::optional<double> filteredExposureMs_;
	bool defaultEnabled_ = true;
	double smoothing_ = 0.15;
};

} /* namespace ipa::soft::algorithms */

} /* namespace libcamera */
