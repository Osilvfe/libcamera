/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024-2025 Red Hat Inc.
 *
 * Auto white balance
 */

#pragma once

#include "libcamera/internal/vector.h"

#include <string>
#include <vector>

#include <libipa/interpolator.h>

#include "algorithm.h"

namespace libcamera {

namespace ipa::soft::algorithms {

class Awb : public Algorithm
{
public:
	Awb() = default;
	~Awb() = default;

	int init(IPAContext &context, const ValueNode &tuningData) override;
	int configure(IPAContext &context, const IPAConfigInfo &configInfo) override;
	void queueRequest(IPAContext &context,
			  const uint32_t frame,
			  IPAFrameContext &frameContext,
			  const ControlList &controls) override;
	void prepare(IPAContext &context,
		     const uint32_t frame,
		     IPAFrameContext &frameContext,
		     DebayerParams *params) override;
	void process(IPAContext &context,
		     const uint32_t frame,
		     IPAFrameContext &frameContext,
		     const SwIspStats *stats,
		     ControlList &metadata) override;

private:
	struct CalibrationRecord {
		unsigned int ct;
		unsigned int flagOffset;
		unsigned int dataOffset;
		unsigned int checksumOffset;
	};

	bool loadCalibration(const std::string &nvmem,
			     const std::vector<CalibrationRecord> &records);
	Vector<double, 2> calibrationGains(unsigned int temperatureK);
	unsigned int estimateTemperature(const RGB<double> &rgb);
	std::optional<RGB<float>> gainsFromTemperature(unsigned int temperatureK);
	unsigned int modeTemperature(int32_t mode) const;

	Interpolator<Vector<double, 2>> colourGains_;
	Interpolator<Vector<double, 2>> calibrationGains_;
	bool calibrated_ = false;
	float smoothing_ = 0.18f;
};

} /* namespace ipa::soft::algorithms */

} /* namespace libcamera */
