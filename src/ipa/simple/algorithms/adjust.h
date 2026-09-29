/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024-2026, Red Hat Inc.
 *
 * Color correction matrix
 */

#pragma once

#include <optional>

#include "libcamera/internal/matrix.h"

#include <libipa/interpolator.h>

#include "algorithm.h"

namespace libcamera {

namespace ipa::soft::algorithms {

constexpr float kDefaultGamma = 2.2f;

class Adjust : public Algorithm
{
public:
	Adjust();
	~Adjust() = default;

	int init(IPAContext &context, const ValueNode &tuningData) override;
	int configure(IPAContext &context,
		      const IPAConfigInfo &configInfo) override;
	void queueRequest(typename Module::Context &context,
			  const uint32_t frame,
			  typename Module::FrameContext &frameContext,
			  const ControlList &controls) override;
	void prepare(IPAContext &context,
		     const uint32_t frame,
		     IPAFrameContext &frameContext,
		     DebayerParams *params) override;
	void process(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     const SwIspStats *stats,
		     ControlList &metadata) override;

private:
	void applySaturation(Matrix<float, 3, 3> &ccm, float saturation);

	float defaultGamma_;
	std::optional<float> defaultContrast_;
	std::optional<float> defaultSaturation_;
	bool autoContrastEnabled_ = false;
	bool manualContrast_ = false;
	float autoContrastMin_ = 0.95f;
	float autoContrastMax_ = 1.18f;
	float autoContrastTargetRange_ = 0.45f;
	float autoContrastRangeGain_ = 0.4f;
	float autoContrastLowLightStartMs_ = 20.0f;
	float autoContrastLowLightEndMs_ = 120.0f;
	float autoContrastSmoothing_ = 0.12f;
	float currentAutoContrast_ = 1.0f;
};

} /* namespace ipa::soft::algorithms */

} /* namespace libcamera */
