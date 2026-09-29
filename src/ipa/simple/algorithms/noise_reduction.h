/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2026 Osilvfe
 */

#pragma once

#include "algorithm.h"

namespace libcamera {

namespace ipa::soft::algorithms {

class NoiseReduction : public Algorithm
{
public:
	int init(IPAContext &context, const ValueNode &tuningData) override;
	int configure(IPAContext &context,
		      const IPAConfigInfo &configInfo) override;
	void queueRequest(IPAContext &context, const uint32_t frame,
			  IPAFrameContext &frameContext,
			  const ControlList &controls) override;
	void prepare(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     DebayerParams *params) override;
	void process(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     const SwIspStats *stats, ControlList &metadata) override;

private:
	float defaultSharpness_ = 1.0f;
	float lowLightSharpness_ = 0.45f;
	float defaultDenoise_ = 0.08f;
	float lowLightDenoise_ = 0.55f;
	float gainStart_ = 2.0f;
	float gainEnd_ = 10.0f;
	float exposureStartMs_ = 15.0f;
	float exposureEndMs_ = 80.0f;
	float smoothing_ = 0.15f;
};

} /* namespace ipa::soft::algorithms */

} /* namespace libcamera */
