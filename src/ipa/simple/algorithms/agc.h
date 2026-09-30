/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Red Hat Inc.
 *
 * Exposure and gain
 */

#pragma once

#include <optional>

#include <libcamera/base/utils.h>

#include "algorithm.h"

namespace libcamera {

namespace ipa::soft::algorithms {

class Agc : public Algorithm
{
public:
	Agc();
	~Agc() = default;

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
		     const SwIspStats *stats,
		     ControlList &metadata) override;

private:
	void updateExposure(IPAContext &context, IPAFrameContext &frameContext,
			    double exposureMSV);
	void updateVblank(const IPAContext &context,
			  IPAFrameContext &frameContext) const;
	void vblankRange(const IPAContext &context,
			 const IPAFrameContext &frameContext,
			 int32_t &vblankLo, int32_t &vblankHi) const;
	int32_t exposureMaxForVblank(const IPAContext &context,
				     int32_t vblank) const;

	double maxDigitalGain_ = 1.0;
	std::optional<utils::Duration> defaultMaxFrameDuration_;
};

} /* namespace ipa::soft::algorithms */

} /* namespace libcamera */
