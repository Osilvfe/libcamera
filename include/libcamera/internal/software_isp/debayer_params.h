/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023-2026 Red Hat Inc.
 *
 * Authors:
 * Hans de Goede <hdegoede@redhat.com>
 *
 * DebayerParams header
 */

#pragma once

#include <array>
#include <stdint.h>

#include "libcamera/internal/matrix.h"
#include "libcamera/internal/vector.h"

namespace libcamera {

struct DebayerParams {
	static constexpr unsigned int kLensShadingWidth = 17;
	static constexpr unsigned int kLensShadingHeight = 13;
	static constexpr unsigned int kLensShadingChannels = 4;
	static constexpr unsigned int kLensShadingSize =
		kLensShadingWidth * kLensShadingHeight * kLensShadingChannels;

	Matrix<float, 3, 3> combinedMatrix = { { 1.0, 0.0, 0.0,
						 0.0, 1.0, 0.0,
						 0.0, 0.0, 1.0 } };
	RGB<float> blackLevel = RGB<float>({ 0.0, 0.0, 0.0 });
	float gamma = 1.0;
	float contrastExp = 1.0;
	RGB<float> gains = RGB<float>({ 1.0, 1.0, 1.0 });
	std::array<float, kLensShadingSize> lensShading;
	bool lensShadingEnabled = false;
};

} /* namespace libcamera */
