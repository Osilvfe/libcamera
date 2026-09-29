/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Edge-aware denoising and sharpening for the software ISP output.
 */

#ifdef GL_ES
precision highp float;
#endif

varying vec2 textureOut;

uniform sampler2D source;
uniform vec2 texel_step;
uniform float noise_reduction;
uniform float sharpness;

void main(void)
{
	vec3 center = texture2D(source, textureOut).rgb;
	vec3 north = texture2D(source, textureOut + vec2(0.0, -texel_step.y)).rgb;
	vec3 south = texture2D(source, textureOut + vec2(0.0, texel_step.y)).rgb;
	vec3 west = texture2D(source, textureOut + vec2(-texel_step.x, 0.0)).rgb;
	vec3 east = texture2D(source, textureOut + vec2(texel_step.x, 0.0)).rgb;
	vec3 blur = (center * 4.0 + north + south + west + east) / 8.0;

	float difference = max(max(abs(center.r - blur.r),
				   abs(center.g - blur.g)),
			       abs(center.b - blur.b));
	float flat_weight = 1.0 - smoothstep(0.015, 0.10, difference);
	vec3 denoised = mix(center, blur,
			    clamp(noise_reduction * flat_weight, 0.0, 1.0));

	vec3 detail = denoised - blur;
	vec3 result = denoised + detail * (0.35 * sharpness);
	gl_FragColor = vec4(clamp(result, 0.0, 1.0), 1.0);
}
