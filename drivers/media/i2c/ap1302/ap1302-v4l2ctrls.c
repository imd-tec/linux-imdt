// SPDX-License-Identifier: GPL-2.0-only
/*
 * Driver for the AP1302 external camera ISP from ON Semiconductor
 *
 * Copyright (C) 2021, Witekio, Inc.
 * Copyright (C) 2021, Xilinx, Inc.
 * Copyright (C) 2021, Laurent Pinchart <laurent.pinchart@ideasonboard.com>
 * Copyright (C) 2021, IMD Technologies Ltd
 *
 */

#include "ap1302.h"

static u16 ap1302_wb_values[] = {
	AP1302_AWB_CTRL_MODE_OFF,	/* V4L2_WHITE_BALANCE_MANUAL */
	AP1302_AWB_CTRL_MODE_AUTO,	/* V4L2_WHITE_BALANCE_AUTO */
	AP1302_AWB_CTRL_MODE_A,		/* V4L2_WHITE_BALANCE_INCANDESCENT */
	AP1302_AWB_CTRL_MODE_D50,	/* V4L2_WHITE_BALANCE_FLUORESCENT */
	AP1302_AWB_CTRL_MODE_D65,	/* V4L2_WHITE_BALANCE_FLUORESCENT_H */
	AP1302_AWB_CTRL_MODE_HORIZON,	/* V4L2_WHITE_BALANCE_HORIZON */
	AP1302_AWB_CTRL_MODE_D65,	/* V4L2_WHITE_BALANCE_DAYLIGHT */
	AP1302_AWB_CTRL_MODE_AUTO,	/* V4L2_WHITE_BALANCE_FLASH */
	AP1302_AWB_CTRL_MODE_D75,	/* V4L2_WHITE_BALANCE_CLOUDY */
	AP1302_AWB_CTRL_MODE_D75,	/* V4L2_WHITE_BALANCE_SHADE */
};

static int ap1302_set_wb_mode(struct ap1302_device *ap1302, s32 mode)
{
	u32 val;
	int ret;

	ret = ap1302_read(ap1302, AP1302_AWB_CTRL, &val);
	if (ret)
		return ret;

	val &= ~AP1302_AWB_CTRL_MODE_MASK;
	val |= ap1302_wb_values[mode];

	if (mode == V4L2_WHITE_BALANCE_FLASH)
		val |= AP1302_AWB_CTRL_FLASH;
	else
		val &= ~AP1302_AWB_CTRL_FLASH;

	return ap1302_write(ap1302, AP1302_AWB_CTRL, val, NULL);
}

static int ap1302_set_exposure(struct ap1302_device *ap1302, s32 mode)
{
	u32 val;
	int ret;

	ret = ap1302_read(ap1302, AP1302_AE_CTRL, &val);
	if (ret)
		return ret;

	val &= ~AP1302_AE_CTRL_MODE_MASK;
	val |= mode;

	return ap1302_write(ap1302, AP1302_AE_CTRL, val, NULL);
}

static int ap1302_set_exp_met(struct ap1302_device *ap1302, s32 val)
{
	return ap1302_write(ap1302, AP1302_AE_MET, val, NULL);
}

static int ap1302_set_gain(struct ap1302_device *ap1302, s32 val)
{
	return ap1302_write(ap1302, AP1302_AE_MANUAL_GAIN, val, NULL);
}

static int ap1302_set_contrast(struct ap1302_device *ap1302, s32 val)
{
	return ap1302_write(ap1302, AP1302_CONTRAST, val, NULL);
}

static int ap1302_set_brightness(struct ap1302_device *ap1302, s32 val)
{
	return ap1302_write(ap1302, AP1302_BRIGHTNESS, val, NULL);
}

static int ap1302_set_saturation(struct ap1302_device *ap1302, s32 val)
{
	return ap1302_write(ap1302, AP1302_SATURATION, val, NULL);
}

static int ap1302_set_gamma(struct ap1302_device *ap1302, s32 val)
{
	return ap1302_write(ap1302, AP1302_GAMMA, val, NULL);
}

static int ap1302_set_zoom(struct ap1302_device *ap1302, s32 val)
{
	return ap1302_write(ap1302, AP1302_DZ_TGT_FCT, val, NULL);
}

static u16 ap1302_sfx_values[] = {
	AP1302_SFX_MODE_SFX_NORMAL,	/* V4L2_COLORFX_NONE */
	AP1302_SFX_MODE_SFX_BW,		/* V4L2_COLORFX_BW */
	AP1302_SFX_MODE_SFX_SEPIA1,	/* V4L2_COLORFX_SEPIA */
	AP1302_SFX_MODE_SFX_NEGATIVE,	/* V4L2_COLORFX_NEGATIVE */
	AP1302_SFX_MODE_SFX_EMBOSS,	/* V4L2_COLORFX_EMBOSS */
	AP1302_SFX_MODE_SFX_SKETCH,	/* V4L2_COLORFX_SKETCH */
	AP1302_SFX_MODE_SFX_BLUISH,	/* V4L2_COLORFX_SKY_BLUE */
	AP1302_SFX_MODE_SFX_GREENISH,	/* V4L2_COLORFX_GRASS_GREEN */
	AP1302_SFX_MODE_SFX_REDISH,	/* V4L2_COLORFX_SKIN_WHITEN */
	AP1302_SFX_MODE_SFX_NORMAL,	/* V4L2_COLORFX_VIVID */
	AP1302_SFX_MODE_SFX_NORMAL,	/* V4L2_COLORFX_AQUA */
	AP1302_SFX_MODE_SFX_NORMAL,	/* V4L2_COLORFX_ART_FREEZE */
	AP1302_SFX_MODE_SFX_NORMAL,	/* V4L2_COLORFX_SILHOUETTE */
	AP1302_SFX_MODE_SFX_SOLARIZE, /* V4L2_COLORFX_SOLARIZATION */
	AP1302_SFX_MODE_SFX_ANTIQUE, /* V4L2_COLORFX_ANTIQUE */
	AP1302_SFX_MODE_SFX_NORMAL,	/* V4L2_COLORFX_SET_CBCR */
};

static int ap1302_set_special_effect(struct ap1302_device *ap1302, s32 val)
{
	return ap1302_write(ap1302, AP1302_SFX_MODE, ap1302_sfx_values[val],
			    NULL);
}

static u16 ap1302_scene_mode_values[] = {
	AP1302_SCENE_CTRL_MODE_NORMAL,		/* V4L2_SCENE_MODE_NONE */
	AP1302_SCENE_CTRL_MODE_BACKLIGHT,	/* V4L2_SCENE_MODE_BACKLIGHT */
	AP1302_SCENE_CTRL_MODE_BEACH,		/* V4L2_SCENE_MODE_BEACH_SNOW */
	AP1302_SCENE_CTRL_MODE_TWILIGHT,	/* V4L2_SCENE_MODE_CANDLE_LIGHT */
	AP1302_SCENE_CTRL_MODE_NORMAL,		/* V4L2_SCENE_MODE_DAWN_DUSK */
	AP1302_SCENE_CTRL_MODE_NORMAL,		/* V4L2_SCENE_MODE_FALL_COLORS */
	AP1302_SCENE_CTRL_MODE_FIREWORKS,	/* V4L2_SCENE_MODE_FIREWORKS */
	AP1302_SCENE_CTRL_MODE_LANDSCAPE,	/* V4L2_SCENE_MODE_LANDSCAPE */
	AP1302_SCENE_CTRL_MODE_NIGHT,		/* V4L2_SCENE_MODE_NIGHT */
	AP1302_SCENE_CTRL_MODE_PARTY,		/* V4L2_SCENE_MODE_PARTY_INDOOR */
	AP1302_SCENE_CTRL_MODE_PORTRAIT,	/* V4L2_SCENE_MODE_PORTRAIT */
	AP1302_SCENE_CTRL_MODE_SPORT,		/* V4L2_SCENE_MODE_SPORTS */
	AP1302_SCENE_CTRL_MODE_SUNSET,		/* V4L2_SCENE_MODE_SUNSET */
	AP1302_SCENE_CTRL_MODE_DOCUMENT,	/* V4L2_SCENE_MODE_TEXT */
};

static int ap1302_set_scene_mode(struct ap1302_device *ap1302, s32 val)
{
	return ap1302_write(ap1302, AP1302_SCENE_CTRL,
			    ap1302_scene_mode_values[val], NULL);
}

static const u16 ap1302_flicker_values[] = {
	AP1302_FLICK_CTRL_MODE_DISABLED,
	AP1302_FLICK_CTRL_FREQ(50) | AP1302_FLICK_CTRL_MODE_MANUAL,
	AP1302_FLICK_CTRL_FREQ(60) | AP1302_FLICK_CTRL_MODE_MANUAL,
	AP1302_FLICK_CTRL_MODE_AUTO,
};

static int ap1302_set_flicker_freq(struct ap1302_device *ap1302, s32 val)
{
	return ap1302_write(ap1302, AP1302_FLICK_CTRL,
			    ap1302_flicker_values[val], NULL);
}

static int ap1302_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ap1302_device *ap1302 =
		container_of(ctrl->handler, struct ap1302_device, ctrls);

	switch (ctrl->id) {
	case V4L2_CID_AUTO_N_PRESET_WHITE_BALANCE:
		return ap1302_set_wb_mode(ap1302, ctrl->val);

	case V4L2_CID_EXPOSURE:
		return ap1302_set_exposure(ap1302, ctrl->val);

	case V4L2_CID_EXPOSURE_METERING:
		return ap1302_set_exp_met(ap1302, ctrl->val);

	case V4L2_CID_GAIN:
		return ap1302_set_gain(ap1302, ctrl->val);

	case V4L2_CID_GAMMA:
		return ap1302_set_gamma(ap1302, ctrl->val);

	case V4L2_CID_CONTRAST:
		return ap1302_set_contrast(ap1302, ctrl->val);

	case V4L2_CID_BRIGHTNESS:
		return ap1302_set_brightness(ap1302, ctrl->val);

	case V4L2_CID_SATURATION:
		return ap1302_set_saturation(ap1302, ctrl->val);

	case V4L2_CID_ZOOM_ABSOLUTE:
		return ap1302_set_zoom(ap1302, ctrl->val);

	case V4L2_CID_COLORFX:
		return ap1302_set_special_effect(ap1302, ctrl->val);

	case V4L2_CID_SCENE_MODE:
		return ap1302_set_scene_mode(ap1302, ctrl->val);

	case V4L2_CID_POWER_LINE_FREQUENCY:
		return ap1302_set_flicker_freq(ap1302, ctrl->val);

	default:
		pr_info("ap1302: s_ctrl invalid\n");
		return -EINVAL;
	}
}

static const struct v4l2_ctrl_ops ap1302_ctrl_ops = {
	.s_ctrl = ap1302_s_ctrl,
};

static const struct v4l2_ctrl_config ap1302_ctrls[] = {
	{
		.ops = &ap1302_ctrl_ops,
		.id = V4L2_CID_AUTO_N_PRESET_WHITE_BALANCE,
		.min = 0,
		.max = 9,
		.def = 1,
	}, {
		.ops = &ap1302_ctrl_ops,
		.id = V4L2_CID_GAMMA,
		.name = "Gamma",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = -32768,
		.max = 32768,
		.step = 1,
		.def = 0,
	}, {
		.ops = &ap1302_ctrl_ops,
		.id = V4L2_CID_CONTRAST,
		.name = "Contrast",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = -32768,
		.max = 0x5999,
		.step = 1,
		.def = 0,
	}, {
		.ops = &ap1302_ctrl_ops,
		.id = V4L2_CID_BRIGHTNESS,
		.name = "Brightness",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = -4096,
		.max = 4096,
		.step = 1,
		.def = 0,
	}, {
		.ops = &ap1302_ctrl_ops,
		.id = V4L2_CID_SATURATION,
		.name = "Saturation",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0,
		.max = 0x4000,
		.step = 1,
		.def = 0x1000,
	}, {
		.ops = &ap1302_ctrl_ops,
		.id = V4L2_CID_EXPOSURE,
		.name = "Exposure",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0x0,
		.max = 0xC,
		.step = 1,
		.def = 0xC,
	}, {
		.ops = &ap1302_ctrl_ops,
		.id = V4L2_CID_EXPOSURE_METERING,
		.name = "Exposure Metering",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0x0,
		.max = 0x3,
		.step = 1,
		.def = 0x1,
	}, {
		.ops = &ap1302_ctrl_ops,
		.id = V4L2_CID_GAIN,
		.name = "Gain",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0,
		.max = 0xFFFF,
		.step = 1,
		.def = 0x100,
	}, {
		.ops = &ap1302_ctrl_ops,
		.id = V4L2_CID_ZOOM_ABSOLUTE,
		.min = 256,
		.max = 4096,
		.step = 1,
		.def = 256,
	}, {
		.ops = &ap1302_ctrl_ops,
		.id = V4L2_CID_COLORFX,
		.min = 0,
		.max = 15,
		.def = 0,
		.menu_skip_mask = BIT(15) | BIT(12) | BIT(11) | BIT(10) | BIT(9),
	}, {
		.ops = &ap1302_ctrl_ops,
		.id = V4L2_CID_SCENE_MODE,
		.min = 0,
		.max = 13,
		.def = 0,
		.menu_skip_mask = BIT(5) | BIT(4),
	}, {
		.ops = &ap1302_ctrl_ops,
		.id = V4L2_CID_POWER_LINE_FREQUENCY,
		.min = 0,
		.max = 3,
		.def = 3,
	},
};

int ap1302_ctrls_init(struct ap1302_device *ap1302)
{
	unsigned int i;
	int ret;

	ret = v4l2_ctrl_handler_init(&ap1302->ctrls, ARRAY_SIZE(ap1302_ctrls));
	if (ret)
		return ret;

	for (i = 0; i < ARRAY_SIZE(ap1302_ctrls); i++)
		v4l2_ctrl_new_custom(&ap1302->ctrls, &ap1302_ctrls[i], NULL);

	if (ap1302->ctrls.error) {
		ret = ap1302->ctrls.error;
		v4l2_ctrl_handler_free(&ap1302->ctrls);
		return ret;
	}

	/* Use same lock for controls as for everything else. */
	ap1302->ctrls.lock = &ap1302->lock;
	ap1302->sd.ctrl_handler = &ap1302->ctrls;

	return 0;
}

void ap1302_ctrls_cleanup(struct ap1302_device *ap1302)
{
	v4l2_ctrl_handler_free(&ap1302->ctrls);
}
