// SPDX-License-Identifier: GPL-2.0
/*
 * OmniVision OV02B10 / OV02B1B 2 MP raw image sensor driver (1600x1200, 1 lane).
 *
 * Bring-up driver for the Xiaomi Redmi Note 10 Pro (sweet) depth camera
 * (Ofilm OV02B1B module). The register sequence is the vendor (Qualcomm
 * CamX) init table for a 19.2 MHz MCLK. Control registers follow the
 * Rockchip BSP ov02b10.c (page select 0xfd, exposure P1:0x0e/0x0f, gain
 * P1:0x22, vblank P1:0x14/0x15, frame-sync update 0xfe = 0x02).
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/units.h>
#include <media/v4l2-cci.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>

#define OV02B10_MCLK_FREQ		(19200 * HZ_PER_KHZ)
#define OV02B10_LINK_FREQ		332800000ULL
#define OV02B10_DATA_LANES		1
/* RAW10, 1 lane, DDR: 332.8 MHz * 2 / 10 */
#define OV02B10_PIXEL_RATE		66560000

#define OV02B10_REG_PAGE		CCI_REG8(0xfd)
#define OV02B10_REG_UPDATE		CCI_REG8(0xfe)
#define OV02B10_UPDATE_FRAME_SYNC	0x02
/* page 0 */
#define OV02B10_REG_CHIP_ID_H		CCI_REG8(0x02)
#define OV02B10_REG_CHIP_ID_L		CCI_REG8(0x03)
#define OV02B10_CHIP_ID			0x002b
#define OV02B10_REG_TEST_PATTERN	CCI_REG8(0x63)
/* page 1 */
#define OV02B10_REG_EXP_H		CCI_REG8(0x0e)
#define OV02B10_REG_EXP_L		CCI_REG8(0x0f)
#define OV02B10_REG_VBLANK_H		CCI_REG8(0x14)
#define OV02B10_REG_VBLANK_L		CCI_REG8(0x15)
#define OV02B10_REG_AGAIN		CCI_REG8(0x22)
/* page 3 */
#define OV02B10_REG_STREAM		CCI_REG8(0xc2)

#define OV02B10_EXPOSURE_MIN		4
#define OV02B10_EXPOSURE_MARGIN		7
#define OV02B10_GAIN_MIN		0x10	/* 1.0x */
#define OV02B10_GAIN_MAX		0xf8
#define OV02B10_GAIN_DEFAULT		0x10

#define OV02B10_WIDTH			1600
#define OV02B10_HEIGHT			1200
#define OV02B10_HTS			1792	/* vendor line_length 448 x 4 */
/* frame length = OV02B10_VTS_BASE + P1:0x14/0x15; vendor sets 0x0010 -> 1236 */
#define OV02B10_VTS_BASE		1220
#define OV02B10_VTS_DEF			1236
#define OV02B10_VTS_MAX			(OV02B10_VTS_BASE + 0xffff)
#define OV02B10_EXPOSURE_DEF		538	/* P1:0x0e/0x0f = 0x021a */

static bool mono;
module_param(mono, bool, 0444);
MODULE_PARM_DESC(mono, "Report Y10 (monochrome) instead of SBGGR10");

#define to_ov02b10(_sd)			container_of(_sd, struct ov02b10, sd)

static const char * const ov02b10_supply_names[] = {
	"dovdd",	/* 1.8 V I/O */
	"avdd",		/* 2.8 V analog */
	"dvdd",		/* optional, the sensor has an internal DVDD LDO */
};

static const s64 ov02b10_link_freq_menu[] = {
	OV02B10_LINK_FREQ,
};

static const char * const ov02b10_test_pattern_menu[] = {
	"Disabled",
	"Color bars",
};

struct ov02b10 {
	struct device *dev;
	struct regmap *regmap;
	struct clk *mclk;
	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[ARRAY_SIZE(ov02b10_supply_names)];

	struct v4l2_subdev sd;
	struct media_pad pad;

	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *exposure;
};

/* Vendor (CamX) initSettings, Ofilm OV02B1B module, 1600x1200 RAW10 1 lane */
static const struct cci_reg_sequence ov02b10_init_regs[] = {
	{ CCI_REG8(0xfc), 0x01 },
	{ CCI_REG8(0xfd), 0x00 },
	{ CCI_REG8(0xfd), 0x00 },
	{ CCI_REG8(0x24), 0x03 },
	{ CCI_REG8(0x25), 0x0c },
	{ CCI_REG8(0x29), 0x02 },
	{ CCI_REG8(0x2a), 0x31 },
	{ CCI_REG8(0x2b), 0x05 },
	{ CCI_REG8(0x1e), 0x17 },
	{ CCI_REG8(0x33), 0x07 },
	{ CCI_REG8(0x35), 0x07 },
	{ CCI_REG8(0x4a), 0x0c },
	{ CCI_REG8(0x3a), 0x05 },
	{ CCI_REG8(0x3b), 0x02 },
	{ CCI_REG8(0x3e), 0x00 },
	{ CCI_REG8(0x46), 0x01 },
	{ CCI_REG8(0x6d), 0x03 },
	{ CCI_REG8(0xfd), 0x01 },
	{ CCI_REG8(0x0e), 0x02 },
	{ CCI_REG8(0x0f), 0x1a },
	{ CCI_REG8(0x14), 0x00 },
	{ CCI_REG8(0x15), 0x10 },
	{ CCI_REG8(0x18), 0x00 },
	{ CCI_REG8(0x22), 0xff },
	{ CCI_REG8(0x23), 0x02 },
	{ CCI_REG8(0x17), 0x2c },
	{ CCI_REG8(0x19), 0x20 },
	{ CCI_REG8(0x1b), 0x06 },
	{ CCI_REG8(0x1c), 0x04 },
	{ CCI_REG8(0x20), 0x03 },
	{ CCI_REG8(0x30), 0x01 },
	{ CCI_REG8(0x33), 0x01 },
	{ CCI_REG8(0x31), 0x0a },
	{ CCI_REG8(0x32), 0x09 },
	{ CCI_REG8(0x38), 0x01 },
	{ CCI_REG8(0x39), 0x01 },
	{ CCI_REG8(0x3a), 0x01 },
	{ CCI_REG8(0x3b), 0x01 },
	{ CCI_REG8(0x4f), 0x04 },
	{ CCI_REG8(0x4e), 0x05 },
	{ CCI_REG8(0x50), 0x01 },
	{ CCI_REG8(0x35), 0x0c },
	{ CCI_REG8(0x45), 0x2a },
	{ CCI_REG8(0x46), 0x2a },
	{ CCI_REG8(0x47), 0x2a },
	{ CCI_REG8(0x48), 0x2a },
	{ CCI_REG8(0x4a), 0x2c },
	{ CCI_REG8(0x4b), 0x2c },
	{ CCI_REG8(0x4c), 0x2c },
	{ CCI_REG8(0x4d), 0x2c },
	{ CCI_REG8(0x56), 0x3a },
	{ CCI_REG8(0x57), 0x0a },
	{ CCI_REG8(0x58), 0x24 },
	{ CCI_REG8(0x59), 0x20 },
	{ CCI_REG8(0x5a), 0x0a },
	{ CCI_REG8(0x5b), 0xff },
	{ CCI_REG8(0x37), 0x0a },
	{ CCI_REG8(0x42), 0x0e },
	{ CCI_REG8(0x68), 0x90 },
	{ CCI_REG8(0x69), 0xcd },
	{ CCI_REG8(0x6a), 0x8f },
	{ CCI_REG8(0x7c), 0x0a },
	{ CCI_REG8(0x7d), 0x0a },
	{ CCI_REG8(0x7e), 0x0a },
	{ CCI_REG8(0x7f), 0x08 },
	{ CCI_REG8(0x83), 0x14 },
	{ CCI_REG8(0x84), 0x14 },
	{ CCI_REG8(0x86), 0x14 },
	{ CCI_REG8(0x87), 0x07 },
	{ CCI_REG8(0x88), 0x0f },
	{ CCI_REG8(0x94), 0x02 },
	{ CCI_REG8(0x98), 0xd1 },
	{ CCI_REG8(0xfe), 0x02 },
	{ CCI_REG8(0xfd), 0x03 },
	{ CCI_REG8(0x97), 0x6c },
	{ CCI_REG8(0x98), 0x60 },
	{ CCI_REG8(0x99), 0x60 },
	{ CCI_REG8(0x9a), 0x6c },
	{ CCI_REG8(0xa1), 0x40 },
	{ CCI_REG8(0xaf), 0x04 },
	{ CCI_REG8(0xb1), 0x40 },
	{ CCI_REG8(0xae), 0x0d },
	{ CCI_REG8(0x88), 0x5b },
	{ CCI_REG8(0x89), 0x7c },
	{ CCI_REG8(0xb4), 0x05 },
	{ CCI_REG8(0x8c), 0x40 },
	{ CCI_REG8(0x8e), 0x40 },
	{ CCI_REG8(0x90), 0x40 },
	{ CCI_REG8(0x92), 0x40 },
	{ CCI_REG8(0x9b), 0x46 },
	{ CCI_REG8(0xac), 0x40 },
	{ CCI_REG8(0xfd), 0x00 },
	{ CCI_REG8(0x5a), 0x15 },
	{ CCI_REG8(0x74), 0x01 },
	{ CCI_REG8(0xfd), 0x00 },
	{ CCI_REG8(0x50), 0x40 },
	{ CCI_REG8(0x52), 0xb0 },
	{ CCI_REG8(0xfd), 0x01 },
	{ CCI_REG8(0x03), 0x70 },
	{ CCI_REG8(0x05), 0x10 },
	{ CCI_REG8(0x07), 0x20 },
	{ CCI_REG8(0x09), 0xb0 },
	{ CCI_REG8(0xfd), 0x03 },
	{ CCI_REG8(0xfb), 0x01 },
};

static int ov02b10_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ov02b10 *ov02b10 = container_of(ctrl->handler, struct ov02b10,
					       ctrl_handler);
	s64 exposure_max;
	u32 vblank_reg;
	int ret = 0;

	if (ctrl->id == V4L2_CID_VBLANK) {
		exposure_max = OV02B10_HEIGHT + ctrl->val -
			       OV02B10_EXPOSURE_MARGIN;
		__v4l2_ctrl_modify_range(ov02b10->exposure,
					 ov02b10->exposure->minimum,
					 exposure_max,
					 ov02b10->exposure->step,
					 ov02b10->exposure->default_value);
	}

	if (!pm_runtime_get_if_active(ov02b10->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		cci_write(ov02b10->regmap, OV02B10_REG_PAGE, 0x01, &ret);
		cci_write(ov02b10->regmap, OV02B10_REG_EXP_H,
			  (ctrl->val >> 8) & 0xff, &ret);
		cci_write(ov02b10->regmap, OV02B10_REG_EXP_L, ctrl->val & 0xff,
			  &ret);
		cci_write(ov02b10->regmap, OV02B10_REG_UPDATE,
			  OV02B10_UPDATE_FRAME_SYNC, &ret);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		cci_write(ov02b10->regmap, OV02B10_REG_PAGE, 0x01, &ret);
		cci_write(ov02b10->regmap, OV02B10_REG_AGAIN, ctrl->val, &ret);
		cci_write(ov02b10->regmap, OV02B10_REG_UPDATE,
			  OV02B10_UPDATE_FRAME_SYNC, &ret);
		break;
	case V4L2_CID_VBLANK:
		/* V4L2 vblank = VTS - height, register = VTS - OV02B10_VTS_BASE */
		vblank_reg = OV02B10_HEIGHT + ctrl->val - OV02B10_VTS_BASE;
		cci_write(ov02b10->regmap, OV02B10_REG_PAGE, 0x01, &ret);
		cci_write(ov02b10->regmap, OV02B10_REG_VBLANK_H,
			  (vblank_reg >> 8) & 0xff, &ret);
		cci_write(ov02b10->regmap, OV02B10_REG_VBLANK_L,
			  vblank_reg & 0xff, &ret);
		cci_write(ov02b10->regmap, OV02B10_REG_UPDATE,
			  OV02B10_UPDATE_FRAME_SYNC, &ret);
		break;
	case V4L2_CID_TEST_PATTERN:
		cci_write(ov02b10->regmap, OV02B10_REG_PAGE, 0x00, &ret);
		cci_write(ov02b10->regmap, OV02B10_REG_TEST_PATTERN,
			  ctrl->val ? 0x01 : 0x00, &ret);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	pm_runtime_put(ov02b10->dev);

	return ret;
}

static const struct v4l2_ctrl_ops ov02b10_ctrl_ops = {
	.s_ctrl = ov02b10_set_ctrl,
};

static int ov02b10_init_controls(struct ov02b10 *ov02b10)
{
	struct v4l2_ctrl_handler *hdl = &ov02b10->ctrl_handler;
	struct v4l2_fwnode_device_properties props;
	s64 vblank, hblank;
	int ret;

	v4l2_ctrl_handler_init(hdl, 9);

	ov02b10->link_freq = v4l2_ctrl_new_int_menu(hdl, &ov02b10_ctrl_ops,
						   V4L2_CID_LINK_FREQ, 0, 0,
						   ov02b10_link_freq_menu);
	if (ov02b10->link_freq)
		ov02b10->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	ov02b10->pixel_rate = v4l2_ctrl_new_std(hdl, &ov02b10_ctrl_ops,
					       V4L2_CID_PIXEL_RATE,
					       OV02B10_PIXEL_RATE,
					       OV02B10_PIXEL_RATE, 1,
					       OV02B10_PIXEL_RATE);

	hblank = OV02B10_HTS - OV02B10_WIDTH;
	ov02b10->hblank = v4l2_ctrl_new_std(hdl, &ov02b10_ctrl_ops,
					   V4L2_CID_HBLANK, hblank, hblank, 1,
					   hblank);
	if (ov02b10->hblank)
		ov02b10->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	vblank = OV02B10_VTS_DEF - OV02B10_HEIGHT;
	ov02b10->vblank = v4l2_ctrl_new_std(hdl, &ov02b10_ctrl_ops,
					   V4L2_CID_VBLANK,
					   OV02B10_VTS_BASE - OV02B10_HEIGHT,
					   OV02B10_VTS_MAX - OV02B10_HEIGHT, 1,
					   vblank);

	ov02b10->exposure = v4l2_ctrl_new_std(hdl, &ov02b10_ctrl_ops,
					     V4L2_CID_EXPOSURE,
					     OV02B10_EXPOSURE_MIN,
					     OV02B10_VTS_DEF -
					     OV02B10_EXPOSURE_MARGIN, 1,
					     OV02B10_EXPOSURE_DEF);

	v4l2_ctrl_new_std(hdl, &ov02b10_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			  OV02B10_GAIN_MIN, OV02B10_GAIN_MAX, 1,
			  OV02B10_GAIN_DEFAULT);

	v4l2_ctrl_new_std_menu_items(hdl, &ov02b10_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(ov02b10_test_pattern_menu) - 1,
				     0, 0, ov02b10_test_pattern_menu);

	ret = v4l2_fwnode_device_parse(ov02b10->dev, &props);
	if (ret)
		goto err_free;

	ret = v4l2_ctrl_new_fwnode_properties(hdl, &ov02b10_ctrl_ops, &props);
	if (ret)
		goto err_free;

	if (hdl->error) {
		ret = hdl->error;
		goto err_free;
	}

	ov02b10->sd.ctrl_handler = hdl;

	return 0;

err_free:
	v4l2_ctrl_handler_free(hdl);

	return ret;
}

static int ov02b10_enable_streams(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state, u32 pad,
				 u64 streams_mask)
{
	struct ov02b10 *ov02b10 = to_ov02b10(sd);
	int ret;

	ret = pm_runtime_resume_and_get(ov02b10->dev);
	if (ret)
		return ret;

	ret = cci_multi_reg_write(ov02b10->regmap, ov02b10_init_regs,
				  ARRAY_SIZE(ov02b10_init_regs), NULL);
	if (ret)
		goto err;

	ret = __v4l2_ctrl_handler_setup(ov02b10->sd.ctrl_handler);
	if (ret)
		goto err;

	cci_write(ov02b10->regmap, OV02B10_REG_PAGE, 0x03, &ret);
	cci_write(ov02b10->regmap, OV02B10_REG_STREAM, 0x01, &ret);
	if (ret)
		goto err;

	return 0;

err:
	dev_err(ov02b10->dev, "failed to start streaming: %d\n", ret);
	pm_runtime_put_autosuspend(ov02b10->dev);

	return ret;
}

static int ov02b10_disable_streams(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state, u32 pad,
				  u64 streams_mask)
{
	struct ov02b10 *ov02b10 = to_ov02b10(sd);
	int ret = 0;

	cci_write(ov02b10->regmap, OV02B10_REG_PAGE, 0x03, &ret);
	cci_write(ov02b10->regmap, OV02B10_REG_STREAM, 0x00, &ret);
	if (ret)
		dev_err(ov02b10->dev, "failed to stop streaming: %d\n", ret);

	pm_runtime_put_autosuspend(ov02b10->dev);

	return ret;
}

/*
 * OV02B10 is a Bayer (BGGR, per the Rockchip BSP driver) sensor; the OV02B1B
 * fitted as a depth camera may be the monochrome variant (unverified), use
 * the "mono" module parameter to report Y10 in that case.
 */
static u32 ov02b10_code(void)
{
	return mono ? MEDIA_BUS_FMT_Y10_1X10 : MEDIA_BUS_FMT_SBGGR10_1X10;
}

static void ov02b10_fill_format(struct v4l2_mbus_framefmt *fmt)
{
	fmt->code = ov02b10_code();
	fmt->width = OV02B10_WIDTH;
	fmt->height = OV02B10_HEIGHT;
	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	fmt->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->xfer_func = V4L2_XFER_FUNC_NONE;
}

static int ov02b10_set_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_format *fmt)
{
	ov02b10_fill_format(&fmt->format);
	*v4l2_subdev_state_get_format(state, 0) = fmt->format;

	return 0;
}

static int ov02b10_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index > 0)
		return -EINVAL;

	code->code = ov02b10_code();

	return 0;
}

static int ov02b10_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index > 0 || fse->code != ov02b10_code())
		return -EINVAL;

	fse->min_width = OV02B10_WIDTH;
	fse->max_width = OV02B10_WIDTH;
	fse->min_height = OV02B10_HEIGHT;
	fse->max_height = OV02B10_HEIGHT;

	return 0;
}

static int ov02b10_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *state,
				struct v4l2_subdev_selection *sel)
{
	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
	case V4L2_SEL_TGT_NATIVE_SIZE:
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = OV02B10_WIDTH;
		sel->r.height = OV02B10_HEIGHT;
		return 0;
	default:
		return -EINVAL;
	}
}

static int ov02b10_init_state(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state)
{
	ov02b10_fill_format(v4l2_subdev_state_get_format(state, 0));

	return 0;
}

static const struct v4l2_subdev_video_ops ov02b10_video_ops = {
	.s_stream = v4l2_subdev_s_stream_helper,
};

static const struct v4l2_subdev_pad_ops ov02b10_pad_ops = {
	.set_fmt = ov02b10_set_pad_format,
	.get_fmt = v4l2_subdev_get_fmt,
	.get_selection = ov02b10_get_selection,
	.enum_mbus_code = ov02b10_enum_mbus_code,
	.enum_frame_size = ov02b10_enum_frame_size,
	.enable_streams = ov02b10_enable_streams,
	.disable_streams = ov02b10_disable_streams,
};

static const struct v4l2_subdev_ops ov02b10_subdev_ops = {
	.video = &ov02b10_video_ops,
	.pad = &ov02b10_pad_ops,
};

static const struct v4l2_subdev_internal_ops ov02b10_internal_ops = {
	.init_state = ov02b10_init_state,
};

static const struct media_entity_operations ov02b10_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static int ov02b10_power_on(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct ov02b10 *ov02b10 = to_ov02b10(sd);
	int ret;

	gpiod_set_value_cansleep(ov02b10->reset_gpio, 1);

	ret = regulator_bulk_enable(ARRAY_SIZE(ov02b10->supplies),
				    ov02b10->supplies);
	if (ret)
		return ret;

	/* vendor sequence: 9 ms after AVDD */
	usleep_range(9000, 10000);

	ret = clk_prepare_enable(ov02b10->mclk);
	if (ret) {
		regulator_bulk_disable(ARRAY_SIZE(ov02b10->supplies),
				       ov02b10->supplies);
		return ret;
	}

	usleep_range(1000, 2000);
	gpiod_set_value_cansleep(ov02b10->reset_gpio, 0);
	/* vendor sequence: 14 ms after reset release */
	usleep_range(14000, 15000);

	return 0;
}

static int ov02b10_power_off(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct ov02b10 *ov02b10 = to_ov02b10(sd);

	gpiod_set_value_cansleep(ov02b10->reset_gpio, 1);
	usleep_range(1000, 2000);
	clk_disable_unprepare(ov02b10->mclk);
	regulator_bulk_disable(ARRAY_SIZE(ov02b10->supplies), ov02b10->supplies);

	return 0;
}

static int ov02b10_identify(struct ov02b10 *ov02b10)
{
	u64 hi = 0, lo = 0;
	int ret = 0;

	cci_write(ov02b10->regmap, OV02B10_REG_PAGE, 0x00, &ret);
	cci_read(ov02b10->regmap, OV02B10_REG_CHIP_ID_H, &hi, &ret);
	cci_read(ov02b10->regmap, OV02B10_REG_CHIP_ID_L, &lo, &ret);
	if (ret)
		return dev_err_probe(ov02b10->dev, ret,
				     "failed to read chip id\n");

	if (((hi << 8) | lo) != OV02B10_CHIP_ID)
		return dev_err_probe(ov02b10->dev, -ENODEV,
				     "chip id mismatch: 0x%04llx != 0x%04x\n",
				     (hi << 8) | lo, OV02B10_CHIP_ID);

	dev_info(ov02b10->dev, "OV02B10/OV02B1B detected (chip id 0x%04x)\n",
		 OV02B10_CHIP_ID);

	return 0;
}

static int ov02b10_check_hwcfg(struct ov02b10 *ov02b10)
{
	struct v4l2_fwnode_endpoint bus_cfg = {
		.bus_type = V4L2_MBUS_CSI2_DPHY,
	};
	struct fwnode_handle *ep;
	unsigned long freq_bitmap;
	int ret;

	ep = fwnode_graph_get_next_endpoint(dev_fwnode(ov02b10->dev), NULL);
	if (!ep)
		return dev_err_probe(ov02b10->dev, -ENXIO,
				     "no endpoint (probe-only mode)\n");

	ret = v4l2_fwnode_endpoint_alloc_parse(ep, &bus_cfg);
	fwnode_handle_put(ep);
	if (ret)
		return ret;

	if (bus_cfg.bus.mipi_csi2.num_data_lanes != OV02B10_DATA_LANES) {
		ret = dev_err_probe(ov02b10->dev, -EINVAL,
				    "only 1 data lane is supported\n");
		goto out;
	}

	ret = v4l2_link_freq_to_bitmap(ov02b10->dev, bus_cfg.link_frequencies,
				       bus_cfg.nr_of_link_frequencies,
				       ov02b10_link_freq_menu,
				       ARRAY_SIZE(ov02b10_link_freq_menu),
				       &freq_bitmap);
out:
	v4l2_fwnode_endpoint_free(&bus_cfg);

	return ret;
}

static int ov02b10_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct ov02b10 *ov02b10;
	unsigned int i;
	int ret;

	ov02b10 = devm_kzalloc(dev, sizeof(*ov02b10), GFP_KERNEL);
	if (!ov02b10)
		return -ENOMEM;

	ov02b10->dev = dev;
	v4l2_i2c_subdev_init(&ov02b10->sd, client, &ov02b10_subdev_ops);

	ov02b10->regmap = devm_cci_regmap_init_i2c(client, 8);
	if (IS_ERR(ov02b10->regmap))
		return dev_err_probe(dev, PTR_ERR(ov02b10->regmap),
				     "failed to init CCI\n");

	ov02b10->mclk = devm_v4l2_sensor_clk_get(dev, NULL);
	if (IS_ERR(ov02b10->mclk))
		return dev_err_probe(dev, PTR_ERR(ov02b10->mclk),
				     "failed to get MCLK\n");

	if (clk_get_rate(ov02b10->mclk) != OV02B10_MCLK_FREQ)
		return dev_err_probe(dev, -EINVAL,
				     "MCLK %lu Hz not supported (need 19.2 MHz)\n",
				     clk_get_rate(ov02b10->mclk));

	ov02b10->reset_gpio = devm_gpiod_get_optional(dev, "reset",
						     GPIOD_OUT_HIGH);
	if (IS_ERR(ov02b10->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ov02b10->reset_gpio),
				     "failed to get reset GPIO\n");

	for (i = 0; i < ARRAY_SIZE(ov02b10_supply_names); i++)
		ov02b10->supplies[i].supply = ov02b10_supply_names[i];

	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(ov02b10->supplies),
				      ov02b10->supplies);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get regulators\n");

	ret = ov02b10_power_on(dev);
	if (ret)
		return ret;

	/* identify before parsing the endpoint, so DT can probe both depth variants */
	ret = ov02b10_identify(ov02b10);
	if (ret)
		goto power_off;

	ret = ov02b10_check_hwcfg(ov02b10);
	if (ret)
		goto power_off;

	ret = ov02b10_init_controls(ov02b10);
	if (ret)
		goto power_off;

	ov02b10->sd.state_lock = ov02b10->ctrl_handler.lock;
	ov02b10->sd.internal_ops = &ov02b10_internal_ops;
	ov02b10->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	ov02b10->sd.entity.ops = &ov02b10_entity_ops;
	ov02b10->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ov02b10->pad.flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&ov02b10->sd.entity, 1, &ov02b10->pad);
	if (ret)
		goto free_ctrls;

	ret = v4l2_subdev_init_finalize(&ov02b10->sd);
	if (ret)
		goto entity_cleanup;

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);

	ret = v4l2_async_register_subdev_sensor(&ov02b10->sd);
	if (ret)
		goto subdev_cleanup;

	pm_runtime_set_autosuspend_delay(dev, 1000);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_idle(dev);

	return 0;

subdev_cleanup:
	v4l2_subdev_cleanup(&ov02b10->sd);
	pm_runtime_disable(dev);
	pm_runtime_set_suspended(dev);
entity_cleanup:
	media_entity_cleanup(&ov02b10->sd.entity);
free_ctrls:
	v4l2_ctrl_handler_free(&ov02b10->ctrl_handler);
power_off:
	ov02b10_power_off(dev);

	return ret;
}

static void ov02b10_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ov02b10 *ov02b10 = to_ov02b10(sd);

	v4l2_async_unregister_subdev(sd);
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(&ov02b10->ctrl_handler);
	pm_runtime_disable(ov02b10->dev);
	if (!pm_runtime_status_suspended(ov02b10->dev)) {
		ov02b10_power_off(ov02b10->dev);
		pm_runtime_set_suspended(ov02b10->dev);
	}
}

static DEFINE_RUNTIME_DEV_PM_OPS(ov02b10_pm_ops, ov02b10_power_off,
				 ov02b10_power_on, NULL);

static const struct of_device_id ov02b10_of_match[] = {
	{ .compatible = "ovti,ov02b10" },
	{ .compatible = "ovti,ov02b1b" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, ov02b10_of_match);

static struct i2c_driver ov02b10_i2c_driver = {
	.driver = {
		.name = "ov02b10",
		.pm = pm_ptr(&ov02b10_pm_ops),
		.of_match_table = ov02b10_of_match,
	},
	.probe = ov02b10_probe,
	.remove = ov02b10_remove,
};
module_i2c_driver(ov02b10_i2c_driver);

MODULE_DESCRIPTION("OmniVision OV02B10/OV02B1B image sensor driver");
MODULE_LICENSE("GPL");
