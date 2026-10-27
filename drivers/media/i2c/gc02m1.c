// SPDX-License-Identifier: GPL-2.0
/*
 * GalaxyCore GC02M1 2 MP raw image sensor driver (1600x1200, 1 lane).
 *
 * Bring-up driver for the Xiaomi Redmi Note 10 Pro (sweet) depth camera.
 * The register sequence is the vendor (Qualcomm CamX) init table for the
 * Sunny GC02M1 module, 19.2 MHz MCLK. Register map (page select 0xfe,
 * exposure 0x03/0x04, frame length 0x41/0x42, gain 0xb6/0xb1/0xb2,
 * stream 0x3e) is shared with the GC02M2 (cf. Rockchip BSP gc02m2.c).
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

#define GC02M1_MCLK_FREQ		(19200 * HZ_PER_KHZ)
#define GC02M1_LINK_FREQ		(336 * HZ_PER_MHZ)
#define GC02M1_DATA_LANES		1
/* Vendor timing: line_length 2192, frame_length 1276, 30.03 fps */
#define GC02M1_PIXEL_RATE		(84 * HZ_PER_MHZ)

#define GC02M1_REG_PAGE			CCI_REG8(0xfe)
#define GC02M1_REG_CHIP_ID_H		CCI_REG8(0xf0)
#define GC02M1_REG_CHIP_ID_L		CCI_REG8(0xf1)
#define GC02M1_CHIP_ID			0x02e0

#define GC02M1_REG_STREAM		CCI_REG8(0x3e)
#define GC02M1_STREAM_ON		0x90
#define GC02M1_STREAM_OFF		0x00

#define GC02M1_REG_EXP_H		CCI_REG8(0x03)
#define GC02M1_REG_EXP_L		CCI_REG8(0x04)
#define GC02M1_EXPOSURE_MIN		4
#define GC02M1_EXPOSURE_MARGIN		16

#define GC02M1_REG_VTS_H		CCI_REG8(0x41)
#define GC02M1_REG_VTS_L		CCI_REG8(0x42)
#define GC02M1_VTS_MAX			0x3fff

#define GC02M1_REG_AGAIN		CCI_REG8(0xb6)
#define GC02M1_REG_DGAIN_H		CCI_REG8(0xb1)
#define GC02M1_REG_DGAIN_L		CCI_REG8(0xb2)
#define GC02M1_GAIN_MIN			64	/* 1.0x, 1/64 units */
#define GC02M1_GAIN_MAX			1023
#define GC02M1_GAIN_DEFAULT		64

/* page 1 */
#define GC02M1_REG_TEST_PATTERN		CCI_REG8(0x8c)
#define GC02M1_TEST_PATTERN_OFF		0x10
#define GC02M1_TEST_PATTERN_ON		0x11

#define GC02M1_WIDTH			1600
#define GC02M1_HEIGHT			1200
#define GC02M1_HTS			2192
#define GC02M1_VTS_DEF			1268	/* 0x41/0x42 = 0x04f4 */
#define GC02M1_EXPOSURE_DEF		1141	/* 0x03/0x04 = 0x047d */

#define to_gc02m1(_sd)			container_of(_sd, struct gc02m1, sd)

static const char * const gc02m1_supply_names[] = {
	"dovdd",	/* 1.8 V I/O */
	"avdd",		/* 2.8 V analog */
	"dvdd",		/* optional, the sensor has an internal DVDD LDO */
};

static const s64 gc02m1_link_freq_menu[] = {
	GC02M1_LINK_FREQ,
};

static const char * const gc02m1_test_pattern_menu[] = {
	"Disabled",
	"Color bars",
};

/*
 * Analog gain steps, from the GC02M2 vendor table (same analog front end).
 * { total gain in 1/64 units, value of register 0xb6 }
 */
static const u16 gc02m1_again_table[][2] = {
	{  64,  0 }, {  96,  1 }, { 127,  2 }, { 157,  3 }, { 198,  4 },
	{ 227,  5 }, { 259,  6 }, { 287,  7 }, { 318,  8 }, { 356,  9 },
	{ 392, 10 }, { 420, 11 }, { 451, 12 }, { 480, 13 }, { 513, 14 },
	{ 646, 15 },
};

struct gc02m1 {
	struct device *dev;
	struct regmap *regmap;
	struct clk *mclk;
	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[ARRAY_SIZE(gc02m1_supply_names)];

	struct v4l2_subdev sd;
	struct media_pad pad;

	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *exposure;
};

/* Vendor (CamX) initSettings, Sunny GC02M1 module, 1600x1200 RAW10 1 lane */
static const struct cci_reg_sequence gc02m1_init_regs[] = {
	{ CCI_REG8(0xfc), 0x01 },
	{ CCI_REG8(0xf4), 0x41 },
	{ CCI_REG8(0xf5), 0xc0 },
	{ CCI_REG8(0xf6), 0x44 },
	{ CCI_REG8(0xf8), 0x46 },
	{ CCI_REG8(0xf9), 0x82 },
	{ CCI_REG8(0xfa), 0x00 },
	{ CCI_REG8(0xfd), 0x80 },
	{ CCI_REG8(0xfc), 0x81 },
	{ CCI_REG8(0xfe), 0x03 },
	{ CCI_REG8(0x01), 0x0b },
	{ CCI_REG8(0xf7), 0x01 },
	{ CCI_REG8(0xfc), 0x80 },
	{ CCI_REG8(0xfc), 0x80 },
	{ CCI_REG8(0xfc), 0x80 },
	{ CCI_REG8(0xfc), 0x8e },
	{ CCI_REG8(0xfe), 0x00 },
	{ CCI_REG8(0x87), 0x09 },
	{ CCI_REG8(0xee), 0x72 },
	{ CCI_REG8(0xfe), 0x01 },
	{ CCI_REG8(0x8c), 0x90 },
	{ CCI_REG8(0xfe), 0x00 },
	{ CCI_REG8(0x90), 0x00 },
	{ CCI_REG8(0x03), 0x04 },
	{ CCI_REG8(0x04), 0x7d },
	{ CCI_REG8(0x41), 0x04 },
	{ CCI_REG8(0x42), 0xf4 },
	{ CCI_REG8(0x05), 0x04 },
	{ CCI_REG8(0x06), 0x48 },
	{ CCI_REG8(0x07), 0x00 },
	{ CCI_REG8(0x08), 0x18 },
	{ CCI_REG8(0x9d), 0x18 },
	{ CCI_REG8(0x09), 0x00 },
	{ CCI_REG8(0x0a), 0x02 },
	{ CCI_REG8(0x0d), 0x04 },
	{ CCI_REG8(0x0e), 0xbc },
	{ CCI_REG8(0x17), 0x80 },
	{ CCI_REG8(0x19), 0x04 },
	{ CCI_REG8(0x24), 0x00 },
	{ CCI_REG8(0x56), 0x20 },
	{ CCI_REG8(0x5b), 0x00 },
	{ CCI_REG8(0x5e), 0x01 },
	{ CCI_REG8(0x21), 0x3c },
	{ CCI_REG8(0x44), 0x20 },
	{ CCI_REG8(0xcc), 0x01 },
	{ CCI_REG8(0x1a), 0x04 },
	{ CCI_REG8(0x1f), 0x11 },
	{ CCI_REG8(0x27), 0x30 },
	{ CCI_REG8(0x2b), 0x00 },
	{ CCI_REG8(0x33), 0x00 },
	{ CCI_REG8(0x53), 0x90 },
	{ CCI_REG8(0xe6), 0x50 },
	{ CCI_REG8(0x39), 0x07 },
	{ CCI_REG8(0x43), 0x04 },
	{ CCI_REG8(0x46), 0x2a },
	{ CCI_REG8(0x7c), 0xa0 },
	{ CCI_REG8(0xd0), 0xbe },
	{ CCI_REG8(0xd1), 0x60 },
	{ CCI_REG8(0xd2), 0x40 },
	{ CCI_REG8(0xd3), 0xf3 },
	{ CCI_REG8(0xde), 0x1d },
	{ CCI_REG8(0xcd), 0x05 },
	{ CCI_REG8(0xce), 0x6f },
	{ CCI_REG8(0xfc), 0x88 },
	{ CCI_REG8(0xfe), 0x10 },
	{ CCI_REG8(0xfe), 0x00 },
	{ CCI_REG8(0xfc), 0x8e },
	{ CCI_REG8(0xfe), 0x00 },
	{ CCI_REG8(0xfe), 0x00 },
	{ CCI_REG8(0xfe), 0x00 },
	{ CCI_REG8(0xfe), 0x00 },
	{ CCI_REG8(0xfc), 0x88 },
	{ CCI_REG8(0xfe), 0x10 },
	{ CCI_REG8(0xfe), 0x00 },
	{ CCI_REG8(0xfc), 0x8e },
	{ CCI_REG8(0xfe), 0x04 },
	{ CCI_REG8(0xe0), 0x01 },
	{ CCI_REG8(0xfe), 0x00 },
	{ CCI_REG8(0xfe), 0x01 },
	{ CCI_REG8(0x53), 0x44 },
	{ CCI_REG8(0x87), 0x51 },
	{ CCI_REG8(0x89), 0x03 },
	{ CCI_REG8(0xfe), 0x00 },
	{ CCI_REG8(0xb0), 0x74 },
	{ CCI_REG8(0xb1), 0x04 },
	{ CCI_REG8(0xb2), 0x00 },
	{ CCI_REG8(0xb6), 0x00 },
	{ CCI_REG8(0xfe), 0x04 },
	{ CCI_REG8(0xd8), 0x00 },
	{ CCI_REG8(0xc0), 0x40 },
	{ CCI_REG8(0xc0), 0x00 },
	{ CCI_REG8(0xc0), 0x00 },
	{ CCI_REG8(0xc0), 0x00 },
	{ CCI_REG8(0xc0), 0x60 },
	{ CCI_REG8(0xc0), 0x00 },
	{ CCI_REG8(0xc0), 0xc0 },
	{ CCI_REG8(0xc0), 0x2a },
	{ CCI_REG8(0xc0), 0x80 },
	{ CCI_REG8(0xc0), 0x00 },
	{ CCI_REG8(0xc0), 0x00 },
	{ CCI_REG8(0xc0), 0x40 },
	{ CCI_REG8(0xc0), 0xa0 },
	{ CCI_REG8(0xc0), 0x00 },
	{ CCI_REG8(0xc0), 0x90 },
	{ CCI_REG8(0xc0), 0x19 },
	{ CCI_REG8(0xc0), 0xc0 },
	{ CCI_REG8(0xc0), 0x00 },
	{ CCI_REG8(0xc0), 0xd0 },
	{ CCI_REG8(0xc0), 0x2f },
	{ CCI_REG8(0xc0), 0xe0 },
	{ CCI_REG8(0xc0), 0x00 },
	{ CCI_REG8(0xc0), 0x90 },
	{ CCI_REG8(0xc0), 0x39 },
	{ CCI_REG8(0xc0), 0x00 },
	{ CCI_REG8(0xc0), 0x01 },
	{ CCI_REG8(0xc0), 0x20 },
	{ CCI_REG8(0xc0), 0x04 },
	{ CCI_REG8(0xc0), 0x20 },
	{ CCI_REG8(0xc0), 0x01 },
	{ CCI_REG8(0xc0), 0xe0 },
	{ CCI_REG8(0xc0), 0x0f },
	{ CCI_REG8(0xc0), 0x40 },
	{ CCI_REG8(0xc0), 0x01 },
	{ CCI_REG8(0xc0), 0xe0 },
	{ CCI_REG8(0xc0), 0x1a },
	{ CCI_REG8(0xc0), 0x60 },
	{ CCI_REG8(0xc0), 0x01 },
	{ CCI_REG8(0xc0), 0x20 },
	{ CCI_REG8(0xc0), 0x25 },
	{ CCI_REG8(0xc0), 0x80 },
	{ CCI_REG8(0xc0), 0x01 },
	{ CCI_REG8(0xc0), 0xa0 },
	{ CCI_REG8(0xc0), 0x2c },
	{ CCI_REG8(0xc0), 0xa0 },
	{ CCI_REG8(0xc0), 0x01 },
	{ CCI_REG8(0xc0), 0xe0 },
	{ CCI_REG8(0xc0), 0x32 },
	{ CCI_REG8(0xc0), 0xc0 },
	{ CCI_REG8(0xc0), 0x01 },
	{ CCI_REG8(0xc0), 0x20 },
	{ CCI_REG8(0xc0), 0x38 },
	{ CCI_REG8(0xc0), 0xe0 },
	{ CCI_REG8(0xc0), 0x01 },
	{ CCI_REG8(0xc0), 0x60 },
	{ CCI_REG8(0xc0), 0x3c },
	{ CCI_REG8(0xc0), 0x00 },
	{ CCI_REG8(0xc0), 0x02 },
	{ CCI_REG8(0xc0), 0xa0 },
	{ CCI_REG8(0xc0), 0x40 },
	{ CCI_REG8(0xc0), 0x80 },
	{ CCI_REG8(0xc0), 0x02 },
	{ CCI_REG8(0xc0), 0x18 },
	{ CCI_REG8(0xc0), 0x5c },
	{ CCI_REG8(0xfe), 0x00 },
	{ CCI_REG8(0x9f), 0x10 },
	{ CCI_REG8(0xfe), 0x00 },
	{ CCI_REG8(0x26), 0x20 },
	{ CCI_REG8(0xfe), 0x01 },
	{ CCI_REG8(0x40), 0x22 },
	{ CCI_REG8(0x46), 0x7f },
	{ CCI_REG8(0x49), 0x0f },
	{ CCI_REG8(0x4a), 0xf0 },
	{ CCI_REG8(0xfe), 0x04 },
	{ CCI_REG8(0x14), 0x80 },
	{ CCI_REG8(0x15), 0x80 },
	{ CCI_REG8(0x16), 0x80 },
	{ CCI_REG8(0x17), 0x80 },
	{ CCI_REG8(0xfe), 0x01 },
	{ CCI_REG8(0x41), 0x20 },
	{ CCI_REG8(0x4c), 0x00 },
	{ CCI_REG8(0x4d), 0x0c },
	{ CCI_REG8(0x44), 0x08 },
	{ CCI_REG8(0x48), 0x03 },
	{ CCI_REG8(0xfe), 0x01 },
	{ CCI_REG8(0x90), 0x01 },
	{ CCI_REG8(0x91), 0x00 },
	{ CCI_REG8(0x92), 0x06 },
	{ CCI_REG8(0x93), 0x00 },
	{ CCI_REG8(0x94), 0x06 },
	{ CCI_REG8(0x95), 0x04 },
	{ CCI_REG8(0x96), 0xb0 },
	{ CCI_REG8(0x97), 0x06 },
	{ CCI_REG8(0x98), 0x40 },
	{ CCI_REG8(0xfe), 0x03 },
	{ CCI_REG8(0x01), 0x23 },
	{ CCI_REG8(0x03), 0xce },
	{ CCI_REG8(0x04), 0x48 },
	{ CCI_REG8(0x15), 0x00 },
	{ CCI_REG8(0x21), 0x10 },
	{ CCI_REG8(0x22), 0x05 },
	{ CCI_REG8(0x23), 0x20 },
	{ CCI_REG8(0x25), 0x20 },
	{ CCI_REG8(0x26), 0x08 },
	{ CCI_REG8(0x29), 0x06 },
	{ CCI_REG8(0x2a), 0x0a },
	{ CCI_REG8(0x2b), 0x08 },
	{ CCI_REG8(0xfe), 0x01 },
	{ CCI_REG8(0x8c), 0x10 },
	{ CCI_REG8(0xfe), 0x00 },
};

static int gc02m1_write_gain(struct gc02m1 *gc02m1, u32 gain)
{
	unsigned int i;
	u32 dgain;
	int ret = 0;

	for (i = ARRAY_SIZE(gc02m1_again_table) - 1; i > 0; i--)
		if (gain >= gc02m1_again_table[i][0])
			break;

	/* digital pre-gain, 0x400 == 1.0x */
	dgain = gain * 1024 / gc02m1_again_table[i][0];

	cci_write(gc02m1->regmap, GC02M1_REG_PAGE, 0x00, &ret);
	cci_write(gc02m1->regmap, GC02M1_REG_AGAIN,
		  gc02m1_again_table[i][1], &ret);
	cci_write(gc02m1->regmap, GC02M1_REG_DGAIN_H, dgain >> 8, &ret);
	cci_write(gc02m1->regmap, GC02M1_REG_DGAIN_L, dgain & 0xff, &ret);

	return ret;
}

static int gc02m1_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct gc02m1 *gc02m1 = container_of(ctrl->handler, struct gc02m1,
					     ctrl_handler);
	s64 exposure_max;
	u32 vts;
	int ret = 0;

	if (ctrl->id == V4L2_CID_VBLANK) {
		exposure_max = GC02M1_HEIGHT + ctrl->val - GC02M1_EXPOSURE_MARGIN;
		__v4l2_ctrl_modify_range(gc02m1->exposure,
					 gc02m1->exposure->minimum,
					 exposure_max,
					 gc02m1->exposure->step,
					 gc02m1->exposure->default_value);
	}

	if (!pm_runtime_get_if_active(gc02m1->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		cci_write(gc02m1->regmap, GC02M1_REG_PAGE, 0x00, &ret);
		cci_write(gc02m1->regmap, GC02M1_REG_EXP_H,
			  (ctrl->val >> 8) & 0x3f, &ret);
		cci_write(gc02m1->regmap, GC02M1_REG_EXP_L, ctrl->val & 0xff,
			  &ret);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		ret = gc02m1_write_gain(gc02m1, ctrl->val);
		break;
	case V4L2_CID_VBLANK:
		vts = GC02M1_HEIGHT + ctrl->val;
		cci_write(gc02m1->regmap, GC02M1_REG_PAGE, 0x00, &ret);
		cci_write(gc02m1->regmap, GC02M1_REG_VTS_H, (vts >> 8) & 0x3f,
			  &ret);
		cci_write(gc02m1->regmap, GC02M1_REG_VTS_L, vts & 0xff, &ret);
		break;
	case V4L2_CID_TEST_PATTERN:
		cci_write(gc02m1->regmap, GC02M1_REG_PAGE, 0x01, &ret);
		cci_write(gc02m1->regmap, GC02M1_REG_TEST_PATTERN,
			  ctrl->val ? GC02M1_TEST_PATTERN_ON :
				      GC02M1_TEST_PATTERN_OFF, &ret);
		cci_write(gc02m1->regmap, GC02M1_REG_PAGE, 0x00, &ret);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	pm_runtime_put(gc02m1->dev);

	return ret;
}

static const struct v4l2_ctrl_ops gc02m1_ctrl_ops = {
	.s_ctrl = gc02m1_set_ctrl,
};

static int gc02m1_init_controls(struct gc02m1 *gc02m1)
{
	struct v4l2_ctrl_handler *hdl = &gc02m1->ctrl_handler;
	struct v4l2_fwnode_device_properties props;
	s64 vblank, hblank;
	int ret;

	v4l2_ctrl_handler_init(hdl, 9);

	gc02m1->link_freq = v4l2_ctrl_new_int_menu(hdl, &gc02m1_ctrl_ops,
						   V4L2_CID_LINK_FREQ, 0, 0,
						   gc02m1_link_freq_menu);
	if (gc02m1->link_freq)
		gc02m1->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	gc02m1->pixel_rate = v4l2_ctrl_new_std(hdl, &gc02m1_ctrl_ops,
					       V4L2_CID_PIXEL_RATE,
					       GC02M1_PIXEL_RATE,
					       GC02M1_PIXEL_RATE, 1,
					       GC02M1_PIXEL_RATE);

	hblank = GC02M1_HTS - GC02M1_WIDTH;
	gc02m1->hblank = v4l2_ctrl_new_std(hdl, &gc02m1_ctrl_ops,
					   V4L2_CID_HBLANK, hblank, hblank, 1,
					   hblank);
	if (gc02m1->hblank)
		gc02m1->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	vblank = GC02M1_VTS_DEF - GC02M1_HEIGHT;
	gc02m1->vblank = v4l2_ctrl_new_std(hdl, &gc02m1_ctrl_ops,
					   V4L2_CID_VBLANK, vblank,
					   GC02M1_VTS_MAX - GC02M1_HEIGHT, 1,
					   vblank);

	gc02m1->exposure = v4l2_ctrl_new_std(hdl, &gc02m1_ctrl_ops,
					     V4L2_CID_EXPOSURE,
					     GC02M1_EXPOSURE_MIN,
					     GC02M1_VTS_DEF -
					     GC02M1_EXPOSURE_MARGIN, 1,
					     GC02M1_EXPOSURE_DEF);

	v4l2_ctrl_new_std(hdl, &gc02m1_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			  GC02M1_GAIN_MIN, GC02M1_GAIN_MAX, 1,
			  GC02M1_GAIN_DEFAULT);

	v4l2_ctrl_new_std_menu_items(hdl, &gc02m1_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(gc02m1_test_pattern_menu) - 1,
				     0, 0, gc02m1_test_pattern_menu);

	ret = v4l2_fwnode_device_parse(gc02m1->dev, &props);
	if (ret)
		goto err_free;

	ret = v4l2_ctrl_new_fwnode_properties(hdl, &gc02m1_ctrl_ops, &props);
	if (ret)
		goto err_free;

	if (hdl->error) {
		ret = hdl->error;
		goto err_free;
	}

	gc02m1->sd.ctrl_handler = hdl;

	return 0;

err_free:
	v4l2_ctrl_handler_free(hdl);

	return ret;
}

static int gc02m1_enable_streams(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state, u32 pad,
				 u64 streams_mask)
{
	struct gc02m1 *gc02m1 = to_gc02m1(sd);
	int ret;

	ret = pm_runtime_resume_and_get(gc02m1->dev);
	if (ret)
		return ret;

	ret = cci_multi_reg_write(gc02m1->regmap, gc02m1_init_regs,
				  ARRAY_SIZE(gc02m1_init_regs), NULL);
	if (ret)
		goto err;

	ret = __v4l2_ctrl_handler_setup(gc02m1->sd.ctrl_handler);
	if (ret)
		goto err;

	cci_write(gc02m1->regmap, GC02M1_REG_PAGE, 0x00, &ret);
	cci_write(gc02m1->regmap, GC02M1_REG_STREAM, GC02M1_STREAM_ON, &ret);
	if (ret)
		goto err;

	return 0;

err:
	dev_err(gc02m1->dev, "failed to start streaming: %d\n", ret);
	pm_runtime_put_autosuspend(gc02m1->dev);

	return ret;
}

static int gc02m1_disable_streams(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state, u32 pad,
				  u64 streams_mask)
{
	struct gc02m1 *gc02m1 = to_gc02m1(sd);
	int ret = 0;

	cci_write(gc02m1->regmap, GC02M1_REG_PAGE, 0x00, &ret);
	cci_write(gc02m1->regmap, GC02M1_REG_STREAM, GC02M1_STREAM_OFF, &ret);
	if (ret)
		dev_err(gc02m1->dev, "failed to stop streaming: %d\n", ret);

	pm_runtime_put_autosuspend(gc02m1->dev);

	return ret;
}

static void gc02m1_fill_format(struct v4l2_mbus_framefmt *fmt)
{
	/* Bayer order assumed from the GC02M2 (mirror/flip register 0x17 = 0x80) */
	fmt->code = MEDIA_BUS_FMT_SRGGB10_1X10;
	fmt->width = GC02M1_WIDTH;
	fmt->height = GC02M1_HEIGHT;
	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	fmt->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->xfer_func = V4L2_XFER_FUNC_NONE;
}

static int gc02m1_set_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_format *fmt)
{
	gc02m1_fill_format(&fmt->format);
	*v4l2_subdev_state_get_format(state, 0) = fmt->format;

	return 0;
}

static int gc02m1_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index > 0)
		return -EINVAL;

	code->code = MEDIA_BUS_FMT_SRGGB10_1X10;

	return 0;
}

static int gc02m1_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index > 0 || fse->code != MEDIA_BUS_FMT_SRGGB10_1X10)
		return -EINVAL;

	fse->min_width = GC02M1_WIDTH;
	fse->max_width = GC02M1_WIDTH;
	fse->min_height = GC02M1_HEIGHT;
	fse->max_height = GC02M1_HEIGHT;

	return 0;
}

static int gc02m1_get_selection(struct v4l2_subdev *sd,
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
		sel->r.width = GC02M1_WIDTH;
		sel->r.height = GC02M1_HEIGHT;
		return 0;
	default:
		return -EINVAL;
	}
}

static int gc02m1_init_state(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state)
{
	gc02m1_fill_format(v4l2_subdev_state_get_format(state, 0));

	return 0;
}

static const struct v4l2_subdev_video_ops gc02m1_video_ops = {
	.s_stream = v4l2_subdev_s_stream_helper,
};

static const struct v4l2_subdev_pad_ops gc02m1_pad_ops = {
	.set_fmt = gc02m1_set_pad_format,
	.get_fmt = v4l2_subdev_get_fmt,
	.get_selection = gc02m1_get_selection,
	.enum_mbus_code = gc02m1_enum_mbus_code,
	.enum_frame_size = gc02m1_enum_frame_size,
	.enable_streams = gc02m1_enable_streams,
	.disable_streams = gc02m1_disable_streams,
};

static const struct v4l2_subdev_ops gc02m1_subdev_ops = {
	.video = &gc02m1_video_ops,
	.pad = &gc02m1_pad_ops,
};

static const struct v4l2_subdev_internal_ops gc02m1_internal_ops = {
	.init_state = gc02m1_init_state,
};

static const struct media_entity_operations gc02m1_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static int gc02m1_power_on(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct gc02m1 *gc02m1 = to_gc02m1(sd);
	int ret;

	gpiod_set_value_cansleep(gc02m1->reset_gpio, 1);

	ret = regulator_bulk_enable(ARRAY_SIZE(gc02m1->supplies),
				    gc02m1->supplies);
	if (ret)
		return ret;

	usleep_range(1000, 2000);

	ret = clk_prepare_enable(gc02m1->mclk);
	if (ret) {
		regulator_bulk_disable(ARRAY_SIZE(gc02m1->supplies),
				       gc02m1->supplies);
		return ret;
	}

	usleep_range(1000, 2000);
	gpiod_set_value_cansleep(gc02m1->reset_gpio, 0);
	usleep_range(3000, 4000);

	return 0;
}

static int gc02m1_power_off(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct gc02m1 *gc02m1 = to_gc02m1(sd);

	gpiod_set_value_cansleep(gc02m1->reset_gpio, 1);
	usleep_range(1000, 2000);
	clk_disable_unprepare(gc02m1->mclk);
	regulator_bulk_disable(ARRAY_SIZE(gc02m1->supplies), gc02m1->supplies);

	return 0;
}

static int gc02m1_identify(struct gc02m1 *gc02m1)
{
	u64 hi = 0, lo = 0;
	int ret = 0;

	cci_write(gc02m1->regmap, GC02M1_REG_PAGE, 0x00, &ret);
	cci_read(gc02m1->regmap, GC02M1_REG_CHIP_ID_H, &hi, &ret);
	cci_read(gc02m1->regmap, GC02M1_REG_CHIP_ID_L, &lo, &ret);
	if (ret)
		return dev_err_probe(gc02m1->dev, ret,
				     "failed to read chip id\n");

	if (((hi << 8) | lo) != GC02M1_CHIP_ID)
		return dev_err_probe(gc02m1->dev, -ENODEV,
				     "chip id mismatch: 0x%04llx != 0x%04x\n",
				     (hi << 8) | lo, GC02M1_CHIP_ID);

	dev_info(gc02m1->dev, "GC02M1 detected (chip id 0x%04x)\n",
		 GC02M1_CHIP_ID);

	return 0;
}

static int gc02m1_check_hwcfg(struct gc02m1 *gc02m1)
{
	struct v4l2_fwnode_endpoint bus_cfg = {
		.bus_type = V4L2_MBUS_CSI2_DPHY,
	};
	struct fwnode_handle *ep;
	unsigned long freq_bitmap;
	int ret;

	ep = fwnode_graph_get_next_endpoint(dev_fwnode(gc02m1->dev), NULL);
	if (!ep)
		return dev_err_probe(gc02m1->dev, -ENXIO,
				     "no endpoint (probe-only mode)\n");

	ret = v4l2_fwnode_endpoint_alloc_parse(ep, &bus_cfg);
	fwnode_handle_put(ep);
	if (ret)
		return ret;

	if (bus_cfg.bus.mipi_csi2.num_data_lanes != GC02M1_DATA_LANES) {
		ret = dev_err_probe(gc02m1->dev, -EINVAL,
				    "only 1 data lane is supported\n");
		goto out;
	}

	ret = v4l2_link_freq_to_bitmap(gc02m1->dev, bus_cfg.link_frequencies,
				       bus_cfg.nr_of_link_frequencies,
				       gc02m1_link_freq_menu,
				       ARRAY_SIZE(gc02m1_link_freq_menu),
				       &freq_bitmap);
out:
	v4l2_fwnode_endpoint_free(&bus_cfg);

	return ret;
}

static int gc02m1_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct gc02m1 *gc02m1;
	unsigned int i;
	int ret;

	gc02m1 = devm_kzalloc(dev, sizeof(*gc02m1), GFP_KERNEL);
	if (!gc02m1)
		return -ENOMEM;

	gc02m1->dev = dev;
	v4l2_i2c_subdev_init(&gc02m1->sd, client, &gc02m1_subdev_ops);

	gc02m1->regmap = devm_cci_regmap_init_i2c(client, 8);
	if (IS_ERR(gc02m1->regmap))
		return dev_err_probe(dev, PTR_ERR(gc02m1->regmap),
				     "failed to init CCI\n");

	gc02m1->mclk = devm_v4l2_sensor_clk_get(dev, NULL);
	if (IS_ERR(gc02m1->mclk))
		return dev_err_probe(dev, PTR_ERR(gc02m1->mclk),
				     "failed to get MCLK\n");

	if (clk_get_rate(gc02m1->mclk) != GC02M1_MCLK_FREQ)
		return dev_err_probe(dev, -EINVAL,
				     "MCLK %lu Hz not supported (need 19.2 MHz)\n",
				     clk_get_rate(gc02m1->mclk));

	gc02m1->reset_gpio = devm_gpiod_get_optional(dev, "reset",
						     GPIOD_OUT_HIGH);
	if (IS_ERR(gc02m1->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(gc02m1->reset_gpio),
				     "failed to get reset GPIO\n");

	for (i = 0; i < ARRAY_SIZE(gc02m1_supply_names); i++)
		gc02m1->supplies[i].supply = gc02m1_supply_names[i];

	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(gc02m1->supplies),
				      gc02m1->supplies);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get regulators\n");

	ret = gc02m1_power_on(dev);
	if (ret)
		return ret;

	/* identify before parsing the endpoint, so DT can probe both depth variants */
	ret = gc02m1_identify(gc02m1);
	if (ret)
		goto power_off;

	ret = gc02m1_check_hwcfg(gc02m1);
	if (ret)
		goto power_off;

	ret = gc02m1_init_controls(gc02m1);
	if (ret)
		goto power_off;

	gc02m1->sd.state_lock = gc02m1->ctrl_handler.lock;
	gc02m1->sd.internal_ops = &gc02m1_internal_ops;
	gc02m1->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	gc02m1->sd.entity.ops = &gc02m1_entity_ops;
	gc02m1->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	gc02m1->pad.flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&gc02m1->sd.entity, 1, &gc02m1->pad);
	if (ret)
		goto free_ctrls;

	ret = v4l2_subdev_init_finalize(&gc02m1->sd);
	if (ret)
		goto entity_cleanup;

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);

	ret = v4l2_async_register_subdev_sensor(&gc02m1->sd);
	if (ret)
		goto subdev_cleanup;

	pm_runtime_set_autosuspend_delay(dev, 1000);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_idle(dev);

	return 0;

subdev_cleanup:
	v4l2_subdev_cleanup(&gc02m1->sd);
	pm_runtime_disable(dev);
	pm_runtime_set_suspended(dev);
entity_cleanup:
	media_entity_cleanup(&gc02m1->sd.entity);
free_ctrls:
	v4l2_ctrl_handler_free(&gc02m1->ctrl_handler);
power_off:
	gc02m1_power_off(dev);

	return ret;
}

static void gc02m1_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct gc02m1 *gc02m1 = to_gc02m1(sd);

	v4l2_async_unregister_subdev(sd);
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(&gc02m1->ctrl_handler);
	pm_runtime_disable(gc02m1->dev);
	if (!pm_runtime_status_suspended(gc02m1->dev)) {
		gc02m1_power_off(gc02m1->dev);
		pm_runtime_set_suspended(gc02m1->dev);
	}
}

static DEFINE_RUNTIME_DEV_PM_OPS(gc02m1_pm_ops, gc02m1_power_off,
				 gc02m1_power_on, NULL);

static const struct of_device_id gc02m1_of_match[] = {
	{ .compatible = "galaxycore,gc02m1" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, gc02m1_of_match);

static struct i2c_driver gc02m1_i2c_driver = {
	.driver = {
		.name = "gc02m1",
		.pm = pm_ptr(&gc02m1_pm_ops),
		.of_match_table = gc02m1_of_match,
	},
	.probe = gc02m1_probe,
	.remove = gc02m1_remove,
};
module_i2c_driver(gc02m1_i2c_driver);

MODULE_DESCRIPTION("GalaxyCore GC02M1 image sensor driver");
MODULE_LICENSE("GPL");
