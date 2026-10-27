// SPDX-License-Identifier: GPL-2.0-only
/*
 * Regulator driver for the OmniVision/Will Semiconductor WL2866D
 * four-channel camera LDO (two DVDD, two AVDD outputs) found on
 * Xiaomi Redmi Note 10 Pro (sweet).
 *
 * Register layout (from the vendor kernel behaviour):
 *   0x03 DVDD1 voltage   0x04 DVDD2 voltage
 *   0x05 AVDD1 voltage   0x06 AVDD2 voltage
 *   0x0e output enable, bit0..3 = DVDD1, DVDD2, AVDD1, AVDD2
 * DVDD:  0.6 V + code * 6 mV    AVDD: 1.2 V + code * 12.5 mV
 */

#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>

#define WL2866D_REG_DVDD1	0x03
#define WL2866D_REG_ENABLE	0x0e
#define WL2866D_DVDD1_DEFAULT	0x64

#define WL2866D_DVDD_NVOLT	201	/* 0.6 V .. 1.8 V */
#define WL2866D_AVDD_NVOLT	249	/* 1.2 V .. 4.3 V */

static const struct regmap_config wl2866d_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0x0f,
};

#define WL2866D_LDO(_id, _name, _supply, _vsel, _bit, _min, _step, _nv)	\
	{								\
		.name = _name,						\
		.of_match = _name,					\
		.regulators_node = "regulators",			\
		.supply_name = _supply,					\
		.id = _id,						\
		.type = REGULATOR_VOLTAGE,				\
		.owner = THIS_MODULE,					\
		.ops = &wl2866d_ops,					\
		.n_voltages = _nv,					\
		.min_uV = _min,						\
		.uV_step = _step,					\
		.vsel_reg = _vsel,					\
		.vsel_mask = 0xff,					\
		.enable_reg = WL2866D_REG_ENABLE,			\
		.enable_mask = BIT(_bit),				\
		.enable_time = 500,					\
	}

static const struct regulator_ops wl2866d_ops = {
	.enable = regulator_enable_regmap,
	.disable = regulator_disable_regmap,
	.is_enabled = regulator_is_enabled_regmap,
	.list_voltage = regulator_list_voltage_linear,
	.map_voltage = regulator_map_voltage_linear,
	.get_voltage_sel = regulator_get_voltage_sel_regmap,
	.set_voltage_sel = regulator_set_voltage_sel_regmap,
};

static const struct regulator_desc wl2866d_regulators[] = {
	WL2866D_LDO(0, "dvdd1", "vin1", 0x03, 0, 600000,  6000, WL2866D_DVDD_NVOLT),
	WL2866D_LDO(1, "dvdd2", "vin1", 0x04, 1, 600000,  6000, WL2866D_DVDD_NVOLT),
	WL2866D_LDO(2, "avdd1", "vin2", 0x05, 2, 1200000, 12500, WL2866D_AVDD_NVOLT),
	WL2866D_LDO(3, "avdd2", "vin2", 0x06, 3, 1200000, 12500, WL2866D_AVDD_NVOLT),
};

static int wl2866d_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct regulator_config config = { };
	struct gpio_desc *enable;
	struct regmap *regmap;
	unsigned int val;
	int i, ret;

	regmap = devm_regmap_init_i2c(client, &wl2866d_regmap_config);
	if (IS_ERR(regmap))
		return dev_err_probe(dev, PTR_ERR(regmap), "regmap init failed\n");

	/* The vendor kernel drives the enable pin as an output at the inactive level. */
	enable = devm_gpiod_get_optional(dev, "enable", GPIOD_OUT_LOW);
	if (IS_ERR(enable))
		return dev_err_probe(dev, PTR_ERR(enable), "enable gpio\n");

	usleep_range(10000, 11000);

	ret = regmap_read(regmap, WL2866D_REG_DVDD1, &val);
	if (ret)
		return dev_err_probe(dev, ret, "chip not responding\n");
	dev_dbg(dev, "DVDD1 register reads 0x%02x\n", val);

	config.dev = dev;
	config.regmap = regmap;

	for (i = 0; i < ARRAY_SIZE(wl2866d_regulators); i++) {
		struct regulator_dev *rdev;

		rdev = devm_regulator_register(dev, &wl2866d_regulators[i], &config);
		if (IS_ERR(rdev))
			return dev_err_probe(dev, PTR_ERR(rdev),
					     "failed to register %s\n",
					     wl2866d_regulators[i].name);
	}

	return 0;
}

static const struct of_device_id wl2866d_of_match[] = {
	{ .compatible = "ovti,wl2866d" },
	{ }
};
MODULE_DEVICE_TABLE(of, wl2866d_of_match);

static struct i2c_driver wl2866d_driver = {
	.driver = {
		.name = "wl2866d-regulator",
		.of_match_table = wl2866d_of_match,
	},
	.probe = wl2866d_probe,
};
module_i2c_driver(wl2866d_driver);

MODULE_DESCRIPTION("WL2866D camera LDO regulator driver");
MODULE_LICENSE("GPL");
