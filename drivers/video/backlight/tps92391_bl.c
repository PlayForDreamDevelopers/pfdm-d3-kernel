#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/of_gpio.h>
#include <linux/of_graph.h>
#include <linux/regulator/consumer.h>

#include "tps92391_bl.h"

//#define REGMAP

#ifdef REGMAP
#define MAX_REG	0xFF
#endif

#define TPS92391_MAX_BRIGHTNESS_LEVEL 4095
#define TPS92391_DEVICE_ID 0x63
#define TPS92391_REG_00 0x00
#define TPS92391_REG_01 0x01
#define TPS92391_REG_02 0x02
#define TPS92391_REG_DEVICE_ID 0x03
#define TPS92391_REG_BRIGHTNESS_LOW 0x10
#define TPS92391_REG_BRIGHTNESS_HIGH 0x11

struct tps92391_bl {
	struct i2c_client *i2c_client;
	struct device *dev;
	struct regmap *regmap;
	int reset_gpio;
	int en_gpio;
	int vdd_en_gpio;
	int vdd_en_gpio2;
	u32 device_id;

	bool suspended;
};

struct tps92391_bl *pdata_prim = NULL;
struct tps92391_bl *pdata_sec = NULL;

static int tps92391_bl_write(struct tps92391_bl *pdata, u8 reg, u16 val)
{
	int ret;
	u8 tx_data[] = {reg, 0x00, 0x00};
	struct i2c_client *client = pdata->i2c_client;
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.buf = tx_data,
			.len = ARRAY_SIZE(tx_data),
		},
	};

	tx_data[1] = (val & 0x0F00) >> 8;
	tx_data[2] = val & 0x00FF;

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));

	if (ret < 0) {
		pr_err("%s: reg 0x%02x, val %02x, error %d\n",
				__func__, reg, val, ret);
		return ret;
	}

	if (ret < ARRAY_SIZE(msgs)) {
		pr_err("%s: reg 0x%02x, val %02x, error %d, ARRAY_SIZE(msgs) = %d\n",
				__func__, reg, val, ret, ARRAY_SIZE(msgs));
		return -EAGAIN;
	}

	pr_err("%s: reg 0x%02x val %02x success %d\n",
			__func__, reg, val, ret);

	return 0;
}

#if 0
static int tps92391_bl_read(struct tps92391_bl *pdata, u8 reg, u8 *buf, u32 size)
{
	u8 vreg[] = {reg};

	struct i2c_client *client = pdata->i2c_client;
	struct i2c_msg msg[2] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 1,
			.buf = vreg,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = size,
			.buf = buf,
		}
	};

	if (i2c_transfer(client->adapter, msg, 2) != 2) {
		pr_err("i2c read failed\n");
		return -EIO;
	}

	return 0;
}
#endif

#ifdef REGMAP
static const struct regmap_config tps92391_bl_regmap_config = {
	.name = "tps92391_bl",
	.reg_bits = 8,
	.val_bits = 8,
	//.reg_stride = 4,
	.max_register = MAX_REG,
	.cache_type = REGCACHE_RBTREE,
	.reg_format_endian = REGMAP_ENDIAN_BIG,
	.val_format_endian = REGMAP_ENDIAN_LITTLE,
};
#endif

static int tps92391_bl_reg_init(struct tps92391_bl *pdata)
{
	int ret = 0;
	int i;

	pr_err("[lcm] %s: enter\n", __func__);

	for (i = 0; i < sizeof (tps92391_bl_init_reg) / sizeof (tps92391_bl_reg); i++) {
		ret = tps92391_bl_write(pdata, tps92391_bl_init_reg[i].reg, tps92391_bl_init_reg[i].val);
	}

	return ret;
}

#if 0
static int tps92391_bl_reg_dump(struct tps92391_bl *pdata)
{
	int ret = 0;
	u8 rbuf[1];
	int i;

	for (i = 0; i < sizeof (reg_dump) / sizeof (u8); i ++) {
		ret = tps92391_bl_read(pdata, reg_dump[i], rbuf, 1);
		pr_err("Read reg = 0x%02x val = 0x%02x\n", reg_dump[i], rbuf[0]);
	}

	return ret;
}
#endif
static int tps92391_bl_parse_dt(struct device *dev, struct tps92391_bl *pdata)
{
	struct device_node *np = dev->of_node;

	pdata->en_gpio = of_get_named_gpio(np, "backlight-en-gpio", 0);
	if (gpio_is_valid(pdata->en_gpio)) {
		if (gpio_request(pdata->en_gpio, "backlight-en-gpio")) {
			 pr_err("%s: tps92391 en gpio request failed\n", __func__);
		}
	} else {
		pr_err("%s: tps92391 bl en gpio not specified\n", __func__);
	}

	#if 0
	pdata->vdd_en_gpio = of_get_named_gpio(np, "vdd-en-gpio", 0);
	if (gpio_is_valid(pdata->vdd_en_gpio)) {
		if (gpio_request(pdata->vdd_en_gpio, "vdd-en-gpio")) {
			 pr_err("%s: tps92391 vdd en gpio request failed\n", __func__);
		}
	} else {
		pr_err("%s: tps92391 vdd en gpio not specified\n", __func__);
		pdata->vdd_en_gpio = -1;
	}

	pdata->vdd_en_gpio2 = of_get_named_gpio(np, "vdd-en-gpio2", 0);
	if (gpio_is_valid(pdata->vdd_en_gpio2)) {
		if (gpio_request(pdata->vdd_en_gpio2, "vdd-en-gpio2")) {
			 pr_err("%s: tps92391 vdd en gpio 2 request failed\n", __func__);
		}
	} else {
		pr_err("%s: tps92391 vdd en gpio 2 not specified\n", __func__);
		pdata->vdd_en_gpio2 = -1;
	}
	#endif

	return 0;
}

void tps92391_bl_power_on(struct tps92391_bl *pdata)
{
	int ret;
	pr_err("%s: %s +\n", __func__, pdata->dev->of_node->name);

	if (!pdata) {
		pr_err("%s: tps92391_bl is null\n", __func__);
		goto exit;
	}

	#if 0
	if (gpio_is_valid(pdata->vdd_en_gpio)) {
		ret = gpio_direction_output(pdata->vdd_en_gpio, 1);
		if (ret)
			pr_err("tps92391_bl vdd en gpio direction failed\n");
		msleep(20);
	}

	if (gpio_is_valid(pdata->vdd_en_gpio2)) {
		ret = gpio_direction_output(pdata->vdd_en_gpio2, 1);
		if (ret)
			pr_err("tps92391_bl vdd en gpio2 direction failed\n");
		msleep(100);
	}
	#endif

	if (gpio_is_valid(pdata->en_gpio)) {
		ret = gpio_direction_output(pdata->en_gpio, 1);
		if (ret)
			pr_err("tps92391_bl en gpio direction failed\n");
		msleep(50);
	}

exit:
	pr_err("%s: -\n", __func__);
}

void tps92391_bl_power_off(struct tps92391_bl *pdata)
{
	pr_err("%s: %s +\n",__func__, pdata->dev->of_node->name);

	if (!pdata) {
		pr_err("%s: tps92391_bl is null\n", __func__);
		goto exit;
	}

	if (gpio_is_valid(pdata->en_gpio)) {
		gpio_direction_output(pdata->en_gpio, 0);
		msleep(10);
	}

	#if 0
	if (gpio_is_valid(pdata->vdd_en_gpio)) {
		gpio_direction_output(pdata->vdd_en_gpio, 0);
		msleep(10);
	}

	if (gpio_is_valid(pdata->vdd_en_gpio2)) {
		gpio_direction_output(pdata->vdd_en_gpio2, 0);
		msleep(10);
	}
	#endif

exit:
	pr_err("%s: -\n",__func__);
}

static int tps92391_set_led_current(u32 level, u32 max_level, u32 power_on)
{
	int ret = 0;
	//u32 level_temp = 0;
	//u8 brightness_low;
	//u8 brightness_high;
	u16 brightness = level;

	if ((pdata_prim == NULL) && (pdata_sec == NULL)) {
		return 0;
	}

	if(pdata_prim->suspended && pdata_sec->suspended && (power_on == 0)){
		return 0;
	}
	
	//pr_err("%s: %d %d\n", __func__, level, max_level);
	pr_err("%s:[lcm] level = %d, max_level = %d, power_on = %d\n", __func__, level, max_level, power_on);
#if 0
	if (max_level == TPS92391_MAX_BRIGHTNESS_LEVEL) {
		brightness_low = level & 0xFF;
		brightness_high = level >> 8;
	} else if (max_level == 255) {
		level_temp = level *16;
		brightness_low = level_temp & 0xFF;
		brightness_high = level_temp >> 8;
	} else {
		brightness_low	 = 0xff;
		brightness_high = 0x00;
	}
#endif

	//pr_err("%s: %d, low %02x. high %02x\n", __func__, level, brightness_low, brightness_high);
	if (pdata_prim != NULL) {
		if (power_on == 1) {
			tps92391_bl_power_on(pdata_prim);
			//ret = tps92391_bl_reg_init(pdata_prim);
			pdata_prim->suspended = 0;
		} else {
			tps92391_bl_power_off(pdata_prim);
			pdata_prim->suspended = 1;
		}
		if(brightness){
			ret = tps92391_bl_write(pdata_prim, 0x02, brightness);
		}
	}

	if (pdata_sec != NULL) {
		if (power_on == 1) {
			tps92391_bl_power_on(pdata_sec);
			//ret = tps92391_bl_reg_init(pdata_sec);
			pdata_prim->suspended = 0;
		} else {
			tps92391_bl_power_off(pdata_sec);
			pdata_prim->suspended = 1;
		}
		if(brightness){
			ret = tps92391_bl_write(pdata_sec, 0x02, brightness);
		}
	}

	return ret;
}

int tps92391_bl_set_led_current(u32 level, u32 max_level, u32 power_on)
{
	int ret = 0;
	u32 device_id = 0;

	pr_err("%s: enter \n", __func__);

	if ((pdata_prim == NULL) && (pdata_sec == NULL)) {
		return 0;
	}

	if (pdata_prim != NULL)
		device_id = pdata_prim->device_id;

	switch(device_id) {
		case TPS92391_DEVICE_ID:
		default:
			ret = tps92391_set_led_current(level, max_level, power_on);
			break;
	}

	return ret;
}

EXPORT_SYMBOL(tps92391_bl_set_led_current);

static int tps92391_bl_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct tps92391_bl *pdata;
	int ret = 0;
	//u8 rbuf[1];

	if (!client || !client->dev.of_node) {
		pr_err("invalid input\n");
		return -EINVAL;
	}

	pr_err("[lcm] %s: enter, i2c flags = %d, addr = %d, name = %s, irq = %d", __func__,
			client->flags, client->addr, client->dev.of_node->name, client->irq);
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("device doesn't support I2C\n");
		return -ENODEV;
	}

	pdata = devm_kzalloc(&client->dev,
			sizeof(struct tps92391_bl), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	pdata->dev = &client->dev;
	pdata->i2c_client = client;

	i2c_set_clientdata(client, pdata);
	dev_set_drvdata(&client->dev, pdata);

#ifdef REGMAP
	pdata->regmap = devm_regmap_init_i2c(client, &tps92391_bl_regmap_config);
	if (IS_ERR(pdata->regmap)) {
		ret = PTR_ERR(pdata->regmap);
		pr_err("failed to initialize regmap: %d\n", ret);
		return ret;
	}
#endif

	tps92391_bl_parse_dt(&client->dev, pdata);
	tps92391_bl_power_on(pdata);

	#if 0
	ret = tps92391_bl_read(pdata, TPS92391_REG_DEVICE_ID, rbuf, 1);
	if (ret == 0) {
		pr_err("Read 0x03 val = 0x%02x\n", rbuf[0]);
		pdata->device_id = rbuf[0];
	} else {
		pr_err("%s: read id failed %d\n", __func__, ret);
		goto i2c_error;
	}
	#endif

	msleep(3);
	ret = tps92391_bl_reg_init(pdata);

	#if 0
	tps92391_bl_reg_dump(pdata);
	#endif

	if ((pdata_prim == NULL) && (!strcmp(client->dev.of_node->name, "tps92391_left")))
		pdata_prim = pdata;
	else if ((pdata_sec == NULL) && (!strcmp(client->dev.of_node->name, "tps92391_right")))
		pdata_sec = pdata;

	pr_err("[lcm] %s: dev name：%s\n", __func__, client->dev.of_node->name);

	return 0;

//i2c_error:
//	if (gpio_is_valid(pdata->en_gpio))
//		gpio_free(pdata->en_gpio);

	return ret;
}

static int tps92391_bl_remove(struct i2c_client *client)
{
	struct tps92391_bl *pdata;

	pdata = i2c_get_clientdata(client);

	if (gpio_is_valid(pdata->en_gpio))
		gpio_free(pdata->en_gpio);

	return 0;
}

static const struct i2c_device_id tps92391_bl_ids[] = {
	{ "tps92391_bl", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, tps92391_bl_ids);

static const struct of_device_id tps92391_bl_of_ids[] = {
	{ .compatible = "tps92391_bl", },
	{ }
};
MODULE_DEVICE_TABLE(of, tps92391_bl_of_ids);

static struct i2c_driver tps92391_bl_driver = {
	.driver = {
		.name = "tps92391_bl",
		.of_match_table = tps92391_bl_of_ids,
	},
	.id_table = tps92391_bl_ids,
	.probe = tps92391_bl_probe,
	.remove	= tps92391_bl_remove,
};
module_i2c_driver(tps92391_bl_driver);

MODULE_DESCRIPTION("i2c backlight driver");
MODULE_LICENSE("GPL");