// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2022-2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/thermal.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pwm.h>
#include <linux/gpio/consumer.h>
#include <linux/timer.h>
#include <linux/irq.h>


#define MAX31760_CTRL_REG1		0x00
#define MAX31760_CTRL_REG2		0x01
#define MAX31760_CTRL_REG3		0x02
#define MAX31760_DUTY_CYCLE_CTRL_REG	0x50
#define MAX31760_TC1H_REG		0x52
#define MAX31760_TC1L_REG		0x53
#define MAX31760_TC2H_REG		0x54
#define MAX31760_TC2L_REG		0x55

#define VDD_MAX_UV	3100000
#define VDD_MIN_UV	3000000
#define VDD_LOAD_UA	300000
#define VCCA_MAX_UV	1800000
#define VCCA_MIN_UV	1800000
#define VCCA_LOAD_UA	600000

#define FAN_SPEED_LEVEL0	0
#define FAN_SPEED_MAX		100
#define SPEED_CAL_CONST		(60 * 100000)
#define MSB_CONVERT_DEC		(256)
#define PWM_FACTOR		39

struct pwm_pinctrl_info {
	struct pinctrl *pinctrl;
	struct pinctrl_state *pwm;
};

struct max31760_data {
	struct device *dev;
	struct i2c_client *i2c_client;
	struct pwm_device *pwm;
	struct thermal_cooling_device *cdev;
	struct mutex update_lock;
	struct pwm_pinctrl_info fan_pinctrl;
	struct gpio_desc *irq_gpio;

	u32 fan_num;
	u32 pwr_en_gpio;
	u32 driver_en_gpio;
	unsigned int cur_state;
	unsigned int pwm_value;
	atomic_t in_suspend;

	int irq;
	atomic_t pulses;
	unsigned int tach_count;
	u8 pulses_per_revolution;
	ktime_t sample_start;
	struct timer_list tach_timer;
};

/* This handler assumes self resetting edge triggered interrupt. */
static irqreturn_t pulse_handler(int irq, void *dev_id)
{
	struct max31760_data *pdata = dev_id;

	atomic_inc(&pdata->pulses);

	return IRQ_HANDLED;
}

static void sample_timer(struct timer_list *t)
{
	struct max31760_data *pdata = from_timer(pdata, t, tach_timer);
	unsigned int delta = ktime_ms_delta(ktime_get(), pdata->sample_start);
	int pulses;

	if (delta) {
		pulses = atomic_read(&pdata->pulses);
		atomic_sub(pulses, &pdata->pulses);
		
		pdata->tach_count = (unsigned int)(pulses * 1000 * 60) /
			(pdata->pulses_per_revolution * delta);

		pdata->sample_start = ktime_get();
	}

	mod_timer(&pdata->tach_timer, jiffies + HZ);
}

static int max31760_read_byte(struct max31760_data *pdata, u8 reg, u8 *val)
{
	int ret;

	struct i2c_client *client = pdata->i2c_client;

	ret = i2c_smbus_read_byte_data(client, reg);
	if (ret < 0)
		dev_err(pdata->dev, "%s failed read reg 0x%02x failure, ret:%d\n", reg, ret);

	*val = (u8)ret;

	dev_dbg(pdata->dev, "success read reg 0x%x=0x%x\n", reg, *val);

	return ret;
}

static int max31760_write_byte(struct max31760_data *pdata, u8 reg, u8 val)
{
	int ret = 0;
	struct i2c_client *client = pdata->i2c_client;

	ret = i2c_smbus_write_byte_data(client, reg, val);
	if (ret < 0) {
		dev_err(pdata->dev, "failed write reg %#x failure, ret:%d\n", reg, ret);
		return ret;
	}

	dev_dbg(pdata->dev, "successfully write reg %#x=%#x\n", reg, val);
	return 0;
}

static void max31760_enable_gpio(struct max31760_data *pdata, int on)
{
	gpio_direction_output(pdata->pwr_en_gpio, on);
	gpio_direction_output(pdata->driver_en_gpio, on);
	dev_dbg(pdata->dev, "max31760 gpio:%d and gpio:%d set to %d\n", pdata->pwr_en_gpio,
		pdata->driver_en_gpio, on);
	usleep_range(20000, 20100);
}

static void max31760_speed_control(struct max31760_data *pdata, unsigned long level)
{
	unsigned long data;

	data = level * 255 / 100;
	max31760_write_byte(pdata, MAX31760_DUTY_CYCLE_CTRL_REG, data);
}

static void max31760_set_cur_state_common(struct max31760_data *pdata,
				unsigned long state)
{
	if (state > FAN_SPEED_MAX) {
		dev_err(pdata->dev, "max31760 fan state(%lu) set failed, need [0-%d]\n", FAN_SPEED_MAX);
		return;
	}

	if (!atomic_read(&pdata->in_suspend))
		max31760_speed_control(pdata, state);
	pdata->cur_state = state;
}

static int max31760_get_max_state(struct thermal_cooling_device *cdev,
				unsigned long *state)
{
	*state = FAN_SPEED_MAX;
	return 0;
}

static int max31760_get_cur_state(struct thermal_cooling_device *cdev,
				unsigned long *state)
{
	struct max31760_data *data = cdev->devdata;

	mutex_lock(&data->update_lock);
	*state = data->cur_state;
	mutex_unlock(&data->update_lock);

	return 0;
}

static int max31760_set_cur_state(struct thermal_cooling_device *cdev,
				unsigned long state)
{
	struct max31760_data *data = cdev->devdata;

	mutex_lock(&data->update_lock);
	max31760_set_cur_state_common(data, state);
	mutex_unlock(&data->update_lock);

	return 0;
}

static struct thermal_cooling_device_ops max31760_cooling_ops = {
	.get_max_state = max31760_get_max_state,
	.get_cur_state = max31760_get_cur_state,
	.set_cur_state = max31760_set_cur_state,
};

static ssize_t speed_control1_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct max31760_data *data = dev_get_drvdata(dev);
	int ret;

	if (!data) {
		pr_err("invalid driver pointer\n");
		return -ENODEV;
	}

	ret = scnprintf(buf, PAGE_SIZE, "%d\n", data->cur_state);

	return ret;
}

static ssize_t speed_control1_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct max31760_data *data = dev_get_drvdata(dev);
	unsigned long value;

	if (kstrtoul(buf, 0, &value))
		return -EINVAL;

	mutex_lock(&data->update_lock);
	max31760_set_cur_state_common(data, value);
	mutex_unlock(&data->update_lock);

	return count;
}

static int max31760_read_speed(struct max31760_data *data,
				u8 index, u32 *speed)
{
	u8 tch = 0, tcl = 0;
	u8 value = 0;
	int ret = 0;

	if (index == 1) {
		ret = max31760_read_byte(data, MAX31760_TC1H_REG, &value);
		if (ret < 0)
			return ret;
		tch = value;

		ret = max31760_read_byte(data, MAX31760_TC1L_REG, &value);
		if (ret < 0)
			return ret;
		tcl = value;
	} else if (index == 2) {
		ret = max31760_read_byte(data, MAX31760_TC2H_REG, &value);
		if (ret < 0)
			return ret;
		tch = value;

		ret = max31760_read_byte(data, MAX31760_TC2L_REG, &value);
		if (ret < 0)
			return ret;
		tcl = value;
	}

	*speed = SPEED_CAL_CONST / (tch * MSB_CONVERT_DEC + tcl) / 2;

	return 0;
}

static ssize_t speed_tc1_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct max31760_data *data = dev_get_drvdata(dev);
	u32 speed;
	int ret;

	if (!data) {
		pr_err("invalid driver pointer\n");
		return -ENODEV;
	}

	ret = max31760_read_speed(data, 1, &speed);
	if (ret < 0) {
		dev_err(data->dev, "can not read fan speed\n");
		return -EINVAL;
	}

	ret = scnprintf(buf, PAGE_SIZE, "%d\n", speed);
	dev_dbg(data->dev, "TC1 current speed is %d\n", speed);

	return ret;
}

static ssize_t speed_tc2_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct max31760_data *pdata = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", pdata->tach_count);
}

static ssize_t pwm_duty_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct max31760_data *data = dev_get_drvdata(dev);
	u32 duty;
	u8 value = 0;
	int ret;

	if (!data) {
		pr_err("invalid driver pointer\n");
		return -ENODEV;
	}

	ret = max31760_read_byte(data, MAX31760_DUTY_CYCLE_CTRL_REG, &value);
	if (ret < 0)
		return ret;

	duty = value * PWM_FACTOR;
	ret = scnprintf(buf, PAGE_SIZE, "%d\n", duty);

	return ret;
}

#ifdef CONFIG_PRODUCT_YVR_D3
static ssize_t i2c_store(struct device *dev, struct device_attribute *attr,
                                     const char *buf, size_t len)
{
       struct max31760_data *pd = dev_get_drvdata(dev);
       int rc = 0;
       int cmd = 0;
       u8 addr[1] = {0};
       u8 buffer[1] = {0};

       if (!pd) {
               pr_err("max: data is null\n");
               goto exit;
       }

       sscanf(buf,"%d %x %x", &cmd, &addr, &buffer);
       pr_err("max: get cmd=%d 0x%02x 0x%02x\n", cmd, addr[0], buffer[0]);

       if (cmd == 0) {
               max31760_write_byte(pd, addr[0], buffer[0]);
       } else {
               rc = max31760_read_byte(pd, addr[0], &buffer[0]);
               if (rc < 0) {
                       pr_err("max: read 0x%02x err\n", addr[0]);
               } else {
                       pr_err("max: read 0x%02x 0x%02x\n", addr[0], buffer[0]);
               }
       }
exit:
       return len;
}
#endif

static int  __set_pwm(struct max31760_data *pdata, unsigned long pwm)
{
	unsigned long period;
	int ret = 0;
	struct pwm_state state = { };

	mutex_lock(&pdata->update_lock);
	if (pdata->pwm_value == pwm)
		goto exit_set_pwm_err;

	pwm_init_state(pdata->pwm, &state);
	period = pdata->pwm->args.period;
	state.duty_cycle = DIV_ROUND_UP(pwm * (period - 1), FAN_SPEED_MAX);
	state.enabled = pwm ? true : false;

	ret = pwm_apply_state(pdata->pwm, &state);
	if (!ret)
		pdata->pwm_value = pwm;
exit_set_pwm_err:
	mutex_unlock(&pdata->update_lock);
	return ret;
}

static ssize_t speed_control2_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	struct max31760_data *pdata = dev_get_drvdata(dev);
	unsigned long pwm;
	int ret;

	if (kstrtoul(buf, 10, &pwm) || pwm > FAN_SPEED_MAX)
		return -EINVAL;

	ret = __set_pwm(pdata, pwm);
	if (ret)
		return ret;

	pdata->pwm_value = pwm;

	return count;
}

static ssize_t speed_control2_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct max31760_data *pdata = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", pdata->pwm_value);
}

static DEVICE_ATTR_RW(speed_control1);
static DEVICE_ATTR_RW(speed_control2);
static DEVICE_ATTR_RO(speed_tc1);
static DEVICE_ATTR_RO(speed_tc2);
static DEVICE_ATTR_RO(pwm_duty);
#ifdef CONFIG_PRODUCT_YVR_D3
static DEVICE_ATTR_WO(i2c);
#endif

static struct attribute *max31760_sysfs_attrs[] = {
	&dev_attr_speed_control1.attr,
	&dev_attr_speed_control2.attr,
	&dev_attr_speed_tc1.attr,
	&dev_attr_speed_tc2.attr,
	&dev_attr_pwm_duty.attr,
#ifdef CONFIG_PRODUCT_YVR_D3
	&dev_attr_i2c.attr,
#endif
	NULL,
};

static int max31760_register_cdev(struct max31760_data *pdata)
{
	int ret = 0;
	char cdev_name[THERMAL_NAME_LENGTH] = "";

	snprintf(cdev_name, THERMAL_NAME_LENGTH, "fan-max31760");

	pdata->cdev = thermal_of_cooling_device_register(pdata->dev->of_node, cdev_name,
						pdata, &max31760_cooling_ops);
	if (IS_ERR(pdata->cdev)) {
		ret = PTR_ERR(pdata->cdev);
		dev_err(pdata->dev, "Cooling register failed for %s, ret:%d\n", cdev_name, ret);
		pdata->cdev = NULL;
		return ret;
	}

	dev_dbg(pdata->dev, "Cooling register success for %s\n", cdev_name);
	return 0;
}

static void max31760_hw_init(struct max31760_data *pdata)
{
	max31760_write_byte(pdata, MAX31760_CTRL_REG1, 0x19);
	max31760_write_byte(pdata, MAX31760_CTRL_REG2, 0x11);
	if (pdata->fan_num == 1)
		max31760_write_byte(pdata, MAX31760_CTRL_REG3, 0x31);
	else if (pdata->fan_num == 2)
		max31760_write_byte(pdata, MAX31760_CTRL_REG3, 0x33);
	mutex_lock(&pdata->update_lock);
	max31760_speed_control(pdata, FAN_SPEED_LEVEL0);
	pdata->cur_state = FAN_SPEED_LEVEL0;
	mutex_unlock(&pdata->update_lock);

	atomic_set(&pdata->in_suspend, 0);
}

static int max31760_parse_dt(struct max31760_data *pdata)
{
	int ret = 0;
	struct device_node *node = pdata->dev->of_node;

	if (!node) {
		pr_err("device tree info missing\n");
		return -EINVAL;
	}

	ret = of_property_read_u32(node, "maxim,fan-num", &pdata->fan_num);
	if (ret)
		pdata->fan_num = 1;
	if (pdata->fan_num > 2)
		pdata->fan_num = 2;

	pdata->pwr_en_gpio = of_get_named_gpio(node, "maxim,pwr-en-gpio", 0);
	if (!gpio_is_valid(pdata->pwr_en_gpio)) {
		dev_err(pdata->dev, "enable gpio not specified\n");
		return -EINVAL;
	}

	pdata->driver_en_gpio = of_get_named_gpio(node, "maxim,driver-en-gpio", 0);
	if (!gpio_is_valid(pdata->driver_en_gpio)) {
		dev_err(pdata->dev, "enable gpio not specified\n");
		return -EINVAL;
	}

	ret = gpio_request(pdata->pwr_en_gpio, "pwr_en_gpio");
	if (ret) {
		pr_err("max31760 enable gpio request failed, ret:%d\n", ret);
		goto error;
	}

	ret = gpio_request(pdata->driver_en_gpio, "driver_en_gpio");
	if (ret) {
		pr_err("max31760 drvr enable gpio request failed, ret:%d\n", ret);
		goto error;
	}

	max31760_enable_gpio(pdata, 1);

	return ret;

error:
	gpio_free(pdata->pwr_en_gpio);
	gpio_free(pdata->driver_en_gpio);
	return ret;
}

static struct attribute_group max31760_attribute_group = {
	.attrs = max31760_sysfs_attrs,
};

static int max31760_remove(struct i2c_client *client)
{
	struct max31760_data *pdata = i2c_get_clientdata(client);

	if (!pdata)
		return 0;

	pwm_disable(pdata->pwm);
	del_timer_sync(&pdata->tach_timer);
	thermal_cooling_device_unregister(pdata->cdev);
	max31760_enable_gpio(pdata, 0);
	gpio_free(pdata->pwr_en_gpio);
	gpio_free(pdata->driver_en_gpio);

	return 0;
}

static int pwm_fan_pinctrl_init(struct i2c_client *client,
					struct max31760_data *pdata)
{
	int rc = 0;

	pdata->fan_pinctrl.pinctrl = devm_pinctrl_get(&client->dev);
	if (IS_ERR_OR_NULL(pdata->fan_pinctrl.pinctrl)) {
		rc = PTR_ERR(pdata->fan_pinctrl.pinctrl);
		pr_err("failed to get pinctrl, rc=%d\n", rc);
		goto error;
	}

	pdata->fan_pinctrl.pwm = pinctrl_lookup_state(pdata->fan_pinctrl.pinctrl,
						       "fan_pwm_out");
	if (IS_ERR_OR_NULL(pdata->fan_pinctrl.pwm)) {
		rc = PTR_ERR(pdata->fan_pinctrl.pwm);
		pr_err("failed to get pinctrl pwm out state, rc=%d\n", rc);
		goto error;
	}

error:
	return rc;
}

static int max31760_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0;
	struct max31760_data *pdata;
	struct device *dev = &client->dev;
	struct pwm_state state = { };

	if (!client || !client->dev.of_node) {
		pr_err("max31760 invalid input\n");
		return -EINVAL;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "device doesn't support I2C\n");
		return -ENODEV;
	}

	pdata = devm_kzalloc(&client->dev, sizeof(struct max31760_data), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	pdata->dev = &client->dev;
	pdata->i2c_client = client;
	i2c_set_clientdata(client, pdata);
	dev_set_drvdata(&client->dev, pdata);
	mutex_init(&pdata->update_lock);

	ret = max31760_parse_dt(pdata);
	if (ret) {
		dev_err(pdata->dev, "failed to parse device tree, ret:%d\n", ret);
		goto fail_parse_dt;
	}

	pdata->pwm = devm_of_pwm_get(dev, dev->of_node, NULL);
	if (IS_ERR(pdata->pwm))
		return dev_err_probe(dev, PTR_ERR(pdata->pwm), "Could not get PWM\n");

	max31760_hw_init(pdata);

	pwm_fan_pinctrl_init(client, pdata);

	ret = pinctrl_select_state(pdata->fan_pinctrl.pinctrl,
			pdata->fan_pinctrl.pwm);
	if (ret)
		dev_err(&client->dev, "Failed to set pinctrl avtive state!\n");

	pdata->pwm_value = FAN_SPEED_MAX/2;

	pwm_init_state(pdata->pwm, &state);

	/* Set duty cycle to 50% allowed and enable PWM output */
	state.duty_cycle = 
		DIV_ROUND_UP((pdata->pwm_value) * (pdata->pwm->args.period - 1), FAN_SPEED_MAX);
	state.enabled = true;

	ret = pwm_apply_state(pdata->pwm, &state);
	if (ret) {
		dev_err(dev, "Failed to configure PWM: %d\n", ret);
		return ret;
	}

	pdata->pulses_per_revolution = 2;

	timer_setup(&pdata->tach_timer, sample_timer, 0);

	pdata->irq_gpio = devm_gpiod_get_optional(dev, "irq",GPIOD_IN);
	if (IS_ERR(pdata->irq_gpio)){
		ret = PTR_ERR(pdata->irq_gpio);
        dev_err(dev, "failed to get gpio, ret:%d\n", ret);
	}
	
	pdata->irq = gpiod_to_irq(pdata->irq_gpio);
	if (pdata->irq < 0) {
		ret = pdata->irq;
		dev_err(dev,
			"Unable to get irq number for GPIO, error %d\n",ret);
	}

	if (pdata->irq > 0) {
		ret = devm_request_irq(dev, pdata->irq, pulse_handler, IRQ_TYPE_EDGE_FALLING,
				       "max31760", pdata);
		if (ret) {
			dev_err(dev, "Failed to request interrupt: %d\n", ret);
			return ret;
		}
		pdata->sample_start = ktime_get();
		mod_timer(&pdata->tach_timer, jiffies + HZ);
	}

	ret = max31760_register_cdev(pdata);
	if (ret) {
		dev_err(pdata->dev, "failed to register cooling device, ret:%d\n", ret);
		goto fail_register_cdev;
	}

	ret = devm_device_add_group(&client->dev, &max31760_attribute_group);
	if (ret < 0) {
		dev_err(pdata->dev, "couldn't register sysfs group\n");
		return ret;
	}

	return ret;

fail_register_cdev:
	max31760_remove(client);
	return ret;
fail_parse_dt:
	i2c_set_clientdata(client, NULL);
	dev_set_drvdata(&client->dev, NULL);
	return ret;
}

static void max31760_shutdown(struct i2c_client *client)
{
	max31760_remove(client);
}

static int max31760_suspend(struct device *dev)
{
	struct max31760_data *pdata = dev_get_drvdata(dev);

	dev_dbg(dev, "enter suspend now\n");
	if (pdata) {
		atomic_set(&pdata->in_suspend, 1);
		mutex_lock(&pdata->update_lock);
		pwm_disable(pdata->pwm);
		max31760_speed_control(pdata, FAN_SPEED_LEVEL0);
		max31760_enable_gpio(pdata, 0);
		mutex_unlock(&pdata->update_lock);
	}

	return 0;
}

static int max31760_resume(struct device *dev)
{
	struct max31760_data *pdata = dev_get_drvdata(dev);

	dev_dbg(dev, "enter resume now\n");
	if (pdata) {
		atomic_set(&pdata->in_suspend, 0);
		mutex_lock(&pdata->update_lock);
		max31760_enable_gpio(pdata, 1);
		max31760_write_byte(pdata, MAX31760_CTRL_REG1, 0x19);
		max31760_write_byte(pdata, MAX31760_CTRL_REG2, 0x11);
		max31760_write_byte(pdata, MAX31760_CTRL_REG3, 0x31);
		pwm_enable(pdata->pwm);
		pdata->cur_state = FAN_SPEED_LEVEL0;
		max31760_set_cur_state_common(pdata, pdata->cur_state);
		mutex_unlock(&pdata->update_lock);
	}

	return 0;
}

static const struct of_device_id max31760_id_table[] = {
	{ .compatible = "maxim,max31760",},
	{ },
};

static const struct i2c_device_id max31760_i2c_table[] = {
	{ "max31760", 0 },
	{ },
};

static SIMPLE_DEV_PM_OPS(max31760_pm_ops, max31760_suspend, max31760_resume);

static struct i2c_driver max31760_i2c_driver = {
	.probe = max31760_probe,
	.remove = max31760_remove,
	.shutdown = max31760_shutdown,
	.driver = {
		.name = "max31760",
		.of_match_table = max31760_id_table,
		.pm = &max31760_pm_ops,
	},
	.id_table = max31760_i2c_table,
};

module_i2c_driver(max31760_i2c_driver);
MODULE_DEVICE_TABLE(i2c, max31760_i2c_table);
MODULE_DESCRIPTION("Maxim 31760 Fan Controller");
MODULE_LICENSE("GPL v2");
