// SPDX-License-Identifier: GPL-2.0
/*
 * File: aw8646_side.c
 *
 * Copyright (c) 2022 AWINIC Technology CO., LTD
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 */
#include "aw8646_side.h"

#define aw8646_side_STEP_DRIVER_VERSION	"v0.1.0"

static int aw8646_side_parse_dts(struct aw8646_side *aw8646_side)
{
	aw8646_side->vdd5v_en_pin_2 = of_get_named_gpio(aw8646_side->dev->of_node, "vdd5v-en-gpio-2", 0);
	if (!gpio_is_valid(aw8646_side->vdd5v_en_pin_2))
		return -EINVAL;

	aw8646_side->nen_pin_2 = of_get_named_gpio(aw8646_side->dev->of_node, "nen-gpio-2", 0);
	if (!gpio_is_valid(aw8646_side->nen_pin_2))
		return -EINVAL;

	aw8646_side->dir_pin_2 = of_get_named_gpio(aw8646_side->dev->of_node, "dir-gpio-2", 0);
	if (!gpio_is_valid(aw8646_side->dir_pin_2))
		return -EINVAL;

	aw8646_side->step_pin_2 = of_get_named_gpio(aw8646_side->dev->of_node, "step-gpio-2", 0);
	if (!gpio_is_valid(aw8646_side->step_pin_2))
		return -EINVAL;

	aw8646_side->nsleep_pin_2 = of_get_named_gpio(aw8646_side->dev->of_node, "nsleep-gpio-2", 0);
	if (!gpio_is_valid(aw8646_side->nsleep_pin_2))
		return -EINVAL;

	AW_LOGI("aw8646_side->vdd5v_en_pin_2 %d", aw8646_side->vdd5v_en_pin_2);
	AW_LOGI("aw8646_side->nen_pin_2 %d", aw8646_side->nen_pin_2);
	AW_LOGI("aw8646_side->dir_pin_2 %d", aw8646_side->dir_pin_2);
	AW_LOGI("aw8646_side->step_pin_2 %d", aw8646_side->step_pin_2);
	AW_LOGI("aw8646_side->nsleep_pin_2 %d", aw8646_side->nsleep_pin_2);

	return 0;
}

static int aw8646_side_gpio_request(struct aw8646_side *aw8646_side)
{
	int ret;
	ret = devm_gpio_request_one(aw8646_side->dev, aw8646_side->vdd5v_en_pin_2, GPIOF_OUT_INIT_HIGH, "aw8646_side_vdd5v_en_2");
	if (ret) {
		AW_LOGE("failed to request vdd5v-en pin_2");
		return ret;
    }

	ret = devm_gpio_request_one(aw8646_side->dev, aw8646_side->nen_pin_2, GPIOF_OUT_INIT_HIGH, "aw8646_side_nen_2");
	if (ret) {
		AW_LOGE("failed to request nen pin_2");
		return ret;
	}

	ret = devm_gpio_request_one(aw8646_side->dev, aw8646_side->dir_pin_2, GPIOF_OUT_INIT_LOW, "aw8646_side_dir_2");
	if (ret) {
		AW_LOGE("failed to request dir pin_2");
		return ret;
	}

	ret = devm_gpio_request_one(aw8646_side->dev, aw8646_side->step_pin_2, GPIOF_OUT_INIT_LOW, "aw8646_side_step_2");
	if (ret) {
		AW_LOGE("failed to request step pin_2");
		return ret;
	}

	ret = devm_gpio_request_one(aw8646_side->dev, aw8646_side->nsleep_pin_2, GPIOF_OUT_INIT_HIGH, "aw8646_side_nsleep_2");
	if (ret) {
		AW_LOGE("failed to request nsleep pin_2");
		return ret;
	}

	return ret;
}

static enum hrtimer_restart aw8646_side_step_timer_func(struct hrtimer *hrtimer)
{
	int ret = 0;
	struct aw8646_side *aw8646_side = container_of(hrtimer, struct aw8646_side, hrtimer);

	if (aw8646_side->timer_cnt < aw8646_side->step_num) {
		aw8646_side->timer_cnt++;
		gpio_set_value_cansleep(aw8646_side->step_pin_2, AW_GPIO_HIGH);
		udelay(aw8646_side->half_period);
		gpio_set_value_cansleep(aw8646_side->step_pin_2, AW_GPIO_LOW);
		hrtimer_forward_now(&aw8646_side->hrtimer, aw8646_side->kinterval);
		ret = HRTIMER_RESTART;
	} else {
		gpio_set_value_cansleep(aw8646_side->vdd5v_en_pin_2, AW_GPIO_HIGH);
		gpio_set_value_cansleep(aw8646_side->nen_pin_2, AW_GPIO_HIGH);
		AW_LOGI("play end, %u steps completed", aw8646_side->timer_cnt);
		ret = HRTIMER_NORESTART;
	}

	return ret;
}

static ssize_t sleep_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int val = 0;
	ssize_t len = 0;
	struct aw8646_side *aw8646_side = dev_get_drvdata(dev);

	val = gpio_get_value_cansleep(aw8646_side->nsleep_pin_2);
	len += snprintf(buf + len, PAGE_SIZE - len, "%d\n", !val);

	return len;
}

static ssize_t sleep_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int val = 0;
	struct aw8646_side *aw8646_side = dev_get_drvdata(dev);

	if (kstrtoint(buf, 0, &val))
		return -EINVAL;

	mutex_lock(&aw8646_side->lock);
	if (val > 0) {
		hrtimer_cancel(&aw8646_side->hrtimer);
		gpio_set_value_cansleep(aw8646_side->nsleep_pin_2, AW_GPIO_LOW);
		gpio_set_value_cansleep(aw8646_side->vdd5v_en_pin_2, AW_GPIO_HIGH);
		gpio_set_value_cansleep(aw8646_side->nen_pin_2, AW_GPIO_HIGH);

		AW_LOGI("set the chip to sleep mode");
	} else {
		gpio_set_value_cansleep(aw8646_side->nsleep_pin_2, AW_GPIO_HIGH);

		AW_LOGI("set the chip to no sleep mode");
	}
	mutex_unlock(&aw8646_side->lock);

	return count;
}
static DEVICE_ATTR_RW(sleep);

static ssize_t direction_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int val = 0;
	ssize_t len = 0;
	struct aw8646_side *aw8646_side = dev_get_drvdata(dev);

	val = gpio_get_value_cansleep(aw8646_side->dir_pin_2);
	len += snprintf(buf + len, PAGE_SIZE - len, "%d\n", val);

	return len;
}

static ssize_t direction_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int val = 0;
	struct aw8646_side *aw8646_side = dev_get_drvdata(dev);

	if (kstrtoint(buf, 0, &val))
		return -EINVAL;

	mutex_lock(&aw8646_side->lock);
	if (val > 0) {
		aw8646_side->dir_pin_state = AW_GPIO_HIGH;
		AW_LOGI("set the dir pin of the chip to high");
	} else {
		aw8646_side->dir_pin_state = AW_GPIO_LOW;
		AW_LOGI("set the dir pin of the chip to low");
	}
	mutex_unlock(&aw8646_side->lock);

	return count;
}
static DEVICE_ATTR_RW(direction);

static ssize_t step_frequency_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	struct aw8646_side *aw8646_side = dev_get_drvdata(dev);

	len += snprintf(buf + len, PAGE_SIZE - len, "%u\n", aw8646_side->step_frequency);

	return len;
}

static ssize_t step_frequency_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	uint32_t val = 0;
	struct aw8646_side *aw8646_side = dev_get_drvdata(dev);

	if (kstrtouint(buf, 0, &val))
		return -EINVAL;

	mutex_lock(&aw8646_side->lock);
	if (val > 0 && val <= aw8646_side_MAX_FREQUENCY) {
		aw8646_side->step_frequency = val;
		AW_LOGI("set the step frequency to %u", aw8646_side->step_frequency);
	} else {
		AW_LOGE("wrong frequency parameter: %u", val);
	}
	mutex_unlock(&aw8646_side->lock);

	return count;
}
static DEVICE_ATTR_RW(step_frequency);

static ssize_t steps_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	struct aw8646_side *aw8646_side = dev_get_drvdata(dev);

	len += snprintf(buf + len, PAGE_SIZE - len, "%u\n", aw8646_side->timer_cnt);

	return len;
}
static DEVICE_ATTR_RO(steps);

static ssize_t activate_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	struct aw8646_side *aw8646_side = dev_get_drvdata(dev);

	len += snprintf(buf + len, PAGE_SIZE - len, "%u\n", aw8646_side->step_num);

	return len;
}

static ssize_t activate_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	uint32_t steps = 0;
	struct aw8646_side *aw8646_side = dev_get_drvdata(dev);

	if (kstrtouint(buf, 0, &steps))
		return -EINVAL;

	AW_LOGI("enter, input steps = %u", steps);

	mutex_lock(&aw8646_side->lock);
	if (steps > 0) {
		/* Stop the currently playing */
		hrtimer_cancel(&aw8646_side->hrtimer);

		gpio_set_value_cansleep(aw8646_side->vdd5v_en_pin_2, AW_GPIO_HIGH);
		gpio_set_value_cansleep(aw8646_side->nen_pin_2, AW_GPIO_HIGH);

		AW_LOGI("stop play, %u steps completed", aw8646_side->timer_cnt);
		aw8646_side->timer_cnt = 0;

		aw8646_side->step_num = steps;
		aw8646_side->half_period = 500000 / aw8646_side->step_frequency;
		aw8646_side->kinterval = ktime_set(0, (aw8646_side->half_period * 1000));

		AW_LOGI("start play, steps = %u, dir = %d", aw8646_side->step_num, aw8646_side->dir_pin_state);
		gpio_set_value_cansleep(aw8646_side->dir_pin_2, aw8646_side->dir_pin_state);
		gpio_set_value_cansleep(aw8646_side->vdd5v_en_pin_2, AW_GPIO_HIGH);
		gpio_set_value_cansleep(aw8646_side->nen_pin_2, AW_GPIO_LOW);

		hrtimer_start(&aw8646_side->hrtimer, ktime_set(0, 1000), HRTIMER_MODE_REL);
	} else {
		hrtimer_cancel(&aw8646_side->hrtimer);

		gpio_set_value_cansleep(aw8646_side->vdd5v_en_pin_2, AW_GPIO_HIGH);
		gpio_set_value_cansleep(aw8646_side->nen_pin_2, AW_GPIO_HIGH);

		AW_LOGI("stop play, %u steps completed", aw8646_side->timer_cnt);
	}
	mutex_unlock(&aw8646_side->lock);

	return count;
}
static DEVICE_ATTR_RW(activate);

static ssize_t stop_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	struct aw8646_side *aw8646_side = dev_get_drvdata(dev);

	len += snprintf(buf + len, PAGE_SIZE - len, "%u\n", aw8646_side->stop_motor);

	return len;
}

static ssize_t stop_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int val = 0;
	struct aw8646_side *aw8646_side = dev_get_drvdata(dev);

	if (kstrtoint(buf, 0, &val))
		return -EINVAL;

	mutex_lock(&aw8646_side->lock);
	if (val > 0) {
		aw8646_side->stop_motor = val;
		/* Stop the currently playing */
		hrtimer_cancel(&aw8646_side->hrtimer);

		gpio_set_value_cansleep(aw8646_side->vdd5v_en_pin_2, AW_GPIO_HIGH);
		gpio_set_value_cansleep(aw8646_side->nen_pin_2, AW_GPIO_HIGH);

		AW_LOGI("stop_store stop play, %u steps completed", aw8646_side->timer_cnt);
		aw8646_side->timer_cnt = 0;
	} else {
		gpio_set_value_cansleep(aw8646_side->vdd5v_en_pin_2, AW_GPIO_HIGH);
		gpio_set_value_cansleep(aw8646_side->nen_pin_2, AW_GPIO_LOW);
		AW_LOGI("stop_store set wrong value");
	}
	mutex_unlock(&aw8646_side->lock);

	return count;
}
static DEVICE_ATTR_RW(stop);
static struct attribute *aw8646_side_attributes[] = {
	&dev_attr_sleep.attr,
	&dev_attr_direction.attr,
	&dev_attr_step_frequency.attr,
	&dev_attr_steps.attr,
	&dev_attr_activate.attr,
	&dev_attr_stop.attr,
	NULL,
};

struct attribute_group aw8646_side_attribute_group = {
	.name = DRIVER_NAME,
	.attrs = aw8646_side_attributes,
};

static int aw8646_side_probe(struct platform_device *pdev)
{
	struct aw8646_side *aw8646_side = NULL;

	AW_LOGI("aw8646_side step driver version %s", aw8646_side_STEP_DRIVER_VERSION);

	aw8646_side = devm_kzalloc(&pdev->dev, sizeof(struct aw8646_side), GFP_KERNEL);
	if (!aw8646_side)
		return -ENOMEM;

	aw8646_side->pdev = pdev;
	aw8646_side->dev = &pdev->dev;
	dev_set_drvdata(&pdev->dev, aw8646_side);

	if (aw8646_side_parse_dts(aw8646_side)) {
		AW_LOGE("failed to parse aw8646_side dts");
		return -ERANGE;
	}

	if (aw8646_side_gpio_request(aw8646_side)) {
		AW_LOGE("failed to request aw8646_side gpio");
		return -ERANGE;
	}

	if (sysfs_create_group(&aw8646_side->dev->kobj, &aw8646_side_attribute_group)) {
		AW_LOGE("failed to creat sysfs group");
		return -ERANGE;
	}

	mutex_init(&aw8646_side->lock);
	hrtimer_init(&aw8646_side->hrtimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	aw8646_side->hrtimer.function = aw8646_side_step_timer_func;

	aw8646_side->step_frequency = aw8646_side_DEFAULT_FREQUENCY;

	AW_LOGI("aw8646_side step driver probe successfully");

	return 0;
}

static int aw8646_side_remove(struct platform_device *pdev)
{
	struct aw8646_side *aw8646_side = dev_get_drvdata(&pdev->dev);

	AW_LOGI("aw8646_side step driver remove");

	hrtimer_cancel(&aw8646_side->hrtimer);

	gpio_set_value_cansleep(aw8646_side->nsleep_pin_2, AW_GPIO_LOW);
	gpio_set_value_cansleep(aw8646_side->vdd5v_en_pin_2, AW_GPIO_HIGH);
	gpio_set_value_cansleep(aw8646_side->nen_pin_2, AW_GPIO_HIGH);

	mutex_destroy(&aw8646_side->lock);
	sysfs_remove_group(&aw8646_side->dev->kobj, &aw8646_side_attribute_group);

	return 0;
}

static int aw8646_side_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct aw8646_side *aw8646_side = dev_get_drvdata(&pdev->dev);

	mutex_lock(&aw8646_side->lock);
	hrtimer_cancel(&aw8646_side->hrtimer);
	gpio_set_value_cansleep(aw8646_side->nsleep_pin_2, AW_GPIO_LOW);
	gpio_set_value_cansleep(aw8646_side->vdd5v_en_pin_2, AW_GPIO_HIGH);
	gpio_set_value_cansleep(aw8646_side->nen_pin_2, AW_GPIO_HIGH);

	AW_LOGI("set the chip to sleep mode");
	mutex_unlock(&aw8646_side->lock);

	return 0;
}

static int aw8646_side_resume(struct platform_device *pdev)
{
	struct aw8646_side *aw8646_side = dev_get_drvdata(&pdev->dev);

	mutex_lock(&aw8646_side->lock);
	gpio_set_value_cansleep(aw8646_side->nsleep_pin_2, AW_GPIO_HIGH);
	AW_LOGI("set the chip to no sleep mode");
	mutex_unlock(&aw8646_side->lock);

	return 0;
}

static void aw8646_side_shutdown(struct platform_device *pdev)
{
	struct aw8646_side *aw8646_side = dev_get_drvdata(&pdev->dev);

	mutex_lock(&aw8646_side->lock);
	hrtimer_cancel(&aw8646_side->hrtimer);
	gpio_set_value_cansleep(aw8646_side->nsleep_pin_2, AW_GPIO_LOW);
	gpio_set_value_cansleep(aw8646_side->vdd5v_en_pin_2, AW_GPIO_HIGH);
	gpio_set_value_cansleep(aw8646_side->nen_pin_2, AW_GPIO_HIGH);

	AW_LOGI("set the chip to sleep mode");
	mutex_unlock(&aw8646_side->lock);
}

const struct of_device_id aw8646_side_dt_match[] = {
	{.compatible = "awinic,aw8646_side_step"},
	{},
};
MODULE_DEVICE_TABLE(of, aw8646_side_dt_match);

static struct platform_driver aw8646_side_driver = {
	.driver = {
		.name	= "aw8646_side_step",
		.of_match_table	= aw8646_side_dt_match,
	},
	.probe		= aw8646_side_probe,
	.remove		= aw8646_side_remove,
	.suspend	= aw8646_side_suspend,
	.resume		= aw8646_side_resume,
	.shutdown	= aw8646_side_shutdown,
};
module_platform_driver(aw8646_side_driver);

MODULE_DESCRIPTION("aw8646_side Step Driver");
MODULE_AUTHOR("Ethan Ren <renzhiqiang@awinic.com>");
MODULE_LICENSE("GPL v2");
