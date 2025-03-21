/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _aw8646_side_H_
#define _aw8646_side_H_

#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/hrtimer.h>
#include <linux/platform_device.h>

#define aw8646_side_MAX_FREQUENCY			250000
#define aw8646_side_DEFAULT_FREQUENCY		2000

struct aw8646_side {
	struct mutex lock;
	struct device *dev;
	struct hrtimer hrtimer;
	struct platform_device *pdev;

	ktime_t kinterval;
	uint32_t step_num;
	uint32_t timer_cnt;
	uint32_t half_period;
	uint32_t step_frequency;
	uint32_t stop_motor;

	int vdd5v_en_pin_2;   /*vdd 5v supply*/
	int nen_pin_2;		/* driver enable control input pin */
	int dir_pin_2;		/* direction input pin */
	int step_pin_2;		/* step input pin */
	int nsleep_pin_2;		/* sleep mode input pin */

	int dir_pin_state;
};

#define DRIVER_NAME				"aw8646_side"

#define AW_LOGI(format, ...) \
	pr_err("[%s][%04d]%s: " format "\n", DRIVER_NAME, __LINE__, __func__, ##__VA_ARGS__)

#define AW_LOGE(format, ...) \
	pr_err("[%s][%04d]%s: " format "\n", DRIVER_NAME, __LINE__, __func__, ##__VA_ARGS__)

enum aw_gpio_state {
	AW_GPIO_LOW = 0,
	AW_GPIO_HIGH = 1,
};

#endif
