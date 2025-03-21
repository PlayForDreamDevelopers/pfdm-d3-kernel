/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _AW8646_H_
#define _AW8646_H_

#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/hrtimer.h>
#include <linux/platform_device.h>
#include <linux/pinctrl/consumer.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>

#define AW8646_MAX_FREQUENCY			250000
#define AW8646_DEFAULT_FREQUENCY		2000

struct fault_pinctrl_info {
	struct pinctrl *pinctrl;
	struct pinctrl_state *fault;
};


struct aw8646 {
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

	int vdd5v_en_pin_1;   /*vdd 5v supply*/
	int nen_pin_1;		/* driver enable control input pin */
	int dir_pin_1;		/* direction input pin */
	int step_pin_1;		/* step input pin */
	int nsleep_pin_1;		/* sleep mode input pin */
	int nfault_pin_1;

	struct fault_pinctrl_info fault_pinctrl;
	struct gpio_desc *fault_irq_gpio;
	int irq;
	int dir_pin_state;
};

#define DRIVER_NAME				"aw8646_step"

#define AW_LOGI(format, ...) \
	pr_info("[%s][%04d]%s: " format "\n", DRIVER_NAME, __LINE__, __func__, ##__VA_ARGS__)

#define AW_LOGE(format, ...) \
	pr_err("[%s][%04d]%s: " format "\n", DRIVER_NAME, __LINE__, __func__, ##__VA_ARGS__)

enum aw_gpio_state {
	AW_GPIO_LOW = 0,
	AW_GPIO_HIGH = 1,
};

#endif
