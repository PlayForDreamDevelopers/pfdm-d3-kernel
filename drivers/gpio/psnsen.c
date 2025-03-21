#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/fs.h>
#include <linux/platform_device.h>
#include <linux/unistd.h>
#include <linux/pinctrl/consumer.h>

static int psensor_set(struct device *dev, int val)
{
	int ret = 0;
	struct pinctrl *pinctrl         = NULL;
	struct pinctrl_state *active    = NULL;
	struct pinctrl_state *suspend   = NULL;

	printk("psensor begin of psensor_set");
	if (dev == NULL) {
		dev_err(dev, "dev err");
		return -1;
	}

	pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR_OR_NULL(pinctrl)) {
		dev_err(dev, "pinctrl get fail");
		return PTR_ERR(pinctrl);
	}

	active = pinctrl_lookup_state(pinctrl, "p_active");
	if (IS_ERR_OR_NULL(active)) {
		dev_err(dev, "pinctrl lookup fail");
		devm_pinctrl_put(pinctrl);
		return PTR_ERR(active);
	}

	suspend = pinctrl_lookup_state(pinctrl, "p_suspend");
	if (IS_ERR_OR_NULL(suspend)) {
		dev_err(dev, "pinctrl lookup fail");
		devm_pinctrl_put(pinctrl);
		return PTR_ERR(suspend);
	}

	if (val == 0) {
		ret = pinctrl_select_state(pinctrl, suspend);
	} else {
		ret = pinctrl_select_state(pinctrl, active);
	}
	if (ret < 0)
		dev_err(dev, "pinctrl set fail");

	devm_pinctrl_put(pinctrl);
	printk("psensor end of psensor_set");
	return ret;
}

static int gpio_up_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct device *p_sensor= devm_kzalloc(&pdev->dev,
				sizeof(struct device), GFP_KERNEL);

	printk("psensor begin of gpio_up_probe");
	if (pdev == NULL) {
		dev_err(&pdev->dev, "dev err");
		return -1;
	}

	if (!p_sensor) {
		dev_err(&pdev->dev, "power p sensor alloc fail");
		return -1;
	}

	ret = psensor_set(&pdev->dev, 1);
	if (ret < 0) {
		dev_err(&pdev->dev, "psensor set fail");
		return ret;
	}
	printk("psensor end of gpio_up_probe");

	return ret;
}

static int gpio_up_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id s_gpio_up_of_match[] = {
	{
		.compatible = "p_en",
	},
	{ /* sentinel */ }
};

static struct platform_driver s_gpio_up_driver = {
	.probe  = gpio_up_probe,
	.remove = gpio_up_remove,
	.driver = {
		.name ="ps_gpio_up",
		.of_match_table = s_gpio_up_of_match,
	},
};

static int __init gpio_up_init(void)
{
	int ret = 0;
	printk("psensor gpio_up_init success!\r\n");
	platform_driver_register(&s_gpio_up_driver);
	return ret;
}

static void __exit gpio_up_exit(void)
{
	printk("psensor gpio_up_exit success!\r\n");
	platform_driver_unregister(&s_gpio_up_driver);
}

module_init(gpio_up_init);
module_exit(gpio_up_exit);
