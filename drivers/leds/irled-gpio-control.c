#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/leds.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/types.h>
#include <linux/regmap.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>
#include <linux/delay.h>
#include <linux/of_gpio.h>

#define FLASH_SOC_MAX_REGULATOR       5

struct platform_irleds_dev {
	struct device       *dev;
	int    				status;
};

struct  power_info
{
	uint32_t                        num_rgltr;
	struct device                  	*dev;
	const char                     	*rgltr_name[FLASH_SOC_MAX_REGULATOR];
	uint32_t                        rgltr_min_volt[FLASH_SOC_MAX_REGULATOR];
	uint32_t                        rgltr_max_volt[FLASH_SOC_MAX_REGULATOR];
	uint32_t                        rgltr_op_mode[FLASH_SOC_MAX_REGULATOR];
	struct regulator               	*rgltr[FLASH_SOC_MAX_REGULATOR];
	struct gpio 					*flash_gpio_req_tbl;
	uint8_t 						flash_gpio_req_tbl_size;
};

struct  power_info *pam2841_power_info = NULL;

struct platform_irleds_dev *irleds_dev;

static int pam2841_regulator_enable(struct regulator *rgltr,
	const char *rgltr_name,
	uint32_t rgltr_min_volt,
	uint32_t rgltr_max_volt,
	uint32_t rgltr_op_mode)
{
	int32_t rc = 0;

	if (!rgltr) {
		printk(KERN_ERR "Invalid NULL parameter\n");
		return -EINVAL;
	}

	if (regulator_count_voltages(rgltr) > 0) {
		printk(KERN_INFO "voltage min=%d, max=%d\n",
			rgltr_min_volt, rgltr_max_volt);

		rc = regulator_set_voltage(
			rgltr, rgltr_min_volt, rgltr_max_volt);
		if (rc) {
			printk(KERN_ERR "%s set voltage failed\n", rgltr_name);
			return rc;
		}

		rc = regulator_set_load(rgltr, rgltr_op_mode);
		if (rc) {
			printk(KERN_ERR "%s set optimum mode failed\n",
				rgltr_name);
			return rc;
		}
	}

	rc = regulator_enable(rgltr);
	if (rc) {
		printk(KERN_ERR  "%s regulator_enable failed\n", rgltr_name);
		return rc;
	}

	return rc;
}

int pam2841_regulator_disable(struct regulator *rgltr,
	const char *rgltr_name, uint32_t rgltr_min_volt,
	uint32_t rgltr_max_volt, uint32_t rgltr_op_mode)
{
	int32_t rc = 0;

	if (!rgltr) {
		printk(KERN_ERR "Invalid NULL parameter");
		return -EINVAL;
	}

	rc = regulator_disable(rgltr);
	if (rc) {
		printk(KERN_ERR "%s regulator disable failed", rgltr_name);
		return rc;
	}

	if (regulator_count_voltages(rgltr) > 0) {
		regulator_set_load(rgltr, 0);
		regulator_set_voltage(rgltr, 0, rgltr_max_volt);
	}

	return rc;
}

static int pam2841_get_dt_gpio_req_tbl(struct device_node *of_node,
	struct power_info *gconf, int *gpio_array,
	uint16_t gpio_array_size)
{
	int32_t rc = 0, i = 0;
	uint32_t count = 0;
	uint32_t *val_array = NULL;

	if (!of_get_property(of_node, "gpio-req-tbl-num", &count))
		return 0;

	count /= sizeof(uint32_t);
	if (!count) {
		dev_err(gconf->dev, "gpio-req-tbl-num 0\n");
		return 0;
	}

	val_array = kcalloc(count, sizeof(uint32_t), GFP_KERNEL);
	if (!val_array)
		return -ENOMEM;

	gconf->flash_gpio_req_tbl = kcalloc(count, sizeof(struct gpio),
		GFP_KERNEL);
	if (!gconf->flash_gpio_req_tbl) {
		rc = -ENOMEM;
		goto free_val_array;
	}
	gconf->flash_gpio_req_tbl_size = count;

	rc = of_property_read_u32_array(of_node, "gpio-req-tbl-num",
		val_array, count);
	if (rc) {
		dev_err(gconf->dev, "failed in reading gpio-req-tbl-num, rc = %d\n",
			rc);
		goto free_gpio_req_tbl;
	}

	for (i = 0; i < count; i++) {
		if (val_array[i] >= gpio_array_size) {
			dev_err(gconf->dev, "gpio req tbl index %d invalid\n",
				val_array[i]);
			goto free_gpio_req_tbl;
		}
		gconf->flash_gpio_req_tbl[i].gpio = gpio_array[val_array[i]];
		dev_info(gconf->dev, "flash_gpio_req_tbl[%d].gpio = %d\n", i,
			gconf->flash_gpio_req_tbl[i].gpio);
	}

	rc = of_property_read_u32_array(of_node, "gpio-req-tbl-flags",
		val_array, count);
	if (rc) {
		dev_err(gconf->dev, "Failed in gpio-req-tbl-flags, rc %d\n", rc);
		goto free_gpio_req_tbl;
	}

	for (i = 0; i < count; i++) {
		gconf->flash_gpio_req_tbl[i].flags = val_array[i];
		dev_info(gconf->dev, "flash_gpio_req_tbl[%d].flags = %ld\n", i,
			gconf->flash_gpio_req_tbl[i].flags);
	}

	for (i = 0; i < count; i++) {
		rc = of_property_read_string_index(of_node,
			"gpio-req-tbl-label", i,
			&gconf->flash_gpio_req_tbl[i].label);
		if (rc) {
			dev_err(gconf->dev, "Failed rc %d\n", rc);
			goto free_gpio_req_tbl;
		}
		dev_info(gconf->dev, "flash_gpio_req_tbl[%d].label = %s\n", i,
			gconf->flash_gpio_req_tbl[i].label);
	}

	kfree(val_array);

	return rc;

free_gpio_req_tbl:
	kfree(gconf->flash_gpio_req_tbl);
free_val_array:
	kfree(val_array);
	gconf->flash_gpio_req_tbl_size = 0;

	return rc;
}

static int get_power_info(struct device *dev)
{
	int i = 0;
	int count = 0;
	int     rc = 0;
	int16_t gpio_array_size = 0;
	int *gpio_array = NULL;
	struct device_node  *of_node = NULL;

	if (!dev) {
		dev_err(dev, "Dev is NULL\n");
		return -EINVAL;
	}

	pam2841_power_info = kcalloc(sizeof(struct power_info), sizeof(uint16_t), GFP_KERNEL);
	if (pam2841_power_info == NULL) {
		dev_err(dev, "kcalloc power_info failed\n");
		return -ENOMEM;
	}

	pam2841_power_info->num_rgltr = 0;
	pam2841_power_info->dev = dev;
	of_node = pam2841_power_info->dev->of_node;

	// get regulator info
	count = of_property_count_strings(of_node, "regulator-names");
	if (count != -EINVAL) {
		if (count <= 0) {
			dev_err(pam2841_power_info->dev, "no regulators found\n");
			count = 0;
			rc = -EINVAL;
			goto free_pam2841_power_info;
		}
		pam2841_power_info->num_rgltr = count;

	} else {
		dev_err(pam2841_power_info->dev, "No regulators node found\n");
		rc = -EINVAL;
		goto free_pam2841_power_info;
	}

	for (i = 0; i < pam2841_power_info->num_rgltr; i++) {
		rc = of_property_read_string_index(of_node,
			"regulator-names", i, &pam2841_power_info->rgltr_name[i]);
		dev_info(pam2841_power_info->dev, "rgltr_name[%d] = %s\n",
			i, pam2841_power_info->rgltr_name[i]);
		if (rc) {
			dev_err(pam2841_power_info->dev, "no regulator resource at cnt=%d\n", i);
			goto free_pam2841_power_info;
		}
	}

	rc = of_property_read_u32_array(of_node, "rgltr-min-voltage",
		pam2841_power_info->rgltr_min_volt, pam2841_power_info->num_rgltr);
	if (rc) {
		dev_err(pam2841_power_info->dev, "No minimum volatage value found, rc=%d\n", rc);
		goto free_pam2841_power_info;
	}

	rc = of_property_read_u32_array(of_node, "rgltr-max-voltage",
		pam2841_power_info->rgltr_max_volt, pam2841_power_info->num_rgltr);
	if (rc) {
		dev_err(pam2841_power_info->dev, "No maximum volatage value found, rc=%d\n", rc);
		goto free_pam2841_power_info;
	}

	rc = of_property_read_u32_array(of_node, "rgltr-load-current",
		pam2841_power_info->rgltr_op_mode, pam2841_power_info->num_rgltr);
	if (rc) {
		dev_err(pam2841_power_info->dev, "No Load curent found rc=%d", rc);
		goto free_pam2841_power_info;
	}

	// get gpio info
	gpio_array_size = of_gpio_count(of_node);

	if (gpio_array_size <= 0)
		return 0;

	dev_info(pam2841_power_info->dev, "gpio count %d\n", gpio_array_size);

	gpio_array = kcalloc(gpio_array_size, sizeof(uint16_t), GFP_KERNEL);
	if (!gpio_array) {
		dev_err(pam2841_power_info->dev, "kcalloc for gpio_array failed\n");
		goto free_pam2841_power_info;
	}

	for (i = 0; i < gpio_array_size; i++) {
		gpio_array[i] = of_get_gpio(of_node, i);
		dev_info(pam2841_power_info->dev, "gpio_array[%d] = %d\n", i, gpio_array[i]);
	}

	rc = pam2841_get_dt_gpio_req_tbl(of_node, pam2841_power_info, gpio_array, gpio_array_size);
	if (rc)	{
		dev_err(pam2841_power_info->dev, "pam2841_get_dt_gpio_req_tbl failed rc=%d\n", rc);
        goto free_gpio_array;
	}
	kfree(gpio_array);

	return 0;
free_gpio_array:
	kfree(gpio_array);
free_pam2841_power_info:
	kfree(pam2841_power_info);
	return rc;
};

static int pam2841_power_up()
{
	size_t i = 0;
	int rc = 0;

	if(pam2841_power_info == NULL) {
		dev_info(pam2841_power_info->dev, "pam2841_power_info is NULL\n");
		return 0;
	}

	for (i = 0; i < pam2841_power_info->num_rgltr; i++) {
		pam2841_power_info->rgltr[i] = regulator_get(
				pam2841_power_info->dev,
				pam2841_power_info->rgltr_name[i]);
		rc =  pam2841_regulator_enable(
			pam2841_power_info->rgltr[i],
			pam2841_power_info->rgltr_name[i],
			pam2841_power_info->rgltr_min_volt[i],
			pam2841_power_info->rgltr_max_volt[i],
			pam2841_power_info->rgltr_op_mode[i]);
		if (rc) {
			dev_err(pam2841_power_info->dev,
				"Reg enable failed\n");
			// goto power_up_failed;
		}
	}

	dev_info(pam2841_power_info->dev, "flash_gpio_req_tbl_size %d\n", pam2841_power_info->flash_gpio_req_tbl_size);
	for (i = 0; i < pam2841_power_info->flash_gpio_req_tbl_size; i++) {
		rc = gpio_request_one(pam2841_power_info->flash_gpio_req_tbl[i].gpio, pam2841_power_info->flash_gpio_req_tbl[i].flags, pam2841_power_info->flash_gpio_req_tbl[i].label);
		if (rc < 0) {
			dev_err(pam2841_power_info->dev, "gpio_request_one failed\n");
			return rc;
		}
		rc = gpio_direction_output(pam2841_power_info->flash_gpio_req_tbl[i].gpio, 0);
		if (rc < 0) {
			dev_err(pam2841_power_info->dev, "gpio_direction_output failed\n");
			return rc;
		}
		gpio_set_value_cansleep(pam2841_power_info->flash_gpio_req_tbl[i].gpio, 1);
		rc = gpio_get_value(pam2841_power_info->flash_gpio_req_tbl[i].gpio);
		dev_info(pam2841_power_info->dev, "gpio_get_value %d\n", rc);
	}

	msleep(2);

	return 0;
}

static int pam2841_power_down()
{
	size_t i = 0;
	int rc = 0;

	if (pam2841_power_info == NULL) {
		return 0;
	}

	for (i = 0; i < pam2841_power_info->num_rgltr; i++) {
		rc =  pam2841_regulator_disable(
			pam2841_power_info->rgltr[i],
			pam2841_power_info->rgltr_name[i],
			pam2841_power_info->rgltr_min_volt[i],
			pam2841_power_info->rgltr_max_volt[i],
			pam2841_power_info->rgltr_op_mode[i]);
		if (rc) {
			dev_err(pam2841_power_info->dev,
				"Reg disable failed\n");
		}
	}

	dev_info(pam2841_power_info->dev, "flash_gpio_req_tbl_size %d\n", pam2841_power_info->flash_gpio_req_tbl_size);
	for (i = 0; i < pam2841_power_info->flash_gpio_req_tbl_size; i++) {
		gpio_set_value_cansleep(pam2841_power_info->flash_gpio_req_tbl[i].gpio, 0);
		rc = gpio_get_value(pam2841_power_info->flash_gpio_req_tbl[i].gpio);
		dev_info(pam2841_power_info->dev, "gpio_get_value %d\n", rc);
		gpio_free(pam2841_power_info->flash_gpio_req_tbl[i].gpio);
	}

	msleep(2);

	return 0;
}

static ssize_t pmxr2230_irleds_enable_store(struct device *dev,
                                      struct device_attribute *attr,
                                      const char *buf, size_t size)
{
	ssize_t rc;

	unsigned int state;
	dev_info(dev, "%s: irled debug: status=%d\n",__func__, irleds_dev->status);

	rc = kstrtouint(buf, 10, &state);
	if (state == 1)
	{
		if (irleds_dev->status != 1)
		{
			pam2841_power_up();
			irleds_dev->status = state;
		}
		dev_info(dev, "%s: irleds_ctrl_gpio set value%d\n",__func__, state);
	}
	else if(state == 0)
	{
		if (irleds_dev->status != 0)
		{
			pam2841_power_down();
			irleds_dev->status = state;
		}
		dev_info(dev, "%s: irleds_ctrl_gpio set value%d\n",__func__, state);
	}
	else
	{
		dev_err(dev, "%s: Value: %d error, only 0 or 1 is abled to set for irleds!\n",__func__, state);
	}
	return size;
}

static ssize_t pmxr2230_irleds_enable_show(struct device *dev,
                                 struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", irleds_dev->status);
}

static DEVICE_ATTR(irleds_enable, S_IRUGO | S_IWUSR, pmxr2230_irleds_enable_show, pmxr2230_irleds_enable_store);

static int irled_probe(struct platform_device *pdev)
{
	int rc = 0;

	irleds_dev = devm_kzalloc(&pdev->dev, sizeof(*irleds_dev), GFP_KERNEL);
	if (!irleds_dev)
		return -ENOMEM;

	irleds_dev->status = 0;

	get_power_info(&pdev->dev);

	rc = device_create_file(&pdev->dev, &dev_attr_irleds_enable);
	if (rc < 0) {
		dev_err(&pdev->dev, "failed to create irled enable sysfs node. rc:%d\n", rc);
		return rc;
	}

	return rc;
}

static int irled_remove(struct platform_device *pdev)
{
	if (irleds_dev->status == 1)
	{
		pam2841_power_down();
	}

	if (pam2841_power_info != NULL && pam2841_power_info->flash_gpio_req_tbl != NULL) {
		kfree(pam2841_power_info->flash_gpio_req_tbl);
		pam2841_power_info->flash_gpio_req_tbl = NULL;
	}

	if (pam2841_power_info != NULL) {
		kfree(pam2841_power_info);
		pam2841_power_info = NULL;
	}

	return 0;
}

const struct of_device_id irleds_match_table[] = {
	{
		.compatible = "diodes,pam2841",
	},
	{},
};


MODULE_DEVICE_TABLE(of, irleds_match_table);

static struct platform_driver logo_led_driver = {
	.driver		= {
		.name = "pam2841-driver",
		.of_match_table = irleds_match_table,
	},
	.probe		= irled_probe,
	.remove		= irled_remove,
};

module_platform_driver(logo_led_driver);

MODULE_DESCRIPTION("PMXR2230 IR LEDS driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:irleds");

