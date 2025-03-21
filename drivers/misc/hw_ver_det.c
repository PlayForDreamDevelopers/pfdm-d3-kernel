#include <linux/module.h>
#include <linux/types.h>
#include <linux/of.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/proc_fs.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <linux/platform_device.h>
#include "linux/hw_ver_det.h"

static struct class *hw_ver_class = NULL;
static struct device *hw_ver_dev = NULL;
static int hw_version = -1;

static ssize_t hw_version_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int rc = 0;
	switch(hw_version) {
		case HW_VER_DVT1:
			rc = sprintf(buf, "HW_VER_DVT1\n");
		break;
		case HW_VER_DVT2:
			rc = sprintf(buf, "HW_VER_DVT2\n");
		break;
		case HW_VER_PVT1:
			rc = sprintf(buf, "HW_VER_PVT1\n");
		break;
		default:
			rc = sprintf(buf, "HW_VER_ERR\n");
		break;
	}
	return rc;
}

static struct device_attribute hw_ver = {
	.attr = {
		.name = "hw_ver",
		.mode = S_IRUSR|S_IRGRP|S_IROTH,//Read only
	},
	.show = hw_version_show,
	.store = NULL,
};

static int hw_ver_det_probe(struct platform_device *pdev)
{
	int ret = 0;
	//board id detect
	hw_ver_class = class_create(THIS_MODULE, "hardware_version");
	if(IS_ERR(hw_ver_class))
	{
		ret = PTR_ERR(hw_ver_class);
		printk(KERN_ERR "Failed to create class.\n");
		return ret;
	}
	hw_ver_dev = device_create(hw_ver_class, NULL, 0, NULL, "hardware_version");
	if (IS_ERR(hw_ver_dev))
	{
		ret = PTR_ERR(hw_ver_class);
		printk(KERN_ERR "Failed to create device(hw_ver_dev)!\n");
		return ret;
	}
	ret = device_create_file(hw_ver_dev, &hw_ver);
	if (ret < 0) return ret;

	return ret;
}

static int hw_ver_det_remove(struct platform_device *pdev)
{
	device_destroy(hw_ver_class, 0);
	class_destroy(hw_ver_class);
	device_remove_file(hw_ver_dev, &hw_ver);
	return 0;
}
static struct of_device_id hw_ver_det_dt_match[] = {
	{ .compatible = "playfordream,hw_ver_det",},
	{ },
};
MODULE_DEVICE_TABLE(of, hw_ver_det_dt_match);

static struct platform_driver hw_ver_det_driver = {
	.driver = {
		.name     = "hw_ver_det",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(hw_ver_det_dt_match),
	},
	.probe        = hw_ver_det_probe,
	.remove = hw_ver_det_remove,
};

static int __init hw_ver_det_init(void)
{
	return platform_driver_register(&hw_ver_det_driver);
}

static void __exit hw_ver_det_exit(void)
{
	platform_driver_unregister(&hw_ver_det_driver);
}

//export interface
int get_board_version(void)
{
    return hw_version;
}

int __init board_hw_ver_init(char *str)
{
	int ret;
	ret = kstrtoint(str, 10, &hw_version);
	if (ret != 0) {
		printk(KERN_ERR "Conversion failed\n");
		hw_version = -1;
    }
	return 0;
}
__setup("androidboot.hardwareid=", board_hw_ver_init);

module_init(hw_ver_det_init);
module_exit(hw_ver_det_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("hw_ver_det driver");