#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/device.h>
#include "crown_device.h"


#define CLASS_NAME "crown"
#define DEVICE_NAME "crown_device"

#define CROWN_TAG "[" DEVICE_NAME "]"

static struct class *crown_class = NULL;

static int __init crown_device_init(void) {
    crown_class = class_create(THIS_MODULE, CLASS_NAME);
    if (IS_ERR(crown_class)) {
        pr_err(CROWN_TAG"Failed to create class\n");
        return PTR_ERR(crown_class);
    }
    pr_info(CROWN_TAG"Crown driver initialized\n");
    return 0;
}

static void __exit crown_device_exit(void) {
    class_destroy(crown_class);
    pr_info(CROWN_TAG"Crown driver exited\n");
}

int crown_device_register(struct crown_dev* crown_dev,void *priv_data) {
    int ret;

    if (!crown_class) {
        pr_err(CROWN_TAG"Crown class not created\n");
        return -ENODEV;
    }

    if (!crown_dev){
        pr_err(CROWN_TAG"Crown dev error\n");
        return -ENODEV;
    }

    crown_dev->dev = device_create(crown_class, NULL, 0, priv_data, crown_dev->name);
    if (IS_ERR(crown_dev->dev)) {
        pr_err(CROWN_TAG"%s Failed to create device %s\n",__func__,crown_dev->name);
        return PTR_ERR(crown_dev->dev);
    }
    dev_set_drvdata(crown_dev->dev, priv_data);

    if (crown_dev->dev_attr_group){
        ret = sysfs_create_group(&crown_dev->dev->kobj, crown_dev->dev_attr_group);
        if (ret) {
            pr_err(CROWN_TAG"%s Failed to create attribute group for device %s\n",__func__, crown_dev->name);
            device_unregister(crown_dev->dev);
            return ret;
        }
    }

    pr_info(CROWN_TAG"Device %s created\n", crown_dev->name);
    return 0;
}

EXPORT_SYMBOL(crown_device_register);

module_init(crown_device_init);
module_exit(crown_device_exit);

MODULE_AUTHOR("yihui.luo@thundercomm.com");
MODULE_DESCRIPTION("crown sensor device driver");
MODULE_LICENSE("GPL v2");
