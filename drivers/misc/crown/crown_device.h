#ifndef _CROWN_CORE_H_
#define _CROWN_CORE_H_

struct crown_dev{
    const char *name;
    struct device *dev;
    struct attribute_group *dev_attr_group;
};

int crown_device_register(struct crown_dev* crown_dev,void *priv_data);
#endif