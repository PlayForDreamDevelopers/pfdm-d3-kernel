#include <linux/kernel.h>
#include <linux/input.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/miscdevice.h>
#include <linux/regulator/consumer.h>
#include <linux/pm_runtime.h>
#include <linux/pinctrl/consumer.h>
#include <linux/types.h>
#include <linux/fs.h>
#include "crown_device.h"
#include "mt3503.h"

struct crown {
	struct mutex lock;
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct regulator *vdd;
	struct pinctrl *vld_pinctrl;
	int irq_gpio;
	unsigned int irq_gpio_flags;
	int irq;
	uint16_t chip_id;
	int enable;
	struct crown_dev crown_dev;
	//struct delayed_work open_work;

	spinlock_t irq_lock;
	struct delayed_work irq_work;
	int32_t irq_status;
    bool is_delay_work;
};

struct crown_reg{
	uint8_t reg;
	uint8_t val;
};


#define DRIVER_NAME "crown,mt3503"
#define DRV_TAG "[" DRIVER_NAME "]"
#define crown_cali_file_path "/backup/mt3503.txt"

static int control_cali_file(void);

static atomic_t sum_data = ATOMIC_INIT(0);
static atomic_t verify_sum_data = ATOMIC_INIT(0);
static atomic_t crown_sum_data = ATOMIC_INIT(0);
static int calibrating = 0;
static int target_data = 0;
static int sum_data_value = 0;
static int verify_data = 0;
static int verify_sum = 0;
//static char *read_buf = NULL;
static int read_value = 0;
//static int open_cali_count = 0;
static struct crown_reg crown_init_regs[] =
{
	{0x09,0x5a},{0x7f,0x5a},{0x0D,0x26},{0x0E,0x00},{0x0B,0x30},
	{0x0A,0x00},{0x0F,0x00},{0x7E,0x01},{0x19,0x0C},{0x26,0x01},
	{0x7E,0x02},{0x57,0x06},{0x28,0x02},{0x29,0x00},{0x2A,0x00},
	{0x2B,0x20},{0x1C,0x44},{0x1F,0x4a},{0x20,0x10},{0x21,0x10},
	{0x75,0x90},{0x76,0x90},{0x77,0xA0},{0x14,0x10},{0x1b,0x72},
	{0x52,0x30},{0x5C,0x50},{0x7E,0x02},{0x67,0x13},{0x2E,0x48},
	{0x30,0x2F},{0x3F,0x06},{0x53,0x06},{0x34,0x56},{0x70,0x40},
	{0x71,0xE8},{0x72,0xcf},{0x7E,0x03},{0x35,0x03},{0x30,0x31},
	{0x40,0x65},{0x7E,0x01},
};

static int crown_set_pinctrl_state(struct crown *crown,bool enable)
{
	struct pinctrl_state *state;
	const char* state_name = enable ? "crown_vld_active":"crown_vld_sleep";
	int ret = 0;

	if (crown->vld_pinctrl){
		state = pinctrl_lookup_state(crown->vld_pinctrl, state_name);
		if (IS_ERR(state)) {
		dev_err(&crown->client->dev, DRV_TAG "%s Failed to lookup sleep_pinctrl state: %s\n", state_name);
			return PTR_ERR(state);
		}
		ret = pinctrl_select_state(crown->vld_pinctrl, state);
		if (ret) {
			dev_err(&crown->client->dev, DRV_TAG "%s Failed to select sleep_pinctrl state: %s\n", state_name);
			return ret;
		}
	}

    return ret;
}

static int crown_hw_enable(struct crown *crown,bool enable){
	int ret = 0;

	ret = enable ? regulator_enable(crown->vdd) : regulator_disable(crown->vdd);
	if(!ret){
		crown->enable = enable;
	}
	msleep(10);
	ret = crown_set_pinctrl_state(crown,enable);
	if (ret) {
		 return ret;
	}

	msleep(5);
	return ret;
}

static int crown_set_irq_pinctrl_state(struct crown *crown, bool enable)
{
	struct pinctrl_state *pins_active;
	const char* state_name = enable ? "crown_irq_active":"crown_irq_sleep";
	int ret = 0;
	if (crown->vld_pinctrl) {
		dev_dbg(&crown->client->dev, DRV_TAG "Enter crown_set_irq_pinctrl_state \n");
		pins_active = pinctrl_lookup_state(crown->vld_pinctrl, state_name);
		if (IS_ERR(pins_active)) {
			dev_err(&crown->client->dev, DRV_TAG "%s Failed to lookup crown_irq_active state\n");
			return PTR_ERR(pins_active);
		}

		ret = pinctrl_select_state(crown->vld_pinctrl, pins_active);
		if (ret) {
			dev_err(&crown->client->dev, DRV_TAG "%s Failed to select crown_irq_active _pinctrl state:\n");
			return ret;
		}
	}

	return ret;
}

static int crown_sw_reset(struct crown *crown){
	int ret = 0;

	dev_dbg(&crown->client->dev, DRV_TAG "Enter crown_sw_reset \n");
	ret = i2c_smbus_write_byte_data(crown->client,CONFIGURATION,0x80);
	if (ret < 0){
		dev_err(&crown->client->dev,DRV_TAG "%s resetting chip",__func__);
	}
	msleep(50);

	return 0;
}

static int crown_chip_init(struct crown *crown){
	int ret = 0;
	int regs_size = ARRAY_SIZE(crown_init_regs);
	int i;

	dev_dbg(&crown->client->dev, DRV_TAG "Enter crown_chip_init \n");
	ret = crown_sw_reset(crown);

	for (i = 0; i < regs_size; i++){
		ret = i2c_smbus_write_byte_data(crown->client,crown_init_regs[i].reg,crown_init_regs[i].val);
		if (ret < 0){
			dev_err(&crown->client->dev, DRV_TAG"%s write reg 0x%x failed,ret:%d",__func__,crown_init_regs[i].reg,ret);
			return ret;
		}
	}
	dev_dbg(&crown->client->dev, DRV_TAG " crown_chip_init finish \n");

	return 0;
}

static int crown_check_chipid(struct crown *crown){
	int ret = 0;
	int retry = 5;

	while (retry--){
		ret = i2c_smbus_read_word_data(crown->client, PRODUCT_ID1);
		if (ret < 0) {
			dev_err(&crown->client->dev, DRV_TAG "%s Failed to read chip id,ret:%d \n",__func__,ret);
			msleep(100);
			continue;
		}

		if (ret == CHIP_ID){
			dev_err(&crown->client->dev, DRV_TAG "%s found chid id: 0x%x\n",__func__,ret);
			crown->chip_id = ret;
			return 0;
		}else{
			dev_err(&crown->client->dev,DRV_TAG "%s error:get chid id 0x%x hw chip 0x%x\n",__func__,ret,CHIP_ID);
			msleep(100);
		}
	}

	return ret;
}

static int crown_write_calibration(struct crown *crown){
	int ret = 0;
    uint8_t reg_value_h,reg_value_l;
    u8 data_read[2];

    reg_value_h = (read_value >> 4) & 0xF0;
    reg_value_l = read_value & 0xFF;
    pr_err("[crown] read_value=%d : reg_value_h=0x%02X reg_value_l=0x%02X\n", read_value, reg_value_h, reg_value_l);
    i2c_smbus_write_byte_data(crown->client, 0x0D, reg_value_l);
    i2c_smbus_write_byte_data(crown->client, 0x0F, reg_value_h);
    msleep(2);
    data_read[0] = i2c_smbus_read_byte_data(crown->client,0x0D);
    data_read[1] = i2c_smbus_read_byte_data(crown->client,0x0F);
    pr_err("[crown] 0x0D :0x%x 0x0f :0x%x \n",data_read[0],data_read[1]);

	return ret;
}
/* static void open_cali_work(struct work_struct *work)
{	
    struct delayed_work *dw = container_of(work,struct delayed_work,work);
    struct crown *p = container_of(dw,struct crown,open_work);
    int ret;
    u8 data_read[4];	
    ret = control_cali_file();
    if(ret < 0) {
        pr_err("[crown] READ_CALI_FILE err ：%d \n",ret);
        if(!open_cali_count){	
            schedule_delayed_work(&p->open_work, msecs_to_jiffies(1000));	
            open_cali_count++;
        }
    }else {
        pr_err("[crown] write cali to 0D&0F\n");
        i2c_smbus_write_byte_data(p->client, 0x0D, read_value);
        i2c_smbus_write_byte_data(p->client, 0x0F, read_value);
        data_read[0] = i2c_smbus_read_byte_data(p->client,0x0D);
        data_read[1] = i2c_smbus_read_byte_data(p->client,0x0F);
        pr_err("[crown] check cali data 0x0D:0x%02x , 0F: 0x%02x \n",data_read[0],data_read[1]);
        //cancel_delayed_work_sync(&p->open_work);
    }
} */

static int calculate_rescali(struct crown *crown)
{
	u8 data[4];
	int res_reset = 0;
	int res_calibration = 0;

	data[0] = i2c_smbus_read_byte_data(crown->client,0x0D);
	data[1] = i2c_smbus_read_byte_data(crown->client,0x0F);

	res_reset = (((u8)data[1]&0xf0) << 4|data[0]);
	res_calibration = res_reset * target_data / abs(sum_data_value);
	dev_dbg(&crown->client->dev, DRV_TAG " res_reset:%d res_calibration:%d \n",res_reset,res_calibration);
	return res_calibration;

}

static int control_cali_file(void)
{
	struct file *file = NULL;
	mm_segment_t old_fs;
	loff_t pos = 0;
	char *read_buf;
	int ret;

    pr_err("[crown] %s enter\n", __func__);

	file = filp_open(crown_cali_file_path, O_RDONLY, 0666);
	if (IS_ERR(file)) {
		pr_err("[crown]--2 open /backup/mt3503.txt file ERROR  \n");
		return PTR_ERR(file);
	}
	read_buf = kzalloc(128, GFP_KERNEL);
	if(NULL == read_buf) {
		pr_err("[crown]read_buf mzalloc failed \n");
		filp_close(file,NULL);
		return -ENOMEM;
	}
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	memset(read_buf, 0, 128);
	ret = kernel_read(file, read_buf, 128, &pos);
	if(ret >= 0){
		pr_err("[crown] Data read_buf from file: %s\n", read_buf);
		if (kstrtoint(read_buf, 10, &read_value) == 0) {
			pr_err("[crown] Data read_value from file: %d\n",read_value);
		}
	} else {
		pr_err("[crown] Failed to parse read data\n");
		ret = -EINVAL;
	}
	kfree(read_buf);

	pr_err("[crown] Finsh read cali file\n");
	set_fs(old_fs);
	filp_close(file, NULL);

	return ret;
}

static ssize_t chipid_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = 0;
	struct crown *crown = dev_get_drvdata(dev);

	if (crown){
		ret = crown_check_chipid(crown);
		if (ret){
			return 0;
		}
		ret = snprintf(buf, PAGE_SIZE, "0x%x\n", crown->chip_id);
		dev_err(&crown->client->dev, DRV_TAG"%s chid id: 0x%x\n",__func__,crown->chip_id);
	}

	return ret;
}

static DEVICE_ATTR_RO(chipid);

static ssize_t enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = 0;
	struct crown *crown = dev_get_drvdata(dev);

	if (crown){
		ret = snprintf(buf, PAGE_SIZE, "0x%x\n", crown->enable);
		dev_err(&crown->client->dev, DRV_TAG"%s enable state: 0x%x\n",__func__,crown->enable);
	}

	return ret;
}

static ssize_t enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count){
    bool enable;
    struct crown *crown = dev_get_drvdata(dev);

    if (!kstrtobool(buf, &enable) && crown){
        crown_hw_enable(crown,enable);
    }
    return count;
}

static DEVICE_ATTR_RW(enable);


static ssize_t reset_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = 0;
	struct crown *crown = dev_get_drvdata(dev);
    int val;

	if (crown){
		ret = snprintf(buf, PAGE_SIZE, "0x%x\n", crown->enable);
		dev_err(&crown->client->dev, DRV_TAG"%s enable state: 0x%x\n",__func__,crown->enable);

        val = i2c_smbus_read_byte_data(crown->client, 0x02);
        dev_err(&crown->client->dev, DRV_TAG"%s 0x02=0x%x\n",__func__,val);
        val = i2c_smbus_read_byte_data(crown->client, 0x03);
        dev_err(&crown->client->dev, DRV_TAG"%s 0x03=0x%x\n",__func__,val);
        val = i2c_smbus_read_byte_data(crown->client, 0x04);
        dev_err(&crown->client->dev, DRV_TAG"%s 0x04=0x%x\n",__func__,val);
        val = i2c_smbus_read_byte_data(crown->client, 0x05);
        dev_err(&crown->client->dev, DRV_TAG"%s 0x05=0x%x\n",__func__,val);
        val = i2c_smbus_read_byte_data(crown->client, 0x08);
        dev_err(&crown->client->dev, DRV_TAG"%s 0x08=0x%x\n",__func__,val);
        val = i2c_smbus_read_byte_data(crown->client, 0x12);
        dev_err(&crown->client->dev, DRV_TAG"%s 0x12=0x%x\n",__func__,val);
        val = i2c_smbus_read_byte_data(crown->client, 0x13);
        dev_err(&crown->client->dev, DRV_TAG"%s 0x13=0x%x\n",__func__,val);
        val = i2c_smbus_read_byte_data(crown->client, 0x14);
        dev_err(&crown->client->dev, DRV_TAG"%s 0x14=0x%x\n",__func__,val);
        val = i2c_smbus_read_byte_data(crown->client, 0x15);
        dev_err(&crown->client->dev, DRV_TAG"%s 0x15=0x%x\n",__func__,val);
        val = i2c_smbus_read_byte_data(crown->client, 0x1a);
        dev_err(&crown->client->dev, DRV_TAG"%s 0x1a=0x%x\n",__func__,val);
	}

	return ret;
}

static ssize_t reset_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count){
    bool enable;
    struct crown *crown = dev_get_drvdata(dev);

    if (!kstrtobool(buf, &enable) && crown){
        crown_chip_init(crown);
    }
    return count;
}

static DEVICE_ATTR_RW(reset);

static ssize_t target_data_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    int ret;
    struct crown *crown = dev_get_drvdata(dev);

    pr_info("[crown] enter:%s buf:%s \n",__func__,buf);
    mutex_lock(&crown->lock);
    if (kstrtoint(buf, 0, &ret) == 0) {
        target_data = ret;
    }
    mutex_unlock(&crown->lock);
    return count;
}

static ssize_t target_data_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct crown *crown = dev_get_drvdata(dev);
    ssize_t sret;

    mutex_lock(&crown->lock);
    sret = snprintf(buf, PAGE_SIZE, "%d\n", target_data);
    mutex_unlock(&crown->lock);

	return sret;
}
static DEVICE_ATTR_RW(target_data);

static ssize_t enable_cali_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct crown *crown = dev_get_drvdata(dev);

    pr_info("[crown]enter:%s buf:%s \n",__func__,buf);
    mutex_lock(&crown->lock);
    if (buf[0] == '1') {
        calibrating = 1;
        atomic_set(&sum_data, 0);
    } else if(buf[0] == '0'){
        calibrating = 0;
    } else{
        mutex_unlock(&crown->lock);
        return -EINVAL;
    }
    mutex_unlock(&crown->lock);

    return count;
}

static ssize_t enable_cali_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct crown *crown = dev_get_drvdata(dev);
	ssize_t ret;

    pr_info("[crown]enter:%s \n",__func__);
    mutex_lock(&crown->lock);
    sum_data_value = atomic_read(&sum_data);
    sum_data_value = abs(sum_data_value);
    pr_info("[crown] sum_data_value :%d \n",sum_data_value);
    ret = snprintf(buf, PAGE_SIZE, "%d\n", sum_data_value);
    mutex_unlock(&crown->lock);

    return ret;
}

static DEVICE_ATTR_RW(enable_cali);

static ssize_t verify_cali_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct crown *crown = dev_get_drvdata(dev);
    pr_info("[crown]enter:%s buf:%s \n",__func__,buf);
    mutex_lock(&crown->lock);
    if (buf[0] == '1') {
        verify_sum = 1;
        atomic_set(&verify_sum_data, 0);
    } else if(buf[0] == '0') {
        verify_sum = 0;
    }else{
        mutex_unlock(&crown->lock);
        return -EINVAL;
    }

    mutex_unlock(&crown->lock);

    return count;
}

static ssize_t verify_cali_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct crown *crown = dev_get_drvdata(dev);
	ssize_t ret;
    pr_info("[crown]enter:%s\n",__func__);
    mutex_lock(&crown->lock);
    verify_data = atomic_read(&verify_sum_data);
    verify_data = abs(verify_data);
    pr_err("[crown] verify_data:%d \n",verify_data);
    ret = snprintf(buf, PAGE_SIZE, "%d\n", verify_data);
    mutex_unlock(&crown->lock);

    return ret;
}

static DEVICE_ATTR_RW(verify_cali);

static ssize_t rw_reg_0D_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct crown *crown = dev_get_drvdata(dev);
    int val,ret;
    uint8_t reg;

    pr_info("[crown]enter:%s buf:%s \n",__func__,buf);
    mutex_lock(&crown->lock);

    if (kstrtoint(buf, 0, &val) == 0) {
        reg = val & 0xFF;
        ret = i2c_smbus_write_byte_data(crown->client, 0x0D, reg);
        if (ret < 0){
            dev_err(&crown->client->dev, DRV_TAG"%s write reg 0x0D failed,ret:%d",__func__,ret);
            mutex_unlock(&crown->lock);
            return ret;
        }
    }
    mutex_unlock(&crown->lock);

    return count;
}

static ssize_t rw_reg_0D_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct crown *crown = dev_get_drvdata(dev);
	int val;
    ssize_t sret;

	mutex_lock(&crown->lock);
	val = i2c_smbus_read_byte_data(crown->client, 0x0D);
	pr_info("[crown]enter:%s buf:0x%x \n",__func__,val);
    sret = scnprintf(buf, PAGE_SIZE, "%d\n", val);
	mutex_unlock(&crown->lock);
	return sret;
}
static DEVICE_ATTR_RW(rw_reg_0D);

static ssize_t rw_reg_0F_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct crown *crown = dev_get_drvdata(dev);
    int val,ret;
    uint8_t reg;

    mutex_lock(&crown->lock);
    pr_info("[crown]enter:%s buf:%s \n",__func__,buf);
    if (kstrtoint(buf, 0, &val) == 0) {
        reg = (val >> 4) & 0xF0;
        ret = i2c_smbus_write_byte_data(crown->client, 0x0F, reg);
        if (ret < 0){
            dev_err(&crown->client->dev, DRV_TAG"%s write reg 0x0F failed,ret:%d",__func__,ret);
            mutex_unlock(&crown->lock);
            return ret;
        }
    }
    mutex_unlock(&crown->lock);

    return count;
}

static ssize_t rw_reg_0F_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct crown *crown = dev_get_drvdata(dev);
    int val;
    ssize_t sret;

    mutex_lock(&crown->lock);
    val = i2c_smbus_read_byte_data(crown->client, 0x0F);
    pr_info("[crown]enter:%s buf:0x%x \n",__func__,val);
    sret = scnprintf(buf, PAGE_SIZE, "%d\n", val);
    mutex_unlock(&crown->lock);
    return sret;
}
static DEVICE_ATTR_RW(rw_reg_0F);

static ssize_t res_calibration_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct crown *crown = dev_get_drvdata(dev);
    int val;
    ssize_t sret;

    mutex_lock(&crown->lock);
    val = calculate_rescali(crown);
    val = abs(val);
    sret = scnprintf(buf, PAGE_SIZE, "%d\n", val);
    mutex_unlock(&crown->lock);
    return sret;
}
static DEVICE_ATTR_RO(res_calibration);

static ssize_t reboot_cali_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct crown *crown = dev_get_drvdata(dev);
    int ret;
    //char hex_value[4];

    mutex_lock(&crown->lock);
    if (buf[0] == '1') {
        pr_info("[crown]enter:%s buf:%s \n",__func__,buf);
        ret = control_cali_file();
        if(ret < 0) {
            pr_err("[crown] READ_CALI_FILE err ：%d \n",ret);
        } else {
            //snprintf(hex_value, sizeof(hex_value), "%02X", (unsigned int)read_value);
            //sscanf(hex_value, "%2hhX", &reg_value);
            if(read_value > 0){
                crown_write_calibration(crown);

            }else{
                pr_err("[crown] calibration data error :0x%x \n",read_value);
            }
        }
    }
    mutex_unlock(&crown->lock);

    return count;
}

static ssize_t reboot_cali_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct crown *crown = dev_get_drvdata(dev);
    int val;
    ssize_t sret;

    mutex_lock(&crown->lock);
    val = i2c_smbus_read_byte_data(crown->client, 0x0F);
    pr_info("[crown]enter:%s buf:0x%x \n",__func__,val);
    sret = scnprintf(buf, PAGE_SIZE, "%d\n", val);
    mutex_unlock(&crown->lock);
    return sret;
}
static DEVICE_ATTR_RW(reboot_cali);

static struct attribute *crown_attributes[] = {
	&dev_attr_chipid.attr,
	&dev_attr_enable.attr,
	&dev_attr_enable_cali.attr,
	&dev_attr_verify_cali.attr,
	&dev_attr_rw_reg_0D.attr,
	&dev_attr_rw_reg_0F.attr,
	&dev_attr_target_data.attr,
	&dev_attr_res_calibration.attr,
	&dev_attr_reboot_cali.attr,
	&dev_attr_reset.attr,
	NULL,
};

struct attribute_group crown_attribute_group = {
	.attrs = crown_attributes,
};

static void crown_read_data(struct crown *crown)
{
	u8 data[10];
	int delta_x = 0;
	//int delta_y = 0;
	int delta_x_repodel = -1;
	int delta_x_repoadd = 1;
    bool loop = false;
    int times = 0;
    static int crown_data = 0;

    atomic_set(&crown_sum_data, 0);
	data[0] = i2c_smbus_read_byte_data(crown->client,Motion_Status);
	if((data[0] & 0x80) == 0x80)
	{
		do{
			data[1] = i2c_smbus_read_byte_data(crown->client,Delta_X_Lo);
			data[2] = i2c_smbus_read_byte_data(crown->client,Delta_Y_Lo);
			data[3] = i2c_smbus_read_byte_data(crown->client,Delta_XY_Hi);
			data[4] = i2c_smbus_read_byte_data(crown->client,0x1a);

			delta_x=((u8)data[3]&0xf0)>>4;
			delta_x=(delta_x<<8)|data[1];
			//delta_y=((u8)data[3])&0x0f;
			//delta_y=(delta_y<<8)|data[2];
			if((delta_x&0x0800)==0x0800)
				delta_x=delta_x-0x1000;
			//if((delta_y&0x0800)==0x0800)
			//	delta_y=delta_y-0x1000;

            if((data[0] == 0xf3) && (data[1] == 0xf3)){
                crown_chip_init(crown);
                crown_write_calibration(crown);
                break;
            }

            if(((data[0] & 0x80) == 0) && (data[1] == 0) && (data[2] == 0) && (data[3] == 0)){
                times++;
            }

			dev_err(&crown->client->dev, "delta_x=%d status=0x%02x reg0x1a=0x%02x times=%d\n", delta_x, data[0], data[4], times);

			if(calibrating) {
				atomic_add(delta_x, &sum_data);
			}else if(verify_sum) {
				atomic_add(delta_x, &verify_sum_data);
			}else{
				atomic_add(delta_x, &crown_sum_data);
                crown_data = atomic_read(&crown_sum_data);
                if(crown_data > 15){
                    if(crown_data > 30)
                        crown_data = 30;
                    else
                        crown_data -= 15;
                    atomic_set(&crown_sum_data, crown_data);
                    delta_x = delta_x_repodel;
                }else if(crown_data < -15){
                    if(crown_data < -30)
                        crown_data = -30;
                    else
                        crown_data += 15;
                    atomic_set(&crown_sum_data, crown_data);
                    delta_x = delta_x_repoadd;
                }else{
                    delta_x = 0;
                }

                dev_dbg(&crown->client->dev, "crown_data=%d delta_x_=%d\n", crown_data, delta_x);

                /* 			
                if(delta_x < 0) {
                    delta_x = delta_x_repoadd;
                } else if(delta_x > 0){
                    delta_x = delta_x_repodel;
                } */

                if(crown->input_dev && delta_x != 0){
                    input_report_rel(crown->input_dev, REL_WHEEL, delta_x);
                    input_sync(crown->input_dev);                
                }
            }

			msleep(20);
			data[0] = i2c_smbus_read_byte_data(crown->client,Motion_Status);
            if((data[0] & 0x80) == 0x80){
                loop = true;
                times = 0;
            }
            else if(times < 5){
                loop = true;
            }
            else{
                loop = false;
           }

		}while(loop);
	}
    if(!crown->is_delay_work)
        schedule_delayed_work(&crown->irq_work, msecs_to_jiffies(200));

}
static void crown_irq_enable(struct crown *crown)
{
    unsigned long spin_lock_flags;

    spin_lock_irqsave(&crown->irq_lock, spin_lock_flags);
    if(crown->irq_status) {
        enable_irq(crown->irq);
        crown->irq_status = 0;
    }
    spin_unlock_irqrestore(&crown->irq_lock, spin_lock_flags);
}

static void crown_irq_disable(struct crown *crown)
{
    unsigned long spin_lock_flags;

    spin_lock_irqsave(&crown->irq_lock, spin_lock_flags);
    if (!crown->irq_status) {
        crown->irq_status = 1;
        disable_irq_nosync(crown->irq);
    }
    spin_unlock_irqrestore(&crown->irq_lock, spin_lock_flags);
}

static void crown_schedule_work(struct work_struct *work)
{
    struct delayed_work *dw = container_of(work, struct delayed_work, work);
    struct crown *crown = container_of(dw,struct crown, irq_work);

	dev_dbg(&crown->client->dev, "enter crown_schedule_work \n");
    crown_irq_disable(crown);
    crown->is_delay_work = true;
	crown_read_data(crown);
    crown_irq_enable(crown);
}

static irqreturn_t crown_irq_handler(int irq,void *dev_id)
{
	struct crown *crown = dev_id;

	dev_dbg(&crown->client->dev, "enter crown_irq_handler \n");

    crown_irq_disable(crown);
    cancel_delayed_work_sync(&crown->irq_work);
    crown->is_delay_work = false;
	crown_read_data(crown);
	//usleep_range(500, 600);
    //schedule_work(&crown->irq_work);
    crown_irq_enable(crown);
	return IRQ_HANDLED;

}

static int crown_parse_dt(struct device *dev,struct crown *crown)
{
	struct device_node *np = dev->of_node;
	int ret = 0;

	dev_err(dev, "enter crown_parse_dt \n");
	if (!np) {
	    dev_err(dev, "crown_parse_dt np is NULL ");
	    return -EINVAL;
	}

	crown->irq_gpio = of_get_named_gpio_flags(np, "mt3503,irq-gpio",0, &crown->irq_gpio_flags);
	if (crown->irq_gpio < 0) {
		dev_err(dev, "Unable to get irq_gpio");
	}

	dev_err(dev, "irq gpio:%d \n",crown->irq_gpio);
	if (gpio_is_valid(crown->irq_gpio)) {
		ret = gpio_request(crown->irq_gpio, "mt3503_irq_gpio");
		if (ret) {
			dev_err(dev, "gpio request failed:%d \n",ret);
			return ret;
		}
	}
	ret = gpio_direction_input(crown->irq_gpio);
	if (ret) {
		dev_err(dev, "Set_direction for INT gpio failed \n");
	    if (gpio_is_valid(crown->irq_gpio)){
			gpio_free(crown->irq_gpio);
		}
		return ret;
	}

	ret = crown_set_irq_pinctrl_state(crown, true);

	return ret;
}

static int crown_input_init(struct crown *crown)
{
	int err = 0;

	dev_err(&crown->client->dev, "enter input init \n");
	crown->input_dev = input_allocate_device();
	if (!crown->input_dev) {
		dev_err(&crown->client->dev, "Failed to allocate input device \n");
		return -ENOMEM;
	}

	crown->input_dev->name = DRIVER_NAME;
	crown->input_dev->phys = "input/crown";
	crown->input_dev->id.bustype = BUS_I2C;
	input_set_drvdata(crown->input_dev, crown);

	input_set_capability(crown->input_dev, EV_REL, REL_WHEEL);

	err = input_register_device(crown->input_dev);
	if (err) {
		dev_err(&crown->client->dev, "unable to register sensor mt3503 INPUT \n");
		input_free_device(crown->input_dev);
	}

	return err;

}

static int crown_register_irq(struct crown *crown)
{
	int err = 0;

	crown->irq = gpio_to_irq(crown->irq_gpio);
	crown->irq_gpio_flags = /* IRQF_TRIGGER_FALLING */IRQF_TRIGGER_LOW | IRQF_ONESHOT;

	dev_err(&crown->client->dev, "Request IRQ:irq:%d, flag:%x ", crown->irq, crown->irq_gpio_flags);

	err = request_threaded_irq(crown->irq,NULL,crown_irq_handler,crown->irq_gpio_flags,crown->client->name, crown);
	if (err) {
		dev_err(&crown->client->dev, "request_threaded_irq Failed :%d",err);
		if (gpio_is_valid(crown->irq_gpio)){
			gpio_free(crown->irq_gpio);
		}
	}
    crown_irq_disable(crown);

	return err;
}

static int crown_probe(struct i2c_client *client)
{
	struct crown *crown;
	int err = 0;

	crown = devm_kzalloc(&client->dev, sizeof(struct crown), GFP_KERNEL);
	if (!crown)
		return -ENOMEM;

	i2c_set_clientdata(client, crown);
	dev_set_drvdata(&client->dev, crown);

	dev_err(&client->dev, "enter crown_probe start-V1\n");
	crown->vdd = devm_regulator_get_optional(&client->dev, "vdd");
	if (IS_ERR(crown->vdd)) {
		err = PTR_ERR(crown->vdd);
		if (err == -EPROBE_DEFER)
			return -EPROBE_DEFER;
	}

	crown->vld_pinctrl = devm_pinctrl_get(&client->dev);
	if (IS_ERR(crown->vld_pinctrl)) {
		dev_err(&client->dev, DRV_TAG"Failed to get vld_pinctrl\n");
		return PTR_ERR(crown->vld_pinctrl);
	}

	if (crown_hw_enable(crown,true)){
		dev_err(&client->dev, DRV_TAG"Failed enable vld and vdd supply\n");
		return -EINVAL;
	}
	dev_err(&client->dev, "Finish crown_hw_enable \n");
	crown->client = client;
    spin_lock_init(&crown->irq_lock);
    //INIT_WORK(&crown->irq_work, crown_schedule_work);
    INIT_DELAYED_WORK(&crown->irq_work, crown_schedule_work);

	err = crown_parse_dt(&client->dev,crown);

	err = crown_input_init(crown);

	err = crown_register_irq(crown);

	err = crown_chip_init(crown);
	if (err){
		return err;
	}

	err = crown_check_chipid(crown);
	if (err){
		dev_err(&client->dev, DRV_TAG"%s check chipid failed (%d)", __func__, err);
		return err;
	}

	crown->crown_dev.name = DRIVER_NAME;
	crown->crown_dev.dev_attr_group = &crown_attribute_group;
	err = crown_device_register(&crown->crown_dev,crown);
	if (err){
		dev_err(&client->dev, DRV_TAG"%s device register failed (%d)", __func__, err);
		return err;
	} 
    crown_irq_enable(crown);

	mutex_init(&crown->lock);

	dev_err(&client->dev, "probe exit\n");
	//INIT_DELAYED_WORK(&crown->open_work,open_cali_work);
	//schedule_delayed_work(&crown->open_work, msecs_to_jiffies(3000));

	return err;
}

static int crown_remove(struct i2c_client *client)
{
	struct crown *crown = i2c_get_clientdata(client);
	int ret = 0;

	dev_err(&crown->client->dev, DRV_TAG"%s.",__func__);
	free_irq(crown->irq, crown);
	gpio_free(crown->irq_gpio);
	input_unregister_device(crown->input_dev);
	mutex_destroy(&crown->lock);
    //cancel_work_sync(&crown->irq_work);
    cancel_delayed_work_sync(&crown->irq_work);
	devm_kfree(&client->dev,crown);

	return ret;
}

static int __maybe_unused crown_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct crown *crown = i2c_get_clientdata(client);

	dev_err(&crown->client->dev, DRV_TAG"%s  regulator disable.",__func__);
    cancel_delayed_work_sync(&crown->irq_work);
    crown_irq_disable(crown);
    crown_set_irq_pinctrl_state(crown, false);
    return crown_hw_enable(crown, false);
}

static int __maybe_unused crown_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct crown *crown = i2c_get_clientdata(client);

	dev_err(&crown->client->dev, DRV_TAG"%s regulator enable.",__func__);
    crown_set_irq_pinctrl_state(crown, true);
    crown_irq_enable(crown);
    return crown_hw_enable(crown, true);
}

static SIMPLE_DEV_PM_OPS(crown_pm_ops, crown_suspend, crown_resume);

static const struct i2c_device_id crown_ids[] = {
	{ "mt3503",0},
	{},
};
MODULE_DEVICE_TABLE(i2c, crown_ids);

static const struct of_device_id crown_of_match_table[] = {
	{ .compatible = "maxic,mt3503", },
	{ },
};

static struct i2c_driver crown_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = crown_of_match_table,
		.pm = &crown_pm_ops,
	},
	.probe_new = crown_probe,
	.remove	= crown_remove,
	.id_table = crown_ids,
};

module_i2c_driver(crown_driver);

MODULE_AUTHOR("yihui.luo@thundercomm.com");
MODULE_DESCRIPTION("crown sensor driver connected on I2C");
MODULE_LICENSE("GPL v2");
MODULE_IMPORT_NS(VFS_internal_I_am_really_a_filesystem_and_am_NOT_a_driver);
