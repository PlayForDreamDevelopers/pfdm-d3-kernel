#include <linux/module.h>
#include <linux/types.h>
#include <linux/of.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/platform_device.h>
#include <linux/timekeeping.h>
#include <linux/qbtimer_offset.h>
#include <linux/delay.h>

#define NSEC_PER_SEC 1000000000L
#define USEC_PER_SEC 1000000L

static struct class *qbtimer_offset = NULL;
static struct device *qbtimer_dev = NULL;
static uint64_t time_offset = 0;
static struct mutex state_mutex;

static uint64_t qtimer_get_freq(void)
{
	uint64_t val = 0;

	asm volatile("mrs %0, cntfrq_el0" : "=r" (val));
	return val;
}

static uint64_t qtimer_get_ticks(void)
{
#ifdef CONFIG_ARM64
	unsigned long long val = 0;
	asm volatile("mrs %0, cntvct_el0" : "=r" (val));
	return val;
#else
	u64 val;
	unsigned long lsb = 0, msb = 0;
	asm volatile("mrrc p15, 1, %[lsb], %[msb], c14"
				 +                                 : [lsb] "=r" (lsb), [msb] "=r" (msb));
	val = ((uint64_t)msb << 32) | lsb;
	return val;
#endif
}

static uint64_t qtimer_ticks_to_ns(uint64_t ticks) //per tick = 1000000000/19200000 ns
{
	return ticks * NSEC_PER_SEC / qtimer_get_freq();
}

uint64_t qtimer_get_time_ns(void)
{
	return qtimer_ticks_to_ns(qtimer_get_ticks());
}
EXPORT_SYMBOL(qtimer_get_time_ns);

static uint64_t qbtimer_get_offset()
{
	uint64_t qtime_now, btime_now, offset = 0;
	struct timespec64 ts;

	qtime_now = qtimer_get_time_ns();

	ktime_get_boottime_ts64(&ts);
	btime_now = (uint64_t)((ts.tv_sec * 1000000000) + ts.tv_nsec);

	if(qtime_now < btime_now){
		pr_info("===qbtimer: wtf, qtimer < btimer ===\n");
		return 0;
	} else {
		offset = qtime_now - btime_now;
	}

	return offset;
}

static ssize_t offset_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	int status = 0;
	int retval = 0;

	status = mutex_lock_interruptible(&state_mutex);
	if (status != 0) {
		printk("Failed to get state mutex: %d", status);
		return status;
	}
	// sprintf(buf + len, "%llu", time_offset)
	retval = scnprintf(buf, PAGE_SIZE, "%llu\n", time_offset);

	mutex_unlock(&state_mutex);

	return retval;
}

static ssize_t offset_store(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t count)
{
	int status = 0;
	uint64_t tmp_time_offset = 0;

	status = kstrtou64(buf, /*base */10, &tmp_time_offset);
	if (status < 0) {
		printk("Failed to parse integer out of %s", buf);
		return -EINVAL;
	}

	status = mutex_lock_interruptible(&state_mutex);
	if (status != 0) {
		printk("Failed to get state mutex: %d", status);
		return status;
	}

	time_offset = tmp_time_offset;
	status = count;

	mutex_unlock(&state_mutex);
	return status;
}
static struct device_attribute offset = {
	.attr = {
		.name = "offset",
		.mode = S_IWUSR | S_IRUGO,
	},
	.show = offset_show,
	.store = offset_store,
};

uint64_t get_qbtimer_offset(void)
{
	return time_offset;
}

#define QBTIMER_OFFSET_NUMBER 10
static int qbtimer_offset_probe(struct platform_device *pdev)
{
	int ret = 0;

	int i = 0;
	uint64_t tmp_ts = 0;
	uint64_t active_count = 0;
	uint64_t qb_offset[QBTIMER_OFFSET_NUMBER] = {0};
	uint64_t max, min, avg, sum;

	for(i=0; i < QBTIMER_OFFSET_NUMBER; i++) {
		tmp_ts = qbtimer_get_offset();
		if (tmp_ts == 0) {
			break;
		} else {
			qb_offset[i] = tmp_ts;
			active_count++;
		}
		udelay(5000);
	}

	if (active_count > 3) {
		//loop active count times, remove max/min and get avg
		sum = 0;
		max = qb_offset[0];
		min = qb_offset[0];
		for (i = 0; i < active_count; i++) {
			if (max < qb_offset[i])
				max = qb_offset[i];

			if (min > qb_offset[i])
				min = qb_offset[i];

			sum += qb_offset[i];
		}
		time_offset = (avg = (sum - max - min) / (active_count - 2));
		pr_info("%s, max[%llu] min[%llu]: avg_offset:%llu %d\n", __func__,
			max, min, time_offset, active_count);
	} else {
		time_offset = qb_offset[0];
		pr_info("%s, offset:%llu %d\n", __func__, time_offset, active_count);
	}

	qbtimer_offset = class_create(THIS_MODULE, "qbtimer_offset");
	if(IS_ERR(qbtimer_offset))
	{
		ret = PTR_ERR(qbtimer_offset);
		printk("Failed to create class.\n");
		return ret;
	}

	qbtimer_dev = device_create(qbtimer_offset, NULL, 0, NULL, "qbtimer");
	if (IS_ERR(qbtimer_dev))
	{
		ret = PTR_ERR(qbtimer_offset);
		printk("Failed to create device(qbtimer_dev)!\n");
		return ret;
	}

	ret = device_create_file(qbtimer_dev, &offset);
	if (ret < 0) return ret;

	mutex_init(&state_mutex);

	return ret;
}

static int qbtimer_offset_remove(struct platform_device *pdev)
{
	device_destroy(qbtimer_offset, 0);
	class_destroy(qbtimer_offset);
	device_remove_file(qbtimer_dev, &offset);

	return 0;
}

static struct of_device_id qbtimer_offset_dt_match[] = {
	{ .compatible = "playfordream,qbtimer_offset",},
	{ },
};
MODULE_DEVICE_TABLE(of, qbtimer_offset_dt_match);

static struct platform_driver qbtimer_offset_driver = {
	.driver = {
		.name     = "qbtimer_offset",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(qbtimer_offset_dt_match),
	},
	.probe        = qbtimer_offset_probe,
	.remove = qbtimer_offset_remove,
};

static int __init qbtimer_offset_init(void)
{
	return platform_driver_register(&qbtimer_offset_driver);
}

static void __exit qbtimer_offset_exit(void)
{
	platform_driver_unregister(&qbtimer_offset_driver);
}

module_init(qbtimer_offset_init);
module_exit(qbtimer_offset_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("qbtimer offset driver");
