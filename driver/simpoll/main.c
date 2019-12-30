/*
 * main.c
 *
 *  Created on: Dec 10, 2019
 *      Author: lay
 */

#include <linux/module.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/poll.h>

#include <asm/uaccess.h>

#define VERSION		"0.0.0.1"


struct simpoll {
	void *data;
};

#ifdef BUILD_TIME
static const char *_build_time = BUILD_TIME;
#else
static const char *_build_time = "";
#endif

static struct cdev _cdev;
static dev_t _devno; /* device number */

static int _open(struct inode *i, struct file *f)
{
	int ret = 0;
	return ret;
}

static int _release(struct inode *i, struct file *f)
{
	return 0;
}

static ssize_t _read(struct file *f, char __user *buf, size_t size, loff_t * offset)
{
	int ret = 0;
	return ret;
}

static ssize_t _write(struct file *f, const char __user *data, size_t len, loff_t *offset)
{
	return 0;
}

static unsigned int _poll(struct file *f, struct poll_table_struct *pt)
{
	unsigned int mask = 0;
	return mask;
}

static loff_t _llseek(struct file *f, loff_t offset, int orig)
{
	return 0;
}

static long _unlocked_ioctl(struct file *f, unsigned int cmd, unsigned long arg)
{
	int ret;
	return ret;
}

static const struct file_operations _fops = {
	.owner = THIS_MODULE,
	.open = _open,
	.release = _release,
	.write = _write,
	.read = _read,
	.poll = _poll,
	.llseek = _llseek,
	.unlocked_ioctl = _unlocked_ioctl
};

static int _probe(struct platform_device *pdev)
{
	int ret = 0;
	struct device *dev = &pdev->dev;
	dev_info(dev, "probe %s (%s)\r\n", VERSION, _build_time);

	return ret;
}

static int _remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	dev_info(dev, "remove\r\n");
	return 0;
}

static struct of_device_id _of_match[] = {
	{ .compatible = "fpga-jpeg", },
	{ /* end of list */},
};
MODULE_DEVICE_TABLE(of, _of_match);

static struct platform_driver _pl_driver = {
	.driver = {
		.name = "simpoll",
		.owner = THIS_MODULE,
		.of_match_table = _of_match,
	},
	.probe = _probe,
	.remove = _remove,
};

static int __init _jpeg_init(void)
{
	int ret = 0;
	printk("==== SIM POLL MODULE INIT ====\r\n");
	cdev_init(&_cdev, &_fops);
	ret = alloc_chrdev_region(&_devno, 0, 1, "simpoll");
	if (ret == 0) {
		ret = cdev_add(&_cdev, _devno, 1);
		if (ret == 0) {
			ret = platform_driver_register(&_pl_driver);
			if (ret) {
				cdev_del(&_cdev);
				printk(KERN_ERR "platform_driver_register error\r\n");
			}
		} else {
			unregister_chrdev_region(_devno, 1);
			printk(KERN_ERR "cdev_add error\r\n");
		}
	} else {
		printk(KERN_ERR "register_chrdev_region error\r\n");
	}
	return ret;
}

static void __exit _jpeg_exit(void)
{
	cdev_del(&_cdev);
	unregister_chrdev_region(_devno, 1);
	platform_driver_unregister(&_pl_driver);
	printk("==== SIM POLL MODULE EXIT ====\r\n");
}

module_init(_jpeg_init);
module_exit(_jpeg_exit);

MODULE_AUTHOR("Lay Zhan");
MODULE_DESCRIPTION("zynq fpga jpeg module driver of prague");
MODULE_LICENSE("GPL v2");

