/*
 * blink.c
 *
 *  Created on: Nov 26, 2019
 *      Author: lay
 */


#include <linux/module.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>


struct blink {
	void __iomem *reg;
};

typedef struct {
	irq_handler_t irq_pf;
	irq_handler_t irq_th_pf;
	const char *name;
} irq_proc_t;


static struct of_device_id __of_match[] =
{
    { .compatible = "vendor,blink", },
    { /* end of list */ },
};
MODULE_DEVICE_TABLE(of, __of_match);


static struct cdev __cdev;
static dev_t __devno; /* device number */

static irqreturn_t _irq_image_cache_done(int irq, void *dev_id) {
	irqreturn_t ret = IRQ_HANDLED;
	struct fpga_blink *fb = (struct fpga_blink *)dev_id;
	printk("IRQ-> %d @%p\r\n", irq, fb);
	return ret;
}

static const irq_proc_t __irq_vic[] = {
	{_irq_image_cache_done, NULL , "IMAGE_CACHE_DONE_INT"},	/* 61 */
	{NULL, NULL, "LVDS_ALL_BITSLIP_DONE"},	/* 62 */
	{NULL, NULL, "ERROR_INT"},	/* 63 */
	{NULL, NULL, "JPEG_DONE_INT"},	/* 64 */
	{NULL, NULL, ""},	/* 65 */
	{NULL, NULL, ""},	/* 66 */
	{NULL, NULL, "AF_DONE"},	/* 67 */
	{NULL, NULL, ""},	/* 68 */
	{NULL, NULL, "AE_DONE"},	/* 84 */
	{NULL, NULL, ""},	/* 85 */
	{NULL, NULL, "HISTOGRAM_DONE"},	/* 86 */
	{NULL, NULL, "PEAK_DETECTION_DONE"},	/* 87 */
	{NULL, NULL, "MIC_CACHE_DONE"},	/* 88 */
	{NULL, NULL, ""},	/* 89 */
	{NULL, NULL, "ACQ_IMX287_RUN"},	/* 90 */
	{NULL, NULL, "IMX287_TRI"},	/* 91 */
};

static const int __irq_porc_num = ARRAY_SIZE(__irq_vic);

static int _probe(struct platform_device *pdev) {
	int ret = 0, irq, i, irq_num;
	struct blink *fb;
    struct resource *r_mem; /* IO mem resources */
    struct device *dev = &pdev->dev;

	dev_info(dev, "probe\r\n");

	fb = devm_kzalloc(&pdev->dev, sizeof(struct blink), GFP_KERNEL);
	if (fb == NULL) {
		dev_err(dev, "kzmalloc memory for fpga blink failed\r\n");
		return -ENOMEM;
	}

	r_mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	fb->reg = devm_ioremap_resource(&pdev->dev, r_mem);
	if (IS_ERR(fb->reg))
		/* don't need to release the devm_* resources */
		return PTR_ERR(fb->reg);

	irq_num = platform_irq_count(pdev);
	if (irq_num > __irq_porc_num) {
		dev_warn(dev, "irq proc number mismatch: dev=%d, dir=%d(use)\r\n", irq_num, __irq_porc_num);
		irq_num = __irq_porc_num;
	}

	for (i = 0; i < irq_num; i++) {
		const irq_proc_t *h = &__irq_vic[i];
		if (h->irq_pf) {
			irq = platform_get_irq(pdev, i);
			if (irq < 0) {
				dev_err(dev, "no irq resource\n");
				return irq;
			}

			dev_info(dev, "IRQ[%d]: %s is %d\r\n", i, h->name, irq);
			ret = devm_request_threaded_irq(&pdev->dev, irq, h->irq_pf, h->irq_th_pf, 0, dev_name(&pdev->dev), fb);
			if (ret) {
				dev_err(dev, "request irq failed\n");
				return ret;
			}
		}
	}

	platform_set_drvdata(pdev, fb);
	return ret;
}

static int _remove(struct platform_device *pdev) {
	struct device *dev = &pdev->dev;
	platform_set_drvdata(pdev, NULL);
	dev_info(dev, "remove\r\n");
	return 0;
}

static struct platform_driver __fpga_blink_driver =
{
    .driver = {
        .name = "blink",
        .owner = THIS_MODULE,
        .of_match_table	= __of_match,
    },
    .probe		= _probe,
    .remove		= _remove,
};

static int _open(struct inode *i, struct file *f) {
	return 0;
}

static int _release(struct inode *i, struct file *f) {
	return 0;
}

static ssize_t _read(struct file *f, char __user *buf, size_t size, loff_t * offset) {
	return 0;
}

static ssize_t _write(struct file *f, const char __user *data, size_t len, loff_t *offset) {
	return 0;
}

static unsigned int _poll (struct file *f, struct poll_table_struct *pt) {
	return 0;
}

static loff_t _llseek(struct file *f, loff_t offset, int orig) {
	return 0;
}

static long _unlocked_ioctl (struct file *f, unsigned int cmd, unsigned long arg) {
	return 0;
}

static const struct file_operations __fops = {
	.owner = THIS_MODULE,
	.open = _open,
	.release = _release,
	.write = _write,
	.read = _read,
	.poll = _poll,
	.llseek = _llseek,
	.unlocked_ioctl = _unlocked_ioctl
};

static int __init _blink_init(void)
{
    int ret = 0;
    printk("==== BLINK MODULE INIT ====\r\n");
    /* init char device */
    cdev_init(&__cdev, &__fops);
    ret = alloc_chrdev_region(&__devno, 0, 1, "blink");
    if (ret) {
    	printk(KERN_ERR "register_chrdev_region error\r\n");
    	goto ps_init_exit;
    }

	ret = cdev_add(&__cdev, __devno, 1);
	if (ret) {
		printk(KERN_ERR "cdev_add error\r\n");
		goto ps_cdev_add_err;
	}

    ret = platform_driver_register(&__fpga_blink_driver);
    if (ret == 0) {
    	goto ps_init_exit;
    }

    cdev_del(&__cdev);
ps_cdev_add_err:
    unregister_chrdev_region(__devno, 1);
ps_init_exit:
    return ret;
}

static void __exit _blink_exit(void)
{
	cdev_del(&__cdev);
	unregister_chrdev_region(__devno, 1);
	platform_driver_unregister(&__fpga_blink_driver);
	printk("==== BLINK MODULE EXIT ====\r\n");
}

module_init(_blink_init);
module_exit(_blink_exit);

MODULE_AUTHOR("Lay Zhan");
MODULE_DESCRIPTION("zynq blink module driver");
MODULE_LICENSE("GPL v2");
