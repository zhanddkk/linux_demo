/*
 * main.c
 *
 *  Created on: Dec 26, 2019
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

#include "../include/fpga_jpeg.h"

#define VERSION                         "0.0.0.1"

// jpeg reg group
#define REG_JPEG_CTRL                   0x0000

#define REG_JPEG_STATUS                 0x0004

#define REG_JPEG_R_ADDR                 0x0008
#define REG_JPEG_G_ADDR                 0x000c
#define REG_JPEG_B_ADDR                 0x0010

#define REG_JPEG_P_ADDR                 0x0014

#define REG_JPEG_WADDR                  0x0018

#define REG_JPEG_SIZE                   0x001c

#define REG_JPEG_QUANTIZATION_LUT_ADDR  0x0108
#define REG_JPEG_QUANTIZATION_LUT_DATA  0x010c

// err reg group
#define REG_ERR_FIFO                    0x0004
#define REG_ERR_NORESP                  0x0008

struct FPGA_JPEG
{
    void __iomem *jpeg_reg_base;
    void __iomem *err_reg_base;
    struct class *dev_class;
    struct device *class_dev;
    wait_queue_head_t r_wait;
    u32 interruped;
    struct JPEG_ENCODE_RESULT_DATA result;
};

typedef struct
{
    irq_handler_t irq_pf;
    irq_handler_t irq_th_pf;
    const char *name;
    u32 flag;
} IRQ_PROC_T;

#ifdef BUILD_TIME
static const char *_build_time = BUILD_TIME;
#else
static const char *_build_time = "";
#endif

static struct cdev _cdev;
static dev_t _devno; /* device number */
static u32 _open_flag = 0;
struct FPGA_JPEG *_dev_priv;

static irqreturn_t _irq_error(int irq, void *dev_id);
static irqreturn_t _irq_jpeg_done(int irq, void *dev_id);

static const IRQ_PROC_T _irq_vic[] =
{
    { _irq_error, NULL, "ERROR_INT", IRQF_SHARED }, /* 63 */
    { _irq_jpeg_done, NULL, "JPEG_DONE_INT", 0 }, /* 64 */
};

static const int _irq_porc_num = ARRAY_SIZE(_irq_vic);

// PL operation
static void _pl_init(void __iomem *reg_base)
{
    // enable jpeg done IRQ
    writel(1, reg_base + REG_JPEG_CTRL);
}

static void _pl_deinit(void __iomem *reg_base)
{
    writel(0, reg_base + REG_JPEG_CTRL);
}

static int _pl_do_encode(void __iomem *reg_base,
    const struct JPEG_ENCODE_PARAM *param)
{
    u32 ctl = readl(reg_base + REG_JPEG_CTRL) & 0x00000001;

#if 0
    writel(ctl, reg_base + REG_JPEG_CTRL);
#endif

    switch (param->type)
    {
    case N_JPEG_ENCODE_TYPE_P:
        writel(param->raw_image_addr.pattern, reg_base + REG_JPEG_P_ADDR);
        break;
    case N_JPEG_ENCODE_TYPE_S:
        ctl |= (1ul << 1);
        writel(param->raw_image_addr.shade.r, reg_base + REG_JPEG_R_ADDR);
        writel(param->raw_image_addr.shade.g, reg_base + REG_JPEG_G_ADDR);
        writel(param->raw_image_addr.shade.b, reg_base + REG_JPEG_B_ADDR);
        break;
    default:
        // return back invalid param
        return -1;
    }
    writel(param->jpeg_addr, reg_base + REG_JPEG_WADDR);
    ctl |= 1ul << 2;
    writel(ctl, reg_base + REG_JPEG_CTRL);
    return 0;
}

static int _pl_set_quantization(void __iomem *reg_base,
    const struct QUANTIZATION *q)
{
    u32 index;
    switch (q->type)
    {
    case N_QUANTIZATION_TYPE_PL:
        writel(0, reg_base + REG_JPEG_QUANTIZATION_LUT_ADDR);
        break;
    case N_QUANTIZATION_TYPE_PC:
        writel(64, reg_base + REG_JPEG_QUANTIZATION_LUT_ADDR);
        break;
    case N_QUANTIZATION_TYPE_SL:
        writel(64 * 2, reg_base + REG_JPEG_QUANTIZATION_LUT_ADDR);
        break;
    case N_QUANTIZATION_TYPE_SC:
        writel(64 * 3, reg_base + REG_JPEG_QUANTIZATION_LUT_ADDR);
        break;

    default:
        return -1;
    }

    for (index = 0; index < ARRAY_SIZE(q->data); index++)
    {
        writel(q->data[index], reg_base + REG_JPEG_QUANTIZATION_LUT_DATA);
    }
    return 0;
}

static irqreturn_t _irq_error(int irq, void *dev_id)
{
    irqreturn_t ret = IRQ_NONE;
    struct FPGA_JPEG *dev_priv = (struct FPGA_JPEG *) dev_id;
    u32 fifo_s = readl(dev_priv->err_reg_base + REG_ERR_FIFO);
    u32 no_resp = readl(dev_priv->err_reg_base + REG_ERR_NORESP);

    if (fifo_s & (1ul << 12))
    {
        printk(KERN_WARNING "IQR[%d]: jpeg2addr fifo in fpga error\r\n", irq);
        ret = IRQ_HANDLED;
    }

    if (no_resp & (1ul << 1))
    {
        printk(
        KERN_WARNING "IQR[%d]: jpeg encode done interrupt not response\r\n",
            irq);
        ret = IRQ_HANDLED;
    }

    if (ret == IRQ_HANDLED)
    {
        dev_priv->result.size = 0;
        dev_priv->result.status = N_JPEG_ENCODE_RESULT_ERROR;
        dev_priv->interruped = 1;
        wake_up_interruptible(&dev_priv->r_wait);
    }
    return ret;
}

static irqreturn_t _irq_jpeg_done(int irq, void *dev_id)
{
    irqreturn_t ret = IRQ_HANDLED;
    struct FPGA_JPEG *dev_priv = (struct FPGA_JPEG *) dev_id;
    u32 size = readl(dev_priv->jpeg_reg_base + REG_JPEG_SIZE);
    writel(0, dev_priv->jpeg_reg_base + REG_JPEG_STATUS);
    dev_priv->result.size = size;
    dev_priv->result.status = N_JPEG_ENCODE_RESULT_SUCCEED;
    dev_priv->interruped = 1;
    wake_up_interruptible(&dev_priv->r_wait);
    return ret;
}

static int _open(struct inode *i, struct file *f)
{
    int ret = 0;
    if (_open_flag == 0)
    {
        f->private_data = _dev_priv;
        dev_info(_dev_priv->class_dev, "open file OK\n");
        _open_flag = 1;
    }
    else
    {
        dev_warn(_dev_priv->class_dev,
            "device has been used, open file failed\n");
        ret = -EBUSY;
    }
    return ret;
}

static int _release(struct inode *i, struct file *f)
{
    _open_flag = 0;
    dev_info(_dev_priv->class_dev, "close file OK\n");
    return 0;
}

static ssize_t _read(struct file *f, char __user *buf, size_t size,
    loff_t * offset)
{
    int ret = 0;
    struct FPGA_JPEG *dev_priv = (struct FPGA_JPEG *) f->private_data;
    if (dev_priv->interruped)
    {
        if (size < sizeof(dev_priv->result))
        {
            dev_warn(_dev_priv->class_dev, "input buffer size is too small\n");
            ret = -ENOMEM;
        }
        else
        {
            if (copy_to_user(buf, &dev_priv->result, sizeof(dev_priv->result)))
            {
                dev_warn(_dev_priv->class_dev, "copy data to user failed\n");
                ret = -EFAULT;
            }
            else
                ret = sizeof(dev_priv->result);
        }
        dev_priv->interruped = 0;
    }
    return ret;
}

static ssize_t _write(struct file *f, const char __user *data, size_t len,
    loff_t *offset)
{
    return 0;
}

static unsigned int _poll(struct file *f, struct poll_table_struct *pt)
{
    unsigned int mask = 0;
    struct FPGA_JPEG *dev_priv = f->private_data;

    poll_wait(f, &dev_priv->r_wait, pt);

    if (dev_priv->interruped)
        mask |= POLLIN | POLLRDNORM;

    return mask;
}

static loff_t _llseek(struct file *f, loff_t offset, int orig)
{
    return 0;
}

static long _unlocked_ioctl(struct file *f, unsigned int cmd, unsigned long arg)
{
    int ret;
    struct FPGA_JPEG *dev_priv = (struct FPGA_JPEG *) f->private_data;
    switch (cmd)
    {
    case IOC_SET_JPEG_QUANTIZATION:
    {
        struct QUANTIZATION a;
        ret = copy_from_user(&a, (const void __user *) arg,
            sizeof(struct QUANTIZATION));
        if (ret == 0)
        {
            ret = _pl_set_quantization(dev_priv->jpeg_reg_base, &a);
        }
    }
        break;
    case IOC_DO_JPEG_ENCODE:
    {
        struct JPEG_ENCODE_PARAM p;
        ret = copy_from_user(&p, (const void __user *) arg,
            sizeof(struct JPEG_ENCODE_PARAM));
        if (ret == 0)
        {
            ret = _pl_do_encode(dev_priv->jpeg_reg_base, &p);
        }
    }
        break;
    default:
        ret = -EPERM;
        break;
    }
    return ret;
}

static const struct file_operations _fops =
{
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
    int i, irq, irq_num;
    struct resource *r_mem; /* IO mem resources */
    struct device *dev = &pdev->dev;

    dev_info(dev, "probe %s (%s)\r\n", VERSION, _build_time);

    _dev_priv = devm_kzalloc(&pdev->dev, sizeof(struct FPGA_JPEG), GFP_KERNEL);
    if (_dev_priv == NULL)
    {
        dev_err(dev, "kzmalloc memory failed\r\n");
        return -ENOMEM;
    }

    r_mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    _dev_priv->jpeg_reg_base = devm_ioremap_resource(&pdev->dev, r_mem);
    if (IS_ERR(_dev_priv->jpeg_reg_base))
    {
        /* don't need to release the devm_* resources */
        return PTR_ERR(_dev_priv->jpeg_reg_base);
    }

    /* error register is shared, don't need to request region here */
    r_mem = platform_get_resource(pdev, IORESOURCE_MEM, 1);
    _dev_priv->err_reg_base = devm_ioremap(&pdev->dev, r_mem->start, resource_size(r_mem));
    if (IS_ERR(_dev_priv->err_reg_base))
    {
        /* don't need to release the devm_* resources */
        return PTR_ERR(_dev_priv->err_reg_base);
    }

    irq_num = platform_irq_count(pdev);
    if (irq_num > _irq_porc_num)
    {
        dev_warn(dev, "irq proc number mismatch: dev=%d, dir=%d(use)\r\n",
            irq_num, _irq_porc_num);
        irq_num = _irq_porc_num;
    }

    for (i = 0; i < irq_num; i++)
    {
        const IRQ_PROC_T *h = &_irq_vic[i];
        if (h->irq_pf)
        {
            irq = platform_get_irq(pdev, i);
            if (irq < 0)
            {
                dev_err(dev, "no irq resource\n");
                return irq;
            }

            dev_info(dev, "IRQ[%d]: %s is %d\r\n", i, h->name, irq);
            ret = devm_request_threaded_irq(&pdev->dev, irq, h->irq_pf,
                h->irq_th_pf, h->flag, dev_name(&pdev->dev), _dev_priv);
            if (ret)
            {
                dev_err(dev, "request irq failed\n");
                return ret;
            }
        }
    }

    _dev_priv->dev_class = class_create(THIS_MODULE, "fpga_jpeg_dev");
    if ((IS_ERR(_dev_priv->dev_class)))
    {
        ret = PTR_ERR(_dev_priv->class_dev);
        dev_err(dev, "class_create failed\n");
        goto fail_class_create_exit;
    }

    _dev_priv->class_dev = device_create(_dev_priv->dev_class, NULL, _devno,
    NULL, "fpga_jpeg");
    if (IS_ERR(_dev_priv->class_dev))
    {
        ret = PTR_ERR(_dev_priv->class_dev);
        dev_err(dev, "device_create failed\n");
        goto fail_device_create_exit;
    }
    init_waitqueue_head(&_dev_priv->r_wait);
    platform_set_drvdata(pdev, _dev_priv);
    _pl_init(_dev_priv->jpeg_reg_base);
    return ret;

fail_device_create_exit:
    class_destroy(_dev_priv->dev_class);

fail_class_create_exit:
    platform_set_drvdata(pdev, NULL);
    return ret;
}

static int _remove(struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    struct FPGA_JPEG * dev_priv = platform_get_drvdata(pdev);
    _pl_deinit(dev_priv->jpeg_reg_base);
    device_destroy(dev_priv->dev_class, _devno);
    class_destroy(dev_priv->dev_class);
    platform_set_drvdata(pdev, NULL);
    dev_info(dev, "remove\r\n");
    return 0;
}

static struct of_device_id _of_match[] =
{
    { .compatible = "fpga-jpeg", },
    { /* end of list */},
};
MODULE_DEVICE_TABLE(of, _of_match);

static struct platform_driver _pl_driver =
{
    .driver =
    {
        .name = "fpga-jpeg",
        .owner = THIS_MODULE,
        .of_match_table = _of_match,
    },
    .probe = _probe,
    .remove = _remove,
};

static int __init _jpeg_init(void)
{
    int ret = 0;
    printk("==== JPEG MODULE INIT ====\r\n");
    cdev_init(&_cdev, &_fops);
    ret = alloc_chrdev_region(&_devno, 0, 1, "fpga-jpeg");
    if (ret == 0)
    {
        ret = cdev_add(&_cdev, _devno, 1);
        if (ret == 0)
        {
            ret = platform_driver_register(&_pl_driver);
            if (ret)
            {
                cdev_del(&_cdev);
                printk(KERN_ERR "platform_driver_register error\r\n");
            }
        }
        else
        {
            unregister_chrdev_region(_devno, 1);
            printk(KERN_ERR "cdev_add error\r\n");
        }
    }
    else
    {
        printk(KERN_ERR "register_chrdev_region error\r\n");
    }
    return ret;
}

static void __exit _jpeg_exit(void)
{
    cdev_del(&_cdev);
    unregister_chrdev_region(_devno, 1);
    platform_driver_unregister(&_pl_driver);
    printk("==== JPEG MODULE EXIT ====\r\n");
}

module_init(_jpeg_init);
module_exit(_jpeg_exit);

MODULE_AUTHOR("Lay Zhan");
MODULE_DESCRIPTION("zynq fpga jpeg module driver of prague");
MODULE_LICENSE("GPL v2");
