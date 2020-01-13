/*
 * main.c
 *
 *  Created on: Jan 2, 2020
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

#include "../include/fpga_peak.h"

#define VERSION                         "0.0.0.1"

// peak reg group
#define REG_PEAK_CTRL                   0x0000
#define REG_PEAK_IRQ_CTRL               0x0004
#define REG_PEAK_IRQ_STATUS             0x0008
#define REG_PEAK_RAWADDR_M1             0x0010
#define REG_PEAK_RAWADDR_M2             0x0014
#define REG_PEAK_RAWADDR_M3             0x0018
#define REG_PEAK_BACKADDR_M1            0x0040
#define REG_PEAK_BACKADDR_M2            0x0044
#define REG_PEAK_BACKADDR_M3            0x0048
#define REG_PEAK_BACK_PARAM             0x004C
#define REG_PEAK_DETECT_IN              0x0050
#define REG_PEAK_DETECT_OUT             0x0054
#define REG_PEAK_RESULT_SIZE            0x0058
#define REG_PEAK_DECT_PARAM             0x005C

// err reg group
#define REG_ERR_FIFO                    0x0004
#define REG_ERR_NORESP                  0x0008

enum dev_type
{
    N_DEV_TYPE_BACK_RM,
    N_DEV_TYPE_DETECT
};

struct FPGA_PEAK_COMM
{
    struct mutex ctrl_reg_mutex;
    void __iomem *peak_reg_base;
    void __iomem *err_reg_base;
    struct class *device_class;
};

struct FPGA_PEAK_DEV
{
    u32 type;
    char *name;
};

union FPGA_PEAK_RESULT
{
    struct PEAK_DETECT_RESULT detect;
    struct PEAK_BACK_RM_RESULT back_rm;
};

struct FPGA_PEAK
{
    struct cdev cdev;
    struct FPGA_PEAK_DEV *dev;
    struct FPGA_PEAK_COMM *comm;
    union FPGA_PEAK_RESULT result;
    struct device *sys_dev;
    wait_queue_head_t r_wait;
    u32 interruped;
    u32 open_flag;
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
static struct FPGA_PEAK *_dev_priv;
static int _major;

static struct FPGA_PEAK_DEV _dev_list[] =
{
    { N_DEV_TYPE_BACK_RM, "bk_rm" },
    { N_DEV_TYPE_DETECT, "detect" }
};
static const int _dev_num = ARRAY_SIZE(_dev_list);
static const int _dev_priv_size = sizeof(struct FPGA_PEAK) * ARRAY_SIZE(_dev_list) + sizeof(struct FPGA_PEAK_COMM);

#define GET_DEV_PRIV_COMM_PTR(dev_priv, dev_num) \
    (struct FPGA_PEAK_COMM *)(((u8 *)(dev_priv)) + sizeof(struct FPGA_PEAK) * (dev_num))\

static irqreturn_t _irq_error(int irq, void *dev_id);
static irqreturn_t _irq_peak_detect_done(int irq, void *dev_id);
static irqreturn_t _irq_peak_back_rm_done(int irq, void *dev_id);

static const IRQ_PROC_T _irq_vic[] =
{
    { _irq_error, NULL, "ERROR_INT", IRQF_SHARED }, /* 63 */
    { _irq_peak_back_rm_done, NULL, "BACKGROUND_REMOVAL_DONE", 0 }, /* 87 */
    { _irq_peak_detect_done, NULL, "PEAK_DETECTION_DONE", 0 }, /* 88 */
};
static const int _irq_porc_num = ARRAY_SIZE(_irq_vic);

static irqreturn_t _irq_error(int irq, void *dev_id)
{
    irqreturn_t ret = IRQ_NONE;
    // TODO FPGA not supports on this version
    return ret;
}

static irqreturn_t _irq_peak_detect_done(int irq, void *dev_id)
{
    irqreturn_t ret = IRQ_HANDLED;
    u32 size;
    struct FPGA_PEAK *dev_priv = ((struct FPGA_PEAK *) dev_id) + 1;
    struct FPGA_PEAK_COMM *comm = dev_priv->comm;

    size = readl(comm->peak_reg_base + REG_PEAK_RESULT_SIZE);
    writel(0b01, comm->peak_reg_base + REG_PEAK_IRQ_CTRL);

    dev_priv->result.detect.size = size;
    dev_priv->result.detect.r_type = N_PEAK_RESULT_TYPE_SUCCEED;
    dev_priv->interruped = 1;
    wake_up_interruptible(&dev_priv->r_wait);

    return ret;
}

static irqreturn_t _irq_peak_back_rm_done(int irq, void *dev_id)
{
    u32 param;
    irqreturn_t ret = IRQ_HANDLED;
    struct FPGA_PEAK *dev_priv = (struct FPGA_PEAK *) dev_id;
    struct FPGA_PEAK_COMM *comm = dev_priv->comm;

    param = readl(comm->peak_reg_base + REG_PEAK_BACK_PARAM);
    writel(0b10, comm->peak_reg_base + REG_PEAK_IRQ_CTRL);

    dev_priv->result.back_rm.histogram_param = param;
    dev_priv->result.detect.r_type = N_PEAK_RESULT_TYPE_SUCCEED;
    dev_priv->interruped = 1;
    wake_up_interruptible(&dev_priv->r_wait);

    return ret;
}

static void _pl_init(void __iomem *reg_base)
{
    u32 val = readl(reg_base + REG_PEAK_CTRL) | (3ul << 2);
    writel(val, reg_base + REG_PEAK_CTRL);
    writel(0, reg_base + REG_PEAK_IRQ_CTRL);
}

static void _pl_deinit(void __iomem *reg_base)
{
    u32 val = readl(reg_base + REG_PEAK_CTRL) & (~(3ul << 2));
    writel(val, reg_base + REG_PEAK_CTRL);
}

static int _pl_do_peak_detect(struct FPGA_PEAK *dev_priv,
    const struct PEAK_DETECT_PARAM *param)
{
    int ret = 0;
    u32 val;
    void __iomem *reg_base = dev_priv->comm->peak_reg_base;

    writel(param->out_addr, reg_base + REG_PEAK_DETECT_OUT);
    writel(param->in_addr, reg_base + REG_PEAK_DETECT_IN);

    if (param->type == N_PEAK_IMG_TYPE_ML)
    {
        writel(param->histogram_param, reg_base + REG_PEAK_DECT_PARAM);
    }

    // need lock
    mutex_lock(&(dev_priv->comm->ctrl_reg_mutex));
    val = (readl(reg_base + REG_PEAK_CTRL) & (~3ul));
    switch (param->type)
    {
    case N_PEAK_IMG_TYPE_CL:
        val &= ~(1 << 4);
        break;
    case N_PEAK_IMG_TYPE_ML:
        val |= (1 << 4);
        break;
    default:
        ret = -1;
        break;
    }
    if (ret == 0)
    {
        writel(val | (1ul << 1), reg_base + REG_PEAK_CTRL);
    }
    // need unlock
    mutex_unlock(&(dev_priv->comm->ctrl_reg_mutex));

    return ret;
}

static int _pl_do_peak_back_rm(struct FPGA_PEAK *dev_priv,
    const struct PEAK_BACK_RM_PARAM *param)
{
    int ret = 0;
    u32 val;
    void __iomem *reg_base = dev_priv->comm->peak_reg_base;

    switch (param->type)
    {
    case N_PEAK_IMG_TYPE_CL:
        writel(param->in_addr.cl[1], reg_base + REG_PEAK_RAWADDR_M2);
        writel(param->in_addr.cl[0], reg_base + REG_PEAK_RAWADDR_M1);

        writel(param->out_addr.cl[1], reg_base + REG_PEAK_BACKADDR_M2);
        writel(param->out_addr.cl[0], reg_base + REG_PEAK_BACKADDR_M1);
        break;
    case N_PEAK_IMG_TYPE_ML:
        writel(param->in_addr.ml[2], reg_base + REG_PEAK_RAWADDR_M3);
        writel(param->in_addr.ml[1], reg_base + REG_PEAK_RAWADDR_M2);
        writel(param->in_addr.ml[0], reg_base + REG_PEAK_RAWADDR_M1);

        writel(param->out_addr.ml[2], reg_base + REG_PEAK_BACKADDR_M3);
        writel(param->out_addr.ml[1], reg_base + REG_PEAK_BACKADDR_M2);
        writel(param->out_addr.ml[0], reg_base + REG_PEAK_BACKADDR_M1);
        break;
    default:
        return -1;
    }

    // need lock
    mutex_lock(&(dev_priv->comm->ctrl_reg_mutex));
    val = (readl(reg_base + REG_PEAK_CTRL) & (~3ul));
    switch (param->type)
    {
    case N_PEAK_IMG_TYPE_CL:
        val |= (1 << 6);
        break;
    case N_PEAK_IMG_TYPE_ML:
        val &= ~(1 << 6);
        break;
    default:
        ret = -1;
        break;
    }

    if (ret == 0)
    {
        writel(val | 1, reg_base + REG_PEAK_CTRL);
    }

    // need unlock
    mutex_unlock(&(dev_priv->comm->ctrl_reg_mutex));

    return ret;
}

static int _open(struct inode *i, struct file *f)
{
    int ret = 0;
    struct FPGA_PEAK *dev_priv = container_of(i->i_cdev, struct FPGA_PEAK,
        cdev);

    if (dev_priv->open_flag == 0)
    {
        f->private_data = dev_priv;
        dev_info(dev_priv->sys_dev, "open file OK\n");
        dev_priv->open_flag = 1;
    }
    else
    {
        dev_warn(dev_priv->sys_dev, "device has been used, open file failed\n");
        ret = -EBUSY;
    }

    return ret;
}

static int _release(struct inode *i, struct file *f)
{
    struct FPGA_PEAK *dev_priv = (struct FPGA_PEAK *) f->private_data;
    dev_priv->open_flag = 0;
    dev_info(dev_priv->sys_dev, "close file OK\n");
    return 0;
}

static ssize_t _read(struct file *f, char __user *buf, size_t size,
    loff_t * offset)
{
    int ret = -1;
    struct FPGA_PEAK *dev_priv = (struct FPGA_PEAK *) f->private_data;
    const void *result_data = NULL;
    size_t result_size = 0;

    switch (dev_priv->dev->type)
    {
    case N_DEV_TYPE_BACK_RM:
        result_data = &(dev_priv->result.back_rm);
        result_size = sizeof(dev_priv->result.back_rm);
        break;
    case N_DEV_TYPE_DETECT:
        result_data = &(dev_priv->result.detect);
        result_size = sizeof(dev_priv->result.detect);
        break;
    default:
        break;
    }

    if (dev_priv->interruped && result_data)
    {
        if (size < result_size)
        {
            dev_warn(dev_priv->sys_dev, "input buffer size is too small\n");
            ret = -ENOMEM;
        }
        else
        {
            if (copy_to_user(buf, result_data, result_size))
            {
                dev_warn(dev_priv->sys_dev, "copy data to user failed\n");
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

    struct FPGA_PEAK *dev_priv = f->private_data;

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
    int ret = 0;
    struct FPGA_PEAK *dev_priv = (struct FPGA_PEAK *) f->private_data;

    switch (dev_priv->dev->type)
    {
    case N_DEV_TYPE_BACK_RM:
        switch (cmd)
        {
        case IOC_DO_PEAK_BACK_RM:
        {
            struct PEAK_BACK_RM_PARAM p;
            ret = copy_from_user(&p, (const void __user *) arg,
                sizeof(struct PEAK_BACK_RM_PARAM));
            if (ret == 0)
            {
                ret = _pl_do_peak_back_rm(dev_priv, &p);
            }
            else
            {
                dev_warn(dev_priv->sys_dev, "copy from user failed\n");
            }
        }
            break;
        default:
            dev_warn(dev_priv->sys_dev, "invalid cmd of ioctl\n");
            ret = -EPERM;
            break;
        }
        break;
    case N_DEV_TYPE_DETECT:
        switch (cmd)
        {
        case IOC_DO_PEAK_DETECT:
        {
            struct PEAK_DETECT_PARAM p;
            ret = copy_from_user(&p, (const void __user *) arg,
                sizeof(struct PEAK_DETECT_PARAM));
            if (ret == 0)
            {
                ret = _pl_do_peak_detect(dev_priv, &p);
            }
            else
            {
                dev_warn(dev_priv->sys_dev, "copy from user failed\n");
            }
        }
            break;
        default:
            dev_warn(dev_priv->sys_dev, "invalid cmd of ioctl\n");
            ret = -EPERM;
            break;
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
    struct FPGA_PEAK_COMM *comm = _dev_priv->comm;
    dev_info(dev, "probe %s (%s)\r\n", VERSION, _build_time);

    r_mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    comm->peak_reg_base = devm_ioremap_resource(&pdev->dev, r_mem);
    if (IS_ERR(comm->peak_reg_base))
    {
        /* don't need to release the devm_* resources */
        return PTR_ERR(comm->peak_reg_base);
    }

    /* error register is shared, don't need to request region here */
    r_mem = platform_get_resource(pdev, IORESOURCE_MEM, 1);
    comm->err_reg_base = devm_ioremap(&pdev->dev, r_mem->start, resource_size(r_mem));
    if (IS_ERR(comm->err_reg_base))
    {
        /* don't need to release the devm_* resources */
        return PTR_ERR(comm->err_reg_base);
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

    comm->device_class = class_create(THIS_MODULE, "fpga_peak_dev");
    if ((IS_ERR(comm->device_class)))
    {
        ret = PTR_ERR(comm->device_class);
        dev_err(dev, "class_create failed\n");
        goto fail_class_create_exit;
    }

    for (i = 0; i < _dev_num; i++)
    {
        char name[32];
        snprintf(name, sizeof(name), "fpga_peak_%s", _dev_priv[i].dev->name);
        _dev_priv[i].sys_dev = device_create(
            comm->device_class,
            NULL,
            MKDEV(_major, i),
            NULL,
            name);
        if (IS_ERR(_dev_priv[i].sys_dev))
        {
            ret = PTR_ERR(_dev_priv[i].sys_dev);
            dev_err(dev, "device_create[%d] failed\n", i);
            goto fail_device_create_exit;
        }

        init_waitqueue_head(&_dev_priv[i].r_wait);
    }

    platform_set_drvdata(pdev, _dev_priv);
    _pl_init(comm->peak_reg_base);
    mutex_init(&(comm->ctrl_reg_mutex));
    return ret;

fail_device_create_exit:
    for (i = 0; i < _dev_num; i++)
    {
        if (!IS_ERR(_dev_priv[i].sys_dev))
        {
            device_destroy(comm->device_class, MKDEV(_major, i));
            _dev_priv[i].sys_dev = NULL;
        }
    }
    class_destroy(comm->device_class);

fail_class_create_exit:
    platform_set_drvdata(pdev, NULL);
    return ret;
}

static int _remove(struct platform_device *pdev)
{
    int i;
    struct device *dev = &pdev->dev;
    struct FPGA_PEAK * dev_priv = platform_get_drvdata(pdev);
    struct FPGA_PEAK_COMM *comm = dev_priv->comm;
    mutex_destroy(&(comm->ctrl_reg_mutex));
    _pl_deinit(comm->peak_reg_base);
    for (i = 0; i < _dev_num; i++)
    {
        device_destroy(comm->device_class, MKDEV(_major, i));
        _dev_priv[i].sys_dev = NULL;
    }
    class_destroy(comm->device_class);
    platform_set_drvdata(pdev, NULL);
    dev_info(dev, "remove\r\n");
    return 0;
}

static struct of_device_id _of_match[] =
{
    { .compatible = "fpga-peak", },
    { /* end of list */},
};
MODULE_DEVICE_TABLE(of, _of_match);

static struct platform_driver _pl_driver =
{
    .driver =
    {
        .name = "fpga-peak",
        .owner = THIS_MODULE,
        .of_match_table = _of_match,
    },
    .probe = _probe,
    .remove = _remove,
};

static int _setup_cdev(struct FPGA_PEAK *dev, int major, int index)
{
    int err, devno = MKDEV(major, index);
    cdev_init(&dev->cdev, &_fops);
    dev->cdev.owner = THIS_MODULE;
    err = cdev_add(&dev->cdev, devno, 1);
    if (err)
    {
        printk(KERN_NOTICE "Error %d adding fpga-peak[%d]\r\n", err, index);
    }
    else
    {
        printk(KERN_INFO "add char device[%d:%d]\r\n", major, index);
    }
    return err;
}

static int __init _peak_init(void)
{
    int ret = 0;
    int i;
    dev_t devno;
    printk("==== PEAK MODULE INIT ====\r\n");
    ret = alloc_chrdev_region(&devno, 0, _dev_num, "fpga-peak");
    if (ret == 0)
    {
        _major = MAJOR(devno);
        _dev_priv = kzalloc(_dev_priv_size, GFP_KERNEL);
        if (!_dev_priv)
        {
            ret = -ENOMEM;
            printk(KERN_ERR "kzalloc error\r\n");
            goto fail_malloc;
        }

        printk(KERN_INFO "dev_priv = %p\r\n", _dev_priv);
        for (i = 0; i < _dev_num; i++)
        {
            if (_setup_cdev(&_dev_priv[i], _major, i) == 0)
            {
                _dev_priv[i].comm = GET_DEV_PRIV_COMM_PTR(_dev_priv, _dev_num);
                _dev_priv[i].dev = &_dev_list[i];
                printk(KERN_INFO "%s[%p]: comm = %p\r\n",
                    _dev_priv[i].dev->name,
                    &_dev_priv[i],
                    _dev_priv[i].comm);
            }
            else
            {
                printk(KERN_ERR "_setup_cdev error\r\n");
                goto fail_setup_cdev;
            }
        }
    }
    ret = platform_driver_register(&_pl_driver);
    return ret;

fail_setup_cdev:
    for (i = 0; i < _dev_num; i++)
    {
        if (_dev_priv[i].dev)
        {
            cdev_del(&_dev_priv[i].cdev);
        }
    }
    kfree(_dev_priv);

fail_malloc:
    unregister_chrdev_region(devno, _dev_num);
    return ret;
}

static void __exit _peak_exit(void)
{
    int i;
    for (i = 0; i < _dev_num; i++)
    {
        cdev_del(&_dev_priv[i].cdev);
    }
    printk(KERN_INFO "delete dev_priv(%p)\r\n", _dev_priv);
    kfree(_dev_priv);
    unregister_chrdev_region(MKDEV(_major, 0), _dev_num);

    platform_driver_unregister(&_pl_driver);
    printk("==== PEAK MODULE EXIT ====\r\n");
}

module_init(_peak_init);
module_exit(_peak_exit);

MODULE_AUTHOR("Lay Zhan");
MODULE_DESCRIPTION("zynq fpga peak module driver of prague");
MODULE_LICENSE("GPL v2");
