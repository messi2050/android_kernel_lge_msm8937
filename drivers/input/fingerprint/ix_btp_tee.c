/* IX BTP Sensor Driver
 *
 * Copyright (c) 2015 Crucialsoft Fingerprint <thkim@crucialtec.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License Version 2
 * as published by the Free Software Foundation.
 */

#define DEBUG

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/of.h>
#include <linux/cdev.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/poll.h>
#include <linux/input.h>
#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#elif defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
#endif
#include <linux/spi/spi.h>
#include <linux/regulator/consumer.h>
#include <linux/wakelock.h>
#include <linux/jiffies.h>

#include "ix_btp.h"
#include "ix_log.h"


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Crucialsoft Fingerprint <www.crucialtec.co.kr>");
MODULE_DESCRIPTION("IX BTP sensor driver.");

#define CONFIG_COMPAT 1
//#define LGE_FINGERPRINT_NO_TTW 1 // disable Touch To Wakeup


/* -------------------------------------------------------------------- */
/* ix driver constants                                                  */
/* -------------------------------------------------------------------- */
#define IX_MAJOR                        250
#define IX_DEV_NAME                     "btp"
#define IX_CLASS_NAME                   "ixsensor"
#define IX_INTERRUPT                    REL_MISC
#define IX_SPI_CLOCK_SPEED              (48 * 100000U)
#define IX_DEFAULT_IRQ_TIMEOUT          (100 * HZ / 1000)
#define IX_RESET_RETRIES				2
#define CS_CHANGE_VALUE                 0
#define IX_WAKELOCK_HOLD_TIME           1000


/* -------------------------------------------------------------------- */
/* ix sensor information                                                */
/* -------------------------------------------------------------------- */
#define IX_HW_ID_SP2                    0x52
#define MAX_RETRY_HW_CHECK_COUNT        5
#define RETRY_WAIT_COUNT    100

/* -------------------------------------------------------------------- */
/* ix spi read/write command                                            */
/* -------------------------------------------------------------------- */
#define IX_SPI_READ_COMMAND             0xA8
#define IX_SPI_WRITE_COMMAND            0xA9

/* CMD ID collected from TzBlspAC.h to change Ownership */
#define TZ_BLSP_MODIFY_OWNERSHIP_ID     3
#define TZBSP_APSS_ID                   3
#define TZBSP_TZ_ID                     1


/* -------------------------------------------------------------------- */
/* ix data types                                                        */
/* -------------------------------------------------------------------- */
struct ix_info{
    u8 chipID;
    u8 revNO;
    u8* version;
    int imageWidth;
    int imageHeight;
    int imageSize;
    int frameMax;
};

struct ix_data {
    struct spi_device *spi;
	struct input_dev *input;
    struct class *class;
    struct device *device;
    struct cdev cdev;
    struct ix_info info;

    struct ix_platform_data *platform_pdata;
    struct wake_lock ttw_wakelock;

    dev_t devno;

    u32 qup_id;
    u32 reset_gpio;
    u32 irq_gpio;
    u32 cs_gpio;
    u32 irq;

    bool power_on;
    bool pipe_owner;

#if defined(CONFIG_FB)
    struct notifier_block fb_notifier;
#endif
};


/* -------------------------------------------------------------------- */
/* global variables                                                     */
/* -------------------------------------------------------------------- */
static int ix_device_count;


/* -------------------------------------------------------------------- */
/* ix init value                                                        */
/* -------------------------------------------------------------------- */
static unsigned char init_seq_value_sp2[][2] = {
    {0xFC, 0xAA}, {0xFC, 0x55}, {0x40, 0x7B}, {0x41, 0xFF}, {0x45, 0x1D},
    {0x46, 0x25}, {0x47, 0xFF}, {0x48, 0x0C}, {0x49, 0x01}, {0x4D, 0x30},
    {0x4E, 0x02}, {0x4F, 0x00}, {0x50, 0x12}, {0x51, 0x07}, {0x52, 0xA9},
    {0x53, 0x00}, {0x54, 0xE2}, {0x55, 0x7F},
    {0x56, 0x00}, //default 0x00 low current
    {0x57, 0x2C}, //default 0x00 spi margin
    {0x58, 0x80}, {0x59, 0x00}, {0x5A, 0x00}, {0x5B, 0x6E}, {0x5C, 0x15},
    {0x5D, 0x1F}, {0x5E, 0x00}, {0x5F, 0x00}, {0x60, 0x00}, {0x61, 0x5A},
    {0x62, 0x00}, {0x63, 0x5A}, {0x64, 0x00}, {0x65, 0x5A}, {0x66, 0x40},
    {0x67, 0x66}, {0x68, 0x0E}, {0x69, 0x2A}, {0x6A, 0x2F}, {0x6B, 0x3D},
    {0x6C, 0x2B}, {0x6D, 0x2E}, {0x6E, 0x42}, {0x6F, 0x53}, {0x70, 0x04},
    {0x71, 0x07}, {0x72, 0x08}, {0x73, 0x12}, {0x74, 0x02}, {0x75, 0x13},
    {0x76, 0x09}, {0x77, 0x13}, {0x78, 0x05}, {0x79, 0x08}, {0x7A, 0x0F},
    {0x7B, 0x00}, {0x7C, 0x00}, {0x80, 0x00}, {0x81, 0x66}, {0x82, 0x1B},
    {0x83, 0x00}, {0x84, 0x03}, {0x86, 0x10}, {0x8E, 0x00}, {0x8F, 0x00},
    {0x57, 0x2C}, {0x38, 0xc1}, {0x3A, 0x01}, {0x3D, 0x00}, {0x39, 0x00},
    {0x3B, 0x10}, {0x3C, 0xFB}, {0x10, 0x40}, {0x12, 0x30}, {0x13, 0x30},
    {0x24, 0x80}, {0x25, 0x05}, {0xA0, 0x10},
};


/* -------------------------------------------------------------------- */
/* ix sensor commands and registers                                     */
/* -------------------------------------------------------------------- */
typedef enum {
    /* commands */
    IX_CMD_SRESET           = 0x02,   /* Soft reset */
    IX_CMD_IDLE             = 0x03,   /* Go to idle state*/
    IX_CMD_SCAN             = 0x04,   /* Go to scan state */
    IX_CMD_DET              = 0x05,   /* Go to detect state */
    IX_CMD_READ             = 0x06,   /* SRAM memory data read */
    IX_CMD_INT_R            = 0x08,   /* Interrupt read */
    IX_CMD_INT_RC           = 0x09,   /* Interrupt read & clear */
    /* registers */
    IX_REG_CHIP_ID          = 0xB9,
    IX_REG_REVISION_NUMBER  = 0xBA,
    IX_REG_VREF_SET2        = 0x54,
    IX_REG_PGA_SET0         = 0x5B,
    IX_REG_PGA_SET1         = 0x5C,
    IX_REG_CELL_AMP_SET     = 0x5D,
    IX_REG_SENS_CTRL        = 0x10,
    IX_REG_INT_SRC_MASK     = 0x2E,
    IX_REG_IMG_DET_CTRL     = 0x38,
    IX_REG_IMG_DET_L_THRES  = 0x3D,
} ix_cmd_reg_t;


/* -------------------------------------------------------------------- */
/* function prototypes                                                  */
/* -------------------------------------------------------------------- */
static int ix_init(void);
static void ix_exit(void);
static int ix_probe(struct spi_device *spi);
static int ix_remove(struct spi_device *spi);
static int ix_suspend(struct device *dev);
static int ix_resume(struct device *dev);	
static int ix_open(struct inode *inode, struct file *file);
static int ix_release(struct inode *inode, struct file *file);
static int ix_spi_write_cmd(struct ix_data *ix, ix_cmd_reg_t addr);
static int ix_spi_write_reg(struct ix_data *ix, ix_cmd_reg_t addr, u8 value);
static int ix_spi_write_2_btye_reg(struct ix_data *ix, ix_cmd_reg_t addr, u8 value);
static int ix_spi_read_reg(struct ix_data *ix, ix_cmd_reg_t addr, u8 *rx);
static int ix_reset(struct ix_data *ix);
static int ix_init_param(struct ix_data *ix);
static int ix_get_hw_id(struct ix_data *ix);
static int ix_check_interrupt(struct ix_data *ix);

#ifdef CONFIG_OF
static int ix_get_of_pdata(struct device *dev, struct ix_platform_data *pdata);
#endif
#if defined(CONFIG_FB)
static int fb_notifier_suspend(struct notifier_block *self,unsigned long event, void *data);
#endif
irqreturn_t ix_interrupt(int irq, void *_ix);
static int ix_cleanup(struct ix_data *ix);


/* -------------------------------------------------------------------- */
/* External interface                                                   */
/* -------------------------------------------------------------------- */
late_initcall(ix_init);
module_exit(ix_exit);

static const struct dev_pm_ops ix_pm = {
	.suspend = ix_suspend,
	.resume = ix_resume
};

#ifdef CONFIG_OF
static struct of_device_id ix_of_match[] = {
  { .compatible = "ix,btp", },
  {},
};

MODULE_DEVICE_TABLE(of, ix_of_match);
#endif

static struct spi_driver ix_driver = {
    .driver = {
        .name   = IX_DEV_NAME,
        .bus    = &spi_bus_type,
        .owner  = THIS_MODULE,
#ifdef LGE_FINGERPRINT_NO_TTW	// not support TTW (Touch To Wakeup)
        .pm     = &ix_pm,
#endif
#ifdef CONFIG_OF
        .of_match_table = ix_of_match,
#endif
    },
    .probe  = ix_probe,
    .remove = ix_remove
};

static const struct file_operations ix_fops = {
    .owner          = THIS_MODULE,
    .open           = ix_open,
    .release        = ix_release,
};

/* -------------------------------------------------------------------- */

static ssize_t ix_show_qup_id(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct ix_data *ix = dev_get_drvdata(dev);
    return sprintf(buf, "%d\n", ix->qup_id);
}

static DEVICE_ATTR(qup_id, S_IRUGO,
		   ix_show_qup_id, NULL);

/* -------------------------------------------------------------------- */
static ssize_t ix_store_spi_prepare_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{

    struct ix_data *ix = dev_get_drvdata(dev);
	int res = 0;
	bool to_tz;

	if(*buf == '1')
	  to_tz = true;
	else if(*buf == '0')
	  to_tz = false;
	else
	  return -EINVAL;

#if 0	// moved to TZ
	res = spi_set_prepare(ix, to_tz);		
#else
	/* set spi ownership flag */
	ix->pipe_owner = to_tz;
#endif

	return res ? res : count;
}

static ssize_t ix_show_spi_prepare(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct ix_data *ix = dev_get_drvdata(dev);

	if(ix->pipe_owner)
	  return sprintf(buf, "%d \n", TZBSP_TZ_ID);
	else
	  return sprintf(buf, "%d \n", TZBSP_APSS_ID);
}

static DEVICE_ATTR(spi_prepare, S_IRUGO | S_IWUSR,
		ix_show_spi_prepare, ix_store_spi_prepare_set);


static struct attribute *ix_attributes[] = {
	&dev_attr_qup_id.attr,
	&dev_attr_spi_prepare.attr,
	NULL
};

static const struct attribute_group ix_attr_group = {
	.attrs = ix_attributes,
};


/* -------------------------------------------------------------------- */

static int ix_regulator_init(struct ix_data *ix,
                    struct ix_platform_data *pdata)
{
    int error = 0;
    struct regulator *vreg;

    pdata->vreg = NULL;
    vreg = regulator_get(&ix->spi->dev, "ix,vddio");
    if (IS_ERR(vreg)) {
        error = PTR_ERR(vreg);
        pr_debug("Regulator get failed, error=%d", error);
        return error;
    }


    if (regulator_count_voltages(vreg) > 0) {
        error = regulator_set_voltage(vreg,
            1800000UL, 1800000UL);
        if (error) {
            pr_debug("regulator set_vtg failed error=%d", error);
            goto err;
        }
    }

    if(regulator_count_voltages(vreg) > 0) {
        error = regulator_set_optimum_mode(vreg, 7000);
        if(error < 0) {
            pr_debug("unable to set current");
            goto err;
        }
    }
    pdata->vreg = vreg;
    return error;
err:
    regulator_put(vreg);
    return error;
}

static int ix_regulator_set(struct ix_data *ix, bool enable)
{
    int error = 0;
    struct ix_platform_data *pdata = ix->platform_pdata;

    pr_debug("power %s!!", (enable) ? "on" : "off");
	pr_debug(KERN_DEBUG ": [DEBUG] %s:%i %s ix_regulator_set(): power %s \n", __FILE__, __LINE__, __func__, (enable) ? "on" : "off");

#if defined (__TEST_APP__)
    if(enable) {
        if(!ix->power_on){
            error = regulator_enable(pdata->vreg);
			dev_info(&ix->spi->dev, "[INFO] regulator_enable on \n");
        }
    } else {
        if(ix->power_on){
            error = regulator_disable(pdata->vreg);			
			dev_info(&ix->spi->dev, "[INFO] regulator_enable off \n");
        }
    }
#else
    error = regulator_enable(pdata->vreg);
#endif

#if defined (__TEST_APP__)
    if(error < 0)
		dev_err(&ix->spi->dev, "can't set(%d) regulator, error(%d)", enable, error);
    else
        ix->power_on = enable;
#endif

	dev_info(&ix->spi->dev, "[INFO] regulator value : %d", regulator_get_voltage(pdata->vreg));
    return error;
}

/* -------------------------------------------------------------------- */
static int ix_gpio_reset(struct ix_data *ix)
{
	int error = 0;
	int counter = IX_RESET_RETRIES;

	while (counter) {
		counter--;

		/* gpio_set_value(ix->reset_gpio, 0); */
		gpio_direction_output(ix->reset_gpio, 0);
		udelay(1000);
		/* mdelay(2); */

		/* gpio_set_value(ix->reset_gpio, 1); */
		gpio_direction_output(ix->reset_gpio, 1);
		udelay(1250);
		/* mdelay(3); */

		error = gpio_get_value(ix->irq_gpio) ? 0 : -EIO;
		if (!error) {
			counter = 0;
		} else {
			PINFO("timed out,retrying ...");

			udelay(1250);
		}
	}

	return error;
}

/* -------------------------------------------------------------------- */
/* function definitions                                                 */
/* -------------------------------------------------------------------- */
static int ix_init(void)
{
    pr_debug(KERN_DEBUG ": [DEBUG] %s:%i %s\n", __FILE__, __LINE__, __func__);

    if (spi_register_driver(&ix_driver)){
	    pr_debug(KERN_DEBUG ": [DEBUG] spi_register_driver() fail %s:%i %s\n", __FILE__, __LINE__, __func__);
        return -EINVAL;
	}
	pr_debug(KERN_DEBUG ": [DEBUG] %s:%i %s ok \n", __FILE__, __LINE__, __func__);
    return 0;
}

/* -------------------------------------------------------------------- */
static void ix_exit(void)
{
    pr_debug(KERN_DEBUG ": [DEBUG] %s:%i %s\n", __FILE__, __LINE__, __func__);

    spi_unregister_driver(&ix_driver);
}

/* -------------------------------------------------------------------- */
static int ix_probe(struct spi_device *spi)
{
    struct ix_platform_data *ix_pdata;
    struct device *dev = &spi->dev;
    struct ix_data *ix = NULL;

    int error = 0;
    int count = 0;

    pr_debug(KERN_DEBUG ": [DEBUG] %s:%i %s\n", __FILE__, __LINE__, __func__);
#if CONFIG_COMPAT
    printk("CONFIG_COMPAT enable \n");
#else
    printk("CONFIG_COMPAT NOT enable \n");
#endif /* CONFIG_COMPAT */

    ix = kzalloc(sizeof(*ix), GFP_KERNEL);
    if (!ix)
    {
        dev_err(&spi->dev, "[ERROR] failed to allocate memory for struct ix_data\n");
        return -ENOMEM;
    }
    
    if(spi->dev.of_node) {
        ix_pdata= devm_kzalloc(dev, sizeof(*ix_pdata), GFP_KERNEL);
        if (!ix_pdata) {
            pr_debug("Failed to allocate memory");
            return -ENOMEM;
        }
        spi->dev.platform_data = ix_pdata;
        ix->platform_pdata = ix_pdata;
        error = ix_get_of_pdata(dev, ix_pdata);
        if( error)
            goto err;
    } else {
        ix_pdata= spi->dev.platform_data;
    }

    spi_set_drvdata(spi, ix);
    ix->spi = spi;
    ix->spi->mode = SPI_MODE_0;
    ix->spi->bits_per_word = 8;
    ix->spi->chip_select = 0;

    error = spi_setup(ix->spi);
    if (error)
    {
        dev_err(&ix->spi->dev, "[ERROR] spi_setup failed.\n");
        goto err;
    }

    if (!ix_pdata)
    {
        dev_err(&ix->spi->dev, "[ERROR] spi->dev.platform_data is NULL.\n");
        error = -EINVAL;
        goto err;
    }

    if((error = of_property_read_u32(ix->spi->dev.of_node, "qcom,qup-id", &ix->qup_id)) < 0) 
    {
        dev_err(&ix->spi->dev,"Error getting qup_id\n");
        goto err;
    }

    if((error = ix_regulator_init(ix, ix_pdata)) < 0)
    {
        dev_err(&ix->spi->dev,"ix_regulator_init err\n");
        goto err;
    }

    if((error = ix_regulator_set(ix, true)))
    {
        dev_err(&ix->spi->dev,"ix_regulator_set err\n");
        goto err;
    }

    error = gpio_request(ix_pdata->irq_gpio, "gpio_irq");
    if (error)
    {
        dev_err(&ix->spi->dev, "[ERROR] gpio_request (irq) failed.\n");
        goto err;
    }
    ix->irq_gpio = ix_pdata->irq_gpio;

    error = gpio_direction_input(ix->irq_gpio);
    if (error)
    {
        dev_err(&ix->spi->dev, "[ERROR] gpio_direction_input (irq) failed.\n");
        goto err;
    }

    ix->irq = gpio_to_irq(ix->irq_gpio);
    if (ix->irq < 0)
    {
        dev_err(&ix->spi->dev, "[ERROR] gpio_to_irq failed.\n");
        error = ix->irq;
        goto err;
    }

    error = gpio_request(ix_pdata->reset_gpio, "gpio_reset");
    if (error)
    {
        dev_err(&ix->spi->dev,  "[ERROR] gpio_request (reset) failed.\n");
        goto err;
    }
    ix->reset_gpio = ix_pdata->reset_gpio;

    error = gpio_direction_output(ix->reset_gpio, 0);
    if (error)
    {
        dev_err(&ix->spi->dev,  "[ERROR] gpio_direction_output(reset) failed.\n");
        goto err;
    }

    error = gpio_request(ix_pdata->cs_gpio, "gpio_cs");
    if (error)
    {
        dev_err(&ix->spi->dev,  "[ERROR] gpio_request (cs) failed.\n");
        goto err;
    }
    ix->cs_gpio = ix_pdata->cs_gpio;

    error = gpio_direction_output(ix->cs_gpio, 0);
    if (error)
    {
        dev_err(&ix->spi->dev,  "[ERROR] gpio_direction_output(cs) failed.\n");
        goto err;
    }

    mdelay(1);
    gpio_set_value(ix->reset_gpio, 1);
    gpio_set_value(ix->cs_gpio, 1);
    mdelay(1);

Retry:

	error = ix_get_hw_id(ix);
	if (error)
		goto err;

	if (ix->info.chipID != IX_HW_ID_SP2)
	{
		count++;

		if(count < MAX_RETRY_HW_CHECK_COUNT)
		{
			udelay(150);
			dev_err(&ix->spi->dev, "[ERROR] **** probe retry ix->info.chipID  0x%x****\n", ix->info.chipID);
			goto Retry;
		}
		goto err;
	}
    
    error = ix_check_interrupt(ix);
    if (error)
    {
        dev_err(&ix->spi->dev, "[ERROR] ix_check_interrupt failed.\n");
        goto err_cdev;
    }

	error = ix_reset(ix);
	if (error)
	{
		dev_err(&ix->spi->dev, "[ERROR] ix_reset failed.\n");
		goto err_cdev;
	}

    ix->devno = MKDEV(IX_MAJOR, ix_device_count++);

    ix->class = class_create(THIS_MODULE, IX_CLASS_NAME);
    if (IS_ERR(ix->class))
    {
        dev_err(&ix->spi->dev, "failed to create class.\n");
        error = PTR_ERR(ix->class);
        goto err;
    }

    ix->device = device_create(ix->class, NULL, ix->devno, NULL, "%s", IX_DEV_NAME);
    if (IS_ERR(ix->device))
    {
        dev_err(&ix->spi->dev, "[ERROR] device_create failed.\n");
        error = PTR_ERR(ix->device);
        goto err;
    }

    error = register_chrdev_region(MKDEV(ix->devno,0), 1, IX_DEV_NAME);
    if (error)
    {
        dev_err(&ix->spi->dev,  "[ERROR] register_chrdev_region failed.\n");
        goto err;

    }

    cdev_init(&ix->cdev, &ix_fops);
    ix->cdev.owner = THIS_MODULE;

    error = cdev_add(&ix->cdev, ix->devno, 1);
    if (error)
    {
        dev_err(&ix->spi->dev, "[ERROR] cdev_add failed.\n");
        goto err_chrdev;
    }

#if defined(CONFIG_FB)
    ix->fb_notifier.notifier_call = fb_notifier_suspend;
#ifdef LGE_FINGERPRINT_NO_TTW
    error = fb_register_client(&ix->fb_notifier);
    if (error)
        dev_err(&ix->spi->dev, "Unable to register fb_notifier: %d\n", error);
#endif
#endif

    /* register input device */
    ix->input = input_allocate_device();
    if(!ix->input) {
        dev_err(&ix->spi->dev, "[ERROR] input_allocate_deivce failed.\n");
        error = -ENOMEM;
        goto err_cdev;
    }
    
    ix->input->name = "fingerprint";
    ix->input->dev.init_name = "lge_fingerprint";


    input_set_capability(ix->input, EV_REL, IX_INTERRUPT);

    //error = request_irq(ix->irq, ix_interrupt, IRQF_TRIGGER_RISING, "ix", ix);
	error = devm_request_threaded_irq(dev, ix->irq, NULL,ix_interrupt,IRQF_TRIGGER_RISING | IRQF_ONESHOT,"ix", ix);
	if (error)
	{
		dev_err(&ix->spi->dev, "[ERROR] request_irq %i failed.\n", ix->irq);
		ix->irq = -EINVAL;
		goto err_cdev;
	}

	disable_irq(ix->irq);
	enable_irq(ix->irq);
	
    enable_irq_wake(ix->irq);
    wake_lock_init(&ix->ttw_wakelock, WAKE_LOCK_SUSPEND, "idxq_ttl_wakelock");

	input_set_drvdata(ix->input, ix);
	error = input_register_device(ix->input);
	if(error) {
		dev_err(&ix->spi->dev, "[ERROR] nput_register_device failed.\n");
		input_free_device(ix->input);
		goto err;
	}

	if(sysfs_create_group(&ix->input->dev.kobj, &ix_attr_group)) {
		dev_err(&ix->spi->dev, "[ERROR] sysfs_create_group failed.\n");
		goto err_sysfs;
	}

    return 0;
	
err_sysfs:
	input_free_device(ix->input);
	input_unregister_device(ix->input);

err_cdev:
    cdev_del(&ix->cdev);
err_chrdev:
    unregister_chrdev_region(ix->devno, 1);
err:
    ix_cleanup(ix);
    spi_set_drvdata(spi, NULL);

    return error;
}

/* -------------------------------------------------------------------- */
static int ix_remove(struct spi_device *spi)
{
    struct ix_data *ix = spi_get_drvdata(spi);

    pr_debug(KERN_DEBUG ": [DEBUG] %s:%i %s\n", __FILE__, __LINE__, __func__);

    sysfs_remove_group(&ix->input->dev.kobj, &ix_attr_group);
    input_free_device(ix->input);
    input_unregister_device(ix->input);

    cdev_del(&ix->cdev);
    unregister_chrdev_region(ix->devno, 1);
    ix_cleanup(ix);
    spi_set_drvdata(spi, NULL);

    return 0;
}

/* -------------------------------------------------------------------- */
static int ix_suspend(struct device *dev)
{

	struct ix_data *ix = dev_get_drvdata(dev);

    disable_irq(ix->irq);
    gpio_direction_output(ix->reset_gpio, 0);
    if(ix_regulator_set(ix, false) < 0)
         PERR("reguator off fail");

    return 0;
}

/* -------------------------------------------------------------------- */
static int ix_resume(struct device *dev)
{
    struct ix_data *ix = dev_get_drvdata(dev);

    PINFO("enter");

    printk("%s", __FUNCTION__);

    if(ix_regulator_set(ix, true) < 0)
        PERR("reguator on fail");

    if(ix_gpio_reset(ix))
        PINFO("reset gpio init fail");

    enable_irq(ix->irq);

    return 0;
}


/* -------------------------------------------------------------------- */
static int ix_open(struct inode *inode, struct file *file)
{
    pr_debug(KERN_DEBUG ": [DEBUG] %s:%i %s\n", __FILE__, __LINE__, __func__);

    return 0;
}

/* -------------------------------------------------------------------- */
static int ix_release(struct inode *inode, struct file *file)
{
    pr_debug(KERN_DEBUG ": [DEBUG] %s:%i %s\n", __FILE__, __LINE__, __func__);

    return 0;
}

/* -------------------------------------------------------------------- */
static int ix_reset(struct ix_data *ix)
{
    int error;

    pr_debug(KERN_DEBUG ": [DEBUG] %s:%i %s\n", __FILE__, __LINE__, __func__);

    error = ix_spi_write_cmd(ix, IX_CMD_SRESET);
    if (error)
        return error;

    error = ix_init_param(ix);
    if (error)
    {
        dev_err(&ix->spi->dev, "[ERROR] ix_init_param failed.\n");
        return error;
    }

    error = ix_spi_write_cmd(ix, IX_CMD_IDLE);
    if(error)
        return error;

    return 0;
}

/* -------------------------------------------------------------------- */
static int ix_init_param(struct ix_data *ix)
{
    int error;
    int i;

    pr_debug(KERN_DEBUG ": [DEBUG] %s:%i %s\n", __FILE__, __LINE__, __func__);

    if (ix->info.chipID == IX_HW_ID_SP2)
    {
        for(i=0; i<ARRAY_SIZE(init_seq_value_sp2); i++)
        {
            error = ix_spi_write_reg(ix, init_seq_value_sp2[i][0], init_seq_value_sp2[i][1]);
            if (error)
                return error;
        }
    }
    else
    {
        dev_err(&ix->spi->dev, "[ERROR] ix_init_param() fail. hardware id mismatch");
        return -1;
    }

    return 0;
}

/* -------------------------------------------------------------------- */
static int ix_get_hw_id(struct ix_data *ix)
{
    int error;
    int cnt = RETRY_WAIT_COUNT;
    pr_debug(KERN_DEBUG ": [DEBUG] %s:%i %s\n", __FILE__, __LINE__, __func__);

    while (cnt--)
    {
        error = ix_spi_write_2_btye_reg(ix, 0x4F, 0x80);
        if(error)
        {
            dev_err(&ix->spi->dev, "[ERROR] ix_spi_write_2_btye_reg 0x4F, 0x80\n");
            return error;
        }

        error = ix_spi_write_reg(ix, 0x4F, 0x80);
        if(error)
        {
            dev_err(&ix->spi->dev, "[ERROR] ix_spi_write_reg 0x4F, 0x80\n");
            return error;
        }

        udelay(50);

        error = ix_spi_read_reg(ix, IX_REG_CHIP_ID, &ix->info.chipID);
        if (error)
        {
            dev_err(&ix->spi->dev, "[ERROR] IX_REG_CHIP_ID failed\n");
            return error;;
        }

        error = ix_spi_read_reg(ix, IX_REG_REVISION_NUMBER, &ix->info.revNO);
        if (error)
        {
            dev_err(&ix->spi->dev, "[ERROR] IX_REG_REVSION_NUMBER failed\n");
            return error;
        }

        if (ix->info.chipID != IX_HW_ID_SP2)
        {
            if (cnt == 1)
            {
                dev_err(&ix->spi->dev, "[ERROR] hardware id mismatch : %02x expected %02x, retry : %d\n",
                    ix->info.chipID, IX_HW_ID_SP2, RETRY_WAIT_COUNT - cnt);
                return -EIO;
            }
        }
        else
        {
            dev_info(&ix->spi->dev, "[INFO] ***************************************************\n");
            dev_info(&ix->spi->dev, "[INFO] * Chip ID[0x%02x] * Revision Number[0x%02x] * Count[%d]\n",
                ix->info.chipID, ix->info.revNO, RETRY_WAIT_COUNT - cnt);
            dev_info(&ix->spi->dev, "[INFO] ***************************************************\n");
            break;
        }
    }

    return 0;
}

/* -------------------------------------------------------------------- */
static int ix_check_interrupt(struct ix_data *ix)
{
    int error;
    int cnt = RETRY_WAIT_COUNT;
    u8 irq;

    error = ix_spi_write_reg(ix, IX_CMD_INT_RC, 0);
    if (error)
    {
        dev_err(&ix->spi->dev, "[ERROR] IX_CMD_INT_RC failed\n");
        return error;;
    }

    while (cnt--)
    {
        udelay(500);

        if (gpio_get_value(ix->irq_gpio)) {
            dev_err(&ix->spi->dev, "[ERROR] %s interrupt pin must be low\n", __func__);

            if(cnt == 1)
                return -EIO;
        }
        else
            break;
    }
    
    dev_info(&ix->spi->dev, "[INFO] interrupt wait count [%d]\n", RETRY_WAIT_COUNT - cnt);

    error = ix_reset(ix);
    if (error < 0)
        return error;

    error = ix_spi_write_reg(ix, IX_REG_INT_SRC_MASK, 0x08);
    if (error < 0)
        return error;

    error = ix_spi_write_cmd(ix, IX_CMD_SCAN);
    if (error < 0)
        return error;

    udelay(1250);

    if (!gpio_get_value(ix->irq_gpio)) {
        dev_err(&ix->spi->dev, "[ERROR] %s interrupt pin must be high\n", __func__);
        return -EIO;
    }

    error = ix_spi_read_reg(ix, IX_CMD_INT_R, &irq);
    if (error)
    {
        dev_err(&ix->spi->dev, "[ERROR] IX_CMD_INT_R failed\n");
        return error;
    }

    if (irq & (1 << 4u)) {
        dev_info(&ix->spi->dev, "[INFO] Interrupt test OK\n");
        return 0;
    }
    else {
        dev_err(&ix->spi->dev, "[ERROR] Not expected irq\n");
        return -EINVAL;
    }

}

#ifdef CONFIG_OF
/* -------------------------------------------------------------------- */
static int ix_get_of_pdata(struct device *dev, struct ix_platform_data *pdata)
{
//    const struct device_node *node = dev->of_node; // original code from crutialtec.
    struct device_node *node = dev->of_node;
    const void *irq_prop = of_get_property(node, "ix,gpio_irq",   NULL);
    const void *rst_prop = of_get_property(node, "ix,gpio_reset", NULL);
    const void *cs_prop  = of_get_property(node, "ix,gpio_cs",    NULL);

    pr_debug(KERN_DEBUG ": [DEBUG] %s:%i %s\n", __FILE__, __LINE__, __func__);

    if (node == NULL) {
        dev_err(dev, "%s: Could not find OF device node\n", __func__);
        goto of_err;
    }

    if (!irq_prop || !rst_prop || !cs_prop) {
        dev_err(dev, "%s: Missing OF property\n", __func__);
        goto of_err;
    }

    pdata->irq_gpio   = of_get_named_gpio(node, "ix,gpio_irq", 0);
    pdata->reset_gpio = of_get_named_gpio(node, "ix,gpio_reset", 0);
    pdata->cs_gpio = of_get_named_gpio(node, "ix,gpio_cs", 0);

    pr_info(KERN_INFO ": [INFO] irq : %d  reset : %d\n", pdata->irq_gpio, pdata->reset_gpio);

    return 0;

of_err:
    pdata->reset_gpio = -EINVAL;
    pdata->irq_gpio   = -EINVAL;
    pdata->cs_gpio   = -EINVAL;

    return -ENODEV;
}
#endif

#if defined(CONFIG_FB)
/* -------------------------------------------------------------------- */
static int fb_notifier_suspend(struct notifier_block *self, unsigned long event, void *data)
{
    struct fb_event *evdata = data;
    int *blank;

    struct ix_data *ix;
    ix = container_of(self, struct ix_data, fb_notifier);

    if (evdata && evdata->data && event == FB_EVENT_BLANK && ix) {
        blank = (evdata->data);
        dev_dbg(&ix->spi->dev, "%s  %d \n", __func__, *blank);

        if (*blank == FB_BLANK_UNBLANK) //resume
        {
            PINFO("resume");

            if(ix_regulator_set(ix, true) < 0)
                PERR("reguator on fail");

            gpio_direction_output(ix->reset_gpio, 1);

            enable_irq(ix->irq);
            input_report_rel(ix->input, IX_INTERRUPT, 1);
            input_sync(ix->input);
        }
        else //suspend
        {
            PINFO("suspend");

            disable_irq(ix->irq);
            gpio_direction_output(ix->reset_gpio, 0);
            if(ix_regulator_set(ix, false) < 0)
                PERR("reguator off fail");
        }
    }

    return 0;
}
#endif

/* -------------------------------------------------------------------- */
irqreturn_t ix_interrupt(int irq, void *_ix)
{
	struct ix_data *ix = _ix;

	if ( gpio_get_value(ix->irq_gpio) )
	{
		wake_lock_timeout(&ix->ttw_wakelock, msecs_to_jiffies(IX_WAKELOCK_HOLD_TIME));
		
		input_report_rel(ix->input, IX_INTERRUPT, 1);
		input_sync(ix->input);

		return IRQ_HANDLED;
	}

	return IRQ_NONE;
}

/* -------------------------------------------------------------------- */
static int ix_cleanup(struct ix_data *ix)
{
    pr_debug(KERN_DEBUG ": [DEBUG] %s:%i %s\n", __FILE__, __LINE__, __func__);


    if (!IS_ERR_OR_NULL(ix->device))
        device_destroy(ix->class, ix->devno);

    if (!IS_ERR_OR_NULL(ix->class))
        class_destroy(ix->class);

    disable_irq_wake(ix->irq);

    if (ix->irq >= 0)
        free_irq(ix->irq, ix);	

    if (gpio_is_valid(ix->irq_gpio))
        gpio_free(ix->irq_gpio);

    if (gpio_is_valid(ix->reset_gpio))
        gpio_free(ix->reset_gpio);

    if (gpio_is_valid(ix->cs_gpio))
        gpio_free(ix->cs_gpio);

    return 0;
}

/* -------------------------------------------------------------------- */
static int ix_spi_write_cmd(struct ix_data *ix, ix_cmd_reg_t addr)
{
    int error;
    u8 tx[2];

    struct spi_message m;
    struct spi_transfer t = {
        .cs_change = CS_CHANGE_VALUE,
        .delay_usecs = 0,
        .speed_hz = IX_SPI_CLOCK_SPEED,
        .tx_buf = tx,
        .rx_buf = NULL,
        .len = 2,
        .tx_dma = 0,
        .rx_dma = 0,
        .bits_per_word = 0,
    };

    tx[0] = IX_SPI_WRITE_COMMAND;
    tx[1] = addr;

    spi_message_init(&m);
    spi_message_add_tail(&t, &m);

    error = spi_sync(ix->spi, &m);
    if (error)
    {
        dev_err(&ix->spi->dev, "[ERROR] spi_sync failed.\n");
        return error;
    }

    return 0;
}

/* -------------------------------------------------------------------- */
static int ix_spi_write_reg(struct ix_data *ix, ix_cmd_reg_t addr, u8 value)
{
    int error;
    u8 tx[3];

    struct spi_message m;
    struct spi_transfer t = {
        .cs_change = CS_CHANGE_VALUE,
        .delay_usecs = 0,
        .speed_hz = IX_SPI_CLOCK_SPEED,
        .tx_buf = tx,
        .rx_buf = NULL,
        .len = 3,
        .tx_dma = 0,
        .rx_dma = 0,
        .bits_per_word = 0,
    };

    tx[0] = IX_SPI_WRITE_COMMAND;
    tx[1] = addr;
    tx[2] = value;

    spi_message_init(&m);
    spi_message_add_tail(&t, &m);

    error = spi_sync(ix->spi, &m);
    if (error)
    {
        dev_err(&ix->spi->dev, "[ERROR] spi_sync failed.\n");
        return error;
    }

    return 0;
}

static int ix_spi_write_2_btye_reg(struct ix_data *ix, ix_cmd_reg_t addr, u8 value)
{
    int error;
    u8 tx[2];

    struct spi_message m;
    struct spi_transfer t = {
        .cs_change = CS_CHANGE_VALUE,
        .delay_usecs = 0,
        .speed_hz = IX_SPI_CLOCK_SPEED,
        .tx_buf = tx,
        .rx_buf = NULL,
        .len = 2,
        .tx_dma = 0,
        .rx_dma = 0,
        .bits_per_word = 0,
    };

    tx[0] = addr;
    tx[1] = value;

    spi_message_init(&m);
    spi_message_add_tail(&t, &m);

    error = spi_sync(ix->spi, &m);
    if (error)
    {
        dev_err(&ix->spi->dev, "[ERROR] spi_sync failed.\n");
        return error;
    }

    return 0;
}

/* -------------------------------------------------------------------- */
static int ix_spi_read_reg(struct ix_data *ix, ix_cmd_reg_t addr, u8 *rx)
{
    int error;
    u8 tx[3] = {0,};

    struct spi_message m;
    struct spi_transfer t = {
        .cs_change = CS_CHANGE_VALUE,
        .delay_usecs = 0,
        .speed_hz = IX_SPI_CLOCK_SPEED,
        .tx_buf = tx,
        .rx_buf = tx,
        .len = 3,
        .tx_dma = 0,
        .rx_dma = 0,
        .bits_per_word = 0,
    };

    tx[0] = IX_SPI_READ_COMMAND;
    tx[1] = addr;

    spi_message_init(&m);
    spi_message_add_tail(&t, &m);

    error = spi_sync(ix->spi, &m);
    if (error)
    {
        dev_err(&ix->spi->dev, "[ERROR] spi_sync failed.\n");
        return error;
    }

    *rx = tx[2];

    return 0;
}

