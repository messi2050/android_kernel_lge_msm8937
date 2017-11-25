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
#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#elif defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
#endif
#include <linux/spi/spi.h>
#include <linux/regulator/consumer.h>


#include "ix_btp.h"


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Crucialsoft Fingerprint <www.crucialtec.co.kr>");
MODULE_DESCRIPTION("IX BTP sensor driver.");

#define CONFIG_COMPAT 1
#define REARRANGED_ADC_TABLE
#define IX_VERSION                      "1.2.0"
#define __TEST_APP__

/* -------------------------------------------------------------------- */
/* ix driver constants                                                  */
/* -------------------------------------------------------------------- */
#define IX_MAJOR                        250
#define IX_DEV_NAME                     "btp"
#define IX_CLASS_NAME                   "ixsensor"
#define IX_WORKER_THREAD_NAME           "ixworker"
#define IX_SPI_CLOCK_SPEED              (48 * 100000U)
#define IX_DEFAULT_IRQ_TIMEOUT          (100 * HZ / 1000)
#define CS_CHANGE_VALUE                 0


/* -------------------------------------------------------------------- */
/* ix sensor information                                                */
/* -------------------------------------------------------------------- */
#define IX_HW_ID_SP2                    0x52
#define IX_SENSOR_WIDTH                 104U
#define IX_SENSOR_HEIGHT                72U
#define IX_SENSOR_SIZE                  (IX_SENSOR_WIDTH * IX_SENSOR_HEIGHT)
#define IX_MAX_FRAME                    25
#define IX_CMD_SIZE                     3
#define IX_IMAGE_BUFFER_SIZE            (IX_SENSOR_SIZE * IX_MAX_FRAME + IX_CMD_SIZE)
#define IX_FINGER_DETECT_THRESH         500
#define IX_FINGER_DETECT_COUNT          9
#define IX_ADC_COUNT                    3
#define IX_FINGER_DETECT_RETRY_COUNT    20


/* -------------------------------------------------------------------- */
/* ix spi read/write command                                            */
/* -------------------------------------------------------------------- */
#define IX_SPI_READ_COMMAND             0xA8
#define IX_SPI_WRITE_COMMAND            0xA9


/* -------------------------------------------------------------------- */
/* ix i/o control                                                       */
/* -------------------------------------------------------------------- */
#define IX_IOCTL_MAGIC_NO               0xFC
#define IX_IOCTL_START_CAPTURE          _IO(IX_IOCTL_MAGIC_NO, 0)
#define IX_IOCTL_ABORT_CAPTURE          _IO(IX_IOCTL_MAGIC_NO, 1)
#define IX_IOCTL_CHECKERBOARD_CAPTURE   _IOW(IX_IOCTL_MAGIC_NO, 2, int)
#define IX_IOCTL_SET_FIRST_TRY          _IOW(IX_IOCTL_MAGIC_NO, 3, int)
#define IX_IOCTL_CHECK_WAKE_UP          _IO(IX_IOCTL_MAGIC_NO, 4)
#define IX_IOCTL_CHECK_FINGER           _IOW(IX_IOCTL_MAGIC_NO, 5, int)
#ifdef REARRANGED_ADC_TABLE
#define IX_IOCTL_SET_ADC_TABLE_INDEX    _IOW(IX_IOCTL_MAGIC_NO, 6, ioctl_adc_table_index_info)
#endif
#if defined (__TEST_APP__)
#define IX_IOCTL_GET_HW_ID              _IO(IX_IOCTL_MAGIC_NO, 10)
#endif

#if defined (__TEST_APP__)
#define MAX_RETRY_HW_CHECK_COUNT        5
#endif

/* -------------------------------------------------------------------- */
/* ix sensor commands and registers                                     */
/* -------------------------------------------------------------------- */
typedef enum {
    /* commands */
    IX_CMD_SRESET           = 0x01,   /* Soft reset */
    IX_CMD_IDLE             = 0x03,   /* Go to idle state*/
    IX_CMD_SCAN             = 0x04,   /* Go to scan state */
    IX_CMD_DET              = 0x05,   /* Go to detect state */
    IX_CMD_READ             = 0x06,   /* SRAM memory data read */
    IX_CMD_INT_R            = 0x08,   /* Interrupt read */
    IX_CMD_INT_RC           = 0x09,   /* Interrupt read & clear */
    /* registers */
    IX_REG_CHIP_ID          = 0xB9,
    IX_REG_REVISION_NUMBER  = 0xBA,
    IX_REG_VREF_SET2        = 0x54, //this register is kind of offset
    IX_REG_PGA_SET0         = 0x5B,
    IX_REG_PGA_SET1         = 0x5C,
    IX_REG_CELL_AMP_SET     = 0x5D,
    IX_REG_SENS_CTRL        = 0x10,
    IX_REG_INT_SRC_MASK     = 0x2E,
    IX_REG_IMG_DET_CTRL     = 0x38,
    IX_REG_IMG_DET_L_THRES  = 0x3D,
} ix_cmd_reg_t;


/* -------------------------------------------------------------------- */
/* ix sensor irq flag                                                   */
/* -------------------------------------------------------------------- */
typedef enum {
    IX_IRQ_MEMORY_READ_END = (1 << 5u),
    IX_IRQ_SCAN_COMPLETE   = (1 << 4u),
    IX_IRQ_IDET_FINGER_OFF = (1 << 3u),
    IX_IRQ_IDET_FINGER_ON  = (1 << 2u),
} ix_irq_reg_t;


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

struct ix_thread_task {
    int mode;
    int should_stop;
    struct semaphore sem_idle;
    wait_queue_head_t wait_job;
    struct task_struct *thread;
};

struct fpc_btp_platform_data {
    int irq_gpio;
    int reset_gpio;
    int cs_gpio;
    int qup_id;
    struct regulator *vreg;
};

struct ix_adc_setup {
    u8 pga_set0[IX_ADC_COUNT];
    u8 pga_set1[IX_ADC_COUNT];
    u8 vref_set2[IX_ADC_COUNT];
    u8 cell_amp_set[IX_ADC_COUNT];
    int index;
};

struct ix_reg_setup {
    u8 finger_down_threshold;
    u8 finger_up_threshold;
};

struct ix_diag {
    u32 selftest;
    u32 sensortest;
    u32 capture_time;
    u32 frames_captured;
    u32 frames_stored;
    u32 finger_threshold;
    u8 adc_count;
};
struct fpc_test_result {
    int  pass;
    int  value;
};

struct ix_data {
    struct spi_device *spi;
    struct class *class;
    struct device *device;
    struct cdev cdev;
    struct semaphore mutex;
    struct ix_thread_task thread_task;
    struct ix_adc_setup adc_setup;
    struct ix_reg_setup reg_setup;
    struct ix_diag diag;
    struct ix_info info;
    /*debug core*/
    struct fpc_test_result spi_result;
    struct ix_platform_data *platform_pdata;
    dev_t devno;
    wait_queue_head_t waiting_data_avail;
    wait_queue_head_t waiting_interrupt_return;
    u32 reset_gpio;
    u32 irq_gpio;
    u32 irq;
    u32 data_offset;
    u32 avail_data;
    u32 current_frame;
    u32 current_adc_table;
    bool capture_done;
    int interrupt_done;
    int pxl_sum[IX_FINGER_DETECT_COUNT];
    u8 *huge_buffer;
    bool isSleep;
    int firstTry;
	bool power_on;
    u8 finger_status;
    u8 request_finger;
#if defined(CONFIG_FB)
    struct notifier_block fb_notifier;
#elif defined(CONFIG_HAS_EARLYSUSPEND)
    struct early_suspend early_suspend;
#endif
};

struct ix_attribute {
    struct device_attribute attr;
    size_t offset;
};

#ifdef REARRANGED_ADC_TABLE
typedef struct {
    int index[6];
    int size;
} __attribute__((packed)) ioctl_adc_table_index_info;

static ioctl_adc_table_index_info g_adc_table_index_info;
#endif

enum {
    FNGR_ST_NONE = 0,
    FNGR_ST_DETECTED,
    FNGR_ST_LOST,
};

enum {
    IX_THREAD_IDLE_MODE = 0,
    IX_THREAD_CAPTURE_MODE,
    IX_THREAD_EXIT,
};

enum {
    IX_MODE_CAPTURE = 0,
    IX_MODE_FINGERUP,
    IX_MODE_FINGERDOWN,
    IX_MODE_STANDBY,
};


/* -------------------------------------------------------------------- */
/* global variables                                                     */
/* -------------------------------------------------------------------- */
static int ix_device_count;


/* -------------------------------------------------------------------- */
/* ix init value                                                        */
/* -------------------------------------------------------------------- */
const static u8 ix_adc_table[IX_ADC_COUNT][4]  = {
  /* set0, set1, set2, amp */
    {0x5E, 0x1B, 0xE2, 0x1F}, // dry
    {0x2E, 0x15, 0xf0, 0x1F}, // normal
    {0xFE, 0x08, 0xF2, 0x1F}, // humid
};

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
/* function prototypes                                                  */
/* -------------------------------------------------------------------- */
static int ix_init(void);
static void ix_exit(void);
static int ix_probe(struct spi_device *spi);
static int ix_remove(struct spi_device *spi);
static int ix_open(struct inode *inode, struct file *file);
static int ix_release(struct inode *inode, struct file *file);
static ssize_t ix_read(struct file *file, char *buff, size_t count, loff_t *ppos);
static ssize_t ix_write(struct file *file, const char *buff, size_t count, loff_t *ppos);
static unsigned int ix_poll(struct file *file, poll_table *wait);
static long ix_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);

static ssize_t ix_show_attr_info(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t ix_store_attr_info(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t ix_show_attr_adc_setup(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t ix_store_attr_adc_setup(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t ix_show_attr_diag(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t ix_store_attr_diag(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t ix_show_attr_reg_setup(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t ix_store_attr_reg_setup(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);

#ifdef CONFIG_OF
static int ix_get_of_pdata(struct device *dev, struct ix_platform_data *pdata);
#endif
#if defined(CONFIG_FB)
static int fb_notifier_suspend(struct notifier_block *self,unsigned long event, void *data);
#elif defined(CONFIG_HAS_EARLYSUSPEND)
static void ix_early_suspend(struct early_suspend *h);
static void ix_late_resume(struct early_suspend *h);
#endif

irqreturn_t ix_interrupt(int irq, void *_ix);
static int ix_cleanup(struct ix_data *ix);
static int ix_wait_for_irq(struct ix_data *ix, int timeout);
static int ix_spi_write_cmd(struct ix_data *ix, ix_cmd_reg_t addr);
static int ix_spi_write_reg(struct ix_data *ix, ix_cmd_reg_t addr, u8 value);
#ifdef __TEST_APP__
static int ix_spi_write_2_btye_reg(struct ix_data *ix, ix_cmd_reg_t addr, u8 value);
#endif
static int ix_spi_read_reg(struct ix_data *ix, ix_cmd_reg_t addr, u8 *rx);
static int ix_spi_read_image(struct ix_data *ix);
static int ix_spi_read_wait_irq(struct ix_data *ix, u8 *irq, bool interrupt_clear);

static void ix_set_info(struct ix_data *ix);
static void ix_refresh(struct ix_data *ix);
static int ix_reset(struct ix_data *ix);
static int ix_init_param(struct ix_data *ix);
static int ix_get_hw_id(struct ix_data *ix);
static void ix_adc_initial_set(struct ix_data *ix);
static int ix_auto_adc_ctrl(struct ix_data *ix, int count);
static int ix_mode_change(struct ix_data *ix, int mode);
static int ix_wait_request_finger(struct ix_data *ix, int request_finger);
static int ix_wait_finger_present(struct ix_data *ix);
static int ix_wait_finger_detach(struct ix_data *ix);
static u8  ix_finger_check(struct ix_data *ix, int count);

static int ix_selftest_short(struct ix_data *ix);
static int ix_sensor_test(struct ix_data *ix);

static int threadfn(void *_ix);
static int ix_start_thread(struct ix_data *ix, int mode);
static void ix_start_capture(struct ix_data *ix);
static int ix_thread_goto_idle(struct ix_data *ix);
static int ix_capture_task(struct ix_data *ix);


/* -------------------------------------------------------------------- */
/* External interface                                                   */
/* -------------------------------------------------------------------- */
late_initcall(ix_init);
module_exit(ix_exit);

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
    .write          = ix_write,
    .read           = ix_read,
    .release        = ix_release,
    .poll           = ix_poll,
    .unlocked_ioctl = ix_ioctl,
#ifdef CONFIG_COMPAT
    .compat_ioctl   = ix_ioctl,
#endif /* CONFIG_COMPAT */
};

/* -------------------------------------------------------------------- */
/* devfs                                                                */
/* -------------------------------------------------------------------- */
#define IX_ATTR(__grp, __field, __mode)             \
{                                                   \
    .attr = __ATTR(__field, (__mode),               \
            ix_show_attr_##__grp,                   \
            ix_store_attr_##__grp),                 \
    .offset = offsetof(struct ix_##__grp, __field)  \
}

#define IX_DEV_ATTR(_grp, _field, _mode)            \
    struct ix_attribute ix_attr_##_field = IX_ATTR(_grp, _field, (_mode))

#define INFO_MODE (S_IRUSR | S_IRGRP | S_IROTH)
static IX_DEV_ATTR(info, chipID,  INFO_MODE);
static IX_DEV_ATTR(info, revNO,   INFO_MODE);
static IX_DEV_ATTR(info, version, INFO_MODE);

static struct attribute *ix_info_attrs[] = {
    &ix_attr_chipID.attr.attr,
    &ix_attr_revNO.attr.attr,
    &ix_attr_version.attr.attr,
    NULL
};

static const struct attribute_group ix_info_attr_group = {
    .attrs = ix_info_attrs,
    .name = "info"
};

#define ADC_SETUP_MODE (S_IWUSR | S_IRUSR | S_IRGRP | S_IWGRP | S_IROTH )
static IX_DEV_ATTR(adc_setup, pga_set0,         ADC_SETUP_MODE);
static IX_DEV_ATTR(adc_setup, pga_set1,         ADC_SETUP_MODE);
static IX_DEV_ATTR(adc_setup, vref_set2,        ADC_SETUP_MODE);
static IX_DEV_ATTR(adc_setup, cell_amp_set,     ADC_SETUP_MODE);
static IX_DEV_ATTR(adc_setup, index,            ADC_SETUP_MODE);

static struct attribute *ix_adc_attrs[] = {
    &ix_attr_pga_set0.attr.attr,
    &ix_attr_pga_set1.attr.attr,
    &ix_attr_vref_set2.attr.attr,
    &ix_attr_cell_amp_set.attr.attr,
    &ix_attr_index.attr.attr,
    NULL
};

static const struct attribute_group ix_adc_attr_group = {
    .attrs = ix_adc_attrs,
    .name = "adc_setup"
};

#define DIAG_MODE (S_IWUSR | S_IRUSR | S_IRGRP | S_IWGRP | S_IROTH )
static IX_DEV_ATTR(diag, selftest,         DIAG_MODE);
static IX_DEV_ATTR(diag, sensortest,       DIAG_MODE);
static IX_DEV_ATTR(diag, capture_time,     DIAG_MODE);
static IX_DEV_ATTR(diag, frames_captured,  DIAG_MODE);
static IX_DEV_ATTR(diag, frames_stored,    DIAG_MODE);
static IX_DEV_ATTR(diag, finger_threshold, DIAG_MODE);
static IX_DEV_ATTR(diag, adc_count,        DIAG_MODE);

static struct attribute *ix_diag_attrs[] = {
    &ix_attr_selftest.attr.attr,
    &ix_attr_sensortest.attr.attr,
    &ix_attr_capture_time.attr.attr,
    &ix_attr_frames_captured.attr.attr,
    &ix_attr_frames_stored.attr.attr,
    &ix_attr_finger_threshold.attr.attr,
    &ix_attr_adc_count.attr.attr,
    NULL
};

static const struct attribute_group ix_diag_attr_group = {
    .attrs = ix_diag_attrs,
    .name = "diag"
};

#define REG_SETUP_MODE (S_IWUSR | S_IRUSR | S_IRGRP | S_IWGRP | S_IROTH )
static IX_DEV_ATTR(reg_setup, finger_down_threshold, REG_SETUP_MODE);
static IX_DEV_ATTR(reg_setup, finger_up_threshold,   REG_SETUP_MODE);

static struct attribute *ix_reg_attrs[] = {
    &ix_attr_finger_down_threshold.attr.attr,
    &ix_attr_finger_up_threshold.attr.attr,
    NULL
};

static const struct attribute_group ix_reg_attr_group = {
    .attrs = ix_reg_attrs,
    .name = "reg_setup"
};


/* -------------------------------------------------------------------- */
/* function definitions                                                 */
/* -------------------------------------------------------------------- */
static int ix_init(void)
{
    pr_debug(KERN_DEBUG ": [DEBUG] %s:%i %s\n", __FILE__, __LINE__, __func__);

    if (spi_register_driver(&ix_driver))
    {
        pr_debug(KERN_DEBUG ": [DEBUG] %s:%i %s spi_register_driver() fail \n", __FILE__, __LINE__, __func__);
        return -EINVAL;
    }

    pr_debug(KERN_DEBUG ": [DEBUG] %s:%i %s spi_register_driver() ok \n", __FILE__, __LINE__, __func__);

    return 0;
}

/* -------------------------------------------------------------------- */
static void ix_exit(void)
{
    pr_debug(KERN_DEBUG ": [DEBUG] %s:%i %s\n", __FILE__, __LINE__, __func__);

    spi_unregister_driver(&ix_driver);
}
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
static int ix_probe(struct spi_device *spi)
{
    struct ix_platform_data *ix_pdata;
    struct device *dev = &spi->dev;
    struct ix_data *ix = NULL;

    int error = 0;
#ifdef __TEST_APP__
    int count = 0;
#endif

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


    ix->huge_buffer = (u8 *)__get_free_pages(GFP_KERNEL, get_order(IX_IMAGE_BUFFER_SIZE));
    if (!ix->huge_buffer)
    {
        dev_err(&ix->spi->dev, "[ERROR] failed to get free pages\n");
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

    }else {
        ix_pdata= spi->dev.platform_data;
    }

    if (!ix_pdata)
    {
        dev_err(&ix->spi->dev, "[ERROR] spi->dev.platform_data is NULL.\n");
        error = -EINVAL;
        goto err;
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

    ix->reset_gpio = -EINVAL;
    ix->irq_gpio = -EINVAL;
    ix->irq = -EINVAL;


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

    error = request_irq(ix->irq, ix_interrupt, IRQF_TRIGGER_RISING, "ix", ix);
    if (error)
    {
        dev_err(&ix->spi->dev, "[ERROR] request_irq %i failed.\n", ix->irq);
        ix->irq = -EINVAL;
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

    mdelay(1);
    gpio_set_value(ix->reset_gpio, 1);
    mdelay(1);

    disable_irq(ix->irq);
    ix->interrupt_done = 0;
    enable_irq(ix->irq);

    init_waitqueue_head(&ix->waiting_interrupt_return);
    init_waitqueue_head(&ix->waiting_data_avail);
    init_waitqueue_head(&ix->thread_task.wait_job);
    sema_init(&ix->thread_task.sem_idle, 0);
    sema_init(&ix->mutex, 0);

    memset(&(ix->diag), 0, sizeof(ix->diag));
    memset(&(ix->info), 0, sizeof(ix->info));

    /* register defalut value */
    ix->diag.finger_threshold = IX_FINGER_DETECT_THRESH;

#ifdef __TEST_APP__
Retry:
    error = ix_spi_write_2_btye_reg(ix, 0x4F, 0x80);
    if(error)
    {
        dev_err(&ix->spi->dev, "[ERROR] ix_spi_write_2_btye_reg 0x4F, 0x80\n");
        goto err;
    }
    else
    {
        dev_err(&ix->spi->dev, "[OK] ix_spi_write_2_btye_reg 0x4F, 0x80\n");
    }

    error = ix_spi_write_reg(ix, 0x4F, 0x80);
    if(error)
    {
        dev_err(&ix->spi->dev, "[ERROR] ix_spi_write_reg 0x4F, 0x80\n");
        goto err;
    }
    else
    {
        dev_err(&ix->spi->dev, "[OK] ix_spi_write_reg 0x4F, 0x80\n");
    }
#endif

    error = ix_get_hw_id(ix);

#ifdef __TEST_APP__
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
#else
    if (error)
        goto err;
#endif
    ix_set_info(ix);

    ix->diag.adc_count = IX_ADC_COUNT;
    ix_adc_initial_set(ix);

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

    error = sysfs_create_group(&spi->dev.kobj, &ix_adc_attr_group);
    if (error)
    {
        dev_err(&ix->spi->dev, "[ERROR] sysfs_create_group failed.\n");
        goto err;
    }

    error = sysfs_create_group(&spi->dev.kobj, &ix_diag_attr_group);
    if (error)
    {
        dev_err(&ix->spi->dev, "[ERROR] sysfs_create_group failed.\n");
        goto err_sysf_1;
    }

    error = sysfs_create_group(&spi->dev.kobj, &ix_reg_attr_group);
    if (error)
    {
        dev_err(&ix->spi->dev, "[ERROR] sysfs_create_group failed.\n");
        goto err_sysf_2;
    }

    error = sysfs_create_group(&spi->dev.kobj, &ix_info_attr_group);
    if (error)
    {
        dev_err(&ix->spi->dev, "[ERROR] sysfs_create_group failed.\n");
        goto err_sysf_3;
    }


    error = register_chrdev_region(MKDEV(ix->devno,0), 1, IX_DEV_NAME);
    if (error)
    {
        dev_err(&ix->spi->dev,  "[ERROR] register_chrdev_region failed.\n");
        goto err_sysf_4;

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
    error = fb_register_client(&ix->fb_notifier);
    if (error)
        dev_err(&ix->spi->dev, "Unable to register fb_notifier: %d\n", error);
#endif

    ix->thread_task.mode = IX_THREAD_IDLE_MODE;
    ix->thread_task.thread = kthread_run(threadfn, ix, "%s", IX_WORKER_THREAD_NAME);
    if (IS_ERR(ix->thread_task.thread))
    {
        dev_err(&ix->spi->dev, "[ERROR] kthread_run failed.\n");
        goto err_cdev;
    }

    error = ix_mode_change(ix, IX_MODE_STANDBY);
    if (error)
    {
        dev_err(&ix->spi->dev, "[ERROR] IX_CMD_CMD_STBY_EN failed.\n");
        goto err_cdev;
    }

    up(&ix->mutex);

    return 0;

err_cdev:
    cdev_del(&ix->cdev);
err_chrdev:
    unregister_chrdev_region(ix->devno, 1);
err_sysf_4:
    sysfs_remove_group(&spi->dev.kobj, &ix_info_attr_group);
err_sysf_3:
    sysfs_remove_group(&spi->dev.kobj, &ix_reg_attr_group);
err_sysf_2:
    sysfs_remove_group(&spi->dev.kobj, &ix_diag_attr_group);
err_sysf_1:
    sysfs_remove_group(&spi->dev.kobj, &ix_adc_attr_group);
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

    sysfs_remove_group(&ix->spi->dev.kobj, &ix_info_attr_group);
    sysfs_remove_group(&ix->spi->dev.kobj, &ix_reg_attr_group);
    sysfs_remove_group(&ix->spi->dev.kobj, &ix_adc_attr_group);
    sysfs_remove_group(&ix->spi->dev.kobj, &ix_diag_attr_group);

    ix_mode_change(ix, IX_MODE_STANDBY);

    cdev_del(&ix->cdev);
    unregister_chrdev_region(ix->devno, 1);
    ix_cleanup(ix);
    spi_set_drvdata(spi, NULL);

    return 0;
}

/* -------------------------------------------------------------------- */
static int ix_open(struct inode *inode, struct file *file)
{
    struct ix_data *ix;
#if defined (__TEST_APP__)
	int error;
#endif

    pr_debug(KERN_DEBUG ": [DEBUG] %s:%i %s\n", __FILE__, __LINE__, __func__);

    ix = container_of(inode->i_cdev, struct ix_data, cdev);

    if (down_interruptible(&ix->mutex))
        return -ERESTARTSYS;

    file->private_data = ix;

    up(&ix->mutex);

#if defined (__TEST_APP__)
    error = ix_regulator_set(ix, true);
    if (error)
    {
        dev_err(&ix->spi->dev,"ix_regulator_set err\n");
        return -EIO;
    }
	mdelay(10);
	error = ix_reset(ix);
    if (error)
    {
        dev_err(&ix->spi->dev, "reset failed\n");
        return -EIO;
    }
#endif

    return 0;
}

/* -------------------------------------------------------------------- */
static int ix_release(struct inode *inode, struct file *file)
{
    struct ix_data *ix = file->private_data;
#if defined (__TEST_APP__)
	int error;
#endif

    pr_debug(KERN_DEBUG ": [DEBUG] %s:%i %s\n", __FILE__, __LINE__, __func__);

    if (ix->isSleep == true)
        return 0;

    if (down_interruptible(&ix->mutex))
        return -ERESTARTSYS;

    if (!atomic_read(&file->f_count))
    {
        ix_mode_change(ix, IX_MODE_STANDBY);
    }
    up(&ix->mutex);

#if defined (__TEST_APP__)
    gpio_set_value(ix->reset_gpio, 0);
    mdelay(1);

    error = ix_regulator_set(ix, false);
    if (error)
    {
        dev_err(&ix->spi->dev,"ix_regulator_set err\n");
        return -EIO;
    }
#endif

    return 0;
}

/* -------------------------------------------------------------------- */
static ssize_t ix_read(struct file *file, char *buff, size_t count, loff_t *ppos)
{
    int error = 0;
    u32 read_cnt;
    u32 remain_cnt = 0;

    struct ix_data *ix = file->private_data;

    pr_debug(KERN_DEBUG ": [DEBUG] %s:%i %s\n", __FILE__, __LINE__, __func__);

    if (down_interruptible(&ix->mutex))
        return -ERESTARTSYS;

    error = wait_event_interruptible( ix->waiting_data_avail, (ix->capture_done || ix->avail_data));

    read_cnt = (count > ix->avail_data) ? ix->avail_data : count;

    if (ix->data_offset + read_cnt >= ix->info.frameMax * ix->info.imageSize)
    {
        remain_cnt = ix->info.frameMax * ix->info.imageSize - ix->data_offset;

        error = copy_to_user(buff, &ix->huge_buffer[ix->data_offset], remain_cnt);
        if (error < 0)
        {
            dev_err(&ix->spi->dev, "[ERROR] copy_to_user failed.\n");
            error = -EFAULT;
            goto out;
        }

        ix->data_offset = 0;
        read_cnt -= remain_cnt;
        ix->avail_data -= remain_cnt;
        if (ix->avail_data == 0)
        {
            error = remain_cnt;
            goto out;
        }
    }

    if (read_cnt > 0)
    {
        error = copy_to_user(buff + remain_cnt, &ix->huge_buffer[ix->data_offset], read_cnt);
        if (error < 0)
        {
            dev_err(&ix->spi->dev, "[ERROR] copy_to_user failed.\n");
            error = -EFAULT;
            goto out;
        }

        ix->data_offset += read_cnt;
        ix->avail_data -= read_cnt;
        error = read_cnt + remain_cnt;
    }

out:
    up(&ix->mutex);

    return error;
}

/* -------------------------------------------------------------------- */
static ssize_t ix_write(struct file *file, const char *buff, size_t count, loff_t *ppos)
{
    pr_debug(KERN_DEBUG ": [DEBUG] %s:%i %s\n", __FILE__, __LINE__, __func__);
    return -ENOTTY;
}

/* -------------------------------------------------------------------- */
static unsigned int ix_poll(struct file *file, poll_table *wait)
{
    unsigned int ret = 0;
    struct ix_data *ix = file->private_data;

    if (down_interruptible(&ix->mutex))
        return -ERESTARTSYS;

    if ( (ix->avail_data == 0) && (ix->capture_done == false) )
        poll_wait(file, &ix->waiting_data_avail, wait);

    if (ix->avail_data > 0)
    {
        ret |= (POLLIN | POLLRDNORM);
    }
    else if (ix->capture_done)
    {
        ret |= POLLHUP;
    }

    up(&ix->mutex);

    return ret;
}

/* -------------------------------------------------------------------- */
static long ix_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    int error;
    struct ix_data *ix = filp->private_data;
    error = 0;

//  pr_debug(KERN_DEBUG ": [DEBUG] %s:%i %s\n", __FILE__, __LINE__, __func__);

    if (ix->isSleep == true)
        return -EAGAIN;

    if (down_interruptible(&ix->mutex))
        return -ERESTARTSYS;

    switch (cmd)
    {
        case IX_IOCTL_START_CAPTURE:
            ix_start_capture(ix);
            break;

        case IX_IOCTL_ABORT_CAPTURE:
            ix_refresh(ix);
            break;

        case IX_IOCTL_SET_FIRST_TRY:
            ix->firstTry = arg;
            break;

        case IX_IOCTL_CHECK_FINGER:
            ix->request_finger = arg;
            error = ix->finger_status;
            break;

        case IX_IOCTL_CHECK_WAKE_UP:
            if (ix->isSleep == true)
                error = -EAGAIN;
            else
                error = 0;
            break;

#ifdef REARRANGED_ADC_TABLE
        case IX_IOCTL_SET_ADC_TABLE_INDEX:
        {
            int size = 0;
            int index = 0;

            error = copy_from_user( (void *)&g_adc_table_index_info, (void *)arg, sizeof(g_adc_table_index_info) );
            size = g_adc_table_index_info.size;
            for(index = 0; index < size; index++)
            {
                pr_debug(KERN_DEBUG " : [DEBUG] g_adc_table_index_info[%d].index = %d\n", index, g_adc_table_index_info.index[index]);
            }
        }
            break;
#endif

#if defined (__TEST_APP__)
        case IX_IOCTL_GET_HW_ID:
            error = ix_get_hw_id(ix);
            if (error)
            {
                dev_err(&ix->spi->dev,"[error] ioctl - hardware id get failed\n");
                break;
            }
            break;
#endif

        default:
            error = -ENOTTY;
            break;
    }
    up(&ix->mutex);

    return error;
}

/* -------------------------------------------------------------------- */
static ssize_t ix_show_attr_info(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct ix_data *ix;
    struct ix_attribute *ix_attr;
    ix = dev_get_drvdata(dev);
    ix_attr = container_of(attr, struct ix_attribute, attr);

    if (ix_attr->offset == offsetof(struct ix_info, chipID))
    {
#if defined(__TEST_APP__)
         int result = ix_get_hw_id(ix);
         if(result == 0)
             return scnprintf(buf, PAGE_SIZE, "%s: ok\n", attr->attr.name);
         else if(result == -EIO)
             return scnprintf(buf, PAGE_SIZE, "%s: id mismatched\n", attr->attr.name);
        else
            return scnprintf(buf, PAGE_SIZE, "%s: sync failed\n", attr->attr.name);
#else
        return scnprintf(buf, PAGE_SIZE, "0x%02x\n", ix->info.chipID);
#endif
    }

    if (ix_attr->offset == offsetof(struct ix_info, revNO))
    {
        return scnprintf(buf, PAGE_SIZE, "0x%02x\n", ix->info.revNO);
    }

    if (ix_attr->offset == offsetof(struct ix_info, version))
    {
        return scnprintf(buf, PAGE_SIZE, "%s\n", ix->info.version);
    }

    return -ENOENT;
}

/* -------------------------------------------------------------------- */
static ssize_t ix_store_attr_info(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    return -EPERM;
}

/* -------------------------------------------------------------------- */
static ssize_t ix_show_attr_adc_setup(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct ix_data *ix;
    struct ix_attribute *ix_attr;
    ix = dev_get_drvdata(dev);
    ix_attr = container_of(attr, struct ix_attribute, attr);

    if (ix_attr->offset == offsetof(struct ix_adc_setup, index))
    {
        return scnprintf(buf, PAGE_SIZE, "%d\n",ix->adc_setup.index);
    }

    if (ix_attr->offset == offsetof(struct ix_adc_setup, pga_set0))
    {
        return scnprintf(buf, PAGE_SIZE, "%x\n", ix->adc_setup.pga_set0[ix->adc_setup.index]);
    }

    if (ix_attr->offset == offsetof(struct ix_adc_setup, pga_set1))
    {
        return scnprintf(buf, PAGE_SIZE, "%x\n", ix->adc_setup.pga_set1[ix->adc_setup.index]);
    }

    if (ix_attr->offset == offsetof(struct ix_adc_setup, vref_set2))
    {
        return scnprintf(buf, PAGE_SIZE, "%x\n", ix->adc_setup.vref_set2[ix->adc_setup.index]);
    }

    if (ix_attr->offset == offsetof(struct ix_adc_setup, cell_amp_set))
    {
        return scnprintf(buf, PAGE_SIZE, "%x\n", ix->adc_setup.cell_amp_set[ix->adc_setup.index]);
    }

    return -ENOENT;
}

/* -------------------------------------------------------------------- */
static ssize_t ix_store_attr_adc_setup(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    u64 val;
    int error = 0;
    struct ix_data *ix;
    struct ix_attribute *ix_attr;
    ix = dev_get_drvdata(dev);

    error = kstrtou64(buf, 0, &val);
    ix_attr = container_of(attr, struct ix_attribute, attr);

    if(!error) {
        if (ix_attr->offset == offsetof(struct ix_adc_setup, pga_set0))
            ix->adc_setup.pga_set0[ix->adc_setup.index] = (u8)val;
        else if (ix_attr->offset == offsetof(struct ix_adc_setup, pga_set1))
            ix->adc_setup.pga_set1[ix->adc_setup.index] = (u8)val;
        else if (ix_attr->offset == offsetof(struct ix_adc_setup, vref_set2))
            ix->adc_setup.vref_set2[ix->adc_setup.index] = (u8)val;
        else if (ix_attr->offset == offsetof(struct ix_adc_setup, cell_amp_set))
            ix->adc_setup.cell_amp_set[ix->adc_setup.index] = (u8)val;
        else if (ix_attr->offset == offsetof(struct ix_adc_setup, index))
            ix->adc_setup.index = (int)val;
        else
            return -ENOENT;

        return strnlen(buf, count);
    }

    return error;
}

/* -------------------------------------------------------------------- */
static ssize_t ix_show_attr_diag(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct ix_data *ix;
    struct ix_attribute *ix_attr;

    ix = dev_get_drvdata(dev);
    ix_attr = container_of(attr, struct ix_attribute, attr);

    if (ix_attr->offset == offsetof(struct ix_diag, selftest))
    {
        ix_selftest_short(ix);
        return scnprintf(buf, PAGE_SIZE, "%i\n", ix->diag.selftest);
    }

    if (ix_attr->offset == offsetof(struct ix_diag, sensortest))
    {
        ix_sensor_test(ix);
        return scnprintf(buf, PAGE_SIZE, "%i\n", ix->diag.sensortest);
    }

    if (ix_attr->offset == offsetof(struct ix_diag, capture_time))
    {
        return scnprintf(buf, PAGE_SIZE, "%i\n", ix->diag.capture_time);
    }

    if (ix_attr->offset == offsetof(struct ix_diag, frames_captured))
    {
        return scnprintf(buf, PAGE_SIZE, "%i\n", ix->diag.frames_captured);
    }

    if (ix_attr->offset == offsetof(struct ix_diag, frames_stored))
    {
        return scnprintf(buf, PAGE_SIZE, "%i\n", ix->diag.frames_stored);
    }

    if (ix_attr->offset == offsetof(struct ix_diag, finger_threshold))
    {
        return scnprintf(buf, PAGE_SIZE, "%i\n", ix->diag.finger_threshold);
    }

    if (ix_attr->offset == offsetof(struct ix_diag, adc_count)){
        return scnprintf(buf, PAGE_SIZE, "%i\n", ix->diag.adc_count);
    }

    return -ENOENT;
}

/* -------------------------------------------------------------------- */
static ssize_t ix_store_attr_diag(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    u32 *target;
    u32 tmp;
    struct ix_data *ix;
    struct ix_attribute *ix_attr;
    ix = dev_get_drvdata(dev);
    ix_attr = container_of(attr, struct ix_attribute, attr);

    if ((sscanf(buf, "%d", &tmp)) <= 0)
        return -EINVAL;

    target = ((void *)&ix->diag) + ix_attr->offset;
    *target = tmp;

    return strnlen(buf, count);
    //return -EPERM;
}

/* -------------------------------------------------------------------- */
static ssize_t ix_show_attr_reg_setup(struct device *dev, struct device_attribute *attr, char *buf)
{
    u8 *target;
    struct ix_data *ix;
    struct ix_attribute *ix_attr;
    ix = dev_get_drvdata(dev);
    ix_attr = container_of(attr, struct ix_attribute, attr);

    target = ((u8 *)&ix->reg_setup) + ix_attr->offset;
    return scnprintf(buf, PAGE_SIZE, "%s: %i\n", attr->attr.name, *target);
}

/* -------------------------------------------------------------------- */
static ssize_t ix_store_attr_reg_setup(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    u8 *target;
    u8 tmp;

    struct ix_data *ix;
    struct ix_attribute *ix_attr;
    ix = dev_get_drvdata(dev);
    ix_attr = container_of(attr, struct ix_attribute, attr);

    if ((sscanf(buf, "%hhu", &tmp)) <= 0)
        return -EINVAL;

    target = ((u8 *)&ix->reg_setup) + ix_attr->offset;
    *target = tmp;

    return strnlen(buf, count);
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

    pr_info(KERN_INFO ": [INFO] irq : %d  reset : %d\n", pdata->irq_gpio, pdata->reset_gpio);

    return 0;

of_err:
    pdata->reset_gpio = -EINVAL;
    pdata->irq_gpio   = -EINVAL;

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

    if (down_interruptible(&ix->mutex))
        return 0;

    if (evdata && evdata->data && event == FB_EVENT_BLANK && ix) {
        blank = (evdata->data);
        dev_dbg(&ix->spi->dev, "%s  %d \n", __func__, *blank);

        if (*blank == FB_BLANK_UNBLANK) //resume
        {
            ix->isSleep = false;

        }
        else //suspend
        {
            if (!ix->isSleep)
            {
                ix->isSleep = true;
                ix_mode_change(ix, IX_MODE_STANDBY);
            }
        }
    }
    up(&ix->mutex);

    return 0;
}
#elif defined(CONFIG_HAS_EARLYSUSPEND)
/* -------------------------------------------------------------------- */
static void ix_early_suspend(struct early_suspend *h)
{
    struct ix_data *ix;
    ix = container_of(h, struct ix_data, early_suspend);

    if (down_interruptible(&ix->mutex))
        return;

    ix->isSleep = true;
    ix_mode_change(ix, IX_MODE_STANDBY);

    up(&ix->mutex);
    return;
}

/* -------------------------------------------------------------------- */
static void ix_late_resume(struct early_suspend *h)
{
    struct ix_data *ix;
    ix = container_of(h, struct ix_data, early_suspend);

    if (down_interruptible(&ix->mutex))
        return;

    fpc1080->isSleep = false;
    up(&ix->mutex);

    return;
}
#endif

/* -------------------------------------------------------------------- */
irqreturn_t ix_interrupt(int irq, void *_ix)
{
    struct ix_data *ix = _ix;

    if ( gpio_get_value(ix->irq_gpio) )
    {
        ix->interrupt_done = 1;
        wake_up_interruptible(&ix->waiting_interrupt_return);
        return IRQ_HANDLED;
    }

    return IRQ_NONE;
}

/* -------------------------------------------------------------------- */
static int ix_cleanup(struct ix_data *ix)
{
    pr_debug(KERN_DEBUG ": [DEBUG] %s:%i %s\n", __FILE__, __LINE__, __func__);

    if (ix->thread_task.thread)
    {
        ix->thread_task.should_stop = 1;
        ix->thread_task.mode = IX_THREAD_EXIT;
        wake_up_interruptible(&ix->thread_task.wait_job);
        kthread_stop(ix->thread_task.thread);
    }

    if (!IS_ERR_OR_NULL(ix->device))
        device_destroy(ix->class, ix->devno);

    class_destroy(ix->class);

    if (ix->irq >= 0)
        free_irq(ix->irq, ix);

    if (gpio_is_valid(ix->irq_gpio))
        gpio_free(ix->irq_gpio);

    if (gpio_is_valid(ix->reset_gpio))
        gpio_free(ix->reset_gpio);

    if (ix->huge_buffer)
    {
        free_pages((unsigned long)ix->huge_buffer, get_order(IX_IMAGE_BUFFER_SIZE));
    }

    kfree(ix);

    return 0;
}

/* -------------------------------------------------------------------- */
static int ix_wait_for_irq(struct ix_data *ix, int timeout)
{
    int result;

    //pr_debug(KERN_DEBUG ": [DEBUG] %s:%i %s\n", __FILE__, __LINE__, __func__);

    if (!timeout)
    {
        result = wait_event_interruptible(ix->waiting_interrupt_return, ix->interrupt_done);
    }
    else
    {
        result = wait_event_interruptible_timeout(ix->waiting_interrupt_return, ix->interrupt_done, timeout);
    }

    if (result < 0)
    {
        dev_err(&ix->spi->dev, "[ERROR] wait_event_interruptible - interrupted by signal.\n");
        return result;
    }

    if (result || !timeout)
    {
        ix->interrupt_done = 0;
        return 0;
    }

    return -ETIMEDOUT;
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

#ifdef __TEST_APP__
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
#endif

/* -------------------------------------------------------------------- */
static int ix_spi_read_reg(struct ix_data *ix, ix_cmd_reg_t addr, u8 *rx)
{
    int error;
    u8 tx[3];

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

/* -------------------------------------------------------------------- */
static int ix_spi_read_image(struct ix_data *ix)
{
    int error;
    u8 tx[IX_CMD_SIZE];
    int pos;
    u8 rx;
    int i;

    struct spi_message m;
    struct spi_transfer t;

    memset(&t, 0, sizeof(struct spi_transfer));

    if (ix->current_frame >= ix->info.frameMax)
    {
        ix->current_frame = 0;
    }

    tx[0] = IX_SPI_READ_COMMAND;
    tx[1] = IX_CMD_READ;
    tx[2] = 0x00;   /* dummy byte */

    t.cs_change = CS_CHANGE_VALUE,
    t.delay_usecs = 0,
    t.speed_hz = IX_SPI_CLOCK_SPEED,
    t.tx_buf = tx,
    t.rx_buf = ix->huge_buffer + (ix->current_frame * (ix->info.imageSize)),
    t.len = IX_CMD_SIZE + ix->info.imageSize,
    t.tx_dma = 0,
    t.rx_dma = 0,
    t.bits_per_word = 0,

    spi_message_init(&m);
    spi_message_add_tail(&t, &m);

    error = ix_spi_write_cmd(ix, IX_CMD_SCAN);
    if (error)
        return error;

    error = ix_spi_read_wait_irq(ix, &rx, true);
    if (error)
        return error;

    if (!(rx & IX_IRQ_SCAN_COMPLETE))
    {
        dev_err(&ix->spi->dev, "[ERROR] %s:%i  irq : %x expected %x\n", __FILE__, __LINE__, rx, IX_IRQ_SCAN_COMPLETE);
        return -EINVAL;
    }

    error = spi_sync(ix->spi, &m);
    if (error)
    {
        dev_err(&ix->spi->dev, "[ERROR] spi_sync failed.\n");
        return error;
    }

    error = ix_spi_read_wait_irq(ix, &rx, true);
    if (error)
        return error;

    if (!(rx & IX_IRQ_MEMORY_READ_END))
    {
        dev_err(&ix->spi->dev, "[ERROR] %s:%i  irq : %x expected %x\n", __FILE__, __LINE__, rx, IX_IRQ_MEMORY_READ_END);
        return -EINVAL;
    }

    error = ix_spi_write_reg(ix, IX_CMD_INT_RC, 0x00);
    if(error)
        return error;

    error = ix_spi_write_cmd(ix, IX_CMD_IDLE);
    if (error)
        return error;

    pos = ix->current_frame * ix->info.imageSize;
    memcpy(&ix->huge_buffer[pos], &ix->huge_buffer[pos + IX_CMD_SIZE], ix->info.imageSize);

    //invert
    for(i=0; i<ix->info.imageSize; i++)
    {
        ix->huge_buffer[pos + i] = 255 - ix->huge_buffer[pos + i];
    }

    return 0;
}

/* -------------------------------------------------------------------- */
static int ix_spi_read_wait_irq(struct ix_data *ix, u8 *irq, bool interrupt_clear)
{
    int error;

    while(true)
    {
        if (ix->thread_task.should_stop)
        {
            dev_info(&ix->spi->dev, "[INFO] ix_spi_read_wait_irq cancel.\n");
            return -EINTR;
        }

        error = ix_wait_for_irq(ix, IX_DEFAULT_IRQ_TIMEOUT);
        if (error == 0)
            break;

        if (error != -ETIMEDOUT)
        {
            dev_err(&ix->spi->dev, "[ERROR] wait_irq timeout.\n");
            return error;
        }
    }

    while(true)
    {
        if (ix->thread_task.should_stop)
        {
            dev_info(&ix->spi->dev, "[INFO] ix_spi_read_wait_irq cancel.\n");
            return -EINTR;
        }

        //read irq
        error = ix_spi_read_reg(ix, IX_CMD_INT_R, irq);
        if (error)
        {
            dev_err(&ix->spi->dev, "[ERROR] IX_CMD_INT_R failed.\n");
            return error;
        }

        if (*irq == 0x00)
            continue;

        //interrupt clear
        if (interrupt_clear)
        {
            error = ix_spi_write_reg(ix, IX_CMD_INT_RC, 0x00);
            if (error)
            {
                dev_err(&ix->spi->dev, "[ERROR] IX_CMD_INT_RC failed.\n");
                return error;
            }
        }

        break;
    }

    return error;
}

/* -------------------------------------------------------------------- */
static void ix_set_info(struct ix_data *ix)
{
    pr_debug(KERN_DEBUG ": [DEBUG] %s:%i %s\n", __FILE__, __LINE__, __func__);

    ix->info.version = IX_VERSION;
    ix->info.imageWidth = IX_SENSOR_WIDTH;
    ix->info.imageHeight = IX_SENSOR_HEIGHT;
    ix->info.imageSize = ix->info.imageWidth * ix->info.imageHeight;
    ix->info.frameMax = IX_MAX_FRAME;
}

/* -------------------------------------------------------------------- */
static void ix_refresh(struct ix_data *ix)
{
    pr_debug(KERN_DEBUG ": [DEBUG] %s:%i %s\n", __FILE__, __LINE__, __func__);

    ix_thread_goto_idle(ix);
    ix->avail_data = 0;
    ix->current_frame = 0;
    ix->data_offset = 0;
    ix->capture_done = false;
    ix->request_finger = FNGR_ST_NONE;
}

/* -------------------------------------------------------------------- */
static int ix_reset(struct ix_data *ix)
{
    int error;
    u8 rx;

    pr_debug(KERN_DEBUG ": [DEBUG] %s:%i %s\n", __FILE__, __LINE__, __func__);

    gpio_set_value(ix->reset_gpio, 0);
    mdelay(1);
    gpio_set_value(ix->reset_gpio, 1);
    mdelay(1);

    disable_irq(ix->irq);
    ix->interrupt_done = 0;
    enable_irq(ix->irq);

    error = ix_spi_read_reg(ix, IX_CMD_INT_R, &rx);
    if (error)
    {
        dev_err(&ix->spi->dev, "[ERROR] IX_CMD_INT_R failed.\n");
        return error;
    }

    error = ix_spi_write_reg(ix, IX_CMD_INT_RC, 0x00);
    if (error)
    {
        dev_err(&ix->spi->dev, "[ERROR] IX_CMD_INT_RC failed.\n");
        return error;
    }

    error = ix_init_param(ix);
    if (error)
    {
        dev_err(&ix->spi->dev, "[ERROR] ix_init_param failed.\n");
        return error;
    }

    error = ix_spi_write_cmd(ix, IX_CMD_IDLE); //IDLE
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

    pr_debug(KERN_DEBUG ": [DEBUG] %s:%i %s\n", __FILE__, __LINE__, __func__);

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

    dev_info(&ix->spi->dev, "[INFO] *****************************************\n");
    dev_info(&ix->spi->dev, "[INFO] * Chip ID[0x%02x] : Revision Number[0x%02x] *\n", ix->info.chipID, ix->info.revNO);
    dev_info(&ix->spi->dev, "[INFO] *****************************************\n");

    if (ix->info.chipID != IX_HW_ID_SP2)
    {
        dev_err(&ix->spi->dev, "[ERROR] hardware id mismatch : %02x expected %02x\n", ix->info.chipID, IX_HW_ID_SP2);
        return -EIO;
    }

    return 0;
}

/* -------------------------------------------------------------------- */
static void ix_adc_initial_set(struct ix_data *ix)
{
    int i = 0;

    for(i = 0; i < IX_ADC_COUNT; i++)
    {
        ix->adc_setup.pga_set0[i] = ix_adc_table[i][0];
        ix->adc_setup.pga_set1[i] = ix_adc_table[i][1];
        ix->adc_setup.vref_set2[i] = ix_adc_table[i][2];
        ix->adc_setup.cell_amp_set[i] = ix_adc_table[i][3];
    }
}

/* -------------------------------------------------------------------- */
static int ix_auto_adc_ctrl(struct ix_data *ix, int count)
{
    int error;

    /* setting adc */
    error = ix_spi_write_reg(ix, IX_REG_PGA_SET0, ix->adc_setup.pga_set0[count]);
    if (error)
        return error;

    error = ix_spi_write_reg(ix, IX_REG_PGA_SET1, ix->adc_setup.pga_set1[count]);
    if (error)
        return error;

    error = ix_spi_write_reg(ix, IX_REG_VREF_SET2, ix->adc_setup.vref_set2[count]);
    if (error)
        return error;

    error = ix_spi_write_reg(ix, IX_REG_CELL_AMP_SET, ix->adc_setup.cell_amp_set[count]);
    if (error)
        return error;

    return 0;
}

/* -------------------------------------------------------------------- */
static int ix_mode_change(struct ix_data *ix, int mode)
{
    int error;

    pr_debug(KERN_DEBUG ": [DEBUG] %s:%i %s\n", __FILE__, __LINE__, __func__);

    switch(mode)
    {
        case IX_MODE_CAPTURE:
            error = ix_spi_write_cmd(ix, IX_CMD_IDLE);
            if (error)
                return error;

            error = ix_spi_write_reg(ix, IX_REG_IMG_DET_CTRL, 0x81);
            if (error)
                return error;

            error = ix_spi_write_reg(ix, IX_REG_SENS_CTRL, 0x00);
            if (error)
                return error;

            error = ix_spi_write_reg(ix, IX_REG_IMG_DET_L_THRES, 0x00);
            if (error)
                return error;

            error = ix_spi_write_reg(ix, IX_CMD_INT_RC, 0x00);
            if (error)
                return error;

            error = ix_spi_write_reg(ix, IX_REG_INT_SRC_MASK, 0x08);
            if (error)
                return error;

            break;

        case IX_MODE_FINGERDOWN:
            error = ix_spi_write_cmd(ix, IX_CMD_IDLE);
            if (error)
                return error;

            error = ix_spi_write_reg(ix, IX_REG_IMG_DET_CTRL, 0xC1);
            if (error)
                return error;

            error = ix_spi_write_reg(ix, IX_REG_SENS_CTRL, 0x40);
            if (error)
                return error;

            error = ix_spi_write_reg(ix, IX_REG_IMG_DET_L_THRES, 0x40);
            if (error)
                return error;

            error = ix_spi_write_reg(ix, IX_CMD_INT_RC, 0x00);
            if (error)
                return error;

            error = ix_spi_write_reg(ix, IX_REG_INT_SRC_MASK, 0x18);
            if (error)
                return error;

            break;

        case IX_MODE_STANDBY:

#if defined (__TEST_APP__)
			gpio_set_value(ix->reset_gpio, 0);
			mdelay(1);
		
			error = ix_regulator_set(ix, false);
			if (error)
			{
				dev_err(&ix->spi->dev,"ix_regulator_set err\n");
				return -EIO;
			}
#else
            ix_reset(ix);
#endif			
            ix_refresh(ix);

            break;

        default:
            break;
    }

    return 0;
}

/* -------------------------------------------------------------------- */
static int ix_wait_request_finger(struct ix_data *ix, int request_finger)
{
    while(request_finger != ix->request_finger)
    {
        if (ix->thread_task.should_stop)
        {
            return -EINTR;
        }
        msleep(10);
    }
    return 0;
}

/* -------------------------------------------------------------------- */
static int ix_wait_finger_present(struct ix_data *ix)
{
    int error;
    int i;
    u8 rx;

    pr_debug(KERN_DEBUG ": [DEBUG] %s:%i %s\n", __FILE__, __LINE__, __func__);

    while(true)
    {
        if (ix->thread_task.should_stop)
        {
            dev_info(&ix->spi->dev, "[INFO] wait finger down cancel.\n");
            return -EINTR;
        }

        error = ix_mode_change(ix, IX_MODE_FINGERDOWN);
        if (error)
            return error;

        error = ix_spi_write_cmd(ix, IX_CMD_SCAN);
        if (error)
            return error;

        error = ix_spi_read_wait_irq(ix, &rx, true);
        if (error)
            return error;

        if (rx & IX_IRQ_IDET_FINGER_ON)
        {
            pr_debug(KERN_DEBUG ": [DEBUG] finger present OK\n");

            error = ix_mode_change(ix, IX_MODE_CAPTURE);
            if (error)
                return error;

            ix_auto_adc_ctrl(ix, 0);

            for (i = 0; i < IX_FINGER_DETECT_RETRY_COUNT; i++)
            {
                error = ix_spi_read_image(ix);
                if (error)
                {
                    return -EINTR;
                }

                if (ix_finger_check(ix, 6) == IX_IRQ_IDET_FINGER_ON)
                    return 0;

                mdelay(20);
            }
        }
        else
        {
            dev_err(&ix->spi->dev, "[ERROR] irq : 0x%02x expected 0x%02x\n", rx, IX_IRQ_IDET_FINGER_ON);

            error = ix_spi_write_reg(ix, IX_CMD_INT_RC, 0x00);
            if (error)
                return error;
        }
    }
}

/* -------------------------------------------------------------------- */
static int ix_wait_finger_detach(struct ix_data *ix)
{
    int error = 0;

    pr_debug(KERN_DEBUG ": [DEBUG] %s:%i %s\n", __FILE__, __LINE__, __func__);

    error = ix_mode_change(ix, IX_MODE_CAPTURE);
    if (error)
        return error;

    error = ix_auto_adc_ctrl(ix, 0);

    while(true)
    {
        if (ix->thread_task.should_stop)
        {
            dev_info(&ix->spi->dev, "[INFO] capture task cancel.\n");
            return -EINTR;
        }

        error = ix_spi_read_image(ix);
        if (error)
        {
            dev_err(&ix->spi->dev, "[ERROR] read image failed.\n");
            return -EINTR;
        }

        if (ix_finger_check(ix, 0) == IX_IRQ_IDET_FINGER_OFF)
        {
            dev_info(&ix->spi->dev, "[INFO] %s:%i  %s finger up.\n", __FILE__, __LINE__, __func__);
            break;
        }
    }

    return 0;
}

/* -------------------------------------------------------------------- */
static u8 ix_finger_check(struct ix_data *ix, int count)
{
    int x_1 = 8;
    int x_2 = ix->info.imageWidth / 2 - 4;
    int x_3 = ix->info.imageWidth - 16;
    int y_1 = 8;
    int y_2 = ix->info.imageHeight / 2 - 4;
    int y_3 = ix->info.imageHeight - 16;

    int coord[][2] = {
        {x_1, y_1}, {x_2, y_1}, {x_3, y_1},
        {x_1, y_2}, {x_2, y_2}, {x_3, y_2},
        {x_1, y_3}, {x_2, y_3}, {x_3, y_3},
    };

    int i, x, y, pos;
    int detect_area_cnt = 0;

    for (i = 0; i < IX_FINGER_DETECT_COUNT; i++)
    {
        ix->pxl_sum[i] = 0;

        for (y = 0; y < 8; y++)
        {
            for (x = 0; x < 8; x++)
            {
                pos = (coord[i][1] + y) * ix->info.imageWidth + coord[i][0] + x;
                ix->pxl_sum[i]+=ix->huge_buffer[pos + (ix->current_frame * ix->info.imageSize)];
            }
        }

        if (ix->pxl_sum[i] > (int)ix->diag.finger_threshold)
            detect_area_cnt++;
    }

    if (detect_area_cnt > count)
    {
        ix->finger_status = FNGR_ST_DETECTED;
        return IX_IRQ_IDET_FINGER_ON;
    }
    else
    {
        ix->finger_status = FNGR_ST_LOST;
        return IX_IRQ_IDET_FINGER_OFF;
    }
}

/* -------------------------------------------------------------------- */
static int ix_selftest_short(struct ix_data *ix)
{
    int error;
    u8 rx;

    pr_debug(KERN_DEBUG ": [DEBUG] %s:%i %s\n", __FILE__, __LINE__, __func__);

    ix_refresh(ix);

    error = ix_reset(ix);
    if (error)
    {
        dev_err(&ix->spi->dev, "ix selftest, reset fail on entry.\n");
        goto err;
    }

    error = ix_spi_read_reg(ix, IX_REG_CHIP_ID, &rx);
    if (error)
    {
        dev_err(&ix->spi->dev, "[ERROR] IX_REG_CHIP_ID failed.\n");
        goto err;
    }

    dev_info(&ix->spi->dev, "[INFO] chip ID : 0x%02x\n", rx);
    if (rx != IX_HW_ID_SP2)
    {
        dev_err(&ix->spi->dev, "[ERROR] hardware id mismatch : %02x expected %02x\n", rx, IX_HW_ID_SP2);
        goto err;
    }

    ix_refresh(ix);

    error = ix_reset(ix);
    if (error)
    {
        dev_err(&ix->spi->dev, "ix selftest, reset fail on entry.\n");
        goto err;
    }

err:
    ix->diag.selftest = (error == 0)? 1 : 0;

    return error;
}

/* -------------------------------------------------------------------- */
static int ix_sensor_test(struct ix_data *ix)
{
    return 1;
}

/* -------------------------------------------------------------------- */
static int threadfn(void *_ix)
{
    struct ix_data *ix = _ix;

    while (!kthread_should_stop())
    {
        up(&ix->thread_task.sem_idle);
        wait_event_interruptible(ix->thread_task.wait_job, ix->thread_task.mode != IX_THREAD_IDLE_MODE);

        down(&ix->thread_task.sem_idle);

        switch (ix->thread_task.mode)
        {
            case IX_THREAD_CAPTURE_MODE:
                ix_capture_task(ix);
                break;

            default:
                break;
        }

        if(ix->thread_task.mode != IX_THREAD_EXIT)
            ix->thread_task.mode = IX_THREAD_IDLE_MODE;
    }

    return 0;
}

/* -------------------------------------------------------------------- */
static int ix_start_thread(struct ix_data *ix, int mode)
{
    pr_debug(KERN_DEBUG ": [DEBUG] %s:%i %s\n", __FILE__, __LINE__, __func__);

    ix->thread_task.should_stop = 0;
    ix->thread_task.mode = mode;
    wake_up_interruptible(&ix->thread_task.wait_job);

    return 0;
}

/* -------------------------------------------------------------------- */
static void ix_start_capture(struct ix_data *ix)
{
    pr_debug(KERN_DEBUG ": [DEBUG] %s:%i %s\n", __FILE__, __LINE__, __func__);

    ix_refresh(ix);
    ix_start_thread(ix, IX_THREAD_CAPTURE_MODE);
    ix->capture_done = false;
}

/* -------------------------------------------------------------------- */
static int ix_thread_goto_idle(struct ix_data *ix)
{
    pr_debug(KERN_DEBUG ": [DEBUG] %s:%i %s\n", __FILE__, __LINE__, __func__);

    ix->thread_task.should_stop = 1;
    ix->thread_task.mode = IX_THREAD_IDLE_MODE;
    if (down_interruptible(&ix->thread_task.sem_idle))
        return -ERESTARTSYS;

    up(&ix->thread_task.sem_idle);

    return 0;
}

/* -------------------------------------------------------------------- */
static int ix_capture_task(struct ix_data *ix)
{
    int error = 0;
    struct timespec ts_start, ts_end, ts_delta;
    u8 finger_status;

    pr_debug(KERN_DEBUG ": [DEBUG] %s:%i %s\n", __FILE__, __LINE__, __func__);

    ix->diag.frames_captured = 0;
    ix->diag.frames_stored = 0;
    ix->diag.capture_time = 0;
    ix->current_adc_table = 0;
    ix->finger_status = FNGR_ST_NONE;
    ix->current_frame = 0;
    ix->avail_data = 0;
    ix->data_offset = 0;

    if (!ix->firstTry)
    {
        error = ix_wait_finger_detach(ix);
        if (error)
        {
            error = -EINTR;
            goto out;
        }
        error = ix_wait_request_finger(ix, FNGR_ST_LOST);
        if (error)
            goto out;
    }

    error = ix_wait_finger_present(ix);
    if (error)
    {
        error = -EINTR;
        goto out;
    }

    error = ix_wait_request_finger(ix, FNGR_ST_DETECTED);
    if (error)
        goto out;

    getnstimeofday(&ts_start);

    while(true)
    {
        if (ix->thread_task.should_stop)
        {
            dev_info(&ix->spi->dev, "[INFO] capture task cancel.\n");
            error = -EINTR;
            break;
        }

        if (ix->current_adc_table < 3)
        {
#ifdef REARRANGED_ADC_TABLE
            int adc_index = g_adc_table_index_info.index[ix->current_adc_table];
            dev_err(&ix->spi->dev, "[INFO] adc_index : %d\n", adc_index);

            ix_auto_adc_ctrl(ix, adc_index);
#else
            ix_auto_adc_ctrl(ix, ix->current_adc_table);
#endif
        }
        else
            break;

        error = ix_spi_read_image(ix);
        if (error)
        {
            dev_err(&ix->spi->dev, "[ERROR] read image failed.\n");
            goto out;
        }

        ix->diag.frames_captured++;

        finger_status = ix_finger_check(ix, 6);

        if (finger_status == IX_IRQ_IDET_FINGER_OFF && (ix->diag.frames_captured > 1) )
        {
            dev_info(&ix->spi->dev, "[INFO] %s:%i finger up.\n", __FILE__, __LINE__);
            break;
        }
        else
        {
            dev_info(&ix->spi->dev, "[INFO] %s:%i adc table index : %d\n", __FILE__, __LINE__, ix->current_adc_table);

            ix->avail_data += ix->info.imageSize;
            ix->current_frame++;
            ix->diag.frames_stored++;

            ix->current_adc_table++;
        }

        wake_up_interruptible(&ix->waiting_data_avail);
    }

    getnstimeofday(&ts_end);
    ts_delta = timespec_sub(ts_end, ts_start);

    ix->diag.capture_time = ts_delta.tv_nsec / NSEC_PER_MSEC;
    ix->diag.capture_time += (ts_delta.tv_sec * MSEC_PER_SEC);

    if (ix->diag.capture_time > 0)
    {
        dev_info(&ix->spi->dev, "[INFO] captured %lu frames (%lu kept) in %lu  ms (%lu fps)\n",
                (long unsigned int)ix->diag.frames_captured,
                (long unsigned int)ix->diag.frames_stored,
                (long unsigned int)ix->diag.capture_time,
                (long unsigned int)(ix->diag.frames_stored * MSEC_PER_SEC / ix->diag.capture_time));
    }

out:
    ix_reset(ix);

    if (error)
        ix->finger_status = FNGR_ST_NONE;
    else
        ix->finger_status = FNGR_ST_LOST;

    ix->capture_done = true;
    wake_up_interruptible(&ix->waiting_data_avail);

    return error;
}

