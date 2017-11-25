/*
 * Copyright (C) 2012 Broadcom Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/jiffies.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <linux/spinlock.h>
#include <linux/poll.h>
#include <linux/version.h>
#include <linux/of_gpio.h>
#include <soc/qcom/lge/board_lge.h>
//#include <soc/qcom/lge/lge_boot_mode.h> //for msm8909

#include <linux/nfc/bcm2079x.h>
#include <linux/wakelock.h>


//#define BCM2079X_M_PLATFORM 1
#ifdef CONFIG_BCM2079X_MTK_PLATFORM
#include <cust_gpio_usage.h>
#include <cust_eint.h>

#include <mach/mt_gpio.h>
#include <mach/eint.h>

#include <linux/dma-mapping.h>
#else
#include <linux/clk.h>
#endif

/* do not change below */
#define MAX_BUFFER_SIZE     288
#define I2C_ID_NAME "bcm2079x"
#define I2C_DEVICE_ADDR 0x76

#ifdef CONFIG_BCM2079X_MTK_PLATFORM
#define GPIO_NFC_VEN  GPIO_NFC_VENB_PIN  //(105| 0x80000000)
#define GPIO_NFC_IRQ  GPIO_IRQ_NFC_PIN   //(5 | 0x80000000)
#define GPIO_NFC_WAKE GPIO_NFC_MODE  //(117 | 0x80000000)

static struct i2c_board_info __initdata nfc_board_info = {I2C_BOARD_INFO(I2C_ID_NAME, I2C_DEVICE_ADDR)};
struct bcm2079x_dev *g_bcm2079x_dev;
#define GET_GPIO(x) mt_get_gpio_in(x)
#define SET_GPIO(x,y) mt_set_gpio_out(x,y)
#else
#define GET_GPIO(x) gpio_get_value(x)
#define SET_GPIO(x,y) gpio_set_value(x,y)
#endif //CONFIG_BCM2079X_MTK_PLATFORM

struct bcm2079x_platform_data platform_data;

/* Read data */
#define PACKET_HEADER_SIZE_NCI  (4)
#define PACKET_HEADER_SIZE_HCI  (3)
#define PACKET_TYPE_NCI     (16)
#define PACKET_TYPE_HCIEV   (4)
#define MAX_PACKET_SIZE     (PACKET_HEADER_SIZE_NCI + 255)
#if defined (BCM2079X_M_PLATFORM)
#define NCI_LENGTH_OFFSET (2)
#define HCI_LENGTH_OFFSET (3)
#endif
#ifdef CONFIG_LGE_NFC_USE_PMIC
#define CLK_DISABLE 0
#define CLK_PIN 1
#define CLK_CONT 2
static struct i2c_client *bcm2079x_client;
#endif

struct bcm2079x_dev {
    wait_queue_head_t read_wq;
    struct mutex rw_mutex;
    struct i2c_client *client;
    struct miscdevice bcm2079x_device;
#ifdef CONFIG_LGE_NFC_USE_PMIC
    struct clk*clk_cont;
    struct clk*clk_pin;
#endif
    unsigned int wake_gpio;
    unsigned int en_gpio;
    unsigned int irq_gpio;
    bool irq_enabled;
    spinlock_t irq_enabled_lock;
    unsigned int count_irq;
    bool irq_wake_enabled;
};

struct wake_lock nfc_wake_lock;

#ifdef CONFIG_BCM2079X_MTK_PLATFORM
static u8*      I2CDMABuf_va = NULL;
static u32      I2CDMABuf_pa = NULL;
#else
struct  clk     *s_clk;
#endif //CONFIG_BCM2079X_MTK_PLATFORM

int nfc_i2c_dma_write(struct i2c_client *client, const uint8_t *buf, int len)
{
#ifdef CONFIG_BCM2079X_MTK_PLATFORM
    int i = 0;
    for(i = 0 ; i < len; i++) {
        I2CDMABuf_va[i] = buf[i];
    }

    if(len < 8) {
        client->addr = client->addr & I2C_MASK_FLAG | I2C_ENEXT_FLAG;
        return i2c_master_send(client, buf, len);
    }
    else {
        client->addr = client->addr & I2C_MASK_FLAG | I2C_DMA_FLAG | I2C_ENEXT_FLAG;
        return i2c_master_send(client, I2CDMABuf_pa, len);
    }
#else
    return i2c_master_send(client, buf, len);
#endif /*End of CONFIG_BCM2079X_MTK_PLATFORM*/
}

int nfc_i2c_dma_read(struct i2c_client *client, uint8_t *buf, int len)
{
#ifdef CONFIG_BCM2079X_MTK_PLATFORM
    int i = 0, ret = 0;
    if(len < 8) {
        client->addr = client->addr & I2C_MASK_FLAG | I2C_ENEXT_FLAG;
        return i2c_master_recv(client, buf, len);
    } else {
        client->addr = client->addr & I2C_MASK_FLAG | I2C_DMA_FLAG | I2C_ENEXT_FLAG;
        ret = i2c_master_recv(client, I2CDMABuf_pa, len);
        if(ret < 0) {
            return ret;
        }
        for(i = 0; i < len; i++) {
            buf[i] = I2CDMABuf_va[i];
        }
    }
    return ret;
#else
    return i2c_master_recv(client, buf, len);
#endif /*End of CONFIG_BCM2079X_MTK_PLATFORM*/
}

static void bcm2079x_init_stat(struct bcm2079x_dev *bcm2079x_dev)
{
    bcm2079x_dev->count_irq = 0;
}

static void bcm2079x_disable_irq(struct bcm2079x_dev *bcm2079x_dev)
{
    unsigned long flags;
    spin_lock_irqsave(&bcm2079x_dev->irq_enabled_lock, flags);
    if (bcm2079x_dev->irq_enabled) {
#ifdef CONFIG_BCM2079X_MTK_PLATFORM
        mt_eint_mask(bcm2079x_dev->client->irq);
#else
        disable_irq_nosync(bcm2079x_dev->client->irq);
#endif /*End of CONFIG_BCM2079X_MTK_PLATFORM*/
        if (bcm2079x_dev->irq_wake_enabled) {
#ifdef CONFIG_BCM2079X_MTK_PLATFORM
#else
        disable_irq_wake(bcm2079x_dev->client->irq);
#endif /*End of CONFIG_BCM2079X_MTK_PLATFORM*/
            bcm2079x_dev->irq_wake_enabled = false;
        }
        bcm2079x_dev->irq_enabled = false;
    }
    spin_unlock_irqrestore(&bcm2079x_dev->irq_enabled_lock, flags);
}

static void bcm2079x_enable_irq(struct bcm2079x_dev *bcm2079x_dev)
{
    unsigned long flags;
    spin_lock_irqsave(&bcm2079x_dev->irq_enabled_lock, flags);
    if (!bcm2079x_dev->irq_enabled) {
        bcm2079x_dev->irq_enabled = true;
#ifdef CONFIG_BCM2079X_MTK_PLATFORM
        mt_eint_unmask(bcm2079x_dev->client->irq);
#else
        enable_irq(bcm2079x_dev->client->irq);
#endif /*End of CONFIG_BCM2079X_MTK_PLATFORM*/
        if (!bcm2079x_dev->irq_wake_enabled) {
#ifdef CONFIG_BCM2079X_MTK_PLATFORM
#else
            enable_irq_wake(bcm2079x_dev->client->irq);
#endif /*End of CONFIG_BCM2079X_MTK_PLATFORM*/
            bcm2079x_dev->irq_wake_enabled = true;
        }
    }
    spin_unlock_irqrestore(&bcm2079x_dev->irq_enabled_lock, flags);
}

/*
 The alias address 0x79, when sent as a 7-bit address from the host processor
 will match the first byte (highest 2 bits) of the default client address
 (0x1FA) that is programmed in bcm20791.
 When used together with the first byte (0xFA) of the byte sequence below,
 it can be used to address the bcm20791 in a system that does not support
 10-bit address and change the default address to 0x38.
 the new address can be changed by changing the CLIENT_ADDRESS below if 0x38
 conflicts with other device on the same i2c bus.
 */
#define ALIAS_ADDRESS     0x79

static int change_client_addr(struct bcm2079x_dev *bcm2079x_dev, int addr)
{
    struct i2c_client *client;
    int ret;
    int i;
    char addr_data[] = {
        0xFA, 0xF2, 0x00, 0x00, 0x00, 0x38, 0x00, 0x00, 0x00, 0x2A
    };
    client = bcm2079x_dev->client;
    client->addr = ALIAS_ADDRESS;
    client->flags &= ~I2C_CLIENT_TEN;

    addr_data[5] = addr & 0xFF;
    ret = 0;
    for (i = 1; i < sizeof(addr_data) - 1; ++i)
        ret += addr_data[i];
    addr_data[sizeof(addr_data) - 1] = (ret & 0xFF);
    pr_info("Change client device from (0x%04X) flag ="
         "0x%04x, addr_data[%ld] = %02x\n",
         client->addr, client->flags, sizeof(addr_data) - 1,
         addr_data[sizeof(addr_data) - 1]);
    ret = nfc_i2c_dma_write(client, addr_data, sizeof(addr_data));
    if (ret != sizeof(addr_data)) {
        client->addr = ALIAS_ADDRESS;
        client->flags &= ~I2C_CLIENT_TEN;
        pr_info("Change client device from (0x%04X) flag ="
             "0x%04x, addr_data[%ld] = %02x\n",
             client->addr, client->flags, sizeof(addr_data) - 1,
             addr_data[sizeof(addr_data) - 1]);
        ret = nfc_i2c_dma_write(client, addr_data, sizeof(addr_data));
    }
    client->addr = addr_data[5];

    pr_info("Change client device changed to (0x%04X) flag = %04x, ret = %d\n",
         client->addr, client->flags, ret);

    return (ret == sizeof(addr_data) ? 0 : -EIO);
}

#ifdef CONFIG_BCM2079X_MTK_PLATFORM
static void bcm2079x_dev_irq_handler(void) {
    struct bcm2079x_dev *bcm2079x_dev = g_bcm2079x_dev;
#else
static irqreturn_t bcm2079x_dev_irq_handler(int irq, void *dev_id) {
    struct bcm2079x_dev *bcm2079x_dev = dev_id;
#endif /*End of CONFIG_BCM2079X_MTK_PLATFORM*/

    //pr_info("%s irq_gpio %d\n", __func__, GET_GPIO(platform_data.irq_gpio));

    wake_up(&bcm2079x_dev->read_wq);
    wake_lock(&nfc_wake_lock);

#ifdef CONFIG_BCM2079X_MTK_PLATFORM
    return;
#else
    return IRQ_HANDLED;
#endif /*End of CONFIG_BCM2079X_MTK_PLATFORM*/
}

static unsigned int bcm2079x_dev_poll(struct file *filp, poll_table *wait)
{
    struct bcm2079x_dev *bcm2079x_dev = filp->private_data;
    unsigned int mask = 0;
    //int ret = 0;
    unsigned long flags;

    //[[ patch for GKI exception from Broadcom CSP 796884
    //pr_info("%s poll_wait before %d\n", __func__, GET_GPIO(platform_data.irq_gpio));
    //ret = wait_event_interruptible(bcm2079x_dev->read_wq, GET_GPIO(platform_data.irq_gpio));
    poll_wait(filp, &bcm2079x_dev->read_wq, wait);
    //pr_info("%s poll_wait after %d\n", __func__, GET_GPIO(platform_data.irq_gpio));

    spin_lock_irqsave(&bcm2079x_dev->irq_enabled_lock, flags);

    if(GET_GPIO(platform_data.irq_gpio) == 1)  //irq gpio get
    {
        mask |= POLLIN | POLLRDNORM;
    }

    spin_unlock_irqrestore(&bcm2079x_dev->irq_enabled_lock, flags);

    return mask;
}

static ssize_t bcm2079x_dev_read(struct file *filp, char __user *buf,
                  size_t count, loff_t *offset)
{
    struct bcm2079x_dev *bcm2079x_dev = filp->private_data;
    unsigned char tmp[MAX_BUFFER_SIZE];

    int total;
#if defined (BCM2079X_M_PLATFORM)
#else
    int len, ret;
    len = 0;
#endif //BCM2079X_M_PLATFORM
    total = 0;

    if (count > MAX_BUFFER_SIZE)
        count = MAX_BUFFER_SIZE;

    mutex_lock(&bcm2079x_dev->rw_mutex);

    /** Default: Read the first 4 bytes to include the length of the NCI or HCI packet.
     ** BCM2079X_M_PLATFORM: Raw mode Read the full length and return.
    **/
#if defined (BCM2079X_M_PLATFORM)
    total = nfc_i2c_dma_read(bcm2079x_dev->client, tmp, count);
#else
    ret = nfc_i2c_dma_read(bcm2079x_dev->client, tmp, 4);

    if (ret == 4) {
        total = ret;
        /** First byte is the packet type
        **/
        switch(tmp[0]) {
            case PACKET_TYPE_NCI:
                len = tmp[PACKET_HEADER_SIZE_NCI-1];
                break;

            case PACKET_TYPE_HCIEV:
                len = tmp[PACKET_HEADER_SIZE_HCI-1];
                if (len == 0)
                    total--;/*Since payload is 0, decrement total size (from 4 to 3) */
                else
                    len--;/*First byte of payload is in tmp[3] already */
                break;

            default:
                len = 0;/*Unknown packet byte */
                break;
        } /* switch*/

        /** make sure full packet fits in the buffer
        **/
        if (len > 0 && (len + total) <= count) {
            /** read the remainder of the packet.
            **/
            ret = nfc_i2c_dma_read(bcm2079x_dev->client, tmp+total, len);

            if (ret == len)
                total += len;
        } /* if */
    } /* if */
#endif /*BCM2079X_M_PLATFORM*/
    //pr_info("I2C read count: %d \n", total);

    mutex_unlock(&bcm2079x_dev->rw_mutex);

#if defined (BCM2079X_M_PLATFORM)
    if (total != count) {
        dev_err(&bcm2079x_dev->client->dev,
            "failed to read data from i2c bus driver, total = 0x%x count = %d\n",
            total, count);
        total = -EIO;
    } else if (copy_to_user(buf, tmp, total)) {
        dev_err(&bcm2079x_dev->client->dev,
            "failed to copy to user space, total = %d\n", total);
        total = -EFAULT;
    }
#else
    if (total > count || copy_to_user(buf, tmp, total)) {
        pr_err("failed to copy to user space, total = %d\n", total);
        total = -EFAULT;
    }
#endif /*BCM2079X_M_PLATFORM*/

    wake_unlock(&nfc_wake_lock);
    return total;
}

static ssize_t bcm2079x_dev_write(struct file *filp, const char __user *buf,
                   size_t count, loff_t *offset)
{
    struct bcm2079x_dev *bcm2079x_dev = filp->private_data;
    char tmp[MAX_BUFFER_SIZE];
    int ret;


    if (count > MAX_BUFFER_SIZE) {
        pr_err("out of memory\n");
        return -ENOMEM;
    }

    if (copy_from_user(tmp, buf, count)) {
        pr_err("failed to copy from user space\n");
        return -EFAULT;
    }

    mutex_lock(&bcm2079x_dev->rw_mutex);
    /* Write data */
    ret = nfc_i2c_dma_write(bcm2079x_dev->client, tmp, count);
    //pr_info("I2C write count: %d \n", count);
    if (ret != count) {
        pr_err("failed to write %d\n", ret);
        ret = -EIO;
    }
    mutex_unlock(&bcm2079x_dev->rw_mutex);

    return ret;
}

static int bcm2079x_dev_open(struct inode *inode, struct file *filp)
{
    int ret = 0;

    struct bcm2079x_dev *bcm2079x_dev = container_of(filp->private_data,
                               struct bcm2079x_dev,
                               bcm2079x_device);
    filp->private_data = bcm2079x_dev;
    bcm2079x_init_stat(bcm2079x_dev);
    bcm2079x_enable_irq(bcm2079x_dev);
    pr_info("%d,%d\n", imajor(inode), iminor(inode));

    return ret;
}

#ifdef CONFIG_LGE_NFC_USE_PMIC
void bcm2079x_get_clk_source(struct i2c_client *bcm2079x_client, struct bcm2079x_dev *bcm2079x_dev)
{
    bcm2079x_dev->clk_cont = clk_get(&bcm2079x_client->dev, "cont_clk");
    if (bcm2079x_dev->clk_cont == NULL) {
        pr_err("%s: BCM2079x could not get cont. clock!\n", __func__);
    }

    bcm2079x_dev->clk_pin = clk_get(&bcm2079x_client->dev, "pin_clk");
    if (bcm2079x_dev->clk_pin == NULL) {
        pr_err("%s: BCM2079x could not get pin clock!\n", __func__);
    }
    pr_err("%s: xo_cont = %p, xo_pin = %p\n", __func__, bcm2079x_dev->clk_cont, bcm2079x_dev->clk_pin); // for debug
}

static void bcm2079x_change_clk(struct bcm2079x_dev *bcm2079x_dev, unsigned int clk_state)
{
    static unsigned int nOldClkState = CLK_DISABLE;
    int ret = 0;

    if (nOldClkState == clk_state) {
        pr_err("%s: Desired clock state(%d) is same as previous state(%d)! Skip!\n", __func__, clk_state, nOldClkState);
    }
    else {
        switch (clk_state) {
            case CLK_DISABLE:
                if (nOldClkState == CLK_PIN) {
                    if (bcm2079x_dev->clk_pin != NULL) {
                        clk_disable_unprepare(bcm2079x_dev->clk_pin);
                        nOldClkState = CLK_DISABLE;
                        pr_err("%s: PMIC Clock is Disabled\n", __func__); // for debug
                    }
                    else {
                        pr_err("%s: BCM2079x could not get clock!\n", __func__);

                    }
                }
                else if (nOldClkState == CLK_CONT) {
                    if (bcm2079x_dev->clk_cont != NULL) {
                        clk_disable_unprepare(bcm2079x_dev->clk_cont);
                        nOldClkState = CLK_DISABLE;
                        pr_err("%s: PMIC Clock is Disabled\n", __func__); // for debug
                    }
                    else {
                        pr_err("%s: BCM2079x could not get clock!\n", __func__);
                    }
                }
                break;
            case CLK_PIN:
                if (bcm2079x_dev->clk_pin != NULL) {
                    ret = clk_prepare_enable(bcm2079x_dev->clk_pin);
                    if (ret) {
                        pr_err("%s: BCM2079x could not enable clock (%d)\n", __func__, ret);
                        clk_disable_unprepare(bcm2079x_dev->clk_pin);
                        nOldClkState = CLK_DISABLE;
                    }
                    nOldClkState = CLK_PIN;
                    pr_err("%s: PMIC Clock source is CXO_D1_PIN!\n", __func__); // for debug
                }
                else {
                    pr_err("%s: BCM2079x could not get pin clock!\n", __func__);
                }
                break;
            case CLK_CONT:
                if (bcm2079x_dev->clk_cont != NULL) {
                    ret = clk_prepare_enable(bcm2079x_dev->clk_cont);
                    if (ret) {
                        pr_err("%s: BCM2079x could not enable clock (%d)\n", __func__, ret);
                        clk_disable_unprepare(bcm2079x_dev->clk_cont);
                        nOldClkState = CLK_DISABLE;
                    }
                    nOldClkState = CLK_CONT;
                    pr_err("%s: PMIC Clock source is CXO_D1!\n", __func__); // for debug
                }
                else {
                    pr_err("%s: BCM2079x could not get cont. clock!\n", __func__);
                }
                break;
            default:
                pr_err("%s: Undefined Clock Setting!\n", __func__);
                break;
        }
    }
}
#endif /* CONFIG_LGE_NFC_USE_PMIC */

static long bcm2079x_dev_unlocked_ioctl(struct file *filp,
                     unsigned int cmd, unsigned long arg)
{
    struct bcm2079x_dev *bcm2079x_dev = filp->private_data;

    pr_info("%s,  cmd (%x) arg %lx \n", __func__, cmd, arg);

    switch (cmd) {
    case BCMNFC_READ_FULL_PACKET:
        break;
    case BCMNFC_READ_MULTI_PACKETS:
        break;
    case BCMNFC_CHANGE_ADDR:
        return change_client_addr(bcm2079x_dev, arg);
    case BCMNFC_POWER_CTL:
        if(arg == 0)
        {
            wake_unlock(&nfc_wake_lock);
        }
#ifdef CONFIG_LGE_NFC_USE_PMIC
        if(arg == 0)
        {
            pr_err("BCM2079x - CLK_DISABLE\n");
            bcm2079x_change_clk(bcm2079x_dev, CLK_DISABLE);
        } else {
            pr_err("BCM2079x - CLK_PIN\n");
            bcm2079x_change_clk(bcm2079x_dev, CLK_PIN);
        }
#endif /*CONFIG_LGE_NFC_USE_PMIC*/
        SET_GPIO(bcm2079x_dev->en_gpio, (int)arg);
        break;
    case BCMNFC_WAKE_CTL:
        SET_GPIO(bcm2079x_dev->wake_gpio, (int)arg);
        break;
#if defined (BCM2079X_M_PLATFORM)
    case BCMNFC_READ_MODE:
        return READ_RAW_PACKET;
#endif /*BCM2079X_M_PLATFORM*/
    default:
        pr_err("%s, unknown cmd (%x, %lx)\n", __func__, cmd, arg);
        return 0;
    }

    return 0;
}

static const struct file_operations bcm2079x_dev_fops = {
    .owner = THIS_MODULE,
    .llseek = no_llseek,
    .poll = bcm2079x_dev_poll,
    .read = bcm2079x_dev_read,
    .write = bcm2079x_dev_write,
    .open = bcm2079x_dev_open,
    .unlocked_ioctl = bcm2079x_dev_unlocked_ioctl,
    .compat_ioctl = bcm2079x_dev_unlocked_ioctl,
};


static int bcm2079x_probe(struct i2c_client *client,
               const struct i2c_device_id *id)
{
    int ret;
    struct bcm2079x_dev *bcm2079x_dev = NULL;
#ifdef CONFIG_LGE_NFC_USE_PMIC
    bcm2079x_client = client;
#endif
    bcm2079x_dev = kzalloc(sizeof(*bcm2079x_dev), GFP_KERNEL);
    if (bcm2079x_dev == NULL) {
        pr_err("failed to allocate memory for module data\n");
        ret = -ENOMEM;
        goto err_exit;
    }

#ifdef CONFIG_BCM2079X_MTK_PLATFORM
    g_bcm2079x_dev = bcm2079x_dev;

    platform_data.irq_gpio = GPIO_NFC_IRQ ;
    platform_data.en_gpio = GPIO_NFC_VEN;
    platform_data.wake_gpio = GPIO_NFC_WAKE;

    /* irq_gpio setup */
    pr_info("%s irq_gpio setup\n", __func__);
    mt_set_gpio_mode(platform_data.irq_gpio, GPIO_IRQ_NFC_PIN_M_EINT);
    mt_set_gpio_dir(platform_data.irq_gpio, GPIO_DIR_IN);
    mt_set_gpio_pull_enable(platform_data.irq_gpio, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(platform_data.irq_gpio, GPIO_PULL_DOWN);

    /* ven_gpio setup */
    pr_info("%s ven_gpio setup\n", __func__);
    mt_set_gpio_mode(platform_data.en_gpio, GPIO_NFC_VENB_PIN_M_GPIO);
    mt_set_gpio_dir(platform_data.en_gpio, GPIO_DIR_OUT);

    /* wake_gpio setup */
    pr_info("%s mode_gpio setup\n", __func__);
    mt_set_gpio_mode(platform_data.wake_gpio, GPIO_NFC_MODE_M_GPIO);
    mt_set_gpio_dir(platform_data.wake_gpio, GPIO_DIR_OUT);

    SET_GPIO(platform_data.en_gpio, 1);
    msleep(50);
    SET_GPIO(platform_data.en_gpio, 0);
    SET_GPIO(platform_data.wake_gpio, 0);
#else
    if(client->dev.of_node){
        platform_data.irq_gpio = of_get_named_gpio_flags(client->dev.of_node, "bcm,gpio_irq", 0, NULL);
        platform_data.en_gpio = of_get_named_gpio_flags(client->dev.of_node, "bcm,gpio_ven", 0, NULL);
        platform_data.wake_gpio = of_get_named_gpio_flags(client->dev.of_node, "bcm,gpio_mode", 0, NULL);

        pr_info("%d, platform_data.irq_gpio\n", platform_data.irq_gpio);
        pr_info("%d, platform_data.en_gpio\n", platform_data.en_gpio);
        pr_info("%d, platform_data.wake_gpio\n", platform_data.wake_gpio);

    }
    else{
        pr_err("nfc probe of_node fail\n");
        ret = -ENODEV;
        goto err_node;
    }
#ifdef CONFIG_LGE_NFC_USE_PMIC
    bcm2079x_get_clk_source(bcm2079x_client, bcm2079x_dev);
#else  // Temp remaining for not applied CONFIG_LGE_NFC_USE_PMIC
    s_clk = clk_get(&client->dev, "ref_clk");
    clk_prepare_enable(s_clk);
#endif

    ret = gpio_request_one(platform_data.irq_gpio, GPIOF_IN, "nfc_int");
    if (ret)
        goto err_en;
    ret = gpio_request_one(platform_data.en_gpio, GPIOF_OUT_INIT_LOW, "nfc_ven");
    if (ret)
        goto err_wake;
    ret = gpio_request_one(platform_data.wake_gpio, GPIOF_OUT_INIT_LOW,"nfc_firm");
    if (ret)
        goto err_exit;

    SET_GPIO(platform_data.en_gpio, 0);
    msleep(50);
    if (lge_get_boot_mode() == LGE_BOOT_MODE_NORMAL){
        SET_GPIO(platform_data.en_gpio, 1);
    }else{
        SET_GPIO(platform_data.en_gpio, 0);
    }
    SET_GPIO(platform_data.wake_gpio, 0);
#endif /*End of CONFIG_BCM2079X_MTK_PLATFORM*/

    bcm2079x_dev->wake_gpio = platform_data.wake_gpio;
    bcm2079x_dev->irq_gpio = platform_data.irq_gpio;
    bcm2079x_dev->en_gpio = platform_data.en_gpio;
    bcm2079x_dev->client = client;

    /* init mutex and queues */
    init_waitqueue_head(&bcm2079x_dev->read_wq);
    mutex_init(&bcm2079x_dev->rw_mutex);
    spin_lock_init(&bcm2079x_dev->irq_enabled_lock);

    bcm2079x_dev->bcm2079x_device.minor = MISC_DYNAMIC_MINOR;
    bcm2079x_dev->bcm2079x_device.name = I2C_ID_NAME;
    bcm2079x_dev->bcm2079x_device.fops = &bcm2079x_dev_fops;

    ret = misc_register(&bcm2079x_dev->bcm2079x_device);
    if (ret) {
        pr_err("misc_register failed\n");
        goto err_misc_register;
    }

    wake_lock_init(&nfc_wake_lock, WAKE_LOCK_SUSPEND, "NFCWAKE");

    /* request irq.  the irq is set whenever the chip has data available
     * for reading.  it is cleared when all data has been read.
     */
    pr_info("requesting IRQ %d\n", client->irq);
    bcm2079x_dev->irq_enabled = true;

#ifdef CONFIG_BCM2079X_MTK_PLATFORM
    client->irq = CUST_EINT_IRQ_NFC_NUM;
    mt_eint_set_sens(CUST_EINT_IRQ_NFC_NUM, CUST_EINT_IRQ_NFC_TYPE);
    mt_eint_set_hw_debounce(CUST_EINT_IRQ_NFC_NUM, CUST_EINT_IRQ_NFC_DEBOUNCE_CN);
    mt_eint_registration(CUST_EINT_IRQ_NFC_NUM, CUST_EINT_IRQ_NFC_TYPE, bcm2079x_dev_irq_handler, 1);
#else
    ret = request_irq(client->irq, bcm2079x_dev_irq_handler,
              IRQF_TRIGGER_RISING, client->name, bcm2079x_dev);
    if (ret) {
        pr_err("request_irq failed\n");
        goto err_request_irq_failed;
    }
#endif /*End of CONFIG_BCM2079X_MTK_PLATFORM*/
    bcm2079x_dev->irq_wake_enabled = false;
    bcm2079x_disable_irq(bcm2079x_dev);
    i2c_set_clientdata(client, bcm2079x_dev);
    pr_info("%s, probing bcm2079x driver exited successfully\n", __func__);
    return 0;

err_request_irq_failed:
    misc_deregister(&bcm2079x_dev->bcm2079x_device);
err_misc_register:
    mutex_destroy(&bcm2079x_dev->rw_mutex);
err_exit:
    gpio_free(platform_data.wake_gpio);
err_wake:
    gpio_free(platform_data.en_gpio);
err_en:
    gpio_free(platform_data.irq_gpio);
err_node:
    kfree(bcm2079x_dev);

    return ret;
}

static int bcm2079x_remove(struct i2c_client *client)
{
    struct bcm2079x_dev *bcm2079x_dev;

    bcm2079x_dev = i2c_get_clientdata(client);
    free_irq(client->irq, bcm2079x_dev);
    misc_deregister(&bcm2079x_dev->bcm2079x_device);
    mutex_destroy(&bcm2079x_dev->rw_mutex);
    gpio_free(bcm2079x_dev->irq_gpio);
    gpio_free(bcm2079x_dev->en_gpio);
    gpio_free(bcm2079x_dev->wake_gpio);
    kfree(bcm2079x_dev);

    return 0;
}

static const struct i2c_device_id bcm2079x_id[] = {
    {I2C_ID_NAME, I2C_DEVICE_ADDR},
    {}
};

static struct i2c_driver bcm2079x_driver = {
    .id_table = bcm2079x_id,
    .probe = bcm2079x_probe,
    .remove = bcm2079x_remove,
    .driver = {
        .owner = THIS_MODULE,
        .name = I2C_ID_NAME,
    },
};

/*
 * module load/unload record keeping
 */

static int __init bcm2079x_dev_init(void)
{
#ifdef CONFIG_BCM2079X_MTK_PLATFORM
    I2CDMABuf_va = (u8 *)dma_alloc_coherent(NULL, 4096, &I2CDMABuf_pa, GFP_KERNEL);
    if(!I2CDMABuf_va) {
        pr_err("%s: Allocate NFC DMA I2C Buffer failed!\n",__func__);
        return ENOMEM;
    }
    i2c_register_board_info(2, &nfc_board_info, 1);
#endif /*End of CONFIG_BCM2079X_MTK_PLATFORM*/
    return i2c_add_driver(&bcm2079x_driver);
}
module_init(bcm2079x_dev_init);

static void __exit bcm2079x_dev_exit(void)
{
#ifdef CONFIG_BCM2079X_MTK_PLATFORM
    if(I2CDMABuf_va) {
        dma_free_coherent(NULL, 4096, I2CDMABuf_va, I2CDMABuf_pa);
        I2CDMABuf_va = NULL;
        I2CDMABuf_pa = 0;
    }
#endif /*End of CONFIG_BCM2079X_MTK_PLATFORM*/
    i2c_del_driver(&bcm2079x_driver);
}
module_exit(bcm2079x_dev_exit);

MODULE_DEVICE_TABLE(i2c, bcm2079x_id);
MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("NFC bcm2079x driver");
MODULE_LICENSE("GPL");
