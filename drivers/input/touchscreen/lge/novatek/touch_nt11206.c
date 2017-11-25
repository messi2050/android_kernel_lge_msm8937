
/***************************************************************************
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *    File  : lgtp_device_nt11206.c
 *
 ***************************************************************************/

/****************************************************************************
* Include Files
****************************************************************************/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/firmware.h>
#include <linux/jiffies.h>

#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <asm/uaccess.h>
#include <linux/slab.h>
#include <linux/proc_fs.h>

/*
 *  Include to touch core Header File
 */
#include <touch_core.h>
#include <touch_hwif.h>

/*
 *  Include to Local Header File
 */
#include "touch_nt11206.h"

/****************************************************************************
* Type Definitions
****************************************************************************/

/****************************************************************************
* Variables
****************************************************************************/

#define TCI_FAIL_NUM 		7
static const char const *tci_debug_str[TCI_FAIL_NUM] = {
	"NONE",
	"DISTANCE_TOUCHSLOP",
	"TIMEOUT_CONTINUOUS_TAP",
	"DISTANCE_GAP_TAP",
	"TIMEOUT_GAP_TAP",
	"TOTAL_TAP_NUM",
	"MULTI_FINGER",
};

struct novatek_ts_data* ts = NULL;
unsigned char *my_image_bin;
unsigned long my_image_size;
static bool probe_complete = false;
static bool init_pass = false;
#if NVT_TOUCH_CTRL_DRIVER
struct touch_core_data *gts;
#endif
static int power_state = -1;
static u8 xdata_tmp[2048] = {0, };
static __s32 xdata[2048] = {0, };
static u8 fw_ver=0;
static u8 x_num=0;
static u8 y_num=0;

#if NVT_TOUCH_EXT_PROC
static struct proc_dir_entry *NVT_proc_fw_version_entry;
static struct proc_dir_entry *NVT_proc_baseline_entry;
static struct proc_dir_entry *NVT_proc_raw_entry;
static struct proc_dir_entry *NVT_proc_diff_entry;
static struct device *g_dev = NULL;
#endif
int nt11206_reg_write(struct device *dev, u16 addr, void *data, int size)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct touch_bus_msg msg;
	int ret = 0;
	int retry_cnt = 0;

	ts->tx_buf[0] = addr;
	memcpy(&ts->tx_buf[1], data, size);

	msg.tx_buf = ts->tx_buf;
	msg.tx_size = size + 1;
	msg.rx_buf = NULL;
	msg.rx_size = 0;

	while(retry_cnt < 5) {
		ret = touch_bus_write(dev, &msg);
		if (ret < 0) {
			TOUCH_E("touch bus write error:%d retry_cnt:%d\n", ret, retry_cnt);
			if(power_state == POWER_OFF) {
				TOUCH_E("TOUCH POWER DOWN\n");
				return -1;
			}
			retry_cnt++;
			msleep(10);
		}
		else {
			return ret;
		}
	}
	return ret;
}

int nt11206_reg_read(struct device *dev, u16 addr, void *data, int size)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct touch_bus_msg msg;
	int ret = 0;
	int retry_cnt = 0;

	ts->tx_buf[0] = addr;

	msg.tx_buf = ts->tx_buf;
	msg.tx_size = 1;

	msg.rx_buf = ts->rx_buf;
	msg.rx_size = size;

	while(retry_cnt < 5) {
		ret = touch_bus_read(dev, &msg);
		if (ret < 0) {
			TOUCH_E("touch bus read error:%d retry_cnt:%d\n", ret, retry_cnt);
			if(power_state == POWER_OFF) {
				TOUCH_E("TOUCH POWER DOWN\n");
				return -1;
			}
			retry_cnt++;
			msleep(10);
		}
		else {
			memcpy(data, &ts->rx_buf[0], size);
			return ret;
		}
	}
	return ret;
}

int nt11206_reg_read_dummy(struct device *dev, u16 address)
{
	uint8_t buf[8]={0};
	struct i2c_client *client = NULL;
	struct i2c_msg msg;
	int ret = -1;
	int retries = 0;

	client = to_i2c_client(dev);
	msg.flags = I2C_M_RD;
	msg.addr  = address;
	msg.len   = 1;
	msg.buf   = buf;

	while(retries < 5)
	{
		ret = i2c_transfer(client->adapter, &msg, 1);
		if(ret == 1)
			break;
		retries++;
	}
	return ret;
}

void large_mdelay(unsigned int msec)
{
	unsigned long endtime = 0;
	unsigned long msec_to_jiffies = 0;

	msec_to_jiffies = msecs_to_jiffies(msec);
	endtime = jiffies + msec_to_jiffies + 1;

	while(jiffies<endtime) ;

	return ;
}
static void nvt_init_locks(struct device *dev)
{
	struct nt11206_data *d = to_nt11206_data(dev);
	wake_lock_init(&d->lpwg_wake_lock, WAKE_LOCK_SUSPEND, "nvt_lpwg");
}
void nvt_set_i2c_debounce(struct device *dev)
{
	int ret = 0;
	u8 buf[8] = {0, };
	u8 reg1_val = 0;
	u8 reg2_val = 0;
	int retry = 0;

	do {
		large_mdelay(20);

		// set xdata index to 0x1F000
		buf[0] = 0x01;
		buf[1] = 0xF0;
		ret = nt11206_reg_write(dev, ADDR_CMD, buf, 2);

		// set I2C debounce
		buf[0] = 0x15;
		buf[1] = 0x17;
		ret = nt11206_reg_write(dev, buf[0], &buf[1], 1);
		large_mdelay(20);

		buf[0] = 0x15;
		buf[1] = 0x00;
		ret = nt11206_reg_read(dev, buf[0], &buf[1], 1);
		reg1_val = buf[1];
		TOUCH_I("(retry cnt:%d) 0x01F015 val=0x%02X\n", retry, reg1_val);
		// set schmitt trigger enable
		buf[0] = 0x3E;
		buf[1] = 0x07;
		ret = nt11206_reg_write(dev, buf[0], &buf[1], 1);
		large_mdelay(20);

		buf[0] = 0x3E;
		buf[1] = 0x00;
		ret = nt11206_reg_read(dev, buf[0], &buf[1], 1);
		reg2_val = buf[1];
		TOUCH_I("(retry cnt:%d) 0x01F03E val=0x%02X\n", retry, reg2_val);
	} while (((reg1_val != 0x17) || (reg2_val != 0x07)) && (retry++ < 20));

	if (retry == 20) {
		TOUCH_E("%s: set i2c debounce failed, reg1_val=0x%02X, reg2_val=0x%02X\n", __func__, reg1_val, reg2_val);
	}
	else {
		TOUCH_I("retry cnt:%d\n", retry);
	}

}
int nt11206_bootloader_read(struct device *dev, u8 *buf, u8 len)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct i2c_msg msgs[2];
	int ret = -1;
	int retries = 0;

	msgs[0].flags = !I2C_M_RD;
	msgs[0].addr  = BOOTLOADER_SLAVE_ADDR;
	msgs[0].len   = 1;
	msgs[0].buf   = &buf[0];

	msgs[1].flags = I2C_M_RD;
	msgs[1].addr  = BOOTLOADER_SLAVE_ADDR;
	msgs[1].len   = len-1;
	msgs[1].buf   = &buf[1];

	while(retries < 5)
	{
		ret = i2c_transfer(client->adapter, msgs, 2);
		if(ret < 0)
			retries++;
		else
			break;
	}
	return ret;
}

int nt11206_bootloader_write(struct device *dev, u8 *data, u8 len)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct i2c_msg msg;
	int ret = -1;
	int retries = 0;

	msg.flags = !I2C_M_RD;
	msg.addr  = BOOTLOADER_SLAVE_ADDR;
	msg.len   = len;
	msg.buf   = data;

	while(retries < 5)
	{
		ret = i2c_transfer(client->adapter, &msg, 1);
		if(ret < 0)
			retries++;
		else
			break;
	}
	return ret;
}
int nt11206_rebase(struct device *dev)
{
	int ret = 0;
	u8 serial_data[2] = {0x01, 0x47};
	u8 rebase_addr = 0x50;
	u8 rebase_value = 0x23;

	ret = nt11206_reg_write(dev, ADDR_CMD, serial_data, 2);
	if (ret < 0) {
		TOUCH_E("%s() serial_data I2C write Fail\n", __func__);
		return ret;
	}
	ret = nt11206_reg_write(dev, rebase_addr, &rebase_value, 1);
	if (ret < 0) {
		TOUCH_E("%s() rebase CMD write Fail\n", __func__);
		return ret;
	}

	TOUCH_I("Rebase Complete\n");
	return ret;
}
void nvt_get_mdata(__s32 *buf, u8 *m_x_num, u8 *m_y_num)
{
	*m_x_num = x_num;
	*m_y_num = y_num;
	memcpy(buf, xdata, 20 * 30 * sizeof(__s32));
}
u8 nvt_clear_fw_status(struct device *dev)
{
	u8 buf[8] = {0, };
	u8 addr = 0;
	int i = 0;
	const int retry = 10;

	//---dummy read to resume TP before writing command---
	nt11206_reg_read_dummy(dev, SLAVE_ADDR);

	for(i=0; i<retry; i++)
	{
		//---set xdata index to 0x14700---
		buf[0]=0x01;
		buf[1]=0x47;
		nt11206_reg_write(dev, ADDR_CMD, buf, 2);

		//---clear fw status---
		addr = 0x51;
		buf[0]=0x00;
		nt11206_reg_write(dev, addr, buf, 1);

		//---read fw status---
		addr = 0x51;
		buf[0]=0xFF;
		nt11206_reg_read(dev, addr, buf, 1);
		if(buf[0] == 0x00)
			break;

		mdelay(10);
	}

	if(i > retry)
		return -1;
	else
		return 0;
}
u8 nvt_check_fw_status(struct device *dev)
{
	u8 buf[8] = {0, };
	u8 addr = 0;
	int i = 0;
	const int retry = 10;

	//---dummy read to resume TP before writing command---
	nt11206_reg_read_dummy(dev, SLAVE_ADDR);

	for(i=0; i<retry; i++)
	{
		//---set xdata index to 0x14700---
		buf[0]=0x01;
		buf[1]=0x47;
		nt11206_reg_write(dev, ADDR_CMD, buf, 2);

		//---read fw status---
		addr = 0x51;
		buf[0]=0x00;
		nt11206_reg_read(dev, addr, buf, 1);

		if((buf[0]&0xF0) == 0xA0)
			break;

		mdelay(10);
	}

	if(i > retry)
		return -1;
	else
		return 0;
}
void nvt_change_mode(struct device *dev, u8 mode)
{
	char buf[8];
	int ret = 0;

	//---dummy read to resume TP before writing command---
	nt11206_reg_read_dummy(dev, SLAVE_ADDR);

	//---set xdata index to 0x14700---
	buf[0] = 0x01;
	buf[1] = 0x47;
	ret = nt11206_reg_write(dev, ADDR_CMD, buf, 2);
	if(ret < 0) {
		TOUCH_E("Serical Data Write Failed\n");
	}

	//---set mode---
	buf[0] = mode;
	buf[1] = 0x00;
	ret = nt11206_reg_write(dev, MODE_CHANGE_OFFSET, buf, 2);
	if(ret < 0) {
		TOUCH_E("MODE CHANGE DATA Write Failed\n");
	}

	if(mode == MODE_CHANGE_NORMAL_MODE)
	{
		buf[0]=0xBB;
		ret = nt11206_reg_write(dev, MODE_CHANGE_NORMAL_OFFSET, &buf[0], 1);
		if(ret < 0) {
			TOUCH_E("MODE CHANGE DATA Write Failed\n");
		}
	}

	if(mode == AUTORC_OFF) {
		msleep(100);
	}
}
void nvt_get_fw_info(struct device *dev)
{
	char buf[64];
	u16 addr = 0;
	//---dummy read to resume TP before writing command---
	nt11206_reg_read_dummy(dev, SLAVE_ADDR);

	//---set xdata index to 0x14700---
	buf[0]=0x01;
	buf[1]=0x47;
	nt11206_reg_write(dev, ADDR_CMD, buf, 2);

	//---read fw info---
	addr = 0x78;
	nt11206_reg_read(dev, addr, buf, 16);
	fw_ver = buf[0];
	x_num = buf[2];
	y_num = buf[3];

	//---clear x_num, y_num if fw info is broken---
	if((buf[0]+buf[1]) != 0xFF)
	{
		TOUCH_E("FW info is broken! fw_ver=0x%02X, ~fw_ver=0x%02X\n",  buf[0], buf[1]);
		x_num = 20;
		y_num = 30;
	}
	return;
}
u8 nvt_get_fw_pipe(struct device *dev)
{
	char buf[8];

	//---dummy read to resume TP before writing command---
	nt11206_reg_read_dummy(dev, SLAVE_ADDR);

	//---set xdata index to 0x14700---
	buf[0]=0x01;
	buf[1]=0x47;
	nt11206_reg_write(dev, ADDR_CMD, buf, 2);

	//---read fw status---
	buf[0]=0x00;
	nt11206_reg_read(dev, MODE_CHANGE_NORMAL_OFFSET, buf, 1);

	return (buf[0] & 0x01);
}

void nvt_read_mdata(struct device *dev, u32 xdata_addr)
{
	int i = 0;
	int j = 0;
	int k = 0;
	char buf[I2C_TANSFER_LENGTH+1] = {0x00,};
	u32 head_addr = 0;
	u16 addr = 0;
	int dummy_len = 0;
	int data_len = 0;
	int residual_len = 0;


	//---set xdata sector address & length---
	head_addr = xdata_addr - (xdata_addr%XDATA_SECTOR_SIZE);
	dummy_len = xdata_addr - head_addr;
	data_len = x_num*y_num*2;
	residual_len = (head_addr + dummy_len + data_len) % XDATA_SECTOR_SIZE;

	//read xdata : step 1
	for(i=0; i<((dummy_len+data_len)/XDATA_SECTOR_SIZE); i++)
	{
		//---change xdata index---
		buf[0]=((head_addr+XDATA_SECTOR_SIZE*i)>>16);
		buf[1]=((head_addr+XDATA_SECTOR_SIZE*i)>>8)&0xFF;
		nt11206_reg_write(dev, ADDR_CMD, buf, 2);

		//---read xdata by I2C_TANSFER_LENGTH
		for(j=0; j<(XDATA_SECTOR_SIZE/I2C_TANSFER_LENGTH); j++)
		{
			//---read data---
			addr = I2C_TANSFER_LENGTH * j;
			nt11206_reg_read(dev, addr, buf, I2C_TANSFER_LENGTH);
			//---copy buf to xdata_tmp---
			for(k=0; k<I2C_TANSFER_LENGTH; k++)
			{
				xdata_tmp[XDATA_SECTOR_SIZE*i + I2C_TANSFER_LENGTH*j + k] = buf[k];
			}
		}
	}

	//read xdata : step2
	if(residual_len != 0)
	{
		//---change xdata index---
		buf[0]=((xdata_addr + data_len - residual_len)>>16);
		buf[1]=((xdata_addr + data_len - residual_len)>>8)&0xFF;
		nt11206_reg_write(dev, ADDR_CMD, buf, 2);

		//---read xdata by I2C_TANSFER_LENGTH
		for(j=0; j<(residual_len/I2C_TANSFER_LENGTH + 1); j++)
		{
			//---read data---
			addr = I2C_TANSFER_LENGTH * j;
			nt11206_reg_read(dev, addr, buf, I2C_TANSFER_LENGTH);

			//---copy buf to xdata_tmp---
			for(k=0; k<I2C_TANSFER_LENGTH; k++)
			{
				xdata_tmp[(dummy_len+data_len-residual_len) + I2C_TANSFER_LENGTH*j + k] = buf[k];
			}
		}
	}

	//---remove dummy data and 2bytes-to-1data---
	for(i=0; i<(data_len/2); i++)
	{
		xdata[i] = (xdata_tmp[dummy_len + i*2] + 256*xdata_tmp[dummy_len + i*2 + 1]);
	}

	//---set xdata index to 0x14700---
	buf[0]=0x01;
	buf[1]=0x47;
	nt11206_reg_write(dev, ADDR_CMD, buf, 2);
}

#if NVT_TOUCH_EXT_PROC
static int c_fw_version_show(struct seq_file *m, void *v)
{
	seq_printf(m, "fw_ver=%d, x_num=%d, y_num=%d\n", fw_ver, x_num, y_num);
	return 0;
}
static int c_show(struct seq_file *m, void *v)
{
	int i, j;

	for(i=0; i<y_num; i++)
	{
		for(j=0; j<x_num; j++)
		{
			seq_printf(m, "%05d, ", (short)xdata[i*x_num+j]);
		}
		seq_puts(m, "\n");
	}
	seq_printf(m, "\n\n");
	return 0;
}
static void *c_start(struct seq_file *m, loff_t *pos)
{
	return *pos < 1 ? (void *)1 : NULL;
}

static void *c_next(struct seq_file *m, void *v, loff_t *pos)
{
	++*pos;
	return NULL;
}

static void c_stop(struct seq_file *m, void *v)
{
	return;
}
const struct seq_operations nvt_fw_version_seq_ops = {
	.start  = c_start,
	.next   = c_next,
	.stop   = c_stop,
	.show   = c_fw_version_show
};

const struct seq_operations nvt_seq_ops = {
	.start  = c_start,
	.next   = c_next,
	.stop   = c_stop,
	.show   = c_show
};

static int nvt_fw_version_open(struct inode *inode, struct file *file)
{
	nvt_get_fw_info(g_dev);
	return seq_open(file, &nvt_fw_version_seq_ops);
}

static const struct file_operations nvt_fw_version_fops = {
	.owner = THIS_MODULE,
	.open = nvt_fw_version_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};

static int nvt_baseline_open(struct inode *inode, struct file *file)
{
	nvt_change_mode(g_dev, TEST_MODE_1);

	if(nvt_check_fw_status(g_dev) != 0) {
		TOUCH_E("FW_STATUS FAIL\n");
		return -EAGAIN;
	}

	nvt_get_fw_info(g_dev);

	nvt_read_mdata(g_dev, BASELINE_ADDR);

	nvt_change_mode(g_dev, MODE_CHANGE_NORMAL_MODE);

	return seq_open(file, &nvt_seq_ops);
}

static const struct file_operations nvt_baseline_fops = {
	.owner = THIS_MODULE,
	.open = nvt_baseline_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};

static int nvt_raw_open(struct inode *inode, struct file *file)
{
	nvt_change_mode(g_dev, TEST_MODE_1);

	if(nvt_check_fw_status(g_dev) != 0) {
		TOUCH_E("FW_STATUS FAIL\n");
		return -EAGAIN;
	}

	nvt_get_fw_info(g_dev);

	if(nvt_get_fw_pipe(g_dev) == 0)
		nvt_read_mdata(g_dev, RAW_PIPE0_ADDR);
	else
		nvt_read_mdata(g_dev, RAW_PIPE1_ADDR);

	nvt_change_mode(g_dev, MODE_CHANGE_NORMAL_MODE);

	return seq_open(file, &nvt_seq_ops);
}

static const struct file_operations nvt_raw_fops = {
	.owner = THIS_MODULE,
	.open = nvt_raw_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};

static int nvt_diff_open(struct inode *inode, struct file *file)
{
	nvt_change_mode(g_dev, TEST_MODE_2);

	if(nvt_check_fw_status(g_dev) != 0) {
		TOUCH_E("FW_STATUS FAIL\n");
		return -EAGAIN;
	}
	nvt_get_fw_info(g_dev);

	if(nvt_get_fw_pipe(g_dev) == 0)
		nvt_read_mdata(g_dev, DIFF_PIPE0_ADDR);
	else
		nvt_read_mdata(g_dev, DIFF_PIPE1_ADDR);

	nvt_change_mode(g_dev, MODE_CHANGE_NORMAL_MODE);

	return seq_open(file, &nvt_seq_ops);
}

static const struct file_operations nvt_diff_fops = {
	.owner = THIS_MODULE,
	.open = nvt_diff_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};
static int nvt_extra_proc_init(void)
{
	int ret = 0;
	NVT_proc_fw_version_entry = proc_create(NVT_FW_VERSION, 0444, NULL,&nvt_fw_version_fops);
	if(NVT_proc_fw_version_entry == NULL)
	{
		printk("%s: create proc/nvt_fw_version Failed!\n", __func__);
		return -1;
	}
	else
	{
		printk("%s: create proc/nvt_fw_version Succeeded!\n", __func__);
	}

	NVT_proc_baseline_entry = proc_create(NVT_BASELINE, 0444, NULL,&nvt_baseline_fops);
	if(NVT_proc_baseline_entry == NULL)
	{
		printk("%s: create proc/nvt_baseline Failed!\n", __func__);
		return -1;
	}
	else
	{
		printk("%s: create proc/nvt_baseline Succeeded!\n", __func__);
	}

	NVT_proc_raw_entry = proc_create(NVT_RAW, 0444, NULL,&nvt_raw_fops);
	if(NVT_proc_raw_entry == NULL)
	{
		printk("%s: create proc/nvt_raw Failed!\n", __func__);
		return -1;
	}
	else
	{
		printk("%s: create proc/nvt_raw Succeeded!\n", __func__);
	}

	NVT_proc_diff_entry = proc_create(NVT_DIFF, 0444, NULL,&nvt_diff_fops);
	if(NVT_proc_diff_entry == NULL)
	{
		printk("%s: create proc/nvt_diff Failed!\n", __func__);
		return -1;
	}
	else
	{
		printk("%s: create proc/nvt_diff Succeeded!\n", __func__);
	}
	return ret;
}
#endif

void nt11206_interrupt_control(struct device *dev, int on_off)
{
	struct touch_core_data *ts = to_touch_core(dev);
	if (on_off) {
		if (atomic_cmpxchg(&ts->state.irq_enable, 0, 1) == 0) {
			touch_enable_irq(ts->irq);

			if (ts->role.use_lpwg)
				touch_enable_irq_wake(ts->irq);
		}
	} else {
		if (atomic_cmpxchg(&ts->state.irq_enable, 1, 0) == 1) {
			if (ts->role.use_lpwg)
				touch_disable_irq_wake(ts->irq);

			disable_irq_nosync(ts->irq);
		}
	}
}

static void parser_finger_events(u8 *buf, struct tp_event *event)
{
	event->id = (buf[0] >> 3) - 1;
	event->status = buf[0] & FINGER_STATUS_MASK;
	event->x = (buf[1] << 4) | ((buf[3] & 0xF0) >> 4);
	event->y = (buf[2] << 4) | (buf[3] & 0x0F);
	event->area = buf[4];
	event->pressure = buf[5];

	if(event->area > 255)
		event->area = 255;
}

// Check to connect TOUCH IC
static int novatek_ts_chipid(struct device *dev)
{
	u8 serial_data[2] = {0x01, 0xF0};
	u8 id_data[2] = {0x00, };
	int ret;

	ret = nt11206_reg_read_dummy(dev, SLAVE_ADDR);

	ret = nt11206_reg_write(dev, ADDR_CMD, serial_data, 2);
	if (ret < 0) {
		TOUCH_E("%s() serial_data I2C write Fail\n", __func__);
		return ret;
	}

	ret = nt11206_reg_read(dev, CHIP_ID_OFFSET, id_data, 2);
	if (ret < 0) {
		TOUCH_E("%s() id_data I2C read Fail\n", __func__);
		return ret;
	}
	TOUCH_I("%s(), CHIP_ID : 0x%02x, CHIP_VERSION:0x%02x\n", __func__, id_data[0], id_data[1]);
	return (int)id_data[0];
}

int nt11206_sw_reset(struct device *dev, SW_RESET_MODE mode)
{
	int ret = 0;
	u8 buf[3] = {0, };

	if(mode == RESET_MCU_NO_BOOT) {
		buf[0] = 0x00;
		buf[1] = 0x5A;
		ret = nt11206_bootloader_write(dev, buf, 2);
		if (ret < 0) {
			TOUCH_E("SW RESET CMD Write Fail\n");
			return ret;
		}
	}
	else if(mode == RESET_MCU_TO_IDLE) {
		buf[0] = 0x00;
		buf[1] = 0xA5;
		ret = nt11206_bootloader_write(dev, buf, 2);
		if (ret < 0) {
			TOUCH_E("SW RESET CMD Write Fail\n");
			return ret;
		}
	}
	else if(mode == RESET_MCU_BOOT) {
		buf[0] = 0xFF;
		buf[1] = 0x00;
		buf[2] = 0x00;
		ret = nt11206_bootloader_write(dev, buf, 3);
		if (ret < 0) {
			TOUCH_E("SW RESET CMD Write Fail\n");
			return ret;
		}
		buf[0] = 0x00;
		buf[1] = 0x69;
		ret = nt11206_bootloader_write(dev, buf, 2);
		if (ret < 0) {
			TOUCH_E("SW RESET CMD Write Fail\n");
			return ret;
		}
	}
	else {
		TOUCH_E("Not defined mode(%d)\n", mode);
	}

	msleep(5);
	TOUCH_I("SW RESET COMPLETE mode:%d\n", mode);
	return ret;
}
static int nt11206_mode_change_check(struct device *dev, u8 *mode, int len)
{
	int i = 0;
	int ret = 0 ;
	u8 mode_check = 0;

	for(i = 0;i < 5; i++) {
		ret = nt11206_reg_write(dev, DEEP_SLEEP_OFFSET, mode, len);
		if (ret < 0) {
			TOUCH_E("%s() gesture_data I2C write Fail\n", __func__);
			return ret;
		}
		msleep(10);

		ret = nt11206_reg_read(dev, MODE_CHANGE_CEHCK, &mode_check, 1);
		if (ret < 0) {
			TOUCH_E("%s() gesture_data I2C read Fail\n", __func__);
			return ret;
		}
		if(mode_check) {
			TOUCH_I("Success to change to Mode(%d)\n", mode[0]);
			return 0;
		}
		msleep(10);
	}

	if(i == 5) {
		ret = nt11206_sw_reset(dev, RESET_MCU_BOOT);
		msleep(200);
		for(i = 0;i < 5; i++) {
			ret = nt11206_reg_write(dev, DEEP_SLEEP_OFFSET, mode, len);
			if (ret < 0) {
				TOUCH_E("%s() gesture_data I2C write Fail\n", __func__);
				return ret;
			}
			msleep(10);

			ret = nt11206_reg_read(dev, MODE_CHANGE_CEHCK, &mode_check, 1);
			if (ret < 0) {
				TOUCH_E("%s() gesture_data I2C read Fail\n", __func__);
				return ret;
			}
			if(mode_check) {
				TOUCH_I("Success to change to Mode(%d)\n", mode[0]);
				return 0;
			}
			msleep(10);
		}
	}

	TOUCH_E("Fail to change to mode(%d)\n", mode[0]);
	return -1;
}
void nt11206_hw_reset(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);

	TOUCH_I("START %s()\n",__func__);
	gpio_set_value(ts->reset_pin, 1);
	touch_msleep(85);
	gpio_set_value(ts->reset_pin, 0);
	mdelay(50);
	gpio_set_value(ts->reset_pin, 1);
	touch_msleep(100);
	TOUCH_I("END %s()\n",__func__);
}

int nt11206_power(struct device *dev, int ctrl)
{
	struct touch_core_data *ts = to_touch_core(dev);
	TOUCH_TRACE();

	switch (ctrl) {
	case POWER_OFF:
		if(power_state != POWER_OFF) {
			touch_report_all_event(ts);
			TOUCH_I("%s, off\n", __func__);
			touch_gpio_direction_output(ts->reset_pin, 0);
			touch_power_vio(dev, 0);
			touch_power_vdd(dev, 0);
			power_state = POWER_OFF;
		}
		break;

	case POWER_ON:
		if(power_state != POWER_ON) {
			TOUCH_I("%s, on\n", __func__);
			touch_gpio_direction_output(ts->reset_pin, 0);
			touch_power_vdd(dev, 1);
			mdelay(1);
			touch_power_vio(dev, 1);
			nt11206_hw_reset(dev);
			power_state = POWER_ON;
		}
		break;

	case POWER_SLEEP:
		TOUCH_I("%s, sleep\n", __func__);
		power_state = POWER_SLEEP;
		break;

	case POWER_WAKE:
		TOUCH_I("%s, wake\n", __func__);
		power_state = POWER_WAKE;
		break;
	}

	return 0;
}
static int novatek_ts_read_fw_info(struct device *dev)
{
	struct nt11206_data *d = to_nt11206_data(dev);
	u8 serial_data[2] = {0x01, 0x7C};
	u8 fw_info[11] = {0x00, };
	int ret;
	int i = 0;

	for(i = 0;i < 5;i++) {
		ret = nt11206_reg_write(dev, ADDR_CMD, serial_data, 2);
		if (ret < 0) {
			TOUCH_E("%s() serial_data I2C write Fail\n", __func__);
			return ret;
		}

		ret = nt11206_reg_read(dev, FW_INFO_OFFSET_1, fw_info, 11);
		if (ret < 0) {
			TOUCH_E("%s() id_data I2C read Fail\n", __func__);
			return ret;
		}

		d->fw.fw_ver = fw_info[0];
		d->fw.fw_ver_bar = fw_info[1];
		d->fw.x_axis_num = fw_info[2];
		d->fw.y_axis_num = fw_info[3];
		d->fw.resolution_x = (fw_info[5] << 8) | fw_info[4];
		d->fw.resolution_y = (fw_info[7] << 8) | fw_info[6];
		d->fw.common_fw_ver = fw_info[8];
		d->fw.max_touch_num = fw_info[9];
		d->fw.b_num = fw_info[10];

		if((d->fw.fw_ver + d->fw.fw_ver_bar) != 0xFF) {
			TOUCH_E("FW Info is broken\n");
			if(i == 3) {
				int ret = 0;
				ret = nt11206_sw_reset(dev, RESET_MCU_BOOT);
				if(ret < 0) {
					TOUCH_E("SW Reset Failed\n");
				}
				large_mdelay(20);
				nvt_set_i2c_debounce(dev);
			}
			large_mdelay(50);
			continue;
		}

		ret = nt11206_reg_write(dev, ADDR_CMD, serial_data, 2);
		if (ret < 0) {
			TOUCH_E("%s() serial_data I2C write Fail\n", __func__);
			return ret;
		}
		memset(fw_info, 0x00, sizeof(fw_info));
		ret = nt11206_reg_read(dev, FW_INFO_OFFSET_2, fw_info, 2);
		if (ret < 0) {
			TOUCH_E("%s() id_data I2C read Fail\n", __func__);
			return ret;
		}

		d->fw.custom_id = fw_info[0];
		d->fw.sensor_id = fw_info[1];
		TOUCH_I("fw_ver:0x%x, fw_ver_bar:0x%x, common_fw_ver:0x%x\n",
				d->fw.fw_ver, d->fw.fw_ver_bar, d->fw.common_fw_ver);
		TOUCH_I("custom_id:0x%x, sensor_id:0x%x\n",
				d->fw.custom_id, d->fw.sensor_id);
		TOUCH_I("x_axis_num:%d, y_axis_num:%d resolution_x:%d, resolution_y:%d\n",
				d->fw.x_axis_num, d->fw.y_axis_num, d->fw.resolution_x, d->fw.resolution_y);
		break;
	}
	return ret;
}

static int novatek_get_cmd_version(struct device *dev, char *buf)
{
	struct nt11206_data *d = to_nt11206_data(dev);
	u8 serial_data[2] = {0x01, 0x7C};
	u8 fw_info[11] = {0x00, };
	int ret = 0;
	int offset = 0;
	int i = 0;

	for(i = 0;i < 5;i++) {
		offset = 0;
		ret = nt11206_reg_write(dev, ADDR_CMD, serial_data, 2);
		if (ret < 0) {
			TOUCH_E("%s() serial_data I2C write Fail\n", __func__);
			offset += snprintf(buf + offset, PAGE_SIZE, "-1\n");
			offset += snprintf(buf + offset, PAGE_SIZE - offset,
					"Read Fail Touch IC Info\n");
			return offset;
		}

		ret = nt11206_reg_read(dev, FW_INFO_OFFSET_1, fw_info, 11);
		if (ret < 0) {
			TOUCH_E("%s() id_data I2C read Fail\n", __func__);
			offset += snprintf(buf + offset, PAGE_SIZE, "-1\n");
			offset += snprintf(buf + offset, PAGE_SIZE - offset,
					"Read Fail Touch IC Info\n");
			return offset;
		}

		d->fw.fw_ver = fw_info[0];
		d->fw.fw_ver_bar = fw_info[1];
		d->fw.x_axis_num = fw_info[2];
		d->fw.y_axis_num = fw_info[3];
		d->fw.resolution_x = (fw_info[5] << 8) | fw_info[4];
		d->fw.resolution_y = (fw_info[7] << 8) | fw_info[6];
		d->fw.common_fw_ver = fw_info[8];
		d->fw.max_touch_num = fw_info[9];
		d->fw.b_num = fw_info[10];

		if((d->fw.fw_ver + d->fw.fw_ver_bar) != 0xFF) {
			TOUCH_E("FW info is broken\n");
			if(i == 3) {
				int ret = 0;
				ret = nt11206_sw_reset(dev, RESET_MCU_BOOT);
				if(ret < 0) {
					TOUCH_E("SW Reset Failed\n");
				}
				large_mdelay(20);
				nvt_set_i2c_debounce(dev);
			}
			large_mdelay(50);
			continue;
		}

		ret = nt11206_reg_write(dev, ADDR_CMD, serial_data, 2);
		if (ret < 0) {
			TOUCH_E("%s() serial_data I2C write Fail\n", __func__);
			offset += snprintf(buf + offset, PAGE_SIZE, "-1\n");
			offset += snprintf(buf + offset, PAGE_SIZE - offset,
					"Read Fail Touch IC Info\n");
			return offset;
		}
		memset(fw_info, 0x00, sizeof(fw_info));
		ret = nt11206_reg_read(dev, FW_INFO_OFFSET_2, fw_info, 2);
		if (ret < 0) {
			TOUCH_E("%s() id_data I2C read Fail\n", __func__);
			offset += snprintf(buf + offset, PAGE_SIZE, "-1\n");
			offset += snprintf(buf + offset, PAGE_SIZE - offset,
					"Read Fail Touch IC Info\n");
			return offset;
		}

		d->fw.custom_id = fw_info[0];
		d->fw.sensor_id = fw_info[1];
		TOUCH_I("fw_ver:0x%x, fw_ver_bar:0x%x, common_fw_ver:0x%x\n",
				d->fw.fw_ver, d->fw.fw_ver_bar, d->fw.common_fw_ver);
		TOUCH_I("custom_id:0x%x, sensor_id:0x%x\n",
				d->fw.custom_id, d->fw.sensor_id);
		TOUCH_I("x_axis_num:%d, y_axis_num:%d resolution_x:%d, resolution_y:%d\n",
				d->fw.x_axis_num, d->fw.y_axis_num, d->fw.resolution_x, d->fw.resolution_y);

		offset += snprintf(buf + offset, PAGE_SIZE, "fw_ver:0x%X, fw_ver_bar:0x%X, common_fw_ver:0x%X, custom_id:0x%X, sensor_id:0x%X\n",
				d->fw.fw_ver, d->fw.fw_ver_bar, d->fw.common_fw_ver, d->fw.custom_id, d->fw.sensor_id);

		offset += snprintf(buf + offset, PAGE_SIZE, "[IC_ID:0x%02X, FW_Ver:0.%d]\n",
				novatek_ts_chipid(dev), d->fw.fw_ver);
		break;
	}
	return offset;
}
static int novatek_get_cmd_atcmd_version(struct device *dev, char *buf)
{
	struct nt11206_data *d = to_nt11206_data(dev);
	u8 serial_data[2] = {0x01, 0x7C};
	u8 fw_info[11] = {0x00, };
	int ret = 0;
	int offset = 0;
	int i = 0;

	for(i = 0; i < 5;i++) {
		offset = 0;
		ret = nt11206_reg_write(dev, ADDR_CMD, serial_data, 2);
		if (ret < 0) {
			TOUCH_E("%s() serial_data I2C write Fail\n", __func__);
			offset += snprintf(buf + offset, PAGE_SIZE, "-1\n");
			offset += snprintf(buf + offset, PAGE_SIZE - offset,
					"Read Fail Touch IC Info\n");
			return offset;
		}

		ret = nt11206_reg_read(dev, FW_INFO_OFFSET_1, fw_info, 11);
		if (ret < 0) {
			TOUCH_E("%s() id_data I2C read Fail\n", __func__);
			offset += snprintf(buf + offset, PAGE_SIZE, "-1\n");
			offset += snprintf(buf + offset, PAGE_SIZE - offset,
					"Read Fail Touch IC Info\n");
			return offset;
		}

		d->fw.fw_ver = fw_info[0];
		d->fw.fw_ver_bar = fw_info[1];
		d->fw.x_axis_num = fw_info[2];
		d->fw.y_axis_num = fw_info[3];
		d->fw.resolution_x = (fw_info[5] << 8) | fw_info[4];
		d->fw.resolution_y = (fw_info[7] << 8) | fw_info[6];
		d->fw.common_fw_ver = fw_info[8];
		d->fw.max_touch_num = fw_info[9];
		d->fw.b_num = fw_info[10];

		if((d->fw.fw_ver + d->fw.fw_ver_bar) != 0xFF) {
			TOUCH_E("FW info is broken\n");
			if(i == 3) {
				int ret = 0;
				ret = nt11206_sw_reset(dev, RESET_MCU_BOOT);
				if(ret < 0) {
					TOUCH_E("SW Reset Failed\n");
				}
				large_mdelay(20);
				nvt_set_i2c_debounce(dev);
			}
			large_mdelay(50);
			continue;
		}

		offset += snprintf(buf + offset, PAGE_SIZE, "v%d.%02d\n",
				d->fw.fw_ver, d->fw.fw_ver_bar);
		break;
	}
	return offset;
}
static int novatek_ts_init_bootloader(struct device *dev)
{
	u8 serial_data[2] = {0x00, 0xA5};
	int ret;

	TOUCH_TRACE();

	//SW RESET
	ret = nt11206_bootloader_write(dev, serial_data, 2);
	if (ret < 0) {
		TOUCH_E("SW RESET Fail\n");
		return ret;
	}
	mdelay(20);

	nvt_set_i2c_debounce(dev);

	serial_data[0] = 0x00;
	serial_data[1] = 0x00;
	ret = nt11206_bootloader_write(dev, serial_data, 2);
	if (ret < 0) {
		TOUCH_E("Initial Flash Block Fail\n");
		return ret;
	}
	mdelay(20);

	serial_data[0] = 0;
	ret = nt11206_bootloader_read(dev, serial_data, 2);
	if (ret < 0) {
		TOUCH_E("Read Fail Status Fail\n");
		return ret;
	}

	if(serial_data[1] != 0xAA) {
		TOUCH_E("Check 0xAA (Inittial Flash Block) error!!\n");
		return -1;
	}

	TOUCH_I("Init OK\n");
	mdelay(20);
	return ret;
}

static int novatek_ts_erase_chip(struct device *dev)
{
	int ret = 0;
	int i = 0;
	u8 buf[5] = {0, };

	//Write enable
	buf[0] = 0x00;
	buf[1] = 0x06;
	ret = nt11206_bootloader_write(dev, buf, 2);
	if(ret < 0) {
		TOUCH_E("Write enable CMD Write Fail\n");
		return ret;
	}
	large_mdelay(10);

	//Check 0xAA (Write Enable)
	buf[0] = 0x00;
	ret = nt11206_bootloader_read(dev, buf, 2);
	if(ret < 0) {
		TOUCH_E("Write enable CMD Status Read Fail\n");
		return -1;
	}
	if(buf[1] != 0xAA) {
		TOUCH_E("Check 0xAA (Write Enable) error!! status=0x%02X\n", buf[1]);
		return -1;
	}
	large_mdelay(10);

	//Chip Erase
	buf[0] = 0x00;
	buf[1] = 0x60;
	buf[2] = 0x00;
	buf[3] = 0x00;
	buf[4] = 0x00;
	ret = nt11206_bootloader_write(dev, buf, 5);
	if(ret < 0) {
		TOUCH_E("Write enable CMD Write Fail\n");
		return -2;
	}
	large_mdelay(20);

	//Check 0xAA (Chip Erase)
	for(i = 0; i < 20; i++) {
		buf[0] = 0x00;
		ret = nt11206_bootloader_read(dev, buf, 2);
		if(ret < 0) {
			TOUCH_E("Chip Erase CMD Read Fail\n");
			return -2;
		}
		if(buf[1] != 0xAA) {
			TOUCH_E("try count:%d Check 0xAA (Chip Erase) error!! status=0x%02X\n", i, buf[1]);
			touch_msleep(250);
		}
		else {
			TOUCH_I("Chip Erase CMD Status Success(status:0x%02x)\n", buf[1]);
			break;
		}
		large_mdelay(10);
	}
	if(i == 20) {
		TOUCH_E("FAIL!!!!!!! Check 0xAA (Chip Erase) error!! status=0x%02X\n", buf[1]);
		return -2;
	}

	//Read Status
	for(i = 0; i < 20; i++) {
		large_mdelay(30);
		buf[0] = 0x00;
		buf[1] = 0x05;
		ret = nt11206_bootloader_write(dev, buf, 5);
		if(ret < 0) {
			TOUCH_E("Chip Erase CMD Read Fail\n");
			return -3;
		}

		buf[0] = 0x00;
		buf[1] = 0x00;
		buf[2] = 0x00;
		ret = nt11206_bootloader_read(dev, buf, 3);
		if(ret < 0) {
			TOUCH_E("Read Status CMD Read Fail\n");
			return -3;
		}
		if(buf[1] != 0xAA && buf[2] == 0x00) {
			TOUCH_E("try count:%d Check 0xAA (Chip Erase) error!! status1=0x%02X, status2=0x%02X\n", i, buf[1], buf[2]);
			touch_msleep(250);
		}
		else {
			TOUCH_I("Read Status CMD Success(status1:0x%02x, status2:0x%02x)\n", buf[1], buf[2]);
			break;
		}
	}

	if(i == 20) {
		TOUCH_E("FAIL!!!!!!! Check 0xAA, 0x00 (Read Status) error!! status1=0x%02X, status2=0x%02X\n", buf[1], buf[2]);
		return -3;
	}

	TOUCH_I("Erase OK\n");

	return ret;
}

static int novatek_ts_flash_chip(struct device *dev, const u8* images, const size_t fw_size)
{
	int ret = 0;
	u8 buf[256] = {0, };
	unsigned long XDATA_Addr = 0x14002;
	unsigned int Flash_Addr;
	int i = 0;
	int j = 0;
	int k = 0;
	u8 tmpvalue = 0;
	int count = 0;
	int progress = 0;

	// change I2C buffer index
	buf[0] = XDATA_Addr >> 16;
	buf[1] = (XDATA_Addr >> 8) & 0xFF;
	ret = nt11206_reg_write(dev, ADDR_CMD, buf, 2);
	if(ret < 0) {
		TOUCH_E("Change I2C buffer index error!!\n");
		return -1;
	}

	if(fw_size % NT_PAGE_SIZE) {
		count = (fw_size / 256) + 1;
	}
	else {
		count = fw_size / 256;
	}

	for(i = 0; i < count; i++) {
		Flash_Addr = i * NT_PAGE_SIZE;

		//Write Enable
		buf[0] = 0x00;
		buf[1] = 0x06;
		ret = nt11206_bootloader_write(dev, buf, 2);
		if(ret < 0) {
			TOUCH_E("Write Enable Write I2C error!\n");
			return -1;
		}
		mdelay(5);

		//Write Page : 256 bytes
		memset(buf, 0, 256);
		for(j = 0; j < NT_MIN((fw_size - Flash_Addr), NT_PAGE_SIZE); j++) {
			buf[j] = images[Flash_Addr + j];
		}

		ret = nt11206_reg_write(dev, UPLOAD_SRAM_FLASH_CMD, buf, NT_MIN(fw_size - (i * NT_PAGE_SIZE), NT_PAGE_SIZE));
		if(ret < 0) {
			TOUCH_E("Write Page error!!, j=%d\n", j);
		}

		if((fw_size - Flash_Addr) >= 256) {
			tmpvalue = ((Flash_Addr & 0xFF0000) >> 16) + ((Flash_Addr & 0x00FF00) >> 8) + (Flash_Addr & 0x0000FF) + 0x00 + 255;
		}
		else {
			tmpvalue = ((Flash_Addr & 0xFF0000) >> 16) + ((Flash_Addr & 0x00FF00) >> 8) + (Flash_Addr & 0x0000FF) + 0x00 + fw_size - Flash_Addr - 1;
		}

		for(k = 0;k < NT_MIN((fw_size - Flash_Addr), NT_PAGE_SIZE);k++) {
			tmpvalue += images[Flash_Addr + k];
		}

		tmpvalue = 255 - tmpvalue + 1;
		//PAGE Program
		buf[0] = 0x00;
		buf[1] = 0x02;
		buf[2] = (Flash_Addr & 0xFF0000) >> 16;
		buf[3] = (Flash_Addr & 0x00FF00) >> 8;
		buf[4] = Flash_Addr & 0x0000FF;
		buf[5] = 0x00;
		buf[6] = NT_MIN((fw_size - Flash_Addr), NT_PAGE_SIZE) - 1;
		buf[7] = tmpvalue;

		ret = nt11206_bootloader_write(dev, buf, 8);
		if(ret < 0) {
			TOUCH_E("Page Program error!!, i=%d\n", i);
			return -3;
		}

		//Check 0xAA (Page Program)
		large_mdelay(10);
		for(k = 0;k < 20;k++) {
			buf[0] = 0x00;
			buf[1] = 0x00;
			ret = nt11206_bootloader_read(dev, buf, 2);
			if(ret < 0) {
				TOUCH_E("Program Status READ CMD Fail\n");
				return -3;
			}

			if(buf[1] == 0xAA || buf[1] == 0xEA) {
				break;
			}
			else {
				TOUCH_E("Program Status ERROR!!!! Retry:%d Read status:0x%02X\n", k, buf[1]);
				large_mdelay(10);
			}
		}
		if(buf[1] == 0xEA) {
			TOUCH_E("Page Program error!! i=%d\n", i);
			return -3;
		}

		for(k = 0;k < 20;k++) {
			//Read Status
			buf[0] = 0x00;
			buf[1] = 0x05;
			ret = nt11206_bootloader_write(dev, buf, 2);
			if(ret < 0) {
				TOUCH_E("Read Status CMD I2C Write Fail\n");
				return -4;
			}
			large_mdelay(10);
			buf[0] = 0x00;
			buf[1] = 0x00;
			buf[2] = 0x00;
			ret = nt11206_bootloader_read(dev, buf, 3);
			if(ret < 0) {
				TOUCH_E("Read Status CMD I2C Read Fail\n");
				return -4;
			}

			if(((buf[1] == 0xAA) && (buf[2] == 0x00)) || (buf[1] == 0xEA)) {
				break;
			}
			else {
				TOUCH_E("READ Status ERROR!!!! Retry:%d Read status1:0x%02X, status2:0x%02X\n", k, buf[1], buf[2]);
				large_mdelay(10);
			}

		}

		if(buf[1] == 0xEA) {
			TOUCH_E("Page Program error!! i=%d\n", i);
			return -4;
		}
		if(progress != (i*100)/count) {
			progress = (i*100)/count;
			TOUCH_I("Programming...%2d%%\r", progress);
		}
	}
	TOUCH_I("Program OK\n");
	return ret;
}
static int verify_flash(struct device *dev, const u8* images, const size_t fw_size)
{
	int ret = 0;
	int retry = 0;
    u8 buf[64];
	u8 status_data[3] = {0x00, };
	u8 addr = 0x00;
	int XDATA_Addr=0x14000;
	int k;
	unsigned short WR_Filechksum = 0;
	unsigned short RD_Filechksum = 0;
	unsigned short WR_Filechksum2 = 0;
	unsigned short RD_Filechksum2 = 0;

	WR_Filechksum = 0x00 + 0x00 + 0x00 + ((fw_size % 65536 - 1) >> 8) + ((fw_size % 65536 - 1) & 0xFF);
	for(k = 0; k < NT_MIN(fw_size, 64*1024); k++) {
		WR_Filechksum = WR_Filechksum + images[k];
	}

	WR_Filechksum = 65535 - WR_Filechksum + 1;
	if(fw_size > 64*1024) {
		WR_Filechksum2 = 0x00 + 0x00 + 0x00 + (((fw_size - 64 * 1024) - 1) >> 8) + (((fw_size - 64 * 1024) - 1) & 0xFF);

		for(k = 0;k<(fw_size - 64 * 1024);k++)
		{
			WR_Filechksum2 += images[k+64*1024];
		}
		WR_Filechksum2 = 65535 - WR_Filechksum2 + 1;
	}

	buf[0] = 0x00;
	buf[1] = 0x07;
	buf[2] = 0x00;
	buf[3] = 0x00;
	buf[4] = 0x00;
	buf[5] = ((NT_MIN(fw_size, 64*1024)-1) >> 8);
	buf[6] = ((NT_MIN(fw_size, 64*1024)-1) & 0xFF);
	ret = nt11206_bootloader_write(dev, buf, 7);
	if(ret < 0) {
		TOUCH_E("Fast Read CMD Error\n");
		return -1;
	}

	touch_msleep(50);
	for(retry = 20; retry > 0; retry--)
	{
		status_data[0] = 0x00;
		ret = nt11206_bootloader_read(dev, status_data, 2);
		if (ret < 0) {
			TOUCH_E("Read Status Fail\n");
			return -1;
		}

		if(status_data[1] != 0xAA) {
			TOUCH_E("Read Status data Fail(0x%02x)\n", status_data[1]);
			large_mdelay(50);
		}
		else {
			break;
		}
	}
	if(retry == 0)
	{
		TOUCH_E("%s() Status data Fail after Fast Read CMD\n\n\n",
				__func__);
		return -1;
	}

	buf[0] = (XDATA_Addr & 0xFF0000) >> 16;
	buf[1] = (XDATA_Addr & 0x00FF00) >> 8;
	ret = nt11206_reg_write(dev, ADDR_CMD, buf, 2);
	if(ret < 0) {
		TOUCH_E("Read Checksum (write addr high byte & middle byte) error\n");
		return -1;
	}

	buf[0] = 0x00;
	buf[1] = 0x00;
	addr = XDATA_Addr & 0x0000FF;
	ret = nt11206_reg_read(dev, addr, buf, 2);
	if(ret < 0) {
		TOUCH_E("Read Checksum error\n");
		return -1;
	}

	RD_Filechksum = (unsigned short)((buf[1] << 8) | buf[0]);
	if(WR_Filechksum != RD_Filechksum)
	{
		TOUCH_E("Verify Fail1!!\n");
		TOUCH_E("RD_Filechksum=0x%04X, WR_Filechksum=0x%04X\n",RD_Filechksum, WR_Filechksum);
		return -1;
	}

	TOUCH_I("RD_Filechksum=0x%04X, WR_Filechksum=0x%04X\n",RD_Filechksum, WR_Filechksum);
	if(fw_size > (64 * 1024)) {
		buf[0] = 0x00;
		buf[1] = 0x07;
		buf[2] = 0x01;
		buf[3] = 0x00;
		buf[4] = 0x00;
		buf[5]=((NT_MIN(fw_size, 64*1024) - 1) >> 8);
		buf[6]=((NT_MIN(fw_size, 64*1024) - 1) & 0xFF);

		ret = nt11206_bootloader_write(dev, buf, 7);
		if(ret < 0) {
			TOUCH_E("Fast Read Command error!!\n");
			return -2;
		}

		TOUCH_I("For Reading Status, Flash Status after Fast Read CMD\n");
		for(retry=200; retry>0; retry--)
		{
			buf[0] = 0x00;
			ret = nt11206_bootloader_read(dev, buf, 3);
			if(ret < 0) {
				TOUCH_E("Read Status Fail\n");
				return -2;
			}

			if(buf[1] != 0xAA) {
				TOUCH_E("Check 0xAA (Fast Read Command) error\n");
			}
			else {
				break;
			}

		}
		if(retry == 0)
		{
			TOUCH_E("Status data Fail after Fast Read CMD\n\n\n");
			return -2;
		}

		buf[0] = XDATA_Addr >> 16;
		buf[1] = (XDATA_Addr >> 8) & 0xFF;
		ret = nt11206_reg_write(dev, ADDR_CMD, buf, 2);
		if(ret < 0) {
			TOUCH_E("Read Checksum (write addr high byte & middle byte) error\n");
			return -1;
		}

		buf[0] = 0x00;
		buf[1] = 0x00;
		addr = XDATA_Addr & 0xFF;
		ret = nt11206_reg_write(dev, addr, buf, 2);
		if(ret < 0) {
			TOUCH_E("Read Checksum error\n");
			return -1;
		}

		RD_Filechksum2 = (unsigned short)((buf[1] << 8) | buf[0]);

		if(WR_Filechksum2 != RD_Filechksum2)
		{
			TOUCH_E("Verify Fail2!!\n");
			TOUCH_E("RD_Filechksum2=0x%04X, WR_Filechksum2=0x%04X\n",RD_Filechksum2, WR_Filechksum2);
			return -1;
		}
		TOUCH_I("RD_Filechksum2=0x%04X, WR_Filechksum2=0x%04X\n",RD_Filechksum2, WR_Filechksum2);
	}

	TOUCH_I("Verify OK\n");
	return ret;
}

static int novatek_lpwg_fail_init(struct device *dev)
{
	struct nt11206_data *d = to_nt11206_data(dev);
	int ret = 0;
	u8 serial_data[2] = {0x01, 0x2F};
	u8 start_addr = 0x00;
	u8 fail_reason[10] = {0x00, };

	if(d->mode_state == POWER_OFF_MODE) {
		return 0;
	}
	TOUCH_I("%s()\n", __func__);
	ret = nt11206_reg_write(dev, ADDR_CMD, serial_data, 2);
	if(ret < 0) {
		TOUCH_E("irq_handler serial data I2C write Fail\n");
		return ret;
	}

	ret = nt11206_reg_write(dev, start_addr, fail_reason, 10);
	if(ret < 0) {
		TOUCH_E("lpwg fail data I2C write Fail\n");
		return ret;
	}

	return 0;
}
static int novatek_lpwg_fail_reason(struct device *dev, int mode)
{
	int ret = 0;
	u8 serial_data[2] = {0x01, 0x2F};
	u8 start_addr = 0x00;
	u8 fail_reason[10] = {0x00, };
	int i = 0;

	if(mode == LPWG_NONE) {
		return ret;
	}

	ret = nt11206_reg_write(dev, ADDR_CMD, serial_data, 2);
	if(ret < 0) {
		TOUCH_E("irq_handler serial data I2C write Fail\n");
		return ret;
	}

	ret = nt11206_reg_read(dev, start_addr, fail_reason, 10);
	if(ret < 0) {
		TOUCH_E("lpwg fail data I2C read Fail\n");
		return ret;
	}

	TOUCH_I("LPWG Fail Reason\n");
	for(i = 0; i < 10;i++) {
		if(mode == LPWG_DOUBLE_TAP) {
			if((fail_reason[i] & 0x0F) < 7) {
				TOUCH_I("Index:%d %s\n",i , tci_debug_str[fail_reason[i] & 0x0F]);
			}
			else {
				TOUCH_E("Index:%d, fail_num:%d\n", i, fail_reason[i] & 0x0F);
			}
		}
		else if(mode == LPWG_PASSWORD) {
			if(((fail_reason[i] & 0xF0) >> 4) < 7) {
				TOUCH_I("Index:%d %s\n",i ,tci_debug_str[(fail_reason[i] & 0xF0) >> 4]);
			}
			else {
				TOUCH_E("Index:%d, fail_num:%d\n", i, (fail_reason[i] & 0xF0) >> 4);
			}
		}
		else {
			return 0;
		}
	}
	printk("\n");
	return ret;
}
static int novatek_lpwg_read(struct device *dev, u8 *buf)
{
	int ret = 0;
	int i = 0;
	u8 serial_data[2] = {0x01, 0x47};
	u8 read_cmd_addr = 0x4E;
	u8 read_data_addr = 0x23;
	u8 read_cmd = 0x22;
	u8 read_check_addr = 0x4F;
	u8 tmp = 0;
	u8 buf_tmp[38] = {0, };

	ret = nt11206_reg_write(dev, ADDR_CMD, serial_data, 2);
	if(ret < 0) {
		TOUCH_E("%s() irq_handler serial data I2C write Fail\n", __func__);
		return ret;
	}

	ret = nt11206_reg_write(dev, read_data_addr, buf_tmp, 38);
	if(ret < 0) {
		TOUCH_E("Read Data Register RESET FAIL\n");
	}

//set 0x00 to 0x1474F
	tmp = 0x00;
	ret = nt11206_reg_write(dev, read_check_addr, &tmp, 1);
	if(ret < 0) {
		TOUCH_E("Write SET 0 to 0x1474F Fail\n");
		return ret;
	}

	ret = nt11206_reg_write(dev, read_cmd_addr, &read_cmd, 1);
	if(ret < 0) {
		TOUCH_E("Write READ_CMD Fail\n");
		return ret;
	}

	for(i=0;i<5;i++) {
		msleep(15);
		tmp = 0;
		ret = nt11206_reg_read(dev, read_check_addr, &tmp, 1);
		if(ret < 0) {
			TOUCH_E("Read Read_Check_Addr Fail\n");
			return ret;
		}
		if(tmp == 0xAA)
			break;
		else {
			TOUCH_E("wait time count:%d\n",i);
		}
	}

	ret = nt11206_reg_read(dev, read_data_addr, buf, 34);
	if(ret < 0) {
		TOUCH_E("Read LPWG Data Fail\n");
		return ret;
	}

	return ret;
}

static void nt11206_get_tci_info(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);

	ts->tci.info[TCI_1].tap_count = 2;
	//TCHIMMAX
	ts->tci.info[TCI_1].min_intertap = 64; //time between press and release, 640ms, 1->10ms
	//NOTCHIMMAX
	ts->tci.info[TCI_1].max_intertap = 80; //time between release and next press, 800ms, 1->10ms
	//TCHMOVMAX
	ts->tci.info[TCI_1].touch_slop = 120; //10mm, 1mm -> 12
	//NOTCHMOVMAX
	ts->tci.info[TCI_1].tap_distance = 240;  //20mm, 1mm -> 12
	ts->tci.info[TCI_1].intr_delay = 0;

	//TCHIMMAX
	ts->tci.info[TCI_2].min_intertap = 64; //time between press and release, 640ms, 1 -> 10ms
	//NOTCHIMMAX
	ts->tci.info[TCI_2].max_intertap = 84; //time between release and next press, 800ms, 1->10ms
	//DTTCHMOVMAX
	ts->tci.info[TCI_2].touch_slop = 120; //10mm, 1mm -> 12
	ts->tci.info[TCI_2].tap_distance = 255; // Full Screen set value??
	ts->tci.info[TCI_2].intr_delay = 0; //atmel T93 18, 19th pre, post
}
static int nt11206_get_position(struct device *dev, int count, u8 *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int ret = 1;
	int i = 0;
	//int start_idx = 22;
	//int start_idx = 31;

	ts->lpwg.code_num = count;
	memset(ts->lpwg.code, 0, sizeof(struct point) * MAX_LPWG_CODE);
	for(i = 2; i < count * 4 + 2;i = i + 4) {
		ts->lpwg.code[i/4].x =  (buf[i + 1] << 8) | buf[i];
		ts->lpwg.code[i/4].y =  (buf[i + 3] << 8) | buf[i + 2];
		if(ts->role.hide_coordinate) {
			TOUCH_I("%s() idx:%d, x:xxxxx, y:xxxxx\n", __func__, i/4);
		}
		else {
			TOUCH_I("%s() idx:%d, x:%d, y:%d\n", __func__, i/4, ts->lpwg.code[i/4].x, ts->lpwg.code[i/4].y);
		}
	}

	ts->lpwg.code[i/4].x =  -1;
	ts->lpwg.code[i/4].y =  -1;
	return ret;
}

static int nt11206_mode_change(struct device *dev, int mode, int screen)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct nt11206_data *d = to_nt11206_data(dev);
	int ret = 0;
	// CMD(0xFF), ADDR_H(0x01), ADDR_L(0x47) (0x014700)
	u8 serial_data[2] = {0x01, 0x47};
	// OFFSET(0x50) + DEEP SLEEP (0x12)
	u8 power_off_data = 0;
	u8 gesture_data[3] = {0, };

	power_off_data = 0x11;
	gesture_data[0] = 0x13;
	gesture_data[1] = 0xFF;
	gesture_data[2] = 0xFF;

	TOUCH_I("%s() START mode:%d, screen:%d\n", __func__, mode, screen);
	if((mode == LPWG_DOUBLE_TAP) || (mode == LPWG_PASSWORD)) {
		if(d->mode_state == POWER_OFF_MODE) {
			return 0;
		}
		ret = nt11206_reg_read_dummy(dev, SLAVE_ADDR);
		if(ret < 0) {
			TOUCH_E("Read dummy Failed\n");
			return ret;
		}
		ret = nt11206_reg_write(dev, ADDR_CMD, serial_data, 2);
		if (ret < 0) {
			TOUCH_E("Serial data I2C write Fail\n");
			return ret;
		}
		ret = nt11206_mode_change_check(dev, gesture_data, 3);
		if(!ret) {
			d->mode_state = GESTURE_MODE;
		}
		else {
			TOUCH_E("Hand shake FAIL\n");
			d->mode_state = GESTURE_MODE;
		}
	}
	else {
		if(screen && (ts->lpwg.qcover != HOLE_NEAR)) {
			if(d->mode_state == POWER_OFF_MODE) {
				nt11206_power(dev, POWER_ON);
				nt11206_interrupt_control(dev, INTERRUPT_ENABLE);
			}
			ret = nt11206_sw_reset(dev, RESET_MCU_BOOT);
			d->mode_state = NORMAL_MODE;
		}
		else {
			if(d->mode_state != POWER_OFF_MODE) {
				ret = nt11206_reg_read_dummy(dev, SLAVE_ADDR);
				if(ret < 0) {
					TOUCH_E("Read dummy Failed\n");
					return ret;
				}
				ret = nt11206_reg_write(dev, ADDR_CMD, serial_data, 2);
				if (ret < 0) {
					TOUCH_E("Serial data I2C write Fail\n");
					return ret;
				}
				ret = nt11206_mode_change_check(dev, &power_off_data, 1);
			}
			else {
				ret = 0;
			}

			if(!ret) {
				nt11206_power(dev, POWER_OFF);
				d->mode_state = POWER_OFF_MODE;
				nt11206_interrupt_control(dev, INTERRUPT_DISABLE);
			}
			else {
				TOUCH_E("Hand shake FAIL\n");
				nt11206_power(dev, POWER_OFF);
				d->mode_state = POWER_OFF_MODE;
				nt11206_interrupt_control(dev, INTERRUPT_DISABLE);
			}
		}
	}

	TOUCH_I("%s() END\n", __func__);
	return 0;
}
static int nt11206_set_lpwg(struct device *dev, int mode)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct nt11206_data *d = to_nt11206_data(dev);
	int ret = -1;
	u8 buf[2] = {0x00, };
	u8 serial_data[2] = {0x01, 0x47};
	u8 start_addr = 0x06;
	u8 write_cmd_addr = 0x4E;
	u8 ctrl_add = 0x4D;
	u8 ctrl_value = 0x00;
	u8 start_protect = 0x44;
	u8 write_cmd = 0x11;
	u8 write_check_addr = 0x4F;
	u8 reset_value = 0x00;
	u8 success = 0;
	int i = 0;
	int j = 0;

	if(d->mode_state == POWER_OFF_MODE) {
		return 0;
	}

	TOUCH_I("LPWG Set Parameter START\n");

	if(mode == LPWG_DOUBLE_TAP) {
		ctrl_value = 0x81;
	}
	else if(mode == LPWG_PASSWORD) {
		if(!ts->tci.info[TCI_2].intr_delay) {
			ctrl_value = 0x83;
		}
		else {
			ctrl_value = 0x83 | 0x20;
		}
	}
	else {
		ctrl_value = 0x00;
		return ret;
	}

	if(d->tci_debug) {
		ctrl_value |= 0x40;
	}

	for(i = 0 ; i < 5;i++) {
		ret = nt11206_reg_write(dev, ADDR_CMD, serial_data, 2);
		if(ret < 0) {
			TOUCH_E("%s() Offset data I2C write Fail\n", __func__);
			return ret;
		}
		//0x1474F set 0x00
		reset_value = 0x00;
		ret = nt11206_reg_write(dev, write_check_addr, &reset_value, 1);
		if(ret < 0) {
			TOUCH_E("Reset Value Write Fail\n");
			return ret;
		}
		//0x1474E set 0x44
		start_protect = 0x44;
		ret = nt11206_reg_write(dev, write_cmd_addr, &start_protect, 1);
		if(ret < 0) {
			TOUCH_E("Protect Value Write Fail\n");
			return ret;
		}

		for(j = 0; j < 5; j++) {
			//Protect reg(0x1474F) read
			buf[0] = 0;
			msleep(5);
			ret = nt11206_reg_read(dev, write_check_addr, buf, 1);
			if(ret < 0) {
				TOUCH_E("Protect Value Read Fail\n");
				return 0;
			}
			if(buf[0] == 0xAA) {
				//0x1474F set 0x00
				reset_value = 0x00;
				ret = nt11206_reg_write(dev, write_check_addr, &reset_value, 1);
				if(ret < 0) {
					TOUCH_E("Reset Value Write Fail\n");
					return ret;
				}
				break;
			}
		}
		if(j == 5) {
			TOUCH_E("Protect Hand Shake FAIL!!!\n");
		}
		ret = nt11206_reg_write(dev, ctrl_add, &ctrl_value, 1);
		if(ret < 0) {
			TOUCH_E("%s() LPWG CTRL data I2C write Fail\n", __func__);
			return ret;
		}

		//XMIN
		buf[0] = 0x76;
		buf[1] = 0x00;
		ret = nt11206_reg_write(dev, start_addr, buf, 2);
		if(ret < 0) {
			TOUCH_E("Write XMIN Fail\n");
			return ret;
		}

		//XMAX
		buf[0] = 0x3A;
		buf[1] = 0x04;
		ret = nt11206_reg_write(dev, start_addr + 2, buf, 2);
		if(ret < 0) {
			TOUCH_E("Write XMAX Fail\n");
			return ret;
		}

		//YMIN
		buf[0] = 0x6F;
		buf[1] = 0x00;
		ret = nt11206_reg_write(dev, start_addr + 4, buf, 2);
		if(ret < 0) {
			TOUCH_E("Write YMIN Fail\n");
			return ret;
		}

		//YMAX
		buf[0] = 0x11;
		buf[1] = 0x07;
		ret = nt11206_reg_write(dev, start_addr + 6, buf, 2);
		if(ret < 0) {
			TOUCH_E("Write YMAX Fail\n");
			return ret;
		}

		//TCHMOVMAX
		buf[0] = ts->tci.info[TCI_1].touch_slop; //120
		buf[1] = 0x00;
		ret = nt11206_reg_write(dev, start_addr + 8, buf, 2);
		if(ret < 0) {
			TOUCH_E("Write TCHMOVMAX Fail\n");
			return ret;
		}

		//NOTCHMOVMAX
		buf[0] = 0xFF;
		buf[1] = 0xFF;
		ret = nt11206_reg_write(dev, start_addr + 10, buf, 2);
		if(ret < 0) {
			TOUCH_E("Write NOTCHMOVMAX Fail\n");
			return ret;
		}

		//TCHIMMIN
		buf[0] = 0x00;
		ret = nt11206_reg_write(dev, start_addr + 12, buf, 1);
		if(ret < 0) {
			TOUCH_E("Write TCHIMMIN Fail\n");
			return ret;
		}

		//TCHIMMAX
		buf[0] = ts->tci.info[TCI_1].min_intertap;;
		ret = nt11206_reg_write(dev, start_addr + 13, buf, 1);
		if(ret < 0) {
			TOUCH_E("Write TCHIMMAX Fail\n");
			return ret;
		}

		//NOTCHIMMIN
		buf[0] = 0x00;
		ret = nt11206_reg_write(dev, start_addr + 14, buf, 1);
		if(ret < 0) {
			TOUCH_E("Write NOTCHIMMIN Fail\n");
			return ret;
		}

		//NOTCHIMMAX
		buf[0] = ts->tci.info[TCI_1].max_intertap;
		ret = nt11206_reg_write(dev, start_addr + 15, buf, 1);
		if(ret < 0) {
			TOUCH_E("Write NOTCHIMMAX Fail\n");
			return ret;
		}

		//TAP CNT
		if(mode == LPWG_DOUBLE_TAP) {
			buf[0] = ts->tci.info[TCI_1].tap_count;
		}
		else {
			buf[0] = ts->tci.info[TCI_2].tap_count;
		}
		ret = nt11206_reg_write(dev, start_addr + 16, buf, 1);
		if(ret < 0) {
			TOUCH_E("Write Tap CNT Fail\n");
			return ret;
		}

		if(mode == LPWG_DOUBLE_TAP) {
			//DTPREWINTIM
			buf[0] = 0x00;
			buf[1] = 0x00;
			ret = nt11206_reg_write(dev, start_addr + 17, buf, 2);
			if(ret < 0) {
				TOUCH_E("Write DTPREWINTIM Fail\n");
				return ret;
			}

			//DTPOSTWINTIM
			buf[0] = 0x32;
			buf[1] = 0x00;
			ret = nt11206_reg_write(dev, start_addr + 19, buf, 2);
			if(ret < 0) {
				TOUCH_E("Write DTPOSTWINTIM Fail\n");
				return ret;
			}
		}
		else if(mode == LPWG_PASSWORD) {
			//DTPREWINTIM
			buf[0] = 0x32;
			buf[1] = 0x00;
			ret = nt11206_reg_write(dev, start_addr + 17, buf, 2);
			if(ret < 0) {
				TOUCH_E("Write DTPREWINTIM Fail\n");
				return ret;
			}

			//DTPOSTWINTIM
			buf[0] = 0x00;
			buf[1] = 0x00;
			ret = nt11206_reg_write(dev, start_addr + 19, buf, 2);
			if(ret < 0) {
				TOUCH_E("Write DTPOSTWINTIM Fail\n");
				return ret;
			}
		}

		//DTTCHMOVMAX
		buf[0] = ts->tci.info[TCI_2].touch_slop;
		buf[1] = 0x00;
		ret = nt11206_reg_write(dev, start_addr + 21, buf, 2);
		if(ret < 0) {
			TOUCH_E("Write DTTCHMOVMAX Fail\n");
			return ret;
		}

		if(mode == LPWG_DOUBLE_TAP) {
			//DTNOTCHMOVMAX
			buf[0] = 0x78;//10mm
			buf[1] = 0;
			ret = nt11206_reg_write(dev, start_addr + 23, buf, 2);
			if(ret < 0) {
				TOUCH_E("Write DTNOTCHMOVMAX Fail\n");
				return ret;
			}
		}
		else if(mode == LPWG_PASSWORD) {
			//DTNOTCHMOVMAX
			buf[0] = 0x54; //7mm
			buf[1] = 0;
			ret = nt11206_reg_write(dev, start_addr + 23, buf, 2);
			if(ret < 0) {
				TOUCH_E("Write DTNOTCHMOVMAX Fail\n");
				return ret;
			}
		}

		//DTTCHTIMMIN
		buf[0] = 0x03; //30ms
		ret = nt11206_reg_write(dev, start_addr + 25, buf, 1);
		if(ret < 0) {
			TOUCH_E("Write DTTCHITIMIN Fail\n");
			return ret;
		}

		//DTTCHTIMMAX
		buf[0] = 0x68; //1040ms
		ret = nt11206_reg_write(dev, start_addr + 26, buf, 1);
		if(ret < 0) {
			TOUCH_E("Write DTTCHTIMMAX Fail\n");
			return ret;
		}

		//DTNOTCHTIMMIN
		buf[0] = 0x04; //40ms
		ret = nt11206_reg_write(dev, start_addr + 27, buf, 1);
		if(ret < 0) {
			TOUCH_E("Write DTNOTCHTIMMIN Fail\n");
			return ret;
		}

		//DTNOTCHTIMMAX
		buf[0] = 0x50; //750ms
		ret = nt11206_reg_write(dev, start_addr + 28, buf, 1);
		if(ret < 0) {
			TOUCH_E("Write DTNOTCHTIMMAX Fail\n");
			return ret;
		}

		//set 0 to 0x1474F
		buf[0] = 0;
		ret = nt11206_reg_write(dev, write_check_addr, buf, 1);
		if(ret < 0) {
			TOUCH_E("Write to check writing LPWG parameters Fail\n");
			return ret;
		}
		write_cmd = 0x11;
		ret = nt11206_reg_write(dev, write_cmd_addr, &write_cmd, 1);
		if(ret < 0) {
			TOUCH_E("Write Write_CMD Fail\n");
			return ret;
		}

		for(j = 0 ; j < 5; j++) {
			buf[0] = 0;
			msleep(5);
			ret = nt11206_reg_read(dev, write_check_addr, buf, 1);
			if(ret < 0) {
				TOUCH_E("Read to check writing LPWG parameters Fail\n");
				return ret;
			}

			if(buf[0] == 0xAA) {
				TOUCH_I("Read to check writing LPWG parameters success\n");
				buf[0] = 0;
				ret = nt11206_reg_write(dev, write_check_addr, buf, 1);
				if(ret < 0) {
					TOUCH_E("Write to check writing LPWG parameters Fail\n");
					return ret;
				}
				success = 1;
				break;
			}
		}
		if(j == 5) {
			TOUCH_E("Protect Hand Shake FAIL!!!\n");
		}
		if(success) {
			break;
		}
	}
	TOUCH_I("LPWG Set Parameter END\n");
	return ret;
}
static int nt11206_lpwg_control(struct device *dev, int mode)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct nt11206_data *d = to_nt11206_data(dev);
	int ret = 0;

	TOUCH_I("(POWER status:%d) nt11206_lpwg_control mode=%d, tci.mode:%d\n", d->mode_state, mode, ts->tci.mode);
	if((!mode && !ts->tci.mode) && (d->mode_state != POWER_OFF_MODE)) {
		return 0;
	}

	ret = novatek_lpwg_fail_init(dev);
	switch (mode) {
	case LPWG_DOUBLE_TAP:
		ts->tci.mode = 0x01;
		ret = nt11206_set_lpwg(dev, mode);
		break;

	case LPWG_PASSWORD:
		ts->tci.mode = 0x02;
		ret = nt11206_set_lpwg(dev, mode);
		break;

	default:
		ts->tci.mode = 0;
		break;
	}
	ret = nt11206_mode_change(dev, mode, ts->lpwg.screen);

	return 0;
}
static int novatek_lpwg_mode(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct nt11206_data *d = to_nt11206_data(dev);

	if (atomic_read(&ts->state.fb) == FB_SUSPEND) {
		if (ts->role.mfts_lpwg && d->boot_mode) {
			nt11206_lpwg_control(dev, LPWG_DOUBLE_TAP);
			return 0;
		}
		if (ts->lpwg.mode == LPWG_NONE) {
			/* deep sleep */
			TOUCH_I("%s(%d) - deep sleep\n",
				__func__, __LINE__);
			nt11206_lpwg_control(dev, LPWG_NONE);
		} else if (ts->lpwg.screen) {
			TOUCH_I("%s(%d) - FB_SUSPEND & screen on -> skip\n",
				__func__, __LINE__);
			return 0;
		} else if (ts->lpwg.sensor == PROX_NEAR) {
			/* deep sleep */
			TOUCH_I("%s(%d) - deep sleep by prox\n",
				__func__, __LINE__);
			nt11206_lpwg_control(dev, LPWG_NONE);
		} else if (ts->lpwg.qcover == HOLE_NEAR) {
			/* knock on */
			TOUCH_I("%s(%d) - deep sleep by hole\n",
				__func__, __LINE__);
			nt11206_lpwg_control(dev, LPWG_NONE);
		} else {
			/* knock on/code */
			TOUCH_I("%s(%d) - knock %d\n",
				__func__, __LINE__, ts->lpwg.mode);
			nt11206_lpwg_control(dev, ts->lpwg.mode);
		}
		return 0;
	}

	/* resume */
	if (ts->lpwg.screen && (ts->lpwg.qcover != HOLE_NEAR)) {
		/* normal */
		TOUCH_I("%s(%d) - normal\n",
				__func__, __LINE__);
		nt11206_lpwg_control(dev, LPWG_NONE);
	} else if (ts->lpwg.mode == LPWG_NONE) {
		/* normal */
		TOUCH_I("%s(%d) - normal on screen off\n",
				__func__, __LINE__);
		nt11206_lpwg_control(dev, LPWG_NONE);
	} else if (ts->lpwg.sensor == PROX_NEAR) {
		/* wake up */
		TOUCH_I("%s(%d) - wake up on screen off and prox\n",
				__func__, __LINE__);
		TOUCH_I("%s - wake up is not ready\n", __func__);
		nt11206_lpwg_control(dev, LPWG_NONE);
	} else {
		/* partial */
		TOUCH_I("%s(%d) - parial mode\n",
				__func__, __LINE__);
		nt11206_lpwg_control(dev, ts->lpwg.mode);
	}

	return 0;
}

static int nt11206_fw_compare(struct device *dev, const struct firmware *fw)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct nt11206_data *d = to_nt11206_data(dev);
	int update = 0;
	u8 fw_ver = 0;
	u8 fw_ver_bar = 0;
	u8 common_fw_ver = 0;

	fw_ver = fw->data[0 + FW_IMG_INFO_START];
	fw_ver_bar = fw->data[1 + FW_IMG_INFO_START];
	common_fw_ver = fw->data[8 + FW_IMG_INFO_START];

	if (ts->force_fwup) {
		update = 1;
	}

	if(fw_ver != d->fw.fw_ver) {
		update = 1;
	}

	if(fw_ver_bar != d->fw.fw_ver_bar) {
		update = 1;
	}

	if(common_fw_ver != d->fw.common_fw_ver) {
		update = 1;
	}

	TOUCH_I("bin-ver:0x%02x, ver_bar:0x%02x, common_ver:0x%02x\n",
			fw_ver, fw_ver_bar, common_fw_ver);
	TOUCH_I("dev-ver:0x%02x, ver_bar:0x%02x, common_ver:0x%02x, update:%d\n",
			d->fw.fw_ver, d->fw.fw_ver_bar, d->fw.common_fw_ver, update);

	return update;
}

static int nt11206_fw_upgrade(struct device *dev, const struct firmware *fw)
{
	int ret = 0;
	u8 buf[2] = {0x00, };
	struct nt11206_data *d = to_nt11206_data(dev);

	d->mode_state = FW_UPGRADE_MODE;

	ret = novatek_ts_chipid(dev);
	if(ret < 0) {
		TOUCH_E("Fail Read CHIP_ID\n");
		d->mode_state = NORMAL_MODE;
		return ret;
	}

	ret = novatek_ts_init_bootloader(dev);
	if(ret < 0) {
		TOUCH_E("Fail Init Bootloader\n");
		d->mode_state = NORMAL_MODE;
		return ret;
	}
	large_mdelay(50);
	ret = novatek_ts_erase_chip(dev);
	if(ret < 0) {
		TOUCH_E("Fail Erase Chip\n");
		d->mode_state = NORMAL_MODE;
		return ret;
	}

	touch_msleep(450);
	ret = novatek_ts_flash_chip(dev, fw->data, fw->size);
	if(ret < 0) {
		TOUCH_E("Fail Flash Chip\n");
		d->mode_state = NORMAL_MODE;
		return ret;
	}

	touch_msleep(500);
	ret = verify_flash(dev, fw->data, fw->size);
	if(ret < 0) {
		TOUCH_E("ret : %d Verfiy Fail\n", ret);
		d->mode_state = NORMAL_MODE;
		return ret;
	}
	buf[0] = 0x00;
	buf[1] = 0x69;
	ret = nt11206_bootloader_write(dev, buf, 2);
	if(ret < 0) {
		TOUCH_E("Bootloader Reset Error\n");
		d->mode_state = NORMAL_MODE;
		return ret;
	}

	d->mode_state = NORMAL_MODE;

	return 0;
}

static int nt11206_upgrade(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	const struct firmware *fw;

	char fwpath[256];
	int ret = 0;
	int i = 0;

	nt11206_interrupt_control(dev, INTERRUPT_DISABLE);
	if (atomic_read(&ts->state.fb) >= FB_SUSPEND) {
		TOUCH_I("state.fb is not FB_RESUME\n");
		nt11206_interrupt_control(dev, INTERRUPT_ENABLE);
		return -EPERM;
	}

	if (ts->test_fwpath[0]) {
		memcpy(fwpath, &ts->test_fwpath[0], sizeof(fwpath));
		TOUCH_I("get fwpath from test_fwpath:%s\n",
			&ts->test_fwpath[0]);
	} else if (ts->def_fwcnt) {
		memcpy(fwpath, ts->def_fwpath[0], sizeof(fwpath));
		TOUCH_I("get fwpath from def_fwpath[0] : %s\n",
			ts->def_fwpath[0]);
	} else {
		TOUCH_E("no firmware file\n");
		return -EPERM;
	}

	if (fwpath == NULL) {
		TOUCH_E("error get fw path\n");
		return -EPERM;
	}

	TOUCH_I("fwpath[%s]\n", fwpath);

	ret = request_firmware(&fw, fwpath, dev);

	if (ret < 0) {
		TOUCH_E("fail to request_firmware fwpath: %s (ret:%d)\n",
			fwpath, ret);

		return ret;
	}

	TOUCH_I("fw size:%zu, data: %p\n", fw->size, fw->data);

	ret = novatek_ts_read_fw_info(dev);
	if(0 > ret) {
		TOUCH_E("Read FW Info Fail\n");
	}

	if (nt11206_fw_compare(dev, fw)) {
		ret = -EINVAL;
		touch_msleep(200);
		for (i = 0; i < 2 && ret; i++) {
			ret = nt11206_fw_upgrade(dev, fw);
		}
	}
	else {
		release_firmware(fw);
		return -EPERM;
	}
	release_firmware(fw);

	return ret;
}
static int nt11206_lpwg(struct device *dev, u32 code, void *param)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int *value = (int *)param;

	TOUCH_TRACE();

	switch (code) {
	case LPWG_ACTIVE_AREA:
		ts->lpwg.area[0].x = value[0];
		ts->lpwg.area[0].y = value[2];
		ts->lpwg.area[1].x = value[1];
		ts->lpwg.area[1].y = value[3];
		TOUCH_I("LPWG AREA (%d,%d)-(%d,%d)\n",
			ts->lpwg.area[0].x, ts->lpwg.area[0].y,
			ts->lpwg.area[1].x, ts->lpwg.area[1].y);
		break;
	case LPWG_TAP_COUNT:
		ts->tci.info[TCI_2].tap_count = value[0];
		break;
	case LPWG_DOUBLE_TAP_CHECK:
		TOUCH_I("%s() value:%d\n",__func__, value[0]);
		if(value[0])
			ts->tci.info[TCI_2].intr_delay = 70;
		else
			ts->tci.info[TCI_2].intr_delay = 0;
		break;

	case LPWG_UPDATE_ALL:
		ts->lpwg.mode = value[0];
		ts->lpwg.screen = value[1];
		ts->lpwg.sensor = value[2];
		ts->lpwg.qcover = value[3];
		TOUCH_I(
			"LPWG_UPDATE_ALL: mode[%d], screen[%s], sensor[%s], qcover[%s]\n",
			ts->lpwg.mode,
			ts->lpwg.screen ? "ON" : "OFF",
			ts->lpwg.sensor ? "FAR" : "NEAR",
			ts->lpwg.qcover ? "CLOSE" : "OPEN");
		novatek_lpwg_mode(dev);
		break;
	case LPWG_REPLY:
		break;

	default:
		TOUCH_I("LPWG UNKNOWN CMD 0x%02x\n", code);
		break;
	}

	return 0;
}


static int nt11206_notify(struct device *dev, ulong event, void *data)
{
	int ret = 0;

	TOUCH_TRACE();

	switch (event) {
	case NOTIFY_CONNECTION:

		break;
	}

	return ret;
}
static int nt11206_register_sysfs(struct device *dev)
{
	int ret = 0;

	TOUCH_TRACE();

	ret = nt11206_touch_register_sysfs(dev);
	if(ret < 0) {
		TOUCH_E("failed to create sysfs\n");
		return ret;
	}

	return 0;
}
static int nt11206_set(struct device *dev, u32 cmd, void *input, void *output)
{
	TOUCH_TRACE();

	return 0;
}

static int nt11206_get(struct device *dev, u32 cmd, void *input, void *output)
{
	int ret = 0;

	TOUCH_D(BASE_INFO, "%s : cmd %d\n", __func__, cmd);

	switch (cmd) {
	case CMD_VERSION:
		ret = novatek_get_cmd_version(dev, (char *)output);
		break;
	case CMD_ATCMD_VERSION:
		ret = novatek_get_cmd_atcmd_version(dev, (char *)output);
		break;
	default:
		break;
	}

	return  ret;
}

static int nt11206_irq_abs(struct device *dev, u8* buffer)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct nt11206_data *d = to_nt11206_data(dev);
	struct tp_event event;
	struct touch_data *pFingerData;
	int ret = 0;
	int i = 0;
	u8 finger_index = 0;
	static u16 pre_pressure = 0;

	ts->new_mask = 0;

	for (i = 0; i < MAX_FINGER_NUM; i++) {
		memset(&event, 0, sizeof(event));
		parser_finger_events(&buffer[i * FINGER_EVENT_LEN], &event);

		if(event.status == 0x07)
			event.status = FINGER_UP;

		if ((event.status == FINGER_NONE) || (event.id > MAX_FINGER_NUM))
			continue;

		/* ignore the event already up. */
		if ((event.status == FINGER_UP) && (d->fingers[i] == FINGER_UP))
			continue;

		if(event.status == FINGER_DOWN
				|| event.status == FINGER_MOVE) {
			ts->new_mask |= (1 << event.id);
			pFingerData = ts->tdata + event.id;
			pFingerData->type = 0;
			pFingerData->id = event.id;
			pFingerData->x = event.x;
			pFingerData->y = event.y;
			if(!event.pressure) {
				TOUCH_E("z-value is zero!!!\n");
				event.pressure = 1;
			}
			if(event.pressure == 255) {
				ts->is_cancel = true;
			}
			if((pre_pressure == event.pressure) && (event.pressure != 255)) {
				event.pressure += 1;
			}
			pre_pressure = event.pressure;
			pFingerData->pressure = event.pressure;
			pFingerData->width_major = 15;
			pFingerData->width_minor = 10;
			finger_index++;

			TOUCH_D(ABS,
				"tdata [id:%d t:%d x:%d y:%d z:%d-%d,%d,%d]\n",
					pFingerData->id,
					pFingerData->type,
					pFingerData->x,
					pFingerData->y,
					pFingerData->pressure,
					pFingerData->width_major,
					pFingerData->width_minor,
					pFingerData->orientation);
		}
		d->fingers[i] = event.status;
	}
	ts->intr_status = TOUCH_IRQ_FINGER;
	ts->tcount = finger_index;

	return ret;
}

static int nt11206_irq_handler(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct nt11206_data *d = to_nt11206_data(dev);
	u8 serial_data[2] = {0x01, 0x47};
	u8 buffer[MAX_FINGER_NUM * FINGER_EVENT_LEN] = {0,};
	int ret = -1;
	u8 read_id;
	static int wrong_int_cnt = 0;
	u8 mode_check = 0;

	//Disable IRQ by using disable_irq_nosync
	nt11206_interrupt_control(dev, INTERRUPT_DISABLE);

	if((ts->lpwg.mode == LPWG_DOUBLE_TAP || ts->lpwg.mode == LPWG_PASSWORD) &&
			(d->mode_state == GESTURE_MODE)) {
		if(atomic_read(&ts->state.pm) != DEV_PM_SUSPEND_IRQ) {
			TOUCH_I("Need wake lock for avoiding I2C Fail\n");
			wake_lock_timeout(&d->lpwg_wake_lock, msecs_to_jiffies(1000));
		}
		ret = novatek_lpwg_read(dev, buffer);
		if(buffer[0]) {
			TOUCH_I("TCI_1\n");
			ret = nt11206_get_position(dev, ts->tci.info[TCI_1].tap_count, buffer);
			ts->intr_status = TOUCH_IRQ_KNOCK;
		}
		else if (buffer[1]) {
			TOUCH_I("TCI_2\n");
			ret = nt11206_get_position(dev, ts->tci.info[TCI_2].tap_count, buffer);
			ts->intr_status = TOUCH_IRQ_PASSWD;
		}
		else if(!buffer[0] && !buffer[1]) {
			if(d->tci_debug) {
				ret = novatek_lpwg_fail_reason(dev, ts->lpwg.mode);
			}
			else {
				if(wrong_int_cnt++ > 10) {
					TOUCH_E("Occure wrong INT\n");
					ret = nt11206_reg_read(dev, MODE_CHANGE_CEHCK, &mode_check, 1);
					if (ret < 0) {
						TOUCH_E("%s() gesture_data I2C read Fail\n", __func__);
						goto to_touch_enable_irq;
					}
					TOUCH_E("mode_check:%d\n", mode_check);
					if(!mode_check) {
						ret = nt11206_sw_reset(dev, RESET_MCU_BOOT);
						msleep(200);
						TOUCH_I("RESET LPWG MODE\n");
						ret = novatek_lpwg_mode(dev);
						wrong_int_cnt = 0;
					}
				}
			}
		}
	}
	else {
		// CMD(0xFF), ADDR_H(0x01), ADDR_L(0x47) OFFSET(0x00) (0x014700)
		ret = nt11206_reg_write(dev, ADDR_CMD, serial_data, 2);
		if(ret < 0) {
			TOUCH_E("%s() irq_handler serial data I2C write Fail\n", __func__);
			goto to_touch_enable_irq;
		}
		// Init Buffer, Read COOR ADDR
		memset(buffer, 0, MAX_FINGER_NUM * FINGER_EVENT_LEN);
		//SET OFFSET
		ret = nt11206_reg_read(dev, READ_COOR_ADDR, buffer, MAX_FINGER_NUM * FINGER_EVENT_LEN);
		if(ret < 0) {
			TOUCH_E("Fail Get COOR ADDR Data\n");
			goto to_touch_enable_irq;
		}
		//TOUCH_ID
		read_id = buffer[0] >> 3;
		if(((read_id > 0) && (read_id < 11)) || (read_id == 0x1F)) {
			ret = nt11206_irq_abs(dev, &buffer[0]);
			if(ret < 0) {
				TOUCH_E("Fail Get nt11206_irq_abs\n");
				goto to_touch_enable_irq;
			}
		}
		else {
			ret = -1;
			goto to_touch_enable_irq;
		}
	}

to_touch_enable_irq:
	//Enable IRQ
	nt11206_interrupt_control(dev, INTERRUPT_ENABLE);
	return ret;
}


static int nt11206_init(struct device *dev)
{
	int ret = 0;
	struct nt11206_data *d = to_nt11206_data(dev);
	
	if(d->mode_state == FW_UPGRADE_MODE)
		return 0;

	TOUCH_TRACE();

	ret = novatek_ts_chipid(dev);
	if(0 > ret) {
		TOUCH_E("Read Chip ID Fail\n");
		return ret;
	}
	init_pass = true;
	return ret;
}
/*******************************************************
  Create Device Node (Proc Entry)
*******************************************************/
#if NVT_TOUCH_CTRL_DRIVER
static struct proc_dir_entry *NVT_proc_entry;
#define PROC_DEVICE_NAME	"NVTflash"
static ssize_t nvt_flash_read(struct file *file, char __user *buff, size_t count, loff_t *offp)
{
	struct nvt_flash_data *dev;
	struct i2c_client *client = to_i2c_client(gts->dev);
	struct i2c_msg msgs[2];
	char *str = NULL;
	char i2c_wr = 0;
	int ret = 0;
	int retries = 0;

	if(file == NULL) {
		return -EFAULT;
	}
	dev = file->private_data;

	if(buff == NULL) {
		return -EFAULT;
	}

	str = (char*) kmalloc(sizeof(char) * 4096, GFP_KERNEL);
	if(str == NULL) {
		return -EFAULT;
	}
	dev->str = str;

	if(copy_from_user(str, buff, count)) {
		if(str) {
			kfree(str);
		}
		dev->str = NULL;
		return -EFAULT;
	}
	i2c_wr = str[0] >> 7;
	if(!str[0] && !str[1]) {
		if(str) {
			kfree(str);
		}
		dev->str = NULL;
		return 0;
	}
	if(i2c_wr == 0) {
		msgs[0].flags = !I2C_M_RD;
		msgs[0].addr  = str[0];
		msgs[0].len   = str[1];
		msgs[0].buf   = &str[2];

		while(retries < 20)
		{
			ret = i2c_transfer(client->adapter, msgs, 1);
			if(ret == 1)
				break;
			else
				TOUCH_E("%s error, retries=%d\n", __func__, retries);

			retries++;
		}
		if(str) {
			kfree(str);
		}
		dev->str = NULL;
		return ret;
	} else if(i2c_wr == 1) {
		msgs[0].flags = !I2C_M_RD;
		msgs[0].addr  = str[0];
		msgs[0].len   = 1;
		msgs[0].buf   = &str[2];

		msgs[1].flags = I2C_M_RD;
		msgs[1].addr  = str[0];
		msgs[1].len   = str[1]-1;
		msgs[1].buf   = &str[3];

		while(retries < 10)
		{
			ret = i2c_transfer(client->adapter, msgs, 2);
			if(ret == 2)
				break;
			else
				TOUCH_E("%s error, retries=%d\n", __func__, retries);

			retries++;
		}

		// copy buff to user if i2c transfer
		if(retries < 10)
		{
			if(copy_to_user(buff, str, count)) {
				if(str) {
					kfree(str);
				}
				dev->str = NULL;
				return -EFAULT;
			}
		}
	}
	else {
		TOUCH_E("Call error, str[0]=%d\n", str[0]);
		if(str) {
			kfree(str);
		}
		dev->str = NULL;
		return -EFAULT;
	}
	if(str) {
		kfree(str);
	}
	dev->str = NULL;
	return ret;
}
static int nvt_flash_open(struct inode *inode, struct file *file)
{
	struct nvt_flash_data *dev;

	dev = kmalloc(sizeof(struct nvt_flash_data), GFP_KERNEL);
	if(dev == NULL)
		return -ENOMEM;

	rwlock_init(&dev->lock);
	file->private_data = dev;

	return 0;
}

static int nvt_flash_close(struct inode *inode, struct file *file)
{
	struct nvt_flash_data *dev = file->private_data;

	if(dev)
		kfree(dev);

	return 0;
}

static const struct file_operations nvt_flash_fops = {
	.owner = THIS_MODULE,
	.open = nvt_flash_open,
	.release = nvt_flash_close,
	.read = nvt_flash_read,
};


static int nvt_flash_proc_init(void)
{
	int ret = 0;

	NVT_proc_entry = proc_create(PROC_DEVICE_NAME, 0444, NULL, &nvt_flash_fops);
	if(NVT_proc_entry == NULL) {
		printk("%s: couldn't create proc entry!\n", __func__);
		ret = -ENOMEM;
		return ret;
	}
	else {
		TOUCH_I("%s: create proc entry success!\n", __func__);
	}
	TOUCH_I("============================================================\n");
	TOUCH_I("Create /proc/NVTflash\n");
	TOUCH_I("============================================================\n");
	return 0;
}
#endif
static int nt11206_probe(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct nt11206_data *d = NULL;
	int ret = 0;

	TOUCH_TRACE();

	d = devm_kzalloc(dev, sizeof(*d), GFP_KERNEL);
	if (!d) {
		TOUCH_E("failed to allocate novatek data\n");
		return -ENOMEM;
	}

	d->dev = dev;

	// finger status init to FINGER_UP.
	memset(d->fingers, FINGER_UP, MAX_FINGER_NUM);
	touch_set_device(ts, d);

	touch_gpio_init(ts->reset_pin, "touch_reset");
	touch_gpio_direction_output(ts->reset_pin, 0);

	touch_gpio_init(ts->int_pin, "touch_int");
	touch_gpio_direction_input(ts->int_pin);

	touch_power_init(dev);
	touch_bus_init(dev, 4096);

	if (touch_boot_mode() == TOUCH_CHARGER_MODE) {
		touch_gpio_init(ts->reset_pin, "touch_reset");
		touch_gpio_direction_output(ts->reset_pin, 0);
		/* Power Off*/
		ret = nt11206_power(dev, POWER_OFF);
		if(ret < 0) {
			TOUCH_E("TOUCH IC Power Off Fail\n");
		}
		return 0;
	}

	d->resume_state = 0;
	nt11206_get_tci_info(dev);
	probe_complete = true;

#if NVT_TOUCH_CTRL_DRIVER
	gts = ts;
	ret = nvt_flash_proc_init();
	if (ret != 0)
	{
		TOUCH_E("nvt_flash_proc_init failed. ret=%d\n", ret);
	}
#endif
#if NVT_TOUCH_EXT_PROC
	g_dev = dev;
	ret = nvt_extra_proc_init();
	if (ret != 0)
	{
		TOUCH_E("nvt_extra_proc_init failed. ret=%d\n", ret);
	}
#endif
	d->boot_mode = touch_boot_mode_check(dev);
	nvt_init_locks(dev);
	return 0;
}

static int nt11206_remove(struct device *dev)
{
	TOUCH_TRACE();

	return 0;
}

static int nt11206_suspend(struct device *dev)
{
	int ret = 0;
	// CMD(0xFF), ADDR_H(0x01), ADDR_L(0x47) (0x014700)
	u8 serial_data[2] = {0x01, 0x47};
	// OFFSET(0x50) + DEEP SLEEP (0x12)
	u8 deep_sleep_data = 0;
	u8 power_off_data = 0;
	u8 gesture_data[3] = {0, };
	struct touch_core_data *ts = to_touch_core(dev);
	struct nt11206_data *d = to_nt11206_data(dev);

	TOUCH_TRACE();

	deep_sleep_data = 0x12;
	power_off_data = 0x11;
	gesture_data[0] = 0x13;
	gesture_data[1] = 0xFF;
	gesture_data[2] = 0xFF;
	if(d->mode_state == FW_UPGRADE_MODE) {
		TOUCH_I("FW Upgrade is working\n");
		return 0;
	}

	if (touch_boot_mode() == TOUCH_CHARGER_MODE)
		return -EPERM;

	TOUCH_I("mfts_lpwg:%d, boot_mode:%d\n", ts->role.mfts_lpwg, d->boot_mode);
	if(init_pass && d->resume_state) {
		d->resume_state = 0;
		memset(d->fingers, FINGER_UP, MAX_FINGER_NUM);
		if(d->boot_mode && !ts->role.mfts_lpwg) {
			ret = nt11206_reg_read_dummy(dev, SLAVE_ADDR);
			ret = nt11206_reg_write(dev, ADDR_CMD, serial_data, 2);
			if (ret < 0) {
				TOUCH_E(" ADDR_CMD serial data I2C write Fail\n");
				return ret;
			}
			ret = nt11206_mode_change_check(dev, &power_off_data, 1);
			if(!ret) {
				nt11206_power(dev, POWER_OFF);
				nt11206_interrupt_control(dev, INTERRUPT_DISABLE);
				d->mode_state = POWER_OFF_MODE;
			}
			else {
				TOUCH_E("Hand shake FAIL\n");
				nt11206_power(dev, POWER_OFF);
				nt11206_interrupt_control(dev, INTERRUPT_DISABLE);
				d->mode_state = POWER_OFF_MODE;
			}
			return -EPERM;
		}
		else if(ts->role.mfts_lpwg && d->boot_mode) {
			ret = novatek_lpwg_mode(dev);
			d->mode_state = GESTURE_MODE;
			return -EPERM;
		}

		TOUCH_I("START mode_state:%d\n", d->mode_state);
		if(d->mode_state == NORMAL_MODE) {
			ret = nt11206_reg_read_dummy(dev, SLAVE_ADDR);
			ret = nt11206_reg_write(dev, ADDR_CMD, serial_data, 2);
			if (ret < 0) {
				TOUCH_E("ADDR_CMD serial data I2C write Fail\n");
				return ret;
			}
			if(ts->lpwg.mode == LPWG_NONE) {
				ret = nt11206_mode_change_check(dev, &power_off_data, 1);
				if(!ret) {
					nt11206_power(dev, POWER_OFF);
					nt11206_interrupt_control(dev, INTERRUPT_DISABLE);
					d->mode_state = POWER_OFF_MODE;
				}
				else {
					TOUCH_E("Hand shake FAIL\n");
					nt11206_power(dev, POWER_OFF);
					nt11206_interrupt_control(dev, INTERRUPT_DISABLE);
					d->mode_state = POWER_OFF_MODE;
				}
			}
			else {
				ret = novatek_lpwg_mode(dev);
				d->mode_state = GESTURE_MODE;
			}
		}
		d->resume_state = 0;
		// finger status init to FINGER_UP.
		memset(d->fingers, FINGER_UP, MAX_FINGER_NUM);
	}

	return ret;
}

static int nt11206_resume(struct device *dev)
{
	int ret = 1;
	static u8 first_resume = 0;
	struct nt11206_data *d = to_nt11206_data(dev);
	struct touch_core_data *ts = to_touch_core(dev);

	TOUCH_TRACE();

	if(d->mode_state == FW_UPGRADE_MODE) {
		TOUCH_I("FW Upgrade is working\n");
		return 0;
	}

	if (touch_boot_mode() == TOUCH_CHARGER_MODE)
		return -EPERM;

	if(init_pass && !d->resume_state) {
		TOUCH_I("START mode_state:%d\n", d->mode_state);
		if(d->mode_state == GESTURE_MODE || d->mode_state == POWER_OFF_MODE) {
			if(d->mode_state == POWER_OFF_MODE) {
				nt11206_power(dev, POWER_ON);
				nt11206_interrupt_control(dev, INTERRUPT_ENABLE);
			}
			else {
				ret = novatek_lpwg_fail_reason(dev, ts->lpwg.mode);
			}
			ret = nt11206_sw_reset(dev, RESET_MCU_BOOT);
			d->mode_state = NORMAL_MODE;
		}
		d->resume_state = 1;
	}

	if(unlikely(!first_resume)) {
		ret = novatek_lpwg_fail_init(dev);
		nt11206_interrupt_control(dev, INTERRUPT_ENABLE);
		first_resume = 1;
	}
	return ret;
}

static struct touch_driver touch_driver = {
	.probe = nt11206_probe,
	.remove = nt11206_remove,
	.suspend = nt11206_suspend,
	.resume = nt11206_resume,
	.init = nt11206_init,
	.irq_handler = nt11206_irq_handler,
	.power = nt11206_power,
	.upgrade = nt11206_upgrade,
	.lpwg = nt11206_lpwg,
	.notify = nt11206_notify,
	.register_sysfs = nt11206_register_sysfs,
	.set = nt11206_set,
	.get = nt11206_get,
};

#define MATCH_NAME			"lge,nt11206"

static struct of_device_id touch_match_ids[] = {
	{ .compatible = MATCH_NAME, },
};

static struct touch_hwif hwif = {
	.bus_type = HWIF_I2C,
	.name = LGE_TOUCH_NAME,
	.owner = THIS_MODULE,
	.of_match_table = of_match_ptr(touch_match_ids),
};

static int __init touch_device_init(void)
{
	TOUCH_TRACE();
	return touch_bus_device_init(&hwif, &touch_driver);
}

static void __exit touch_device_exit(void)
{
	TOUCH_TRACE();
	touch_bus_device_exit(&hwif);
}

module_init(touch_device_init);
module_exit(touch_device_exit);

MODULE_AUTHOR("jaekyung83.lee@lge.com");
MODULE_DESCRIPTION("LGE Novatek touch driver");
MODULE_LICENSE("GPL");
