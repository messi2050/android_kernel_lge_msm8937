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
 *    File  : lgtp_platform_api.c
 *    Author(s)   : D3 BSP Touch Team < d3-bsp-touch@lge.com >
 *    Description :
 *
 ***************************************************************************/
#define LGTP_MODULE "[PLATFORM]"

/****************************************************************************
* Include Files
****************************************************************************/
#include <linux/input/unified_driver_3/lgtp_common.h>

#include <linux/input/unified_driver_3/lgtp_common_driver.h>
#include <linux/input/unified_driver_3/lgtp_platform_api.h>
#include <linux/input/unified_driver_3/lgtp_model_config.h>


/****************************************************************************
* Manifest Constants / Defines
****************************************************************************/

/****************************************************************************
 * Macros
 ****************************************************************************/

/****************************************************************************
* Type Definitions
****************************************************************************/
#if defined(TOUCH_PLATFORM_MTK)
#define MAX_I2C_TRANSFER_SIZE 255
#endif

/****************************************************************************
* Variables
****************************************************************************/
#if defined(TOUCH_PLATFORM_MTK)
static u8 *I2CDMABuf_va;
static u32 I2CDMABuf_pa;
#endif

#if defined(TOUCH_PLATFORM_QCT)
static int nIrq_num = 0;
atomic_t	touch_irq_mask;
#endif

/****************************************************************************
* Extern Function Prototypes
****************************************************************************/


/****************************************************************************
* Local Function Prototypes
****************************************************************************/


/****************************************************************************
* Local Functions
****************************************************************************/
#if 0 /* TBD */
#if defined(TOUCH_PLATFORM_QCT)
#define istate core_internal_state__do_not_mess_with_it
#define IRQS_PENDING 0x00000200
#endif
static void TouchClearPendingIrq(void)
{
	#if defined(TOUCH_PLATFORM_QCT)
	unsigned long flags;
	struct irq_desc *desc = irq_to_desc(nIrq_num);

	if (desc) {
		raw_spin_lock_irqsave(&desc->lock, flags);
		desc->istate &= ~IRQS_PENDING;
		raw_spin_unlock_irqrestore(&desc->lock, flags);
	}
	#endif
}
#endif

static void TouchMaskIrq(void)
{
	#if defined(TOUCH_PLATFORM_QCT)
	struct irq_desc *desc = irq_to_desc(nIrq_num);

	if (desc->irq_data.chip->irq_mask)
		desc->irq_data.chip->irq_mask(&desc->irq_data);
	#endif
}

static void TouchUnMaskIrq(void)
{
	#if defined(TOUCH_PLATFORM_QCT)
	struct irq_desc *desc = irq_to_desc(nIrq_num);

	if (desc->irq_data.chip->irq_unmask)
		desc->irq_data.chip->irq_unmask(&desc->irq_data);
	#endif
}

#if defined(TOUCH_PLATFORM_MTK)
static int dma_allocation(void)
{
	I2CDMABuf_va = (u8 *)dma_alloc_coherent(NULL, 4096, &I2CDMABuf_pa, GFP_KERNEL);
	if (I2CDMABuf_va == NULL) {
		TOUCH_ERR("fail to allocate DMA\n");
		return TOUCH_FAIL;
	}

	return TOUCH_SUCCESS;
}

static int i2c_dma_write(struct i2c_client *client, const uint8_t *buf, int len)
{
	int i = 0;

	for (i = 0; i < len; i++)
		I2CDMABuf_va[i] = buf[i];

	if (len < 8) {
		client->addr = client->addr & I2C_MASK_FLAG | I2C_ENEXT_FLAG;
		client->timing = 400;
		return i2c_master_send(client, buf, len);
	} else {
		client->addr = client->addr & I2C_MASK_FLAG | I2C_DMA_FLAG | I2C_ENEXT_FLAG;
		client->timing = 400;
		return i2c_master_send(client, I2CDMABuf_pa, len);
	}
}

static int i2c_dma_read(struct i2c_client *client, uint8_t *buf, int len)
{
	int i = 0;
	int ret = 0;

	if (len < 8) {
		client->addr = client->addr & I2C_MASK_FLAG | I2C_ENEXT_FLAG;
		client->timing = 400;
		return i2c_master_recv(client, buf, len);
	} else {
		client->addr = client->addr & I2C_MASK_FLAG | I2C_DMA_FLAG | I2C_ENEXT_FLAG;
		client->timing = 400;
		ret = i2c_master_recv(client, I2CDMABuf_pa, len);
		if (ret < 0)
			return ret;

		for (i = 0; i < len; i++)
			buf[i] = I2CDMABuf_va[i];
	}

	return ret;
}

static int i2c_msg_transfer(struct i2c_client *client, struct i2c_msg *msgs, int count)
{
	int i = 0;
	int ret = 0;

	for (i = 0; i < count; i++) {
		if (msgs[i].flags & I2C_M_RD)
			ret = i2c_dma_read(client, msgs[i].buf, msgs[i].len);
		else
			ret = i2c_dma_write(client, msgs[i].buf, msgs[i].len);

		if (ret < 0)
			return ret;
	}

	return 0;
}
#endif

static int i2c_read(struct i2c_client *client, u8 *reg, int regLen, u8 *buf, int dataLen)
{
#if defined(TOUCH_PLATFORM_QCT)

	int result = TOUCH_SUCCESS;
	int ret = 0;

	struct i2c_msg msgs[2] = {
		{ .addr = client->addr, .flags = 0, .len = regLen, .buf = reg, },
		{ .addr = client->addr, .flags = I2C_M_RD, .len = dataLen, .buf = buf, },
	};

	ret = i2c_transfer(client->adapter, msgs, 2);
	if (ret < 0)
		result = TOUCH_FAIL;

	return result;

#elif defined(TOUCH_PLATFORM_MTK)

	if (dataLen <= MAX_I2C_TRANSFER_SIZE) {
		int result = TOUCH_SUCCESS;
		int ret = 0;

		struct i2c_msg msgs[2] = {
			{ .addr = client->addr, .flags = 0, .len = regLen, .buf = reg, },
			{ .addr = client->addr, .flags = I2C_M_RD, .len = dataLen, .buf = buf, },
		};

		ret = i2c_msg_transfer(client, msgs, 2);
		if (ret < 0)
			result = TOUCH_FAIL;

		return result;
	} else {
		int result = TOUCH_SUCCESS;
		int ret = 0;
		int i = 0;

		int msgCount = 0;
		int remainedDataLen = 0;

		struct i2c_msg *msgs = NULL;

		remainedDataLen = dataLen%MAX_I2C_TRANSFER_SIZE;

		msgCount = 1; /* msg for register */
		msgCount += (int)(dataLen/MAX_I2C_TRANSFER_SIZE); /* add msgs for data read */
		if (remainedDataLen > 0)
			msgCount += 1; /* add msg for remained data */

		msgs = (struct i2c_msg *)kcalloc(msgCount, sizeof(struct i2c_msg), GFP_KERNEL);
		if (msgs != NULL)
			memset(msgs, 0x00, sizeof(struct i2c_msg));
		else
			return TOUCH_FAIL;

		msgs[0].addr = client->addr;
		msgs[0].flags = 0;
		msgs[0].len = regLen;
		msgs[0].buf = reg;

		for (i = 1; i < msgCount; i++) {
			msgs[i].addr = client->addr;
			msgs[i].flags = I2C_M_RD;
			msgs[i].len = MAX_I2C_TRANSFER_SIZE;
			msgs[i].buf = buf + MAX_I2C_TRANSFER_SIZE * (i-1);
		}

		if (remainedDataLen > 0)
			msgs[msgCount-1].len = remainedDataLen;

		ret = i2c_msg_transfer(client, msgs, msgCount);
		if (ret < 0)
			result = TOUCH_FAIL;

		kfree(msgs);

		return result;
	}

#else
	#error "Platform should be defined"
#endif

}

static int i2c_write(struct i2c_client *client, u8 *reg, int regLen, u8 *buf, int dataLen)
{
	int result = TOUCH_SUCCESS;
	int ret = 0;

	struct i2c_msg msg = {
		.addr = client->addr, .flags = 0, .len = (regLen+dataLen), .buf = NULL,
	};

	#if defined(TOUCH_PLATFORM_MTK)
	if (dataLen > MAX_I2C_TRANSFER_SIZE) {
		TOUCH_ERR("data length to write is exceed the limit ( length = %d, limit = %d )\n", dataLen,
				MAX_I2C_TRANSFER_SIZE);
		TOUCH_ERR("You should implement to overcome this problem like read\n");
		return TOUCH_FAIL;
	}
	#endif

	u8 *pTmpBuf = NULL;

	pTmpBuf = (u8 *)kcalloc(1, regLen+dataLen, GFP_KERNEL);
	if (pTmpBuf != NULL)
		memset(pTmpBuf, 0x00, regLen+dataLen);
	else
		return TOUCH_FAIL;

	memcpy(pTmpBuf, reg, regLen);
	memcpy((pTmpBuf+regLen), buf, dataLen);

	msg.buf = pTmpBuf;

	#if defined(TOUCH_PLATFORM_QCT)
	ret = i2c_transfer(client->adapter, &msg, 1);
	#elif defined(TOUCH_PLATFORM_MTK)
	ret = i2c_msg_transfer(client, &msg, 1);
	#else
	#error "Platform should be defined"
	#endif

	if (ret < 0)
		result = TOUCH_FAIL;

	kfree(pTmpBuf);

	return result;
}

/****************************************************************************
* Global Functions
****************************************************************************/
#if defined(TOUCH_DEVICE_LU201X) || defined(TOUCH_DEVICE_LU202X)
int Lu20xx_I2C_Read(struct i2c_client *client, u16 addr, u8 *rxbuf, int len)
{
	int ret = 0;
	u8 regValue[2] = {((addr>>8) & 0xFF), (addr & 0xFF)};

	ret = i2c_read(client, regValue, 2, rxbuf, len);
	if (ret == TOUCH_FAIL) {
		if (printk_ratelimit())
			TOUCH_ERR("failed to read i2c ( reg = %d )\n", addr);

		return TOUCH_FAIL;
	}

	return TOUCH_SUCCESS;
}

int Lu20xx_I2C_Write(struct i2c_client *client, u16 addr, u8 *txbuf, int len)
{
	int ret = 0;
	u8 regValue[2] = {((addr>>8) & 0xFF), (addr & 0xFF)};

	ret = i2c_write(client, regValue, 2, txbuf, len);
	if (ret == TOUCH_FAIL) {
		if (printk_ratelimit())
			TOUCH_ERR("failed to write i2c ( reg = %d )\n", addr);

		return TOUCH_FAIL;
	}

	return TOUCH_SUCCESS;
}
#endif

#if defined(TOUCH_LGD_PHASE2)
int Mit200_I2C_Read(struct i2c_client *client, u8 *cmd,  int cmdLen, u8 *rxbuf, int len)
{
	int ret = 0;

	ret = i2c_read(client, cmd, cmdLen, rxbuf, len);
	if (ret == TOUCH_FAIL) {
		if (printk_ratelimit())
			TOUCH_ERR("failed to read i2c\n");

		return TOUCH_FAIL;
	}

	return TOUCH_SUCCESS;
}

int Mit200_I2C_Write(struct i2c_client *client, u8 *cmd,  int cmdLen, u8 *txbuf, int len)
{
	int ret = 0;

	ret = i2c_write(client, cmd, cmdLen, txbuf, len);
	if (ret == TOUCH_FAIL) {
		if (printk_ratelimit())
			TOUCH_ERR("failed to read i2c\n");

		return TOUCH_FAIL;
	}

	return TOUCH_SUCCESS;
}
#endif

int Touch_I2C_Read_Byte(struct i2c_client *client, u8 addr, u8 *rxbuf)
{
	int ret = 0;
	int i = 0;
	u8 regValue = addr;

	do {
		ret = i2c_read(client, &regValue, 1, rxbuf, 1);
		if (ret == TOUCH_FAIL) {
			if (printk_ratelimit())
				TOUCH_ERR("failed to read i2c byte ( reg = %d )\n", addr);
			
				TOUCH_LOG("i2c retry [%d]\n", i+1);
				msleep(5);
		}else{
			goto out;
		}
	} while (++i<10);
		TOUCH_ERR("i2c transfer failed\n");
	return TOUCH_FAIL;

out:
	return TOUCH_SUCCESS;
}

int Touch_I2C_Write_Byte(struct i2c_client *client, u8 addr, u8 txbuf)
{
	int ret = 0;
	int i = 0;	
	u8 regValue = addr;
	u8 data = txbuf;

	do {
		ret = i2c_write(client, &regValue, 1, &data, 1);
		if (ret == TOUCH_FAIL) {
			if (printk_ratelimit())
				TOUCH_ERR("failed to write i2c byte ( reg = %d )\n", addr);
			
				TOUCH_LOG("i2c retry [%d]\n", i+1);
				msleep(25);
		}else{
			goto out;
		}
	} while (++i<10);
		TOUCH_ERR("i2c transfer failed\n");
	return TOUCH_FAIL;

out:
	return TOUCH_SUCCESS;
}

int Touch_I2C_Read(struct i2c_client *client, u8 addr, u8 *rxbuf, int len)
{
	int ret = 0;
	int i = 0;
	u8 regValue = addr;

	do {
		ret = i2c_read(client, &regValue, 1, rxbuf, len);
		if (ret == TOUCH_FAIL) {
			if (printk_ratelimit())
				TOUCH_ERR("failed to read i2c ( reg = %d )\n", addr);
			
				TOUCH_ERR("i2c retry [%d]\n", i+1);
				msleep(5);
		}else{
			goto out;
		}
	} while (++i<10);
		TOUCH_ERR("i2c transfer failed\n");
	return TOUCH_FAIL;

out:
	return TOUCH_SUCCESS;
}

int Touch_I2C_Write(struct i2c_client *client, u8 addr, u8 *txbuf, int len)
{
	int ret = 0;
	int i = 0;
	u8 regValue = addr;

	do {
		ret = i2c_write(client, &regValue, 1, txbuf, len);
		if (ret == TOUCH_FAIL) {
			if (printk_ratelimit())
				TOUCH_ERR("failed to write i2c ( reg = %d )\n", addr);
			
				TOUCH_ERR("i2c retry [%d]\n", i+1);
				msleep(25);
		}else{
			goto out;
		}
	} while (++i<10);
		TOUCH_ERR("i2c transfer failed\n");
	return TOUCH_FAIL;

out:
	return TOUCH_SUCCESS;
}

#if 1 /* LGE_BSP_COMMON : branden.you@lge.com_20141105 : */
int touch_i2c_read_for_query(int slave_addr, u8 *reg, u8 regLen, u8 *buf, u8 dataLen)
{
	int ret = 0;
	struct i2c_adapter *adap = NULL;

	struct i2c_msg msgs[2] = {
		{ .addr = slave_addr, .flags = 0, .len = regLen, .buf = reg, },
		{ .addr = slave_addr, .flags = I2C_M_RD, .len = dataLen, .buf = buf, },
	};

	adap = i2c_get_adapter(TOUCH_I2C_BUS_NUM);
	if (adap == NULL)
		return TOUCH_FAIL;

	ret = i2c_transfer(adap, msgs, 2);
	if (ret < 0) {
		if (printk_ratelimit())
			TOUCH_ERR("failed to read i2c data ( error = %d )\n", ret);

		return TOUCH_FAIL;
	}

	return TOUCH_SUCCESS;

}
#endif

int TouchInitializePlatform(void)
{
	int ret = 0;

	TOUCH_FUNC();

	#if defined(TOUCH_PLATFORM_MTK)
	{
		ret = dma_allocation();
		if (ret == TOUCH_FAIL)
			return TOUCH_FAIL;
	}
	#endif

	#if defined(TOUCH_PLATFORM_QCT)

	ret = gpio_request(TOUCH_GPIO_RESET, "touch_reset");
	if (ret < 0)
		TOUCH_ERR("FAIL: touch reset gpio_request\n");
	else
		TOUCH_LOG("SUCCESS: touch reset gpio_request [%d] \n", TOUCH_GPIO_RESET);

	gpio_direction_output(TOUCH_GPIO_RESET, 1);

	ret = gpio_request(TOUCH_GPIO_INTERRUPT, "touch_int");
	if (ret < 0)
		TOUCH_ERR("FAIL: touch int gpio_request\n");
	else
		TOUCH_LOG("SUCCESS: touch int gpio_request [%d]\n", TOUCH_GPIO_INTERRUPT);

	gpio_direction_input(TOUCH_GPIO_INTERRUPT);

	#if defined(TOUCH_GPIO_MAKER_ID)
	ret = gpio_request(TOUCH_GPIO_MAKER_ID, "touch_maker");
	if (ret < 0)
		TOUCH_ERR("FAIL: touch make id gpio_request\n");

	gpio_direction_input(TOUCH_GPIO_MAKER_ID);
	#endif

	#elif defined(TOUCH_MODEL_LION_3G)

	#elif defined(TOUCH_PLATFORM_MTK)

	mt_set_gpio_mode(GPIO_TOUCH_RESET, GPIO_TOUCH_RESET_M_GPIO);
	mt_set_gpio_dir(GPIO_TOUCH_RESET, GPIO_DIR_OUT);

	mt_set_gpio_mode(GPIO_TOUCH_INT, GPIO_TOUCH_INT_M_EINT);
	mt_set_gpio_dir(GPIO_TOUCH_INT, GPIO_DIR_IN);

	#else
	#error "Platform should be defined"
	#endif

	return TOUCH_SUCCESS;
}

void TouchPower(int isOn)
{
	/* to avoid prequently change of this file */
	TouchPowerModel(isOn);
}

void TouchAssertReset(void)
{
	/* to avoid prequently change of this file */
	TouchAssertResetModel();
}

void TouchResetCtrl(int isHigh)
{

	if (isHigh) {
		#if defined(TOUCH_PLATFORM_QCT)
		gpio_set_value(TOUCH_GPIO_RESET, 1);
		#elif defined(TOUCH_MODEL_LION_3G)

		#elif defined(TOUCH_PLATFORM_MTK)
		mt_set_gpio_out(GPIO_TOUCH_RESET, GPIO_OUT_ONE);
		#else
		#error "Platform should be defined"
		#endif

		TOUCH_LOG("Reset Pin was set to HIGH\n");
	} else {
		#if defined(TOUCH_PLATFORM_QCT)
		gpio_set_value(TOUCH_GPIO_RESET, 0);
		#elif defined(TOUCH_MODEL_LION_3G)

		#elif defined(TOUCH_PLATFORM_MTK)
		mt_set_gpio_out(GPIO_TOUCH_RESET, GPIO_OUT_ZERO);
		#else
		#error "Platform should be defined"
		#endif

		TOUCH_LOG("Reset Pin was set to LOW\n");
	}

}

int TouchIsIrqRequested(void)
{
	TOUCH_FUNC();
	if (nIrq_num == 0)
		return 0;
	else
		return 1;
}
EXPORT_SYMBOL(TouchIsIrqRequested);

int TouchFreeIrq(void)
{
	TOUCH_FUNC();
	#if defined(TOUCH_PLATFORM_QCT)

	if(nIrq_num != 0 ) {

		free_irq(nIrq_num, NULL);
		nIrq_num = 0;
	}
	
	return TOUCH_SUCCESS;
	#endif 
}
EXPORT_SYMBOL(TouchFreeIrq);

int TouchRegisterIrq(TouchDriverData *pDriverData, irq_handler_t irqHandler, irq_handler_t threaded_irqHandler)
{
	//TOUCH_FUNC();

	#if defined(TOUCH_PLATFORM_QCT)
	{
		int ret = 0;
		if(nIrq_num == 0 ) {
			TOUCH_LOG("Check client->irq  %d\n", pDriverData->client->irq);
			nIrq_num = pDriverData->client->irq;
			ret = request_threaded_irq(pDriverData->client->irq, irqHandler, threaded_irqHandler, TOUCH_IRQ_FLAGS, pDriverData->client->name,pDriverData);
			if (ret < 0) {
				TOUCH_ERR("failed at request_irq() ( error = %d )\n", ret);
				nIrq_num = 0;
				return TOUCH_FAIL;
			}
			enable_irq_wake(pDriverData->client->irq);
			
			nIrq_num = pDriverData->client->irq;
		} else {
			TOUCH_ERR(" Already requested \n");
		}
	}
	#elif defined(TOUCH_PLATFORM_MTK)

	mt_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_TYPE, irqHandler, 1);

	#else
	#error "Platform should be defined"
	#endif

	return TOUCH_SUCCESS;
}

void TouchEnableIrq(void)
{
	//TOUCH_FUNC();

	if(nIrq_num != 0)
		TouchUnMaskIrq();
		
	/* TouchClearPendingIrq(); TBD */

	#if defined(TOUCH_PLATFORM_QCT)

	if (atomic_read(&touch_irq_mask) != 0) {
		atomic_set(&touch_irq_mask, 0);
		if(nIrq_num != 0)
			enable_irq(nIrq_num);
	}
	
	#elif defined(TOUCH_PLATFORM_MTK)

	mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);

	#else
	#error "Platform should be defined"
	#endif

	//TOUCH_LOG("Interrupt Enabled\n");
}

void TouchDisableIrq(void)
{
	//TOUCH_FUNC();
	#if defined(TOUCH_PLATFORM_QCT)

	if (atomic_read(&touch_irq_mask) == 0) {
		atomic_set(&touch_irq_mask, 1);
		if(nIrq_num != 0)
			disable_irq_nosync(nIrq_num);
	}

	#elif defined(TOUCH_PLATFORM_MTK)

	mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);

	#else
	#error "Platform should be defined"
	#endif
	
	if(nIrq_num != 0)
		TouchMaskIrq();
	
	//TOUCH_LOG("Interrupt Disabled\n");
}

int TouchReadMakerId(void)
{
	int pinState = 1; /* default value is "ONE" */

	#if defined(TOUCH_PLATFORM_QCT)

	#if defined(TOUCH_GPIO_MAKER_ID)
	pinState = gpio_get_value(TOUCH_GPIO_MAKER_ID);
	//TOUCH_LOG("TOUCH_MAKER_ID pinState %d\n", pinState);
	#endif

	#elif defined(TOUCH_PLATFORM_MTK)

	#else
	#error "Platform should be defined"
	#endif

	return pinState;
}

int TouchReadInterrupt(void)
{
	int gpioState = 0;

	//TOUCH_FUNC();

	#if defined(TOUCH_PLATFORM_QCT)

	gpioState = gpio_get_value(TOUCH_GPIO_INTERRUPT);

	TOUCH_LOG("Touch Read Interrupt [%d] gpioState %d \n", TOUCH_GPIO_INTERRUPT, gpioState);

	#elif defined(TOUCH_PLATFORM_MTK)

	gpioState = mt_get_gpio_in(GPIO_TOUCH_INT); /* TBD */

	#else
	#error "Platform  should be defined"
	#endif

	return gpioState;
}

int TouchReadGpioReset(void)
{
	int gpioState = 0;

	//TOUCH_FUNC();

	#if defined(TOUCH_PLATFORM_QCT)

	gpioState = gpio_get_value(TOUCH_GPIO_RESET);

	TOUCH_LOG("Touch Read Interrupt [%d] gpioState %d \n", TOUCH_GPIO_RESET, gpioState);

	#elif defined(TOUCH_PLATFORM_MTK)

	gpioState = mt_get_gpio_in(GPIO_TOUCH_RESET); /* TBD */

	#else
	#error "Platform  should be defined"
	#endif

	return gpioState;
}

void TouchGpioSetInterrupt(int value )
{
	//TOUCH_FUNC();

	#if defined(TOUCH_PLATFORM_QCT)

	gpio_direction_output(TOUCH_GPIO_INTERRUPT, value);

	gpio_set_value(TOUCH_GPIO_INTERRUPT, value );
	udelay(100);
	
	gpio_direction_input(TOUCH_GPIO_INTERRUPT);

	TOUCH_LOG("Touch GPIO Set Interrupt [%d] set %d \n", TOUCH_GPIO_INTERRUPT, value);

	#elif defined(TOUCH_PLATFORM_MTK)

	// Please implement

	#else
	#error "Platform  should be defined"
	#endif

	return;
}

void TouchGpioSetReset(int value)
{
	//TOUCH_FUNC();

	#if defined(TOUCH_PLATFORM_QCT)

	gpio_set_value(TOUCH_GPIO_RESET, value );

	udelay(100);

	TOUCH_LOG("Touch GPIO Set Reset [%d] set %d \n", TOUCH_GPIO_RESET, value);

	#elif defined(TOUCH_PLATFORM_MTK)

	// Please implement

	#else
	#error "Platform  should be defined"
	#endif

	return;
}

#if defined(TOUCH_DEVICE_LU201X) || defined(TOUCH_DEVICE_LU202X)
void TouchIntPinToggle(void)
{
	#if defined(TOUCH_PLATFORM_MSM8210) || defined(TOUCH_PLATFORM_MSM8916)

	/* Add interrupt pin change for output mode */
	gpio_direction_output(TOUCH_GPIO_INTERRUPT, 0);
	/* Add here interrupt pin goes to low */
	gpio_set_value(TOUCH_GPIO_INTERRUPT, 0);
	udelay(100);

	/* Add here interrupt pin goes to high */
	gpio_set_value(TOUCH_GPIO_INTERRUPT, 1);
	/* Add here interrupt pin change for input mode */
	gpio_direction_input(TOUCH_GPIO_INTERRUPT);
	mdelay(1);

	#else
	#error "Platform  should be defined"
	#endif

}
#endif

int TouchGetBootMode(void)
{
	int mode = BOOT_NORMAL;

	#if defined(TOUCH_PLATFORM_MSM8916) || defined(TOUCH_PLATFORM_MSM8939) \
		|| defined (TOUCH_PLATFORM_MSM8952) || defined (TOUCH_PLATFORM_MSM8937)
	{

		enum lge_boot_mode_type lge_boot_mode;

		lge_boot_mode = lge_get_boot_mode();

		if (lge_boot_mode == LGE_BOOT_MODE_CHARGERLOGO)
			mode = BOOT_OFF_CHARGING;
		else if (lge_boot_mode == LGE_BOOT_MODE_NORMAL)
			mode = BOOT_NORMAL;
		else if (lge_get_mfts_mode() == 1)
			mode = BOOT_MFTS;
		else
			mode = BOOT_MINIOS;

	}
	#elif defined(TOUCH_PLATFORM_MSM8210)
	{
		enum lge_boot_mode_type lge_boot_mode;

		lge_boot_mode = lge_get_boot_mode();

		if (lge_boot_mode == LGE_BOOT_MODE_CHARGERLOGO)
			mode = BOOT_OFF_CHARGING;
		else
			mode = BOOT_NORMAL;
	}
	#elif defined(TOUCH_PLATFORM_MTK)

	#else
	#error "Platform should be defined"
	#endif

	return mode;
}


#if defined(TOUCH_PLATFORM_MTK) /* TBD */
bool key_lock_status = 0;
void touch_keylock_enable(int key_lock)
{
	TOUCH_FUNC();

	if (!key_lock) {
		TouchEnableIrq();
		key_lock_status = 0;
	} else {
		TouchDisableIrq();
		key_lock_status = 1;
	}
}
EXPORT_SYMBOL(touch_keylock_enable);
#endif



/* End Of File */

