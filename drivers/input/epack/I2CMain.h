#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/of_gpio.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/input/epack_core.h>

#define SEND_G_PACKNO	1
#define RCV_G_PACKNO	2
#define VAILD_CHECK_BYTE_8	8

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C (Master) Callback Function									                                   */
/*---------------------------------------------------------------------------------------------------------*/
int I2C_MasterSendData(struct i2c_client *client);
int I2C_MasterRcvData(struct i2c_client *client);
bool SendData(struct i2c_client *client);
bool RcvData(struct i2c_client *client);

bool epack_i2c_read_reg(struct epack_dev_data *epack,unsigned long cmdData,void *value,int length);
