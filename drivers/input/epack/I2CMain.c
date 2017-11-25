#include "cmd.h"
#include "I2CMain.h"

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/of_gpio.h>
#include <linux/interrupt.h>
#include <linux/delay.h>

#define HEXA 0x100

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C (Master) Callback Function									                                   */
/*---------------------------------------------------------------------------------------------------------*/


int I2C_MasterSendData(struct i2c_client *client)
{
	int result;
	struct i2c_msg msgs = {

	.addr = client->addr,
	.flags = client->flags,
	.len = 64,
	.buf = (u8 *)sendbuf,
	};

	epack_log("%s sendbuf : \n", __func__);
 	print_array(sendbuf,PACKET_SIZE);

	result = i2c_transfer(client->adapter, &msgs, 1);

	return result;
}

int I2C_MasterRcvData(struct i2c_client *client)
{
	int result;

	struct i2c_msg msgs = {

	.addr = client->addr,
	.flags = client->flags | I2C_M_RD,
	.len = 64,
	.buf = rcvbuf,
	};

	result = i2c_transfer(client->adapter, &msgs, 1);

	epack_log("%s rcvbuf : \n", __func__);
 	print_array(rcvbuf,PACKET_SIZE);

	return result;
}


bool send_data(struct i2c_client *client)
{
	int result;

	gcksum = check_sum(sendbuf, PACKET_SIZE);

	result = I2C_MasterSendData(client);

	if(result <= 0)
	{
		epack_log("%s  result= %d exit.", __func__, result);
		return 0;
	}

	return result;
}

bool RcvData(struct i2c_client *client)
{
	int result;
	unsigned short lcksum;
	int rcv_g_packno;
	u8 *buf;

	msleep(50);//50ms

	result = I2C_MasterRcvData(client);

	if(result <= 0)
	{
		epack_log("%s  result= %d exit.",__func__,  result);
		return 0;
	}

	buf = rcvbuf;
	memcpy(&lcksum, buf, 2);
	buf += 4;

	rcv_g_packno = buf[0]+ (HEXA)*buf[1] + (HEXA)*(HEXA)*buf[2] + (HEXA)*(HEXA)*(HEXA)*buf[3];
	epack_log("%s   buf[0]: %x buf[1]: %x buf[2]: %x buf[3]: %x\n", __func__,buf[0],buf[1],buf[2],buf[3]);

	//if((unsigned short)(buf[0]) != g_packno)
	if(rcv_g_packno != g_packno)
	{
		epack_log("%s g_packno=%d rcv %d\n", __func__, g_packno, rcv_g_packno );
		result = 0;
	}
	else
	{
		if(lcksum != gcksum)
		{
			epack_log("%s gcksum=%x lcksum=%x\n", __func__, gcksum, lcksum);
			result = 0;
		}
		g_packno++;
	}
	return result;
}

int epack_i2c_write(struct i2c_client *client,uint8_t* sendbuf)
{
	struct i2c_msg msgs = {
	.addr = client->addr,
	.flags = client->flags,
	.len = 64,
	.buf = sendbuf,
	};

	return i2c_transfer(client->adapter, &msgs, 1);
}

int epack_i2c_read(struct i2c_client *client,uint8_t* rcvbuf)
{
	struct i2c_msg msgs = {
	.addr = client->addr,
	.flags = client->flags | I2C_M_RD,
	.len = 64,
	.buf = rcvbuf,
	};

	return i2c_transfer(client->adapter, &msgs, 1);
}

void epack_create_i2c_packet(unsigned long cmdData,int8_t *sendbuf){
	unsigned long cmd = cmdData;
	unsigned int g_packno = SEND_G_PACKNO;

	memset(sendbuf,0, PACKET_SIZE);
	memcpy(sendbuf+0, &cmd, 4);
	memcpy(sendbuf+4, &g_packno, 4);
}

/*  epack_i2c_read_reg
    - this is the abstract verion of i2c comm. function
	  ,not using global variable, protect the narrow region with mutex
    1. send @cmdData to epack
    2. receive data from epack and store it to @value (size: @length)
    3. to validate comm., check checksum and g_packno
*/

bool epack_i2c_read_reg(struct epack_dev_data *epack,unsigned long cmdData,void *value,int length)
{
	struct i2c_client *client = epack->client;
	int result;
	u8 sendbuf[PACKET_SIZE];
	u8 rcvbuf[PACKET_SIZE];
	unsigned short gcksum,lcksum;
	int g_packno;

	epack_create_i2c_packet(cmdData,sendbuf);

	mutex_lock(&epack->i2c_lock);
	result = epack_i2c_write(client,sendbuf);
	if(result <= 0) {
		epack_log("%s Failed to write I2C for CMD %lu \n", __func__,cmdData);
		mutex_unlock(&epack->i2c_lock);
		return 0;
	}

	gcksum = check_sum(sendbuf, PACKET_SIZE);
	msleep(50);

	result = epack_i2c_read(client,rcvbuf);
	if(result <= 0) {
		epack_log("%s Failed to read I2C for CMD %lu \n", __func__,cmdData);
		mutex_unlock(&epack->i2c_lock);
		return 0;
	}
	mutex_unlock(&epack->i2c_lock);

	g_packno = rcvbuf[4]+ (HEXA)*rcvbuf[5] + (HEXA)*(HEXA)*rcvbuf[6] + (HEXA)*(HEXA)*(HEXA)*rcvbuf[7];
	if(g_packno != RCV_G_PACKNO) {
		epack_log("%s rcv_g_packno=%d\n", __func__, g_packno);
		return 0;
	}

	memcpy(&lcksum, rcvbuf, 2);
	if(lcksum != gcksum) {
		epack_log("%s gcksum=%x lcksum=%x\n", __func__, gcksum, lcksum);
		return 0;
	}

	memcpy(value,rcvbuf+VAILD_CHECK_BYTE_8,length);
	return result;
}
