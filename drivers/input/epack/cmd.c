#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/of_gpio.h>
#include <linux/interrupt.h>
#include <linux/file.h>
#include <linux/syscalls.h>	/* for file access */
#include <linux/uaccess.h>	/* for file access */
#include <linux/firmware.h>

#include "I2CMain.h"
#include "cmd.h"

u8 *imageBegin;
u8 *imageEnd;
uint8_t rcvbuf[PACKET_SIZE];
uint8_t sendbuf[PACKET_SIZE];
uint8_t aprom_buf[PAGE_SIZE_EPACK];
uint8_t FileBuffer[FILE_BUFFER];
//uint8_t FileBuffer2[FILE_BUFFER];


unsigned int g_packno = 1;
unsigned short gcksum;

void print_array(u8* buf,int size)
{
		int i=0;
	    for(i=0;i<size;i++)
        pr_err("%x ", buf[i]);
	return;
}

void print_array2(const u8* buf,int size)
{
		int i=0;
	    for(i=0;i<size;i++)
        pr_err("%x ", buf[i]);
	return;
}

int check_sum (unsigned char *buf, int len)
{
    int i;
    int c;

    for (c=0, i=0; i < len; i++) {
        c += buf[i];
    }
    return (c);
}

static int CalCheckSum(u8 *buf, int len)
{
	int i;
	int lcksum = 0;
	u8 aprom_buf[PAGE_SIZE_EPACK];
	memset(aprom_buf,0,PAGE_SIZE_EPACK);

	epack_log("[CMD]%s start \n",__func__);
	for(i = 0; i < len; i+=PAGE_SIZE_EPACK)
	{
		memcpy(aprom_buf, buf + i, PAGE_SIZE_EPACK);
		if(len - i >= PAGE_SIZE_EPACK)
			lcksum += check_sum(aprom_buf, PAGE_SIZE_EPACK);
		else
			lcksum += check_sum(aprom_buf, len - i);
	}

  return lcksum;

}


bool cmd_sync_packno(struct i2c_client *client)
{
	bool result;
	unsigned long cmdData;
	g_packno=1;
	epack_log("[CMD]%s start \n",__func__);
	//sync send&recv packno
	memset(sendbuf, 0, PACKET_SIZE); // 0 reset, PACKET_SIZE=64
	cmdData = CMD_SYNC_PACKNO;//CMD_SYNC_PACKNO
	memcpy(sendbuf+0, &cmdData, 4);
	memcpy(sendbuf+4, &g_packno, 4);
	memcpy(sendbuf+8, &g_packno, 4);
	g_packno++;

	result = send_data(client);
	if(result == 0)
		return result;

	result = RcvData(client);

	return result;
}

bool cmd_hw_version(struct i2c_client *client, int flag, unsigned int *hwver)
{
	bool result;
	unsigned long cmdData;
	unsigned int lhwver;
	g_packno=1;
	epack_log("[CMD]%s start \n",__func__);
	memset(sendbuf, 0, PACKET_SIZE);
	cmdData = CMD_GET_HWVER;
	memcpy(sendbuf+0, &cmdData, 4);
	memcpy(sendbuf+4, &g_packno, 4);
	g_packno++;

	result = send_data(client);
	if(result == 0)
		return result;

	result = RcvData(client);
	if(result > 0)
	{
		memcpy(&lhwver, rcvbuf+8, 4);
		*hwver = lhwver;
	}

	return result;
}



bool cmd_fw_version(struct i2c_client *client, int flag, unsigned int *fwver)
{
	bool result;
	unsigned long cmdData;
	unsigned int lfwver;
	g_packno=1;
	epack_log("[CMD]%s start \n",__func__);
	memset(sendbuf, 0, PACKET_SIZE);
	cmdData = CMD_GET_FWVER;
	memcpy(sendbuf+0, &cmdData, 4);
	memcpy(sendbuf+4, &g_packno, 4);
	g_packno++;

	result = send_data(client);
	if(result == 0)
		return result;

	result = RcvData(client);
	if(result > 0)
	{
		memcpy(&lfwver, rcvbuf+8, 4);
		*fwver = lfwver;
	}

	return result;
}


bool cmd_run_cmd(struct i2c_client *client,unsigned int  cmd, int *data)
{
	bool result;
	unsigned int cmdData;
	g_packno=1;
	epack_log("[CMD]%s start cmd : 0x%x\n",__func__, cmd);
	//sync send&recv packno
	memset(sendbuf, 0, PACKET_SIZE);
	cmdData = cmd;
	memcpy(sendbuf+0, &cmdData, 4);
	memcpy(sendbuf+4, &g_packno, 4);
	if(cmd == CMD_WRITE_CHECKSUM)
	{
		memcpy(sendbuf+8, &data[0], 4);
		memcpy(sendbuf+12, &data[1], 4);
	}
	g_packno++;

	result = send_data(client);
	if(result == 0)
		return result;

	if((cmd == CMD_ERASE_ALL) || (cmd == CMD_GET_FLASHMODE) || (cmd == CMD_WRITE_CHECKSUM))
	{
		if(cmd == CMD_WRITE_CHECKSUM)
			msleep(200);//0.2s
		result = RcvData(client);
		if(result>0)
		{
			if(cmd == CMD_GET_FLASHMODE)
			{
				memcpy(&cmdData, rcvbuf+8, 4);
				*data = cmdData;
			}
		}

	}
	else if((cmd == CMD_RUN_APROM) || (cmd == CMD_RUN_LDROM) || (cmd == CMD_RESET))
		msleep(500);//0.5s

	return result;
}



bool cmd_get_deviceID(struct i2c_client *client, int flag, unsigned int *devid)
{
	bool result;
	unsigned long cmdData;
	unsigned int ldevid;
	g_packno=1;
	epack_log("[CMD]%s start \n",__func__);
	//sync send&recv packno
	memset(sendbuf, 0, PACKET_SIZE);
	cmdData = CMD_GET_DEVICEID;
	memcpy(sendbuf+0, &cmdData, 4);
	memcpy(sendbuf+4, &g_packno, 4);
	g_packno++;

	result = send_data(client);
	if(result == 0)
		return result;

	result = RcvData(client);
	if(result>0)
		{
			memcpy(&ldevid, rcvbuf+8, 4);
			*devid = ldevid;
		}

		return result;
}

bool cmd_get_config(struct i2c_client *client, int flag, unsigned int *config)
{
	bool result;
	unsigned long cmdData;
	unsigned int lconfig[2];
	g_packno=1;
	epack_log("[CMD]%s start \n",__func__);
	//sync send&recv packno
	memset(sendbuf, 0, PACKET_SIZE);
	cmdData = CMD_READ_CONFIG;
	memcpy(sendbuf+0, &cmdData, 4);
	memcpy(sendbuf+4, &g_packno, 4);
	g_packno++;

	result = send_data(client);
	if(result == 0)
		return result;;

	result = RcvData(client);
	if(result>0)
		{
		memcpy(&lconfig[0], rcvbuf+8, 4);
		memcpy(&lconfig[1], rcvbuf+12, 4);
		config[0] = lconfig[0];
		config[1] = lconfig[1];
	}

	return result;
}
/*
//uint32_t def_config[2] = {0xFFFFFF7F, 0x0001F000};
//CmdUpdateConfig(FALSE, def_config)
BOOL CmdUpdateConfig(int flag, uint32_t *conf)
{
	BOOL Result;
	unsigned long cmdData;

	//sync send&recv packno
	memset(sendbuf, 0, PACKET_SIZE);
	cmdData = CMD_UPDATE_CONFIG;
	WordsCpy(sendbuf+0, &cmdData, 4);
	WordsCpy(sendbuf+4, &g_packno, 4);
	WordsCpy(sendbuf+8, conf, 8);
	g_packno++;

	Result = SendData();
	if(Result == FALSE)
		return Result;

	Result = RcvData();

	return Result;
}
*/




// flag 0: update FW from boot.img
// flag 1: update FW from temp bin file
bool cmd_update_aprom(struct i2c_client *client, int flag, const char *filename,struct device *dev, unsigned int hwver)
{
	bool result, file_exist=0;
	unsigned int  config[2];
	//unsigned int devid, i, mode;
	unsigned long readcn, sendcn, cmdData, startaddr, totallen, pos;
	unsigned short lcksum, get_cksum;
	char *bin_path;
	struct file *file;
	mm_segment_t old_fs;
	loff_t pos_off = 0;
	loff_t file_size = 0;
	const struct firmware *fw;
	static u8 line[98304] = {0};

	epack_log("[CMD]%s start \n",__func__);
	if(flag)
		{
			old_fs = get_fs();
			set_fs(KERNEL_DS);

			file = filp_open(filename, O_RDONLY, 0666);

			if (!IS_ERR(file))
				{
					epack_log("FW file is exist under %s \n",filename);
					file_exist = 1;

						if (file_exist)
							{
								file_size = vfs_llseek(file, 0, SEEK_END);
								vfs_read(file, line, sizeof(line), &pos_off);
							}
					epack_log("%s : [%s] file_size(include version info 2bytes) = %lld\n",		__func__, filename, file_size);
				}
			else
				{
					epack_log("FW file is not exist under %s \n",filename);
					file_exist = 0;
					set_fs(old_fs);
					return file_exist;
				}
		}
	else
		{
			if(hwver ==0x04)
				{
					bin_path = DEFAULT_FW_PATH_REV_04;
				}
			else if(hwver ==0x05)
				{
					bin_path = DEFAULT_FW_PATH_REV_05;
				}
			result = request_firmware(&fw, bin_path, dev);
			if (result < 0)
				{
					epack_log("fail to request_firmware fwpath: %s (result:%d)\n",bin_path, result);
					return result;
				}
			epack_log("fw size(include version info 2bytes):%d, data_pointer: %p\n", fw->size, fw->data);
		}

	// send updata aprom command
	memset(sendbuf, 0, PACKET_SIZE);
	//cmdData = CMD_UPDATE_APROM;//CMD_UPDATE_APROM
	cmdData = CMD_UPDATE_DATE_FLASH;//CMD_UPDATE_APROM

	memcpy(sendbuf+0, &cmdData, 4);
	memcpy(sendbuf+4, &g_packno, 4);
	g_packno++;
	//start address
	startaddr = 0;
	memcpy(sendbuf+8, &startaddr, 4);

	// Try to obtain hFile's size
	if(flag)
		{
			imageBegin = (u8*)line;
			totallen = (file_size)-2;  // excluding version info
		}
	else
		{
			imageBegin = (u8*)fw->data;
			totallen = (fw->size)-2;  // excluding version info
		}
	epack_log("imageBegin: %p\n", imageBegin);
	memcpy(sendbuf+12, &totallen, 4);

	//read data from aprom.bin
	pos = 0;

	memcpy(FileBuffer, imageBegin, FILE_BUFFER);
	pos += FILE_BUFFER;

	readcn = FILE_BUFFER;
	sendcn = PACKET_SIZE - 16;
	memcpy(sendbuf+16, FileBuffer, sendcn);

	//send CMD_UPDATE_APROM
	result = send_data(client);
	if(result == 0)
		return result;

	result = RcvData(client);
	if(result == 0)
			return result;

	while(1)
	{

		//WriteFile
		while(sendcn < readcn)
		{
			sendbuf[0] = 0x00;
			cmdData = 0x00000000;//continue
			memcpy(sendbuf+0, &cmdData, 4);
			memcpy(sendbuf+4, &g_packno, 4);
			g_packno++;
			memcpy(sendbuf+8, FileBuffer+sendcn, PACKET_SIZE-8);
			result = send_data(client);
			if(result == 0)
				return result;

			result = RcvData(client);
			if(result == 0)
				return result;
			sendcn += PACKET_SIZE-8;
			if((sendcn < readcn) && (readcn - sendcn < PACKET_SIZE-8))
			{
				memcpy(FileBuffer, FileBuffer + sendcn, readcn - sendcn);
				sendcn = readcn - sendcn;
				break;
			}
		}

		if(sendcn >= readcn)
			sendcn = 0;
		readcn = 0;
		if(pos + FILE_BUFFER - sendcn > totallen)
			readcn = totallen - pos;
		else
			readcn = FILE_BUFFER - sendcn;
		if(readcn)
			{
				memcpy(FileBuffer + sendcn, imageBegin+pos, readcn);
		//memcpy(FileBuffer + sendcn, (char*)FileBuffer2+pos, readcn);
			}pos += readcn;

		if(sendcn == 0)
		{
			if(readcn == 0)
				break;
		}
		else
			readcn += sendcn;

		sendcn = 0;
	}
	epack_log("%s get checksum\n", __func__);
	memcpy(&get_cksum, rcvbuf+8, 2);
	lcksum = CalCheckSum((uint8_t*)imageBegin, totallen);
	if(result > 0)
	{
		if(lcksum == get_cksum)
		{
			config[0] = totallen;
			config[1] = lcksum;
			epack_log("%s write ck %lu %x\n",__func__, totallen, lcksum);
			result = cmd_run_cmd(client, CMD_WRITE_CHECKSUM, config);
			if(result > 0)
				epack_log("%s update success\n",__func__);
			else
				epack_log("%s update fail\n",__func__);
		}
		else
			epack_log("%s check cksum error, %x(should be %x)\n",__func__, lcksum, get_cksum);
	}
	else
		epack_log("%s Fail\n",__func__);

	epack_log("%s update finished\n",__func__);

	g_packno=1;

	if(flag)
		{
			set_fs(old_fs);
			filp_close(file, 0);
		}
	else
		{
			release_firmware(fw);
		}

	return result;
}

/* CMD_GET_BAT_VOLTAGE - pack vbat (unit:mv)*/
bool cmd_get_pack_voltage(struct epack_dev_data *epack, int *pack_voltage)
{
	bool result;
	u16 value;

	result = epack_i2c_read_reg(epack,CMD_GET_BAT_VOLTAGE,&value,sizeof(value));

	if(!result) {
		epack_log("[CMD]%s is failed\n",__func__);
		return 0;
	}
	epack_log("[CMD]%s : raw_data 0x%x\n",__func__,value);
	*pack_voltage = value;

	return 1;
}

/* CMD_GET_BAT_TEMP : pack temp (unit:0.1degree, signed value)*/
bool cmd_get_pack_temp(struct epack_dev_data *epack, int *pack_temp)
{
	bool result;
	u16 value;

	result = epack_i2c_read_reg(epack,CMD_GET_BAT_TEMP,&value,sizeof(value));

	if(!result) {
		epack_log("[CMD]%s is failed\n",__func__);
		return 0;
	}
	epack_log("[CMD]%s : raw_data 0x%x \n",__func__,value);
	*pack_temp = value;

	return 1;
}

/* CMD_GET_CHG_STATUS : pack bq24297 REG08 value*/
bool cmd_get_pack_chg_status(struct epack_dev_data *epack, int *chg_status)
{
	bool result;
	u8 value;

	result = epack_i2c_read_reg(epack,CMD_GET_CHG_STATUS,&value,sizeof(value));

	if(!result) {
		epack_log("[CMD]%s is failed\n",__func__);
		return 0;
	}
	epack_log("[CMD]%s : raw_data 0x%x \n",__func__,value);
	*chg_status = value;
	return 1;
}

/* CMD_GET_FAULT_STATUS : pack bq24297 REG09 value*/
bool cmd_get_pack_fault_status(struct epack_dev_data *epack, int *fault_status)
{
	bool result;
	u8 value;

	result = epack_i2c_read_reg(epack,CMD_GET_FAULT_STATUS,&value,sizeof(value));

	if(!result) {
		epack_log("[CMD]%s is failed\n",__func__);
		return 0;
	}
	epack_log("[CMD]%s : raw_data 0x%x \n",__func__,value);
	*fault_status = value;
	return 1;
}

/* CMD_GET_FAULT_STATUS
    bit[0] : pack tps61230A(pogo pin VBUS 5V) enable pin control status
	         0: ouput disable, 1: output enable
    bit[1] : pack USB type A OTG Power status
	         0: OTG Power Off, 1: OTG Power On
*/
bool cmd_get_pack_output_status(struct epack_dev_data *epack, int *output_status)
{
	bool result;
	u8 value;

	result = epack_i2c_read_reg(epack,CMD_GET_OUTPUT_STATUS,&value,sizeof(value));

	if(!result) {
		epack_log("[CMD]%s is failed\n",__func__);
		return 0;
	}
	epack_log("[CMD]%s : raw_data 0x%x \n",__func__,value);
	*output_status = value;
	return 1;
}

