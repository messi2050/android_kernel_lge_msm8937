//EEPROM MAP & CheckSum for HI-841, Camera-Driver@lge.com, 2015-06-11
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/crc32.h>
#include "msm_sd.h"
#include "msm_cci.h"
#include "msm_eeprom.h"

typedef enum COWELL_8MP_EEPROM_MAP {
	//HI841 (COWELL) EEPROM MAP
	//add 13M mn34153(COWELL)

	//Big-Endian: MSB[15:8] LSB[7:0]
	AWB_5100K_START_ADDR     = 0x0000,
	AWB_5100K_END_ADDR       = 0x0005,

	LSC_5100K_START_ADDR     = 0x000C,
	LSC_5100K_END_ADDR       = 0x037F,

	//Big-Endian: MSB[15:8] LSB[7:0]
	AWB_2800K_START_ADDR     = 0x0382,
	AWB_2800K_END_ADDR       = 0x0387,

	LSC_2800K_START_ADDR     = 0x038A,
	LSC_2800K_END_ADDR       = 0x06FD,

	//add for 13M mn34153
	AWB_4100K_START_ADDR     = 0x0800,
	AWB_4100K_END_ADDR       = 0x0805,

	LSC_4100K_START_ADDR     = 0x080C,
	LSC_4100K_END_ADDR       = 0x0B7F,


	//Big-Endian: MSB[15:8] LSB[7:0]
	VCM_STARTCODE_START_ADDR = 0x0703,
	VCM_STARTCODE_END_ADDR   = 0x070A,

	//LSC CHECK SUM
    //add 13M mn34153
	LSC_4100K_CHECKSUM_START_ADDR = 0x07F3,
	LSC_4100K_CHECKSUM_END_ADDR   = 0x07F5,

	LSC_5100K_CHECKSUM_START_ADDR = 0x07F6,
	LSC_5100K_CHECKSUM_END_ADDR   = 0x07F8,

	LSC_2800K_CHECKSUM_START_ADDR = 0x07F9,
	LSC_2800K_CHECKSUM_END_ADDR   = 0x07FB,

	//TOTAL CHECKSUM
	DATA_START_ADDR           = 0x0000,
	DATA_END_ADDR             = 0x07FB,
	DATA_END_ADDR_13M         = 0x0B7F,//add 13M mn34153

	TOTAL_CHECKSUM_START_ADDR = 0x07FC,
	TOTAL_CHECKSUM_END_ADDR   = 0x07FF,

	//META INFO
	MODULE_VENDOR_ID_ADDR     = 0x0700,
	EEPROM_VERSION_ADDR       = 0x0770,
	RESERVED_META_START_ADDR  = 0x077A,
	RESERVED_META_END_ADDR    = 0x077F,

} MAP_COWELL;

int32_t msm_eeprom_checksum_cowell_hi841(struct msm_eeprom_ctrl_t *e_ctrl)
{
	uint32_t addr = 0;
	int32_t rc = -EFAULT;

	uint32_t dataSum_Total  = 0;
	uint32_t checkSum_Total = 0;

	//DataSum
	for(addr = DATA_START_ADDR; addr <= DATA_END_ADDR; addr++)
	{
		//1. AWB, VCM Data Area (Word Area)
		if ((addr >= AWB_5100K_START_ADDR && addr <= AWB_5100K_END_ADDR)
		||  (addr >= AWB_2800K_START_ADDR && addr <= AWB_2800K_END_ADDR)
		||  (addr >= VCM_STARTCODE_START_ADDR && addr <= VCM_STARTCODE_END_ADDR))
		{
			dataSum_Total += shiftedSum(e_ctrl, addr, addr+1, BigEndian);
			addr++;
		}
		//2. LSC Data Area
		else if ((addr >= LSC_5100K_START_ADDR && addr <= LSC_5100K_END_ADDR)
			 ||  (addr >= LSC_2800K_START_ADDR && addr <= LSC_2800K_END_ADDR))
		{
			dataSum_Total += e_ctrl->cal_data.mapdata[addr];
		}
		//3. No Data Area: SKip
		else {
		}
	}

	//CheckSum
	checkSum_Total  = shiftedSum(e_ctrl, TOTAL_CHECKSUM_START_ADDR, TOTAL_CHECKSUM_END_ADDR, BigEndian);

	pr_err("[CHECK][BEFORE] e_ctrl->is_supported: %X\n", e_ctrl->is_supported);

	//Check: Total CheckSum
	if (dataSum_Total == checkSum_Total) {
		pr_err("[CHECK] Total CheckSum Valid\n");
		e_ctrl->is_supported |= 0x1F;
	}
	else {
		pr_err("[CHECK][FAIL] dataSum_Total = %u, checkSum_Total = %u\n",
				dataSum_Total, checkSum_Total);
	}

	pr_err("[CHECK][AFTER] e_ctrl->is_supported: %X\n", e_ctrl->is_supported);

	if(e_ctrl->is_supported == 0x1F) { //All bits are On (AWB 5K 0x01 AWB 3K 0x02 LSC 5K 0x04, LSC 3K 0x08, Total 0x10)
		pr_err("%s checksum succeed!\n", __func__);
		rc = 0;
	} else {
		//each bit (in e_ctrl->is_supported) indicates the checksum results.
	}

	return rc;
};

int32_t msm_eeprom_checksum_cowell_mn34153(struct msm_eeprom_ctrl_t *e_ctrl)
{
	uint32_t addr = 0;
	int32_t rc = -EFAULT;

	uint32_t dataSum_Total  = 0;
	uint32_t checkSum_Total = 0;

	//DataSum
	for(addr = DATA_START_ADDR; addr <= DATA_END_ADDR_13M; addr++)
	{
		//1. AWB, VCM Data Area (Word Area)
		if ((addr >= AWB_5100K_START_ADDR && addr <= AWB_5100K_END_ADDR)
		||  (addr >= AWB_4100K_START_ADDR && addr <= AWB_4100K_END_ADDR)
		||  (addr >= AWB_2800K_START_ADDR && addr <= AWB_2800K_END_ADDR)
		||  (addr >= VCM_STARTCODE_START_ADDR && addr <= VCM_STARTCODE_END_ADDR))
		{
			dataSum_Total += shiftedSum(e_ctrl, addr, addr+1, BigEndian);
			addr++;
		}
		//2. LSC Data Area
		else if ((addr >= LSC_5100K_START_ADDR && addr <= LSC_5100K_END_ADDR)
			 ||  (addr >= LSC_4100K_START_ADDR && addr <= LSC_4100K_END_ADDR)
			 ||  (addr >= LSC_2800K_START_ADDR && addr <= LSC_2800K_END_ADDR))
		{
			dataSum_Total += e_ctrl->cal_data.mapdata[addr];
		}
		//3. No Data Area: SKip
		else {
		}
	}

	//CheckSum
	checkSum_Total  = shiftedSum(e_ctrl, TOTAL_CHECKSUM_START_ADDR, TOTAL_CHECKSUM_END_ADDR, BigEndian);

	pr_err("[CHECK][BEFORE] e_ctrl->is_supported: %X\n", e_ctrl->is_supported);

	//Check: Total CheckSum
	if (dataSum_Total == checkSum_Total) {
		pr_err("[CHECK] Total CheckSum Valid\n");
		e_ctrl->is_supported |= 0x1F;
	}
	else {
		pr_err("[CHECK][FAIL] dataSum_Total = %u, checkSum_Total = %u\n",
				dataSum_Total, checkSum_Total);
	}

	pr_err("[CHECK][AFTER] e_ctrl->is_supported: %X\n", e_ctrl->is_supported);

	if(e_ctrl->is_supported == 0x1F) { //All bits are On (AWB 5K 0x01 AWB 3K 0x02 LSC 5K 0x04, LSC 3K 0x08, Total 0x10)
		pr_err("%s checksum succeed!\n", __func__);
		rc = 0;
	} else {
		//each bit (in e_ctrl->is_supported) indicates the checksum results.
	}

	return rc;
};

int32_t msm_eeprom_checksum_cowell_hi842(struct msm_eeprom_ctrl_t *e_ctrl)
{
	uint32_t addr = 0;
	int32_t rc = -EFAULT;

	uint32_t dataSum_Total  = 0;
	uint32_t checkSum_Total = 0;

	//DataSum
	for(addr = DATA_START_ADDR; addr <= DATA_END_ADDR; addr++)
	{
		//1. AWB, VCM Data Area (Word Area)
		if ((addr >= AWB_5100K_START_ADDR && addr <= AWB_5100K_END_ADDR)
		||  (addr >= AWB_2800K_START_ADDR && addr <= AWB_2800K_END_ADDR)
		||  (addr >= VCM_STARTCODE_START_ADDR && addr <= VCM_STARTCODE_END_ADDR))
		{
			dataSum_Total += shiftedSum(e_ctrl, addr, addr+1, BigEndian);
			addr++;
		}
		//2. LSC Data Area
		else if ((addr >= LSC_5100K_START_ADDR && addr <= LSC_5100K_END_ADDR)
			 ||  (addr >= LSC_2800K_START_ADDR && addr <= LSC_2800K_END_ADDR))
		{
			dataSum_Total += e_ctrl->cal_data.mapdata[addr];
		}
		//3. No Data Area: SKip
		else {
		}
	}

	//CheckSum
	checkSum_Total  = shiftedSum(e_ctrl, TOTAL_CHECKSUM_START_ADDR, TOTAL_CHECKSUM_END_ADDR, BigEndian);

	pr_err("[CHECK][BEFORE] e_ctrl->is_supported: %X\n", e_ctrl->is_supported);

	//Check: Total CheckSum
	if (dataSum_Total == checkSum_Total) {
		pr_err("[CHECK] Total CheckSum Valid\n");
		e_ctrl->is_supported |= 0x1F;
	}
	else {
		pr_err("[CHECK][FAIL] dataSum_Total = %u, checkSum_Total = %u\n",
				dataSum_Total, checkSum_Total);
	}

	pr_err("[CHECK][AFTER] e_ctrl->is_supported: %X\n", e_ctrl->is_supported);

	if(e_ctrl->is_supported == 0x1F) { //All bits are On (AWB 5K 0x01 AWB 3K 0x02 LSC 5K 0x04, LSC 3K 0x08, Total 0x10)
		pr_err("%s checksum succeed!\n", __func__);
		rc = 0;
	} else {
		//each bit (in e_ctrl->is_supported) indicates the checksum results.
	}

	return rc;
};
