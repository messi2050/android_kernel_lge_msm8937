//EEPROM MAP & CheckSum for HI1332, Camera-Driver@lge.com, 2015-08-27
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/crc32.h>
#include "msm_sd.h"
#include "msm_cci.h"
#include "msm_eeprom.h"

typedef enum OFILM_13MP_EEPROM_MAP {
	//HI1332 (OFILM) EEPROM MAP
	AWB_5100K_START_ADDR       = 0x0000,
	AWB_5100K_END_ADDR         = 0x0005,
	AWB_5100K_CHECKSUM_MSB     = 0x0006,
	AWB_5100K_CHECKSUM_LSB     = 0x0007,

	AWB_3000K_START_ADDR       = 0x0382,
	AWB_3000K_END_ADDR         = 0x0387,
	AWB_3000K_CHECKSUM_MSB     = 0x0388,
	AWB_3000K_CHECKSUM_LSB     = 0x0389,

	LSC_5100K_START_ADDR       = 0x000C,
	LSC_5100K_END_ADDR         = 0x037F,
	LSC_5100K_CHECKSUM_MSB     = 0x0380,
	LSC_5100K_CHECKSUM_LSB     = 0x0381,

	LSC_4000K_START_ADDR       = 0x038A,
	LSC_4000K_END_ADDR         = 0x06FD,
	LSC_4000K_CHECKSUM_MSB     = 0x06FE,
	LSC_4000K_CHECKSUM_LSB     = 0x06FF,

	//META INFO
	MODULE_VENDOR_ID_ADDR     = 0x0700,
	EEPROM_VERSION_ADDR       = 0x0770,
} MAP_OFILM;


int32_t msm_eeprom_checksum_ofilm_hi1332(struct msm_eeprom_ctrl_t *e_ctrl) {

	int32_t rc = -EFAULT;

	uint32_t dataSum_AWB_5K = 0;
	uint32_t dataSum_AWB_3K = 0;
	uint32_t dataSum_LSC_5K = 0;
	uint32_t dataSum_LSC_4K = 0;
	uint32_t trimSum_LSC_5K = 0;
	uint32_t trimSum_LSC_4K = 0;

	uint32_t checkSum_AWB_5K, checkSum_AWB_3K;
	uint32_t checkSum_LSC_5K, checkSum_LSC_4K;

	//Add: DataSum
	dataSum_AWB_5K = ((e_ctrl->cal_data.mapdata[0x0000]*256) + e_ctrl->cal_data.mapdata[0x0001])
						+ ((e_ctrl->cal_data.mapdata[0x0002]*256) + e_ctrl->cal_data.mapdata[0x0003])
						+((e_ctrl->cal_data.mapdata[0x0004]*256) + e_ctrl->cal_data.mapdata[0x0005]);
	dataSum_AWB_3K = ((e_ctrl->cal_data.mapdata[0x0382]*256) + e_ctrl->cal_data.mapdata[0x0383])
						+ ((e_ctrl->cal_data.mapdata[0x0384]*256) + e_ctrl->cal_data.mapdata[0x0385])
						+((e_ctrl->cal_data.mapdata[0x0386]*256) + e_ctrl->cal_data.mapdata[0x0387]);
	dataSum_LSC_5K = accumulation(e_ctrl, LSC_5100K_START_ADDR, LSC_5100K_END_ADDR);
	dataSum_LSC_4K = accumulation(e_ctrl, LSC_4000K_START_ADDR, LSC_4000K_END_ADDR);
	//Trimming Overflow
	trimSum_LSC_5K = dataSum_LSC_5K & 0xFFFF;
	trimSum_LSC_4K = dataSum_LSC_4K & 0xFFFF;

	//Get: CheckSum
	checkSum_AWB_5K = (e_ctrl->cal_data.mapdata[AWB_5100K_CHECKSUM_MSB]*256)
						+ e_ctrl->cal_data.mapdata[AWB_5100K_CHECKSUM_LSB];
	checkSum_AWB_3K = (e_ctrl->cal_data.mapdata[AWB_3000K_CHECKSUM_MSB]*256)
						+ e_ctrl->cal_data.mapdata[AWB_3000K_CHECKSUM_LSB];
	checkSum_LSC_5K = (e_ctrl->cal_data.mapdata[LSC_5100K_CHECKSUM_MSB]*256)
						+ e_ctrl->cal_data.mapdata[LSC_5100K_CHECKSUM_LSB];
	checkSum_LSC_4K = (e_ctrl->cal_data.mapdata[LSC_4000K_CHECKSUM_MSB]*256)
						+ e_ctrl->cal_data.mapdata[LSC_4000K_CHECKSUM_LSB];

	pr_err("[CHECK][BEFORE] e_ctrl->is_supported: %X\n", e_ctrl->is_supported);

	//Check#1: AWB 5K Part
	if (dataSum_AWB_5K == checkSum_AWB_5K) {
		pr_err("[CHECK] AWB 5K Data Valid\n");
		e_ctrl->is_supported |= 0x01;
	}
	else {
		pr_err("[CHECK][FAIL] dataSum_AWB_5K = %d, checkSum_AWB_5K = %d\n",
				dataSum_AWB_5K, checkSum_AWB_5K);
	}
	//Check#2: AWB 3K Part
	if (dataSum_AWB_3K == checkSum_AWB_3K) {
		pr_err("[CHECK] AWB 3K Data Valid\n");
		e_ctrl->is_supported |= 0x02;
	}
	else {
		pr_err("[CHECK][FAIL] dataSum_AWB_3K = %d, checkSum_AWB_3K = %d\n",
				dataSum_AWB_3K, checkSum_AWB_3K);
	}
	//Check#3: LSC 5K Part
	if (trimSum_LSC_5K == checkSum_LSC_5K) {
		pr_err("[CHECK] LSC 5K Data Valid\n");
		e_ctrl->is_supported |= 0x04;
	}
	else {
		pr_err("[CHECK][FAIL] dataSum_LSC_5K = %d, trimSum_LSC_5K = %d, checkSum_LSC_5K = %d\n",
				dataSum_LSC_5K, trimSum_LSC_5K, checkSum_LSC_5K);
	}

	//Check#4: LSC 4K Part
	if (trimSum_LSC_4K == checkSum_LSC_4K) {
		pr_err("[CHECK] LSC 4K Data Valid\n");
		e_ctrl->is_supported |= 0x08;
	}
	else {
		pr_err("[CHECK][FAIL] dataSum_LSC_4K = %d, trimSum_LSC_4K = %d, checkSum_LSC_4K = %d\n",
				dataSum_LSC_4K, trimSum_LSC_4K, checkSum_LSC_4K);
	}

	if(e_ctrl->is_supported == 0xF) { //All bits are On
		pr_err("%s checksum succeed!\n", __func__);
		rc = 0;
	} else {
		//each bit (in e_ctrl->is_supported) indicates the checksum results.
	}
	return rc;
}