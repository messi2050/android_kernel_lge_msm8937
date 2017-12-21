//EEPROM MAP & CheckSum for T4KB3, Camera-Driver@lge.com, 2015-06-11
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/crc32.h>
#include "msm_sd.h"
#include "msm_cci.h"
#include "msm_eeprom.h"

typedef enum IMTECH_8MP_EEPROM_MAP {
	//T4KB3 (IMTECH) EEPROM MAP
	//Big-Endian: MSB[15:8] LSB[7:0]
	AWB_5100K_START_ADDR       = 0x0000,
	AWB_5100K_END_ADDR         = 0x0005,
	AWB_5100K_CHECKSUM_MSB     = 0x0006,
	AWB_5100K_CHECKSUM_LSB     = 0x0007,

	LSC_5100K_START_ADDR       = 0x000C,
	LSC_5100K_END_ADDR         = 0x037F,
	LSC_5100K_CHECKSUM_MSB     = 0x0380,
	LSC_5100K_CHECKSUM_LSB     = 0x0381,

	AWB_3000K_START_ADDR       = 0x0382,
	AWB_3000K_END_ADDR         = 0x0387,
	AWB_3000K_CHECKSUM_MSB     = 0x0388,
	AWB_3000K_CHECKSUM_LSB     = 0x0389,

	LSC_4000K_START_ADDR       = 0x038A,
	LSC_4000K_END_ADDR         = 0x06FD,
	LSC_4000K_CHECKSUM_MSB     = 0x06FE,
	LSC_4000K_CHECKSUM_LSB     = 0x06FF,

#if 0 //VCM CAL IS NOT USED for COST REASON
	//Big-Endian: MSB[15:8] LSB[7:0]
	VCM_STARTCODE_START_ADDR   = 0x0703,
	VCM_STARTCODE_END_ADDR     = 0x070A,
	//"Little-Endian": LSB[7:0] MSB[15:8]
	VCM_STARTCODE_CHECKSUM_LSB = 0x070B,
	VCM_STARTCODE_CHECKSUM_MSB = 0x070C,

	VCM_DAC_START_ADDR         = 0x0710,
	VCM_DAC_END_ADDR           = 0x0751,
	VCM_DAC_CHECKSUM_LSB       = 0x0752,
	VCM_DAC_CHECKSUM_MSB       = 0x0753,
#endif

	//TOTAL CHECKSUM (Big-Endian)
	DATA_START_ADDR            = 0x0000,
	DATA_END_ADDR              = 0x07FB,
	TOTAL_CHECKSUM_START_ADDR  = 0x07FC,
	TOTAL_CHECKSUM_END_ADDR    = 0x07FF,

	//META INFO
	MODULE_VENDOR_ID_ADDR     = 0x0700,
	EEPROM_VERSION_ADDR       = 0x0770,
} MAP_IMTECH;

int32_t msm_eeprom_checksum_imtech_ov8858(struct msm_eeprom_ctrl_t *e_ctrl)
{
	int32_t rc = -EFAULT;

	uint32_t dataSum_AWB_5K = 0;
	uint32_t dataSum_AWB_3K = 0;
	uint32_t dataSum_LSC_5K = 0;
	uint32_t dataSum_LSC_4K = 0;
	uint32_t trimSum_LSC_5K = 0;
	uint32_t trimSum_LSC_4K = 0;
	uint32_t dataSum_Total  = 0;

	uint32_t checkSum_AWB_5K, checkSum_AWB_3K;
	uint32_t checkSum_LSC_5K, checkSum_LSC_4K, checkSum_Total;

	//Add: DataSum
	dataSum_AWB_5K = accumulation(e_ctrl, AWB_5100K_START_ADDR, AWB_5100K_END_ADDR);
	dataSum_AWB_3K = accumulation(e_ctrl, AWB_3000K_START_ADDR, AWB_3000K_END_ADDR);
	dataSum_LSC_5K = accumulation(e_ctrl, LSC_5100K_START_ADDR, LSC_5100K_END_ADDR);
	dataSum_LSC_4K = accumulation(e_ctrl, LSC_4000K_START_ADDR, LSC_4000K_END_ADDR);
	dataSum_Total = accumulation(e_ctrl, DATA_START_ADDR, DATA_END_ADDR);

	//Trimming Overflow
	trimSum_LSC_5K = dataSum_LSC_5K & 0xFFFF;
	trimSum_LSC_4K = dataSum_LSC_4K & 0xFFFF;

	//Get: CheckSum
	checkSum_AWB_5K = shiftedSum(e_ctrl, AWB_5100K_CHECKSUM_MSB, AWB_5100K_CHECKSUM_LSB, BigEndian);
	checkSum_AWB_3K = shiftedSum(e_ctrl, AWB_3000K_CHECKSUM_MSB, AWB_3000K_CHECKSUM_LSB, BigEndian);
	checkSum_LSC_5K = shiftedSum(e_ctrl, LSC_5100K_CHECKSUM_MSB, LSC_5100K_CHECKSUM_LSB, BigEndian);
	checkSum_LSC_4K = shiftedSum(e_ctrl, LSC_4000K_CHECKSUM_MSB, LSC_4000K_CHECKSUM_LSB, BigEndian);
	checkSum_Total  = shiftedSum(e_ctrl, TOTAL_CHECKSUM_START_ADDR, TOTAL_CHECKSUM_END_ADDR, BigEndian);

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

	//Check#5: Total CheckSum
	if (dataSum_Total == checkSum_Total) {
		pr_err("[CHECK] Total CheckSum Valid\n");
		e_ctrl->is_supported |= 0x1F;
	}
	else {
		pr_err("[CHECK][FAIL] dataSum_Total = %u, checkSum_Total = %u\n",
				dataSum_Total, checkSum_Total);
	}

	pr_err("[CHECK][AFTER] e_ctrl->is_supported: %X\n", e_ctrl->is_supported);

	if(e_ctrl->is_supported == 0x1F) { //All bits are On
		pr_err("%s checksum succeed!\n", __func__);
		rc = 0;
	} else {
		//each bit (in e_ctrl->is_supported) indicates the checksum results.
	}

	return rc;
};

int32_t msm_eeprom_checksum_imtech_t4kb3(struct msm_eeprom_ctrl_t *e_ctrl)
{
	int32_t rc = -EFAULT;

	uint32_t dataSum_AWB_5K = 0;
	uint32_t dataSum_AWB_3K = 0;
	uint32_t dataSum_LSC_5K = 0;
	uint32_t dataSum_LSC_4K = 0;
	uint32_t trimSum_LSC_5K = 0;
	uint32_t trimSum_LSC_4K = 0;
	uint32_t dataSum_Total  = 0;

	uint32_t checkSum_AWB_5K, checkSum_AWB_3K;
	uint32_t checkSum_LSC_5K, checkSum_LSC_4K, checkSum_Total;

	//Add: DataSum
	dataSum_AWB_5K = accumulation(e_ctrl, AWB_5100K_START_ADDR, AWB_5100K_END_ADDR);
	dataSum_AWB_3K = accumulation(e_ctrl, AWB_3000K_START_ADDR, AWB_3000K_END_ADDR);
	dataSum_LSC_5K = accumulation(e_ctrl, LSC_5100K_START_ADDR, LSC_5100K_END_ADDR);
	dataSum_LSC_4K = accumulation(e_ctrl, LSC_4000K_START_ADDR, LSC_4000K_END_ADDR);
	dataSum_Total = accumulation(e_ctrl, DATA_START_ADDR, DATA_END_ADDR);

	//Trimming Overflow
	trimSum_LSC_5K = dataSum_LSC_5K & 0xFFFF;
	trimSum_LSC_4K = dataSum_LSC_4K & 0xFFFF;

	//Get: CheckSum
	checkSum_AWB_5K = shiftedSum(e_ctrl, AWB_5100K_CHECKSUM_MSB, AWB_5100K_CHECKSUM_LSB, BigEndian);
	checkSum_AWB_3K = shiftedSum(e_ctrl, AWB_3000K_CHECKSUM_MSB, AWB_3000K_CHECKSUM_LSB, BigEndian);
	checkSum_LSC_5K = shiftedSum(e_ctrl, LSC_5100K_CHECKSUM_MSB, LSC_5100K_CHECKSUM_LSB, BigEndian);
	checkSum_LSC_4K = shiftedSum(e_ctrl, LSC_4000K_CHECKSUM_MSB, LSC_4000K_CHECKSUM_LSB, BigEndian);
	checkSum_Total  = shiftedSum(e_ctrl, TOTAL_CHECKSUM_START_ADDR, TOTAL_CHECKSUM_END_ADDR, BigEndian);

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

	//Check#5: Total CheckSum
	if (dataSum_Total == checkSum_Total) {
		pr_err("[CHECK] Total CheckSum Valid\n");
		e_ctrl->is_supported |= 0x1F;
	}
	else {
		pr_err("[CHECK][FAIL] dataSum_Total = %u, checkSum_Total = %u\n",
				dataSum_Total, checkSum_Total);
	}

	pr_err("[CHECK][AFTER] e_ctrl->is_supported: %X\n", e_ctrl->is_supported);

	if(e_ctrl->is_supported == 0x1F) { //All bits are On
		pr_err("%s checksum succeed!\n", __func__);
		rc = 0;
	} else {
		//each bit (in e_ctrl->is_supported) indicates the checksum results.
	}

	return rc;
};

int32_t msm_eeprom_checksum_imtech_hi841(struct msm_eeprom_ctrl_t *e_ctrl) {
	uint32_t k, awb_datasum_5k, awb_datasum_3k;
	uint32_t awb_checksum_5k, lsc_checksum_5k, lsc_checksum_4k, awb_checksum_3k;
	uint32_t lsc_datasum_5k, lsc_datasum_4k, lsc_cal_5k, lsc_cal_4k;
	uint32_t total_datasum, total_checksum;
	int32_t rc = -EFAULT;

	lsc_datasum_5k = 0;
	lsc_datasum_4k = 0;
	lsc_cal_5k = 0;
	lsc_cal_4k = 0;
	total_datasum = 0;
	total_checksum = 0;

	awb_datasum_5k = 0;
	for(k = 0; k < 0x06; k++) {
		awb_datasum_5k += e_ctrl->cal_data.mapdata[k];
	}
	awb_checksum_5k = (e_ctrl->cal_data.mapdata[0x0006]*256) + e_ctrl->cal_data.mapdata[0x0007];

	awb_datasum_3k = 0;
	for(k = 0x382; k < 0x388; k++) {
		awb_datasum_3k += e_ctrl->cal_data.mapdata[k];
	}
	awb_checksum_3k = (e_ctrl->cal_data.mapdata[0x388]*256) + e_ctrl->cal_data.mapdata[0x389];

	for (k = 0x0C; k < 0x380; k++) {
		lsc_datasum_5k += e_ctrl->cal_data.mapdata[k];
		lsc_cal_5k = lsc_datasum_5k & 0xffff;
	}

	for (k = 0x38A; k < 0x6fE; k++) {
		lsc_datasum_4k += (e_ctrl->cal_data.mapdata[k]);
		lsc_cal_4k = lsc_datasum_4k & 0xffff;
	}
	lsc_checksum_5k = (e_ctrl->cal_data.mapdata[0x0380]*256) + e_ctrl->cal_data.mapdata[0x0381];
	lsc_checksum_4k = (e_ctrl->cal_data.mapdata[0x06FE]*256) + e_ctrl->cal_data.mapdata[0x06FF];

	for(k = 0x0; k < 0x7fC; k++) {
		total_datasum += e_ctrl->cal_data.mapdata[k];
		total_datasum &= 0xffffff;
	}
	total_checksum = (e_ctrl->cal_data.mapdata[0x07FC] << 24) |
		(e_ctrl->cal_data.mapdata[0x07FD] << 16) |
		(e_ctrl->cal_data.mapdata[0x07FE] << 8) |
		(e_ctrl->cal_data.mapdata[0x07FF] << 0);

	pr_info("%s %d verify eeprom data, id = 0x%x, name = %s\n",
	 __func__, __LINE__, e_ctrl->cal_data.mapdata[0x700], e_ctrl->eboard_info->eeprom_name);

	if((awb_datasum_5k == awb_checksum_5k) && (awb_datasum_3k == awb_checksum_3k)
	   && (lsc_cal_5k == lsc_checksum_5k) && (lsc_cal_4k == lsc_checksum_4k)
	   && (total_datasum == total_checksum)){
		rc = 0;
	} else {
		pr_err("awb_datasum_5k = %d, awb_checksum_5k = %d\n", awb_datasum_5k, awb_checksum_5k);
		pr_err("awb_datasum_3k = %d, awb_checksum_3k = %d\n", awb_datasum_3k, awb_checksum_3k);
		pr_err("lsc_datasum_5k = %d, lsc_cal_5k = %d, lsc_checksum_5k = %d\n", lsc_datasum_5k, lsc_cal_5k, lsc_checksum_5k);
		pr_err("lsc_datasum_4k = %d, lsc_cal_4k = %d, lsc_checksum_4k = %d\n", lsc_datasum_4k, lsc_cal_4k, lsc_checksum_4k);
	}
	return rc;
};
