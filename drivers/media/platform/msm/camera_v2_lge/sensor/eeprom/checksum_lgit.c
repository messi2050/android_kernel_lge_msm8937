//EEPROM MAP & CheckSum for HI-841, Camera-Driver@lge.com, 2015-06-11
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/crc32.h>
#include "msm_sd.h"
#include "msm_cci.h"
#include "msm_eeprom.h"

typedef enum LGIT_8MP_EEPROM_MAP {
	//T4KA3 (LGIT) EEPROM MAP
	//Little-Endian: LSB[7:0] MSB[15:8]
	AWB_5100K_START_ADDR       = 0x0000,
	AWB_5100K_END_ADDR         = 0x0005,
	AWB_5100K_CHECKSUM_LSB     = 0x0006,
	AWB_5100K_CHECKSUM_MSB     = 0x0007,

	AWB_3000K_START_ADDR       = 0x0382,
	AWB_3000K_END_ADDR         = 0x0387,
	AWB_3000K_CHECKSUM_LSB     = 0x0388,
	AWB_3000K_CHECKSUM_MSB     = 0x0389,

	LSC_5100K_START_ADDR       = 0x000C,
	LSC_5100K_END_ADDR         = 0x037F,
	LSC_5100K_CHECKSUM_LSB     = 0x0380,
	LSC_5100K_CHECKSUM_MSB     = 0x0381,

	LSC_4000K_START_ADDR       = 0x038A,
	LSC_4000K_END_ADDR         = 0x06FD,
	LSC_4000K_CHECKSUM_LSB     = 0x06FE,
	LSC_4000K_CHECKSUM_MSB     = 0x06FF,

	LSC_3000K_START_ADDR       = 0x0A00,
	LSC_3000K_END_ADDR         = 0x0D73,
	LSC_3000K_CHECKSUM_LSB     = 0x0D74,
	LSC_3000K_CHECKSUM_MSB     = 0x0D75,

	//TOTAL CHECKSUM (Little-Endian)
	DATA_START_ADDR            = 0x0000,
	DATA_END_ADDR              = 0x07FB,
	TOTAL_CHECKSUM_START_ADDR  = 0x07FC,
	TOTAL_CHECKSUM_END_ADDR    = 0x07FF,

	//META INFO
	MODULE_VENDOR_ID_ADDR     = 0x0700,
	EEPROM_VERSION_ADDR       = 0x0770,
} MAP_LGIT;


int32_t msm_eeprom_checksum_lgit_v0d_t4ka3(struct msm_eeprom_ctrl_t *e_ctrl)
{
	int32_t rc = -EFAULT;

	uint32_t dataSum_AWB_5K = 0;
	uint32_t dataSum_AWB_3K = 0;
	uint32_t dataSum_LSC_5K = 0;
	uint32_t dataSum_LSC_4K = 0;
	uint32_t dataSum_LSC_3K = 0;
	uint32_t trimSum_LSC_5K = 0;
	uint32_t trimSum_LSC_4K = 0;
	uint32_t trimSum_LSC_3K = 0;
	uint32_t dataSum_Total  = 0;

	uint32_t checkSum_AWB_5K, checkSum_AWB_3K;
	uint32_t checkSum_LSC_5K, checkSum_LSC_4K, checkSum_LSC_3K, checkSum_Total;

	//Add: DataSum
	dataSum_AWB_5K = accumulation(e_ctrl, AWB_5100K_START_ADDR, AWB_5100K_END_ADDR);
	dataSum_AWB_3K = accumulation(e_ctrl, AWB_3000K_START_ADDR, AWB_3000K_END_ADDR);
	dataSum_LSC_5K = accumulation(e_ctrl, LSC_5100K_START_ADDR, LSC_5100K_END_ADDR);
	dataSum_LSC_4K = accumulation(e_ctrl, LSC_4000K_START_ADDR, LSC_4000K_END_ADDR);
	dataSum_LSC_3K = accumulation(e_ctrl, LSC_3000K_START_ADDR, LSC_3000K_END_ADDR);
	dataSum_Total = accumulation(e_ctrl, DATA_START_ADDR, DATA_END_ADDR);

	//Trimming Overflow
	trimSum_LSC_5K = dataSum_LSC_5K & 0xFFFF;
	trimSum_LSC_4K = dataSum_LSC_4K & 0xFFFF;
	trimSum_LSC_3K = dataSum_LSC_3K & 0xFFFF;

	//Get: CheckSum
	checkSum_AWB_5K = shiftedSum(e_ctrl, AWB_5100K_CHECKSUM_LSB, AWB_5100K_CHECKSUM_MSB, LittleEndian);
	checkSum_AWB_3K = shiftedSum(e_ctrl, AWB_3000K_CHECKSUM_LSB, AWB_3000K_CHECKSUM_MSB, LittleEndian);
	checkSum_LSC_5K = shiftedSum(e_ctrl, LSC_5100K_CHECKSUM_LSB, LSC_5100K_CHECKSUM_MSB, LittleEndian);
	checkSum_LSC_4K = shiftedSum(e_ctrl, LSC_4000K_CHECKSUM_LSB, LSC_4000K_CHECKSUM_MSB, LittleEndian);
	checkSum_LSC_3K = shiftedSum(e_ctrl, LSC_3000K_CHECKSUM_LSB, LSC_3000K_CHECKSUM_MSB, LittleEndian);
	checkSum_Total  = shiftedSum(e_ctrl, TOTAL_CHECKSUM_START_ADDR, TOTAL_CHECKSUM_END_ADDR, LittleEndian);

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

	//Check#5: LSC 3K Part
	if (trimSum_LSC_3K == checkSum_LSC_3K) {
		pr_err("[CHECK] LSC 3K Data Valid\n");
		e_ctrl->is_supported |= 0x10;
	}
	else {
		pr_err("[CHECK][FAIL] dataSum_LSC_3K = %d, trimSum_LSC_3K = %d, checkSum_LSC_3K = %d\n",
				dataSum_LSC_3K, trimSum_LSC_3K, checkSum_LSC_3K);
	}

	//Check#5: Total CheckSum
	if (dataSum_Total == checkSum_Total) {
		pr_err("[CHECK] Total CheckSum Valid\n");
		e_ctrl->is_supported |= 0x3F;
	}
	else {
		pr_err("[CHECK][FAIL] dataSum_Total = %u, checkSum_Total = %u\n",
				dataSum_Total, checkSum_Total);
	}

	pr_err("[CHECK][AFTER] e_ctrl->is_supported: %X\n", e_ctrl->is_supported);

	if(e_ctrl->is_supported == 0x3F) { //All bits are On
		pr_err("%s checksum succeed!\n", __func__);
		rc = 0;
	} else {
		//each bit (in e_ctrl->is_supported) indicates the checksum results.
	}

	return rc;
}

int32_t msm_eeprom_checksum_lgit_v0d(struct msm_eeprom_ctrl_t *e_ctrl) {
	uint32_t k, awb_datasum_5k, awb_datasum_3k;
	uint32_t awb_checksum_5k, lsc_checksum_5k, lsc_checksum_4k, awb_checksum_3k;
	uint32_t lsc_datasum_5k, lsc_datasum_4k, lsc_cal_5k, lsc_cal_4k;
	int32_t rc = -EFAULT;

	lsc_datasum_5k = 0;
	lsc_datasum_4k = 0;
	lsc_cal_5k = 0;
	lsc_cal_4k = 0;

	awb_datasum_5k = 0;
	for(k = 0; k < 0x06; k++) {
		awb_datasum_5k += e_ctrl->cal_data.mapdata[k];
	}
	awb_checksum_5k = (e_ctrl->cal_data.mapdata[0x0007]*256) + e_ctrl->cal_data.mapdata[0x0006];

	awb_datasum_3k = 0;
	for(k = 0x382; k < 0x388; k++) {
		awb_datasum_3k += e_ctrl->cal_data.mapdata[k];
	}
	awb_checksum_3k = (e_ctrl->cal_data.mapdata[0x389]*256) + e_ctrl->cal_data.mapdata[0x388];

	for (k = 0x0c; k < 0x380; k++) {
		lsc_datasum_5k += e_ctrl->cal_data.mapdata[k];
		lsc_cal_5k = lsc_datasum_5k & 0xffff;
	}

	for (k = 0x38a; k < 0x6fe; k++) {
		lsc_datasum_4k += (e_ctrl->cal_data.mapdata[k]);
		lsc_cal_4k = lsc_datasum_4k & 0xffff;
	}

	lsc_checksum_5k = (e_ctrl->cal_data.mapdata[0x0381]*256) + e_ctrl->cal_data.mapdata[0x0380];
	lsc_checksum_4k = (e_ctrl->cal_data.mapdata[0x06FF]*256) + e_ctrl->cal_data.mapdata[0x06FE];

	pr_info("%s %d verify eeprom data, id = 0x%x, ver = 0x%x\n",
	 __func__, __LINE__, e_ctrl->cal_data.mapdata[0x700], e_ctrl->cal_data.mapdata[0x770]);

	if((awb_datasum_5k == awb_checksum_5k) && (awb_datasum_3k == awb_checksum_3k)
	   && (lsc_cal_5k == lsc_checksum_5k) && (lsc_cal_4k == lsc_checksum_4k)){
		rc = 0;
	} else {
		pr_err("awb_datasum_5k = %d, awb_checksum_5k = %d\n", awb_datasum_5k, awb_checksum_5k);
		pr_err("awb_datasum_3k = %d, awb_checksum_3k = %d\n", awb_datasum_3k, awb_checksum_3k);
		pr_err("lsc_datasum_5k = %d, lsc_cal_5k = %d, lsc_checksum_5k = %d\n", lsc_datasum_5k, lsc_cal_5k, lsc_checksum_5k);
		pr_err("lsc_datasum_4k = %d, lsc_cal_4k = %d, lsc_checksum_4k = %d\n", lsc_datasum_4k, lsc_cal_4k, lsc_checksum_4k);
	}

	return rc;
}

int32_t msm_eeprom_checksum_lgit_at24c16d(struct msm_eeprom_ctrl_t *e_ctrl) {
	uint16_t awb_5k = 0, awb_5k_checksum = 0;
	uint16_t lsc_5k = 0, lsc_5k_checksum = 0;
	uint16_t sensor_id = 0, sensor_id_checksum = 0;
	int32_t rc = -EFAULT;
	int i;

	for(i = 0x0; i < 0x6; i++) {
		awb_5k += e_ctrl->cal_data.mapdata[i];
	}
	awb_5k_checksum = (e_ctrl->cal_data.mapdata[0x7] << 8) |
		e_ctrl->cal_data.mapdata[0x6];

	for(i = 0xC; i < 0x380; i++) {
		lsc_5k += e_ctrl->cal_data.mapdata[i];
	}
	lsc_5k_checksum = (e_ctrl->cal_data.mapdata[0x381] << 8) |
		e_ctrl->cal_data.mapdata[0x380];

/*LGE_CHANGE_S, Make EEPROM CheckSum Property for AAT Mode, 2016-02-23, seungmin.hong@lge.com*/
	if(!awb_5k_checksum || !lsc_5k_checksum){
		//Data does NOT exist
		pr_err("%s EEPROM CRC Data does NOT exist!\n", __func__);
		return rc;
	}
/*LGE_CHANGE_E, Make EEPROM CheckSum Property for AAT Mode, 2016-02-23, seungmin.hong@lge.com*/
	if( awb_5k == awb_5k_checksum &&
		lsc_5k == lsc_5k_checksum ) {
		pr_err("EEPROM data verified\n");
		e_ctrl->is_supported = 0x01;
		rc = 0;
	} else {
		pr_err("awb_5k = 0x%x, awb_5k_checksum = 0x%x, "
			"lsc_5k = 0x%x, lsc_5k_checksum = 0x%x, "
			"sensor_id = 0x%x, sensor_id_checksum = 0x%x\n",
			awb_5k, awb_5k_checksum, lsc_5k, lsc_5k_checksum,
			sensor_id, sensor_id_checksum);
		e_ctrl->is_supported = 0x00;
	}
	return rc;
}

int32_t msm_eeprom_checksum_lgit_hi1332(struct msm_eeprom_ctrl_t *e_ctrl) {
	uint16_t awb_5k = 0, awb_5k_checksum = 0;
	uint16_t lsc_5k = 0, lsc_5k_checksum = 0;
	uint16_t sensor_id = 0, sensor_id_checksum = 0;
	int32_t rc = -EFAULT;
	int i;

	for(i = 0x0; i < 0x6; i++) {
		awb_5k += e_ctrl->cal_data.mapdata[i];
	}
	awb_5k_checksum = (e_ctrl->cal_data.mapdata[0x7] << 8) |
		e_ctrl->cal_data.mapdata[0x6];

	for(i = 0xC; i < 0x380; i++) {
		lsc_5k += e_ctrl->cal_data.mapdata[i];
	}
	lsc_5k_checksum = (e_ctrl->cal_data.mapdata[0x381] << 8) |
		e_ctrl->cal_data.mapdata[0x380];

/*LGE_CHANGE_S, Make EEPROM CheckSum Property for AAT Mode, 2016-02-23, seungmin.hong@lge.com*/
	if(!awb_5k_checksum || !lsc_5k_checksum){
		//Data does NOT exist
		pr_err("%s EEPROM CRC Data does NOT exist!\n", __func__);
		return rc;
	}
/*LGE_CHANGE_E, Make EEPROM CheckSum Property for AAT Mode, 2016-02-23, seungmin.hong@lge.com*/
	if( awb_5k == awb_5k_checksum &&
		lsc_5k == lsc_5k_checksum ) {
		pr_err("EEPROM data verified\n");
		e_ctrl->is_supported = 0x01;
		rc = 0;
	} else {
		pr_err("awb_5k = 0x%x, awb_5k_checksum = 0x%x, "
			"lsc_5k = 0x%x, lsc_5k_checksum = 0x%x, "
			"sensor_id = 0x%x, sensor_id_checksum = 0x%x\n",
			awb_5k, awb_5k_checksum, lsc_5k, lsc_5k_checksum,
			sensor_id, sensor_id_checksum);
		e_ctrl->is_supported = 0x00;
	}
	return rc;
}

int32_t msm_eeprom_checksum_lgit_imx258(struct msm_eeprom_ctrl_t *e_ctrl) {
	uint16_t awb_5k = 0, awb_5k_checksum = 0;
	uint16_t lsc_5k = 0, lsc_5k_checksum = 0;
	uint16_t sensor_id = 0, sensor_id_checksum = 0;
	int32_t rc = -EFAULT;
	int i;

	for(i = 0x0; i < 0x6; i++) {
		awb_5k += e_ctrl->cal_data.mapdata[i];
	}
	awb_5k_checksum = (e_ctrl->cal_data.mapdata[0x7] << 8) |
		e_ctrl->cal_data.mapdata[0x6];

	for(i = 0xC; i < 0x380; i++) {
		lsc_5k += e_ctrl->cal_data.mapdata[i];
	}
	lsc_5k_checksum = (e_ctrl->cal_data.mapdata[0x381] << 8) |
		e_ctrl->cal_data.mapdata[0x380];

/*LGE_CHANGE_S, Make EEPROM CheckSum Property for AAT Mode, 2016-02-23, seungmin.hong@lge.com*/
	if(!awb_5k_checksum || !lsc_5k_checksum){
		//Data does NOT exist
		pr_err("%s EEPROM CRC Data does NOT exist!\n", __func__);
		return rc;
	}
/*LGE_CHANGE_E, Make EEPROM CheckSum Property for AAT Mode, 2016-02-23, seungmin.hong@lge.com*/
	if( awb_5k == awb_5k_checksum &&
		lsc_5k == lsc_5k_checksum ) {
		pr_err("EEPROM data verified\n");
		e_ctrl->is_supported = 0x01;
		rc = 0;
	} else {
		pr_err("awb_5k = 0x%x, awb_5k_checksum = 0x%x, "
			"lsc_5k = 0x%x, lsc_5k_checksum = 0x%x, "
			"sensor_id = 0x%x, sensor_id_checksum = 0x%x\n",
			awb_5k, awb_5k_checksum, lsc_5k, lsc_5k_checksum,
			sensor_id, sensor_id_checksum);
		e_ctrl->is_supported = 0x00;
	}
	return rc;
}

int32_t msm_eeprom_checksum_lgit_hi842(struct msm_eeprom_ctrl_t *e_ctrl) {
	uint16_t awb_5k = 0, awb_5k_checksum = 0;
	uint16_t lsc_5k = 0, lsc_5k_checksum = 0;
	uint16_t sensor_id = 0, sensor_id_checksum = 0;
	int32_t rc = -EFAULT;
	int i;

	for(i = 0x0; i < 0x6; i++) {
		awb_5k += e_ctrl->cal_data.mapdata[i];
	}
	awb_5k_checksum = (e_ctrl->cal_data.mapdata[0x7] << 8) |
		e_ctrl->cal_data.mapdata[0x6];

	for(i = 0xC; i < 0x380; i++) {
		lsc_5k += e_ctrl->cal_data.mapdata[i];
	}
	lsc_5k_checksum = (e_ctrl->cal_data.mapdata[0x381] << 8) |
		e_ctrl->cal_data.mapdata[0x380];

/*LGE_CHANGE_S, Make EEPROM CheckSum Property for AAT Mode, 2016-02-23, seungmin.hong@lge.com*/
	if(!awb_5k_checksum || !lsc_5k_checksum){
		//Data does NOT exist
		pr_err("%s EEPROM CRC Data does NOT exist!\n", __func__);
		return rc;
	}
/*LGE_CHANGE_E, Make EEPROM CheckSum Property for AAT Mode, 2016-02-23, seungmin.hong@lge.com*/
	if( awb_5k == awb_5k_checksum &&
		lsc_5k == lsc_5k_checksum ) {
		pr_err("EEPROM data verified\n");
		e_ctrl->is_supported = 0x01;
		rc = 0;
	} else {
		pr_err("awb_5k = 0x%x, awb_5k_checksum = 0x%x, "
			"lsc_5k = 0x%x, lsc_5k_checksum = 0x%x, "
			"sensor_id = 0x%x, sensor_id_checksum = 0x%x\n",
			awb_5k, awb_5k_checksum, lsc_5k, lsc_5k_checksum,
			sensor_id, sensor_id_checksum);
		e_ctrl->is_supported = 0x00;
	}
	return rc;
}

int32_t msm_eeprom_checksum_lgit_imx234(struct msm_eeprom_ctrl_t *e_ctrl) {
	uint16_t awb_5k = 0, awb_5k_checksum = 0;
	uint16_t lsc_5k = 0, lsc_5k_checksum = 0;
	uint16_t sensor_id = 0, sensor_id_checksum = 0;
	int32_t rc = -EFAULT;
	int i;

	for(i = 0x0; i < 0x6; i++) {
		awb_5k += e_ctrl->cal_data.mapdata[i];
	}
	awb_5k_checksum = (e_ctrl->cal_data.mapdata[0x7] << 8) |
		e_ctrl->cal_data.mapdata[0x6];

	for(i = 0xC; i < 0x380; i++) {
		lsc_5k += e_ctrl->cal_data.mapdata[i];
	}
	lsc_5k_checksum = (e_ctrl->cal_data.mapdata[0x381] << 8) |
		e_ctrl->cal_data.mapdata[0x380];

/*LGE_CHANGE_S, Make EEPROM CheckSum Property for AAT Mode, 2016-02-23, seungmin.hong@lge.com*/
	if(!awb_5k_checksum || !lsc_5k_checksum){
		//Data does NOT exist
		pr_err("%s EEPROM CRC Data does NOT exist!\n", __func__);
		return rc;
	}
/*LGE_CHANGE_E, Make EEPROM CheckSum Property for AAT Mode, 2016-02-23, seungmin.hong@lge.com*/
	if( awb_5k == awb_5k_checksum &&
		lsc_5k == lsc_5k_checksum ) {
		pr_err("EEPROM data verified\n");
		e_ctrl->is_supported = 0x01;
		rc = 0;
	} else {
		pr_err("awb_5k = 0x%x, awb_5k_checksum = 0x%x, "
			"lsc_5k = 0x%x, lsc_5k_checksum = 0x%x, "
			"sensor_id = 0x%x, sensor_id_checksum = 0x%x\n",
			awb_5k, awb_5k_checksum, lsc_5k, lsc_5k_checksum,
			sensor_id, sensor_id_checksum);
		e_ctrl->is_supported = 0x00;
	}
	return rc;
}

int32_t msm_eeprom_checksum_m24c32_hi553(struct msm_eeprom_ctrl_t *e_ctrl)
{
	int32_t rc = -EFAULT;

	uint32_t dataSum_AWB_5K = 0;
	uint32_t dataSum_AWB_3K = 0;
	uint32_t dataSum_LSC_5K = 0;
	uint32_t dataSum_LSC_4K = 0;
	uint32_t dataSum_LSC_3K = 0;
	uint32_t trimSum_LSC_5K = 0;
	uint32_t trimSum_LSC_4K = 0;
	uint32_t trimSum_LSC_3K = 0;
	uint32_t dataSum_Total  = 0;

	uint32_t checkSum_AWB_5K, checkSum_AWB_3K;
	uint32_t checkSum_LSC_5K, checkSum_LSC_4K, checkSum_LSC_3K, checkSum_Total;

	//Add: DataSum
	dataSum_AWB_5K = accumulation(e_ctrl, AWB_5100K_START_ADDR, AWB_5100K_END_ADDR);
	dataSum_AWB_3K = accumulation(e_ctrl, AWB_3000K_START_ADDR, AWB_3000K_END_ADDR);
	dataSum_LSC_5K = accumulation(e_ctrl, LSC_5100K_START_ADDR, LSC_5100K_END_ADDR);
	dataSum_LSC_4K = accumulation(e_ctrl, LSC_4000K_START_ADDR, LSC_4000K_END_ADDR);
	dataSum_LSC_3K = accumulation(e_ctrl, LSC_3000K_START_ADDR, LSC_3000K_END_ADDR);
	dataSum_Total = accumulation(e_ctrl, DATA_START_ADDR, DATA_END_ADDR);

	//Trimming Overflow
	trimSum_LSC_5K = dataSum_LSC_5K & 0xFFFF;
	trimSum_LSC_4K = dataSum_LSC_4K & 0xFFFF;
	trimSum_LSC_3K = dataSum_LSC_3K & 0xFFFF;

	//Get: CheckSum
	checkSum_AWB_5K = shiftedSum(e_ctrl, AWB_5100K_CHECKSUM_LSB, AWB_5100K_CHECKSUM_MSB, LittleEndian);
	checkSum_AWB_3K = shiftedSum(e_ctrl, AWB_3000K_CHECKSUM_LSB, AWB_3000K_CHECKSUM_MSB, LittleEndian);
	checkSum_LSC_5K = shiftedSum(e_ctrl, LSC_5100K_CHECKSUM_LSB, LSC_5100K_CHECKSUM_MSB, LittleEndian);
	checkSum_LSC_4K = shiftedSum(e_ctrl, LSC_4000K_CHECKSUM_LSB, LSC_4000K_CHECKSUM_MSB, LittleEndian);
	checkSum_LSC_3K = shiftedSum(e_ctrl, LSC_3000K_CHECKSUM_LSB, LSC_3000K_CHECKSUM_MSB, LittleEndian);
	checkSum_Total  = shiftedSum(e_ctrl, TOTAL_CHECKSUM_START_ADDR, TOTAL_CHECKSUM_END_ADDR, LittleEndian);

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

#if 0
	//Check#5: LSC 3K Part
	if (trimSum_LSC_3K == checkSum_LSC_3K) {
		pr_err("[CHECK] LSC 3K Data Valid\n");
		e_ctrl->is_supported |= 0x10;
	}
	else {
		pr_err("[CHECK][FAIL] dataSum_LSC_3K = %d, trimSum_LSC_3K = %d, checkSum_LSC_3K = %d\n",
				dataSum_LSC_3K, trimSum_LSC_3K, checkSum_LSC_3K);
	}
#endif

	//Check#5: Total CheckSum
	if (dataSum_Total == checkSum_Total) {
		pr_err("[CHECK] Total CheckSum Valid\n");
		e_ctrl->is_supported |= 0x3F;
	}
	else {
		pr_err("[CHECK][FAIL] dataSum_Total = %u, checkSum_Total = %u\n",
				dataSum_Total, checkSum_Total);
	}

	pr_err("[CHECK][AFTER] e_ctrl->is_supported: %X\n", e_ctrl->is_supported);

	if(e_ctrl->is_supported == 0x3F) { //All bits are On
		pr_err("%s checksum succeed!\n", __func__);
		rc = 0;
	} else {
		//each bit (in e_ctrl->is_supported) indicates the checksum results.
	}

	return rc;
}

