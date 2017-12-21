//EEPROM MAP & CheckSum Code Refinement, Camera-Driver@lge.com, 2015-06-11
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/crc32.h>
#include "msm_sd.h"
#include "msm_cci.h"
#include "msm_eeprom.h"

int32_t msm_eeprom_checksum_imtech(struct msm_eeprom_ctrl_t *e_ctrl) {
	int32_t rc = -EFAULT;

	if(e_ctrl->cal_data.num_data < 0x770) {
		pr_err("%s num_data = %d\n", __func__, e_ctrl->cal_data.num_data);
		return rc;
	}

	if(!strncmp("hi841", e_ctrl->eboard_info->eeprom_name, 5)) {
		rc = msm_eeprom_checksum_imtech_hi841(e_ctrl);
	}
	else if(!strncmp("t4kb3", e_ctrl->eboard_info->eeprom_name, 5)) {
		rc = msm_eeprom_checksum_imtech_t4kb3(e_ctrl);
	}
	else if(!strncmp("ov8858", e_ctrl->eboard_info->eeprom_name, 6)) {
		rc = msm_eeprom_checksum_imtech_ov8858(e_ctrl);
	}
	else {
		pr_err("%s vendor = imtech, name = %s\n", __func__, e_ctrl->eboard_info->eeprom_name);
	}

	return rc;
}

//LGE_CHANGE_S, S5K5E2 EEPROM CHECKSUM, 2015-09-18
int32_t msm_s5k5e2_eeprom_checksum(struct msm_eeprom_ctrl_t *e_ctrl)
{
	uint32_t CheckSum = 0;
	uint32_t VCM_Checksum = 0;
	uint32_t Total_CheckSum = 0;
	uint32_t DataSum = 0;
	int i = 0;
	int32_t rc = -EFAULT;

	const int AWB_5100K_START_ADDR   = 0x0000;
	const int AWB_5100K_END_ADDR     = 0x0005;
	const int LSC_5100K_START_ADDR   = 0x000C;
	const int LSC_5100K_END_ADDR     = 0x037F;
	const int VCM_5100K_START_ADDR	 = 0x0390;
	const int VCM_5100K_END_ADDR	 = 0x0397;

	const int CHECKSUM_5100K_ADDR1     = 0x0380;
	const int CHECKSUM_5100K_ADDR2     = 0x0381;
	const int CHECKSUM_5100K_ADDR3     = 0x0382;
	const int CHECKSUM_5100K_ADDR4     = 0x0383;
	const int CHECKSUM_VCM_ADDR1     = 0x0398;
	const int CHECKSUM_VCM_ADDR2	 = 0x0399;

	//calculate LSC, AWB checksum
	CheckSum =  (e_ctrl->cal_data.mapdata[CHECKSUM_5100K_ADDR1] << 24)
			+ (e_ctrl->cal_data.mapdata[CHECKSUM_5100K_ADDR2] << 16)
			+ (e_ctrl->cal_data.mapdata[CHECKSUM_5100K_ADDR3] << 8)
			+  e_ctrl->cal_data.mapdata[CHECKSUM_5100K_ADDR4];
	//calculate VCM checksum
	VCM_Checksum = (e_ctrl->cal_data.mapdata[CHECKSUM_VCM_ADDR1] << 8)
			+  e_ctrl->cal_data.mapdata[CHECKSUM_VCM_ADDR2];

	//AWB datasum
	for( i = AWB_5100K_START_ADDR; i <= AWB_5100K_END_ADDR; i=i+2 ) {
		DataSum  += (e_ctrl->cal_data.mapdata[i] << 8)| e_ctrl->cal_data.mapdata[i+1];
	}
	//LSC datasum
	for( i = LSC_5100K_START_ADDR; i <= LSC_5100K_END_ADDR; i++ ) {
		DataSum  += e_ctrl->cal_data.mapdata[i];
	}

	//VCM datasum
	for( i = VCM_5100K_START_ADDR; i <= VCM_5100K_END_ADDR; i=i+2 ) {
		DataSum  += (e_ctrl->cal_data.mapdata[i] << 8)| e_ctrl->cal_data.mapdata[i+1];
	}

	Total_CheckSum = CheckSum + VCM_Checksum;
	pr_err("[CHECK] Total_CheckSum: 0x%04x, DataSum: 0x%04x\n", Total_CheckSum, DataSum);

	if(VCM_Checksum == 0) {
		//Data does NOT exist
		pr_err("%s S5K5E2 EEPROM NOT Supported for VCM!\n", __func__);
		return rc;
	}else if (CheckSum == 0) {
		//Data does NOT exist
		pr_err("%s S5K5E2 EEPROM NOT Supported for 5100K!\n", __func__);
		return rc;
	}

	if( Total_CheckSum != DataSum ) {
		//data exist, But CheckSum Failed!
		pr_err("%s S5K5E2 EEPROM CheckSum error !\n", __func__);
	} else {
		pr_err("%s [CHECK] S5K5E2 EEPROM Total CheckSum Valid\n", __func__);
		e_ctrl->is_supported |= 0x1F;
		pr_err("%s checksum succeed!\n", __func__);
		rc = 0;
	}

	return rc;
}
//LGE_CHANGE_E, S5K5E2 EEPROM CHECKSUM, 2015-09-18

int32_t msm_eeprom_checksum_ofilm(struct msm_eeprom_ctrl_t *e_ctrl) {
	int32_t rc = -EFAULT;

	if(e_ctrl->cal_data.num_data < 0x770) {
		pr_err("%s num_data = %d\n", __func__, e_ctrl->cal_data.num_data);
		return rc;
	}

	if(!strncmp("ofilm_hi1332", e_ctrl->eboard_info->eeprom_name, 12)) {
		rc = msm_eeprom_checksum_ofilm_hi1332(e_ctrl);
	} else {
		pr_err("%s vendor = O-Film,  name = %s\n", __func__, e_ctrl->eboard_info->eeprom_name);
	}
	return rc;
}

int32_t msm_eeprom_checksum_sunny(struct msm_eeprom_ctrl_t *e_ctrl) {
	int32_t rc = -EFAULT;

	if(e_ctrl->cal_data.num_data < 0x770) {
		pr_err("%s num_data = %d\n", __func__, e_ctrl->cal_data.num_data);
		return rc;
	}

	if(!strncmp("ov13850", e_ctrl->eboard_info->eeprom_name, 7)) {
		rc = msm_eeprom_checksum_sunny_ov13850(e_ctrl);
	} else if(!strncmp("sunny_hi1332", e_ctrl->eboard_info->eeprom_name, 12)) {
		rc = msm_eeprom_checksum_sunny_hi1332(e_ctrl);
	} else {
		pr_err("%s vendor = Sunny, name = %s\n", __func__, e_ctrl->eboard_info->eeprom_name);
	}
	return rc;
}

int32_t msm_eeprom_checksum_lgit(struct msm_eeprom_ctrl_t *e_ctrl) {
	int32_t rc = -EFAULT;
	uint8_t eeprom_ver = 0xff;

	if(e_ctrl->cal_data.num_data < 0x770) {
		pr_err("%s num_data = %d\n", __func__, e_ctrl->cal_data.num_data);
		return rc;
	}

	eeprom_ver = e_ctrl->cal_data.mapdata[0x770];
	pr_err("%s eeprom_ver = 0x%02X\n", __func__, eeprom_ver);

	switch(eeprom_ver) {
		case 0x0d:
			if(!strncmp("t4ka3", e_ctrl->eboard_info->eeprom_name, 5)) {
				rc = msm_eeprom_checksum_lgit_v0d_t4ka3(e_ctrl);
			}
			else {
				rc = msm_eeprom_checksum_lgit_v0d(e_ctrl);
			}
			break;
		case 0xff:
			if(!strncmp("at24c16d", e_ctrl->eboard_info->eeprom_name, 8)) {
				rc = msm_eeprom_checksum_lgit_at24c16d(e_ctrl);
			} else if(!strncmp("lgit_hi1332", e_ctrl->eboard_info->eeprom_name, 11)) {
				rc = msm_eeprom_checksum_lgit_hi1332(e_ctrl);
			} else if(!strncmp("lgit_imx258", e_ctrl->eboard_info->eeprom_name, 11)) {
				rc = msm_eeprom_checksum_lgit_imx258(e_ctrl);
			} else if(!strncmp("hi842", e_ctrl->eboard_info->eeprom_name, 5)) {
				rc = msm_eeprom_checksum_lgit_hi842(e_ctrl);
			} else if(!strncmp("lgit_imx234", e_ctrl->eboard_info->eeprom_name, 11)) {
				rc = msm_eeprom_checksum_lgit_imx234(e_ctrl);
			} else if(!strncmp("m24c32_hi553", e_ctrl->eboard_info->eeprom_name, 11)) {
				rc = msm_eeprom_checksum_m24c32_hi553(e_ctrl);
			} else {
				rc = -EFAULT;
				pr_err("%s failed !!! no matching sensors for eeprom ver = 0x%x\n", __func__, eeprom_ver);
			}
			break;
		default:
			pr_info("eeprom ver = 0x%x\n", eeprom_ver);
			break;
	}
	return rc;
}

int32_t msm_eeprom_checksum_cowell(struct msm_eeprom_ctrl_t *e_ctrl)
{
	int32_t rc = -EFAULT;

	//Cowell Module have not set EEPROM VERSION [0x770] yet (2015-06-11)
	if(!strncmp("hi841", e_ctrl->eboard_info->eeprom_name, 5)) {
		rc = msm_eeprom_checksum_cowell_hi841(e_ctrl);
	}
	else if(!strncmp("zc533", e_ctrl->eboard_info->eeprom_name, 5)) {
		if (e_ctrl->cal_data.mapdata[0x700] == 0x13) {
			rc = msm_eeprom_checksum_cowell_hi842(e_ctrl);
		}
		else {
			rc = msm_eeprom_checksum_cowell_mn34153(e_ctrl);
		}
	}
	else {
		pr_err("%s failed to identifying eeprom version\n", __func__);
		pr_err("%s vendor = cowell, name = %s\n", __func__, e_ctrl->eboard_info->eeprom_name);
	}

	return rc;
}

// Helper function for shifted add
uint32_t shiftedSum(struct msm_eeprom_ctrl_t *e_ctrl, uint32_t startAddr, uint32_t endAddr, Endian endian)
{
	int addr = 0;
	int sum = 0;

	//input validataion
	int diff = endAddr - startAddr;
	if (diff > 4 || diff < -4) {
		pr_err("%s faild: exceed 32bit numbers\n", __func__);
		return 0;
	}

	//cumulative addition with shifting
	if (endian == BigEndian) {
		for (addr = startAddr; addr <= endAddr; addr++) {
			sum = (sum << 8) + (e_ctrl->cal_data.mapdata[addr]);
			//pr_err("[CHECK] e_ctrl->cal_data.mapdata[0x%04X]: 0x%04X\n",
			//			addr, e_ctrl->cal_data.mapdata[addr]);
		}
	}
	else { //LittleEndian
		for (addr = endAddr; addr >= startAddr; addr--) {
			sum = (sum << 8) + (e_ctrl->cal_data.mapdata[addr]);
			//pr_err("[CHECK] e_ctrl->cal_data.mapdata[0x%04X]: 0x%04X\n",
			//			addr, e_ctrl->cal_data.mapdata[addr]);
		}
	}

	return sum;
}

uint32_t accumulation(struct msm_eeprom_ctrl_t *e_ctrl, uint32_t startAddr, uint32_t endAddr)
{
	int addr = 0;
	int sum = 0;

	//input validataion
	int diff = endAddr - startAddr;
	if (diff < 0) {
		pr_err("%s failed: you must set (endAddr >= startAddr)\n", __func__);
		return 0;
	}

	//cumulative addition with shifting
	for (addr = startAddr; addr <= endAddr; addr++) {
		sum += (e_ctrl->cal_data.mapdata[addr]);
		//pr_err("[CHECK] e_ctrl->cal_data.mapdata[0x%04X]: 0x%04X\n",
		//			addr, e_ctrl->cal_data.mapdata[addr]);
	}

	return sum;
}
