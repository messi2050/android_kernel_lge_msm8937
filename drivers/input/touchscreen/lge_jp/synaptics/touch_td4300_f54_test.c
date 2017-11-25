/* +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   Copyright ?2012 Synaptics Incorporated. All rights reserved.

   The information in this file is confidential under the terms
   of a non-disclosure agreement with Synaptics and is provided
   AS IS.

   The information in this file shall remain the exclusive property
   of Synaptics and may be the subject of Synaptics?patents, in
   whole or part. Synaptics?intellectual property rights in the
   information in this file are not expressly or implicitly licensed
   or otherwise transferred to you as a result of such information
   being made available to you.
   +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */

/* FullRawCapacitance Support 0D button */
#define TS_MODULE "[refcode_f54]"

/*
 *  Include to touch core Header File
 */
#include <touch_hwif.h>
#include <touch_core.h>

/*
 *  Include to Local Header File
 */
#include "touch_td4300.h"
#include "touch_td4300_prd.h"
#include "touch_td4300_f54_test.h"
#include "touch_td4300_test_limits.h"

const int DefaultTimeout = 30;	/* In counts */

int pageNum;
int scanMaxPageCount = 5;
int input;
int MaxArrayLength;

unsigned char ButtonTx[8];
unsigned char ButtonRx[8];
unsigned char ButtonCount;

unsigned char mask;

unsigned char F01DataBase;
unsigned char F01QueryBase;
unsigned char F01ControlBase;
unsigned char F01CommandBase;

unsigned char F51DataBase;
unsigned char F51QueryBase;
unsigned char F51ControlBase;
unsigned char F51CommnadBase;

unsigned char F54DataBase;
unsigned char F54QueryBase;
unsigned char F54ControlBase;
unsigned char F54CommandBase;

unsigned char F55DataBase;
unsigned char F55QueryBase;
unsigned char F55ControlBase;
unsigned char F55DataBase;

unsigned char RxChannelCount;
unsigned char TxChannelCount;
unsigned char TouchControllerFamily;
unsigned char CurveCompensationMode;
unsigned char NumOfSensingFreq;

bool bHaveF01;
bool bHaveF11;
bool bHaveF1A;
bool bHaveF12;
bool bHaveF34;
bool bHaveF54;
bool bHaveF55;

bool SignalClarityOn;
bool bHavePixelTouchThresholdTuning;
bool bHaveInterferenceMetric;
bool bHaveRelaxationControl;
bool bHaveSensorAssignment;
bool bHaveSenseFrequencyControl;
bool bHaveFirmwareNoiseMitigation;
bool bHaveIIRFilter;
bool bHaveCmnRemoval;
bool bHaveCmnMaximum;
bool bHaveTouchHysteresis;
bool bHaveEdgeCompensation;
bool bHavePerFrequencyNoiseControl;
bool bHaveSignalClarity;
bool bHaveMultiMetricStateMachine;
bool bHaveVarianceMetric;
bool bHave0DRelaxationControl;
bool bHave0DAcquisitionControl;
bool bHaveSlewMetric;
bool bHaveHBlank;
bool bHaveVBlank;
bool bHaveLongHBlank;
bool bHaveNoiseMitigation2;
bool bHaveSlewOption;
bool bHaveEnhancedStretch;
bool bHaveStartupFastRelaxation;
bool bHaveESDControl;
bool bHaveEnergyRatioRelaxation;
bool ButtonShared;
bool bIncellDevice;

bool bHaveCtrl11;
bool bHaveCtrl86;
bool bHaveCtrl87;
bool bHaveCtrl88;
bool bHaveCtrl89;
bool bHaveCtrl90;
bool bHaveCtrl91;
bool bHaveCtrl92;
bool bHaveCtrl93;
bool bHaveCtrl94;
bool bHaveCtrl95;
bool bHaveCtrl96;
bool bHaveCtrl97;
bool bHaveCtrl98;
bool bHaveCtrl99;
bool bHaveCtrl100;
bool bHaveCtrl101;
bool bHaveCtrl102;

bool bHaveF54Query13;
bool bHaveF54Query15;
bool bHaveF54Query16;
bool bHaveF54Query17;
bool bHaveF54Query18;
bool bHaveF54Query19;
bool bHaveF54Query20;
bool bHaveF54Query21;
bool bHaveF54Query22;
bool bHaveF54Query23;
bool bHaveF54Query24;
bool bHaveF54Query25;
bool bHaveF54Query26;
bool bHaveF54Query27;
bool bHaveF54Query28;
bool bHaveF54Query29;
bool bHaveF54Query30;
bool bHaveF54Query31;
bool bHaveF54Query32;
bool bHaveF54Query33;
bool bHaveF54Query34;
bool bHaveF54Query35;
bool bHaveF54Query36;
bool bHaveF54Query37;
bool bHaveF54Query38;
bool bHaveF54Query39;
bool bHaveF54Query40;
bool bHaveF54Query41;
bool bHaveF54Query42;
bool bHaveF54Query43;
bool bHaveF54Query44;
bool bHaveF54Query45;
bool bHaveF54Query46;
bool bHaveF54Query47;
bool bHaveF54Query48;
bool bHaveF54Query49;
bool bHaveF54Query50;
bool bHaveF54Query51;
bool bHaveF54Query52;
bool bHaveF54Query53;
bool bHaveF54Query54;
bool bHaveF54Query55;

bool bHaveF54Ctrl07;
bool bHaveF54Ctrl41;
bool bHaveF54Ctrl57;
bool bHaveF54Ctrl103;
bool bHaveF54Ctrl104;
bool bHaveF54Ctrl105;
bool bHaveF54Ctrl106;
bool bHaveF54Ctrl107;
bool bHaveF54Ctrl108;
bool bHaveF54Ctrl109;
bool bHaveF54Ctrl110;
bool bHaveF54Ctrl111;
bool bHaveF54Ctrl112;
bool bHaveF54Ctrl113;
bool bHaveF54Ctrl114;
bool bHaveF54Ctrl115;
bool bHaveF54Ctrl116;
bool bHaveF54Ctrl117;
bool bHaveF54Ctrl118;
bool bHaveF54Ctrl119;
bool bHaveF54Ctrl120;
bool bHaveF54Ctrl121;
bool bHaveF54Ctrl122;
bool bHaveF54Ctrl123;
bool bHaveF54Ctrl124;
bool bHaveF54Ctrl125;
bool bHaveF54Ctrl126;
bool bHaveF54Ctrl127;
bool bHaveF54Ctrl128;
bool bHaveF54Ctrl129;
bool bHaveF54Ctrl130;
bool bHaveF54Ctrl131;
bool bHaveF54Ctrl132;
bool bHaveF54Ctrl133;
bool bHaveF54Ctrl134;
bool bHaveF54Ctrl135;
bool bHaveF54Ctrl136;
bool bHaveF54Ctrl137;
bool bHaveF54Ctrl138;
bool bHaveF54Ctrl139;
bool bHaveF54Ctrl140;
bool bHaveF54Ctrl141;
bool bHaveF54Ctrl142;
bool bHaveF54Ctrl143;
bool bHaveF54Ctrl144;
bool bHaveF54Ctrl145;
bool bHaveF54Ctrl146;
bool bHaveF54Ctrl147;
bool bHaveF54Ctrl148;
bool bHaveF54Ctrl149;
bool bHaveF54Ctrl150;
bool bHaveF54Ctrl151;
bool bHaveF54Ctrl152;
bool bHaveF54Ctrl153;
bool bHaveF54Ctrl154;
bool bHaveF54Ctrl155;
bool bHaveF54Ctrl156;
bool bHaveF54Ctrl157;
bool bHaveF54Ctrl158;
bool bHaveF54Ctrl159;
bool bHaveF54Ctrl160;
bool bHaveF54Ctrl161;
bool bHaveF54Ctrl162;
bool bHaveF54Ctrl163;
bool bHaveF54Ctrl164;
bool bHaveF54Ctrl165;
bool bHaveF54Ctrl166;
bool bHaveF54Ctrl167;
bool bHaveF54Ctrl168;
bool bHaveF54Ctrl169;
bool bHaveF54Ctrl170;
bool bHaveF54Ctrl171;
bool bHaveF54Ctrl172;
bool bHaveF54Ctrl173;
bool bHaveF54Ctrl174;
bool bHaveF54Ctrl175;
bool bHaveF54Ctrl176;
bool bHaveF54Ctrl177;
bool bHaveF54Ctrl178;
bool bHaveF54Ctrl179;
bool bHaveF54Ctrl180;
bool bHaveF54Ctrl181;
bool bHaveF54Ctrl182;
bool bHaveF54Ctrl183;
bool bHaveF54Ctrl184;
bool bHaveF54Ctrl185;
bool bHaveF54Ctrl186;
bool bHaveF54Ctrl187;
bool bHaveF54Ctrl188;
bool bHaveF54Ctrl189;
bool bHaveF54Ctrl198;

unsigned char F1AControlBase;
unsigned char F12ControlBase;
unsigned char F12QueryBase;
unsigned char F12_2DTxCount;
unsigned char F12_2DRxCount;
unsigned char F12Support;
unsigned char F12ControlRegisterPresence;

unsigned char F54Ctrl07Offset;
unsigned char F54Ctrl41Offset;
unsigned char F54Ctrl57Offset;
unsigned char F54Ctrl88Offset;
unsigned char F54Ctrl89Offset;
unsigned char F54Ctrl91Offset;
unsigned char F54Ctrl96Offset;
unsigned char F54Ctrl98Offset;
unsigned char F54Ctrl102Offset;
unsigned char F54Ctrl149Offset;
unsigned char F54Ctrl188Offset;
unsigned char F54Ctrl189Offset;
unsigned char F54Query54Sub0;
unsigned char F54Query54Sub1;
unsigned char F54Ctrl198_00_Size;
unsigned char F54Ctrl198_01_Size;

/* Assuming Tx = 32 & Rx = 32 to accommodate any configuration */
short Image1[TRX_MAX][TRX_MAX];
int ImagepF[TRX_MAX][TRX_MAX];
int RXROE[TRX_MAX];
int TXROE[TRX_MAX];
int AbsSigned32Data[TRX_mapping_max];
unsigned char AbsADCRangeData[TRX_mapping_max];
unsigned char Data[TRX_MAX * TRX_MAX * 4];
unsigned char TRxPhysical[TRX_mapping_max];
unsigned char TRxPhysical_bit[TRX_BITMAP_LENGTH];
unsigned char err_array[7] = {0};

unsigned char TRX_Open[7] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
unsigned char TRX_Gnd[7] = {0xff, 0xff, 0xff, 0xff, 0x3, 0xff, 0xfc};
unsigned char TRX_Short[7] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
int HighResistanceLowerLimit[3] = {-1000, -1000, -400};
int HighResistanceUpperLimit[3] = {450, 700, 20};
unsigned int AbsShort[TRX_MAX * 2] = {0};
unsigned int AbsOpen[TRX_MAX * 2] = {0};
int AbsTxShortLimit = 0;
int AbsRxShortLimit = 0;
int AbsTxOpenLimit = 0;
int AbsRxOpenLimit = 1000;
int AbsRawRef[16] = {77, 11919, 14023, 15163, 16192, 18319, 19337, 21491,
	22633, 24692, 26853, 27993, 30147, 32253, 34411, 37605};
short NoiseDeltaMin[TRX_MAX][TRX_MAX];
short NoiseDeltaMax[TRX_MAX][TRX_MAX];
short NoiseLimitLow = -8;
short NoiseLimitHigh = 8;

struct timeval t_interval[TIME_PROFILE_MAX];
static int f54len;
static int wlen;
char f54buf[BUF_SIZE] = {0};
char wbuf[BUF_SIZE] = {0};

/* Function to switch beteen register pages */
bool switchPage(struct device *dev, int page)
{
	unsigned char values[1] = {0};
	unsigned char data = 0;
	unsigned int count = 0;

	pageNum = values[0] = page;

	do {
		Write8BitRegisters(dev, 0xFF, values, 1);
		touch_msleep(20);
		Read8BitRegisters(dev, 0xFF, &data, 1);
		count++;
	} while ((int)data != page && (count < DefaultTimeout));

	if (count >= DefaultTimeout) {
		TOUCH_E("Timeout -- Page switch fail !\n");
		return -EAGAIN;
	}

	return true;
}

void Reset(struct device *dev)
{
	unsigned char data;

	switchPage(dev, 0x00);

	data = 0x01;
	Write8BitRegisters(dev, F01CommandBase, &data, 1);

	touch_msleep(300);
}

/* Compare Report type #20 data against test limits */
int CompareImageReport(void)
{
	bool result = true;
	int i, j;

	/* Compare 0D area */
	if (ButtonCount > 0) {
		for (i = 0; i < ButtonCount; i++) {
			if ((ImagepF[TxChannelCount - 1][F12_2DRxCount + i] <
			LowerImageLimit[TxChannelCount - 1][F12_2DRxCount + i])
			|| (ImagepF[TxChannelCount - 1][F12_2DRxCount + i] >
			UpperImageLimit[TxChannelCount - 1][F12_2DRxCount + i])) {
				result = false;
				break;
			}
		}

	}

	/* Compare 2D area */
	for (j = 0; j < (int)F12_2DRxCount; j++) {
		for (i = 0; i < (int)F12_2DTxCount; i++) {
			if ((ImagepF[i][j] < LowerImageLimit[i][j])
					|| (ImagepF[i][j] >
						UpperImageLimit[i][j]))	{
				result = false;
				f54len += snprintf(f54buf + f54len,
							sizeof(f54buf) - f54len,
							"FAIL, Tx[%d] Rx[%d] = %d\n",
							i, j, ImagepF[i][j]);
			}
		}
	}

	if (result) {
		TOUCH_I("Full Raw Capacitance Test passed.\n");
		f54len += snprintf(f54buf + f54len, sizeof(f54buf) - f54len,
				"\nFull Raw Capacitance Image Test passed.\n\n");
	} else {
		TOUCH_I("Full Raw Capacitance Test failed.\n");
		f54len += snprintf(f54buf + f54len, sizeof(f54buf) - f54len,
				"\nFull Raw Capacitance Image Test failed.\n\n");
	}

	return (result) ? 1 : 0;
}

/* Compare Report type #4 data against test limits */
int CompareHighResistance(int maxRxpF, int maxTxpF, int minpF)
{
	bool result = true;

	if (maxRxpF > HighResistanceUpperLimit[0]
			|| maxRxpF < HighResistanceLowerLimit[0])
		result = false;

	if (maxTxpF > HighResistanceUpperLimit[1]
			|| maxTxpF < HighResistanceLowerLimit[1])
		result = false;

	if (minpF > HighResistanceUpperLimit[2]
			|| minpF < HighResistanceLowerLimit[2])
		result = false;

	if (result == false) {
		TOUCH_I("HighResistance Test failed.\n");
		f54len += snprintf(f54buf + f54len, sizeof(f54buf) - f54len,
				"\nHighResistance Test failed.\n\n");
	} else {
		TOUCH_I("HighResistance Test passed.\n");
		f54len += snprintf(f54buf + f54len, sizeof(f54buf) - f54len,
				"\nHighResistance Test passed.\n\n");
	}

	return (result) ? 1 : 0;
}


/* Compare Report type #22 data against test limits */
int CompareSensorSpeedReport(void)
{
	bool result = true;
	int i, j = 0;

	for (i = 0; i < (int)F12_2DTxCount; i++) {
		for (j = 0; j < (int)F12_2DRxCount; j++) {
			if ((ImagepF[i][j] < SensorSpeedLowerImageLimit[i][j])
			|| (ImagepF[i][j] > SensorSpeedUpperImageLimit[i][j])) {
				result = false;
				TOUCH_I("Fail, Tx[%d], Rx[%d] = %d\n",
					i, j, ImagepF[i][j]);
				wlen += snprintf(wbuf + wlen, sizeof(wbuf) - wlen,
						"Fail, Tx[%d], Rx[%d] = %d\n",
						i, j, ImagepF[i][j]);
				break;
			}
		}
	}

	if (result) {
		TOUCH_I("Sensor Speed Test passed.\n");
		wlen += snprintf(wbuf + wlen, sizeof(wbuf) - wlen,
				"\nSensor Speed Test passed.\n\n");
	} else {
		TOUCH_I("Sensor Speed Test failed.\n");
		wlen += snprintf(wbuf + wlen, sizeof(wbuf) - wlen,
				"\nSensor Speed Test failed.\n\n");
	}

	return (result) ? 1 : 0;
}

/* Compare Report type #23 data against test limits */
int CompareADCReport(void)
{
	bool result = true;
	int i, j = 0;

	for (i = 0; i < (int)F12_2DTxCount; i++) {
		for (j = 0; j < (int)F12_2DRxCount; j++) {
			if ((Image1[i][j] < ADCLowerImageLimit[i][j]) ||
				(Image1[i][j] > ADCUpperImageLimit[i][j])) {
				result = false;
				TOUCH_I("Fail, Tx[%d], Rx[%d] = %d\n",
					i, j, Image1[i][j]);
				wlen += snprintf(wbuf + wlen, sizeof(wbuf) - wlen,
						"Fail, Tx[%d], Rx[%d] = %d\n",
						i, j, Image1[i][j]);
				break;
			}
		}
	}

	if (result) {
		TOUCH_I("ADC Range Test passed.\n");
		wlen += snprintf(wbuf + wlen, sizeof(wbuf) - wlen,
				"\nADC Range Test passed.\n\n");
	} else {
		TOUCH_I("ADC Range Test failed.\n");
		wlen += snprintf(wbuf + wlen, sizeof(wbuf) - wlen,
				"\nADC Range Test failed.\n\n");
	}

	return (result) ? 1 : 0;
}

void CompareAbsADCRangeReport(void)
{
	bool result = true;
	int i = 0;

	for (i = 0; i < RxChannelCount + F12_2DTxCount; i++) {
		if (i == F12_2DRxCount)
			i = RxChannelCount;
		if ((AbsADCRangeData[i] < LOWER_ABS_ADC_RANGE_LIMIT)
				|| (AbsADCRangeData[i] >
					UPPER_ABS_ADC_RANGE_LIMIT)) {
			result = false;
			break;
		}
	}

	if (result)
		TOUCH_I("\nAbs Sensing ADC Range Test Passed.\n");
	else
		TOUCH_I("\nAbs Sensing ADC Range Test Failed.\n");
}

int CompareAbsRawReport(u16 input)
{
	bool result = true;
	int i = 0;
	int lower_limit = 0;
	int upper_limit = 0;

	if (input == eRT_AbsRaw) {
		lower_limit = LOWER_ABS_RAW_CAP_LIMIT;
		upper_limit = UPPER_ABS_RAW_CAP_LIMIT;
	} else {
		lower_limit = LOWER_ABS_LPWG_RAW_CAP_LIMIT;
		upper_limit = UPPER_ABS_LPWG_RAW_CAP_LIMIT;
	}

	for (i = 0; i < RxChannelCount + F12_2DTxCount; i++) {
		if (i == F12_2DRxCount)
			i = RxChannelCount;
		if (i == 30)
			break;
		if ((AbsSigned32Data[i] < lower_limit)
			|| (AbsSigned32Data[i] > upper_limit)) {
			result = false;
			break;
		}
	}

	if (result == false) {
		if (input == eRT_AbsRaw) {
			TOUCH_I("Abs Rawdata Test Failed.\n");
			f54len += snprintf(f54buf + f54len, sizeof(f54buf) - f54len,
					"\nAbs Rawdata Test Failed.\n\n");
		} else {
			TOUCH_I("Abs LPWG Rawdata Test Failed.\n");
			f54len += snprintf(f54buf + f54len, sizeof(f54buf) - f54len,
					"\nAbs LPWG Rawdata Test Failed.\n\n");
		}
	} else {
		if (input == eRT_AbsRaw) {
			TOUCH_I("Abs Rawdata Test Passed.\n");
			f54len += snprintf(f54buf + f54len, sizeof(f54buf) - f54len,
					"\nAbs Rawdata Test Passed.\n\n");
		} else {
			TOUCH_I("Abs LPWG Rawdata Test Passed.\n");
			f54len += snprintf(f54buf + f54len, sizeof(f54buf) - f54len,
					"\nAbs LPWG Rawdata Test Passed.\n\n");
		}
	}

	return (result) ? 1 : 0;
}

int CompareAbsOpen(void)
{
	bool result = true;
	int i = 0;

	for (i = 0; i < ((int)F12_2DRxCount + (int)F12_2DTxCount); i++) {
		if (i < (int)F12_2DRxCount) {
			if (AbsOpen[i] <= AbsRxOpenLimit) {
				result = false;
				TOUCH_I("Fail, Rx[%d] = %d\n", i, AbsOpen[i]);
			}

		} else {
			if (AbsOpen[i] <= AbsTxOpenLimit) {
				result = false;
				TOUCH_I("Fail, Tx[%d] = %d\n",
					i - (int)F12_2DRxCount, AbsOpen[i]);
			}
		}

	}

	TOUCH_I("AbsRxOpenLimit:  %d  AbsTxOpenLimit : %d\n",
			AbsRxOpenLimit, AbsTxOpenLimit);

	if (result) {
		TOUCH_I("Abs Sensing Open Test Passed.\n");
		f54len += snprintf(f54buf + f54len, sizeof(f54buf) - f54len,
				"Abs Sensing Open Test passed.\n\n");
	} else {
		TOUCH_I("Abs Sensing Open Test Failed.\n");
		f54len += snprintf(f54buf + f54len, sizeof(f54buf) - f54len,
				"Abs Sensing Open Test failed.\n\n");
	}

	return (result) ? 1 : 0;
}

int CompareAbsShort(void)
{
	bool result = true;
	int i = 0;

	for (i = 0; i < ((int)F12_2DRxCount + (int)F12_2DTxCount); i++) {
		if (i < (int)F12_2DRxCount) {
			if (AbsShort[i] >= AbsRxShortLimit) {
				result = false;
				TOUCH_I("Fail, Rx[%d] = %d\n", i, AbsShort[i]);
			}

		} else {
			if (AbsShort[i] >= AbsTxShortLimit) {
				result = false;
				TOUCH_I("Fail, Tx[%d] = %d\n",
					i - (int)F12_2DRxCount, AbsShort[i]);
			}

		}

	}

	TOUCH_I("AbsRxShortLimit:  %d  AbsTxShortLimit : %d\n",
			AbsRxShortLimit , AbsTxShortLimit);

	if (result) {
		TOUCH_I("Abs Sensing Short Test Passed.\n");
		f54len += snprintf(f54buf + f54len, sizeof(f54buf) - f54len,
				"Abs Sensing Short Test passed.\n\n");
	} else {
		TOUCH_I("Abs Sensing Short Test Failed.\n");
		f54len += snprintf(f54buf + f54len, sizeof(f54buf) - f54len,
				"Abs Sensing Short Test failed.\n\n");
	}

	return (result) ? 1 : 0;

}

/* Compare Report type #24 data against test limits */
void CompareTRexOpenTestReport(void)
{
	bool result = true;
	int i, j = 0;
	int buf_1, buf_2 = 0;

	for (i = 0; i < TRX_BITMAP_LENGTH; i++) {
		for (j = 0; j < 8; j++) {
			buf_1 = Data[i] & (0x01 << j);
			buf_2 = TRX_Open[i] & (0x01 << j);
			if (buf_1 != buf_2) {
				result = false;
				wlen += snprintf(wbuf + wlen, sizeof(wbuf) - wlen,
						"Fail, TRx[%d]\n", ((i * 8) + j));
			}
		}
	}

	if (result) {
		TOUCH_I("TRex-TRex Open Test passed.\n");
		wlen += snprintf(wbuf + wlen, sizeof(wbuf) - wlen,
				"TRex-TRex Open Test passed.\n\n");
	} else {
		TOUCH_I("TRex-TRex Open Test failed.\n");
		wlen += snprintf(wbuf + wlen, sizeof(wbuf) - wlen,
				"TRex-TRex Open Test failed.\n\n");
	}
}

/* Compare Report type #25 data against test limits */
int CompareTRexGroundTestReport(void)
{
	bool result = true;
	int i, j, k = 0;

	for (i = 0; i < TRX_BITMAP_LENGTH; i++) {
		Data[i] &= TRxPhysical_bit[i];
		if (Data[i] != TRX_Short[i]) {
			err_array[i] = Data[i] ^ TRX_Short[i];
			result = false;
		}
	}

	for (i = 0; i < TRX_BITMAP_LENGTH; i++) {
		if (Data[i] != TRX_Gnd[i]) {
			err_array[i] = Data[i] ^ TRX_Gnd[i];
			result = false;
		}
	}

	for (i = 0; i < TRX_BITMAP_LENGTH; i++) {
		for (j = 0; j < 8; j++) {
			k = 0x01 << j;
			if (err_array[i] & k) {
				result = false;
				f54len += snprintf(f54buf + f54len,
						sizeof(f54buf) - f54len,
						"Fail, TRx[%d]\n", ((i * 8) + j));
			}
		}
	}

	if (result) {
		TOUCH_I("TRex-TRex Gnd Test passed.\n");
		f54len += snprintf(f54buf + f54len, sizeof(f54buf) - f54len,
				"TRex-TRex Gnd Test passed.\n\n");
	} else {
		TOUCH_I("TRex-TRex Gnd Test failed.\n");
		f54len += snprintf(f54buf + f54len, sizeof(f54buf) - f54len,
				"TRex-TRex Gnd Test failed.\n\n");
	}

	return (result) ? 1 : 0;
}

/* Compare Report type #26 data against test limits */
int CompareTRexShortTestReport(void)
{
	bool result = true;
	int i, j, k = 0;

	for (i = 0; i < TRX_BITMAP_LENGTH; i++) {
		Data[i] &= TRxPhysical_bit[i];
		if (Data[i] != TRX_Short[i]) {
			err_array[i] = Data[i] ^ TRX_Short[i];
			result = false;
		}
	}

	for (i = 0; i < TRX_BITMAP_LENGTH; i++) {
		for (j = 0; j < 8; j++) {
			k = 0x01 << j;
			if (err_array[i] & k) {
				result = false;
				f54len += snprintf(f54buf + f54len,
						sizeof(f54buf) - f54len,
						"Fail, TRx[%d]\n", ((i * 8) + j));
			}
		}
	}

	if (result) {
		TOUCH_I("TRex-TRex Short Test passed.\n");
		f54len += snprintf(f54buf + f54len, sizeof(f54buf) - f54len,
				"TRex-TRex Short Test passed.\n\n");
	} else {
		TOUCH_I("TRex-TRex Short Test failed.\n");
		f54len += snprintf(f54buf + f54len, sizeof(f54buf) - f54len,
				"TRex-TRex Short Test failed.\n\n");
	}

	return (result) ? 1 : 0;
}

/* Compare Report type #2 data against test limits */
int CompareNoiseReport(void)
{
	bool result = true;
	int i, j = 0;

	wlen += snprintf(wbuf + wlen,
			sizeof(wbuf) - wlen,
			"   : ");

	for (i = 0; i < (int)RxChannelCount; i++)
		wlen += snprintf(wbuf + wlen, sizeof(wbuf) - wlen,
				" [%2d] ", i);

	for (i = 0; i < TxChannelCount; i++) {
		wlen += snprintf(wbuf + wlen, sizeof(wbuf) - wlen,
				"\n[%2d] ", i);
		for (j = 0; j < RxChannelCount; j++) {
			ImagepF[i][j] = NoiseDeltaMax[i][j] -
				NoiseDeltaMin[i][j];
			wlen += snprintf(wbuf + wlen, sizeof(wbuf) - wlen,
					"%5d ", ImagepF[i][j]);
		}
	}
	wlen += snprintf(wbuf + wlen, sizeof(wbuf) - wlen, "\n");

	if (result == false) {
		TOUCH_I("Noise Test failed.\n");
		wlen += snprintf(wbuf + wlen, sizeof(wbuf) - wlen,
				"\nNoise Test failed.\n\n");
	} else {
		TOUCH_I("Noise Test passed.\n");
		wlen += snprintf(wbuf + wlen, sizeof(wbuf) - wlen,
				"\nNoise Test passed.\n\n");
	}

	return (result) ? 1 : 0;
}

/* Construct data with Report Type #20 data */
int ReadImageReport(struct device *dev)
{
	int ret = 0;
	int i, j, k = 0;
	int min = 9999;
	int max = 0;

	Read8BitRegisters(dev, (F54DataBase + REPORT_DATA_OFFEST),
			&Data[0], MaxArrayLength);

	TOUCH_I("Full Raw Capacitance Test\n");
	f54len += snprintf(f54buf + f54len, sizeof(f54buf) - f54len,
			"[Full Raw Capacitance Test]\n");

	f54len += snprintf(f54buf + f54len, sizeof(f54buf) - f54len,
			"Tx = %d, Rx = %d\n",
			(int)TxChannelCount, (int)RxChannelCount);

	f54len += snprintf(f54buf + f54len, sizeof(f54buf) - f54len, "   : ");

	for (i = 0; i < (int)RxChannelCount; i++)
		f54len += snprintf(f54buf + f54len, sizeof(f54buf) - f54len,
				" [%2d] ", i);

	for (i = 0; i < (int)TxChannelCount; i++) {
		f54len += snprintf(f54buf + f54len, sizeof(f54buf) - f54len,
				"\n[%2d] ", i);
		for (j = 0; j < (int)RxChannelCount; j++) {
			Image1[i][j] = ((short)Data[k]
					| (short)Data[k + 1] << 8);
			ImagepF[i][j] = Image1[i][j];
			f54len += snprintf(f54buf + f54len,
					sizeof(f54buf) - f54len,
					"%5d ", ImagepF[i][j]);
			k = k + 2;
			if (ImagepF[i][j] != 0 &&
				ImagepF[i][j] < min)
				min = ImagepF[i][j];

			if (ImagepF[i][j] > max)
				max = ImagepF[i][j];
		}
	}

	f54len += snprintf(f54buf + f54len, sizeof(f54buf) - f54len, "\n");

	f54len += snprintf(f54buf + f54len, sizeof(f54buf) - f54len,
				"\nRawdata min : %d, max : %d\n", min, max);

	ret = CompareImageReport();

	write_file(dev, f54buf, TIME_INFO_SKIP);
	touch_msleep(30);

	/* Reset Device */
	Reset(dev);

	return ret;
}

/* Print Rawdata or Delta */
int GetImageReport(struct device *dev, char *buf)
{
	int ret = 0;
	int i, j, k = 0;

	Read8BitRegisters(dev, (F54DataBase + REPORT_DATA_OFFEST),
			&Data[0], MaxArrayLength);

	*buf = 0;
	ret += snprintf(buf + ret, PAGE_SIZE - ret, "Tx = %d, Rx = %d\n",
			(int)TxChannelCount, (int)RxChannelCount);

	for (i = 0; i < (int)TxChannelCount; i++) {
		for (j = 0; j < (int)RxChannelCount; j++) {
			Image1[i][j] = ((short)Data[k]
					| (short)Data[k + 1] << 8);
			ImagepF[i][j] = Image1[i][j];
			k = k + 2;
		}
	}

	for (i = 0; i < (int)RxChannelCount; i++) {
		ret += snprintf(buf + ret, PAGE_SIZE - ret, "[%2d] ", i);
		for (j = 0; j < (int)TxChannelCount; j++) {
			ret += snprintf(buf + ret, PAGE_SIZE - ret,
					"%5d ", ImagepF[j][i]);
		}
		ret += snprintf(buf + ret, PAGE_SIZE - ret, "\n");
	}

	return ret;
}

/* Construct data with Report Type #2 data */
int ReadNoiseReport(struct device *dev)
{
	int ret = 0;
	int i, j, k = 0;

	/* Set FIFO index */
	unsigned char fifoIndex[2] = {0, 0};
	Write8BitRegisters(dev, F54DataBase + 1, fifoIndex, sizeof(fifoIndex));

	Read8BitRegisters(dev, (F54DataBase + REPORT_DATA_OFFEST),
			&Data[0], MaxArrayLength);

	TOUCH_I("Noise Test\n");
	wlen += snprintf(wbuf + wlen, sizeof(wbuf) - wlen, "[Noise Test]\n");

	for (i = 0; i < (int)TxChannelCount; i++) {
		for (j = 0; j < (int)RxChannelCount; j++) {
			Image1[i][j] = (short)Data[k]
				| ((short)Data[k + 1] << 8);
			ImagepF[i][j] = Image1[i][j];

			if (ImagepF[i][j] < NoiseDeltaMin[i][j])
				NoiseDeltaMin[i][j] = ImagepF[i][j];

			if (ImagepF[i][j] > NoiseDeltaMax[i][j])
				NoiseDeltaMax[i][j] = ImagepF[i][j];

			k = k + 2;
		}
	}

	ret = CompareNoiseReport();
	write_file(dev, wbuf, TIME_INFO_SKIP);
	touch_msleep(30);

	/* Reset Device */
	Reset(dev);

	return ret;
}

/* Construct data with Report Type #4 data */
int ReadHighResistanceReport(struct device *dev)
{
	short maxRx, maxTx, min;
	int maxRxpF, maxTxpF, minpF;
	int ret = 0;
	int i = 0;

	Read8BitRegisters(dev, (F54DataBase + REPORT_DATA_OFFEST), &Data[0], 6);

	maxRx = ((short)Data[0] | (short)Data[1] << 8);
	maxTx = ((short)Data[2] | (short)Data[3] << 8);
	min = ((short)Data[4] | (short)Data[5] << 8);

	maxRxpF = maxRx;
	maxTxpF = maxTx;
	minpF = min;

	TOUCH_I("High Resistance Test\n");
	f54len += snprintf(f54buf + f54len, sizeof(f54buf) - f54len,
			"[High Resistance Test]\n");

	f54len += snprintf(f54buf + f54len, sizeof(f54buf) - f54len,
			"Max Rx Offset(pF) = %d\n", maxRxpF);

	f54len += snprintf(f54buf + f54len, sizeof(f54buf) - f54len,
			"Max Tx Offset(pF) = %d\n", maxTxpF);

	f54len += snprintf(f54buf + f54len, sizeof(f54buf) - f54len,
			"Min(pF) = %d\n", minpF);

	f54len += snprintf(f54buf + f54len, sizeof(f54buf) - f54len,
			"=====================================================\n");

	f54len += snprintf(f54buf + f54len, sizeof(f54buf) - f54len, " Parameters: ");

	f54len += snprintf(f54buf + f54len, sizeof(f54buf) - f54len,
			"%5d %5d %5d ", maxRxpF, maxTxpF, minpF);

	f54len += snprintf(f54buf + f54len, sizeof(f54buf) - f54len, "\n\n Limits(+) : ");

	for (i = 0; i < 3; i++)
		f54len += snprintf(f54buf + f54len, sizeof(f54buf) - f54len,
				"%5d ", HighResistanceUpperLimit[i]);

	f54len += snprintf(f54buf + f54len, sizeof(f54buf) - f54len, "\n Limits(-) : ");

	for (i = 0; i < 3; i++)
		f54len += snprintf(f54buf + f54len, sizeof(f54buf) - f54len,
				"%5d ", HighResistanceLowerLimit[i]);

	f54len += snprintf(f54buf + f54len, sizeof(f54buf) - f54len,
			"\n-----------------------------------------------------\n");

	ret = CompareHighResistance(maxRxpF, maxTxpF, minpF);
	write_file(dev, f54buf, TIME_INFO_SKIP);
	touch_msleep(30);

	/* Reset Device */
	Reset(dev);

	return ret;
}

/* Construct data with Report Type #13 data */
void ReadMaxMinReport(struct device *dev)
{
	short max, min;
	int maxpF, minpF;

	Read8BitRegisters(dev, (F54DataBase + REPORT_DATA_OFFEST), &Data[0], 4);

	max = ((short)Data[0] | (short)Data[1] << 8);
	min = ((short)Data[2] | (short)Data[3] << 8);
	maxpF = max;
	minpF = min;

	f54len += snprintf(f54buf + f54len, sizeof(f54buf) - f54len,
			"[Full Raw Capacitance Max/Min Test]\n");

	TOUCH_I("\nRaw Capacitance Maximum and Minimum Test:\n");
	/* TOUCH_I("Max = 0x%x\n", max);
	  TOUCH_I("Min = 0x%x\n", min); */
	TOUCH_I("Max(pF) = %d\n", maxpF);
	TOUCH_I("Min(pF) = %d\n", minpF);

	/* Reset Device */
	Reset(dev);
}

/* Construct data with Report Type #23 data */
int ReadADCRangeReport(struct device *dev)
{
	int temp = TxChannelCount;
	int ret = 0;
	int i, j, k = 0;

	if (SignalClarityOn) {
		if ((TxChannelCount / 4) != 0) {
			temp = (4 - (TxChannelCount % 4)) +  TxChannelCount;
			Read8BitRegisters(dev, (F54DataBase + REPORT_DATA_OFFEST),
					&Data[0], (temp * RxChannelCount * 2));
		}
	} else {
		Read8BitRegisters(dev, (F54DataBase + REPORT_DATA_OFFEST),
				&Data[0], MaxArrayLength);
	}

	k = 0;

	TOUCH_I("ADC Range Test\n");
	wlen += snprintf(wbuf + wlen, sizeof(wbuf) - wlen, "[ADC Range Test]\n");

	for (i = 0; i < (int)TxChannelCount; i++) {
		for (j = 0; j < (int)RxChannelCount; j++) {
			Image1[i][j] = ((unsigned short)Data[k]);
			k = k + 2;
		}
	}

	ret = CompareADCReport();
	write_file(dev, wbuf, TIME_INFO_SKIP);
	touch_msleep(20);

	/* Reset Device */
	Reset(dev);

	return ret;
}

void ReadAbsADCRangeReport(struct device *dev)
{
	int i, k = 0;

	Read8BitRegisters(dev, (F54DataBase + REPORT_DATA_OFFEST),
			&Data[0], 2 * (RxChannelCount + TxChannelCount));

	f54len += snprintf(f54buf + f54len, sizeof(f54buf) - f54len,
			"[Abs Sensing ADC Range Test]\n");

	TOUCH_I("Abs Sensing ADC Range Data:\n");
	TOUCH_I("Rx: ");
	for (i = 0; i < (int)RxChannelCount; i++) {
		AbsADCRangeData[k / 2] = (unsigned char)Data[k];
		TOUCH_I("%d ", AbsADCRangeData[k / 2]);
		k = k + 2;
	}
	TOUCH_I("\n");
	TOUCH_I("Tx: ");
	for (i = 0; i < (int)TxChannelCount; i++) {
		AbsADCRangeData[k / 2] = (unsigned char)Data[k];
		TOUCH_I("%d ", AbsADCRangeData[k / 2]);
		k = k + 2;
	}
	TOUCH_I("\n");

	CompareAbsADCRangeReport();

	/* Reset Device */
	Reset(dev);
}

void ReadAbsDeltaReport(struct device *dev)
{
	int i, k = 0;
	int *p32data;

	Read8BitRegisters(dev, (F54DataBase + REPORT_DATA_OFFEST),
			&Data[0], 4 * (RxChannelCount + TxChannelCount));

	p32data = (int *)&Data[0];

	f54len += snprintf(f54buf + f54len, sizeof(f54buf) - f54len,
			"[Abs Sensing Delta Capacitance Test]\n");

	f54len += snprintf(f54buf + f54len, sizeof(f54buf) - f54len,
			"Abs Sensing Delta Capacitance Data:\n");
	f54len += snprintf(f54buf + f54len, sizeof(f54buf) - f54len, "Rx: ");

	for (i = 0; i < (int)RxChannelCount; i++) {
		AbsSigned32Data[k] = (int)*p32data;
		f54len += snprintf(f54buf + f54len, sizeof(f54buf) - f54len,
				"%d ", AbsSigned32Data[k]);
		k++;
		p32data++;
	}

	f54len += snprintf(f54buf + f54len, sizeof(f54buf) - f54len, "\nTx: ");

	for (i = 0; i < (int)TxChannelCount; i++) {
		AbsSigned32Data[k] = (int)*p32data;
		f54len += snprintf(f54buf + f54len, sizeof(f54buf) - f54len,
				"%d ", AbsSigned32Data[k]);
		k++;
		p32data++;
	}

	f54len += snprintf(f54buf + f54len, sizeof(f54buf) - f54len, "\n");

	/* Reset Device */
	Reset(dev);
}

/* Construct data with Report Type #38 data */
int ReadAbsRawReport(struct device *dev, u16 input)
{
	int i, k = 0;
	int *p32data;
	int ret = 0;

	Read8BitRegisters(dev, (F54DataBase + REPORT_DATA_OFFEST),
			&Data[0], 4 * (RxChannelCount + TxChannelCount));

	p32data = (int *)&Data[0];

	if (input == eRT_AbsRaw) {
		TOUCH_I("Abs Rawdata Test\n");
		f54len += snprintf(f54buf + f54len, sizeof(f54buf) - f54len,
				"[Abs Rawdata Test]\n");
	} else {
		TOUCH_I("Abs LPWG Rawdata Test\n");
		f54len += snprintf(f54buf + f54len, sizeof(f54buf) - f54len,
				"[Abs LPWG Rawdata Test]\n");
	}

	for (i = 0; i < (int)RxChannelCount; i++) {
		AbsSigned32Data[k] = (int)*p32data;
		f54len += snprintf(f54buf + f54len, sizeof(f54buf) - f54len,
				"Rx[%2d] = %d\n", i, AbsSigned32Data[k]);
		k++;
		p32data++;
	}

	ret = CompareAbsRawReport(input);

	write_file(dev, f54buf, TIME_INFO_SKIP);
	touch_msleep(30);

	/* Reset Device */
	Reset(dev);

	return ret;
}

/* Construct data with Report Type #38 data */
int ReadAbsRawOpen(struct device *dev)
{
	int i = 0;
	int ret = 0;
	unsigned char k = 0;

	Read8BitRegisters(dev, (F54DataBase + REPORT_DATA_OFFEST),
			&Data[0], (F12_2DRxCount + F12_2DTxCount) * 4);

	TOUCH_I("Abs Sensing Open Test Data:\n");
	f54len += snprintf(f54buf + f54len, sizeof(f54buf) - f54len,
			"[Abs Sensing Raw Open Capcitance Test]\n");

	f54len += snprintf(f54buf + f54len, sizeof(f54buf) - f54len,
			"Abs Sensing Open Test Data:\n");

	for (i = 0; i < ((int)F12_2DRxCount + (int)F12_2DTxCount); i++)	{
		AbsOpen[i] = (unsigned int)Data[k] |
			((unsigned int)Data[k + 1] << 8) |
			((unsigned int)Data[k + 2] << 16) |
			((unsigned int)Data[k + 3] << 24);

		k += 4;

		if (i < (int)F12_2DRxCount) {
			TOUCH_I("Rx[%d]: %d, ", i, AbsOpen[i]);
			f54len += snprintf(f54buf + f54len,
					sizeof(f54buf) - f54len,
					"%5d ", AbsOpen[i]);
		} else {
			TOUCH_I("Tx[%d]: %d, ",
					i - (int)F12_2DRxCount, AbsOpen[i]);
			f54len += snprintf(f54buf + f54len,
					sizeof(f54buf) - f54len,
					"%5d ", AbsOpen[i]);
		}

		if (i == ((int)F12_2DRxCount - 1)) {
			TOUCH_I("\n");
			f54len += snprintf(f54buf + f54len,
					sizeof(f54buf) - f54len, "\n");
		}
	}
	TOUCH_I("\n");

	ret = CompareAbsOpen();

	return ret;
}

/* Construct data with Report Type #38 data */
int ReadAbsRawShort(struct device *dev)
{
	int i = 0;
	int ret = 0;
	unsigned char k = 0;

	Read8BitRegisters(dev, (F54DataBase + REPORT_DATA_OFFEST),
			&Data[0], (F12_2DRxCount + F12_2DTxCount) * 4);

	TOUCH_I("Abs Sensing Short Test Data:\n");
	f54len += snprintf(f54buf + f54len, sizeof(f54buf) - f54len,
			"[Abs Sensing Raw Short Capcitance Test]\n");

	f54len += snprintf(f54buf + f54len, sizeof(f54buf) - f54len,
			"Abs Sensing Short Test Data:\n");

	for (i = 0; i < ((int)F12_2DRxCount + (int)F12_2DTxCount); i++)	{

		AbsShort[i] = (unsigned int)Data[k] |
			((unsigned int)Data[k + 1] << 8) |
			((unsigned int)Data[k + 2] << 16) |
			((unsigned int)Data[k + 3] << 24);

		k += 4;

		if (i < (int)F12_2DRxCount) {
			TOUCH_I("Rx[%d]: %d, ", i, AbsShort[i]);
			f54len += snprintf(f54buf + f54len,
					sizeof(f54buf) - f54len,
					"%5d ", AbsShort[i]);
		} else {
			TOUCH_I("Tx[%d]: %d, ",
					i - (int)F12_2DRxCount, AbsShort[i]);
			f54len += snprintf(f54buf + f54len,
					sizeof(f54buf) - f54len,
					"%5d ", AbsShort[i]);
		}

		if (i == ((int)F12_2DRxCount - 1)) {
			TOUCH_I("\n");
			f54len += snprintf(f54buf + f54len,
					sizeof(f54buf) - f54len, "\n");
		}
	}
	TOUCH_I("\n");

	ret = CompareAbsShort();

	return ret;
}

/* Construct data with Report Type #22 data */
int ReadSensorSpeedReport(struct device *dev)
{
	int i, j, k = 0;
	int ret = 0;

	Read8BitRegisters(dev, (F54DataBase + REPORT_DATA_OFFEST),
			&Data[0], MaxArrayLength);

	TOUCH_I("Sensor Speed Test\n");
	wlen += snprintf(wbuf + wlen, sizeof(wbuf) - wlen,
			"[Sensor Speed Test]\n");

	for (i = 0; i < (int)TxChannelCount; i++) {
		for (j = 0; j < (int)RxChannelCount; j++) {
			Image1[i][j] = ((short)Data[k]
					| (short)Data[k+1] << 8);
			ImagepF[i][j] = Image1[i][j];
			k = k + 2;
		}
	}

	ret = CompareSensorSpeedReport();
	write_file(dev, wbuf, TIME_INFO_SKIP);
	touch_msleep(20);

	/* Reset Device */
	Reset(dev);

	return ret;
}

/* Construct data with Report Type #24 data */
void ReadTRexOpenReport(struct device *dev)
{
	int i = 0;

	Read8BitRegisters(dev, (F54DataBase + REPORT_DATA_OFFEST),
				&Data[0], TRX_BITMAP_LENGTH);

	TOUCH_I("TRex-TRex Open Test\n");
	wlen += snprintf(wbuf + wlen, sizeof(wbuf) - wlen, "[TRex Open Test]\n");

	for (i = 0; i < TRX_BITMAP_LENGTH; i++)	{
		wlen += snprintf(wbuf + wlen, sizeof(wbuf) - wlen,
				"TRex-TRex Open Test Data = %#x\n", Data[i]);
	}

	wlen += snprintf(wbuf + wlen, sizeof(wbuf) - wlen, "\n");

	CompareTRexOpenTestReport();

	/* Reset Device */
	Reset(dev);
}

/* Construct data with Report Type #25 data  */
int ReadTRexGroundReport(struct device *dev)
{
	int ret = 0;
	int i = 0;

	Read8BitRegisters(dev, (F54DataBase + REPORT_DATA_OFFEST),
				&Data[0], TRX_BITMAP_LENGTH);

	TOUCH_I("TRex-TRex Gnd Test\n");
	f54len += snprintf(f54buf + f54len, sizeof(f54buf) - f54len,
			"[TRex Gnd Test]\n");

	for (i = 0; i < TRX_BITMAP_LENGTH; i++)	{
		f54len += snprintf(f54buf + f54len, sizeof(f54buf) - f54len,
				"TRex-TRex Gnd Test Data = %#x\n", Data[i]);
	}

	f54len += snprintf(f54buf + f54len, sizeof(f54buf) - f54len, "\n");

	ret = CompareTRexGroundTestReport();

	write_file(dev, f54buf, TIME_INFO_SKIP);
	touch_msleep(30);

	/* Reset Device */
	Reset(dev);

	return ret;
}

/* Construct data with Report Type #26 data */
int ReadTRexShortReport(struct device *dev)
{
	int ret = 0;
	int i= 0;

	Read8BitRegisters(dev, (F54DataBase + REPORT_DATA_OFFEST),
				&Data[0], TRX_BITMAP_LENGTH);

	TOUCH_I("TRex-TRex Short Test\n");
	f54len += snprintf(f54buf + f54len, sizeof(f54buf) - f54len,
			"[TRex Short Test]\n");

	for (i = 0; i < TRX_BITMAP_LENGTH; i++)	{
		f54len += snprintf(f54buf + f54len, sizeof(f54buf) - f54len,
				"TRex-TRex Short Test Data = %#x\n", Data[i]);
	}

	f54len += snprintf(f54buf + f54len, sizeof(f54buf) - f54len, "\n");

	ret = CompareTRexShortTestReport();
	write_file(dev, f54buf, TIME_INFO_SKIP);
	touch_msleep(30);

	/* Reset Device */
	Reset(dev);

	return ret;
}

/* Function to handle report reads based on user input */
int ReadReport(struct device *dev, u16 input, char *buf)
{
	int ret = 0;
	unsigned char data;
	unsigned int count = 0;

	/* Set the GetReport bit to run the AutoScan */
	data = 0x01;
	if (Write8BitRegisters(dev, F54CommandBase, &data, 1) < 0)
		goto error;

	do {
		if (Read8BitRegisters(dev, F54CommandBase, &data, 1) < 0)
			goto error;
		touch_msleep(1);
		count++;
	} while (data != 0x00 && (count < DefaultTimeout));

	if (count >= DefaultTimeout) {
		TOUCH_E("Timeout - Not supported Report Type in FW\n");
		Reset(dev);
		return -EAGAIN;
	}

	do_gettimeofday(&t_interval[ENDTIME]);

	TOUCH_I("Takes %lu ticks\n",
			get_time_interval(t_interval[ENDTIME].tv_sec,
				t_interval[STARTTIME].tv_sec));

	switch (input) {
	case eRT_FullRawCapacitance:
		ret = ReadImageReport(dev);
		break;
	case eRT_ADCRange:
		ret = ReadADCRangeReport(dev);
		break;
	case eRT_SensorSpeed:
		ret = ReadSensorSpeedReport(dev);
		break;
	case eRT_TRexOpen:
		ReadTRexOpenReport(dev);
		break;
	case eRT_TRexGround:
		ret = ReadTRexGroundReport(dev);
		break;
	case eRT_TRexShort:
		ret = ReadTRexShortReport(dev);
		break;
	case eRT_HighResistance:
		ret = ReadHighResistanceReport(dev);
		break;
	case eRT_FullRawCapacitanceMaxMin:
		ReadMaxMinReport(dev);
		break;
	case eRT_AbsADCRange:
		ReadAbsADCRangeReport(dev);
		break;
	case eRT_AbsDelta:
		ReadAbsDeltaReport(dev);
		break;
	case eRT_AbsRaw:
	case eRT_AbsLpwgRaw:
		ret = ReadAbsRawReport(dev, input);
		break;
	case eRT_RawImageRT3:
		ret = GetImageReport(dev, buf);
		break;
	case eRT_Normalized16BitImageReport:
		ret = ReadNoiseReport(dev);
		break;
	case eRT_AbsShort:
		ret = ReadAbsRawShort(dev);
		break;
	case eRT_AbsOpen:
		ret = ReadAbsRawOpen(dev);
		break;
	default:
		break;
	}

	return ret;

error:
	TOUCH_E("[%s] ReadReport fail\n", __func__);
	return -EAGAIN;
}


/* Examples of reading query registers.
 Real applications often do not need to read query registers at all.
 */
void RunQueries(struct device *dev)
{
	unsigned short cAddr = 0xEE;
	unsigned char cFunc = 0;
	int rxCount = 0;
	int txCount = 0;
	int offset = 0;
	int q_offset = 0;
	int i, j = 0;
	int tsvd_select;
	int tsvd;
	int tshd;
	int tsstb;
	int tsfrq;
	int tsfst;
	int exvcom_pin_type;
	int exvcom1;
	int exvcom_sel;
	int exvcom2;
	int enable_guard;
	int guard_ring;
	int enable_verf;
	int verf;
	bool HasCtrl102Sub1;
	bool HasCtrl102Sub2;
	bool HasCtrl102Sub4;
	bool HasCtrl102Sub5;
	bool HasCtrl102Sub9;
	bool HasCtrl102Sub10;
	bool HasCtrl102Sub11;
	bool HasCtrl102Sub12;

	/* Scan Page Description Table (PDT)
	   to find all RMI functions presented by this device.
	   The Table starts at $00EE. This and every sixth register
	   (decrementing) is a function number
	   except when this "function number" is $00, meaning end of PDT.
	   In an actual use case this scan might be done only once
	   on first run or before compile.
	*/
	do {
		Read8BitRegisters(dev, cAddr, &cFunc, 1);
		if (cFunc == 0)
			break;

		switch (cFunc) {
		case 0x01:
			if (!bHaveF01) {
				Read8BitRegisters(dev, (cAddr - 3), &F01ControlBase, 1);
				Read8BitRegisters(dev, (cAddr - 4), &F01CommandBase, 1);
			}
			break;

		case 0x12:
			if (!bHaveF12) {
				Read8BitRegisters(dev, (cAddr - 3), &F12ControlBase, 1);
				Read8BitRegisters(dev, (cAddr - 5), &F12QueryBase, 1);
				Read8BitRegisters(dev, (F12QueryBase), &F12Support, 1);

				if ((F12Support | 0x00) == 0) {
					TOUCH_I("Device not support F12.\n");
					break;
				}
				Read8BitRegisters(dev, (F12QueryBase + 5), Data, 2);
				mask = 0x01;
				for (j = 0; j < 8; j++) {
					if ((Data[1] & mask) == 1)
						offset++;
					Data[1] >>= 1;
				}
				Read8BitRegisters(dev, (F12ControlBase + offset),
						Data, 14);
				F12_2DRxCount = Data[12];
				F12_2DTxCount = Data[13];

				if (TRX_MAX <= F12_2DRxCount)
					F12_2DRxCount = TRX_MAX;
				if (TRX_MAX <= F12_2DTxCount)
					F12_2DTxCount = 16;

				offset = 0;
			}
			break;

		case 0x51:
			Read8BitRegisters(dev, (cAddr - 2), &F51DataBase, 1);
			Read8BitRegisters(dev, (cAddr - 3), &F51ControlBase, 1);
			Read8BitRegisters(dev, F51DataBase, &Data[0], 4);
			break;

		case 0x54:
			if (!bHaveF54) {
				Read8BitRegisters(dev, (cAddr - 2), &F54DataBase, 1);
				Read8BitRegisters(dev, (cAddr - 3), &F54ControlBase, 1);
				Read8BitRegisters(dev, (cAddr - 4), &F54CommandBase, 1);
				Read8BitRegisters(dev, (cAddr - 5), &F54QueryBase, 1);
				Read8BitRegisters(dev, F54QueryBase, &RxChannelCount, 1);
				Read8BitRegisters(dev, (F54QueryBase + 1), &TxChannelCount, 1);

				if (TRX_MAX <= RxChannelCount)
					RxChannelCount = TRX_MAX;
				if (TRX_MAX <= TxChannelCount)
					TxChannelCount = TRX_MAX;

				MaxArrayLength = (int)RxChannelCount * (int)TxChannelCount * 2;

				Read8BitRegisters(dev, F54QueryBase, Data, 60);
				TouchControllerFamily = Data[5];
				offset++;	/* Ctrl 00 */

				if (TouchControllerFamily == 0x0 ||
						TouchControllerFamily == 0x01)
					offset++;	/* Ctrl 01 */
				offset += 2;	/* Ctrl 02 */
				bHavePixelTouchThresholdTuning =
					((Data[6] & 0x01) == 0x01);

				if (bHavePixelTouchThresholdTuning)
					offset++; /* Ctrl 03 */

				if (TouchControllerFamily == 0x0 ||
						TouchControllerFamily == 0x01)
					offset += 3;	/* Ctrl 04/05/06 */

				if (TouchControllerFamily == 0x01) {
					F54Ctrl07Offset = offset;
					offset++;	/* Ctrl 07 */
					bHaveF54Ctrl07 = true;
				}

				/* Ctrl 08 */
				if (TouchControllerFamily == 0x0 ||
						TouchControllerFamily == 0x01)
					offset += 2;
				/* Ctrl 09 */
				if (TouchControllerFamily == 0x0 ||
						TouchControllerFamily == 0x01)
					offset++;
				bHaveInterferenceMetric = ((Data[7] & 0x02) == 0x02);
				/* Ctrl 10 */
				if (bHaveInterferenceMetric)
					offset++;
				bHaveCtrl11 = ((Data[7] & 0x10) == 0x10);
				/* Ctrl 11 */
				if (bHaveCtrl11)
					offset += 2;
				bHaveRelaxationControl = ((Data[7] & 0x80) == 0x80);
				/* Ctrl 12/13 */
				if (bHaveRelaxationControl)
					offset += 2;
				bHaveSensorAssignment = ((Data[7] & 0x01) == 0x01);
				/* Ctrl 14 */
				if (bHaveSensorAssignment)
					offset++;
				/* Ctrl 15 */
				if (bHaveSensorAssignment)
					offset += RxChannelCount;
				/* Ctrl 16 */
				if (bHaveSensorAssignment)
					offset += TxChannelCount;
				bHaveSenseFrequencyControl =
					((Data[7] & 0x04) == 0x04);
				NumOfSensingFreq = (Data[13] & 0x0F);
				/* Ctrl 17/18/19 */
				if (bHaveSenseFrequencyControl)
					offset += (3 * (int)NumOfSensingFreq);
				offset++;	/* Ctrl 20 */
				if (bHaveSenseFrequencyControl)
					offset += 2;	/* Ctrl 21 */
				bHaveFirmwareNoiseMitigation = ((Data[7] & 0x08) == 0x08);
				if (bHaveFirmwareNoiseMitigation)
					offset++;	/* Ctrl 22 */
				if (bHaveFirmwareNoiseMitigation)
					offset += 2;	/* Ctrl 23 */
				if (bHaveFirmwareNoiseMitigation)
					offset += 2;	/* Ctrl 24 */
				if (bHaveFirmwareNoiseMitigation)
					offset++;	/* Ctrl 25 */
				if (bHaveFirmwareNoiseMitigation)
					offset++;	/* Ctrl 26 */
				bHaveIIRFilter = ((Data[9] & 0x02) == 0x02);
				if (bHaveIIRFilter)
					offset++;	/* Ctrl 27 */
				if (bHaveFirmwareNoiseMitigation)
					offset += 2;	/* Ctrl 28 */
				bHaveCmnRemoval = ((Data[9] & 0x04) == 0x04);
				bHaveCmnMaximum = ((Data[9] & 0x08) == 0x08);
				if (bHaveCmnRemoval)
					offset++;	/* Ctrl 29 */
				if (bHaveCmnMaximum)
					offset++;	/* Ctrl 30 */
				bHaveTouchHysteresis = ((Data[9] & 0x10) == 0x10);
				if (bHaveTouchHysteresis)
					offset++;	/* Ctrl 31 */
				bHaveEdgeCompensation = ((Data[9] & 0x20) == 0x20);
				if (bHaveEdgeCompensation)
					offset += 2;	/* Ctrl 32 */
				if (bHaveEdgeCompensation)
					offset += 2;	/* Ctrl 33 */
				if (bHaveEdgeCompensation)
					offset += 2;	/* Ctrl 34 */
				if (bHaveEdgeCompensation)
					offset += 2;	/* Ctrl 35 */
				CurveCompensationMode = (Data[8] & 0x03);
				if (CurveCompensationMode == 0x02) {
					offset += (int)RxChannelCount;
				} else if (CurveCompensationMode == 0x01) {
					offset += ((int)RxChannelCount  > (int)TxChannelCount) ?
						(int)RxChannelCount : (int)TxChannelCount;
				}	/* Ctrl 36 */

				if (CurveCompensationMode == 0x02) {
					/* Ctrl 37 */
					offset += (int)TxChannelCount;
				}

				bHavePerFrequencyNoiseControl = ((Data[9] & 0x40) == 0x40);

				/* Ctrl 38/39/40 */
				if (bHavePerFrequencyNoiseControl)
					offset += (3 * (int)NumOfSensingFreq);

				bHaveSignalClarity = ((Data[10] & 0x04) == 0x04);

				if (bHaveSignalClarity) {
					F54Ctrl41Offset = offset;
					offset++;	/* Ctrl 41 */
					/* bHaveF54Ctrl41 = bSignalClarityOn */
					SignalClarityOn = true;
				} else
					SignalClarityOn = false;

				bHaveMultiMetricStateMachine = ((Data[10] & 0x02) == 0x02);
				bHaveVarianceMetric = ((Data[10] & 0x08) == 0x08);
				if (bHaveVarianceMetric)
					offset += 2;	/* Ctrl 42 */
				if (bHaveMultiMetricStateMachine)
					offset += 2;	/* Ctrl 43 */
				/* Ctrl 44/45/46/47/48/49/50/51/52/53/54 */
				if (bHaveMultiMetricStateMachine)
					offset += 11;

				bHave0DRelaxationControl = ((Data[10] & 0x10) == 0x10);
				bHave0DAcquisitionControl = ((Data[10] & 0x20) == 0x20);
				if (bHave0DRelaxationControl)
					offset += 2;	/*Ctrl 55/56 */
				if (bHave0DAcquisitionControl) {
					F54Ctrl57Offset = offset;
					offset++;	/* Ctrl 57 */
					bHaveF54Ctrl57 = true;
				}
				if (bHave0DAcquisitionControl)
					offset += 1;	/* Ctrl 58 */

				bHaveSlewMetric = ((Data[10] & 0x80) == 0x80);
				bHaveHBlank = ((Data[11] & 0x01) == 0x01);
				bHaveVBlank = ((Data[11] & 0x02) == 0x02);
				bHaveLongHBlank = ((Data[11] & 0x04) == 0x04);
				bHaveNoiseMitigation2 = ((Data[11] & 0x20) == 0x20);
				bHaveSlewOption = ((Data[12] & 0x02) == 0x02);

				if (bHaveHBlank)
					offset += 1;	/* Ctrl 59 */

				if (bHaveHBlank || bHaveVBlank || bHaveLongHBlank)
					offset += 3;	/* Ctrl 60/61/62 */

				if (bHaveSlewMetric || bHaveHBlank
						|| bHaveVBlank
						|| bHaveLongHBlank
						|| bHaveNoiseMitigation2
						|| bHaveSlewOption)
					offset += 1;	/* Ctrl 63 */

				if (bHaveHBlank)
					offset += 28;	/* Ctrl 64/65/66/67 */
				else if (bHaveVBlank || bHaveLongHBlank)
					offset += 4;	/* Ctrl 64/65/66/67 */

				if (bHaveHBlank || bHaveVBlank || bHaveLongHBlank)
					offset += 8;	/* Ctrl 68/69/70/71/72/73 */

				if (bHaveSlewMetric)
					offset += 2;	/* Ctrl 74 */

				bHaveEnhancedStretch = ((Data[9] & 0x80) == 0x80);
				/* Ctrl 75 */
				if (bHaveEnhancedStretch)
					offset += (int)NumOfSensingFreq;

				bHaveStartupFastRelaxation = ((Data[11] & 0x08) == 0x08);
				if (bHaveStartupFastRelaxation)
					offset += 1;	/* Ctrl 76 */

				bHaveESDControl = ((Data[11] & 0x10) == 0x10);
				if (bHaveESDControl)
					offset += 2;	/* Ctrl 77/78 */

				if (bHaveNoiseMitigation2)
					offset += 5;	/* Ctrl 79/80/81/82/83 */

				bHaveEnergyRatioRelaxation = ((Data[11] & 0x80) == 0x80);
				if (bHaveEnergyRatioRelaxation)
					offset += 2;	/* Ctrl 84/85 */

				bHaveF54Query13 = ((Data[12] & 0x08) == 0x08);
				if (bHaveSenseFrequencyControl) {
					q_offset = 13;
					NumOfSensingFreq = (Data[13] & 0x0F);
				} else
					q_offset = 12;
				if (bHaveF54Query13)
					q_offset++;
				bHaveCtrl86 = (bHaveF54Query13 && ((Data[13] & 0x01) == 0x01));
				bHaveCtrl87 = (bHaveF54Query13 && ((Data[13] & 0x02) == 0x02));
				bHaveCtrl88 = ((Data[12] & 0x40) == 0x40);

				if (bHaveCtrl86)
					offset += 1;	/* Ctrl 86 */
				if (bHaveCtrl87)
					offset += 1;	/* Ctrl 87 */
				if (bHaveCtrl88) {
					F54Ctrl88Offset = offset;
					offset++;	/* Ctrl 88 */
				}
				bHaveCtrl89 = ((Data[q_offset]
							& 0x20) == 0x20);
				bHaveCtrl89 = (bHaveCtrl89 | ((Data[q_offset]
							& 0x40) == 0x40));
				bHaveCtrl89 = (bHaveCtrl89 | ((Data[q_offset]
							& 0x80) == 0x80));
				if (bHaveCtrl89)
					offset++;
				bHaveF54Query15 = ((Data[12] & 0x80) == 0x80);
				if (bHaveF54Query15)
					q_offset++; 	/* query_offset = 14 */
				bHaveCtrl90 = (bHaveF54Query15 &&
								((Data[q_offset] & 0x01) == 0x01));
				if (bHaveCtrl90)
					offset++;	/* offset = 1b */
				bHaveF54Query16 = ((Data[q_offset] & 0x8) == 0x8);
				bHaveF54Query20 = ((Data[q_offset] & 0x10) == 0x10);
				bHaveF54Query21 = ((Data[q_offset] & 0x20) == 0x20);
				bHaveF54Query22 = ((Data[q_offset] & 0x40) == 0x40);
				bHaveF54Query25 = ((Data[q_offset] & 0x80) == 0x80);
				if (bHaveF54Query16)
					q_offset++;	/* query_offset = 15 */
				bHaveF54Query17 = ((Data[q_offset] & 0x1) == 0x1);
				bHaveCtrl92 = ((Data[q_offset] & 0x4) == 0x4);
				bHaveCtrl93 = ((Data[q_offset] & 0x8) == 0x8);
				bHaveCtrl94 = ((Data[q_offset] & 0x10) == 0x10);
				bHaveF54Query18 = bHaveCtrl94;
				bHaveCtrl95 = ((Data[q_offset] & 0x20) == 0x20);
				bHaveF54Query19 = bHaveCtrl95;
				bHaveCtrl99 = ((Data[q_offset] & 0x40) == 0x40);
				bHaveCtrl100 = ((Data[q_offset] & 0x80) == 0x80);
				if (bHaveF54Query17)
					q_offset++;	/* query_offset = 16 */
				if (bHaveF54Query18)
					q_offset++;	/* query_offset = 17 */
				if (bHaveF54Query19)
					q_offset++;	/* query_offset = 18 */
				if (bHaveF54Query20)
					q_offset++;	/* query_offset = 19 */
				if (bHaveF54Query21)
					q_offset++;	/* query_offset = 20 */
				bHaveCtrl91 = ((Data[q_offset] & 0x4) == 0x4);
				bHaveCtrl96  = ((Data[q_offset] & 0x8) == 0x8);
				bHaveCtrl97  = ((Data[q_offset] & 0x10) == 0x10);
				bHaveCtrl98  = ((Data[q_offset] & 0x20) == 0x20);
				bHaveF54Query24  = ((Data[q_offset] & 0x80) == 0x80);
				if (bHaveF54Query22)
					q_offset++;	/* query_offset = 21 */
				bHaveCtrl101 = ((Data[q_offset] & 0x2) == 0x2);
				bHaveF54Query23 = ((Data[q_offset] & 0x8) == 0x8);
				bHaveF54Query26 = ((Data[q_offset] & 0x10) == 0x10);
				bHaveF54Ctrl103 = ((Data[q_offset] & 0x10) == 0x10);
				bHaveF54Ctrl104 = ((Data[q_offset] & 0x20) == 0x20);
				bHaveF54Ctrl105 = ((Data[q_offset] & 0x40) == 0x40);
				bHaveF54Query28 = ((Data[q_offset] & 0x80) == 0x80);
				if (bHaveF54Query23) {
					q_offset++;	/* query_offset = 22 */
					bHaveCtrl102 = ((Data[q_offset] & 0x01) == 0x01);
				} else
					bHaveCtrl102 = false;
				if (bHaveCtrl91) {
					F54Ctrl91Offset = offset;
					offset++;
				}
				if (bHaveCtrl92)
					offset++;
				if (bHaveCtrl93)
					offset++;
				if (bHaveCtrl94)
					offset++;
				if (bHaveCtrl95)
					offset++;
				if (bHaveCtrl96) {
					F54Ctrl96Offset = offset;
					offset++;
				}
				if (bHaveCtrl97)
					offset++;
				if (bHaveCtrl98) {
					F54Ctrl98Offset = offset;
					offset++;
				}
				if (bHaveCtrl99)
					offset++;
				if (bHaveCtrl100)
					offset++;
				if (bHaveCtrl101)
					offset++;
				if (bHaveCtrl102) {
					unsigned char addr;
					bIncellDevice = true;
					F54Ctrl102Offset = offset;
					HasCtrl102Sub1 = (Data[q_offset] & 0x02);
					HasCtrl102Sub2 = (Data[q_offset] & 0x04);
					HasCtrl102Sub4 = (Data[q_offset] & 0x08);
					HasCtrl102Sub5 = (Data[q_offset] & 0x010);
					HasCtrl102Sub9 = (Data[q_offset] & 0x020);
					HasCtrl102Sub10 = (Data[q_offset] & 0x40);
					HasCtrl102Sub11 = (Data[q_offset] & 0x80);
					HasCtrl102Sub12 = false;
					offset = 0;
					addr = F54ControlBase + F54Ctrl102Offset;
					Read8BitRegisters(dev, addr, &Data[0], 27);
					tsvd_select = Data[0] & 0x03;
					tsvd = Data[1 + tsvd_select];
					offset = offset + 4;
					tshd = Data[offset];
					if (HasCtrl102Sub1) {
						offset = offset + 2;
						tsstb = Data[offset];
					}
					if (HasCtrl102Sub2) {
						tsfrq = Data[offset + 2];
						tsfst = Data[offset + 3];
						offset = offset + 3;
					}
					/* Ctrl102Sub3 */
					/* 0 = GPIO, 1 = TRX */
					exvcom_pin_type = (Data[offset + 1] & 0x01);
					exvcom1 = Data[offset + 2];
					offset = offset + 2;
					if (HasCtrl102Sub4) {
						exvcom_sel = (Data[offset + 1] & 0x03);
						exvcom2 = Data[offset + 2];
						offset = offset + 4;
					}
					if (HasCtrl102Sub5) {
						enable_guard = (Data[offset + 1] & 0x01);
						guard_ring = Data[offset + 2];
						offset = offset + 2;
					}
					/* Ctrl102Sub6, 7, 8 */
					offset = offset + 5;
					if (HasCtrl102Sub9)
						offset++;
					if (HasCtrl102Sub10) {
						exvcom_sel = Data[offset + 2];
						offset = offset + 2;
					}
					if (HasCtrl102Sub11)
						offset++;
					if (bHaveF54Query25)
						HasCtrl102Sub12 = (Data[q_offset + 1] & 0x02);
					if (HasCtrl102Sub12) {
						enable_verf = ((Data[offset + 1]) & 0x01);
						verf = (Data[offset + 2]);
					}
				}
				if (bHaveF54Query24)
					q_offset++;
				if (bHaveF54Query25)
					q_offset++;	/* Query 25 */
				bHaveF54Ctrl106 =
					((Data[q_offset] & 0x01) == 0x01);
				bHaveF54Ctrl107 =
					((Data[q_offset] & 0x04) == 0x04);
				bHaveF54Ctrl108 =
					((Data[q_offset] & 0x08) == 0x08);
				bHaveF54Ctrl109 =
					((Data[q_offset] & 0x10) == 0x10);
				bHaveF54Query27 =
					((Data[q_offset] & 0x80) == 0x80);

				if (bHaveF54Query26)
					q_offset++;

				if (bHaveF54Query27) {
					q_offset++;
					bHaveF54Ctrl110 = ((Data[q_offset] & 0x01) == 0x01);
					bHaveF54Ctrl111 = ((Data[q_offset] & 0x04) == 0x04);
					bHaveF54Ctrl112 = ((Data[q_offset] & 0x08) == 0x08);
					bHaveF54Ctrl113 = ((Data[q_offset] & 0x10) == 0x10);
					bHaveF54Ctrl114 = ((Data[q_offset] & 0x40) == 0x40);
					bHaveF54Query29 = ((Data[q_offset] & 0x80) == 0x80);
				}
				if (bHaveF54Query28)
					q_offset++;
				if (bHaveF54Query29) {
					q_offset++;
					bHaveF54Ctrl115 = ((Data[q_offset] & 0x01) == 0x01);
					bHaveF54Ctrl116 = ((Data[q_offset] & 0x10) == 0x10);
					bHaveF54Ctrl117 = ((Data[q_offset] & 0x40) == 0x40);
					bHaveF54Query30 = ((Data[q_offset] & 0x80) == 0x80);
				}
				if (bHaveF54Query30) {
					q_offset++;
					bHaveF54Ctrl118 = ((Data[q_offset] & 0x01) == 0x01);
					bHaveF54Ctrl119 = ((Data[q_offset] & 0x02) == 0x02);
					bHaveF54Ctrl120 = ((Data[q_offset] & 0x04) == 0x04);
					bHaveF54Ctrl121 = ((Data[q_offset] & 0x08) == 0x08);
					bHaveF54Ctrl122 = ((Data[q_offset] & 0x10) == 0x10);
					bHaveF54Query31 = ((Data[q_offset] & 0x10) == 0x10);
					bHaveF54Ctrl123 = ((Data[q_offset] & 0x40) == 0x40);
					bHaveF54Query32 = ((Data[q_offset] & 0x80) == 0x80);
				}
				if (bHaveF54Query31)
					q_offset++;
				if (bHaveF54Query32) {
					q_offset++;
					bHaveF54Ctrl125 = ((Data[q_offset] & 0x01) == 0x01);
					bHaveF54Ctrl126 = ((Data[q_offset] & 0x02) == 0x02);
					bHaveF54Ctrl127 = ((Data[q_offset] & 0x04) == 0x04);
					bHaveF54Query33 = ((Data[q_offset] & 0x10) == 0x10);
					bHaveF54Query34 = ((Data[q_offset] & 0x40) == 0x40);
					bHaveF54Query35 = ((Data[q_offset] & 0x80) == 0x80);
				}
				if (bHaveF54Query33) {
					q_offset++;
					bHaveF54Ctrl128 = ((Data[q_offset] & 0x01) == 0x01);
					bHaveF54Ctrl129 = ((Data[q_offset] & 0x02) == 0x02);
					bHaveF54Ctrl130 = ((Data[q_offset] & 0x04) == 0x04);
					bHaveF54Ctrl131 = ((Data[q_offset] & 0x08) == 0x08);
					bHaveF54Ctrl132 = ((Data[q_offset] & 0x10) == 0x10);
					bHaveF54Ctrl133 = ((Data[q_offset] & 0x20) == 0x20);
					bHaveF54Ctrl134 = ((Data[q_offset] & 0x40) == 0x40);
					bHaveF54Query36 = ((Data[q_offset] & 0x80) == 0x80);
				}
				if (bHaveF54Query34)
					q_offset++;
				if (bHaveF54Query35) {
					q_offset++;
					bHaveF54Ctrl135 = ((Data[q_offset] & 0x02) == 0x02);
					bHaveF54Ctrl136 = ((Data[q_offset] & 0x04) == 0x04);
					bHaveF54Ctrl137 = ((Data[q_offset] & 0x08) == 0x08);
					bHaveF54Ctrl138 = ((Data[q_offset] & 0x10) == 0x10);
					bHaveF54Ctrl139 = ((Data[q_offset] & 0x20) == 0x20);
					bHaveF54Ctrl140 = ((Data[q_offset] & 0x80) == 0x80);
				}
				if (bHaveF54Query36) {
					q_offset++;
					bHaveF54Ctrl141 = ((Data[q_offset] & 0x01) == 0x01);
					bHaveF54Ctrl142 = ((Data[q_offset] & 0x02) == 0x02);
					bHaveF54Query37 = ((Data[q_offset] & 0x04) == 0x04);
					bHaveF54Ctrl143 = ((Data[q_offset] & 0x08) == 0x08);
					bHaveF54Ctrl144 = ((Data[q_offset] & 0x10) == 0x10);
					bHaveF54Ctrl145 = ((Data[q_offset] & 0x20) == 0x20);
					bHaveF54Ctrl146 = ((Data[q_offset] & 0x40) == 0x40);
					bHaveF54Query38 = ((Data[q_offset] & 0x80) == 0x80);
				}
				if (bHaveF54Query37)
					q_offset++;
				if (bHaveF54Query38) {
					q_offset++;
					bHaveF54Ctrl147 = ((Data[q_offset] & 0x01) == 0x01);
					bHaveF54Ctrl148 = ((Data[q_offset] & 0x02) == 0x02);
					bHaveF54Ctrl149 = ((Data[q_offset] & 0x04) == 0x04);
					bHaveF54Query39 = ((Data[q_offset] & 0x80) == 0x80);
				}
				if (bHaveF54Query39) {
					q_offset++;
					bHaveF54Ctrl154 = ((Data[q_offset] & 0x01) == 0x01);
					bHaveF54Query40 = ((Data[q_offset] & 0x80) == 0x80);
				}
				if (bHaveF54Query40) {
					q_offset++;
					bHaveF54Ctrl163 = bHaveF54Query41 =
						((Data[q_offset] & 0x02) == 0x02);
					bHaveF54Ctrl164 =
						((Data[q_offset] & 0x04) == 0x04);
					bHaveF54Ctrl165 = bHaveF54Query42 =
						((Data[q_offset] & 0x08) == 0x08);
					bHaveF54Ctrl166 = ((Data[q_offset] & 0x10) == 0x10);
					bHaveF54Ctrl167 = ((Data[q_offset] & 0x20) == 0x20);
					bHaveF54Query43 = ((Data[q_offset] & 0x80) == 0x80);
				}
				if (bHaveF54Query41)
					q_offset++;
				if (bHaveF54Query42)
					q_offset++;
				if (bHaveF54Query43) {
					q_offset++;
					bHaveF54Ctrl172 = bHaveF54Query44 = bHaveF54Query45 =
						((Data[q_offset] & 0x08) == 0x08);
					bHaveF54Ctrl173 = ((Data[q_offset] & 0x10) == 0x10);
					bHaveF54Ctrl175 = ((Data[q_offset] & 0x40) == 0x40);
					bHaveF54Query46 = ((Data[q_offset] & 0x80) == 0x80);
				}
				if (bHaveF54Query44)
					q_offset++;
				if (bHaveF54Query45)
					q_offset++;
				if (bHaveF54Query46) {
					q_offset++;
					bHaveF54Ctrl176 = ((Data[q_offset] & 0x01) == 0x01);
					bHaveF54Ctrl179 = ((Data[q_offset] & 0x04) == 0x04);
					bHaveF54Query47 = ((Data[q_offset] & 0x80) == 0x80);
				}
				if (bHaveF54Query47) {
					q_offset++;
					bHaveF54Ctrl185 = ((Data[q_offset] & 0x10) == 0x10);
					bHaveF54Ctrl186 = ((Data[q_offset] & 0x20) == 0x20);
					bHaveF54Ctrl187 = ((Data[q_offset] & 0x40) == 0x40);
					bHaveF54Query49 = ((Data[q_offset] & 0x80) == 0x80);
				}
				if (bHaveF54Query49) {
					q_offset++;
					bHaveF54Ctrl188 = ((Data[q_offset] & 0x04) == 0x04);
					bHaveF54Ctrl189 = ((Data[q_offset] & 0x20) == 0x20);
				}
				if (bHaveF54Query50) {
					q_offset++;
					bHaveF54Query51 = ((Data[q_offset] & 0x80) == 0x80);
				}
				if (bHaveF54Query51) {
					q_offset++;
					bHaveF54Ctrl198 = ((Data[q_offset] & 0x20) == 0x20);
					bHaveF54Query54 = bHaveF54Query53 =
						((Data[q_offset] & 0x20) == 0x20);
				}
				if (bHaveF54Query53) {
					q_offset++;
					F54Query54Sub0 = (Data[q_offset]);
					q_offset++;
					F54Query54Sub1 = (Data[q_offset]);
				}
				if (bHaveF54Query54) {
					q_offset++;	/* ANALOG_QUERY54(00)/00 */
					q_offset++;	/* ANALOG_QUERY54(01)/00 */
					q_offset += F54Query54Sub0;	/* ANALOG_QUERY54(02)/00 */
					q_offset++;	/* command 0 */
					F54Ctrl198_00_Size = (Data[q_offset]);
					q_offset += 2;	/* command 1 */
					F54Ctrl198_01_Size = (Data[q_offset]);
				}
				/* from Ctrl 103 */
				bHaveF54Ctrl124 = false;	/* Reserved */
				if (bHaveF54Ctrl103)
					offset++;
				if (bHaveF54Ctrl104)
					offset++;
				if (bHaveF54Ctrl105)
					offset++;
				if (bHaveF54Ctrl106)
					offset++;
				if (bHaveF54Ctrl107)
					offset++;
				if (bHaveF54Ctrl108)
					offset++;
				if (bHaveF54Ctrl109)
					offset++;
				if (bHaveF54Ctrl110)
					offset++;
				if (bHaveF54Ctrl111)
					offset++;
				if (bHaveF54Ctrl112)
					offset++;
				if (bHaveF54Ctrl113)
					offset++;
				if (bHaveF54Ctrl114)
					offset++;
				if (bHaveF54Ctrl115)
					offset++;
				if (bHaveF54Ctrl116)
					offset++;
				if (bHaveF54Ctrl117)
					offset++;
				if (bHaveF54Ctrl118)
					offset++;
				if (bHaveF54Ctrl119)
					offset++;
				if (bHaveF54Ctrl120)
					offset++;
				if (bHaveF54Ctrl121)
					offset++;
				if (bHaveF54Ctrl122)
					offset++;
				if (bHaveF54Ctrl123)
					offset++;
				if (bHaveF54Ctrl124)
					offset++;
				if (bHaveF54Ctrl125)
					offset++;
				if (bHaveF54Ctrl126)
					offset++;
				if (bHaveF54Ctrl127)
					offset++;
				if (bHaveF54Ctrl128)
					offset++;
				if (bHaveF54Ctrl129)
					offset++;
				if (bHaveF54Ctrl130)
					offset++;
				if (bHaveF54Ctrl131)
					offset++;
				if (bHaveF54Ctrl132)
					offset++;
				if (bHaveF54Ctrl133)
					offset++;
				if (bHaveF54Ctrl134)
					offset++;
				if (bHaveF54Ctrl135)
					offset++;
				if (bHaveF54Ctrl136)
					offset++;
				if (bHaveF54Ctrl137)
					offset++;
				if (bHaveF54Ctrl138)
					offset++;
				if (bHaveF54Ctrl139)
					offset++;
				if (bHaveF54Ctrl140)
					offset++;
				if (bHaveF54Ctrl141)
					offset++;
				if (bHaveF54Ctrl142)
					offset++;
				if (bHaveF54Ctrl143)
					offset++;
				if (bHaveF54Ctrl144)
					offset++;
				if (bHaveF54Ctrl145)
					offset++;
				if (bHaveF54Ctrl146)
					offset++;
				if (bHaveF54Ctrl147)
					offset++;
				if (bHaveF54Ctrl148)
					offset++;
				if (bHaveF54Ctrl149) {
					offset++;
					F54Ctrl149Offset = offset;
				}
				if (bHaveF54Ctrl150)
					offset++;
				if (bHaveF54Ctrl151)
					offset++;
				if (bHaveF54Ctrl112)
					offset++;
				if (bHaveF54Ctrl153)
					offset++;
				if (bHaveF54Ctrl154)
					offset++;
				if (bHaveF54Ctrl155)
					offset++;
				if (bHaveF54Ctrl156)
					offset++;
				if (bHaveF54Ctrl157)
					offset++;
				if (bHaveF54Ctrl158)
					offset++;
				if (bHaveF54Ctrl159)
					offset++;
				if (bHaveF54Ctrl150)
					offset++;
				if (bHaveF54Ctrl161)
					offset++;
				if (bHaveF54Ctrl162)
					offset++;
				if (bHaveF54Ctrl163)
					offset++;
				if (bHaveF54Ctrl164)
					offset++;
				if (bHaveF54Ctrl165)
					offset++;
				if (bHaveF54Ctrl166)
					offset++;
				if (bHaveF54Ctrl167)
					offset++;
				if (bHaveF54Ctrl168)
					offset++;
				if (bHaveF54Ctrl169)
					offset++;
				if (bHaveF54Ctrl170)
					offset++;
				if (bHaveF54Ctrl171)
					offset++;
				if (bHaveF54Ctrl172)
					offset++;
				if (bHaveF54Ctrl173)
					offset++;
				if (bHaveF54Ctrl174)
					offset++;
				if (bHaveF54Ctrl175)
					offset++;
				if (bHaveF54Ctrl176)
					offset++;
				if (bHaveF54Ctrl177)
					offset++;
				if (bHaveF54Ctrl178)
					offset++;
				if (bHaveF54Ctrl179)
					offset++;
				if (bHaveF54Ctrl180)
					offset++;
				if (bHaveF54Ctrl181)
					offset++;
				if (bHaveF54Ctrl182)
					offset++;
				if (bHaveF54Ctrl183)
					offset++;
				if (bHaveF54Ctrl184)
					offset++;
				if (bHaveF54Ctrl185)
					offset++;
				if (bHaveF54Ctrl186)
					offset++;
				if (bHaveF54Ctrl187)
					offset++;
				if (bHaveF54Ctrl188) {
					offset++;
					F54Ctrl188Offset = offset;
				}
				if (bHaveF54Ctrl189) {
					offset++;
					F54Ctrl189Offset = offset;
				}
			}
			break;

		case 0x55:
			if (!bHaveF55) {
				Read8BitRegisters(dev, (cAddr - 3), &F55ControlBase, 1);
				Read8BitRegisters(dev, (cAddr - 5), &F55QueryBase, 1);

				Read8BitRegisters(dev, F55QueryBase, &RxChannelCount, 1);
				Read8BitRegisters(dev, (F55QueryBase+1), &TxChannelCount, 1);

				rxCount = 0;
				txCount = 0;

				/* Read Sensor Mapping */
				Read8BitRegisters(dev, (F55ControlBase + 1), Data,
						(int)RxChannelCount);

				for (i = 0; i < (int)RxChannelCount; i++) {
					if (Data[i] != 0xFF) {
						rxCount++;
						TRxPhysical[i] = Data[i];
					} else
						break;
				}
				Read8BitRegisters(dev, (F55ControlBase + 2), Data,
						(int)TxChannelCount);

				for (i = 0; i < (int)TxChannelCount; i++) {
					if (Data[i] != 0xFF) {
						TRxPhysical[rxCount + i] = Data[i];
						txCount++;
					} else
						break;
				}

				for (i = (rxCount + txCount);
						i < (TRX_mapping_max); i++) {
					TRxPhysical[i] = 0xFF;
				}

				RxChannelCount = rxCount;
				TxChannelCount = txCount;

				if (TRX_MAX <= RxChannelCount)
					RxChannelCount = TRX_MAX;
				if (TRX_MAX <= TxChannelCount)
					TxChannelCount = TRX_MAX;

				MaxArrayLength = (int)RxChannelCount * (int)TxChannelCount * 2;
				if (((int)TxChannelCount - F12_2DTxCount == 0)
						&& ButtonCount > 0) {
					ButtonShared = true;
				}
			}
			break;
		default:	/* Any other function */
			break;
		}
		cAddr -= 6;
	} while (true);
}

/*
The following function is necessary to setup the Function $54 tests.i
The setup needs to be done once
after entering into page 0x01. As long as the touch controller stays in page1
the setup does not
need to be repeated.
*/
int TestPreparation(struct device *dev)
{
	unsigned char data = 0;
	unsigned char addr = 0;
	unsigned int count = 0;

	/* Turn off CBC. */
	if (bHaveF54Ctrl07) {
		addr = F54ControlBase + F54Ctrl07Offset;
		Read8BitRegisters(dev, addr, &data, 1);
		/* data = data & 0xEF;*/
		data = 0;
		Write8BitRegisters(dev, addr, &data, 1);
	} else if (bHaveCtrl88) {
		addr = F54ControlBase + F54Ctrl88Offset;
		Read8BitRegisters(dev, addr, &data, 1);
		data = data & 0xDF;
		Write8BitRegisters(dev, addr, &data, 1);
	}

	/* Turn off CBC2. */
	if (bHaveF54Ctrl149) {
		addr = F54ControlBase + F54Ctrl149Offset;
		Read8BitRegisters(dev, addr, &data, 1);
		data = data & 0xFE;
		Write8BitRegisters(dev, addr, &data, 1);
	}

	/* Turn off 0D CBC. */
	if (bHaveF54Ctrl57) {
		addr = F54ControlBase + F54Ctrl57Offset;
		Read8BitRegisters(dev, addr, &data, 1);
		/*ata = data & 0xEF;*/
		data = 0;
		Write8BitRegisters(dev, addr, &data, 1);
	}

	/* Turn off SignalClarity.
	   ForceUpdate is required for the change to be effective
	*/
	if (bHaveSignalClarity) {
		addr = F54ControlBase + F54Ctrl41Offset;
		Read8BitRegisters(dev, addr, &data, 1);
		data = data | 0x01;
		Write8BitRegisters(dev, addr, &data, 1);
	}

	/* Apply ForceUpdate. */
	Read8BitRegisters(dev, F54CommandBase, &data, 1);
	data = data | 0x04;
	Write8BitRegisters(dev, F54CommandBase, &data, 1);

	/* Wait complete */
	do {
		Read8BitRegisters(dev, F54CommandBase, &data, 1);
		touch_msleep(3);
		count++;
	} while (data != 0x00 && (count < DefaultTimeout));

	if (count >= DefaultTimeout) {
		TOUCH_E("Timeout -- ForceUpdate can not complete\n");
		Reset(dev);
		return -EAGAIN;
	}

	/* Apply ForceCal. */
	Read8BitRegisters(dev, F54CommandBase, &data, 1);
	data = data | 0x02;
	Write8BitRegisters(dev, F54CommandBase, &data, 1);

	/* Wait complete */
	count = 0;
	do {
		Read8BitRegisters(dev, F54CommandBase, &data, 1);
		touch_msleep(3);
		count++;
	} while (data != 0x00 && (count < DefaultTimeout));

	if (count >= DefaultTimeout) {
		TOUCH_E("Timeout -- ForceCal can not complete\n");
		Reset(dev);
		return -EAGAIN;
	}

	return 1;
}

int IncellTestMode(struct device *dev)
{
	unsigned char data = 0;
	unsigned int count = 0;

	/* Enter Incell Test Mode */
	Read8BitRegisters(dev, F54CommandBase, &data, 1);
	data = data | 0x10;
	Write8BitRegisters(dev, F54CommandBase, &data, 1);

	/* Wait complete */
	do {
		Read8BitRegisters(dev, F54CommandBase, &data, 1);
		touch_msleep(3);
		count++;
	} while (data != 0x00 && (count < DefaultTimeout));

	if (count >= DefaultTimeout) {
		TOUCH_E("Timeout -- Enter Incell Test Mode can not complete\n");
		Reset(dev);
		return -EAGAIN;
	}

	return 1;
}
/*
The following funtion illustrates the steps in getting
a full raw image report (report #20) by Function $54.
*/
int ImageTest(struct device *dev, int mode, char *buf)
{
	unsigned char data;
	u16 report_type;
	int ret = 0;

	ret = TestPreparation(dev);

	if (ret == 1) {
		memcpy(LowerImageLimit, LowerImage, sizeof(LowerImageLimit));
		memcpy(UpperImageLimit, UpperImage, sizeof(UpperImageLimit));

		/* Assign report type for Full Raw Image */
		data = eRT_FullRawCapacitance;	/* Raw Capacitance mode */

		Write8BitRegisters(dev, F54DataBase, &data, 1);

		do_gettimeofday(&t_interval[STARTTIME]);

		if (mode == 1)
			report_type = eRT_RawImageRT3;	/* Rawdata Display mode */
		else
			report_type = eRT_FullRawCapacitance;	/* Rawdata Store mode */

		ret = ReadReport(dev, report_type, buf);
	} else {
		ret = -ECANCELED;
	}

	return ret;
}

int DeltaTest(struct device *dev, char *buf)
{
	unsigned char data;
	u16 report_type;

	memcpy(LowerImageLimit, LowerImage, sizeof(LowerImageLimit));
	memcpy(UpperImageLimit, UpperImage, sizeof(UpperImageLimit));

	/* Assign report type for Full Raw Image */
	data = eRT_Normalized16BitImageReport;	/* Delta mode */

	Write8BitRegisters(dev, F54DataBase, &data, 1);

	do_gettimeofday(&t_interval[STARTTIME]);

	report_type = eRT_RawImageRT3;

	return ReadReport(dev, report_type, buf);
}


int NoiseTest(struct device *dev, char *buf)
{
	unsigned char data;
	u16 report_type;

	memset(NoiseDeltaMin, 0, TRX_MAX * TRX_MAX * sizeof(short));
	memset(NoiseDeltaMax, 0, TRX_MAX * TRX_MAX * sizeof(short));

	/* Assign report type for Full Raw Image */
	data = eRT_Normalized16BitImageReport;	/* Delta mode */

	Write8BitRegisters(dev, F54DataBase, &data, 1);

	report_type = eRT_Normalized16BitImageReport;

	return ReadReport(dev, report_type, buf);
}

/*
 * The following funtion illustrates the steps in getting
 a sensor speed test report (report #22) by Function $54.
*/
int SensorSpeed(struct device *dev, char *buf)
{
	unsigned char data;
	u16 report_type;

	memcpy(SensorSpeedLowerImageLimit, SensorSpeedLowerImage,
			sizeof(SensorSpeedLowerImageLimit));
	memcpy(SensorSpeedUpperImageLimit, SensorSpeedUpperImage,
			sizeof(SensorSpeedUpperImageLimit));

	/* Assign report type for Sensor Speed Test */
	data = eRT_SensorSpeed;		/* Sensor Speed mode */
	Write8BitRegisters(dev, F54DataBase, &data, 1);

	do_gettimeofday(&t_interval[STARTTIME]);

	report_type = eRT_SensorSpeed;

	return ReadReport(dev, data, buf);
}

/* The following funtion illustrates
the steps in getting a ADC Range report (report #23) by Function $54.
*/
int ADCRange(struct device *dev, char *buf)
{
	unsigned char data;
	u16 report_type;

	memcpy(ADCLowerImageLimit, ADCLowerImage, sizeof(ADCLowerImageLimit));
	memcpy(ADCUpperImageLimit, ADCUpperImage, sizeof(ADCUpperImageLimit));

	Read8BitRegisters(dev, (F54ControlBase + F54Ctrl41Offset), &data, 1);
	if (data & 0x01)
		SignalClarityOn = false;
	else
		SignalClarityOn = true;

	/* Assign report type for ADC Range report */
	data = eRT_ADCRange;	/* ADC Range mode */
	Write8BitRegisters(dev, F54DataBase, &data, 1);

	do_gettimeofday(&t_interval[STARTTIME]);

	report_type = eRT_ADCRange;

	return ReadReport(dev, report_type, buf);
}

void AbsADCRange(struct device *dev, char *buf)
{
	unsigned char data;
	u16 report_type;

	if (TestPreparation(dev)) {
		/* Assign report type for Abs Sensing ADC Range report */
		data = eRT_AbsADCRange;		/* Abs ADC Range mode */
		Write8BitRegisters(dev, F54DataBase, &data, 1);

		do_gettimeofday(&t_interval[STARTTIME]);

		report_type = eRT_AbsADCRange;

		ReadReport(dev, report_type, buf);
	}
}

/* report type 40 */
int AbsDelta(struct device *dev, char *buf)
{
	unsigned char data;
	u16 report_type;

	/* Assign report type for Abs Sensing Delta Capacitance report */
	data = eRT_AbsDelta;	/* Abs Delta mode */
	Write8BitRegisters(dev, F54DataBase, &data, 1);

	do_gettimeofday(&t_interval[STARTTIME]);

	report_type = eRT_AbsDelta;

	return ReadReport(dev, report_type, buf);
}

/* report type 38 */
int AbsRaw(struct device *dev, int mode, char *buf)
{
	unsigned char data;
	u16 report_type;
	int ret = 0;

	if (bHaveCtrl98) {
		Read8BitRegisters(dev, (F54ControlBase + F54Ctrl98Offset),
				&Data[0], 6);

		/* AbsRx Low Reference */
		AbsRxShortLimit = AbsRawRef[Data[0]] * 275/100;

		/* AbsTx Low Reference */
		AbsTxShortLimit = AbsRawRef[Data[5]] * 275/100;

		/* AbsTx Low Reference */
		AbsTxOpenLimit =  AbsRawRef[Data[5]] * 75/100;
	}

	/* Assign report type for Abs Sensing Raw Capacitance report */
	data = eRT_AbsRaw;	/* Abs Raw mode */
	Write8BitRegisters(dev, F54DataBase, &data, 1);

	do_gettimeofday(&t_interval[STARTTIME]);

	if (mode == 0)
		report_type = eRT_AbsShort;	/* Abs Sensing Short Test mode */
	else if (mode == 1)
		report_type = eRT_AbsOpen;	/* Abs Sensing Open Test mode */
	else if (mode == 2)
		report_type = eRT_AbsRaw;	/* Abs Sensing Rawdata Test mode */
	else
		report_type = eRT_AbsLpwgRaw;	/* Abs Sensing LPWG Rawdata Test mode */

	ret = ReadReport(dev, report_type, buf);

	return ret;
}

/* The following funtion illustrates the steps
in getting a TRex-Opens(No sensor) report (report #24) by Function $54.
*/
void TRexOpenTest(struct device *dev, char *buf)
{
	unsigned char data;
	u16 report_type;

	if (TestPreparation(dev)) {
		/* Assign report type for TRex Open Test */
		data = eRT_TRexOpen;	/* TRex Open mode */
		Write8BitRegisters(dev, F54DataBase, &data, 1);

		report_type = eRT_TRexOpen;
		ReadReport(dev, report_type, buf);
	}
}

/*The following funtion illustrates the steps
in getting a TRex-to-GND(No sensor) report (report #25) by Function $54.
*/
int TRexGroundTest(struct device *dev, char *buf)
{
	unsigned char data;
	unsigned char buffer[2] = {0};
	u16 report_type;
	int ret;
	int pre_ret, mode_ret;

	pre_ret = TestPreparation(dev);
	mode_ret = IncellTestMode(dev);

	if (pre_ret && mode_ret) {
		/* Assign report type for TRex Ground Test*/
		data = eRT_TRexGround;	/* TRex Ground mode */
		Write8BitRegisters(dev, F54DataBase, &data, 1);

		/* Reset Index */
		Write8BitRegisters(dev, F54DataBase + 1, &buffer[0], 2);

		report_type = eRT_TRexGround;

		ret = ReadReport(dev, report_type, buf);
	} else {
		ret = -ECANCELED;
	}

	return ret;
}

/* The following funtion illustrates the steps
in getting a TRex-TRex short(No sensor) report (report #26) by Function $54.
*/
int TRexShortTest(struct device *dev, char *buf)
{
	unsigned char data;
	unsigned char buffer[2] = {0};
	u16 report_type;
	int ret;
	int pre_ret, mode_ret;

	pre_ret = TestPreparation(dev);
	mode_ret = IncellTestMode(dev);

	if (pre_ret && mode_ret) {
		/* Assign report type for TRex Short Test*/
		data = eRT_TRexShort;	/* TRex Short mode */
		Write8BitRegisters(dev, F54DataBase, &data, 1);

		/* Reset Index */
		Write8BitRegisters(dev, F54DataBase + 1, &buffer[0], 2);

		report_type = eRT_TRexShort;

		ret = ReadReport(dev, report_type, buf);
	} else {
		ret = -ECANCELED;
	}

	return ret;
}

/* This test is to retreive the high resistance report, report type #4. */
int HighResistanceTest(struct device *dev, char *buf)
{
	unsigned char data;
	u16 report_type;
	int ret;

	ret = TestPreparation(dev);

	if (ret == 1) {
		/* Assign report type for High Resistance report*/
		data = eRT_HighResistance;	/* High Resistance mode */
		Write8BitRegisters(dev, F54DataBase, &data, 1);

		report_type = eRT_HighResistance;

		ret = ReadReport(dev, report_type, buf);
	} else {
		ret = -ECANCELED;
	}

	return ret;
}

/* This test is to retreive the maximum and minimum pixel report, report type #13. */
void MaxMinTest(struct device *dev, char *buf)
{
	unsigned char data;
	u16 report_type;

	if (TestPreparation(dev)) {
		/* Assign report type for Max Min report */
		/* Raw Capacitance Max/Min mode */
		data = eRT_FullRawCapacitanceMaxMin;
		Write8BitRegisters(dev, F54DataBase, &data, 1);

		report_type = eRT_FullRawCapacitanceMaxMin;
		ReadReport(dev, report_type, buf);
	}
}

void SCAN_PDT(struct device *dev)
{
	int i;

	for (i = 0; i < scanMaxPageCount; i++) {
		if (switchPage(dev, i))
			RunQueries(dev);
	}
}

/* Main entry point for the application */
int F54Test(struct device *dev, u16 input, int mode, char *buf)
{
	int ret = 0;
	unsigned char data;
	int retry_cnt1 = 0;
	int retry_cnt2 = 0;

retry:
	ret = switchPage(dev, 0x01);
	if (ret == -EAGAIN && ++retry_cnt1 <= 3) {
		TOUCH_I("retry switchPage, count = %d\n", retry_cnt1);
		goto retry;
	} else if (ret == 0) {
		return ret;
	}

	data = 0x00;
	Write8BitRegisters(dev, F54DataBase + 1, &data, 1);
	Write8BitRegisters(dev, F54DataBase + 2, &data, 1);

	f54len = 0;
	f54buf[0] = 0;
	wlen = 0;
	wbuf[0] = 0;

	switch (input) {
	case eRT_FullRawCapacitance:
		ret = ImageTest(dev, mode, buf);
		break;
	case eRT_ADCRange:
		ret = ADCRange(dev, buf);
		break;
	case eRT_SensorSpeed:
		ret = SensorSpeed(dev, buf);
		break;
	case eRT_TRexOpen:
		TRexOpenTest(dev, buf);
		break;
	case eRT_TRexGround:
		ret = TRexGroundTest(dev, buf);
		break;
	case eRT_TRexShort:
		ret = TRexShortTest(dev, buf);
		break;
	case eRT_HighResistance:
		ret = HighResistanceTest(dev, buf);
		break;
	case eRT_FullRawCapacitanceMaxMin:
		MaxMinTest(dev, buf);
		break;
	case eRT_AbsADCRange:
		AbsADCRange(dev, buf);
		break;
	case eRT_AbsDelta:
		ret = AbsDelta(dev, buf);
		break;
	case eRT_AbsRaw:
		ret = AbsRaw(dev, mode, buf);
		break;
	case eRT_Normalized16BitImageReport:
		if (mode == 1)
			ret = NoiseTest(dev, buf);
		else
			ret = DeltaTest(dev, buf);
		break;
	default:
		return -EINVAL;
	}

	if (switchPage(dev, 0x00) != true) {
		TOUCH_I("switchPage failed\n");

		/*Reset Device*/
		Reset(dev);
	}

	if (ret == -EAGAIN && ++retry_cnt2 <= 3) {
		TOUCH_I("retry Test, count = %d\n", retry_cnt2);
		goto retry;
	}

	return ret;
}
