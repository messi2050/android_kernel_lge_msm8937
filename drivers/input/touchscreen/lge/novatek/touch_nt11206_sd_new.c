/* touch_nt11206_sd.c
 *
 * Copyright (C) 2015 LGE.
 *
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
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/syscalls.h>
#include <linux/file.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>

/*
 *  Include to touch core Header File
 */
#include <touch_hwif.h>
#include <touch_core.h>

/*
 *  Include to Local Header File
 */
#include "touch_nt11206.h"



#define I2C_TANSFER_LENGTH  64

#define AIN_TX_NUM	20
#define AIN_RX_NUM	30
#define PSConfig_Tolerance_Postive			250
#define PSConfig_Tolerance_Negative			-250
#define PSConfig_DiffLimitG_Postive			250
#define PSConfig_DiffLimitG_Negative		-250
#define PSConfig_Tolerance_Postive_Short	250
#define PSConfig_Tolerance_Negative_Short	-250
#define PSConfig_DiffLimitG_Postive_Short	250
#define PSConfig_DiffLimitG_Negative_Short	-250
#define PSConfig_Tolerance_Postive_Mutual	250
#define PSConfig_Tolerance_Negative_Mutual	-250
#define PSConfig_DiffLimitG_Postive_Mutual	250
#define PSConfig_DiffLimitG_Negative_Mutual	-250
#define PSConfig_Tolerance_Postive_FW		250
#define PSConfig_Tolerance_Negative_FW		-250
#define PSConfig_DiffLimitG_Postive_FW		250
#define PSConfig_DiffLimitG_Negative_FW		-250

#define PSConfig_Rawdata_Limit_Postive_Short_RXRX	-1500
#define PSConfig_Rawdata_Limit_Negative_Short_RXRX	-8600
#define PSConfig_Rawdata_Limit_Postive_Short_TXRX	-1500
#define PSConfig_Rawdata_Limit_Negative_Short_TXRX	-6500
#define PSConfig_Rawdata_Limit_Postive_Short_TXTX	-1000
#define PSConfig_Rawdata_Limit_Negative_Short_TXTX	-10000

#define NORMAL_MODE 0x00
#define TEST_MODE_1 0x21    //before algo.
#define TEST_MODE_2 0x22    //after algo.
#define BASELINE_ADDR   0x11054
#define XDATA_SECTOR_SIZE   256
s16 BoundaryShort_RXRX[30] = {
	-2938, -2950, -2930, -2941, -2966, -2957, -2987, -2969,	-2964, -2975, -2988, -2986, -3014, -2982, -2980, -2982,	-2973, -3004, -3023, -2994, -2988, -3006, -3013, -3023,	-3026, -3032, -2997, -3029, -3008, -3036,
};

s16 BoundaryShort_TXRX[30 * 20] = {
	-2858, -2884, -2873, -2839, -2903, -2904, -2885, -2903, -2907, -2871, -2925, -2932, -2913, -2921, -2930, -2878, -2914, -2952, -2927, -2936, -2936, -2912, -2955, -2977, -2928, -2973, -2942, -2930, -2949, -2984,
	-2920, -2893, -2880, -2897, -2913, -2908, -2945, -2914, -2917, -2936, -2935, -2941, -2975, -2930, -2937, -2942, -2924, -2961, -2988, -2945, -2943, -2971, -2966, -2982, -2988, -2985, -2952, -2991, -2961, -2993,
	-2910, -2890, -2879, -2889, -2910, -2908, -2934, -2911, -2917, -2925, -2933, -2941, -2966, -2928, -2937, -2933, -2920, -2961, -2978, -2944, -2943, -2962, -2963, -2980, -2979, -2981, -2955, -2984, -2959, -2995,
	-2906, -2890, -2880, -2886, -2910, -2909, -2930, -2911, -2918, -2921, -2933, -2941, -2962, -2926, -2937, -2931, -2921, -2962, -2974, -2943, -2944, -2958, -2962, -2980, -2978, -2981, -2957, -2981, -2958, -2998,
	-2904, -2889, -2879, -2882, -2909, -2908, -2928, -2911, -2918, -2919, -2932, -2939, -2959, -2925, -2936, -2928, -2920, -2961, -2971, -2943, -2942, -2956, -2961, -2980, -2975, -2980, -2956, -2978, -2958, -2994,
	-2903, -2888, -2878, -2882, -2910, -2908, -2927, -2911, -2918, -2918, -2932, -2940, -2959, -2927, -2937, -2928, -2920, -2962, -2972, -2942, -2943, -2955, -2961, -2980, -2974, -2979, -2956, -2978, -2957, -2995,
	-2903, -2889, -2880, -2881, -2909, -2907, -2927, -2910, -2919, -2919, -2932, -2941, -2958, -2927, -2937, -2927, -2921, -2963, -2972, -2942, -2943, -2954, -2962, -2979, -2975, -2980, -2956, -2978, -2957, -2996,
	-2903, -2889, -2881, -2882, -2910, -2908, -2928, -2910, -2919, -2917, -2932, -2940, -2958, -2927, -2938, -2928, -2920, -2962, -2970, -2943, -2944, -2955, -2962, -2979, -2975, -2980, -2956, -2978, -2957, -2997,
	-2901, -2888, -2880, -2880, -2907, -2907, -2926, -2911, -2917, -2917, -2932, -2940, -2958, -2926, -2936, -2926, -2919, -2962, -2970, -2941, -2943, -2953, -2962, -2980, -2974, -2979, -2955, -2976, -2957, -2995,
	-2902, -2890, -2879, -2882, -2909, -2908, -2927, -2911, -2918, -2917, -2932, -2939, -2958, -2926, -2936, -2928, -2919, -2961, -2971, -2943, -2944, -2955, -2963, -2979, -2974, -2979, -2956, -2978, -2958, -2996,
	-2869, -2882, -2878, -2847, -2903, -2905, -2894, -2904, -2916, -2882, -2926, -2938, -2924, -2921, -2935, -2892, -2914, -2961, -2936, -2937, -2943, -2919, -2956, -2977, -2938, -2974, -2954, -2943, -2950, -2993,
	-2919, -2891, -2881, -2898, -2912, -2909, -2943, -2915, -2919, -2934, -2935, -2941, -2975, -2930, -2937, -2944, -2924, -2963, -2988, -2946, -2945, -2972, -2966, -2981, -2991, -2983, -2957, -2996, -2961, -2998,
	-2910, -2890, -2881, -2890, -2911, -2909, -2936, -2912, -2919, -2927, -2934, -2941, -2967, -2928, -2938, -2936, -2921, -2963, -2980, -2944, -2945, -2964, -2964, -2981, -2982, -2981, -2957, -2986, -2959, -2997,
	-2905, -2890, -2880, -2886, -2910, -2908, -2931, -2911, -2919, -2921, -2932, -2940, -2962, -2927, -2937, -2930, -2920, -2961, -2975, -2943, -2944, -2958, -2962, -2979, -2976, -2980, -2956, -2979, -2956, -2996,
	-2905, -2889, -2880, -2883, -2909, -2908, -2929, -2910, -2918, -2919, -2932, -2940, -2959, -2927, -2937, -2928, -2921, -2962, -2972, -2942, -2943, -2956, -2962, -2979, -2975, -2980, -2956, -2979, -2957, -2996,
	-2904, -2889, -2880, -2881, -2910, -2907, -2928, -2910, -2918, -2919, -2932, -2941, -2959, -2926, -2937, -2928, -2919, -2962, -2971, -2942, -2944, -2955, -2962, -2979, -2974, -2980, -2956, -2979, -2958, -2996,
	-2903, -2889, -2880, -2881, -2908, -2907, -2927, -2909, -2919, -2918, -2932, -2939, -2958, -2926, -2937, -2926, -2919, -2962, -2971, -2942, -2943, -2955, -2962, -2980, -2974, -2980, -2956, -2977, -2957, -2995,
	-2904, -2888, -2881, -2881, -2910, -2908, -2927, -2911, -2919, -2917, -2933, -2940, -2959, -2926, -2937, -2928, -2920, -2961, -2971, -2942, -2944, -2955, -2962, -2980, -2975, -2980, -2955, -2979, -2958, -2996,
	-2903, -2890, -2880, -2881, -2909, -2908, -2927, -2911, -2917, -2918, -2932, -2940, -2957, -2926, -2936, -2928, -2919, -2962, -2971, -2943, -2943, -2954, -2962, -2980, -2974, -2980, -2957, -2977, -2958, -2996,
	-2903, -2887, -2880, -2880, -2909, -2907, -2927, -2911, -2919, -2919, -2932, -2940, -2959, -2927, -2936, -2928, -2918, -2963, -2972, -2941, -2943, -2955, -2962, -2980, -2973, -2979, -2956, -2978, -2959, -2996,
};
s16 BoundaryShort_TXTX[20] = {
	-2903, -2888, -2879, -2881, -2909, -2907, -2927, -2910, -2919, -2917, -2932, -2939, -2958, -2926, -2937, -2928, -2920, -2962, -2972, -2941,
};

s16 BoundaryOpen[30*20] = {
	1120,	741,	1280,	1070,	736,	1280,	1040,	729,	1280,	1030,	728,	1290,	1020,	728,	1290,	1010,	715,	1270,	993,	711,
	1280,	997,	711,	1280,	998,	709,	1280,	1000,	711,	1270,	520,	676,	1470,	520,	677,	1460,	512,	672,	1460,	508,
	671,	1450,	506,	671,	1460,	498,	656,	1420,	486,	653,	1440,	486,	653,	1430,	485,	650,	1430,	483,	649,	1420,
	422,	626,	1370,	418,	627,	1360,	420,	625,	1360,	419,	626,	1360,	419,	626,	1360,	419,	614,	1330,	401,	608,
	1340,	402,	608,	1340,	400,	605,	1340,	398,	604,	1330,	434,	638,	1390,	428,	636,	1390,	432,	638,	1380,	432,
	638,	1380,	432,	638,	1390,	432,	632,	1350,	413,	618,	1360,	413,	619,	1360,	412,	616,	1360,	409,	614,	1350,
	394,	586,	1270,	389,	584,	1260,	394,	589,	1260,	394,	589,	1250,	395,	590,	1260,	395,	585,	1240,	382,	571,
	1240,	376,	570,	1230,	375,	567,	1230,	372,	566,	1230,	400,	571,	1230,	398,	571,	1230,	400,	576,	1230,	402,
	576,	1230,	402,	577,	1240,	403,	572,	1210,	398,	563,	1210,	384,	558,	1210,	383,	555,	1210,	380,	554,	1200,
	385,	547,	1130,	381,	544,	1120,	380,	545,	1120,	382,	545,	1110,	382,	545,	1120,	383,	542,	1100,	380,	540,
	1110,	368,	530,	1100,	367,	527,	1100,	365,	526,	1090,	390,	511,	1040,	389,	511,	1040,	387,	510,	1040,	388,
	510,	1030,	388,	511,	1040,	388,	507,	1020,	386,	507,	1030,	378,	497,	1020,	374,	495,	1020,	372,	495,	1010,
	370,	478,	882,	369,	479,	876,	368,	478,	876,	369,	479,	871,	369,	479,	879,	370,	476,	858,	367,	476,
	871,	367,	469,	857,	356,	464,	857,	355,	463,	854,	411,	475,	864,	408,	475,	857,	408,	475,	859,	410,
	474,	853,	410,	477,	867,	413,	475,	845,	407,	472,	853,	410,	471,	840,	398,	461,	841,	402,	465,	850,
	1010,	722,	1290,	1010,	719,	1290,	1000,	717,	1280,	1010,	716,	1280,	1010,	720,	1280,	1010,	717,	1270,	1010,	721,
	1270,	1030,	720,	1260,	1050,	715,	1250,	1080,	718,	1250,	496,	663,	1450,	495,	661,	1450,	494,	661,	1440,	496,
	660,	1440,	497,	662,	1440,	500,	659,	1430,	498,	662,	1436,	505,	663,	1440,	509,	657,	1430,	497,	655,	1440,
	413,	619,	1350,	411,	617,	1350,	411,	617,	1350,	413,	616,	1350,	412,	617,	1350,	414,	615,	1340,	410,	617,
	1340,	413,	617,	1340,	414,	616,	1330,	402,	607,	1340,	423,	628,	1380,	421,	627,	1370,	422,	627,	1370,	423,
	626,	1370,	422,	627,	1370,	425,	626,	1360,	421,	628,	1360,	424,	628,	1360,	425,	628,	1360,	417,	620,	1370,
	386,	580,	1250,	385,	579,	1250,	386,	579,	1240,	387,	578,	1240,	386,	579,	1240,	388,	577,	1240,	384,	579,
	1240,	387,	579,	1240,	387,	579,	1240,	386,	572,	1240,	396,	568,	1230,	394,	567,	1220,	394,	566,	1220,	396,
	566,	1220,	395,	567,	1220,	397,	565,	1210,	393,	567,	1210,	396,	567,	1210,	395,	566,	1210,	390,	559,	1210,
	379,	541,	1120,	376,	539,	1110,	377,	539,	1110,	378,	538,	1110,	378,	539,	1110,	379,	537,	1100,	374,	536,
	1100,	373,	533,	1100,	369,	530,	1090,	367,	528,	1100,	385,	507,	1030,	384,	506,	1030,	384,	505,	1020,	384,
	504,	1020,	381,	502,	1020,	379,	497,	1010,	372,	496,	1010,	374,	495,	1010,	374,	495,	1010,	371,	492,	1020,
	367,	476,	876,	364,	473,	869,	362,	470,	861,	359,	466,	857,	356,	465,	856,	356,	463,	853,	353,	464,
	853,	355,	464,	855,	356,	464,	853,	352,	460,	854,	415,	476,	861,	402,	468,	843,	404,	464,	840,	401,
	469,	834,	403,	469,	846,	398,	470,	834,	398,	464,	837,	397,	470,	834,	399,	464,	836,	392,	466,	834,
};
s16 BoudaryFWMutual[20*30] = {
	1710,	1710,	1708,	1700,	1700,	1700,	1690,	1690,	1703,	1700,	1650,	1660,	1670,	1660,	1680,	1700,	1710,	1720,	1750,	1760,
	728,	738,	745,	744,	747,	751,	750,	752,	760,	761,	715,	721,	726,	730,	737,	753,	755,	763,	778,	776,
	1130,	1140,	1150,	1150,	1150,	1150,	1150,	1150,	1160,	1150,	1150,	1140,	1140,	1150,	1140,	1160,	1160,	1160,	1160,	1150,
	1190,	1180,	1190,	1190,	1180,	1180,	1180,	1180,	1190,	1190,	1180,	1180,	1180,	1190,	1180,	1200,	1200,	1200,	1210,	1210,
	491,	495,	502,	502,	504,	505,	506,	507,	511,	511,	487,	491,	493,	494,	498,	512,	512,	514,	521,	522,
	366,	375,	378,	378,	383,	384,	385,	385,	390,	387,	359,	363,	364,	366,	376,	384,	386,	389,	395,	395,
	372,	378,	378,	377,	379,	380,	380,	380,	381,	378,	366,	369,	369,	370,	382,	384,	384,	386,	385,	386,
	519,	525,	526,	527,	526,	528,	527,	528,	531,	530,	514,	518,	519,	520,	525,	536,	535,	537,	540,	540,
	1150,	1150,	1150,	1150,	1150,	1150,	1150,	1150,	1150,	1150,	1140,	1150,	1150,	1150,	1140,	1160,	1160,	1160,	1170,	1170,
	1130,	1140,	1140,	1136,	1130,	1130,	1130,	1130,	1140,	1140,	1130,	1130,	1130,	1130,	1130,	1150,	1150,	1150,	1150,	1150,
	509,	515,	516,	516,	516,	517,	517,	518,	520,	520,	502,	506,	508,	509,	520,	526,	527,	528,	529,	531,
	357,	362,	362,	362,	363,	363,	364,	364,	365,	362,	349,	352,	353,	354,	368,	369,	370,	371,	369,	371,
	349,	349,	349,	348,	349,	349,	350,	350,	350,	348,	338,	340,	341,	345,	355,	355,	356,	356,	354,	355,
	489,	494,	495,	494,	494,	496,	496,	496,	498,	498,	483,	486,	488,	489,	500,	505,	505,	506,	504,	506,
	1050,	1050,	1050,	1050,	1050,	1050,	1050,	1050,	1050,	1050,	1040,	1040,	1040,	1050,	1050,	1060,	1060,	1060,	1060,	1060,
	1000,	999,	1000,	1000,	997,	998,	999,	999,	1000,	1000,	994,	994,	995,	999,	1000,	1010,	1010,	1010,	1010,	1020,
	465,	466,	468,	468,	468,	469,	469,	470,	472,	471,	457,	460,	461,	467,	473,	478,	478,	479,	477,	481,
	336,	337,	337,	337,	338,	338,	338,	338,	339,	336,	326,	329,	329,	339,	342,	343,	344,	343,	342,	343,
	300,	303,	305,	307,	309,	309,	310,	310,	310,	309,	298,	301,	301,	310,	312,	312,	313,	313,	314,	314,
	424,	429,	431,	433,	435,	437,	437,	438,	440,	439,	424,	427,	428,	437,	438,	442,	442,	444,	446,	445,
	882,	887,	887,	890,	889,	890,	891,	892,	898,	899,	884,	886,	887,	893,	891,	903,	900,	903,	907,	906,
	857,	853,	854,	856,	853,	856,	859,	861,	865,	867,	857,	857,	858,	867,	860,	871,	868,	870,	872,	875,
	418,	419,	420,	421,	422,	426,	428,	430,	431,	431,	418,	420,	422,	430,	430,	433,	433,	434,	436,	437,
	304,	305,	305,	305,	309,	311,	314,	314,	315,	313,	304,	306,	308,	314,	316,	316,	316,	317,	318,	318,
	292,	293,	293,	293,	293,	294,	296,	299,	301,	301,	293,	294,	302,	303,	304,	303,	304,	304,	305,	304,
	397,	398,	399,	399,	398,	400,	401,	404,	408,	409,	398,	400,	404,	409,	409,	411,	411,	412,	414,	414,
	748,	748,	748,	749,	746,	747,	749,	752,	758,	762,	751,	752,	752,	761,	754,	764,	761,	763,	765,	768,
	737,	739,	738,	741,	737,	745,	738,	741,	743,	755,	760,	753,	754,	762,	758,	768,	761,	765,	765,	767,
	403,	402,	406,	403,	407,	407,	407,	405,	408,	414,	403,	401,	408,	410,	412,	413,	411,	413,	414,	414,
	311,	315,	314,	315,	314,	319,	317,	320,	320,	326,	315,	313,	320,	320,	322,	320,	321,	320,	320,	321,
};
#define MaxStatisticsBuf 100
static int StatisticsNum[MaxStatisticsBuf];
static long int StatisticsSum[MaxStatisticsBuf];
static long int golden_Ratio[20*30] = {0, };
struct test_cmd {
	u32 addr;
	u8 len;
	u8 data[40];
};

struct test_cmd short_test_rxrx[] = {
	//# Stop WDT
	{.addr=0x1F028, .len=2,  .data={0x07, 0x55}},
	//# Trim OSC to 60MHz
	//{.addr=0x1F386, .len=1,  .data={0x2A}},
	//# Bypass ControlRAM
	{.addr=0x1F211, .len=1,  .data={0x01}},
	//# Demod @ DC DDFS_2_2_init = 90degree
	{.addr=0x1F150, .len=3,  .data={0x00, 0x00, 0x04}},
	//# Demod @ DC DDFS_2_2_step = 0
	{.addr=0x1F186, .len=2,  .data={0x00, 0x00}},
	//# TIA Rf gain = max
	{.addr=0x1F2D9, .len=1,  .data={0x0E}},
	//# STD_Gain
	{.addr=0x1F2DA, .len=1,  .data={0x0A}},
	//# Warm up
	{.addr=0x1F218, .len=1,  .data={0xFF}},
	//# Sensing start
	{.addr=0x1F220, .len=2,  .data={0xFF, 0xFF}},
	//# VRADC_SEL = 1.75v
	{.addr=0x1F30B, .len=1,  .data={0x02}},
	//# Enable all STD_EN
	{.addr=0x1F2D0, .len=7,  .data={0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x07}},
	//# Enable all ADC_EN
	{.addr=0x1F302, .len=7,  .data={0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x07}},
	//# Enable BIAS_En, STD_BIAS ; Disable TIA_BIAS CC_BIAS
	{.addr=0x1F2FD, .len=4,  .data={0x01, 0x00, 0x01, 0x00}},
	//# Unlock ADC power down
	{.addr=0x1F309, .len=1,  .data={0x00}},
	//# ADC input Range = 0.25~2.25V
	{.addr=0x1F30A, .len=1,  .data={0x00}},
	//# Enable Hanning filter
	{.addr=0x1F1C7, .len=1,  .data={0x20}},
	//# TIA bypass Switch Enable
	{.addr=0x1F310, .len=1,  .data={0x07}},
	//# OB_DM_EN
	{.addr=0x1F1C3, .len=2,  .data={0x00, 0x01}},
	//# Raw1_Base_Addr
	{.addr=0x1F196, .len=2,  .data={0x88, 0x0F}},
	//# POWER ON SEQUENCE
	{.addr=0x1F320, .len=1,  .data={0x06}},
	{.addr=0x1F321, .len=1,  .data={0x13}},
	{.addr=0x1F32A, .len=1,  .data={0x00}},
	{.addr=0x1F327, .len=1,  .data={0x03}},
	{.addr=0x1F328, .len=1,  .data={0x43}},
	{.addr=0x1F328, .len=1,  .data={0x13}},
	{.addr=0x1F328, .len=1,  .data={0x17}},
	//# DAC ON SEQUENCE
	{.addr=0x1F1D5, .len=1,  .data={0x40}},
	{.addr=0x1F1D6, .len=1,  .data={0xFF}},
	{.addr=0x1F1D5, .len=1,  .data={0x7F}},
	{.addr=0x1F1E0, .len=1,  .data={0x01}},
	{.addr=0x1F1DC, .len=1,  .data={0x01}},
	{.addr=0x1F1DE, .len=1,  .data={0x01}},
};

struct test_cmd short_test_txrx[] = {
	//# Stop WDT
	{.addr=0x1F028, .len=2,  .data={0x07, 0x55}},
	//# Trim OSC to 60MHz
	//{.addr=0x1F386, .len=1,  .data={0x2A}},
	//# Bypass ControlRAM
	{.addr=0x1F211, .len=1,  .data={0x01}},
	//# Demod @ DC DDFS_2_2_init = 90degree
	{.addr=0x1F150, .len=3,  .data={0x00, 0x00, 0x04}},
	//# Demod @ DC DDFS_2_2_step = 0
	{.addr=0x1F186, .len=2,  .data={0x00, 0x00}},
	//# TIA Rf gain
	{.addr=0x1F2D9, .len=1,  .data={0x0E}},
	//# STD_Gain
	{.addr=0x1F2DA, .len=1,  .data={0x0A}},
	//# Warm up
	{.addr=0x1F218, .len=1,  .data={0xFF}},
	//# Sensing start
	{.addr=0x1F220, .len=2,  .data={0xFF, 0xFF}},
	//# VRADC_SEL = 1.75v
	{.addr=0x1F30B, .len=1,  .data={0x02}},
	//# RX_CFG all sensing
	{.addr=0x1F238, .len=33, .data={0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02}},
	//# Enable all STD_EN
	{.addr=0x1F2D0, .len=7,  .data={0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x07}},
	//# Enable all ADC_EN
	{.addr=0x1F302, .len=7,  .data={0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x07}},
	//# Enable BIAS_En, STD_BIAS ; Disable TIA_BIAS CC_BIAS
	{.addr=0x1F2FD, .len=4,  .data={0x01, 0x00, 0x01, 0x00}},
	//# Unlock ADC power down
	{.addr=0x1F309, .len=1,  .data={0x00}},
	//# ADC input Range = 0.25~2.25V
	{.addr=0x1F30A, .len=1,  .data={0x00}},
	//# Enable Hanning filter
	{.addr=0x1F1C7, .len=1,  .data={0x20}},
	//# TIA bypass Switch Enable
	{.addr=0x1F310, .len=1,  .data={0x07}},
	//# OB_DM_EN
	{.addr=0x1F1C3, .len=2,  .data={0x00, 0x01}},
	//# Raw1_Base_Addr
	{.addr=0x1F196, .len=2,  .data={0x88, 0x0F}},
	//# POWER ON SEQUENCE
	{.addr=0x1F320, .len=1,  .data={0x06}},
	{.addr=0x1F321, .len=1,  .data={0x13}},
	{.addr=0x1F32A, .len=1,  .data={0x00}},
	{.addr=0x1F327, .len=1,  .data={0x03}},
	{.addr=0x1F328, .len=1,  .data={0x43}},
	{.addr=0x1F328, .len=1,  .data={0x13}},
	{.addr=0x1F328, .len=1,  .data={0x17}},
	//# DAC ON SEQUENCE
	{.addr=0x1F1D5, .len=1,  .data={0x40}},
	{.addr=0x1F1D6, .len=1,  .data={0xFF}},
	{.addr=0x1F1D5, .len=1,  .data={0x7F}},
	{.addr=0x1F1E0, .len=1,  .data={0x01}},
	{.addr=0x1F1DC, .len=1,  .data={0x01}},
	{.addr=0x1F1DE, .len=1,  .data={0x01}},
};

struct test_cmd short_test_txtx[] = {
	//# Stop WDT
	{.addr=0x1F028, .len=2,  .data={0x07, 0x55}},
	//# Trim OSC to 60MHz
	//{.addr=0x1F386, .len=1,  .data={0x2A}},
	//# Bypass ControlRAM
	{.addr=0x1F211, .len=1,  .data={0x01}},
	//# Demod @ DC DDFS_2_2_init = 90degree
	{.addr=0x1F150, .len=3,  .data={0x00, 0x00, 0x04}},
	//# Demod @ DC DDFS_2_2_step = 0
	{.addr=0x1F186, .len=2,  .data={0x00, 0x00}},
	//# Release Test Channel & Output ATEST[1:0] to RX_Channel
	{.addr=0x1F1E5, .len=1,  .data={0x09}},
	//# TIA Rf gain = max
	{.addr=0x1F2D9, .len=1,  .data={0x0E}},
	//# STD_Gain
	{.addr=0x1F2DA, .len=1,  .data={0x0A}},
	//# Warm up
	{.addr=0x1F218, .len=1,  .data={0xFF}},
	//# Sensing start
	{.addr=0x1F220, .len=2,  .data={0xFF, 0xFF}},
	//# VRADC_SEL = 1.75v
	{.addr=0x1F30B, .len=1,  .data={0x02}},
	//# RX_CFG all GND
	{.addr=0x1F238, .len=33,  .data={0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01}},
	//# TX_CFG all GND
	//# Enable all STD_EN
	{.addr=0x1F2D0, .len=7,  .data={0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x07}},
	//# Enable all ADC_EN
	{.addr=0x1F302, .len=7,  .data={0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x07}},
	//# Enable BIAS_En, STD_BIAS ; Disable TIA_BIAS CC_BIAS
	{.addr=0x1F2FD, .len=4,  .data={0x01, 0x00, 0x01, 0x00}},
	//# Unlock ADC power down
	{.addr=0x1F309, .len=1,  .data={0x00}},
	//# ADC input Range = 0.25~2.25V
	{.addr=0x1F30A, .len=1,  .data={0x00}},
	//# Enable Hanning filter
	{.addr=0x1F1C7, .len=1,  .data={0x20}},
	//# TIA bypass Switch Enable
	{.addr=0x1F310, .len=1,  .data={0x07}},
	//# OB_DM_EN
	{.addr=0x1F1C3, .len=2,  .data={0x00, 0x01}},
	//# Raw1_Base_Addr
	{.addr=0x1F196, .len=2,  .data={0x88, 0x0F}},
	//# POWER ON SEQUENCE
	{.addr=0x1F320, .len=1,  .data={0x06}},
	{.addr=0x1F321, .len=1,  .data={0x13}},
	{.addr=0x1F32A, .len=1,  .data={0x00}},
	{.addr=0x1F327, .len=1,  .data={0x03}},
	{.addr=0x1F328, .len=1,  .data={0x43}},
	{.addr=0x1F328, .len=1,  .data={0x13}},
	{.addr=0x1F328, .len=1,  .data={0x17}},
	//# DAC ON SEQUENCE
	{.addr=0x1F1D5, .len=1,  .data={0x40}},
	{.addr=0x1F1D6, .len=1,  .data={0xFF}},
	{.addr=0x1F1D5, .len=1,  .data={0x7F}},
	{.addr=0x1F1E0, .len=1,  .data={0x01}},
	{.addr=0x1F1DC, .len=1,  .data={0x01}},
	{.addr=0x1F1DE, .len=1,  .data={0x01}},
};

struct test_cmd short_test_rx_all_gnd[] = {
    {.addr=0x1F238, .len=AIN_RX_NUM, .data={0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01}},
};
struct test_cmd short_test_tx_all_vref[] = {
    {.addr=0x1F259, .len=AIN_TX_NUM, .data={0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07}},
};
struct test_cmd short_test_tx_all_gnd[] = {
    {.addr=0x1F259, .len=AIN_TX_NUM, .data={0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04}},
};

struct test_cmd open_test[] = {
	//# Stop WDT
	{.addr=0x1F028, .len=2,	 .data={0x07, 0x55}},
	//# Trim OSC to 60MHz
	//{.addr=0x1F386, .len=1,  .data={0x2A}},
	//# Bypass ControlRAM
	{.addr=0x1F211, .len=1,  .data={0x01}},
	//# Demod @ for Hanning DDFS_2_1_init = 90degree
	{.addr=0x1F14C,	.len=3,  .data={0x00, 0x00, 0x04}},
	//# Demod @ for Q DDFS_2_2_init = 90degree
	{.addr=0x1F150, .len=3,  .data={0x00, 0x00, 0x04}},
	//# Demod @ for I DDFS_2_3_init = 0
	{.addr=0x1F154, .len=3,  .data={0x00, 0x00, 0x00}},
	//# TIA Rf gain = max
	{.addr=0x1F2D9, .len=1,  .data={0x0E}},
	//# STD_Gain = max
	{.addr=0x1F2DA, .len=1,  .data={0x0A}},
	//# DDFS2_1_Step
	//#	{.addr=0x1F184, .len=2,  .data={0xA7, 0x0D}},
	//# DDFS2_2_Step
	{.addr=0x1F186, .len=2,  .data={0x93, 0x18}},
	//# DDFS2_3_Step
	{.addr=0x1F188, .len=2,  .data={0x93, 0x18}},
	//# VRADC_SEL = 1.75v
	{.addr=0x1F30B, .len=1,  .data={0x02}},
	//# DDFS1_1_Step
	{.addr=0x1F160, .len=3,  .data={0xBA, 0x49, 0x00}},
	//# RX_CFG all sensing
	{.addr=0x1F238, .len=33, .data={0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x01, 0x01, 0x01}},
	//# Enable all TIA
	{.addr=0x1F2C8, .len=7,  .data={0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x07}},
	//# Enable all STD_EN
	{.addr=0x1F2D0, .len=7,  .data={0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x07}},
	//# Enable all ADC_EN
	{.addr=0x1F302, .len=7,  .data={0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x07}},
	//# Enable BIAS_En, STD_BIAS ; Disable TIA_BIAS CC_BIAS
	{.addr=0x1F2FD, .len=4,  .data={0x01, 0x01, 0x01, 0x00}},
	//# Unlock ADC power down
	{.addr=0x1F309, .len=1,  .data={0x00}},
	//# Enable Hanning filter
	{.addr=0x1F1C7, .len=1,  .data={0x20}},
	//# RXIG Gain
	{.addr=0x1F2DB, .len=1,  .data={0x03}},
	//# Raw1_Base_Addr
	{.addr=0x1F196, .len=2,  .data={0x88, 0x0F}},
	//# Raw2_Base_Addr
	{.addr=0x1F198, .len=2,  .data={0xB0, 0x14}},
	//# Workaround : set UC num >1
	//{.addr=0x1F203, .len=1,  .data={0x02}},
	//# POWER ON SEQUENCE
	{.addr=0x1F320, .len=1,  .data={0x06}},
	{.addr=0x1F321, .len=1,  .data={0x13}},
	{.addr=0x1F32A, .len=1,  .data={0x00}},
	{.addr=0x1F327, .len=1,  .data={0x03}},
	{.addr=0x1F328, .len=1,  .data={0x43}},
	{.addr=0x1F328, .len=1,  .data={0x13}},
	{.addr=0x1F328, .len=1,  .data={0x17}},
	//# DAC ON SEQUENCE
	{.addr=0x1F1D5, .len=1,  .data={0x40}},
	{.addr=0x1F1D6, .len=1,  .data={0xFF}},
	{.addr=0x1F1D5, .len=1,  .data={0x7F}},
	{.addr=0x1F1E0, .len=1,  .data={0x01}},
	{.addr=0x1F1DC, .len=1,  .data={0x01}},
	{.addr=0x1F1DE, .len=1,  .data={0x01}},
};

struct test_cmd open_test_tx_all_gnd[] = {
    {.addr=0x1F259, .len=AIN_TX_NUM, .data={0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04}},
};

typedef enum {
	RESET_STATE_INIT = 0xA0,// IC reset
	RESET_STATE_REK,        // ReK baseline
	RESET_STATE_REK_FINISH, // baseline is ready
	RESET_STATE_NORMAL_RUN  // normal run
} RST_COMPLETE_STATE;

typedef enum {
	SHORT_RXRX = 0,
	SHORT_TXTX,
	SHORT_TXRX,
	OPEN,
	FW_MUTUAL
} CHANNEL_TEST_ITEM;

static int nvt_check_fw_reset_state(struct device *dev, RST_COMPLETE_STATE check_reset_state)
{
	int ret = 0;
	u8 buf[8] = {0, };
	int retry = 0;

	while(1) {
		msleep(20);

		//---read reset state---
		buf[0] = 0x60;
		buf[1] = 0x00;
		ret = nt11206_reg_read(dev, buf[0], &buf[1], 1);
		if(buf[1] >= check_reset_state) {
			ret = 0;
			break;
		}

		if(unlikely(retry++ > 50)) {
			ret = -1;
			TOUCH_E("%s: error, retry=%d, buf[1]=0x%02X\n", __func__, retry, buf[1]);
			break;
		}
	}

	return ret;
}

static void nvt_sw_reset_idle(struct device *dev)
{
	u8 buf[4] = {0, };
	int ret = 0;

	//---write i2c cmds to reset idle---
	buf[0] = 0xFF;
	buf[1] = 0x00;
	buf[2] = 0x00;
	ret = nt11206_bootloader_write(dev, buf, 3);

	//---write i2c cmds to reset idle---
	buf[0] = 0x00;
	buf[1] = 0xA5;
	ret = nt11206_bootloader_write(dev, buf, 2);

	large_mdelay(20);
	nvt_set_i2c_debounce(dev);
}

static int nvt_set_adc_oper(struct device *dev)
{
	int ret = 0;
	uint8_t buf[4] = {0,};
	int i;
	const int retry = 10;

	//---write i2c cmds to set ADC operation---
	buf[0] = 0x01;
	buf[1] = 0xF2;
	ret = nt11206_reg_write(dev, ADDR_CMD, buf, 2);

	//---write i2c cmds to set ADC operation---
	buf[0] = 0x10;
	buf[1] = 0x01;
	ret = nt11206_reg_write(dev, buf[0], &buf[1], 1);

	for(i=0; i<retry; i++) {
		//---read ADC status---
		buf[0]=0x10;
		ret = nt11206_reg_read(dev, buf[0], &buf[1], 1);

		if(buf[1] == 0x00) {
			break;
		}

		msleep(10);
	}

	if(i >= retry) {
		TOUCH_E("%s: Failed!\n", __func__);
		return -1;
	}
	else
		return 0;
}

static void nvt_write_test_cmd(struct device *dev, struct test_cmd *cmds, int cmd_num)
{
	int ret = 0;
	int i = 0;
	int j = 0;
	u8 buf[64] = {0, };

	for(i=0; i<cmd_num; i++) {
		//---set xdata index---
		buf[0] = ((cmds[i].addr >> 16) & 0xFF);
		buf[1] = ((cmds[i].addr >> 8) & 0xFF);
		ret = nt11206_reg_write(dev, ADDR_CMD, buf, 2);

		//---write test cmds---
		buf[0] = (cmds[i].addr & 0xFF);
		for(j=0; j<cmds[i].len; j++)
		{
			buf[1+j] = cmds[i].data[j];
		}
		ret = nt11206_reg_write(dev, buf[0], &buf[1], cmds[i].len);
	}
}

static int nvt_read_short_rxrx(struct device *dev, __s32* rawdata_short_rxrx)
{
	int ret = 0;
	int i = 0;
	u8 buf[64] = {0, };

	//---write i2c cmds to set RX-RX mode---
	nvt_write_test_cmd(dev, short_test_rxrx, sizeof(short_test_rxrx)/sizeof(short_test_rxrx[0]));
	if(nvt_set_adc_oper(dev) != 0) {
		return -EAGAIN;
	}

	//---write i2c cmds to set RX all GND---
	for(i=0; i<AIN_RX_NUM; i++) {
		short_test_rx_all_gnd->data[i] = 0x02;	//set test pin to sample
		nvt_write_test_cmd(dev, short_test_rx_all_gnd, sizeof(short_test_rx_all_gnd)/sizeof(short_test_rx_all_gnd[0]));

		if(nvt_set_adc_oper(dev) != 0) {
			return -EAGAIN;
		}

		//---change xdata index---
		buf[0]=0x01;
		buf[1]=0x0F;
		ret = nt11206_reg_write(dev, ADDR_CMD, buf, 2);

		//---read data---
		buf[0]=(0x88 + i*2);
		ret = nt11206_reg_read(dev, buf[0], &buf[1], 2);
		rawdata_short_rxrx[i] = (s16)(buf[1] + 256*buf[2]);

		short_test_rx_all_gnd->data[i] = 0x01; //restore to default
	}
#if 0 //debuf
	TOUCH_I("[SHORT_RXRX_RAWDATA]\n");
	for(i=0; i<AIN_RX_NUM; i++) {
		printk("%5d, ", rawdata_short_rxrx[i]);
	}
	printk("\n");
#endif
	return 0;
}

static int nvt_read_short_txrx(struct device *dev, __s32 *rawdata_short_txrx)
{
	int ret = 0;
	int i = 0;
	int j = 0;
	u8 buf[64] = {0, };

	//---write i2c cmds to set RX-RX mode---
	nvt_write_test_cmd(dev, short_test_txrx, sizeof(short_test_txrx)/sizeof(short_test_txrx[0]));
	if(nvt_set_adc_oper(dev) != 0) {
		return -EAGAIN;
	}

	//---write i2c cmds to set TX all Vref---
	for(i=0; i<AIN_TX_NUM; i++) {
		short_test_tx_all_vref->data[i] = 0x04;	//set test pin to GND
		nvt_write_test_cmd(dev, short_test_tx_all_vref, sizeof(short_test_tx_all_vref)/sizeof(short_test_tx_all_vref[0]));

		if(nvt_set_adc_oper(dev) != 0) {
			return -EAGAIN;
		}

		//---change xdata index---
		buf[0]=0x01;
		buf[1]=0x0F;
		ret = nt11206_reg_write(dev, ADDR_CMD, buf, 2);
		//---read data---
		buf[0]=0x88;
		ret = nt11206_reg_read(dev, buf[0], &buf[1], AIN_RX_NUM*2);

		for(j=0; j<AIN_RX_NUM; j++) {
			rawdata_short_txrx[i*AIN_RX_NUM+j] = (s16)(buf[j*2+1] + 256*buf[j*2+2]);
		}

		short_test_tx_all_vref->data[i] = 0x07; //restore to default
	}

#if 0 //debuf
	TOUCH_I("[SHORT_TXRX_RAWDATA]\n");
	for(i=0; i<AIN_TX_NUM; i++) {
		for(j=0; j<AIN_RX_NUM; j++) {
			printk("%5d", rawdata_short_txrx[i*AIN_RX_NUM+j]);
		}
		printk("\n");
	}
#endif
	return 0;
}

static int nvt_read_short_txtx(struct device *dev, __s32 *rawdata_short_txtx)
{
	int ret = 0;
	int i = 0;
	u8 buf[64] = {0, };

	//---write i2c cmds to set TX-TX mode---
	nvt_write_test_cmd(dev, short_test_txtx, sizeof(short_test_txtx)/sizeof(short_test_txtx[0]));
	if(nvt_set_adc_oper(dev) != 0) {
		return -EAGAIN;
	}

	//---write i2c cmds to set TX all GND---
	for(i=0; i<AIN_TX_NUM; i++) {
		short_test_tx_all_gnd->data[i] = 0x05;	//set test pin to sample
		nvt_write_test_cmd(dev, short_test_tx_all_gnd, sizeof(short_test_tx_all_gnd)/sizeof(short_test_tx_all_gnd[0]));

		if(nvt_set_adc_oper(dev) != 0) {
			return -EAGAIN;
		}

		//---change xdata index---
		buf[0]=0x01;
		buf[1]=0x0F;
		ret = nt11206_reg_write(dev, ADDR_CMD, buf, 2);
		//---read data---
		buf[0]=(0x88 + i*2);
		ret = nt11206_reg_read(dev, buf[0], &buf[1], 2);
		rawdata_short_txtx[i] = (s16)(buf[1] + 256 * buf[2]);

		short_test_tx_all_gnd->data[i] = 0x01; //restore to default
	}
#if 0 //debug
	TOUCH_I("[SHORT_TXTX_RAWDATA]\n");
	for(i=0; i<AIN_TX_NUM; i++) {
		printk("%5d, ", rawdata_short_txtx[i]);
	}
	printk("\n");
#endif
	return 0;
}
static u32 nvt_sqrt(u32 sqsum)
{
	u32 sq_rt = 0;

	int g0 = 0;
	int g1 = 0;
	int g2 = 0;
	int g3 = 0;
	int g4 = 0;
	int seed = 0;
	int next = 0;
	int step = 0;

	g4 =  sqsum / 100000000;
	g3 = (sqsum - g4*100000000) / 1000000;
	g2 = (sqsum - g4*100000000 - g3*1000000) / 10000;
	g1 = (sqsum - g4*100000000 - g3*1000000 - g2*10000) / 100;
	g0 = (sqsum - g4*100000000 - g3*1000000 - g2*10000 - g1*100);

	next = g4;
	step = 0;
	seed = 0;
	while(((seed + 1) * (step + 1)) <= next) {
		step++;
		seed++;
	}

	sq_rt = seed * 10000;
	next = (next - (seed * step))*100 + g3;

	step = 0;
	seed = 2 * seed * 10;
	while(((seed + 1)*(step + 1)) <= next)
	{
		step++;
		seed++;
	}

	sq_rt = sq_rt + step * 1000;
	next = (next - seed * step) * 100 + g2;
	seed = (seed + step) * 10;
	step = 0;
	while(((seed + 1) * (step + 1)) <= next) {
		step++;
		seed++;
	}

	sq_rt = sq_rt + step * 100;
	next = (next - seed * step) * 100 + g1;
	seed = (seed + step) * 10;
	step = 0;

	while(((seed + 1) * (step + 1)) <= next) {
		step++;
		seed++;
	}

	sq_rt = sq_rt + step * 10;
	next = (next - seed * step) * 100 + g0;
	seed = (seed + step) * 10;
	step = 0;

	while(((seed + 1) * (step + 1)) <= next) {
		step++;
		seed++;
	}

	sq_rt = sq_rt + step;

	return sq_rt;
}

static int nvt_read_open(struct device *dev, s16 rawdata_open_raw1[], s16 rawdata_open_raw2[], __s32 rawdata_open[], __s32 rawdata_open_aver[], int index)
{
	int ret = 0;
	int i = 0;
	int j = 0;
	u8 buf[64] = {0, };
	int raw_index = 0;

	//---write i2c cmds to set SingleScan mode---
	nvt_write_test_cmd(dev, open_test, sizeof(open_test)/sizeof(open_test[0]));
	if(nvt_set_adc_oper(dev) != 0) {
		return -EAGAIN;
	}

	//---write i2c cmds to set TX all GND---
	for(i=0; i<AIN_TX_NUM; i++) {
		open_test_tx_all_gnd->data[i] = 0x08; //set test pin to drive
		nvt_write_test_cmd(dev, open_test_tx_all_gnd, sizeof(open_test_tx_all_gnd)/sizeof(open_test_tx_all_gnd[0]));

		if(nvt_set_adc_oper(dev) != 0) {
			return -EAGAIN;
		}

		if(nvt_set_adc_oper(dev) != 0) {
			return -EAGAIN;
		}

		//---change xdata index---
		buf[0] = 0x01;
		buf[1] = 0x0F;
		ret = nt11206_reg_write(dev, ADDR_CMD, buf, 2);

		//---read data raw1---
		buf[0] = 0x88;
		ret = nt11206_reg_read(dev, buf[0], &buf[1], AIN_RX_NUM * 2);

		for(j=0; j<AIN_RX_NUM; j++)	{
			rawdata_open_raw1[i*AIN_RX_NUM+j] = (s16)(buf[j * 2 + 1] + 256 * buf[j * 2 + 2]);
		}

		//---change xdata index---
		buf[0] = 0x01;
		buf[1] = 0x14;
		ret = nt11206_reg_write(dev, ADDR_CMD, buf, 2);
		//---read data raw2---
		buf[0] = 0xB0;
		ret = nt11206_reg_read(dev, buf[0], &buf[1], AIN_RX_NUM * 2);

		for(j=0; j<AIN_RX_NUM; j++) {
			rawdata_open_raw2[i * AIN_RX_NUM+j] = (s16)(buf[j * 2 + 1] + 256 * buf[j * 2 + 2]);
		}

		open_test_tx_all_gnd->data[i] = 0x04; //restore to default

		//--IQ---
		for(j=0; j<AIN_RX_NUM; j++) {
			raw_index = i * AIN_RX_NUM + j;
			rawdata_open[raw_index] = nvt_sqrt(rawdata_open_raw1[raw_index] * rawdata_open_raw1[raw_index] + rawdata_open_raw2[raw_index] * rawdata_open_raw2[raw_index]);
			rawdata_open_aver[raw_index] += rawdata_open[raw_index];
		}
	}
	if(index == 2) {
		for(i = 0; i < AIN_TX_NUM; i++) {
			for(j = 0; j < AIN_RX_NUM; j++) {
				raw_index = i * AIN_RX_NUM + j;
				rawdata_open_aver[raw_index] = rawdata_open_aver[raw_index] / 3;
			}
		}
	}
#if 0 //debug
	TOUCH_I("[OPEN_RAWDATA]\n");
	for(i=0; i<AIN_TX_NUM; i++) {
		for(j=0; j<AIN_RX_NUM; j++)	{
			printk("%5d, ", rawdata_open[i*AIN_RX_NUM+j]);
		}
		printk("\n");
	}
#endif
	return 0;
}
static int nvt_read_baseline(struct device *dev, __s32 *xdata)
{
	u8 x_num = 0;
	u8 y_num = 0;

	nvt_change_mode(dev, TEST_MODE_1);

	if(nvt_clear_fw_status(dev) != 0) {
		return -EAGAIN;
	}

	if(nvt_check_fw_status(dev) != 0) {
		return -EAGAIN;
	}

	nvt_get_fw_info(dev);
	nvt_read_mdata(dev, BASELINE_ADDR);
	nvt_get_mdata(xdata, &x_num, &y_num);

	nvt_change_mode(dev, MODE_CHANGE_NORMAL_MODE);

	return 0;
}
static int Test_CaluateGRatioAndNormal(s16 boundary[], __s32 rawdata[], u8 x_len, u8 y_len)
{
	int i = 0;
	int j = 0;
	int k = 0;
	long int tmpValue = 0;
	long int MaxSum = 0;
	int SumCnt = 0;
	int MaxNum = 0;
	int MaxIndex = 0;
	int Max = -99999999;
	int Min =  99999999;
	int offset = 0;
	int Data = 0;	// double
	int StatisticsStep = 0;

	//--------------------------------------------------
	//1. (Testing_CM - Golden_CM ) / Testing_CM
	//--------------------------------------------------
	for(j=0; j<y_len; j++) {
		for(i=0; i<x_len; i++) {
			Data = rawdata[j * x_len + i];
			if(Data == 0)
				Data = 1;

			golden_Ratio[j * x_len + i] = Data - boundary[j * x_len + i];
			golden_Ratio[j * x_len + i] = ((golden_Ratio[j * x_len + i]*1000) / Data);	// *1000 before division
		}
	}

	//--------------------------------------------------------
	// 2. Mutual_GoldenRatio*1000
	//--------------------------------------------------------
	for(j=0; j<y_len; j++) {
		for(i=0; i<x_len; i++) {
			golden_Ratio[j * x_len + i] *= 1000;
		}
	}

	//--------------------------------------------------------
	// 3. Calculate StatisticsStep
	//--------------------------------------------------------
	for(j=0; j<y_len; j++) {
		for(i=0; i<x_len; i++) {
			if (Max < golden_Ratio[j * x_len + i])
				Max = (int)golden_Ratio[j * x_len + i];
			if (Min > golden_Ratio[j * x_len + i])
				Min = (int)golden_Ratio[j * x_len + i];
		}
	}

	offset = 0;
	if(Min < 0) // add offset to get erery element Positive
	{
		offset = 0 - Min;
		for(j=0; j<y_len; j++) {
			for(i=0; i<x_len; i++) {
				golden_Ratio[j * x_len + i] += offset;
			}
		}
		Max += offset;
	}
	StatisticsStep = Max / MaxStatisticsBuf;
	StatisticsStep += 1;
	if(StatisticsStep < 0) {
		TOUCH_E("FAIL! (StatisticsStep < 0)\n");
		return 1;
	}

	//--------------------------------------------------------
	// 4. Start Statistics and Average
	//--------------------------------------------------------
	memset(StatisticsSum, 0, sizeof(long int)*MaxStatisticsBuf);
	memset(StatisticsNum, 0, sizeof(int)* MaxStatisticsBuf);
	for(i=0; i<MaxStatisticsBuf; i++) {
		StatisticsSum[i] = 0;
		StatisticsNum[i] = 0;
	}
	for(j=0; j<y_len; j++) {
		for(i=0; i<x_len; i++) {
			tmpValue = golden_Ratio[j * x_len + i];
			tmpValue /= StatisticsStep;
			StatisticsNum[tmpValue] += 2;
			StatisticsSum[tmpValue] += (2 * golden_Ratio[j * x_len + i]);

			if((tmpValue + 1) < MaxStatisticsBuf) {
				StatisticsNum[tmpValue + 1] += 1;
				StatisticsSum[tmpValue + 1] += golden_Ratio[j * x_len + i];
			}

			if ((tmpValue - 1) >= 0) {
				StatisticsNum[tmpValue - 1] += 1;
				StatisticsSum[tmpValue - 1] += golden_Ratio[j * x_len + i];
			}
		}
	}
	//Find out Max Statistics
	MaxNum = 0;
	for(k=0; k<MaxStatisticsBuf; k++) {
		if(MaxNum < StatisticsNum[k]) {
			MaxSum = StatisticsSum[k];
			MaxNum = StatisticsNum[k];
			MaxIndex = k;
		}
	}
	//Caluate Statistics Average
	if(MaxSum > 0) {
		if (StatisticsNum[MaxIndex] != 0) {
			tmpValue = (long)(StatisticsSum[MaxIndex] / StatisticsNum[MaxIndex]) * 2;
			SumCnt += 2;
		}

		if ((MaxIndex + 1) < (MaxStatisticsBuf)) {
			if (StatisticsNum[MaxIndex + 1] != 0)
			{
				tmpValue += (long)(StatisticsSum[MaxIndex + 1] / StatisticsNum[MaxIndex + 1]);
				SumCnt++;
			}
		}

		if ((MaxIndex - 1) >= 0) {
			if (StatisticsNum[MaxIndex - 1] != 0) {
				tmpValue += (long)(StatisticsSum[MaxIndex - 1] / StatisticsNum[MaxIndex - 1]);
				SumCnt++;
			}
		}

		if (SumCnt > 0) {
			tmpValue /= SumCnt;
		}
	}
	else {// Too Separately
		StatisticsSum[0] = 0;
		StatisticsNum[0] = 0;
		for(j=0; j<y_len; j++) 	{
			for(i=0; i<x_len; i++) {
				StatisticsSum[0] += (long int)golden_Ratio[j * x_len + i];
				StatisticsNum[0]++;
			}
		}
		tmpValue = StatisticsSum[0] / StatisticsNum[0];
	}

	tmpValue -= offset;
	for(j= 0; j<y_len; j++) {
		for(i=0; i<x_len; i++) {
			golden_Ratio[j * x_len + i] -= offset;

			golden_Ratio[j * x_len + i] = golden_Ratio[j * x_len + i] - tmpValue;
			golden_Ratio[j * x_len + i] = golden_Ratio[j * x_len + i] / 1000;
		}
	}
	return 0;
}
static int RawDataTest_Sub(s16 boundary[], __s32 rawdata[], u8 RecordResult[], u8 x_ch, u8 y_ch,
		int Tol_P, int Tol_N, int Dif_P, int Dif_N, int Rawdata_Limit_Postive, int Rawdata_Limit_Negative)
{
	int i = 0;
	int j = 0;
	int iArrayIndex=0;
	__s32 iBoundary=0;
	s64 iTolLowBound = 0;
	s64 iTolHighBound = 0;
	bool isAbsCriteria = false;
	bool isPass = true;

	if(Rawdata_Limit_Postive != 0 || Rawdata_Limit_Negative != 0) {
		isAbsCriteria = true;
	}

	for(j=0; j<y_ch; j++) {
		for(i=0; i<x_ch; i++) {
			iArrayIndex = j * x_ch + i;
			iBoundary = boundary[iArrayIndex];

			RecordResult[iArrayIndex] = 0x00;	// default value for PASS

			if(isAbsCriteria) {
				iTolLowBound = Rawdata_Limit_Negative * 1000;
				iTolHighBound = Rawdata_Limit_Postive * 1000;
			}
			else {
				if(iBoundary > 0) {
					iTolLowBound = (iBoundary * (1000 + Tol_N));
					iTolHighBound = (iBoundary * (1000 + Tol_P));
				}
				else {
					iTolLowBound = (iBoundary * (1000 - Tol_N));
					iTolHighBound = (iBoundary * (1000 - Tol_P));
				}
			}

			if((rawdata[iArrayIndex] * 1000) > iTolHighBound) {
				RecordResult[iArrayIndex] |= 0x01;
			}

			if((rawdata[iArrayIndex] * 1000) < iTolLowBound) {
				RecordResult[iArrayIndex] |= 0x02;
			}
		}
	}

	if(!isAbsCriteria) {
		Test_CaluateGRatioAndNormal(boundary, rawdata, x_ch, y_ch);

		for(j=0; j<y_ch; j++) {
			for(i=0; i<x_ch; i++) {
				iArrayIndex = j * x_ch + i;

				if(golden_Ratio[iArrayIndex] > Dif_P) {
					RecordResult[iArrayIndex] |= 0x04;
				}

				if(golden_Ratio[iArrayIndex] < Dif_N) {
					RecordResult[iArrayIndex] |= 0x08;
				}
			}
		}
	}

	//---Check RecordResult---
	for(j=0; j<y_ch; j++) {
		for(i=0; i<x_ch; i++) {
			if(RecordResult[j * x_ch + i] != 0) {
				isPass = false;
				break;
			}
		}
	}

	if(isPass == false) {
		return -1;	// FAIL
	}
	else {
		return 0;	// PASS
	}
}
void print_selftest_result(struct device *dev, CHANNEL_TEST_ITEM item, int TestResult, u8 RecordResult[], __s32 rawdata[], u8 x_len, u8 y_len, int mode)
{
	struct nt11206_data *d = to_nt11206_data(dev);
	char *write_buf = NULL;
	int ret = 0;
    int i = 0;
	int j = 0;

	if(mode) {
		return;
	}

	write_buf = kmalloc(sizeof(char) * LOG_BUF_SIZE, GFP_KERNEL);
	if(write_buf) {
		memset(write_buf, 0, sizeof(char) * LOG_BUF_SIZE);
	}
	else {
		return;
	}

	switch(item) {
		case SHORT_RXRX:
			TOUCH_I("[SHORT_RXRX_RAWDATA]\n");
			ret += snprintf(write_buf + ret, LOG_BUF_SIZE - ret, "[SHORT_RXRX_RAWDATA]\n");
		break;

		case SHORT_TXTX:
			TOUCH_I("[SHORT_TXTX_RAWDATA]\n");
			ret += snprintf(write_buf + ret, LOG_BUF_SIZE - ret, "[SHORT_TXTX_RAWDATA]\n");
		break;

		case SHORT_TXRX:
			TOUCH_I("[SHORT_TXRX_RAWDATA]\n");
			ret += snprintf(write_buf + ret, LOG_BUF_SIZE - ret, "[SHORT_TXRX_RAWDATA]\n");
		break;

		case OPEN:
			TOUCH_I("[OPEN_RAWDATA]\n");
			ret += snprintf(write_buf + ret, LOG_BUF_SIZE - ret, "[OPEN_RAWDATA]\n");
		break;

		case FW_MUTUAL:
			TOUCH_I("[FW_MUTUAL_RAWDATA]\n");
			ret += snprintf(write_buf + ret, LOG_BUF_SIZE - ret, "[FW_MUTUAL_RAWDATA]\n");
		break;

		default:

		break;
	}

	for(i=0; i<y_len; i++) {
		for(j=0; j<x_len; j++) {
			printk("%7d", rawdata[i*x_len+j]);
			ret += snprintf(write_buf + ret, LOG_BUF_SIZE - ret, "%7d", rawdata[i*x_len+j]);
		}
		printk("\n");
		ret += snprintf(write_buf + ret, LOG_BUF_SIZE - ret, "\n");
	}

	switch(item) {
		case SHORT_RXRX:
			TOUCH_I("[SHORT_RXRX_RESULT]\n");
			ret += snprintf(write_buf + ret, LOG_BUF_SIZE - ret, "[SHORT_RXRX_RESULT]\n");
		break;

		case SHORT_TXTX:
			TOUCH_I("[SHORT_TXTX_RESULT]\n");
			ret += snprintf(write_buf + ret, LOG_BUF_SIZE - ret, "[SHORT_TXTX_RESULT]\n");
		break;

		case SHORT_TXRX:
			TOUCH_I("[SHORT_TXRX_RESULT]\n");
			ret += snprintf(write_buf + ret, LOG_BUF_SIZE - ret, "[SHORT_TXRX_RESULT]\n");
		break;

		case OPEN:
			TOUCH_I("[OPEN_RESULT]\n");
			ret += snprintf(write_buf + ret, LOG_BUF_SIZE - ret, "[OPEN_RESULT]\n");
		break;

		case FW_MUTUAL:
			TOUCH_I("[FW_MUTUAL_RESULT]\n");
			ret += snprintf(write_buf + ret, LOG_BUF_SIZE - ret, "[FW_MUTUAL_RESULT]\n");
		break;
		default:

		break;
	}

	switch(TestResult) {
		case 0:
			TOUCH_I("PASS!\n\n");
			ret += snprintf(write_buf + ret, LOG_BUF_SIZE - ret, "[PASS]\n\n");
		break;

		case 1:
			TOUCH_E("ERROR! Read Data FAIL!\n");
			ret += snprintf(write_buf + ret, LOG_BUF_SIZE - ret, "ERROR! Read Data FAIL!\n");
		break;

		case -1:
			TOUCH_E ("FAIL!\n");
			ret += snprintf(write_buf + ret, LOG_BUF_SIZE - ret, "[FAIL]\n");
		break;
	}

	if(d->boot_mode == NORMAL_BOOT) {
		write_file(NORMAL_SELF_TEST_FILE_PATH, write_buf, 1);
		log_file_size_check(dev, NORMAL_SELF_TEST_FILE_PATH);
	}
	else {
		write_file(SELF_TEST_FILE_PATH, write_buf, 1);
		log_file_size_check(dev, SELF_TEST_FILE_PATH);
	}

	if(TestResult == -1) {
		memset(write_buf, 0, sizeof(char) * LOG_BUF_SIZE);
		ret = 0;
		TOUCH_I("RecordResult:\n");
		ret += snprintf(write_buf + ret, LOG_BUF_SIZE - ret, "[RECORD_RESULT]\n");
		for(i=0; i<y_len; i++) {
			for(j=0; j<x_len; j++) {
				printk("0x%02X, ", RecordResult[i*x_len+j]);
				ret += snprintf(write_buf + ret, LOG_BUF_SIZE - ret, "0x%02X, ", RecordResult[i*x_len+j]);
			}
			printk("\n");
			ret += snprintf(write_buf + ret, LOG_BUF_SIZE - ret, "\n");
		}
		ret += snprintf(write_buf + ret, LOG_BUF_SIZE - ret, "\n");
		printk("\n");
		if(d->boot_mode == NORMAL_BOOT) {
			write_file(NORMAL_SELF_TEST_FILE_PATH, write_buf, 1);
			log_file_size_check(dev, NORMAL_SELF_TEST_FILE_PATH);
		}
		else {
			write_file(SELF_TEST_FILE_PATH, write_buf, 1);
			log_file_size_check(dev, SELF_TEST_FILE_PATH);
		}
	}

	if(write_buf) {
		kfree(write_buf);
	}
}
#if 0 //FOR DEBUGGING
void print_baseline_check_result(struct device *dev, CHANNEL_TEST_ITEM item, int TestResult, u8 RecordResult[], __s32 rawdata[], u8 x_len, u8 y_len)
{
    int i = 0;
	int j = 0;

	switch(TestResult) {
		case 0:
			TOUCH_I("PASS!\n\n");
		break;

		case 1:
			TOUCH_E("ERROR! Read Data FAIL!\n");
		break;

		case -1:
			TOUCH_E ("FAIL!\n");
		break;
	}

	if(TestResult == -1) {
		TOUCH_I("[FW_MUTUAL_RAWDATA]\n");

		for(i=0; i<y_len; i++) {
			for(j=0; j<x_len; j++) {
				printk("%7d", rawdata[i*x_len+j]);
			}
			printk("\n");
		}

		TOUCH_I("[FW_MUTUAL_RESULT]\n");

		TOUCH_I("RecordResult:\n");
		for(i=0; i<y_len; i++) {
			for(j=0; j<x_len; j++) {
				printk("0x%02X, ", RecordResult[i*x_len+j]);
			}
			printk("\n");
		}
	}
}
#endif
int nt11206_selftest(struct device *dev, char* buf, u8 mode)
{
	struct nt11206_data *d = to_nt11206_data(dev);
	u8 x_num=0;
	u8 y_num=0;
	int ret = 0;
	int i = 0;
	int TestResult_Short_RXRX = 0;
	int TestResult_Short_TXRX = 0;
	int TestResult_Short_TXTX = 0;
	int TestResult_Open = 0;
	int TestResult_FW_MUTUAL = 0;

	u8 *RecordResult = NULL;
	s16 *rawdata_16 = NULL;
	s16 *rawdata_16_1 = NULL;
	__s32 *rawdata_32 = NULL;
	__s32 *open_rawdata_32 = NULL;

	RecordResult = (u8*)kmalloc(sizeof(u8) * 20 * 30, GFP_KERNEL);
	rawdata_16 = (s16*)kmalloc(sizeof(s16) * 20 * 30, GFP_KERNEL);
	rawdata_16_1 = (s16*)kmalloc(sizeof(s16) * 20 * 30, GFP_KERNEL);
	rawdata_32 = (__s32*)kmalloc(sizeof(u32) * 20 * 30, GFP_KERNEL);
	open_rawdata_32 = (__s32*)kmalloc(sizeof(u32) * 20 * 30, GFP_KERNEL);

	x_num = d->fw.x_axis_num;
	y_num = d->fw.y_axis_num;

	if((RecordResult == NULL) || (rawdata_16 == NULL) || (rawdata_16_1 == NULL) || (rawdata_32 == NULL) || (open_rawdata_32 == NULL)) {
		TOUCH_E("Alloc Failed\n");
		if(RecordResult)
			kfree(RecordResult);

		if(rawdata_16)
			kfree(rawdata_16);

		if(rawdata_16_1)
			kfree(rawdata_16_1);

		if(rawdata_32)
			kfree(rawdata_32);

		if(open_rawdata_32)
			kfree(open_rawdata_32);
		return -1;
	}

	if(	d->resume_state == 0 && !mode) {
		TOUCH_E("LCD OFF, mode:%d\n", mode);
		ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "LCD OFF\n");
		if(RecordResult)
			kfree(RecordResult);

		if(rawdata_16)
			kfree(rawdata_16);

		if(rawdata_16_1)
			kfree(rawdata_16_1);

		if(rawdata_32)
			kfree(rawdata_32);

		if(open_rawdata_32)
			kfree(open_rawdata_32);
		return ret;
	}
	TOUCH_I("x_axis_num:%d, y_axis_num:%d\n", d->fw.x_axis_num, d->fw.y_axis_num);
	if((x_num != 20) || (y_num != 30)) {
		TOUCH_E("FW Info is broken\n");
		x_num = 20;
		y_num = 30;
	}

	//---Reset IC & into idle---
	nt11206_hw_reset(dev);
	nvt_change_mode(dev, AUTORC_OFF);
	nvt_check_fw_reset_state(dev, RESET_STATE_REK_FINISH);

	if(nvt_read_baseline(dev, rawdata_32) != 0) {
		TestResult_FW_MUTUAL = 1; //1:ERROR
	}
	else {
		//---Self Test Check ---	// 0:PASS, -1:FAIL
		TestResult_FW_MUTUAL = RawDataTest_Sub(BoudaryFWMutual, rawdata_32, RecordResult, AIN_TX_NUM, AIN_RX_NUM,
				PSConfig_Tolerance_Postive_FW, PSConfig_Tolerance_Negative_FW, PSConfig_DiffLimitG_Postive_FW, PSConfig_DiffLimitG_Negative_FW,
				0, 0);
	}
	print_selftest_result(dev, FW_MUTUAL, TestResult_FW_MUTUAL, RecordResult, rawdata_32, AIN_TX_NUM, AIN_RX_NUM, mode);
	if(mode) {
		ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "LPWG RAWDATA : ");
	}
	else {
		ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "Raw Data : ");
	}
	switch(TestResult_FW_MUTUAL) {
		case 0:
			ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "Pass\n");
			break;

		case 1:
			ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "Fail\n");
			ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "Self Test ERROR! Read Data FAIL!\n");
			break;

		case -1:
			ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "Fail\n");
			break;
	}
	if(mode) {
		if(RecordResult)
			kfree(RecordResult);

		if(rawdata_16)
			kfree(rawdata_16);

		if(rawdata_16_1)
			kfree(rawdata_16_1);

		if(rawdata_32)
			kfree(rawdata_32);

		if(open_rawdata_32)
			kfree(open_rawdata_32);

		nt11206_hw_reset(dev);
		return ret;
	}
	//---Reset IC & into idle---
	nvt_sw_reset_idle(dev);
	msleep(100);

	//---Short Test RX-RX---
	memset(RecordResult, 0, sizeof(u8) * 20 * 30);
	memset(rawdata_16, 0, sizeof(s16) * 20 * 30);
	memset(rawdata_16_1, 0, sizeof(s16) * 20 * 30);
	memset(rawdata_32, 0, sizeof(__s32) * 20 * 30);
	if(nvt_read_short_rxrx(dev, rawdata_32) != 0) {
		TestResult_Short_RXRX = 1;	// 1:ERROR
	}
	else {
		//---Self Test Check --- 		// 0:PASS, -1:FAIL
		TestResult_Short_RXRX = RawDataTest_Sub(BoundaryShort_RXRX, rawdata_32, RecordResult, AIN_RX_NUM, 1,
												PSConfig_Tolerance_Postive_Short, PSConfig_Tolerance_Negative_Short, PSConfig_DiffLimitG_Postive_Short, PSConfig_DiffLimitG_Negative_Short,
												PSConfig_Rawdata_Limit_Postive_Short_RXRX, PSConfig_Rawdata_Limit_Negative_Short_RXRX);
	}
	print_selftest_result(dev, SHORT_RXRX, TestResult_Short_RXRX, RecordResult, rawdata_32, AIN_RX_NUM, 1, mode);

	//---Short Test TX-RX---
	memset(RecordResult, 0, sizeof(u8) * 20 * 30);
	memset(rawdata_16, 0, sizeof(s16) * 20 * 30);
	memset(rawdata_16_1, 0, sizeof(s16) * 20 * 30);
	memset(rawdata_32, 0, sizeof(__s32) * 20 * 30);
	if(nvt_read_short_txrx(dev, rawdata_32) != 0) {
		TestResult_Short_TXRX = 1;	// 1:ERROR
	}
	else {
		//---Self Test Check ---		// 0:PASS, -1:FAIL
		TestResult_Short_TXRX = RawDataTest_Sub(BoundaryShort_TXRX, rawdata_32, RecordResult, AIN_TX_NUM, AIN_RX_NUM,
    											PSConfig_Tolerance_Postive_Short, PSConfig_Tolerance_Negative_Short, PSConfig_DiffLimitG_Postive_Short, PSConfig_DiffLimitG_Negative_Short,
    											PSConfig_Rawdata_Limit_Postive_Short_TXRX, PSConfig_Rawdata_Limit_Negative_Short_TXRX);
	}
	print_selftest_result(dev, SHORT_TXRX, TestResult_Short_TXRX, RecordResult, rawdata_32, AIN_TX_NUM, AIN_RX_NUM, mode);

	//---Short Test TX-TX---
	memset(RecordResult, 0, sizeof(u8) * 20 * 30);
	memset(rawdata_16, 0, sizeof(s16) * 20 * 30);
	memset(rawdata_16_1, 0, sizeof(s16) * 20 * 30);
	memset(rawdata_32, 0, sizeof(__s32) * 20 * 30);
	if(nvt_read_short_txtx(dev, rawdata_32) != 0) {
		TestResult_Short_TXTX = 1;	// 1:ERROR
	}
	else {
		//---Self Test Check ---		// 0:PASS, -1:FAIL
		TestResult_Short_TXTX = RawDataTest_Sub(BoundaryShort_TXTX, rawdata_32, RecordResult, AIN_TX_NUM, 1,
												PSConfig_Tolerance_Postive_Short, PSConfig_Tolerance_Negative_Short, PSConfig_DiffLimitG_Postive_Short, PSConfig_DiffLimitG_Negative_Short,
												PSConfig_Rawdata_Limit_Postive_Short_TXTX, PSConfig_Rawdata_Limit_Negative_Short_TXTX);
	}

	print_selftest_result(dev, SHORT_TXTX, TestResult_Short_TXTX, RecordResult, rawdata_32, AIN_TX_NUM, 1, mode);

	//---Reset IC & into idle---
	nvt_sw_reset_idle(dev);
	msleep(100);
	memset(open_rawdata_32, 0, sizeof(__s32) * 20 * 30);
	for(i = 0; i < 3; i++) {
		//---Open Test---
		memset(RecordResult, 0, sizeof(u8) * 20 * 30);
		memset(rawdata_16, 0, sizeof(s16) * 20 * 30);
		memset(rawdata_16_1, 0, sizeof(s16) * 20 * 30);
		memset(rawdata_32, 0, sizeof(__s32) * 20 * 30);
		TestResult_Open= nvt_read_open(dev, rawdata_16, rawdata_16_1, rawdata_32, open_rawdata_32, i);
		if(TestResult_Open != 0) {
			TestResult_Open = 1;	// 1:ERROR
			break;
		}
		msleep(10);
	}
	TestResult_Open = RawDataTest_Sub(BoundaryOpen, open_rawdata_32, RecordResult,AIN_TX_NUM, AIN_RX_NUM,
			PSConfig_Tolerance_Postive_Mutual, PSConfig_Tolerance_Negative_Mutual, PSConfig_DiffLimitG_Postive_Mutual, PSConfig_DiffLimitG_Negative_Mutual,
			0, 0);
	print_selftest_result(dev, OPEN, TestResult_Open, RecordResult, open_rawdata_32, AIN_TX_NUM, AIN_RX_NUM, mode);

	ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "Channel Status : ");
	if(!TestResult_Short_RXRX && !TestResult_Short_TXRX && !TestResult_Short_TXTX && !TestResult_Open) {
		ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "Pass\n");
	}
	else {
		ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "Fail\n");
	}

	nt11206_hw_reset(dev);

	if(RecordResult)
		kfree(RecordResult);

	if(rawdata_16)
		kfree(rawdata_16);

	if(rawdata_16_1)
		kfree(rawdata_16_1);

	if(rawdata_32)
		kfree(rawdata_32);

	if(open_rawdata_32)
		kfree(open_rawdata_32);

	return ret;
}
