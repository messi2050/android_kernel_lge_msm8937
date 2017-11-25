/*
 * lp5523.c - LP5523, LP55231 LED Driver
 *
 * Copyright (C) 2010 Nokia Corporation
 * Copyright (C) 2012 Texas Instruments
 *
 * Contact: Samu Onkalo <samu.p.onkalo@nokia.com>
 *          Milo(Woogyom) Kim <milo.kim@ti.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 */

#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/i2c.h>
#include <linux/leds.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_data/leds-lp55xx.h>
#include <linux/slab.h>

#include <linux/gpio.h>
#include <linux/interrupt.h>

#if IS_ENABLED(CONFIG_LGE_LEDS_LP5523)
#include <soc/qcom/lge/board_lge.h>

#include <linux/file.h>
#include <asm/uaccess.h>
#include <linux/syscalls.h>
#endif

#include "leds-lp55xx-common.h"

#define LP5523_PROGRAM_LENGTH		32	/* bytes */
#define LP5523_NUM_OF_PAGE			6
/* Memory is used like this:
   0x00 engine 1 program
   0x10 engine 2 program
   0x20 engine 3 program
   0x30 engine 1 muxing info
   0x40 engine 2 muxing info
   0x50 engine 3 muxing info
*/
#define LP5523_MAX_LEDS			9
#define LP5523_MAX_LED_CURRENT	255
#define LP5523_MAX_LED_BRIGHTNESS	255


/* Registers */
#define LP5523_REG_ENABLE		0x00
#define LP5523_REG_OP_MODE		0x01
#define LP5523_REG_ENABLE_LEDS_MSB	0x04
#define LP5523_REG_ENABLE_LEDS_LSB	0x05
#define LP5523_REG_LED_CTRL_BASE	0x06
#define LP5523_REG_LED_PWM_BASE		0x16
#define LP5523_REG_LED_CURRENT_BASE	0x26
#define LP5523_REG_CONFIG		0x36
#define LP5523_REG_STATUS		0x3A
#define LP5523_REG_RESET		0x3D
#define LP5523_REG_LED_TEST_CTRL	0x41
#define LP5523_REG_LED_TEST_ADC		0x42
#define LP5523_REG_MASTER_FADER_BASE	0x48
#define LP5523_REG_CH1_PROG_START	0x4C
#define LP5523_REG_CH2_PROG_START	0x4D
#define LP5523_REG_CH3_PROG_START	0x4E
#define LP5523_REG_PROG_PAGE_SEL	0x4F
#define LP5523_REG_PROG_MEM		0x50

/* Bit description in registers */
#define LP5523_ENABLE			0x40
#define LP5523_AUTO_INC			0x40
#define LP5523_PWR_SAVE			0x20
#define LP5523_PWM_PWR_SAVE		0x04
#define LP5523_CP_AUTO			0x18
#define LP5523_AUTO_CLK			0x02
#define LP5523_FORCED_INT_CLK	0x01

#define LP5523_EN_LEDTEST		0x80
#define LP5523_LEDTEST_DONE		0x80
#define LP5523_RESET			0xFF
#define LP5523_ADC_SHORTCIRC_LIM	80
#define LP5523_EXT_CLK_USED		0x08
#define LP5523_ENG_STATUS_MASK		0x07

#define LP5523_FADER_MAPPING_MASK	0xC0
#define LP5523_FADER_MAPPING_SHIFT	6

/* Memory Page Selection */
#define LP5523_PAGE_ENG1		0
#define LP5523_PAGE_ENG2		1
#define LP5523_PAGE_ENG3		2
#define LP5523_PAGE_MUX1		3
#define LP5523_PAGE_MUX2		4
#define LP5523_PAGE_MUX3		5

/* Program Memory Operations */
#define LP5523_MODE_ENG1_M		0x30	/* Operation Mode Register */
#define LP5523_MODE_ENG2_M		0x0C
#define LP5523_MODE_ENG3_M		0x03
#define LP5523_MODE_ALL_ENGS	0x3F

#define LP5523_LOAD_ENG1		0x10
#define LP5523_LOAD_ENG2		0x04
#define LP5523_LOAD_ENG3		0x01
#define LP5523_LOAD_ALL_ENGS	0x15


#define LP5523_ENG1_IS_LOADING(mode)	\
	((mode & LP5523_MODE_ENG1_M) == LP5523_LOAD_ENG1)
#define LP5523_ENG2_IS_LOADING(mode)	\
	((mode & LP5523_MODE_ENG2_M) == LP5523_LOAD_ENG2)
#define LP5523_ENG3_IS_LOADING(mode)	\
	((mode & LP5523_MODE_ENG3_M) == LP5523_LOAD_ENG3)

#define LP5523_EXEC_ENG1_M		0x30	/* Enable Register */
#define LP5523_EXEC_ENG2_M		0x0C
#define LP5523_EXEC_ENG3_M		0x03
#define LP5523_EXEC_M			0x3F
#define LP5523_RUN_ENG1			0x20
#define LP5523_RUN_ENG2			0x08
#define LP5523_RUN_ENG3			0x02

#define LED_ACTIVE(mux, led)		(!!(mux & (0x0001 << led)))

enum lp5523_chip_id {
	LP5523,
	LP55231,
};

#if IS_ENABLED(CONFIG_LGE_LEDS_LP5523)
#define LP5523_ENABLE_DEBBUG_LOG 0
static u8 hidden_led_current = 0;
#endif

#if !defined(CONFIG_LGE_LEDS_LP5523)
static int lp5523_init_program_engine(struct lp55xx_chip *chip);
#endif
static inline void lp5523_wait_opmode_done(void)
{
	usleep_range(1000, 2000);
}

static void lp5523_set_led_current(struct lp55xx_led *led, u8 led_current)
{
	led->led_current = led_current;
	lp55xx_write(led->chip, LP5523_REG_LED_CURRENT_BASE + led->chan_nr,
		led_current);
}

static int lp5523_post_init_device(struct lp55xx_chip *chip)
{
	int ret;
	int i;

	ret = lp55xx_write(chip, LP5523_REG_ENABLE, LP5523_ENABLE);
	if (ret)
		return ret;

	/* Chip startup time is 500 us, 1 - 2 ms gives some margin */
	usleep_range(1000, 2000);
#if IS_ENABLED(CONFIG_LGE_LEDS_LP5523)
	for (i = 0; i < LP5523_MAX_LEDS; i++) {
		ret =+ lp55xx_write(chip, LP5523_REG_LED_CURRENT_BASE + i,
			chip->pdata->led_config[i].led_current);
	}
	if (ret)
		return ret;

	ret = lp55xx_write(chip, LP5523_REG_CONFIG,
			    LP5523_AUTO_INC | LP5523_PWR_SAVE |
			    LP5523_CP_AUTO | LP5523_FORCED_INT_CLK |
			    LP5523_PWM_PWR_SAVE);

#else
	ret = lp55xx_write(chip, LP5523_REG_CONFIG,
			    LP5523_AUTO_INC | LP5523_PWR_SAVE |
			    LP5523_CP_AUTO | LP5523_AUTO_CLK |
			    LP5523_PWM_PWR_SAVE);
#endif
	if (ret)
		return ret;

	/* turn on all leds */
	ret = lp55xx_write(chip, LP5523_REG_ENABLE_LEDS_MSB, 0x01);
	if (ret)
		return ret;

	ret = lp55xx_write(chip, LP5523_REG_ENABLE_LEDS_LSB, 0xff);
	if (ret)
		return ret;
#if IS_ENABLED(CONFIG_LGE_LEDS_LP5523)
	return ret;
#else
	return lp5523_init_program_engine(chip);
#endif
}

static void lp5523_load_engine(struct lp55xx_chip *chip)
{
	enum lp55xx_engine_index idx = chip->engine_idx;
	u8 mask[] = {
		[LP55XX_ENGINE_1] = LP5523_MODE_ENG1_M,
		[LP55XX_ENGINE_2] = LP5523_MODE_ENG2_M,
		[LP55XX_ENGINE_3] = LP5523_MODE_ENG3_M,
	};

	u8 val[] = {
		[LP55XX_ENGINE_1] = LP5523_LOAD_ENG1,
		[LP55XX_ENGINE_2] = LP5523_LOAD_ENG2,
		[LP55XX_ENGINE_3] = LP5523_LOAD_ENG3,
	};
	lp55xx_update_bits(chip, LP5523_REG_OP_MODE, mask[idx], val[idx]);
	lp5523_wait_opmode_done();
}

static void lp5523_load_engine_and_select_page(struct lp55xx_chip *chip)
{
	enum lp55xx_engine_index idx = chip->engine_idx;
	u8 page_sel[] = {
		[LP55XX_ENGINE_1] = LP5523_PAGE_ENG1,
		[LP55XX_ENGINE_2] = LP5523_PAGE_ENG2,
		[LP55XX_ENGINE_3] = LP5523_PAGE_ENG3,
	};

	lp5523_load_engine(chip);

	lp55xx_write(chip, LP5523_REG_PROG_PAGE_SEL, page_sel[idx]);
}

#if IS_ENABLED(CONFIG_LGE_LEDS_LP5523)
static void lp5523_load_all_engines(struct lp55xx_chip *chip)
{
	lp55xx_update_bits(chip, LP5523_REG_OP_MODE, LP5523_MODE_ALL_ENGS, LP5523_LOAD_ALL_ENGS);
	lp5523_wait_opmode_done();
}
#endif

static void lp5523_stop_all_engines(struct lp55xx_chip *chip)
{
	lp55xx_update_bits(chip, LP5523_REG_OP_MODE, LP5523_MODE_ALL_ENGS, 0);
	lp5523_wait_opmode_done();
}

static void lp5523_stop_engine(struct lp55xx_chip *chip)
{
	enum lp55xx_engine_index idx = chip->engine_idx;
	u8 mask[] = {
		[LP55XX_ENGINE_1] = LP5523_MODE_ENG1_M,
		[LP55XX_ENGINE_2] = LP5523_MODE_ENG2_M,
		[LP55XX_ENGINE_3] = LP5523_MODE_ENG3_M,
	};

	lp55xx_update_bits(chip, LP5523_REG_OP_MODE, mask[idx], 0);

	lp5523_wait_opmode_done();
}

static void lp5523_turn_off_channels(struct lp55xx_chip *chip)
{
	int i;

	for (i = 0; i < LP5523_MAX_LEDS; i++)
		lp55xx_write(chip, LP5523_REG_LED_PWM_BASE + i, 0);
}

static void lp5523_run_engine(struct lp55xx_chip *chip, bool start)
{
	int ret;
	u8 mode;
	u8 exec;

	/* stop engine */
	if (!start) {
		lp5523_stop_engine(chip);
		lp5523_turn_off_channels(chip);
		return;
	}

	/*
	 * To run the engine,
	 * operation mode and enable register should updated at the same time
	 */

	ret = lp55xx_read(chip, LP5523_REG_OP_MODE, &mode);
	if (ret)
		return;

	ret = lp55xx_read(chip, LP5523_REG_ENABLE, &exec);
	if (ret)
		return;

	/* change operation mode to RUN only when each engine is loading */
	if (LP5523_ENG1_IS_LOADING(mode)) {
		mode = (mode & ~LP5523_MODE_ENG1_M) | LP5523_RUN_ENG1;
		exec = (exec & ~LP5523_EXEC_ENG1_M) | LP5523_RUN_ENG1;
	}

	if (LP5523_ENG2_IS_LOADING(mode)) {
		mode = (mode & ~LP5523_MODE_ENG2_M) | LP5523_RUN_ENG2;
		exec = (exec & ~LP5523_EXEC_ENG2_M) | LP5523_RUN_ENG2;
	}

	if (LP5523_ENG3_IS_LOADING(mode)) {
		mode = (mode & ~LP5523_MODE_ENG3_M) | LP5523_RUN_ENG3;
		exec = (exec & ~LP5523_EXEC_ENG3_M) | LP5523_RUN_ENG3;
	}

	lp55xx_write(chip, LP5523_REG_OP_MODE, mode);
	lp5523_wait_opmode_done();

	lp55xx_update_bits(chip, LP5523_REG_ENABLE, LP5523_EXEC_M, exec);
}

#if !defined(CONFIG_LGE_LEDS_LP5523)
static int lp5523_init_program_engine(struct lp55xx_chip *chip)
{
	int i;
	int j;
	int ret;
	u8 status;
	/* one pattern per engine setting LED MUX start and stop addresses */
	static const u8 pattern[][LP5523_PROGRAM_LENGTH] =  {
		{ 0x9c, 0x30, 0x9c, 0xb0, 0x9d, 0x80, 0xd8, 0x00, 0},
		{ 0x9c, 0x40, 0x9c, 0xc0, 0x9d, 0x80, 0xd8, 0x00, 0},
		{ 0x9c, 0x50, 0x9c, 0xd0, 0x9d, 0x80, 0xd8, 0x00, 0},
	};

	/* hardcode 32 bytes of memory for each engine from program memory */
	ret = lp55xx_write(chip, LP5523_REG_CH1_PROG_START, 0x00);
	if (ret)
		return ret;

	ret = lp55xx_write(chip, LP5523_REG_CH2_PROG_START, 0x10);
	if (ret)
		return ret;

	ret = lp55xx_write(chip, LP5523_REG_CH3_PROG_START, 0x20);
	if (ret)
		return ret;

	/* write LED MUX address space for each engine */
	for (i = LP55XX_ENGINE_1; i <= LP55XX_ENGINE_3; i++) {
		chip->engine_idx = i;
		lp5523_load_engine_and_select_page(chip);

		for (j = 0; j < LP5523_PROGRAM_LENGTH; j++) {
			ret = lp55xx_write(chip, LP5523_REG_PROG_MEM + j,
					pattern[i - 1][j]);
			if (ret)
				goto out;
		}
	}

	lp5523_run_engine(chip, true);

	/* Let the programs run for couple of ms and check the engine status */
	usleep_range(3000, 6000);
	lp55xx_read(chip, LP5523_REG_STATUS, &status);
	status &= LP5523_ENG_STATUS_MASK;

	if (status != LP5523_ENG_STATUS_MASK) {
		dev_err(&chip->cl->dev,
			"cound not configure LED engine, status = 0x%.2x\n",
			status);
		ret = -1;
	}

out:
	lp5523_stop_all_engines(chip);
	return ret;
}
#endif
static int lp5523_update_program_memory(struct lp55xx_chip *chip,
					const u8 *data, size_t size)
{
	u8 pattern[LP5523_PROGRAM_LENGTH] = {0};
	unsigned cmd;
	char c[3];
	int nrchars;
	int ret;
	int offset = 0;
	int i = 0;

	while ((offset < size - 1) && (i < LP5523_PROGRAM_LENGTH)) {
		/* separate sscanfs because length is working only for %s */
		ret = sscanf(data + offset, "%2s%n ", c, &nrchars);
		if (ret != 1)
			goto err;

		ret = sscanf(c, "%2x", &cmd);
		if (ret != 1)
			goto err;

		pattern[i] = (u8)cmd;
		offset += nrchars;
		i++;
	}

	/* Each instruction is 16bit long. Check that length is even */
	if (i % 2)
		goto err;

	for (i = 0; i < LP5523_PROGRAM_LENGTH; i++) {
		ret = lp55xx_write(chip, LP5523_REG_PROG_MEM + i, pattern[i]);
		if (ret)
			return -EINVAL;
	}

	return size;

err:
	dev_err(&chip->cl->dev, "wrong pattern format\n");
	return -EINVAL;
}

static void lp5523_firmware_loaded(struct lp55xx_chip *chip)
{
	const struct firmware *fw = chip->fw;

	if (fw->size > LP5523_PROGRAM_LENGTH) {
		dev_err(&chip->cl->dev, "firmware data size overflow: %zu\n",
			fw->size);
		return;
	}

	/*
	 * Program momery sequence
	 *  1) set engine mode to "LOAD"
	 *  2) write firmware data into program memory
	 */

	lp5523_load_engine_and_select_page(chip);
	lp5523_update_program_memory(chip, fw->data, fw->size);
}

static ssize_t show_engine_mode(struct device *dev,
				struct device_attribute *attr,
				char *buf, int nr)
{
	struct lp55xx_led *led = i2c_get_clientdata(to_i2c_client(dev));
	struct lp55xx_chip *chip = led->chip;
	enum lp55xx_engine_mode mode = chip->engines[nr - 1].mode;

	switch (mode) {
	case LP55XX_ENGINE_RUN:
		return sprintf(buf, "run\n");
	case LP55XX_ENGINE_LOAD:
		return sprintf(buf, "load\n");
	case LP55XX_ENGINE_DISABLED:
	default:
		return sprintf(buf, "disabled\n");
	}
}
show_mode(1)
show_mode(2)
show_mode(3)

static ssize_t store_engine_mode(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t len, int nr)
{
	struct lp55xx_led *led = i2c_get_clientdata(to_i2c_client(dev));
	struct lp55xx_chip *chip = led->chip;
	struct lp55xx_engine *engine = &chip->engines[nr - 1];

	mutex_lock(&chip->lock);

	chip->engine_idx = nr;

	if (!strncmp(buf, "run", 3)) {
		lp5523_run_engine(chip, true);
		engine->mode = LP55XX_ENGINE_RUN;
	} else if (!strncmp(buf, "load", 4)) {
		lp5523_stop_engine(chip);
		lp5523_load_engine(chip);
		engine->mode = LP55XX_ENGINE_LOAD;
	} else if (!strncmp(buf, "disabled", 8)) {
		lp5523_stop_engine(chip);
		engine->mode = LP55XX_ENGINE_DISABLED;
	}

	mutex_unlock(&chip->lock);

	return len;
}
store_mode(1)
store_mode(2)
store_mode(3)

static int lp5523_mux_parse(const char *buf, u16 *mux, size_t len)
{
	u16 tmp_mux = 0;
	int i;

	len = min_t(int, len, LP5523_MAX_LEDS);

	for (i = 0; i < len; i++) {
		switch (buf[i]) {
		case '1':
			tmp_mux |= (1 << i);
			break;
		case '0':
			break;
		case '\n':
			i = len;
			break;
		default:
			return -1;
		}
	}
	*mux = tmp_mux;

	return 0;
}

static void lp5523_mux_to_array(u16 led_mux, char *array)
{
	int i, pos = 0;
	for (i = 0; i < LP5523_MAX_LEDS; i++)
		pos += sprintf(array + pos, "%x", LED_ACTIVE(led_mux, i));

	array[pos] = '\0';
}

static ssize_t show_engine_leds(struct device *dev,
			    struct device_attribute *attr,
			    char *buf, int nr)
{
	struct lp55xx_led *led = i2c_get_clientdata(to_i2c_client(dev));
	struct lp55xx_chip *chip = led->chip;
	char mux[LP5523_MAX_LEDS + 1];

	lp5523_mux_to_array(chip->engines[nr - 1].led_mux, mux);

	return sprintf(buf, "%s\n", mux);
}
show_leds(1)
show_leds(2)
show_leds(3)

static int lp5523_load_mux(struct lp55xx_chip *chip, u16 mux, int nr)
{
	struct lp55xx_engine *engine = &chip->engines[nr - 1];
	int ret;
	u8 mux_page[] = {
		[LP55XX_ENGINE_1] = LP5523_PAGE_MUX1,
		[LP55XX_ENGINE_2] = LP5523_PAGE_MUX2,
		[LP55XX_ENGINE_3] = LP5523_PAGE_MUX3,
	};

	lp5523_load_engine(chip);

	ret = lp55xx_write(chip, LP5523_REG_PROG_PAGE_SEL, mux_page[nr]);
	if (ret)
		return ret;

	ret = lp55xx_write(chip, LP5523_REG_PROG_MEM , (u8)(mux >> 8));
	if (ret)
		return ret;

	ret = lp55xx_write(chip, LP5523_REG_PROG_MEM + 1, (u8)(mux));
	if (ret)
		return ret;

	engine->led_mux = mux;
	return 0;
}

static ssize_t store_engine_leds(struct device *dev,
			     struct device_attribute *attr,
			     const char *buf, size_t len, int nr)
{
	struct lp55xx_led *led = i2c_get_clientdata(to_i2c_client(dev));
	struct lp55xx_chip *chip = led->chip;
	struct lp55xx_engine *engine = &chip->engines[nr - 1];
	u16 mux = 0;
	ssize_t ret;

	if (lp5523_mux_parse(buf, &mux, len))
		return -EINVAL;

	mutex_lock(&chip->lock);

	chip->engine_idx = nr;
	ret = -EINVAL;

	if (engine->mode != LP55XX_ENGINE_LOAD)
		goto leave;

	if (lp5523_load_mux(chip, mux, nr))
		goto leave;

	ret = len;
leave:
	mutex_unlock(&chip->lock);
	return ret;
}
store_leds(1)
store_leds(2)
store_leds(3)

static ssize_t store_engine_load(struct device *dev,
			     struct device_attribute *attr,
			     const char *buf, size_t len, int nr)
{
	struct lp55xx_led *led = i2c_get_clientdata(to_i2c_client(dev));
	struct lp55xx_chip *chip = led->chip;
	int ret;

	mutex_lock(&chip->lock);

	chip->engine_idx = nr;
	lp5523_load_engine_and_select_page(chip);
	ret = lp5523_update_program_memory(chip, buf, len);

	mutex_unlock(&chip->lock);

	return ret;
}
store_load(1)
store_load(2)
store_load(3)

static ssize_t lp5523_selftest(struct device *dev,
			       struct device_attribute *attr,
			       char *buf)
{
	struct lp55xx_led *led = i2c_get_clientdata(to_i2c_client(dev));
	struct lp55xx_chip *chip = led->chip;
	struct lp55xx_platform_data *pdata = chip->pdata;
	int i, ret, pos = 0;
	u8 status, adc, vdd;

	mutex_lock(&chip->lock);

	ret = lp55xx_read(chip, LP5523_REG_STATUS, &status);
	if (ret < 0)
		goto fail;

	/* Check that ext clock is really in use if requested */
	if (pdata->clock_mode == LP55XX_CLOCK_EXT) {
		if  ((status & LP5523_EXT_CLK_USED) == 0)
			goto fail;
	}

	/* Measure VDD (i.e. VBAT) first (channel 16 corresponds to VDD) */
	lp55xx_write(chip, LP5523_REG_LED_TEST_CTRL, LP5523_EN_LEDTEST | 16);
	usleep_range(3000, 6000); /* ADC conversion time is typically 2.7 ms */
	ret = lp55xx_read(chip, LP5523_REG_STATUS, &status);
	if (ret < 0)
		goto fail;

	if (!(status & LP5523_LEDTEST_DONE))
		usleep_range(3000, 6000); /* Was not ready. Wait little bit */

	ret = lp55xx_read(chip, LP5523_REG_LED_TEST_ADC, &vdd);
	if (ret < 0)
		goto fail;

	vdd--;	/* There may be some fluctuation in measurement */

	for (i = 0; i < LP5523_MAX_LEDS; i++) {
		/* Skip non-existing channels */
		if (pdata->led_config[i].led_current == 0)
			continue;

		/* Set default current */
		lp55xx_write(chip, LP5523_REG_LED_CURRENT_BASE + i,
			pdata->led_config[i].led_current);

		lp55xx_write(chip, LP5523_REG_LED_PWM_BASE + i, 0xff);
		/* let current stabilize 2 - 4ms before measurements start */
		usleep_range(2000, 4000);
		lp55xx_write(chip, LP5523_REG_LED_TEST_CTRL,
			     LP5523_EN_LEDTEST | i);
		/* ADC conversion time is 2.7 ms typically */
		usleep_range(3000, 6000);
		ret = lp55xx_read(chip, LP5523_REG_STATUS, &status);
		if (ret < 0)
			goto fail;

		if (!(status & LP5523_LEDTEST_DONE))
			usleep_range(3000, 6000);/* Was not ready. Wait. */

		ret = lp55xx_read(chip, LP5523_REG_LED_TEST_ADC, &adc);
		if (ret < 0)
			goto fail;

		if (adc >= vdd || adc < LP5523_ADC_SHORTCIRC_LIM)
			pos += sprintf(buf + pos, "LED %d FAIL\n", i);

		lp55xx_write(chip, LP5523_REG_LED_PWM_BASE + i, 0x00);

		/* Restore current */
		lp55xx_write(chip, LP5523_REG_LED_CURRENT_BASE + i,
			led->led_current);
		led++;
	}
	if (pos == 0)
		pos = sprintf(buf, "OK\n");
	goto release_lock;
fail:
	pos = sprintf(buf, "FAIL\n");

release_lock:
	mutex_unlock(&chip->lock);

	return pos;
}

#define show_fader(nr)						\
static ssize_t show_master_fader##nr(struct device *dev,	\
			    struct device_attribute *attr,	\
			    char *buf)				\
{								\
	return show_master_fader(dev, attr, buf, nr);		\
}

#define store_fader(nr)						\
static ssize_t store_master_fader##nr(struct device *dev,	\
			     struct device_attribute *attr,	\
			     const char *buf, size_t len)	\
{								\
	return store_master_fader(dev, attr, buf, len, nr);	\
}

static ssize_t show_master_fader(struct device *dev,
				 struct device_attribute *attr,
				 char *buf, int nr)
{
	struct lp55xx_led *led = i2c_get_clientdata(to_i2c_client(dev));
	struct lp55xx_chip *chip = led->chip;
	int ret;
	u8 val;

	mutex_lock(&chip->lock);
	ret = lp55xx_read(chip, LP5523_REG_MASTER_FADER_BASE + nr - 1, &val);
	mutex_unlock(&chip->lock);

	if (ret == 0)
		ret = sprintf(buf, "%u\n", val);

	return ret;
}
show_fader(1)
show_fader(2)
show_fader(3)

static ssize_t store_master_fader(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t len, int nr)
{
	struct lp55xx_led *led = i2c_get_clientdata(to_i2c_client(dev));
	struct lp55xx_chip *chip = led->chip;
	int ret;
	unsigned long val;

	if (kstrtoul(buf, 0, &val))
		return -EINVAL;

	if (val > 0xff)
		return -EINVAL;

	mutex_lock(&chip->lock);
	ret = lp55xx_write(chip, LP5523_REG_MASTER_FADER_BASE + nr - 1,
			   (u8)val);
	mutex_unlock(&chip->lock);

	if (ret == 0)
		ret = len;

	return ret;
}
store_fader(1)
store_fader(2)
store_fader(3)

static ssize_t show_master_fader_leds(struct device *dev,
				      struct device_attribute *attr,
				      char *buf)
{
	struct lp55xx_led *led = i2c_get_clientdata(to_i2c_client(dev));
	struct lp55xx_chip *chip = led->chip;
	int i, ret, pos = 0;
	u8 val;

	mutex_lock(&chip->lock);

	for (i = 0; i < LP5523_MAX_LEDS; i++) {
		ret = lp55xx_read(chip, LP5523_REG_LED_CTRL_BASE + i, &val);
		if (ret)
			goto leave;

		val = (val & LP5523_FADER_MAPPING_MASK)
			>> LP5523_FADER_MAPPING_SHIFT;
		if (val > 3) {
			ret = -EINVAL;
			goto leave;
		}
		buf[pos++] = val + '0';
	}
	buf[pos++] = '\n';
	ret = pos;
leave:
	mutex_unlock(&chip->lock);
	return ret;
}

static ssize_t store_master_fader_leds(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t len)
{
	struct lp55xx_led *led = i2c_get_clientdata(to_i2c_client(dev));
	struct lp55xx_chip *chip = led->chip;
	int i, n, ret;
	u8 val;

	n = min_t(int, len, LP5523_MAX_LEDS);

	mutex_lock(&chip->lock);

	for (i = 0; i < n; i++) {
		if (buf[i] >= '0' && buf[i] <= '3') {
			val = (buf[i] - '0') << LP5523_FADER_MAPPING_SHIFT;
			ret = lp55xx_update_bits(chip,
						 LP5523_REG_LED_CTRL_BASE + i,
						 LP5523_FADER_MAPPING_MASK,
						 val);
			if (ret)
				goto leave;
		} else {
			ret = -EINVAL;
			goto leave;
		}
	}
	ret = len;
leave:
	mutex_unlock(&chip->lock);
	return ret;
}

#if IS_ENABLED(CONFIG_LGE_LEDS_LP5523)
static ssize_t show_led_pattern(struct device *dev,
				      struct device_attribute *attr,
				      char *buf)
{
	struct lp55xx_led *led = i2c_get_clientdata(to_i2c_client(dev));
	struct lp55xx_chip *chip = led->chip;
	struct lp55xx_platform_data *pdata = chip->pdata;

	return sprintf(buf, "%d\n", pdata->pattern_play_id);
}
static ssize_t play_pattern(struct device *dev, int val)
{
	struct lp55xx_led *led = i2c_get_clientdata(to_i2c_client(dev));
	struct lp55xx_chip *chip = led->chip;

	struct lp55xx_platform_data *pdata = chip->pdata;
	int i, j, pattern_idx;
	int ret;
	dev_info(dev, "[%s] pattern[%d]\n", __func__, (int)val);

	mutex_lock(&chip->lock);
	if (!pdata->is_enabled) {
		ret = lp55xx_init_device(chip);
		if (ret) {
			dev_err(dev, "[%s] init faied\n",  __func__);
			mutex_unlock(&chip->lock);
			return -EINVAL;
		}
	}

	if (val == 0) {
		pdata->pattern_play_id = 0;
		lp5523_stop_all_engines(chip);
		lp5523_turn_off_channels(chip);
		if (pdata->is_enabled)
			lp55xx_deinit_device(chip);
		dev_err(dev, "[%s]stopped pattern [%d]\n",  __func__,(int)val);
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}
	for (i = 0; i < pdata->num_patterns ; i++) {
		if(val == pdata->patterns[i]->pattern_idx) {
			pattern_idx = i;
			break;
		}
	}
	if(LP5523_ENABLE_DEBBUG_LOG)
		dev_info(dev, "[%s] i = %d , pdata->num_patterns= [%d]\n", __func__, i, pdata->num_patterns);

	if (i >= pdata->num_patterns) {
		dev_err(dev, "[%s] no pattern id [%d]\n",  __func__,(int)val);
		if (pdata->is_enabled)
			lp55xx_deinit_device(chip);
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}

	pdata->pattern_play_id = val;

	lp5523_stop_all_engines(chip);
	lp5523_load_all_engines(chip);

	for (i = 0; i < LP5523_NUM_OF_PAGE; i++) {
		lp55xx_write(chip, LP5523_REG_PROG_PAGE_SEL, i);

		for(j = 0; j < LP5523_PROGRAM_LENGTH; j++) {
				lp55xx_write(chip, LP5523_REG_PROG_MEM + j,
					pdata->patterns[pattern_idx]->hex_of_pattern[i*LP5523_PROGRAM_LENGTH+j]);
		}
	}

	for(i = 0; i < LP5523_NUM_ENGINE ; i++) {
			lp55xx_write(chip,LP5523_REG_CH1_PROG_START + i ,pdata->patterns[pattern_idx]->start_addr[i]);
	}

	lp5523_run_engine(chip, 1);

	mutex_unlock(&chip->lock);

	return val;
}
static ssize_t store_led_hidden_pattern(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t len)
{
	struct lp55xx_led *led = i2c_get_clientdata(to_i2c_client(dev));
	struct lp55xx_chip *chip = led->chip;
	struct lp55xx_platform_data *pdata = chip->pdata;
	unsigned long val;
	int i, ret = len;

	if (kstrtoul(buf, 0, &val))
		return -EINVAL;

	dev_info(dev, "[%s] pattern[%d]\n", __func__, (int)val);

	if (val < 0 || val  > pdata->num_patterns + 1)
		return -EINVAL;

	if (val == 0) {
		mutex_lock(&chip->lock);
		for(i = 0; i < LP5523_MAX_LEDS; i++) {
			lp55xx_write(chip, LP5523_REG_LED_PWM_BASE + i,
				     0);
		}
		mutex_unlock(&chip->lock);
		ret = play_pattern(dev, val);
	}
	else if (val <= pdata->num_patterns)
		ret = play_pattern(dev, (int)pdata->patterns[val-1]->pattern_idx);
	else {
		if (pdata->is_enabled)
			play_pattern(dev, 0);
		mutex_lock(&chip->lock);
		if (!pdata->is_enabled) {
			if (lp55xx_init_device(chip)) {
				dev_err(dev, "[%s] init faied\n",  __func__);
				mutex_unlock(&chip->lock);
				return -EINVAL;
			}
		}
		for (i = 0; i < LP5523_MAX_LEDS; i++) {
			lp55xx_write(chip, LP5523_REG_LED_PWM_BASE + i,
				     LP5523_MAX_LED_BRIGHTNESS);
		}
		mutex_unlock(&chip->lock);
	}

	return ret;
}

static ssize_t show_led_current(struct device *dev,
				      struct device_attribute *attr,
				      char *buf)
{
	struct lp55xx_led *led = i2c_get_clientdata(to_i2c_client(dev));
	struct lp55xx_chip *chip = led->chip;
	struct lp55xx_platform_data *pdata = chip->pdata;
	if(!hidden_led_current)
		hidden_led_current = pdata->led_config[0].led_current /
		pdata->channels_of_each_led[0];

	return sprintf(buf, "%d\n", hidden_led_current);
}

static ssize_t store_led_current(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t len)
{
	struct lp55xx_led *led = i2c_get_clientdata(to_i2c_client(dev));
	struct lp55xx_chip *chip = led->chip;
	struct lp55xx_platform_data *pdata = chip->pdata;
	unsigned long val;
	int i, ret = 1;

	if (kstrtoul(buf, 0, &val))
		return -EINVAL;

	dev_info(dev, "[%s] led_current[%d]\n", __func__, (int)val);

	if (val <= 0 || val  > (LP5523_MAX_LED_CURRENT >> 1) )
		return -EINVAL;
	hidden_led_current = val;
	mutex_lock(&chip->lock);
	if (!pdata->is_enabled) {
		ret = lp55xx_init_device(chip);
		if (ret) {
			dev_err(dev, "[%s] init faied\n",  __func__);
			mutex_unlock(&chip->lock);
			return -EINVAL;
		}
	}
	for (i = 0; i < LP5523_MAX_LEDS; i++) {
		lp55xx_write(led->chip, LP5523_REG_LED_CURRENT_BASE + i,
				val * pdata->channels_of_each_led[i]);
	}
	mutex_unlock(&chip->lock);

	return len;
}

static ssize_t store_led_pattern(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t len)
{
	struct lp55xx_led *led = i2c_get_clientdata(to_i2c_client(dev));
	struct lp55xx_chip *chip = led->chip;
	struct lp55xx_platform_data *pdata = chip->pdata;
	unsigned long val;

	if (kstrtoul(buf, 0, &val))
		return -EINVAL;

	dev_info(dev, "[%s] pattern[%d], played_pattern[%d]\n", __func__, (int)val, pdata->pattern_play_id);

	if (val == pdata->pattern_play_id) {
		dev_err(dev, "[%s] already played pattern[%d]\n", __func__, (int)val);
		return -EINVAL;
	}

	return play_pattern(dev, (int)val);
}

static ssize_t show_enable_led(struct device *dev,
				      struct device_attribute *attr,
				      char *buf)
{
	struct lp55xx_led *led = i2c_get_clientdata(to_i2c_client(dev));
	struct lp55xx_chip *chip = led->chip;
	struct lp55xx_platform_data *pdata = chip->pdata;

	return sprintf(buf, "%d\n", pdata->is_enabled);
}

static ssize_t store_enable_led(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t len)
{
	struct lp55xx_led *led = i2c_get_clientdata(to_i2c_client(dev));
	struct lp55xx_chip *chip = led->chip;
	struct lp55xx_platform_data *pdata = chip->pdata;
	int ret;
	unsigned long val;

	if (kstrtoul(buf, 0, &val))
		return -EINVAL;

	dev_info(dev, "[%s] enable[%d], is_enabled[%d]\n", __func__, (int)val, pdata->is_enabled);

	if (val == pdata->is_enabled) {
		dev_err(dev, "[%s] already %s\n", __func__, pdata->is_enabled ? "enabled": "disabled");
		return -EINVAL;
	}
	mutex_lock(&chip->lock);
	if (!pdata->is_enabled) {
		ret = lp55xx_init_device(chip);
		if (ret) {
			dev_err(dev, "[%s] init faied\n",  __func__);
			mutex_unlock(&chip->lock);
			return -EINVAL;
		}
	}
	else {
		lp55xx_deinit_device(chip);
	}
	mutex_unlock(&chip->lock);

	return val;
}

static u8 get_num(char element[])
{
	u8 num = 0;
	if(element[0] >= 'A')
		num = (u8)(element[0] - 'A' + 10);
	else
		num = (u8)(element[0] - '0');

	num = num * 16;

	if(element[1] >= 'A')
		num += (u8)(element[1] - 'A' + 10);
	else
		num += (u8)(element[1] - '0');
	return num;
}
static ssize_t show_pattern_test(struct device *dev,
				      struct device_attribute *attr,
				      char *buf)
{
	struct lp55xx_led *led = i2c_get_clientdata(to_i2c_client(dev));
	struct lp55xx_chip *chip = led->chip;
	struct lp55xx_platform_data *pdata = chip->pdata;

	int fd = 0;
	int i, j, k,ret, inst_idx = 0;
	char pattern_buf[50] = {0,};
	char element[2] = {0,};
	u8 instructions[LP5523_NUM_INST_MEM];
	u8 start_addr[LP5523_NUM_ENGINE];
	mm_segment_t old_fs = get_fs();

	set_fs(KERNEL_DS);
	fd = sys_open("/data/logger/test.hex", O_RDONLY, 0);
	sys_chmod("/data/logger/test.hex", 0666);

	if (fd >= 0) {
		for(i = 0; i < 12; i++) {
			sys_read(fd, pattern_buf, sizeof(pattern_buf));

			k = 0;
			for(j = 0; j < 50; j++) {
				if(pattern_buf[j] != 13 && pattern_buf[j] != 10 && pattern_buf[j] != 32 ) { // 10 : Line Feed, 13:Carriage Return, 32:Space
					element[k++] = pattern_buf[j];
					if(k == 2) {
						instructions[inst_idx++] = get_num(element);
						k = 0;
					}
				}
			}
		}

		for(i = 0; i < 3; i++) {
			sys_read(fd, pattern_buf, 15);
			element[0] = pattern_buf[2];
			element[1] = pattern_buf[3];

			start_addr[i] = get_num(element);
		}
		sys_close(fd);
	}
	set_fs(old_fs);

	mutex_lock(&chip->lock);
	if (!pdata->is_enabled) {
		ret = lp55xx_init_device(chip);
		if (ret) {
			dev_err(dev, "[%s] init faied\n",  __func__);
			mutex_unlock(&chip->lock);
			return -EINVAL;
		}
	}

	lp5523_stop_all_engines(chip);
	lp5523_load_all_engines(chip);

	for (i = 0; i < LP5523_NUM_OF_PAGE; i++) {
		lp55xx_write(chip, LP5523_REG_PROG_PAGE_SEL, i);

		for(j = 0; j < LP5523_PROGRAM_LENGTH; j++) {
				lp55xx_write(chip, LP5523_REG_PROG_MEM + j,
					instructions[i*LP5523_PROGRAM_LENGTH+j]);
		}
	}
	for(i = 0; i < LP5523_NUM_ENGINE ; i++) {
			lp55xx_write(chip,LP5523_REG_CH1_PROG_START + i ,start_addr[i]);
	}
	lp5523_run_engine(chip, 1);
	mutex_unlock(&chip->lock);

	return fd;
}
static ssize_t show_pattern_num(struct device *dev,
				      struct device_attribute *attr,
				      char *buf)
{
	struct lp55xx_led *led = i2c_get_clientdata(to_i2c_client(dev));
	struct lp55xx_chip *chip = led->chip;
	struct lp55xx_platform_data *pdata = chip->pdata;

	return sprintf(buf, "%d\n", pdata->num_patterns);
}


#endif
 static void lp5523_led_brightness_work(struct work_struct *work)
{
	struct lp55xx_led *led = container_of(work, struct lp55xx_led,
					      brightness_work);
	struct lp55xx_chip *chip = led->chip;

	mutex_lock(&chip->lock);
	lp55xx_write(chip, LP5523_REG_LED_PWM_BASE + led->chan_nr,
		     led->brightness);
	mutex_unlock(&chip->lock);
}

static LP55XX_DEV_ATTR_RW(engine1_mode, show_engine1_mode, store_engine1_mode);
static LP55XX_DEV_ATTR_RW(engine2_mode, show_engine2_mode, store_engine2_mode);
static LP55XX_DEV_ATTR_RW(engine3_mode, show_engine3_mode, store_engine3_mode);
static LP55XX_DEV_ATTR_RW(engine1_leds, show_engine1_leds, store_engine1_leds);
static LP55XX_DEV_ATTR_RW(engine2_leds, show_engine2_leds, store_engine2_leds);
static LP55XX_DEV_ATTR_RW(engine3_leds, show_engine3_leds, store_engine3_leds);
static LP55XX_DEV_ATTR_WO(engine1_load, store_engine1_load);
static LP55XX_DEV_ATTR_WO(engine2_load, store_engine2_load);
static LP55XX_DEV_ATTR_WO(engine3_load, store_engine3_load);
static LP55XX_DEV_ATTR_RO(selftest, lp5523_selftest);
static LP55XX_DEV_ATTR_RW(master_fader1, show_master_fader1,
			  store_master_fader1);
static LP55XX_DEV_ATTR_RW(master_fader2, show_master_fader2,
			  store_master_fader2);
static LP55XX_DEV_ATTR_RW(master_fader3, show_master_fader3,
			  store_master_fader3);
static LP55XX_DEV_ATTR_RW(master_fader_leds, show_master_fader_leds,
			  store_master_fader_leds);
#if IS_ENABLED(CONFIG_LGE_LEDS_LP5523)
static LP55XX_DEV_ATTR_RW(pattern, show_led_pattern,
			  store_led_pattern);
static LP55XX_DEV_ATTR_RW(enable_led, show_enable_led,
			  store_enable_led);
static LP55XX_DEV_ATTR_RO(pattern_test, show_pattern_test);
static LP55XX_DEV_ATTR_RO(num_of_pattern, show_pattern_num);

static LP55XX_DEV_ATTR_WO(hidden_pattern, store_led_hidden_pattern);
static LP55XX_DEV_ATTR_RW(hidden_led_current, show_led_current, store_led_current);
#endif
static struct attribute *lp5523_attributes[] = {
	&dev_attr_engine1_mode.attr,
	&dev_attr_engine2_mode.attr,
	&dev_attr_engine3_mode.attr,
	&dev_attr_engine1_load.attr,
	&dev_attr_engine2_load.attr,
	&dev_attr_engine3_load.attr,
	&dev_attr_engine1_leds.attr,
	&dev_attr_engine2_leds.attr,
	&dev_attr_engine3_leds.attr,
	&dev_attr_selftest.attr,
	&dev_attr_master_fader1.attr,
	&dev_attr_master_fader2.attr,
	&dev_attr_master_fader3.attr,
	&dev_attr_master_fader_leds.attr,
#if IS_ENABLED(CONFIG_LGE_LEDS_LP5523)
	&dev_attr_pattern.attr,
	&dev_attr_enable_led.attr,
	&dev_attr_pattern_test.attr,
	&dev_attr_num_of_pattern.attr,
	&dev_attr_hidden_pattern.attr,
	&dev_attr_hidden_led_current.attr,
#endif
	NULL,
};

static const struct attribute_group lp5523_group = {
	.attrs = lp5523_attributes,
};

/* Chip specific configurations */
static struct lp55xx_device_config lp5523_cfg = {
	.reset = {
		.addr = LP5523_REG_RESET,
		.val  = LP5523_RESET,
	},
	.enable = {
		.addr = LP5523_REG_ENABLE,
		.val  = LP5523_ENABLE,
	},
	.max_channel  = LP5523_MAX_LEDS,
	.post_init_device   = lp5523_post_init_device,
	.brightness_work_fn = lp5523_led_brightness_work,
	.set_led_current    = lp5523_set_led_current,
	.firmware_cb        = lp5523_firmware_loaded,
	.run_engine         = lp5523_run_engine,
	.dev_attr_group     = &lp5523_group,
};

static irqreturn_t lp5523_irq_handler(int irq, void *d)
{
	struct lp55xx_chip *chip = d;
	struct lp55xx_platform_data *pdata = chip->pdata;
	u8 status = 0;

	lp55xx_read(chip, LP5523_REG_STATUS, &status);
	status &= LP5523_ENG_STATUS_MASK;

	if (status) {
#if IS_ENABLED(CONFIG_LGE_LEDS_LP5523)
		mutex_lock(&chip->lock);
		if (pdata->is_enabled) {
			dev_info(&chip->cl->dev, "[%s]stopped pattern [%d] stopped_engine[%d]\n",  __func__,pdata->pattern_play_id, status);
			pdata->pattern_play_id = 0;
			lp5523_stop_all_engines(chip);
			lp5523_turn_off_channels(chip);
			lp55xx_deinit_device(chip);
		} else {
			dev_err(&chip->cl->dev, "[%s]already stopped\n",  __func__);
		}
		mutex_unlock(&chip->lock);
#else
		dev_info(&chip->cl->dev, "Engine IRQ status: 0x%x\n", status);
#endif
	}

	return IRQ_HANDLED;
}

static void lp5523_setup_irq(struct lp55xx_chip *chip)
{
	struct device *dev = &chip->cl->dev;
	int irq_gpio = chip->pdata->irq_gpio;
	int irq;
	int err;

	/* Interrupt handling is optional */
	if (!gpio_is_valid(irq_gpio))
		return;

	err = devm_gpio_request_one(dev, irq_gpio, GPIOF_IN, "LP5523_INT");
	if (err) {
		dev_info(dev, "Failed to request GPIO %d: %d\n", irq_gpio, err);
		return;
	}

	irq = gpio_to_irq(irq_gpio);
	err = request_threaded_irq(irq, NULL, lp5523_irq_handler,
				   IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				   "lp5523-irq", chip);
	if (err) {
		dev_info(dev, "Failed to request irq %d: %d\n", irq, err);
		return;
	}

	chip->irq = irq;
}

static int lp5523_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int ret;
	struct lp55xx_chip *chip;
	struct lp55xx_led *led;
	struct lp55xx_platform_data *pdata = dev_get_platdata(&client->dev);
	struct device_node *np = client->dev.of_node;
#if IS_ENABLED(CONFIG_LGE_LEDS_LP5523)
	enum hw_rev_type rev = lge_get_board_revno();
#endif

	if (!pdata) {
		if (np) {
			pdata = lp55xx_of_populate_pdata(&client->dev, np);
			if (IS_ERR(pdata))
				return PTR_ERR(pdata);
		} else {
			dev_err(&client->dev, "no platform data\n");
			return -EINVAL;
		}
	}

	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	led = devm_kzalloc(&client->dev,
			sizeof(*led) * pdata->num_channels, GFP_KERNEL);
	if (!led)
		return -ENOMEM;

	chip->cl = client;
	chip->pdata = pdata;
	chip->cfg = &lp5523_cfg;

	mutex_init(&chip->lock);

	i2c_set_clientdata(client, led);

	ret = lp55xx_init_device(chip);
	if (ret)
		goto err_init;

	dev_info(&client->dev, "%s Programmable led chip found\n", id->name);

	ret = lp55xx_register_leds(led, chip);
	if (ret)
		goto err_register_leds;

	ret = lp55xx_register_sysfs(chip);
	if (ret) {
		dev_err(&client->dev, "registering sysfs failed\n");
		goto err_register_sysfs;
	}
#if IS_ENABLED(CONFIG_LGE_LEDS_LP5523)
	if(rev >= HW_REV_B)
#endif
		lp5523_setup_irq(chip);

#if IS_ENABLED(CONFIG_LGE_LEDS_LP5523)
	pdata->pattern_play_id = 1; // Play PowerOn Pattern
	play_pattern(&client->dev, 1);
#endif
	return 0;

err_register_sysfs:
	lp55xx_unregister_leds(led, chip);
err_register_leds:
	lp55xx_deinit_device(chip);
err_init:
	return ret;
}

static int lp5523_remove(struct i2c_client *client)
{
	struct lp55xx_led *led = i2c_get_clientdata(client);
	struct lp55xx_chip *chip = led->chip;

	lp5523_stop_all_engines(chip);

	if (chip->irq)
		free_irq(chip->irq, chip);

#if IS_ENABLED(CONFIG_LGE_LEDS_LP5523)
{
	int i;
	for(i = 0; i < chip->pdata->num_patterns; i++) {
		kfree(chip->pdata->patterns[i]->hex_of_pattern);
		kfree(chip->pdata->patterns[i]->start_addr);
		kfree(chip->pdata->patterns[i]);
	}
	kfree(chip->pdata->channels_of_each_led);
	kfree(chip->pdata->patterns);
}
#endif
	lp55xx_unregister_sysfs(chip);
	lp55xx_unregister_leds(led, chip);
	lp55xx_deinit_device(chip);

	return 0;
}

static const struct i2c_device_id lp5523_id[] = {
	{ "lp5523",  LP5523 },
	{ "lp55231", LP55231 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, lp5523_id);

#ifdef CONFIG_OF
static const struct of_device_id of_lp5523_leds_match[] = {
	{ .compatible = "national,lp5523", },
	{ .compatible = "ti,lp55231", },
	{},
};

MODULE_DEVICE_TABLE(of, of_lp5523_leds_match);
#endif

static struct i2c_driver lp5523_driver = {
	.driver = {
		.name	= "lp5523x",
		.of_match_table = of_match_ptr(of_lp5523_leds_match),
	},
	.probe		= lp5523_probe,
	.remove		= lp5523_remove,
	.id_table	= lp5523_id,
};

module_i2c_driver(lp5523_driver);

MODULE_AUTHOR("Mathias Nyman <mathias.nyman@nokia.com>");
MODULE_AUTHOR("Milo Kim <milo.kim@ti.com>");
MODULE_DESCRIPTION("LP5523 LED engine");
MODULE_LICENSE("GPL");
