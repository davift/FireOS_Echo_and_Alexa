// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2021 MediaTek Inc.
 * Author: Housong Zhang <housong.zhang@mediatek.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files
 * (the "Software"), to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify, merge,
 * publish, distribute, sublicense, and/or sell copies of the Software,
 * and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include <common.h>
#include <clk.h>
#include <dm.h>
#include <i2c.h>
#include <asm/io.h>
#include <asm/cache.h>
#include <linux/math64.h>

#define I2C_RS_TRANSFER			(1 << 4)
#define I2C_ARB_LOST			(1 << 3)
#define I2C_HS_NACKERR			(1 << 2)
#define I2C_ACKERR			(1 << 1)
#define I2C_TRANSAC_COMP		(1 << 0)
#define I2C_TRANSAC_START		(1 << 0)
#define I2C_RS_MUL_CNFG			(1 << 15)
#define I2C_RS_MUL_TRIG			(1 << 14)
#define I2C_TRANS_ADDR_ONLY		(1 << 8)
#define I2C_DCM_DISABLE			0x0000
#define I2C_IO_CONFIG_OPEN_DRAIN	0x0003
#define I2C_IO_CONFIG_PUSH_PULL		0x0000
#define I2C_SOFT_RST			0x0001
#define I2C_HANDSHAKE_RST		0x0020
#define I2C_FIFO_ADDR_CLR		0x0001
#define I2C_FIFO_ADDR_CLR_MCH		0x0004
#define I2C_DELAY_LEN			0x0002
#define I2C_TIME_CLR_VALUE		0x0000
#define I2C_TIME_DEFAULT_VALUE		0x0003
#define I2C_WRRD_TRANAC_VALUE		0x0002
#define I2C_RD_TRANAC_VALUE		0x0001
#define I2C_SCL_MIS_COMP_VALUE		0x0000
#define I2C_CHN_CLR_FLAG		0x0000

#define I2C_DMA_CON_TX			0x0000
#define I2C_DMA_CON_RX			0x0001
#define I2C_DMA_ASYNC_MODE		0x0004
#define I2C_DMA_SKIP_CONFIG		0x0010
#define I2C_DMA_DIR_CHANGE		0x0200
#define I2C_DMA_START_EN		0x0001
#define I2C_DMA_INT_FLAG_NONE		0x0000
#define I2C_DMA_CLR_FLAG		0x0000
#define I2C_DMA_WARM_RST		0x0001
#define I2C_DMA_HARD_RST		0x0002
#define I2C_DMA_HANDSHAKE_RST		0x0004

#define MAX_SAMPLE_CNT_DIV		8
#define MAX_STEP_CNT_DIV		64
#define MAX_CLOCK_DIV			256
#define MAX_HS_STEP_CNT_DIV		8
#define I2C_STANDARD_MODE_BUFFER	(1000 / 2)
#define I2C_FAST_MODE_BUFFER		(300 / 2)
#define I2C_FAST_MODE_PLUS_BUFFER	(20 / 2)

/* I2C Frequency Modes */
#define I2C_MAX_STANDARD_MODE_FREQ	100000
#define I2C_MAX_FAST_MODE_FREQ		400000
#define I2C_MAX_FAST_MODE_PLUS_FREQ	1000000
#define I2C_MAX_HIGH_SPEED_MODE_FREQ	3400000

#define I2C_CONTROL_RS                  (0x1 << 1)
#define I2C_CONTROL_DMA_EN              (0x1 << 2)
#define I2C_CONTROL_CLK_EXT_EN          (0x1 << 3)
#define I2C_CONTROL_DIR_CHANGE          (0x1 << 4)
#define I2C_CONTROL_ACKERR_DET_EN       (0x1 << 5)
#define I2C_CONTROL_TRANSFER_LEN_CHANGE (0x1 << 6)
#define I2C_CONTROL_DMAACK_EN           (0x1 << 8)
#define I2C_CONTROL_ASYNC_MODE          (0x1 << 9)
#define I2C_CONTROL_WRAPPER             (0x1 << 0)

enum DMA_REGS_OFFSET {
	OFFSET_INT_FLAG = 0x0,
	OFFSET_INT_EN = 0x04,
	OFFSET_EN = 0x08,
	OFFSET_RST = 0x0c,
	OFFSET_CON = 0x18,
	OFFSET_TX_MEM_ADDR = 0x1c,
	OFFSET_RX_MEM_ADDR = 0x20,
	OFFSET_TX_LEN = 0x24,
	OFFSET_RX_LEN = 0x28,
	OFFSET_TX_4G_MODE = 0x54,
	OFFSET_RX_4G_MODE = 0x58,
};

enum i2c_trans_st_rs {
	I2C_TRANS_STOP = 0,
	I2C_TRANS_REPEATED_START,
};

enum mtk_trans_op {
	I2C_MASTER_WR = 1,
	I2C_MASTER_RD,
	I2C_MASTER_WRRD,
};

enum I2C_REGS_OFFSET {
	OFFSET_DATA_PORT,
	OFFSET_SLAVE_ADDR,
	OFFSET_INTR_MASK,
	OFFSET_INTR_STAT,
	OFFSET_CONTROL,
	OFFSET_TRANSFER_LEN,
	OFFSET_TRANSAC_LEN,
	OFFSET_DELAY_LEN,
	OFFSET_TIMING,
	OFFSET_START,
	OFFSET_EXT_CONF,
	OFFSET_FIFO_STAT,
	OFFSET_FIFO_THRESH,
	OFFSET_FIFO_ADDR_CLR,
	OFFSET_IO_CONFIG,
	OFFSET_RSV_DEBUG,
	OFFSET_HS,
	OFFSET_SOFTRESET,
	OFFSET_DCM_EN,
	OFFSET_MULTI_DMA,
	OFFSET_PATH_DIR,
	OFFSET_DEBUGSTAT,
	OFFSET_DEBUGCTRL,
	OFFSET_TRANSFER_LEN_AUX,
	OFFSET_CLOCK_DIV,
	OFFSET_LTIMING,
	OFFSET_SCL_HIGH_LOW_RATIO,
	OFFSET_HS_SCL_HIGH_LOW_RATIO,
	OFFSET_SCL_MIS_COMP_POINT,
	OFFSET_STA_STO_AC_TIMING,
	OFFSET_HS_STA_STO_AC_TIMING,
	OFFSET_SDA_TIMING,
};

static const u16 mt_i2c_regs_v1[] = {
	[OFFSET_DATA_PORT] = 0x0,
	[OFFSET_SLAVE_ADDR] = 0x4,
	[OFFSET_INTR_MASK] = 0x8,
	[OFFSET_INTR_STAT] = 0xc,
	[OFFSET_CONTROL] = 0x10,
	[OFFSET_TRANSFER_LEN] = 0x14,
	[OFFSET_TRANSAC_LEN] = 0x18,
	[OFFSET_DELAY_LEN] = 0x1c,
	[OFFSET_TIMING] = 0x20,
	[OFFSET_START] = 0x24,
	[OFFSET_EXT_CONF] = 0x28,
	[OFFSET_FIFO_STAT] = 0x30,
	[OFFSET_FIFO_THRESH] = 0x34,
	[OFFSET_FIFO_ADDR_CLR] = 0x38,
	[OFFSET_IO_CONFIG] = 0x40,
	[OFFSET_RSV_DEBUG] = 0x44,
	[OFFSET_HS] = 0x48,
	[OFFSET_SOFTRESET] = 0x50,
	[OFFSET_DCM_EN] = 0x54,
	[OFFSET_PATH_DIR] = 0x60,
	[OFFSET_DEBUGSTAT] = 0x64,
	[OFFSET_DEBUGCTRL] = 0x68,
	[OFFSET_TRANSFER_LEN_AUX] = 0x6c,
	[OFFSET_CLOCK_DIV] = 0x70,
	[OFFSET_SCL_HIGH_LOW_RATIO] = 0x74,
	[OFFSET_HS_SCL_HIGH_LOW_RATIO] = 0x78,
	[OFFSET_SCL_MIS_COMP_POINT] = 0x7C,
	[OFFSET_STA_STO_AC_TIMING] = 0x80,
	[OFFSET_HS_STA_STO_AC_TIMING] = 0x84,
	[OFFSET_SDA_TIMING] = 0x88,
};

static const u16 mt_i2c_regs_v2[] = {
	[OFFSET_DATA_PORT] = 0x0,
	[OFFSET_SLAVE_ADDR] = 0x94,
	[OFFSET_INTR_MASK] = 0x8,
	[OFFSET_INTR_STAT] = 0xc,
	[OFFSET_CONTROL] = 0x10,
	[OFFSET_TRANSFER_LEN] = 0x14,
	[OFFSET_TRANSAC_LEN] = 0x18,
	[OFFSET_DELAY_LEN] = 0x1c,
	[OFFSET_TIMING] = 0x20,
	[OFFSET_START] = 0x24,
	[OFFSET_EXT_CONF] = 0x28,
	[OFFSET_LTIMING] = 0x2c,
	[OFFSET_HS] = 0x30,
	[OFFSET_IO_CONFIG] = 0x34,
	[OFFSET_FIFO_ADDR_CLR] = 0x38,
	[OFFSET_SDA_TIMING] = 0x3c,
	[OFFSET_TRANSFER_LEN_AUX] = 0x44,
	[OFFSET_CLOCK_DIV] = 0x48,
	[OFFSET_SOFTRESET] = 0x50,
	[OFFSET_MULTI_DMA] = 0x8c,
	[OFFSET_SCL_MIS_COMP_POINT] = 0x90,
	[OFFSET_DEBUGSTAT] = 0xe0,
	[OFFSET_DEBUGCTRL] = 0xe8,
	[OFFSET_FIFO_STAT] = 0xf4,
	[OFFSET_FIFO_THRESH] = 0xf8,
	[OFFSET_DCM_EN] = 0xf88,
};

/**
 * struct i2c_adapter_quirks - describe flaws of an i2c adapter
 * @flags: see I2C_AQ_* for possible flags and read below
 * @max_num_msgs: maximum number of messages per transfer
 * @max_write_len: maximum length of a write message
 * @max_read_len: maximum length of a read message
 * @max_comb_1st_msg_len: maximum length of the first msg in a combined message
 * @max_comb_2nd_msg_len: maximum length of the second msg in a combined message
 *
 * Note about combined messages: Some I2C controllers can only send one message
 * per transfer, plus something called combined message or write-then-read.
 * This is (usually) a small write message followed by a read message and
 * barely enough to access register based devices like EEPROMs. There is a flag
 * to support this mode. It implies max_num_msg = 2 and does the length checks
 * with max_comb_*_len because combined message mode usually has its own
 * limitations. Because of HW implementations, some controllers can actually do
 * write-then-anything or other variants. To support that, write-then-read has
 * been broken out into smaller bits like write-first and read-second which can
 * be combined as needed.
 */

struct i2c_adapter_quirks {
	u64 flags;
	int max_num_msgs;
	u16 max_write_len;
	u16 max_read_len;
	u16 max_comb_1st_msg_len;
	u16 max_comb_2nd_msg_len;
};

/**
 * struct i2c_timings - I2C timing information
 * @bus_freq_hz: the bus frequency in Hz
 * @scl_rise_ns: time SCL signal takes to rise in ns; t(r) in the I2C specification
 * @scl_fall_ns: time SCL signal takes to fall in ns; t(f) in the I2C specification
 * @scl_int_delay_ns: time IP core additionally needs to setup SCL in ns
 * @sda_fall_ns: time SDA signal takes to fall in ns; t(f) in the I2C specification
 * @sda_hold_ns: time IP core additionally needs to hold SDA in ns
 */
struct i2c_timings {
	u32 bus_freq_hz;
	u32 scl_rise_ns;
	u32 scl_fall_ns;
	u32 scl_int_delay_ns;
	u32 sda_fall_ns;
	u32 sda_hold_ns;
};

struct mtk_i2c_ac_timing {
	u16 htiming;
	u16 ltiming;
	u16 hs;
	u16 ext;
	u16 inter_clk_div;
	u16 scl_hl_ratio;
	u16 hs_scl_hl_ratio;
	u16 sta_stop;
	u16 hs_sta_stop;
	u16 sda_timing;
};

struct mtk_i2c_compatible {
	const struct i2c_adapter_quirks *quirks;
	const u16 *regs;
	unsigned char pmic_i2c: 1;
	unsigned char dcm: 1;
	unsigned char auto_restart: 1;
	unsigned char aux_len_reg: 1;
	unsigned char timing_adjust: 1;
	unsigned char dma_sync: 1;
	unsigned char ltiming_adjust: 1;
	unsigned char apdma_sync: 1;
	unsigned char send_slave_addr_en: 1;
	unsigned char max_dma_support;
};

struct mtk_i2c {
	struct device *dev;

	struct i2c_timings timing_info;
	/* set in i2c probe */
	void __iomem *base;		/* i2c base addr */
	void __iomem *chn_base;		/* i2c-chn base addr */
	void __iomem *pdmabase;		/* dma base address*/
	struct clk clk_main;		/* main clock for i2c bus */
	struct clk clk_dma;		    /* DMA clock for i2c via DMA */
	u32 clock;
	bool use_push_pull;		/* IO config push-pull mode */
	bool multi_user;
	u16 irq_stat;			/* interrupt status */
	unsigned int clk_src_div;
	unsigned int speed_hz;		/* The speed in transfer */
	enum mtk_trans_op op;
	u16 timing_reg;
	u16 high_speed_reg;
	u16 ltiming_reg;
	unsigned char auto_restart;
	bool ignore_restart_irq;
	struct mtk_i2c_ac_timing ac_timing;
	const struct mtk_i2c_compatible *dev_comp;
	u8 *tx_buff;
	u8 *rx_buff;
};

/**
 * struct i2c_spec_values:
 * @min_low_ns: min LOW period of the SCL clock
 * @min_su_sta_ns: min set-up time for a repeated START condition
 * @max_hd_dat_ns: max data hold time
 * @min_su_dat_ns: min data set-up time
 */
struct i2c_spec_values {
	unsigned int min_low_ns;
	unsigned int min_su_sta_ns;
	unsigned int max_hd_dat_ns;
	unsigned int min_su_dat_ns;
};

static const struct i2c_spec_values standard_mode_spec = {
	.min_low_ns = 4700 + I2C_STANDARD_MODE_BUFFER,
	.min_su_sta_ns = 4700 + I2C_STANDARD_MODE_BUFFER,
	.max_hd_dat_ns = 3450 - I2C_STANDARD_MODE_BUFFER,
	.min_su_dat_ns = 250 + I2C_STANDARD_MODE_BUFFER,
};

static const struct i2c_spec_values fast_mode_spec = {
	.min_low_ns = 1300 + I2C_FAST_MODE_BUFFER,
	.min_su_sta_ns = 600 + I2C_FAST_MODE_BUFFER,
	.max_hd_dat_ns = 900 - I2C_FAST_MODE_BUFFER,
	.min_su_dat_ns = 100 + I2C_FAST_MODE_BUFFER,
};

static const struct i2c_spec_values fast_mode_plus_spec = {
	.min_low_ns = 500 + I2C_FAST_MODE_PLUS_BUFFER,
	.min_su_sta_ns = 260 + I2C_FAST_MODE_PLUS_BUFFER,
	.max_hd_dat_ns = 400 - I2C_FAST_MODE_PLUS_BUFFER,
	.min_su_dat_ns = 50 + I2C_FAST_MODE_PLUS_BUFFER,
};

static void mtk_i2c_writew(struct mtk_i2c *i2c, u16 val,
			   enum I2C_REGS_OFFSET reg)
{
	writel(val, i2c->base + i2c->dev_comp->regs[reg]);
}

static u16 mtk_i2c_readw(struct mtk_i2c *i2c, enum I2C_REGS_OFFSET reg)
{
	return readl(i2c->base + i2c->dev_comp->regs[reg]);
}

static void mtk_i2c_clk_enable(struct mtk_i2c *i2c)
{
	clk_enable(&i2c->clk_main);
	clk_enable(&i2c->clk_dma);
}

static void mtk_i2c_clk_disable(struct mtk_i2c *i2c)
{
	clk_disable(&i2c->clk_dma);
	clk_disable(&i2c->clk_main);
}

static void mtk_i2c_init_hw(struct mtk_i2c *i2c)
{
	u16 control_reg;
	u16 intr_stat_reg;

	mtk_i2c_writew(i2c, I2C_CHN_CLR_FLAG, OFFSET_START);
	intr_stat_reg = mtk_i2c_readw(i2c, OFFSET_INTR_STAT);
	mtk_i2c_writew(i2c, intr_stat_reg, OFFSET_INTR_STAT);

	if (i2c->dev_comp->apdma_sync) {
		writel(I2C_DMA_WARM_RST, i2c->pdmabase + OFFSET_RST);
		udelay(10);
		writel(I2C_DMA_CLR_FLAG, i2c->pdmabase + OFFSET_RST);
		udelay(10);
		writel(I2C_DMA_HANDSHAKE_RST | I2C_DMA_HARD_RST,
		       i2c->pdmabase + OFFSET_RST);
		mtk_i2c_writew(i2c, I2C_HANDSHAKE_RST | I2C_SOFT_RST,
			       OFFSET_SOFTRESET);
		udelay(10);
		writel(I2C_DMA_CLR_FLAG, i2c->pdmabase + OFFSET_RST);
		mtk_i2c_writew(i2c, I2C_CHN_CLR_FLAG, OFFSET_SOFTRESET);
	} else {
		writel(I2C_DMA_HARD_RST, i2c->pdmabase + OFFSET_RST);
		udelay(50);
		writel(I2C_DMA_CLR_FLAG, i2c->pdmabase + OFFSET_RST);
		mtk_i2c_writew(i2c, I2C_SOFT_RST, OFFSET_SOFTRESET);
	}
	/* Set ioconfig */
	if (i2c->use_push_pull)
		mtk_i2c_writew(i2c, I2C_IO_CONFIG_PUSH_PULL, OFFSET_IO_CONFIG);
	else
		mtk_i2c_writew(i2c, I2C_IO_CONFIG_OPEN_DRAIN, OFFSET_IO_CONFIG);

	if (i2c->dev_comp->dcm && !i2c->multi_user)
		mtk_i2c_writew(i2c, I2C_DCM_DISABLE, OFFSET_DCM_EN);

	mtk_i2c_writew(i2c, i2c->timing_reg, OFFSET_TIMING);
	mtk_i2c_writew(i2c, i2c->high_speed_reg, OFFSET_HS);
	if (i2c->dev_comp->ltiming_adjust)
		mtk_i2c_writew(i2c, i2c->ltiming_reg, OFFSET_LTIMING);

	if (i2c->dev_comp->timing_adjust) {
		mtk_i2c_writew(i2c, i2c->ac_timing.ext, OFFSET_EXT_CONF);
		mtk_i2c_writew(i2c, i2c->ac_timing.inter_clk_div,
			       OFFSET_CLOCK_DIV);
		mtk_i2c_writew(i2c, I2C_SCL_MIS_COMP_VALUE,
			       OFFSET_SCL_MIS_COMP_POINT);
		mtk_i2c_writew(i2c, i2c->ac_timing.sda_timing,
			       OFFSET_SDA_TIMING);

		if (i2c->dev_comp->ltiming_adjust) {
			mtk_i2c_writew(i2c, i2c->ac_timing.htiming,
				       OFFSET_TIMING);
			mtk_i2c_writew(i2c, i2c->ac_timing.hs, OFFSET_HS);
			mtk_i2c_writew(i2c, i2c->ac_timing.ltiming,
				       OFFSET_LTIMING);
		} else {
			mtk_i2c_writew(i2c, i2c->ac_timing.scl_hl_ratio,
				       OFFSET_SCL_HIGH_LOW_RATIO);
			mtk_i2c_writew(i2c, i2c->ac_timing.hs_scl_hl_ratio,
				       OFFSET_HS_SCL_HIGH_LOW_RATIO);
			mtk_i2c_writew(i2c, i2c->ac_timing.sta_stop,
				       OFFSET_STA_STO_AC_TIMING);
			mtk_i2c_writew(i2c, i2c->ac_timing.hs_sta_stop,
				       OFFSET_HS_STA_STO_AC_TIMING);
		}
	}

	control_reg = I2C_CONTROL_ACKERR_DET_EN |
		      I2C_CONTROL_CLK_EXT_EN | I2C_CONTROL_DMA_EN;
	if (i2c->dev_comp->dma_sync)
		control_reg |= I2C_CONTROL_DMAACK_EN | I2C_CONTROL_ASYNC_MODE;

	mtk_i2c_writew(i2c, control_reg, OFFSET_CONTROL);
	mtk_i2c_writew(i2c, I2C_DELAY_LEN, OFFSET_DELAY_LEN);
}

static const struct i2c_spec_values *mtk_i2c_get_spec(unsigned int speed)
{
	if (speed <= I2C_MAX_STANDARD_MODE_FREQ)
		return &standard_mode_spec;
	else if (speed <= I2C_MAX_FAST_MODE_FREQ)
		return &fast_mode_spec;
	else
		return &fast_mode_plus_spec;
}

static int mtk_i2c_max_step_cnt(unsigned int target_speed)
{
	if (target_speed > I2C_MAX_FAST_MODE_PLUS_FREQ)
		return MAX_HS_STEP_CNT_DIV;
	else
		return MAX_STEP_CNT_DIV;
}

/*
 * Check and Calculate i2c ac-timing
 *
 * Hardware design:
 * sample_ns = (1000000000 * (sample_cnt + 1)) / clk_src
 * xxx_cnt_div =  spec->min_xxx_ns / sample_ns
 *
 * Sample_ns is rounded down for xxx_cnt_div would be greater
 * than the smallest spec.
 * The sda_timing is chosen as the middle value between
 * the largest and smallest.
 */
static int mtk_i2c_check_ac_timing(struct mtk_i2c *i2c,
				   unsigned int clk_src,
				   unsigned int check_speed,
				   unsigned int step_cnt,
				   unsigned int sample_cnt)
{
	const struct i2c_spec_values *spec;
	unsigned int su_sta_cnt, low_cnt, high_cnt, max_step_cnt;
	unsigned int sda_max, sda_min, clk_ns, max_sta_cnt = 0x3f;
	unsigned int sample_ns = div_u64(1000000000ULL * (sample_cnt + 1),
					 clk_src);

	if (!i2c->dev_comp->timing_adjust)
		return 0;

	if (i2c->dev_comp->ltiming_adjust)
		max_sta_cnt = 0x100;

	spec = mtk_i2c_get_spec(check_speed);

	if (i2c->dev_comp->ltiming_adjust)
		clk_ns = 1000000000 / clk_src;
	else
		clk_ns = sample_ns / 2;

	su_sta_cnt = DIV_ROUND_UP(spec->min_su_sta_ns +
				  i2c->timing_info.scl_int_delay_ns, clk_ns);
	if (su_sta_cnt > max_sta_cnt)
		return -1;

	low_cnt = DIV_ROUND_UP(spec->min_low_ns, sample_ns);
	max_step_cnt = mtk_i2c_max_step_cnt(check_speed);
	if ((2 * step_cnt) > low_cnt && low_cnt < max_step_cnt) {
		if (low_cnt > step_cnt) {
			high_cnt = 2 * step_cnt - low_cnt;
		} else {
			high_cnt = step_cnt;
			low_cnt = step_cnt;
		}
	} else {
		return -2;
	}

	sda_max = spec->max_hd_dat_ns / sample_ns;
	if (sda_max > low_cnt)
		sda_max = 0;

	sda_min = DIV_ROUND_UP(spec->min_su_dat_ns, sample_ns);
	if (sda_min < low_cnt)
		sda_min = 0;

	if (sda_min > sda_max)
		return -3;

	if (check_speed > I2C_MAX_FAST_MODE_PLUS_FREQ) {
		if (i2c->dev_comp->ltiming_adjust) {
			i2c->ac_timing.hs = I2C_TIME_DEFAULT_VALUE |
				(sample_cnt << 12) | (high_cnt << 8);
			i2c->ac_timing.ltiming &= ~GENMASK(15, 9);
			i2c->ac_timing.ltiming |= (sample_cnt << 12) |
				(low_cnt << 9);
			i2c->ac_timing.ext &= ~GENMASK(7, 1);
			i2c->ac_timing.ext |= (su_sta_cnt << 1) | (1 << 0);
		} else {
			i2c->ac_timing.hs_scl_hl_ratio = (1 << 12) |
				(high_cnt << 6) | low_cnt;
			i2c->ac_timing.hs_sta_stop = (su_sta_cnt << 8) |
				su_sta_cnt;
		}
		i2c->ac_timing.sda_timing &= ~GENMASK(11, 6);
		i2c->ac_timing.sda_timing |= (1 << 12) |
			((sda_max + sda_min) / 2) << 6;
	} else {
		if (i2c->dev_comp->ltiming_adjust) {
			i2c->ac_timing.htiming = (sample_cnt << 8) | (high_cnt);
			i2c->ac_timing.ltiming = (sample_cnt << 6) | (low_cnt);
			i2c->ac_timing.ext = (su_sta_cnt << 8) | (1 << 0);
		} else {
			i2c->ac_timing.scl_hl_ratio = (1 << 12) |
				(high_cnt << 6) | low_cnt;
			i2c->ac_timing.sta_stop = (su_sta_cnt << 8) |
				su_sta_cnt;
		}

		i2c->ac_timing.sda_timing = (1 << 12) |
			(sda_max + sda_min) / 2;
	}

	return 0;
}

/*
 * Calculate i2c port speed
 *
 * Hardware design:
 * i2c_bus_freq = parent_clk / (clock_div * 2 * sample_cnt * step_cnt)
 * clock_div: fixed in hardware, but may be various in different SoCs
 *
 * The calculation want to pick the highest bus frequency that is still
 * less than or equal to i2c->speed_hz. The calculation try to get
 * sample_cnt and step_cn
 */
static int mtk_i2c_calculate_speed(struct mtk_i2c *i2c, unsigned int clk_src,
				   unsigned int target_speed,
				   unsigned int *timing_step_cnt,
				   unsigned int *timing_sample_cnt)
{
	unsigned int step_cnt;
	unsigned int sample_cnt;
	unsigned int max_step_cnt;
	unsigned int base_sample_cnt = MAX_SAMPLE_CNT_DIV;
	unsigned int base_step_cnt;
	unsigned int opt_div;
	unsigned int best_mul;
	unsigned int cnt_mul;
	int ret = -EINVAL;

	if (target_speed > I2C_MAX_HIGH_SPEED_MODE_FREQ)
		target_speed = I2C_MAX_HIGH_SPEED_MODE_FREQ;

	max_step_cnt = mtk_i2c_max_step_cnt(target_speed);
	base_step_cnt = max_step_cnt;
	/* Find the best combination */
	opt_div = DIV_ROUND_UP(clk_src >> 1, target_speed);
	best_mul = MAX_SAMPLE_CNT_DIV * max_step_cnt;

	/* Search for the best pair (sample_cnt, step_cnt) with
	 * 0 < sample_cnt < MAX_SAMPLE_CNT_DIV
	 * 0 < step_cnt < max_step_cnt
	 * sample_cnt * step_cnt >= opt_div
	 * optimizing for sample_cnt * step_cnt being minimal
	 */
	for (sample_cnt = 1; sample_cnt <= MAX_SAMPLE_CNT_DIV; sample_cnt++) {
		step_cnt = DIV_ROUND_UP(opt_div, sample_cnt);
		cnt_mul = step_cnt * sample_cnt;
		if (step_cnt > max_step_cnt)
			continue;
		if (cnt_mul < best_mul) {
			ret = mtk_i2c_check_ac_timing(i2c, clk_src,
				target_speed, step_cnt - 1, sample_cnt - 1);
			if (ret)
				continue;

			best_mul = cnt_mul;
			base_sample_cnt = sample_cnt;
			base_step_cnt = step_cnt;
			if (best_mul == opt_div)
				break;
		}
	}

	if (ret)
		return -EINVAL;

	sample_cnt = base_sample_cnt;
	step_cnt = base_step_cnt;

	if ((clk_src / (2 * sample_cnt * step_cnt)) > target_speed) {
		/* In this case, hardware can't support such
		 * low i2c_bus_freq
		 */
		dev_err(i2c->dev, "Unsupported speed (%uhz)\n",	target_speed);
		return -EINVAL;
	}

	*timing_step_cnt = step_cnt - 1;
	*timing_sample_cnt = sample_cnt - 1;

	return 0;
}

static int mtk_i2c_set_speed(struct udevice *dev, unsigned int speed)
{
	unsigned int clk_src;
	unsigned int step_cnt;
	unsigned int sample_cnt;
	unsigned int l_step_cnt;
	unsigned int l_sample_cnt;
	unsigned int target_speed;
	unsigned int clk_div;
	unsigned int max_clk_div;
	unsigned int parent_clk;
	int ret;

	struct mtk_i2c *i2c = dev_get_priv(dev);

	i2c->speed_hz = speed;
	target_speed = i2c->speed_hz;

	parent_clk = i2c->clock / i2c->clk_src_div;
	if (i2c->dev_comp->timing_adjust)
		max_clk_div = MAX_CLOCK_DIV;
	else
		max_clk_div = 1;

	for (clk_div = 1; clk_div <= max_clk_div; clk_div++) {
		clk_src = parent_clk / clk_div;

		if (target_speed > I2C_MAX_FAST_MODE_PLUS_FREQ) {
			/* Set master code speed register */
			ret = mtk_i2c_calculate_speed(i2c, clk_src,
						      I2C_MAX_FAST_MODE_FREQ,
						      &l_step_cnt,
						      &l_sample_cnt);
			if (ret < 0)
				continue;

			i2c->timing_reg = (l_sample_cnt << 8) | l_step_cnt;

			/* Set the high speed mode register */
			ret = mtk_i2c_calculate_speed(i2c, clk_src,
						      target_speed, &step_cnt,
						      &sample_cnt);
			if (ret < 0)
				continue;

			i2c->high_speed_reg = I2C_TIME_DEFAULT_VALUE |
					(sample_cnt << 12) | (step_cnt << 8);

			if (i2c->dev_comp->ltiming_adjust)
				i2c->ltiming_reg =
					(l_sample_cnt << 6) | l_step_cnt |
					(sample_cnt << 12) | (step_cnt << 9);
		} else {
			ret = mtk_i2c_calculate_speed(i2c, clk_src,
						      target_speed, &l_step_cnt,
						      &l_sample_cnt);
			if (ret < 0)
				continue;

			i2c->timing_reg = (l_sample_cnt << 8) | l_step_cnt;

			/* Disable the high speed transaction */
			i2c->high_speed_reg = I2C_TIME_CLR_VALUE;

			if (i2c->dev_comp->ltiming_adjust)
				i2c->ltiming_reg =
					(l_sample_cnt << 6) | l_step_cnt;
		}

		break;
	}

	i2c->ac_timing.inter_clk_div = clk_div - 1;
	return 0;
}

static void i2c_dump_register(struct mtk_i2c *i2c)
{
	dev_err(i2c->dev, "i2c->base:0x%x, i2c->dma:0x%x\n",i2c->base, i2c->pdmabase);
	dev_err(i2c->dev, "SLAVE_ADDR: 0x%x, INTR_MASK: 0x%x\n",
		mtk_i2c_readw(i2c, OFFSET_SLAVE_ADDR),
		mtk_i2c_readw(i2c, OFFSET_INTR_MASK));
	dev_err(i2c->dev, "INTR_STAT: 0x%x, CONTROL: 0x%x\n",
		mtk_i2c_readw(i2c, OFFSET_INTR_STAT),
		mtk_i2c_readw(i2c, OFFSET_CONTROL));
	dev_err(i2c->dev, "TRANSFER_LEN: 0x%x, TRANSAC_LEN: 0x%x\n",
		mtk_i2c_readw(i2c, OFFSET_TRANSFER_LEN),
		mtk_i2c_readw(i2c, OFFSET_TRANSAC_LEN));
	dev_err(i2c->dev, "DELAY_LEN: 0x%x, HTIMING: 0x%x\n",
		mtk_i2c_readw(i2c, OFFSET_DELAY_LEN),
		mtk_i2c_readw(i2c, OFFSET_TIMING));
	dev_err(i2c->dev, "START: 0x%x, EXT_CONF: 0x%x\n",
		mtk_i2c_readw(i2c, OFFSET_START),
		mtk_i2c_readw(i2c, OFFSET_EXT_CONF));
	dev_err(i2c->dev, "HS: 0x%x, IO_CONFIG: 0x%x\n",
		mtk_i2c_readw(i2c, OFFSET_HS),
		mtk_i2c_readw(i2c, OFFSET_IO_CONFIG));
	dev_err(i2c->dev, "TRANSFER_LEN_AUX: 0x%x\n",
		mtk_i2c_readw(i2c, OFFSET_TRANSFER_LEN_AUX));
	dev_err(i2c->dev, "CLOCK_DIV: 0x%x, FIFO_STAT: 0x%x\n",
		mtk_i2c_readw(i2c, OFFSET_CLOCK_DIV),
		mtk_i2c_readw(i2c, OFFSET_FIFO_STAT));
	dev_err(i2c->dev, "DEBUGCTRL : 0x%x, DEBUGSTAT: 0x%x\n",
		mtk_i2c_readw(i2c, OFFSET_DEBUGCTRL),
		mtk_i2c_readw(i2c, OFFSET_DEBUGSTAT));
	if (i2c->dev_comp->regs == mt_i2c_regs_v2) {
		dev_err(i2c->dev, "LTIMING: 0x%x, MULTI_DMA: 0x%x\n",
			mtk_i2c_readw(i2c, OFFSET_LTIMING),
			mtk_i2c_readw(i2c, OFFSET_MULTI_DMA));
	}
	dev_err(i2c->dev, "\nDMA_INT_FLAG: 0x%x, DMA_INT_EN: 0x%x\n",
		readl(i2c->pdmabase + OFFSET_INT_FLAG),
		readl(i2c->pdmabase + OFFSET_INT_EN));
	dev_err(i2c->dev, "DMA_EN: 0x%x, DMA_CON: 0x%x\n",
		readl(i2c->pdmabase + OFFSET_EN),
		readl(i2c->pdmabase + OFFSET_CON));
	dev_err(i2c->dev, "DMA_TX_MEM_ADDR: 0x%x, DMA_RX_MEM_ADDR: 0x%x\n",
		readl(i2c->pdmabase + OFFSET_TX_MEM_ADDR),
		readl(i2c->pdmabase + OFFSET_RX_MEM_ADDR));
	dev_err(i2c->dev, "DMA_TX_LEN: 0x%x, DMA_RX_LEN: 0x%x\n",
		readl(i2c->pdmabase + OFFSET_TX_LEN),
		readl(i2c->pdmabase + OFFSET_RX_LEN));
	dev_err(i2c->dev, "DMA_TX_4G_MODE: 0x%x, DMA_RX_4G_MODE: 0x%x\n",
		readl(i2c->pdmabase + OFFSET_TX_4G_MODE),
		readl(i2c->pdmabase + OFFSET_RX_4G_MODE));
}

static int mtk_i2c_do_transfer(struct mtk_i2c *i2c, struct i2c_msg *msgs,
			       int num, int left_num)
{
	u16 addr_reg;
	u16 start_reg;
	u16 control_reg;
	u16 restart_flag = 0;
	u16 dma_sync = 0;
	u32 reg_4g_mode;
	dma_addr_t rpaddr = 0;
	dma_addr_t wpaddr = 0;
	u16 tmo_poll = 200000;
	int ret = 1;

	i2c->irq_stat = 0;

	if (i2c->auto_restart)
		restart_flag = I2C_RS_TRANSFER;

	//reinit_completion(&i2c->msg_complete);

	control_reg = mtk_i2c_readw(i2c, OFFSET_CONTROL) &
			~(I2C_CONTROL_DIR_CHANGE | I2C_CONTROL_RS);
	if ((i2c->speed_hz > I2C_MAX_FAST_MODE_PLUS_FREQ) || (left_num >= 1))
		control_reg |= I2C_CONTROL_RS;

	if (i2c->op == I2C_MASTER_WRRD)
		control_reg |= I2C_CONTROL_DIR_CHANGE | I2C_CONTROL_RS;

	mtk_i2c_writew(i2c, control_reg, OFFSET_CONTROL);

	/*Set slave address*/
	addr_reg = (i2c->op == I2C_MASTER_RD) ? ((msgs->addr << 1) | 0x1) : (msgs->addr << 1);
	if ((msgs->buf == NULL) && i2c->dev_comp->send_slave_addr_en)
		mtk_i2c_writew(i2c, addr_reg | I2C_TRANS_ADDR_ONLY, OFFSET_SLAVE_ADDR);
	else
		mtk_i2c_writew(i2c, addr_reg, OFFSET_SLAVE_ADDR);

	/* Clear interrupt status */
	mtk_i2c_writew(i2c, restart_flag | I2C_HS_NACKERR | I2C_ACKERR |
			    I2C_ARB_LOST | I2C_TRANSAC_COMP, OFFSET_INTR_STAT);

	if (i2c->multi_user)
		mtk_i2c_writew(i2c, I2C_FIFO_ADDR_CLR_MCH, OFFSET_FIFO_ADDR_CLR);
	else
		mtk_i2c_writew(i2c, I2C_FIFO_ADDR_CLR, OFFSET_FIFO_ADDR_CLR);

	/* Enable interrupt */
	mtk_i2c_writew(i2c, restart_flag | I2C_HS_NACKERR | I2C_ACKERR |
			    I2C_ARB_LOST | I2C_TRANSAC_COMP, OFFSET_INTR_MASK);

	/* Set transfer and transaction len */
	if (i2c->op == I2C_MASTER_WRRD) {
		if (i2c->dev_comp->aux_len_reg) {
			mtk_i2c_writew(i2c, msgs->len, OFFSET_TRANSFER_LEN);
			mtk_i2c_writew(i2c, (msgs + 1)->len,
					    OFFSET_TRANSFER_LEN_AUX);
		} else {
			mtk_i2c_writew(i2c, msgs->len | ((msgs + 1)->len) << 8,
					    OFFSET_TRANSFER_LEN);
		}
		mtk_i2c_writew(i2c, I2C_WRRD_TRANAC_VALUE, OFFSET_TRANSAC_LEN);
	} else {
		mtk_i2c_writew(i2c, msgs->len, OFFSET_TRANSFER_LEN);
		mtk_i2c_writew(i2c, num, OFFSET_TRANSAC_LEN);
	}

	if (i2c->dev_comp->apdma_sync) {
		dma_sync = I2C_DMA_SKIP_CONFIG | I2C_DMA_ASYNC_MODE;
		if (i2c->op == I2C_MASTER_WRRD)
			dma_sync |= I2C_DMA_DIR_CHANGE;
	}

	/* Prepare buffer data to start transfer */
	if (i2c->op == I2C_MASTER_RD) {
		writel(I2C_DMA_INT_FLAG_NONE, i2c->pdmabase + OFFSET_INT_FLAG);
		writel(I2C_DMA_CON_RX | dma_sync, i2c->pdmabase + OFFSET_CON);

		i2c->rx_buff = (u8 *)noncached_alloc(msgs->len,
				ARCH_DMA_MINALIGN);
		writel((u32)i2c->rx_buff, i2c->pdmabase +
		OFFSET_RX_MEM_ADDR);
		writel(msgs->len, i2c->pdmabase + OFFSET_RX_LEN);
		if (i2c->dev_comp->max_dma_support > 32) {
			reg_4g_mode = upper_32_bits(rpaddr);
			writel(reg_4g_mode, i2c->pdmabase + OFFSET_RX_4G_MODE);
		}
	} else if (i2c->op == I2C_MASTER_WR) {
		writel(I2C_DMA_INT_FLAG_NONE, i2c->pdmabase + OFFSET_INT_FLAG);
		writel(I2C_DMA_CON_TX | dma_sync, i2c->pdmabase + OFFSET_CON);
		i2c->tx_buff = (u8 *)noncached_alloc(msgs->len,
				ARCH_DMA_MINALIGN);
		memcpy(i2c->tx_buff, msgs->buf, msgs->len);
		writel((u32)i2c->tx_buff, i2c->pdmabase +
			  OFFSET_TX_MEM_ADDR);
		writel(msgs->len, i2c->pdmabase + OFFSET_TX_LEN);

		if (i2c->dev_comp->max_dma_support > 32) {
			reg_4g_mode = upper_32_bits(wpaddr);
			writel(reg_4g_mode, i2c->pdmabase + OFFSET_TX_4G_MODE);
		}
	} else {
		writel(I2C_DMA_CLR_FLAG, i2c->pdmabase + OFFSET_INT_FLAG);
		writel(I2C_DMA_CLR_FLAG | dma_sync, i2c->pdmabase + OFFSET_CON);
		i2c->tx_buff = (u8 *)noncached_alloc(msgs->len,
				ARCH_DMA_MINALIGN);
		i2c->rx_buff = (u8 *)noncached_alloc((msgs + 1)->len,
				ARCH_DMA_MINALIGN);
		memcpy(i2c->tx_buff, msgs->buf, msgs->len);
		writel((u32)i2c->tx_buff, i2c->pdmabase +
			  OFFSET_TX_MEM_ADDR);
		writel((u32)i2c->rx_buff, i2c->pdmabase +
			  OFFSET_RX_MEM_ADDR);
		writel(msgs->len, i2c->pdmabase + OFFSET_TX_LEN);
		writel((msgs + 1)->len, i2c->pdmabase +
			  OFFSET_RX_LEN);
		if (i2c->dev_comp->max_dma_support > 32) {
			reg_4g_mode = upper_32_bits(wpaddr);
			writel(reg_4g_mode, i2c->pdmabase + OFFSET_TX_4G_MODE);

			reg_4g_mode = upper_32_bits(rpaddr);
			writel(reg_4g_mode, i2c->pdmabase + OFFSET_RX_4G_MODE);
		}
	}

	writel(I2C_DMA_START_EN, i2c->pdmabase + OFFSET_EN);

	if (!i2c->auto_restart) {
		start_reg = I2C_TRANSAC_START;
	} else {
		start_reg = I2C_TRANSAC_START | I2C_RS_MUL_TRIG;
		if (left_num >= 1)
			start_reg |= I2C_RS_MUL_CNFG;
	}
	mtk_i2c_writew(i2c, start_reg, OFFSET_START);

	// ret = wait_for_completion_timeout(&i2c->msg_complete,
					  // i2c->adap.timeout);
    /*master read && poll mode*/
	for (;;){
		/*check the interrupt status register*/
		i2c->irq_stat = mtk_i2c_readw(i2c, OFFSET_INTR_STAT);
		if(i2c->irq_stat & (I2C_HS_NACKERR | I2C_ACKERR )) {
            break;
        }else if(i2c->irq_stat &  I2C_TRANSAC_COMP) {
            ret = 1;
            break;
        }
		udelay(10);
        tmo_poll --;
        if(tmo_poll == 0) {
            ret = 0;
            break;
        }
    }
	/* Clear interrupt mask */
	mtk_i2c_writew(i2c, ~(restart_flag | I2C_HS_NACKERR | I2C_ACKERR |
			    I2C_ARB_LOST | I2C_TRANSAC_COMP), OFFSET_INTR_MASK);

	if (i2c->op == I2C_MASTER_WR) {

	} else if (i2c->op == I2C_MASTER_RD) {
		memcpy(msgs->buf, i2c->rx_buff, msgs->len);
	} else {
		memcpy((msgs + 1)->buf, i2c->rx_buff, (msgs + 1)->len);
	}

	if (ret == 0) {
		dev_err(i2c->dev, "addr: %x, transfer timeout\n", msgs->addr);
		i2c_dump_register(i2c);
		mtk_i2c_init_hw(i2c);
		writel(0x5, i2c->chn_base +i2c->dev_comp->regs[OFFSET_SOFTRESET]);
		return -ETIMEDOUT;
	}

	if (i2c->irq_stat & (I2C_HS_NACKERR | I2C_ACKERR)) {
		dev_dbg(i2c->dev, "addr: %x, transfer ACK error\n", msgs->addr);
		mtk_i2c_init_hw(i2c);
		return -ENXIO;
	}
	return 0;
}

static int mtk_i2c_transfer(struct udevice *dev, struct i2c_msg msgs[], int num)
{
	int ret;
	int left_num = num;
	struct mtk_i2c *i2c = dev_get_priv(dev);

	mtk_i2c_clk_enable(i2c);

	mtk_i2c_init_hw(i2c);
	i2c->auto_restart = i2c->dev_comp->auto_restart;

	/* checking if we can skip restart and optimize using WRRD mode */
	if (i2c->auto_restart && num == 2) {
		if (!(msgs[0].flags & I2C_M_RD) && (msgs[1].flags & I2C_M_RD) &&
		    msgs[0].addr == msgs[1].addr) {
			i2c->auto_restart = 0;
		}
	}

	if (i2c->auto_restart && num >= 2 &&
		i2c->speed_hz > I2C_MAX_FAST_MODE_PLUS_FREQ)
		/* ignore the first restart irq after the master code,
		 * otherwise the first transfer will be discarded.
		 */
		i2c->ignore_restart_irq = true;
	else
		i2c->ignore_restart_irq = false;

	while (left_num--) {
		if (msgs->flags & I2C_M_RD)
			i2c->op = I2C_MASTER_RD;
		else
			i2c->op = I2C_MASTER_WR;

		if (!i2c->auto_restart) {
			if (num > 1) {
				/* combined two messages into one transaction */
				i2c->op = I2C_MASTER_WRRD;
				left_num--;
			}
		}

		/* always use DMA mode. */
		ret = mtk_i2c_do_transfer(i2c, msgs, num, left_num);
		if (ret < 0)
			goto err_exit;

		msgs++;
	}

	mtk_i2c_clk_disable(i2c);

err_exit:
	return ret;
}

static int mtk_i2c_ofdata_to_platdata(struct udevice *dev)
{
	int val;
	struct mtk_i2c *i2c = dev_get_priv(dev);
	struct clk clk;
	int node;

	i2c->chn_base = dev_remap_addr_index(dev, 0);
	i2c->pdmabase = dev_remap_addr_index(dev, 1);

	node = dev_of_offset(dev);
	if (node < 0)
		dev_err(i2c->dev, "%s: get node from fdt fail\n", __func__);

	i2c->multi_user = fdtdec_get_bool(gd->fdt_blob, node, "mediatek,enable-multi-user");
	if (i2c->multi_user)
		i2c->base = i2c->chn_base + 0x100;
	else
		i2c->base = i2c->chn_base;

	val = clk_get_by_index(dev, 0, &i2c->clk_main);;
	if (val == 0) {
		i2c->clock = clk_get_rate(&i2c->clk_main);
	    if (!i2c->clock)
		   return -EINVAL;
	} else {
		dev_err(i2c->dev, "mtk i2c: Get clock rate failed \n");
		return -EINVAL;
	}

	val = clk_get_by_index(dev, 1, &i2c->clk_dma);
	if (val) {
		dev_err(i2c->dev, "Get apdma clock failed\n");
		return val;
	}

	val = dev_read_u32_default(dev, "clock-div", -1);
	if(val != -1)
	    i2c->clk_src_div = val;
	else
		i2c->clk_src_div = 1;

	return 0;
}

static int mtk_i2c_probe(struct udevice *dev)
{
	struct mtk_i2c *i2c = dev_get_priv(dev);

	i2c->dev_comp = (struct mtk_i2c_compatible *)dev_get_driver_data(dev);
	return 0;
}

static int mtk_i2c_deblock(struct udevice *dev)
{
	struct mtk_i2c *i2c = dev_get_priv(dev);

	return 0;
}

static const struct mtk_i2c_compatible mt8518_compat = {
	.regs = mt_i2c_regs_v1,
	.pmic_i2c = 0,
	.dcm = 0,
	.auto_restart = 1,
	.aux_len_reg = 1,
	.timing_adjust = 1,
	.dma_sync = 0,
	.ltiming_adjust = 0,
	.apdma_sync = 0,
	.send_slave_addr_en = 1,
	.max_dma_support = 33,
};

static const struct mtk_i2c_compatible mt8512_compat = {
	.regs = mt_i2c_regs_v1,
	.pmic_i2c = 0,
	.dcm = 0,
	.auto_restart = 1,
	.aux_len_reg = 1,
	.timing_adjust = 1,
	.dma_sync = 1,
	.ltiming_adjust = 0,
	.apdma_sync = 0,
	.send_slave_addr_en = 1,
	.max_dma_support = 33,
};

static const struct mtk_i2c_compatible mt8519_compat = {
	.regs = mt_i2c_regs_v2,
	.pmic_i2c = 0,
	.dcm = 0,
	.auto_restart = 1,
	.aux_len_reg = 1,
	.timing_adjust = 1,
	.dma_sync = 0,
	.ltiming_adjust = 1,
	.apdma_sync = 0,
	.send_slave_addr_en = 0,
	.max_dma_support = 36,
};

static const struct dm_i2c_ops mtk_i2c_ops = {
	.xfer		= mtk_i2c_transfer,
	.set_bus_speed	= mtk_i2c_set_speed,
	.deblock	= mtk_i2c_deblock,
};

static const struct udevice_id mtk_i2c_ids[] = {
	{
		.compatible = "mediatek,mt8518-i2c",
		.data = (ulong)&mt8518_compat,
	},
	{
		.compatible = "mediatek,mt8512-i2c",
		.data = (ulong)&mt8512_compat,
	},
	{
		.compatible = "mediatek,mt8519-i2c",
		.data = (ulong)&mt8519_compat,
	},
	{
	}
};

U_BOOT_DRIVER(mtk_uboot_i2c) = {
	.name		= "mtk_uboot_i2c",
	.id		= UCLASS_I2C,
	.of_match	= mtk_i2c_ids,
	.ofdata_to_platdata = mtk_i2c_ofdata_to_platdata,
	.probe		= mtk_i2c_probe,
	.priv_auto_alloc_size = sizeof(struct mtk_i2c),
	.ops		= &mtk_i2c_ops,
};
