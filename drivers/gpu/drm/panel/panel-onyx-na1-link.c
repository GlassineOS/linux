// SPDX-License-Identifier: GPL-2.0-only
/*
 * Onyx Boox Note Air (NA1) e-ink wire link.
 *
 * The NA1 e-ink panel is driven as a software TCON: every drive frame is
 * computed by the CPU and sent as a raw 285x1423 RGB888 frame over DSI
 * (command mode, 2 lanes) to a TC358762 bridge, whose 24 parallel outputs
 * carry 16 source-data bits (8 px x 2-bit phase) and the panel control
 * signals (GDCLK/GDSP/SDSP/SDOE/GDOE/SDLE, bits 17-22). This "panel" presents
 * that wire geometry to the DSI host; one DRM commit = one frame on the wire.
 * The bridge cannot re-scan a frame (1024x24 FIFO, no frame memory).
 *
 * It also sequences, exactly as the stock Onyx 4.4 drivers do:
 *  - the TC358762 (I2C 0x0b): power, wake (0x047c = 0), stock init table
 *    tc358762_reg_1872x1404_es103_90hz, MD0 (gpio "mode0") = 1 while frames
 *    stream (stock tc358762_output_enable);
 *  - the MAX17135 e-ink PMIC (I2C 0x48) high-voltage rails.
 *
 * MAX17135 safety rules (VCOM is factory-calibrated in the PMIC's NVM):
 *  - the ONLY register this driver ever writes is ENABLE 0x09, and only its
 *    bits 0 (EN: display rails) and 1 (VCOM_EN); there is no other PMIC write
 *    path in this file. Never 0x08 (VCOM DVR) or 0x0C (PROGRAM -> NVM).
 *  - before enabling, the PMIC must identify as the stock part, VCOM must read
 *    0x6c (-1.58 V, stock) and the timing registers must equal the stock DT;
 *    stock max17135_setup_timings() would reprogram NVM on a mismatch, we
 *    refuse instead.
 *  - the rails come up only with the module parameter hv=1 (default 0) and
 *    are forced off after hv_max_ms (backstop for a crashed frame source);
 *    a write to hv_kick restarts that count while they are up.
 * The wakeup / vcom_ctrl pins stay low (gpio-hogs): stock (max17135_pass_num
 * = 2, control_mode 0) never drives them either; the rails are register-
 * switched, so the hogs are NOT a protection by themselves.
 *
 * Stock sequences (decompiled, delays in ms):
 *  bridge up:   vdd1v8 on, osc 1, 5, mode0 0 + reset asserted, 1, vddc 1, 1,
 *               vddio on, 3, reset released, 3; 0x047c = 0, 31, check
 *               0x04a0 == 0x6200, table.
 *  bridge down: mode0 0, reset asserted, 1, vddio off, 1, vddc 0, 1, vdd1v8
 *               off, osc 0.
 *  rails up:    V3P3 (vdd, pm660l_l8) on; 0x09 |= 1; wait pwrgood (1 ms polls,
 *               up to max_wait * 20; else FAULT 0x0A bit 7 must be set);
 *               0x09 |= 2.
 *  rails down:  0x09 &= ~2; 0x09 &= ~1; max_wait (24); V3P3 off.
 *
 * Link check before HV: the DSI bridge's pre_enable only logs a
 * failed host power-on (e.g. "DSI PLL lock failed") and the chain goes on to
 * enable, which used to raise the rails with no frames behind them (two
 * incidents: the panel then got HV with a dead or slow link). With the
 * host's link clocks in DT ("pclk", "byte"), enable first requires both to be
 * running and pclk within 3% of the mode's clock; else the rails stay off
 * (link_refusals counts it). A failed power-on rolls the clocks back to off.
 * DRM hooks: prepare = bridge up (before the DSI host powers the lanes, so no
 * DSI traffic during the I2C table), enable = MD0 1 + rails (if hv; with hv=0
 * only the read-only PMIC pre-flight check, under V3P3), disable
 * = rails down + MD0 0, unprepare = bridge down. The first frame of a modeset
 * is sent after enable, i.e. with the rails already up: it must be neutral.
 *
 * Link pacing: stock runs the link at pclk 36.65 MHz / byte clock 54.98 MHz
 * (~440 Mbit/s per lane), so one frame takes ~11.06 ms on the wire. DPU
 * command mode starts each frame on a timer tick of 1/refresh, so the frame
 * plus the next commit must fit in one period. The mode is therefore tunable
 * at runtime (module parameters htotal and refresh, read at every connector
 * probe): pclk = htotal * 1426 * refresh. Measured flip rates:
 * 288/90 -> 45 Hz, 300/85 -> 84.5, 310/85 -> 85.00, 320/90 -> 90.00.
 * Default 310/85: the waveform file's 85 Hz, pclk 37.58 MHz = stock + 2.5%
 * (the bridge's DBI output must outrun the input; keep close to stock).
 *
 * Reading bridge registers over DSI (generic read of 0x04a0, 2-byte LE
 * address) was tried: the bridge answers with a valid generic long
 * read response (DI 0x1a, ECC + CRC OK) but WC 5 and an all-zero payload,
 * which msm_dsi_host_cmd_rx rejects. Register access stays on I2C.
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/pm_domain.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/workqueue.h>

#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>
#include <drm/drm_probe_helper.h>

/* stock eink-timings: line = 285 clocks, frame = 1423 lines */
static unsigned int htotal = 310;
module_param(htotal, uint, 0644);
MODULE_PARM_DESC(htotal, "line length in pixel clocks (>= 287)");
static unsigned int refresh = 85;
module_param(refresh, uint, 0644);
MODULE_PARM_DESC(refresh, "frame (timer) rate in Hz");
static bool hv;
module_param(hv, bool, 0644);
MODULE_PARM_DESC(hv, "allow the MAX17135 high-voltage rails at enable (default off)");
static unsigned int link_refusals;
module_param(link_refusals, uint, 0444);
MODULE_PARM_DESC(link_refusals, "times HV was refused because the DSI link was not up");

static unsigned int hv_max_ms = 3000;
module_param(hv_max_ms, uint, 0644);
MODULE_PARM_DESC(hv_max_ms, "force the rails off after this long (ms, 100..30000)");
static int temperature = -128;
module_param(temperature, int, 0444);
MODULE_PARM_DESC(temperature, "MAX17135 reg 0x00 (deg C) at the last rails-up; -128 = none (the NA1 has no thermistor on it: reads 127 = invalid; userspace uses the battery temperature, as stock)");
static bool hv_on;
module_param(hv_on, bool, 0444);
MODULE_PARM_DESC(hv_on, "rails currently up");
/*
 * hv_kick (2026-10-14, pen latency): hv_max_ms counted from rails-up, so the
 * frame source had to cycle the link (~100 ms without frames) every < 30 s of
 * continuous driving. A write to hv_kick while the rails are up restarts the
 * count instead: the backstop becomes "hv_max_ms since the last kick" and
 * still fires when the frame source dies. It only re-arms the timer, it never
 * touches the PMIC. One instance (the NA1 has one panel): na1_ctx.
 * Hard cap: this panel never sees the frames (they go DPU -> DSI host), so it
 * cannot tell a kick after a real flip from one by a wedged frame source that
 * still kicks; kicks never extend a rails-up past hv_cap_ms, then the timer
 * fires regardless (epdd cycles the link once before that).
 */
static struct na1_link *na1_ctx;
static DEFINE_MUTEX(na1_ctx_lock);	/* na1_ctx vs probe/remove */
static unsigned int hv_cap_ms = 600000;
module_param(hv_cap_ms, uint, 0644);
MODULE_PARM_DESC(hv_cap_ms, "kicks never keep the rails up longer than this (ms, 1000..3600000)");
static unsigned int hv_kicks;
module_param(hv_kicks, uint, 0444);
MODULE_PARM_DESC(hv_kicks, "hv_kick writes that re-armed the rails timeout");
/*
 * VDDMX vote while the link is up (2026-10-02, the power-key display stall):
 * four times the DSI PLL died under a running link after the device had sat
 * idle (pclk0/byte0 "stuck at on" at power-off = no source), never in 64
 * scripted rounds with the SoC kept busy. Nothing on the APSS votes MX (genpd
 * mx/mx_ao/mx_vfl all off-0), so its corner is whatever the other RPM masters
 * want, and the PLLs' digital side sits on it. The DT gives this device the
 * active-only MX domain; from prepare (before the DSI host powers the lanes)
 * to unprepare it holds this RPM level (RPM_SMD_LEVEL_*: 128 SVS, 256 NOM;
 * 0 = no vote). Read at every prepare, so it can be A/B'd live.
 */
static unsigned int mx_level;
module_param(mx_level, uint, 0644);
MODULE_PARM_DESC(mx_level, "RPM VDDMX level held while the link is up (0 = none, 128 svs, 256 nom)");
static unsigned int mx_voted;
module_param(mx_voted, uint, 0444);
MODULE_PARM_DESC(mx_voted, "VDDMX level held right now");
/*
 * More holds while the link is up, for the same stall (2026-10-02, after the
 * s5a voltage vote alone did not fix it: s5 stayed at 1.328 V and the PLL still
 * died when an l6a consumer went off within ~5 ms of its lock):
 *  - hold_rfclk: keep the WCN RF clock buffer (RPM_SMD_RF_CLK1_PIN, the one
 *    ath10k votes) on, so WiFi going off does not switch it during a lock;
 *  - rail_load_ua: a load vote on s5a and s2b (the parents of the DSI LDOs
 *    l1a and l1b, and of the WCN rails) so the RPM keeps them in PWM instead
 *    of AUTO, and a load that disappears from them does not step the buck.
 * Both read at every prepare; held shows what is held (1 rfclk, 2 s5a, 4 s2b).
 */
static bool hold_rfclk;
module_param(hold_rfclk, bool, 0644);
MODULE_PARM_DESC(hold_rfclk, "hold the WCN RF clock buffer on while the link is up");
static unsigned int rail_load_ua;
module_param(rail_load_ua, uint, 0644);
MODULE_PARM_DESC(rail_load_ua, "load vote on s5a + s2b while the link is up (uA, 0 = none)");
/*
 * The traced stall (2026-10-02, wf-trace): the PLL locks fine; ~280 ms into the
 * link ath10k powers the WCN down (rf_clk1 off, then l19, l6, l9, l5 within
 * 2.4 ms) and the frame in flight never completes. hold_wcn keeps the chosen
 * WCN rails enabled from prepare to unprepare (bitmask: 8 l5 cx/mx, 16 l9 xo,
 * 32 l6 rf, 64 l19 3.3 V), each with wcn_load_ua (0 = no load vote), so a
 * WiFi/BT power-down during a link does not switch them off. Bisect in the
 * WCN's own power-down order: hold l5 before l9 before l6 before l19 (8, 24,
 * 56, 120), never a rail without the ones under it.
 */
/*
 * Diagnostic, default off. The bisect on b93cf7c6 (120 and 56 clean, 24 and 8
 * stalled) found l6 switching off mid-link; the cause was the DT: the DSI vdda
 * is l6 on this board (stock vdda-1p2-supply), we had it on l1a.
 */
static unsigned int hold_wcn;
module_param(hold_wcn, uint, 0644);
MODULE_PARM_DESC(hold_wcn, "WCN rails held while the link is up: 8 l5, 16 l9, 32 l6, 64 l19");
static unsigned int wcn_load_ua;
module_param(wcn_load_ua, uint, 0644);
MODULE_PARM_DESC(wcn_load_ua, "load vote on each held WCN rail (uA)");
static unsigned int held;
module_param(held, uint, 0444);
MODULE_PARM_DESC(held, "held right now: 1 rfclk, 2 s5a load, 4 s2b load, 8 l5, 16 l9, 32 l6, 64 l19");

static const char * const wcn_supply[] = { "wcn-l5", "wcn-l9", "wcn-l6", "wcn-l19" };

#define TC_SLEEP	0x047c
#define TC_IDREG	0x04a0
#define TC_ID		0x6200

#define MAX_TEMP	0x00
#define MAX_PRODUCT_REV	0x06
#define MAX_PRODUCT_ID	0x07
#define MAX_DVR		0x08
#define MAX_ENABLE	0x09
#define MAX_FAULT	0x0a
#define MAX_TIMING1	0x10
#define MAX_EN		BIT(0)
#define MAX_VCOM_EN	BIT(1)
#define MAX_WAIT_MS	24		/* stock DT max_wait */
#define MAX_VCOM_STOCK	0x6c		/* -1.58 V */

/* stock DT gvee_pwrup, vneg_pwrup, vpos_pwrup, gvdd_pwrup, gvdd_pwrdn, vpos_pwrdn, vneg_pwrdn, gvee_pwrdn */
static const u8 max_timings_stock[8] = { 0, 10, 20, 25, 0, 3, 10, 20 };

/* stock tc358762_reg_1872x1404_es103_90hz (timing_index 10), in order */
static const u32 tc_table[][2] = {
	{ 0x047c, 0x0 }, { 0x0210, 0x7 }, { 0x0164, 0x23 }, { 0x0168, 0x23 }, { 0x0144, 0x0 },
	{ 0x0148, 0x0 }, { 0x0114, 0x7 }, { 0x0450, 0x1 }, { 0x0454, 0x122 }, { 0x0484, 0x0 },
	{ 0x0480, 0x1f }, { 0x0400, 0x0 }, { 0x0410, 0x7 }, { 0x0418, 0x3c }, { 0x0440, 0x120 },
	{ 0x0464, 0x40f }, { 0x0420, 0x100000 }, { 0x0104, 0x1 }, { 0x0204, 0x1 }, { 0x0468, 0x4 },
	{ 0x0470, 0x6c420000 },
};

struct na1_link {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	struct device *dev;
	struct i2c_client *tc, *pmic;
	struct gpio_desc *osc, *mode0, *reset, *vddc, *pwrgood;
	struct regulator *vdd1v8, *vddio, *v3p3;
	struct regulator *s5a, *s2b;	/* parents of the DSI LDOs (optional, for rail_load_ua) */
	struct clk *rfclk;		/* WCN RF clock buffer (optional, for hold_rfclk) */
	struct regulator *wcn[4];	/* WCN rails, wcn_supply[] order (optional, for hold_wcn) */
	struct clk *pclk, *byte;	/* the DSI host's link clocks (optional) */
	struct device *mx;		/* virtual device of the VDDMX domain (optional) */
	struct mutex lock;		/* rails */
	struct delayed_work hv_timeout;
	unsigned long rails_since;	/* jiffies at rails-up, for hv_cap_ms */
	bool bridge_up, rails_up;
	bool enabled;			/* between panel enable and disable (under lock) */
};

static void na1_link_fill_mode(struct drm_display_mode *m)
{
	unsigned int ht = clamp(htotal, 287u, 2048u), hz = clamp(refresh, 10u, 120u);

	*m = (struct drm_display_mode) {
		.clock = ht * 1426 * hz / 1000,
		.hdisplay = 285,
		.hsync_start = 285 + 1,
		.hsync_end = 285 + 2,
		.htotal = ht,
		.vdisplay = 1423,
		.vsync_start = 1423 + 1,
		.vsync_end = 1423 + 2,
		.vtotal = 1426,
		.width_mm = 210,
		.height_mm = 157,
		.type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED,
	};
}

/* TC358762: 16-bit register big-endian, 32-bit data little-endian */
static int tc_write(struct na1_link *ctx, u16 reg, u32 val)
{
	u8 b[6] = { reg >> 8, reg & 0xff, val, val >> 8, val >> 16, val >> 24 };
	struct i2c_msg m = { .addr = ctx->tc->addr, .len = sizeof(b), .buf = b };
	int ret = i2c_transfer(ctx->tc->adapter, &m, 1);

	return ret == 1 ? 0 : ret < 0 ? ret : -EIO;
}

static int tc_read(struct na1_link *ctx, u16 reg, u32 *val)
{
	u8 a[2] = { reg >> 8, reg & 0xff }, d[4];
	struct i2c_msg m[2] = {
		{ .addr = ctx->tc->addr, .len = 2, .buf = a },
		{ .addr = ctx->tc->addr, .flags = I2C_M_RD, .len = 4, .buf = d },
	};
	int ret = i2c_transfer(ctx->tc->adapter, m, 2);

	if (ret != 2)
		return ret < 0 ? ret : -EIO;
	*val = d[0] | d[1] << 8 | d[2] << 16 | (u32)d[3] << 24;
	return 0;
}

static int max_read(struct na1_link *ctx, u8 reg)
{
	return i2c_smbus_read_byte_data(ctx->pmic, reg);
}

/* The one and only MAX17135 write: ENABLE (0x09), bits EN / VCOM_EN only. */
static int max_enable_bits(struct na1_link *ctx, u8 set, u8 clear)
{
	int v = max_read(ctx, MAX_ENABLE);

	if (WARN_ON((set | clear) & ~(MAX_EN | MAX_VCOM_EN)))
		return -EINVAL;
	if (v < 0)
		return v;
	return i2c_smbus_write_byte_data(ctx->pmic, MAX_ENABLE, (v & ~clear) | set);
}

static void na1_bridge_down(struct na1_link *ctx)
{
	if (!ctx->bridge_up)
		return;
	gpiod_set_value_cansleep(ctx->mode0, 0);
	gpiod_set_value_cansleep(ctx->reset, 1);
	usleep_range(1000, 1500);
	regulator_disable(ctx->vddio);
	usleep_range(1000, 1500);
	gpiod_set_value_cansleep(ctx->vddc, 0);
	usleep_range(1000, 1500);
	regulator_disable(ctx->vdd1v8);
	gpiod_set_value_cansleep(ctx->osc, 0);
	ctx->bridge_up = false;
}

static int na1_bridge_up(struct na1_link *ctx)
{
	u32 v;
	int i, ret;

	ret = regulator_enable(ctx->vdd1v8);
	if (ret)
		return ret;
	gpiod_set_value_cansleep(ctx->osc, 1);
	usleep_range(5000, 6000);
	gpiod_set_value_cansleep(ctx->mode0, 0);
	gpiod_set_value_cansleep(ctx->reset, 1);
	usleep_range(1000, 1500);
	gpiod_set_value_cansleep(ctx->vddc, 1);
	usleep_range(1000, 1500);
	ret = regulator_enable(ctx->vddio);
	if (ret) {
		gpiod_set_value_cansleep(ctx->vddc, 0);
		gpiod_set_value_cansleep(ctx->osc, 0);
		regulator_disable(ctx->vdd1v8);
		return ret;
	}
	usleep_range(3000, 4000);
	gpiod_set_value_cansleep(ctx->reset, 0);
	usleep_range(3000, 4000);
	ctx->bridge_up = true;

	/* the chip leaves reset asleep (all registers read 0x01010101) */
	ret = tc_write(ctx, TC_SLEEP, 0);
	if (ret)
		goto fail;
	msleep(31);
	ret = tc_read(ctx, TC_IDREG, &v);
	if (ret)
		goto fail;
	if (v != TC_ID) {
		dev_err(ctx->dev, "TC358762 id 0x%08x, expected 0x%x\n", v, TC_ID);
		ret = -ENODEV;
		goto fail;
	}
	for (i = 0; i < ARRAY_SIZE(tc_table); i++) {
		ret = tc_write(ctx, tc_table[i][0], tc_table[i][1]);
		if (ret)
			goto fail;
	}
	for (i = 0; i < ARRAY_SIZE(tc_table); i++) {
		u16 reg = tc_table[i][0];

		ret = tc_read(ctx, reg, &v);
		if (ret)
			goto fail;
		/* STARTPPI / STARTDSI self-clear (stock's own dump reads 0) */
		if (v != tc_table[i][1] && reg != 0x0104 && reg != 0x0204) {
			dev_err(ctx->dev, "TC358762 0x%04x = 0x%08x, wrote 0x%08x\n",
				reg, v, tc_table[i][1]);
			ret = -EIO;
			goto fail;
		}
	}
	dev_info(ctx->dev, "TC358762 up: id 0x%x, stock table verified\n", TC_ID);
	return 0;
fail:
	dev_err(ctx->dev, "TC358762 bring-up failed: %d\n", ret);
	na1_bridge_down(ctx);
	return ret;
}

static void na1_rails_down(struct na1_link *ctx)
{
	int ret;

	lockdep_assert_held(&ctx->lock);
	if (!ctx->rails_up)
		return;
	ret = max_enable_bits(ctx, 0, MAX_VCOM_EN);
	if (!ret)
		ret = max_enable_bits(ctx, 0, MAX_EN);
	if (ret)
		dev_err(ctx->dev, "MAX17135 rails off FAILED: %d\n", ret);
	msleep(MAX_WAIT_MS);
	regulator_disable(ctx->v3p3);
	ctx->rails_up = false;
	hv_on = false;
	dev_info(ctx->dev, "MAX17135 rails down\n");
}

/* everything the PMIC must look like before we touch ENABLE */
static int na1_pmic_check(struct na1_link *ctx)
{
	int i, v;

	v = max_read(ctx, MAX_PRODUCT_ID);
	if (v != 'M' || max_read(ctx, MAX_PRODUCT_REV) != 0) {
		dev_err(ctx->dev, "MAX17135 id 0x%x: not the stock part\n", v);
		return -ENODEV;
	}
	v = max_read(ctx, MAX_DVR);
	if (v != MAX_VCOM_STOCK) {
		dev_err(ctx->dev, "MAX17135 VCOM DVR 0x%x != stock 0x%x: refusing\n", v, MAX_VCOM_STOCK);
		return -EINVAL;
	}
	for (i = 0; i < 8; i++) {
		v = max_read(ctx, MAX_TIMING1 + i);
		if (v != max_timings_stock[i]) {
			dev_err(ctx->dev, "MAX17135 timing%d = %d != stock %d: refusing\n",
				i + 1, v, max_timings_stock[i]);
			return -EINVAL;
		}
	}
	v = max_read(ctx, MAX_ENABLE);
	if (v < 0 || (v & (MAX_EN | MAX_VCOM_EN))) {
		dev_err(ctx->dev, "MAX17135 ENABLE = 0x%x before rails-up: refusing\n", v);
		return -EINVAL;
	}
	v = max_read(ctx, MAX_FAULT);
	if (v != 0) {
		dev_err(ctx->dev, "MAX17135 FAULT = 0x%x: refusing\n", v);
		return -EIO;
	}
	return 0;
}

static int na1_rails_up(struct na1_link *ctx)
{
	int i, v, ret;

	lockdep_assert_held(&ctx->lock);
	ret = regulator_enable(ctx->v3p3);
	if (ret)
		return ret;
	ctx->rails_up = true;	/* from here on, na1_rails_down() undoes it */

	ret = na1_pmic_check(ctx);
	if (ret)
		goto fail;
	ret = max_enable_bits(ctx, MAX_EN, 0);
	if (ret)
		goto fail;
	for (i = 0; i < MAX_WAIT_MS * 20; i++) {
		if (gpiod_get_value_cansleep(ctx->pwrgood) > 0)
			break;
		usleep_range(1000, 1100);
	}
	if (i == MAX_WAIT_MS * 20) {
		v = max_read(ctx, MAX_FAULT);
		if (v < 0 || !(v & BIT(7))) {
			dev_err(ctx->dev, "MAX17135 no power good (FAULT 0x%x)\n", v);
			ret = -ETIMEDOUT;
			goto fail;
		}
	}
	ret = max_enable_bits(ctx, MAX_VCOM_EN, 0);
	if (ret)
		goto fail;
	v = max_read(ctx, MAX_TEMP);
	temperature = v < 0 || v == 0x7f ? -128 : (s8)v;	/* 0x7f: no valid thermistor reading */
	hv_on = true;
	ctx->rails_since = jiffies;
	/* mod_: a kick racing a disable may have left the timer armed */
	mod_delayed_work(system_percpu_wq, &ctx->hv_timeout,
			 msecs_to_jiffies(clamp(hv_max_ms, 100u, 30000u)));
	if (temperature == -128)
		dev_info(ctx->dev, "MAX17135 rails up (power good after %d ms)\n", i);
	else
		dev_info(ctx->dev, "MAX17135 rails up (power good after %d ms), temp %d C\n", i, temperature);
	return 0;
fail:
	na1_rails_down(ctx);
	return ret;
}

static void na1_hv_timeout(struct work_struct *work)
{
	struct na1_link *ctx = container_of(to_delayed_work(work), struct na1_link, hv_timeout);

	mutex_lock(&ctx->lock);
	if (ctx->rails_up) {
		dev_warn(ctx->dev, "HV on longer than %u ms since rails-up/kick (or hv_cap_ms %u): forcing rails off\n",
			 hv_max_ms, hv_cap_ms);
		na1_rails_down(ctx);
	}
	mutex_unlock(&ctx->lock);
}

static int na1_hv_kick_set(const char *val, const struct kernel_param *kp)
{
	struct na1_link *ctx;
	unsigned long cap, up;
	int ret = -ENODEV;

	mutex_lock(&na1_ctx_lock);
	ctx = na1_ctx;
	if (ctx) {
		mutex_lock(&ctx->lock);
		ret = 0;
		cap = msecs_to_jiffies(clamp(hv_cap_ms, 1000u, 3600000u));
		up = jiffies - ctx->rails_since;
		if (ctx->rails_up && up < cap) {
			mod_delayed_work(system_percpu_wq, &ctx->hv_timeout,
					 min(msecs_to_jiffies(clamp(hv_max_ms, 100u, 30000u)), cap - up));
			hv_kicks++;
		}
		mutex_unlock(&ctx->lock);
	}
	mutex_unlock(&na1_ctx_lock);
	return ret;
}

static const struct kernel_param_ops na1_hv_kick_ops = {
	.set = na1_hv_kick_set,
};
module_param_cb(hv_kick, &na1_hv_kick_ops, NULL, 0200);
MODULE_PARM_DESC(hv_kick, "write: restart the hv_max_ms count if the rails are up");

static int na1_link_check(struct na1_link *ctx);

/*
 * Rails alone, with the link kept up ("warm idle", 2026-10-14): the frame
 * source drops the rails while the DRM pipe and the bridge stay up (no frames
 * on the wire), and raises them again without a modeset (skips the ~45 ms of
 * bridge power-up + init table, and the DSI host power-on). Same sequences and
 * guards as enable/disable: rails up only with hv=1, only between panel enable
 * and disable, only with the DSI link clocks running at the mode's rate,
 * after the read-only PMIC pre-flight check, and always with the hv_max_ms
 * timer armed (na1_rails_up). MD0 stays as enable left it (1): the bridge's
 * outputs hold the last (neutral) frame, as during an idle gap with the rails
 * up. Writing 0 is always allowed. Reads as hv_on. No new PMIC write path:
 * the same na1_rails_up/na1_rails_down (ENABLE 0x09 bits 0/1 only).
 * Uses the one-instance na1_ctx of the hv_kick param (0050).
 */

static int na1_rails_set(const char *val, const struct kernel_param *kp)
{
	struct na1_link *ctx;
	bool on;
	int ret;

	ret = kstrtobool(val, &on);
	if (ret)
		return ret;
	mutex_lock(&na1_ctx_lock);
	ctx = na1_ctx;
	if (!ctx) {
		ret = -ENODEV;
	} else if (!on) {
		cancel_delayed_work_sync(&ctx->hv_timeout);
		mutex_lock(&ctx->lock);
		na1_rails_down(ctx);
		mutex_unlock(&ctx->lock);
	} else {
		mutex_lock(&ctx->lock);
		if (ctx->rails_up)
			ret = 0;		/* the timer armed at its rails-up still runs */
		else if (!hv)
			ret = -EPERM;
		else if (!ctx->enabled || !ctx->bridge_up)
			ret = -ENOTCONN;
		else {
			ret = na1_link_check(ctx);
			if (!ret)
				ret = na1_rails_up(ctx);
		}
		mutex_unlock(&ctx->lock);
	}
	mutex_unlock(&na1_ctx_lock);
	return ret;
}

static const struct kernel_param_ops na1_rails_ops = {
	.set = na1_rails_set,
	.get = param_get_bool,
};
module_param_cb(rails, &na1_rails_ops, &hv_on, 0644);
MODULE_PARM_DESC(rails, "MAX17135 rails while the link stays up: write 1 = up (hv=1, panel enabled, link checked, hv_max_ms armed), 0 = down; reads hv_on");

static int na1_link_get_modes(struct drm_panel *panel, struct drm_connector *connector)
{
	struct drm_display_mode m;

	na1_link_fill_mode(&m);
	return drm_connector_helper_get_modes_fixed(connector, &m);
}

static void na1_mx_vote(struct na1_link *ctx)
{
	unsigned int level = READ_ONCE(mx_level);
	int ret;

	if (!ctx->mx || !level || mx_voted)
		return;
	ret = pm_runtime_resume_and_get(ctx->mx);
	if (ret) {
		dev_warn_ratelimited(ctx->dev, "MX domain on: %d\n", ret);
		return;
	}
	ret = dev_pm_genpd_set_performance_state(ctx->mx, level);
	if (ret) {
		dev_warn_ratelimited(ctx->dev, "MX level %u: %d\n", level, ret);
		pm_runtime_put(ctx->mx);
		return;
	}
	mx_voted = level;
}

static void na1_mx_unvote(struct na1_link *ctx)
{
	if (!mx_voted)
		return;
	dev_pm_genpd_set_performance_state(ctx->mx, 0);
	pm_runtime_put(ctx->mx);
	mx_voted = 0;
}

static void na1_hold_rail(struct na1_link *ctx, struct regulator *reg, unsigned int bit,
			  unsigned int load)
{
	if (!reg)
		return;
	if (regulator_set_load(reg, load) || regulator_enable(reg)) {
		dev_warn_ratelimited(ctx->dev, "rail load hold %u failed\n", bit);
		regulator_set_load(reg, 0);
		return;
	}
	held |= bit;
}

static void na1_hold(struct na1_link *ctx)
{
	unsigned int load = READ_ONCE(rail_load_ua), mask;
	int i;

	if (held)
		return;
	if (READ_ONCE(hold_rfclk) && ctx->rfclk && !clk_prepare_enable(ctx->rfclk))
		held |= 1;
	if (load) {
		na1_hold_rail(ctx, ctx->s5a, 2, load);
		na1_hold_rail(ctx, ctx->s2b, 4, load);
	}
	mask = READ_ONCE(hold_wcn);
	load = READ_ONCE(wcn_load_ua);
	for (i = 0; i < ARRAY_SIZE(ctx->wcn); i++)
		if (mask & (8 << i))
			na1_hold_rail(ctx, ctx->wcn[i], 8 << i, load);
}

static void na1_release(struct na1_link *ctx)
{
	int i;

	for (i = ARRAY_SIZE(ctx->wcn) - 1; i >= 0; i--)
		if (held & (8 << i)) {
			regulator_disable(ctx->wcn[i]);
			regulator_set_load(ctx->wcn[i], 0);
		}
	if (held & 4) {
		regulator_disable(ctx->s2b);
		regulator_set_load(ctx->s2b, 0);
	}
	if (held & 2) {
		regulator_disable(ctx->s5a);
		regulator_set_load(ctx->s5a, 0);
	}
	if (held & 1)
		clk_disable_unprepare(ctx->rfclk);
	held = 0;
}

static int na1_link_prepare(struct drm_panel *panel)
{
	struct na1_link *ctx = container_of(panel, struct na1_link, panel);
	int ret;

	na1_hold(ctx);
	na1_mx_vote(ctx);
	ret = na1_bridge_up(ctx);
	if (ret) {
		na1_mx_unvote(ctx);
		na1_release(ctx);
	}
	return ret;
}

/* the DSI link is really up: link clocks running, pclk at the mode's rate */
static int na1_link_check(struct na1_link *ctx)
{
	struct drm_display_mode m;
	unsigned long want, rate;

	if (!ctx->pclk || !ctx->byte)
		return 0;
	if (!__clk_is_enabled(ctx->pclk) || !__clk_is_enabled(ctx->byte)) {
		link_refusals++;
		dev_err(ctx->dev, "DSI link clocks off (host power-on failed?): rails stay off\n");
		return -EIO;
	}
	na1_link_fill_mode(&m);
	want = m.clock * 1000UL;
	rate = clk_get_rate(ctx->pclk);
	if (rate < want - want / 33 || rate > want + want / 33) {
		link_refusals++;
		dev_err(ctx->dev, "DSI pclk %lu Hz, mode wants %lu: rails stay off\n", rate, want);
		return -EIO;
	}
	return 0;
}

static int na1_link_enable(struct drm_panel *panel)
{
	struct na1_link *ctx = container_of(panel, struct na1_link, panel);
	int ret = 0;

	gpiod_set_value_cansleep(ctx->mode0, 1);	/* DBI-B: stock output_enable(1) */
	mutex_lock(&ctx->lock);
	if (hv) {
		ret = na1_link_check(ctx);
		if (!ret)
			ret = na1_rails_up(ctx);
	} else if (!regulator_enable(ctx->v3p3)) {
		/* hv off: the same pre-flight checks, read-only */
		ret = na1_pmic_check(ctx);
		regulator_disable(ctx->v3p3);
		dev_info(ctx->dev, "hv=0: rails stay off; MAX17135 pre-flight check %s (%d)\n",
			 ret ? "FAILED" : "ok", ret);
		ret = 0;
	}
	ctx->enabled = true;
	mutex_unlock(&ctx->lock);
	return ret;
}

static int na1_link_disable(struct drm_panel *panel)
{
	struct na1_link *ctx = container_of(panel, struct na1_link, panel);

	cancel_delayed_work_sync(&ctx->hv_timeout);
	mutex_lock(&ctx->lock);
	ctx->enabled = false;
	na1_rails_down(ctx);
	mutex_unlock(&ctx->lock);
	gpiod_set_value_cansleep(ctx->mode0, 0);	/* stock frame_done: output_enable(0) */
	return 0;
}

static int na1_link_unprepare(struct drm_panel *panel)
{
	struct na1_link *ctx = container_of(panel, struct na1_link, panel);

	na1_bridge_down(ctx);
	na1_mx_unvote(ctx);
	na1_release(ctx);
	return 0;
}

static const struct drm_panel_funcs na1_link_funcs = {
	.prepare = na1_link_prepare,
	.enable = na1_link_enable,
	.disable = na1_link_disable,
	.unprepare = na1_link_unprepare,
	.get_modes = na1_link_get_modes,
};

static struct i2c_client *na1_get_client(struct device *dev, const char *prop)
{
	struct device_node *np = of_parse_phandle(dev->of_node, prop, 0);
	struct i2c_client *c;

	if (!np)
		return ERR_PTR(dev_err_probe(dev, -EINVAL, "missing %s\n", prop));
	c = of_find_i2c_device_by_node(np);
	of_node_put(np);
	return c ? c : ERR_PTR(-EPROBE_DEFER);
}

static void na1_mx_detach(void *data)
{
	struct na1_link *ctx = data;

	na1_mx_unvote(ctx);
	dev_pm_domain_detach(ctx->mx, true);
}

static void na1_put_clients(void *data)
{
	struct na1_link *ctx = data;

	put_device(&ctx->tc->dev);
	put_device(&ctx->pmic->dev);
}

static int na1_link_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct na1_link *ctx;
	int i, ret;

	ctx = devm_drm_panel_alloc(dev, struct na1_link, panel, &na1_link_funcs,
				   DRM_MODE_CONNECTOR_DSI);
	if (IS_ERR(ctx))
		return PTR_ERR(ctx);

	ctx->dsi = dsi;
	ctx->dev = dev;
	mutex_init(&ctx->lock);
	INIT_DELAYED_WORK(&ctx->hv_timeout, na1_hv_timeout);
	mipi_dsi_set_drvdata(dsi, ctx);

	ctx->tc = na1_get_client(dev, "onyx,bridge");
	if (IS_ERR(ctx->tc))
		return PTR_ERR(ctx->tc);
	ctx->pmic = na1_get_client(dev, "onyx,pmic");
	if (IS_ERR(ctx->pmic)) {
		put_device(&ctx->tc->dev);
		return PTR_ERR(ctx->pmic);
	}
	ret = devm_add_action_or_reset(dev, na1_put_clients, ctx);
	if (ret)
		return ret;

	/* all off / asserted: the bridge is unpowered at boot */
	ctx->osc = devm_gpiod_get(dev, "osc", GPIOD_OUT_LOW);
	ctx->mode0 = devm_gpiod_get(dev, "mode0", GPIOD_OUT_LOW);
	ctx->reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	ctx->vddc = devm_gpiod_get(dev, "vddc", GPIOD_OUT_LOW);
	ctx->pwrgood = devm_gpiod_get(dev, "pmic-pwrgood", GPIOD_IN);
	if (IS_ERR(ctx->osc) || IS_ERR(ctx->mode0) || IS_ERR(ctx->reset) ||
	    IS_ERR(ctx->vddc) || IS_ERR(ctx->pwrgood))
		return dev_err_probe(dev, -EINVAL, "gpios\n");
	ctx->vdd1v8 = devm_regulator_get(dev, "vdd1v8");
	ctx->vddio = devm_regulator_get(dev, "vddio");
	ctx->v3p3 = devm_regulator_get(dev, "v3p3");
	ctx->pclk = devm_clk_get_optional(dev, "pclk");
	ctx->byte = devm_clk_get_optional(dev, "byte");
	if (IS_ERR(ctx->pclk) || IS_ERR(ctx->byte))
		return dev_err_probe(dev, -EPROBE_DEFER, "link clocks\n");
	if (!ctx->pclk || !ctx->byte)
		dev_warn(dev, "no DSI link clocks in DT: HV is not gated on the link\n");
	if (IS_ERR(ctx->vdd1v8) || IS_ERR(ctx->vddio) || IS_ERR(ctx->v3p3))
		return dev_err_probe(dev, -EPROBE_DEFER, "supplies\n");
	ctx->rfclk = devm_clk_get_optional(dev, "rfclk");
	if (IS_ERR(ctx->rfclk))
		return dev_err_probe(dev, PTR_ERR(ctx->rfclk), "RF clock\n");
	ctx->s5a = devm_regulator_get_optional(dev, "s5a");
	if (IS_ERR(ctx->s5a)) {
		if (PTR_ERR(ctx->s5a) != -ENODEV)
			return dev_err_probe(dev, PTR_ERR(ctx->s5a), "s5a\n");
		ctx->s5a = NULL;
	}
	ctx->s2b = devm_regulator_get_optional(dev, "s2b");
	if (IS_ERR(ctx->s2b)) {
		if (PTR_ERR(ctx->s2b) != -ENODEV)
			return dev_err_probe(dev, PTR_ERR(ctx->s2b), "s2b\n");
		ctx->s2b = NULL;
	}
	for (i = 0; i < ARRAY_SIZE(ctx->wcn); i++) {
		ctx->wcn[i] = devm_regulator_get_optional(dev, wcn_supply[i]);
		if (IS_ERR(ctx->wcn[i])) {
			if (PTR_ERR(ctx->wcn[i]) != -ENODEV)
				return dev_err_probe(dev, PTR_ERR(ctx->wcn[i]), "%s\n", wcn_supply[i]);
			ctx->wcn[i] = NULL;
		}
	}
	ctx->mx = dev_pm_domain_attach_by_name(dev, "mx");
	if (IS_ERR(ctx->mx))
		return dev_err_probe(dev, PTR_ERR(ctx->mx), "MX power domain\n");
	if (ctx->mx) {
		ret = devm_add_action_or_reset(dev, na1_mx_detach, ctx);
		if (ret)
			return ret;
	}

	dsi->lanes = 2;
	dsi->format = MIPI_DSI_FMT_RGB888;
	/* command mode, continuous HS clock */
	dsi->mode_flags = MIPI_DSI_MODE_LPM;

	drm_panel_add(&ctx->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		dev_err(dev, "mipi_dsi_attach: %d\n", ret);
		drm_panel_remove(&ctx->panel);
		return ret;
	}
	mutex_lock(&na1_ctx_lock);
	na1_ctx = ctx;
	mutex_unlock(&na1_ctx_lock);
	dev_info(dev, "NA1 e-ink link: 285x1423 RGB888, DSI cmd mode, 2 lanes; bridge %s, pmic %s\n",
		 dev_name(&ctx->tc->dev), dev_name(&ctx->pmic->dev));
	return 0;
}

static void na1_link_remove(struct mipi_dsi_device *dsi)
{
	struct na1_link *ctx = mipi_dsi_get_drvdata(dsi);

	mutex_lock(&na1_ctx_lock);
	na1_ctx = NULL;		/* no kick re-arms the timer from here on */
	mutex_unlock(&na1_ctx_lock);
	cancel_delayed_work_sync(&ctx->hv_timeout);
	mutex_lock(&ctx->lock);
	ctx->enabled = false;
	na1_rails_down(ctx);
	mutex_unlock(&ctx->lock);
	na1_bridge_down(ctx);
	na1_release(ctx);
	mipi_dsi_detach(dsi);
	drm_panel_remove(&ctx->panel);
}

static void na1_link_shutdown(struct mipi_dsi_device *dsi)
{
	struct na1_link *ctx = mipi_dsi_get_drvdata(dsi);

	mutex_lock(&na1_ctx_lock);
	na1_ctx = NULL;
	mutex_unlock(&na1_ctx_lock);
	cancel_delayed_work_sync(&ctx->hv_timeout);
	mutex_lock(&ctx->lock);
	ctx->enabled = false;
	na1_rails_down(ctx);
	mutex_unlock(&ctx->lock);
}

static const struct of_device_id na1_link_of_match[] = {
	{ .compatible = "onyx,noteair-eink-link" },
	{ }
};
MODULE_DEVICE_TABLE(of, na1_link_of_match);

static struct mipi_dsi_driver na1_link_driver = {
	.probe = na1_link_probe,
	.remove = na1_link_remove,
	.shutdown = na1_link_shutdown,
	.driver = {
		.name = "panel-onyx-na1-link",
		.of_match_table = na1_link_of_match,
	},
};
module_mipi_dsi_driver(na1_link_driver);

MODULE_DESCRIPTION("Onyx Boox Note Air e-ink DSI wire link, TC358762 + MAX17135 sequencing");
MODULE_LICENSE("GPL");
