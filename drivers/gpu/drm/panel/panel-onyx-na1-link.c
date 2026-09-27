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
 *    are forced off after hv_max_ms (backstop for a crashed frame source).
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
	struct clk *pclk, *byte;	/* the DSI host's link clocks (optional) */
	struct mutex lock;		/* rails */
	struct delayed_work hv_timeout;
	bool bridge_up, rails_up;
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
	schedule_delayed_work(&ctx->hv_timeout,
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
		dev_warn(ctx->dev, "HV on longer than %u ms: forcing rails off\n", hv_max_ms);
		na1_rails_down(ctx);
	}
	mutex_unlock(&ctx->lock);
}

static int na1_link_get_modes(struct drm_panel *panel, struct drm_connector *connector)
{
	struct drm_display_mode m;

	na1_link_fill_mode(&m);
	return drm_connector_helper_get_modes_fixed(connector, &m);
}

static int na1_link_prepare(struct drm_panel *panel)
{
	return na1_bridge_up(container_of(panel, struct na1_link, panel));
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
	mutex_unlock(&ctx->lock);
	return ret;
}

static int na1_link_disable(struct drm_panel *panel)
{
	struct na1_link *ctx = container_of(panel, struct na1_link, panel);

	cancel_delayed_work_sync(&ctx->hv_timeout);
	mutex_lock(&ctx->lock);
	na1_rails_down(ctx);
	mutex_unlock(&ctx->lock);
	gpiod_set_value_cansleep(ctx->mode0, 0);	/* stock frame_done: output_enable(0) */
	return 0;
}

static int na1_link_unprepare(struct drm_panel *panel)
{
	na1_bridge_down(container_of(panel, struct na1_link, panel));
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
	int ret;

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
	dev_info(dev, "NA1 e-ink link: 285x1423 RGB888, DSI cmd mode, 2 lanes; bridge %s, pmic %s\n",
		 dev_name(&ctx->tc->dev), dev_name(&ctx->pmic->dev));
	return 0;
}

static void na1_link_remove(struct mipi_dsi_device *dsi)
{
	struct na1_link *ctx = mipi_dsi_get_drvdata(dsi);

	cancel_delayed_work_sync(&ctx->hv_timeout);
	mutex_lock(&ctx->lock);
	na1_rails_down(ctx);
	mutex_unlock(&ctx->lock);
	na1_bridge_down(ctx);
	mipi_dsi_detach(dsi);
	drm_panel_remove(&ctx->panel);
}

static void na1_link_shutdown(struct mipi_dsi_device *dsi)
{
	struct na1_link *ctx = mipi_dsi_get_drvdata(dsi);

	cancel_delayed_work_sync(&ctx->hv_timeout);
	mutex_lock(&ctx->lock);
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
