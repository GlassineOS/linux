// SPDX-License-Identifier: GPL-2.0-only
/*
 * Awinic AW88194 smart PA (chip id 0x1806): the Onyx Boox Note Air speaker amp.
 *
 * The register sequence follows Awinic's GPL driver for the chip
 * (github.com/awinic-driver/aw88194, aw88194.c); the NA1 itself ran their
 * later aw881xx v0.1.6 with the same files. Brought up on the first unmute,
 * when the I2S bit clock runs (the amp's PLL locks on it):
 *   1. the register profile   <prefix>_reg.bin  (le16 address, le16 value pairs)
 *   2. the DSP firmware        <prefix>_fw.bin   -> DSP memory 0x8c00 (le16 words)
 *   3. the DSP config          <prefix>_cfg.bin  -> DSP memory 0x8600
 *   4. DSP check (PLL locked, DSP watchdog running), power up, unmute.
 * Afterwards unmute/mute only power it up/down; the profile stays loaded until
 * the chip is reset. Speaker calibration (Re) and the battery monitor of the
 * vendor driver are left out.
 */
#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#define AW88194_REG_ID		0x00
#define AW88194_REG_SYSST	0x01
#define AW88194_REG_SYSCTRL	0x04
#define AW88194_REG_I2SCTRL	0x05
#define AW88194_REG_PWMCTRL	0x08
#define AW88194_REG_SYSCTRL2	0x11
#define AW88194_REG_DSPMADD	0x40
#define AW88194_REG_DSPMDAT	0x41
#define AW88194_REG_WDT		0x42
#define AW88194_REG_MAX		0x7f

#define AW88194_CHIP_ID			0x1806
#define AW88194_SYSST_PLLS		BIT(0)
#define AW88194_SYSCTRL_PWDN		BIT(0)
#define AW88194_SYSCTRL_DSP_BYPASS	BIT(2)
#define AW88194_PWMCTRL_HMUTE		BIT(0)
#define AW88194_SYSCTRL2_MEMCLK_PLL	BIT(12)
#define AW88194_I2SCTRL_SR		GENMASK(3, 0)
#define AW88194_I2SCTRL_FMS		GENMASK(7, 6)

#define AW88194_DSP_FW_ADDR	0x8c00
#define AW88194_DSP_CFG_ADDR	0x8600

struct aw88194 {
	struct device *dev;
	struct regmap *regmap;
	struct gpio_desc *reset;
	struct mutex lock;		/* the bring-up sequence vs. mute */
	const char *fw_prefix;
	bool loaded;			/* profile + DSP firmware + config in the chip */
};

static int aw88194_bits(struct aw88194 *aw, unsigned int reg, unsigned int mask, bool set)
{
	return regmap_update_bits(aw->regmap, reg, mask, set ? mask : 0);
}

static void aw88194_mute(struct aw88194 *aw, bool mute)
{
	aw88194_bits(aw, AW88194_REG_PWMCTRL, AW88194_PWMCTRL_HMUTE, mute);
}

static void aw88194_power(struct aw88194 *aw, bool on)
{
	aw88194_bits(aw, AW88194_REG_SYSCTRL, AW88194_SYSCTRL_PWDN, !on);
}

static bool aw88194_pll_locked(struct aw88194 *aw)
{
	unsigned int v = 0;

	return !regmap_read(aw->regmap, AW88194_REG_SYSST, &v) && (v & AW88194_SYSST_PLLS);
}

static int aw88194_request(struct aw88194 *aw, const char *part, const struct firmware **fw)
{
	char name[96];
	int ret;

	snprintf(name, sizeof(name), "%s_%s.bin", aw->fw_prefix, part);
	ret = request_firmware(fw, name, aw->dev);
	if (ret)
		dev_err(aw->dev, "%s: %d\n", name, ret);
	else if ((*fw)->size % 2 || !(*fw)->size) {
		dev_err(aw->dev, "%s: bad size %zu\n", name, (*fw)->size);
		release_firmware(*fw);
		ret = -EINVAL;
	}
	return ret;
}

static int aw88194_load_regs(struct aw88194 *aw)
{
	const struct firmware *fw;
	size_t i;
	int ret = aw88194_request(aw, "reg", &fw);

	if (ret)
		return ret;
	for (i = 0; i + 4 <= fw->size && !ret; i += 4)
		ret = regmap_write(aw->regmap, fw->data[i] | fw->data[i + 1] << 8,
				   fw->data[i + 2] | fw->data[i + 3] << 8);
	release_firmware(fw);
	return ret;
}

static int aw88194_load_dsp(struct aw88194 *aw, const char *part, unsigned int addr)
{
	const struct firmware *fw;
	size_t i;
	int ret = aw88194_request(aw, part, &fw);

	if (ret)
		return ret;
	ret = regmap_write(aw->regmap, AW88194_REG_DSPMADD, addr);
	for (i = 0; i + 2 <= fw->size && !ret; i += 2)
		ret = regmap_write(aw->regmap, AW88194_REG_DSPMDAT,
				   fw->data[i] | fw->data[i + 1] << 8);
	release_firmware(fw);
	return ret;
}

/* PLL locked and, unless the profile bypasses it, the DSP running */
static int aw88194_dsp_check(struct aw88194 *aw, bool bypass)
{
	unsigned int wdt;
	int i;

	aw88194_power(aw, true);
	aw88194_bits(aw, AW88194_REG_SYSCTRL2, AW88194_SYSCTRL2_MEMCLK_PLL, true);
	for (i = 0; i < 5; i++) {
		if (aw88194_pll_locked(aw)) {
			if (bypass)
				return 0;
			aw88194_bits(aw, AW88194_REG_SYSCTRL, AW88194_SYSCTRL_DSP_BYPASS, true);
			aw88194_bits(aw, AW88194_REG_SYSCTRL, AW88194_SYSCTRL_DSP_BYPASS, false);
			usleep_range(1000, 2000);
			if (!regmap_read(aw->regmap, AW88194_REG_WDT, &wdt) && wdt)
				return 0;
			dev_warn(aw->dev, "DSP watchdog not running\n");
		}
		usleep_range(2000, 3000);
	}
	return -EIO;
}

/* the whole bring-up: needs the bit clock (called on unmute) */
static int aw88194_cold_start(struct aw88194 *aw)
{
	unsigned int sysctrl = 0;
	bool bypass;
	int ret, i;

	ret = aw88194_load_regs(aw);
	if (ret)
		return ret;
	regmap_read(aw->regmap, AW88194_REG_SYSCTRL, &sysctrl);
	bypass = sysctrl & AW88194_SYSCTRL_DSP_BYPASS;
	for (i = 0; i < 10 && !aw88194_pll_locked(aw); i++)
		usleep_range(2000, 3000);
	if (i == 10) {
		dev_err(aw->dev, "no I2S clock (PLL not locked)\n");
		return -EIO;
	}
	aw88194_mute(aw, true);
	aw88194_bits(aw, AW88194_REG_SYSCTRL, AW88194_SYSCTRL_DSP_BYPASS, true);
	aw88194_bits(aw, AW88194_REG_SYSCTRL2, AW88194_SYSCTRL2_MEMCLK_PLL, true);
	ret = aw88194_load_dsp(aw, "fw", AW88194_DSP_FW_ADDR);
	if (!ret)
		ret = aw88194_load_dsp(aw, "cfg", AW88194_DSP_CFG_ADDR);
	if (!ret)
		ret = aw88194_dsp_check(aw, bypass);
	if (ret) {
		dev_err(aw->dev, "bring-up failed: %d\n", ret);
		return ret;
	}
	aw->loaded = true;
	dev_info(aw->dev, "profile, DSP firmware and config loaded (DSP %s)\n",
		 bypass ? "bypassed" : "on");
	return 0;
}

static int aw88194_mute_stream(struct snd_soc_dai *dai, int mute, int stream)
{
	struct aw88194 *aw = snd_soc_component_get_drvdata(dai->component);
	int ret = 0;

	if (stream != SNDRV_PCM_STREAM_PLAYBACK)
		return 0;
	mutex_lock(&aw->lock);
	if (mute) {
		aw88194_mute(aw, true);
		aw88194_power(aw, false);
	} else {
		if (!aw->loaded)
			ret = aw88194_cold_start(aw);
		if (!ret) {
			aw88194_power(aw, true);
			aw88194_mute(aw, false);
		}
	}
	mutex_unlock(&aw->lock);
	/* a failed bring-up leaves the speaker silent, the stream runs on */
	return 0;
}

static int aw88194_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	struct aw88194 *aw = snd_soc_component_get_drvdata(dai->component);
	unsigned int sr, fms;

	switch (params_rate(params)) {
	case 8000: sr = 1; break;
	case 16000: sr = 3; break;
	case 32000: sr = 6; break;
	case 44100: sr = 7; break;
	case 48000: sr = 8; break;
	default: return -EINVAL;
	}
	switch (params_width(params)) {
	case 16: fms = 0; break;
	case 24: fms = 2; break;
	case 32: fms = 3; break;
	default: return -EINVAL;
	}
	return regmap_update_bits(aw->regmap, AW88194_REG_I2SCTRL,
				  AW88194_I2SCTRL_SR | AW88194_I2SCTRL_FMS,
				  FIELD_PREP(AW88194_I2SCTRL_SR, sr) |
				  FIELD_PREP(AW88194_I2SCTRL_FMS, fms));
}

static int aw88194_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	if ((fmt & SND_SOC_DAIFMT_FORMAT_MASK) != SND_SOC_DAIFMT_I2S ||
	    (fmt & SND_SOC_DAIFMT_CLOCK_PROVIDER_MASK) != SND_SOC_DAIFMT_BC_FC)
		return -EINVAL;
	return 0;
}

static const struct snd_soc_dai_ops aw88194_dai_ops = {
	.set_fmt = aw88194_set_fmt,
	.hw_params = aw88194_hw_params,
	.mute_stream = aw88194_mute_stream,
	.no_capture_mute = 1,
};

static struct snd_soc_dai_driver aw88194_dai = {
	.name = "aw88194-aif",
	.playback = {
		.stream_name = "Speaker Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE |
			   SNDRV_PCM_FMTBIT_S32_LE,
	},
	.ops = &aw88194_dai_ops,
};

static const struct snd_soc_dapm_widget aw88194_widgets[] = {
	SND_SOC_DAPM_OUTPUT("SPK"),
};

static const struct snd_soc_dapm_route aw88194_routes[] = {
	{ "SPK", NULL, "Speaker Playback" },
};

static const struct snd_soc_component_driver aw88194_component = {
	.dapm_widgets = aw88194_widgets,
	.num_dapm_widgets = ARRAY_SIZE(aw88194_widgets),
	.dapm_routes = aw88194_routes,
	.num_dapm_routes = ARRAY_SIZE(aw88194_routes),
	.endianness = 1,
};

static const struct regmap_config aw88194_regmap = {
	.reg_bits = 8,
	.val_bits = 16,
	.max_register = AW88194_REG_MAX,
	.cache_type = REGCACHE_NONE,
};

static int aw88194_i2c_probe(struct i2c_client *i2c)
{
	struct device *dev = &i2c->dev;
	struct aw88194 *aw;
	unsigned int id = 0;
	int ret, i;

	aw = devm_kzalloc(dev, sizeof(*aw), GFP_KERNEL);
	if (!aw)
		return -ENOMEM;
	aw->dev = dev;
	mutex_init(&aw->lock);
	i2c_set_clientdata(i2c, aw);
	aw->regmap = devm_regmap_init_i2c(i2c, &aw88194_regmap);
	if (IS_ERR(aw->regmap))
		return PTR_ERR(aw->regmap);
	aw->fw_prefix = "awinic/aw88194_spk";
	of_property_read_string(dev->of_node, "firmware-name", &aw->fw_prefix);

	/* reset pulse (asserted = low, as the vendor driver: 0, 1 ms, 1, 1 ms) */
	aw->reset = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(aw->reset))
		return dev_err_probe(dev, PTR_ERR(aw->reset), "reset gpio\n");
	usleep_range(1000, 2000);
	gpiod_set_value_cansleep(aw->reset, 0);
	usleep_range(1000, 2000);

	for (i = 0; i < 5; i++) {
		ret = regmap_read(aw->regmap, AW88194_REG_ID, &id);
		if (!ret && id == AW88194_CHIP_ID)
			break;
		usleep_range(2000, 3000);
	}
	if (ret)
		return dev_err_probe(dev, ret, "no chip id\n");
	if (id != AW88194_CHIP_ID)
		return dev_err_probe(dev, -ENODEV, "chip id 0x%04x, not 0x%04x\n",
				     id, AW88194_CHIP_ID);
	aw88194_mute(aw, true);
	aw88194_power(aw, false);
	dev_info(dev, "AW88194 (0x%04x), firmware %s_*.bin\n", id, aw->fw_prefix);

	return devm_snd_soc_register_component(dev, &aw88194_component, &aw88194_dai, 1);
}

static const struct of_device_id aw88194_of_match[] = {
	{ .compatible = "awinic,aw88194" },
	{ }
};
MODULE_DEVICE_TABLE(of, aw88194_of_match);

static struct i2c_driver aw88194_i2c_driver = {
	.driver = {
		.name = "aw88194",
		.of_match_table = aw88194_of_match,
	},
	.probe = aw88194_i2c_probe,
};
module_i2c_driver(aw88194_i2c_driver);

MODULE_DESCRIPTION("Awinic AW88194 smart PA");
MODULE_LICENSE("GPL");
