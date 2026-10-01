// SPDX-License-Identifier: GPL-2.0-only
/*
 * NA1 (Onyx Boox Note Air, SDM636): raw dump of the RPM External Log (debug).
 *
 * The RPM keeps a ring log in its DATARAM (phys 0x290000, RPM view 0x90000):
 * the word at +0x18 points to the header ("RPM External Log"); header +0x24 =
 * buffer address, +0x28 = length (both RPM view). /sys/kernel/debug/na1_rpm_log
 * prints the header and the whole ring as raw 32-bit words; the host decodes
 * (a host-side tool). Read it right after a resume.
 *
 * Only DATARAM is mapped, and only aligned 32-bit readl()s are used, mapped
 * lazily on the first read: a /dev/mem read of RPM RAM sent the NA1 to 900E
 * (2026-10-01). Never touch code RAM (0x200000).
 *
 * The ring holds ~0.2 s of awake traffic, so a read from userspace after a
 * resume only sees the resume. Once mapped (after a first, supervised read),
 * every system resume snapshots header + ring in the noirq phase (s2idle runs
 * no syscore hooks): /sys/kernel/debug/na1_rpm_log_resume.
 */
#include <linux/debugfs.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/mutex.h>
#include <linux/ktime.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/seq_file.h>

#define NA1_DATARAM_PHYS	0x290000
#define NA1_DATARAM_SIZE	0x14000
#define NA1_DATARAM_RPM		0x90000		/* the same RAM in the RPM's view */
#define NA1_HDR_WORDS		16
#define NA1_LOG_MAX		0x4000

static void __iomem *na1_dataram;
static DEFINE_MUTEX(na1_rpm_log_lock);

/* the last resume snapshot: header words, then the ring */
static u32 na1_snap[NA1_HDR_WORDS + NA1_LOG_MAX / 4];
static u32 na1_snap_ptr, na1_snap_buf, na1_snap_len;
static unsigned int na1_snap_count;
static u64 na1_snap_ns;

/* RPM-view address of n bytes -> offset in the map, or -1 */
static long na1_rpm_off(u32 rpm, u32 n)
{
	if (rpm < NA1_DATARAM_RPM || rpm & 3 ||
	    rpm - NA1_DATARAM_RPM + (u64)n > NA1_DATARAM_SIZE)
		return -1;
	return rpm - NA1_DATARAM_RPM;
}

static int na1_rpm_log_show(struct seq_file *m, void *unused)
{
	u32 ptr, buf, len, i;
	long hdr, off;

	mutex_lock(&na1_rpm_log_lock);
	if (!na1_dataram)
		WRITE_ONCE(na1_dataram, ioremap(NA1_DATARAM_PHYS, NA1_DATARAM_SIZE));
	if (!na1_dataram) {
		mutex_unlock(&na1_rpm_log_lock);
		return -ENOMEM;
	}

	ptr = readl(na1_dataram + 0x18);
	hdr = na1_rpm_off(ptr, NA1_HDR_WORDS * 4);
	seq_printf(m, "header @rpm 0x%08x\n", ptr);
	if (hdr < 0) {
		seq_puts(m, "header pointer outside DATARAM\n");
		goto out;
	}
	for (i = 0; i < NA1_HDR_WORDS; i++)
		seq_printf(m, "%08x%c", readl(na1_dataram + hdr + i * 4),
			   (i % 8 == 7) ? '\n' : ' ');

	buf = readl(na1_dataram + hdr + 0x24);
	len = readl(na1_dataram + hdr + 0x28);
	seq_printf(m, "ring @rpm 0x%08x len 0x%x\n", buf, len);
	off = na1_rpm_off(buf, len);
	if (off < 0 || !len || len > NA1_LOG_MAX || len & 3) {
		seq_puts(m, "ring outside DATARAM or bad length\n");
		goto out;
	}
	for (i = 0; i < len / 4; i++)
		seq_printf(m, "%08x%c", readl(na1_dataram + off + i * 4),
			   (i % 8 == 7) ? '\n' : ' ');
out:
	mutex_unlock(&na1_rpm_log_lock);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(na1_rpm_log);

/* noirq resume: no locks; the map only ever goes from NULL to set */
static int na1_rpm_log_resume_noirq(struct device *dev)
{
	void __iomem *base = READ_ONCE(na1_dataram);
	u32 ptr, buf, len, i;
	long hdr, off;

	if (!base)
		return 0;
	ptr = readl(base + 0x18);
	hdr = na1_rpm_off(ptr, NA1_HDR_WORDS * 4);
	if (hdr < 0)
		return 0;
	buf = readl(base + hdr + 0x24);
	len = readl(base + hdr + 0x28);
	off = na1_rpm_off(buf, len);
	if (off < 0 || !len || len > NA1_LOG_MAX || len & 3)
		return 0;
	for (i = 0; i < NA1_HDR_WORDS; i++)
		na1_snap[i] = readl(base + hdr + i * 4);
	for (i = 0; i < len / 4; i++)
		na1_snap[NA1_HDR_WORDS + i] = readl(base + off + i * 4);
	na1_snap_ptr = ptr;
	na1_snap_buf = buf;
	na1_snap_len = len;
	na1_snap_ns = ktime_get_boottime_ns();
	na1_snap_count++;
	return 0;
}

static int na1_rpm_log_resume_show(struct seq_file *m, void *unused)
{
	u32 i;

	if (!na1_snap_count) {
		seq_puts(m, "no snapshot (not mapped yet: read na1_rpm_log once, then suspend)\n");
		return 0;
	}
	seq_printf(m, "snapshot %u at boottime %llu ns\n", na1_snap_count, na1_snap_ns);
	seq_printf(m, "header @rpm 0x%08x\n", na1_snap_ptr);
	for (i = 0; i < NA1_HDR_WORDS; i++)
		seq_printf(m, "%08x%c", na1_snap[i], (i % 8 == 7) ? '\n' : ' ');
	seq_printf(m, "ring @rpm 0x%08x len 0x%x\n", na1_snap_buf, na1_snap_len);
	for (i = 0; i < na1_snap_len / 4; i++)
		seq_printf(m, "%08x%c", na1_snap[NA1_HDR_WORDS + i], (i % 8 == 7) ? '\n' : ' ');
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(na1_rpm_log_resume);

static const struct dev_pm_ops na1_rpm_log_pm = {
	.resume_noirq = na1_rpm_log_resume_noirq,
};

static struct platform_driver na1_rpm_log_driver = {
	.driver = {
		.name = "na1-rpm-log",
		.pm = &na1_rpm_log_pm,
	},
};

static int __init na1_rpm_log_init(void)
{
	struct platform_device *pdev;

	if (!of_machine_is_compatible("onyx,noteair"))
		return 0;
	debugfs_create_file("na1_rpm_log", 0400, NULL, NULL, &na1_rpm_log_fops);
	debugfs_create_file("na1_rpm_log_resume", 0400, NULL, NULL, &na1_rpm_log_resume_fops);
	if (platform_driver_register(&na1_rpm_log_driver))
		return 0;
	pdev = platform_device_register_simple("na1-rpm-log", -1, NULL, 0);
	if (IS_ERR(pdev))
		platform_driver_unregister(&na1_rpm_log_driver);
	return 0;
}
late_initcall(na1_rpm_log_init);
