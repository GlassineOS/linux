// SPDX-License-Identifier: GPL-2.0-only
/*
 * Test hooks for the kexec probes on the Note Air (SDM636). Probe kernels only:
 * GLASSINE_KXTEST is set by the probe kit's configuration, never by a system or
 * trial kernel, and every hook is off unless the command line asks for it.
 *
 *   kxtest.wdt=1       log the APSS watchdog's registers (enable, bark, bite,
 *                      status) at early_initcall, twice 1 s apart (does it count?)
 *                      and again at late_initcall (after qcom-wdt probed)
 *   kxtest.rproc=1     5 s after late_initcall, log the line state of each remote
 *                      processor's smp2p interrupts (ready, fatal, handover,
 *                      stop-ack) from the DT, without binding or requesting
 *                      anything: is the ADSP/modem the old kernel left still up?
 *   kxtest.hang=MODE   a deliberate hang at boot:
 *                        early_irqoff  early_initcall, boot CPU, interrupts off,
 *                                      before the other CPUs and before any
 *                                      watchdog driver (only the HW watchdog the
 *                                      old kernel armed can end it)
 *                        late_sleep    late_initcall: the init thread sleeps for
 *                                      ever, the kernel stays healthy (a boot that
 *                                      never reaches userspace; the watchdog core
 *                                      pings until watchdog.open_timeout)
 *                        late_panic    late_initcall: panic()
 *   kxtest.trigger=X   (sysfs, at run time) the same from userspace:
 *                        panic         panic()
 *                        irqoff_one    the highest online CPU spins with
 *                                      interrupts off (hard lockup detector)
 *                        irqoff_all    stop_machine(): every CPU spins with
 *                                      interrupts off (only the HW watchdog)
 *   kxtest.backstop=S  with a hang or trigger: emergency restart S s later from a
 *                      timer on CPU 0 (never the CPU irqoff_one takes); 0 = none.
 *                      It cannot help early_irqoff or irqoff_all.
 * Markers: "KXTEST:".
 */

#include <linux/delay.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/reboot.h>
#include <linux/stop_machine.h>
#include <linux/timer.h>
#include <linux/workqueue.h>

/* kxtest.* on the command line, /sys/module/kxtest/parameters/ at run time */
#undef MODULE_PARAM_PREFIX
#define MODULE_PARAM_PREFIX "kxtest."

static bool kx_wdt;
module_param_named(wdt, kx_wdt, bool, 0444);
static bool kx_rproc;
module_param_named(rproc, kx_rproc, bool, 0444);
static char kx_hang[16];
module_param_string(hang, kx_hang, sizeof(kx_hang), 0444);
static unsigned int kx_backstop;
module_param_named(backstop, kx_backstop, uint, 0444);

/* the kpss watchdog (qcom-wdt reg_offset_data_kpss) */
#define KX_WDT_EN	0x08
#define KX_WDT_STS	0x0c
#define KX_WDT_BARK	0x10
#define KX_WDT_BITE	0x14

static void kx_wdt_log(const char *when, bool twice)
{
	struct device_node *np;
	void __iomem *b;
	u32 sts;

	np = of_find_compatible_node(NULL, NULL, "qcom,kpss-wdt");
	if (!np) {
		pr_info("KXTEST: wdt %s: no qcom,kpss-wdt node\n", when);
		return;
	}
	b = of_iomap(np, 0);
	of_node_put(np);
	if (!b) {
		pr_info("KXTEST: wdt %s: cannot map\n", when);
		return;
	}
	sts = readl(b + KX_WDT_STS);
	pr_info("KXTEST: wdt %s: en 0x%x sts 0x%08x bark %u bite %u (ticks, 32768/s)\n",
		when, readl(b + KX_WDT_EN), sts, readl(b + KX_WDT_BARK), readl(b + KX_WDT_BITE));
	if (twice) {
		mdelay(1000);
		pr_info("KXTEST: wdt %s +1 s: en 0x%x sts 0x%08x (was 0x%08x)\n",
			when, readl(b + KX_WDT_EN), readl(b + KX_WDT_STS), sts);
	}
	iounmap(b);
}

static void kx_backstop_fn(struct timer_list *t)
{
	pr_emerg("KXTEST: backstop after %u s, restarting\n", kx_backstop);
	emergency_restart();
}
static DEFINE_TIMER(kx_backstop_timer, kx_backstop_fn);

static void kx_arm_backstop(const char *what)
{
	if (!kx_backstop)
		return;
	kx_backstop_timer.expires = jiffies + kx_backstop * HZ;
	add_timer_on(&kx_backstop_timer, cpumask_first(cpu_online_mask));
	pr_info("KXTEST: %s: backstop armed on cpu %u, %u s\n", what,
		cpumask_first(cpu_online_mask), kx_backstop);
}

static int kx_spin_all(void *unused)
{
	for (;;)
		cpu_relax();
	return 0;
}

static int kx_spin_one(void *unused)
{
	pr_emerg("KXTEST: irqoff_one: cpu %d spins with interrupts off\n", smp_processor_id());
	local_irq_disable();
	for (;;)
		cpu_relax();
	return 0;
}

static int kx_trigger_set(const char *val, const struct kernel_param *kp)
{
	if (sysfs_streq(val, "panic")) {
		kx_arm_backstop("panic");
		panic("KXTEST: panic on request");
	} else if (sysfs_streq(val, "irqoff_one")) {
		unsigned int cpu = cpumask_last(cpu_online_mask);
		struct task_struct *t;

		if (cpu == cpumask_first(cpu_online_mask))
			return -EINVAL;	/* would take the backstop's CPU */
		kx_arm_backstop("irqoff_one");
		t = kthread_create(kx_spin_one, NULL, "kxtest-spin");
		if (IS_ERR(t))
			return PTR_ERR(t);
		kthread_bind(t, cpu);
		wake_up_process(t);
	} else if (sysfs_streq(val, "irqoff_all")) {
		pr_emerg("KXTEST: irqoff_all: every CPU spins with interrupts off\n");
		stop_machine(kx_spin_all, NULL, NULL);
	} else {
		return -EINVAL;
	}
	return 0;
}

static const struct kernel_param_ops kx_trigger_ops = {
	.set = kx_trigger_set,
};
module_param_cb(trigger, &kx_trigger_ops, NULL, 0200);

static int __init kx_early(void)
{
	if (kx_wdt)
		kx_wdt_log("early_initcall", true);
	if (!strcmp(kx_hang, "early_irqoff")) {
		pr_emerg("KXTEST: hang early_irqoff: boot CPU spins with interrupts off\n");
		local_irq_disable();
		for (;;)
			cpu_relax();
	}
	return 0;
}
early_initcall(kx_early);

static const char * const kx_rproc_irqs[] = { "ready", "fatal", "handover", "stop-ack" };

static void kx_rproc_fn(struct work_struct *w)
{
	struct device_node *np;

	for_each_node_with_property(np, "interrupt-names") {
		char line[160];
		int i, n = 0;

		if (!of_device_is_compatible(np, "qcom,sdm660-adsp-pas") &&
		    !of_device_is_compatible(np, "qcom,sdm660-mss-pil"))
			continue;
		n += scnprintf(line + n, sizeof(line) - n, "%pOF:", np);
		for (i = 0; i < ARRAY_SIZE(kx_rproc_irqs); i++) {
			bool st = false;
			int irq = of_irq_get_byname(np, kx_rproc_irqs[i]);
			int r;

			if (irq <= 0) {
				n += scnprintf(line + n, sizeof(line) - n, " %s=?(%d)",
					       kx_rproc_irqs[i], irq);
				continue;
			}
			r = irq_get_irqchip_state(irq, IRQCHIP_STATE_LINE_LEVEL, &st);
			n += scnprintf(line + n, sizeof(line) - n, " %s=%s", kx_rproc_irqs[i],
				       r ? "err" : st ? "1" : "0");
		}
		pr_info("KXTEST: rproc %s\n", line);
	}
}
static DECLARE_DELAYED_WORK(kx_rproc_work, kx_rproc_fn);

static int __init kx_late(void)
{
	if (kx_wdt)
		kx_wdt_log("late_initcall", false);
	if (kx_rproc)
		schedule_delayed_work(&kx_rproc_work, 5 * HZ);
	if (!strcmp(kx_hang, "late_panic")) {
		kx_arm_backstop("late_panic");
		panic("KXTEST: hang late_panic");
	}
	if (!strcmp(kx_hang, "late_sleep")) {
		kx_arm_backstop("late_sleep");
		pr_emerg("KXTEST: hang late_sleep: the init thread sleeps for ever\n");
		for (;;)
			msleep(1000);
	}
	return 0;
}
late_initcall(kx_late);
