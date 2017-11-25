/*
 * lge_handle_panic.c
 *
 * Copyright (C) 2014 LGE, Inc
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/reboot.h>
#include <linux/io.h>
#include <linux/init.h>
#include <linux/memblock.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/cpu.h>
#include <linux/delay.h>
#include <asm/setup.h>
#include <soc/qcom/subsystem_restart.h>
#include <soc/qcom/lge/lge_handle_panic.h>
#include <soc/qcom/scm.h>
#include <linux/input.h>
#ifdef CONFIG_LGE_BOOT_LOCKUP_DETECT
#include <linux/reboot.h>
#include <soc/qcom/lge/board_lge.h>
#endif

#if defined(CONFIG_ARCH_MSM8909)
#define NO_ADSP
#endif


#define PANIC_HANDLER_NAME        "panic-handler"

#define RESTART_REASON_ADDR       0x65C
#define DLOAD_MODE_ADDR           0x0
#define CRASH_HANDLER_DATA_ADDR   0x3C

#define CRASH_HANDLER_MAGIC_VALUE 0x4c474500
#define CRASH_HANDLER_MAGIC_ADDR  (CRASH_HANDLER_DATA_ADDR)
#define RAM_CONSOLE_ADDR_ADDR     (CRASH_HANDLER_DATA_ADDR + 0x4)
#define RAM_CONSOLE_SIZE_ADDR     (CRASH_HANDLER_DATA_ADDR + 0x8)
#define FB_ADDR_ADDR              (CRASH_HANDLER_DATA_ADDR + 0xC)

#define RESTART_REASON      (msm_imem_base + RESTART_REASON_ADDR)
#define CRASH_HANDLER_MAGIC (msm_imem_base + CRASH_HANDLER_MAGIC_ADDR)
#define RAM_CONSOLE_ADDR    (msm_imem_base + RAM_CONSOLE_ADDR_ADDR)
#define RAM_CONSOLE_SIZE    (msm_imem_base + RAM_CONSOLE_SIZE_ADDR)
#define FB_ADDR             (msm_imem_base + FB_ADDR_ADDR)

static void *msm_imem_base;
static int dummy_arg;

static int subsys_crash_magic = 0x0;

static struct panic_handler_data *panic_handler;

#define KEY_CRASH_TIMEOUT 1000
static int gen_key_panic = 0;
static int key_crash_cnt = 0;
static unsigned long key_crash_last_time = 0;
static DEFINE_SPINLOCK(lge_panic_lock);
static unsigned int sctrl_reg, ttbr0_reg, ttbr1_reg, ttbcr_reg;

void lge_get_mmu_cp15_register(void)
{
	asm volatile ("mrc p15, 0, %0, c1, c0, 0\n" : "=r" (sctrl_reg));
	asm volatile ("mrc p15, 0, %0, c2, c0, 0\n" : "=r" (ttbr0_reg));
	asm volatile ("mrc p15, 0, %0, c2, c0, 1\n" : "=r" (ttbr1_reg));
	asm volatile ("mrc p15, 0, %0, c2, c0, 2\n" : "=r" (ttbcr_reg));

	printk("SCTRL: %08x  TTBR0: %08x TTBR1: %08x  TTBCR: %08x \n", sctrl_reg, ttbr0_reg, ttbr1_reg, ttbcr_reg);
}

void lge_set_subsys_crash_reason(const char *name, int type)
{
	const char *subsys_name[] = { "adsp", "mba", "modem", "wcnss", "venus" };
	int i = 0;

	if (!name)
		return;

	for (i = 0; i < ARRAY_SIZE(subsys_name); i++) {
		if (!strncmp(subsys_name[i], name, 40)) {
			subsys_crash_magic = LGE_RB_MAGIC | ((i+1) << 12)
					| type;
			break;
		}
	}

	return;
}

void lge_set_ram_console_addr(unsigned int addr, unsigned int size)
{
	__raw_writel(addr, RAM_CONSOLE_ADDR);
	__raw_writel(size, RAM_CONSOLE_SIZE);
}

void lge_set_fb_addr(unsigned int addr)
{
	__raw_writel(addr, FB_ADDR);
}

void lge_set_restart_reason(unsigned int reason)
{
	__raw_writel(reason, RESTART_REASON);
}

void lge_set_panic_reason(void)
{
	if (lge_get_download_mode() && gen_key_panic) {
		lge_set_restart_reason(LGE_RB_MAGIC | LGE_ERR_KERN | LGE_ERR_KEY);
		return;
	}

	if (subsys_crash_magic == 0)
		lge_set_restart_reason(LGE_RB_MAGIC | LGE_ERR_KERN);
	else
		lge_set_restart_reason(subsys_crash_magic);
}

inline static void lge_set_key_crash_cnt(int key, int* clear)
{
	unsigned long cur_time = 0;
	unsigned long key_crash_gap = 0;

	cur_time = jiffies_to_msecs(jiffies);
	key_crash_gap = cur_time - key_crash_last_time;

	if ((key_crash_cnt != 0) && (key_crash_gap > KEY_CRASH_TIMEOUT)) {
		pr_debug("%s: Ready to panic %d : over time %ld!\n", __func__, key, key_crash_gap);
		return;
	}

	*clear = 0;
	key_crash_cnt++;
	key_crash_last_time = cur_time;

	pr_info("%s: Ready to panic %d : count %d, time gap %ld!\n", __func__, key, key_crash_cnt, key_crash_gap);
}

void lge_gen_key_panic(int key, int status)
{
	int clear = 1;
	int order = key_crash_cnt % 3;
	static int valid = 0;

	if(lge_get_download_mode() != 1)
		return;

	if(((key == KEY_VOLUMEDOWN) && (order == 0))
		|| ((key == KEY_POWER) && (order == 1))
		|| ((key == KEY_VOLUMEUP) && (order == 2))) {
		if(status == 0) {
			if(valid == 1) {
				valid = 0;
				lge_set_key_crash_cnt(key, &clear);
			}
		} else {
			valid = 1;
			return;
		}
	}

	if (clear == 1) {
		if (valid == 1)
			valid = 0;

		if (key_crash_cnt > 0)
			key_crash_cnt = 0;

		pr_debug("%s: Ready to panic %d : cleared!\n", __func__, key);
		return;
	}

	if (key_crash_cnt >= 7) {
		gen_key_panic = 1;
		panic("%s: Generate panic by key!\n", __func__);
	}
}

static int gen_bug(const char *val, struct kernel_param *kp)
{
	BUG();
	return 0;
}
module_param_call(gen_bug, gen_bug, param_get_bool, &dummy_arg,
		S_IWUSR | S_IRUGO);

static int gen_panic(const char *val, struct kernel_param *kp)
{
	panic("generate test-panic");
	return 0;
}
module_param_call(gen_panic, gen_panic, param_get_bool, &dummy_arg,
		S_IWUSR | S_IRUGO);

#if !defined(NO_ADSP)
static int gen_adsp_panic(const char *val, struct kernel_param *kp)
{
	subsystem_restart("adsp");
	return 0;
}
module_param_call(gen_adsp_panic, gen_adsp_panic, param_get_bool, &dummy_arg,
		S_IWUSR | S_IRUGO);
#endif

static int gen_mba_panic(const char *val, struct kernel_param *kp)
{
	subsystem_restart("mba");
	return 0;
}
module_param_call(gen_mba_panic, gen_mba_panic, param_get_bool, &dummy_arg,
		S_IWUSR | S_IRUGO);

static int gen_modem_panic(const char *val, struct kernel_param *kp)
{
	subsystem_restart("modem");
	return 0;
}
module_param_call(gen_modem_panic, gen_modem_panic, param_get_bool, &dummy_arg,
		S_IWUSR | S_IRUGO);

static int gen_wcnss_panic(const char *val, struct kernel_param *kp)
{
	subsystem_restart("wcnss");
	return 0;
}
module_param_call(gen_wcnss_panic, gen_wcnss_panic, param_get_bool, &dummy_arg,
		S_IWUSR | S_IRUGO);

static int gen_venus_panic(const char *val, struct kernel_param *kp)
{
	subsystem_restart("venus");
	return 0;
}
module_param_call(gen_venus_panic, gen_venus_panic, param_get_bool, &dummy_arg,
		S_IWUSR | S_IRUGO);

#define WDT0_RST        0x04
#define WDT0_EN         0x08
#define WDT0_BARK_TIME  0x10
#define WDT0_BITE_TIME  0x14

extern void __iomem *wdt_timer_get_timer0_base(void);

static int gen_wdt_bark(const char *val, struct kernel_param *kp)
{
	void __iomem *msm_tmr0_base;
	msm_tmr0_base = wdt_timer_get_timer0_base();

	pr_info("%s\n", __func__);
	writel_relaxed(0, msm_tmr0_base + WDT0_EN);
	writel_relaxed(1, msm_tmr0_base + WDT0_RST);
	writel_relaxed(0x31F3, msm_tmr0_base + WDT0_BARK_TIME);
	writel_relaxed(5 * 0x31F3, msm_tmr0_base + WDT0_BITE_TIME);
	writel_relaxed(1, msm_tmr0_base + WDT0_EN);
	mb();
	mdelay(5000);

	pr_err("%s failed\n", __func__);

	return -EIO;
}
module_param_call(gen_wdt_bark, gen_wdt_bark, param_get_bool,
		&dummy_arg, S_IWUSR | S_IRUGO);

static int gen_wdt_bite(const char *val, struct kernel_param *kp)
{
	void __iomem *msm_tmr0_base;
	msm_tmr0_base = wdt_timer_get_timer0_base();

	pr_info("%s\n", __func__);
	writel_relaxed(0, msm_tmr0_base + WDT0_EN);
	writel_relaxed(1, msm_tmr0_base + WDT0_RST);
	writel_relaxed(5 * 0x31F3, msm_tmr0_base + WDT0_BARK_TIME);
	writel_relaxed(0x31F3, msm_tmr0_base + WDT0_BITE_TIME);
	writel_relaxed(1, msm_tmr0_base + WDT0_EN);
	mb();
	mdelay(5000);

	pr_err("%s failed\n", __func__);

	return -EIO;
}
module_param_call(gen_wdt_bite, gen_wdt_bite, param_get_bool,
		&dummy_arg, S_IWUSR | S_IRUGO);

#define REG_MPM2_WDOG_BASE            0x004AA000
#define REG_OFFSET_MPM2_WDOG_RESET    0x0
#define REG_OFFSET_MPM2_WDOG_BITE_VAL 0x10

#define REG_VAL_WDOG_RESET_DO_RESET   0x1
#define REG_VAL_WDOG_BITE_VAL         0x400

#define SCM_SVC_SEC_WDOG_TRIG         0x8

static int gen_sec_wdt_scm(const char *val, struct kernel_param *kp)
{
	struct scm_desc desc;
	desc.args[0] = 0;
	desc.arginfo = SCM_ARGS(1);

	pr_info("%s\n", __func__);
	scm_call2_atomic(SCM_SIP_FNID(SCM_SVC_BOOT,
			SCM_SVC_SEC_WDOG_TRIG), &desc);

	pr_err("%s failed\n", __func__);

	return -EIO;
}
module_param_call(gen_sec_wdt_scm, gen_sec_wdt_scm, param_get_bool,
		&dummy_arg, S_IWUSR | S_IRUGO);

static int gen_sec_wdt_bite(const char *val, struct kernel_param *kp)
{
	void *sec_wdog_virt;
	sec_wdog_virt = ioremap(REG_MPM2_WDOG_BASE, SZ_4K);

	if (!sec_wdog_virt) {
		pr_err("unable to map sec wdog page\n");
		return -ENOMEM;
	}

	pr_info("%s\n", __func__);
	writel_relaxed(REG_VAL_WDOG_RESET_DO_RESET,
		sec_wdog_virt + REG_OFFSET_MPM2_WDOG_RESET);
	writel_relaxed(REG_VAL_WDOG_BITE_VAL,
		sec_wdog_virt + REG_OFFSET_MPM2_WDOG_BITE_VAL);
	mb();
	mdelay(5000);

	pr_err("%s failed\n", __func__);
	iounmap(sec_wdog_virt);

	return -EIO;
}
module_param_call(gen_sec_wdt_bite, gen_sec_wdt_bite, param_get_bool,
		&dummy_arg, S_IWUSR | S_IRUGO);

void lge_disable_watchdog(void)
{
	static int once = 1;
	void __iomem *msm_tmr0_base;

	if (once > 1)
		return;

	msm_tmr0_base = wdt_timer_get_timer0_base();
	if (!msm_tmr0_base)
		return;

	writel_relaxed(0, msm_tmr0_base + WDT0_EN);
	mb();
	once++;

	pr_info("%s\n", __func__);
}

void lge_panic_handler_fb_free_page(unsigned long mem_addr, unsigned long size)
{
	unsigned long pfn_start, pfn_end, pfn_idx;

	pfn_start = mem_addr >> PAGE_SHIFT;
	pfn_end = (mem_addr + size) >> PAGE_SHIFT;
	for (pfn_idx = pfn_start; pfn_idx < pfn_end; pfn_idx++)
		free_reserved_page(pfn_to_page(pfn_idx));
}

void lge_panic_handler_fb_cleanup(void)
{
	static int free = 1;

	if (!panic_handler || free > 1)
		return;

	if (panic_handler->fb_addr && panic_handler->fb_size) {
		memblock_free(panic_handler->fb_addr, panic_handler->fb_size);
		lge_panic_handler_fb_free_page(
			panic_handler->fb_addr, panic_handler->fb_size);
		free++;

		pr_info("%s: free[@0x%lx+@0x%lx)\n", PANIC_HANDLER_NAME,
			panic_handler->fb_addr, panic_handler->fb_size);
	}
}

#ifdef CONFIG_LGE_BOOT_LOCKUP_DETECT
#define LOCKUP_WQ_NOT_START_YET (-1)
#define LOCKUP_WQ_PAUSED        (0)
#define LOCKUP_WQ_STARTED       (1)
#define LOCKUP_WQ_CANCELED      (2)

#define REBOOT_DEADLINE msecs_to_jiffies(30 * 1000)

static struct delayed_work lge_panic_reboot_work;

static void lge_panic_reboot_work_func(struct work_struct *work)
{
    pr_emerg("==========================================================\n");
	pr_emerg("WARNING: detecting lockup during reboot! forcing panic....\n");
	pr_emerg("==========================================================\n");

	BUG();
}

static int lge_panic_reboot_handler(struct notifier_block *this,
		unsigned long event, void *ptr)
{
	if (lge_get_download_mode() != 1)
		return NOTIFY_DONE;

	INIT_DELAYED_WORK(&lge_panic_reboot_work, lge_panic_reboot_work_func);
	queue_delayed_work(system_highpri_wq, &lge_panic_reboot_work,
		   REBOOT_DEADLINE);

	return NOTIFY_DONE;
}

static struct notifier_block lge_panic_reboot_notifier = {
	lge_panic_reboot_handler,
	NULL,
	0
};
#endif

static int lge_handler_panic(struct notifier_block *this,
                 unsigned long event,
                 void *ptr)
{
	unsigned long flags;
	spin_lock_irqsave(&lge_panic_lock, flags);

	printk(KERN_CRIT "%s called\n", __func__);
	lge_get_mmu_cp15_register();

	spin_unlock_irqrestore(&lge_panic_lock, flags);
	return NOTIFY_DONE;
}

static struct notifier_block lge_panic_blk = {
    .notifier_call  = lge_handler_panic,
    .priority    = 1004,
};

static int __init lge_panic_handler_early_init(void)
{
	struct device_node *np;
	uint32_t crash_handler_magic = 0;
	uint32_t mem_addr = 0;
	uint32_t mem_size = 0;
#ifdef CONFIG_LGE_BOOT_LOCKUP_DETECT
	int ret = 0;
#endif
	panic_handler = kzalloc(sizeof(*panic_handler), GFP_KERNEL);
	if (!panic_handler) {
		pr_err("could not allocate memory for panic_handler\n");
		return -ENOMEM;
	}

	np = of_find_compatible_node(NULL, NULL, "qcom,msm-imem");
	if (!np) {
		pr_err("unable to find DT imem node\n");
		return -ENODEV;
	}
	msm_imem_base = of_iomap(np, 0);
	if (!msm_imem_base) {
		pr_err("unable to map imem\n");
		return -ENODEV;
	}

	/* Set default restart_reason to hw reset. */
	lge_set_restart_reason(LGE_RB_MAGIC | LGE_ERR_TZ);

	np = of_find_compatible_node(NULL, NULL, "crash_fb");
	if (!np) {
		pr_err("unable to find crash_fb node\n");
		return -ENODEV;
	}

	of_property_read_u32(np, "mem-addr", (u32*)&panic_handler->fb_addr);
	of_property_read_u32(np, "mem-size", (u32*)&panic_handler->fb_size);

	pr_info("%s: reserved[@0x%lx+@0x%lx)\n", PANIC_HANDLER_NAME,
		panic_handler->fb_addr, panic_handler->fb_size);

	lge_set_fb_addr(panic_handler->fb_addr);

	np = of_find_compatible_node(NULL, NULL, "ramoops");
#ifdef CONFIG_LGE_BOOT_LOCKUP_DETECT
	/* register reboot notifier for detecting reboot lockup */
	ret = register_reboot_notifier(&lge_panic_reboot_notifier);
#endif
	if (!np) {
		pr_err("unable to find DT ramoops node\n");
		return -ENODEV;
	}

	atomic_notifier_chain_register(&panic_notifier_list, &lge_panic_blk);

	of_property_read_u32(np, "mem-address", &mem_addr);
	of_property_read_u32(np, "mem-size", &mem_size);
	pr_info("mem-address=0x%x\n", mem_addr);
	pr_info("mem-size=0x%x\n", mem_size);
	lge_set_ram_console_addr(mem_addr, mem_size);

	/* check struct boot_shared_imem_cookie_type is matched */
	crash_handler_magic = __raw_readl(CRASH_HANDLER_MAGIC);
	WARN(crash_handler_magic != CRASH_HANDLER_MAGIC_VALUE,
			"Check sbl's struct boot_shared_imem_cookie_type.\n"
			"Need to update lge_handle_panic's imem offset.\n");

	return 0;
}

early_initcall(lge_panic_handler_early_init);

static int __init lge_panic_handler_probe(struct platform_device *pdev)
{
	int ret = 0;

	return ret;
}

static int lge_panic_handler_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver panic_handler_driver __refdata = {
	.probe = lge_panic_handler_probe,
	.remove = lge_panic_handler_remove,
	.driver = {
		.name = PANIC_HANDLER_NAME,
		.owner = THIS_MODULE,
	},
};

static int __init lge_panic_handler_init(void)
{
	return platform_driver_register(&panic_handler_driver);
}

static void __exit lge_panic_handler_exit(void)
{
	platform_driver_unregister(&panic_handler_driver);
}

module_init(lge_panic_handler_init);
module_exit(lge_panic_handler_exit);

MODULE_DESCRIPTION("LGE panic handler driver");
MODULE_AUTHOR("SungEun Kim <cleaneye.kim@lge.com>");
MODULE_LICENSE("GPL");
