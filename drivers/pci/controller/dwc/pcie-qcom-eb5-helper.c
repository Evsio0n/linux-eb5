// SPDX-License-Identifier: GPL-2.0
/*
 * pcie-qcom-eb5-helper.c - PCIe1 ASM2806 bridge bring-up helper for QRB5165 EB5
 *
 * The Qualcomm vendor 4.19 kernel has a private "use-pcie-bridge-asm2806" DT
 * property that drives gpio141 (ASM2806 bridge-enable) before the RC1 PCIe
 * controller enumerates the bus.  Mainline qcom-pcie has no such logic.
 *
 * This driver:
 *   1. Acquires gpio141 and drives it HIGH on probe (before qcom-pcie touches
 *      PERST#).
 *   2. Registers itself as a dummy fixed-rate clock provider (#clock-cells = <0>),
 *      which qrb5165-eb5.dts adds to pcie1's clock list.  fw_devlink sees this
 *      phandle and guarantees that pcie1 will NOT be probed until this driver's
 *      probe() returns successfully.
 *   3. Schedules a deferred rescan after the ASM2806 cascade downstream links
 *      finish training.  The rescan explicitly programs bridge MEMORY_BASE/LIMIT
 *      registers to hardware — pci_assign_unassigned_bus_resources() only updates
 *      kernel data structures but does NOT write the bridge window registers,
 *      which would leave the RTL8168 endpoints inaccessible.
 *
 * Result: gpio141 is driven high BEFORE qcom_pcie_host_init() de-asserts PERST#,
 * so the ASM2806 bridge is powered and ready for config-space enumeration.
 *
 * Nothing in qcom.c / pcie-qcom.c is modified.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/clk-provider.h>
#include <linux/clk.h>
#include <linux/pci.h>
#include <linux/workqueue.h>
#include <linux/io.h>

/* PCI domain 1, root bus 0 — matches pcie1 (1c08000.pcie) */
#define EB5_PCIE1_DOMAIN	1
#define EB5_PCIE1_ROOT_BUS	0

/*
 * Poll interval while waiting for pcie1 to create its root bus.
 * pcie1 probe can take 10-15 s at boot; we retry up to 30 times (30 s).
 */
#define EB5_POLL_INTERVAL_MS	1000
#define EB5_POLL_MAX_RETRIES	30

#define DRV_NAME "qcom-eb5-pcie1-helper"

/*
 * Absolute physical address of pcie1 PARF_BDF_TO_SID_CFG.
 * From sm8250.dtsi: pcie1 reg[0] = <0 0x01c08000 0 0x3000> (reg-names = "parf").
 * PARF_BDF_TO_SID_CFG offset = 0x2c00  =>  0x01c08000 + 0x2c00 = 0x01c0ac00.
 *
 * BIT(0) = BDF_TO_SID_BYPASS: when set, inbound PCIe completions skip the
 * SMMU BDF-to-SID lookup.  Hardware power-on default is 0 (bypass OFF);
 * we must assert it explicitly before any config-space reads reach the RC.
 */
#define PARF_BDF_TO_SID_CFG_PHYS	0x01c0ac00

struct eb5_pcie_helper {
	struct gpio_desc    *lan_en_gpio;
	struct clk_hw        clk_hw;
	struct delayed_work  rescan_work;
};

static const struct clk_ops eb5_lan_clk_ops = { /* no-op clock, ordering only */ };

/*
 * program_bridge_windows - write bridge memory windows to hardware registers.
 *
 * pci_assign_unassigned_bus_resources() assigns memory windows in the kernel's
 * resource tree but does NOT write PCI_MEMORY_BASE / PCI_MEMORY_LIMIT to the
 * bridge's config space.  Without this step the bridge does not forward memory
 * transactions downstream, so endpoint drivers fail on their very first MMIO
 * access.  Recurse into child buses so all levels of ASM2806 are programmed.
 */
static void program_bridge_windows(struct pci_bus *bus)
{
	struct pci_dev *dev;

	list_for_each_entry(dev, &bus->devices, bus_list) {
		struct resource *res;
		u16 cmd;

		if (!dev->subordinate)
			continue;

		/* Write 32-bit non-prefetchable memory window */
		res = &dev->resource[PCI_BRIDGE_MEM_WINDOW];
		if (resource_size(res) > 0) {
			u16 mem_base  = (res->start >> 16) & 0xfff0;
			u16 mem_limit = (res->end   >> 16) & 0xfff0;

			pci_write_config_word(dev, PCI_MEMORY_BASE,  mem_base);
			pci_write_config_word(dev, PCI_MEMORY_LIMIT, mem_limit);
			dev_info(&dev->dev, "bridge mem window programmed: %pR\n", res);
		}

		/* Write I/O window if present */
		res = &dev->resource[PCI_BRIDGE_IO_WINDOW];
		if (resource_size(res) > 0) {
			pci_write_config_byte(dev, PCI_IO_BASE,
					      (res->start >> 8) & 0xf0);
			pci_write_config_byte(dev, PCI_IO_LIMIT,
					      (res->end   >> 8) & 0xf0);
		}

		/* Enable bus-mastering and memory-space decoding on the bridge */
		pci_read_config_word(dev, PCI_COMMAND, &cmd);
		cmd |= PCI_COMMAND_MEMORY | PCI_COMMAND_MASTER;
		pci_write_config_word(dev, PCI_COMMAND, cmd);

		program_bridge_windows(dev->subordinate);
	}
}

static void eb5_pcie1_rescan_work(struct work_struct *work)
{
	struct eb5_pcie_helper *h =
		container_of(work, struct eb5_pcie_helper, rescan_work.work);
	struct pci_bus *root_bus;
	int retries = 0;

	/* Re-assert gpio141 in case it was reset during suspend/resume */
	gpiod_set_value_cansleep(h->lan_en_gpio, 1);

	/*
	 * pcie1 probe (and bus creation) can take 10-15 s at boot — much
	 * longer than the helper's own probe.  Poll until the root bus
	 * appears, then give the ASM2806 downstream links an extra second
	 * to finish training before we scan.
	 */
	while (retries < EB5_POLL_MAX_RETRIES) {
		root_bus = pci_find_bus(EB5_PCIE1_DOMAIN, EB5_PCIE1_ROOT_BUS);
		if (root_bus)
			break;
		pr_debug(DRV_NAME ": waiting for pcie1 bus (attempt %d/%d)\n",
			 retries + 1, EB5_POLL_MAX_RETRIES);
		msleep(EB5_POLL_INTERVAL_MS);
		retries++;
	}

	if (!root_bus) {
		pr_err(DRV_NAME ": domain %u bus %02x not found after %d s, giving up\n",
		       EB5_PCIE1_DOMAIN, EB5_PCIE1_ROOT_BUS, EB5_POLL_MAX_RETRIES);
		return;
	}

	/* Extra settling time for ASM2806 downstream link training */
	msleep(1000);

	pr_info(DRV_NAME ": rescanning pcie1 (domain %u bus %02x) after %d poll(s)\n",
		EB5_PCIE1_DOMAIN, EB5_PCIE1_ROOT_BUS, retries);

	/*
	 * Re-assert BDF_TO_SID_BYPASS before SBR in case qcom_pcie_host_init()
	 * cleared it between probe and now (e.g. via a wake/suspend cycle).
	 */
	{
		void __iomem *parf_sid = ioremap(PARF_BDF_TO_SID_CFG_PHYS, 4);

		if (parf_sid) {
			writel(BIT(0), parf_sid);
			iounmap(parf_sid);
			pr_info(DRV_NAME ": BDF_TO_SID_BYPASS re-asserted before SBR\n");
		}
	}

	/*
	 * Issue a Secondary Bus Reset (SBR) via the root port's Bridge Control
	 * register.  This pulses the downstream PERST# from the RC side without
	 * needing direct GPIO access.  The ASM2806 may have been unresponsive
	 * during the initial enumeration because its internal init was not yet
	 * complete when qcom-pcie first de-asserted PERST# at probe time.
	 * Driving SBR here — after gpio141 has been high for several seconds —
	 * gives the ASM2806 a clean reset cycle with power already stable.
	 */
	{
		struct pci_dev *rp = pci_get_domain_bus_and_slot(
					EB5_PCIE1_DOMAIN, 0, PCI_DEVFN(0, 0));
		if (rp) {
			u16 bctl;

			pci_read_config_word(rp, PCI_BRIDGE_CONTROL, &bctl);
			/* Assert Secondary Bus Reset */
			pci_write_config_word(rp, PCI_BRIDGE_CONTROL,
					      bctl | PCI_BRIDGE_CTL_BUS_RESET);
			msleep(100); /* hold reset ≥ 100 ms (PCIe r3.0 §6.6.1) */
			/* De-assert Secondary Bus Reset */
			pci_write_config_word(rp, PCI_BRIDGE_CONTROL, bctl);
			msleep(500); /* wait for ASM2806 to finish link re-training */
			dev_info(&rp->dev,
				 DRV_NAME ": SBR pulse done, waiting for ASM2806\n");
			pci_dev_put(rp);
		} else {
			pr_warn(DRV_NAME ": root port not found, skipping SBR\n");
		}
	}

	pci_lock_rescan_remove();

	/* Step 1: discover new devices (ASM2806 cascade + RTL8168) */
	pci_scan_child_bus(root_bus);

	/* Step 2: assign BARs and bridge windows in kernel resource structs */
	pci_assign_unassigned_bus_resources(root_bus);

	/*
	 * Step 3: write bridge MEMORY_BASE/LIMIT to hardware config space.
	 * This is the step that pci_rescan_bus() / pci_assign_…() omit,
	 * and without it the RTL8168 endpoints are unreachable via MMIO.
	 */
	program_bridge_windows(root_bus);

	/* Step 4: add devices to driver model — triggers driver probes */
	pci_bus_add_devices(root_bus);

	pci_unlock_rescan_remove();

	pr_info(DRV_NAME ": rescan complete, bridges programmed\n");
}

/* Forward declarations for sysfs attribute */
static ssize_t reset_asm2806_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count);
static DEVICE_ATTR_WO(reset_asm2806);

static int eb5_pcie_helper_probe(struct platform_device *pwd)
{
	struct device *dev = &pwd->dev;
	struct eb5_pcie_helper *h;
	struct clk_init_data init = {};
	int ret;

	h = devm_kzalloc(dev, sizeof(*h), GFP_KERNEL);
	if (!h)
		return -ENOMEM;

	/*
	 * Acquire gpio141 (ASM2806 bridge-enable) and drive it HIGH.
	 * This must happen before qcom-pcie de-asserts PERST# on RC1.
	 * The clock provider registration below ensures that qcom-pcie
	 * does not even begin probing until after this point.
	 */
	h->lan_en_gpio = devm_gpiod_get(dev, "lan-en", GPIOD_OUT_HIGH);
	if (IS_ERR(h->lan_en_gpio))
		return dev_err_probe(dev, PTR_ERR(h->lan_en_gpio),
				 "failed to get lan-en gpio\n");

	msleep(100); /* allow ASM2806 power rails to stabilize */
	dev_info(dev, "lan-en (gpio141) asserted high\n");

	/*
	 * Register a zero-rate fixed clock so that fw_devlink can enforce
	 * the probe ordering: pcie1 (which lists us in its 'clocks') will
	 * not probe before of_clk_add_hw_provider() returns.
	 */
	init.name = DRV_NAME;
	init.ops  = &eb5_lan_clk_ops;
	h->clk_hw.init = &init;

	ret = devm_clk_hw_register(dev, &h->clk_hw);
	if (ret)
		return dev_err_probe(dev, ret, "clk_hw_register failed\n");

	ret = devm_of_clk_add_hw_provider(dev, of_clk_hw_simple_get, &h->clk_hw);
	if (ret)
		return dev_err_probe(dev, ret,
				 "of_clk_add_hw_provider failed\n");

	/* Start polling immediately; the work itself waits for the bus */
	INIT_DELAYED_WORK(&h->rescan_work, eb5_pcie1_rescan_work);
	schedule_delayed_work(&h->rescan_work, 0);

	platform_set_drvdata(pwd, h);
	ret = device_create_file(dev, &dev_attr_reset_asm2806);
	if (ret)
		dev_warn(dev, "failed to create reset_asm2806 sysfs: %d\n", ret);

	dev_info(dev, "clock provider registered, waiting for pcie1 bus\n");
	return 0;
}

static void eb5_pcie_helper_remove(struct platform_device *pdev)
{
	struct eb5_pcie_helper *h = platform_get_drvdata(pdev);

	cancel_delayed_work_sync(&h->rescan_work);
	device_remove_file(&pdev->dev, &dev_attr_reset_asm2806);
}

static const struct of_device_id eb5_pcie_helper_of_match[] = {
	{ .compatible = "qcom,eb5-pcie1-helper" },
	{}
};
MODULE_DEVICE_TABLE(of, eb5_pcie_helper_of_match);

static struct platform_driver eb5_pcie_helper_driver = {
	.probe          = eb5_pcie_helper_probe,
	.remove         = eb5_pcie_helper_remove,
	.driver = {
		.name           = DRV_NAME,
		.of_match_table = eb5_pcie_helper_of_match,
	},
};
builtin_platform_driver(eb5_pcie_helper_driver);

/* Sysfs interface to trigger full reset sequence */
static ssize_t reset_asm2806_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct eb5_pcie_helper *h = dev_get_drvdata(dev);
	void __iomem *parf;
	struct pci_bus *root_bus;

	dev_info(dev, "triggering full ASM2806 reset sequence...\n");

	/* 1. Power cycle gpio141 */
	gpiod_set_value_cansleep(h->lan_en_gpio, 0);
	msleep(200);
	gpiod_set_value_cansleep(h->lan_en_gpio, 1);
	msleep(500);

	/* 2. Write bypass bit */
	parf = ioremap(PARF_BDF_TO_SID_CFG_PHYS, 4);
	if (parf) {
		writel(BIT(0), parf);
		dev_info(dev, "BDF_TO_SID_BYPASS set\n");
		iounmap(parf);
	}

	/* 3. Trigger SBR */
	root_bus = pci_find_bus(EB5_PCIE1_DOMAIN, EB5_PCIE1_ROOT_BUS);
	if (root_bus) {
		struct pci_dev *rp = pci_get_domain_bus_and_slot(
			EB5_PCIE1_DOMAIN, 0, PCI_DEVFN(0, 0));
		if (rp) {
			u16 bctl;
			pci_read_config_word(rp, PCI_BRIDGE_CONTROL, &bctl);
			pci_write_config_word(rp, PCI_BRIDGE_CONTROL,
					      bctl | PCI_BRIDGE_CTL_BUS_RESET);
			msleep(100);
			pci_write_config_word(rp, PCI_BRIDGE_CONTROL, bctl);
			msleep(500);
			pci_dev_put(rp);
			dev_info(dev, "SBR done\n");
		}
	}

	/* 4. Rescan */
	pci_lock_rescan_remove();
	pci_scan_child_bus(root_bus);
	pci_bus_add_devices(root_bus);
	pci_unlock_rescan_remove();

	dev_info(dev, "reset sequence complete\n");
	return count;
}
