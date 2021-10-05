// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2007-2019 Advanced Micro Devices, Inc.
 * Author: Nathan Fontenot <nathan.fontenot@amd.com>
 * Maintainer: Nathan Fontenot <nathan.fontenot@amd.com>
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/ktime.h>
#include <linux/timekeeping.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/sysfs.h>
#include <linux/slab.h>
#include <linux/cpu.h>
#include <linux/kobject.h>
#include <linux/errno.h>
#include <linux/processor.h>
#include <linux/topology.h>
#include "amd_hsmp.h"

#define DRV_MODULE_DESCRIPTION  "AMD Host System Management Port Test driver"
#define DRV_MODULE_VERSION      "0.1"

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Nathan Fontenot <nathan.fontenot@amd.com");
MODULE_DESCRIPTION(DRV_MODULE_DESCRIPTION);
MODULE_VERSION(DRV_MODULE_VERSION);

static int pass, fail, tbd;

static void do_simple_read_test(const char *name, int (*func)(int, u32 *))
{
	int rc;
	u32 val;

	/* Proper socket */
	pr_info("Reading %s for socket 0\n", name);
	rc = func(0, &val);
	if (rc) {
		fail++;
		pr_err("Reading %s returned %d\n", name, rc);
	} else {
		pass++;
	}

	/* Invalid socket */
	pr_info("Reading %s for socket 5\n", name);
	rc = func(5, &val);
	if (rc == -ENODEV) {
		pass++;
	} else {
		pr_err("Reading %s returned %d\n", name, rc);
		fail++;
	}

	/* Invalid return value */
	pr_info("Reading %s with invalid pointer\n", name);
	rc = func(0, NULL);
	if (rc == -EINVAL) {
		pass++;
	} else {
		pr_err("Reading %s returned %d\n", name, rc);
		fail++;
	}
}

static void do_xgmi_test(void)
{
	int speed, pstate;
	int rc;

	pr_info("Reading xGMI pstate\n");
	rc = amd_get_xgmi_pstate(&pstate);
	/* Ensure xgmi_* is supported */
	if (rc == -ENODEV) {
		pr_info("xGMI pstate not supported\n");
		return;
	}

	/* Now check for errors */
	if (rc) {
		pr_err("Reading xGMI pstate returned %d\n", rc);
		fail++;
	} else {
		pass++;
	}

	pr_info("Reading xGMI pstate with invalid pointer\n");
	rc = amd_get_xgmi_pstate(NULL);
	if (rc != -EINVAL) {
		pr_err("reading xGMI pstate returned %d\n", rc);
		fail++;
	} else {
		pass++;
	}

	pr_info("Reading xGMI speed\n");
	rc = amd_get_xgmi_speed(&speed);
	if (rc) {
		pr_err("Reading xGMI speed returned %d\n", rc);
		fail++;
	} else {
		pass++;
	}

	pr_info("Reading xGMI speed with invalid pointer\n");
	rc = amd_get_xgmi_speed(NULL);
	if (rc != -EINVAL) {
		pr_err("reading xGMI speed returned %d\n", rc);
		fail++;
	} else {
		pass++;
	}

	pr_info("Setting xGMI pstate to 0\n");
	rc = hsmp_set_xgmi_pstate(0);
	if (rc) {
		pr_err("Setting xGMI pstate returned %d\n", rc);
		fail++;
	} else {
		pass++;
	}

	pr_info("Setting xGMI pstate to 1\n");
	rc = hsmp_set_xgmi_pstate(1);
	if (rc) {
		pr_err("Setting xGMI pstate returned %d\n", rc);
		fail++;
	} else {
		pass++;
	}

	pr_info("Setting xGMI pstate to -1\n");
	rc = hsmp_set_xgmi_pstate(-1);
	if (rc) {
		pr_err("Setting xGMI pstate returned %d\n", rc);
		fail++;
	} else {
		pass++;
	}

	pr_info("Setting xGMI pstate to invalid value (32)\n");
	rc = hsmp_set_xgmi_pstate(32);
	if (rc != -EINVAL) {
		pr_err("Setting xGMI pstate returned %d\n", rc);
		fail++;
	} else {
		pass++;
	}
}

static void do_df_pstate_test(void)
{
	int rc;

	pr_info("Setting DF P-state to 2 for socket 0\n");
	rc = hsmp_set_df_pstate(0, 2);
	if (rc) {
		pr_err("Setting DF P-state returned %d\n", rc);
		fail++;
	} else {
		pass++;
	}

	pr_info("Setting DF P-state to 15 for socket 0\n");
	rc = hsmp_set_df_pstate(0, 15);
	if (rc != -EINVAL) {
		pr_err("Setting DF P-state returned %d\n", rc);
		fail++;
	} else {
		pass++;
	}

	pr_info("Setting DF P-state to 2 for socket 4\n");
	rc = hsmp_set_df_pstate(4, 2);
	if (rc != -ENODEV) {
		pr_err("Setting DF P-state returned %d\n", rc);
		fail++;
	} else {
		pass++;
	}
}

static void do_power_limit_write(void)
{
	int rc;

	pr_info("Writing power limit of 5 for socket 0\n");
	rc = hsmp_set_power_limit(0, 5);
	if (rc) {
		pr_err("Write power limit returned %d\n", rc);
		fail++;
	} else {
		pass++;
	}

	pr_info("Writing power limit with invalid socket\n");
	rc = hsmp_set_power_limit(10, 5);
	if (rc != -ENODEV) {
		pr_err("Write power limit returned %d\n", rc);
		fail++;
	} else {
		pass++;
	}
}

static void do_boost_limit_test(void)
{
	u32 limit;
	int rc;

	pr_info("Setting boost limit cpu 0 to 3150\n");
	rc = hsmp_set_boost_limit_cpu(0, 3150);
	if (rc) {
		pr_err("Setting boost limit returned %d\n", rc);
		fail++;
	} else {
		pass++;
	}

	pr_info("Setting boost limit for socket 0 to 3150\n");
	rc = hsmp_set_boost_limit_socket(0, 3150);
	if (rc) {
		pr_err("Setting boost limit returned %d\n", rc);
		fail++;
	} else {
		pass++;
	}

	pr_info("Setting boost limit for socket 10 to 3150\n");
	rc = hsmp_set_boost_limit_socket(10, 3150);
	if (rc != -ENODEV) {
		pr_err("Setting boost limit returned %d\n", rc);
		fail++;
	} else {
		pass++;
	}

	pr_info("Setting system boost limit to 3150\n");
	rc = hsmp_set_boost_limit_system(3150);
	if (rc) {
		pr_err("Setting boost limit returned %d\n", rc);
		fail++;
	} else {
		pass++;
	}

	pr_info("Reading cpu boost limit\n");
	rc = hsmp_get_boost_limit_cpu(0, &limit);
	if (rc) {
		pr_err("Reading boost limit returned %d\n", rc);
		fail++;
	} else {
		pass++;
	}
}

static void do_fabric_clocks_test(void)
{
	u32 fclk, mclk;
	int rc;

	/* Proper socket */
	pr_info("Reading fabric_clocks for socket 0\n");
	rc = hsmp_get_fabric_clocks(0, &fclk, &mclk);
	if (rc) {
		fail++;
		pr_err("Reading fabric_clocks returned %d\n", rc);
	} else {
		pass++;
	}

	/* Invalid socket */
	pr_info("Reading fabric_clocks for socket 5\n");
	rc = hsmp_get_fabric_clocks(5, &fclk, &mclk);
	if (rc == -ENODEV) {
		pass++;
	} else {
		pr_err("Reading fabric_clocks returned %d\n", rc);
		fail++;
	}

	/* Invalid return value */
	pr_info("Reading fabric_clocks with invalid fclk pointers\n");
	rc = hsmp_get_fabric_clocks(0, NULL, &mclk);
	if (rc == -EINVAL) {
		pr_err("Reading fabric_clocks returned %d\n", rc);
		fail++;
	} else {
		pass++;
	}

	pr_info("Reading fabric_clocks with invalid mclk pointer\n");
	rc = hsmp_get_fabric_clocks(0, &fclk, NULL);
	if (rc == -EINVAL) {
		pr_err("Reading fabric_clocks returned %d\n", rc);
		fail++;
	} else {
		pass++;
	}

	pr_info("Reading fabric_clocks with invalid clk pointers\n");
	rc = hsmp_get_fabric_clocks(0, NULL, NULL);
	if (rc == -EINVAL) {
		pass++;
	} else {
		pr_err("Reading fabric_clocks returned %d\n", rc);
		fail++;
	}
}

/*
 * DDR Bandwidth interface is only supported on HSMP protocol >= 3.
 * Check for return code of ENOMSG indicating we are running on an
 * unsupported protocol level.
 */
static void do_ddr_bw_test(void)
{
	struct hsmp_ddr_bw ddr_bw;
	int rc;

	/* Proper Socket */
	pr_info("Reading DDR Bandwidth for socket 0\n");
	rc = hsmp_get_ddr_bandwidth(0, &ddr_bw);
	if (rc) {
		if (rc == -EOPNOTSUPP) {
			pass++;
			pr_info("Reading DDR Bandwidth not supported\n");
		} else {
			fail++;
			pr_err("Reading DDR Bandwidth returned %d\n", rc);
		}
	} else {
		pass++;
	}

	/* Invalid Socket */
	pr_info("Reading DDR Bandwidth for socket 5\n");
	rc = hsmp_get_ddr_bandwidth(0, &ddr_bw);
	if (rc == -ENODEV || rc == -EOPNOTSUPP) {
		pass++;
	} else {
		fail++;
		pr_err("Reading DDR Bandwidth returned %d\n", rc);
	}

	/* NULL struct pointer */
	pr_info("Reading DDR Bandwidth with invalid struct pointer\n");
	rc = hsmp_get_ddr_bandwidth(0, NULL);
	if (rc == -EINVAL || rc == -EOPNOTSUPP) {
		pass++;
	} else {
		pr_err("Reading DDR Bandwidth returned %d (%d %d)\n", rc, -EINVAL, -EOPNOTSUPP);
		fail++;
	}
}

static void do_hsmp_tests(void)
{
	/* Test for exported symbol amd_smu_fw */
	pr_info("Validating exported amd_smu_fw %u.%u.%u\n",
		amd_smu_fw.ver.major, amd_smu_fw.ver.minor,
		amd_smu_fw.ver.debug);
	pass++;

	/* tctl */
	do_simple_read_test("tctl", &amd_get_tctl);
	do_xgmi_test();

	/* power */
	do_simple_read_test("power", &hsmp_get_power);

	/* power limit */
	do_power_limit_write();
	do_simple_read_test("power_limit", &hsmp_get_power_limit);

	/* power limit max */
	do_simple_read_test("power_limit_max", &hsmp_get_power_limit_max);

	do_boost_limit_test();

	/* proc hot */
	do_simple_read_test("proc_hot", &hsmp_get_proc_hot);

	do_df_pstate_test();

	/* fabric clocks */
	do_fabric_clocks_test();

	/* max cclk */
	do_simple_read_test("max_cclk", &hsmp_get_max_cclk);

	/* c0 residency */
	do_simple_read_test("c0_residency", &hsmp_get_c0_residency);

	/* DDR Bandwidth */
	do_ddr_bw_test();
}

static ssize_t hsmp_test_store(struct kobject *kobj,
			       struct kobj_attribute *attr,
			       const char *buf, size_t count)
{
	pass = 0;
	fail = 0;
	tbd = 0;

	do_hsmp_tests();
	return count;
}

static ssize_t hsmp_test_show(struct kobject *kobj,
			      struct kobj_attribute *attr,
			      char *buf)
{
	return sprintf(buf, "%d,%d,%d\n", pass, fail, tbd);
}

struct kobj_attribute hsmp_test = __ATTR_RW(hsmp_test);

static int __init hsmp_test_init(void)
{
	struct kobject *cpu_kobj = &cpu_subsys.dev_root->kobj;

	pr_info("Test module loaded\n");
	WARN_ON(sysfs_create_file(cpu_kobj, &hsmp_test.attr));

	return 0;
}

static void __exit hsmp_test_exit(void)
{
	struct kobject *cpu_kobj = &cpu_subsys.dev_root->kobj;

	sysfs_remove_file(cpu_kobj, &hsmp_test.attr);
}

module_init(hsmp_test_init);
module_exit(hsmp_test_exit);

