/*
 * Copyright (C) 2007-2019 Advanced Micro Devices, Inc.
 * Author: Lewis Carroll <lewis.carroll@amd.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 */

/*
 * AMD Host System Management Port driver
 * SysFS interface
 *
 * Parent directory: /sys/devices/system/cpu/amd_hsmp/
 * General structure:
 * cpuX/                      Directory for each possible CPU
 *     boost_limit            (RW) HSMP boost limit for the core in MHz
 * socketX/                   Directory for each possible socket
 *     boost_limit            (WO) Set HSMP boost limit for the socket in MHz
 *     c0_residency           (RO) Average % all cores are in C0 state
 *     cclk_limit             (RO) Most restrictive Core CLK limit in MHz
 *     fabric_clocks          (RO) Data Fabric (FCLK) and memory (MCLK) in MHz
 *     fabric_pstate          (WO) Set data fabric P state 0-3
 *     power                  (RO) Average socket power in milliwatts
 *     power_limit            (RW) Socket power limit in milliwatts
 *     power_limit_max        (RO) Maximum possible value for power limit in milliwatts
 *     proc_hot               (RO) Socket PROC_HOT status (1 = active, 0 = inactive)
 *     xgmi2_width            (WO) XGMI2 Link Width min and max (2, 8 or 16 lanes - 2P only)
 * boost_limit                (WO) Set HSMP boost limit for the system in MHz
 * hsmp_proto_version         (RO) HSMP protocol implementation
 * smu_fw_version             (RO) SMU firmware version signature
 *
 * See comments in amd_hsmp1.h.
 */

#include <linux/kernel.h>
#include <linux/cpu.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/slab.h>
#include "hsmp_types.h"
//#include <asm/amd_hsmp1.h>	If in tree
#include "amd_hsmp1.h"

static struct kobject *kobj_top;
static struct kobject *kobj_socket[MAX_SOCKETS];
static struct kobject **kobj_cpu;

/* Helper macros */
#define DECLARE_SHOW(_name)			\
ssize_t show_##_name (struct kobject *kobj,	\
	struct kobj_attribute *attr, char *buf)

#define DECLARE_STORE(_name)						\
	ssize_t store_##_name (struct kobject *kobj,			\
	struct kobj_attribute *attr, const char *buf, size_t count)

#define FILE_ATTR_WO(_name)								\
	struct kobj_attribute _name = __ATTR(_name, 0200, NULL, store_##_name)

#define FILE_ATTR_RO(_name)								\
	struct kobj_attribute _name = __ATTR(_name, 0444, show_##_name, NULL)

#define FILE_ATTR_RW(_name)								\
	struct kobj_attribute rw_##_name = __ATTR(_name, 0644, show_##_name, store_##_name)

extern u32 amd_smu_fw_ver;
extern u32 amd_hsmp_proto_ver;

static DECLARE_SHOW(smu_firmware_version)
{
	struct smu_fw *smu_fw_ver = (struct smu_fw *)&amd_smu_fw_ver;
	return sprintf(buf, "%u.%u.%u\n", smu_fw_ver->major, smu_fw_ver->minor, smu_fw_ver->debug);
}
static FILE_ATTR_RO(smu_firmware_version);

static DECLARE_SHOW(hsmp_protocol_version)
{
	return sprintf(buf, "%u\n", amd_hsmp_proto_ver);
}
static FILE_ATTR_RO(hsmp_protocol_version);

static int kobj_to_socket(struct kobject *kobj)
{
	int socket;
	for (socket = 0; socket < topology_max_packages(); socket++) {
		if (kobj == kobj_socket[socket])
			return socket;
	}
	return -1;
}

static int kobj_to_cpu(struct kobject *kobj)
{
	int cpu;
	for_each_possible_cpu(cpu) {
		if (unlikely(kobj == kobj_cpu[cpu]))
			return cpu;
	}
	return -1;
}

static DECLARE_STORE(boost_limit)
{
	int socket, cpu;
	u32 limit_mhz = 0;

	/*
	 * TODO do we need to add a bounds check here? Assuming for now that
	 * SMU firmware handles any possible value we pass to it.
	 */
	sscanf(buf, "%u", (unsigned int *)&limit_mhz);
	if (!limit_mhz) {
		pr_info("HSMP: Invalid argument written to boost_limit: %s", buf);
		return count;
	}

	/* Which file was written? */
	if (kobj == kobj_top) {
		pr_info("HSMP: Setting system boost limit to %u MHz\n", limit_mhz);
		for (socket = 0; socket < topology_max_packages(); socket++)
			hsmp1_set_boost_limit_socket(socket, limit_mhz);
	} else if ((socket = kobj_to_socket(kobj)) >= 0) {
		pr_info("HSMP: Setting socket %d boost limit to %u MHz\n", socket, limit_mhz);
		hsmp1_set_boost_limit_socket(socket, limit_mhz);
	} else if ((cpu = kobj_to_cpu(kobj)) >= 0) {
		pr_info("HSMP: Setting CPU %d boost limit to %u MHz\n", cpu, limit_mhz);
		hsmp1_set_boost_limit(cpu, limit_mhz);
	}
	return count;
}
static FILE_ATTR_WO(boost_limit);

static DECLARE_SHOW(boost_limit)
{
	u32 limit_mhz = 0;
	hsmp1_get_boost_limit(kobj_to_cpu(kobj), &limit_mhz);
	return sprintf(buf, "%u\n", limit_mhz);
}
static FILE_ATTR_RW(boost_limit);

static DECLARE_SHOW(power)
{
	u32 power_mw = 0;
	hsmp1_get_power(kobj_to_socket(kobj), &power_mw);
	return sprintf(buf, "%u\n", power_mw);
}
static FILE_ATTR_RO(power);

static DECLARE_STORE(power_limit)
{
	int socket = kobj_to_socket(kobj);
	u32 limit_mw = 0;

	/*
	 * TODO do we need to add a bounds check here? Assuming for now that
	 * SMU firmware handles any possible value we pass to it.
	 */
	sscanf(buf, "%u", (unsigned int *)&limit_mw);
	if (!limit_mw) {
		pr_info("HSMP: Invalid argument written to power_limit: %s", buf);
		return count;
	}

	pr_info("HSMP: Setting socket %d power limit to %u mW\n", socket, limit_mw);
	hsmp1_set_power_limit(socket, limit_mw);
	return count;
}

static DECLARE_SHOW(power_limit)
{
	u32 limit_mw = 0;
	hsmp1_get_power_limit(kobj_to_socket(kobj), &limit_mw);
	return sprintf(buf, "%u\n", limit_mw);
}
static FILE_ATTR_RW(power_limit);

static DECLARE_SHOW(power_limit_max)
{
	u32 limit_mw = 0;
	hsmp1_get_power_limit_max(kobj_to_socket(kobj), &limit_mw);
	return sprintf(buf, "%u\n", limit_mw);
}
static FILE_ATTR_RO(power_limit_max);

static DECLARE_SHOW(proc_hot)
{
	bool proc_hot = false;
	hsmp1_get_proc_hot(kobj_to_socket(kobj), &proc_hot);
	return sprintf(buf, "%s\n", proc_hot ? "active" : "inactive");
}
static FILE_ATTR_RO(proc_hot);

static DECLARE_STORE(xgmi2_width)
{
	unsigned min = 0, max = 0;

	sscanf(buf, "%u,%u", &min, &max);
	if ((min != 2 && min != 8 && min != 16) ||
	    (max != 2 && max != 8 && max != 16)) {
		pr_info("HSMP: Invalid range written to xgmi2_width: %s", buf);
		return count;
	}

	pr_info("HSMP: Setting xGMI2 link width range to %u - %u lanes\n", min, max);
	hsmp1_set_xgmi2_link_width(min, max);

	return count;
}
static FILE_ATTR_WO(xgmi2_width);

static DECLARE_STORE(fabric_pstate)
{
	int socket = kobj_to_socket(kobj);
	int p_state = 0xFF;

	/*
	 * TODO do we need to add a bounds check here? Assuming for now that
	 * SMU firmware handles any possible value we pass to it.
	 */
	sscanf(buf, "%d", (int *)&p_state);
	if (p_state < -1 || p_state > 3) {
		pr_info("HSMP: Invalid argument written to fabric_pstate: %s", buf);
		return count;
	}

	if (p_state == -1)
		pr_info("HSMP: Enabling socket %d auto data fabric P-states\n", socket);
	else
		pr_info("HSMP: Setting socket %d data fabric P-state to %d\n", socket, p_state);
	hsmp1_set_df_pstate(socket, p_state);
	return count;
}
static FILE_ATTR_WO(fabric_pstate);

static DECLARE_SHOW(fabric_clocks)
{
	u32 fclk = 0, memclk = 0;
	hsmp1_get_fabric_clocks(kobj_to_socket(kobj), &fclk, &memclk);
	return sprintf(buf, "%u,%u\n", fclk, memclk);
}
static FILE_ATTR_RO(fabric_clocks);

static DECLARE_SHOW(cclk_limit)
{
	u32 max_mhz = 0;
	hsmp1_get_max_cclk(kobj_to_socket(kobj), &max_mhz);
	return sprintf(buf, "%u\n", max_mhz);
}
static FILE_ATTR_RO(cclk_limit);

static DECLARE_SHOW(c0_residency)
{
	u32 residency = 0;
	hsmp1_get_c0_residency(kobj_to_socket(kobj), &residency);
	return sprintf(buf, "%u\n", residency);
}
static FILE_ATTR_RO(c0_residency);

/* Entry point to set-up SysFS interface */
void __init amd_hsmp1_sysfs_init(void)
{
	int socket, cpu;
	char temp_name[8];
	ssize_t size = num_possible_cpus() * sizeof (struct kobject *);

	/* Top HSMP directory */
	WARN_ON(!(kobj_top = kobject_create_and_add("amd_hsmp", &cpu_subsys.dev_root->kobj)));
	if (!kobj_top)
		return;
	WARN_ON(sysfs_create_file(kobj_top, &smu_firmware_version.attr));
	WARN_ON(sysfs_create_file(kobj_top, &hsmp_protocol_version.attr));
	WARN_ON(sysfs_create_file(kobj_top, &boost_limit.attr));
	if (topology_max_packages() > 1)
		WARN_ON(sysfs_create_file(kobj_top, &xgmi2_width.attr));

	/* Directory for each socket */
	for (socket = 0; socket < topology_max_packages(); socket++) {
		snprintf(temp_name, 8, "socket%d", socket);
		WARN_ON(!(kobj_socket[socket] = kobject_create_and_add(temp_name, kobj_top)));
		if (!kobj_socket[socket])
			continue;
		WARN_ON(sysfs_create_file(kobj_socket[socket], &boost_limit.attr));
		WARN_ON(sysfs_create_file(kobj_socket[socket], &power.attr));
		WARN_ON(sysfs_create_file(kobj_socket[socket], &rw_power_limit.attr));
		WARN_ON(sysfs_create_file(kobj_socket[socket], &power_limit_max.attr));
		WARN_ON(sysfs_create_file(kobj_socket[socket], &proc_hot.attr));
		WARN_ON(sysfs_create_file(kobj_socket[socket], &fabric_pstate.attr));
		WARN_ON(sysfs_create_file(kobj_socket[socket], &fabric_clocks.attr));
		WARN_ON(sysfs_create_file(kobj_socket[socket], &cclk_limit.attr));
		WARN_ON(sysfs_create_file(kobj_socket[socket], &c0_residency.attr));
	}

	/* Directory for each possible CPU */
	WARN_ON(!(kobj_cpu = (struct kobject **)kzalloc(size, GFP_KERNEL)));
	if (!kobj_cpu)
		return;
	for_each_possible_cpu(cpu) {
		snprintf(temp_name, 8, "cpu%d", cpu);
		WARN_ON(!(kobj_cpu[cpu] = kobject_create_and_add(temp_name, kobj_top)));
		if (!kobj_cpu[cpu])
			continue;
		WARN_ON(sysfs_create_file(kobj_cpu[cpu], &rw_boost_limit.attr));
	}
}

/* Exit point to free SysFS interface */
void __exit amd_hsmp1_sysfs_fini(void)
{
	int socket, cpu;

	if (!kobj_top)
		return;

	/* Remove files at top level directory */
	sysfs_remove_file(kobj_top, &smu_firmware_version.attr);
	sysfs_remove_file(kobj_top, &hsmp_protocol_version.attr);
	sysfs_remove_file(kobj_top, &boost_limit.attr);
	if (topology_max_packages() > 1)
		sysfs_remove_file(kobj_top, &xgmi2_width.attr);

	/* Remove socket directories */
	for (socket = 0; socket < topology_max_packages(); socket++) {
		if (!kobj_socket[socket])
			continue;
		sysfs_remove_file(kobj_socket[socket], &boost_limit.attr);
		sysfs_remove_file(kobj_socket[socket], &power.attr);
		sysfs_remove_file(kobj_socket[socket], &rw_power_limit.attr);
		sysfs_remove_file(kobj_socket[socket], &power_limit_max.attr);
		sysfs_remove_file(kobj_socket[socket], &proc_hot.attr);
		sysfs_remove_file(kobj_socket[socket], &fabric_pstate.attr);
		sysfs_remove_file(kobj_socket[socket], &cclk_limit.attr);
		sysfs_remove_file(kobj_socket[socket], &c0_residency.attr);
		kobject_put(kobj_socket[socket]);
	}

	/* Remove CPU directories */
	if (kobj_cpu) {
		for_each_possible_cpu(cpu) {
			if (!kobj_cpu[cpu])
				continue;
			sysfs_remove_file(kobj_cpu[cpu], &rw_boost_limit.attr);
			kobject_put(kobj_cpu[cpu]);
		}
		kfree(kobj_cpu);
	}

	/* Top HSMP directory */
	kobject_put(kobj_top);
}
