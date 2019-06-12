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
 * Protocol version 1 API
 *
 * Note on boost limits.
 *
 * For every core, the System Management Unit maintains several different limits
 * on core frequency. Because limits are maintained by the SMU and not by cores
 * themselves, limits can be set and read regardless of the core hotplug state.
 * This also means when using hotplug to bring a core back online, any previous
 * limits at a core or socket level will remain enforced.
 *
 * The various limits to per-core boost include the following:
 * - The fused in SKU-specific maximum boost frequency
 * - Any limit set by UEFI firmware
 * - Any limit set by Platform Management (BMC) via the processor's Advanced
 *   Platform Management Link (APML)
 * - Limit set by HSMP (this interface)
 *
 * When setting the boost limit for a core, the actual limit that is enforced
 * will be constrained to a range of valid values for the processor and will be
 * the lowest of any of the above limits. To remove the HSMP limit for a core,
 * write a value equal to or greater than the fused-in maximum frequency. A
 * value of 0xFFFF works well.
 *
 * If SMT is enabled, it is only necessary to set a limit for one of the two
 * thread siblings. Since the limit is a core limit, it will apply to both
 * siblings.
 *
 * Boost limits are set and read in units of MHz.
 *
 * All functions return 0 for success, negative error code for failure.

 */

/*
 * Get average socket power consumption (in milliwatts).
 * If power_mw is NULL, this function does nothing and returns -EINVAL.
 */
int hsmp1_get_power(int socket, u32 *power_mw);

/*
 * Set socket power consumption limit (in milliwatts).
 */
int hsmp1_set_power_limit(int socket, u32 limit_mw);

/*
 * Get socket power consumption limit (in milliwatts).
 * If limit_mw is NULL, this function does nothing and returns -EINVAL.
 */
int hsmp1_get_power_limit(int socket, u32 *limit_mw);

/*
 * Get the maximum socket power consumption limit that can be set
 * (in milliwatts).
 * If limit_mw is NULL, this function does nothing and returns -EINVAL.
 */
int hsmp1_get_power_limit_max(int socket, u32 *limit_mw);

/*
 * Set HSMP boost limit for a specific core.
 */
int hsmp1_set_boost_limit(int cpu, u32 limit_mhz);

/*
 * Set HSMP boost limit for all cores in the specified socket.
 */
int hsmp1_set_boost_limit_socket(int socket, u32 limit_mhz);

/*
 * Get boost limit for a specific core.
 * If limit_mhz is NULL, this function does nothing and returns -EINVAL.
 */
int hsmp1_get_boost_limit(int cpu, u32 *limit_mhz);

/*
 * Get normalized status of the processor's PROC_HOT input.
 * If proc_hot is NULL, this function does nothing and returns -EINVAL.
 * Returns true if PROC_HOT is active, false if PROC_HOT is not active.
 */
int hsmp1_get_proc_hot(int socket, bool *proc_hot);

/*
 * Set xGMI2 link width range (2P system only). Returns -ENODEV if called
 * in a 1P system. Valid values for link_width are 2, 8 and 16.
 * Returns -EINVAL for any other value.
 */
int hsmp1_set_xgmi2_link_width(unsigned width_min, unsigned width_max);

/*
 * Set Data Fabric P state and disable automatic P state selection. Acceptable
 * values for the P state are 0 - 3. Passing a value of -1 will enable
 * automatic P state selection based on data fabric utilization (analogous to
 * APBEnable).
 */
int hsmp1_set_df_pstate(int socket, int p_state);

/*
 * Get Data Fabric clock and memory clock in MHz. If both pointers are
 * NULL, this function does nothing and returns -EINVAL.
 */
int hsmp1_get_fabric_clocks(int socket, u32 *fclk, u32 *memclk);

/*
 * Get the maximum core clock (cclk) allowed by the most restrictive of any of
 * the control subsystems in the SOC for any core. If max_mhz is NULL, this
 * function does nothing and returns -EINVAL.
 */
int hsmp1_get_max_cclk(int socket, u32 *max_mhz);

/*
 * Get the C0 residency percentage for all cores in the socket. Residency is
 * returned as an integer between 0 and 100 inclusive. If residency is NULL,
 * this function does nothing and returns -EINVAL.
 */
int hsmp1_get_c0_residency(int socket, u32 *residency);
