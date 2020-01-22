// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2007-2019 Advanced Micro Devices, Inc.
 * Author: Lewis Carroll <lewis.carroll@amd.com>
 * Maintainer: Nathan Fontenot <nathan.fontenot@amd.com>
 */

/*
 * AMD Host System Management Port driver
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
 * Typical error codes are -ENODEV if the specified socket or CPU does not
 * exist, and -EINVAL if a function that retrieves a value is passed a NULL
 * pointer.
 */

/*
 * Get average socket power consumption (in milliwatts).
 * If power_mw is NULL, this function does nothing and returns -EINVAL.
 * Returns -ENODEV if the specified socket does not exist.
 */
int hsmp_get_power(int socket_id, u32 *power_mw);

/*
 * Set socket power consumption limit (in milliwatts).
 * Returns -ENODEV if the specified socket does not exist.
 */
int hsmp_set_power_limit(int socket_id, u32 limit_mw);

/*
 * Get socket power consumption limit (in milliwatts).
 * If limit_mw is NULL, this function does nothing and returns -EINVAL.
 * Returns -ENODEV if the specified socket does not exist.
 */
int hsmp_get_power_limit(int socket_id, u32 *limit_mw);

/*
 * Get the maximum socket power consumption limit that can be set
 * (in milliwatts).
 * If limit_mw is NULL, this function does nothing and returns -EINVAL.
 * Returns -ENODEV if the specified socket does not exist.
 */
int hsmp_get_power_limit_max(int socket_id, u32 *limit_mw);

/*
 * Set HSMP boost limit for the system.
 */
int hsmp_set_boost_limit_system(u32 limit_mhz);

/*
 * Set HSMP boost limit for a specific core.
 * Returns -ENODEV if the specified CPU does not exist.
 */
int hsmp_set_boost_limit_cpu(int cpu, u32 limit_mhz);

/*
 * Set HSMP boost limit for all cores in the specified socket.
 * Returns -ENODEV if the specified socket does not exist.
 */
int hsmp_set_boost_limit_socket(int socket_id, u32 limit_mhz);

/*
 * Get HSMP boost limit for a specific core.
 * If limit_mhz is NULL, this function does nothing and returns -EINVAL.
 * Returns -ENODEV if the specified CPU does not exist.
 */
int hsmp_get_boost_limit_cpu(int cpu, u32 *limit_mhz);

/*
 * Get normalized status of the processor's PROC_HOT input.
 * proc_hot is set to 1 if PROC_HOT is active, 0 if PROC_HOT is not active.
 * If proc_hot is NULL, this function does nothing and returns -EINVAL.
 * Returns -ENODEV if the specified socket does not exist.
 */
int hsmp_get_proc_hot(int socket_id, u32 *proc_hot);

/*
 * Set xGMI link P-state and disable automatic P-state selection. Acceptable
 * values for the P-state are family specific. For family 17h models 30h-3fh
 * (Rome), acceptable values are 0 (link width = 16) and 1 (link width = 8).
 * Passing a value of -1 will enable automatic link width selection based on
 * link utilization.
 * Returns -EINVAL for any unacceptable value.
 * Returns -ENODEV if called on a 1P system.
 * Returns -ENODEV if the specified socket does not exist.
 */
int hsmp_set_xgmi_pstate(int pstate);

/*
 * Set data fabric P-state and disable automatic P-state selection (analogous
 * to the UEFI setup option APBDIS=1). APB stands for Algorithmic Performance
 * Boost. Acceptable values for the P-state are 0 - 3. Passing a value of -1
 * will enable automatic P-state selection based on data fabric utilization
 * (analogous to APBDIS=0).
 * Returns -EINVAL for any unacceptable value.
 * Returns -ENODEV if the specified socket does not exist.
 */
int hsmp_set_df_pstate(int socket_id, int p_state);

/*
 * Get Data Fabric clock and memory clock in MHz. If both pointers are
 * NULL, this function does nothing and returns -EINVAL.
 * Returns -ENODEV if the specified socket does not exist.
 */
int hsmp_get_fabric_clocks(int socket_id, u32 *fclk, u32 *memclk);

/*
 * Get the maximum core clock (cclk) allowed by the most restrictive of any of
 * the control subsystems in the SOC for any core. If max_mhz is NULL, this
 * function does nothing and returns -EINVAL.
 * Returns -ENODEV if the specified socket does not exist.
 */
int hsmp_get_max_cclk(int socket_id, u32 *max_mhz);

/*
 * Get the C0 residency percentage for all cores in the socket. Residency is
 * returned as an integer between 0 and 100 inclusive. If residency is NULL,
 * this function does nothing and returns -EINVAL.
 * Returns -ENODEV if the specified socket does not exist.
 */
int hsmp_get_c0_residency(int socket_id, u32 *residency);

/*
 * Set the NBIO (PCI-e interface) P-state for the specified PCI-e bus number
 * and disable automatic P-state selection. Acceptable values for the P-state
 * are 0 and 1. Use P-state 0 for minimum latency for PCI-e devices attached
 * to the specified bus and P-state 1 for minimum power. Passing a value of
 * -1 will enable automatic P-state selection based on bus utilization.
 * Returns -EINVAL for any unacceptable value.
 * Returns -ENODEV if the specified bus does not exist.
 */
int hsmp_set_nbio_pstate(u8 bus_num, int pstate);

/*
 * Get current Thermal Control (TCTL) value for socket. Returns 0 for
 * success and sets tctl. Note TCTL is NOT temperature. TCTL is a
 * unitless figure with a value from 0 - 100, where 100 usually means
 * the processor will initiate PROC_HOT actions and 95 usually means
 * the processor will initiate thermal throttling actions.
 * If tctl is NULL, this function does nothing and returns -EINVAL.
 * Returns -ENODEV if the specified socket does not exist.
 */
int amd_get_tctl(int socket_id, u32 *tctl);

/*
 * These two functions depend on making certain internal-only SMU
 * registers public domain. Until this happens, these two functions
 * cannot be released into open source.
 */

/*
 * Get current xGMI link width (2P system only). Returns -ENODEV if
 * called in a 1P system. Returns 0 for success and sets width.
 * Possible values for width are family-specific. For Family 17h
 * Model 30 (Rome), possible width values are 2, 8 and 16.
 * If width is NULL, this function does nothing and returns -EINVAL.
 */
int amd_get_xgmi_pstate(int *width);

/*
 * Get xGMI link speed (2P system only). Returns -ENODEV if called
 * in a 1P system. Returns 0 for success and sets speed which should
 * be interpreted as Mbps.
 * If width is NULL, this function does nothing and returns -EINVAL.
 */
int amd_get_xgmi_speed(u32 *speed);
