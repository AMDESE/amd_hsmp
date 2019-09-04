// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2007-2019 Advanced Micro Devices, Inc.
 * Author: Lewis Carroll <lewis.carroll@amd.com>
 * Maintainer: Nathan Fontenant <nathan.fontenant@amd.com>
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
 *
 * SysFS interface
 *
 * Parent directory: /sys/devices/system/cpu/amd_hsmp/
 * General structure:
 *
 * amd_hsmp/pci0000:XX    Directory for each PCI-e bus
 *     nbio_pstate        (WO) Set PCI-e bus interface P-state
 *
 * amd_hsmp/cpuX/         Directory for each possible CPU
 *     boost_limit        (RW) HSMP boost limit for the core in MHz
 *
 * amd_hsmp/socketX/      Directory for each possible socket
 *     boost_limit        (WO) Set HSMP boost limit for the socket in MHz
 *     c0_residency       (RO) Average % all cores are in C0 state
 *     cclk_limit         (RO) Most restrictive core clock (CCLK) limit in MHz
 *     fabric_clocks      (RO) Data fabric (FCLK) and memory (MCLK) in MHz
 *     fabric_pstate      (WO) Set data fabric P-state, -1 for autonomous
 *     power              (RO) Average socket power in milliwatts
 *     power_limit        (RW) Socket power limit in milliwatts
 *     power_limit_max    (RO) Maximum possible value for power limit in mW
 *     proc_hot           (RO) Socket PROC_HOT status (1 = active, 0 = inactive)
 *     tctl               (RO) Thermal Control value (see note below)
 *
 * boost_limit            (WO) Set HSMP boost limit for the system in MHz
 * hsmp_proto_version     (RO) HSMP protocol implementation
 * smu_fw_version         (RO) SMU firmware version signature
 * xgmi_pstate            (RW) xGMI P-state, -1 for autonomous (2P only)
 * xgmi_speed             (RO) xGMI link speed in Mbps (2P only) - WARNING
 *
 * nbio_pstate - write a value of 0 or 1 to set a specific PCI-e bus interface
 * P-state and disable automatic P-state selection. Use P-state 0 for minimum
 * latency for attached PCI-e devices or use P-state 2 for minimum power.
 * Write a value of -1 to enable autonomous P-state selection. Note these
 * directories will only exist on a system supporting HSMP protocol version 2.
 *
 * fabric_pstate - write a value of 0 - 3 to set a specific data fabric
 * P-state. Write a value of -1 to enable autonomous data fabric P-state
 * selection.
 *
 * fabric_clocks returns two comma separated values. The first is the fabric
 * clock (FCLK) in MHz, and the second is the memory clock (MCLK) in MHz.
 *
 * tctl is NOT the socket temperature. tctl is NOT temperature. tctl is a
 * unitless figure with a value from 0 - 100, where 100 usually means the
 * processor will initiate PROC_HOT actions and 95 usually means the processor
 * will begin thermal throttling actions.
 *
 * xgmi_pstate will only exist on 2P platforms. Write a value of 0 or 1 to set
 * a specific link P-state. Write a value of -1 to enable autonomous link
 * width selection. On family 17h model 30h-3fh, link P-state 0 corresponds to
 * a link width of 16 lanes and link P-state 1 corresponds to 8 lanes.
 *
 * SUPPORTED ONLY ONCE SMU REGISTERS ARE MADE PUBLIC:
 * Reading the xgmi_pstate file will return the current link P-state.
 * Reading the xgmi_speed file will return the link speed in Mbps.
 *
 * See comments in amd_hsmp.h for additional information.
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

#define DRV_MODULE_DESCRIPTION	"AMD Host System Management Port driver"
#define DRV_MODULE_VERSION	"0.8-internal"

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Lewis Carroll <lewis.carroll@amd.com>");
MODULE_DESCRIPTION(DRV_MODULE_DESCRIPTION);
MODULE_VERSION(DRV_MODULE_VERSION);

#define MAX_SOCKETS	2
#define MAX_NBIOS	8

/*
 * All protocol versions are required to support
 * these four status / error codes
 */
#define HSMP_STATUS_NOT_READY	0x00
#define HSMP_STATUS_OK		0x01
#define HSMP_ERR_INVALID_MSG	0xFE
#define HSMP_ERR_REQUEST_FAIL	0xFF

#ifndef __ro_after_init
#define __ro_after_init __read_mostly
#endif

/*
 * Expand as needed to cover all access ports types.
 * Current definition is for PCI-e config space access.
 */
struct smu_port {
	u32 index_reg;	/* PCI-e index register for SMU access */
	u32 data_reg;	/* PCI-e data register for SMU access */
};
static struct smu_port smu, hsmp __ro_after_init;

static struct {
	u32 mbox_msg_id;	/* SMU or MSR register for HSMP message ID */
	u32 mbox_status;	/* SMU or MSR register for HSMP status word */
	u32 mbox_data;		/* SMU or MSR base for message argument(s) */
	u32 mbox_timeout;	/* Timeout in MS to consider the SMU hung */
} hsmp_access __ro_after_init;

struct smu_fw {
	u8 debug;	/* Debug version number */
	u8 minor;	/* Minor version number */
	u8 major;	/* Major version number */
	u8 unused;
};

static u32 __ro_after_init amd_smu_fw_ver;
static u32 __ro_after_init amd_hsmp_proto_ver;
static int __ro_after_init amd_num_sockets;

/* Lookup tables for for North Bridges */
static struct nbio_dev {
	struct pci_dev *dev;		/* Pointer to PCI-e device */
	struct kobject *kobj;		/* SysFS directory */
	int		socket;		/* Physical socket number */
	u8		bus_num;	/* PCI-e bus number */
	u8		id;		/* NBIO tile within the socket */
} nbios[MAX_NBIOS] __ro_after_init;

static struct socket {
	struct pci_dev *dev;		/* Pointer to PCI-e device */
	struct kobject *kobj;		/* SysFS directory */
	struct mutex mutex;		/* lock to serialize reads/writes */
} sockets[MAX_SOCKETS];

static struct kobject *kobj_top;
static struct kobject **kobj_cpu;

/*
 * Message types
 *
 * All implementations are required to support HSMP_TEST, HSMP_GET_SMU_VER,
 * and HSMP_GET_PROTO_VER. All other messages are implementation dependent.
 */
enum hsmp_msg_t {HSMP_TEST				=  1,
		 HSMP_GET_SMU_VER			=  2,
		 HSMP_GET_PROTO_VER			=  3,
		 HSMP_GET_SOCKET_POWER			=  4,
		 HSMP_SET_SOCKET_POWER_LIMIT		=  5,
		 HSMP_GET_SOCKET_POWER_LIMIT		=  6,
		 HSMP_GET_SOCKET_POWER_LIMIT_MAX	=  7,
		 HSMP_SET_BOOST_LIMIT			=  8,
		 HSMP_SET_BOOST_LIMIT_SOCKET		=  9,
		 HSMP_GET_BOOST_LIMIT			= 10,
		 HSMP_GET_PROC_HOT			= 11,
		 HSMP_SET_XGMI_LINK_WIDTH		= 12,
		 HSMP_SET_DF_PSTATE			= 13,
		 HSMP_AUTO_DF_PSTATE			= 14,
		 HSMP_GET_FCLK_MCLK			= 15,
		 HSMP_GET_CCLK_THROTTLE_LIMIT		= 16,
		 HSMP_GET_C0_PERCENT			= 17,
		 HSMP_SET_NBIO_DPM_LEVEL		= 18
};

struct hsmp_message {
	enum hsmp_msg_t	msg_num;	/* Message number */
	u16		num_args;	/* Number of arguments in message */
	u16		response_sz;	/* Number of expected response words */
	u32		args[8];	/* Argument(s) */
	u32		response[8];	/* Response word(s) */
};

typedef int (*hsmp_send_message_t)(int, struct hsmp_message *);
static hsmp_send_message_t __ro_after_init hsmp_send_message;

/*
 * SMU access functions
 * Must be called with the socket mutex held. Returns 0 on success, negative
 * error code on failure. The return status is for the SMU access, not the
 * result of the intended SMU or HSMP operation.
 *
 * SMU PCI config space access method
 * There are two access apertures defined in the PCI-e config space for the
 * North Bridge, one for general purpose SMU register reads/writes and a second
 * aperture specific for HSMP messages and responses. For both reads and writes,
 * step one is to write the register to be accessed to the appropriate aperture
 * index register. Step two is to read or write the appropriate aperture data
 * register.
 */
static inline int smu_pci_write(struct pci_dev *root, u32 reg_addr,
				u32 reg_data, struct smu_port *port)
{
	int err;

	pr_debug("pci_write_config_dword addr 0x%08X, data 0x%08X\n",
		 port->index_reg, reg_addr);
	err = pci_write_config_dword(root, port->index_reg, reg_addr);
	if (err)
		return err;

	pr_debug("pci_write_config_dword addr 0x%08X, data 0x%08X\n",
		 port->data_reg, reg_data);
	err = pci_write_config_dword(root, port->data_reg, reg_data);
	if (err)
		return err;

	return 0;
}

static inline int smu_pci_read(struct pci_dev *root, u32 reg_addr,
			       u32 *reg_data, struct smu_port *port)
{
	int err;

	pr_debug("pci_write_config_dword addr 0x%08X, data 0x%08X\n",
		 port->index_reg, reg_addr);
	err = pci_write_config_dword(root, port->index_reg, reg_addr);
	if (err)
		return err;

	err = pci_read_config_dword(root, port->data_reg, reg_data);
	if (err)
		return err;
	pr_debug("pci_read_config_dword  addr 0x%08X, data 0x%08X\n",
		 port->data_reg, *reg_data);
	return 0;
}

/*
 * Send a message to the SMU access port via PCI-e config space registers.
 * The caller is expected to zero out any unused arguments. If a response
 * is expected, the number of response words should be greater than 0.
 * Returns 0 for success and populates the requested number of arguments
 * in the passed struct. Returns a negative error code for failure.
 */
static int send_message_pci(int socket_id, struct hsmp_message *msg)
{
	struct socket *socket = &sockets[socket_id];
	struct timespec64 ts, tt;
	int err;
	u32 mbox_status;
	unsigned int arg_num = 0;
	int retries = 0;
	static bool smu_is_hung[MAX_SOCKETS] = { false };

	/*
	 * In the unlikely case the SMU hangs, don't bother sending
	 * any more messages to the SMU on this socket.
	 */
	if (unlikely(smu_is_hung[socket_id]))
		return -ETIMEDOUT;

	pr_debug("Socket %d sending message ID %d\n", socket_id, msg->msg_num);
	while (msg->num_args && arg_num < msg->num_args) {
		pr_debug("    arg[%d:] 0x%08X\n", arg_num, msg->args[arg_num]);
		arg_num++;
	}

	arg_num = 0;

	mutex_lock(&socket->mutex);

	/* Zero the status register */
	mbox_status = HSMP_STATUS_NOT_READY;
	err = smu_pci_write(socket->dev, hsmp_access.mbox_status,
			    mbox_status, &hsmp);
	if (err) {
		pr_err("Error %d clearing mailbox status register on socket %d\n",
		       err, socket_id);
		goto out_unlock;
	}

	/* Write any message arguments */
	while (arg_num < msg->num_args) {
		err = smu_pci_write(socket->dev,
				    hsmp_access.mbox_data + (arg_num << 2),
				    msg->args[arg_num], &hsmp);
		if (err) {
			pr_err("Error %d writing message argument %d on socket %d\n",
			       err, arg_num, socket_id);
			goto out_unlock;
		}
		arg_num++;
	}

	/* Write the message ID which starts the operation */
	err = smu_pci_write(socket->dev, hsmp_access.mbox_msg_id,
			    msg->msg_num, &hsmp);
	if (err) {
		pr_err("Error %d writing message ID %u on socket %d\n",
		       err, msg->msg_num, socket_id);
		goto out_unlock;
	}

	/* Pre-calculate the time-out */
	ktime_get_real_ts64(&ts);
	tt = ts;
	timespec64_add_ns(&tt, hsmp_access.mbox_timeout * NSEC_PER_MSEC);

	/*
	 * Assume it takes at least one SMU FW cycle (1 MS) to complete
	 * the operation. Some operations might complete in two, some in
	 * more. So first thing we do is yield the CPU.
	 */
retry:
	usleep_range(1000, 2000);
	err = smu_pci_read(socket->dev, hsmp_access.mbox_status,
			   &mbox_status, &hsmp);
	if (err) {
		pr_err("Message ID %u - error %d reading mailbox status on socket %d\n",
		       err, msg->msg_num, socket_id);
		goto out_unlock;
	}
	if (mbox_status == HSMP_STATUS_NOT_READY) {
		/* SMU has not responded to the message yet */
		struct timespec64 tv;

		ktime_get_real_ts64(&tv);
		if (unlikely(timespec64_compare(&tv, &tt) > 0)) {
			pr_err("SMU timeout for message ID %u on socket %d\n",
			       msg->msg_num, socket_id);
			err = -ETIMEDOUT;
			goto out_unlock;
		}
		retries++;
		goto retry;
	}

	/* SMU has responded - check for error */
	ktime_get_real_ts64(&tt);
	tt = timespec64_sub(tt, ts);
	pr_debug("Socket %d message ack after %u ns, %d retries\n",
		 socket_id, ((unsigned int)timespec64_to_ns(&tt)), retries);

	if (unlikely(mbox_status == HSMP_ERR_INVALID_MSG)) {
		pr_err("Invalid message ID %u on socket %d\n",
		       msg->msg_num, socket_id);
		err = -ENOMSG;
		goto out_unlock;
	} else if (unlikely(mbox_status == HSMP_ERR_REQUEST_FAIL)) {
		pr_err("Message ID %u failed on socket %d\n",
		       msg->msg_num, socket_id);
		err = -EFAULT;
		goto out_unlock;
	} else if (unlikely(mbox_status != HSMP_STATUS_OK)) {
		pr_err("Message ID %u unknown failure (status = 0x%X) on socket %d\n",
		       msg->msg_num, mbox_status, socket_id);
		err = -EIO;
		goto out_unlock;
	}

	/* SMU has responded OK. Read response data */
	arg_num = 0;
	while (arg_num < msg->response_sz) {
		err = smu_pci_read(socket->dev,
				   hsmp_access.mbox_data + (arg_num << 2),
				   &msg->response[arg_num], &hsmp);
		if (err) {
			pr_err("Error %d reading response %u for message ID %u on socket %d\n",
			       err, arg_num, msg->msg_num, socket_id);
			goto out_unlock;
		}
		arg_num++;
	}

out_unlock:
	mutex_unlock(&socket->mutex);

	if (unlikely(err == -ETIMEDOUT))
		smu_is_hung[socket_id] = true;

	return err;
}

static struct nbio_dev *bus_to_nbio(u8 bus_num)
{
	int i;

	for (i = 0; i < MAX_NBIOS; i++) {
		if (nbios[i].bus_num == bus_num)
			return &nbios[i];
	}

	return NULL;
}

static int undef_tctl_fn(int socket)
{
	return -ENODEV;
}
static int (*__get_tctl)(int) __ro_after_init = undef_tctl_fn;

/*
 * These two placeholders can be removed if the needed SMU
 * registers are not made public.
 */
static int undef_link_pstate_fn(void)
{
	return -ENODEV;
}
static int (*__get_link_pstate)(void) __ro_after_init = undef_link_pstate_fn;

static int undef_link_speed_fn(void)
{
	return -ENODEV;
}
static int (*__get_link_speed)(void) __ro_after_init = undef_link_speed_fn;

int amd_get_tctl(int socket, u32 *tctl)
{
	int val;

	if (tctl == NULL)
		return -EINVAL;
	if (socket >= amd_num_sockets)
		return -ENODEV;

	val = __get_tctl(socket);
	if (val >= 0) {
		*tctl = val;
		return 0;
	} else
		return val;
}
EXPORT_SYMBOL(amd_get_tctl);

/*
 * These two functions can be removed if the needed SMU
 * registers are not made public.
 */
int amd_get_xgmi_pstate(int *pstate)
{
	int val;

	if (pstate == NULL)
		return -EINVAL;

	val = __get_link_pstate();
	if (val >= 0) {
		*pstate = val;
		return 0;
	} else
		return val;
}
EXPORT_SYMBOL(amd_get_xgmi_pstate);

int amd_get_xgmi_speed(u32 *speed)
{
	u32 val;

	if (speed == NULL)
		return -EINVAL;

	val = __get_link_speed();
	if (val >= 0) {
		*speed = val;
		return 0;
	} else
		return val;
}
EXPORT_SYMBOL(amd_get_xgmi_speed);

int hsmp_get_power(int socket, u32 *power_mw)
{
	int err;
	struct hsmp_message msg = { 0 };

	if (unlikely(power_mw == NULL))
		return -EINVAL;
	if (unlikely(socket >= amd_num_sockets))
		return -ENODEV;

	msg.msg_num     = HSMP_GET_SOCKET_POWER;
	msg.response_sz = 1;
	err = hsmp_send_message(socket, &msg);
	if (unlikely(err))
		pr_err("Failed to get socket %d power, err = %d\n",
		       socket, err);
	else
		*power_mw = msg.response[0];

	return err;
}
EXPORT_SYMBOL(hsmp_get_power);

int hsmp_set_power_limit(int socket, u32 limit_mw)
{
	int err;
	struct hsmp_message msg = { 0 };

	if (unlikely(socket >= amd_num_sockets))
		return -ENODEV;

	/* TODO do we need to do any bounds checking here?
	 * For now assuming SMU firmware will take care of it.
	 */

	msg.msg_num  = HSMP_SET_SOCKET_POWER_LIMIT;
	msg.num_args = 1;
	msg.args[0]  = limit_mw;
	err = hsmp_send_message(socket, &msg);
	if (unlikely(err))
		pr_err("Failed to set socket %d power limit, err = %d\n",
		       socket, err);
	else
		pr_info("Socket %d power limit set to %u mW\n",
			socket, limit_mw);

	return err;
}
EXPORT_SYMBOL(hsmp_set_power_limit);

int hsmp_get_power_limit(int socket, u32 *limit_mw)
{
	int err;
	struct hsmp_message msg = { 0 };

	if (unlikely(limit_mw == NULL))
		return -EINVAL;
	if (unlikely(socket >= amd_num_sockets))
		return -ENODEV;

	msg.msg_num     = HSMP_GET_SOCKET_POWER_LIMIT;
	msg.response_sz = 1;
	err = hsmp_send_message(socket, &msg);
	if (unlikely(err))
		pr_err("Failed to get socket %d power limit, err = %d\n",
		       socket, err);
	else
		*limit_mw = msg.response[0];

	return err;
}
EXPORT_SYMBOL(hsmp_get_power_limit);

int hsmp_get_power_limit_max(int socket, u32 *limit_mw)
{
	int err;
	struct hsmp_message msg = { 0 };

	if (unlikely(limit_mw == NULL))
		return -EINVAL;
	if (unlikely(socket >= amd_num_sockets))
		return -ENODEV;

	msg.msg_num     = HSMP_GET_SOCKET_POWER_LIMIT_MAX;
	msg.response_sz = 1;
	err = hsmp_send_message(socket, &msg);
	if (unlikely(err))
		pr_err("Failed to get socket %d max power limit, err = %d\n",
		       socket, err);
	else
		*limit_mw = msg.response[0];

	return err;
}
EXPORT_SYMBOL(hsmp_get_power_limit_max);

int hsmp_set_boost_limit_cpu(int cpu, u32 limit_mhz)
{
	int err, socket;
	struct hsmp_message msg = { 0 };

	if (unlikely(!cpu_present(cpu)))
		return -ENODEV;

	/* TODO do we need to do any bounds checking here?
	 * For now assuming SMU firmware will take care of it.
	 */

	socket       = cpu_data(cpu).phys_proc_id;
	msg.msg_num  = HSMP_SET_BOOST_LIMIT;
	msg.num_args = 1;
	msg.args[0]  = cpu_data(cpu).apicid << 16 | limit_mhz;
	err = hsmp_send_message(socket, &msg);
	if (unlikely(err))
		pr_err("Failed to set CPU %d boost limit, err = %d\n",
		       cpu, err);
	else
		pr_info("Set CPU %d boost limit to %u MHz\n", cpu, limit_mhz);

	return err;
}
EXPORT_SYMBOL(hsmp_set_boost_limit_cpu);

int hsmp_set_boost_limit_socket(int socket, u32 limit_mhz)
{
	int err;
	struct hsmp_message msg = { 0 };

	if (unlikely(socket >= amd_num_sockets))
		return -ENODEV;

	/* TODO do we need to do any bounds checking here?
	 * For now assuming SMU firmware will take care of it.
	 */

	msg.msg_num  = HSMP_SET_BOOST_LIMIT_SOCKET;
	msg.num_args = 1;
	msg.args[0]  = limit_mhz;
	err = hsmp_send_message(socket, &msg);
	if (unlikely(err))
		pr_err("Failed to set socket %d boost limit, err = %d\n",
		       socket, err);
	else
		pr_info("Set socket %d boost limit to %u MHz\n",
			socket, limit_mhz);

	return err;
}
EXPORT_SYMBOL(hsmp_set_boost_limit_socket);

int hsmp_set_boost_limit_system(u32 limit_mhz)
{
	int socket, _err;
	int err = 0;

	for (socket = 0; socket < amd_num_sockets; socket++) {
		_err = hsmp_set_boost_limit_socket(socket, limit_mhz);
		if (_err)
			err = _err;
	}

	return err;
}
EXPORT_SYMBOL(hsmp_set_boost_limit_system);

int hsmp_get_boost_limit_cpu(int cpu, u32 *limit_mhz)
{
	int err, socket;
	struct hsmp_message msg = { 0 };

	if (unlikely(limit_mhz == NULL))
		return -EINVAL;
	if (unlikely(!cpu_present(cpu)))
		return -ENODEV;

	socket          = cpu_data(cpu).phys_proc_id;
	msg.msg_num     = HSMP_GET_BOOST_LIMIT;
	msg.num_args    = 1;
	msg.response_sz = 1;
	msg.args[0]     = cpu_data(cpu).apicid;
	err = hsmp_send_message(socket, &msg);
	if (unlikely(err))
		pr_err("Failed to get CPU %d boost limit, err = %d\n",
		       cpu, err);
	else
		*limit_mhz = msg.response[0];

	return err;
}
EXPORT_SYMBOL(hsmp_get_boost_limit_cpu);

int hsmp_get_proc_hot(int socket, u32 *proc_hot)
{
	int err;
	struct hsmp_message msg = { 0 };

	if (unlikely(proc_hot == NULL))
		return -EINVAL;
	if (unlikely(socket >= amd_num_sockets))
		return -ENODEV;

	msg.msg_num     = HSMP_GET_PROC_HOT;
	msg.response_sz = 1;
	err = hsmp_send_message(socket, &msg);
	if (unlikely(err))
		pr_err("Failed to get socket %d PROC_HOT, err = %d\n",
		       socket, err);
	else
		*proc_hot = msg.response[0];

	return err;
}
EXPORT_SYMBOL(hsmp_get_proc_hot);

int hsmp_set_xgmi_pstate(int pstate)
{
	u8 width_min, width_max;
	int socket, _err;
	int err = 0;
	struct hsmp_message msg = { 0 };

	if (unlikely(amd_num_sockets < 2))
		return -ENODEV;

	switch (pstate) {
	case -1:
		width_min = 1;
		width_max = 2;
		pr_info("Enabling xGMI dynamic link width management\n");
		break;
	case 0:
		width_min = width_max = 2;
		pr_info("Setting xGMI link width to 16 lanes\n");
		break;
	case 1:
		width_min = width_max = 1;
		pr_info("Setting xGMI link width to 8 lanes\n");
		break;
	default:
		pr_warn("Invalid xGMI link P-state specified: %d\n", pstate);
		return -EINVAL;
	}

	msg.msg_num  = HSMP_SET_XGMI_LINK_WIDTH;
	msg.num_args = 1;
	msg.args[0]  = (width_min << 8) | width_max;
	for (socket = 0; socket < amd_num_sockets; socket++) {
		_err = hsmp_send_message(socket, &msg);
		if (_err) {
			pr_err("Failed to set socket %d xGMI link P-state, err = %d\n",
			       socket, err);
			err = _err;
		}
	}

	return err;
}
EXPORT_SYMBOL(hsmp_set_xgmi_pstate);

int hsmp_set_df_pstate(int socket, int pstate)
{
	int err;
	struct hsmp_message msg = { 0 };

	if (unlikely(socket >= amd_num_sockets))
		return -ENODEV;
	if (pstate < -1 || pstate > 3) {
		pr_warn("Invalid socket %d data fabric P-state specified: %d\n",
			socket, pstate);
		return -EINVAL;
	}

	if (pstate == -1)
		msg.msg_num = HSMP_AUTO_DF_PSTATE;
	else {
		msg.num_args = 1;
		msg.msg_num  = HSMP_SET_DF_PSTATE;
		msg.args[0]  = pstate;
	}

	err = hsmp_send_message(socket, &msg);
	if (unlikely(err))
		pr_err("Failed to set socket %d fabric P-state, err = %d\n",
		       socket, err);
	else
		pr_info("Set socket %d data fabric P-state to %d\n",
			socket, pstate);

	return err;
}
EXPORT_SYMBOL(hsmp_set_df_pstate);

int hsmp_get_fabric_clocks(int socket, u32 *fclk, u32 *memclk)
{
	int err;
	struct hsmp_message msg = { 0 };

	if (unlikely(fclk == NULL && memclk == NULL))
		return -EINVAL;
	if (unlikely(socket >= amd_num_sockets))
		return -ENODEV;

	msg.msg_num     = HSMP_GET_FCLK_MCLK;
	msg.response_sz = 2;
	err = hsmp_send_message(socket, &msg);
	if (unlikely(err))
		pr_err("Failed to get socket %d fabric clocks, err = %d\n",
		       socket, err);
	else {
		if (fclk)
			*fclk = msg.response[0];
		if (memclk)
			*memclk = msg.response[1];
	}

	return err;
}
EXPORT_SYMBOL(hsmp_get_fabric_clocks);

int hsmp_get_max_cclk(int socket, u32 *max_mhz)
{
	int err;
	struct hsmp_message msg = { 0 };

	if (unlikely(max_mhz == NULL))
		return -EINVAL;
	if (unlikely(socket >= amd_num_sockets))
		return -ENODEV;

	msg.msg_num     = HSMP_GET_CCLK_THROTTLE_LIMIT;
	msg.response_sz = 1;
	err = hsmp_send_message(socket, &msg);
	if (unlikely(err))
		pr_err("Failed to get socket %d max boost limit, err = %d\n",
		       socket, err);
	else
		*max_mhz = msg.response[0];

	return err;
}
EXPORT_SYMBOL(hsmp_get_max_cclk);

int hsmp_get_c0_residency(int socket, u32 *residency)
{
	int err;
	struct hsmp_message msg = { 0 };

	if (unlikely(residency == NULL))
		return -EINVAL;
	if (unlikely(socket > amd_num_sockets))
		return -ENODEV;

	msg.msg_num     = HSMP_GET_C0_PERCENT;
	msg.response_sz = 1;
	err = hsmp_send_message(socket, &msg);
	if (unlikely(err))
		pr_err("Failed to get socket %d C0 residency, err = %d\n",
		       socket, err);
	else
		*residency = msg.response[0];

	return err;
}
EXPORT_SYMBOL(hsmp_get_c0_residency);

int hsmp_set_nbio_pstate(u8 bus_num, int pstate)
{
	int err;
	u8 dpm_min, dpm_max;
	struct nbio_dev *nbio;
	struct hsmp_message msg = { 0 };

	if (amd_hsmp_proto_ver < 2)
		return -EOPNOTSUPP;

	nbio = bus_to_nbio(bus_num);
	if (unlikely(!nbio))
		return -ENODEV;

	/*
	 * DPM level 1 is not currently used. DPM level 2 is the max for
	 * non-ESM devices and DPM level 3 is the max for ESM devices.
	 */
	switch (pstate) {
	case -1:
		dpm_min = 0;
		dpm_max = 3;
		break;
	case 0:
		dpm_min = 2;
		dpm_max = 3;
		break;
	case 1:
		dpm_min = 0;
		dpm_max = 0;
		break;
	default:
		pr_warn("Invalid NBIO P-state specified: %d\n", pstate);
		return -EINVAL;
	}

	msg.msg_num = HSMP_SET_NBIO_DPM_LEVEL;
	msg.num_args = 1;
	msg.args[0] = (nbio->id << 16) | (dpm_max << 8) | dpm_min;
	err = hsmp_send_message(nbio->socket, &msg);
	if (err) {
		pr_err("Failed to set bus 0x%02X (socket %d NBIO %d) P-state\n",
		       bus_num, nbio->socket, nbio->id);
		return err;
	}

	if (dpm_min == dpm_max)
		pr_info("Set bus 0x%02X (socket %d NBIO %d) to P-state %d\n",
			bus_num, nbio->socket, nbio->id, pstate);
	else
		pr_info("Enabled bus 0x%02X (socket %d NBIO %d) auto P-state\n",
			bus_num, nbio->socket, nbio->id);

	return 0;
}
EXPORT_SYMBOL(hsmp_set_nbio_pstate);

/*
 * SysFS interface
 */

/* Helper macros */
#define FILE_ATTR_WO(_name)					\
	struct kobj_attribute _name = __ATTR_WO(_name)

#define FILE_ATTR_RO(_name)					\
	struct kobj_attribute _name = __ATTR_RO(_name)

#define FILE_ATTR_RW(_name)					\
	struct kobj_attribute rw_##_name = __ATTR_RW(_name)

static ssize_t smu_firmware_version_show(struct kobject *kobj,
					 struct kobj_attribute *attr,
					 char *buf)
{
	struct smu_fw *smu_fw_ver = (struct smu_fw *)&amd_smu_fw_ver;

	return sprintf(buf, "%u.%u.%u\n", smu_fw_ver->major,
		       smu_fw_ver->minor, smu_fw_ver->debug);
}
static FILE_ATTR_RO(smu_firmware_version);

static ssize_t hsmp_protocol_version_show(struct kobject *kobj,
					  struct kobj_attribute *attr,
					  char *buf)
{
	return sprintf(buf, "%u\n", amd_hsmp_proto_ver);
}
static FILE_ATTR_RO(hsmp_protocol_version);

static int kobj_to_socket(struct kobject *kobj)
{
	int socket;

	for (socket = 0; socket < amd_num_sockets; socket++)
		if (kobj == sockets[socket].kobj)
			return socket;

	return -1;
}

static int kobj_to_cpu(struct kobject *kobj)
{
	int cpu;

	for_each_present_cpu(cpu)
		if (unlikely(kobj == kobj_cpu[cpu]))
			return cpu;

	return -1;
}

static struct nbio_dev *kobj_to_nbio(struct kobject *kobj)
{
	int i;

	for (i = 0; i < MAX_NBIOS; i++) {
		if (nbios[i].kobj == kobj)
			return &nbios[i];
	}

	return NULL;
}

static ssize_t boost_limit_store(struct kobject *kobj,
				 struct kobj_attribute *attr,
				 const char *buf, size_t count)
{
	int err, socket, cpu;
	u32 limit_mhz = 0;

	err = kstrtouint(buf, 10, &limit_mhz);
	if (err)
		return err;

	socket = kobj_to_socket(kobj);
	cpu = kobj_to_cpu(kobj);

	/* Which file was written? */
	if (kobj == kobj_top)
		err = hsmp_set_boost_limit_system(limit_mhz);
	else if (socket >= 0)
		err = hsmp_set_boost_limit_socket(socket, limit_mhz);
	else if (cpu >= 0)
		err = hsmp_set_boost_limit_cpu(cpu, limit_mhz);

	if (err)
		return err;

	return count;
}
static FILE_ATTR_WO(boost_limit);

static ssize_t boost_limit_show(struct kobject *kobj,
				struct kobj_attribute *attr,
				char *buf)
{
	int err;
	u32 limit_mhz;

	err = hsmp_get_boost_limit_cpu(kobj_to_cpu(kobj), &limit_mhz);
	if (err)
		return err;

	return sprintf(buf, "%u\n", limit_mhz);
}
static FILE_ATTR_RW(boost_limit);

static ssize_t power_show(struct kobject *kobj,
			  struct kobj_attribute *attr, char *buf)
{
	u32 power_mw;
	int err;

	err = hsmp_get_power(kobj_to_socket(kobj), &power_mw);
	if (err)
		return err;

	return sprintf(buf, "%u\n", power_mw);
}
static FILE_ATTR_RO(power);

static ssize_t power_limit_store(struct kobject *kobj,
				 struct kobj_attribute *attr,
				 const char *buf, size_t count)
{
	u32 limit_mw;
	int err;
	int socket = kobj_to_socket(kobj);

	err = kstrtouint(buf, 10, &limit_mw);
	if (err)
		return err;

	err = hsmp_set_power_limit(socket, limit_mw);
	if (err)
		return err;

	return count;
}

static ssize_t power_limit_show(struct kobject *kobj,
				struct kobj_attribute *attr,
				char *buf)
{
	u32 limit_mw;
	int err;

	err = hsmp_get_power_limit(kobj_to_socket(kobj), &limit_mw);
	if (err)
		return err;

	return sprintf(buf, "%u\n", limit_mw);
}
static FILE_ATTR_RW(power_limit);

static ssize_t power_limit_max_show(struct kobject *kobj,
				    struct kobj_attribute *attr,
				    char *buf)
{
	u32 limit_mw;
	int err;

	err = hsmp_get_power_limit_max(kobj_to_socket(kobj), &limit_mw);
	if (err)
		return err;

	return sprintf(buf, "%u\n", limit_mw);
}
static FILE_ATTR_RO(power_limit_max);

static ssize_t proc_hot_show(struct kobject *kobj,
			     struct kobj_attribute *attr,
			     char *buf)
{
	u32 proc_hot = false;
	int err;

	err = hsmp_get_proc_hot(kobj_to_socket(kobj), &proc_hot);
	if (err)
		return err;

	return sprintf(buf, "%s\n", proc_hot ? "active" : "inactive");
}
static FILE_ATTR_RO(proc_hot);

static ssize_t xgmi_pstate_store(struct kobject *kobj,
				struct kobj_attribute *attr,
				const char *buf, size_t count)
{
	int pstate, err;

	err = kstrtoint(buf, 10, &pstate);
	if (err)
		return err;

	err = hsmp_set_xgmi_pstate(pstate);
	if (err)
		return err;

	return count;
}

/*
 * Pending SMU register public availability, may need to remove
 * xGMI pstate show and xgmi_speed show functions below and
 * restore RO attribute for the above function.
 */
static ssize_t xgmi_pstate_show(struct kobject *kobj,
			       struct kobj_attribute *attr,
			       char *buf)
{
	int err, pstate;

	err = amd_get_xgmi_pstate(&pstate);
	if (err)
		return err;

	return sprintf(buf, "%d\n", pstate);
}
static FILE_ATTR_RW(xgmi_pstate);

static ssize_t xgmi_speed_show(struct kobject *kobj,
			       struct kobj_attribute *attr,
			       char *buf)
{
	int err, speed;

	err = amd_get_xgmi_speed(&speed);
	if (err)
		return err;

	return sprintf(buf, "%d\n", speed);
}
static FILE_ATTR_RO(xgmi_speed);

static ssize_t fabric_pstate_store(struct kobject *kobj,
				   struct kobj_attribute *attr,
				   const char *buf, size_t count)
{
	int err, pstate;
	int socket = kobj_to_socket(kobj);

	err = kstrtoint(buf, 10, &pstate);
	if (err)
		return err;

	err = hsmp_set_df_pstate(socket, pstate);
	if (err)
		return err;

	return count;
}
static FILE_ATTR_WO(fabric_pstate);

static ssize_t fabric_clocks_show(struct kobject *kobj,
				  struct kobj_attribute *attr,
				  char *buf)
{
	u32 fclk, memclk;
	int err;

	err = hsmp_get_fabric_clocks(kobj_to_socket(kobj), &fclk, &memclk);
	if (err)
		return err;

	return sprintf(buf, "%u,%u\n", fclk, memclk);
}
static FILE_ATTR_RO(fabric_clocks);

static ssize_t cclk_limit_show(struct kobject *kobj,
			       struct kobj_attribute *attr,
			       char *buf)
{
	u32 max_mhz;
	int err;

	err = hsmp_get_max_cclk(kobj_to_socket(kobj), &max_mhz);
	if (err)
		return err;

	return sprintf(buf, "%u\n", max_mhz);
}
static FILE_ATTR_RO(cclk_limit);

static ssize_t c0_residency_show(struct kobject *kobj,
				 struct kobj_attribute *attr,
				 char *buf)
{
	u32 residency;
	int err;

	err = hsmp_get_c0_residency(kobj_to_socket(kobj), &residency);
	if (err)
		return err;

	return sprintf(buf, "%u\n", residency);
}
static FILE_ATTR_RO(c0_residency);

static ssize_t nbio_pstate_store(struct kobject *kobj,
				 struct kobj_attribute *attr,
				 const char *buf, size_t count)
{
	struct nbio_dev *nbio;
	int err, pstate;

	err = kstrtoint(buf, 10, &pstate);
	if (err)
		return err;

	nbio = kobj_to_nbio(kobj);
	if (!nbio)
		return -EINVAL;

	err = hsmp_set_nbio_pstate(nbio->bus_num, pstate);
	if (err)
		return err;

	return count;
}
static FILE_ATTR_WO(nbio_pstate);

static ssize_t tctl_show(struct kobject *kobj,
			 struct kobj_attribute *attr,
			 char *buf)
{
	int err;
	u32 tctl;

	err = amd_get_tctl(kobj_to_socket(kobj), &tctl);
	if (err)
		return err;

	return sprintf(buf, "%u\n", tctl);
}
static FILE_ATTR_RO(tctl);

/* Entry point to set-up SysFS interface */
static void __init hsmp_sysfs_init(void)
{
	struct kobject *kobj;
	int socket, cpu, i;
	char temp_name[16];
	ssize_t size;

	/* Top HSMP directory */
	WARN_ON(!(kobj_top = kobject_create_and_add("amd_hsmp",
						&cpu_subsys.dev_root->kobj)));
	if (!kobj_top)
		return;
	WARN_ON(sysfs_create_file(kobj_top, &smu_firmware_version.attr));
	WARN_ON(sysfs_create_file(kobj_top, &hsmp_protocol_version.attr));
	WARN_ON(sysfs_create_file(kobj_top, &boost_limit.attr));
	if (amd_num_sockets > 1) {
		/*
		 * Pending SMU register public availability, may need to
		 * change xgmi_pstate back to WO, and remove xgmi_speed.
		 */
		WARN_ON(sysfs_create_file(kobj_top, &xgmi_speed.attr));
		WARN_ON(sysfs_create_file(kobj_top, &rw_xgmi_pstate.attr));
	}

	if (amd_hsmp_proto_ver >= 2) {
		/* Directory for each PCI-e bus */
		for (i = 0; i < MAX_NBIOS; i++) {
			if (!nbios[i].dev)
				continue;

			snprintf(temp_name, 16, "pci0000:%02x",
				 nbios[i].bus_num);

			kobj = kobject_create_and_add(temp_name, kobj_top);
			if (!kobj) {
				pr_err("Could not create %s directory\n",
				       temp_name);
				continue;
			}

			WARN_ON(sysfs_create_file(kobj, &nbio_pstate.attr));
			nbios[i].kobj = kobj;
		}
	}

	/* Directory for each socket */
	for (socket = 0; socket < amd_num_sockets; socket++) {
		snprintf(temp_name, 16, "socket%d", socket);
		kobj = kobject_create_and_add(temp_name, kobj_top);
		if (!kobj) {
			pr_err("Could not create %s directory\n", temp_name);
			continue;
		}

		WARN_ON(sysfs_create_file(kobj, &boost_limit.attr));
		WARN_ON(sysfs_create_file(kobj, &power.attr));
		WARN_ON(sysfs_create_file(kobj, &rw_power_limit.attr));
		WARN_ON(sysfs_create_file(kobj, &power_limit_max.attr));
		WARN_ON(sysfs_create_file(kobj, &proc_hot.attr));
		WARN_ON(sysfs_create_file(kobj, &fabric_pstate.attr));
		WARN_ON(sysfs_create_file(kobj, &fabric_clocks.attr));
		WARN_ON(sysfs_create_file(kobj, &cclk_limit.attr));
		WARN_ON(sysfs_create_file(kobj, &c0_residency.attr));
		WARN_ON(sysfs_create_file(kobj, &tctl.attr));

		sockets[socket].kobj = kobj;
	}

	/*
	 * Directory for each present CPU. Note we are using present CPUs, not
	 * possible CPUs since some BIOSs are buggy w.r.t. the list of possible
	 * CPUs (e.g. if SMT disabled in BIOS, the SMT siblings are still
	 * listed as possible even though they can't be brought online without
	 * a reboot. Note HSMP doesn't care about online vs. offline CPUs.
	 */
	size = num_present_cpus() * sizeof(struct kobject *);
	kobj_cpu = kzalloc(size, GFP_KERNEL);
	if (!kobj_cpu)
		return;

	for_each_present_cpu(cpu) {
		snprintf(temp_name, 16, "cpu%d", cpu);
		kobj_cpu[cpu] = kobject_create_and_add(temp_name, kobj_top);
		if (!kobj_cpu[cpu]) {
			pr_err("Couldn't create %s directory\n", temp_name);
			continue;
		}

		WARN_ON(sysfs_create_file(kobj_cpu[cpu],
					  &rw_boost_limit.attr));
	}
}

/* Exit point to free SysFS interface */
static void __exit hsmp_sysfs_fini(void)
{
	int socket, cpu, i;
	struct kobject *kobj;

	if (!kobj_top)
		return;

	/* Remove files at top level directory */
	sysfs_remove_file(kobj_top, &smu_firmware_version.attr);
	sysfs_remove_file(kobj_top, &hsmp_protocol_version.attr);
	sysfs_remove_file(kobj_top, &boost_limit.attr);
	if (amd_num_sockets > 1) {
		/*
		 * Pending SMU register public availability, may need to
		 * change xgmi_pstate back to WO, and remove xgmi_speed.
		 */
		sysfs_remove_file(kobj_top, &xgmi_speed.attr);
		sysfs_remove_file(kobj_top, &rw_xgmi_pstate.attr);
	}

	if (amd_hsmp_proto_ver >= 2) {
		/* Remove directory for each PCI-e bus */
		for (i = 0; i < MAX_NBIOS; i++) {
			kobj = nbios[i].kobj;
			if (!kobj)
				continue;

			sysfs_remove_file(kobj, &nbio_pstate.attr);
			kobject_put(kobj);
		}
	}

	/* Remove socket directories */
	for (socket = 0; socket < amd_num_sockets; socket++) {
		kobj = sockets[socket].kobj;
		if (!kobj)
			continue;

		sysfs_remove_file(kobj, &boost_limit.attr);
		sysfs_remove_file(kobj, &power.attr);
		sysfs_remove_file(kobj, &rw_power_limit.attr);
		sysfs_remove_file(kobj, &power_limit_max.attr);
		sysfs_remove_file(kobj, &proc_hot.attr);
		sysfs_remove_file(kobj, &fabric_pstate.attr);
		sysfs_remove_file(kobj, &cclk_limit.attr);
		sysfs_remove_file(kobj, &c0_residency.attr);
		sysfs_remove_file(kobj, &tctl.attr);
		kobject_put(kobj);
	}

	/* Remove CPU directories */
	if (kobj_cpu) {
		for_each_present_cpu(cpu) {
			kobj = kobj_cpu[cpu];
			if (!kobj)
				continue;

			sysfs_remove_file(kobj, &rw_boost_limit.attr);
			kobject_put(kobj);
		}
		kfree(kobj_cpu);
	}

	/* Top HSMP directory */
	kobject_put(kobj_top);
}

/*
 * Define later - these functions must be executed
 * on the socket for which the message is intended.
static int send_message_msr(struct hsmp_message *msg) { }
static int send_message_mmio(struct hsmp_message *msg) { }
*/

/*
 * Port set-up for each supported chip family
 */

/*
 * Zen 2 - Rome
 * HSMP and SMU access is via PCI-e config space data / index register pair.
 */
#define F17M30_SMU_THERM_CTRL 0x00059800
static int f17m30_get_tctl(int socket_id)
{
	struct socket *socket = &sockets[socket_id];
	int err;
	u32 val;

	mutex_lock(&socket->mutex);
	err = smu_pci_read(socket->dev, F17M30_SMU_THERM_CTRL, &val, &smu);
	mutex_unlock(&socket->mutex);
	if (err) {
		pr_err("Error %d reading THERM_CTRL register\n", err);
		return err;
	}

	pr_debug("THERM_CTRL raw val: 0x%08X\n", val);

	val >>= 24;
	val  &= 0xFF;

	return val;
}

/*
 * As mentioned above, the get pstate and get speed functions may have to
 * come out until the SMU registers are public.
 */
#define F17M30_SMU_XGMI2_G0_PCS_LINK_STATUS1	0x12EF0050
static int f17m30_get_xgmi2_pstate(void)
{
	struct socket *socket = &sockets[0];
	int err;
	u32 val;

	mutex_lock(&socket->mutex);
	err = smu_pci_read(socket->dev, F17M30_SMU_XGMI2_G0_PCS_LINK_STATUS1,
			   &val, &smu);
	mutex_unlock(&socket->mutex);
	if (err) {
		pr_err("Error %d reading xGMI2 G0 PCS link status register\n",
		       err);
		return err;
	}

	pr_debug("XGMI2_G0_PCS_LINK_STATUS1 raw val: 0x%08X\n", val);

	val >>= 16;
	val  &= 0x3F;
	if (val > 4)
		return 0;
	else if (val > 2)
		return 1;

	pr_warn("Unable to determine xGMI2 link P-state, status = 0x%02X\n", val);
	return -1;
}

#define F17M30_SMU_XGMI2_G0_PCS_CONTEXT5	0x12EF0114
#define F17M30_SMU_FCH_PLL_CTRL0		0x02D02330
static int f17m30_get_xgmi2_speed(void)
{
	struct socket *socket = &sockets[0];
	int err1, err2, refclk;
	u32 freqcnt, refclksel;

	mutex_lock(&socket->mutex);
	err1 = smu_pci_read(socket->dev, F17M30_SMU_XGMI2_G0_PCS_CONTEXT5,
			    &freqcnt, &smu);
	if (!err1)
		err2 = smu_pci_read(socket->dev, F17M30_SMU_FCH_PLL_CTRL0,
				    &refclksel, &smu);
	mutex_unlock(&socket->mutex);
	if (err1) {
		pr_err("Error %d reading xGMI2 G0 PCS context register\n",
		       err1);
		return err1;
	}

	pr_debug("XGMI2_G0_PCS_CONTEXT5 raw val: 0x%08X\n", freqcnt);

	if (err2) {
		pr_err("Error %d reading reference clock select\n", err2);
		return err2;
	}

	pr_debug("FCH_PLL_CTRL0 raw val: 0x%08X\n", refclksel);

	freqcnt >>= 4;
	freqcnt  &= 0x7F;

	if (refclksel & 0xFF)
		refclk = 133;
	else
		refclk = 100;

	return freqcnt * 2 * refclk;
}

static int f17h_m30h_init(void)
{
	struct pci_dev *dev = NULL;
	int i, socket;
	u8 id;

	/* Offsets in PCI-e config space */
	smu.index_reg  = 0x60;
	smu.data_reg   = 0x64;
	hsmp.index_reg = 0xC4;
	hsmp.data_reg  = 0xC8;

	/* Offsets in SMU address space */
	hsmp_access.mbox_msg_id  = 0x3B10534;
	hsmp_access.mbox_status  = 0x3B10980;
	hsmp_access.mbox_data    = 0x3B109E0;

	hsmp_access.mbox_timeout = 500;

	hsmp_send_message = &send_message_pci;
	__get_tctl = f17m30_get_tctl;

	pr_info("Detected family 17h model 30h-30f CPU (Rome)\n");

	/*
	 * Family 17h model 30 has four North Bridges per socket (PCI device
	 * ID = 0x1480). We can't count on every kernel to report the number
	 * of physical sockets correctly (what if the kernel hasn't been
	 * updated for Rome?) We also can't assume overlapping bus numbering
	 * between 1P and 2P - 1P has 0x00, 0x40, 0x80 and 0xC0 and 2P has
	 * 0x00, 0x20, 0x40 ... 0xE0. So we'll first find all the NBIO
	 * devices then from there figure out what we've got.
	 */
	i = 0;
	do {
		dev = pci_get_device(PCI_VENDOR_ID_AMD, 0x1480, dev);
		if (dev && dev->bus) {
			pr_debug("Found NBIO bus 0x%02X\n", dev->bus->number);
			nbios[i].dev = dev;
			nbios[i].bus_num = dev->bus->number;
			i++;
		}
	} while (dev && 1 <= MAX_NBIOS);

	if (i != 4 && i != 8) {
		pr_err("Expected 4 or 8 NBIOs, found %d - giving up\n", i);
		return -ENODEV;
	}

	/* Finish the table - handle any funny NBIO numbering here */
	amd_num_sockets = i >> 2;
	for (i--; i >= 0; i--) {
		id = nbios[i].bus_num >> (7 - amd_num_sockets);

		socket = id >> 2;
		id &= 3;

		/* Cache NBIO 0 device for each socket */
		if (id == 0)
			sockets[socket].dev = nbios[i].dev;

		/* NBIO 2 and 3 are swapped */
		if (id == 2 || id == 3)
			id ^= 0x1;

		nbios[i].socket = socket;
		nbios[i].id = id;
	}

	/* Dump the table */
	pr_debug("Bus\tSocket\tNBIO\n");
	for (i = 0; i < MAX_NBIOS; i++)
		pr_debug("0x%02X\t%d\t%d\n", nbios[i].bus_num,
			 nbios[i].socket, nbios[i].id);

	pr_debug("Detected %d socket(s)\n", amd_num_sockets);

	/* Pending SMU register public availability */
	if (amd_num_sockets > 1) {
		__get_link_pstate = f17m30_get_xgmi2_pstate;
		__get_link_speed = f17m30_get_xgmi2_speed;
	}

	return 0;
}

/*
 * Check if this CPU supports HSMP (based on vendor, family, model). If so,
 * attempt a test message and if successful, retrieve the protocol version
 * and SMU firmware version. Check if the protocol version is supported.
 * Returns 0 for success
 * Returns -ENODEV for unsupported protocol version, unsupported CPU,
 * or if probe or test message fails.
 */
static int __init hsmp_probe(void)
{
	struct cpuinfo_x86 *c = &boot_cpu_data;
	struct hsmp_message msg = { 0 };
	int hsmp_bad = 0;
	struct smu_fw *smu_fw_ver;
	int err, _err, socket;

	if (c->x86_vendor != X86_VENDOR_AMD)
		return -ENODEV;

	/*
	 * Call the set-up function for a supported CPU,
	 * then drop through to the probe function.
	 */
	if (c->x86 == 0x17 && c->x86_model >= 0x30 && c->x86_model <= 0x3F) {
		err = f17h_m30h_init();	/* Zen 2 - Rome */
		if (err)
			return err;
	} else {
		/* Add additional supported family / model combinations */
		return -ENODEV;
	}

	for (socket = 0; socket < amd_num_sockets; socket++)
		mutex_init(&sockets[socket].mutex);

	/*
	 * Check each port to be safe. The test message takes one argument and
	 * returns the value of that argument + 1. The protocol version and SMU
	 * version messages take no arguments and return one.
	 */
	msg.args[0]     = 0xDEADBEEF;
	msg.response_sz = 1;
	for (socket = 0; socket < amd_num_sockets; socket++) {
		msg.msg_num = HSMP_TEST;
		msg.num_args = 1;

		_err = hsmp_send_message(socket, &msg);
		if (_err) {
			hsmp_bad = 1;
			err = _err;
			continue;
		}

		if (msg.response[0] != msg.args[0] + 1) {
			pr_err("Socket %d test message failed, Expected 0x%08X, received 0x%08X\n",
			       socket, msg.args[0] + 1, msg.response[0]);
			hsmp_bad = 1;
			err = -EBADE;
		}
	}

	msg.msg_num = HSMP_GET_SMU_VER;
	msg.num_args = 0;

	_err = hsmp_send_message(0, &msg);
	if (_err) {
		hsmp_bad = 1;
		err = _err;
	}
	amd_smu_fw_ver = msg.response[0];

	msg.msg_num  = HSMP_GET_PROTO_VER;
	_err = hsmp_send_message(0, &msg);
	if (_err) {
		hsmp_bad = 1;
		err = _err;
	}
	amd_hsmp_proto_ver = msg.response[0];

	if (hsmp_bad)
		return err;

	smu_fw_ver = (struct smu_fw *)&amd_smu_fw_ver;
	pr_info("Protocol version %u, SMU firmware version %u.%u.%u\n",
		amd_hsmp_proto_ver, smu_fw_ver->major,
		smu_fw_ver->minor, smu_fw_ver->debug);

	if (amd_hsmp_proto_ver != 1 && amd_hsmp_proto_ver != 2) {
		pr_err("Unsupported protocol version\n");
		return -ENODEV;
	}

	if (amd_hsmp_proto_ver == 1)
		pr_info("No NBIO P-state control with this protocol version\n");

	return 0;
}

static int __init hsmp_init(void)
{
	int err;

	pr_info("%s version %s\n", DRV_MODULE_DESCRIPTION, DRV_MODULE_VERSION);

	err = hsmp_probe();
	if (err)
		return err;

	if (amd_hsmp_proto_ver <= 2)
		hsmp_sysfs_init();

	return 0;
}

static void __exit hsmp_exit(void)
{
	pr_info("Driver unload\n");
	hsmp_sysfs_fini();
}

module_init(hsmp_init);
module_exit(hsmp_exit);
