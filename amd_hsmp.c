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
 * amd_hsmp/cpuX/         Directory for each possible CPU
 *     boost_limit        (RW) HSMP boost limit for the core in MHz
 *
 * amd_hsmp/socketX/      Directory for each possible socket
 *     boost_limit        (WO) Set HSMP boost limit for the socket in MHz
 *     c0_residency       (RO) Average % all cores are in C0 state
 *     cclk_limit         (RO) Most restrictive core clock (CCLK) limit in MHz
 *     fabric_clocks      (RO) Data fabric (FCLK) and memory (MCLK) in MHz
 *     fabric_pstate      (WO) Set data fabric P-state 0-3, -1 for autonomous
 *     power              (RO) Average socket power in milliwatts
 *     power_limit        (RW) Socket power limit in milliwatts
 *     power_limit_max    (RO) Maximum possible value for power limit in mW
 *     proc_hot           (RO) Socket PROC_HOT status (1 = active, 0 = inactive)
 *     tctl               (RO) Thermal Control value (see note below)
 *
 * boost_limit            (WO) Set HSMP boost limit for the system in MHz
 * hsmp_proto_version     (RO) HSMP protocol implementation
 * smu_fw_version         (RO) SMU firmware version signature
 * xgmi_speed             (RO) xGMI link speed in Mbps (2P only) - WARNING
 * xgmi_width             (RW) xGMI link width (2P only - see note) - WARNING
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
 * xgmi_width will only exist on 2P platforms. The only valid settings are
 * the following values:
 * 16 - force link width to 16 lanes (highest performance and power)
 *  8 - force link width to  8 lanes (good compromise)
 * -1 - Allow autonomous link width selection.
 *
 * SUPPORTED ONLY ONCE SMU REGISTERS ARE MADE PUBLIC:
 * Reading the xgmi_width file will return the current link width (8 or 16).
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
#define DRV_MODULE_VERSION	"0.61"

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Lewis Carroll <lewis.carroll@amd.com>");
MODULE_DESCRIPTION(DRV_MODULE_DESCRIPTION);
MODULE_VERSION(DRV_MODULE_VERSION);

#define MAX_SOCKETS 2

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

/* Serialize access to the SMU */
static DEFINE_MUTEX(lock_socket0);
static DEFINE_MUTEX(lock_socket1);

/* Pointer to North Bridges */
static struct pci_dev __ro_after_init *nb_root[MAX_SOCKETS] = { NULL };

static struct kobject *kobj_top;
static struct kobject *kobj_socket[MAX_SOCKETS];
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
		 HSMP_GET_C0_PERCENT			= 17
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

static inline void lock_socket(int socket)
{
	if (socket == 0)
		mutex_lock(&lock_socket0);
	else
		mutex_lock(&lock_socket1);
}

static inline void unlock_socket(int socket)
{
	if (socket == 0)
		mutex_unlock(&lock_socket0);
	else
		mutex_unlock(&lock_socket1);
}

/*
 * SMU access functions
 * Must be called with the socket_lock held. Returns 0 on success, negative
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
static int send_message_pci(int socket, struct hsmp_message *msg)
{
	struct pci_dev *root = nb_root[socket];
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
	if (unlikely(smu_is_hung[socket]))
		return -ETIMEDOUT;

	pr_debug("Socket %d sending message ID %d\n", socket, msg->msg_num);
	while (msg->num_args && arg_num < msg->num_args) {
		pr_debug("    arg[%d:] 0x%08X\n", arg_num, msg->args[arg_num]);
		arg_num++;
	}

	arg_num = 0;

	lock_socket(socket);

	/* Zero the status register */
	mbox_status = HSMP_STATUS_NOT_READY;
	err = smu_pci_write(root, hsmp_access.mbox_status, mbox_status, &hsmp);
	if (err) {
		pr_err("Error %d clearing mailbox status register on socket %d\n",
		       err, socket);
		goto out_unlock;
	}

	/* Write any message arguments */
	while (arg_num < msg->num_args) {
		err = smu_pci_write(root,
				    hsmp_access.mbox_data + (arg_num << 2),
				    msg->args[arg_num], &hsmp);
		if (err) {
			pr_err("Error %d writing message argument %d on socket %d\n",
			       err, arg_num, socket);
			goto out_unlock;
		}
		arg_num++;
	}

	/* Write the message ID which starts the operation */
	err = smu_pci_write(root, hsmp_access.mbox_msg_id, msg->msg_num, &hsmp);
	if (err) {
		pr_err("Error %d writing message ID %u on socket %d\n",
		       err, msg->msg_num, socket);
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
	err = smu_pci_read(root, hsmp_access.mbox_status, &mbox_status, &hsmp);
	if (err) {
		pr_err("Message ID %u - error %d reading mailbox status on socket %d\n",
		       err, msg->msg_num, socket);
		goto out_unlock;
	}
	if (mbox_status == HSMP_STATUS_NOT_READY) {
		/* SMU has not responded to the message yet */
		struct timespec64 tv;

		ktime_get_real_ts64(&tv);
		if (unlikely(timespec64_compare(&tv, &tt) > 0)) {
			pr_err("SMU timeout for message ID %u on socket %d\n",
			       msg->msg_num, socket);
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
		 socket, ((unsigned int)timespec64_to_ns(&tt)), retries);

	if (unlikely(mbox_status == HSMP_ERR_INVALID_MSG)) {
		pr_err("Invalid message ID %u on socket %d\n",
		       msg->msg_num, socket);
		err = -ENOMSG;
		goto out_unlock;
	} else if (unlikely(mbox_status == HSMP_ERR_REQUEST_FAIL)) {
		pr_err("Message ID %u failed on socket %d\n",
		       msg->msg_num, socket);
		err = -EFAULT;
		goto out_unlock;
	} else if (unlikely(mbox_status != HSMP_STATUS_OK)) {
		pr_err("Message ID %u unknown failure (status = 0x%X) on socket %d\n",
		       msg->msg_num, mbox_status, socket);
		err = -EIO;
		goto out_unlock;
	}

	/* SMU has responded OK. Read response data */
	arg_num = 0;
	while (arg_num < msg->response_sz) {
		err = smu_pci_read(root,
				   hsmp_access.mbox_data + (arg_num << 2),
				   &msg->response[arg_num], &hsmp);
		if (err) {
			pr_err("Error %d reading response %u for message ID %u on socket %d\n",
			       err, arg_num, msg->msg_num, socket);
			goto out_unlock;
		}
		arg_num++;
	}

out_unlock:
	unlock_socket(socket);

	if (unlikely(err == -ETIMEDOUT))
		smu_is_hung[socket] = true;

	return err;
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
static int undef_link_width_fn(void)
{
	return -ENODEV;
}
static int (*__get_link_width)(void) __ro_after_init = undef_link_width_fn;

static int undef_link_speed_fn(void)
{
	return -ENODEV;
}
static int (*__get_link_speed)(void) __ro_after_init = undef_link_speed_fn;

int amd_get_tctl(int socket, u32 *tctl)
{
	int val;

	if (tctl == NULL || socket >= amd_num_sockets)
		return -EINVAL;

	val = __get_tctl(socket);
	if (val > 0) {
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
int amd_get_xgmi_width(int *width)
{
	int val;

	if (width == NULL)
		return -EINVAL;

	val = __get_link_width();
	if (val > 0) {
		*width = val;
		return 0;
	} else
		return val;
}
EXPORT_SYMBOL(amd_get_xgmi_width);

int amd_get_xgmi_speed(u32 *speed)
{
	u32 val;

	if (speed == NULL)
		return -EINVAL;

	val = __get_link_speed();
	if (val > 0) {
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

	if (unlikely(power_mw == NULL || socket >= amd_num_sockets))
		return -EINVAL;

	msg.msg_num     = HSMP_GET_SOCKET_POWER;
	msg.response_sz = 1;
	err = hsmp_send_message(socket, &msg);
	if (likely(!err))
		*power_mw = msg.response[0];

	return err;
}
EXPORT_SYMBOL(hsmp_get_power);

int hsmp_set_power_limit(int socket, u32 limit_mw)
{
	struct hsmp_message msg = { 0 };

	if (unlikely(socket >= amd_num_sockets))
		return -EINVAL;

	msg.msg_num  = HSMP_SET_SOCKET_POWER_LIMIT;
	msg.num_args = 1;
	msg.args[0]  = limit_mw;

	pr_info("Setting socket %d power limit to %u mW\n", socket, limit_mw);
	return hsmp_send_message(socket, &msg);
}
EXPORT_SYMBOL(hsmp_set_power_limit);

int hsmp_get_power_limit(int socket, u32 *limit_mw)
{
	int err;
	struct hsmp_message msg = { 0 };

	if (unlikely(limit_mw == NULL || socket >= amd_num_sockets))
		return -EINVAL;

	msg.msg_num     = HSMP_GET_SOCKET_POWER_LIMIT;
	msg.response_sz = 1;
	err = hsmp_send_message(socket, &msg);
	if (likely(!err))
		*limit_mw = msg.response[0];

	return err;
}
EXPORT_SYMBOL(hsmp_get_power_limit);

int hsmp_get_power_limit_max(int socket, u32 *limit_mw)
{
	int err;
	struct hsmp_message msg = { 0 };

	if (unlikely(limit_mw == NULL || socket >= amd_num_sockets))
		return -EINVAL;

	msg.msg_num     = HSMP_GET_SOCKET_POWER_LIMIT_MAX;
	msg.response_sz = 1;
	err = hsmp_send_message(socket, &msg);
	if (likely(!err))
		*limit_mw = msg.response[0];

	return err;
}
EXPORT_SYMBOL(hsmp_get_power_limit_max);

int hsmp_set_boost_limit_cpu(int cpu, u32 limit_mhz)
{
	int socket;
	struct hsmp_message msg = { 0 };

	socket       = cpu_data(cpu).phys_proc_id;
	msg.msg_num  = HSMP_SET_BOOST_LIMIT;
	msg.num_args = 1;
	msg.args[0]  = cpu_data(cpu).apicid << 16 | limit_mhz;

	pr_info("Setting CPU %d boost limit to %u MHz\n", cpu, limit_mhz);
	return hsmp_send_message(socket, &msg);
}
EXPORT_SYMBOL(hsmp_set_boost_limit_cpu);

int hsmp_set_boost_limit_socket(int socket, u32 limit_mhz)
{
	struct hsmp_message msg = { 0 };

	if (unlikely(socket >= amd_num_sockets))
		return -EINVAL;

	msg.msg_num  = HSMP_SET_BOOST_LIMIT_SOCKET;
	msg.num_args = 1;
	msg.args[0]  = limit_mhz;

	pr_info("Setting socket %d boost limit to %u MHz\n", socket,
		limit_mhz);
	return hsmp_send_message(socket, &msg);
}
EXPORT_SYMBOL(hsmp_set_boost_limit_socket);

int hsmp_set_boost_limit_system(u32 limit_mhz)
{
	int rc, socket;

	pr_info("Setting system boost limit to %u MHz\n", limit_mhz);
	for (socket = 0; socket < amd_num_sockets; socket++) {
		rc = hsmp_set_boost_limit_socket(socket, limit_mhz);
		if (rc) {
			pr_err("Could not set boost limit for socket %d\n",
			       socket);
			return rc;
		}
	}

	return 0;
}
EXPORT_SYMBOL(hsmp_set_boost_limit_system);

int hsmp_get_boost_limit_cpu(int cpu, u32 *limit_mhz)
{
	int err, socket;
	struct hsmp_message msg = { 0 };

	if (unlikely(limit_mhz == NULL))
		return -EINVAL;

	socket          = cpu_data(cpu).phys_proc_id;
	msg.msg_num     = HSMP_GET_BOOST_LIMIT;
	msg.num_args    = 1;
	msg.response_sz = 1;
	msg.args[0]     = cpu_data(cpu).apicid;

	err = hsmp_send_message(socket, &msg);
	if (likely(!err))
		*limit_mhz = msg.response[0];

	return err;
}
EXPORT_SYMBOL(hsmp_get_boost_limit_cpu);

int hsmp_get_proc_hot(int socket, bool *proc_hot)
{
	int err;
	struct hsmp_message msg = { 0 };

	if (unlikely(proc_hot == NULL || socket >= amd_num_sockets))
		return -EINVAL;

	msg.msg_num     = HSMP_GET_PROC_HOT;
	msg.response_sz = 1;
	err = hsmp_send_message(socket, &msg);
	if (likely(!err))
		*proc_hot = msg.response[0] ? true : false;

	return err;
}
EXPORT_SYMBOL(hsmp_get_proc_hot);

int hsmp_set_xgmi_link_width(int width)
{
	u8 width_min, width_max;
	int socket, _err;
	int err = 0;
	struct hsmp_message msg = { 0 };

	if (unlikely(amd_num_sockets < 2))
		return -ENODEV;

	switch (width) {
	case -1:
		width_min = 1;
		width_max = 2;
		pr_info("Enabling xGMI dynamic link width management\n");
		break;
	case 8:
		width_min = width_max = 1;
		pr_info("Setting xGMI link width to 8 lanes\n");
		break;
	case 16:
		width_min = width_max = 2;
		pr_info("Setting xGMI link width to 16 lanes\n");
		break;
	default:
		pr_err("Invalid link width specified: %d\n", width);
		return -EINVAL;
	}

	msg.msg_num  = HSMP_SET_XGMI_LINK_WIDTH;
	msg.num_args = 1;
	msg.args[0]  = (width_min << 8) | width_max;
	for (socket = 0; socket < amd_num_sockets; socket++) {
		_err = hsmp_send_message(socket, &msg);
		if (_err)
			err = _err;
	}

	return err;
}
EXPORT_SYMBOL(hsmp_set_xgmi_link_width);

int hsmp_set_df_pstate(int socket, int p_state)
{
	struct hsmp_message msg = { 0 };

	if (unlikely(socket >= amd_num_sockets))
		return -EINVAL;

	if (p_state == -1) {
		pr_info("Enabling socket %d auto data fabric P-states\n",
			socket);
		msg.msg_num = HSMP_AUTO_DF_PSTATE;
	} else if (p_state <= 3) {
		pr_info("Setting socket %d data fabric P-state to %d\n",
			socket, p_state);
		msg.num_args = 1;
		msg.msg_num  = HSMP_SET_DF_PSTATE;
		msg.args[0]  = p_state;
	} else {
		return -EINVAL;
	}

	return hsmp_send_message(socket, &msg);
}
EXPORT_SYMBOL(hsmp_set_df_pstate);

int hsmp_get_fabric_clocks(int socket, u32 *fclk, u32 *memclk)
{
	int err;
	struct hsmp_message msg = { 0 };

	if (unlikely((fclk == NULL && memclk == NULL) ||
		      socket >= amd_num_sockets))
		return -EINVAL;

	msg.msg_num     = HSMP_GET_FCLK_MCLK;
	msg.response_sz = 2;
	err = hsmp_send_message(socket, &msg);
	if (likely(!err)) {
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

	if (unlikely(max_mhz == NULL || socket >= amd_num_sockets))
		return -EINVAL;

	msg.msg_num     = HSMP_GET_CCLK_THROTTLE_LIMIT;
	msg.response_sz = 1;
	err = hsmp_send_message(socket, &msg);
	if (likely(!err))
		*max_mhz = msg.response[0];

	return err;
}
EXPORT_SYMBOL(hsmp_get_max_cclk);

int hsmp_get_c0_residency(int socket, u32 *residency)
{
	int err;
	struct hsmp_message msg = { 0 };

	if (unlikely(residency == NULL || socket > amd_num_sockets))
		return -EINVAL;

	msg.msg_num     = HSMP_GET_C0_PERCENT;
	msg.response_sz = 1;
	err = hsmp_send_message(socket, &msg);
	if (likely(!err))
		*residency = msg.response[0];

	return err;
}
EXPORT_SYMBOL(hsmp_get_c0_residency);

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

	for (socket = 0; socket < amd_num_sockets; socket++) {
		if (kobj == kobj_socket[socket])
			return socket;
	}
	return -1;
}

static int kobj_to_cpu(struct kobject *kobj)
{
	int cpu;

	for_each_present_cpu(cpu) {
		if (unlikely(kobj == kobj_cpu[cpu]))
			return cpu;
	}
	return -1;
}

static ssize_t boost_limit_store(struct kobject *kobj,
				 struct kobj_attribute *attr,
				 const char *buf, size_t count)
{
	int rc, socket, cpu;
	u32 limit_mhz = 0;

	/*
	 * TODO do we need to add a bounds check here? Assuming for now that
	 * SMU firmware handles any possible value we pass to it.
	 */
	rc = kstrtouint(buf, 10, &limit_mhz);
	if (rc || !limit_mhz) {
		pr_err("Invalid argument written to boost_limit: %s", buf);
		return -EINVAL;
	}

	socket = kobj_to_socket(kobj);
	cpu = kobj_to_cpu(kobj);

	/* Which file was written? */
	if (kobj == kobj_top)
		rc = hsmp_set_boost_limit_system(limit_mhz);
	else if (socket >= 0)
		rc = hsmp_set_boost_limit_socket(socket, limit_mhz);
	else if (cpu >= 0)
		rc = hsmp_set_boost_limit_cpu(cpu, limit_mhz);

	if (rc)
		return rc;

	return count;
}
static FILE_ATTR_WO(boost_limit);

static ssize_t boost_limit_show(struct kobject *kobj,
				struct kobj_attribute *attr,
				char *buf)
{
	u32 limit_mhz = 0;

	hsmp_get_boost_limit_cpu(kobj_to_cpu(kobj), &limit_mhz);
	return sprintf(buf, "%u\n", limit_mhz);
}
static FILE_ATTR_RW(boost_limit);

static ssize_t power_show(struct kobject *kobj,
			  struct kobj_attribute *attr, char *buf)
{
	u32 power_mw = 0;
	int rc;

	rc = hsmp_get_power(kobj_to_socket(kobj), &power_mw);
	if (rc)
		return rc;

	return sprintf(buf, "%u\n", power_mw);
}
static FILE_ATTR_RO(power);

static ssize_t power_limit_store(struct kobject *kobj,
				 struct kobj_attribute *attr,
				 const char *buf, size_t count)
{
	int socket = kobj_to_socket(kobj);
	u32 limit_mw = 0;
	int rc;

	/*
	 * TODO do we need to add a bounds check here? Assuming for now that
	 * SMU firmware handles any possible value we pass to it.
	 */
	rc = kstrtouint(buf, 10, &limit_mw);
	if (rc || !limit_mw) {
		pr_err("Invalid argument written to power_limit: %s", buf);
		return -EINVAL;
	}

	rc = hsmp_set_power_limit(socket, limit_mw);
	if (rc)
		return rc;

	return count;
}

static ssize_t power_limit_show(struct kobject *kobj,
				struct kobj_attribute *attr,
				char *buf)
{
	u32 limit_mw = 0;
	int rc;

	rc = hsmp_get_power_limit(kobj_to_socket(kobj), &limit_mw);
	if (rc)
		return rc;

	return sprintf(buf, "%u\n", limit_mw);
}
static FILE_ATTR_RW(power_limit);

static ssize_t power_limit_max_show(struct kobject *kobj,
				    struct kobj_attribute *attr,
				    char *buf)
{
	u32 limit_mw = 0;
	int rc;

	rc = hsmp_get_power_limit_max(kobj_to_socket(kobj), &limit_mw);
	if (rc)
		return rc;

	return sprintf(buf, "%u\n", limit_mw);
}
static FILE_ATTR_RO(power_limit_max);

static ssize_t proc_hot_show(struct kobject *kobj,
			     struct kobj_attribute *attr,
			     char *buf)
{
	bool proc_hot = false;
	int rc;

	rc = hsmp_get_proc_hot(kobj_to_socket(kobj), &proc_hot);
	if (rc)
		return rc;

	return sprintf(buf, "%s\n", proc_hot ? "active" : "inactive");
}
static FILE_ATTR_RO(proc_hot);

static ssize_t xgmi_width_store(struct kobject *kobj,
				struct kobj_attribute *attr,
				const char *buf, size_t count)
{
	int width = 0;
	int rc;

	 /* Logic below handles kstrtoint non-zero return val */
	rc = kstrtoint(buf, 10, &width);
	if (width != -1 && width != 8 && width != 16) {
		pr_info("Invalid value written to xgmi_width: %s", buf);
		return -EINVAL;
	}

	rc = hsmp_set_xgmi_link_width(width);
	if (rc)
		return rc;

	return count;
}

/*
 * Pending SMU register public availability, may need to remove xGMI width show
 * and xgmi_speed show functions below and restore RO attribute for the above
 * function.
 */
static ssize_t xgmi_width_show(struct kobject *kobj,
			       struct kobj_attribute *attr,
			       char *buf)
{
	int rc;
	int width = 0;

	rc = amd_get_xgmi_width(&width);
	if (rc)
		return rc;

	return sprintf(buf, "%d\n", width);
}
static FILE_ATTR_RW(xgmi_width);

static ssize_t xgmi_speed_show(struct kobject *kobj,
			       struct kobj_attribute *attr,
			       char *buf)
{
	int rc;
	int speed = 0;

	rc = amd_get_xgmi_speed(&speed);
	if (rc)
		return rc;

	return sprintf(buf, "%d\n", speed);
}
static FILE_ATTR_RO(xgmi_speed);

static ssize_t fabric_pstate_store(struct kobject *kobj,
				   struct kobj_attribute *attr,
				   const char *buf, size_t count)
{
	int socket = kobj_to_socket(kobj);
	int p_state = 0xFF;
	int rc;

	/*
	 * TODO do we need to add a bounds check here? Assuming for now that
	 * SMU firmware handles any possible value we pass to it.
	 */
	rc = kstrtoint(buf, 10, &p_state);
	if (rc || p_state < -1 || p_state > 3) {
		pr_err("Invalid argument written to fabric_pstate: %s", buf);
		return -EINVAL;
	}

	rc = hsmp_set_df_pstate(socket, p_state);
	if (rc)
		return rc;

	return count;
}
static FILE_ATTR_WO(fabric_pstate);

static ssize_t fabric_clocks_show(struct kobject *kobj,
				  struct kobj_attribute *attr,
				  char *buf)
{
	u32 fclk = 0, memclk = 0;
	int rc;

	rc = hsmp_get_fabric_clocks(kobj_to_socket(kobj), &fclk, &memclk);
	if (rc)
		return rc;

	return sprintf(buf, "%u,%u\n", fclk, memclk);
}
static FILE_ATTR_RO(fabric_clocks);

static ssize_t cclk_limit_show(struct kobject *kobj,
			       struct kobj_attribute *attr,
			       char *buf)
{
	u32 max_mhz = 0;
	int rc;

	rc = hsmp_get_max_cclk(kobj_to_socket(kobj), &max_mhz);
	if (rc)
		return rc;

	return sprintf(buf, "%u\n", max_mhz);
}
static FILE_ATTR_RO(cclk_limit);

static ssize_t c0_residency_show(struct kobject *kobj,
				 struct kobj_attribute *attr,
				 char *buf)
{
	u32 residency = 0;
	int rc;

	rc = hsmp_get_c0_residency(kobj_to_socket(kobj), &residency);
	if (rc)
		return rc;

	return sprintf(buf, "%u\n", residency);
}
static FILE_ATTR_RO(c0_residency);

static ssize_t tctl_show(struct kobject *kobj,
			 struct kobj_attribute *attr,
			 char *buf)
{
	int rc;
	u32 tctl = 0;

	rc = amd_get_tctl(kobj_to_socket(kobj), &tctl);
	if (rc)
		return rc;

	return sprintf(buf, "%u\n", tctl);
}
static FILE_ATTR_RO(tctl);

/* Entry point to set-up SysFS interface */
void __init amd_hsmp1_sysfs_init(void)
{
	struct kobject *kobj;
	int socket, cpu;
	char temp_name[8];
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
		 * change xgmi_width back to WO, and remove xgmi_speed.
		 */
		WARN_ON(sysfs_create_file(kobj_top, &xgmi_speed.attr));
		WARN_ON(sysfs_create_file(kobj_top, &rw_xgmi_width.attr));
	}

	/* Directory for each socket */
	for (socket = 0; socket < amd_num_sockets; socket++) {
		snprintf(temp_name, 8, "socket%d", socket);
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

		kobj_socket[socket] = kobj;
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
		snprintf(temp_name, 8, "cpu%d", cpu);
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
void __exit amd_hsmp1_sysfs_fini(void)
{
	int socket, cpu;

	if (!kobj_top)
		return;

	/* Remove files at top level directory */
	sysfs_remove_file(kobj_top, &smu_firmware_version.attr);
	sysfs_remove_file(kobj_top, &hsmp_protocol_version.attr);
	sysfs_remove_file(kobj_top, &boost_limit.attr);
	if (amd_num_sockets > 1) {
		/*
		 * Pending SMU register public availability, may need to
		 * change xgmi_width back to WO, and remove xgmi_speed.
		 */
		sysfs_remove_file(kobj_top, &xgmi_speed.attr);
		sysfs_remove_file(kobj_top, &rw_xgmi_width.attr);
	}

	/* Remove socket directories */
	for (socket = 0; socket < amd_num_sockets; socket++) {
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
		sysfs_remove_file(kobj_socket[socket], &tctl.attr);
		kobject_put(kobj_socket[socket]);
	}

	/* Remove CPU directories */
	if (kobj_cpu) {
		for_each_present_cpu(cpu) {
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
static int f17m30_get_tctl(int socket)
{
	int rc;
	u32 val;
	struct pci_dev *root = nb_root[socket];

	lock_socket(socket);

	rc = smu_pci_read(root, F17M30_SMU_THERM_CTRL, &val, &smu);
	if (rc) {
		pr_err("Error %d reading THERM_CTRL register\n", rc);
		goto out_unlock2;
	}

	pr_debug("Thermal Control raw val: %d\n", val);

	val >>= 24;
	val  &= 0xFF;
	rc = val;

out_unlock2:
	unlock_socket(socket);
	return rc;
}

/*
 * As mentioned above, the get width and get speed functions may have to
 * come out until the SMU registers are public.
 */
#define F17M30_SMU_XGMI2_G0_PCS_LINK_STATUS1	0x12EF0050
static int f17m30_get_xgmi2_width(void)
{
	int rc;
	u32 val;
	struct pci_dev *root = nb_root[0];

	lock_socket(0);

	rc = smu_pci_read(root, F17M30_SMU_XGMI2_G0_PCS_LINK_STATUS1,
			&val, &smu);
	if (rc) {
		pr_err("Error %d reading xGMI2 G0 PCS link status register\n",
				rc);
		goto out_unlock3;
	}

	val >>= 16;
	val  &= 0x3F;
	if (val > 4)
		rc = 16;
	else if (val > 2)
		rc = 8;
	else
		rc = 2;

out_unlock3:
	unlock_socket(0);
	return rc;
}

#define F17M30_SMU_XGMI2_G0_PCS_CONTEXT5	0x12EF0114
#define F17M30_SMU_FCH_PLL_CTRL0		0x02D02330
static int f17m30_get_xgmi2_speed(void)
{
	int rc, refclk;
	u32 freqcnt, refclksel;
	struct pci_dev *root = nb_root[0];

	lock_socket(0);

	rc = smu_pci_read(root, F17M30_SMU_XGMI2_G0_PCS_CONTEXT5,
			&freqcnt, &smu);
	if (rc) {
		pr_err("Error %d reading xGMI2 G0 PCS context register\n", rc);
		goto out_unlock4;
	}
	freqcnt >>= 4;
	freqcnt  &= 0x7F;

	rc = smu_pci_read(root, F17M30_SMU_FCH_PLL_CTRL0, &refclksel, &smu);
	if (rc) {
		pr_err("Error %d reading reference clock select\n", rc);
		goto out_unlock4;
	}
	if (refclksel & 0xFF)
		refclk = 133;
	else
		refclk = 100;

	rc = freqcnt * 2 * refclk;

out_unlock4:
	unlock_socket(0);
	return rc;
}

#define AMD17H_P0_NBIO_BUS_NUM		0x00
#define AMD17H_P1_NBIO_BUS_NUM		0x80
static u32 amd_nbio_bus_num[] = {AMD17H_P0_NBIO_BUS_NUM,
				 AMD17H_P1_NBIO_BUS_NUM};

static int f17h_m30h_init(void)
{
	struct pci_dev *root = NULL;
	struct pci_bus *bus  = NULL;
	int i, bus_num;

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
	 * Family 17h model 30 has four North Bridges per socket. We can't
	 * count on every kernel to report the number correctly (what if the
	 * kernel hasn't been updated for Rome?) so let's count them here
	 * to determine the number of sockets. We also can't assume fixed
	 * bus number for 1P vs. 2P - 1P has 0x00, 0x40, 0x80 and 0xC0 and
	 * the one we want is 0x00. 2P has 0x00, 0x20, 0x40 ... 0xE0 and
	 * the two we want are 0x00 and 0x80.
	 */
	for (bus_num = 0xE0; bus_num >= 0x00; bus_num -= 0x20) {
		bus = pci_find_bus(0, bus_num);
		if (bus)
			amd_num_sockets++;
	}

	amd_num_sockets >>= 2;
	pr_info("Detected %d socket(s)\n", amd_num_sockets);

	for (i = 0; i < amd_num_sockets; i++) {
		bus = pci_find_bus(0, amd_nbio_bus_num[i]);
		if (!bus) {
			pr_warn("Failed to find PCI root bus for socket %d\n",
				i);
			return -ENODEV;
		}

		root = pci_get_slot(bus, 0);
		if (!root) {
			pr_warn("Failed to find NBIO PCI device for socket %d\n",
				i);
			return -ENODEV;
		}

		nb_root[i] = root;
	}

	/* Pending SMU register public availability */
	if (amd_num_sockets > 1) {
		__get_link_width = f17m30_get_xgmi2_width;
		__get_link_speed = f17m30_get_xgmi2_speed;
	}

	return 0;
}

/* Decrement reference count for NBIO PCI devs if needed */
static void put_pci_devs(void)
{
	int socket;

	for (socket = 0; socket < MAX_SOCKETS; socket++)
		if (nb_root[socket])
			pci_dev_put(nb_root[socket]);
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

	/*
	 * Check each port to be safe. The test message takes one argument and
	 * returns the value of that argument + 1. The protocol version and SMU
	 * version messages take no arguments and return one.
	 */
	msg.args[0]     = 0xDEADBEEF;
	msg.response_sz = 1;
	for (socket = 0; socket < amd_num_sockets; socket++) {
		msg.msg_num  = HSMP_TEST;
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

	msg.msg_num  = HSMP_GET_SMU_VER;
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

	if (amd_hsmp_proto_ver != 1) {
		pr_err("Unsupported protocol version\n");
		return -ENODEV;
	}

	return 0;
}

static int __init hsmp_init(void)
{
	int err;

	pr_info("%s version %s\n", DRV_MODULE_DESCRIPTION, DRV_MODULE_VERSION);

	err = hsmp_probe();
	if (err) {
		put_pci_devs();
		return err;
	}

	if (amd_hsmp_proto_ver == 1)
		amd_hsmp1_sysfs_init();

	return 0;
}

static void __exit hsmp_exit(void)
{
	pr_info("Driver unload\n");
	amd_hsmp1_sysfs_fini();
	put_pci_devs();
}

module_init(hsmp_init);
module_exit(hsmp_exit);
