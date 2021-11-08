// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2021 Advanced Micro Devices, Inc.
 * Author: Nathan Fontenot <nathan.fontenot@amd.com>
 * Author: Lewis Carroll <lewis.carroll@amd.com>
 */

/*
 * AMD Host System Management Port driver
 *
 * SysFS interface
 *
 * Parent directory: /sys/devices/platform/amd_hsmp/
 * General structure:
 *
 * amd_hsmp/cpuX/         Directory for each possible CPU
 *     boost_limit        (RW) HSMP boost limit for the core in MHz
 *
 * amd_hsmp/socketX/      Directory for each possible socket
 *     nbioX/		  Directory for each NBIO
 *         nbio_pstate	  (WO) Set PCI-e bus interface P-state
 *         bus		  (RO) Bus range for NBIO
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
 * P-state (APBDIS=1). Write a value of -1 to enable autonomous data fabric
 * P-state selection (APBDIS=0).
 *
 * fabric_clocks returns two comma separated values. The first is the fabric
 * clock (FCLK) in MHz, and the second is the memory clock (MCLK) in MHz.
 * These clocks are set by the current data fabric P-state.
 *
 * tctl is NOT the socket temperature. tctl is NOT temperature. tctl is a
 * unitless figure with a value from 0 - 100, where 100 usually means the
 * processor will initiate PROC_HOT actions and 95 usually means the processor
 * will begin thermal throttling actions.
 *
 * xgmi_pstate will only exist on 2P platforms. Write a value from 0 to 2 to set
 * a specific link P-state. Write a value of -1 to enable autonomous link
 * width selection. Link P-state 0 corresponds to a link width of 16 lanes and
 * link P-state 1 corresponds to 8 lanes. For family 19h only, a link P-state of
 * 2 corresponds to a link width of 2 lanes.
 *
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
#include <linux/nospec.h>
#include <linux/platform_device.h>
#include <asm/amd_nb.h>
#include <asm/cpu_device_id.h>

#include "amd_hsmp.h"

#define DRV_MODULE_DESCRIPTION	"AMD Host System Management Port driver"
#define DRV_MODULE_VERSION	"1.0"

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION(DRV_MODULE_DESCRIPTION);
MODULE_VERSION(DRV_MODULE_VERSION);

static int raw_intf;
module_param(raw_intf, int, 0);
MODULE_PARM_DESC(raw_intf, "Enable raw HSMP interface");

static int f17h_support;
module_param(f17h_support, int, 0);
MODULE_PARM_DESC(f17h_support, "Support AMD Family 17h");

/* Highest HSMP protocol supported by driver */
#define HSMP_SUPPORTED_PROTO	3

/*
 * All protocol versions are required to support
 * these four status / error codes
 */
#define HSMP_STATUS_NOT_READY	0x00
#define HSMP_STATUS_OK		0x01
#define HSMP_ERR_INVALID_MSG	0xFE
#define HSMP_ERR_INVALID_ARGS	0xFF

#ifndef __ro_after_init
#define __ro_after_init __read_mostly
#endif

#define HSMP_MSG_REG		0x3B10534
#define HSMP_STATUS_REG		0x3B10980
#define HSMP_DATA_REG		0x3B109E0
#define HSMP_TIMEOUT		500

struct nbio_dev {
	struct pci_dev	*dev;		/* Pointer to PCI-e device */
	struct kobject	*kobj;		/* Sysfs entry */
	int		socket_id;	/* Physical socket number */
	u8		bus_base;	/* PCI-e bus base */
	u8		bus_limit;	/* PCI-e bus limit */
	u8		id;		/* NBIO tile within the socket */
};

static struct nbio_dev *nbios;

struct socket {
	struct pci_dev	*dev;		/* Pointer to PCI-e device */
	struct kobject	*kobj;		/* SysFS directory */
	struct mutex	mutex;		/* lock to serialize reads/writes */
	int		id;
	bool		hung;
};

static struct socket *sockets;

static struct kobject **kobj_cpu __ro_after_init;
static struct bin_attribute **hsmp_raw_battrs;
static u32 amd_hsmp_proto_ver __ro_after_init;
struct platform_device *amd_hsmp_pdev;

static bool hsmp_enabled;
static int num_sockets;
static int num_nbios;

union amd_smu_firmware amd_smu_fw __ro_after_init;
EXPORT_SYMBOL_GPL(amd_smu_fw);

/*
 * Message types
 *
 * All implementations are required to support HSMP_TEST, HSMP_GET_SMU_VER,
 * and HSMP_GET_PROTO_VER. All other messages are implementation dependent.
 */
enum hsmp_msg_t {
	HSMP_TEST				=  1,
	HSMP_GET_SMU_VER			=  2,
	HSMP_GET_PROTO_VER			=  3,
	HSMP_GET_SOCKET_POWER			=  4,
	HSMP_SET_SOCKET_POWER_LIMIT		=  5,
	HSMP_GET_SOCKET_POWER_LIMIT		=  6,
	HSMP_GET_SOCKET_POWER_LIMIT_MAX		=  7,
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
	HSMP_SET_NBIO_DPM_LEVEL			= 18,
	HSMP_GET_DDR_BANDWIDTH			= 20,
};

struct hsmp_message {
	enum hsmp_msg_t	msg_num;	/* Message number */
	u16		num_args;	/* Number of arguments in message */
	u16		response_sz;	/* Number of expected response words */
	u32		args[8];	/* Argument(s) */
	u32		response[8];	/* Response word(s) */
};

#define is_amd_fam_19h()	(boot_cpu_data.x86 == 0x19)

/*
 * SMN access functions
 * Must be called with the socket mutex held. Returns 0 on success, negative
 * error code on failure. The return status is for the SMN access, not the
 * result of the intended SMN or HSMP operation.
 *
 * SMN PCI config space access method
 * There are two access apertures defined in the PCI-e config space for the
 * North Bridge, one for general purpose SMN register reads/writes and a second
 * aperture specific for HSMP messages and responses. For both reads and writes,
 * step one is to write the register to be accessed to the appropriate aperture
 * index register. Step two is to read or write the appropriate aperture data
 * register.
 */
struct smn_pci_port {
	u32 index_reg;	/* PCI-e index register for SMN access */
	u32 data_reg;	/* PCI-e data register for SMN access */
};

static struct smn_pci_port smn_port = {
	.index_reg = 0x60,
	.data_reg  = 0x64,
};

static struct smn_pci_port hsmp_port = {
	.index_reg = 0xC4,
	.data_reg  = 0xC8,
};

enum smn_rdwr {
	SMN_READ,
	SMN_WRITE
};

static int __smn_rdwr(struct pci_dev *root, u32 addr, u32 *data,
		      struct smn_pci_port *port, enum smn_rdwr rdwr)
{
	int err;

	err = pci_write_config_dword(root, port->index_reg, addr);
	if (err)
		return err;

	if (rdwr == SMN_READ)
		err = pci_read_config_dword(root, port->data_reg, data);
	else
		err = pci_write_config_dword(root, port->data_reg, *data);

	return err;
}

static struct socket *get_nbio_socket(int socket_id)
{
	struct socket *socket = NULL;

	if (socket_id >= 0 && socket_id < num_sockets) {
		socket_id = array_index_nospec(socket_id, num_sockets);
		socket = &sockets[socket_id];
	}

	return socket;
}

static int smn_read(int socket_id, u32 addr, u32 *data)
{
	struct socket *socket;
	int err;

	if (!hsmp_enabled)
		return -EOPNOTSUPP;

	socket = get_nbio_socket(socket_id);
	if (!socket)
		return -ENODEV;

	mutex_lock(&socket->mutex);
	err = __smn_rdwr(socket->dev, addr, data, &smn_port, SMN_READ);
	mutex_unlock(&socket->mutex);

	return err;
}

static int hsmp_read(struct socket *socket, u32 addr, u32 *data)
{
	return __smn_rdwr(socket->dev, addr, data, &hsmp_port, SMN_READ);
}

static int hsmp_write(struct socket *socket, u32 addr, u32 data)
{
	return __smn_rdwr(socket->dev, addr, &data, &hsmp_port, SMN_WRITE);
}

static int hsmp_send_msg_error(int status, int socket_id,
			       struct hsmp_message *msg)
{
	int err;

	switch (status) {
	case HSMP_STATUS_NOT_READY:
		pr_err("SMU timeout for message ID %u on socket %d\n",
		       msg->msg_num, socket_id);
		err = -ETIMEDOUT;
		break;
	case HSMP_ERR_INVALID_MSG:
		pr_err("Invalid message ID %u on socket %d\n",
		       msg->msg_num, socket_id);
		err = -EOPNOTSUPP;
		break;
	case HSMP_ERR_INVALID_ARGS:
		pr_err("Message ID %u failed on socket %d\n",
		       msg->msg_num, socket_id);
		err = -EINVAL;
		break;
	default:
		pr_err("Message ID %u unknown failure (status = 0x%X) on socket %d\n",
		       msg->msg_num, status, socket_id);
		err = -EIO;
		break;
	}

	return err;
}

/*
 * Send a message to the SMN access port via PCI-e config space registers.
 * The caller is expected to zero out any unused arguments. If a response
 * is expected, the number of response words should be greater than 0.
 * Returns 0 for success and populates the requested number of arguments
 * in the passed struct. Returns a negative error code for failure.
 */
static int __hsmp_send_msg(struct socket *socket, struct hsmp_message *msg)
{
	unsigned long short_sleep, timeout;
	u32 status;
	int i, err;

	/* Zero the status register */
	status = HSMP_STATUS_NOT_READY;
	err = hsmp_write(socket, HSMP_STATUS_REG, status);
	if (err) {
		pr_err("Error %d clearing status register on socket %d\n",
		       err, socket->id);
		return err;
	}

	/* Write any message arguments */
	for (i = 0; i < msg->num_args; i++) {
		hsmp_write(socket, HSMP_DATA_REG + (i << 2), msg->args[i]);
		if (err) {
			pr_err("Error %d writing message argument %d on socket %d\n",
			       err, i, socket->id);
			return err;
		}
	}

	/* Write the message ID which starts the operation */
	err = hsmp_write(socket, HSMP_MSG_REG, msg->msg_num);
	if (err) {
		pr_err("Error %d writing message ID %u on socket %d\n",
		       err, msg->msg_num, socket->id);
		return err;
	}

	/*
	 * Depending on when the trigger write completes relative to the SMU
	 * firmware 1 ms cycle, the operation may take from tens of us to 1 ms
	 * to complete. Some operations may take more. Therefore we will try
	 * a few short duration sleeps and switch to long sleeps if we don't
	 * succeed quickly.
	 */
	short_sleep = jiffies + nsecs_to_jiffies(500);
	timeout = jiffies + msecs_to_jiffies(HSMP_TIMEOUT);

	while (time_before(jiffies, timeout)) {
		err = hsmp_read(socket, HSMP_STATUS_REG, &status);
		if (err) {
			pr_err("Message ID %u - error %d reading status on socket %d\n",
			       err, msg->msg_num, socket->id);
			return err;
		}

		if (status == HSMP_STATUS_NOT_READY) {
			if (time_before(jiffies, short_sleep))
				usleep_range(25, 50);
			else
				usleep_range(1000, 2000);

			continue;
		}

		break;
	}

	if (status != HSMP_STATUS_OK)
		return hsmp_send_msg_error(status, socket->id, msg);

	/* SMU has responded OK. Read response data */
	for (i = 0; i < msg->response_sz; i++) {
		err = hsmp_read(socket, HSMP_DATA_REG + (i << 2),
				&msg->response[i]);
		if (err) {
			pr_err("Error %d reading response %u for message ID %u on socket %d\n",
			       err, i, msg->msg_num, socket->id);
			return err;
		}
	}

	return 0;
}

static int hsmp_send_message(int socket_id, struct hsmp_message *msg)
{
	struct socket *socket;
	int i, err;

	if (!hsmp_enabled)
		return -EOPNOTSUPP;

	socket = get_nbio_socket(socket_id);
	if (!socket)
		return -ENODEV;

	/*
	 * In the unlikely case the SMU hangs, don't bother sending
	 * any more messages on this socket.
	 */
	if (socket->hung)
		return -ETIMEDOUT;

	pr_debug("Socket %d sending message ID %d\n", socket_id, msg->msg_num);
	for (i = 0; i < msg->num_args; i++)
		pr_debug("    arg[%d:] 0x%08X\n", i, msg->args[i]);

	mutex_lock(&socket->mutex);
	err = __hsmp_send_msg(socket, msg);
	mutex_unlock(&socket->mutex);

	if (err == -ETIMEDOUT)
		socket->hung = true;

	return err;
}

static int hsmp_get_one_value(int socket_id, enum hsmp_msg_t msg_id, u32 *val)
{
	struct hsmp_message msg = { 0 };
	int err;

	if (!val)
		return -EINVAL;

	msg.msg_num     = msg_id;
	msg.response_sz = 1;

	err = hsmp_send_message(socket_id, &msg);
	if (err)
		return err;

	*val = msg.response[0];
	return 0;
}

static int hsmp_set_one_value(int socket_id, enum hsmp_msg_t msg_id, u32 val)
{
	struct hsmp_message msg = { 0 };

	msg.msg_num  = msg_id;
	msg.num_args = 1;
	msg.args[0]  = val;

	return hsmp_send_message(socket_id, &msg);
}

static struct nbio_dev *bus_to_nbio(u8 bus_num)
{
	int i;

	for (i = 0; i < num_nbios; i++) {
		if (bus_num >= nbios[i].bus_base &&
		    bus_num <= nbios[i].bus_limit)
			return &nbios[i];
	}

	return NULL;
}

#define SMN_THERM_CTRL 0x00059800

int amd_get_tctl(int socket_id, u32 *tctl)
{
	int err;
	u32 val;

	if (!tctl)
		return -EINVAL;

	err = smn_read(socket_id, SMN_THERM_CTRL, &val);
	if (err)
		return err;

	*tctl = (val >> 24) & 0xFF;
	return 0;
}
EXPORT_SYMBOL_GPL(amd_get_tctl);

#define SMN_XGMI2_G0_PCS_LINK_STATUS1	0x12EF0050
#define XGMI_LINK_WIDTH_X2		BIT(1)
#define XGMI_LINK_WIDTH_X8		BIT(2)
#define XGMI_LINK_WIDTH_X16		BIT(5)

int amd_get_xgmi_pstate(int *pstate)
{
	int err;
	u32 val;

	if (!pstate)
		return -EINVAL;

	err = smn_read(0, SMN_XGMI2_G0_PCS_LINK_STATUS1, &val);
	if (err)
		return err;

	val = (val >> 16) & 0x3F;

	if (val & XGMI_LINK_WIDTH_X16) {
		*pstate = 0;
		return 0;
	} else if (val & XGMI_LINK_WIDTH_X8) {
		*pstate = 1;
		return 0;
	} else if ((val & XGMI_LINK_WIDTH_X2) && is_amd_fam_19h()) {
		*pstate = 2;
		return 0;
	}

	return -1;
}
EXPORT_SYMBOL_GPL(amd_get_xgmi_pstate);

#define SMN_XGMI2_G0_PCS_CONTEXT5	0x12EF0114
#define SMN_FCH_PLL_CTRL0		0x02D02330
#define REF_CLK_100MHZ			0x00
#define REF_CLK_133MHZ			0x55

int amd_get_xgmi_speed(u32 *speed)
{
	u32 freqcnt, refclksel;
	int err;

	if (!speed)
		return -EINVAL;

	err = smn_read(0, SMN_XGMI2_G0_PCS_CONTEXT5, &freqcnt);
	if (err)
		return err;

	err = smn_read(0, SMN_FCH_PLL_CTRL0, &refclksel);
	if (err)
		return err;

	freqcnt = (freqcnt >> 3) & 0xFE;
	refclksel &= 0xFF;

	if (refclksel == REF_CLK_100MHZ) {
		*speed = freqcnt * 100;
		return 0;
	} else if (refclksel == REF_CLK_133MHZ) {
		*speed = freqcnt * 133;
		return 0;
	}

	return -1;
}
EXPORT_SYMBOL_GPL(amd_get_xgmi_speed);

int hsmp_get_power(int socket_id, u32 *power_mw)
{
	return hsmp_get_one_value(socket_id, HSMP_GET_SOCKET_POWER, power_mw);
}
EXPORT_SYMBOL_GPL(hsmp_get_power);

int hsmp_set_power_limit(int socket_id, u32 limit_mw)
{
	int err;

	err = hsmp_set_one_value(socket_id, HSMP_SET_SOCKET_POWER_LIMIT,
				 limit_mw);
	if (err)
		return err;

	pr_info("Socket %d power limit set to %u mW\n", socket_id, limit_mw);
	return 0;
}
EXPORT_SYMBOL_GPL(hsmp_set_power_limit);

int hsmp_get_power_limit(int socket_id, u32 *limit_mw)
{
	return hsmp_get_one_value(socket_id, HSMP_GET_SOCKET_POWER_LIMIT,
				  limit_mw);
}
EXPORT_SYMBOL_GPL(hsmp_get_power_limit);

int hsmp_get_power_limit_max(int socket_id, u32 *limit_mw)
{
	return hsmp_get_one_value(socket_id, HSMP_GET_SOCKET_POWER_LIMIT_MAX,
				  limit_mw);
}
EXPORT_SYMBOL_GPL(hsmp_get_power_limit_max);

int hsmp_set_boost_limit_cpu(int cpu, u16 limit_mhz)
{
	int err, socket_id;
	u32 boost_limit_arg;

	if (!cpu_present(cpu))
		return -ENODEV;

	socket_id = cpu_data(cpu).phys_proc_id;
	boost_limit_arg = cpu_data(cpu).apicid << 16 | limit_mhz;

	err = hsmp_set_one_value(socket_id, HSMP_SET_BOOST_LIMIT,
				 boost_limit_arg);
	if (err)
		return err;

	pr_info("Set CPU %d boost limit to %u MHz\n", cpu, limit_mhz);
	return err;
}
EXPORT_SYMBOL_GPL(hsmp_set_boost_limit_cpu);

int hsmp_set_boost_limit_socket(int socket_id, u16 limit_mhz)
{
	int err;

	err = hsmp_set_one_value(socket_id, HSMP_SET_BOOST_LIMIT_SOCKET,
				 limit_mhz);
	if (err)
		return err;

	pr_info("Set socket %d boost limit to %u MHz\n", socket_id, limit_mhz);
	return err;
}
EXPORT_SYMBOL_GPL(hsmp_set_boost_limit_socket);

int hsmp_set_boost_limit_system(u16 limit_mhz)
{
	int socket_id, _err;
	int err = 0;

	for (socket_id = 0; socket_id < num_sockets; socket_id++) {
		_err = hsmp_set_boost_limit_socket(socket_id, limit_mhz);
		if (_err)
			err = _err;
	}

	return err;
}
EXPORT_SYMBOL_GPL(hsmp_set_boost_limit_system);

int hsmp_get_boost_limit_cpu(int cpu, u32 *limit_mhz)
{
	struct hsmp_message msg = { 0 };
	int err, socket_id;

	if (!limit_mhz)
		return -EINVAL;

	if (!cpu_present(cpu))
		return -ENODEV;

	socket_id       = cpu_data(cpu).phys_proc_id;
	msg.msg_num     = HSMP_GET_BOOST_LIMIT;
	msg.num_args    = 1;
	msg.response_sz = 1;
	msg.args[0]     = cpu_data(cpu).apicid;

	err = hsmp_send_message(socket_id, &msg);
	if (err)
		return err;

	*limit_mhz = msg.response[0];
	return err;
}
EXPORT_SYMBOL_GPL(hsmp_get_boost_limit_cpu);

int hsmp_get_proc_hot(int socket_id, u32 *proc_hot)
{
	return hsmp_get_one_value(socket_id, HSMP_GET_PROC_HOT, proc_hot);
}
EXPORT_SYMBOL_GPL(hsmp_get_proc_hot);

int hsmp_set_xgmi_pstate(int pstate)
{
	const char *xgmi_width_msg;
	u8 width_min, width_max;
	u32 pstate_settings;
	int socket_id, _err;
	int err = 0;

	if (num_sockets < 2)
		return -ENODEV;

	switch (pstate) {
	case -1:
		width_min = is_amd_fam_19h() ? 0 : 1;
		width_max = 2;
		xgmi_width_msg = "dynamic management";
		break;
	case 0:
		width_min = 2;
		width_max = 2;
		xgmi_width_msg = "16 lanes";
		break;
	case 1:
		width_min = 1;
		width_max = 1;
		xgmi_width_msg = "8 lanes";
		break;
	case 2:
		if (is_amd_fam_19h()) {
			width_min = 0;
			width_max = 0;
			xgmi_width_msg = "2 lanes";
		} else {
			err = -EINVAL;
		}

		break;
	default:
		err = -EINVAL;
		break;
	}

	if (err)
		return err;

	pstate_settings = (width_min << 8) | width_max;

	for (socket_id = 0; socket_id < num_sockets; socket_id++) {
		_err = hsmp_set_one_value(socket_id, HSMP_SET_XGMI_LINK_WIDTH,
					  pstate_settings);
		if (_err)
			err = _err;
		else
			pr_info("Socket %d xGMI link width set to %s\n", socket_id,
				xgmi_width_msg);
	}

	return err;
}
EXPORT_SYMBOL_GPL(hsmp_set_xgmi_pstate);

int hsmp_set_df_pstate(int socket_id, int pstate)
{
	struct hsmp_message msg = { 0 };
	int err;

	if (pstate < -1 || pstate > 3)
		return -EINVAL;

	if (pstate == -1) {
		msg.msg_num = HSMP_AUTO_DF_PSTATE;
	} else {
		msg.num_args = 1;
		msg.msg_num  = HSMP_SET_DF_PSTATE;
		msg.args[0]  = pstate;
	}

	err = hsmp_send_message(socket_id, &msg);
	if (err)
		return err;

	pr_info("Set socket %d data fabric P-state to %d\n", socket_id, pstate);
	return 0;
}
EXPORT_SYMBOL_GPL(hsmp_set_df_pstate);

int hsmp_get_fabric_clocks(int socket_id, u32 *fclk, u32 *memclk)
{
	struct hsmp_message msg = { 0 };
	int err;

	if (!fclk && !memclk)
		return -EINVAL;

	msg.msg_num     = HSMP_GET_FCLK_MCLK;
	msg.response_sz = 2;

	err = hsmp_send_message(socket_id, &msg);
	if (err)
		return err;

	if (fclk)
		*fclk = msg.response[0];
	if (memclk)
		*memclk = msg.response[1];

	return 0;
}
EXPORT_SYMBOL_GPL(hsmp_get_fabric_clocks);

int hsmp_get_max_cclk(int socket_id, u32 *max_mhz)
{
	return hsmp_get_one_value(socket_id, HSMP_GET_CCLK_THROTTLE_LIMIT,
				  max_mhz);
}
EXPORT_SYMBOL_GPL(hsmp_get_max_cclk);

int hsmp_get_c0_residency(int socket_id, u32 *residency)
{
	return hsmp_get_one_value(socket_id, HSMP_GET_C0_PERCENT,
				  residency);
}
EXPORT_SYMBOL_GPL(hsmp_get_c0_residency);

int hsmp_set_nbio_pstate(u8 bus_num, int pstate)
{
	struct nbio_dev *nbio;
	u8 dpm_min, dpm_max;
	u32 pstate_settings;
	int err;

	nbio = bus_to_nbio(bus_num);
	if (!nbio)
		return -ENODEV;

	/*
	 * DPM level 1 is not currently used. DPM level 2 is the max for
	 * non-ESM devices and DPM level 3 is the max for ESM devices.
	 */
	switch (pstate) {
	case -1:
		dpm_min = 0;
		dpm_max = 2;
		break;
	case 0:
		dpm_min = 2;
		dpm_max = 2;
		break;
	case 1:
		dpm_min = 0;
		dpm_max = 0;
		break;
	default:
		return -EINVAL;
	}

	pstate_settings = (nbio->id << 16) | (dpm_max << 8) | dpm_min;
	err = hsmp_set_one_value(nbio->socket_id, HSMP_SET_NBIO_DPM_LEVEL,
				 pstate_settings);
	if (err)
		return err;

	if (dpm_min == dpm_max)
		pr_info("Set bus 0x%02X (socket %d NBIO %d) to P-state %d\n",
			bus_num, nbio->socket_id, nbio->id, pstate);
	else
		pr_info("Enabled bus 0x%02X (socket %d NBIO %d) auto P-state\n",
			bus_num, nbio->socket_id, nbio->id);

	return 0;
}
EXPORT_SYMBOL_GPL(hsmp_set_nbio_pstate);

int hsmp_get_ddr_bandwidth(int socket_id, struct hsmp_ddr_bw *ddr_bw)
{
	int err;
	u32 bw;

	if (amd_hsmp_proto_ver < 3)
		return -EOPNOTSUPP;

	err = hsmp_get_one_value(socket_id, HSMP_GET_DDR_BANDWIDTH, &bw);
	if (err)
		return err;

	ddr_bw->max_bandwidth = bw >> 19;
	ddr_bw->utilized_bandwidth = (bw >> 7) & 0xFFF;
	ddr_bw->percent_utilized = bw & 0x7F;

	return 0;
}
EXPORT_SYMBOL_GPL(hsmp_get_ddr_bandwidth);

/* SysFS interfaces */
#define HSMP_ATTR_WO(_name)	static struct kobj_attribute _name = __ATTR_WO(_name)
#define HSMP_ATTR_RO(_name)	static struct kobj_attribute _name = __ATTR_RO(_name)
#define HSMP_ATTR_RW(_name)	static struct kobj_attribute rw_##_name = __ATTR_RW(_name)

static int kobj_to_socket(struct kobject *kobj)
{
	int socket_id;

	for (socket_id = 0; socket_id < num_sockets; socket_id++)
		if (kobj == sockets[socket_id].kobj)
			return socket_id;

	return -1;
}

static int kobj_to_cpu(struct kobject *kobj)
{
	int cpu;

	for_each_present_cpu(cpu) {
		if (kobj == kobj_cpu[cpu])
			return cpu;
	}

	return -1;
}

static struct nbio_dev *kobj_to_nbio(struct kobject *kobj)
{
	int i;

	for (i = 0; i < num_nbios; i++) {
		if (nbios[i].kobj == kobj)
			return &nbios[i];
	}

	return NULL;
}

#define hsmp_socket_attr_show(_name, _fn)					\
static ssize_t _name##_show(struct kobject *kobj, struct kobj_attribute *attr,	\
			    char *buf)						\
{                                                                               \
	u32 val;								\
	int err;								\
										\
	err = _fn(kobj_to_socket(kobj), &val);					\
	if (err)								\
		return err;							\
										\
	return sprintf(buf, "%u\n", val);					\
}

static ssize_t smu_firmware_version_show(struct kobject *kobj,
					 struct kobj_attribute *attr,
					 char *buf)
{
	return sprintf(buf, "%u.%u.%u\n", amd_smu_fw.ver.major,
		       amd_smu_fw.ver.minor, amd_smu_fw.ver.debug);
}
HSMP_ATTR_RO(smu_firmware_version);

static ssize_t hsmp_protocol_version_show(struct kobject *kobj,
					  struct kobj_attribute *attr,
					  char *buf)
{
	return sprintf(buf, "%u\n", amd_hsmp_proto_ver);
}
HSMP_ATTR_RO(hsmp_protocol_version);

hsmp_socket_attr_show(power, hsmp_get_power);
HSMP_ATTR_RO(power);

static ssize_t power_limit_store(struct kobject *kobj,
				 struct kobj_attribute *attr,
				 const char *buf, size_t count)
{
	int socket_id = kobj_to_socket(kobj);
	u32 limit_mw;
	int err;

	err = kstrtouint(buf, 10, &limit_mw);
	if (err)
		return err;

	err = hsmp_set_power_limit(socket_id, limit_mw);
	if (err)
		return err;

	return count;
}

hsmp_socket_attr_show(power_limit, hsmp_get_power_limit);
HSMP_ATTR_RW(power_limit);

hsmp_socket_attr_show(power_limit_max, hsmp_get_power_limit_max);
HSMP_ATTR_RO(power_limit_max);

static ssize_t boost_limit_store(struct kobject *kobj,
				 struct kobj_attribute *attr,
				 const char *buf, size_t count)
{
	int err, socket_id, cpu;
	u16 limit_mhz;

	err = kstrtou16(buf, 10, &limit_mhz);
	if (err)
		return err;

	socket_id = kobj_to_socket(kobj);
	cpu = kobj_to_cpu(kobj);

	/* Which file was written? */
	if (kobj == &amd_hsmp_pdev->dev.kobj)
		err = hsmp_set_boost_limit_system(limit_mhz);
	else if (socket_id >= 0)
		err = hsmp_set_boost_limit_socket(socket_id, limit_mhz);
	else if (cpu >= 0)
		err = hsmp_set_boost_limit_cpu(cpu, limit_mhz);

	if (err)
		return err;

	return count;
}
HSMP_ATTR_WO(boost_limit);

static ssize_t boost_limit_show(struct kobject *kobj,
				struct kobj_attribute *attr,
				char *buf)
{
	u32 limit_mhz;
	int err;

	err = hsmp_get_boost_limit_cpu(kobj_to_cpu(kobj), &limit_mhz);
	if (err)
		return err;

	return sprintf(buf, "%u\n", limit_mhz);
}
HSMP_ATTR_RW(boost_limit);

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
HSMP_ATTR_RO(proc_hot);

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
HSMP_ATTR_RW(xgmi_pstate);

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
HSMP_ATTR_RO(xgmi_speed);

static ssize_t fabric_pstate_store(struct kobject *kobj,
				   struct kobj_attribute *attr,
				   const char *buf, size_t count)
{
	int socket_id = kobj_to_socket(kobj);
	int err, pstate;

	err = kstrtoint(buf, 10, &pstate);
	if (err)
		return err;

	err = hsmp_set_df_pstate(socket_id, pstate);
	if (err)
		return err;

	return count;
}
HSMP_ATTR_WO(fabric_pstate);

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
HSMP_ATTR_RO(fabric_clocks);

hsmp_socket_attr_show(cclk_limit, hsmp_get_max_cclk);
HSMP_ATTR_RO(cclk_limit);

hsmp_socket_attr_show(c0_residency, hsmp_get_c0_residency);
HSMP_ATTR_RO(c0_residency);

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

	err = hsmp_set_nbio_pstate(nbio->bus_base, pstate);
	if (err)
		return err;

	return count;
}
HSMP_ATTR_WO(nbio_pstate);

static ssize_t nbio_bus_show(struct kobject *kobj, struct kobj_attribute *attr,
			     char *buf)
{
	struct nbio_dev *nbio;

	nbio = kobj_to_nbio(kobj);
	if (!nbio)
		return -EINVAL;

	return sprintf(buf, "%02x-%02x\n", nbio->bus_base, nbio->bus_limit);
}
HSMP_ATTR_RO(nbio_bus);

hsmp_socket_attr_show(tctl, amd_get_tctl)
HSMP_ATTR_RO(tctl);

#define ddr_attr_show(_name)						\
static ssize_t ddr_##_name##_show(struct kobject *kobj,			\
				  struct kobj_attribute *attr,		\
				  char *buf)				\
{									\
	struct hsmp_ddr_bw ddr_bw;					\
	int err;							\
									\
	err = hsmp_get_ddr_bandwidth(kobj_to_socket(kobj), &ddr_bw);	\
	if (err)							\
		return err;						\
									\
	return sprintf(buf, "%u\n", ddr_bw._name);			\
}									\
HSMP_ATTR_RO(ddr_##_name)						\

ddr_attr_show(max_bandwidth);
ddr_attr_show(utilized_bandwidth);
ddr_attr_show(percent_utilized);

static struct hsmp_message raw_hsmp_msgs[2];

static ssize_t hsmp_raw_intf_show(struct file *fp, struct kobject *kobj,
				  struct bin_attribute *battr, char *buf,
				  loff_t off, size_t count)
{
	return memory_read_from_buffer(buf, battr->size, &off, battr->private,
				       battr->size);
}

static ssize_t hsmp_raw_intf_store(struct file *fp, struct kobject *kobj,
				   struct bin_attribute *battr, char *buf,
				   loff_t off, size_t count)
{
	struct hsmp_message msg;
	int rc;

	if (count < battr->size)
		return -EINVAL;

	memcpy(&msg, buf, battr->size);

	rc = hsmp_send_message(kobj_to_socket(kobj), &msg);
	if (rc)
		return rc;

	memcpy(battr->private, &msg, battr->size);
	return battr->size;
}

static void add_hsmp_raw_intf(struct kobject *kobj, int socket_id)
{
	struct bin_attribute *attr;
	int rc;

	attr = kzalloc(sizeof(*attr), GFP_KERNEL);
	if (!attr)
		return;

	sysfs_bin_attr_init(attr);
	attr->attr.name = "hsmp_raw";
	attr->attr.mode = 0600;
	attr->read = hsmp_raw_intf_show;
	attr->write = hsmp_raw_intf_store;
	attr->private = &raw_hsmp_msgs[socket_id];
	attr->size = sizeof(struct hsmp_message);

	rc = sysfs_create_bin_file(kobj, attr);
	if (rc) {
		kfree(attr);
		return;
	}

	hsmp_raw_battrs[socket_id] = attr;
}

static struct attribute *hsmp_attrs[] = {
	&smu_firmware_version.attr,
	&hsmp_protocol_version.attr,
	&boost_limit.attr,
	NULL,
};
ATTRIBUTE_GROUPS(hsmp);

static struct attribute *hsmp_multisocket_attrs[] = {
	&xgmi_speed.attr,
	&rw_xgmi_pstate.attr,
	NULL,
};
ATTRIBUTE_GROUPS(hsmp_multisocket);

static struct attribute *hsmp_nbio_attrs[] = {
	&nbio_pstate.attr,
	&nbio_bus.attr,
	NULL,
};
ATTRIBUTE_GROUPS(hsmp_nbio);

static struct attribute *hsmp_socket_attrs[] = {
	&boost_limit.attr,
	&power.attr,
	&rw_power_limit.attr,
	&power_limit_max.attr,
	&proc_hot.attr,
	&fabric_pstate.attr,
	&fabric_clocks.attr,
	&cclk_limit.attr,
	&c0_residency.attr,
	&tctl.attr,
	NULL,
};
ATTRIBUTE_GROUPS(hsmp_socket);

static struct attribute *hsmp_socket_v4_attrs[] = {
	&ddr_max_bandwidth.attr,
	&ddr_utilized_bandwidth.attr,
	&ddr_percent_utilized.attr,
	NULL,
};
ATTRIBUTE_GROUPS(hsmp_socket_v4);

static int __init hsmp_sysfs_init(struct platform_device *pdev)
{
	int socket_id, cpu, i;
	struct kobject *kobj;
	char temp_name[16];
	ssize_t size;
	int err;

	err = sysfs_create_groups(&pdev->dev.kobj, hsmp_groups);
	if (err)
		return err;

	if (num_sockets > 1) {
		err = sysfs_create_groups(&pdev->dev.kobj, hsmp_multisocket_groups);
		if (err)
			return err;
	}

	if (raw_intf) {
		hsmp_raw_battrs = kcalloc(num_sockets, sizeof(*hsmp_raw_battrs),
					  GFP_KERNEL);
		if (!hsmp_raw_battrs) {
			pr_info("Could not create raw interface\n");
			raw_intf = 0;
		}
	}

	/* Directory for each socket */
	for (socket_id = 0; socket_id < num_sockets; socket_id++) {
		snprintf(temp_name, 16, "socket%d", socket_id);
		kobj = kobject_create_and_add(temp_name, &pdev->dev.kobj);
		if (!kobj)
			return -ENOMEM;

		err = sysfs_create_groups(kobj, hsmp_socket_groups);
		if (err)
			return err;

		/* directory for each NBIO */
		for (i = 0; i < num_nbios; i++) {
			struct kobject *nbio_kobj;

			if (nbios[i].socket_id != socket_id)
				continue;

			snprintf(temp_name, 6, "nbio%d", nbios[i].id);
			nbio_kobj = kobject_create_and_add(temp_name, kobj);
			if (!nbio_kobj)
				return -ENOMEM;

			err = sysfs_create_groups(nbio_kobj, hsmp_nbio_groups);
			if (err)
				return err;

			nbios[i].kobj = nbio_kobj;
		}

		if (amd_hsmp_proto_ver >= 3) {
			err = sysfs_create_groups(kobj, hsmp_socket_v4_groups);
			if (err)
				return err;
		}

		if (raw_intf)
			add_hsmp_raw_intf(kobj, socket_id);

		sockets[socket_id].kobj = kobj;
	}

	/*
	 * Directory for each present CPU. Note we are using present CPUs, not
	 * possible CPUs since some BIOSs are buggy w.r.t. the list of possible
	 * CPUs (e.g. if SMT disabled in BIOS, the SMT siblings are still
	 * listed as possible even though they can't be brought online without
	 * a reboot). Note HSMP doesn't care about online vs. offline CPUs.
	 */
	size = num_present_cpus() * sizeof(struct kobject *);
	kobj_cpu = kzalloc(size, GFP_KERNEL);
	if (!kobj_cpu)
		return -ENOMEM;

	for_each_present_cpu(cpu) {
		snprintf(temp_name, 16, "cpu%d", cpu);
		kobj_cpu[cpu] = kobject_create_and_add(temp_name, &pdev->dev.kobj);
		if (!kobj_cpu[cpu])
			return -ENOMEM;

		err = sysfs_create_file(kobj_cpu[cpu], &rw_boost_limit.attr);
		if (err)
			return err;
	}

	return 0;
}

static void __exit hsmp_sysfs_fini(struct platform_device *pdev)
{
	int socket_id, cpu, i;
	struct kobject *kobj;

	/* Remove files at top level directory */
	sysfs_remove_groups(&pdev->dev.kobj, hsmp_groups);

	if (num_sockets > 1)
		sysfs_remove_groups(&pdev->dev.kobj, hsmp_multisocket_groups);

	/* Remove socket directories */
	for (socket_id = 0; socket_id < num_sockets; socket_id++) {
		kobj = sockets[socket_id].kobj;
		if (!kobj)
			continue;

		/* remove per-NBIO dirs */
		for (i = 0; i < num_nbios; i++) {
			if (nbios[i].socket_id != socket_id)
				continue;

			sysfs_remove_groups(nbios[i].kobj, hsmp_nbio_groups);
			kobject_put(nbios[i].kobj);
		}

		sysfs_remove_groups(kobj, hsmp_socket_groups);

		if (amd_hsmp_proto_ver >= 3)
			sysfs_remove_groups(kobj, hsmp_socket_v4_groups);

		if (raw_intf && hsmp_raw_battrs[socket_id]) {
			sysfs_remove_bin_file(kobj, hsmp_raw_battrs[socket_id]);
			kfree(hsmp_raw_battrs[socket_id]);
		}

		kobject_put(kobj);
	}

	kfree(hsmp_raw_battrs);

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
}

static void hsmp_cleanup(void)
{
	int i;

	hsmp_enabled = false;

	if (amd_hsmp_pdev) {
		hsmp_sysfs_fini(amd_hsmp_pdev);
		platform_device_unregister(amd_hsmp_pdev);
	}

	if (sockets) {
		for (i = 0; i < num_sockets; i++)
			mutex_destroy(&sockets[i].mutex);

		kfree(sockets);
	}

	kfree(nbios);
}

#define F19h_IOHC_DEVID			0x1480
#define SMN_IOHCMISC0_NB_BUS_NUM_CNTL	0x13B10044  /* Address in SMN space */
#define SMN_IOHCMISC_OFFSET		0x00100000  /* Offset for MISC[1..3] */

static int init_nbio_devs(void)
{
	struct pci_dev *dev = NULL;
	int i;

	for_each_pci_dev(dev) {
		if (dev->vendor == PCI_VENDOR_ID_AMD &&
		    dev->device == F19h_IOHC_DEVID)
			num_nbios++;
	}

	if (num_nbios == 0) {
		pr_err("No NBIO IOHC devices found\n");
		return -EOPNOTSUPP;
	}

	nbios = kcalloc(num_nbios, sizeof(struct nbio_dev), GFP_KERNEL);
	if (!nbios)
		return -ENOMEM;

	i = 0;
	for_each_pci_dev(dev) {
		if (dev->vendor == PCI_VENDOR_ID_AMD &&
		    dev->device == F19h_IOHC_DEVID) {
			if (i == num_nbios) {
				pr_err("Found more than %d IOHCs- giving up\n", num_nbios);
				pci_dev_put(dev);
				return -EOPNOTSUPP;
			}

			nbios[i].dev = dev;
			nbios[i++].bus_base = dev->bus->number;
		}
	}

	/* Sort NBIOs by bus */
	for (i = 0; i < num_nbios - 1; i++) {
		int j;

		for (j = i + 1; j < num_nbios; j++) {
			if (nbios[j].bus_base < nbios[i].bus_base) {
				struct nbio_dev temp_nbio = nbios[i];

				nbios[i] = nbios[j];
				nbios[j] = temp_nbio;
			}
		}
	}

	/* Calculate bus limits */
	for (i = 0; i < num_nbios; i++)
		nbios[i].bus_limit = nbios[i + 1].bus_base - 1;
	nbios[num_nbios - 1].bus_limit = 0xff;

	return 0;
}

static int init_sockets(void)
{
	int i;

	num_sockets = amd_nb_num();
	sockets = kcalloc(num_sockets, sizeof(struct socket), GFP_KERNEL);
	if (!sockets)
		return -ENOMEM;

	for (i = 0; i < num_sockets; i++) {
		struct amd_northbridge *nb;

		nb = node_to_amd_nb(i);

		sockets[i].dev = nb->root;
		sockets[i].id = i;
		mutex_init(&sockets[i].mutex);
	}

	return 0;
}

static int do_hsmp_init(void)
{
	int nbios_per_socket;
	int i, err;

	/*
	 * Here we need to build a table of the NBIO devices in the system.
	 * This will be done by iterating through all PCI devices on the
	 * system looking for IOHC devices (DevID = 0x1480). Once we have
	 * the devices and their bus numbers we can then map them to the
	 * socket and NBIO ID. This is done by reading the four IOHCMISC[0..3]
	 * registers per socket - where for each of these IOHCMISCx devices,
	 * x is the NBIO ID within the socket. Within the IOHCMISCx space
	 * there is a register NB_BUS_NUM_CNTL that holds the base bus number.
	 * This is mapped to a bus num we already found to allow us to finish
	 * building the map.
	 */
	err = init_nbio_devs();
	if (err)
		return err;

	err = init_sockets();
	if (err)
		return err;

	/* Get IOHC ID for each bus base */
	nbios_per_socket = num_nbios / num_sockets;

	for (i = 0; i < num_nbios; i++) {
		int nbio_id, socket_id;
		struct nbio_dev *nbio;
		u32 addr, val;
		u8 base;

		nbio_id = i % nbios_per_socket;
		socket_id = i / nbios_per_socket;

		addr = SMN_IOHCMISC0_NB_BUS_NUM_CNTL +
		       nbio_id * SMN_IOHCMISC_OFFSET;

		err = smn_read(socket_id, addr, &val);
		if (err) {
			pr_err("Error %d accessing socket %d IOHCMISC%d\n",
			       err, socket_id, nbio_id);
			return -ENODEV;
		}

		base = val & 0xFF;

		/* Look up this bus base in our array */
		nbio = bus_to_nbio(base);
		if (!nbio) {
			pr_err("Unable to map bus 0x%02X to an IOHC device\n",
			       base);
			return -ENODEV;
		}

		nbio->socket_id = socket_id;
		nbio->id        = nbio_id;
	}

	for (i = 0; i < num_nbios; i++)
		pr_debug("NBIO%d pci bus 0x%02X - 0x%02x --> Socket %d\n",
			 nbios[i].id, nbios[i].bus_base, nbios[i].bus_limit,
			 nbios[i].socket_id);

	return 0;
}

/*
 * Check if HSMP is supported by attempting a test message. If successful,
 * retrieve the HSMP protocol version and SMU firmware version.
 * Returns 0 for success
 * Returns -ENODEV if probe or test message fails.
 */
static int __init hsmp_probe(void)
{
	struct hsmp_message msg = { 0 };
	int socket_id;
	int err = 0;

	/*
	 * Check each port to be safe. The test message takes one argument and
	 * returns the value of that argument + 1.
	 */
	msg.args[0]     = 0xDEADBEEF;
	msg.response_sz = 1;
	for (socket_id = 0; socket_id < num_sockets; socket_id++) {
		msg.msg_num = HSMP_TEST;
		msg.num_args = 1;

		err = hsmp_send_message(socket_id, &msg);
		if (err) {
			if ((err == -ETIMEDOUT) || (err == -ETIMEDOUT))
				pr_err("HSMP appears to be disabled by the system firmware\n");
			return err;
		}

		if (msg.response[0] != msg.args[0] + 1) {
			pr_err("Socket %d test message failed, Expected 0x%08X, received 0x%08X\n",
			       socket_id, msg.args[0] + 1, msg.response[0]);
			return -EBADE;
		}
	}

	err = hsmp_get_one_value(0, HSMP_GET_SMU_VER, &amd_smu_fw.raw_u32);
	if (err) {
		pr_err("Could not determine SMU firmware version\n");
		return err;
	}

	hsmp_get_one_value(0, HSMP_GET_PROTO_VER, &amd_hsmp_proto_ver);
	if (err) {
		pr_err("Could not determine HSMP protocol version\n");
		return err;
	}

	return 0;
}

#define AMD_MATCH_FAM(_family)						\
	X86_MATCH_VENDOR_FAM_MODEL_STEPPINGS_FEATURE(AMD, _family,	\
						     X86_MODEL_ANY,	\
						     X86_STEPPING_ANY,	\
						     X86_FEATURE_ANY, 0)

static const struct x86_cpu_id amd_hsmp_cpuids[] = {
	AMD_MATCH_FAM(0x17),
	AMD_MATCH_FAM(0x19),
	{},
};
MODULE_DEVICE_TABLE(x86cpu, amd_hsmp_cpuids);

static int __init hsmp_init(void)
{
	const struct x86_cpu_id *m;
	int err;

	m = x86_match_cpu(amd_hsmp_cpuids);
	if (!m || (m->family == 0x17 && !f17h_support))
		return -EOPNOTSUPP;

	/*
	 * Assume HSMP is enabled on the platform, we need to do
	 * this so we can read SMN registers during initialization.
	 */
	hsmp_enabled = true;

	err = do_hsmp_init();
	if (err)
		goto hsmp_init_fail;

	err = hsmp_probe();
	if (err)
		goto hsmp_init_fail;

	amd_hsmp_pdev = platform_device_alloc("amd_hsmp", PLATFORM_DEVID_NONE);
	if (!amd_hsmp_pdev) {
		pr_err("Failed to allocate HSMP platform device\n");
		err = -EINVAL;
		goto hsmp_init_fail;
	}

	err = platform_device_add(amd_hsmp_pdev);
	if (err) {
		pr_err("Failed to register platform driver\n");
		goto hsmp_init_fail;
	}

	err = hsmp_sysfs_init(amd_hsmp_pdev);
	if (err) {
		pr_err("Failed to initialize sysfs files\n");
		goto hsmp_init_fail;
	}

	pr_info("%s version %s\n", DRV_MODULE_DESCRIPTION, DRV_MODULE_VERSION);
	pr_info("HSMP Protocol version %u, SMU firmware version %u.%u.%u\n",
		amd_hsmp_proto_ver, amd_smu_fw.ver.major,
		amd_smu_fw.ver.minor, amd_smu_fw.ver.debug);

	if (amd_hsmp_proto_ver > HSMP_SUPPORTED_PROTO)
		pr_warn("Driver supports HSMP protocol v%d, not all functions will be available\n",
			HSMP_SUPPORTED_PROTO);

	return 0;

hsmp_init_fail:
	hsmp_cleanup();
	return err;
}

static void __exit hsmp_exit(void)
{
	pr_info("Driver unload\n");
	hsmp_cleanup();
}

module_init(hsmp_init);
module_exit(hsmp_exit);
