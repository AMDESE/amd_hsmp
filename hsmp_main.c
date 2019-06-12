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
 * Module probe, init, exit
 * Basic hardware access routines
 */

#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/ktime.h>
#include <linux/timekeeping.h>
#include <linux/delay.h>
#include <linux/module.h>
#include "hsmp_types.h"

//#define HSMP_DEBUG
//#define HSMP_DEBUG_PCI

#define DRV_MODULE_DESCRIPTION	"AMD Host System Management Port driver"
#define DRV_MODULE_VERSION	"0.3"

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Lewis Carroll <lewis.carroll@amd.com>");
MODULE_DESCRIPTION(DRV_MODULE_DESCRIPTION);
MODULE_VERSION(DRV_MODULE_VERSION);

static struct smu_access hsmp;
static hsmp_send_message_t hsmp_send_message;

u32 amd_smu_fw_ver;
u32 amd_hsmp_proto_ver;

/* Serialize access to the HSMP mailbox */
static DEFINE_MUTEX(hsmp_lock_socket0);
static DEFINE_MUTEX(hsmp_lock_socket1);
static struct pci_dev *nb_root[MAX_SOCKETS] = { NULL };	/* Pointer to North Bridge */

/* Callback functions to init send message method for each protocol */
extern void amd_hsmp1_init(hsmp_send_message_t);

#ifdef CONFIG_SYSFS
extern void __init amd_hsmp1_sysfs_init(void);
extern void __exit amd_hsmp1_sysfs_fini(void);
#endif

/*
 * SMU access functions
 * Must be called with hsmp_lock held. Returns 0 on success,
 * negative error code on failure. This status is for the SMU
 * access, not the result of the intended HSMP operation.
 */

/*
 * SMU PCI config space access method. To write an SMU register, write the register
 * address to the index register then write the register value to the data register.
 * To read an SMU register, write the register address to the index register then
 * read the data register.
 */
static inline int smu_pci_write(struct pci_dev *root, u32 reg_addr, u32 reg_data)
{
	int err;

#ifdef HSMP_DEBUG_PCI
	pr_info("  pci_write_config_dword addr 0x%08X, data 0x%08X\n", hsmp.index_reg, reg_addr);
#endif
	if (unlikely(err = pci_write_config_dword(root, hsmp.index_reg, reg_addr)))
		return err;
#ifdef HSMP_DEBUG_PCI
	pr_info("  pci_write_config_dword addr 0x%08X, data 0x%08X\n", hsmp.data_reg, reg_data);
#endif
	if (unlikely(err = pci_write_config_dword(root, hsmp.data_reg, reg_data)))
		return err;

	return 0;
}

static inline int smu_pci_read(struct pci_dev *root, u32 reg_addr, u32 *reg_data)
{
	int err;

#ifdef HSMP_DEBUG_PCI
	pr_info("  pci_write_config_dword addr 0x%08X, data 0x%08X\n", hsmp.index_reg, reg_addr);
#endif
	if (unlikely(err = pci_write_config_dword(root, hsmp.index_reg, reg_addr)))
		return err;

	if (unlikely(err = pci_read_config_dword(root, hsmp.data_reg, reg_data)))
		return err;
#ifdef HSMP_DEBUG_PCI
	pr_info("  pci_read_config_dword  addr 0x%08X, data 0x%08X\n", hsmp.data_reg, *reg_data);
#endif
	return 0;
}

/*
 * Send a message to the SMU access port via PCI-e config space registers.
 * The caller is expected to zero out any unused arguments.
 * If a response is expected, the number of response words should be greater than 0.
 * Returns 0 for success and populates the requested number of arguments in the passed
 * struct. Returns a negative error code for failure.
 */
static int send_message_pci(int socket, struct hsmp_message *msg)
{
	struct pci_dev *root = nb_root[socket];
	struct timespec64 ts, tt;
	int err;
	u32 mbox_status;
	unsigned int arg_num = 0;
#ifdef HSMP_DEBUG
	int retries = 0;
#endif
	static bool smu_is_hung[MAX_SOCKETS] = { false };

	/*
	 * In the unlikely case the SMU hangs, don't bother sending
	 * any more messages to the SMU on this socket.
	 */
	if (unlikely(smu_is_hung[socket]))
		return -ETIMEDOUT;

#ifdef HSMP_DEBUG
	pr_info("HSMP: Socket %d sending message ID %d\n", socket, msg->msg_num);
        if (msg->num_args) {
		pr_info("      args: 0x%08X", msg->args[arg_num++]);
		while (arg_num < msg->num_args) pr_info(KERN_CONT "  0x%08X", msg->args[arg_num++]);
		pr_info(KERN_CONT "\n");
	}
	arg_num = 0;
#endif

	if (socket == 0)
		mutex_lock(&hsmp_lock_socket0);
	else
		mutex_lock(&hsmp_lock_socket1);

	/* Zero the status register */
	mbox_status = HSMP_STATUS_NOT_READY;
	if (unlikely(err = smu_pci_write(root, hsmp.mbox_status, mbox_status))) {
		pr_err("HSMP: Error %d clearing mailbox status register on socket %d\n",
				err, socket);
		goto out_unlock;
	}

	/* Write any message arguments */
	while (arg_num < msg->num_args) {
		if (unlikely(err = smu_pci_write(root, hsmp.mbox_data + (arg_num << 2), msg->args[arg_num]))) {
			pr_err("HSMP: Error %d writing message argument %d on socket %d\n",
					err, arg_num, socket);
			goto out_unlock;
		}
		arg_num++;
	}

	/* Write the message ID which starts the operation */
	if (unlikely(err = smu_pci_write(root, hsmp.mbox_msg_id, msg->msg_num))) {
		pr_err("HSMP: Error %d writing message ID %u on socket %d\n",
				err, msg->msg_num, socket);
		goto out_unlock;
	}

	/* Pre-calculate the time-out */
	ktime_get_real_ts64(&ts);
	tt = ts;
	timespec64_add_ns(&tt, hsmp.mbox_timeout * NSEC_PER_MSEC);

	/*
	 * Assume it takes at least one SMU FW cycle (1 MS) to complete the operation.
	 * Some operations might complete in two, some in more. So first thing we do is
	 * yield the CPU.
	 */
retry:
	yield();	// Don't hog the CPU
	if (unlikely(err = smu_pci_read(root, hsmp.mbox_status, &mbox_status))) {
		pr_err("HSMP: Message ID %u - error %d reading mailbox status on socket %d\n",
				err, msg->msg_num, socket);
		goto out_unlock;
	}
	if (mbox_status == HSMP_STATUS_NOT_READY) {
		/* SMU has not responded to the message yet */
		struct timespec64 tv;
		ktime_get_real_ts64(&tv);
		if (unlikely(timespec64_compare(&tv, &tt) > 0)) {
			pr_err(KERN_CRIT "HSMP: SMU timeout for message ID %u on socket %d\n",
				msg->msg_num, socket);
			err = -ETIMEDOUT;
			goto out_unlock;
		}
#ifdef HSMP_DEBUG
		retries++;
#endif
		goto retry;
	}

	/* SMU has responded - check for error */
#ifdef HSMP_DEBUG
	ktime_get_real_ts64(&tt);
	tt = timespec64_sub(tt, ts);
	pr_info("HSMP: Socket %d message ack after %u ns, %d retries\n",
			socket, ((unsigned)timespec64_to_ns(&tt)), retries);
#endif

	if (unlikely(mbox_status == HSMP_ERR_INVALID_MSG)) {
		pr_err("HSMP: Invalid message ID %u on socket %d\n", msg->msg_num, socket);
		err = -ENOMSG;
		goto out_unlock;
	} else if (unlikely(mbox_status == HSMP_ERR_REQUEST_FAIL)) {
		pr_err("HSMP: Message ID %u failed on socket %d\n", msg->msg_num, socket);
		err = -EFAULT;
		goto out_unlock;
	} else if (unlikely(mbox_status != HSMP_STATUS_OK)) {
		pr_err("HSMP: Message ID %u unknown failure (status = 0x%X) on socket %d\n",
				msg->msg_num, mbox_status, socket);
		err = -EIO;
		goto out_unlock;
	}

	/* SMU has responded OK. Read response data */
	arg_num = 0;
	while (arg_num < msg->response_sz) {
		if (unlikely(err = smu_pci_read(root, hsmp.mbox_data + (arg_num << 2), &msg->response[arg_num]))) {
			pr_err("HSMP: Error %d reading response %u for message ID %u on socket %d\n",
					err, arg_num, msg->msg_num, socket);
			goto out_unlock;
		}
		arg_num++;
	}

out_unlock:
	if (socket == 0)
		mutex_unlock(&hsmp_lock_socket0);
	else
		mutex_unlock(&hsmp_lock_socket1);

	if (unlikely(err == -ETIMEDOUT))
		smu_is_hung[socket] = true;

	return err;
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

/* Zen 2 - Rome. HSMP access is via PCI-e config space data / index register pair */
#define PCI_DEVICE_ID_AMD_17H_M30H_ROOT	0x1480
#define AMD17H_P0_NBIO_BUS_NUM		0x00
#define AMD17H_P1_NBIO_BUS_NUM		0x80
static int f17h_m30h_init (void)
{
	struct pci_dev *root = NULL;
	struct pci_bus *bus  = NULL;

	hsmp.index_reg    = 0xC4;	// Offset in config space
	hsmp.data_reg     = 0xC8;	// Offset in config space
	hsmp.mbox_msg_id  = 0x3B10534;
	hsmp.mbox_status  = 0x3B10980;
	hsmp.mbox_data    = 0x3B109E0;
	hsmp.mbox_timeout = 500;

	hsmp_send_message = &send_message_pci;

	pr_info("HSMP: Detected family 17h model 30h-30f CPU (Rome)\n");

	if (!(bus = pci_find_bus(0, AMD17H_P0_NBIO_BUS_NUM))) {
		pr_warn("HSMP: Failed to find PCI root bus for socket 0\n");
		return -ENODEV;
	}
	if (!(root = pci_get_slot(bus, 0))) {
		pr_warn("HSMP: Failed to find NBIO PCI device for socket 0\n");
		return -ENODEV;
	}
	nb_root[0] = root;
	if (topology_max_packages() == 1)
		return 0;

	if (!(bus = pci_find_bus(0, AMD17H_P1_NBIO_BUS_NUM))) {
		pr_warn("HSMP: Failed to find PCI root bus for socket 1\n");
		return -ENODEV;
	}
	if (!(root = pci_get_slot(bus, 0))){
		pr_warn("HSMP: Failed to find NBIO PCI device for socket 1\n");
		return -ENODEV;
	}
	nb_root[1] = root;

	return 0;
}

/* Decrement reference count for NBIO PCI devs if needed */
static void put_pci_devs (void)
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

	/* Call set-up function for supported CPUs and drop through to probe function */
	if (c->x86 == 0x17 && c->x86_model >= 0x30 && c->x86_model <= 0x3F) {
		if ((err = f17h_m30h_init()))	// Zen 2 - Rome
			return err;
	} else	// Add additional supported family / model combinations
		return -ENODEV;

	/*
	 * Check each port to be safe. The test message takes one argument and
	 * returns the value of that argument + 1. The protocol version and SMU
	 * version messages take no arguments and return one.
	 */
	msg.args[0]     = 0xDEADBEEF;
	msg.response_sz = 1;
	for (socket = 0; socket < topology_max_packages(); socket++) {
		msg.msg_num  = HSMP_TEST;
		msg.num_args = 1;
		if ((_err = hsmp_send_message(socket, &msg))) {
			hsmp_bad = 1;
			err = _err;
			continue;
		}

		if (msg.response[0] != msg.args[0] + 1) {
			pr_err("HSMP: Socket %d test message failed, "
					"Expected 0x%08X, received 0x%08X\n",
					socket, msg.args[0] + 1, msg.response[0]);
			hsmp_bad = 1;
			err = -EBADE;
		}
	}

	msg.msg_num  = HSMP_GET_SMU_VER;
	msg.num_args = 0;
	if ((_err = hsmp_send_message(0, &msg))) {
		hsmp_bad = 1;
		err = _err;
	}
	amd_smu_fw_ver = msg.response[0];

	msg.msg_num  = HSMP_GET_PROTO_VER;
	if ((_err = hsmp_send_message(0, &msg))) {
		hsmp_bad = 1;
		err = _err;
	}
	amd_hsmp_proto_ver = msg.response[0];

	if (hsmp_bad)
		return err;

	smu_fw_ver = (struct smu_fw *)&amd_smu_fw_ver;
	pr_info("HSMP: Protocol version %u, SMU firmware version %u.%u.%u\n",
			amd_hsmp_proto_ver, smu_fw_ver->major,
			smu_fw_ver->minor, smu_fw_ver->debug);

	if (amd_hsmp_proto_ver != 1) {
		pr_err("HSMP: Unsupported protocol version\n");
		return -ENODEV;
	}

	return 0;
}

static int __init hsmp_init(void)
{
	int err;

	pr_info("HSMP: %s version %s\n", DRV_MODULE_DESCRIPTION, DRV_MODULE_VERSION);

	if ((err = hsmp_probe())) {
		put_pci_devs();
		return err;
	}

	/*
	 * Tell the protocol handler how to talk to the port. We do it this way
	 * to avoid otherwise exposing the send_message function to the kernel.
	 */
	if (amd_hsmp_proto_ver == 1) {
		amd_hsmp1_init(hsmp_send_message);
#ifdef CONFIG_SYSFS
		amd_hsmp1_sysfs_init();
#endif
	}

	return 0;
}

static void __exit hsmp_exit(void)
{
	pr_info("HSMP: Driver unload\n");
#ifdef CONFIG_SYSFS
	amd_hsmp1_sysfs_fini();
#endif
	put_pci_devs();
}

module_init(hsmp_init);
module_exit(hsmp_exit);
