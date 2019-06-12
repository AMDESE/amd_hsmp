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
 */

#include <linux/kernel.h>

#define MAX_SOCKETS 2

struct hsmp_message {
	u32	msg_num;	// Message number
	u16	num_args;	// NUmber of arguments in message
	u16	response_sz;	// Number of expected response words
	u32	args[8];	// Argument(s)
	u32	response[8];	// Response word(s)
};

/*
 * All protocol versions are required to support these
 * three messages and these four status / error codes
 */
#define HSMP_TEST		   1
#define HSMP_GET_SMU_VER	   2
#define HSMP_GET_PROTO_VER	   3
#define HSMP_STATUS_NOT_READY	0x00
#define HSMP_STATUS_OK		0x01
#define HSMP_ERR_INVALID_MSG	0xFE	
#define HSMP_ERR_REQUEST_FAIL	0xFF

typedef int (*hsmp_send_message_t)(int, struct hsmp_message *);

/*
 * Expand as needed to cover all access ports types.
 * Current definition is for PCI-e config space access.
 */
struct smu_access {
	u32	index_reg;	// Trigger register for SMU access port
	u32	data_reg;	// Data register for SMU access port
	u32	mbox_msg_id;	// SMU register for HSMP message ID
	u32	mbox_status;	// SMU register for HSMP status word
	u32	mbox_data;	// SMU base register for HSMP argument(s)
	u32	mbox_timeout;	// Timeout in MS to consider the SMU hung
};

struct smu_fw {
	u8	debug;		// Debug version number
	u8	minor;		// Minor version number
	u8	major;		// Major version number
	u8	unused;
};
