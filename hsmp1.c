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
 * Protocol version 1 messages and functions
 */

#include <linux/kernel.h>
#include <asm-generic/errno.h>
#include <asm/processor.h>
#include <asm/topology.h>
#include "hsmp_types.h"
//#include <asm/amd_hsmp1.h>	If in tree
#include "amd_hsmp1.h"

/* Port access method */
static hsmp_send_message_t hsmp_send_message;

/* Message types */
#define HSMP1_GET_SOCKET_POWER			 4
#define HSMP1_SET_SOCKET_POWER_LIMIT		 5
#define HSMP1_GET_SOCKET_POWER_LIMIT		 6
#define HSMP1_GET_SOCKET_POWER_LIMIT_MAX	 7
#define HSMP1_SET_BOOST_LIMIT			 8
#define HSMP1_SET_BOOST_LIMIT_SOCKET		 9
#define HSMP1_GET_BOOST_LIMIT			10
#define HSMP1_GET_PROC_HOT			11
#define HSMP1_SET_XGMI2_LINK_WIDTH		12
#define HSMP1_SET_DF_PSTATE			13
#define HSMP1_AUTO_DF_PSTATE			14
#define HSMP1_GET_FCLK_MCLK			15
#define HSMP1_GET_CCLK_THROTTLE_LIMIT		16
#define HSMP1_GET_C0_PERCENT			17

/* Message ID 4 */
int hsmp1_get_power(int socket, u32 *power_mw)
{
	int err;
	struct hsmp_message msg = { 0 };

	if (unlikely(power_mw == NULL || socket > topology_max_packages()))
		return -EINVAL;

	msg.msg_num     = HSMP1_GET_SOCKET_POWER;
	msg.response_sz = 1;
	err = hsmp_send_message(socket, &msg);
	if (likely(!err))
		*power_mw = msg.response[0];

	return err;
}
EXPORT_SYMBOL(hsmp1_get_power);

/* Message ID 5 */
int hsmp1_set_power_limit(int socket, u32 limit_mw)
{
	struct hsmp_message msg = { 0 };

	if (unlikely(socket > topology_max_packages()))
		return -EINVAL;

	msg.msg_num  = HSMP1_SET_SOCKET_POWER_LIMIT;
	msg.num_args = 1;
	msg.args[0]  = limit_mw;

	return hsmp_send_message(socket, &msg);
}
EXPORT_SYMBOL(hsmp1_set_power_limit);

/* Message ID 6 */
int hsmp1_get_power_limit(int socket, u32 *limit_mw)
{
	int err;
	struct hsmp_message msg = { 0 };

	if (unlikely(limit_mw == NULL || socket > topology_max_packages()))
		return -EINVAL;

	msg.msg_num     = HSMP1_GET_SOCKET_POWER_LIMIT;
	msg.response_sz = 1;
	err = hsmp_send_message(socket, &msg);
	if (likely(!err))
		*limit_mw = msg.response[0];

	return err;
}
EXPORT_SYMBOL(hsmp1_get_power_limit);

/* Message ID 7 */
int hsmp1_get_power_limit_max(int socket, u32 *limit_mw)
{
	int err;
	struct hsmp_message msg = { 0 };

	if (unlikely(limit_mw == NULL || socket > topology_max_packages()))
		return -EINVAL;

	msg.msg_num     = HSMP1_GET_SOCKET_POWER_LIMIT_MAX;
	msg.response_sz = 1;
	err = hsmp_send_message(socket, &msg);
	if (likely(!err))
		*limit_mw = msg.response[0];

	return err;
}
EXPORT_SYMBOL(hsmp1_get_power_limit_max);

/* Message ID 8 */
int hsmp1_set_boost_limit (int cpu, u32 limit_mhz)
{
	int socket;
	struct hsmp_message msg = { 0 };

	socket       = cpu_data(cpu).phys_proc_id;
	msg.msg_num  = HSMP1_SET_BOOST_LIMIT;
	msg.num_args = 1;
	msg.args[0]  = cpu_data(cpu).apicid << 16 | limit_mhz;

	return hsmp_send_message(socket, &msg);
}
EXPORT_SYMBOL(hsmp1_set_boost_limit);

/* Message ID 9 */
int hsmp1_set_boost_limit_socket (int socket, u32 limit_mhz)
{
	struct hsmp_message msg = { 0 };

	if (unlikely(socket > topology_max_packages()))
		return -EINVAL;

	msg.msg_num  = HSMP1_SET_BOOST_LIMIT_SOCKET;
	msg.num_args = 1;
	msg.args[0]  = limit_mhz;

	return hsmp_send_message(socket, &msg);
}
EXPORT_SYMBOL(hsmp1_set_boost_limit_socket);

/* Message ID 10 */
int hsmp1_get_boost_limit (int cpu, u32 *limit_mhz)
{
	int err, socket;
	struct hsmp_message msg = { 0 };

	if (unlikely(limit_mhz == NULL))
		return -EINVAL;

	socket          = cpu_data(cpu).phys_proc_id;
	msg.msg_num     = HSMP1_GET_BOOST_LIMIT;
	msg.num_args    = 1;
	msg.response_sz = 1;
	msg.args[0]     = cpu_data(cpu).apicid;

	err = hsmp_send_message(socket, &msg);
	if (likely(!err))
		*limit_mhz = msg.response[0];

	return err;
}
EXPORT_SYMBOL(hsmp1_get_boost_limit);

/* Message ID 11 */
/*
 * TODO verify how to interpret response argument.
 * For now, assuming non-zero means PROC_HOT is active.
 */
int hsmp1_get_proc_hot(int socket, bool *proc_hot)
{
	int err;
	struct hsmp_message msg = { 0 };

	if (unlikely(proc_hot == NULL || socket > topology_max_packages()))
		return -EINVAL;

	msg.msg_num     = HSMP1_GET_PROC_HOT;
	msg.response_sz = 1;
	err = hsmp_send_message(socket, &msg);
	if (likely(!err))
		*proc_hot = msg.response[0] ? true : false;

	return err;
}
EXPORT_SYMBOL(hsmp1_get_proc_hot);

/* Message ID 12 */
int hsmp1_set_xgmi2_link_width(unsigned width_min, unsigned width_max)
{
	u8 min, max;
	int socket, _err;
	int err = 0;
	struct hsmp_message msg = { 0 };

	if (unlikely(topology_max_packages() < 2))
		return -ENODEV;

	switch (width_min) {
	case 2:
		min = 0;
		break;
	case 8:
		min = 1;
		break;
	case 16:
		min = 2;
		break;
	default:
		return -EINVAL;
	}

	switch (width_max) {
	case 2:
		max = 0;
		break;
	case 8:
		max = 1;
		break;
	case 16:
		max = 2;
		break;
	default:
		return -EINVAL;
	}

	msg.msg_num  = HSMP1_SET_XGMI2_LINK_WIDTH;
	msg.num_args = 1;
	msg.args[0]  = (min << 8) | max;
	for (socket  = 0; socket < topology_max_packages(); socket++) {
		if(unlikely((_err = hsmp_send_message(socket, &msg))))
			err = _err;
	}

	return err;
}
EXPORT_SYMBOL(hsmp1_set_xgmi2_link_width);

/* Message ID 13 & 14 */
int hsmp1_set_df_pstate(int socket, int p_state)
{
	struct hsmp_message msg = { 0 };

	if (unlikely(socket > topology_max_packages()))
		return -EINVAL;

	if (p_state == -1)
		msg.msg_num  = HSMP1_AUTO_DF_PSTATE;
	else if (p_state <= 3) {
		msg.num_args = 1;
		msg.msg_num  = HSMP1_SET_DF_PSTATE;
		msg.args[0]  = p_state;
	} else
		return -EINVAL;

	return hsmp_send_message(socket, &msg);
}
EXPORT_SYMBOL(hsmp1_set_df_pstate);

/* Message ID 15 */
int hsmp1_get_fabric_clocks(int socket, u32 *fclk, u32 *memclk)
{
	int err;
	struct hsmp_message msg = { 0 };

	if (unlikely((fclk == NULL && memclk == NULL) || socket > topology_max_packages()))
		return -EINVAL;

	msg.msg_num     = HSMP1_GET_FCLK_MCLK;
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
EXPORT_SYMBOL(hsmp1_get_fabric_clocks);

/* Message ID 16 */
int hsmp1_get_max_cclk(int socket, u32 *max_mhz)
{
	int err;
	struct hsmp_message msg = { 0 };

	if (unlikely(max_mhz == NULL || socket > topology_max_packages()))
		return -EINVAL;

	msg.msg_num     = HSMP1_GET_CCLK_THROTTLE_LIMIT;
	msg.response_sz = 1;
	err = hsmp_send_message(socket, &msg);
	if (likely(!err))
		*max_mhz = msg.response[0];

	return err;
}
EXPORT_SYMBOL(hsmp1_get_max_cclk);

/* Message ID 17 */
int hsmp1_get_c0_residency(int socket, u32 *residency)
{
	int err;
	struct hsmp_message msg = { 0 };

	if (unlikely(residency == NULL || socket > topology_max_packages()))
		return -EINVAL;

	msg.msg_num     = HSMP1_GET_C0_PERCENT;
	msg.response_sz = 1;
	err = hsmp_send_message(socket, &msg);
	if (likely(!err))
		*residency = msg.response[0];

	return err;
}
EXPORT_SYMBOL(hsmp1_get_c0_residency);

/*
 * We do it this way to avoid otherwise exposing
 * the send_message function to the kernel.
 */
void __init amd_hsmp1_init(hsmp_send_message_t send_message)
{
	hsmp_send_message = send_message;
}
