// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2007-2019 Advanced Micro Devices, Inc.
 * Author:: Nathan Fontenot <nathan.fontenot@amd.com>
 *
 * Test case for raw HSMP interface
 *
 * This test uses the HSMP_TEST message to do a simple test
 * to ensure the raw interface is working. The test case
 * purposely returns either 1 for success or zero for failure.
 */

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <linux/types.h>

static char *raw_file = "/sys/devices/system/cpu/amd_hsmp/socket0/hsmp_raw";

/* Ported over from amd_hsmp driver.
 *
 * If this struct changes in the driver this needs to be updated.
 */
struct hsmp_message {
	__u32	msg_num;	/* Message number */
	__u16	num_args;	/* Number of arguments in message */
	__u16	response_sz;	/* Number of expected response words */
	__u32	args[8];	/* Argument(s) */
	__u32	response[8];	/* Response word(s) */
} msg;

int
main(int argc, char **argv)
{
	int fd, rc;

	memset(&msg, 0, sizeof(msg));

	msg.msg_num = 1;
	msg.num_args = 1;
	msg.response_sz = 1;
	msg.args[0] = 1;

	fd = open(raw_file, O_RDWR);
	if (fd <= 0) {
		printf("Open of raw interface failed: %s\n", strerror(errno));
		exit(0);
	}

	rc = write(fd, &msg, sizeof(msg));
	if (rc != sizeof(msg)) {
		printf("Invalid write to raw interface; expected %ld, received %d\n",
		       sizeof(msg), rc);
		close(fd);
		exit(0);
	}

	lseek(fd, 0, SEEK_SET);

	rc = read(fd, &msg, sizeof(msg));
	close(fd);
	if (rc != sizeof(msg)) {
		printf("Invalid read from raw interface; expected %ld, received %d\n",
		       sizeof(msg), rc);
		exit(0);
	}

	if (msg.response[0] != 2) {
		printf("Received wrong response; expected 2, received %d\n",
		       msg.response[0]);
		exit(0);
	}

	return 1;
}

