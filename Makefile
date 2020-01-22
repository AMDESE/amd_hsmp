# SPDX-License-Identifier: GPL-2.0
#
# Makefile for AMD Host System Management Port driver
#
# Copyright (C) 2019 Advanced Micro Devices, Inc.
#
# Author:
#   Lewis Carroll <lewis.carroll@amd.com>
#

# If KDIR is not specified, assume the development source link
# is in the modules directory for the running kernel
KDIR ?= /lib/modules/`uname -r`/build

default:
	$(MAKE) -C $(KDIR) M=$$PWD modules

modules: default

modules_install:
	$(MAKE) -C $(KDIR) M=$$PWD modules_install

test:
	export CONFIG_HSMP_TEST=m; \
	$(MAKE) -C $(KDIR) M=$$PWD

clean:
	$(MAKE) -C $(KDIR) M=$$PWD clean
