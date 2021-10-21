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

debug:
	$(MAKE) CFLAGS_MODULE=-DDEBUG -C $(KDIR) M=$$PWD modules

modules: default

modules_install:
	$(MAKE) -C $(KDIR) M=$$PWD modules_install

test: debug
	export CONFIG_HSMP_TEST=m; \
	$(MAKE) -C $(KDIR) M=$$PWD
	$(CC) -o raw_test raw_test.c

clean:
	$(MAKE) -C $(KDIR) M=$$PWD clean
	@rm -f raw_test

help:
	@echo "\nThe following make targets are supported:\n"
	@echo "default\t\tBuild the driver module (or if no make target is supplied)"
	@echo "modules\t\tSame as default"
	@echo "debug\t\tBuild the driver module with debug output"
	@echo "modules_install\tBuild and install the driver module"
	@echo "test\t\tBuild the driver test module"
	@echo "clean"
	@echo

.PHONY: default debug modules modules_install test clean help

