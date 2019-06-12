#
# Makefile for AMD Host System Management Port driver
#
# Copyright (C) 2019 Advanced Micro Devices, Inc.
#
# Author:
#   Lewis Carroll <lewis.carroll@amd.com>
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2

# If KDIR is not specified, assume the development source link
# is in the modules directory for the running kernel
KDIR ?= /lib/modules/`uname -r`/build

default:
    $(MAKE) -C $(KDIR) M=$$PWD

clean:
    $(MAKE) -C $(KDIR) M=$$PWD clean
