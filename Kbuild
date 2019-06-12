#
# Kbuild for AMD Host System Management Port driver
#
# Copyright (C) 2019 Advanced Micro Devices, Inc.
#
# Author:
#   Lewis Carroll <lewis.carroll@amd.com>
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2

obj-m := amd_hsmp.o
amd_hsmp-y := hsmp_main.o hsmp1.o
amd_hsmp-$(CONFIG_SYSFS) += hsmp1_sysfs.o
