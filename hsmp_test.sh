#! /bin/bash

# SPDX-License-Identifier: GPL-2.0

# Copyright (C) 2007-2019 Advanced Micro Devices, Inc.
# Author: Nathan Fontenot <nathan.fontenot@amd.com>
# Maintainer: Nathan Fontenot <nathan.fontenot@amd.com>


# do we continue testing
continue=1

verbose=0

MOD_NAME=amd_hsmp.ko
TEST_MOD_NAME=hsmp_test.ko
MOD_SHORTNAME=${MOD_NAME%.ko}
TEST_MOD_SHORTNAME=${TEST_MOD_NAME%.ko}

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
NC='\033[0m'

PASS=${GREEN}"pass"${NC}
FAILED=${RED}"failed"${NC}
TBD=${YELLOW}"TBD"${NC}

AMD_FAMILY_17h=23 # Rome
AMD_FAMILY_19h=25 # Milan

HSMP_SYSFS_BASE_DIR=/sys/devices/system/cpu/amd_hsmp

declare -a hsmp_files=("boost_limit"
		       "hsmp_protocol_version"
		       "smu_firmware_version"
		       "xgmi_pstate")

declare -a hsmp_cpu_files=("boost_limit")

declare -a hsmp_socket_files=("boost_limit"
			      "c0_residency"
			      "cclk_limit"
			      "fabric_clocks"
			      "fabric_pstate"
			      "power"
			      "power_limit"
			      "power_limit_max"
			      "proc_hot"
			      "tctl")

declare -a nbio_files_1p=("pci0000:00/nbio_pstate"
			  "pci0000:40/nbio_pstate"
			  "pci0000:80/nbio_pstate"
			  "pci0000:c0/nbio_pstate")

declare -a nbio_files_2p=("pci0000:00/nbio_pstate"
			  "pci0000:20/nbio_pstate"
			  "pci0000:40/nbio_pstate"
			  "pci0000:60/nbio_pstate"
			  "pci0000:80/nbio_pstate"
			  "pci0000:a0/nbio_pstate"
			  "pci0000:c0/nbio_pstate"
			  "pci0000:e0/nbio_pstate")

declare -a readable_files=("hsmp_protocol_version"
			   "smu_firmware_version"
			   "c0_residency"
			   "cclk_limit"
			   "fabric_clocks"
			   "power"
			   "power_limit"
			   "power_limit_max"
			   "proc_hot"
			   "tctl")

# Assume cpu 0 and socket 0
HSMP_CPU=0
HSMP_SOCKET=0

total_pass=0
total_failed=0
total_tbd=0

mark_passed()
{
	total_passed=$[$total_passed + 1]
}

mark_failed()
{
	total_failed=$[$total_failed + 1]
}

mark_tbd()
{
	total_tbd=$[$total_tbd + 1]
}

file_is_readable()
{
	local file=$1
	local valid=0

	for f in ${readable_files[@]}; do
		if [[ "$f" = "$file" ]]; then
			valid=1
		fi
	done

	if [ $valid -eq 1 ]; then
		echo 1
	else
		echo 0
	fi
}

hsmp_cpu_dir()
{
	echo $HSMP_SYSFS_BASE_DIR/cpu$HSMP_CPU
}

hsmp_socket_dir()
{
	echo $HSMP_SYSFS_BASE_DIR/socket$HSMP_SOCKET
}

pr_debug()
{
	if [ $verbose -ne 0 ]; then
		printf $1
	fi
}

get_cpu_family()
{
	local arr

	CPU_FAMILY=`lscpu | grep "CPU family"`
	read -r -a arr <<< $CPU_FAMILY
	CPU_FAMILY=${arr[2]}

	if [[ $CPU_FAMILY -eq $AMD_FAMILY_17h ]]; then
		printf "Testing on Rome system\n"
	elif [[ $CPU_FAMILY -eq $AMD_FAMILY_19h ]]; then
		printf "Testing on Milan system\n"
	else
		printf "Testing on Unknown system, CPU family $CPU_FAMILY\n"
	fi
}

hsmp_test_init()
{
	# Validate HSMP Driver module
	if [[ -z "$AMD_HSMP_KO" ]]; then
		AMD_HSMP_KO=$PWD/$MOD_NAME
	fi

	if [[ ! -f "$AMD_HSMP_KO" ]]; then
		printf "Kernel module \"$AMD_HSMP_KO\" doesn't exist\n"
		exit -1
	else
		printf "Using $AMD_HSMP_KO\n"
	fi

	# Validate HSMP Test driver
	if [[ -z "$HSMP_TEST_KO" ]]; then
		HSMP_TEST_KO=$PWD/$TEST_MOD_NAME
	fi

	if [[ ! -f "$HSMP_TEST_KO" ]]; then
		printf "Kernel module \"$HSMP_TEST_KO\" doesn't exist\n"
		HSMP_TEST_KO=""
	else
		printf "Using $HSMP_TEST_KO\n"
	fi

	PRESENT_CPUS=`cat /sys/devices/system/cpu/present`
	PRESENT_CPUS=`seq ${PRESENT_CPUS/-/ }`

	NUM_SOCKETS=`lscpu | grep Socket | awk '{print $2}'`
	PRESENT_SOCKETS=$((NUM_SOCKETS - 1))
	PRESENT_SOCKETS=`seq 0 ${PRESENT_SOCKETS}`
	
	printf "Using $AMD_HSMP_KO\n"

	get_cpu_family
}

validate_dir()
{
	local dir=$1

	if [ ! -d $dir ]; then
		printf "    $file...$FAILED\n"
	fi
}

validate_file()
{
	local file=$1

	if [ ! -f $file ]; then
		printf "    $file...$FAILED\n"
	fi
}

validate_sysfs_files()
{
	printf "Validating sysfs files exist\n"
	validate_dir $HSMP_SYSFS_BASE_DIR
	for f in $hsmp_files; do
		validate_file $HSMP_SYSFS_BASE_DIR/$f
	done

	pr_debug "\n"

	for HSMP_CPU in $PRESENT_CPUS; do
		local cpu_dir=$(hsmp_cpu_dir)
		validate_dir $cpu_dir
		for f in ${hsmp_cpu_files[@]}; do
			validate_file $cpu_dir/$f
		done
	done

	pr_debug "\n"
	
	for HSMP_SOCKET in $PRESENT_SOCKETS; do
		local socket_dir=$(hsmp_socket_dir)
		validate_dir $socket_dir
		for f in ${hsmp_socket_files[@]}; do
			validate_file $socket_dir/$f
		done
	done

	pr_debug "\n"

	if [ $NUM_SOCKETS -eq 1 ]; then
		for FILE in ${nbio_files_1p[@]}; do
			validate_file $HSMP_SYSFS_BASE_DIR/$FILE
		done
	else
		for FILE in ${nbio_files_2p[@]}; do
			validate_file $HSMP_SYSFS_BASE_DIR/$FILE
		done
	fi

	pr_debug "\n"
}

read_file()
{
	local file=$1

	printf "    Reading $file..."

	if [ ! -r $file ]; then
		printf "$FAILED\n"
		printf "        file is not readable\n"
		mark_failed
		return
	fi

	local val=`cat $file 2>&1`

	cat $file >/dev/null 2>&1
	if [ "$?" -eq 0 ]; then
		printf "$PASS\n"
		printf "        read: $val\n"
		mark_passed
	else
		printf "$FAILED\n"
		printf "        $val\n"
		mark_failed
	fi
}

write_fabric_pstate()
{
	local base_dir=$1
	local file=$base_dir/fabric_pstate

	printf "    Checking $file..."

	if [ ! -w $file ]; then
		printf "$FAILED\n"
		printf "        file is not writeable\n"
		mark_failed
		return
	else
		printf "$PASS\n"
		mark_passed
	fi

	# Valid values to write to fabric_pstate are 0..3 and -1
	for i in 0 1 2 3 -1; do
		printf "    Writing $i to $file..."
		echo $i > $file
		if [ "$?" -ne 0 ]; then
			printf "$FAILED\n"
			mark_failed
		else
			printf "$PASS\n"
			mark_passed
		fi
	done

	#set -x
	printf "    Writing 4 to $file...\n        "
	echo 4 > $file
	if [ "$?" -ne 0 ]; then
		printf "        Expecting 'Invalid Argument'...$PASS\n"
		mark_passed
	else
		printf "$FAILED\n"
		marks_failed
	fi
}

write_power_limit()
{
	local base_dir=$1
	local file=$base_dir/power_limit

	printf "    Checking $file..."
	
	if [ ! -w $file ]; then
		printf "$FAILED\n"
		printf "        file is not writeable\n"
		mark_failed
		return
	else
		printf "$PASS\n"
		mark_passed
	fi

	# Check valid write values
	# 120000 mW is a good test value applicable to most SKUs
	printf "    Writing 120 to $file..."
	echo 120000 > $file
	if [ "$?" -ne 0 ]; then
		printf "$FAILED\n"
		mark_failed
	else
		printf "$PASS\n"
		mark_passed
	fi
}

# TODO: Add read_xgmi_pstate and read_xgmi_speed functionality once supported
write_xgmi_pstate()
{
	local base_dir=$1
	local file=$base_dir/xgmi_pstate

	# xgmi_pstate is only created on 2P systems, make sure it's present
	if [ ! -f $file ]; then
		return
	fi

	printf "    Checking $file..."
	
	if [ ! -w $file ]; then
		printf "$FAILED\n"
		printf "        file is not writeable\n"
		mark_failed
		return
	else
		printf "$PASS\n"
		mark_passed
	fi
	
	# For Rome and Milan systems, xgmi_pstate valid values are
	# -1, 0, and 1. For Milan the value of 2 is also valid.
	for i in 1 0 -1; do
		printf "    Writing $i to $file..."
		echo $i > $file
		if [ "$?" -ne 0 ]; then
			printf "$FAILED\n"
			mark_failed
		else
			printf "$PASS\n"
			mark_passed
		fi
	done

	if [[ $CPU_FAMILY -eq $AMD_FAMILY_19h ]]; then
		printf "    Writing 2 to $file..."
		echo 2 > $file
		if [ "$?" -ne 0 ]; then
			printf "$FAILED\n"
			mark_failed
		else
			printf "$PASS\n"
			mark_passed
		fi
	fi

	# Validate write failures with invalid link P-state
	printf "    Writing 3 to $file...\n        "
	echo 3 > $file
	if [ "$?" -ne 0 ]; then
		printf "        Expecting 'Invalid Argument'...$PASS\n"
		mark_passed
	else
		printf "$FAILED\n"
		mark_failed
	fi
}

write_nbio_pstate()
{
	local file=$1
	printf "    Checking $file..."

	if [ ! -w $file ]; then
		printf "$FAILED\n"
		printf "        file is not writeable\n"
		mark_failed
		return
	else
		printf "$PASS\n"
		mark_passed
	fi

	# The only valid values to write to nbio_pstate are -1, 0, and 1.
	# Validate these values.
	for i in 1 0 -1; do
		printf "    Writing $i to $file..."
		echo $i > $file
		if [ "$?" -ne 0 ]; then
			printf "$FAILED\n"
			mark_failed
		else
			printf "$PASS\n"
			mark_passed
		fi
	done

	# Validate write failures with invalid NBIO P-state
	printf "    Writing 3 to $file...\n        "
	echo 3 > $file
	if [ "$?" -ne 0 ]; then
		printf "        Expecting 'Invalid Argument'...$PASS\n"
		mark_passed
	else
		printf "$FAILED\n"
		mark_failed
	fi
}

write_boost_limit()
{
	local base_dir=$1
	local file=$base_dir/boost_limit

	printf "    Checking $file..."
	
	if [ ! -w $file ]; then
		printf "$FAILED\n"
		printf "        file is not writeable\n"
		mark_failed
		return
	else
		printf "$PASS\n"
		mark_passed
	fi
	
	# Check valid write values
	# 3150 MHz is a good test value applicable to all SKUs
	printf "    Writing 3150 to $file..."
	echo 3150 > $file
	if [ "$?" -ne 0 ]; then
		printf "$FAILED\n"
		mark_failed
	else
		printf "$PASS\n"
		mark_passed
	fi
}

# Validate module loads
load_hsmp_driver()
{
	printf "Loading $MOD_NAME..."
	
	sudo insmod $AMD_HSMP_KO
	if [[ $? -ne 0 ]]; then
		printf "$FAILED\n"
		mark_failed
	else
		# verify module loaded
		lsmod | grep amd_hsmp > /dev/null
		if [[ $? -ne 0 ]]; then
			printf "$FAILED\n"
			printf "lsmod does not show module loaded.\n"
			mark_failed
		else
			printf "$PASS\n"
			mark_passed
		fi
	fi
}

unload_hsmp_driver()
{
	printf "Unloading $MOD_NAME..."
	
	sudo rmmod $MOD_SHORTNAME
	if [[ $? -ne 0 ]]; then
		printf "$FAILED\n"
		mark_failed
	else
		# verify module unloaded
		lsmod | grep amd_hsmp > /dev/null
		if [[ $? -ne 1 ]]; then
			printf "$FAILED\n"
			printf "lsmod still shows module loaded.\n"
			mark_failed
		else
			printf "$PASS\n"
			mark_passed
		fi
	fi
}

load_hsmp_test_driver()
{
	printf "Loading $TEST_MOD_NAME..."

	sudo insmod $HSMP_TEST_KO
	if [[ $? -ne 0 ]]; then
		printf "$FAILED\n"
	else
		# verify module loaded
		lsmod | grep hsmp_test > /dev/null
		if [[ $? -ne 0 ]]; then
			printf "$FAILED\n"
			printf "lsmod does not show module loaded.\n"
		else
			printf "\n"
		fi
	fi
}

unload_hsmp_test_driver()
{
	printf "Unloading $TEST_MOD_NAME..."

	sudo rmmod $TEST_MOD_SHORTNAME
	# verify module unloaded
	lsmod | grep hsmp_test > /dev/null
	if [[ $? -ne 1 ]]; then
		printf "$FAILED\n"
		printf "lsmod still shows module loaded.\n"
	else
		printf "\n"
	fi
}


## Main
hsmp_test_init

load_hsmp_driver

printf "\n"
validate_sysfs_files

# Default testing is for cpu0 and socket0
HSMP_CPU=0
HSMP_SOCKET=0

printf "\n"
printf "Validating sysfs file read functionality\n"

# At root level, boost_limit is not readable
read_file $HSMP_SYSFS_BASE_DIR/hsmp_protocol_version
read_file $HSMP_SYSFS_BASE_DIR/smu_firmware_version
read_file $HSMP_SYSFS_BASE_DIR/xgmi_pstate
read_file $HSMP_SYSFS_BASE_DIR/xgmi_speed

printf "\n"

# All cpu files are readable
for f in ${hsmp_cpu_files[@]}; do
	cpu_dir=$(hsmp_cpu_dir)
	read_file $cpu_dir/$f
done

printf "\n"

for f in ${hsmp_socket_files[@]}; do
	valid=$(file_is_readable $f)
	if [ $valid -eq 1 ]; then
		socket_dir=$(hsmp_socket_dir)
		read_file $socket_dir/$f
	fi
done

# Writing to files needs to be done on a per-file basis since
# each file has it's own valid set of values.
#
# The boost_limit file is a slight aberration since it appears
# at three levels. Each instance takes the same set of valid
# input.
# amd_hsmp/boost_limit
# amd_hsmp/cpuX/boost_limit
# amd_hsmp/socketX/boost_limit
#
# amd_hsmp/socketX/fabric_pstate
# amd_hsmp/socketX/power_limit
# amd_hsmp/socketX/xgmi_pstate
cpu_dir=$(hsmp_cpu_dir)
socket_dir=$(hsmp_socket_dir)

printf "\n"
printf "Validating sysfs file write functionality\n"
write_boost_limit $HSMP_SYSFS_BASE_DIR
printf "\n"

write_xgmi_pstate $HSMP_SYSFS_BASE_DIR
printf "\n"

write_boost_limit $cpu_dir
printf "\n"

write_boost_limit $socket_dir
printf "\n"

write_fabric_pstate $socket_dir
printf "\n"

write_power_limit $socket_dir
printf "\n"

write_nbio_pstate $HSMP_SYSFS_BASE_DIR/${nbio_files_1p[0]}
printf "\n"

if [[ -n $HSMP_TEST_KO ]]; then
	printf "Running HSMP Test Driver\n"
	printf "See dmesg output for test scenario result details\n"

	load_hsmp_test_driver

	echo 1 > /sys/devices/system/cpu/hsmp_test
	test_res=`cat /sys/devices/system/cpu/hsmp_test`

	pass=${test_res%%,*}
	total_passed=$[total_passed + pass]

	test_res=${test_res#*,}
	fail=${test_res%%,*}
	total_failed=$[total_failed + fail]

	tbd=${test_res#*,}
	total_tbd=$[total_tbd + tbd]

	printf "Driver results: $PASS=$pass, $FAILED=$fail, $TBD=$tbd\n"
	unload_hsmp_test_driver
fi

printf "\n"

unload_hsmp_driver
# ??? Validate sysfs files are removed

total_tests=$[total_passed + total_failed + total_tbd]
printf "\n"
printf "Test Results:\n"
printf "===============================\n"
printf "total tests: $total_tests\n"
printf "$PASS:        $total_passed\n"
printf "$FAILED:      $total_failed\n"
printf "$TBD:         $total_tbd\n"
