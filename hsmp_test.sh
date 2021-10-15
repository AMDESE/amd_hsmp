#! /bin/bash
# SPDX-License-Identifier: GPL-2.0

# Copyright (C) 2007-2019 Advanced Micro Devices, Inc.
# Author: Nathan Fontenot <nathan.fontenot@amd.com>
# Maintainer: Nathan Fontenot <nathan.fontenot@amd.com>

verbose=0

MOD_NAME=amd_hsmp.ko
TEST_MOD_NAME=hsmp_test.ko
MOD_SHORTNAME=${MOD_NAME%.ko}
TEST_MOD_SHORTNAME=${TEST_MOD_NAME%.ko}

unload_amd_hsmp_ko=0
unload_hsmp_test_ko=0
skip_hsmp_ktests=0

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
		       "smn_firmware_version"
		       "xgmi_pstate")

declare -a hsmp_ddr_files=("ddr_max_bandwidth"
			   "ddr_utilized_bandwidth"
			   "ddr_percent_utilized")

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

declare -a hsmp_nbio_files=("nbio_pstate"
			    "nbio_bus")

declare -a readable_files=("hsmp_protocol_version"
			   "smn_firmware_version"
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
	local str=$1

	total_passed=$(($total_passed + 1))

	if [ ! -z "$str" ]; then
		if [ $verbose -ne 0 ]; then
			printf "$str"
		fi
	fi
}

mark_failed()
{
	local str=$1

	total_failed=$(($total_failed + 1))

	if [ ! -z "$str" ]; then
		printf "$str"
	fi
}

mark_tbd()
{
	total_tbd=$(($total_tbd + 1))
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

pr_verbose()
{
	if [ $verbose -ne 0 ]; then
		printf "$1"
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

get_hsmp_protocol()
{
	HSMP_PROTOCOL=`cat $HSMP_SYSFS_BASE_DIR/hsmp_protocol_version`

	printf "HSMP Protocol $HSMP_PROTOCOL\n"
}

validate_dir()
{
	local dir=$1

	if [ ! -d $dir ]; then
		printf "    $dir...$FAILED\n"
		return 1
	fi

	pr_verbose "    $dir...$PASS\n"
	return 0
}

validate_file()
{
	local file=$1

	if [ ! -f $file ]; then
		printf "    $file...$FAILED\n"
		return 1
	fi

	pr_verbose "    $file...$PASS\n"
	return 0
}

validate_sysfs_files()
{
	local status=0

	printf "Validating sysfs files exist\n"
	validate_dir $HSMP_SYSFS_BASE_DIR
	for f in $hsmp_files; do
		validate_file $HSMP_SYSFS_BASE_DIR/$f
		if [ $? -eq 1 ]; then
			status=1
		fi
	done

	if [ $status -eq 1 ]; then
		mark_failed
	else
		mark_passed
	fi

	pr_verbose "\n"

	printf "Validating per-cpu sysfs files exist\n"
	status=0
	for HSMP_CPU in $PRESENT_CPUS; do
		local cpu_dir=$(hsmp_cpu_dir)
		validate_dir $cpu_dir
		if [ $? -eq 1 ]; then
			status=1
		fi

		for f in ${hsmp_cpu_files[@]}; do
			validate_file $cpu_dir/$f
			if [ $? -eq 1 ]; then
				status=1
			fi
		done
	done

	if [ $status -eq 1 ]; then
		mark_failed
	else
		mark_passed
	fi

	pr_verbose "\n"

	printf "Validating per-socket sys files exist\n"
	status=0
	for HSMP_SOCKET in $PRESENT_SOCKETS; do
		local socket_dir=$(hsmp_socket_dir)
		validate_dir $socket_dir
		if [ $? -eq 1 ]; then
			status=1
		fi

		for f in ${hsmp_socket_files[@]}; do
			validate_file $socket_dir/$f
			if [ $? -eq 1 ]; then
				status = 1
			fi
		done

		# Validate per-NBIO directories
		for nbio_dir in `ls -d $socket_dir/nbio*`; do
			for f in ${hsmp_nbio_files[@]}; do
				validate_file $nbio_dir/$f
				if [ $? -eq 1 ]; then
					status = 1
				fi
			done
		done

		if [ $HSMP_PROTOCOL -ge 3 ]; then
			for f in ${hsmp_ddr_files[@]};do
				validate_file $socket_dir/$f
				if [ $? -eq 1 ]; then
					status=1
				fi
			done
		fi
	done

	if [ $status -eq 1 ]; then
		mark_failed
	else
		mark_passed
	fi
}

read_file()
{
	local file=$1

	if [ ! -r $file ]; then
		mark_failed "    $file not readable...$FAILED\n"
		return
	fi

	local val=`cat $file 2>&1`

	cat $file >/dev/null 2>&1
	if [ "$?" -eq 0 ]; then
		mark_passed "    Reading $file ($val)...$PASS\n"
	else
		mark_failed "    Reading $file ($val)...$FAILED\n"
	fi
}

write_fabric_pstate()
{
	local base_dir=$1
	local file=$base_dir/fabric_pstate

	if [ ! -w $file ]; then
		mark_failed "    $file not readable...$FAILED\n"
		return
	else
		mark_passed "    Checking $file...$PASS\n"
	fi

	# Valid values to write to fabric_pstate are 0..3 and -1
	for i in 0 1 2 3 -1; do
		echo $i > $file
		if [ "$?" -ne 0 ]; then
			mark_failed "    Writing $i to $file...$FAILED\n"
		else
			mark_passed "    Writing $i to $file...$PASS\n"
		fi
	done

	res=$((echo "4" > $file) 2>&1)
        if [ "$?" -ne 0 ]; then
                mark_passed "    Writing 4 to $file...$PASS\n"
                pr_verbose  "        $res\n"
                pr_verbose  "        Expecting 'Invalid Argument'\n"
        else
                mark_failed "    Writing 4 to $file...$FAILED\n"
        fi

}

write_power_limit()
{
	local base_dir=$1
	local file=$base_dir/power_limit

	if [ ! -w $file ]; then
		mark_failed "    $file not readable...$FAILED\n"
		return
	else
		mark_passed "    Checking $file...$PASS\n"
	fi

	# Check valid write values
	# 120000 mW is a good test value applicable to most SKUs
	echo 120000 > $file
	if [ "$?" -ne 0 ]; then
		mark_failed "    Writing 120 to $file...$FAILED\n"
	else
		mark_passed "    Writing 120 to $file...$PASS\n"
	fi
}

# TODO: Add read_xgmi_pstate and read_xgmi_speed functionality once supported
write_xgmi_pstate()
{
	local res
	local base_dir=$1
	local file=$base_dir/xgmi_pstate

	# xgmi_pstate is only created on 2P systems, make sure it's present
	if [ ! -f $file ]; then
		return
	fi

	if [ ! -w $file ]; then
		mark_failed "    $file not readable...$FAILED\n"
		return
	else
		mark_passed "    Checking $file...$PASS\n"
	fi

	# For Rome and Milan systems, xgmi_pstate valid values are
	# -1, 0, and 1. For Milan the value of 2 is also valid.
	for i in 1 0 -1; do
		echo $i > $file
		if [ "$?" -ne 0 ]; then
			mark_failed "    Writing $i to $file...$FAILED\n"
		else
			mark_passed "    Writing $i to $file...$PASS\n"
		fi
	done

	if [[ $CPU_FAMILY -eq $AMD_FAMILY_19h ]]; then
		echo 2 > $file
		if [ "$?" -ne 0 ]; then
			mark_failed "    Writing 2 to $file...$FAILED\n"
		else
			mark_passed "    Writing 2 to $file...$PASS\n"
		fi
	fi

	# Validate write failures with invalid link P-state
	res=$((echo "3" > $file) 2>&1)
	if [ "$?" -ne 0 ]; then
		mark_passed "    Writing 3 to $file...$PASS\n"
		pr_verbose  "        $res\n"
		pr_verbose  "        Expecting 'Invalid Argument'\n"
	else
		printf "$FAILED\n"
		mark_failed "    Writing 3 to $file...$FAILED\n"
	fi
}

write_nbio_pstate()
{
	local file=$1

	if [ $HSMP_PROTOCOL -lt 2 ]; then
		return
	fi

	if [ ! -w $file ]; then
		mark_failed "    $file is not readable...$FAILED\n"
		return
	else
		mark_passed "    Checking $file...$PASS\n"
	fi

	# The only valid values to write to nbio_pstate are -1, 0, and 1.
	# Validate these values.
	for i in 1 0 -1; do
		echo $i > $file
		if [ "$?" -ne 0 ]; then
			mark_failed "    Writing $i to $file...$FAILED\n"
		else
			mark_passed "    Writing $i to $file...$PASS\n"
		fi
	done

	# Validate write failures with invalid NBIO P-state
        res=$((echo "3" > $file) 2>&1)
        if [ "$?" -ne 0 ]; then
                mark_passed "    Writing 3 to $file...$PASS\n"
                pr_verbose  "        $res\n"
                pr_verbose  "        Expecting 'Invalid Argument'\n"
        else
                mark_failed "    Writing 3 to $file...$FAILED\n"
        fi

}

write_boost_limit()
{
	local base_dir=$1
	local file=$base_dir/boost_limit

	if [ ! -w $file ]; then
		mark_failed "    $file is not readable...$FAILED\n"
		return
	else
		mark_passed "    Checking $file...$PASS\n"
	fi

	# Check valid write values
	# 3150 MHz is a good test value applicable to all SKUs
	echo 3150 > $file
	if [ "$?" -ne 0 ]; then
		mark_failed "    Writing 3150 to $file...$FAILED\n"
	else
		mark_passed "    Writing 3150 to $file...$PASS\n"
	fi
}

# Validate module loads
load_hsmp_driver()
{
	# If the hsmp driver module is already loaded, use that.
	lsmod | grep amd_hsmp > /dev/null
	if [[ $? -eq 0 ]]; then
		printf "Using the currently loaded HSMP Driver\n"
		return
	fi

	# If a kernel module was specified on the command line
	# use that, otherwise look for a kernel mmodule in the
	# current directory.
	if [[ -z "$AMD_HSMP_KO" ]]; then
		AMD_HSMP_KO=$PWD/$MOD_NAME
	fi

	if [[ ! -f "$AMD_HSMP_KO" ]]; then
		printf "\"$AMD_HSMP_KO\" doesn't exist\n"
		exit -1
	fi

	printf "Loading $AMD_HSMP_KO..."

	sudo insmod $AMD_HSMP_KO f17h_support=1 raw_intf=1
	if [[ $? -ne 0 ]]; then
		mark_failed "$FAILED\n"
	else
		# verify module loaded
		lsmod | grep amd_hsmp > /dev/null
		if [[ $? -ne 0 ]]; then
			mark_failed "$FAILED\n    lsmod does not show module loaded.\n"
		else
			mark_passed "$PASS"
			unload_amd_hsmp_ko=1
		fi
	fi

	printf "\n"
}

unload_hsmp_driver()
{
	if [[ $unload_amd_hsmp_ko -eq 0 ]]; then
		return
	fi

	printf "Unloading $MOD_NAME..."

	sudo rmmod $MOD_SHORTNAME
	if [[ $? -ne 0 ]]; then
		mark_failed "$FAILED\n"
	else
		# verify module unloaded
		lsmod | grep amd_hsmp > /dev/null
		if [[ $? -ne 1 ]]; then
			mark_failed "$FAILED\n    lsmod still shows module loaded.\n"
		else
			mark_passed "$PASS"
			printf "\n"
		fi
	fi

	printf "\n"
}

load_hsmp_test_driver()
{
	# If the hsmp driver module is already loaded, use that.
	lsmod | grep hsmp_test > /dev/null
	if [[ $? -eq 0 ]]; then
		printf "Using the currently loaded HSMP Test Driver\n"
		return
	fi

	# If a kernel test module was specified on the command line
	# use that, otherwise look for a kernel test module in the
	# current directory.
	if [[ -z "$HSMP_TEST_KO" ]]; then
		HSMP_TEST_KO=$PWD/$TEST_MOD_NAME
	fi

	if [[ ! -f "$HSMP_TEST_KO" ]]; then
		printf "HSMP Test module \"$HSMP_TEST_KO\" doesn't exist\n"
		skip_hsmp_ktests=1
		return
	fi

	printf "Loading $HSMP_TEST_KO..."

	sudo insmod $HSMP_TEST_KO
	if [[ $? -ne 0 ]]; then
		printf "$FAILED\n"
		skip_hsmp_ktests=1
	else
		# verify module loaded
		lsmod | grep hsmp_test > /dev/null
		if [[ $? -ne 0 ]]; then
			printf "$FAILED\n"
			printf "lsmod does not show module loaded.\n"
			skip_hsmp_ktests=1
		else
			unload_hsmp_test_ko=1
			printf "\n"
		fi
	fi
}

unload_hsmp_test_driver()
{
	if [[ $unload_hsmp_test_ko -eq 0 ]]; then
		return
	fi

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

hsmp_test_init()
{
	load_hsmp_driver

	PRESENT_CPUS=`cat /sys/devices/system/cpu/present`
	PRESENT_CPUS=`seq ${PRESENT_CPUS/-/ }`

	NUM_SOCKETS=`lscpu | grep Socket | awk '{print $2}'`
	PRESENT_SOCKETS=$((NUM_SOCKETS - 1))
	PRESENT_SOCKETS=`seq 0 ${PRESENT_SOCKETS}`

	get_cpu_family
	get_hsmp_protocol

	printf "\n"
}


## Main

while getopts "k:t:v" opt; do
	case ${opt} in
		k ) AMD_HSMP_KO=$OPTARG
		    ;;
		t ) HSMP_TEST_KO=$OPTARG
		    ;;
		v ) verbose=1
		    ;;
	esac
done

hsmp_test_init

validate_sysfs_files

# Default testing is for cpu0 and socket0
HSMP_CPU=0
HSMP_SOCKET=0

printf "\n"
printf "Validating sysfs file read functionality\n"

# At root level, boost_limit is not readable
read_file $HSMP_SYSFS_BASE_DIR/hsmp_protocol_version
read_file $HSMP_SYSFS_BASE_DIR/smn_firmware_version

# xgmi_* files are only created on systems with more than 1 socket
if [ -f $HSMP_SYSFS_BASE_DIR/xgmi_pstate ]; then
	read_file $HSMP_SYSFS_BASE_DIR/xgmi_pstate
fi

if [ -f $HSMP_SYSFS_BASE_DIR/xgmi_speed ]; then
	read_file $HSMP_SYSFS_BASE_DIR/xgmi_speed
fi

# All cpu files are readable
pr_verbose "\n"
printf "Validating per-cpu sysfs file read functionality\n"
for f in ${hsmp_cpu_files[@]}; do
	cpu_dir=$(hsmp_cpu_dir)
	read_file $cpu_dir/$f
done

pr_verbose "\n"
printf "Validating per-socket sysfs file read functionality\n"
for f in ${hsmp_socket_files[@]}; do
	valid=$(file_is_readable $f)
	if [ $valid -eq 1 ]; then
		socket_dir=$(hsmp_socket_dir)
		read_file $socket_dir/$f
	fi
done

pr_verbose "\n"
if [ $HSMP_PROTOCOL -ge 3 ]; then
	printf "Validating DDR sysfs file read functionality\n"
	for f in ${hsmp_ddr_files[@]}; do
		socket_dir=$(hsmp_socket_dir)
		read_file $socket_dir/$f
	done
fi

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
write_xgmi_pstate $HSMP_SYSFS_BASE_DIR
write_boost_limit $cpu_dir
write_boost_limit $socket_dir
write_fabric_pstate $socket_dir
write_power_limit $socket_dir
write_nbio_pstate $socket_dir/nbio0/nbio_pstate
printf "\n"

load_hsmp_test_driver
if [[ $skip_hsmp_ktests -eq 1 ]]; then
	printf "Skipping HSMP kernel interface tests\n"
else
	printf "Running HSMP Test Driver\n"
	printf "See dmesg output for test scenario result details\n"

	echo 1 > /sys/devices/system/cpu/hsmp_test
	test_res=`cat /sys/devices/system/cpu/hsmp_test`

	pass=${test_res%%,*}
	total_passed=$((total_passed + pass))

	test_res=${test_res#*,}
	fail=${test_res%%,*}
	total_failed=$((total_failed + fail))

	tbd=${test_res#*,}
	total_tbd=$((total_tbd + tbd))

	printf "Driver results: $PASS=$pass, $FAILED=$fail, $TBD=$tbd\n"
	unload_hsmp_test_driver
fi

printf "\n"

# Test raw interface if enabled on kernel load.
# The test script will use an already loaded driver so we may be on a
# system that the driver does not have the raw_interface or it was not
# enabled at driver load.
if [ -f $socket_dir/hsmp_raw ]; then
	printf "Testing raw HSMP interface\n"
	./raw_test
	if [ "$?" -ne 1 ]; then
		mark_failed "    raw interface...$FAILED\n"
	else
		mark_passed "    raw interface...$PASS\n"
	fi
fi

printf "\n"

unload_hsmp_driver
# ??? Validate sysfs files are removed

total_tests=$((total_passed + total_failed + total_tbd))
printf "\n"
printf "Test Results:\n"
printf "===============================\n"
printf "total tests: $total_tests\n"
printf "$PASS:        $total_passed\n"
printf "$FAILED:      $total_failed\n"
printf "$TBD:         $total_tbd\n"
