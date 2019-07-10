#! /bin/bash

# do we continue testing
continue=1

verbose=0

MOD_NAME=amd_hsmp.ko
MOD_SHORTNAME=${MOD_NAME%.ko}

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
NC='\033[0m'

PASS=${GREEN}"pass"${NC}
FAILED=${RED}"failed"${NC}
TBD=${YELLOW}"TBD"${NC}

HSMP_SYSFS_BASE_DIR=/sys/devices/system/cpu/amd_hsmp

declare -a hsmp_files=("boost_limit"
		       "hsmp_protocol_version"
		       "smu_firmware_version"
		       "xgmi_width")

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

hsmp_test_init()
{
	if [[ -z "$AMD_HSMP_KO" ]]; then
		AMD_HSMP_KO=$PWD/$MOD_NAME
	fi

	# Assume cpu0 and socket0 unless specified
	if [[ -z "$HSMP_CPU" ]]; then
		HSMP_CPU=cpu0
	fi

	if [[ -z "$HSMP_SOCKET" ]]; then
		HSMP_SOCKET=socket0
	fi

	POSSIBLE_CPUS=`cat /sys/devices/system/cpu/possible`
	POSSIBLE_CPUS=${POSSIBLE_CPUS##*-}
	
	POSSIBLE_SOCKETS=`cat /sys/devices/system/node/possible`
	POSSIBLE_SOCKETS=${POSSIBLE_SOCKETS##*-}
	
	printf "Using $AMD_HSMP_KO\n"
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

	local cpu=0
	while [ $cpu -le $POSSIBLE_CPUS ]; do
		HSMP_CPU=$cpu
		local cpu_dir=$(hsmp_cpu_dir)
		validate_dir $cpu_dir
		for f in ${hsmp_cpu_files[@]}; do
			validate_file $cpu_dir/$f
		done

		cpu=$[$cpu + 1]
	done

	pr_debug "\n"
	
	local socket=0
	while [ $socket -le $POSSIBLE_SOCKETS ]; do
		HSMP_SOCKET=$socket
		local socket_dir=$(hsmp_socket_dir)
		validate_dir $socket_dir
		for f in ${hsmp_socket_files[@]}; do
			validate_file $socket_dir/$f
		done

		socket=$[$socket + 1]
	done

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

# TODO: Add read_xgmi_width and read_xgmi_speed functionality once supported
write_xgmi_width()
{
	local base_dir=$1
	local file=$base_dir/xgmi_width

	# xgmi_width is only created on 2P systems, make sure it's present
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
	
	# The only valid values to write to xgmi_width are -1, 8, and 16.
	# Validate these values.
	for i in 8 16 -1; do
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

	# Validate write failures with invalid link width
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
load_module()
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

unload_module()
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


## Main
hsmp_test_init

load_module

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

printf "    Reading $HSMP_SYSFS_BASE_DIR/xgmi_width...$TBD"
mark_tbd
printf "    Reading $HSMP_SYSFS_BASE_DIR/xgmi_speed...$TBD"
mark_tbd

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
# amd_hsmp/socketX/xgmi_width
cpu_dir=$(hsmp_cpu_dir)
socket_dir=$(hsmp_socket_dir)

printf "\n"
printf "Validating sysfs file write functionality\n"
write_boost_limit $HSMP_SYSFS_BASE_DIR
printf "\n"
write_xgmi_width $HSMP_SYSFS_BASE_DIR
printf "\n"
write_boost_limit $cpu_dir
printf "\n"
write_boost_limit $socket_dir
printf "\n"

write_fabric_pstate $socket_dir
printf "\n"
write_power_limit $socket_dir

printf "\n"
unload_module
# ??? Validate sysfs files are removed

total_tests=$[total_passed + total_failed + total_tbd]
printf "\n"
printf "Total Tests: $total_tests\n"
printf "$PASS: $total_passed\n"
printf "$FAILED: $total_failed\n"
printf "$TBD: $total_tbd\n"
