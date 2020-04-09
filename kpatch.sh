#/bin/bash

GIT=/usr/bin/git
PATCH=/usr/bin/patch

current_branch=""
branch=hsmp_kpatch

mk_files()
{
	echo "Creating Kconfig patch..."
	kconfig_patch=$(mktemp)
	cat > $kconfig_patch << EOF
diff --git a/drivers/misc/Kconfig b/drivers/misc/Kconfig
index 99e151475d8f..a82599a7aae8 100644
--- a/drivers/misc/Kconfig
+++ b/drivers/misc/Kconfig
@@ -465,6 +465,16 @@ config PVPANIC
          a paravirtualized device provided by QEMU; it lets a virtual machine
          (guest) communicate panic events to the host.

+config AMD_HSMP
+       tristate "AMD HSMP Driver"
+       depends on X86
+       default m
+       help
+         The HSMP driver is an experimental kernel module for providing
+         userspace and kernel access to the Host System Management Port
+         on AMD systems. Initially this module is supported only on AMD
+         family 17h and family 19h".
+
 source "drivers/misc/c2port/Kconfig"
 source "drivers/misc/eeprom/Kconfig"
 source "drivers/misc/cb710/Kconfig"
EOF

	echo "Creating commit message file..."
	commit_msg=$(mktemp)
	cat > $commit_msg << EOF
[INTERNAL] drivers/misc: Add HSMP Driver

The HSMP driver is an experimental kernel module for providing userspace
and kernel access to the Host System Management Port on AMD systems.
Initially this module is supported only on AMD family 17h anf family 19h.

See amd_hsmp.h for descriptions of the kernel API.
See amd_hsmp.c for details about the SysFS interface.

Signed-off-by: Nathan Fontenot <nafonten@amd.com>
EOF
}

rm_files()
{
	# delete files
	if [ ! -z $kconfig_patch ]; then
		rm $kconfig_patch
	fi

	if [ ! -z $commit_msg ]; then
		rm $commit_msg
	fi
}

hsmp_source=$1

echo "Validating HSMP source directory..."
if [ -z $hsmp_source ]; then
	echo "No kernel source path specified"
	exit -1
fi

# Verify kernel source dir exists
if [ ! -d $hsmp_source ]; then
	echo "HSMP Driver source $hsmp_source appears invalid"
	exit -1
fi

echo "Verifying we are in a kernel source tree..."
# Verify we're in a kernel source tree, simple check for Kbuild
if [ ! -f ./Kbuild ]; then
	echo "This does not appear to be a linux kernel directory"
	return -1
fi

# The updates to drivers/misc/Kconfig is a patch we keep in this
# script, verify it will apply before doing any work
mk_files

echo "Ensuring Kconfig patch applies cleanly..."
$PATCH -p1 --dry-run < $kconfig_patch
if [ "$?" -ne 0 ]; then
	echo "Failed to verify patch for drivers/misc/Kconfig"
	rm_files
	exit -1
fi

# If a new branch is specified, or we're just creating a patch
# create the new branch now.
current_branch=$($GIT rev-parse --abbrev-ref HEAD)

echo "Creating kernel git tree branch $branch..."
$GIT checkout -b $branch
if [ "$?" -ne 0 ]; then
	echo "Failed to create kernel branch"
	rm_files
	exit -1
fi

# Apply patch to drivers/misc/Kconfig. We verified it will apply
# earlier so shouldn't need to check for success.
echo "Applying Kconfig patch..."
$PATCH -p1 < $kconfig_patch

echo "Updating drivers/misc/Makefile..."
# Patch drivers/misc/Makefile
echo "obj-$(CONFIG_AMD_HSMP)         += amd_hsmp.o" >> drivers/misc/Makefile

# Copy driver code to drivers/misc
echo "Copying HSMP Driver source to drivers/misc..."
cp $hsmp_source/amd_hsmp.c drivers/misc
$GIT add drivers/misc/amd_hsmp.c

cp $hsmp_source/amd_hsmp.h drivers/misc
$GIT add drivers/misc/amd_hsmp.h

echo "Committing HSMP Driver..."
$GIT commit -a --author="Nathan Fontenot <nathan.fontenot@amd.com>" -F $commit_msg

# Create patch
echo "Creating HSMP Driver patch..."
patch_file=$($GIT format-patch -1 HEAD)

# delete the kernel branch
$GIT checkout $current_branch
$GIT branch -D $branch

rm_files

echo "Patch file $patch_file created"
