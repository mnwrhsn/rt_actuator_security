#!/bin/sh

# This script will copy the contents to removable storage for
# Raspberry pi

# check lsblk output and make sure the device id, eg. /dev/sdbX

DEV_BOOT="/dev/sdb1"
DEV_ROOTFS="/dev/sdb2"

# OPTEE_ROOT="/home/mhasan/workspace/rpi_optee310"
OPTEE_ROOT="/home/mhasan/workspace/rt_sec_io_tz"

# mount filesystem

echo "Mounting filesystem..."

[ ! -d "/media/boot" ] && {
    sudo mkdir /media/boot
}
[ ! -d "/media/rootfs" ] && {
    sudo mkdir /media/rootfs
}

sudo mount $DEV_BOOT /media/boot
sudo mount $DEV_ROOTFS /media/rootfs

echo "Copy boot ..."

# copy filesystme, library
cd /media
LOC="$OPTEE_ROOT/out-br/images/rootfs.cpio.gz"
sudo gunzip -cd $LOC | sudo cpio -iudmv "boot/*"

echo "Enable I2C..."

echo "dtparam=i2c_arm=on" >> boot/config.txt
echo "dtparam=spi=on" >> boot/config.txt

echo "Copy library and examples ..."

LOC="$OPTEE_ROOT/module_output/lib/*"
sudo cp -r $LOC rootfs/lib/

LOC="$OPTEE_ROOT/optee_client/out/export/*"
sudo cp -r $LOC rootfs/

# copy OPTEE examples
LOC="$OPTEE_ROOT/optee_examples/out/ca/*"
sudo cp -r $LOC  rootfs/bin/

# copy TA files
LOC="$OPTEE_ROOT/optee_examples/out/ta/*"
# create directory if doesn't exist
sudo mkdir rootfs/lib/optee_armtz/
sudo cp -r $LOC rootfs/lib/optee_armtz/

echo "Unmounting filesystem..."

# Unmount
sudo umount /media/boot/
sudo umount /media/rootfs/

# sudo rm -r /media/boot
# sudo rm -r /media/rootfs

echo "Copy Script finished!"
