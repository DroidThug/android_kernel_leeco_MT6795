#!/bin/sh

#
 # Custom build script by DroidThug
 #
 # This software is licensed under the terms of the GNU General Public
 # License version 2, as published by the Free Software Foundation, and
 # may be copied, distributed, and modified under those terms.
 #
 # This program is distributed in the hope that it will be useful,
 # but WITHOUT ANY WARRANTY; without even the implied warranty of
 # MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 # GNU General Public License for more details.
 #
#
restore='\033[0m'
KERNEL_DIR=$PWD
KERNEL="Image.gz-dtb"
ANYKERNEL_DIR="$KERNEL_DIR/release/AnyKernel2"
REPACK_DIR="$ANYKERNEL_DIR"
ZIP_MOVE="$KERNEL_DIR"
KERN_IMG=$KERNEL_DIR/arch/arm64/boot/Image.gz-dtb
BASE_VER="Stock"
VER="_$(date +"%Y-%m-%d"-%H%M)"
export ZIP_VER="$BASE_VER$VER$TC"
BUILD_START=$(date +"%s")
blue='\033[0;34m'
cyan='\033[0;36m'
yellow='\033[0;33m'
green='\033[01;32m'
red='\033[0;31m'
blink_red='\033[05;31m'
nocol='\033[0m'
TC="UBERTC"
# Modify the following variable if you want to build
export KBUILD_BUILD_USER="DroidThug"
MODULES_DIR=$KERNEL_DIR/arch/arm/boot/AnyKernel2/modules
if [ -d "/home/travis" ]; then
export KBUILD_BUILD_HOST="TravisCI"
echo "Hello from Travis!"
echo "Skipping export variables. I iz da kewl bot!"
is_travis=true
else
echo "Hello Human!"
export KBUILD_BUILD_HOST="EvoqueUnit"
export ARCH=arm64
export LD_LIBRARY_PATH="/home/DroidThug/aarch64-linux-android-gcc-4.9/lib"
export CROSS_COMPILE="/home/DroidThug/aarch64-linux-android-gcc-4.9/bin/aarch64-linux-android-"
export SUBARCH=arm64
export STRIP="/home/DroidThug/aarch64-linux-android-gcc-4.9/bin/aarch64-linux-android-"
is_hooman=true
fi

compile_clean_dirty ()
{
if(whiptail --title "LeEco KERNEL" --yesno "Would you like to keep this build clean?" 10 70) then
    echo -e "$green Cleaning $ZIP_VER Kernel! $nocol"
    echo -e "$red****************************************************************************************************************"
    echo "          Cleaning Up Before Compile          "
    echo -e "********************************************************************************************************************$nocol"
    make clean && make mrproper
else
      echo -e "$red Keeping it Dirty! Eww? $nocol"
fi
}

echo -e "${green}"
echo "--------------------------------------------------------"
echo "      Initializing build to compile Ver: $ZIP_VER    "
echo "--------------------------------------------------------"

echo -e "$yellow***********************************************"
echo "          Compiling KERNEL         "
echo -e "***********************************************$nocol"
rm -f $KERN_IMG
if [ "$is_travis" = true ] ; then
    echo 'Cleaning by default!'
    make clean && make mrproper
elif [ "$is_hooman" = true ] ; then
    git clone https://github.com/DroidThug/AnyKernel2 release/AnyKernel2
    echo 'Choose hooman! Choose wisely!'
    compile_clean_dirty
fi
echo -e "$yellow***********************************************"
echo "          Initialising DEFCONFIG        "
echo -e "***********************************************$nocol"
make x500_defconfig 
echo -e "$yellow***********************************************"
echo "          Cooking INFERNUS         "
echo -e "***********************************************$nocol"
time make -j8
cp -vr $KERN_IMG $REPACK_DIR/Image.gz-dtb


############
## DEPLOY ##
############
 #
 # This software is licensed under the terms of the GNU General Public
 # License version 2, as published by the Free Software Foundation, and
 # may be copied, distributed, and modified under those terms.
 #
 # This program is distributed in the hope that it will be useful,
 # but WITHOUT ANY WARRANTY; without even the implied warranty of
 # MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 # GNU General Public License for more details.
 #
#

if [ "$is_travis" = true ] ; then
    BUILDER=travisCI
    echo "Build success"
elif [ "$is_hooman" = true ] ; then
    BUILDER=$KBUILD_BUILD_USER
    echo -e 'Builder detected: $BUILDER'
    cd release/AnyKernel2
    zip -r9 $ZIP_VER-$BUILDER.zip * -x README.md $ZIP_VER-$BUILDER.zip
    echo "done!"
    echo 'Zipped succesfully'
fi

## END
