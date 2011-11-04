1. How to Build
    - get Toolchain
        From Codesourcery site( http://www.codesourcery.com )
            Ex) Download : http://www.codesourcery.com/sgpp/lite/arm/portal/package7813/public/arm-none-eabi/arm-2010.09-51-arm-none-eabi-i686-pc-linux-gnu.tar.bz2
            recommand :
                Feature : ARM
                Target OS : "EABI"
                package : "IA32 GNU/Linux TAR"

        - edit Makefile
                edit "CROSS_COMPILE" to right toolchain path(You downloaded).
                Ex) ARCH		?= arm
                    CROSS_COMPILE	?= /opt/toolchains/arm-2010.09/bin/arm-none-eabi-   // You have to check compiler path

        - command
                $ cd kernel
                $ make ancora_rev00_defconfig
                $ make

2. Output files
        - Kernel : kernel/arch/arm/boot/zImage