EXTRA_CFLAGS += -Dlinux -DLINUX -DBDC -DBCMDRIVER -DBCMDONGLEHOST -DDHDTHREAD -DBCMWPA2 \
    -DDHD_GPL -DDHD_SCHED -DBCMSDIO -DBCMLXSDMMC -DBCMPLATFORM_BUS -DDHD_BCMEVENTS      \
    -DSHOW_EVENTS -DANDROID_SPECIFIC -DENABLE_DEEP_SLEEP

################ OPTIONAL FEATURES ############################################################
#TO ENALBLE OPTIONAL FEATURES UNCOMMENT THE CORRESPONDING FLAGS

# For Samsung Aries only.
EXTRA_CFLAGS += -DCUSTOMER_HW_SAMSUNG

# For MACID management
ifeq ($(CONFIG_MACH_ANCORA),y)
EXTRA_CFLAGS += -DREAD_MACADDR
else ifeq ($(CONFIG_MACH_ANCORA_TMO),y)
EXTRA_CFLAGS += -DREAD_MACADDR
else
EXTRA_CFLAGS += -DWRITE_MACADDR
endif

# For AUTH time patch
# EXTRA_CFLAGS += -DAUTH_TIME_PATCH

# Check if this is required. SDIO_ISR_THREAD is mutually exclusive with OOB_INTR_ONLY
#EXTRA_CFLAGS += -DSDIO_ISR_THREAD

# For OOB only
EXTRA_CFLAGS += -DOOB_INTR_ONLY

# For HW_OOB
EXTRA_CFLAGS += -DHW_OOB

# FOR SOFTAP
EXTRA_CFLAGS += -DSOFTAP

# For Debug
EXTRA_CFLAGS += -DDHD_DEBUG -DSRCBASE=\"$(src)/src\"

# HOST WAKEUP
EXTRA_CFLAGS += -DBCM_HOSTWAKE

#STATIC MEMORY ALLOCATION FEATURE
EXTRA_CFLAGS += -DDHD_USE_STATIC_BUF
EXTRA_CFLAGS += -DOTHER_STATIC_BUF

EXTRA_CFLAGS += -DEMBEDDED_PLATFORM

EXTRA_CFLAGS += -DARP_OFFLOAD_SUPPORT
EXTRA_CFLAGS += -DDO_NOT_CHANGE_SSID
EXTRA_CFLAGS += -DPKT_FILTER_SUPPORT
#Disable TPC feature for FCC 
#EXTRA_CFLAGS += -DFCC_CERT

#Disable PowerSave mode for OTA or certification test
#EXTRA_CFLAGS += -DBCMDISABLE_PM

# Trigger first broadscan in driver
EXTRA_CFLAGS += -DCONFIG_FIRST_SCAN

# COMBO SCAN
EXTRA_CFLAGS += -DCSCAN

# For CCKM support
EXTRA_CFLAGS += -DBCMCCX

# For Debug
#EXTRA_CFLAGS += -DBCMDBG_EVENT

# FW Debug Trace
#EXTRA_CFLAGS += -DUSE_FW_TRACE

# OKC
EXTRA_CFLAGS += -DUSE_OKC

# Keep Alive
EXTRA_CFLAGS += -DUSE_KEEP_ALIVE

###############################################################################################

EXTRA_CFLAGS += -I$(src)/src/include/
EXTRA_CFLAGS += -I$(src)/src/dhd/sys/
EXTRA_CFLAGS += -I$(src)/src/dongle/
EXTRA_CFLAGS += -I$(src)/src/bcmsdio/sys/
EXTRA_CFLAGS += -I$(src)/src/wl/sys/
EXTRA_CFLAGS += -I$(src)/src/shared/

EXTRA_LDFLAGS += --strip-debug

KBUILD_CFLAGS += -I$(LINUXDIR)/include -I$(shell pwd)

obj-m	+= dhd.o

dhd-y := src/dhd/sys/dhd_linux.o \
         src/dhd/sys/dhd_common.o \
         src/dhd/sys/dhd_cdc.o \
         src/dhd/sys/dhd_linux_sched.o\
         src/dhd/sys/dhd_sdio.o \
         src/dhd/sys/dhd_custom_gpio.o \
         src/dhd/sys/dhd_custom_sec.o \
         src/shared/aiutils.o \
         src/shared/bcmutils.o \
         src/shared/bcmwifi.o \
         src/shared/hndpmu.o \
         src/shared/linux_osl.o \
         src/shared/sbutils.o \
         src/shared/siutils.o \
         src/bcmsdio/sys/bcmsdh_sdmmc.o \
         src/bcmsdio/sys/bcmsdh.o \
         src/bcmsdio/sys/bcmsdh_linux.o \
         src/bcmsdio/sys/bcmsdh_sdmmc_linux.o \
         src/wl/sys/wl_iw.o
all:
	@echo "$(MAKE) --no-print-directory -C $(KDIR) SUBDIRS=$(CURDIR) modules"
	@$(MAKE) --no-print-directory -C $(KDIR) \
		SUBDIRS=$(CURDIR) modules

clean: 
	rm -rf *.o *.o.cmd *.ko *.mod.c *~ .*.cmd Module.symvers modules.order .tmp_versions	\
 	 src/dhd/sys/dhd_linux.o \
         src/dhd/sys/dhd_common.o \
         src/dhd/sys/dhd_cdc.o \
         src/dhd/sys/dhd_linux_sched.o\
         src/dhd/sys/dhd_sdio.o \
         src/dhd/sys/dhd_custom_gpio.o \
         src/dhd/sys/dhd_custom_sec.o \
         src/shared/aiutils.o \
         src/shared/bcmutils.o \
         src/shared/bcmwifi.o \
         src/shared/hndpmu.o \
         src/shared/linux_osl.o \
         src/shared/sbutils.o \
         src/shared/siutils.o \
         src/bcmsdio/sys/bcmsdh_sdmmc.o \
         src/bcmsdio/sys/bcmsdh.o \
         src/bcmsdio/sys/bcmsdh_linux.o \
         src/bcmsdio/sys/bcmsdh_sdmmc_linux.o \
         src/wl/sys/wl_iw.o \
 	 src/dhd/sys/.dhd_linux.o.cmd \
         src/dhd/sys/.dhd_common.o.cmd \
         src/dhd/sys/.dhd_cdc.o.cmd \
         src/dhd/sys/.dhd_linux_sched.o.cmd\
         src/dhd/sys/.dhd_sdio.o.cmd \
         src/dhd/sys/.dhd_custom_gpio.o.cmd \
         src/dhd/sys/.dhd_custom_sec.o.cmd \
         src/shared/.aiutils.o.cmd \
         src/shared/.bcmutils.o.cmd \
         src/shared/.bcmwifi.o.cmd \
         src/shared/.hndpmu.o.cmd \
         src/shared/.linux_osl.o.cmd \
         src/shared/.sbutils.o.cmd \
         src/shared/.siutils.o.cmd \
         src/bcmsdio/sys/.bcmsdh_sdmmc.o.cmd \
         src/bcmsdio/sys/.bcmsdh.o.cmd \
         src/bcmsdio/sys/.bcmsdh_linux.o.cmd \
         src/bcmsdio/sys/.bcmsdh_sdmmc_linux.o.cmd \
         src/wl/sys/.wl_iw.o.cmd

install:
	@$(MAKE) --no-print-directory -C $(KDIR) \
		SUBDIRS=$(CURDIR) modules_install
