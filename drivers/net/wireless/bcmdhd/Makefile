# bcmdhd
DHDCFLAGS = -Wall -Wstrict-prototypes -Dlinux -DBCMDRIVER                     \
        -DBCMDONGLEHOST -DUNRELEASEDCHIP -DBCMDMA32 -DWLBTAMP -DBCMFILEIMAGE  \
        -DDHDTHREAD -DDHD_GPL -DDHD_SCHED -DDHD_DEBUG -DBDC                   \
        -DDHD_BCMEVENTS -DSHOW_EVENTS -DDONGLEOVERLAYS -DBCMDBG               \
        -DCUSTOMER_HW_SAMSUNG -DHW_OOB -DOOB_INTR_ONLY                        \
        -DMMC_SDIO_ABORT -DBCMSDIO -DBCMLXSDMMC -DBCMPLATFORM_BUS -DWLP2P     \
        -DNEW_COMPAT_WIRELESS -DWIFI_ACT_FRAME -DARP_OFFLOAD_SUPPORT          \
        -DKEEP_ALIVE -DCSCAN -DPKT_FILTER_SUPPORT                             \
        -DEMBEDDED_PLATFORM -DPNO_SUPPORT

ifeq ($(CONFIG_BCM4329),m)
DHDCFLAGS += -DBCM4329_CHIP
DHDCFLAGS += -DMCAST_LIST_ACCUMULATION
#DHDCFLAGS += -DCONFIG_CONTROL_PM
DHDCFLAGS += -DROAM_ENABLE -DROAM_API -DROAM_CHANNEL_CACHE
endif

ifeq ($(CONFIG_BCM4329),y)
DHDCFLAGS += -DBCM4329_CHIP
DHDCFLAGS += -DMCAST_LIST_ACCUMULATION
#DHDCFLAGS += -DCONFIG_CONTROL_PM
DHDCFLAGS += -DROAM_ENABLE -DROAM_API -DROAM_CHANNEL_CACHE
endif

ifeq ($(CONFIG_MACH_ANCORA),y)
DHDCFLAGS += -DREAD_MACADDR
else ifeq ($(CONFIG_MACH_ANCORA_TMO),y)
DHDCFLAGS += -DREAD_MACADDR
else ifeq ($(CONFIG_MACH_ARIESVE),y)
DHDCFLAGS += -DWRITE_MACADDR
endif

# For p2p connection issue
DHDCFLAGS += -DWL_CFG80211_GON_COLLISION
DHDCFLAGS += -DWL_CFG80211_SYNC_GON_TIME
#end p2p connection

# For Passing all multicast packets to host when not in suspend mode.
DHDCFLAGS += -DPASS_ALL_MCAST_PKTS

# For INITIAL 2G scan features
DHDCFLAGS += -DUSE_INITIAL_2G_SCAN

DHDCFLAGS +=-DINITIAL_2G_SCAN_BY_ESCAN

# For Scan result patch
DHDCFLAGS += -DESCAN_RESULT_PATCH

# For Static Buffer
ifeq ($(CONFIG_BROADCOM_WIFI_RESERVED_MEM),y)
DHDCFLAGS += -DCONFIG_DHD_USE_STATIC_BUF
endif

# For Debug
EXTRA_CFLAGS += $(DHDCFLAGS) -DWL_CFG80211 -DRSSI_OFFSET=0
EXTRA_CFLAGS += -DDHD_DEBUG -DSRCBASE=\"$(src)\"

EXTRA_CFLAGS += -I$(src)/include/ -I$(src)/

EXTRA_LDFLAGS += --strip-debug
KBUILD_CFLAGS += -I$(LINUXDIR)/include -I$(shell pwd)

obj-m   += dhd.o

dhd-y := bcmsdh.o	bcmsdh_linux.o \
	 bcmsdh_sdmmc.o	bcmsdh_sdmmc_linux.o \
	 dhd_bta.o	dhd_cdc.o \
	 dhd_common.o	dhd_custom_gpio.o \
	 dhd_custom_sec.o \
	 dhd_linux.o	dhd_linux_sched.o \
	 dhd_cfg80211.o	dhd_sdio.o \
	 aiutils.o	bcmevent.o \
	 bcmutils.o	bcmwifi_channels.o \
	 hndpmu.o	linux_osl.o \
	 sbutils.o	siutils.o \
	 wl_android.o	wl_cfg80211.o \
	 wl_cfgp2p.o	wldev_common.o \
	 wl_linux_mon.o	wl_roam.o

all:
	@echo "$(MAKE) --no-print-directory -C $(KDIR) SUBDIRS=$(CURDIR) modules"
	@$(MAKE) --no-print-directory -C $(KDIR) \
		SUBDIRS=$(CURDIR) modules

clean:
	rm -rf *.o *.ko *.mod.c *~ .*.cmd \
	Module.symvers modules.order .tmp_versions modules.builtin \
	bcmsdio/sys/*.o \
	bcmsdio/sys/*.o.cmd \
	bcmsdio/sys/.*.o.cmd \
        dhd/sys/*.o \
	dhd/sys/*.o.cmd \
        dhd/sys/.*.o.cmd \
        shared/*.o \
	shared/*.o.cmd \
        shared/.*.o.cmd \
        wl/sys/*.o \
        wl/sys/*.o.cmd \
	wl/sys/.*.o.cmd \
	wl/bcmwifi/src/bcmwifi_channels.o \
	wl/bcmwifi/src/.*.o.cmd

install:
	@$(MAKE) --no-print-directory -C $(KDIR) \
		SUBDIRS=$(CURDIR) modules_install
