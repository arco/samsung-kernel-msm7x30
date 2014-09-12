# bcmdhd
#
#
#
#
#

DHDCFLAGS = -Wall -Wstrict-prototypes -Dlinux -DBCMDRIVER                     \
        -DCUSTOMER_HW4 -DBLOCK_IPV6_PACKET -DPASS_IPV4_SUSPEND                \
        -DSUPPORT_DEEP_SLEEP -DSIMPLE_MAC_PRINT                               \
        -DBCMDONGLEHOST -DUNRELEASEDCHIP -DBCMDMA32 -DBCMFILEIMAGE            \
        -DDHDTHREAD -DBDC -DHW_OOB -DOOB_INTR_ONLY                            \
        -DDHD_BCMEVENTS -DSHOW_EVENTS -DBCMDBG -DROAM_ENABLE                  \
        -DMMC_SDIO_ABORT -DBCMSDIO -DBCMLXSDMMC -DBCMPLATFORM_BUS -DWLP2P     \
        -DWIFI_ACT_FRAME -DARP_OFFLOAD_SUPPORT -DESCAN_RESULT_PATCH           \
        -DKEEP_ALIVE -DCSCAN -DPKT_FILTER_SUPPORT                             \
        -DEMBEDDED_PLATFORM -DPNO_SUPPORT                                     \
	    -Idrivers/net/wireless/bcmdhd -Idrivers/net/wireless/bcmdhd/include   \
	    -Idrivers/net/wireless/bcmdhd/common/include

DHDCFLAGS += -DCUSTOM_SDIO_F2_BLKSIZE=128

# For p2p connection issue
DHDCFLAGS += -DWL_SCB_TIMEOUT=10

# For Passing all multicast packets to host when not in suspend mode.
DHDCFLAGS += -DPASS_ALL_MCAST_PKTS

# Early suspend
DHDCFLAGS += -DDHD_USE_EARLYSUSPEND
#DHDCFLAGS += -DSUPPORT_PM2_ONLY

# For Static Buffer
ifeq ($(CONFIG_BROADCOM_WIFI_RESERVED_MEM),y)
  DHDCFLAGS += -DCONFIG_DHD_USE_STATIC_BUF
  DHDCFLAGS += -DENHANCED_STATIC_BUF
  DHDCFLAGS += -DSTATIC_WL_PRIV_STRUCT
endif

# SoftAP
DHDCFLAGS += -DSUPPORT_AUTO_CHANNEL -DSUPPORT_HIDDEN_AP
DHDCFLAGS += -DSUPPORT_SOFTAP_SINGL_DISASSOC
DHDCFLAGS += -DUSE_STAMAC_4SOFTAP

# DPC priority
DHDCFLAGS += -DCUSTOM_DPC_PRIO_SETTING=98

# WiFi turn off delay
DHDCFLAGS += -DDEFAULT_WIFI_TURNOFF_DELAY=100

# WiFi Kernel thread type
DHDCFLAGS += -DUSE_KTHREAD_API

ifneq ($(CONFIG_BCM4329),)
  DHDCFLAGS += -DBCM4329_CHIP
  #DHDCFLAGS += -DMCAST_LIST_ACCUMULATION
  DHDCFLAGS += -DCUSTOM_GLOM_SETTING=0
  DHDCFLAGS += -DPASS_ARP_PACKET
endif

ifeq ($(CONFIG_MACH_ANCORA),y)
DHDCFLAGS += -DREAD_MACADDR
else ifeq ($(CONFIG_MACH_ANCORA_TMO),y)
DHDCFLAGS += -DREAD_MACADDR
else ifeq ($(CONFIG_MACH_ARIESVE),y)
DHDCFLAGS += -DWRITE_MACADDR
endif

DHDOFILES = aiutils.o bcmsdh_sdmmc_linux.o dhd_linux.o siutils.o bcmutils.o   \
	dhd_linux_sched.o dhd_sdio.o bcmwifi_channels.o bcmevent.o hndpmu.o   \
	bcmsdh.o dhd_cdc.o bcmsdh_linux.o dhd_common.o dhd_wlfc.o dhd_ip.o    \
	linux_osl.o bcmsdh_sdmmc.o dhd_custom_gpio.o sbutils.o wldev_common.o \
	wl_android.o dhd_pno.o dhd_custom_sec.o

obj-$(CONFIG_BCMDHD) += dhd.o
dhd-objs += $(DHDOFILES)
ifneq ($(CONFIG_WIRELESS_EXT),)
dhd-objs += wl_iw.o
DHDCFLAGS += -DSOFTAP -DWL_WIRELESS_EXT -DUSE_IW
endif
ifneq ($(CONFIG_CFG80211),)
dhd-objs += wl_cfg80211.o wl_cfgp2p.o wl_linux_mon.o dhd_cfg80211.o
DHDCFLAGS += -DWL_CFG80211 -DWL_CFG80211_STA_EVENT -DWL_ENABLE_P2P_IF
DHDCFLAGS += -DWL_IFACE_COMB_NUM_CHANNELS
DHDCFLAGS += -DCUSTOM_ROAM_TRIGGER_SETTING=-75
DHDCFLAGS += -DCUSTOM_ROAM_DELTA_SETTING=10
DHDCFLAGS += -DCUSTOM_KEEP_ALIVE_SETTING=28000
DHDCFLAGS += -DCUSTOM_PNO_EVENT_LOCK_xTIME=10
endif
ifneq ($(CONFIG_DHD_USE_SCHED_SCAN),)
DHDCFLAGS += -DWL_SCHED_SCAN
endif
EXTRA_CFLAGS = $(DHDCFLAGS)
ifeq ($(CONFIG_BCMDHD),m)
EXTRA_LDFLAGS += --strip-debug
endif
