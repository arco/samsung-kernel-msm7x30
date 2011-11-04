
# Makefile for wlan sdio if driver

librasdioif-objs += libra_sdioif.o

ifdef CONFIG_ARCH_MSM8X60
	librasdioif-objs += qcomwlan_pwrif.o
endif

obj-$(CONFIG_LIBRA_SDIOIF) += librasdioif.o
