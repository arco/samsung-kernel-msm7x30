# Makefile for the Linux Dpram drivers.

# Each configuration option enables a list of files.

#ifeq ($(CONFIG_ARM),y)
#EXTRA_CFLAGS    += -Idrivers/fsr/Inc -Idrivers/fsr/PAM/MSM7k -D__CC_ARM
#endif

# FSR base module
obj-m			+= sec_param.o

# Should keep the build sequence. (fsr_base -> bml_block)
sec_param-objs	+= param.o

