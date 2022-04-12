ifeq ($(ARCH),x86_64)
  export KERNEL_DIR = /lib/modules/$(shell uname -r)/build
#  export KERNEL_DIR = /usr/src/linux-4.19.183-uni-col
else ifdef DEVICE
 include $(PWD)/openwrt.mk
endif

# standard flags for module builds
EXTRA_CFLAGS += -DLINUX -D__KERNEL__ -DMODULE -O2 -pipe -Wall

TARGET=test_main.o
obj-m:=$(TARGET)

all:
#	make x86
	make arm
#	make mips
#	make ramips

arch:
	echo "!!! $(DEVICE) !!! $(ARCH) !!! $(KERNEL_DIR) !!!"
	$(MAKE) -C $(KERNEL_DIR) M=$$PWD

x86:
	$(MAKE) arch ARCH=x86_64 DEVICE=x86_native

mips:
	$(MAKE) arch ARCH=mips DEVICE=ath79-mikrotik

ramips:
	$(MAKE) arch ARCH=mips DEVICE=ramips-mt7621

arm:
	$(MAKE) arch ARCH=arm DEVICE=hap_ac2

arm-b1300:
	$(MAKE) arch ARCH=arm DEVICE=gl-b1300

arm64:
	$(MAKE) arch ARCH=arm64 DEVICE=rb5009

clean:
		rm -f .*.cmd *.mod.c *.mod *.ko *.o *~ core $(TARGETS)
		rm -Rf .tmp_versions built-in.a *.symvers *.order .tmp_*
