export CROSS_COMPILE=/home/spepc/RV1126B/02fireware/atk_dlrv1126b_linux6.1_sdk_release_v1.0.0_20251202/prebuilts/gcc/linux-x86/aarch64/gcc-arm-10.3-2021.07-x86_64-aarch64-none-linux-gnu/bin/aarch64-none-linux-gnu-
export ARCH=arm64
obj-m += iris_driver.o
KDIR :=/home/spepc/RV1126B/02fireware/atk_dlrv1126b_linux6.1_sdk_release_v1.0.0_20251202/kernel-6.1
PWD ?= $(shell pwd)

all:
	make -C $(KDIR) M=$(PWD) modules  
clean:
	make -C $(KDIR) M=$(PWD) clean 