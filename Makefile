obj-m		:= nrf24l01.o
nrf24l01-objs:= nrf24l01_drv.o nrf24l01_spi.o nrf24l01_ops.o
ARCH		:= arm
PWD		:= $(shell pwd)

modules:
	make -C $(KERN_SRC) C=1 M=$(PWD) ARCH=$(ARCH) CROSS_COMPILE=$(CCPREFIX) modules

clean:
	make -C $(KERN_SRC) M=$(PWD) ARCH=$(ARCH) CROSS_COMPILE=$(CCPREFIX) clean

raspi_install:
	scp nrf24l01.ko $(RASPI_USER)@$(RASPI_ADDR):$(RASPI_PATH)