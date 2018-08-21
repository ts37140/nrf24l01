/*
 * Copyright (c) 2018, Tero Salminen
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#define PINFO(_dev, fmt, args...) \
	dev_info(_dev, "%s: "fmt, __func__, ##args)

#define PERR(_dev, fmt, args...) \
	dev_err(_dev, "%s (%d): "fmt, __func__, __LINE__, ##args)


#include <linux/types.h>
#include <asm-generic/errno-base.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/kdev_t.h>
#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/spi/spi.h>
#include <linux/of.h>

#define NRF24L01_SPI_BUFFER		70
#define NRF24L01_SPI_XFER_BUFFER	26

/* Status register value is always returned in the first
 * byte of SPI read operation.
 */
struct nrf24l01_reg_read {
	uint8_t status;
	uint8_t value;
};

/* 3 registers contain address length of 5 bytes */
struct nrf24l01_reg_read_addr {
	uint8_t status;
	uint8_t value[5];
};

/* NRF24L01 registers in ascending order by address */
struct nrf24l01_registers {
	struct nrf24l01_reg_read	config;
	struct nrf24l01_reg_read	en_aa;
	struct nrf24l01_reg_read	en_rxaddr;
	struct nrf24l01_reg_read	setup_aw;
	struct nrf24l01_reg_read	setup_retr;
	struct nrf24l01_reg_read	rf_ch;
	struct nrf24l01_reg_read	rf_setup;
	struct nrf24l01_reg_read	status;
	struct nrf24l01_reg_read	observe_tx;
	struct nrf24l01_reg_read	rpd;
	struct nrf24l01_reg_read_addr	rx_addr_p0;
	struct nrf24l01_reg_read_addr	rx_addr_p1;
	struct nrf24l01_reg_read	rx_addr_p2;
	struct nrf24l01_reg_read	rx_addr_p3;
	struct nrf24l01_reg_read	rx_addr_p4;
	struct nrf24l01_reg_read	rx_addr_p5;
	struct nrf24l01_reg_read_addr	tx_addr;
	struct nrf24l01_reg_read	rx_pw_p0;
	struct nrf24l01_reg_read	rx_pw_p1;
	struct nrf24l01_reg_read	rx_pw_p2;
	struct nrf24l01_reg_read	rx_pw_p3;
	struct nrf24l01_reg_read	rx_pw_p4;
	struct nrf24l01_reg_read	rx_pw_p5;
	struct nrf24l01_reg_read	fifo_status;
	struct nrf24l01_reg_read	dynpd;
	struct nrf24l01_reg_read	feature;
};

struct nrf24l01_spi_ops {

	struct nrf24l01_registers reg_map;
};

struct priv_data {
	dev_t dev_num;

	struct cdev cdev;

	struct device *dev;

	struct spi_device *spi_dev;

	struct class *nrf24l01_class;

	struct nrf24l01_spi_ops spi_ops;
};

ssize_t nrf24l01_sysfs_config_show(struct device *dev,
	struct device_attribute *attr, char *buff);
ssize_t nrf24l01_sysfs_config_store(struct device *dev,
	struct device_attribute *attr, const char *buff, size_t count);
ssize_t nrf24l01_sysfs_status_show(struct device *dev,
	struct device_attribute *attr, char *buff);
ssize_t nrf24l01_sysfs_status_store(struct device *dev,
	struct device_attribute *attr, const char *buff, size_t count);
ssize_t nrf24l01_sysfs_reg_map_show(struct device *dev,
	struct device_attribute *attr, char *buff);
int nrf24l01_open(struct inode *inode, struct file *filp);
int nrf24l01_release(struct inode *inode, struct file *filp);
ssize_t nrf24l01_read(struct file *filp, char __user *ubuff,
	size_t count, loff_t *offp);
ssize_t nrf24l01_write(struct file *filp, const char __user *ubuff,
	size_t count, loff_t *offp);

int nrf24l01_spi_read_reg_map(struct spi_device *spi);
int nrf24l01_spi_setup(struct spi_device *spi);
