/*
 * Copyright (c) 2018, Tero Salminen
 * All rights reserved.
 *  
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
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
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/spinlock.h>
#include <linux/mutex.h>

/* Status register value is always returned in the first
 * byte of SPI read operation.
 */
struct nrf24l01_reg_read {
	uint8_t status;
	uint8_t value;
};

/* 3 registers contain address length of 5 bytes */
#define NRF24L01_MAX_ADDR_LEN		5
struct nrf24l01_reg_read_addr {
	uint8_t status;
	uint8_t value[NRF24L01_MAX_ADDR_LEN];
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

/* rx/tx payload spi transfer buffers */
#define NRF24L01_PAYLOAD_LEN		32
struct nrf24l01_rxtx_payload {
	uint8_t data_len;
	uint8_t cmd;				/* tx_buf[0] */
	uint8_t tx[NRF24L01_PAYLOAD_LEN];	/* tx_buf[1..31] */
	uint8_t status_reg;			/* rx_buf[0] */
	uint8_t rx[NRF24L01_PAYLOAD_LEN];	/* rx_buf[1..31] */
};

#define NRF24L01_CONFIG_PTX		0 /* tx mode */
#define NRF24L01_CONFIG_PRX		1 /* rx mode */

struct nrf24l01_spi_ops {
	struct nrf24l01_registers	reg_map;
	struct gpio_desc 		*gpio_ce_rxtx;
	struct gpio_desc 		*gpio_irq;

	wait_queue_head_t 		waitq;

	volatile uint8_t		rf_done;

	/* config reg: payload size */
	uint8_t				payload_size;
	struct nrf24l01_rxtx_payload 	payload;

	/* config reg: rx/tx mode */
	uint8_t				prim_rx;
};

struct priv_data {
	dev_t 			dev_num;

	struct cdev 		cdev;

	struct device 		*dev;

	struct spi_device 	*spi_dev;

	struct class 		*nrf24l01_class;

	struct nrf24l01_spi_ops	spi_ops;

	uint32_t		byte_count;
};

ssize_t nrf24l01_sysfs_reg_store(struct device *dev,
	struct device_attribute *attr, const char *buff, size_t count);
ssize_t nrf24l01_sysfs_reg_map_show(struct device *dev,
	struct device_attribute *attr, char *buff);
ssize_t nrf24l01_sysfs_info_show(struct device *dev,
	struct device_attribute *attr, char *buff);
int nrf24l01_open(struct inode *inode, struct file *filp);
int nrf24l01_release(struct inode *inode, struct file *filp);
ssize_t nrf24l01_read(struct file *filp, char __user *ubuff,
	size_t count, loff_t *offp);
ssize_t nrf24l01_write(struct file *filp, const char __user *ubuff,
	size_t count, loff_t *offp);

int nrf24l01_spi_send_payload(struct spi_device *spi);
int nrf24l01_spi_receive_payload(struct spi_device *spi);
int nrf24l01_spi_write_register(struct spi_device *spi, unsigned int reg,
	unsigned long long val);
int nrf24l01_spi_read_reg_map(struct spi_device *spi);
int nrf24l01_spi_refresh_payload_size(struct spi_device *spi);
int nrf24l01_spi_refresh_primrx_status(struct spi_device *spi);
int nrf24l01_spi_setup(struct spi_device *spi);
