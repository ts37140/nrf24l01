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

#include "nrf24l01.h"

/* 1 byte for register address + max 5 bytes data + newline:
 * e.g. "0x07 0xE1E2E3E4E5"
 */
#define NRF24L01_MAX_REG_INPUT_LEN	18

static DEFINE_MUTEX(dev_mutex);

ssize_t nrf24l01_sysfs_reg_store(struct device *dev,
		struct device_attribute *attr, const char *buff, size_t count)
{
	int res = -EPERM;
	struct priv_data *priv = dev_get_drvdata(dev);
	unsigned int reg = 0;
	char *val_str = NULL;
	unsigned long long val = 0;

	/* No other spi traffic when char dev is opened */
	if (!mutex_trylock(&dev_mutex)) {
		PERR(priv->dev, "device busy\n");
		return -EBUSY;
	}

	if (count > NRF24L01_MAX_REG_INPUT_LEN) {
		PERR(priv->dev, "buffer too long\n");
		return -EINVAL;
	}

	/* Space-character is separating register and value.
	 * First part is register.
	 */
	val_str = memchr(buff, ' ', count);
	if (val_str == NULL) {
		PERR(priv->dev, "value not found\n");
		res = -EINVAL;
		goto out;
	}

	*val_str = 0;
	res = kstrtouint(buff, 16, &reg);
	if (res) {
		PERR(priv->dev, "register error %d\n", res);
		goto out;;
	}

	/* And rest of string is value */
	val_str++;

	res = kstrtoull(val_str, 16, &val);
	if (res) {
		PERR(priv->dev, "value error %d\n", res);
		goto out;
	}

	PINFO(priv->dev, "reg:0x%x, val:0x%llx\n", reg, val);

	res = nrf24l01_spi_write_register(priv->spi_dev, reg, val);
	if (res)
		goto out;

	res = count;

out:
	mutex_unlock(&dev_mutex);

	return res;
}

ssize_t nrf24l01_sysfs_reg_map_show(struct device *dev,
		struct device_attribute *attr, char *buff)
{
	int res = 0;
	struct priv_data *priv = dev_get_drvdata(dev);
	struct nrf24l01_registers *reg = &priv->spi_ops.reg_map;
	ssize_t len = 0;

	/* No other spi traffic when char dev is opened */
	if (!mutex_trylock(&dev_mutex)) {
		PERR(priv->dev, "device busy\n");
		return -EBUSY;
	}

	res = nrf24l01_spi_read_reg_map(priv->spi_dev);
	if (res)
		goto out;

	len += sprintf(buff + len,
		"config (0x0): 0x%02x\n", reg->config.value);
	len += sprintf(buff + len,
		"en_aa (0x1): 0x%02x\n", reg->en_aa.value);
	len += sprintf(buff + len,
		"en_rxaddr (0x2): 0x%02x\n", reg->en_rxaddr.value);
	len += sprintf(buff + len,
		"setup_aw (0x3): 0x%02x\n", reg->setup_aw.value);
	len += sprintf(buff + len,
		"setup_retr (0x4): 0x%02x\n", reg->setup_retr.value);
	len += sprintf(buff + len,
		"rf_ch (0x5): 0x%02x\n", reg->rf_ch.value);
	len += sprintf(buff + len,
		"rf_setup (0x6): 0x%02x\n", reg->rf_setup.value);
	len += sprintf(buff + len,
		"status (0x7): 0x%02x\n", reg->status.value);
	len += sprintf(buff + len,
		"observe_tx (0x8): 0x%02x\n", reg->observe_tx.value);
	len += sprintf(buff + len,
		"rpd (0x9): 0x%02x\n", reg->rpd.value);
	len += sprintf(buff + len,
		"rx_addr_p0 (0xA): 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n",
		reg->rx_addr_p0.value[0], reg->rx_addr_p0.value[1],
		reg->rx_addr_p0.value[2], reg->rx_addr_p0.value[3],
		reg->rx_addr_p0.value[4]);
	len += sprintf(buff + len,
		"rx_addr_p1 (0xB): 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n",
		reg->rx_addr_p1.value[0], reg->rx_addr_p1.value[1],
		reg->rx_addr_p1.value[2], reg->rx_addr_p1.value[3],
		reg->rx_addr_p1.value[4]);
	len += sprintf(buff + len,
		"rx_addr_p2 (0xC): 0x%02x\n", reg->rx_addr_p2.value);
	len += sprintf(buff + len,
		"rx_addr_p3 (0xD): 0x%02x\n", reg->rx_addr_p3.value);
	len += sprintf(buff + len,
		"rx_addr_p4 (0xE): 0x%02x\n", reg->rx_addr_p4.value);
	len += sprintf(buff + len,
		"rx_addr_p5 (0xF): 0x%02x\n", reg->rx_addr_p5.value);
	len += sprintf(buff + len,
		"tx_addr (0x10): 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n",
		reg->tx_addr.value[0], reg->tx_addr.value[1],
		reg->tx_addr.value[2], reg->tx_addr.value[3],
		reg->tx_addr.value[4]);
	len += sprintf(buff + len,
		"rx_pw_p0 (0x11): 0x%02x\n", reg->rx_pw_p0.value);
	len += sprintf(buff + len,
		"rx_pw_p1 (0x12): 0x%02x\n", reg->rx_pw_p1.value);
	len += sprintf(buff + len,
		"rx_pw_p2 (0x13): 0x%02x\n", reg->rx_pw_p2.value);
	len += sprintf(buff + len,
		"rx_pw_p3 (0x14): 0x%02x\n", reg->rx_pw_p3.value);
	len += sprintf(buff + len,
		"rx_pw_p4 (0x15): 0x%02x\n", reg->rx_pw_p4.value);
	len += sprintf(buff + len,
		"rx_pw_p5 (0x16): 0x%02x\n", reg->rx_pw_p5.value);
	len += sprintf(buff + len,
		"fifo_status (0x17): 0x%02x\n", reg->fifo_status.value);
	len += sprintf(buff + len,
		"dynpd (0x1C): 0x%02x\n", reg->dynpd.value);
	len += sprintf(buff + len,
		"feature (0x1D): 0x%02x\n", reg->feature.value);

	res = len;
out:
	mutex_unlock(&dev_mutex);

	return res;
}

ssize_t nrf24l01_sysfs_info_show(struct device *dev,
	struct device_attribute *attr, char *buff)
{
	struct priv_data *priv = dev_get_drvdata(dev);

	return sprintf(buff, "byte count: %d\n", priv->byte_count);
}

int nrf24l01_open(struct inode *inode, struct file *filp)
{
	int res = -EPERM;
	struct priv_data *priv = filp->private_data;

	priv = container_of(inode->i_cdev, struct priv_data, cdev);
	filp->private_data = priv;

	res = mutex_lock_interruptible(&dev_mutex);
	if (res) {
		PERR(priv->dev, "mutex interrupted %d\n", res);
		return res;
	}

	PINFO(priv->dev, "mutex acquired\n");

	res = nrf24l01_spi_refresh_payload_size(priv->spi_dev);
	if (res)
		goto err;

	priv->byte_count = 0;

	return res;

err:
	mutex_unlock(&dev_mutex);
	return res;
}

int nrf24l01_release(struct inode *inode, struct file *filp)
{
	mutex_unlock(&dev_mutex);

	return 0;
}

ssize_t nrf24l01_read(struct file *filp, char __user *ubuff,
		size_t count, loff_t *offp)
{
	ssize_t res = -EPERM;
	struct priv_data *priv = filp->private_data;

	if (count < NRF24L01_PAYLOAD_LEN) {
		PERR(priv->dev, "input buffer too small: %d\n", count);
		return -ENOMEM;
	}

	res = nrf24l01_spi_receive_payload(priv->spi_dev);
	if (res)
		return res;

	res = copy_to_user(ubuff, priv->spi_ops.payload.rx,
			priv->spi_ops.payload_size);
	if (res) {
		PERR(priv->dev, "error copy_to_user %d\n", res);
		return -EFAULT;
	}

	res = priv->spi_ops.payload_size;

	/* offp is not updated. The whole "file" is always
	 * returned to userspace.
	 */

	priv->byte_count += res;

	return res;
}

ssize_t nrf24l01_write(struct file *filp, const char __user *ubuff,
		size_t count, loff_t *offp)
{
	ssize_t res = -EPERM;
	struct priv_data *priv = filp->private_data;

	if (count > NRF24L01_PAYLOAD_LEN) {
		count = NRF24L01_PAYLOAD_LEN;
	}

	res = copy_from_user(priv->spi_ops.payload.tx, ubuff, count);
	if (res) {
		PERR(priv->dev, "error copy_from_user %d\n", res);
		return -EFAULT;
	}

	priv->spi_ops.payload.data_len = count;

	res = nrf24l01_spi_send_payload(priv->spi_dev);
	if (res)
		return res;
	else {
		*offp += count;
		priv->byte_count += count;

		return count;
	}
}


