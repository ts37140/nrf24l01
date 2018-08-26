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

#include "nrf24l01.h"

/* 1 byte for register address + max 5 bytes data + newline:
 * e.g. "0x07 0xE1E2E3E4E5"
 */
#define NRF24L01_MAX_REG_INPUT_LEN	18

ssize_t nrf24l01_sysfs_reg_store(struct device *dev,
		struct device_attribute *attr, const char *buff, size_t count)
{
	int res = EPERM;
	struct priv_data *priv = dev_get_drvdata(dev);
	unsigned int reg = 0;
	char *val_str = NULL;
	unsigned long long val = 0;

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
		return -EINVAL;
	}

	*val_str = 0;
	res = kstrtouint(buff, 16, &reg);
	if (res) {
		PERR(priv->dev, "register error %d\n", res);
		return -EINVAL;
	}

	/* And rest of string is value */
	val_str++;

	res = kstrtoull(val_str, 16, &val);
	if (res) {
		PERR(priv->dev, "value error %d\n", res);
		return -EINVAL;
	}

	PINFO(priv->dev, "reg:0x%x, val:0x%llx\n", reg, val);

	res = nrf24l01_spi_write_register(priv->spi_dev, reg, val);
	if (res)
		return -EINVAL;

	return count;
}

ssize_t nrf24l01_sysfs_reg_map_show(struct device *dev,
		struct device_attribute *attr, char *buff)
{
	int res = 0;
	struct priv_data *priv = dev_get_drvdata(dev);
	struct nrf24l01_registers *reg = &priv->spi_ops.reg_map;
	ssize_t len = 0;

	res = nrf24l01_spi_read_reg_map(priv->spi_dev);
	if (res)
		return 0;

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

	return len;
}

int nrf24l01_open(struct inode *inode, struct file *filp)
{
	return -EPERM;
}

int nrf24l01_release(struct inode *inode, struct file *filp)
{
	return -EPERM;
}

ssize_t nrf24l01_read(struct file *filp, char __user *ubuff,
		size_t count, loff_t *offp)
{
	return -EPERM;
}

ssize_t nrf24l01_write(struct file *filp, const char __user *ubuff,
		size_t count, loff_t *offp)
{
	return -EPERM;
}


