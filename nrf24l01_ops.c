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

ssize_t nrf24l01_sysfs_config_show(struct device *dev,
		struct device_attribute *attr, char *buff)
{
	struct priv_data *priv = dev_get_drvdata(dev);

	return sprintf(buff, "0x%02x\n", priv->spi_ops.reg_map.config.value);
}

ssize_t nrf24l01_sysfs_config_store(struct device *dev,
		struct device_attribute *attr, const char *buff, size_t count)
{
	return -EPERM;
}

ssize_t nrf24l01_sysfs_status_show(struct device *dev,
		struct device_attribute *attr, char *buff)
{
	struct priv_data *priv = dev_get_drvdata(dev);

	return sprintf(buff, "0x%02x\n", priv->spi_ops.reg_map.status.value);
}

ssize_t nrf24l01_sysfs_status_store(struct device *dev,
		struct device_attribute *attr, const char *buff, size_t count)
{
	return -EPERM;
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

	len += sprintf(buff + len, "config: 0x%02x\n", reg->config.value);
	len += sprintf(buff + len, "en_aa: 0x%02x\n", reg->en_aa.value);
	len += sprintf(buff + len,
			"en_rxaddr: 0x%02x\n", reg->en_rxaddr.value);
	len += sprintf(buff + len, "setup_aw: 0x%02x\n", reg->setup_aw.value);
	len += sprintf(buff + len,
			"setup_retr: 0x%02x\n", reg->setup_retr.value);
	len += sprintf(buff + len, "rf_ch: 0x%02x\n", reg->rf_ch.value);
	len += sprintf(buff + len, "rf_setup: 0x%02x\n", reg->rf_setup.value);
	len += sprintf(buff + len, "status: 0x%02x\n", reg->status.value);
	len += sprintf(buff + len,
			"observe_tx: 0x%02x\n", reg->observe_tx.value);
	len += sprintf(buff + len, "rpd: 0x%02x\n", reg->rpd.value);
	len += sprintf(buff + len,
			"rx_addr_p0: 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n",
			reg->rx_addr_p0.value[0], reg->rx_addr_p0.value[1],
			reg->rx_addr_p0.value[2], reg->rx_addr_p0.value[3],
			reg->rx_addr_p0.value[4]);
	len += sprintf(buff + len,
			"rx_addr_p1: 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n",
			reg->rx_addr_p1.value[0], reg->rx_addr_p1.value[1],
			reg->rx_addr_p1.value[2], reg->rx_addr_p1.value[3],
			reg->rx_addr_p1.value[4]);
	len += sprintf(buff + len,
			"rx_addr_p2: 0x%02x\n", reg->rx_addr_p2.value);
	len += sprintf(buff + len,
			"rx_addr_p3: 0x%02x\n", reg->rx_addr_p3.value);
	len += sprintf(buff + len,
			"rx_addr_p4: 0x%02x\n", reg->rx_addr_p4.value);
	len += sprintf(buff + len,
			"rx_addr_p5: 0x%02x\n", reg->rx_addr_p5.value);
	len += sprintf(buff + len,
			"tx_addr: 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n",
			reg->tx_addr.value[0], reg->tx_addr.value[1],
			reg->tx_addr.value[2], reg->tx_addr.value[3],
			reg->tx_addr.value[4]);
	len += sprintf(buff + len, "rx_pw_p0: 0x%02x\n", reg->rx_pw_p0.value);
	len += sprintf(buff + len, "rx_pw_p1: 0x%02x\n", reg->rx_pw_p1.value);
	len += sprintf(buff + len, "rx_pw_p2: 0x%02x\n", reg->rx_pw_p2.value);
	len += sprintf(buff + len, "rx_pw_p3: 0x%02x\n", reg->rx_pw_p3.value);
	len += sprintf(buff + len, "rx_pw_p4: 0x%02x\n", reg->rx_pw_p4.value);
	len += sprintf(buff + len, "rx_pw_p5: 0x%02x\n", reg->rx_pw_p5.value);
	len += sprintf(buff + len,
			"fifo_status: 0x%02x\n", reg->fifo_status.value);
	len += sprintf(buff + len, "dynpd: 0x%02x\n", reg->dynpd.value);
	len += sprintf(buff + len, "feature: 0x%02x\n", reg->feature.value);

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


