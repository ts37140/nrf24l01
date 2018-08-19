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


