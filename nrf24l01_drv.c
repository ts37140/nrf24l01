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
#include <linux/device.h>
#include <linux/compiler-gcc.h>

#define NRF24L01_DRV_NAME "nrf24l01"
#define NRF24L01_NODE_NAME "nrf24l01"

MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR("TERO SALMINEN");

static struct device_attribute nrf24l01_attr[] = {
	{
		.attr = {
			.name = "config",
			.mode = 0664
		},
		.show	= nrf24l01_sysfs_config_show,
		.store	= nrf24l01_sysfs_config_store,
	},
	{
		.attr = {
			.name = "status",
			.mode = 0664
		},
		.show	= nrf24l01_sysfs_status_show,
		.store	= nrf24l01_sysfs_status_store,
	},
	{
		.attr = {
			.name = "reg_map",
			.mode = 0444
		},
		.show	= nrf24l01_sysfs_reg_map_show,
	},
};

static struct attribute *nrf24l01_sysfs_entries[] = {
	&nrf24l01_attr[0].attr,
	&nrf24l01_attr[1].attr,
	&nrf24l01_attr[2].attr,
	NULL
};

static const struct attribute_group nrf24l01_attribute_group = {
	.name = NULL,
	.attrs = nrf24l01_sysfs_entries,
};

static const struct attribute_group *nrf24l01_attribute_groups[] = {
	&nrf24l01_attribute_group,
	NULL
};

static const struct file_operations nrf24l01_fops = {
	.owner		= THIS_MODULE,
	.open		= nrf24l01_open,
	.release	= nrf24l01_release,
	.read		= nrf24l01_read,
	.write		= nrf24l01_write,
};

static int nrf24l01_probe(struct spi_device *spi)
{
	int res = -EPERM;
	struct priv_data *priv = NULL;

	priv = devm_kzalloc(&spi->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		PERR(&spi->dev, "out of memory\n");
		return -ENOMEM;
	}

	priv->spi_dev = spi;
	spi_set_drvdata(spi, priv);

	priv->nrf24l01_class = class_create(THIS_MODULE, NRF24L01_DRV_NAME);
	if (!priv->nrf24l01_class) {
		PERR(&spi->dev, "class creation failed\n");
		return -EPERM;
	}

	res = alloc_chrdev_region(&priv->dev_num, 0, 1, NRF24L01_DRV_NAME);
	if (res) {
		PERR(&spi->dev, "register device no failed %d\n", res);
		goto out_alloc_chrdev_error;
	}

	cdev_init(&priv->cdev, &nrf24l01_fops);
	res = cdev_add(&priv->cdev, priv->dev_num, 1);
	if (res) {
		PERR(&spi->dev, "adding character device failed %d\n", res);
		goto out_cdev_error;
	}

	priv->dev = device_create_with_groups(
			priv->nrf24l01_class,
			NULL,
			priv->dev_num,
			priv,
			nrf24l01_attribute_groups,
			NRF24L01_NODE_NAME);

	if (!priv->dev) {
		PERR(&spi->dev, "device creation failed\n");

		res = -EPERM;
		goto out_device_create_error;
	}

	PINFO(priv->dev, "\n");

	res = nrf24l01_spi_setup(spi);
	if (res)
		goto out_spi_error;

	return res;

out_spi_error:
	device_destroy(priv->nrf24l01_class, priv->cdev.dev);

out_device_create_error:
	cdev_del(&priv->cdev);

out_cdev_error:
	unregister_chrdev_region(priv->dev_num, 1);

out_alloc_chrdev_error:
	class_destroy(priv->nrf24l01_class);

	return res;
}

static int nrf24l01_remove(struct spi_device *spi)
{
	struct priv_data *priv = spi_get_drvdata(spi);

	PINFO(priv->dev, "\n");

	device_destroy(priv->nrf24l01_class, priv->cdev.dev);

	cdev_del(&priv->cdev);

	unregister_chrdev_region(priv->dev_num, 1);

	class_destroy(priv->nrf24l01_class);

	return 0;
}

static const struct of_device_id nrf24l01_dt_ids[] = {
	{ .compatible = "nordic,nrf24l01", },
	{ }
};
MODULE_DEVICE_TABLE(of, nrf24l01_dt_ids);

static const struct spi_device_id nrf24l01_spi_driver_ids[] = {
	{ "nrf24l01", 0 },
	{ }
};

static struct spi_driver nrf24_spi_driver = {
	.driver = {
		.name	= NRF24L01_DRV_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = nrf24l01_dt_ids,
	},
	.probe		= nrf24l01_probe,
	.remove		= nrf24l01_remove,
	.id_table	= nrf24l01_spi_driver_ids,
};

module_spi_driver(nrf24_spi_driver);

