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
#include <linux/device.h>
#include <linux/compiler-gcc.h>

#define NRF24L01_DRV_NAME "nrf24l01"
#define NRF24L01_NODE_NAME "nrf24l01"

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("TERO SALMINEN");

static struct device_attribute nrf24l01_attr[] = {
	{
		.attr = {
			.name = "set_reg",
			.mode = 0220
		},
		.store	= nrf24l01_sysfs_reg_store,
	},
	{
		.attr = {
			.name = "reg_map",
			.mode = 0444
		},
		.show	= nrf24l01_sysfs_reg_map_show,
	},
	{
		.attr = {
			.name = "info",
			.mode = 0444
		},
		.show	= nrf24l01_sysfs_info_show,
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

	init_waitqueue_head(&priv->spi_ops.waitq);

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

