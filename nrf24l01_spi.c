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

#define NRF24L01_SPI_CS_DELAY_US	5

#define NRF24L01_REG_WRITE_MASK	(0x20)

/* nRF24L01 registers */
#define NRF24L01_REG_NUM		26

#define NRF24L01_REG_CONFIG		0x00
#define NRF24L01_REG_EN_AA		0x01
#define NRF24L01_REG_EN_RXADDR		0x02
#define NRF24L01_REG_SETUP_AW		0x03
#define NRF24L01_REG_SETUP_RETR		0x04
#define NRF24L01_REG_RF_CH		0x05
#define NRF24L01_REG_RF_SETUP		0x06
#define NRF24L01_REG_STATUS		0x07
#define NRF24L01_REG_OBSERVE_TX		0x08
#define NRF24L01_REG_RPD		0x09
#define NRF24L01_REG_RX_ADDR_P0		0x0A
#define NRF24L01_REG_RX_ADDR_P1		0x0B
#define NRF24L01_REG_RX_ADDR_P2		0x0C
#define NRF24L01_REG_RX_ADDR_P3		0x0D
#define NRF24L01_REG_RX_ADDR_P4		0x0E
#define NRF24L01_REG_RX_ADDR_P5		0x0F
#define NRF24L01_REG_TX_ADDR		0x10
#define NRF24L01_REG_RX_PW_P0		0x11
#define NRF24L01_REG_RX_PW_P1		0x12
#define NRF24L01_REG_RX_PW_P2		0x13
#define NRF24L01_REG_RX_PW_P3		0x14
#define NRF24L01_REG_RX_PW_P4		0x15
#define NRF24L01_REG_RX_PW_P5		0x16
#define NRF24L01_REG_FIFO_STAT		0x17

#define NRF24L01_REG_DYNPD		0x1C
#define NRF24L01_REG_FEATURE		0x1D

/* selftest value which is written to config register */
#define NRF24L01_SELFTEST_CONFIG	0x7D

/* max size for address is 5 bytes */
#define NRF24L01_MAX_ADDR_VALUE		0xFFFFFFFFFFULL

#define NRF24L01_WRITE_TX_PAYLOAD	0xA0

/* Clear RX_DR, TX_DS and MAX_RT IRQs from status reg */
#define NRF24L01_CLEAR_IRQS		0x70

#define NRF24L01_USLEEP_MIN		20
#define NRF24L01_USLEEP_MAX		40
#define NRF24L01_SPI_USLEEP()		\
	do {				\
		usleep_range(20, 40);	\
	} while (0)

static irqreturn_t nrf24l01_gpio_irq(int irq, void *driver_data)
{
	struct priv_data *priv = driver_data;

	priv->spi_ops.busy = 0;

	wake_up_interruptible(&priv->spi_ops.waitq);

	return IRQ_HANDLED;
}

static int nrf24l01_spi_clear_irqs(struct spi_device *spi)
{
	int res = -EPERM;
	struct priv_data *priv = spi_get_drvdata(spi);
	uint8_t tx_buf[2] = {
		NRF24L01_REG_WRITE_MASK | NRF24L01_REG_STATUS,	/* cmd */
		NRF24L01_CLEAR_IRQS				/* value */
	};

	res = spi_write(spi, tx_buf, 2);
	if (res)
		PERR(priv->dev, "SPI error %d\n", res);

	return res;
}

int nrf24l01_spi_send_payload(struct spi_device *spi)
{
	int res = -EPERM;
	struct priv_data *priv = spi_get_drvdata(spi);
	struct nrf24l01_rxtx_payload *payload =
		&priv->spi_ops.payload;
	struct spi_transfer spi_xfer = {
		.cs_change = 1,
		.delay_usecs = NRF24L01_SPI_CS_DELAY_US
	};

	/* IRQ bit might be set if previous sending was interrupted */
	res = nrf24l01_spi_clear_irqs(spi);
	if (res)
		return res;

	priv->spi_ops.busy = 1;

	payload->cmd = NRF24L01_WRITE_TX_PAYLOAD;
	spi_xfer.tx_buf = &payload->cmd;
	spi_xfer.rx_buf = &payload->rx_status;
	spi_xfer.len = 1 + payload->data_len;

	res = spi_sync_transfer(spi, &spi_xfer, 1);
	if (res) {
		PERR(priv->dev, "SPI error %d\n", res);
		return res;
	}

	gpiod_set_value(priv->spi_ops.gpio_ce_rxtx, 1);
	NRF24L01_SPI_USLEEP();
	gpiod_set_value(priv->spi_ops.gpio_ce_rxtx, 0);

	res = wait_event_interruptible(priv->spi_ops.waitq, !priv->spi_ops.busy);
	if (res)
		PERR(priv->dev, "wait interrupted %d\n", res);

	return res;
}

int nrf24l01_spi_write_register(struct spi_device *spi, unsigned int reg,
		unsigned long long val)
{
	int res = -EPERM;
	struct priv_data *priv = spi_get_drvdata(spi);
	struct nrf24l01_rxtx_payload *buff = &priv->spi_ops.payload;
	struct spi_transfer spi_xfer = {
		.tx_buf = &buff->cmd,
		.rx_buf = &buff->rx_status,
		.delay_usecs = NRF24L01_SPI_CS_DELAY_US
	};

	PINFO(priv->dev, "\n");

	if ((reg > NRF24L01_REG_FIFO_STAT && reg < NRF24L01_REG_DYNPD) ||
		reg > NRF24L01_REG_FEATURE) {
		PERR(priv->dev, "invalid register\n");
		return -EINVAL;
	}

	/* over 1 byte value can be written only to 3 address registers */
	if (val > 0xFF &&
		(reg != NRF24L01_REG_RX_ADDR_P0 &&
		 reg != NRF24L01_REG_RX_ADDR_P1 &&
		 reg != NRF24L01_REG_TX_ADDR)) {
		PERR(priv->dev, "value out of range\n");
		return -EINVAL;
	}

	if (val > NRF24L01_MAX_ADDR_VALUE) {
		PERR(priv->dev, "invalid value\n");
		return -EINVAL;
	}

	buff->cmd = NRF24L01_REG_WRITE_MASK | reg;
	if (reg == NRF24L01_REG_RX_ADDR_P0 ||
		reg == NRF24L01_REG_RX_ADDR_P1 ||
		reg == NRF24L01_REG_TX_ADDR) {
		memcpy(buff->tx, &val, NRF24L01_MAX_ADDR_LEN);
		spi_xfer.len = 1 + NRF24L01_MAX_ADDR_LEN;
	} else {
		buff->tx[0] = (uint8_t)val;
		spi_xfer.len = 2;
	}

	res = spi_sync_transfer(spi, &spi_xfer, 1);
	if (res)
		PERR(priv->dev, "SPI error %d\n", res);

	return res;
}

int nrf24l01_spi_read_reg_map(struct spi_device *spi)
{
	int res = -EPERM;
	struct priv_data *priv = spi_get_drvdata(spi);
	uint8_t *tx_buf = NULL;
	uint8_t *rx_buf = NULL;
	struct spi_transfer *spi_xfer = NULL;
	unsigned int xfer_idx = 0;
	unsigned int buf_idx = 0;

	PINFO(priv->dev, "\n");

	spi_xfer = devm_kzalloc(priv->dev,
		NRF24L01_REG_NUM * sizeof(struct spi_transfer), GFP_KERNEL);
	if (!spi_xfer) {
		PERR(priv->dev, "out of memory\n");
		res = -ENOMEM;
		goto out;
	}

	/* Private-data register map can be directly used
	 * as a rx-buffer.
	 */
	rx_buf = (uint8_t *)&priv->spi_ops.reg_map;

	tx_buf = devm_kzalloc(priv->dev,
		sizeof(struct nrf24l01_registers), GFP_KERNEL);
	if (!tx_buf) {
		PERR(priv->dev, "out of memory\n");
		res = -ENOMEM;
		goto out;
	}

	/* Chip registers address starts from 0x0 and is incremented
	 * by 1 byte for each subsequent register.
	 *
	 * Register reading starts by writing register address to the chip
	 */
	for (xfer_idx = 0; xfer_idx < NRF24L01_REG_NUM; xfer_idx++) {
		/* Set register address. Register map has a hole
		 * before 2 last registers
		 */
		switch (xfer_idx) {
		case (NRF24L01_REG_FIFO_STAT + 1):
			tx_buf[buf_idx] = NRF24L01_REG_DYNPD;
			break;
		case (NRF24L01_REG_FIFO_STAT + 2):
			tx_buf[buf_idx] = NRF24L01_REG_FEATURE;
			break;

		default:
			tx_buf[buf_idx] = xfer_idx;
		}

		spi_xfer[xfer_idx].tx_buf = &tx_buf[buf_idx];
		spi_xfer[xfer_idx].rx_buf = &rx_buf[buf_idx];
		spi_xfer[xfer_idx].cs_change = 1;
		spi_xfer[xfer_idx].delay_usecs = NRF24L01_SPI_CS_DELAY_US;

		spi_xfer[xfer_idx].len = 2;
		buf_idx += 2;

		/* 3 address register have size of 5 bytes */
		if (xfer_idx == NRF24L01_REG_RX_ADDR_P0 ||
			xfer_idx == NRF24L01_REG_RX_ADDR_P1 ||
			xfer_idx == NRF24L01_REG_TX_ADDR) {

			spi_xfer[xfer_idx].len = 6;
			buf_idx += 4;
		}
	}

	res = spi_sync_transfer(spi, spi_xfer, xfer_idx);
	if (res)
		PERR(priv->dev, "SPI error %d\n", res);

out:
	devm_kfree(priv->dev, spi_xfer);
	devm_kfree(priv->dev, tx_buf);

	return res;
}

static int nrf24l01_spi_selftest(struct spi_device *spi)
{
	int res = -EPERM;
	struct priv_data *priv = spi_get_drvdata(spi);
	uint8_t tx_buf[2] = { 0 };
	struct nrf24l01_reg_read rx_buf = { 0 };
	uint8_t curr_config = 0;

	struct spi_transfer spi_xfer = {
		.tx_buf = tx_buf,
		.rx_buf = &rx_buf,
		.len = 2,
		.delay_usecs = NRF24L01_SPI_CS_DELAY_US
	};

	/* Save current config reg value */
	tx_buf[0] = NRF24L01_REG_CONFIG;

	res = spi_sync_transfer(spi, &spi_xfer, 1);
	if (res) {
		PERR(priv->dev, "SPI error %d\n", res);
		return res;
	}

	curr_config = rx_buf.value;

	/* Write selftest config */
	tx_buf[0] = NRF24L01_REG_WRITE_MASK | NRF24L01_REG_CONFIG;
	tx_buf[1] = NRF24L01_SELFTEST_CONFIG;

	res = spi_sync_transfer(spi, &spi_xfer, 1);
	if (res) {
		PERR(priv->dev, "SPI error %d\n", res);
		return res;
	}

	/* And read it back to verify SPI communication */
	tx_buf[0] = NRF24L01_REG_CONFIG;
	rx_buf.value = 0;

	res = spi_sync_transfer(spi, &spi_xfer, 1);
	if (res) {
		PERR(priv->dev, "SPI error %d\n", res);
		return res;
	}

	/* SPI-communication fails. Written and read values do not match */
	if (rx_buf.value != NRF24L01_SELFTEST_CONFIG) {
		PERR(priv->dev, "Selftest failed: value 0x%x\n",
			rx_buf.value);
		return -EIO;
	}

	/* And finally restore original config value */
	tx_buf[0] = NRF24L01_REG_WRITE_MASK | NRF24L01_REG_CONFIG;
	tx_buf[1] = curr_config;

	res = spi_sync_transfer(spi, &spi_xfer, 1);
	if (res) {
		PERR(priv->dev, "SPI error %d\n", res);
		return res;
	}

	res = spi_sync_transfer(spi, &spi_xfer, 1);
	if (res) {
		PERR(priv->dev, "SPI error %d\n", res);
		return res;
	}

	/* Store latest config and status register values */
	priv->spi_ops.reg_map.config.value = curr_config;
	priv->spi_ops.reg_map.status.value = rx_buf.status;

	return res;
}

int nrf24l01_spi_setup(struct spi_device *spi)
{
	int res = -EPERM;
	struct priv_data *priv = spi_get_drvdata(spi);

	PINFO(priv->dev, "\n");

	priv->spi_ops.gpio_ce_rxtx = devm_gpiod_get(&spi->dev, "ce", GPIOD_OUT_LOW);
	if (IS_ERR(priv->spi_ops.gpio_ce_rxtx)) {
		res = PTR_ERR(priv->spi_ops.gpio_ce_rxtx);
		PERR(priv->dev, "gpio ce error %d\n", res);
		return res;
	}

	priv->spi_ops.gpio_irq = devm_gpiod_get(&spi->dev, "irq", GPIOD_ASIS);
	if (IS_ERR(priv->spi_ops.gpio_irq)) {
		res = PTR_ERR(priv->spi_ops.gpio_irq);
		PERR(priv->dev, "gpio irq error %d\n", res);
		return res;
	}

	res = gpiod_to_irq(priv->spi_ops.gpio_irq);
	if (res < 0) {
		PERR(priv->dev, "irq error %d\n", res);
		return res;
	}

	res = devm_request_irq(&spi->dev, res, nrf24l01_gpio_irq,
			IRQF_TRIGGER_FALLING, dev_name(&spi->dev), priv);
	if (res < 0) {
		PERR(priv->dev, "irq req error %d\n", res);
		return res;
	}

	res = nrf24l01_spi_selftest(spi);
	if (res)
		return res;

	res = nrf24l01_spi_clear_irqs(spi);

	return res;
}
