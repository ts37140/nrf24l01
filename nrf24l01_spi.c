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

static int nrf24l01_spi_selftest(struct spi_device *spi)
{
	int res = -EPERM;
	struct priv_data *priv = spi_get_drvdata(spi);
	struct spi_message m;
	uint8_t tx_buf[2] = { 0 };
	struct nrf24l01_reg_read rx_buf = { 0 };
	uint8_t curr_config = 0;

	struct spi_transfer spi_xfer = {
		.tx_buf = tx_buf,
		.rx_buf = &rx_buf,
		.len = 2,
		.cs_change = 1,
		.delay_usecs = NRF24L01_SPI_CS_DELAY_US
	};

	/* Save current config reg value */
	tx_buf[0] = NRF24L01_REG_CONFIG;

	spi_message_init_with_transfers(&m, &spi_xfer, 1);

	spi_bus_lock(priv->spi_dev->master);

	res = spi_sync_locked(spi, &m);
	if (res) {
		PERR(priv->dev, "SPI error %d\n", res);
		goto out;
	}

	curr_config = rx_buf.value;

	/* Write selftest config */
	tx_buf[0] = NRF24L01_REG_WRITE_MASK | NRF24L01_REG_CONFIG;
	tx_buf[1] = NRF24L01_SELFTEST_CONFIG;

	spi_message_init_with_transfers(&m, &spi_xfer, 1);
	res = spi_sync_locked(spi, &m);
	if (res) {
		PERR(priv->dev, "SPI error %d\n", res);
		goto out;
	}

	/* And read it back to verify SPI communication */
	tx_buf[0] = NRF24L01_REG_CONFIG;
	rx_buf.value = 0;

	spi_message_init_with_transfers(&m, &spi_xfer, 1);
	res = spi_sync_locked(spi, &m);
	if (res) {
		PERR(priv->dev, "SPI error %d\n", res);
		goto out;
	}

	/* SPI-communication fails. Written and read values do not match */
	if (rx_buf.value != NRF24L01_SELFTEST_CONFIG) {
		PERR(priv->dev, "Selftest failed: value 0x%x\n",
			rx_buf.value);
		res = -EIO;
		goto out;
	}

	/* And finally restore original config value */
	tx_buf[0] = NRF24L01_REG_WRITE_MASK | NRF24L01_REG_CONFIG;
	tx_buf[1] = curr_config;

	spi_message_init_with_transfers(&m, &spi_xfer, 1);
	res = spi_sync_locked(spi, &m);
	if (res) {
		PERR(priv->dev, "SPI error %d\n", res);
		goto out;
	}

	spi_message_init_with_transfers(&m, &spi_xfer, 1);
	res = spi_sync_locked(spi, &m);
	if (res) {
		PERR(priv->dev, "SPI error %d\n", res);
		goto out;
	}

	/* Store latest config and status register values */
	priv->spi_ops.reg_map.config.value = curr_config;
	priv->spi_ops.reg_map.status.value = rx_buf.status;

out:
	spi_bus_unlock(priv->spi_dev->master);

	return res;
}

int nrf24l01_spi_setup(struct spi_device *spi)
{
	int res = 0;
	struct priv_data *priv = spi_get_drvdata(spi);

	PINFO(priv->dev, "\n");

	res = nrf24l01_spi_selftest(spi);

	return res;
}
