/**
 *
 * \file
 *
 * \brief This module contains NMC1000 bus wrapper APIs implementation.
 *
 * Copyright (c) 2016-2022 Microchip Technology Inc. and its subsidiaries.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Subject to your compliance with these terms, you may use Microchip
 * software and any derivatives exclusively with Microchip products.
 * It is your responsibility to comply with third party license terms applicable
 * to your use of third party software (including open source software) that
 * may accompany Microchip software.
 *
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE,
 * INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY,
 * AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT WILL MICROCHIP BE
 * LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR CONSEQUENTIAL
 * LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE
 * SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE
 * POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT
 * ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY
 * RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
 * THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * \asf_license_stop
 *
 */

#include <stdio.h>
#include "bsp/include/nm_bsp.h"
#include "common/include/nm_common.h"
#include "bus_wrapper/include/nm_bus_wrapper.h"
#include "asf.h"
#include "conf_winc.h"

#define NM_BUS_MAX_TRX_SZ 4096

tstrNmBusCapabilities egstrNmBusCapabilities =
{
	NM_BUS_MAX_TRX_SZ
};

#ifdef CONF_WINC_USE_I2C
#define SLAVE_ADDRESS 0x60

/** Number of times to try to send packet if failed. */
#define I2C_TIMEOUT 100

static sint8 nm_i2c_write(uint8 *b, uint16 sz)
{
	sint8 result = M2M_SUCCESS;
	twihs_packet_t packet_tx;
	uint16_t timeout = 0;

	/* Configure the data packet to be transmitted */
	packet_tx.chip        = SLAVE_ADDRESS;
	packet_tx.addr[0]     = 0;
	packet_tx.addr[1]     = 0;
	packet_tx.addr[2]     = 0;
	packet_tx.addr_length = 0;
	packet_tx.buffer      = b;
	packet_tx.length      = sz;

	while(twihs_master_write(CONF_WINC_I2C, &packet_tx) != TWIHS_SUCCESS) {
		if (timeout++ == I2C_TIMEOUT) {
			break;
		}
	}
	return result;
}

static sint8 nm_i2c_read(uint8 *rb, uint16 sz)
{
	sint8 result = M2M_SUCCESS;
	twihs_packet_t packet_rx;
	uint16_t timeout = 0;

	/* Configure the data packet to be received */
	packet_rx.chip        = SLAVE_ADDRESS;
	packet_rx.addr[0]     = 0;
	packet_rx.addr[1]     = 0;
	packet_rx.addr[2]     = 0;
	packet_rx.addr_length = 0;
	packet_rx.buffer      = rb;
	packet_rx.length      = sz;

	while (twihs_master_read(CONF_WINC_I2C, &packet_rx) != TWIHS_SUCCESS) {
		if (timeout++ == I2C_TIMEOUT) {
			break;
		}
	}
	return result;
}

static sint8 nm_i2c_write_special(uint8 *wb1, uint16 sz1, uint8 *wb2, uint16 sz2)
{
	static uint8 tmp[NM_BUS_MAX_TRX_SZ];
	m2m_memcpy(tmp, wb1, sz1);
	m2m_memcpy(&tmp[sz1], wb2, sz2);
	return nm_i2c_write(tmp, sz1+sz2);
}
#endif

#ifdef CONF_WINC_USE_SPI
/** PIO instance used by CS. */
Pio *p_pio_cs;

/** Fast CS macro. */
#define SPI_ASSERT_CS()		do {p_pio_cs->PIO_CODR = 1 << (CONF_WINC_SPI_CS_GPIO & 0x1F);} while(0)
#define SPI_DEASSERT_CS()	do {p_pio_cs->PIO_SODR = 1 << (CONF_WINC_SPI_CS_GPIO & 0x1F);} while(0)

sint8 nm_spi_rw(uint8 *pu8Mosi, uint8 *pu8Miso, uint16 u16Sz)
{
	uint8 u8Dummy = 0;
	uint8 u8SkipMosi = 0, u8SkipMiso = 0;
	uint16_t txd_data = 0;
	uint16_t rxd_data = 0;
	uint8_t uc_pcs;

	if (!pu8Mosi) {
		pu8Mosi = &u8Dummy;
		u8SkipMosi = 1;
	}
	else if(!pu8Miso) {
		pu8Miso = &u8Dummy;
		u8SkipMiso = 1;
	}
	else {
		return M2M_ERR_BUS_FAIL;
	}

	SPI_ASSERT_CS();
	while (u16Sz) {
		txd_data = *pu8Mosi;
		spi_write(CONF_WINC_SPI, txd_data, 0, 0);

		/* Read SPI master data register. */
		spi_read(CONF_WINC_SPI, &rxd_data, &uc_pcs);
		*pu8Miso = rxd_data;

		u16Sz--;
		if (!u8SkipMiso)
			pu8Miso++;
		if (!u8SkipMosi)
			pu8Mosi++;
	}
	SPI_DEASSERT_CS();

	return M2M_SUCCESS;
}
#endif

/*
 *	@fn		nm_bus_init
 *	@brief	Initialize the bus wrapper
 *	@return	M2M_SUCCESS in case of success and M2M_ERR_BUS_FAIL in case of failure
 */
sint8 nm_bus_init(void *pvinit)
{
	sint8 result = M2M_SUCCESS;
#ifdef CONF_WINC_USE_I2C
	twihs_options_t opt;

	/* Enable the peripheral clock for TWI */
	pmc_enable_periph_clk(CONF_WINC_I2C_ID);

	/* Configure the options of TWI driver */
	opt.master_clk = sysclk_get_peripheral_hz();
	opt.speed      = CONF_WINC_TWIHS_CLOCK;

	if (twihs_master_init(CONF_WINC_I2C, &opt) != TWIHS_SUCCESS) {
		M2M_ERR("-E-\tTWI master initialization failed.\r");
		while (1) {
			/* Capture error */
		}
	}

#elif CONF_WINC_USE_SPI
	/* Configure SPI pins. */
	ioport_set_pin_mode(CONF_WINC_SPI_MISO_GPIO, CONF_WINC_SPI_MISO_FLAGS);
	ioport_set_pin_mode(CONF_WINC_SPI_MOSI_GPIO, CONF_WINC_SPI_MOSI_FLAGS);
	ioport_set_pin_mode(CONF_WINC_SPI_CLK_GPIO, CONF_WINC_SPI_CLK_FLAGS);
	ioport_set_pin_mode(CONF_WINC_SPI_CS_GPIO, CONF_WINC_SPI_CS_FLAGS);
	ioport_disable_pin(CONF_WINC_SPI_MISO_GPIO);
	ioport_disable_pin(CONF_WINC_SPI_MOSI_GPIO);
	ioport_disable_pin(CONF_WINC_SPI_CLK_GPIO);

	/* Configure CS for manual (GPIO) control. */
	p_pio_cs = (Pio *)((uint32_t)PIOA + (PIO_DELTA * (CONF_WINC_SPI_CS_GPIO >> 5)));
	SPI_DEASSERT_CS();
	ioport_set_pin_dir(CONF_WINC_SPI_CS_GPIO, IOPORT_DIR_OUTPUT);
	ioport_enable_pin(CONF_WINC_SPI_CS_GPIO);

	spi_enable_clock(CONF_WINC_SPI);
	spi_disable(CONF_WINC_SPI);
	spi_reset(CONF_WINC_SPI);
	spi_set_master_mode(CONF_WINC_SPI);
	spi_disable_mode_fault_detect(CONF_WINC_SPI);
	spi_set_peripheral_chip_select_value(CONF_WINC_SPI, CONF_WINC_SPI_NPCS);
	spi_set_clock_polarity(CONF_WINC_SPI,
			CONF_WINC_SPI_NPCS, CONF_WINC_SPI_POL);
	spi_set_clock_phase(CONF_WINC_SPI, CONF_WINC_SPI_NPCS, CONF_WINC_SPI_PHA);
	spi_set_bits_per_transfer(CONF_WINC_SPI, CONF_WINC_SPI_NPCS, SPI_CSR_BITS_8_BIT);
	spi_set_baudrate_div(CONF_WINC_SPI, CONF_WINC_SPI_NPCS,
			(sysclk_get_peripheral_hz() / CONF_WINC_SPI_CLOCK));
	spi_set_transfer_delay(CONF_WINC_SPI, CONF_WINC_SPI_NPCS, CONF_WINC_SPI_DLYBS,
			CONF_WINC_SPI_DLYBCT);
	spi_enable(CONF_WINC_SPI);

	nm_bsp_reset();
	nm_bsp_sleep(1);
#endif
	return result;
}

/*
 *	@fn		nm_bus_ioctl
 *	@brief	send/receive from the bus
 *	@param[IN]	u8Cmd
 *					IOCTL command for the operation
 *	@param[IN]	pvParameter
 *					Arbitrary parameter depenging on IOCTL
 *	@return	M2M_SUCCESS in case of success and M2M_ERR_BUS_FAIL in case of failure
 *	@note	For SPI only, it's important to be able to send/receive at the same time
 */
sint8 nm_bus_ioctl(uint8 u8Cmd, void* pvParameter)
{
	sint8 s8Ret = 0;
	switch(u8Cmd)
	{
#ifdef CONF_WINC_USE_I2C
		case NM_BUS_IOCTL_R: {
			tstrNmI2cDefault *pstrParam = (tstrNmI2cDefault *)pvParameter;
			s8Ret = nm_i2c_read(pstrParam->pu8Buf, pstrParam->u16Sz);
		}
		break;
		case NM_BUS_IOCTL_W: {
			tstrNmI2cDefault *pstrParam = (tstrNmI2cDefault *)pvParameter;
			s8Ret = nm_i2c_write(pstrParam->pu8Buf, pstrParam->u16Sz);
		}
		break;
		case NM_BUS_IOCTL_W_SPECIAL: {
			tstrNmI2cSpecial *pstrParam = (tstrNmI2cSpecial *)pvParameter;
			s8Ret = nm_i2c_write_special(pstrParam->pu8Buf1, pstrParam->u16Sz1, pstrParam->pu8Buf2, pstrParam->u16Sz2);
		}
		break;
#elif CONF_WINC_USE_SPI
		case NM_BUS_IOCTL_RW: {
			tstrNmSpiRw *pstrParam = (tstrNmSpiRw *)pvParameter;
			s8Ret = nm_spi_rw(pstrParam->pu8InBuf, pstrParam->pu8OutBuf, pstrParam->u16Sz);
		}
		break;
#endif
		default:
			s8Ret = -1;
			M2M_ERR("Invalid IOCTL command!\n");
			break;
	}

	return s8Ret;
}

/*
 *	@fn		nm_bus_deinit
 *	@brief	De-initialize the bus wrapper
 */
sint8 nm_bus_deinit(void)
{
	sint8 result = M2M_SUCCESS;

#ifdef CONF_WINC_USE_I2C
	//TODO:
#endif /* CONF_WINC_USE_I2C */
#ifdef CONF_WINC_USE_SPI
	spi_disable(CONF_WINC_SPI);
	ioport_set_pin_dir(CONF_WINC_SPI_MOSI_GPIO, IOPORT_DIR_INPUT);
	ioport_set_pin_dir(CONF_WINC_SPI_MISO_GPIO, IOPORT_DIR_INPUT);
	ioport_set_pin_dir(CONF_WINC_SPI_CLK_GPIO, IOPORT_DIR_INPUT);
	ioport_set_pin_dir(CONF_WINC_SPI_CS_GPIO, IOPORT_DIR_INPUT);
#endif /* CONF_WINC_USE_SPI */
	return result;
}


/**
 *  @fn         nm_bus_speed
 *  @brief      Either set the bus speed to default (HIGH) or a reduced speed (LOW)
				to increase stability during WINC wakeup
 *  @param [in] uint8 level
 *                  HIGH(1) or LOW(0)
 *  @return     M2M_SUCCESS in case of success and M2M_ERR_INVALID_ARG in case of an
				incorrect parameter
 */
sint8 nm_bus_speed(uint8 level)
{
	/* No clock adjustment is necessary for SAM4S */
	if ((level == HIGH) || (level == LOW))
		return M2M_SUCCESS;

	return M2M_ERR_INVALID_ARG;
}
