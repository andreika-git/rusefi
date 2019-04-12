/*
    ChibiOS - Copyright (C) 2014-2015 Fabio Utzig

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/**
 * @file    SPIv1/hal_spi_lld.c
 * @brief   KINETIS SPI subsystem low level driver source.
 *
 * @addtogroup SPI
 * @{
 */

#include "hal.h"

//_spi_isr_code

#if HAL_USE_SPI || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/** @brief SPI0 driver identifier.*/
#if KINETIS_SPI_USE_SPI0 || defined(__DOXYGEN__)
SPIDriver SPID1;
#endif

/** @brief SPI1 driver identifier.*/
#if KINETIS_SPI_USE_SPI1 || defined(__DOXYGEN__)
SPIDriver SPID2;
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

static int32_t spi_detectPCS(ioportid_t ssport, uint16_t sspad) {
	// todo: check if PCS corresponds to SPI number
	if (ssport == GPIOA) {
		switch (sspad) {
		case 6: 
			return kLPSPI_MasterPcs1;
		case 15:
			return kLPSPI_MasterPcs3;
		case 16:
			return kLPSPI_MasterPcs2;
		}
	} else if (ssport == GPIOB) {
		switch (sspad) {
		case 0:
			return kLPSPI_MasterPcs0;
		case 5:
			return kLPSPI_MasterPcs1;
		case 17:
			return kLPSPI_MasterPcs3;
		}
	} else if (ssport == GPIOD && sspad == 3) {
		return kLPSPI_MasterPcs0;
	} else if (ssport == GPIOE && sspad == 6) {
		return kLPSPI_MasterPcs2;
	}
	// wrong/unrecognized PCS!
	return -1;
}

static int32_t spi_detectBaudRate(SPIDriver *spip) {
	static const int baudRates[] = { 21000000, 10500000, 5250000, 2626000, 1312500, 656250, 328125, 164060 };
	int flags = 0;
	if (spip->config->cr1 & SPI_CR1_BR_0)
		flags |= 1;
	if (spip->config->cr1 & SPI_CR1_BR_1)
		flags |= 2;
	if (spip->config->cr1 & SPI_CR1_BR_2)
		flags |= 4;
	int br = baudRates[flags];
	// SPI1 is faster on STM32 (42 MHz max) so we imitate this behavior
	if (spip == &SPID1)
		br *= 2;
	return br;
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

void spi_lld_callback(LPSPI_Type *base, lpspi_master_handle_t *handle,
                      status_t status, void *userData) {
	SPIDriver *spi = (SPIDriver *)userData;

	// todo: check status?
	_spi_isr_code(spi);
}

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level SPI driver initialization.
 *
 * @notapi
 */
void spi_lld_init(void) {
#if KINETIS_SPI_USE_SPI0
  spiObjectInit(&SPID1);
#endif
#if KINETIS_SPI_USE_SPI1
  spiObjectInit(&SPID2);
#endif
}

/**
 * @brief   Configures and activates the SPI peripheral.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 *
 * @notapi
 */
void spi_lld_start(SPIDriver *spip) {

  /* If in stopped state then enables the SPI and DMA clocks.*/
  if (spip->state == SPI_STOP) {
  	clock_ip_name_t clockName;
#if KINETIS_SPI_USE_SPI0
    if (&SPID1 == spip) {
      spip->spi = LPSPI0;
      clockName = kCLOCK_Lpspi0;
    }
#endif

#if KINETIS_SPI_USE_SPI1
    if (&SPID2 == spip) {
      spip->spi = LPSPI1;
      clockName = kCLOCK_Lpspi1;
    }
#endif

	int pcsIdx = spi_detectPCS(spip->config->ssport, spip->config->sspad);
	osalDbgAssert(pcsIdx >= 0, "invalid SPI PCS pin");
	spip->flags = pcsIdx;// | kLPSPI_MasterPcsContinuous | kLPSPI_MasterByteSwap;
	
	//CLOCK_SetIpSrc(clockName, kCLOCK_IpSrcSysPllAsync);

	if (1) {
		lpspi_master_config_t masterConfig;
		LPSPI_MasterGetDefaultConfig(&masterConfig);

	    masterConfig.baudRate = spi_detectBaudRate(spip);
	    masterConfig.bitsPerFrame = (spip->config->cr1 & SPI_CR1_DFF) ? 16 : 8;
	    masterConfig.cpol = (spip->config->cr1 & SPI_CR1_CPOL) ? kLPSPI_ClockPolarityActiveLow : kLPSPI_ClockPolarityActiveHigh;
	    masterConfig.cpha = (spip->config->cr1 & SPI_CR1_CPHA) ? kLPSPI_ClockPhaseSecondEdge : kLPSPI_ClockPhaseFirstEdge;
	    masterConfig.direction = kLPSPI_MsbFirst;

	    masterConfig.pcsToSckDelayInNanoSec = 1000000000 / masterConfig.baudRate * 2;
	    masterConfig.lastSckToPcsDelayInNanoSec = 1000000000 / masterConfig.baudRate * 2;
	    masterConfig.betweenTransferDelayInNanoSec = 1000000000 / masterConfig.baudRate * 2;
	    
	    masterConfig.whichPcs = pcsIdx;
	    
		uint32_t srcClock_Hz = CLOCK_GetIpFreq(clockName);
		LPSPI_MasterInit(spip->spi, &masterConfig, srcClock_Hz);

		LPSPI_MasterTransferCreateHandle(spip->spi, &spip->masterHandle, spi_lld_callback, spip);
	} else {
		// todo: add slave support
	}

	// todo: add DMA support?
    //nvicEnableVector(DMA0_IRQn, KINETIS_SPI0_RX_DMA_IRQ_PRIORITY);
  }
}

/**
 * @brief   Deactivates the SPI peripheral.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 *
 * @notapi
 */
void spi_lld_stop(SPIDriver *spip) {

  /* If in ready state then disables the SPI clock.*/
  if (spip->state == SPI_READY) {

    //nvicDisableVector(DMA0_IRQn);

#if KINETIS_SPI_USE_SPI0
    if (&SPID1 == spip) {
    	LPSPI_Deinit(LPSPI0);
    }
#endif

#if KINETIS_SPI_USE_SPI1
    if (&SPID2 == spip) {
    	LPSPI_Deinit(LPSPI1);
    }
#endif
  }
}

/**
 * @brief   Asserts the slave select signal and prepares for transfers.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 *
 * @notapi
 */
void spi_lld_select(SPIDriver *spip) {

	//palClearPad(spip->config->ssport, spip->config->sspad);
}

/**
 * @brief   Deasserts the slave select signal.
 * @details The previously selected peripheral is unselected.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 *
 * @notapi
 */
void spi_lld_unselect(SPIDriver *spip) {

	//palSetPad(spip->config->ssport, spip->config->sspad);
}

/**
 * @brief   Ignores data on the SPI bus.
 * @details This asynchronous function starts the transmission of a series of
 *          idle words on the SPI bus and ignores the received data.
 * @post    At the end of the operation the configured callback is invoked.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @param[in] n         number of words to be ignored
 *
 * @notapi
 */
void spi_lld_ignore(SPIDriver *spip, size_t n) {
/*
  spip->count = n;
  spip->rxbuf = NULL;
  spip->txbuf = NULL;

  spi_start_xfer(spip, false);
*/
}

/**
 * @brief   Exchanges data on the SPI bus.
 * @details This asynchronous function starts a simultaneous transmit/receive
 *          operation.
 * @post    At the end of the operation the configured callback is invoked.
 * @note    The buffers are organized as uint8_t arrays for data sizes below or
 *          equal to 8 bits else it is organized as uint16_t arrays.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @param[in] n         number of words to be exchanged
 * @param[in] txbuf     the pointer to the transmit buffer
 * @param[out] rxbuf    the pointer to the receive buffer
 *
 * @notapi
 */
void spi_lld_exchange(SPIDriver *spip, size_t n,
                      const void *txbuf, void *rxbuf) {
	spip->masterXfer.txData = (uint8_t *)txbuf;
	spip->masterXfer.rxData = rxbuf;
	spip->masterXfer.dataSize = n;
	spip->masterXfer.configFlags = spip->flags;

	LPSPI_MasterTransferNonBlocking(spip->spi, &spip->masterHandle, &spip->masterXfer);
}

/**
 * @brief   Sends data over the SPI bus.
 * @details This asynchronous function starts a transmit operation.
 * @post    At the end of the operation the configured callback is invoked.
 * @note    The buffers are organized as uint8_t arrays for data sizes below or
 *          equal to 8 bits else it is organized as uint16_t arrays.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @param[in] n         number of words to send
 * @param[in] txbuf     the pointer to the transmit buffer
 *
 * @notapi
 */
void spi_lld_send(SPIDriver *spip, size_t n, const void *txbuf) {

	spip->masterXfer.txData = (uint8_t *)txbuf;
	spip->masterXfer.rxData = NULL;
	spip->masterXfer.dataSize = n;
	spip->masterXfer.configFlags = spip->flags;

	LPSPI_MasterTransferNonBlocking(spip->spi, &spip->masterHandle, &spip->masterXfer);
}

/**
 * @brief   Receives data from the SPI bus.
 * @details This asynchronous function starts a receive operation.
 * @post    At the end of the operation the configured callback is invoked.
 * @note    The buffers are organized as uint8_t arrays for data sizes below or
 *          equal to 8 bits else it is organized as uint16_t arrays.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @param[in] n         number of words to receive
 * @param[out] rxbuf    the pointer to the receive buffer
 *
 * @notapi
 */
void spi_lld_receive(SPIDriver *spip, size_t n, void *rxbuf) {

	spip->masterXfer.txData = NULL;
	spip->masterXfer.rxData = (uint8_t *)rxbuf;
	spip->masterXfer.dataSize = n;
	spip->masterXfer.configFlags = spip->flags;

	LPSPI_MasterTransferNonBlocking(spip->spi, &spip->masterHandle, &spip->masterXfer);
}

/**
 * @brief   Exchanges one frame using a polled wait.
 * @details This synchronous function exchanges one frame using a polled
 *          synchronization method. This function is useful when exchanging
 *          small amount of data on high speed channels, usually in this
 *          situation is much more efficient just wait for completion using
 *          polling than suspending the thread waiting for an interrupt.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @param[in] frame     the data frame to send over the SPI bus
 * @return              The received data frame from the SPI bus.
 */
uint16_t spi_lld_polled_exchange(SPIDriver *spip, uint16_t frame) {
	// todo: use faster LPSPI_WriteData()
	uint16_t rxFrame = 0;
	spip->masterXfer.txData = (uint8_t *)&frame;
	spip->masterXfer.rxData = (uint8_t *)&rxFrame;
	spip->masterXfer.dataSize = sizeof(frame);
	spip->masterXfer.configFlags = spip->flags;

	LPSPI_MasterTransferBlocking(spip->spi, &spip->masterXfer);
	// todo: check status?
	return rxFrame;
}

#endif /* HAL_USE_SPI */

/** @} */
