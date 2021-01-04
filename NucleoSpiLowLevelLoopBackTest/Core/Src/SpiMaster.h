/*
 * SpiMaster.h
 *
 *  Created on: 2020-12-17
 *      Author: Jim Fred
 */

#ifndef SRC_SPIMASTER_H_
#define SRC_SPIMASTER_H_

#include "stm32f7xx_ll_spi.h"
#include "GpioFast.h" // for _INLINE_FAST
#include "assert.h"

namespace SpiMaster
{
static const unsigned buffer_size = 12;
extern uint8_t tx_buf[buffer_size];
extern uint8_t rx_buf[buffer_size];
extern SPI_TypeDef        & r_spi   ;
extern DMA_TypeDef        & r_dma   ;
extern DMA_Stream_TypeDef & r_ds_rx ;
extern DMA_Stream_TypeDef & r_ds_tx ;

static const unsigned dma_lisr_err_mask =
DMA_LISR_TEIF0 |
DMA_LISR_DMEIF0 |
DMA_LISR_FEIF0;

static const unsigned dma_hisr_err_mask =
DMA_HISR_TEIF5 |
DMA_HISR_DMEIF5 |
DMA_HISR_FEIF5;

static const unsigned dma_lisr_mask =
DMA_LISR_TCIF0 |
DMA_LISR_HTIF0 |
dma_lisr_err_mask;

static const unsigned dma_hisr_mask =
DMA_HISR_TCIF5 |
DMA_HISR_HTIF5 |
dma_hisr_err_mask;


void init();

_INLINE_FAST void reload()
{
  assert(READ_REG(r_ds_rx.PAR)  == (uint32_t)&(r_spi.DR)); // peripheral address
  assert(READ_REG(r_ds_rx.M0AR) == (uint32_t)&rx_buf[0]);  // memory address
  assert(READ_REG(r_ds_tx.M0AR) == (uint32_t)&tx_buf[0]);  // memory address
  assert(READ_REG(r_ds_tx.PAR)  == (uint32_t)&(r_spi.DR)); // peripheral address

  //PB6_on();
  WRITE_REG(r_dma.LIFCR, dma_lisr_mask);
  WRITE_REG(r_dma.HIFCR, dma_hisr_mask);

  WRITE_REG(r_ds_rx.NDTR, buffer_size);
  WRITE_REG(r_ds_tx.NDTR, buffer_size);
  //PB6_off(); // 40 ns.

}

_INLINE_FAST void start()
{
  /*
   * The manual specifies this sequence when starting communication using DMA:
   * 1. Enable DMA Rx buffer in the RXDMAEN bit in the SPI_CR2 register.
   * 2. Enable DMA streams for Tx and Rx in DMA registers.
   * 3. Enable DMA Tx buffer in the TXDMAEN bit in the SPI_CR2 register.
   * 4. Enable the SPI by setting the SPE bit.
   * But, HAL swaps 3 and 4.
   */
  //PB6_on();

  SET_BIT(r_spi.CR2, SPI_CR2_RXDMAEN);   // [1]
  SET_BIT(r_ds_rx.CR, DMA_SxCR_EN);      // [2]
  SET_BIT(r_ds_tx.CR, DMA_SxCR_EN);      // [2]
  SET_BIT(r_spi.CR2, SPI_CR2_TXDMAEN);   // [3]
  SET_BIT(r_spi.CR1, SPI_CR1_SPE);       // [4]

  //PB6_off(); // 380 ns
}

_INLINE_FAST void stop()
{
  /* The manual specifies this sequence when stopping communication using DMA:
   * 1. Disable DMA streams for Tx and Rx in the DMA registers.
   * 2. Disable the SPI by following the SPI disable procedure.
   * 3. Disable DMA Tx and Rx buffers by clearing the TXDMAEN and RXDMAEN bits in the SPI_CR2 register.
   */
  //PB6_on();

  // [1]
  CLEAR_BIT(r_ds_rx.CR, DMA_SxCR_EN);
  CLEAR_BIT(r_ds_tx.CR, DMA_SxCR_EN);

  // [2]
  CLEAR_BIT(r_spi.CR1, SPI_CR1_SPE);

  // [3]
  CLEAR_BIT(r_spi.CR2, SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN);

  //PB6_off(); // 300 ns

  assert(!READ_BIT(r_ds_rx.CR, DMA_SxCR_EN));
  assert(!READ_BIT(r_ds_tx.CR, DMA_SxCR_EN));
  assert(!READ_BIT(r_spi.SR, SPI_SR_BSY));

  assert(!READ_BIT(r_spi.CR2, SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN));
}

// async poll to determine if DMA TX is still pending.
_INLINE_FAST bool pending()
{
  return
    !READ_REG(r_dma.HISR & DMA_HISR_TCIF5) || // Transfer Complete
      // TODO: READ_REG(r_ds_tx.NDTR) ||
    READ_BIT(r_spi.SR, SPI_SR_BSY);
}

// Release SS (turn nSS on).
_INLINE_FAST void ss_release() { SPI3_NSS_on(); }

// Assert SS (turn nSS off).
_INLINE_FAST void ss_assert() { SPI3_NSS_off(); }

}

#endif /* SRC_SPIMASTER_H_ */
