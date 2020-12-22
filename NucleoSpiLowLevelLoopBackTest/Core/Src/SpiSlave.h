/*
 * SpiSlave.h
 *
 *  Created on: 2020-12-17
 *      Author: Jim Fred
 */

#ifndef SRC_SPISLAVE_H_
#define SRC_SPISLAVE_H_

#include "stm32f7xx_ll_spi.h"
#include "GpioFast.h" // for _INLINE_FAST
#include "assert.h"
#include "string.h"

// This enables asserts used to check assumptions.
#define SPI_SLAVE_CHECK 0

namespace SpiSlave
{
static const unsigned buffer_size = 24;
extern uint8_t tx_buf[buffer_size];
extern uint8_t rx_buf[buffer_size];
extern SPI_TypeDef        & r_spi   ;
extern DMA_TypeDef        & r_dma   ;
extern DMA_Stream_TypeDef & r_ds_rx ;
extern DMA_Stream_TypeDef & r_ds_tx ;

static const unsigned dma_lisr_err_mask =
DMA_LISR_TEIF0 |
DMA_LISR_DMEIF0 |
DMA_LISR_FEIF0 |
DMA_LISR_TEIF3 |
DMA_LISR_DMEIF3 |
DMA_LISR_FEIF3;


static const unsigned dma_lisr_mask =
DMA_LISR_TCIF0 |
DMA_LISR_HTIF0 |
DMA_LISR_TCIF3 |
DMA_LISR_HTIF3 |
dma_lisr_err_mask;


void init();

_INLINE_FAST void reload()
{
#if SPI_SLAVE_CHECK
  assert(READ_REG(r_ds_rx.PAR) == (uint32_t)&(r_spi.DR)); // peripheral address
  assert(READ_REG(r_ds_rx.M0AR) == (uint32_t)&rx_buf[0]); // memory address
  assert(READ_REG(r_ds_tx.M0AR) == (uint32_t)&tx_buf[0]); // memory address
  assert(READ_REG(r_ds_tx.PAR) == (uint32_t)&(r_spi.DR)); // peripheral address
#endif // SPI_SLAVE_CHECK

  // PB6_on();

  //PB6_on();
  WRITE_REG(r_dma.LIFCR, dma_lisr_mask); // Reset LISR bits (TC, error etc).

  WRITE_REG(r_ds_rx.NDTR, buffer_size);
  WRITE_REG(r_ds_tx.NDTR, buffer_size);
  //PB6_off(); // 40 ns.

  // PB6_off(); // 36 nsec

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
  // PB6_on();
  SET_BIT(r_spi.CR2, SPI_CR2_RXDMAEN);   // [1]
  SET_BIT(r_ds_rx.CR, DMA_SxCR_EN);      // [2]
  SET_BIT(r_ds_tx.CR, DMA_SxCR_EN);      // [2]
  SET_BIT(r_spi.CR2, SPI_CR2_TXDMAEN);   // [3]
  SET_BIT(r_spi.CR1, SPI_CR1_SPE);       // [4]

  // PB6_off(); // 260 ns
}

///_INLINE_FAST
inline void stop()
{
  /* The manual specifies this sequence when stopping communication using DMA:
   * 1. Disable DMA streams for Tx and Rx in the DMA registers.
   * 2. Disable the SPI by following the SPI disable procedure.
   * 3. Disable DMA Tx and Rx buffers by clearing the TXDMAEN and RXDMAEN bits in the SPI_CR2 register.
   */
  /// PB6_on();

  // [1]
  CLEAR_BIT(r_ds_rx.CR, DMA_SxCR_EN);
  CLEAR_BIT(r_ds_tx.CR, DMA_SxCR_EN);

  // [2]
  CLEAR_BIT(r_spi.CR1, SPI_CR1_SPE);

  // [3]
  CLEAR_BIT(r_spi.CR2, SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN);

  /// PB6_off(); // 292 ns

#if SPI_SLAVE_CHECK
  assert(!READ_BIT(r_ds_rx.CR, DMA_SxCR_EN));
  assert(!READ_BIT(r_ds_tx.CR, DMA_SxCR_EN));
  assert(!READ_BIT(r_spi.CR2, SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN));
#endif

}


// Reset the SPI TX FIFO.
// The only way to clear the TX FIFO is to reset the whole SPI port. See...
// https://electronics.stackexchange.com/questions/260856/stm32-spi-slave-reset-dma-state-on-high-nss?rq=1
// After reset, the SPI configuration needs to be restored.
// To quickly restore the configuration, save the settings before reset and then re-apply after reset.
// Timing is critical if this is done upon assertion of nSS.
// Caution: This is hard-coded for SPI1 because the RCC bit is for pSPIx.
// This reset is intended to be done 'frequently' when AdrStbl is asserted (or, optionally, when released, when there is more time).
//
// Three main steps:
// [1] save configuration in 2 SPI configuration registers, CR1 and CR2.
//     Only 16 bits are used in required but process all 32 - execution time is the same.
//     CRCPR is the only other configurable register and is unused but its default of 7 is ok.
// [2] reset the SPI port using RCC. This effectively resets the whole SPI port.
//     This is a 3-step process: saving the register, OR-ing the reset bit and restoring the register.
//     Restoring the register is slightly faster than AND-ing the reset bit back to zero.
// [3] restore SPI configuration registers, CR1 and CR2.
// Timing measurements:
//   0.494 usec. with -O0
//   0.146  usec. with -Ofast (3.8x faster)
// For details on gcc's attribute optimize, see https://stackoverflow.com/a/5581131/101252
//
// I've since discovered that this, although necessary on the SloSpi, may not be needed on the FastSpi because idle zeros are OK as long as
// idle zeros are not part of fixed-length segments. Idle zeros are allowed before the initial F10001
inline __attribute__((always_inline)) void __attribute__((optimize("-Ofast"))) spi_fifo_reset()
{
  // PB6_on();

  const register uint32_t save_CR1 = r_spi.CR1; // Save
  const register uint32_t save_CR2 = r_spi.CR2;

  uint32_t const rcc = RCC->APB2RSTR; // Save
  RCC->APB2RSTR = rcc | RCC_APB2RSTR_SPI1RST; // Assert reset
  RCC->APB2RSTR = rcc; // Release reset by restoring the register.

  r_spi.CR1 = save_CR1; // Restore.
  r_spi.CR2 = save_CR2;

  // PB6_off(); // 176 nsec.

  // These asserts were used to verify this technique of saving and restoring the config registers.
  // I never saw these asserts fail.
#if SPI_SLAVE_CHECK
  assert(r_spi.CR1 == save_CR1); // Verify (optional).
  assert(r_spi.CR2 == save_CR2);
#endif
}


_INLINE_FAST bool pending() { return READ_REG(r_ds_tx.NDTR) || READ_BIT(r_spi.SR, SPI_SR_BSY); }

_INLINE_FAST void restart()
{
  SpiSlave::spi_fifo_reset();
  SpiSlave::reload();
  SpiSlave::start();
}

}

#endif /* SRC_SPISLAVE_H_ */
