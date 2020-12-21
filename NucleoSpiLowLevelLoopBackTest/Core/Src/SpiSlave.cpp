/*
 * SpiSlave.cpp
 *
 *  Created on: 2020-12-17
 *      Author: Jim Fred
 */

#include "SpiSlave.h"

namespace SpiSlave
{

uint8_t tx_buf[buffer_size];
uint8_t rx_buf[buffer_size];
SPI_TypeDef        & r_spi   = *SPI1;
DMA_TypeDef        & r_dma   = *DMA2;
DMA_Stream_TypeDef & r_ds_rx = *DMA2_Stream0;
DMA_Stream_TypeDef & r_ds_tx = *DMA2_Stream3;
EXTI_TypeDef       & r_exti  = *EXTI;

// Create a mask to
// - check bits in the DMA LISR register and
// - clear corresponding bits in the DMA LIFCR register.
// These bits must be cleared before starting a new DMA transfer.
// There are 10 bits: 5 bits, TCIF HTIF TEIF DMEIF FEIF, for 2 streams, stream0 for RX. stream3 for TX.

void init()
{
  // Assume that the xISR and xIFCR_C bits are the same.
  assert(
    DMA_LISR_TCIF0  == DMA_LIFCR_CTCIF0  &&
    DMA_LISR_HTIF0  == DMA_LIFCR_CHTIF0  &&
    DMA_LISR_TEIF0  == DMA_LIFCR_CTEIF0  &&
    DMA_LISR_DMEIF0 == DMA_LIFCR_CDMEIF0 &&
    DMA_LISR_FEIF0  == DMA_LIFCR_CFEIF0  &&
    DMA_LISR_TCIF3  == DMA_LIFCR_CTCIF3  &&
    DMA_LISR_HTIF3  == DMA_LIFCR_CHTIF3  &&
    DMA_LISR_TEIF3  == DMA_LIFCR_CTEIF3  &&
    DMA_LISR_DMEIF3 == DMA_LIFCR_CDMEIF3 &&
    DMA_LISR_FEIF3  == DMA_LIFCR_CFEIF3
  );

  // Check configuration.
  assert(!READ_BIT(r_spi.CR1, SPI_CR1_MSTR)); // Should already be a Slave.
  assert(READ_BIT(r_ds_rx.CR, DMA_SxCR_DIR) ==              0); // 0              is  Peripheral to memory.
  assert(READ_BIT(r_ds_tx.CR, DMA_SxCR_DIR) == DMA_SxCR_DIR_0); // DMA_SxCR_DIR_0 is  Memory to peripheral.
  assert(READ_BIT(r_spi.CR1, SPI_CR1_SSM)); // Software Select Management (of nSS) must be enabled if using the SPI_CR1_SSI.
  assert(!READ_BIT(r_spi.CR2, SPI_CR2_SSOE)); // For slave, SSOE should be off.

  assert(!READ_BIT(r_spi.CR1, SPI_CR1_SPE)); // Should not be enabled.

  static const unsigned check_ds_cr_clear_mask =
      DMA_SxCR_EN    |
      DMA_SxCR_DBM   |
      DMA_SxCR_TCIE  |
      DMA_SxCR_HTIE  |
      DMA_SxCR_TEIE  |
      DMA_SxCR_DMEIE;

  assert(!READ_BIT(r_ds_rx.CR, check_ds_cr_clear_mask)); // Should already be cleared.
  assert(!READ_BIT(r_ds_tx.CR, check_ds_cr_clear_mask));

  assert(!READ_BIT(r_ds_rx.FCR, DMA_SxFCR_DMDIS)); // Direct mode enabled (FIFO off).
  assert(!READ_BIT(r_ds_tx.FCR, DMA_SxFCR_DMDIS)); // Direct mode enabled (FIFO off).

  assert(!READ_BIT(r_dma.LISR, dma_lisr_mask));

  // Initialize.
  //WRITE_REG(r_ds_rx.NDTR, buffer_size);
  WRITE_REG(r_ds_rx.PAR, (uint32_t )&(r_spi.DR)); // peripheral address
  WRITE_REG(r_ds_rx.M0AR, (uint32_t )&rx_buf[0]); // memory address
  //WRITE_REG(r_ds_tx.NDTR, buffer_size);
  WRITE_REG(r_ds_tx.M0AR, (uint32_t )&tx_buf[0]); // memory address
  WRITE_REG(r_ds_tx.PAR, (uint32_t )&(r_spi.DR)); // peripheral address

  SET_BIT(r_spi.CR1, SPI_CR1_SSI); // Initially, release software-control of nSS - 1=release.

  memcpy(tx_buf, "\x01\x02\x04\x08\x10\x20\x40\x80", 8);

}




} // namespace SpiSlave

// Interrupt handler for slave-side nSS signal.
// This replaces a definition in stm32f7xx_it.c to optimize performance.
extern "C" __attribute__((optimize("-Ofast"))) void EXTI4_IRQHandler(void)
{
#if SPI_SLAVE_CHECK
  assert(SpiSlave::r_exti.PR & SPI1_NSS_Pin); // Assumed to be the only trigger for this handler.
#endif

  SpiSlave::r_exti.PR = SPI1_NSS_Pin; // |= not needed for the PR register because writing a zero-bit has no effect.

  if (SPI1_NSS_get())
  {
    SpiSlave::r_spi.CR1 |= SPI_CR1_SSI;
    PB6_pulse(4);
  }
  else
  {
    SpiSlave::r_spi.CR1 &= ~SPI_CR1_SSI; // Start Slave.
    PB6_pulse(2);
  }
}

