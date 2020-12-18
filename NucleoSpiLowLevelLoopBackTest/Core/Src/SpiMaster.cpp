/*
 * SpiMaster.cpp
 *
 *  Created on: 2020-12-17
 *      Author: Jim Fred
 */

#include "SpiMaster.h"

namespace SpiMaster
{

uint8_t tx_buf[buffer_size];
uint8_t rx_buf[buffer_size];
SPI_TypeDef        & r_spi   = *SPI3;
DMA_TypeDef        & r_dma   = *DMA1;
DMA_Stream_TypeDef & r_ds_rx = *DMA1_Stream0;
DMA_Stream_TypeDef & r_ds_tx = *DMA1_Stream5;

// Create a mask to
// - check bits in the DMA LISR register and
// - clear corresponding bits in the DMA LIFCR register.
// These bits must be cleared before starting a new DMA transfer.
// There are 10 bits: 5 bits, TCIF HTIF TEIF DMEIF FEIF, for 2 streams, stream0 for RX. stream5 for TX.



void init()
{
  // Assume that the xISR and xIFCR_C bits are the same.
  assert(
    DMA_LISR_TCIF0  == DMA_LIFCR_CTCIF0  &&
    DMA_LISR_HTIF0  == DMA_LIFCR_CHTIF0  &&
    DMA_LISR_TEIF0  == DMA_LIFCR_CTEIF0  &&
    DMA_LISR_DMEIF0 == DMA_LIFCR_CDMEIF0 &&
    DMA_LISR_FEIF0  == DMA_LIFCR_CFEIF0  &&
    DMA_HISR_TCIF5  == DMA_HIFCR_CTCIF5  &&
    DMA_HISR_HTIF5  == DMA_HIFCR_CHTIF5  &&
    DMA_HISR_TEIF5  == DMA_HIFCR_CTEIF5  &&
    DMA_HISR_DMEIF5 == DMA_HIFCR_CDMEIF5 &&
    DMA_HISR_FEIF5  == DMA_HIFCR_CFEIF5
  );

  // Check assumptions.
  assert(READ_BIT(r_spi.CR1, SPI_CR1_MSTR)); // Should already be a master.

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
  assert(!READ_BIT(r_dma.HISR, dma_hisr_mask));

  // Initialize.
  WRITE_REG(r_ds_rx.NDTR, buffer_size);
  WRITE_REG(r_ds_rx.M0AR, (uint32_t )&(r_spi.DR)); // source address
  WRITE_REG(r_ds_rx.PAR, (uint32_t )&rx_buf[0]);   // destination address
  WRITE_REG(r_ds_tx.NDTR, buffer_size);
  WRITE_REG(r_ds_tx.M0AR, (uint32_t )&tx_buf[0]);  // source address
  WRITE_REG(r_ds_tx.PAR, (uint32_t )&(r_spi.DR));  // destination address

}






} // namespace SpiMaster

