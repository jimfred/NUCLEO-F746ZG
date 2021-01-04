// DmaMemSet.cpp

#include "DmaMemSet.h"
#include "assert.h"

namespace DmaMemSet
{

DMA_TypeDef        & r_dma = *DMA2;
DMA_Stream_TypeDef & r_ds  = *DMA2_Stream1;

uint32_t * p_dst32;
uint32_t  /*__attribute__ ((section (".sram2")))*/ src;
unsigned  xfer_qty;



void init()
{
  src = 0;
  assert((r_ds.CR & DMA_SxCR_DIR_Msk) == LL_DMA_DIRECTION_MEMORY_TO_MEMORY);
  DMA2_Stream1->PAR  = (uint32_t)&src; // Set M2M Src Address
}

void memset(uint32_t * p_dst32, uint32_t src, uint32_t qty_32)
{
  load(p_dst32, src, qty_32);
  go();
}

void load(uint32_t * p_dst32_arg, uint32_t val, uint32_t qty_32)
{
  p_dst32 = p_dst32_arg;
  DMA2_Stream1->M0AR = (uint32_t)p_dst32; // Set M2M Dst Address
  src = val;
  xfer_qty = qty_32;
}

void go()
{
  DMA2_Stream1->NDTR = xfer_qty;
  SET_BIT(DMA2->LIFCR, DMA_LIFCR_CTCIF1 | DMA_LIFCR_CHTIF1 | DMA_LIFCR_CTEIF1 | DMA_LIFCR_CDMEIF1 | DMA_LIFCR_CFEIF1);
  SET_BIT(DMA2_Stream1->CR, DMA_SxCR_EN);
}

static uint32_t test_buffer[5000];
static uint32_t test_buffer_canary;

bool test_verify()
{
  if (test_buffer_canary != 0x12345678)
  {
    return false;
  }

  for (unsigned i = 0; i < xfer_qty; i++)
  {
    if (p_dst32[i] != src)
    {
      return false;
    }
  }
  return true;
}


void test_init()
{
  DmaMemSet::load(test_buffer, src, countof(test_buffer));
  test_buffer_canary = 0x12345678;
}

void test_update()
{


  static unsigned fault_qty;
  static unsigned ok_qty;

  assert(!(r_dma.LISR & (DMA_LIFCR_CTEIF1 | DMA_LIFCR_CDMEIF1 | DMA_LIFCR_CFEIF1)));

  const bool xfer_complete = r_dma.LISR & DMA_LISR_TCIF1;


  if (xfer_complete)
  {
    if (!test_verify())
    {
      fault_qty++;
    }
    else
    {
      ok_qty++;
    }
  }


  const bool enabled = READ_BIT(DMA2_Stream1->CR, DMA_SxCR_EN);

  if (xfer_complete || !enabled)
  {
    PB6_off(); // 157 usec @ 5000
    static uint8_t val;
    ++val;
    src = (val<<24) | (val<<16) | (val<<8) | val;

    go();
    PB6_on();

  }

  LD3_set(0 < fault_qty);


} // test_update

} // namespace DmaMemSet

// eof
