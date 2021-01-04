// DmaMemSet.h

#ifndef __DmaMemSet_h__
#define __DmaMemSet_h__

#include "main.h"

namespace DmaMemSet
{
extern unsigned fault_qty;
extern unsigned ok_qty;

void init();
void memset(uint32_t * p_dst32, uint32_t src, uint32_t qty_32);
void load(uint32_t * p_dst32, uint32_t val, uint32_t qty_32);
void go();

void test_init();
void test_update();

};

#endif // include guard

// eof
