// assert.h
// Intended to be used in:
// - HardFault_Handler() etc in stm32f3xx_it.c
// - Error_Handler() in main.cpp, referenced by CubeMX-generated code.
// - assert_param() in stm32f3xx_hal_conf.h which is referenced in HAL code
// - unit tests
// Intended to replace system library's assert.h to reduce size of __FILE__ strings.

#ifndef __assert_h__
#define __assert_h__

#include "main.h" // for USE_FULL_ASSERT

#ifdef __cplusplus
 extern "C" {
#endif

#ifdef  USE_FULL_ASSERT
    void bkpt();

    // Define assert.
    // See also, assert_param used by HAL.
    // Derived from https://chromium.googlesource.com/chromiumos/platform/ec/+/master/builtin/assert.h
    #define assert(cond) do { if (!(cond)) { bkpt(); } } while (0)
#else
#define assert(cond)
#endif

#ifdef __cplusplus
}
#endif

#endif // include guard

// eof
