/*
 * GpioFast.h
 * Fast GPIO pin manipulation.
 * Useful for code-timing checks.
 * For use with C and C++.
 */

/**
 * Define _INLINE_FAST, a more concise variant of frequently used attributes.
 * Regarding inlining:
 *   Despite the statement "An Inline Function is As Fast As a Macro" at https://gcc.gnu.org/onlinedocs/gcc/Inline.html,
 *   the compiler doesn't always take the 'inline' hint.
 *   __attribute__((always_inline)) seemed to be needed because a simple 'inline' would still result in a branch instruction.
 *   People say 'never force inline' (https://stackoverflow.com/a/7866917/101252) but I figure it's no worse than a #define.
 * For poke_p8_from_32b, benchmark was 0.264 usec with, and 1.4 usec without, __attribute__((always_inline)) which is a 5x speed difference.
 */
#define _INLINE_FAST inline __attribute__((always_inline)) __attribute__((optimize("-Ofast")))

/**
 * Define a macro to generate functions.
 * In modern times, we'd use a C++ class instead of these cumbersome macros, but
 * there are two reasons for using macros:
 * [1] Legacy C code can't use C++ classes.
 * [2] I found cases where C++ classes were slower unless the caller was optimized -Ofast.
 *
 * Why two GPIO_output_functions (with and without the '2')? See https://stackoverflow.com/a/1254012/101252 that
 * describes the need for an extra level of "function-like macro" when using the ##
 * pre-processor token-concatenate operator.
 *
 * Example usage:
 *   GPIO_output_functions(dbg);
 *
 * ...will result in a series of functions e.g.,
 * dbg_on() and dbg_off() etc.
 *
 * 'dbg' is assumed to be a symbol defined by CubeMX with a corresponding dbg_GPIO_Port and dbg_PIN in main.h.
 * The generated functions are inline fast functions that are fast even in un-optimized code.
 * These functions are preferred over macros (for debugging, argument-checking) and work in gcc C code.
 * A getter and setter is included. unsigned is used instead of bool or BOOL to be compatible with C and C++.
 *
 * _pulse() is useful for debugging, to differentiate between different events on the logic analyzer.
 */
#define GPIO_output_functions2(x) \
  _INLINE_FAST void  x ## _on()  { x ## _GPIO_Port->BSRR = (uint32_t)( x ## _Pin      ); } \
  _INLINE_FAST void  x ## _off() { x ## _GPIO_Port->BSRR = (uint32_t)( x ## _Pin << 16); } \
  _INLINE_FAST void  x ## _set(uint32_t arg) { if (arg) x ## _on(); else x ## _off(); }    \
  _INLINE_FAST uint32_t x ## _get() { return 0 != (x ## _GPIO_Port->IDR & x ## _Pin); }    \
  _INLINE_FAST void  x ## _pulse(unsigned qty) { for (; qty; qty--) { x ## _on(); x ## _off(); } }

#define GPIO_output_functions(x) GPIO_output_functions2(x)


#define GPIO_input_functions2(x) \
  _INLINE_FAST uint32_t x ## _get() { return 0 != (x ## _GPIO_Port->IDR & x ## _Pin); }
#define GPIO_input_functions(x) GPIO_input_functions2(x)
