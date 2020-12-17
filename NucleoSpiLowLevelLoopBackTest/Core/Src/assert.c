// assert.cpp

#include "assert.h"

#ifdef  USE_FULL_ASSERT

  // bkpt()
  // Set a breakpoint at this function.
  // Similar to __BKPT() but __BKPT() doesn't behave well in Atollic/eclipse/gdb.
  // __BKPT() works ok in IAR but in Atollic/gdb, the debugger would hang or sometimes not stop at the bkpt.
  // This requires that the debugger set a breakpoint at bkpt which could be automated in the startup script.
  void bkpt() {;}

#endif // USE_FULL_ASSERT

// eof
