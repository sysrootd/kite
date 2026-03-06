#ifndef MEM_H
#define MEM_H

#include <errno.h>
#include <stddef.h>

// (static memory pool allocation to avoid malloc)
// Define here how much memory required for your tcbs in bytes
// Warning total bytes should align word size
  
#define TCB_POOL_SIZE   4096

void *TCB_pool(ptrdiff_t incr);

#endif