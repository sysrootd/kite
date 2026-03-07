#ifndef MEM_H
#define MEM_H

// (pre static memory pool allocation to avoid malloc)
// Define here how much memory required for your tcbs in bytes
// Warning: total bytes should align word
  
#define TCB_POOL_SIZE   4096

void *TCB_pool(ptrdiff_t incr);

#endif