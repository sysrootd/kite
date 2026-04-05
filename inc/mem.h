#ifndef MEM_H
#define MEM_H

#include "sched.h"
// (pre static memory pool allocation to avoid malloc)
// Define here how much memory required for your tcbs in bytes
// Warning: total bytes should align word
  
#define TCB_POOL_SIZE   4096

TCB_t* TCB_pool(void);

#endif