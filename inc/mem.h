#ifndef MEM_H
#define MEM_H

#include <stddef.h>
#include "sched.h"

#define TCB_POOL_SIZE  512U

KITE_STATIC_ASSERT((TCB_POOL_SIZE % 8U) == 0U,
    "TCB_POOL_SIZE must be a multiple of 8");
KITE_STATIC_ASSERT(TCB_POOL_SIZE >= 256U,
    "TCB_POOL_SIZE too small for any useful task count");

TCB_t *TCB_pool(void);

#endif