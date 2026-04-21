#ifndef MEM_H
#define MEM_H

#include <stddef.h>
#include <stdint.h>
#include "sched.h"

#define WORD_SIZE        sizeof(uint32_t)
#define TCB_POOL_WORDS   512U

KITE_STATIC_ASSERT((TCB_POOL_WORDS * WORD_SIZE) >= 2048U,
                   "Pool must be at least 2KB");

TCB_t *TCB_pool(void);

#endif