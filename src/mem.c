#include <stddef.h>
#include <stdint.h>

#include "mem.h"

static uint8_t tcb_pool_mem[TCB_POOL_SIZE] __attribute__((aligned(8)));
static uint8_t *pool_start = tcb_pool_mem;
static uint8_t *pool_end   = tcb_pool_mem + TCB_POOL_SIZE;

TCB_t* TCB_pool(void)
{
    size_t size = sizeof(TCB_t);

    if ((pool_start + size) > pool_end)
    {
        return NULL;
    }

    TCB_t *tcb = (TCB_t *)pool_start;
    pool_start += size;

    return tcb;
}