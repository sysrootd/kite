#include <stddef.h>
#include <stdint.h>

#include "mem.h"

static uint8_t  tcb_pool_mem[TCB_POOL_SIZE] __attribute__((aligned(8)));
static uint8_t *pool_ptr = tcb_pool_mem;
static uint8_t *pool_end = tcb_pool_mem + TCB_POOL_SIZE;

TCB_t *TCB_pool(void)
{
    size_t size = (sizeof(TCB_t) + 7U) & ~7U;

    if ((pool_ptr + size) > pool_end)
    {
        return NULL;
    }

    TCB_t *tcb = (TCB_t *)pool_ptr;
    pool_ptr  += size;

    return tcb;
}