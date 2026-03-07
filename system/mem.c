
#include <errno.h>
#include <stddef.h>

#include "mem.h"

extern char _ebss;

static char *pool_start = &_ebss;
static char *pool_end = &_ebss + TCB_POOL_SIZE;

void *TCB_pool(ptrdiff_t incr)
{
    char *prev_pool_end = pool_start;

    if (pool_start + incr > pool_end)
    {
        errno = ENOMEM;
        return (void *)-1;
    }

    pool_start += incr;
    return (void *)prev_pool_end;
}