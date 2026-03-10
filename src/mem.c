
#include <errno.h>
#include <stddef.h>

#include "mem.h"

extern char _ebss;//end of .bss section sysmbol from linker

static char *pool_start = &_ebss;
static char *pool_end = &_ebss + TCB_POOL_SIZE;

TCB_t* TCB_pool(ptrdiff_t incr)
{
    char *prev_pool_end = pool_start;

    if (pool_start + incr > pool_end)
    {
        errno = ENOMEM;
        return (void *)-1;
    }

    pool_start += incr;
    return (TCB_t *)prev_pool_end;
}