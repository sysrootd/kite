#include <errno.h>
#include <stddef.h>

extern char _ebss;
extern char _estack;

void * _sbrk(ptrdiff_t incr)
{
    static char *heap_end = &_ebss;
    char *prev_heap_end = heap_end;

    if (heap_end + incr > &_estack)
    {
        errno = ENOMEM;
        return (void *)-1;
    }

    heap_end += incr;
    return (void *)prev_heap_end;
}