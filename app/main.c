#include "mcu.h"
#include "board.h"
#include "task.h"
#include "sched.h"

int main(void)
{
    board_init();
    tasks_init();

    kite_start();

    while (1)
    {
        __WFI();
    }
}
