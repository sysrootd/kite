#include "rtos.h"
#include "gpio.h"

#define RED_LED     14
#define GREEN_LED   13

static semaphore_t sem_red;
static semaphore_t sem_green;

void task1_handler(void) {
    while (1) {
        semaphore_wait(&sem_red);
        gpio_write(GPIOB, RED_LED, 1);
        task_delay(500);
        gpio_write(GPIOB, RED_LED, 0);
        semaphore_post(&sem_green);
    }
}

void task2_handler(void) {
    while (1) {
        semaphore_wait(&sem_green);
        gpio_write(GPIOB, GREEN_LED, 1);
        task_delay(500);
        gpio_write(GPIOB, GREEN_LED, 0);
        semaphore_post(&sem_red);
    }
}

int main(void) {

    gpio_init(GPIOB, RED_LED, OUTPUT, PP, FAST, PU, 0);
    gpio_init(GPIOB, GREEN_LED, OUTPUT, PP, FAST, PU, 0);

    semaphore_init(&sem_red, 1);
    semaphore_init(&sem_green, 0);

    create_task(0, task1_handler, 256U);
    create_task(0, task2_handler, 256U);

    scheduler_init();
    scheduler_start();
    
    for (;;);
}





