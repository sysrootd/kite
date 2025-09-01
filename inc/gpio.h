#ifndef GPIO_H
#define GPIO_H

#include <stdint.h>
#include "stm32f401.h"


/* ===============================
 * RCC AHB1ENR bits
 * =============================== */
#define GPIOA_EN   (1U << 0)
#define GPIOB_EN   (1U << 1)
#define GPIOC_EN   (1U << 2)
#define GPIOD_EN   (1U << 3)
#define GPIOE_EN   (1U << 4)
#define GPIOH_EN   (1U << 7)

/* ===============================
 * Config Options
 * =============================== */
#define IN      0x0U
#define OUT     0x1U
#define AF      0x2U
#define ANALOG  0x3U

#define PP      0x0U
#define OD      0x1U

#define LOW     0x0U
#define MED     0x1U
#define FAST    0x2U
#define HIGH    0x3U

#define NOPULL  0x0U
#define PU      0x1U
#define PD      0x2U

/* ===============================
 * GPIO Functions
 * =============================== */
static inline void gpio_clk(GPIO_TypeDef *port) {
    if (port == GPIOA) RCC->AHB1ENR |= GPIOA_EN;
    else if (port == GPIOB) RCC->AHB1ENR |= GPIOB_EN;
    else if (port == GPIOC) RCC->AHB1ENR |= GPIOC_EN;
    else if (port == GPIOD) RCC->AHB1ENR |= GPIOD_EN;
    else if (port == GPIOE) RCC->AHB1ENR |= GPIOE_EN;
    else if (port == GPIOH) RCC->AHB1ENR |= GPIOH_EN;
}

/* ===============================
 * GPIO Init
 * =============================== */
static inline void gpio_init(GPIO_TypeDef *port, int pin,
                             int mode, int type,
                             int speed, int pull,
                             int af) {
    gpio_clk(port);

    /* mode */
    port->MODER &= ~(3U << (pin * 2));
    port->MODER |= (mode << (pin * 2));

    /* output type */
    port->OTYPER &= ~(1U << pin);
    port->OTYPER |= (type << pin);

    /* speed */
    port->OSPEEDR &= ~(3U << (pin * 2));
    port->OSPEEDR |= (speed << (pin * 2));

    /* pull-up/down */
    port->PUPDR &= ~(3U << (pin * 2));
    port->PUPDR |= (pull << (pin * 2));

    /* alternate function */
    if (mode == AF) {
        int idx = pin / 8;
        int pos = (pin % 8) * 4;
        port->AFR[idx] &= ~(0xFU << pos);
        port->AFR[idx] |= (af << pos);
    }
}

static inline void gpio_write(GPIO_TypeDef *port, int pin, int val) {
    if (val) port->BSRR = (1U << pin);
    else     port->BSRR = (1U << (pin + 16));
}

static inline void gpio_toggle(GPIO_TypeDef *port, int pin) {
    port->ODR ^= (1U << pin);
}

static inline int gpio_read(GPIO_TypeDef *port, int pin) {
    return (port->IDR >> pin) & 1U;
}

#endif /* GPIO_H */
