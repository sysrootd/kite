#ifndef GPIO_H
#define GPIO_H

#include "coreM4.h"
#include "mcu.h"

#define GPIOA_EN    (1UL << 0)
#define GPIOB_EN    (1UL << 1)
#define GPIOC_EN    (1UL << 2)
#define GPIOD_EN    (1UL << 3)
#define GPIOE_EN    (1UL << 4)
#define GPIOH_EN    (1UL << 7)

#define SYSCFG_EN   (1UL << 14)

#define GPIO_MODE_INPUT   0x0U   
#define GPIO_MODE_OUTPUT  0x1U   
#define GPIO_MODE_AF      0x2U   
#define GPIO_MODE_ANALOG  0x3U   

#define INPUT   GPIO_MODE_INPUT
#define OUTPUT  GPIO_MODE_OUTPUT
#define AF      GPIO_MODE_AF
#define ANALOG  GPIO_MODE_ANALOG

#define GPIO_OTYPE_PP   0x0U     
#define GPIO_OTYPE_OD   0x1U     
#define PP  GPIO_OTYPE_PP
#define OD  GPIO_OTYPE_OD

#define GPIO_SPEED_LOW     0x0U  
#define GPIO_SPEED_MEDIUM  0x1U  
#define GPIO_SPEED_FAST    0x2U  
#define GPIO_SPEED_HIGH    0x3U  
#define LOW     GPIO_SPEED_LOW
#define MEDIUM  GPIO_SPEED_MEDIUM
#define FAST    GPIO_SPEED_FAST
#define HIGH    GPIO_SPEED_HIGH

#define GPIO_PULL_NONE  0x0U     
#define GPIO_PULL_UP    0x1U     
#define GPIO_PULL_DOWN  0x2U     
#define NOPULL  GPIO_PULL_NONE
#define PU      GPIO_PULL_UP
#define PD      GPIO_PULL_DOWN

#define GPIO_TRIGGER_RISING      0x1U   
#define GPIO_TRIGGER_FALLING     0x2U   
#define GPIO_TRIGGER_BOTH_EDGES  0x3U   
#define RISING      GPIO_TRIGGER_RISING
#define FALLING     GPIO_TRIGGER_FALLING
#define BOTH_EDGES  GPIO_TRIGGER_BOTH_EDGES

static inline uint8_t _gpio_port_code(GPIO_TypeDef *port) {
    if      (port == GPIOA) return 0x0U;
    else if (port == GPIOB) return 0x1U;
    else if (port == GPIOC) return 0x2U;
    else if (port == GPIOD) return 0x3U;
    else if (port == GPIOE) return 0x4U;
    else if (port == GPIOH) return 0x7U;
    return 0x0U;
}

static inline IRQn_Type _gpio_exti_irqn(int pin) {
    if      (pin <= 4)  return (IRQn_Type)(EXTI0_IRQn + pin);
    else if (pin <= 9)  return EXTI9_5_IRQn;
    else                return EXTI15_10_IRQn;
}

static inline void gpio_clk(GPIO_TypeDef *port) {
    if      (port == GPIOA) RCC->AHB1ENR |= GPIOA_EN;
    else if (port == GPIOB) RCC->AHB1ENR |= GPIOB_EN;
    else if (port == GPIOC) RCC->AHB1ENR |= GPIOC_EN;
    else if (port == GPIOD) RCC->AHB1ENR |= GPIOD_EN;
    else if (port == GPIOE) RCC->AHB1ENR |= GPIOE_EN;
    else if (port == GPIOH) RCC->AHB1ENR |= GPIOH_EN;
}

static inline void gpio_init(GPIO_TypeDef *port, int pin,
                             int mode,  int type,
                             int speed, int pull,
                             int af) {
    gpio_clk(port);
    
    port->MODER   &= ~(3UL  << ((uint32_t)pin * 2U));
    port->MODER   |=  ((uint32_t)mode  << ((uint32_t)pin * 2U));
    
    port->OTYPER  &= ~(1UL  << (uint32_t)pin);
    port->OTYPER  |=  ((uint32_t)type  << (uint32_t)pin);
    
    port->OSPEEDR &= ~(3UL  << ((uint32_t)pin * 2U));
    port->OSPEEDR |=  ((uint32_t)speed << ((uint32_t)pin * 2U));
    
    port->PUPDR   &= ~(3UL  << ((uint32_t)pin * 2U));
    port->PUPDR   |=  ((uint32_t)pull  << ((uint32_t)pin * 2U));
    
    if (mode == GPIO_MODE_AF) {
        if (pin < 8) {
            port->AFRL &= ~(0xFUL << ((uint32_t)pin * 4U));
            port->AFRL |=  ((uint32_t)af << ((uint32_t)pin * 4U));
        } else {
            port->AFRH &= ~(0xFUL << (((uint32_t)pin - 8U) * 4U));
            port->AFRH |=  ((uint32_t)af << (((uint32_t)pin - 8U) * 4U));
        }
    }
}

static inline void gpio_write(GPIO_TypeDef *port, int pin, int val) {
    if (val) port->BSRR = (1UL << (uint32_t)pin);          
    else     port->BSRR = (1UL << ((uint32_t)pin + 16U));  
}

static inline void gpio_set(GPIO_TypeDef *port, int pin) {
    port->BSRR = (1UL << (uint32_t)pin);
}

static inline void gpio_reset(GPIO_TypeDef *port, int pin) {
    port->BSRR = (1UL << ((uint32_t)pin + 16U));
}

static inline void gpio_toggle(GPIO_TypeDef *port, int pin) {
    port->ODR ^= (1UL << (uint32_t)pin);
}

static inline int gpio_read(GPIO_TypeDef *port, int pin) {
    return (int)((port->IDR >> (uint32_t)pin) & 1UL);
}

static inline int gpio_read_output(GPIO_TypeDef *port, int pin) {
    return (int)((port->ODR >> (uint32_t)pin) & 1UL);
}

static inline void gpio_exti_route(GPIO_TypeDef *port, int pin) {
    RCC->APB2ENR |= SYSCFG_EN;

    uint8_t  code    = _gpio_port_code(port);
    uint32_t cr_idx  = (uint32_t)pin >> 2U;          
    uint32_t bit_pos = ((uint32_t)pin & 0x3U) * 4U;  

    SYSCFG->EXTICR[cr_idx] &= ~(0xFUL  << bit_pos);
    SYSCFG->EXTICR[cr_idx] |=  ((uint32_t)code << bit_pos);
}

static inline void gpio_irq_set_trigger(int pin, int trigger) {
    uint32_t mask = (1UL << (uint32_t)pin);

    if (trigger & GPIO_TRIGGER_RISING)  EXTI->RTSR |=  mask;
    else                                EXTI->RTSR &= ~mask;

    if (trigger & GPIO_TRIGGER_FALLING) EXTI->FTSR |=  mask;
    else                                EXTI->FTSR &= ~mask;
}

static inline void gpio_irq_enable(int pin) {
    EXTI->IMR |= (1UL << (uint32_t)pin);
    NVIC_EnableIRQ(_gpio_exti_irqn(pin));
}

static inline void gpio_irq_disable(int pin) {
    EXTI->IMR &= ~(1UL << (uint32_t)pin);
}

static inline void gpio_irq_clear_pending(int pin) {
    EXTI->PR = (1UL << (uint32_t)pin);
}

static inline int gpio_irq_is_pending(int pin) {
    return (int)((EXTI->PR >> (uint32_t)pin) & 1UL);
}

static inline void gpio_irq_sw_trigger(int pin) {
    EXTI->SWIER |= (1UL << (uint32_t)pin);
}

static inline void gpio_irq_init(GPIO_TypeDef *port, int pin,
                                 int trigger, uint32_t priority) {
    gpio_clk(port);
    
    port->MODER &= ~(3UL << ((uint32_t)pin * 2U));
    
    gpio_exti_route(port, pin);
    gpio_irq_set_trigger(pin, trigger);

    EXTI->PR = (1UL << (uint32_t)pin);

    NVIC_SetPriority(_gpio_exti_irqn(pin), priority);
    gpio_irq_enable(pin);
}

static inline void gpio_event_enable(int pin) {
    EXTI->EMR |= (1UL << (uint32_t)pin);
    gpio_irq_set_trigger(pin, GPIO_TRIGGER_RISING);
}

static inline void gpio_event_disable(int pin) {
    EXTI->EMR &= ~(1UL << (uint32_t)pin);
}

static inline void gpio_irq_event_enable(int pin) {
    EXTI->IMR |= (1UL << (uint32_t)pin);
    EXTI->EMR |= (1UL << (uint32_t)pin);
}

static inline void gpio_irq_input(GPIO_TypeDef *port, int pin,
                                  int trigger, int pull,
                                  uint32_t priority) {
    gpio_init(port, pin, GPIO_MODE_INPUT, GPIO_OTYPE_PP,
              GPIO_SPEED_LOW, pull, 0);
    gpio_irq_init(port, pin, trigger, priority);
}

static inline void gpio_output(GPIO_TypeDef *port, int pin) {
    gpio_init(port, pin, GPIO_MODE_OUTPUT, GPIO_OTYPE_PP,
              GPIO_SPEED_MEDIUM, GPIO_PULL_NONE, 0);
}

#endif 