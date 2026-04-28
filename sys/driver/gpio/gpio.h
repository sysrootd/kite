#ifndef GPIO_H
#define GPIO_H

/* gpio.h depends on coreM4.h for IRQn_Type, NVIC_*, SCB, and
   on mcu.h for GPIO_TypeDef / RCC / GPIOx port pointers.         */
#include "coreM4.h"
#include "mcu.h"

/* =========================================================================
   gpio.h  —  STM32F4 GPIO + EXTI Interrupt Support
   Covers: GPIO init/read/write/toggle, alternate functions,
           SYSCFG EXTI routing, edge-trigger, NVIC control (via CMSIS),
           software triggers, event (WFE) mode, pending-flag management.
   ========================================================================= */


/* =========================================================================
   RCC Clock-Enable Bits
   ========================================================================= */

/* AHB1ENR — GPIO port clocks */
#define GPIOA_EN    (1UL << 0)
#define GPIOB_EN    (1UL << 1)
#define GPIOC_EN    (1UL << 2)
#define GPIOD_EN    (1UL << 3)
#define GPIOE_EN    (1UL << 4)
#define GPIOH_EN    (1UL << 7)

/* APB2ENR — SYSCFG clock (required for EXTI port routing) */
#define SYSCFG_EN   (1UL << 14)


/* =========================================================================
   GPIO Configuration Constants
   ========================================================================= */

/* Pin Mode  (MODER — 2 bits per pin) */
#define GPIO_MODE_INPUT   0x0U   /* Digital input (reset state)              */
#define GPIO_MODE_OUTPUT  0x1U   /* General-purpose output                   */
#define GPIO_MODE_AF      0x2U   /* Alternate function                       */
#define GPIO_MODE_ANALOG  0x3U   /* Analog / ADC                             */

/* Short aliases for backwards compatibility */
#define INPUT   GPIO_MODE_INPUT
#define OUTPUT  GPIO_MODE_OUTPUT
#define AF      GPIO_MODE_AF
#define ANALOG  GPIO_MODE_ANALOG

/* Output Type  (OTYPER — 1 bit per pin) */
#define GPIO_OTYPE_PP   0x0U     /* Push-pull  (default)                    */
#define GPIO_OTYPE_OD   0x1U     /* Open-drain                              */
#define PP  GPIO_OTYPE_PP
#define OD  GPIO_OTYPE_OD

/* Output Speed  (OSPEEDR — 2 bits per pin) */
#define GPIO_SPEED_LOW     0x0U  /* ~  2 MHz                                */
#define GPIO_SPEED_MEDIUM  0x1U  /* ~ 25 MHz                                */
#define GPIO_SPEED_FAST    0x2U  /* ~ 50 MHz                                */
#define GPIO_SPEED_HIGH    0x3U  /* ~100 MHz                                */
#define LOW     GPIO_SPEED_LOW
#define MEDIUM  GPIO_SPEED_MEDIUM
#define FAST    GPIO_SPEED_FAST
#define HIGH    GPIO_SPEED_HIGH

/* Pull-up / Pull-down  (PUPDR — 2 bits per pin) */
#define GPIO_PULL_NONE  0x0U     /* No pull (floating)                      */
#define GPIO_PULL_UP    0x1U     /* Internal pull-up                        */
#define GPIO_PULL_DOWN  0x2U     /* Internal pull-down                      */
#define NOPULL  GPIO_PULL_NONE
#define PU      GPIO_PULL_UP
#define PD      GPIO_PULL_DOWN


/* =========================================================================
   EXTI Trigger Edge  (used with gpio_irq_init / gpio_irq_set_trigger)
   ========================================================================= */
#define GPIO_TRIGGER_RISING      0x1U   /* Rising  edge only                */
#define GPIO_TRIGGER_FALLING     0x2U   /* Falling edge only                */
#define GPIO_TRIGGER_BOTH_EDGES  0x3U   /* Both edges                       */
#define RISING      GPIO_TRIGGER_RISING
#define FALLING     GPIO_TRIGGER_FALLING
#define BOTH_EDGES  GPIO_TRIGGER_BOTH_EDGES

/* =========================================================================
   Internal Helpers  (not part of the public API — prefixed with _gpio_)
   ========================================================================= */

/**
 * _gpio_port_code
 * Returns the 4-bit SYSCFG EXTICR port-select code for a GPIO port.
 * A=0, B=1, C=2, D=3, E=4, H=7  (matches RM0368 Table 27).
 */
static inline uint8_t _gpio_port_code(GPIO_TypeDef *port) {
    if      (port == GPIOA) return 0x0U;
    else if (port == GPIOB) return 0x1U;
    else if (port == GPIOC) return 0x2U;
    else if (port == GPIOD) return 0x3U;
    else if (port == GPIOE) return 0x4U;
    else if (port == GPIOH) return 0x7U;
    return 0x0U;
}

/**
 * _gpio_exti_irqn
 * Returns the IRQn_Type for a given EXTI line (pin 0-15).
 *   Lines 0-4   → individual vectors (EXTI0_IRQn .. EXTI4_IRQn)
 *   Lines 5-9   → shared EXTI9_5_IRQn
 *   Lines 10-15 → shared EXTI15_10_IRQn
 */
static inline IRQn_Type _gpio_exti_irqn(int pin) {
    if      (pin <= 4)  return (IRQn_Type)(EXTI0_IRQn + pin);
    else if (pin <= 9)  return EXTI9_5_IRQn;
    else                return EXTI15_10_IRQn;
}


/* =========================================================================
   GPIO Clock Enable
   ========================================================================= */
/**
 * gpio_clk
 * Enable the AHB1 peripheral clock for the given GPIO port.
 */
static inline void gpio_clk(GPIO_TypeDef *port) {
    if      (port == GPIOA) RCC->AHB1ENR |= GPIOA_EN;
    else if (port == GPIOB) RCC->AHB1ENR |= GPIOB_EN;
    else if (port == GPIOC) RCC->AHB1ENR |= GPIOC_EN;
    else if (port == GPIOD) RCC->AHB1ENR |= GPIOD_EN;
    else if (port == GPIOE) RCC->AHB1ENR |= GPIOE_EN;
    else if (port == GPIOH) RCC->AHB1ENR |= GPIOH_EN;
}


/* =========================================================================
   GPIO Pin Initialisation
   ========================================================================= */
/**
 * gpio_init
 * Configure all registers for a single GPIO pin.
 *
 * @param port   GPIOx base pointer  (e.g. GPIOA)
 * @param pin    Pin number          (0-15)
 * @param mode   GPIO_MODE_*        (INPUT / OUTPUT / AF / ANALOG)
 * @param type   GPIO_OTYPE_*       (PP / OD)
 * @param speed  GPIO_SPEED_*       (LOW / MEDIUM / FAST / HIGH)
 * @param pull   GPIO_PULL_*        (NOPULL / PU / PD)
 * @param af     Alternate-function index (0-15); ignored when mode != AF
 */
static inline void gpio_init(GPIO_TypeDef *port, int pin,
                             int mode,  int type,
                             int speed, int pull,
                             int af) {
    gpio_clk(port);

    /* Mode (2 bits) */
    port->MODER   &= ~(3UL  << ((uint32_t)pin * 2U));
    port->MODER   |=  ((uint32_t)mode  << ((uint32_t)pin * 2U));

    /* Output type (1 bit) */
    port->OTYPER  &= ~(1UL  << (uint32_t)pin);
    port->OTYPER  |=  ((uint32_t)type  << (uint32_t)pin);

    /* Output speed (2 bits) */
    port->OSPEEDR &= ~(3UL  << ((uint32_t)pin * 2U));
    port->OSPEEDR |=  ((uint32_t)speed << ((uint32_t)pin * 2U));

    /* Pull-up / pull-down (2 bits) */
    port->PUPDR   &= ~(3UL  << ((uint32_t)pin * 2U));
    port->PUPDR   |=  ((uint32_t)pull  << ((uint32_t)pin * 2U));

    /* Alternate function (4 bits in AFR[L/H]) */
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


/* =========================================================================
   GPIO Digital I/O
   ========================================================================= */

/**
 * gpio_write
 * Set (val != 0) or clear (val == 0) a pin atomically using BSRR.
 * Interrupt-safe — no read-modify-write on ODR.
 */
static inline void gpio_write(GPIO_TypeDef *port, int pin, int val) {
    if (val) port->BSRR = (1UL << (uint32_t)pin);          /* BS — set   */
    else     port->BSRR = (1UL << ((uint32_t)pin + 16U));  /* BR — reset */
}

/** Atomically drive a pin HIGH */
static inline void gpio_set(GPIO_TypeDef *port, int pin) {
    port->BSRR = (1UL << (uint32_t)pin);
}

/** Atomically drive a pin LOW */
static inline void gpio_reset(GPIO_TypeDef *port, int pin) {
    port->BSRR = (1UL << ((uint32_t)pin + 16U));
}

/**
 * gpio_toggle
 * Toggle output via XOR on ODR.
 * Not atomic — guard with __disable_irq()/__enable_irq() if called from
 * both main context and an ISR on the same port.
 */
static inline void gpio_toggle(GPIO_TypeDef *port, int pin) {
    port->ODR ^= (1UL << (uint32_t)pin);
}

/**
 * gpio_read
 * Sample the pin's logic level from IDR.
 * @return  1 = HIGH, 0 = LOW
 */
static inline int gpio_read(GPIO_TypeDef *port, int pin) {
    return (int)((port->IDR >> (uint32_t)pin) & 1UL);
}

/**
 * gpio_read_output
 * Read the programmed output state from ODR (not the actual pin level).
 * Useful on open-drain pins where IDR reflects the external level.
 * @return  1 if output bit is set, 0 otherwise.
 */
static inline int gpio_read_output(GPIO_TypeDef *port, int pin) {
    return (int)((port->ODR >> (uint32_t)pin) & 1UL);
}


/* =========================================================================
   SYSCFG / EXTI Routing
   ========================================================================= */

/**
 * gpio_exti_route
 * Enable SYSCFG clock and connect the GPIO port to the EXTI line for
 * the given pin via SYSCFG_EXTICR.
 *
 * Each EXTICR register covers 4 consecutive pins (4 bits each):
 *   EXTICR[0] → pins  0-3
 *   EXTICR[1] → pins  4-7
 *   EXTICR[2] → pins  8-11
 *   EXTICR[3] → pins 12-15
 *
 * Call this BEFORE enabling the EXTI interrupt.
 */
static inline void gpio_exti_route(GPIO_TypeDef *port, int pin) {
    RCC->APB2ENR |= SYSCFG_EN;

    uint8_t  code    = _gpio_port_code(port);
    uint32_t cr_idx  = (uint32_t)pin >> 2U;          /* EXTICR[0-3]          */
    uint32_t bit_pos = ((uint32_t)pin & 0x3U) * 4U;  /* Bit offset in word   */

    SYSCFG->EXTICR[cr_idx] &= ~(0xFUL  << bit_pos);
    SYSCFG->EXTICR[cr_idx] |=  ((uint32_t)code << bit_pos);
}


/* =========================================================================
   EXTI Interrupt Configuration
   ========================================================================= */

/**
 * gpio_irq_set_trigger
 * Set which edge fires the EXTI interrupt / event for a pin.
 * @param pin      EXTI line (= pin number, 0-15)
 * @param trigger  GPIO_TRIGGER_RISING | GPIO_TRIGGER_FALLING | GPIO_TRIGGER_BOTH_EDGES
 */
static inline void gpio_irq_set_trigger(int pin, int trigger) {
    uint32_t mask = (1UL << (uint32_t)pin);
);
    if (trigger & GPIO_TRIGGER_RISING)  EXTI->RTSR |=  mask;
    else                                EXTI->RTSR &= ~mask;

    if (trigger & GPIO_TRIGGER_FALLING) EXTI->FTSR |=  mask;
    else                                EXTI->FTSR &= ~mask;
}

/**
 * gpio_irq_enable
 * Unmask the EXTI IMR bit for the pin and enable the NVIC vector.
 * Safe to call on shared vectors (lines 5-9, 10-15) — per-pin gating
 * is handled through the individual IMR bits.
 */
static inline void gpio_irq_enable(int pin) {
    EXTI->IMR |= (1UL << (uint32_t)pin);
    NVIC_EnableIRQ(_gpio_exti_irqn(pin));
}

/**
 * gpio_irq_disable
 * Mask the EXTI IMR bit for the pin only.
 * The NVIC vector is left enabled intentionally — other pins on the same
 * shared vector (e.g. 5-9, 10-15) would break if the NVIC were disabled.
 * To fully shut down a shared vector, mask all pins on it first, then call
 * NVIC_DisableIRQ(_gpio_exti_irqn(pin)) manually.
 */
static inline void gpio_irq_disable(int pin) {
    EXTI->IMR &= ~(1UL << (uint32_t)pin);
}

/**
 * gpio_irq_clear_pending
 * Clear the EXTI pending flag for a pin (write-1-to-clear on PR).
 * *** MUST be called inside every EXTI ISR ***
 * Failing to clear the flag causes the ISR to re-enter immediately on exit.
 */
static inline void gpio_irq_clear_pending(int pin) {
    EXTI->PR = (1UL << (uint32_t)pin);
}

/**
 * gpio_irq_is_pending
 * Poll the EXTI pending flag without entering an ISR.
 * Essential in shared-vector ISRs to determine which pin fired.
 * @return  1 if pending, 0 otherwise.
 */
static inline int gpio_irq_is_pending(int pin) {
    return (int)((EXTI->PR >> (uint32_t)pin) & 1UL);
}

/**
 * gpio_irq_sw_trigger
 * Assert the EXTI line in software via the SWIER register.
 * Generates an interrupt if the IMR bit is set (good for ISR unit-testing).
 * Hardware auto-sets PR when SWIER is written; clear it in the ISR via
 * gpio_irq_clear_pending().
 */
static inline void gpio_irq_sw_trigger(int pin) {
    EXTI->SWIER |= (1UL << (uint32_t)pin);
}

/**
 * gpio_irq_init
 * All-in-one helper: clock the port, set INPUT mode, route via SYSCFG,
 * configure the trigger edge, clear stale pending flags, set NVIC priority,
 * and enable the interrupt.
 *
 * @param port      GPIOx pointer  (e.g. GPIOC)
 * @param pin       Pin number     (0-15)
 * @param trigger   GPIO_TRIGGER_RISING / FALLING / BOTH_EDGES
 * @param priority  NVIC pre-empt priority  (0 = highest)
 *
 * Example:
 *   gpio_irq_init(GPIOC, 13, GPIO_TRIGGER_FALLING, 5);
 */
static inline void gpio_irq_init(GPIO_TypeDef *port, int pin,
                                 int trigger, uint32_t priority) {
    gpio_clk(port);

    /* Set pin to input mode (MODER = 0b00) */
    port->MODER &= ~(3UL << ((uint32_t)pin * 2U));

    /* Route GPIO port → EXTI line */
    gpio_exti_route(port, pin);

    /* Configure trigger edge */
    gpio_irq_set_trigger(pin, trigger);

    /* Clear any stale pending flag before arming */
    EXTI->PR = (1UL << (uint32_t)pin);

    /* Set NVIC priority, then enable NVIC + EXTI */
    NVIC_SetPriority(_gpio_exti_irqn(pin), priority);
    gpio_irq_enable(pin);
}


/* =========================================================================
   EXTI Event Mode  (WFE — wake CPU without vectoring to an ISR)
   Setting the EMR bit instead of IMR lets a pin edge wake the core from
   a WFE sleep without running an interrupt handler.
   ========================================================================= */

/**
 * gpio_event_enable
 * Enable WFE wake-on-event for a pin.
 * Does NOT touch the IMR or NVIC — no ISR will run.
 * Call gpio_irq_set_trigger() afterwards to change the default (RISING) edge.
 */
static inline void gpio_event_enable(int pin) {
    EXTI->EMR |= (1UL << (uint32_t)pin);
    gpio_irq_set_trigger(pin, GPIO_TRIGGER_RISING);
}

/**
 * gpio_event_disable
 * Disable WFE wake-on-event for a pin.
 */
static inline void gpio_event_disable(int pin) {
    EXTI->EMR &= ~(1UL << (uint32_t)pin);
}

/**
 * gpio_irq_event_enable
 * Enable BOTH interrupt (IMR) AND event (EMR) for a pin.
 * The CPU wakes from WFE AND runs the ISR when the edge occurs.
 */
static inline void gpio_irq_event_enable(int pin) {
    EXTI->IMR |= (1UL << (uint32_t)pin);
    EXTI->EMR |= (1UL << (uint32_t)pin);
}


/* =========================================================================
   Convenience Combo Functions
   ========================================================================= */

/**
 * gpio_irq_input
 * Configure a pin as a pulled digital input AND arm its EXTI interrupt
 * in a single call.
 *
 * @param port      GPIOx pointer
 * @param pin       Pin number (0-15)
 * @param trigger   GPIO_TRIGGER_RISING / FALLING / BOTH_EDGES
 * @param pull      GPIO_PULL_UP / GPIO_PULL_DOWN / GPIO_PULL_NONE
 * @param priority  NVIC priority (0 = highest)
 *
 * Example — user button on PC13, falling edge, pull-up, priority 5:
 *   gpio_irq_input(GPIOC, 13, GPIO_TRIGGER_FALLING, GPIO_PULL_UP, 5);
 */
static inline void gpio_irq_input(GPIO_TypeDef *port, int pin,
                                  int trigger, int pull,
                                  uint32_t priority) {
    gpio_init(port, pin, GPIO_MODE_INPUT, GPIO_OTYPE_PP,
              GPIO_SPEED_LOW, pull, 0);
    gpio_irq_init(port, pin, trigger, priority);
}

/**
 * gpio_output
 * Configure a pin as a push-pull output with medium speed (common case).
 *
 * Example:
 *   gpio_output(GPIOB, 5);   // PB5 → PP output, medium speed, no pull
 */
static inline void gpio_output(GPIO_TypeDef *port, int pin) {
    gpio_init(port, pin, GPIO_MODE_OUTPUT, GPIO_OTYPE_PP,
              GPIO_SPEED_MEDIUM, GPIO_PULL_NONE, 0);
}


#endif /* GPIO_H */