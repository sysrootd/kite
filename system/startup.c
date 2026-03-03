// Startup code for STM32F401x bare-metal with configurable clock

#include <stdint.h>
#include "stm32f4xx.h"

// CMSIS clock variable (normally provided by system_*.c in HAL projects)
uint32_t SystemCoreClock = 16000000U;  // Default reset value (HSI), updated by SystemInit()

// Main function
int main(void);

// Symbols defined in linker script
extern uint32_t _sidata;      // Start of initialized data in FLASH
extern uint32_t _sdata;       // Start of data section in RAM
extern uint32_t _edata;       // End of data section in RAM
extern uint32_t _sbss;        // Start of BSS section in RAM
extern uint32_t _ebss;        // End of BSS section in RAM
extern uint32_t _estack;      // Top of stack (from linker script)
extern uint32_t _sram_start;  // Start of SRAM (from linker script)
extern uint32_t _sram_size;   // Total SRAM size (from linker script)

// Linker-provided heap/stack size symbols used by runtime
extern uint32_t _Min_Heap_Size;   // Configured minimum heap size
extern uint32_t _Min_Stack_Size;  // Configured minimum stack size

// RCC_CR bits for clock configuration
#define RCC_CR_HSION    (1U << 0)
#define RCC_CR_HSIRDY   (1U << 1)
#define RCC_CR_PLLON    (1U << 24)
#define RCC_CR_PLLRDY   (1U << 25)

// RCC_CFGR bits for clock source selection
#define RCC_CFGR_SW_HSI 0x0U
#define RCC_CFGR_SW_PLL 0x2U
#define RCC_CFGR_SWS    (3U << 2)
#define RCC_CFGR_SWS_PLL (0x8U)

// FLASH access control register bits
#define FLASH_ACR_LATENCY_Pos 0
#define FLASH_ACR_ICEN  (1U << 9)
#define FLASH_ACR_DCEN  (1U << 10)
#define FLASH_ACR_PRFTEN (1U << 8)

// Clock configuration options
#define SYSCLK_HSI_16MHZ   0
#define SYSCLK_PLL_84MHZ   1
#define SYSCLK_PLL_50MHZ   2

#ifndef SYSCLK_CONFIG
#define SYSCLK_CONFIG SYSCLK_HSI_16MHZ
#endif

// Default handler for unimplemented exceptions
void Default_Handler(void) {
    while (1) {}
}

// Cortex-M4 core handlers
void Reset_Handler(void);
void NMI_Handler(void)                   __attribute__((weak, alias("Default_Handler")));
void HardFault_Handler(void)             __attribute__((weak, alias("Default_Handler")));
void MemManage_Handler(void)             __attribute__((weak, alias("Default_Handler")));
void BusFault_Handler(void)              __attribute__((weak, alias("Default_Handler")));
void UsageFault_Handler(void)            __attribute__((weak, alias("Default_Handler")));
void SVC_Handler(void)                   __attribute__((weak, alias("Default_Handler")));
void DebugMon_Handler(void)              __attribute__((weak, alias("Default_Handler")));
void PendSV_Handler(void)                __attribute__((weak, alias("Default_Handler")));
void SysTick_Handler(void)               __attribute__((weak, alias("Default_Handler")));

// External interrupt handlers
void WWDG_IRQHandler(void)              __attribute__((weak, alias("Default_Handler")));
void PVD_IRQHandler(void)               __attribute__((weak, alias("Default_Handler")));
void TAMP_STAMP_IRQHandler(void)        __attribute__((weak, alias("Default_Handler")));
void RTC_WKUP_IRQHandler(void)          __attribute__((weak, alias("Default_Handler")));
void FLASH_IRQHandler(void)             __attribute__((weak, alias("Default_Handler")));
void RCC_IRQHandler(void)               __attribute__((weak, alias("Default_Handler")));
void EXTI0_IRQHandler(void)             __attribute__((weak, alias("Default_Handler")));
void EXTI1_IRQHandler(void)             __attribute__((weak, alias("Default_Handler")));
void EXTI2_IRQHandler(void)             __attribute__((weak, alias("Default_Handler")));
void EXTI3_IRQHandler(void)             __attribute__((weak, alias("Default_Handler")));
void EXTI4_IRQHandler(void)             __attribute__((weak, alias("Default_Handler")));
void DMA1_Stream0_IRQHandler(void)      __attribute__((weak, alias("Default_Handler")));
void DMA1_Stream1_IRQHandler(void)      __attribute__((weak, alias("Default_Handler")));
void DMA1_Stream2_IRQHandler(void)      __attribute__((weak, alias("Default_Handler")));
void DMA1_Stream3_IRQHandler(void)      __attribute__((weak, alias("Default_Handler")));
void DMA1_Stream4_IRQHandler(void)      __attribute__((weak, alias("Default_Handler")));
void DMA1_Stream5_IRQHandler(void)      __attribute__((weak, alias("Default_Handler")));
void DMA1_Stream6_IRQHandler(void)      __attribute__((weak, alias("Default_Handler")));
void ADC_IRQHandler(void)               __attribute__((weak, alias("Default_Handler")));
void EXTI9_5_IRQHandler(void)           __attribute__((weak, alias("Default_Handler")));
void TIM1_BRK_TIM9_IRQHandler(void)     __attribute__((weak, alias("Default_Handler")));
void TIM1_UP_TIM10_IRQHandler(void)     __attribute__((weak, alias("Default_Handler")));
void TIM1_TRG_COM_TIM11_IRQHandler(void)__attribute__((weak, alias("Default_Handler")));
void TIM1_CC_IRQHandler(void)           __attribute__((weak, alias("Default_Handler")));
void TIM2_IRQHandler(void)              __attribute__((weak, alias("Default_Handler")));
void TIM3_IRQHandler(void)              __attribute__((weak, alias("Default_Handler")));
void TIM4_IRQHandler(void)              __attribute__((weak, alias("Default_Handler")));
void I2C1_EV_IRQHandler(void)           __attribute__((weak, alias("Default_Handler")));
void I2C1_ER_IRQHandler(void)           __attribute__((weak, alias("Default_Handler")));
void I2C2_EV_IRQHandler(void)           __attribute__((weak, alias("Default_Handler")));
void I2C2_ER_IRQHandler(void)           __attribute__((weak, alias("Default_Handler")));
void SPI1_IRQHandler(void)              __attribute__((weak, alias("Default_Handler")));
void SPI2_IRQHandler(void)              __attribute__((weak, alias("Default_Handler")));
void USART1_IRQHandler(void)            __attribute__((weak, alias("Default_Handler")));
void USART2_IRQHandler(void)            __attribute__((weak, alias("Default_Handler")));
void EXTI15_10_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));
void RTC_Alarm_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));
void OTG_FS_WKUP_IRQHandler(void)       __attribute__((weak, alias("Default_Handler")));
void DMA1_Stream7_IRQHandler(void)      __attribute__((weak, alias("Default_Handler")));
void SDIO_IRQHandler(void)              __attribute__((weak, alias("Default_Handler")));
void TIM5_IRQHandler(void)              __attribute__((weak, alias("Default_Handler")));
void SPI3_IRQHandler(void)              __attribute__((weak, alias("Default_Handler")));
void DMA2_Stream0_IRQHandler(void)      __attribute__((weak, alias("Default_Handler")));
void DMA2_Stream1_IRQHandler(void)      __attribute__((weak, alias("Default_Handler")));
void DMA2_Stream2_IRQHandler(void)      __attribute__((weak, alias("Default_Handler")));
void DMA2_Stream3_IRQHandler(void)      __attribute__((weak, alias("Default_Handler")));
void DMA2_Stream4_IRQHandler(void)      __attribute__((weak, alias("Default_Handler")));
void OTG_FS_IRQHandler(void)            __attribute__((weak, alias("Default_Handler")));
void DMA2_Stream5_IRQHandler(void)      __attribute__((weak, alias("Default_Handler")));
void DMA2_Stream6_IRQHandler(void)      __attribute__((weak, alias("Default_Handler")));
void DMA2_Stream7_IRQHandler(void)      __attribute__((weak, alias("Default_Handler")));
void USART6_IRQHandler(void)            __attribute__((weak, alias("Default_Handler")));
void I2C3_EV_IRQHandler(void)           __attribute__((weak, alias("Default_Handler")));
void I2C3_ER_IRQHandler(void)           __attribute__((weak, alias("Default_Handler")));
void FPU_IRQHandler(void)               __attribute__((weak, alias("Default_Handler")));
void SPI4_IRQHandler(void)              __attribute__((weak, alias("Default_Handler")));


// Vector table
__attribute__((section(".isr_vector")))
void (* const g_pfnVectors[])(void) = {
    (void (*)(void))(&_estack),
    Reset_Handler,
    NMI_Handler,
    HardFault_Handler,
    MemManage_Handler,
    BusFault_Handler,
    UsageFault_Handler,
    0,0,0,0,
    SVC_Handler,
    DebugMon_Handler,
    0,
    PendSV_Handler,
    SysTick_Handler,
    // External interrupts
    WWDG_IRQHandler,
    PVD_IRQHandler,
    TAMP_STAMP_IRQHandler,
    RTC_WKUP_IRQHandler,
    FLASH_IRQHandler,
    RCC_IRQHandler,
    EXTI0_IRQHandler,
    EXTI1_IRQHandler,
    EXTI2_IRQHandler,
    EXTI3_IRQHandler,
    EXTI4_IRQHandler,
    DMA1_Stream0_IRQHandler,
    DMA1_Stream1_IRQHandler,
    DMA1_Stream2_IRQHandler,
    DMA1_Stream3_IRQHandler,
    DMA1_Stream4_IRQHandler,
    DMA1_Stream5_IRQHandler,
    DMA1_Stream6_IRQHandler,
    ADC_IRQHandler,
    EXTI9_5_IRQHandler,
    TIM1_BRK_TIM9_IRQHandler,
    TIM1_UP_TIM10_IRQHandler,
    TIM1_TRG_COM_TIM11_IRQHandler,
    TIM1_CC_IRQHandler,
    TIM2_IRQHandler,
    TIM3_IRQHandler,
    TIM4_IRQHandler,
    I2C1_EV_IRQHandler,
    I2C1_ER_IRQHandler,
    I2C2_EV_IRQHandler,
    I2C2_ER_IRQHandler,
    SPI1_IRQHandler,
    SPI2_IRQHandler,
    USART1_IRQHandler,
    USART2_IRQHandler,
    EXTI15_10_IRQHandler,
    RTC_Alarm_IRQHandler,
    OTG_FS_WKUP_IRQHandler,
    DMA1_Stream7_IRQHandler,
    SDIO_IRQHandler,
    TIM5_IRQHandler,
    SPI3_IRQHandler,
    DMA2_Stream0_IRQHandler,
    DMA2_Stream1_IRQHandler,
    DMA2_Stream2_IRQHandler,
    DMA2_Stream3_IRQHandler,
    DMA2_Stream4_IRQHandler,
    OTG_FS_IRQHandler,
    DMA2_Stream5_IRQHandler,
    DMA2_Stream6_IRQHandler,
    DMA2_Stream7_IRQHandler,
    USART6_IRQHandler,
    I2C3_EV_IRQHandler,
    I2C3_ER_IRQHandler,
    FPU_IRQHandler,
    SPI4_IRQHandler,
};

void SystemInit(void) {

#if SYSCLK_CONFIG == SYSCLK_HSI_16MHZ
    // Run from HSI (16 MHz)
    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY));

    RCC->CFGR &= ~0x3U;
    RCC->CFGR |= RCC_CFGR_SW_HSI;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SW_HSI);

    // Update CMSIS variable
    SystemCoreClock = 16000000U;

#elif SYSCLK_CONFIG == SYSCLK_PLL_84MHZ
    // Configure PLL for 84 MHz
    // Disable PLL
    RCC->CR &= ~RCC_CR_PLLON;
    // Wait until PLL turned off; use register pointer not macro alone
    while (RCC->CR & RCC_CR_PLLRDY);

    // PLLM=16, PLLN=336, PLLP=4, PLLQ=7
    RCC->PLLCFGR = (16U << 0)   |   // PLLM
                  (336U << 6)  |   // PLLN
                  (1U << 16)   |   // PLLP=4 (01)
                  (7U << 24)   |   // PLLQ
                  (0U << 22);      // HSI as source

    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));

    // Flash wait states: 2WS for 84MHz
    FLASH->ACR = (2U << FLASH_ACR_LATENCY_Pos) |
                FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_PRFTEN;

    RCC->CFGR &= ~0x3U;
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);

    // Update CMSIS variable
    SystemCoreClock = 84000000U;

#elif SYSCLK_CONFIG == SYSCLK_PLL_50MHZ
    // Configure PLL for ~50 MHz
    // Disable PLL
    RCC->CR &= ~RCC_CR_PLLON;
    while (RCC->CR & RCC_CR_PLLRDY);

    // PLLM=16, PLLN=200, PLLP=4 -> 200/4=50 MHz
    RCC->PLLCFGR = (16U << 0)   |   // PLLM
                  (200U << 6)  |   // PLLN
                  (1U << 16)   |   // PLLP=4
                  (4U << 24)   |   // PLLQ=4
                  (0U << 22);      // HSI source

    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));

    // Flash wait states: 1WS for 50MHz
    FLASH->ACR = (1U << FLASH_ACR_LATENCY_Pos) |
                FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_PRFTEN;

    RCC->CFGR &= ~0x3U;
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);

    // Update CMSIS variable
    SystemCoreClock = 50000000U;

#else
# error "Invalid SYSCLK_CONFIG value!"
#endif
}

void Reset_Handler(void) {

    uint32_t *src, *dst;

    // Copy .data from Flash to SRAM
    src = &_sidata;
    for (dst = &_sdata; dst < &_edata; )
        *(dst++) = *(src++);

    // Zero-fill .bss
    for (dst = &_sbss; dst < &_ebss; )
        *(dst++) = 0;

    SystemInit();

    main();

    while (1) {}
}
