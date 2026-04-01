#include <stdint.h>
#include "stm32f4xx.h"

uint32_t SystemCoreClock;


//---------------------------------------IMPORTANT SECTION---------------------------------------
//Change here your mcu specific base(HSI clk) and its supported higest(PLL clk) freq

#define BASE_CLOCK_SPEED        16000000U
#define HIGHEST_CLOCK_SPEED     84000000U

//------------------------------------------------------------------------------------------------

#define CLOCK_SET_25_PERCENTAGE     BASE_CLOCK_SPEED
#define CLOCK_SET_50_PERCENTAGE     ((HIGHEST_CLOCK_SPEED * 50U) / 100U)
#define CLOCK_SET_75_PERCENTAGE     ((HIGHEST_CLOCK_SPEED * 75U) / 100U)
#define CLOCK_SET_100_PERCENTAGE    HIGHEST_CLOCK_SPEED

#define CLOCK_PROFILE_LOW       0U
#define CLOCK_PROFILE_MEDIUM    1U
#define CLOCK_PROFILE_HIGH      2U
#define CLOCK_PROFILE_MAX       3U

//select required clk speed
#define SYSTEM_CLOCK_PROFILE        CLOCK_PROFILE_MEDIUM

#define RCC_CFGR_SW_MASK        (0x3U << RCC_CFGR_SW_Pos)
#define RCC_CFGR_SWS_MASK       (0x3U << RCC_CFGR_SWS_Pos)

#define FLASH_ACR_LATENCY_0WS   (0x0U << FLASH_ACR_LATENCY_Pos)
#define FLASH_ACR_LATENCY_1WS   (0x1U << FLASH_ACR_LATENCY_Pos)
#define FLASH_ACR_LATENCY_2WS   (0x2U << FLASH_ACR_LATENCY_Pos)

#define RCC_CFGR_PPRE1_DIV1     (0x0U << 10)
#define RCC_CFGR_PPRE1_DIV2     (0x4U << 10)

#define PLLCFGR_PLLM(v)         ((uint32_t)(v) << 0)
#define PLLCFGR_PLLN(v)         ((uint32_t)(v) << 6)
#define PLLCFGR_PLLP_DIV2       (0x0U << 16)
#define PLLCFGR_PLLP_DIV4       (0x1U << 16)
#define PLLCFGR_PLLP_DIV6       (0x2U << 16)
#define PLLCFGR_PLLP_DIV8       (0x3U << 16)
#define PLLCFGR_PLLSRC_HSI      (0x0U)
#define PLLCFGR_PLLSRC_HSE      (0x1U << 22)

static void clock_reset_to_safe_hsi(void);
static void clock_profile_low(void);
static void clock_profile_medium(void);
static void clock_profile_high(void);
static void clock_profile_max(void);

int main(void);


//sysmbols from linker
extern uint32_t _sidata;
extern uint32_t _sdata;
extern uint32_t _edata;
extern uint32_t _sbss;
extern uint32_t _ebss;
extern uint32_t _estack;


//default handler for unimplimented handlers
void Default_Handler(void)
{
    while (1) {}
}


//core handlers 
void Reset_Handler(void);
void NMI_Handler(void)                    __attribute__((weak, alias("Default_Handler")));
void HardFault_Handler(void)              __attribute__((weak, alias("Default_Handler")));
void MemManage_Handler(void)              __attribute__((weak, alias("Default_Handler")));
void BusFault_Handler(void)               __attribute__((weak, alias("Default_Handler")));
void UsageFault_Handler(void)             __attribute__((weak, alias("Default_Handler")));
void SVC_Handler(void)                    __attribute__((weak, alias("Default_Handler")));
void DebugMon_Handler(void)               __attribute__((weak, alias("Default_Handler")));
void PendSV_Handler(void)                 __attribute__((weak, alias("Default_Handler")));
void SysTick_Handler(void)                __attribute__((weak, alias("Default_Handler")));
//mcu specific handlers(add or change as per mcu(by default its for stm32f401 series))
void WWDG_IRQHandler(void)                __attribute__((weak, alias("Default_Handler")));
void PVD_IRQHandler(void)                 __attribute__((weak, alias("Default_Handler")));
void TAMP_STAMP_IRQHandler(void)          __attribute__((weak, alias("Default_Handler")));
void RTC_WKUP_IRQHandler(void)            __attribute__((weak, alias("Default_Handler")));
void FLASH_IRQHandler(void)               __attribute__((weak, alias("Default_Handler")));
void RCC_IRQHandler(void)                 __attribute__((weak, alias("Default_Handler")));
void EXTI0_IRQHandler(void)               __attribute__((weak, alias("Default_Handler")));
void EXTI1_IRQHandler(void)               __attribute__((weak, alias("Default_Handler")));
void EXTI2_IRQHandler(void)               __attribute__((weak, alias("Default_Handler")));
void EXTI3_IRQHandler(void)               __attribute__((weak, alias("Default_Handler")));
void EXTI4_IRQHandler(void)               __attribute__((weak, alias("Default_Handler")));
void DMA1_Stream0_IRQHandler(void)        __attribute__((weak, alias("Default_Handler")));
void DMA1_Stream1_IRQHandler(void)        __attribute__((weak, alias("Default_Handler")));
void DMA1_Stream2_IRQHandler(void)        __attribute__((weak, alias("Default_Handler")));
void DMA1_Stream3_IRQHandler(void)        __attribute__((weak, alias("Default_Handler")));
void DMA1_Stream4_IRQHandler(void)        __attribute__((weak, alias("Default_Handler")));
void DMA1_Stream5_IRQHandler(void)        __attribute__((weak, alias("Default_Handler")));
void DMA1_Stream6_IRQHandler(void)        __attribute__((weak, alias("Default_Handler")));
void ADC_IRQHandler(void)                 __attribute__((weak, alias("Default_Handler")));
void EXTI9_5_IRQHandler(void)             __attribute__((weak, alias("Default_Handler")));
void TIM1_BRK_TIM9_IRQHandler(void)       __attribute__((weak, alias("Default_Handler")));
void TIM1_UP_TIM10_IRQHandler(void)       __attribute__((weak, alias("Default_Handler")));
void TIM1_TRG_COM_TIM11_IRQHandler(void)  __attribute__((weak, alias("Default_Handler")));
void TIM1_CC_IRQHandler(void)             __attribute__((weak, alias("Default_Handler")));
void TIM2_IRQHandler(void)                __attribute__((weak, alias("Default_Handler")));
void TIM3_IRQHandler(void)                __attribute__((weak, alias("Default_Handler")));
void TIM4_IRQHandler(void)                __attribute__((weak, alias("Default_Handler")));
void I2C1_EV_IRQHandler(void)             __attribute__((weak, alias("Default_Handler")));
void I2C1_ER_IRQHandler(void)             __attribute__((weak, alias("Default_Handler")));
void I2C2_EV_IRQHandler(void)             __attribute__((weak, alias("Default_Handler")));
void I2C2_ER_IRQHandler(void)             __attribute__((weak, alias("Default_Handler")));
void SPI1_IRQHandler(void)                __attribute__((weak, alias("Default_Handler")));
void SPI2_IRQHandler(void)                __attribute__((weak, alias("Default_Handler")));
void USART1_IRQHandler(void)              __attribute__((weak, alias("Default_Handler")));
void USART2_IRQHandler(void)              __attribute__((weak, alias("Default_Handler")));
void EXTI15_10_IRQHandler(void)           __attribute__((weak, alias("Default_Handler")));
void RTC_Alarm_IRQHandler(void)           __attribute__((weak, alias("Default_Handler")));
void OTG_FS_WKUP_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));
void DMA1_Stream7_IRQHandler(void)        __attribute__((weak, alias("Default_Handler")));
void SDIO_IRQHandler(void)                __attribute__((weak, alias("Default_Handler")));
void TIM5_IRQHandler(void)                __attribute__((weak, alias("Default_Handler")));
void SPI3_IRQHandler(void)                __attribute__((weak, alias("Default_Handler")));
void DMA2_Stream0_IRQHandler(void)        __attribute__((weak, alias("Default_Handler")));
void DMA2_Stream1_IRQHandler(void)        __attribute__((weak, alias("Default_Handler")));
void DMA2_Stream2_IRQHandler(void)        __attribute__((weak, alias("Default_Handler")));
void DMA2_Stream3_IRQHandler(void)        __attribute__((weak, alias("Default_Handler")));
void DMA2_Stream4_IRQHandler(void)        __attribute__((weak, alias("Default_Handler")));
void OTG_FS_IRQHandler(void)              __attribute__((weak, alias("Default_Handler")));
void DMA2_Stream5_IRQHandler(void)        __attribute__((weak, alias("Default_Handler")));
void DMA2_Stream6_IRQHandler(void)        __attribute__((weak, alias("Default_Handler")));
void DMA2_Stream7_IRQHandler(void)        __attribute__((weak, alias("Default_Handler")));
void USART6_IRQHandler(void)              __attribute__((weak, alias("Default_Handler")));
void I2C3_EV_IRQHandler(void)             __attribute__((weak, alias("Default_Handler")));
void I2C3_ER_IRQHandler(void)             __attribute__((weak, alias("Default_Handler")));
void FPU_IRQHandler(void)                 __attribute__((weak, alias("Default_Handler")));
void SPI4_IRQHandler(void)                __attribute__((weak, alias("Default_Handler")));

__attribute__((section(".isr_vector")))
void (* const g_pfnVectors[])(void) = {
    (void (*)(void))(&_estack),
    Reset_Handler,                      //core handlers
    NMI_Handler,
    HardFault_Handler,
    MemManage_Handler,
    BusFault_Handler,
    UsageFault_Handler,
    0, 0, 0, 0,
    SVC_Handler,
    DebugMon_Handler,
    0,
    PendSV_Handler,
    SysTick_Handler,                    //(core handlers end)total 10 core handlers i guess

    WWDG_IRQHandler,                    //(mcu handlers start) Don't know how may will be
    PVD_IRQHandler,                     //change and include as per your mcu specific(read stm reference manual!!) :)
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


//systeminit to config clk, don't touch any from here lol
void SystemInit(void)
{
    clock_reset_to_safe_hsi();

    switch (SYSTEM_CLOCK_PROFILE)
    {
        case CLOCK_PROFILE_LOW:
            clock_profile_low();
            break;

        case CLOCK_PROFILE_MEDIUM:
            clock_profile_medium();
            break;

        case CLOCK_PROFILE_HIGH:
            clock_profile_high();
            break;

        case CLOCK_PROFILE_MAX:
        default:
            clock_profile_max();
            break;
    }
}

static void clock_reset_to_safe_hsi(void)
{
    RCC->CR |= RCC_CR_HSION;
    while ((RCC->CR & RCC_CR_HSIRDY) == 0U);

    RCC->CFGR &= ~RCC_CFGR_SW_MASK;
    RCC->CFGR |= RCC_CFGR_SW_HSI;
    while ((RCC->CFGR & RCC_CFGR_SWS_MASK) != RCC_CFGR_SWS_HSI);

    RCC->CR &= ~RCC_CR_PLLON;
    while (RCC->CR & RCC_CR_PLLRDY);

    RCC->CFGR = 0x00000000U;
    RCC->PLLCFGR = 0x24003010U;

    FLASH->ACR =
        FLASH_ACR_LATENCY_0WS |
        FLASH_ACR_ICEN |
        FLASH_ACR_DCEN |
        FLASH_ACR_PRFTEN;

    SystemCoreClock = CLOCK_SET_25_PERCENTAGE;
}

static void clock_profile_low(void)
{
    SystemCoreClock = CLOCK_SET_25_PERCENTAGE;
}

static void clock_profile_medium(void)
{
    FLASH->ACR =
        FLASH_ACR_LATENCY_1WS |
        FLASH_ACR_ICEN |
        FLASH_ACR_DCEN |
        FLASH_ACR_PRFTEN;

    RCC->CFGR = 0x00000000U;

    RCC->PLLCFGR =
        PLLCFGR_PLLM(16U)     |
        PLLCFGR_PLLN(336U)    |
        PLLCFGR_PLLP_DIV8     |
        PLLCFGR_PLLSRC_HSI;

    RCC->CR |= RCC_CR_PLLON;
    while ((RCC->CR & RCC_CR_PLLRDY) == 0U);

    RCC->CFGR &= ~RCC_CFGR_SW_MASK;
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS_MASK) != RCC_CFGR_SWS_PLL);

    SystemCoreClock = CLOCK_SET_50_PERCENTAGE;
}

static void clock_profile_high(void)
{
    FLASH->ACR =
        FLASH_ACR_LATENCY_2WS |
        FLASH_ACR_ICEN |
        FLASH_ACR_DCEN |
        FLASH_ACR_PRFTEN;

    RCC->CFGR = RCC_CFGR_PPRE1_DIV2;

    RCC->PLLCFGR =
        PLLCFGR_PLLM(16U)     |
        PLLCFGR_PLLN(256U)    |
        PLLCFGR_PLLP_DIV4     |
        PLLCFGR_PLLSRC_HSI;

    RCC->CR |= RCC_CR_PLLON;
    while ((RCC->CR & RCC_CR_PLLRDY) == 0U);

    RCC->CFGR &= ~RCC_CFGR_SW_MASK;
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS_MASK) != RCC_CFGR_SWS_PLL);

    SystemCoreClock = CLOCK_SET_75_PERCENTAGE;
}

static void clock_profile_max(void)
{
    FLASH->ACR =
        FLASH_ACR_LATENCY_2WS |
        FLASH_ACR_ICEN |
        FLASH_ACR_DCEN |
        FLASH_ACR_PRFTEN;

    RCC->CFGR = RCC_CFGR_PPRE1_DIV2;

    RCC->PLLCFGR =
        PLLCFGR_PLLM(16U)     |
        PLLCFGR_PLLN(336U)    |
        PLLCFGR_PLLP_DIV4     |
        PLLCFGR_PLLSRC_HSI;

    RCC->CR |= RCC_CR_PLLON;
    while ((RCC->CR & RCC_CR_PLLRDY) == 0U);

    RCC->CFGR &= ~RCC_CFGR_SW_MASK;
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS_MASK) != RCC_CFGR_SWS_PLL);

    SystemCoreClock = CLOCK_SET_100_PERCENTAGE;
}

void Reset_Handler(void)
{
    uint32_t *src;
    uint32_t *dst;

    src = &_sidata;
    for (dst = &_sdata; dst < &_edata; )
    {
        *dst++ = *src++;
    }

    for (dst = &_sbss; dst < &_ebss; )
    {
        *dst++ = 0U;
    }

    SystemInit();
    main();  //haa.. finally!! call main

    while (1) {}
}