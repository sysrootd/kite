#include <stdint.h>
#include "mcu.h"

#define CLOCK_SET_25_PERCENTAGE     BASE_CLOCK_SPEED
#define CLOCK_SET_50_PERCENTAGE     ((HIGHEST_CLOCK_SPEED * 50U) / 100U)
#define CLOCK_SET_75_PERCENTAGE     ((HIGHEST_CLOCK_SPEED * 75U) / 100U)
#define CLOCK_SET_100_PERCENTAGE    HIGHEST_CLOCK_SPEED

#define CLOCK_PROFILE_LOW       0U
#define CLOCK_PROFILE_MEDIUM    1U
#define CLOCK_PROFILE_HIGH      2U
#define CLOCK_PROFILE_MAX      3U

#include "../app/config.h"

#ifndef ENABLE_FPU
#define ENABLE_FPU 0
#endif

#ifndef ENABLE_MPU
#define ENABLE_MPU 0
#endif

uint32_t SystemCoreClock;

extern uint32_t _sidata;
extern uint32_t _sdata;
extern uint32_t _edata;
extern uint32_t _sbss;
extern uint32_t _ebss;

extern void __libc_init_array(void);
extern int main(void);
extern void SystemInit(void);

static void enable_fpu(void);
static void enable_mpu(void);

void Reset_Handler(void)
{
    uint32_t *src = &_sidata;
    uint32_t *dst = &_sdata;

    while (dst < &_edata)
    {
        *dst++ = *src++;
    }

    dst = &_sbss;
    while (dst < &_ebss)
    {
        *dst++ = 0;
    }

    SystemInit();
    __libc_init_array();
    main();

    while (1)
    {
    }
}

//------------------------------------------------------------------------------------------------

#define RCC_CFGR_SW_MASK        (0x3U << RCC_CFGR_SW_Pos)
#define RCC_CFGR_SWS_MASK       (0x3U << RCC_CFGR_SWS_Pos)

#define RCC_CFGR_HPRE_Pos       4U
#define RCC_CFGR_HPRE_Msk       (0xFU << RCC_CFGR_HPRE_Pos)
#define RCC_CFGR_PPRE1_Pos      10U
#define RCC_CFGR_PPRE1_Msk      (0x7U << RCC_CFGR_PPRE1_Pos)
#define RCC_CFGR_PPRE2_Pos      13U
#define RCC_CFGR_PPRE2_Msk      (0x7U << RCC_CFGR_PPRE2_Pos)

#define RCC_CFGR_PPRE1_DIV1     (0x0U << RCC_CFGR_PPRE1_Pos)
#define RCC_CFGR_PPRE1_DIV2     (0x4U << RCC_CFGR_PPRE1_Pos)

#define FLASH_ACR_LATENCY_0WS   (0x0U << FLASH_ACR_LATENCY_Pos)
#define FLASH_ACR_LATENCY_1WS   (0x1U << FLASH_ACR_LATENCY_Pos)
#define FLASH_ACR_LATENCY_2WS   (0x2U << FLASH_ACR_LATENCY_Pos)

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
static void SystemCoreClockUpdate(void);
static uint32_t get_pll_output_freq(void);
static uint32_t get_ahb_prescaler(void);
static uint32_t pll_p_divider(uint32_t pllcfgr);

//systeminit to config clk
void SystemInit(void)
{
#if ENABLE_FPU
    enable_fpu();
#endif

#if ENABLE_MPU
    enable_mpu();
#endif

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

    SystemCoreClockUpdate();
}

static void enable_fpu(void)
{
    SCB->CPACR |= CPACR_CP10_CP11_FULL;
    data_synchronization_barrier();
    instruction_synchronization_barrier();
}

static void enable_mpu(void)
{
    MPU->CTRL = 0U;
    data_synchronization_barrier();
    instruction_synchronization_barrier();

    MPU->RNR = 0U;
    MPU->RBAR = 0x00000000U;
    MPU->RASR = MPU_RASR_ENABLE_Msk |
                MPU_RASR_FULL_ACCESS |
                (0x1FU << MPU_RASR_SIZE_Pos);

    data_synchronization_barrier();
    instruction_synchronization_barrier();

    MPU->CTRL = MPU_CTRL_PRIVDEFENA_Msk | MPU_CTRL_ENABLE_Msk;
    data_synchronization_barrier();
    instruction_synchronization_barrier();
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

    RCC->CFGR &= ~(RCC_CFGR_PPRE1_Msk | RCC_CFGR_PPRE2_Msk | RCC_CFGR_HPRE_Msk | RCC_CFGR_SW_MASK);

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
}

static void clock_profile_high(void)
{
    FLASH->ACR =
        FLASH_ACR_LATENCY_2WS |
        FLASH_ACR_ICEN |
        FLASH_ACR_DCEN |
        FLASH_ACR_PRFTEN;

    RCC->CFGR &= ~(RCC_CFGR_PPRE2_Msk | RCC_CFGR_HPRE_Msk | RCC_CFGR_SW_MASK);
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;

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
}

static void clock_profile_max(void)
{
    FLASH->ACR =
        FLASH_ACR_LATENCY_2WS |
        FLASH_ACR_ICEN |
        FLASH_ACR_DCEN |
        FLASH_ACR_PRFTEN;

    RCC->CFGR &= ~(RCC_CFGR_PPRE2_Msk | RCC_CFGR_HPRE_Msk | RCC_CFGR_SW_MASK);
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;

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
}

static uint32_t pll_p_divider(uint32_t pllcfgr)
{
    uint32_t pllp = (pllcfgr >> 16) & 0x3U;

    switch (pllp)
    {
        case 0U: return 2U;
        case 1U: return 4U;
        case 2U: return 6U;
        default: return 8U;
    }
}

static uint32_t get_pll_output_freq(void)
{
    uint32_t pllcfgr = RCC->PLLCFGR;
    uint32_t pllm = pllcfgr & 0x3FU;
    uint32_t plln = (pllcfgr >> 6) & 0x1FFU;

    if (pllm == 0U)
        return BASE_CLOCK_SPEED;

    return (BASE_CLOCK_SPEED / pllm) * plln / pll_p_divider(pllcfgr);
}

static uint32_t get_ahb_prescaler(void)
{
    uint32_t hpre = (RCC->CFGR & RCC_CFGR_HPRE_Msk) >> RCC_CFGR_HPRE_Pos;

    if (hpre < 8U)
        return 1U;

    switch (hpre)
    {
        case 8U:  return 2U;
        case 9U:  return 4U;
        case 10U: return 8U;
        case 11U: return 16U;
        case 12U: return 64U;
        case 13U: return 128U;
        case 14U: return 256U;
        default:  return 512U;
    }
}

static void SystemCoreClockUpdate(void)
{
    uint32_t sysclk = BASE_CLOCK_SPEED;
    uint32_t sws = RCC->CFGR & RCC_CFGR_SWS_MASK;

    if (sws == RCC_CFGR_SWS_PLL)
    {
        sysclk = get_pll_output_freq();
    }

    SystemCoreClock = sysclk / get_ahb_prescaler();
}
