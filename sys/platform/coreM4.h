#ifndef COREM4_H
#define COREM4_H

#include <stdint.h>

//--------------------------------M4 CMSIS definitions-------------------------------------
#define __CM4_REV              0x0001U
#define __MPU_PRESENT          1U
#define __NVIC_PRIO_BITS       4U
#define __Vendor_SysTickConfig 0U

typedef enum {
    NonMaskableInt_IRQn         = -14,
    MemoryManagement_IRQn       = -12,
    BusFault_IRQn               = -11,
    UsageFault_IRQn             = -10,
    SVCall_IRQn                 = -5,
    DebugMonitor_IRQn           = -4,
    PendSV_IRQn                 = -2,
    SysTick_IRQn                = -1,
    WWDG_IRQn                   = 0,
    PVD_IRQn                    = 1,
    TAMP_STAMP_IRQn             = 2,
    RTC_WKUP_IRQn               = 3,
    FLASH_IRQn                  = 4,
    RCC_IRQn                    = 5,
    EXTI0_IRQn                  = 6,
    EXTI1_IRQn                  = 7,
    EXTI2_IRQn                  = 8,
    EXTI3_IRQn                  = 9,
    EXTI4_IRQn                  = 10,
    DMA1_Stream0_IRQn           = 11,
    DMA1_Stream1_IRQn           = 12,
    DMA1_Stream2_IRQn           = 13,
    DMA1_Stream3_IRQn           = 14,
    DMA1_Stream4_IRQn           = 15,
    DMA1_Stream5_IRQn           = 16,
    DMA1_Stream6_IRQn           = 17,
    ADC_IRQn                    = 18,
    EXTI9_5_IRQn                = 23,
    TIM1_BRK_TIM9_IRQn          = 24,
    TIM1_UP_TIM10_IRQn          = 25,
    TIM1_TRG_COM_TIM11_IRQn     = 26,
    TIM1_CC_IRQn                = 27,
    TIM2_IRQn                   = 28,
    TIM3_IRQn                   = 29,
    TIM4_IRQn                   = 30,
    I2C1_EV_IRQn                = 31,
    I2C1_ER_IRQn                = 32,
    I2C2_EV_IRQn                = 33,
    I2C2_ER_IRQn                = 34,
    SPI1_IRQn                   = 35,
    SPI2_IRQn                   = 36,
    USART1_IRQn                 = 37,
    USART2_IRQn                 = 38,
    EXTI15_10_IRQn              = 40,
    RTC_Alarm_IRQn              = 41,
    OTG_FS_WKUP_IRQn            = 42,
    DMA1_Stream7_IRQn           = 47,
    SDIO_IRQn                   = 49,
    TIM5_IRQn                   = 50,
    SPI3_IRQn                   = 51,
    DMA2_Stream0_IRQn           = 56,
    DMA2_Stream1_IRQn           = 57,
    DMA2_Stream2_IRQn           = 58,
    DMA2_Stream3_IRQn           = 59,
    DMA2_Stream4_IRQn           = 60,
    OTG_FS_IRQn                 = 67,
    DMA2_Stream5_IRQn           = 68,
    DMA2_Stream6_IRQn           = 69,
    DMA2_Stream7_IRQn           = 70,
    USART6_IRQn                 = 71,
    I2C3_EV_IRQn                = 72,
    I2C3_ER_IRQn                = 73,
    FPU_IRQn                    = 81,
    SPI4_IRQn                   = 84
} IRQn_Type;

#ifndef __I
#define __I volatile const
#endif
#ifndef __O
#define __O volatile
#endif
#ifndef __IO
#define __IO volatile
#endif

typedef struct
{
    __IO uint32_t CTRL;
    __IO uint32_t LOAD;
    __IO uint32_t VAL;
    __I  uint32_t CALIB;
} SysTick_Type;

typedef struct
{
    __IO uint32_t ISER[8U];
         uint32_t RESERVED0[24U];
    __IO uint32_t ICER[8U];
         uint32_t RESERVED1[24U];
    __IO uint32_t ISPR[8U];
         uint32_t RESERVED2[24U];
    __IO uint32_t ICPR[8U];
         uint32_t RESERVED3[24U];
    __IO uint32_t IABR[8U];
         uint32_t RESERVED4[56U];
    __IO uint8_t  IP[240U];
         uint32_t RESERVED5[644U];
    __O  uint32_t STIR;
} NVIC_Type;

typedef struct
{
    __I  uint32_t CPUID;
    __IO uint32_t ICSR;
    __IO uint32_t VTOR;
    __IO uint32_t AIRCR;
    __IO uint32_t SCR;
    __IO uint32_t CCR;
    __IO uint32_t SHP[12U];
    __IO uint32_t SHCSR;
    __IO uint32_t CFSR;
    __IO uint32_t HFSR;
    __IO uint32_t DFSR;
    __IO uint32_t MMFAR;
    __IO uint32_t BFAR;
    __IO uint32_t AFSR;
    __I  uint32_t PFR[2U];
    __I  uint32_t DFR;
    __I  uint32_t ADR;
    __I  uint32_t MMFR[4U];
    __I  uint32_t ISAR[5U];
         uint32_t RESERVED0[5U];
    __IO uint32_t CPACR;
} SCB_Type;

#define DWT_BASE            (0xE0001000UL)
#define CoreDebug_BASE      (0xE000EDF0UL)
#define SysTick_BASE        (0xE000E010UL)
#define NVIC_BASE           (0xE000E100UL)
#define SCB_BASE            (0xE000ED00UL)

typedef struct {
    volatile uint32_t CTRL;
    volatile uint32_t CYCCNT;
    volatile uint32_t CPICNT;
    volatile uint32_t EXCCNT;
    volatile uint32_t SLEEPCNT;
    volatile uint32_t LSUCNT;
    volatile uint32_t FOLDCNT;
    volatile uint32_t PCSR;
} DWT_Type;

typedef struct {
    uint32_t RESERVED0[3];
    volatile uint32_t DEMCR;
} CoreDebug_Type;

#define DWT                 ((DWT_Type *) DWT_BASE)
#define CoreDebug           ((CoreDebug_Type *) CoreDebug_BASE)

#define CoreDebug_DEMCR_TRCENA_Msk (1UL << 24)
#define DWT_CTRL_CYCCNTENA_Msk      (1UL << 0)

#define SysTick             ((SysTick_Type *)SysTick_BASE)
#define NVIC                ((NVIC_Type *)NVIC_BASE)
#define SCB                 ((SCB_Type *)SCB_BASE)

#define SysTick_CTRL_ENABLE_Pos             0U
#define SysTick_CTRL_ENABLE_Msk             (1UL << SysTick_CTRL_ENABLE_Pos)
#define SysTick_CTRL_TICKINT_Pos            1U
#define SysTick_CTRL_TICKINT_Msk            (1UL << SysTick_CTRL_TICKINT_Pos)
#define SysTick_CTRL_CLKSOURCE_Pos          2U
#define SysTick_CTRL_CLKSOURCE_Msk          (1UL << SysTick_CTRL_CLKSOURCE_Pos)
#define SysTick_CTRL_COUNTFLAG_Pos          16U
#define SysTick_CTRL_COUNTFLAG_Msk          (1UL << SysTick_CTRL_COUNTFLAG_Pos)

#define SCB_ICSR_PENDSVSET_Pos              28U
#define SCB_ICSR_PENDSVSET_Msk              (1UL << SCB_ICSR_PENDSVSET_Pos)

#define SCB_SHCSR_MEMFAULTENA_Pos           16U
#define SCB_SHCSR_MEMFAULTENA_Msk           (1UL << SCB_SHCSR_MEMFAULTENA_Pos)
#define SCB_SHCSR_BUSFAULTENA_Pos           17U
#define SCB_SHCSR_BUSFAULTENA_Msk           (1UL << SCB_SHCSR_BUSFAULTENA_Pos)
#define SCB_SHCSR_USGFAULTENA_Pos           18U
#define SCB_SHCSR_USGFAULTENA_Msk           (1UL << SCB_SHCSR_USGFAULTENA_Pos)

#define SCB_SCR_SLEEPDEEP_Pos               2U
#define SCB_SCR_SLEEPDEEP_Msk               (1UL << SCB_SCR_SLEEPDEEP_Pos)

#define CPACR_CP10_POS       20U
#define CPACR_CP11_POS       22U
#define CPACR_CP10_CP11_FULL (0xF << CPACR_CP10_POS)

#define MPU_BASE             (0xE000ED90UL)
#define MPU_CTRL_ENABLE_Pos  0U
#define MPU_CTRL_ENABLE_Msk  (1UL << MPU_CTRL_ENABLE_Pos)
#define MPU_CTRL_PRIVDEFENA_Pos 2U
#define MPU_CTRL_PRIVDEFENA_Msk (1UL << MPU_CTRL_PRIVDEFENA_Pos)
#define MPU_RASR_ENABLE_Pos  0U
#define MPU_RASR_ENABLE_Msk  (1UL << MPU_RASR_ENABLE_Pos)
#define MPU_RASR_SIZE_Pos    1U
#define MPU_RASR_SIZE_Msk    (0x1FU << MPU_RASR_SIZE_Pos)
#define MPU_RASR_FULL_ACCESS (0x3U << 24)
#define MPU_RASR_AP_Pos               24U
#define MPU_RASR_AP_Msk               (0x7UL << MPU_RASR_AP_Pos)
#define MPU_RASR_XN_Pos               28U
#define MPU_RASR_XN_Msk               (0x1UL << MPU_RASR_XN_Pos)
#define MPU_RBAR_REGION_Pos           0U
#define MPU_RBAR_REGION_Msk           (0xFUL << MPU_RBAR_REGION_Pos)
#define MPU_RBAR_VALID_Pos            4U
#define MPU_RBAR_VALID_Msk            (0x1UL << MPU_RBAR_VALID_Pos)

typedef struct
{
    volatile uint32_t TYPE;
    volatile uint32_t CTRL;
    volatile uint32_t RNR;
    volatile uint32_t RBAR;
    volatile uint32_t RASR;
} MPU_TypeDef;

#define MPU ((MPU_TypeDef *)MPU_BASE)

static inline uint32_t __get_BASEPRI(void)
{
    uint32_t result;
    __asm volatile ("mrs %0, BASEPRI" : "=r" (result) );
    return result;
}

static inline void __set_BASEPRI(uint32_t value)
{
    __asm volatile ("msr BASEPRI, %0" : : "r" (value) : "memory");
}

static inline uint32_t __get_PRIMASK(void)
{
    uint32_t result;
    __asm volatile ("mrs %0, PRIMASK" : "=r" (result) );
    return result;
}

static inline void __set_PRIMASK(uint32_t value)
{
    __asm volatile ("msr PRIMASK, %0" : : "r" (value) : "memory");
}

static inline void __enable_irq(void)
{
    __asm volatile ("cpsie i" : : : "memory");
}

static inline void __disable_irq(void)
{
    __asm volatile ("cpsid i" : : : "memory");
}

static inline void __ISB(void)
{
    __asm volatile ("isb 0xF" : : : "memory");
}

static inline void instruction_synchronization_barrier(void)
{
    __ISB();
}

static inline void __DSB(void)
{
    __asm volatile ("dsb 0xF" : : : "memory");
}

static inline void data_synchronization_barrier(void)
{
    __DSB();
}

static inline void __NOP(void)
{
    __asm volatile ("nop");
}

static inline void __WFI(void)
{
    __asm volatile ("wfi");
}

static inline uint32_t __get_PSP(void)
{
    uint32_t result;
    __asm volatile ("MRS %0, psp" : "=r" (result));
    return result;
}

static inline void __set_PSP(uint32_t topOfProcStack)
{
    __asm volatile ("MSR psp, %0" : : "r" (topOfProcStack) : "sp");
}

static inline uint32_t __get_MSP(void)
{
    uint32_t result;
    __asm volatile ("MRS %0, msp" : "=r" (result));
    return result;
}

static inline void __set_MSP(uint32_t topOfMainStack)
{
    __asm volatile ("MSR msp, %0" : : "r" (topOfMainStack) : "sp");
}

static inline uint32_t __get_CONTROL(void)
{
    uint32_t result;
    __asm volatile ("MRS %0, control" : "=r" (result));
    return result;
}

static inline void __set_CONTROL(uint32_t control)
{
    __asm volatile ("MSR control, %0" : : "r" (control) : "memory");
    __ISB();
}

static inline void NVIC_EnableIRQ(IRQn_Type IRQn)
{
    if (IRQn < 32)
    {
        NVIC->ISER[0] = (1U << IRQn);
    }
    else if (IRQn < 64)
    {
        NVIC->ISER[1] = (1U << (IRQn - 32));
    }
    else
    {
        NVIC->ISER[2] = (1U << (IRQn - 64));
    }
}

static inline void NVIC_SetPriority(IRQn_Type IRQn, uint32_t priority)
{
    if ((int32_t)IRQn < 0)
    {
        SCB->SHP[((uint32_t)((int32_t)IRQn & 0xFUL)) - 4UL] =
            (uint8_t)((priority << (8U - __NVIC_PRIO_BITS)) & 0xFFUL);
    }
    else
    {
        NVIC->IP[(uint32_t)IRQn] =
            (uint8_t)((priority << (8U - __NVIC_PRIO_BITS)) & 0xFFUL);
    }
}

static inline uint32_t SysTick_Config(uint32_t ticks)
{
    if ((ticks - 1UL) > 0xFFFFFFUL)
    {
        return 1UL;
    }

    SysTick->LOAD = (uint32_t)(ticks - 1UL);
    SysTick->VAL  = 0UL;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |
                    SysTick_CTRL_TICKINT_Msk   |
                    SysTick_CTRL_ENABLE_Msk;

    return 0UL;
}

#endif // COREM4_H
