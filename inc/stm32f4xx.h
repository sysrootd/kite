#ifndef STM32F4XX_H
#define STM32F4XX_H

#include <stdint.h>

#define __CM4_REV              0x0001U
#define __MPU_PRESENT          1U
#define __NVIC_PRIO_BITS       4U
#define __Vendor_SysTickConfig 0U

typedef enum {
    NonMaskableInt_IRQn   = -14,
    MemoryManagement_IRQn = -12,
    BusFault_IRQn         = -11,
    UsageFault_IRQn       = -10,
    SVCall_IRQn           = -5,
    DebugMonitor_IRQn     = -4,
    PendSV_IRQn           = -2,
    SysTick_IRQn          = -1,
    WWDG_IRQn             = 0,
    PVD_IRQn              = 1,
    TAMP_STAMP_IRQn       = 2,
    RTC_WKUP_IRQn         = 3,
    FLASH_IRQn            = 4,
    RCC_IRQn              = 5,
    EXTI0_IRQn            = 6,
    EXTI1_IRQn            = 7,
    EXTI2_IRQn            = 8,
    EXTI3_IRQn            = 9,
    EXTI4_IRQn            = 10,
    DMA1_Stream0_IRQn     = 11,
    DMA1_Stream1_IRQn     = 12,
    DMA1_Stream2_IRQn     = 13,
    DMA1_Stream3_IRQn     = 14,
    DMA1_Stream4_IRQn     = 15,
    DMA1_Stream5_IRQn     = 16,
    DMA1_Stream6_IRQn     = 17,
    ADC_IRQn              = 18,
    EXTI9_5_IRQn          = 23,
    TIM1_BRK_TIM9_IRQn    = 24,
    TIM1_UP_TIM10_IRQn    = 25,
    TIM1_TRG_COM_TIM11_IRQn = 26,
    TIM1_CC_IRQn          = 27,
    TIM2_IRQn             = 28,
    TIM3_IRQn             = 29,
    TIM4_IRQn             = 30,
    I2C1_EV_IRQn          = 31,
    I2C1_ER_IRQn          = 32,
    I2C2_EV_IRQn          = 33,
    I2C2_ER_IRQn          = 34,
    SPI1_IRQn             = 35,
    SPI2_IRQn             = 36,
    USART1_IRQn           = 37,
    USART2_IRQn           = 38,
    EXTI15_10_IRQn        = 40,
    RTC_Alarm_IRQn        = 41,
    OTG_FS_WKUP_IRQn      = 42,
    DMA1_Stream7_IRQn     = 47,
    SDIO_IRQn             = 49,
    TIM5_IRQn             = 50,
    SPI3_IRQn             = 51,
    DMA2_Stream0_IRQn     = 56,
    DMA2_Stream1_IRQn     = 57,
    DMA2_Stream2_IRQn     = 58,
    DMA2_Stream3_IRQn     = 59,
    DMA2_Stream4_IRQn     = 60,
    OTG_FS_IRQn           = 67,
    DMA2_Stream5_IRQn     = 68,
    DMA2_Stream6_IRQn     = 69,
    DMA2_Stream7_IRQn     = 70,
    USART6_IRQn           = 71,
    I2C3_EV_IRQn          = 72,
    I2C3_ER_IRQn          = 73,
    FPU_IRQn              = 81,
    SPI4_IRQn             = 84
} IRQn_Type;

#include "core_cm4.h"

#define PERIPH_BASE           0x40000000UL
#define APB1PERIPH_BASE       PERIPH_BASE
#define APB2PERIPH_BASE       (PERIPH_BASE + 0x10000UL)
#define AHB1PERIPH_BASE       (PERIPH_BASE + 0x20000UL)
#define AHB2PERIPH_BASE       0x50000000UL

#define GPIOA_BASE            (AHB1PERIPH_BASE + 0x0000UL)
#define GPIOB_BASE            (AHB1PERIPH_BASE + 0x0400UL)
#define GPIOC_BASE            (AHB1PERIPH_BASE + 0x0800UL)
#define GPIOD_BASE            (AHB1PERIPH_BASE + 0x0C00UL)
#define GPIOE_BASE            (AHB1PERIPH_BASE + 0x1000UL)
#define GPIOH_BASE            (AHB1PERIPH_BASE + 0x1C00UL)
#define RCC_BASE              (AHB1PERIPH_BASE + 0x3800UL)
#define FLASH_R_BASE          0x40023C00UL

#define TIM2_BASE             (APB1PERIPH_BASE + 0x0000UL)
#define TIM3_BASE             (APB1PERIPH_BASE + 0x0400UL)
#define TIM4_BASE             (APB1PERIPH_BASE + 0x0800UL)
#define TIM5_BASE             (APB1PERIPH_BASE + 0x0C00UL)
#define USART2_BASE           (APB1PERIPH_BASE + 0x4400UL)
#define I2C1_BASE             (APB1PERIPH_BASE + 0x5400UL)
#define I2C2_BASE             (APB1PERIPH_BASE + 0x5800UL)
#define I2C3_BASE             (APB1PERIPH_BASE + 0x5C00UL)
#define PWR_BASE              (APB1PERIPH_BASE + 0x7000UL)
#define SPI2_BASE             (APB1PERIPH_BASE + 0x3800UL)

#define TIM1_BASE             (APB2PERIPH_BASE + 0x0000UL)
#define USART1_BASE           (APB2PERIPH_BASE + 0x1000UL)
#define USART6_BASE           (APB2PERIPH_BASE + 0x1400UL)
#define ADC1_BASE             (APB2PERIPH_BASE + 0x2000UL)
#define SDIO_BASE             (APB2PERIPH_BASE + 0x2C00UL)
#define SPI1_BASE             (APB2PERIPH_BASE + 0x3000UL)
#define SPI4_BASE             (APB2PERIPH_BASE + 0x3400UL)
#define SYSCFG_BASE           (APB2PERIPH_BASE + 0x3800UL)
#define EXTI_BASE             (APB2PERIPH_BASE + 0x3C00UL)
#define TIM9_BASE             (APB2PERIPH_BASE + 0x4000UL)
#define TIM10_BASE            (APB2PERIPH_BASE + 0x4400UL)
#define TIM11_BASE            (APB2PERIPH_BASE + 0x4800UL)

typedef struct {
    volatile uint32_t MODER;
    volatile uint32_t OTYPER;
    volatile uint32_t OSPEEDR;
    volatile uint32_t PUPDR;
    volatile uint32_t IDR;
    volatile uint32_t ODR;
    volatile uint32_t BSRR;
    volatile uint32_t LCKR;
    volatile uint32_t AFRL;
    volatile uint32_t AFRH;
} GPIO_TypeDef;

typedef struct {
    volatile uint32_t CR;
    volatile uint32_t PLLCFGR;
    volatile uint32_t CFGR;
    volatile uint32_t CIR;
    volatile uint32_t AHB1RSTR;
    volatile uint32_t AHB2RSTR;
    uint32_t RESERVED0[2];
    volatile uint32_t APB1RSTR;
    volatile uint32_t APB2RSTR;
    uint32_t RESERVED1[2];
    volatile uint32_t AHB1ENR;
    volatile uint32_t AHB2ENR;
    uint32_t RESERVED2[2];
    volatile uint32_t APB1ENR;
    volatile uint32_t APB2ENR;
    uint32_t RESERVED3[2];
    volatile uint32_t AHB1LPENR;
    volatile uint32_t AHB2LPENR;
    uint32_t RESERVED4[2];
    volatile uint32_t APB1LPENR;
    volatile uint32_t APB2LPENR;
    uint32_t RESERVED5[2];
    volatile uint32_t BDCR;
    volatile uint32_t CSR;
    uint32_t RESERVED6[2];
    volatile uint32_t SSCGR;
    volatile uint32_t PLLI2SCFGR;
} RCC_TypeDef;

typedef struct {
    volatile uint32_t IMR;
    volatile uint32_t EMR;
    volatile uint32_t RTSR;
    volatile uint32_t FTSR;
    volatile uint32_t SWIER;
    volatile uint32_t PR;
} EXTI_TypeDef;

typedef struct {
    volatile uint32_t MEMRMP;
    volatile uint32_t PMC;
    volatile uint32_t EXTICR[4];
    uint32_t RESERVED0[2];
    volatile uint32_t CMPCR;
} SYSCFG_TypeDef;

typedef struct {
    volatile uint32_t SR;
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t SMPR1;
    volatile uint32_t SMPR2;
    volatile uint32_t JOFR1;
    volatile uint32_t JOFR2;
    volatile uint32_t JOFR3;
    volatile uint32_t JOFR4;
    volatile uint32_t HTR;
    volatile uint32_t LTR;
    volatile uint32_t SQR1;
    volatile uint32_t SQR2;
    volatile uint32_t SQR3;
    volatile uint32_t JSQR;
    volatile uint32_t JDR1;
    volatile uint32_t JDR2;
    volatile uint32_t JDR3;
    volatile uint32_t JDR4;
    volatile uint32_t DR;
} ADC_TypeDef;

typedef struct {
    volatile uint32_t SR;
    volatile uint32_t DR;
    volatile uint32_t BRR;
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t CR3;
    volatile uint32_t GTPR;
} USART_TypeDef;

typedef struct {
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t SR;
    volatile uint32_t DR;
    volatile uint32_t CRCPR;
    volatile uint32_t RXCRCR;
    volatile uint32_t TXCRCR;
    volatile uint32_t I2SCFGR;
    volatile uint32_t I2SPR;
} SPI_TypeDef;

typedef struct {
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t OAR1;
    volatile uint32_t OAR2;
    volatile uint32_t DR;
    volatile uint32_t SR1;
    volatile uint32_t SR2;
    volatile uint32_t CCR;
    volatile uint32_t TRISE;
    volatile uint32_t FLTR;
} I2C_TypeDef;

typedef struct {
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t SMCR;
    volatile uint32_t DIER;
    volatile uint32_t SR;
    volatile uint32_t EGR;
    volatile uint32_t CCMR1;
    volatile uint32_t CCMR2;
    volatile uint32_t CCER;
    volatile uint32_t CNT;
    volatile uint32_t PSC;
    volatile uint32_t ARR;
    volatile uint32_t RCR;
    volatile uint32_t CCR1;
    volatile uint32_t CCR2;
    volatile uint32_t CCR3;
    volatile uint32_t CCR4;
    volatile uint32_t BDTR;
    volatile uint32_t DCR;
    volatile uint32_t DMAR;
} TIM_TypeDef;

#define GPIOA               ((GPIO_TypeDef *) GPIOA_BASE)
#define GPIOB               ((GPIO_TypeDef *) GPIOB_BASE)
#define GPIOC               ((GPIO_TypeDef *) GPIOC_BASE)
#define GPIOD               ((GPIO_TypeDef *) GPIOD_BASE)
#define GPIOE               ((GPIO_TypeDef *) GPIOE_BASE)
#define GPIOH               ((GPIO_TypeDef *) GPIOH_BASE)

#define RCC                 ((RCC_TypeDef *) RCC_BASE)
#define EXTI                ((EXTI_TypeDef *) EXTI_BASE)
#define SYSCFG              ((SYSCFG_TypeDef *) SYSCFG_BASE)

#define ADC1                ((ADC_TypeDef *) ADC1_BASE)

#define USART1              ((USART_TypeDef *) USART1_BASE)
#define USART2              ((USART_TypeDef *) USART2_BASE)
#define USART6              ((USART_TypeDef *) USART6_BASE)

#define SPI1                ((SPI_TypeDef *) SPI1_BASE)
#define SPI2                ((SPI_TypeDef *) SPI2_BASE)
#define SPI4                ((SPI_TypeDef *) SPI4_BASE)

#define I2C1                ((I2C_TypeDef *) I2C1_BASE)
#define I2C2                ((I2C_TypeDef *) I2C2_BASE)
#define I2C3                ((I2C_TypeDef *) I2C3_BASE)

#define TIM1                ((TIM_TypeDef *) TIM1_BASE)
#define TIM2                ((TIM_TypeDef *) TIM2_BASE)
#define TIM3                ((TIM_TypeDef *) TIM3_BASE)
#define TIM4                ((TIM_TypeDef *) TIM4_BASE)
#define TIM5                ((TIM_TypeDef *) TIM5_BASE)
#define TIM9                ((TIM_TypeDef *) TIM9_BASE)
#define TIM10               ((TIM_TypeDef *) TIM10_BASE)
#define TIM11               ((TIM_TypeDef *) TIM11_BASE)

#endif // STM32F4XX_H