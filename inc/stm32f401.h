#ifndef STM32F401_H
#define STM32F401_H

#include <stdint.h>

/* =====================================================================
 *   Base Addresses
 * ===================================================================== */
#define PERIPH_BASE           0x40000000UL
#define APB1PERIPH_BASE       PERIPH_BASE
#define APB2PERIPH_BASE       (PERIPH_BASE + 0x10000UL)
#define AHB1PERIPH_BASE       (PERIPH_BASE + 0x20000UL)
#define AHB2PERIPH_BASE       0x50000000UL

/* --- AHB1 --- */
#define GPIOA_BASE            (AHB1PERIPH_BASE + 0x0000UL)
#define GPIOB_BASE            (AHB1PERIPH_BASE + 0x0400UL)
#define GPIOC_BASE            (AHB1PERIPH_BASE + 0x0800UL)
#define GPIOD_BASE            (AHB1PERIPH_BASE + 0x0C00UL)
#define GPIOE_BASE            (AHB1PERIPH_BASE + 0x1000UL)
#define GPIOH_BASE            (AHB1PERIPH_BASE + 0x1C00UL)
#define RCC_BASE              (AHB1PERIPH_BASE + 0x3800UL)
#define FLASH_R_BASE          0x40023C00UL

/* --- APB1 --- */
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

/* --- APB2 --- */
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

/* =====================================================================
 *   Peripheral Structs
 * ===================================================================== */

/* GPIO */
typedef struct {
    volatile uint32_t MODER;    /*!< 0x00 */
    volatile uint32_t OTYPER;   /*!< 0x04 */
    volatile uint32_t OSPEEDR;  /*!< 0x08 */
    volatile uint32_t PUPDR;    /*!< 0x0C */
    volatile uint32_t IDR;      /*!< 0x10 */
    volatile uint32_t ODR;      /*!< 0x14 */
    volatile uint32_t BSRR;     /*!< 0x18 */
    volatile uint32_t LCKR;     /*!< 0x1C */
    volatile uint32_t AFRL;     /*!< 0x20 */
    volatile uint32_t AFRH;     /*!< 0x24 */
} GPIO_TypeDef;

/* RCC */
typedef struct {
    volatile uint32_t CR;           /*!< 0x00 */
    volatile uint32_t PLLCFGR;      /*!< 0x04 */
    volatile uint32_t CFGR;         /*!< 0x08 */
    volatile uint32_t CIR;          /*!< 0x0C */
    volatile uint32_t AHB1RSTR;     /*!< 0x10 */
    volatile uint32_t AHB2RSTR;     /*!< 0x14 */
    uint32_t RESERVED0[2];          /*!< 0x18,0x1C */
    volatile uint32_t APB1RSTR;     /*!< 0x20 */
    volatile uint32_t APB2RSTR;     /*!< 0x24 */
    uint32_t RESERVED1[2];          /*!< 0x28,0x2C */
    volatile uint32_t AHB1ENR;      /*!< 0x30 */
    volatile uint32_t AHB2ENR;      /*!< 0x34 */
    uint32_t RESERVED2[2];          /*!< 0x38,0x3C */
    volatile uint32_t APB1ENR;      /*!< 0x40 */
    volatile uint32_t APB2ENR;      /*!< 0x44 */
    uint32_t RESERVED3[2];          /*!< 0x48,0x4C */
    volatile uint32_t AHB1LPENR;    /*!< 0x50 */
    volatile uint32_t AHB2LPENR;    /*!< 0x54 */
    uint32_t RESERVED4[2];          /*!< 0x58,0x5C */
    volatile uint32_t APB1LPENR;    /*!< 0x60 */
    volatile uint32_t APB2LPENR;    /*!< 0x64 */
    uint32_t RESERVED5[2];          /*!< 0x68,0x6C */
    volatile uint32_t BDCR;         /*!< 0x70 */
    volatile uint32_t CSR;          /*!< 0x74 */
    uint32_t RESERVED6[2];          /*!< 0x78,0x7C */
    volatile uint32_t SSCGR;        /*!< 0x80 */
    volatile uint32_t PLLI2SCFGR;   /*!< 0x84 */
} RCC_TypeDef;

/* EXTI */
typedef struct {
    volatile uint32_t IMR;
    volatile uint32_t EMR;
    volatile uint32_t RTSR;
    volatile uint32_t FTSR;
    volatile uint32_t SWIER;
    volatile uint32_t PR;
} EXTI_TypeDef;

/* SYSCFG */
typedef struct {
    volatile uint32_t MEMRMP;
    volatile uint32_t PMC;
    volatile uint32_t EXTICR[4];
    uint32_t RESERVED0[2];
    volatile uint32_t CMPCR;
} SYSCFG_TypeDef;

/* ADC */
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

/* USART */
typedef struct {
    volatile uint32_t SR;
    volatile uint32_t DR;
    volatile uint32_t BRR;
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t CR3;
    volatile uint32_t GTPR;
} USART_TypeDef;

/* SPI */
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

/* I2C */
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

/* TIM */
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

/* =====================================================================
 *   Cortex-M4 Core Registers (NVIC, SysTick, SCB)
 * ===================================================================== */

#define SCS_BASE             (0xE000E000UL)
#define SysTick_BASE         (SCS_BASE + 0x0010UL)
#define NVIC_BASE            (SCS_BASE + 0x0100UL)
#define SCB_BASE             (SCS_BASE + 0x0D00UL)

/* SysTick */
typedef struct {
    volatile uint32_t CTRL;
    volatile uint32_t LOAD;
    volatile uint32_t VAL;
    volatile uint32_t CALIB;
} SysTick_Type;

/* NVIC */
typedef struct {
    volatile uint32_t ISER[8];
    uint32_t RESERVED0[24];
    volatile uint32_t ICER[8];
    uint32_t RESERVED1[24];
    volatile uint32_t ISPR[8];
    uint32_t RESERVED2[24];
    volatile uint32_t ICPR[8];
    uint32_t RESERVED3[24];
    volatile uint32_t IABR[8];
    uint32_t RESERVED4[56];
    volatile uint8_t  IP[240];
    uint32_t RESERVED5[644];
    volatile uint32_t STIR;
} NVIC_Type;

/* SCB */
typedef struct {
    volatile uint32_t CPUID;
    volatile uint32_t ICSR;
    volatile uint32_t VTOR;
    volatile uint32_t AIRCR;
    volatile uint32_t SCR;
    volatile uint32_t CCR;
    volatile uint8_t  SHP[12];
    volatile uint32_t SHCSR;
    volatile uint32_t CFSR;
    volatile uint32_t HFSR;
    uint32_t RESERVED;
    volatile uint32_t MMAR;
    volatile uint32_t BFAR;
    volatile uint32_t AFSR;
} SCB_Type;

/* =====================================================================
 *   Peripheral Instances
 * ===================================================================== */

/* GPIO */
#define GPIOA               ((GPIO_TypeDef *) GPIOA_BASE)
#define GPIOB               ((GPIO_TypeDef *) GPIOB_BASE)
#define GPIOC               ((GPIO_TypeDef *) GPIOC_BASE)
#define GPIOD               ((GPIO_TypeDef *) GPIOD_BASE)
#define GPIOE               ((GPIO_TypeDef *) GPIOE_BASE)
#define GPIOH               ((GPIO_TypeDef *) GPIOH_BASE)

/* RCC, EXTI, SYSCFG */
#define RCC                 ((RCC_TypeDef *) RCC_BASE)
#define EXTI                ((EXTI_TypeDef *) EXTI_BASE)
#define SYSCFG              ((SYSCFG_TypeDef *) SYSCFG_BASE)

/* ADC */
#define ADC1                ((ADC_TypeDef *) ADC1_BASE)

/* USART */
#define USART1              ((USART_TypeDef *) USART1_BASE)
#define USART2              ((USART_TypeDef *) USART2_BASE)
#define USART6              ((USART_TypeDef *) USART6_BASE)

/* SPI */
#define SPI1                ((SPI_TypeDef *) SPI1_BASE)
#define SPI2                ((SPI_TypeDef *) SPI2_BASE)
#define SPI4                ((SPI_TypeDef *) SPI4_BASE)

/* I2C */
#define I2C1                ((I2C_TypeDef *) I2C1_BASE)
#define I2C2                ((I2C_TypeDef *) I2C2_BASE)
#define I2C3                ((I2C_TypeDef *) I2C3_BASE)

/* TIM */
#define TIM1                ((TIM_TypeDef *) TIM1_BASE)
#define TIM2                ((TIM_TypeDef *) TIM2_BASE)
#define TIM3                ((TIM_TypeDef *) TIM3_BASE)
#define TIM4                ((TIM_TypeDef *) TIM4_BASE)
#define TIM5                ((TIM_TypeDef *) TIM5_BASE)
#define TIM9                ((TIM_TypeDef *) TIM9_BASE)
#define TIM10               ((TIM_TypeDef *) TIM10_BASE)
#define TIM11               ((TIM_TypeDef *) TIM11_BASE)

/* Cortex-M4 Core */
#define SysTick             ((SysTick_Type *) SysTick_BASE)
#define NVIC                ((NVIC_Type *) NVIC_BASE)
#define SCB                 ((SCB_Type *) SCB_BASE)
