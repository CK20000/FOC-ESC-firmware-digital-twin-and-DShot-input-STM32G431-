#ifndef VIRTUAL_STM32_H
#define VIRTUAL_STM32_H
#include <stdint.h>
#include <stdbool.h>

#define __IO volatile
#define __I  volatile const
#define __O  volatile

typedef struct { __IO uint32_t CR1,CR2,SMCR,DIER,SR,EGR,CCMR1,CCMR2,CCER,
                  CNT,PSC,ARR,RCR,CCR1,CCR2,CCR3,CCR4,BDTR; } TIM_TypeDef;
typedef struct { __IO uint32_t ISR,IER,CR,CFGR,SMPR1,SMPR2,JSQR,JDR1,JDR2,JDR3; } ADC_TypeDef;
typedef struct { __IO uint32_t IMR1,EMR1,RTSR1,FTSR1,SWIER1,PR1; } EXTI_TypeDef;
typedef struct { __IO uint32_t MEMRMP,CFGR1,EXTICR[4]; } SYSCFG_TypeDef;
typedef struct { __IO uint32_t MODER,OTYPER,OSPEEDR,PUPDR,IDR,ODR,BSRR,LCKR,AFR[2],BRR; } GPIO_TypeDef;
typedef struct { __IO uint32_t CR,ICSCR,CFGR,PLLCFGR,AHB1ENR,AHB2ENR,APB1ENR1,APB1ENR2,APB2ENR; } RCC_TypeDef;
typedef struct { __IO uint32_t CSR,WDATA,RDATA; } CORDIC_TypeDef;
typedef struct { __IO uint32_t CTRL,CYCCNT; } DWT_TypeDef;
typedef struct { __IO uint32_t DHCSR,DCRSR,DCRDR,DEMCR; } CoreDebug_TypeDef;

extern TIM_TypeDef       *TIM1, *TIM2;
extern ADC_TypeDef       *ADC1, *ADC2;
extern EXTI_TypeDef      *EXTI;
extern SYSCFG_TypeDef    *SYSCFG;
extern GPIO_TypeDef      *GPIOA, *GPIOB;
extern RCC_TypeDef       *RCC;
extern CORDIC_TypeDef    *CORDIC;
extern DWT_TypeDef       *DWT;
extern CoreDebug_TypeDef *CoreDebug;

#define TIM_SR_UIF                  (1UL<<0)
#define TIM_CR1_CEN                 (1UL<<0)
#define TIM_EGR_UG                  (1UL<<0)
#define TIM_CR2_MMS_Msk             (0x7UL<<4)
#define TIM_CR2_MMS_Pos             4
#define ADC_CR_JADSTART             (1UL<<2)
#define RCC_AHB1ENR_CORDICEN        (1UL<<28)
#define RCC_AHB2ENR_GPIOAEN         (1UL<<0)
#define RCC_AHB2ENR_GPIOBEN         (1UL<<1)
#define RCC_APB1ENR1_TIM2EN         (1UL<<0)
#define RCC_APB2ENR_SYSCFGEN        (1UL<<0)
#define GPIO_MODER_MODE0            (3UL<<0)
#define GPIO_MODER_MODE0_Pos        0
#define GPIO_PUPDR_PUPD0            (3UL<<0)
#define GPIO_PUPDR_PUPD0_Pos        0
#define SYSCFG_EXTICR1_EXTI0        (0xFUL<<0)
#define SYSCFG_EXTICR1_EXTI0_Pos    0
#define EXTI_IMR1_IM0               (1UL<<0)
#define EXTI_RTSR1_RT0              (1UL<<0)
#define EXTI_FTSR1_FT0              (1UL<<0)
#define EXTI_PR1_PIF0               (1UL<<0)
#define CORDIC_CSR_PRECISION_Pos    4
#define CORDIC_CSR_NRES             (1UL<<1)
#define DWT_CTRL_CYCCNTENA_Msk      (1UL<<0)
#define CoreDebug_DEMCR_TRCENA_Msk  (1UL<<24)

typedef enum { EXTI0_IRQn = 6 } IRQn_Type;

static inline void NVIC_SetPriority(IRQn_Type i, uint32_t p) { (void)i; (void)p; }
static inline void NVIC_EnableIRQ(IRQn_Type i)  { (void)i; }
static inline void NVIC_DisableIRQ(IRQn_Type i) { (void)i; }

void HAL_Delay(uint32_t ms);
uint32_t HAL_GetTick(void);

uint32_t sil_tim1_get_ccr1(void);  uint32_t sil_tim1_get_ccr2(void);  uint32_t sil_tim1_get_ccr3(void);
void sil_tim1_set_ccr1(uint32_t);  void sil_tim1_set_ccr2(uint32_t);  void sil_tim1_set_ccr3(uint32_t);
uint32_t sil_tim1_get_sr(void);    void sil_tim1_set_sr(uint32_t);
void sil_adc1_set_jdr1(uint32_t);  void sil_adc2_set_jdr1(uint32_t);
uint32_t sil_adc1_get_jdr1(void);  uint32_t sil_adc2_get_jdr1(void);
uint32_t sil_tim2_get_cnt(void);   void sil_tim2_set_cnt(uint32_t);   void sil_tim2_advance(uint32_t);
float sil_dshot_get_throttle(void); uint8_t sil_dshot_get_armed(void);
void sil_dshot_set_throttle(float t); void sil_dshot_set_armed(uint8_t a);
void sil_adc1_set_jdr3(uint32_t);
#endif
