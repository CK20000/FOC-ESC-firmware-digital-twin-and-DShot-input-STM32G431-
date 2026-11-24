#include "virtual_stm32.h"
#include "foc_dshot.h"

static TIM_TypeDef       g_tim1={0}, g_tim2={0};
static ADC_TypeDef       g_adc1={0}, g_adc2={0};
static EXTI_TypeDef      g_exti={0};
static SYSCFG_TypeDef    g_syscfg={0};
static GPIO_TypeDef      g_gpioa={0}, g_gpiob={0};
static RCC_TypeDef       g_rcc={0};
static CORDIC_TypeDef    g_cordic={0};
static DWT_TypeDef       g_dwt={0};
static CoreDebug_TypeDef g_coredebug={0};

TIM_TypeDef       *TIM1=&g_tim1, *TIM2=&g_tim2;
ADC_TypeDef       *ADC1=&g_adc1, *ADC2=&g_adc2;
EXTI_TypeDef      *EXTI=&g_exti;
SYSCFG_TypeDef    *SYSCFG=&g_syscfg;
GPIO_TypeDef      *GPIOA=&g_gpioa, *GPIOB=&g_gpiob;
RCC_TypeDef       *RCC=&g_rcc;
CORDIC_TypeDef    *CORDIC=&g_cordic;
DWT_TypeDef       *DWT=&g_dwt;
CoreDebug_TypeDef *CoreDebug=&g_coredebug;

void HAL_Delay(uint32_t ms) { (void)ms; }
uint32_t HAL_GetTick(void)  { return g_dwt.CYCCNT / 170000; }

uint32_t sil_tim1_get_ccr1(void){ return TIM1->CCR1; }
uint32_t sil_tim1_get_ccr2(void){ return TIM1->CCR2; }
uint32_t sil_tim1_get_ccr3(void){ return TIM1->CCR3; }
void sil_tim1_set_ccr1(uint32_t v){ TIM1->CCR1=v; }
void sil_tim1_set_ccr2(uint32_t v){ TIM1->CCR2=v; }
void sil_tim1_set_ccr3(uint32_t v){ TIM1->CCR3=v; }
uint32_t sil_tim1_get_sr(void){ return TIM1->SR; }
void sil_tim1_set_sr(uint32_t v){ TIM1->SR=v; }
void sil_adc1_set_jdr1(uint32_t v){ ADC1->JDR1=v; }
void sil_adc2_set_jdr1(uint32_t v){ ADC2->JDR1=v; }
uint32_t sil_adc1_get_jdr1(void){ return ADC1->JDR1; }
uint32_t sil_adc2_get_jdr1(void){ return ADC2->JDR1; }
uint32_t sil_tim2_get_cnt(void){ return TIM2->CNT; }
void sil_tim2_set_cnt(uint32_t v){ TIM2->CNT=v; }
void sil_tim2_advance(uint32_t t){ TIM2->CNT+=t; g_dwt.CYCCNT+=t; }
float sil_dshot_get_throttle(void){ return dshot.throttle; }
uint8_t sil_dshot_get_armed(void){ return dshot.armed ? 1 : 0; }

void sil_dshot_set_throttle(float t){ dshot.throttle = t; }
void sil_dshot_set_armed(uint8_t a){ dshot.armed = a ? true : false; }
void sil_adc1_set_jdr3(uint32_t v){ ADC1->JDR3=v; }
