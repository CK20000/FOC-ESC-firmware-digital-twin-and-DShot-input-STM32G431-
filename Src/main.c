/*
 * main.c — STM32G431 FOC skeleton + DShot decode test (Flash Dump)
 */
#include "stm32g4xx_hal.h"
#include "foc_core.h"
#include "foc_dshot.h"

/* HAL handles */
ADC_HandleTypeDef   hadc1, hadc2;
DAC_HandleTypeDef   hdac3;
TIM_HandleTypeDef   htim1;
UART_HandleTypeDef  huart2;
CORDIC_HandleTypeDef hcordic;

/* Forward declarations */
void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_ADC1_Init(void);
void MX_ADC2_Init(void);
void MX_DAC3_Init(void);
void MX_TIM1_Init(void);
void MX_USART2_UART_Init(void);
void MX_CORDIC_Init(void);

/* CCM RAM */
extern uint32_t _siccmram;
extern uint32_t _sccmram;
extern uint32_t _eccmram;

/* Struct to save the DShot results to Flash */
typedef struct {
    uint32_t magic;           // 0xDEADBEEF
    uint32_t loop_count;      // Should be 1440000 (30s at 48kHz)
    uint32_t max_cycles;      // FOC execution cycles
    uint32_t frames_ok;       // <--- Did DShot decode? (Should be ~30000)
    uint32_t frames_bad;      // <--- Bad frames
    uint32_t throttle_x1000;  // <--- Should be 500 (if FC set to 50%)
    uint32_t rx_count;        // Leftover edges in buffer
    uint32_t final_state;     // FOC State
} DShotDebug;

void Copy_CCM_RAM(void) {
    uint32_t *src = &_siccmram;
    uint32_t *dst = &_sccmram;
    while (dst < &_eccmram) {
        *dst++ = *src++;
    }
}

void save_debug_to_flash(void) {
    DShotDebug dbg;
    dbg.magic          = 0xDEADBEEF;
    dbg.loop_count     = 1440000;
    dbg.max_cycles     = foc_debug.max_cycles;
    dbg.frames_ok      = dshot.frames_ok;
    dbg.frames_bad     = dshot.frames_bad;
    dbg.throttle_x1000 = (uint32_t)(dshot.throttle * 1000.0f);
    dbg.rx_count       = dshot._rx_count;
    dbg.final_state    = (uint32_t)foc_get_state();

    HAL_FLASH_Unlock();
    FLASH_EraseInitTypeDef e = {0};
    uint32_t PageError = 0;
    e.TypeErase = FLASH_TYPEERASE_PAGES;
    e.Banks = 1;
    e.Page = 63;
    e.NbPages = 1;
    HAL_FLASHEx_Erase(&e, &PageError);

    uint64_t *src = (uint64_t *)&dbg;
    for (int i = 0; i < 4; i++) {
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 0x0801F800 + i*8, src[i]);
    }
    HAL_FLASH_Lock();
}

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_ADC1_Init();
    MX_ADC2_Init();
    MX_DAC3_Init();
    MX_TIM1_Init();
    MX_USART2_UART_Init();
    MX_CORDIC_Init();
    Copy_CCM_RAM();
    foc_setup();

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    /* Run for exactly 30 seconds (48kHz * 30 = 1,440,000 loops) */
    for (uint32_t i = 0; i < 1440000; i++) {
        foc_loop();
    }

    /* Save results to Flash and halt */
    save_debug_to_flash();
    while (1) { __NOP(); }
}

/* ============================================================
 * System clock — 170 MHz from HSI 16 MHz + PLL
 * ============================================================ */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

    osc.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    osc.HSIState       = RCC_HSI_ON;
    osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    osc.PLL.PLLState   = RCC_PLL_ON;
    osc.PLL.PLLSource  = RCC_PLLSOURCE_HSI;
    osc.PLL.PLLM       = RCC_PLLM_DIV4;
    osc.PLL.PLLN       = 85;
    osc.PLL.PLLP       = RCC_PLLP_DIV2;
    osc.PLL.PLLQ       = RCC_PLLQ_DIV2;
    osc.PLL.PLLR       = RCC_PLLR_DIV2;
    HAL_RCC_OscConfig(&osc);

    clk.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                       | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV1;
    clk.APB2CLKDivider = RCC_HCLK_DIV1;
    HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_4);
}

void MX_GPIO_Init(void) {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef g = {0};
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_HIGH;

    g.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10;
    HAL_GPIO_Init(GPIOA, &g);
    HAL_GPIO_WritePin(GPIOA, g.Pin, GPIO_PIN_SET);

    g.Pin = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    HAL_GPIO_Init(GPIOB, &g);
    HAL_GPIO_WritePin(GPIOB, g.Pin, GPIO_PIN_SET);
}

void MX_ADC1_Init(void) { __HAL_RCC_ADC12_CLK_ENABLE(); hadc1.Instance = ADC1; HAL_ADC_Init(&hadc1); }
void MX_ADC2_Init(void) { hadc2.Instance = ADC2; HAL_ADC_Init(&hadc2); }

void MX_DAC3_Init(void) {
    __HAL_RCC_DAC3_CLK_ENABLE();
    hdac3.Instance = DAC3;
    HAL_DAC_Init(&hdac3);
    DAC_ChannelConfTypeDef cfg = {0};
    cfg.DAC_Trigger = DAC_TRIGGER_NONE;
    cfg.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
    HAL_DAC_ConfigChannel(&hdac3, &cfg, DAC_CHANNEL_1);
    HAL_DAC_SetValue(&hdac3, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 2048);
}

void MX_TIM1_Init(void) {
    __HAL_RCC_TIM1_CLK_ENABLE();
    GPIO_InitTypeDef g = {0};
    g.Mode = GPIO_MODE_AF_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = GPIO_AF6_TIM1;
    g.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10;
    HAL_GPIO_Init(GPIOA, &g);
    g.Pin = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    HAL_GPIO_Init(GPIOB, &g);

    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 0;
    htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
    htim1.Init.Period = 4250;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 1;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_PWM_Init(&htim1);
}

void MX_USART2_UART_Init(void) {
    __HAL_RCC_USART2_CLK_ENABLE();
    GPIO_InitTypeDef g = {0};
    g.Mode = GPIO_MODE_AF_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = GPIO_AF7_USART2;
    g.Pin = GPIO_PIN_2 | GPIO_PIN_3;
    HAL_GPIO_Init(GPIOA, &g);

    huart2.Instance = USART2;
    huart2.Init.BaudRate = 2000000;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart2);
}

void MX_CORDIC_Init(void) {
    __HAL_RCC_CORDIC_CLK_ENABLE();
    hcordic.Instance = CORDIC;
    HAL_CORDIC_Init(&hcordic);
}

void SysTick_Handler(void) { HAL_IncTick(); }