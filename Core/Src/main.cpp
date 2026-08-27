/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.cpp
 * @brief          : PWM Sine + Dual PWM IC + FSM Button + ISR Profiling + HW
 *UART
 ******************************************************************************
 */
/* USER CODE END Header */
#include "main.h"

/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>

#include "diagnostics.hpp"
#include "frame.hpp"
#include "tx_manager.hpp"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1; /* ISR duration measurement on PA8 */
TIM_HandleTypeDef htim2; /* 10 kHz PWM on PA0 and PA1 */
TIM_HandleTypeDef htim3; /* PWM Input Capture on PA6 */
TIM_HandleTypeDef htim4; /* High-frequency telemetry timer (50 Hz) */
TIM_HandleTypeDef htim5; /* Debounce & sine stepping timer (1 ms) */

DMA_HandleTypeDef hdma_tim2_ch1;
DMA_HandleTypeDef hdma_usart1_tx;
UART_HandleTypeDef huart1;

#define SINE_SAMPLES 100

/* Scaled LUT values x10 for 10 kHz carrier (ARR = 1599) */
const uint16_t sine_lut[SINE_SAMPLES] = {
    800,  840,  890,  940,  990,  1040, 1090, 1130, 1180, 1220, 1260, 1300,
    1340, 1370, 1410, 1440, 1470, 1490, 1510, 1530, 1550, 1570, 1580, 1580,
    1590, 1590, 1590, 1580, 1580, 1570, 1550, 1530, 1510, 1490, 1470, 1440,
    1410, 1370, 1340, 1300, 1260, 1220, 1180, 1130, 1090, 1040, 990,  940,
    890,  840,  800,  750,  700,  650,  600,  550,  500,  460,  410,  370,
    330,  290,  250,  220,  180,  150,  120,  100,  80,   60,   50,   50,
    50,   50,   50,   50,   50,   60,   80,   100,  120,  150,  180,  220,
    250,  290,  330,  370,  410,  460,  500,  550,  600,  650,  700,  750};

/* ==============================================================================
 * ATOMIC PIN OPERATIONS VIA BSRR / IDR (1 cycle)
 * ==============================================================================
 */
#define BTN_READ() ((GPIOB->IDR & GPIO_PIN_0) == 0) /* true if pressed (0) */
#define DEBUG_PIN_HIGH() (GPIOB->BSRR = GPIO_PIN_1) /* PB1 High (ISR Start) */
#define DEBUG_PIN_LOW()                                                        \
  (GPIOB->BSRR = (GPIO_PIN_1 << 16)) /* PB1 Low (ISR End) */
#define LED_PC13_ON() (GPIOC->BSRR = (GPIO_PIN_13 << 16)) /* PC13 Low (On) */
#define LED_PC13_OFF() (GPIOC->BSRR = GPIO_PIN_13)        /* PC13 High (Off) */

/* ==============================================================================
 * GLOBAL VARIABLES
 * ==============================================================================
 */
volatile uint32_t g_tim3_overflows = 0;
volatile uint32_t g_t_rising_prev = 0;
volatile uint32_t g_measured_period_ticks = 0;
volatile uint32_t g_measured_pulse_ticks = 0;
volatile bool g_has_first_rising = false;

/* Global variable to store ISR execution duration (in 16 MHz ticks = 62.5 ns)
 */
volatile uint32_t g_isr_duration_ticks = 0;

volatile bool g_btn_event = false;
volatile bool g_btn_state = false;
enum class BtnState { IDLE, PRESSED, WAIT_DOUBLE };

struct __attribute__((packed)) PwmMetrics {
  uint32_t frequency_hz;
  uint8_t duty_cycle_percent;
  uint16_t isr_time_ticks;
};

protocol::DiagnosticsStats g_stats{};
protocol::UartTxManager g_tx_manager(huart1);
static uint8_t g_msg_seq_num = 0;

/* Timing and protocol constants */
constexpr uint32_t ACK_TIMEOUT_MS = 200;
constexpr uint8_t MAX_RETRIES = 3;
/* MODIFIED: Increased interval from 300ms/20000ms to 750ms for clearly visible
 * toggling */
constexpr uint32_t LED_TOGGLE_INTERVAL_MS = 750; /* LED toggle interval (ms) */
constexpr uint32_t SHORT_PRESS_TIMEOUT_MS =
    300; /* Double-click wait window (ms) */
constexpr uint32_t LONG_PRESS_THRESHOLD_MS =
    1000; /* Long press threshold (ms) */

/* Private function prototypes -----------------------------------------------*/
extern "C" {
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_USART1_UART_Init(void);
}

inline uint32_t get_absolute_capture(TIM_HandleTypeDef *htim,
                                     uint32_t channel) {
  uint32_t overflows = g_tim3_overflows;
  uint32_t ccr = HAL_TIM_ReadCapturedValue(htim, channel);
  if (__HAL_TIM_GET_FLAG(htim, TIM_FLAG_UPDATE) != RESET) {
    if (ccr < 0x8000)
      overflows++;
  }
  return (overflows << 16) | (ccr & 0xFFFF);
}

int main(void) {
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_USART1_UART_Init();

  LED_PC13_OFF();

  /* 1. Start standard PWM output (sine duty cycle is updated in TIM5 ISR) */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

  /* 2. Start PWM period capture (TIM3) */
  __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_UPDATE);
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);

  /* 3. Start ISR execution time measurement (TIM1) */
  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_2);

  /* 4. Configure telemetry timer (TIM4 - 50 Hz / 20 ms interval) */
  __HAL_TIM_ENABLE_IT(&htim4, TIM_IT_UPDATE);
  HAL_TIM_Base_Start(&htim4);

  /* 5. Start debounce and sine step timer (TIM5 - 1 ms) */
  HAL_TIM_Base_Start_IT(&htim5);

  BtnState btn_fsm = BtnState::IDLE;
  uint32_t press_time = 0;

  uint8_t led_blink_count = 0;
  uint32_t led_timer = 0;

  while (1) {
    g_tx_manager.process_timeouts(HAL_GetTick());

    /* ==============================================================================
     * BUTTON FINITE STATE MACHINE (FSM)
     * ==============================================================================
     */
    if (g_btn_event) {
      g_btn_event = false;
      if (g_btn_state) { /* Pressed */
        if (btn_fsm == BtnState::IDLE) {
          btn_fsm = BtnState::PRESSED;
          press_time = HAL_GetTick();
        } else if (btn_fsm == BtnState::WAIT_DOUBLE) {
          btn_fsm = BtnState::IDLE;
          /* MODIFIED: Double click -> 4 toggles (2 on, 2 off) */
          led_blink_count = 4;
        }
      } else { /* Released */
        if (btn_fsm == BtnState::PRESSED) {
          if (HAL_GetTick() - press_time > LONG_PRESS_THRESHOLD_MS) {
            btn_fsm = BtnState::IDLE;
          } else {
            btn_fsm = BtnState::WAIT_DOUBLE;
            press_time = HAL_GetTick();
          }
        }
      }
    }

    if (btn_fsm == BtnState::WAIT_DOUBLE &&
        (HAL_GetTick() - press_time > SHORT_PRESS_TIMEOUT_MS)) {
      btn_fsm = BtnState::IDLE;
      /* MODIFIED: Short click -> 2 toggles (1 on, 1 off) */
      led_blink_count = 2;
    }

    if (btn_fsm == BtnState::PRESSED &&
        (HAL_GetTick() - press_time > LONG_PRESS_THRESHOLD_MS)) {
      btn_fsm = BtnState::IDLE;
      /* MODIFIED: Long press -> 10 toggles (5 on, 5 off) */
      led_blink_count = 10;
    }

    /* ==============================================================================
     * ASYNCHRONOUS LED INDICATOR
     * ==============================================================================
     */
    if (led_blink_count > 0) {
      if (HAL_GetTick() - led_timer > LED_TOGGLE_INTERVAL_MS) {
        led_timer = HAL_GetTick();
        static bool led_state = false;
        led_state = !led_state;
        if (led_state)
          LED_PC13_ON();
        else
          LED_PC13_OFF();
        led_blink_count--;
      }
    } else {
      LED_PC13_OFF();
    }

    /* ==============================================================================
     * TELEMETRY TRANSMISSION (50 Hz / Every 20 ms)
     * ==============================================================================
     */
    if (__HAL_TIM_GET_FLAG(&htim4, TIM_FLAG_UPDATE)) {
      __HAL_TIM_CLEAR_IT(&htim4, TIM_IT_UPDATE);

      if (g_measured_period_ticks > 0) {
        __disable_irq();
        uint32_t period = g_measured_period_ticks;
        uint32_t pulse = g_measured_pulse_ticks;
        uint32_t isr_duration = g_isr_duration_ticks;
        __enable_irq();

        PwmMetrics metrics{};
        metrics.frequency_hz = 16000000 / period;
        uint32_t duty = (pulse * 100) / period;
        metrics.duty_cycle_percent =
            static_cast<uint8_t>(duty > 100 ? 100 : duty);
        metrics.isr_time_ticks = static_cast<uint16_t>(isr_duration);

        bool sent = g_tx_manager.send_frame_with_ack(
            static_cast<uint8_t>(protocol::MessageType::DATA), g_msg_seq_num,
            reinterpret_cast<const uint8_t *>(&metrics), sizeof(metrics),
            ACK_TIMEOUT_MS, MAX_RETRIES);

        if (sent)
          g_msg_seq_num++;
      }
    }
  }
}

void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    Error_Handler();

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
    Error_Handler();
}

static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /* LED PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* PB1 Debug Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* PB0 Button */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

static void MX_DMA_Init(void) {
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);
}

static void MX_TIM1_Init(void) {
  TIM_IC_InitTypeDef sConfigIC = {0};
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_TIM1_CLK_ENABLE();

  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
    Error_Handler();
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
    Error_Handler();

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
    Error_Handler();

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
    Error_Handler();

  HAL_NVIC_SetPriority(TIM1_CC_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);
}

static void MX_TIM2_Init(void) {
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_TIM2_CLK_ENABLE();

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1599; /* 16 MHz / 1600 = 10 kHz PWM */
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  HAL_TIM_Base_Init(&htim2);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig);
  HAL_TIM_PWM_Init(&htim2);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1);

  sConfigOC.Pulse = 1190; /* Scaled to ~75% of 1600 */
  HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2);

  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  hdma_tim2_ch1.Instance = DMA1_Stream5;
  hdma_tim2_ch1.Init.Channel = DMA_CHANNEL_3;
  hdma_tim2_ch1.Init.Direction = DMA_MEMORY_TO_PERIPH;
  hdma_tim2_ch1.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_tim2_ch1.Init.MemInc = DMA_MINC_ENABLE;
  hdma_tim2_ch1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  hdma_tim2_ch1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
  hdma_tim2_ch1.Init.Mode = DMA_CIRCULAR;
  hdma_tim2_ch1.Init.Priority = DMA_PRIORITY_LOW;
  hdma_tim2_ch1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
  HAL_DMA_Init(&hdma_tim2_ch1);

  __HAL_LINKDMA(&htim2, hdma[TIM_DMA_ID_CC1], hdma_tim2_ch1);
}

static void MX_TIM3_Init(void) {
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_TIM3_CLK_ENABLE();

  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
    Error_Handler();

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig);
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
    Error_Handler();

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
    Error_Handler();

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
    Error_Handler();

  HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);
}

static void MX_TIM4_Init(void) {
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  __HAL_RCC_TIM4_CLK_ENABLE();
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 1600 - 1; /* 16 MHz / 1600 = 10 kHz clock */
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 200 - 1; /* 10 kHz / 200 = 50 Hz interrupt rate */
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
    Error_Handler();
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig);
}

static void MX_TIM5_Init(void) {
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  __HAL_RCC_TIM5_CLK_ENABLE();
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 16 - 1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 1000 - 1;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
    Error_Handler();
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig);

  HAL_NVIC_SetPriority(TIM5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM5_IRQn);
}

static void MX_USART1_UART_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_USART1_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
    Error_Handler();

  hdma_usart1_tx.Instance = DMA2_Stream7;
  hdma_usart1_tx.Init.Channel = DMA_CHANNEL_4;
  hdma_usart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
  hdma_usart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_usart1_tx.Init.MemInc = DMA_MINC_ENABLE;
  hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_usart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma_usart1_tx.Init.Mode = DMA_NORMAL;
  hdma_usart1_tx.Init.Priority = DMA_PRIORITY_LOW;
  hdma_usart1_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
  if (HAL_DMA_Init(&hdma_usart1_tx) != HAL_OK)
    Error_Handler();

  __HAL_LINKDMA(&huart1, hdmatx, hdma_usart1_tx);

  HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
}

/*
 * INTERRUPT HANDLERS AND CALLBACKS
 */
extern "C" {

void DMA1_Stream5_IRQHandler(void) { HAL_DMA_IRQHandler(&hdma_tim2_ch1); }
void DMA2_Stream7_IRQHandler(void) { HAL_DMA_IRQHandler(&hdma_usart1_tx); }

void USART1_IRQHandler(void) {
  if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_ORE) != RESET) {
    __HAL_UART_CLEAR_OREFLAG(&huart1);
  }
  if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_NE) != RESET) {
    __HAL_UART_CLEAR_NEFLAG(&huart1);
  }
  if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_FE) != RESET) {
    __HAL_UART_CLEAR_FEFLAG(&huart1);
  }

  HAL_UART_IRQHandler(&huart1);
}

void TIM3_IRQHandler(void) { HAL_TIM_IRQHandler(&htim3); }
void TIM1_CC_IRQHandler(void) { HAL_TIM_IRQHandler(&htim1); }

void TIM5_IRQHandler(void) {
  DEBUG_PIN_HIGH(); /* Pulse high on PB1 (ISR measurement start) */

  if (__HAL_TIM_GET_FLAG(&htim5, TIM_FLAG_UPDATE)) {
    __HAL_TIM_CLEAR_IT(&htim5, TIM_IT_UPDATE);

    /* Software sine wave stepping: 50 ms per step = 5.0 s total period */
    static uint32_t sine_timer = 0;
    static uint16_t sine_index = 0;
    if (++sine_timer >= 50) {
      sine_timer = 0;
      TIM2->CCR1 = sine_lut[sine_index];
      sine_index = (sine_index + 1) % SINE_SAMPLES;
    }

    /* Button debouncing logic */
    static uint16_t history = 0x0000;
    history = (history << 1) | (BTN_READ() ? 1 : 0);

    bool new_state = g_btn_state;
    if ((history & 0x03FF) == 0x03FF)
      new_state = true;
    if ((history & 0x03FF) == 0x0000)
      new_state = false;

    if (new_state != g_btn_state) {
      g_btn_state = new_state;
      g_btn_event = true;
    }
  }

  DEBUG_PIN_LOW(); /* Pulse low on PB1 (ISR measurement end) */
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM3) {
    g_tim3_overflows =
        g_tim3_overflows + 1; /* Explicit assignment to avoid C++20 warning */
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART1) {
    g_tx_manager.on_tx_complete_isr();
  }
}

/* Input Capture Callback: Hardware measurement of PB1 strobe width on pin PA8
 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM3) {
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
      uint32_t t_rising_curr = get_absolute_capture(htim, TIM_CHANNEL_1);
      if (g_has_first_rising) {
        g_measured_period_ticks = t_rising_curr - g_t_rising_prev;
      }
      g_t_rising_prev = t_rising_curr;
      g_has_first_rising = true;
    } else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
      if (g_has_first_rising) {
        uint32_t t_falling = get_absolute_capture(htim, TIM_CHANNEL_2);
        g_measured_pulse_ticks = t_falling - g_t_rising_prev;
      }
    }
  }

  /* High level duration measurement on PA8 (TIM1_CH1 -> CH2) */
  if (htim->Instance == TIM1 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
    uint32_t start = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
    uint32_t end = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);

    /* Calculate and store ISR duration into global variable */
    g_isr_duration_ticks =
        (end >= start) ? (end - start) : ((0xFFFF - start) + end + 1);
  }
}

void Error_Handler(void) {
  __disable_irq();
  while (1) {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {}
#endif

} /* extern "C" */
