/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.cpp
 * @brief          : PWM Sine + Dual PWM Input Capture + Multi-overflow +
 *TxManager
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
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3; /* PWM Input Capture на PA6 */
TIM_HandleTypeDef htim4; /* Ежесекундный будильник */
DMA_HandleTypeDef hdma_tim2_ch1;
UART_HandleTypeDef huart1;

#define SINE_SAMPLES 100

const uint16_t sine_lut[SINE_SAMPLES] = {
    80,  84,  89,  94,  99,  104, 109, 113, 118, 122, 126, 130, 134, 137, 141,
    144, 147, 149, 151, 153, 155, 157, 158, 158, 159, 159, 159, 158, 158, 157,
    155, 153, 151, 149, 147, 144, 141, 137, 134, 130, 126, 122, 118, 113, 109,
    104, 99,  94,  89,  84,  80,  75,  70,  65,  60,  55,  50,  46,  41,  37,
    33,  29,  25,  22,  18,  15,  12,  10,  8,   6,   4,   2,   1,   1,   0,
    0,   0,   1,   1,   2,   4,   6,   8,   10,  12,  15,  18,  22,  25,  29,
    33,  37,  41,  46,  50,  55,  60,  65,  70,  75};

/* Переменные расширенного 32-битного Input Capture */
volatile uint32_t g_tim3_overflows = 0;
volatile uint32_t g_t_rising_prev = 0;
volatile uint32_t g_measured_period_ticks = 0;
volatile uint32_t g_measured_pulse_ticks = 0;
volatile bool g_has_first_rising = false;

/* Структура отправки метрик по UART */
struct __attribute__((packed)) PwmMetrics {
  uint32_t frequency_hz;
  uint8_t duty_cycle_percent;
};

protocol::DiagnosticsStats g_stats{};
protocol::UartTxManager g_tx_manager(huart1);
static uint8_t g_msg_seq_num = 0;

constexpr uint32_t ACK_TIMEOUT_MS = 200;
constexpr uint8_t MAX_RETRIES = 3;

/* Private function prototypes -----------------------------------------------*/
extern "C" {
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
}

/* Корректный расчёт 32-битного времени с учётом нераспределённого переполнения
 */
inline uint32_t get_absolute_capture(TIM_HandleTypeDef *htim,
                                     uint32_t channel) {
  uint32_t overflows = g_tim3_overflows;
  uint32_t ccr = HAL_TIM_ReadCapturedValue(htim, channel);

  /* Если прерывание по переполнению ещё не успело обработаться:
     - ccr < 0x8000 означает, что захват произошёл ПОСЛЕ переполнения ->
     прибавляем +1
     - ccr >= 0x8000 означает, что захват произошёл ДО переполнения -> оставляем
     текущий overflows */
  if (__HAL_TIM_GET_FLAG(htim, TIM_FLAG_UPDATE) != RESET) {
    if (ccr < 0x8000) {
      overflows++;
    }
  }
  return (overflows << 16) | (ccr & 0xFFFF);
}

int main(void) {
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_DMA_Init();

  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();

  /* 1. Запуск ШИМ и синуса */
  HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_1, (uint32_t *)sine_lut,
                        SINE_SAMPLES);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

  /* 2. Запуск захвата периода (CH1) и длительности (CH2) */
  __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_UPDATE);
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);

  /* 3. Настройка секундного таймера WFE */
  SCB->SCR |= SCB_SCR_SEVONPEND_Msk;
  __HAL_TIM_ENABLE_IT(&htim4, TIM_IT_UPDATE);
  HAL_TIM_Base_Start(&htim4);

  while (1) {
    g_tx_manager.process_timeouts(HAL_GetTick());

    __WFE();

    if (__HAL_TIM_GET_FLAG(&htim4, TIM_FLAG_UPDATE)) {
      __HAL_TIM_CLEAR_IT(&htim4, TIM_IT_UPDATE);
      NVIC_ClearPendingIRQ(TIM4_IRQn);

      if (g_measured_period_ticks > 0) {
        __disable_irq();
        uint32_t period = g_measured_period_ticks;
        uint32_t pulse = g_measured_pulse_ticks;
        __enable_irq();

        PwmMetrics metrics{};
        metrics.frequency_hz = 16000000 / period;

        /* Безопасный расчёт скважности с ограничением до 100% */
        uint32_t duty = (pulse * 100) / period;
        metrics.duty_cycle_percent =
            static_cast<uint8_t>(duty > 100 ? 100 : duty);

        bool sent = g_tx_manager.send_frame_with_ack(
            static_cast<uint8_t>(protocol::MessageType::DATA), g_msg_seq_num,
            reinterpret_cast<const uint8_t *>(&metrics), sizeof(metrics),
            ACK_TIMEOUT_MS, MAX_RETRIES);

        if (sent) {
          g_msg_seq_num++;
        }
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

static void MX_GPIO_Init(void) { __HAL_RCC_GPIOA_CLK_ENABLE(); }

static void MX_DMA_Init(void) {
  __HAL_RCC_DMA1_CLK_ENABLE();
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
}

static void MX_TIM2_Init(void) {
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_TIM2_CLK_ENABLE();

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 159;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 199;
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

  sConfigOC.Pulse = 40;
  HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2);

  /* Увеличена скорость GPIO для ровных фронтов ШИМ */
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
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  htim4.Init.Prescaler = 16000 - 1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1000 - 1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
    Error_Handler();

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig);
}

static void MX_USART1_UART_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_USART1_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
}

/*
 * ОБРАБОТЧИКИ ПРЕРЫВАНИЙ И CALLBACK
 */
extern "C" {

void DMA1_Stream5_IRQHandler(void) { HAL_DMA_IRQHandler(&hdma_tim2_ch1); }

void TIM3_IRQHandler(void) { HAL_TIM_IRQHandler(&htim3); }

/* Обработчик переполнения таймера TIM3 (вызывается из HAL_TIM_IRQHandler) */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM3) {
    g_tim3_overflows++;
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART1) {
    g_tx_manager.on_tx_complete_isr();
  }
}

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
