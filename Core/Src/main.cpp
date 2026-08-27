/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.cpp
 * @brief          : FSM Button (PA0) + DSP Filters + Oversampling + Volts 
 *                   + JSON UART + SPWM (PA5) + PWM (PA1)
 ******************************************************************************
 */
/* USER CODE END Header */
#include "main.h"
#include <stdio.h>
#include <string.h>

struct CalibrationConfig {
    float v_ref;        // Опорное напряжение АЦП (Вольты)
    float offset;       // Смещение нуля (Вольты)
    float gain;         // Коэффициент усиления (делитель напряжения)
};

// Конфиг: АЦП запитан от 3.3В, без смещения, читаем напрямую 1:1
const CalibrationConfig sensor_cfg = {3.3f, 0.0f, 1.0f};

/* Функции перевода попугаев АЦП в Вольты */
float adc12_to_volts(uint16_t raw_12bit) {
    return ((float)raw_12bit / 4095.0f) * sensor_cfg.v_ref * sensor_cfg.gain + sensor_cfg.offset;
}

float adc16_to_volts(uint16_t os_16bit) {
    return ((float)os_16bit / 65535.0f) * sensor_cfg.v_ref * sensor_cfg.gain + sensor_cfg.offset;
}

/* ==============================================================================
 * НАСТРОЙКИ АЦП И БУФЕРОВ
 * ==============================================================================
 */
#define NUM_ADC_CHANNELS 2
#define ADC_BUF_SIZE 2000 // 1000 сэмплов на канал 

volatile uint16_t adc_buffer[ADC_BUF_SIZE];
volatile uint8_t flag_half_ready = 0;
volatile uint8_t flag_full_ready = 0;

volatile uint32_t dwt_timestamp_prev = 0;
volatile uint32_t dwt_cycles_diff = 0;
volatile uint32_t missed_blocks = 0;

// Расчет по теореме Найквиста (fs > 100 kHz для корректного захвата гармоник 10 kHz ШИМ)
constexpr uint32_t THEORETICAL_SAMPLE_RATE = 262500; 
constexpr uint32_t ALLOWED_ERROR_HZ = 1000;

ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;

TIM_HandleTypeDef htim2; // PA5 - SPWM Синус 1 кГц
TIM_HandleTypeDef htim5; // PA1 - PWM 10 кГц
DMA_HandleTypeDef hdma_tim2_ch1;

/* Таблица синуса (100 точек) */
const uint16_t sine_lut[100] = {
    420, 446, 472, 498, 524, 549, 574, 597, 620, 642, 663, 683, 702, 720, 737, 752, 765, 778, 789, 799,
    807, 814, 820, 824, 827, 828, 827, 824, 820, 814, 807, 799, 789, 778, 765, 752, 737, 720, 702, 683,
    663, 642, 620, 597, 574, 549, 524, 498, 472, 446, 420, 394, 368, 342, 316, 291, 266, 243, 220, 198,
    177, 157, 138, 120, 103,  88,  75,  62,  51,  41,  33,  26,  20,  16,  13,  12,  13,  16,  20,  26,
     33,  41,  51,  62,  75,  88, 103, 120, 138, 157, 177, 198, 220, 243, 266, 291, 316, 342, 368, 394
};

/* ==============================================================================
 * DSP ФИЛЬТРЫ И OVERSAMPLING
 * ==============================================================================
 */
// 1. OVERSAMPLING: Увеличиваем разрядность на 4 бита (до 16-bit). Нужно 4^4 = 256 отсчетов.
#define OS_N 256
#define OS_SHIFT 4
uint16_t os_buf[OS_N] = {0};
uint16_t os_idx = 0;
uint32_t os_sum = 0;
uint16_t last_os_16bit = 0;

// 2. Скользящее среднее (MA)
#define MA_SIZE 16
uint16_t ma_buf[MA_SIZE] = {0};
uint16_t ma_med_buf[MA_SIZE] = {0};
uint8_t ma_idx = 0;
uint32_t ma_sum = 0;
uint32_t ma_med_sum = 0;

// 3. Экспоненциальный фильтр (EMA)
float ema_val = 0;
float ema_med_val = 0;
const float ALPHA = 0.15f;

// 4. Медиана
uint16_t med_buf[3] = {0};
uint8_t med_idx = 0;

inline uint16_t get_median3(uint16_t a, uint16_t b, uint16_t c) {
    if (a > b) { uint16_t t=a; a=b; b=t; }
    if (b > c) { uint16_t t=b; b=c; c=t; }
    if (a > b) { uint16_t t=a; a=b; b=t; }
    return b;
}

#define BTN_READ() ((GPIOA->IDR & GPIO_PIN_0) == 0)
volatile bool g_btn_state = false;

#define DWT_CTRL    (*(volatile uint32_t *)0xE0001000)
#define DWT_CYCCNT  (*(volatile uint32_t *)0xE0001004)
#define DEMCR       (*(volatile uint32_t *)0xE000EDFC)

void Enable_DWT(void) {
    DEMCR |= 0x01000000;
    DWT_CYCCNT = 0;
    DWT_CTRL |= 1;
}

extern "C" {
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM5_Init(void);
}

int main(void) {
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM5_Init();

  Enable_DWT();

  /* Запускаем ШИМ 1 кГц Синус на PA5 (через DMA) */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_DMA_Start(&hdma_tim2_ch1, (uint32_t)sine_lut, (uint32_t)&TIM2->CCR1, 100);
  __HAL_TIM_ENABLE_DMA(&htim2, TIM_DMA_CC1);

  /* Запускаем ШИМ 10 кГц на PA1 */
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);

  /* Запускаем АЦП */
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, ADC_BUF_SIZE);

  uint32_t debounce_timer = 0;
  uint32_t telemetry_timer = 0;
  char json_buf[150];

  uint16_t last_raw=0, last_med=0, last_ma=0, last_ma_med=0;

  while (1) {
    /* Антидребезг кнопки (1 мс) на PA0 */
    if (HAL_GetTick() - debounce_timer >= 1) {
        debounce_timer = HAL_GetTick();
        static uint16_t btn_hist = 0;
        btn_hist = (btn_hist << 1) | (BTN_READ() ? 1 : 0);
        if ((btn_hist & 0x03FF) == 0x03FF) g_btn_state = true;
        if ((btn_hist & 0x03FF) == 0x0000) g_btn_state = false;
    }

    /* Обработка данных АЦП */
    uint32_t process_start = 0;
    uint32_t process_end = 0;
    bool process_now = false;

    if (flag_half_ready) {
        process_start = 0;
        process_end = ADC_BUF_SIZE / 2;
        process_now = true;
    } else if (flag_full_ready) {
        process_start = ADC_BUF_SIZE / 2;
        process_end = ADC_BUF_SIZE;
        process_now = true;
    }

    if (process_now) {
        /* Применяем каскадную фильтрацию к Каналу 3 (Синус) */
        for (uint32_t i = process_start; i < process_end; i += NUM_ADC_CHANNELS) {
            last_raw = adc_buffer[i]; 

            // 1. Медиана
            med_buf[med_idx] = last_raw;
            med_idx = (med_idx + 1) % 3;
            last_med = get_median3(med_buf[0], med_buf[1], med_buf[2]);

            // 2. Экспоненциальное сглаживание
            ema_val = ALPHA * last_raw + (1.0f - ALPHA) * ema_val;
            ema_med_val = ALPHA * last_med + (1.0f - ALPHA) * ema_med_val;

            // 3. Скользящее среднее
            ma_sum -= ma_buf[ma_idx];
            ma_buf[ma_idx] = last_raw;
            ma_sum += last_raw;
            last_ma = ma_sum / MA_SIZE;

            // 4. Скользящее среднее + Медиана
            ma_med_sum -= ma_med_buf[ma_idx];
            ma_med_buf[ma_idx] = last_med;
            ma_med_sum += last_med;
            last_ma_med = ma_med_sum / MA_SIZE;

            ma_idx = (ma_idx + 1) % MA_SIZE;

            // 5. OVERSAMPLING (Увеличение разрядности до 16-бит)
            os_sum -= os_buf[os_idx];
            os_buf[os_idx] = last_raw;
            os_sum += last_raw;
            last_os_16bit = os_sum >> OS_SHIFT; // Результат от 0 до 65535
            os_idx = (os_idx + 1) % OS_N;
        }

        /* ==========================================================
         * СТРЕСС-ТЕСТ (НАГРУЗКА ПО КНОПКЕ)
         * ==========================================================
         * Половина буфера DMA заполняется за ~1.9 мс.
         * Задержка 5 мс заблокирует CPU, и DMA перезапишет данные.
         */
        if (g_btn_state) {
            HAL_Delay(5); 
        }

        /* Только после завершения ВСЕХ вычислений (и задержек) сбрасываем флаг */
        if (process_start == 0) flag_half_ready = 0;
        else flag_full_ready = 0;
    }

    /* Отправка телеметрии (JSON) каждые 50 мс */
    if (HAL_GetTick() - telemetry_timer >= 50) {
        telemetry_timer = HAL_GetTick();

        __disable_irq();
        uint32_t current_missed = missed_blocks;
        __enable_irq();

        // МАШТАБИРОВАНИЕ В ВОЛЬТЫ
        float v_raw = adc12_to_volts(last_raw);
        float v_ema = adc12_to_volts((uint16_t)ema_val);
        float v_os  = adc16_to_volts(last_os_16bit);

        // Хак для вывода float без флагов линкера (формат X.XXXX Вольт)
        int raw_i = (int)v_raw; int raw_f = (int)((v_raw - raw_i) * 10000);
        int ema_i = (int)v_ema; int ema_f = (int)((v_ema - ema_i) * 10000);
        int os_i  = (int)v_os;  int os_f  = (int)((v_os - os_i) * 10000);

        /* Формируем JSON (с Вольтами, оверсэмплингом и счетчиком ошибок) */
        snprintf(json_buf, sizeof(json_buf), 
                 "{\"raw_v\":%d.%04d,\"ema_v\":%d.%04d,\"os_v\":%d.%04d,\"btn\":%d,\"miss\":%lu}\n",
                 raw_i, raw_f, ema_i, ema_f, os_i, os_f, g_btn_state, current_missed);

        HAL_UART_Transmit(&huart1, (uint8_t*)json_buf, strlen(json_buf), 10);
    }
  }
}

/* ==============================================================================
 * ИНИЦИАЛИЗАЦИЯ ПЕРИФЕРИИ (Из предыдущих настроек)
 * ==============================================================================
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) Error_Handler();
}

static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  // PA0 - Кнопка
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // PA2, PA3 - Входы АЦП (Loopback)
  GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // PA1 - TIM5 CH2, PA5 - TIM2 CH1
  GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

static void MX_TIM2_Init(void) {
  TIM_OC_InitTypeDef sConfigOC = {0};
  __HAL_RCC_TIM2_CLK_ENABLE();

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0; 
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 840 - 1; 
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_PWM_Init(&htim2);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1);

  hdma_tim2_ch1.Instance = DMA1_Stream5;
  hdma_tim2_ch1.Init.Channel = DMA_CHANNEL_3;
  hdma_tim2_ch1.Init.Direction = DMA_MEMORY_TO_PERIPH;
  hdma_tim2_ch1.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_tim2_ch1.Init.MemInc = DMA_MINC_ENABLE;
  hdma_tim2_ch1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  hdma_tim2_ch1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
  hdma_tim2_ch1.Init.Mode = DMA_CIRCULAR;
  hdma_tim2_ch1.Init.Priority = DMA_PRIORITY_LOW;
  HAL_DMA_Init(&hdma_tim2_ch1);
  __HAL_LINKDMA(&htim2, hdma[TIM_DMA_ID_CC1], hdma_tim2_ch1);
}

static void MX_TIM5_Init(void) {
  TIM_OC_InitTypeDef sConfigOC = {0};
  __HAL_RCC_TIM5_CLK_ENABLE();

  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0; 
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 8400 - 1; 
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_PWM_Init(&htim5);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 6300; 
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_2); 
}

static void MX_ADC1_Init(void) {
  ADC_ChannelConfTypeDef sConfig = {0};
  __HAL_RCC_ADC1_CLK_ENABLE();

  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4; 
  hadc1.Init.Resolution = ADC_RESOLUTION_12B; 
  hadc1.Init.ScanConvMode = ENABLE;                
  hadc1.Init.ContinuousConvMode = ENABLE;          
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;                  
  hadc1.Init.DMAContinuousRequests = ENABLE;       
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  HAL_ADC_Init(&hadc1);

  sConfig.Channel = ADC_CHANNEL_3; 
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

  sConfig.Channel = ADC_CHANNEL_2; 
  sConfig.Rank = 2;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

  hdma_adc1.Instance = DMA2_Stream0;
  hdma_adc1.Init.Channel = DMA_CHANNEL_0;
  hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
  hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
  hdma_adc1.Init.Mode = DMA_CIRCULAR;
  hdma_adc1.Init.Priority = DMA_PRIORITY_HIGH;
  HAL_DMA_Init(&hdma_adc1);
  __HAL_LINKDMA(&hadc1, DMA_Handle, hdma_adc1);
}

static void MX_DMA_Init(void) {
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
}

static void MX_USART1_UART_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_USART1_CLK_ENABLE();
  
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
  HAL_UART_Init(&huart1);
}

extern "C" {
void DMA2_Stream0_IRQHandler(void) { HAL_DMA_IRQHandler(&hdma_adc1); }

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc) {
    if (hadc->Instance == ADC1) {
        if (flag_half_ready == 1) missed_blocks++; 
        flag_half_ready = 1;
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    if (hadc->Instance == ADC1) {
        uint32_t current_dwt = DWT_CYCCNT;
        dwt_cycles_diff = current_dwt - dwt_timestamp_prev;
        dwt_timestamp_prev = current_dwt;

        if (flag_full_ready == 1) missed_blocks++; 
        flag_full_ready = 1;
    }
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc) {
    if (hadc->ErrorCode & HAL_ADC_ERROR_OVR) missed_blocks++;
}

void Error_Handler(void) {
  __disable_irq();
  while (1) {}
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) Error_Handler();
}

static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  // PA0 - Кнопка
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // PA2, PA3 - Входы АЦП (Loopback)
  GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // PA1 - TIM5 CH2, PA5 - TIM2 CH1
  GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

static void MX_TIM2_Init(void) {
  TIM_OC_InitTypeDef sConfigOC = {0};
  __HAL_RCC_TIM2_CLK_ENABLE();

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0; 
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 840 - 1; // 84 MHz / 840 = 100 kHz Update
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_PWM_Init(&htim2);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1); // PA5

  hdma_tim2_ch1.Instance = DMA1_Stream5;
  hdma_tim2_ch1.Init.Channel = DMA_CHANNEL_3;
  hdma_tim2_ch1.Init.Direction = DMA_MEMORY_TO_PERIPH;
  hdma_tim2_ch1.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_tim2_ch1.Init.MemInc = DMA_MINC_ENABLE;
  hdma_tim2_ch1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  hdma_tim2_ch1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
  hdma_tim2_ch1.Init.Mode = DMA_CIRCULAR;
  hdma_tim2_ch1.Init.Priority = DMA_PRIORITY_LOW;
  HAL_DMA_Init(&hdma_tim2_ch1);
  __HAL_LINKDMA(&htim2, hdma[TIM_DMA_ID_CC1], hdma_tim2_ch1);
}

static void MX_TIM5_Init(void) {
  TIM_OC_InitTypeDef sConfigOC = {0};
  __HAL_RCC_TIM5_CLK_ENABLE();

  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0; 
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 8400 - 1; // 84 MHz / 8400 = 10 kHz
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_PWM_Init(&htim5);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 6300; // Duty 3/4 = 75%
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_2); // PA1
}

static void MX_ADC1_Init(void) {
  ADC_ChannelConfTypeDef sConfig = {0};
  __HAL_RCC_ADC1_CLK_ENABLE();

  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4; // 21 MHz
  hadc1.Init.Resolution = ADC_RESOLUTION_12B; 
  hadc1.Init.ScanConvMode = ENABLE;                
  hadc1.Init.ContinuousConvMode = ENABLE;          
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;                  
  hadc1.Init.DMAContinuousRequests = ENABLE;       
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  HAL_ADC_Init(&hadc1);

  // Настройка выборки 28 циклов. Итого 40 циклов на канал.
  sConfig.Channel = ADC_CHANNEL_3; // PA3 (читает синус с PA5)
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

  sConfig.Channel = ADC_CHANNEL_2; // PA2 (читает ШИМ с PA1)
  sConfig.Rank = 2;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

  hdma_adc1.Instance = DMA2_Stream0;
  hdma_adc1.Init.Channel = DMA_CHANNEL_0;
  hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
  hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
  hdma_adc1.Init.Mode = DMA_CIRCULAR;
  hdma_adc1.Init.Priority = DMA_PRIORITY_HIGH;
  HAL_DMA_Init(&hdma_adc1);
  __HAL_LINKDMA(&hadc1, DMA_Handle, hdma_adc1);
}

static void MX_DMA_Init(void) {
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
}

static void MX_USART1_UART_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_USART1_CLK_ENABLE();
  
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
  HAL_UART_Init(&huart1);
}

extern "C" {
void DMA2_Stream0_IRQHandler(void) { HAL_DMA_IRQHandler(&hdma_adc1); }

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc) {
    if (hadc->Instance == ADC1) {
        if (flag_half_ready == 1) missed_blocks++; 
        flag_half_ready = 1;
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    if (hadc->Instance == ADC1) {
        uint32_t current_dwt = DWT_CYCCNT;
        dwt_cycles_diff = current_dwt - dwt_timestamp_prev;
        dwt_timestamp_prev = current_dwt;

        if (flag_full_ready == 1) missed_blocks++; 
        flag_full_ready = 1;
    }
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc) {
    if (hadc->ErrorCode & HAL_ADC_ERROR_OVR) missed_blocks++;
}

void Error_Handler(void) {
  __disable_irq();
  while (1) {}
}
}