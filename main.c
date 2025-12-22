#include "main.h"
#include "stdio.h"
#include "stdlib.h"
#include "ctype.h"
#include "string.h"
#include "cmsis_os.h"
#include "usb_host.h"
#include "pid_controller.h"

//*******************************************************************************************************

ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

DMA_HandleTypeDef hdma_adc1;

osMessageQueueId_t ADC_QueueHandle;

osMessageQueueId_t UART_QueueHandle;

#define ADC_QUEUE_LENGTH   1
#define ADC_QUEUE_ITEMSIZE sizeof(adc_data_t)

char adc_uart_buf[64];
extern float filtered_degree;
extern const float ALPHA;

volatile uint8_t rx_idx = 0;
volatile uint8_t rx_completed = 0;
uint8_t g_rx_byte;
char rx_cmd_buf[32];

typedef struct
{
    uint16_t x;
    uint16_t y;
} adc_data_t;

typedef struct {
    float degree_x;
    float degree_y;
    float setpoint_x;
    float setpoint_y;
    float pwm_x;
    float pwm_y;
    uint16_t adc_x;
    uint16_t adc_y;
} UART_Data_t;


typedef struct {
    float degree;
    float setpoint;
    float pwm;
    uint16_t adc_raw;
} SharedData_t;


SharedData_t g_telemetry_data;

osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

osThreadId_t ADC_TaskHandle;
const osThreadAttr_t ADC_Task_attributes = {
  .name = "ADC_Task",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t UART_TaskHandle;
const osThreadAttr_t UART_Task_attributes = {
  .name = "UART_Task",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t CONTROL_TaskHandle;
const osThreadAttr_t CONTROL_Task_attributes = {
  .name = "CONTROL_Task",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

osSemaphoreId_t Sem_UART_Rx_DoneHandle;
const osSemaphoreAttr_t Sem_UART_Rx_Done_attributes = {
  .name = "Sem_UART_Rx_Done"
};


void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);

void StartDefaultTask(void *argument);
void adc_task(void *argument);
void uart_task(void *argument);
void control_task(void *argument);

//*******************************************************************************************************

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();


  osKernelInitialize();

  ADC_QueueHandle = osMessageQueueNew(
                      ADC_QUEUE_LENGTH,
                      ADC_QUEUE_ITEMSIZE,
                      NULL);

  if (ADC_QueueHandle == NULL)
  {
      Error_Handler();
  }


  UART_QueueHandle = osMessageQueueNew(5, sizeof(UART_Data_t), NULL);

  if (UART_QueueHandle == NULL) {
      Error_Handler();
  }


  Sem_UART_Rx_DoneHandle = osSemaphoreNew(1, 1, &Sem_UART_Rx_Done_attributes);

  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  ADC_TaskHandle = osThreadNew(adc_task, NULL, &ADC_Task_attributes);

  UART_TaskHandle = osThreadNew(uart_task, NULL, &UART_Task_attributes);

  CONTROL_TaskHandle = osThreadNew(control_task, NULL, &CONTROL_Task_attributes);

  HAL_UART_Receive_IT(&huart2, &g_rx_byte, 1);

  osKernelStart();

  while (1)
  {

  }

}

//*******************************************************************************************************

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_ADC1_Init(void)
{

  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;

  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  // 1. Kanal: PA1 (X-Ekseni)
      ADC_ChannelConfTypeDef sConfig1 = {0};
      sConfig1.Channel = ADC_CHANNEL_1; // PA1 pinine karşılık gelir
      sConfig1.Rank = 1; // Birinci sırada okunacak
      sConfig1.SamplingTime = ADC_SAMPLETIME_84CYCLES;
      if (HAL_ADC_ConfigChannel(&hadc1, &sConfig1) != HAL_OK) { Error_Handler(); }

      // 2. Kanal: PD6 (Y-Ekseni)
      ADC_ChannelConfTypeDef sConfig2 = {0};
      sConfig2.Channel = ADC_CHANNEL_3; // PA3 pinine karşılık gelir
      sConfig2.Rank = 2; // İkinci sırada okunacak
      sConfig2.SamplingTime = ADC_SAMPLETIME_84CYCLES;
      if (HAL_ADC_ConfigChannel(&hadc1, &sConfig2) != HAL_OK) { Error_Handler(); }

}


static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 83;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 0;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;

  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_NVIC_SetPriority(USART2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);

}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {

    	 HAL_GPIO_TogglePin(GPIOD, LD3_Pin); //Debug

        if (rx_completed == 0) {
            if (g_rx_byte == '\n' || g_rx_byte == '\r') {
                if (rx_idx > 0) {
                    rx_cmd_buf[rx_idx] = '\0';
                    rx_completed = 1;
                }
            } else {
                if (rx_idx < sizeof(rx_cmd_buf) - 1) {
                    rx_cmd_buf[rx_idx++] = g_rx_byte;
                }
            }
        }
        HAL_UART_Receive_IT(&huart2, &g_rx_byte, 1);
    }
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  // ===== ADC PINS: PA1 & PA3 ANALOG =====
  GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);


  // ===== USART2 TX (PA2) =====
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // ===== USART2 RX (PD6) =====
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  // Green LED PD12
   GPIO_InitStruct.Pin = LD4_Pin;
   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
   HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

   // RedLED PD14
    GPIO_InitStruct.Pin = LD5_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);


  /*Configure GPIO pin : PE3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : I2S3_WS_Pin */
  GPIO_InitStruct.Pin = I2S3_WS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(I2S3_WS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI1_SCK_Pin SPI1_MISO_Pin SPI1_MOSI_Pin */
  GPIO_InitStruct.Pin = SPI1_SCK_Pin|SPI1_MISO_Pin|SPI1_MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : I2S3_MCK_Pin I2S3_SCK_Pin I2S3_SD_Pin */
  GPIO_InitStruct.Pin = I2S3_MCK_Pin|I2S3_SCK_Pin|I2S3_SD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Audio_SCL_Pin Audio_SDA_Pin */
  GPIO_InitStruct.Pin = Audio_SCL_Pin|Audio_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

}

//*******************************************************************************************************


void StartDefaultTask(void *argument)
{

  MX_USB_HOST_Init();

  for(;;)
  {

    osDelay(1);
  }

}


void adc_task(void *argument)
{
    adc_data_t adc_data;
    for (;;)
    {
        HAL_ADC_Start(&hadc1);

        // X Ekseni
        HAL_ADC_PollForConversion(&hadc1, 10);
        adc_data.x = HAL_ADC_GetValue(&hadc1);

        // Y Ekseni
        HAL_ADC_PollForConversion(&hadc1, 10);
        adc_data.y = HAL_ADC_GetValue(&hadc1);

        HAL_ADC_Stop(&hadc1);

        osMessageQueuePut(ADC_QueueHandle, &adc_data, 0, 0);
        osDelay(1);
    }
}


void uart_task(void *argument) {
    UART_Data_t log_data;

    for(;;) {
        // Kesme arka planda buffer ı doldurma
        if (rx_completed) {
            // Mavi LED debug icin
            HAL_GPIO_WritePin(GPIOD, LD6_Pin, GPIO_PIN_SET);

            char *ptr = rx_cmd_buf;
            // Sayı kısmına kadar ilerle harfleri atla
            while (*ptr && !((*ptr >= '0' && *ptr <= '9') || *ptr == '-' || *ptr == '.')) ptr++;

            if (*ptr != '\0') {
                float val = (float)atof(ptr);
                char mode = rx_cmd_buf[0];
                char axis = rx_cmd_buf[1];

                // Kırmızı LED (LD5)  X komutu geldiğinde yanar
                if (mode == 'X') {
                    g_control_data.setpoint_x = val;
                    HAL_GPIO_TogglePin(GPIOD, LD5_Pin);
                }
                else if (mode == 'Y') g_control_data.setpoint_y = val;
                else if (mode == 'P') { if (axis == 'X') pid_x.Kp = val; else if (axis == 'Y') pid_y.Kp = val; }
                else if (mode == 'I') { if (axis == 'X') pid_x.Ki = val; else if (axis == 'Y') pid_y.Ki = val; }
                else if (mode == 'D') { if (axis == 'X') pid_x.Kd = val; else if (axis == 'Y') pid_y.Kd = val; }

                // Yeşil LED (LD4)  Her basarılı komutta yanar
                HAL_GPIO_WritePin(GPIOD, LD4_Pin, GPIO_PIN_SET);
            }

            // Temizlik ve Sıfırlama
            rx_idx = 0;
            rx_completed = 0; // Kesmeyi sifirlama yeni paket almak icin
            memset(rx_cmd_buf, 0, sizeof(rx_cmd_buf));

            osDelay(5); //  Led leri gormek icin cok kia bir bekleme
            HAL_GPIO_WritePin(GPIOD, LD4_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOD, LD6_Pin, GPIO_PIN_RESET);
        }

        // Telemetri gonderimi
        if (osMessageQueueGet(UART_QueueHandle, &log_data, NULL, 0) == osOK) {
            char uart_buf[100];
            int len = snprintf(uart_buf, sizeof(uart_buf), "%.2f,%.2f,%.2f,%.2f\n",
                               log_data.degree_x, log_data.setpoint_x,
                               log_data.degree_y, log_data.setpoint_y);
            HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, len, 10);
        }

        // Hata kontrolu
        if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_ORE)) {
            __HAL_UART_CLEAR_OREFLAG(&huart2);
            HAL_UART_Receive_IT(&huart2, &g_rx_byte, 1);
        }

        osDelay(10);
    }
}

void control_task(void *argument)
{
    adc_data_t adc_data;
    UART_Data_t log_data;

    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 1650);

    for (;;)
    {
        if (osMessageQueueGet(ADC_QueueHandle, &adc_data, NULL, osWaitForever) == osOK) {
            // X Ekseni
            float cur_deg_x = ADC_To_Degree(adc_data.x);
            static float filt_deg_x = 0;
            filt_deg_x = (ALPHA * cur_deg_x) + ((1.0f - ALPHA) * filt_deg_x);
            float pwm_x = Calculate_PID(&pid_x, filt_deg_x, g_control_data.setpoint_x);

            // Y Ekseni
            float cur_deg_y = ADC_To_Degree(adc_data.y);
            static float filt_deg_y = 0;
            filt_deg_y = (ALPHA * cur_deg_y) + ((1.0f - ALPHA) * filt_deg_y);
            float pwm_y = Calculate_PID(&pid_y, filt_deg_y, g_control_data.setpoint_y);

            // PWM Sinirlari
            if (pwm_x < 400) {pwm_x = 400;} if (pwm_x > 2400) {pwm_x = 2400;}
            if (pwm_y < 400) {pwm_y = 400;} if (pwm_y > 2400) {pwm_y = 2400;}

            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint16_t)pwm_x);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (uint16_t)pwm_y);

            // UART Kuyruğuna(Queue) veriyi 0 bekleme ile gönder
            // Eğer kuyruk doluysa program o veriyi pas gec
            log_data.degree_x = filt_deg_x;
            log_data.setpoint_x = g_control_data.setpoint_x;
            log_data.pwm_x = pwm_x;
            log_data.degree_y = filt_deg_y;
            log_data.setpoint_y = g_control_data.setpoint_y;
            log_data.pwm_y = pwm_y;

            osMessageQueuePut(UART_QueueHandle, &log_data, 0, 0);
        }

    }
}

//*******************************************************************************************************

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

  if (htim->Instance == TIM1)
  {
    HAL_IncTick();
  }

}

void Error_Handler(void)
{

  __disable_irq();
  while (1)
  {
  }

}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
