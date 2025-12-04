/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include<stdio.h>
#include<string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
// Define a command structure to hold command number and optional arguments
typedef struct {
    uint8_t cmd_no;       // Command number
    uint8_t cmd_arg[10];  // Arguments for the command (currently unused)
} APP_CMD_t;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;
UART_HandleTypeDef huart2;

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */

char msg[100];                        // Buffer for UART messages
uint8_t command_buffer[20];           // Buffer to store UART input
uint8_t command_length = 0;           // Length of command input

// Menu string displayed on UART
char menu[] =
    "\r\n1 -> LED_ON          "
    "\r\n2 -> LED_OFF         "
    "\r\n3 -> LED_TOGGLE      "
    "\r\n4 -> LED_READ_STATUS "
    "\r\n5 -> RTC_PRINT       "
    "\r\n0 -> EXIT            "
    "\r\nENTER YOUR OPTION:\r\n";

// Command codes
#define LED_ON          1
#define LED_OFF         2
#define LED_TOGGLE      3
#define LED_READ_STATUS 4
#define RTC_READ        5

// Task handles
TaskHandle_t xTaskHandle1=NULL;
TaskHandle_t xTaskHandle2=NULL;
TaskHandle_t xTaskHandle3=NULL;
TaskHandle_t xTaskHandle4=NULL;

// Queues
QueueHandle_t command_queue;     // Queue for command processing
QueueHandle_t uart_write_queue;  // Queue for UART output

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_RTC_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
uint8_t getCommandCode(uint8_t *buffer);   // Extract numeric command from buffer

void make_led_on(void);                     // Turn LED on
void make_led_off(void);                    // Turn LED off
void led_toggle(void);                      // Toggle LED
void read_led_status(char *task_msg);      // Send LED status to UART
void read_rtc_info(char *task_msg);        // Send RTC info to UART
void print_error_message(char *task_msg);  // Send error message to UART

uint8_t rx_byte;                            // UART received byte
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
// UART receive interrupt callback
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (huart->Instance == USART2)
    {
        // Echo received byte back to UART
        HAL_UART_Transmit_IT(&huart2, &rx_byte, 1);

        // Store byte in command buffer
        command_buffer[command_length++] = rx_byte;

        // If Enter key is pressed, notify Task1 (menu) and Task2 (command handler)
        if (rx_byte == '\r')
        {
            command_length = 0;
            vTaskNotifyGiveFromISR(xTaskHandle1, &xHigherPriorityTaskWoken);
            vTaskNotifyGiveFromISR(xTaskHandle2, &xHigherPriorityTaskWoken);
        }

        // Start next UART receive interrupt
        HAL_UART_Receive_IT(&huart2, &rx_byte, 1);

        // Yield if higher priority task was woken
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

// Simple function to send string to UART
void uart_msg(char *s)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)s, strlen(s), 1000);
}
/* USER CODE END 0 */

/* USER CODE BEGIN 4 */

// Task1: Display menu on UART
void Task1_menu_display(void *params)
{
    char *pdata = menu;
    while(1)
    {
        xQueueSend(uart_write_queue, &pdata, portMAX_DELAY); // Send menu string to UART task
        xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);          // Wait until Enter is pressed
    }
}

// Task2: Handle input commands
void Task2_cmd_handling(void *params)
{
    uint8_t cmd_code = 0;
    APP_CMD_t *new_cmd;

    while(1)
    {
        xTaskNotifyWait(0,0,NULL,portMAX_DELAY);           // Wait for notification from ISR

        // Allocate memory for command
        new_cmd = (APP_CMD_t*)pvPortMalloc(sizeof(APP_CMD_t));

        taskENTER_CRITICAL();
        cmd_code = getCommandCode(command_buffer);       // Extract numeric command
        new_cmd->cmd_no = cmd_code;
        taskEXIT_CRITICAL();

        // Send command pointer to processing queue
        xQueueSend(command_queue, &new_cmd, portMAX_DELAY);
    }
}

// Task3: Process commands
void Task3_cmd_processing(void *params)
{
    APP_CMD_t *new_cmd;
    char task_msg[50];

    while(1)
    {
        // Receive command from queue
        xQueueReceive(command_queue, (void*)&new_cmd, portMAX_DELAY);

        // Execute corresponding action
        if(new_cmd->cmd_no == LED_ON)
        {
            make_led_on();
        }
        else if(new_cmd->cmd_no == LED_OFF)
        {
            make_led_off();
        }
        else if(new_cmd->cmd_no == LED_TOGGLE)
        {
            led_toggle();
        }
        else if(new_cmd->cmd_no == LED_READ_STATUS)
        {
            read_led_status(task_msg);
        }
        else if(new_cmd->cmd_no == RTC_READ)
        {
            read_rtc_info(task_msg);
        }
        else
        {
            print_error_message(task_msg);
        }

        vPortFree(new_cmd); // Free allocated memory
    }
}

// Task4: Write messages to UART
void Task4_uart_write(void *params)
{
    char *data = NULL;
    while(1)
    {
         xQueueReceive(uart_write_queue, &data, portMAX_DELAY); // Wait for message
         uart_msg(data);                                        // Send to UART
    }
}

// Extract numeric command from buffer
uint8_t getCommandCode(uint8_t *buffer)
{
    return buffer[0] - 48; // Convert ASCII '0'-'9' to number
}

// LED control functions
void make_led_on(void)  { HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, SET); }
void make_led_off(void) { HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, RESET); }
void led_toggle(void)   { HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5); }

// Read LED status and send to UART
void read_led_status(char *task_msg)
{
    sprintf(task_msg , "\r\nLED status is : %d\r\n", HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5));
    xQueueSend(uart_write_queue, &task_msg, portMAX_DELAY);
}

// Read RTC time/date and send to UART
void read_rtc_info(char *task_msg)
{
    RTC_TimeTypeDef sTime = {0};
    RTC_DateTypeDef sDate = {0};

    // Initialize RTC if not already initialized
    if ((RTC->ISR & RTC_ISR_INITS) == 0)
    {
        sTime.Hours = 5; sTime.Minutes = 0; sTime.Seconds = 0;
        sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
        sTime.StoreOperation = RTC_STOREOPERATION_RESET;
        HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);

        sDate.WeekDay = RTC_WEEKDAY_THURSDAY;
        sDate.Month = RTC_MONTH_SEPTEMBER;
        sDate.Date = 11;
        sDate.Year = 25;
        HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
    }

    HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

    sprintf(task_msg, "Time: %02d:%02d:%02d | Date: %02d-%02d-20%02d\r\n",
            sTime.Hours, sTime.Minutes, sTime.Seconds,
            sDate.Date, sDate.Month, sDate.Year);

    xQueueSend(uart_write_queue, &task_msg, portMAX_DELAY);
}

// Send error message to UART
void print_error_message(char *task_msg)
{
    sprintf(task_msg, "\r\nInvalid command received\r\n");
    xQueueSend(uart_write_queue, &task_msg, portMAX_DELAY);
}


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
 // osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
 // defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  sprintf(msg,"\r\nQUEUE IMPLEMETATION \r\n");
  uart_msg(msg);

  command_queue = xQueueCreate(10, sizeof(APP_CMD_t*));

  uart_write_queue = xQueueCreate(10, sizeof(char*));

  /* add threads, ... */
  if((command_queue != NULL ) && (uart_write_queue != NULL ))
  {
      // Create tasks
      xTaskCreate(Task1_menu_display, "TASK 1-MENU", configMINIMAL_STACK_SIZE, NULL, 1, &xTaskHandle1);
      xTaskCreate(Task2_cmd_handling, "TASK 2-CMD_HANDLING", configMINIMAL_STACK_SIZE, NULL, 2, &xTaskHandle2);
      xTaskCreate(Task3_cmd_processing, "TASK 3-CMD_PROCESS", configMINIMAL_STACK_SIZE, NULL, 2, &xTaskHandle3);
      xTaskCreate(Task4_uart_write, "TASK 4-UART_WRITE", configMINIMAL_STACK_SIZE, NULL, 2, &xTaskHandle4);

      HAL_UART_Receive_IT(&huart2, &rx_byte, 1); // Start UART receive interrupt

      osKernelStart(); // Start FreeRTOS scheduler
  }
  else
  {
	  sprintf(msg,"QUEUE Creation Failed\r\n");
	  uart_msg(msg);
  }
  /* USER CODE END RTOS_THREADS */



  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
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
