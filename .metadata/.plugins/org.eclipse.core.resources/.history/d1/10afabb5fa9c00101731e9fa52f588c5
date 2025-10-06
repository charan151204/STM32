#include "main.h"                          // Include main header file

uint8_t rx_data;                           // Variable to store received byte
UART_HandleTypeDef huart2;                 // Handle for UART2

void SystemClock_Config(void);             // Function to configure system clock
static void MX_GPIO_Init(void);            // Function to initialize GPIO
static void MX_USART2_UART_Init(void);     // Function to initialize UART2

// Callback function called when UART receive is complete
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)           // Check if source is USART2
  {
    HAL_UART_Transmit(&huart2, &rx_data, 1, 100); // Send received byte back
    HAL_UART_Receive_IT(&huart2, &rx_data, 1);    // Restart UART receive interrupt
  }
}

int main(void)
{
  HAL_Init();                              // Initialize HAL library
  SystemClock_Config();                    // Configure system clock
  MX_GPIO_Init();                          // Initialize GPIO
  MX_USART2_UART_Init();                   // Initialize UART2
  HAL_UART_Receive_IT(&huart2, &rx_data, 1); // Enable UART receive interrupt

  while (1)                                // Infinite loop
  {
  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0}; // Oscillator configuration struct
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0}; // Clock configuration struct

  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK) // Set power scaling
  {
    Error_Handler();                       // Error if configuration fails
  }

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI; // Select HSI oscillator
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;                   // Turn on HSI
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT; // Default calibration
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;               // Enable PLL
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;       // PLL source = HSI
  RCC_OscInitStruct.PLL.PLLM = 1;                            // PLL divider M
  RCC_OscInitStruct.PLL.PLLN = 10;                           // PLL multiplier N
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;                // PLL divider P
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;                // PLL divider Q
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;                // PLL divider R
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)       // Apply oscillator config
  {
    Error_Handler();                     // Error if configuration fails
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                              | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2; // Select clocks
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;  // System clock = PLL
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;         // AHB divider = 1
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;          // APB1 divider = 1
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;          // APB2 divider = 1

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) // Apply clock config
  {
    Error_Handler();                     // Error if configuration fails
  }
}

static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;                       // Select USART2
  huart2.Init.BaudRate = 115200;                  // Set baud rate
  huart2.Init.WordLength = UART_WORDLENGTH_8B;    // 8-bit word length
  huart2.Init.StopBits = UART_STOPBITS_1;         // 1 stop bit
  huart2.Init.Parity = UART_PARITY_NONE;          // No parity
  huart2.Init.Mode = UART_MODE_TX_RX;             // Enable transmit and receive
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;    // No hardware flow control
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;// Oversampling by 16
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE; // Disable one-bit sampling
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT; // No advanced features
  if (HAL_UART_Init(&huart2) != HAL_OK)           // Initialize UART
  {
    Error_Handler();                  // Error if initialization fails
  }
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0}; // GPIO configuration struct

  __HAL_RCC_GPIOC_CLK_ENABLE();           // Enable clock for GPIOC
  __HAL_RCC_GPIOH_CLK_ENABLE();           // Enable clock for GPIOH
  __HAL_RCC_GPIOA_CLK_ENABLE();           // Enable clock for GPIOA
  __HAL_RCC_GPIOB_CLK_ENABLE();           // Enable clock for GPIOB

  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET); // Reset LED pin

  GPIO_InitStruct.Pin = LD2_Pin;          // Select LED pin
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // Output push-pull mode
  GPIO_InitStruct.Pull = GPIO_NOPULL;     // No pull-up or pull-down
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; // Low speed
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct); // Initialize LED pin
}

void Error_Handler(void)
{
  __disable_irq();                        // Disable all interrupts
  while (1)                               // Stay here if error occurs
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) // Assert failure handler
{
}
#endif
