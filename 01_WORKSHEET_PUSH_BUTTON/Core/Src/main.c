#include "main.h"
#include <stdbool.h>

/* Pin Definitions */
#define LED_PIN         GPIO_PIN_5
#define LED_PORT        GPIOA

#define BUTTON1_PIN     GPIO_PIN_13
#define BUTTON1_PORT    GPIOC

#define BUTTON2_PIN     GPIO_PIN_0
#define BUTTON2_PORT    GPIOB

/* Timing Constants */
#define DEBOUNCE_DELAY      50      // ms
#define LONG_PRESS_TIME    2000     // ms
#define BLINK_INTERVAL     100      // ms

/* Variables */
uint8_t ledState = 0;

uint8_t lastButton1State = 1;
uint32_t lastDebounce1 = 0;
bool button1Pressed = false;
uint32_t pressStart1 = 0;

uint8_t lastButton2State = 1;
uint32_t lastDebounce2 = 0;
bool button2Pressed = false;
uint32_t pressStart2 = 0;

/* Non-blocking blink variables */
bool blinking = false;
uint8_t blinkTimes = 0;
uint32_t blinkStartTime = 0;
bool blinkLEDState = false;

/* Function Prototypes */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void toggleLED(void);
void startBlink(uint8_t times);
void processBlink(void);

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();

    while (1)
    {
        /* -------- BUTTON 1 (toggle / long press) -------- */
        uint8_t reading1 = HAL_GPIO_ReadPin(BUTTON1_PORT, BUTTON1_PIN);

        if (reading1 != lastButton1State)
            lastDebounce1 = HAL_GetTick();

        if ((HAL_GetTick() - lastDebounce1) > DEBOUNCE_DELAY)
        {
            // Pressed
            if (reading1 == GPIO_PIN_RESET && !button1Pressed)
            {
                button1Pressed = true;
                pressStart1 = HAL_GetTick();
            }

            // Released
            if (reading1 == GPIO_PIN_SET && button1Pressed)
            {
                uint32_t duration = HAL_GetTick() - pressStart1;
                button1Pressed = false;

                if (duration >= LONG_PRESS_TIME)
                    startBlink(3);  // Long press -> 3 blinks
                else
                    toggleLED();    // Short press -> toggle
            }
        }
        lastButton1State = reading1;

        /* -------- BUTTON 2 (double-blink or reset) -------- */
        uint8_t reading2 = HAL_GPIO_ReadPin(BUTTON2_PORT, BUTTON2_PIN);

        if (reading2 != lastButton2State)
            lastDebounce2 = HAL_GetTick();

        if ((HAL_GetTick() - lastDebounce2) > DEBOUNCE_DELAY)
        {
            if (reading2 == GPIO_PIN_RESET && !button2Pressed)
            {
                button2Pressed = true;
                pressStart2 = HAL_GetTick();
            }

            if (reading2 == GPIO_PIN_SET && button2Pressed)
            {
                button2Pressed = false;
                startBlink(2);  // Button2 -> double blink
                ledState = 0;
                HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
            }
        }
        lastButton2State = reading2;

        /* -------- Non-blocking blink handler -------- */
        processBlink();
    }
}

/* Toggle LED immediately */
void toggleLED(void)
{
    ledState = !ledState;
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, ledState ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* Start non-blocking blink */
void startBlink(uint8_t times)
{
    blinking = true;
    blinkTimes = times * 2; // ON + OFF counts
    blinkStartTime = HAL_GetTick();
    blinkLEDState = true;
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
}

/* Process non-blocking blink */
void processBlink(void)
{
    if (!blinking) return;

    if ((HAL_GetTick() - blinkStartTime) >= BLINK_INTERVAL)
    {
        blinkLEDState = !blinkLEDState;
        HAL_GPIO_WritePin(LED_PORT, LED_PIN, blinkLEDState ? GPIO_PIN_SET : GPIO_PIN_RESET);
        blinkStartTime = HAL_GetTick();
        blinkTimes--;

        if (blinkTimes == 0)
            blinking = false;
    }
}

/* GPIO Initialization (adjust for your pins) */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    // LED
    GPIO_InitStruct.Pin = LED_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);

    // Button1
    GPIO_InitStruct.Pin = BUTTON1_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(BUTTON1_PORT, &GPIO_InitStruct);

    // Button2
    GPIO_InitStruct.Pin = BUTTON2_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(BUTTON2_PORT, &GPIO_InitStruct);
}

/* System Clock Configuration (CubeMX default) */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 1;
    RCC_OscInitStruct.PLL.PLLN = 10;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|
                                  RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4);
}

/* Error Handler */
void Error_Handler(void)
{
    __disable_irq();
    while (1) { }
}
