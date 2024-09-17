/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
// Define LCD Pins
#define LCD_RS_GPIO_Port GPIOA
#define LCD_RS_Pin GPIO_PIN_0

#define LCD_EN_GPIO_Port GPIOA
#define LCD_EN_Pin GPIO_PIN_1

#define LCD_D4_GPIO_Port GPIOB
#define LCD_D4_Pin GPIO_PIN_12

#define LCD_D5_GPIO_Port GPIOB
#define LCD_D5_Pin GPIO_PIN_13

#define LCD_D6_GPIO_Port GPIOB
#define LCD_D6_Pin GPIO_PIN_14

#define LCD_D7_GPIO_Port GPIOB
#define LCD_D7_Pin GPIO_PIN_15

// Define delay for LCD (HAL_Delay uses ms)
#define LCD_DELAY HAL_Delay(5)

#define DEBOUNCE_DELAY 50
// Function Prototypes.
void lcd_enable(void);
void lcd_send_nibble(uint8_t nibble);
void lcd_send(uint8_t data, uint8_t rs);
void lcd_send_command(uint8_t command);
void lcd_send_data(uint8_t data);
void lcd_init(void);
void lcd_clear(void);
void lcd_set_cursor(uint8_t row, uint8_t col);
void lcd_send_string(char *str);

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
    // Semaphores
  SemaphoreHandle_t buttonSemaphore;
  SemaphoreHandle_t lcdMutex;

  // Shared Variables
  volatile uint8_t button1_state = 0;
  volatile uint8_t button2_state = 0;
  volatile uint8_t led_red_state = 0;
  volatile uint8_t led_green_state = 0;

  // Task Prototypes
  void vTaskButtonRead(void *pvParameters);
  void vTaskLEDControl(void *pvParameters);
  void vTaskLCDUpdate(void *pvParameters);



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
static void task1_handler(void* parameters);
static void task2_handler(void* parameters);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Send Enable Pulse to latch the data/command
void lcd_enable(void) {
    HAL_GPIO_WritePin(LCD_EN_GPIO_Port, LCD_EN_Pin, GPIO_PIN_SET); // EN High
    LCD_DELAY;
    HAL_GPIO_WritePin(LCD_EN_GPIO_Port, LCD_EN_Pin, GPIO_PIN_RESET); // EN Low
    LCD_DELAY;
}

// Send 4 bits to the LCD
void lcd_send_nibble(uint8_t nibble) {
    HAL_GPIO_WritePin(LCD_D4_GPIO_Port, LCD_D4_Pin, (nibble >> 0) & 0x01);
    HAL_GPIO_WritePin(LCD_D5_GPIO_Port, LCD_D5_Pin, (nibble >> 1) & 0x01);
    HAL_GPIO_WritePin(LCD_D6_GPIO_Port, LCD_D6_Pin, (nibble >> 2) & 0x01);
    HAL_GPIO_WritePin(LCD_D7_GPIO_Port, LCD_D7_Pin, (nibble >> 3) & 0x01);
    lcd_enable();
}

// Send Command or Data (RS low for command, RS high for data)
void lcd_send(uint8_t data, uint8_t rs) {
    HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, rs); // Set RS

    // Send upper nibble
    lcd_send_nibble(data >> 4);
    // Send lower nibble
    lcd_send_nibble(data & 0x0F);
}

// Send Command
void lcd_send_command(uint8_t command) {
    lcd_send(command, 0); // RS = 0 for command
}

// Send Data
void lcd_send_data(uint8_t data) {
    lcd_send(data, 1); // RS = 1 for data
}

void lcd_init(void) {
	  HAL_Delay(50);

	  // Initialize LCD in 4-bit mode
	     lcd_send_nibble(0x03); // Send 0x03 three times (reset sequence)
	     HAL_Delay(5);
	     lcd_send_nibble(0x03);
	     HAL_Delay(1);
	     lcd_send_nibble(0x03);
	     lcd_send_nibble(0x02); // Set to 4-bit mode

	     // Now configure LCD
	     lcd_send_command(0x28); // 4-bit mode, 2 lines, 5x7 format
	     HAL_Delay(1);
	     lcd_send_command(0x0C); // Display ON, cursor OFF
	     HAL_Delay(1);
	     lcd_send_command(0x01); // Clear display
	     HAL_Delay(2);
	     lcd_send_command(0x06); // Entry mode, auto-increment cursor
	     HAL_Delay(1);
}

// Clear LCD display
void lcd_clear(void) {
    lcd_send_command(0x01);  // Clear display
    HAL_Delay(2);            // Wait for the clear operation
}

// Set cursor position (row: 0 or 1, col: 0-15)
void lcd_set_cursor(uint8_t row, uint8_t col) {
    uint8_t address = (row == 0) ? col : (0x40 + col);
    lcd_send_command(0x80 | address);  // Set DDRAM address command
}

// Send string to LCD
void lcd_send_string(char *str) {
    while (*str) {
        lcd_send_data(*str++);
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */

int main(void)
{
  /* USER CODE BEGIN 1 */
//   TaskHandle_t task1_handle;
//   TaskHandle_t task2_handle;
//
//   BaseType_t store_status;
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
  /* USER CODE BEGIN 2 */

  lcd_init();
   /* Semaphore Creation */
  buttonSemaphore = xSemaphoreCreateBinary();
  lcdMutex = xSemaphoreCreateMutex();

  /* Create the tasks */
  xTaskCreate(vTaskButtonRead, "ButtonRead", 128, NULL, 1, NULL);
  xTaskCreate(vTaskLEDControl, "LEDControl", 128, NULL, 1, NULL);
  xTaskCreate(vTaskLCDUpdate,"LCDUpdate",128,NULL,1,NULL);

   /* Start the scheduler */
  vTaskStartScheduler();




  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  // LED TURN ON
//   HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_SET); //PC6 - HIGH -TURN ON
//   HAL_Delay(500);
//
//   HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_SET); //PB13 - HIGH -TURN ON
//   HAL_Delay(500);
//
//   HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_SET); //PB14 - HIGH -TURN ON
//   HAL_Delay(500);
//
//   HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_SET); //PB15- HIGH -TURN ON
//   HAL_Delay(500);
//
//   // LED TURN OFF
//   HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_RESET);// PC6 - LOW -TURN OFF
//   HAL_Delay(500);
//
//   HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_RESET);// PB13 - LOW -TURN OFF
//   HAL_Delay(500);
//
//   HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_RESET);// PB14 - LOW -TURN OFF
//   HAL_Delay(500);
//
//   HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_RESET);// PB15 - LOW -TURN OFF
//   HAL_Delay(500);
//	  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);  // Assuming LED on PB0
//	  HAL_Delay(500);  // 500ms delay

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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LCD_RS_Pin|LCD_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LCD_D4_Pin|LCD_D5_Pin|LED_LCD_D6_Pin|LED4_LCD_D7_Pin
                          |Switch2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LCD_RS_Pin LCD_EN_Pin */
  GPIO_InitStruct.Pin = LCD_RS_Pin|LCD_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_D4_Pin LCD_D5_Pin LED_LCD_D6_Pin LED4_LCD_D7_Pin
                           Switch2_Pin */
  GPIO_InitStruct.Pin = LCD_D4_Pin|LCD_D5_Pin|LED_LCD_D6_Pin|LED4_LCD_D7_Pin
                          |Switch2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LED1_Pin */
  GPIO_InitStruct.Pin = LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Switch4_Pin */
  GPIO_InitStruct.Pin = Switch4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Switch4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Switch3_Pin Switch1_Pin */
  GPIO_InitStruct.Pin = Switch3_Pin|Switch1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// Task 1: Button Read (100ms)
void vTaskButtonRead(void *pvParameters) {
    uint8_t last_button1_state = 0;
    uint8_t last_button2_state = 0;

    while (1) {
        // Read the current state of the buttons
        uint8_t current_button1_state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15);  // Button 1
        uint8_t current_button2_state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4);   // Button 2

        // Debouncing logic for Button 1
        if (current_button1_state != last_button1_state) {
            vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_DELAY));  // Wait for debounce time
            current_button1_state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15);  // Read button again after delay
            if (current_button1_state != last_button1_state) {
                last_button1_state = current_button1_state;  // Update the last state

                // If button is pressed (assuming active-low logic)
                if (current_button1_state == GPIO_PIN_RESET) {
                    button1_state = 1;  // Update shared state variable
                    xSemaphoreGive(buttonSemaphore);  // Trigger LED control task
                } else {
                    button1_state = 0;  // Button released
                }
            }
        }

        // Debouncing logic for Button 2
        if (current_button2_state != last_button2_state) {
            vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_DELAY));  // Wait for debounce time
            current_button2_state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4);  // Read button again after delay
            if (current_button2_state != last_button2_state) {
                last_button2_state = current_button2_state;  // Update the last state

                // If button is pressed (assuming active-low logic)
                if (current_button2_state == GPIO_PIN_RESET) {
                    button2_state = 1;  // Update shared state variable
                    xSemaphoreGive(buttonSemaphore);  // Trigger LED control task
                } else {
                    button2_state = 0;  // Button released
                }
            }
        }

        // Small delay to avoid CPU hogging
        vTaskDelay(pdMS_TO_TICKS(100));  // Check buttons every 100ms
    }
}

// Task 2: LED Control
void vTaskLEDControl(void *pvParameters) {
    while (1) {
        // Wait for semaphore from button read task
        if (xSemaphoreTake(buttonSemaphore, portMAX_DELAY) == pdTRUE) {

            // If both buttons are pressed, maintain the LED state
            if (button1_state == 1 && button2_state == 1) {
                // Both buttons pressed - Do nothing to keep LEDs in the same state
                continue;
            }

            // If button 1 is pressed (Red LED ON) - keep it ON until released
            if (button1_state == 1) {
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);   // Red LED ON (PC6)

                // Wait until button 1 is released before turning off
                while (button1_state == 1) {
                    vTaskDelay(pdMS_TO_TICKS(50)); // Poll every 50ms
                }
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET); // Red LED OFF when released
            }

            // If button 2 is pressed (Green LED ON) - keep it ON until released
            if (button2_state == 1) {
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);  // Green LED ON (PB13)

                // Wait until button 2 is released before turning off
                while (button2_state == 1) {
                    vTaskDelay(pdMS_TO_TICKS(50)); // Poll every 50ms
                }
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET); // Green LED OFF when released
            }
        }

        // If no buttons are pressed, start LED toggling
        if (button1_state == 0 && button2_state == 0) {
            // Toggle Red LED
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);   // Toggle Red LED (PC6)
            vTaskDelay(pdMS_TO_TICKS(500));          // 500ms delay for Red LED

            // Check for button press before toggling Green LED
            if (button1_state == 1 || button2_state == 1) {
                continue;  // Exit toggling if any button is pressed
            }

            // Toggle Green LED
            HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13);  // Toggle Green LED (PB13)
            vTaskDelay(pdMS_TO_TICKS(500));          // 500ms delay for Green LED

            // Check for button press again
            if (button1_state == 1 || button2_state == 1) {
                continue;  // Exit toggling if any button is pressed
            }
        }
    }
}// Task 3: LCD Update


void vTaskLCDUpdate(void *pvParameters) {
    // Previous states to track changes
    uint8_t prev_button1_state = 0;
    uint8_t prev_button2_state = 0;
    uint8_t prev_led_red_state = 0;
    uint8_t prev_led_green_state = 0;

    while (1) {
        // Check if states have changed
        if (button1_state != prev_button1_state || button2_state != prev_button2_state ||
            led_red_state != prev_led_red_state || led_green_state != prev_led_green_state) {

            // Acquire mutex before updating LCD
            if (xSemaphoreTake(lcdMutex, portMAX_DELAY) == pdTRUE) {
                lcd_clear();  // Clear only if something changed
                lcd_set_cursor(0, 0);
                lcd_send_string("BT 1: ");
                lcd_send_string(button1_state ? "Pressed" : "Released");

                lcd_set_cursor(1, 0);
                lcd_send_string("BT 2: ");
                lcd_send_string(button2_state ? "Pressed" : "Released");

                xSemaphoreGive(lcdMutex);  // Release mutex
            }

            // Update previous states
            prev_button1_state = button1_state;
            prev_button2_state = button2_state;
            prev_led_red_state = led_red_state;
            prev_led_green_state = led_green_state;
        }

        // Add delay to avoid rapid updates
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
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

#ifdef  USE_FULL_ASSERT
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
