/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  * @author         : Divesh Dutt
  * Email           : diveshdutt2@gmail.com
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "FreeRTOS.h"
#include "task.h"
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
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
uint8_t rx_byte;
char rx_buffer[128];
int rx_index = 0;
char parse_buffer[128];
volatile uint8_t line_ready = 0;
char DefBuffer[30];
char hrtBuffer[30];
float latitude = 0.0;
float longitude = 0.0;
int gps_fix = 0;
TaskHandle_t GpsTaskHandle = NULL;
TaskHandle_t HeartbeatTaskHandle = NULL;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_UART5_Init(void);
void StartDefaultTask(void *argument);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

/* USER CODE BEGIN PFP */
char* extract_gps_column(char *str, int field_index);
void parse_gps_line(char *line);
void StartGpsTask(void *argument);
void StartHeartbeatTask(void *argument);

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
	 HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
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
  MX_UART5_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart5, &rx_byte, 1);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

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
  /* creation of defaultTask created by cubeMX*/
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  xTaskCreate(
        StartGpsTask,     // 1. Name of the function loop code block
        "GpsTask",        // 2. text label name for debugging diagnostics
        256,              // 3. Need Large stack since it do complex task 256 words = 1024 bytes
        NULL,             // 4. Parameter object hook pointer passed down to task
        25,                //5. Execution Priority Rank (Higher = More Urgent) since default task has priority 24
        &GpsTaskHandle    // 6. task handler
    );

  xTaskCreate(
       StartHeartbeatTask,     // 1.Name of the function loop code block
       "HeartTask",            // 2.Diagnostic text tag used for trace tools
       128,                    // 3.Stack depth size (128 words = 512 bytes is enough for basic GPIOs)
       NULL,                   // 4.Parameter pointer argument hook
       24,                     // 5.Execution Priority Rank (Higher = More Urgent) since default task has priority 24
       &HeartbeatTaskHandle    // 6.task handler
   );
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */


  /* Start scheduler */
  osKernelStart();

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_UART5;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Uart5ClockSelection = RCC_UART5CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 9600;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

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
  huart2.Init.BaudRate = 9600;
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
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE BEGIN MX_GPIO_Init_2 */
   GPIO_InitTypeDef GPIO_InitStruct = {0};

   /* 1. Configure the LED Pin (PA5) to start in a LOW state */
   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

   /* 2. Set up the GPIO parameters for PA5 */
   GPIO_InitStruct.Pin = GPIO_PIN_5;
   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;       // Push-Pull Output Mode
   GPIO_InitStruct.Pull = GPIO_NOPULL;               // No internal pull-up/pull-down resistors
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;      // Low speed is perfect for a basic blinking LED

   /* 3. Apply the settings directly to the hardware register */
   HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
   /* USER CODE END MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART5) // Change to your exact GPS UART instance
    {
        if (rx_byte == '\n' || rx_byte == '\r')   // check for the end of sequence
        {
            if (rx_index > 0)
            {
                rx_buffer[rx_index] = '\0';
                strncpy(parse_buffer, rx_buffer,sizeof(parse_buffer)-1); // Copy from rx_buffer to parsing buffer (parse_buffer) which can be parsed later
                parse_buffer[sizeof(parse_buffer)-1] = '\0';

                //signal the GPS task that a line is ready
               BaseType_t xHigherPriorityTaskWoken = pdFALSE;
                vTaskNotifyGiveFromISR(GpsTaskHandle, &xHigherPriorityTaskWoken);
               // portYIELD_FROM_ISR(pdTRUE);  //it will do the context switch immediately to StartGpsTask
                portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            }
            rx_index = 0;   // rx_index = 0 so that next sequence of data can be received
        }
        else if (rx_index < sizeof(rx_buffer) - 1)  //rx_buffer should not be more that 128 bytes
        {
            rx_buffer[rx_index++] = rx_byte;
        }
        HAL_UART_Receive_IT(&huart5, &rx_byte, 1);  //receieve the data again in interrupt mode
    }
}

void StartHeartbeatTask(void *argument)
{
    for(;;)
    {

        // Toggle the on-board LED on Port A, Pin 5
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
        sprintf(hrtBuffer,"Led Blinks on PA5...\r\n");
       	HAL_UART_Transmit(&huart2,(uint8_t*) hrtBuffer, strlen(hrtBuffer), 100);

        // Block this thread for exactly 500 milliseconds (yielding CPU power back to the kernel)

        vTaskDelay(pdMS_TO_TICKS(500));

    }
}

void StartGpsTask(void *argument)
{
    char display_msg[128];

    /* Threads must run inside an infinite loop */
    for(;;)
    {
    	 // It wakes up the exact microsecond the ISR finishes copying a new line.
    	ulTaskNotifyTake(pdTRUE, portMAX_DELAY);


    	parse_gps_line(parse_buffer);
        // Print a diagnostic message to prove the scheduler is alive
        sprintf(display_msg, "GPS is active...\r\n");
        HAL_UART_Transmit(&huart2, (uint8_t*)display_msg, strlen(display_msg), 100);

        	   if (gps_fix > 0)
               {
                   sprintf(display_msg, "STATUS: Locked! | LAT: %.6f | LON: %.6f\r\n", latitude, longitude);
                   HAL_UART_Transmit(&huart2, (uint8_t*)display_msg, strlen(display_msg), 100);
               }
               else
               {
                   // Only print on the GGA line to keep the screen cleanly moving once a second
                   if (strncmp(parse_buffer, "$GNGGA", 6) == 0 || strncmp(parse_buffer, "$GPGGA", 6) == 0)
                   {
                       sprintf(display_msg, "STATUS: Searching for Satellites...\r\n");
                       HAL_UART_Transmit(&huart2, (uint8_t*)display_msg, strlen(display_msg), 100);
                   }
               }

               // 4. Clear the flag so the interrupt knows the snapshot was processed
        line_ready = 0;
        vTaskDelay(pdMS_TO_TICKS(500));

    }
}

/**
 * @brief  Extracts, validates, and parses raw NMEA GGA sentences from the GPS module.
 * @note   Handles both $GNGGA and $GPGGA sentences dynamically. Converts raw coordinates
 *         from NMEA Degree-Minutes (DDMM.MMMMM) format to standard Decimal Degrees (DD.DDDDDD).
 *
 * @param  line: Pointer to the null-terminated string containing a single raw NMEA sentence.
 * @retval None
 */
void parse_gps_line(char *line)
{
    if (strncmp(line, "$GNGGA", 6) == 0 || strncmp(line, "$GPGGA", 6) == 0)
    {
        char *fix_ptr = extract_gps_column(line, 6);
        if (fix_ptr && strlen(fix_ptr) > 0)
        {
            gps_fix = atoi(fix_ptr);
        }
        else
        {
            gps_fix = 0;
        }

        // Only parse coordinate positions if the GPS module sees data fields
        char *lat_ptr = extract_gps_column(line, 2);

        if (lat_ptr && strlen(lat_ptr) > 0)
        {
            // Convert ASCII to float while the buffer safely holds latitude
            float raw_lat = atof(lat_ptr);

            // LATITUDE CONVERSION (NMEA DDMM.MMMMM -> Decimal DD.DDDDDD)
            int lat_degrees = (int)(raw_lat / 100);
            float lat_minutes = raw_lat - (lat_degrees * 100);
            latitude = lat_degrees + (lat_minutes / 60.0);

            // Apply Southern Hemisphere correction
            if (*extract_gps_column(line, 3) == 'S')
            {
                latitude = -latitude;
            }
        }

        // 2. EXTRACT AND PROCESS LONGITUDE SECOND
        char *lon_ptr = extract_gps_column(line, 4);

        if (lon_ptr && strlen(lon_ptr) > 0)
        {
            // Convert ASCII to float now that the buffer holds longitude
            float raw_lon = atof(lon_ptr);

            // LONGITUDE CONVERSION (NMEA DDDMM.MMMMM -> Decimal DD.DDDDDD)
            int lon_degrees = (int)(raw_lon / 100);
            float lon_minutes = raw_lon - (lon_degrees * 100);
            longitude = lon_degrees + (lon_minutes / 60.0);

            // Apply Western Hemisphere correction
            if (*extract_gps_column(line, 5) == 'W')
            {
                longitude = -longitude;
            }
        }

        // Set status to valid display mode since parsing completed successfully
        gps_fix = 1;
        }
        else
        {
            // Fields are empty, meaning no lock yet
            gps_fix = 0;
        }
    }


/**
 * @brief  Extracts a specific comma-separated data field from a raw NMEA sentence.
 * @note   This function counts commas to locate the target column.
 *         Trailing control characters (\r, \n) or asterisk delimiters (*) will
 *         automatically terminate the parsing field.
 *
 * @param  line:   Pointer to the null-terminated raw NMEA string (e.g., $GNGGA,...).
 * @param  target_column: The index of the column to extract (0-indexed or 1-indexed,
 *  (from Datasheet of GPS Module) depending on implementation. Here, 2=Lat, 4=Lon, 6=quality).
 *
 * @retval char*:  Pointer to a static null-terminated string containing the column data.
 *                 Returns NULL if the target column index is not found in the line.
 */

char* extract_gps_column(char *str, int field_index)
{
	// Local static buffer to store and return the text field
    static char field_buffer[32];
    int comma_count = 0;
    int buf_idx = 0;
    // Reset buffer to prevent garbage data to corrupt
    memset(field_buffer, 0, sizeof(field_buffer));

    // Traverse the string until we find the start of the target column
    while (*str && comma_count < field_index)
    {
        if (*str == ',')
        {
            comma_count++;
        }
        str++;
        // Return NULL if the string ended before reaching the desired column
        if (*str == '\0')
         {
             return NULL;
         }
    }

    //Copy characters from the column until hitting a delimiter or boundary
    while (*str && *str != ',' && *str != '*' && *str != '\r' && *str != '\n')
    {
        if (buf_idx < 31)
        {
            field_buffer[buf_idx++] = *str;
        }
        str++;
    }
    //Explicitly null-terminate the extracted string
    field_buffer[buf_idx] = '\0';
    return field_buffer;
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */

  /* Infinite loop */
  for(;;)
  {
	 sprintf(DefBuffer,"Default Task is running...\r\n");
	 HAL_UART_Transmit(&huart2,(uint8_t*) DefBuffer, strlen(DefBuffer), 100);
     osDelay(500);
  }
  /* USER CODE END 5 */
}

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
  if (htim->Instance == TIM6)
  {
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
