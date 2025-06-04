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
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "AccelData.h"
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MPU6050_ADDR_7BIT   0x68
#define MPU6050_ADDR_8BIT  (MPU6050_ADDR_7BIT << 1)
#define ACCEL_XOUT_H       0x3B
#define PWR_MGMT_1         0x6B
#define WHO_AM_I           0x75
#define ACCEL_SCALE        16384.0f
#define FLASH_STUB_ADDR       0x08040000U
#define FLASH_SECTOR_NUMBER   FLASH_SECTOR_6
#define FLASH_VOLTAGE_RANGE   FLASH_VOLTAGE_RANGE_3
#define SAMPLE_BUF_SIZE 10
#define FLASH_LOG_START     0x08040000U

#define FLASH_LOG_SECTOR    FLASH_SECTOR_6
#define FLASH_LOG_BANK      FLASH_BANK_1
#define FLASH_LOG_RANGE     FLASH_VOLTAGE_RANGE_3
#define FLASH_SECTOR_SIZE  (128U * 1024U)  /* 128 KB sector size */

#define SAMPLE_RATE_HZ      100
#define SAMPLE_PERIOD_MS   (1000 / SAMPLE_RATE_HZ)

#define LAUNCH_DEBOUNCE_SAMPLES 2
#define ACCEL_SCALE       16384.0f
#define BYTES_PER_SAMPLE     6U
#define LAUNCH_THRESHOLD_G  5.5f
#define MAX_FLIGHT_TIME_MS 150000U //20000U = 20 seconds. 125000U = 125s.

#define PACKET_PERIOD_MS        500      // send 'X' every 500 ms
#define BLINK_PERIOD_MS         500      // blink LED at 0.5 Hz

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart4;   // make sure this matches your MX_UART4_Init() handle
static uint32_t lastPacket_ms = 0;
static uint32_t write_addr;
static bool in_flight  = false;
static AccelRaw    sample_buf[SAMPLE_BUF_SIZE];
static uint32_t    sample_count = 0;
AccelRaw offset;
static int landed_count = 0;
#define LAND_DEBOUNCE_S   2       // require 2 s of “rest ?
#define LAND_DEBOUNCE_SAMPLES  (LAND_DEBOUNCE_S * SAMPLE_RATE_HZ)
static uint32_t flight_start_ms;
static int  launch_count = 0;
static uint32_t led_blink_ms = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
int  __io_putchar(int ch);
void I2C_Scan(void);
void MPU6050_Init(void);
void MPU6050_ReadWhoAmI(void);
void MPU6050_ReadAccelRaw(AccelRaw *r);
void MPU6050_CalibrateRaw(AccelRaw *offset);
void Flash_WriteData(uint32_t address, AccelRaw *data, size_t count);
void Flash_ReadData(uint32_t address, AccelRaw *data, size_t count);
void start_logging(void);
void log_sample(AccelRaw *r);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void start_logging(void) {

    write_addr  = FLASH_LOG_START;
    flight_start_ms = HAL_GetTick();
    in_flight = true;
    printf("=== Logging STARTED @%08lX ===\r\n", write_addr);
}

void log_sample(AccelRaw *r) {
    // write x,y,z as half‑words
    HAL_FLASH_Unlock();
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, write_addr + 0, (uint64_t)(uint16_t)r->x);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, write_addr + 2, (uint64_t)(uint16_t)r->y);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, write_addr + 4, (uint64_t)(uint16_t)r->z);
    HAL_FLASH_Lock();
    write_addr += BYTES_PER_SAMPLE;
}

// Retarget printf to UART
int __io_putchar(int ch) {
    HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

void I2C_Scan(void) {
    printf("I2C scan...\r\n");
    for (uint8_t addr = 1; addr < 128; addr++) {
        if (HAL_I2C_IsDeviceReady(&hi2c1, addr << 1, 2, 10) == HAL_OK) {
            printf(" Found device at 0x%02X\r\n", addr);
        }
    }
}

void Flash_WriteData(uint32_t address, AccelRaw *data, size_t count)
{
    // 1) Unlock and erase (with Banks field)
    HAL_FLASH_Unlock();
    FLASH_EraseInitTypeDef eraseInit = {0};
    uint32_t sectorError = 0;

    eraseInit.TypeErase    = FLASH_TYPEERASE_SECTORS;
    eraseInit.Banks        = FLASH_BANK_1;             // make sure we select the correct bank
    eraseInit.Sector       = FLASH_SECTOR_NUMBER;
    eraseInit.NbSectors    = 1;
    eraseInit.VoltageRange = FLASH_VOLTAGE_RANGE;

    if (HAL_FLASHEx_Erase(&eraseInit, &sectorError) != HAL_OK) {
        printf("Flash erase failed (err=0x%08lX)\r\n", HAL_FLASH_GetError());
        HAL_FLASH_Lock();
        return;
    }

    // 2) Program each sample as three 16‑bit half‑words
    uint32_t addr = address;
    for (size_t i = 0; i < count; i++) {
        uint16_t vals[3] = {
            (uint16_t)data[i].x,
            (uint16_t)data[i].y,
            (uint16_t)data[i].z
        };
        for (int k = 0; k < 3; k++) {
            if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,
                                  addr,
                                  (uint64_t)vals[k]) != HAL_OK) {
                printf("Flash write failed at 0x%08lX (err=0x%08lX)\r\n",
                       addr, HAL_FLASH_GetError());
                HAL_FLASH_Lock();
                return;
            }
            addr += 2;
        }
    }

    // 3) Lock and report success
    HAL_FLASH_Lock();
    printf("Data written to FLASH @0x%08lX (%u samples)\r\n",
           address, (unsigned)count);
}

void Flash_ReadData(uint32_t address, AccelRaw *data, size_t count) {
    memcpy(data, (AccelRaw*)address, count * sizeof(AccelRaw));
}


// Wake up & configure sensor
void MPU6050_Init(void) {
    uint8_t zero = 0;
    // Try 8‑bit first; if it NACKs, switch to 7‑bit in the macro
    if (HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR_8BIT,
                          PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT,
                          &zero, 1, HAL_MAX_DELAY) != HAL_OK) {
        printf("Init failed (PWR_MGMT_1)\r\n"); return;
    }
    HAL_Delay(100);
    if (HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR_8BIT,
                          0x1C, I2C_MEMADD_SIZE_8BIT,
                          &zero, 1, HAL_MAX_DELAY) != HAL_OK) {
        printf("Init failed (ACCEL_CONFIG)\r\n"); return;
    }
    printf("MPU6050 init ok\r\n");
}

void MPU6050_ReadWhoAmI(void) {
    uint8_t who = 0;
    if (HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR_8BIT,
                         WHO_AM_I, I2C_MEMADD_SIZE_8BIT,
                         &who, 1, HAL_MAX_DELAY) == HAL_OK) {
        printf("WHO_AM_I = 0x%02X\r\n", who);
    } else {
        printf("WhoAmI error\r\n");
    }
}

void MPU6050_ReadAccelRaw(AccelRaw *r) {
    uint8_t buf[6];
    if (HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR_8BIT,
                         ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT,
                         buf, 6, HAL_MAX_DELAY) != HAL_OK) {
        printf("Raw read error\r\n");
        return;
    }
    r->x = (int16_t)(buf[0]<<8 | buf[1]);
    r->y = (int16_t)(buf[2]<<8 | buf[3]);
    r->z = (int16_t)(buf[4]<<8 | buf[5]);
}

void MPU6050_ReadAccelScaled(AccelScaled *s) {
    AccelRaw r;
    MPU6050_ReadAccelRaw(&r);
    s->x = r.x / ACCEL_SCALE;
    s->y = r.y / ACCEL_SCALE;
    s->z = r.z / ACCEL_SCALE;
}

void MPU6050_CalibrateRaw(AccelRaw *off) {
    AccelRaw r; int64_t sx=0, sy=0, sz=0;
    for (int i=0; i<1000; i++) {
        MPU6050_ReadAccelRaw(&r);
        sx += r.x; sy += r.y; sz += r.z;
        HAL_Delay(2);
    }
    off->x = sx/1000;
    off->y = sy/1000;
    off->z = sz/1000 - (int16_t)ACCEL_SCALE;  // remove 1g
    printf("Offsets: %d, %d, %d\r\n", off->x, off->y, off->z);
}

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
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */
  I2C_Scan();
  MPU6050_Init();
  MPU6050_ReadWhoAmI();

  // 1) Calibrate & pre erase now, while you’re assembling
    AccelRaw offset;
    MPU6050_CalibrateRaw(&offset);

    // Erase the logging sector now (not at launch)
    {
      HAL_FLASH_Unlock();
      FLASH_EraseInitTypeDef e = {0};
      uint32_t err;
      e.TypeErase    = FLASH_TYPEERASE_SECTORS;
      e.Banks        = FLASH_LOG_BANK;
      e.Sector       = FLASH_LOG_SECTOR;
      e.NbSectors    = 1;
      e.VoltageRange = FLASH_LOG_RANGE;
      HAL_FLASHEx_Erase(&e, &err);
      HAL_FLASH_Lock();
    }

  // 2) Idle / wait for launch
    uint32_t lastPacket_ms = HAL_GetTick();
    uint32_t flight_start_ms = 0;
    bool     in_flight      = false;


    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
    printf("Waiting for launch… threshold=%.1f g\r\n", LAUNCH_THRESHOLD_G);


  while (!in_flight) {
      AccelRaw r;
      MPU6050_ReadAccelRaw(&r);
      float ax = (r.x - offset.x)/ACCEL_SCALE;
      float ay = (r.y - offset.y)/ACCEL_SCALE;
      float az = (r.z - offset.z)/ACCEL_SCALE;
      float mag = sqrtf(ax*ax + ay*ay + az*az);

      if ((HAL_GetTick() - lastPacket_ms) >= PACKET_PERIOD_MS) {
            lastPacket_ms += PACKET_PERIOD_MS;
            uint8_t ch = 'X';
            HAL_UART_Transmit(&huart4, &ch, 1, HAL_MAX_DELAY);
          }

      if (mag > LAUNCH_THRESHOLD_G) {
          if (++launch_count >= LAUNCH_DEBOUNCE_SAMPLES) {
              // entered flight
              in_flight      = true;
              flight_start_ms = HAL_GetTick();
              launch_count    = 0;
              HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
              printf("=== Logging STARTED ===\r\n");
          }
      } else {
          launch_count = 0;
      }
  }

  lastPacket_ms = HAL_GetTick();
  led_blink_ms      = HAL_GetTick();

  // 3) Flight logging loop
  while (in_flight) {
      AccelRaw r;
      MPU6050_ReadAccelRaw(&r);

      // Convert & print in g‑units
      float ax = (r.x - offset.x)/ACCEL_SCALE;
      float ay = (r.y - offset.y)/ACCEL_SCALE;
      float az = (r.z - offset.z)/ACCEL_SCALE;
      printf("Ax %+5.2f g  Ay %+5.2f g  Az %+5.2f g\r\n",
             ax, ay, az);

      // Flash log the sample
      if (write_addr + BYTES_PER_SAMPLE <= FLASH_LOG_START + FLASH_SECTOR_SIZE) {
          log_sample(&r);
      }

      // LED blink
      if ((HAL_GetTick() - led_blink_ms) >= BLINK_PERIOD_MS) {
        led_blink_ms = HAL_GetTick();
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
      }

      if ((HAL_GetTick() - lastPacket_ms) >= PACKET_PERIOD_MS) {
            lastPacket_ms += PACKET_PERIOD_MS;
            uint8_t ch = 'X';
            HAL_UART_Transmit(&huart4, &ch, 1, HAL_MAX_DELAY);
          }

      // timeout check…
      if ((HAL_GetTick() - flight_start_ms) >= MAX_FLIGHT_TIME_MS) {
          in_flight = false;
          // one‑shot LED
          HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
          break;
      }

      HAL_Delay(SAMPLE_PERIOD_MS);
  }

  // 4) Post flight: you can DumpFlash() or just sit here
  printf("Flight complete. Data @ 0x%08X → 0x%08X\r\n",
         FLASH_LOG_START, write_addr);

  uint32_t count = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

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

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
