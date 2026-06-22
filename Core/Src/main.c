/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "dht11.h"
#include "oled.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* 宏定义 — 系统状态 / 告警阈值 / 定时器参数 */
/* USER CODE BEGIN PD */

/* 系统状态, 后面用 switch 判断 */
#define STATE_IDLE     0   /* 还没按 KEY1, 等启动 */
#define STATE_SAFE     1   /* 一切正常 */
#define STATE_ERROR1   2   /* 温度超了 */
#define STATE_ERROR2   3   /* 电压不对 */

/* 阈值 */
#define TEMP_THRESHOLD    28.0f   /* 温度阈值: 28°C */
#define VOLT_THRESHOLD    2.0f    /* 电压阈值: 2.0V */

/*ADC 通道索引 (与 CubeMX 配置的 Rank 顺序一致)*/
#define ADC_IDX_VREF      0   /* 内部基准电压, 用于反算 VDDA */
#define ADC_IDX_CH3       1   /* PA3: 电位器1 */
#define ADC_IDX_CH4       2   /* PA4: 电位器2 */
#define ADC_CHANNELS      3

/* 按键 */
#define LONG_PRESS_MS     1500    /* 长按判定时间 */

/*  蜂鸣器 PWM (TIM1 CH4, 定时器时钟 1MHz)  */
#define BUZZER_1KHZ_ARR   999     /* 1kHz: 1M / (999+1) */
#define BUZZER_2KHZ_ARR   499     /* 2kHz: 1M / (499+1) */

/*  RGB LED PWM (TIM3 CH2/3/4, ARR=99)  */
#define RGB_MAX           99      /* 满亮度占空比 */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/*  全局变量 */
/* USER CODE BEGIN PV */

/*  ADC + DMA  */
volatile uint16_t adcBuf[ADC_CHANNELS] = {0};  /* DMA 循环缓冲区 */

/* DHT11 数据 */
DHT11_Data dht11Data;
float temperature = 0.0f;          /* 当前温度 (°C) */
float vref_voltage  = 0.0f;        /* 由 VREFINT 反算的实际 VDDA */
float voltage_ch3   = 0.0f;        /* PA3 电位器电压 */
float voltage_ch4   = 0.0f;        /* PA4 电位器电压 */

/*  系统状态  */
uint8_t sysState = STATE_IDLE;     /* 当前状态 */
uint8_t sysRunning = 0;            /* 0=待启动, 1=采集中 */

/*  告警闪烁 / 蜂鸣计时  */
uint32_t lastBlinkTick = 0;
uint32_t lastBuzzerTick = 0;
uint8_t  blinkState = 0;           /* 0=LED灭, 1=LED亮 */
uint8_t  buzzerState = 0;          /* 0=静音, 1=鸣叫 */

/*  按键  */
uint8_t  key1Pressed = 0;          /* KEY1 去抖标志 */
uint32_t key2PressStart = 0;       /* KEY2 按下时刻 */
uint8_t  alarmLatched = 0;         /* FSAE 锁存: 1=告警已锁存，需手动复位＋故障排除 */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void  ADC_StartDMA(void);
float ADC_CalcVoltage(uint16_t adcVal);
void  Buzzer_SetFreq(uint16_t arr, uint16_t ccr);
void  Buzzer_Off(void);
void  RGB_SetColor(uint16_t r, uint16_t g, uint16_t b);
void  RGB_Off(void);
void  Key_Scan(void);
void  Alarm_Handler(void);
void  OLED_UpdateDisplay(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
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

  /* 强制复位 I2C1 外设, 防止总线挂死后复位也无法恢复 */
  __HAL_RCC_I2C1_CLK_ENABLE();
  __HAL_RCC_I2C1_FORCE_RESET();
  __HAL_RCC_I2C1_RELEASE_RESET();

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  /* 初始化 DWT */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  /*  初始化 OLED 并显示欢迎界面  */
  OLED_Init();
  OLED_ShowString(0, 0, "System Ready", 8);
  OLED_ShowString(0, 2, "Press KEY1 Start", 8);
  HAL_Delay(1000);

  /*  启动 ADC + DMA 连续采集  */
  ADC_StartDMA();

  /*  初始状态: 关闭所有输出  */
  RGB_Off();
  Buzzer_Off();

  printf("\r\n========== FSAE Signal Monitor ==========\r\n");
  printf("H/W: STM32F103 | HSE 72MHz | DHT11 + ADCx2\r\n");
  printf("Thresholds: Temp>28C | Volt<2.0V\r\n");
  printf("Press KEY1 to start acquisition\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /*  按键扫描  */
    Key_Scan();

    if (sysRunning)
    {
        /* 读取 DHT11  */
        if (DHT11_Read(&dht11Data) == 0)
            temperature = (float)dht11Data.temperature;

        /* 计算各路电压 */
        /* 先根据 VREFINT 校准计算 VDDA */
        uint16_t vref_adc = adcBuf[ADC_IDX_VREF];
        vref_voltage = 1.20f * 4096.0f / (float)vref_adc;

        /* 各通道电压 = ADC值 * VDDA / 4096 */
        voltage_ch3 = ADC_CalcVoltage(adcBuf[ADC_IDX_CH3]);
        voltage_ch4 = ADC_CalcVoltage(adcBuf[ADC_IDX_CH4]);

        /* ── 告警判断 (FSAE 锁存逻辑) ── */

        /* 未锁存时才检测新告警 */
        if (!alarmLatched)
        {
            if (temperature > TEMP_THRESHOLD)
            {
                sysState = STATE_ERROR1;
                alarmLatched = 1;   /* 锁存! 即使温度恢复正常也不自动清除 */
            }
            else if (voltage_ch3 < VOLT_THRESHOLD || voltage_ch4 < VOLT_THRESHOLD)
            {
                sysState = STATE_ERROR2;
                alarmLatched = 1;   /* 锁存! */
            }
            else
            {
                sysState = STATE_SAFE;
            }
        }
        /* 已锁存 → 保持告警状态, 不管当前读数是否恢复 */

        /*  告警处理 (LED + 蜂鸣器)  */
        Alarm_Handler();

        /*  OLED 显示更新  */
        OLED_UpdateDisplay();

        /*  串口打印  */
        {
            const char *stateStr = "IDLE";
            if (sysState == STATE_SAFE)   stateStr = "SAFE";
            if (sysState == STATE_ERROR1) stateStr = "ERROR1";
            if (sysState == STATE_ERROR2) stateStr = "ERROR2";
            printf("Temp:%dC  V3:%d.%02dV  V4:%d.%02dV  State:%s\r\n",
                   (int)temperature,
                   (int)voltage_ch3, (int)(voltage_ch3 * 100) % 100,
                   (int)voltage_ch4, (int)(voltage_ch4 * 100) % 100,
                   stateStr);
        }
    }
    else
    {
        /* 未启动时显示提示 */
        OLED_Clear();
        OLED_ShowString(0, 2, "Press KEY1", 8);
        OLED_ShowString(0, 3, "to Start", 8);
    }

    HAL_Delay(200);  /* 主循环周期 ~200ms */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* ================================================================
 * 启动 ADC + DMA 连续循环采集 (ContinuousConvMode=ENABLE)
 * ================================================================ */
void ADC_StartDMA(void)
{
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adcBuf, ADC_CHANNELS);
}

/* ================================================================
 * 根据 ADC 值计算电压 (基于 VREFINT 校准)
 * ================================================================ */
float ADC_CalcVoltage(uint16_t adcVal)
{
    return (float)adcVal * vref_voltage / 4096.0f;
}

/* ================================================================
 * 蜂鸣器: 设置频率 (TIM1 CH1, PA8)
 *   arr: 自动重装载值
 *   ccr: 比较值 (占空比 50%: arr/2)
 * ================================================================ */
void Buzzer_SetFreq(uint16_t arr, uint16_t ccr)
{
    __HAL_TIM_SET_AUTORELOAD(&htim1, arr);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
}

void Buzzer_Off(void)
{
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
}

/* ================================================================
 * RGB LED 控制 (TIM3 CH2=蓝PA7, CH3=绿PB0, CH4=红PB1)
 *   ARR=99 → 占空比范围 0~99
 * ================================================================ */
void RGB_SetColor(uint16_t r, uint16_t g, uint16_t b)
{
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, r);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, g);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, b);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
}

void RGB_Off(void)
{
    HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
}

/* ================================================================
 * 按键扫描
 *   KEY1 (PA1): 短按 → 启动采集
 *   KEY2 (PA2): 长按 > LONG_PRESS_MS → 复位告警锁存
 *     仅当故障已排除 (温度/电压均正常) 时才允许清除 (FSAE 安全回路规范)
 * ═══════════════════════════════════════════════════════════════════════════ */
void Key_Scan(void)
{
    /* ── KEY1: 按下启动采集 ── */
    if (HAL_GPIO_ReadPin(key_1_GPIO_Port, key_1_Pin) == GPIO_PIN_RESET)
    {
        HAL_Delay(20); /* 消抖 */
        if (HAL_GPIO_ReadPin(key_1_GPIO_Port, key_1_Pin) == GPIO_PIN_RESET)
        {
            if (!key1Pressed)
            {
                key1Pressed = 1;
                sysRunning = 1;
                sysState = STATE_SAFE;
                OLED_Clear();
                printf("KEY1 Pressed - Acquisition Started\r\n");
            }
        }
    }
    else { key1Pressed = 0; }

    /* ── KEY2: 长按复位锁存 ── */
    if (HAL_GPIO_ReadPin(key_2_GPIO_Port, key_2_Pin) == GPIO_PIN_RESET)
    {
        if (key2PressStart == 0)
        {
            key2PressStart = HAL_GetTick();
        }
        else if ((HAL_GetTick() - key2PressStart) > LONG_PRESS_MS)
        {
            if (temperature <= TEMP_THRESHOLD
                && voltage_ch3 >= VOLT_THRESHOLD
                && voltage_ch4 >= VOLT_THRESHOLD)
            {
                alarmLatched = 0;
                sysState = STATE_SAFE;
                RGB_Off();
                Buzzer_Off();
                OLED_Clear();
                printf("KEY2 Long Press - Alarm Reset OK\r\n");
            }
            else
            {
                printf("KEY2 Long Press - FAIL: Fault still present!\r\n");
            }
            key2PressStart = 0;
        }
    }
    else { key2PressStart = 0; }
}

/* ================================================================
 * 告警处理: LED 闪烁 + 蜂鸣器
 *
 * SAFE:    绿灯常亮,            蜂鸣器静音
 * ERROR1:  红灯 1Hz 闪烁,       蜂鸣器 1kHz 间歇 (0.5s 周期)
 * ERROR2:  蓝灯 2Hz 闪烁,       蜂鸣器 2kHz 间歇 (0.2s 周期)
 * ═══════════════════════════════════════════════════════════════════════════ */
void Alarm_Handler(void)
{
    uint32_t now = HAL_GetTick();

    switch (sysState)
    {
    case STATE_SAFE:
        RGB_SetColor(0, RGB_MAX, 0);
        Buzzer_Off();
        break;

    case STATE_ERROR1:
    {
        uint32_t half = 500;
        if ((now - lastBlinkTick) >= half) { lastBlinkTick = now; blinkState = !blinkState; }
        blinkState ? RGB_SetColor(RGB_MAX, 0, 0) : RGB_Off();

        if ((now - lastBuzzerTick) >= half) { lastBuzzerTick = now; buzzerState = !buzzerState; }
        buzzerState ? Buzzer_SetFreq(BUZZER_1KHZ_ARR, BUZZER_1KHZ_ARR / 2) : Buzzer_Off();
        break;
    }

    case STATE_ERROR2:
    {
        uint32_t half = 250, buzHalf = 200;
        if ((now - lastBlinkTick) >= half) { lastBlinkTick = now; blinkState = !blinkState; }
        blinkState ? RGB_SetColor(0, 0, RGB_MAX) : RGB_Off();

        if ((now - lastBuzzerTick) >= buzHalf) { lastBuzzerTick = now; buzzerState = !buzzerState; }
        buzzerState ? Buzzer_SetFreq(BUZZER_2KHZ_ARR, BUZZER_2KHZ_ARR / 2) : Buzzer_Off();
        break;
    }

    default: break;
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 * OLED_UpdateDisplay — 刷新 OLED 显示
 *
 * 布局 (128×64):
 *   Line 0: Temp: xx.x C
 *   Line 1: V_CH3: x.xxx V
 *   Line 2: V_CH4: x.xxx V
 *   Line 3: Tth: 28 C
 *   Line 4: Vth: 2.00 V
 *   Line 6-7: 状态 (16px 大字) — SAFE / ERROR1 / ERROR2
 * ═══════════════════════════════════════════════════════════════════════════ */
void OLED_UpdateDisplay(void)
{
    OLED_Clear();

    /* 传感器数据 */
    OLED_ShowString(0, 0, "Temp:", 8);
    OLED_ShowFloat(48, 0, temperature, 3, 1, 8);
    OLED_ShowChar(96, 0, 'C', 8);

    OLED_ShowString(0, 1, "V_CH3:", 8);
    OLED_ShowFloat(48, 1, voltage_ch3, 1, 3, 8);
    OLED_ShowChar(96, 1, 'V', 8);

    OLED_ShowString(0, 2, "V_CH4:", 8);
    OLED_ShowFloat(48, 2, voltage_ch4, 1, 3, 8);
    OLED_ShowChar(96, 2, 'V', 8);

    /* 阈值 */
    OLED_ShowString(0, 3, "Tth:", 8);
    OLED_ShowFloat(30, 3, TEMP_THRESHOLD, 2, 0, 8);
    OLED_ShowChar(54, 3, 'C', 8);

    OLED_ShowString(0, 4, "Vth:", 8);
    OLED_ShowFloat(30, 4, VOLT_THRESHOLD, 1, 2, 8);
    OLED_ShowChar(54, 4, 'V', 8);

    /* 状态 (锁存未清除时显示 LATCHED) */
    switch (sysState)
    {
    case STATE_SAFE:
        OLED_Printf(0, 6, 16, alarmLatched ? "LATCHED" : "SAFE");
        break;
    case STATE_ERROR1:
        OLED_Printf(0, 6, 16, "ERROR1");
        break;
    case STATE_ERROR2:
        OLED_Printf(0, 6, 16, "ERROR2");
        break;
    default: break;
    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* 进入错误处理时 PC13 LED 快速闪烁 (5Hz) */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  GPIO_InitTypeDef gpio = {0};
  gpio.Pin = GPIO_PIN_13;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &gpio);
  __disable_irq();
  while (1)
  {
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    HAL_Delay(100);
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
