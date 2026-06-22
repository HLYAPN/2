#include "dht11.h"

/* DHT11 GPIO: PA0, 开漏输出 + 上拉 */
#define DHT11_PORT      GPIOA
#define DHT11_PIN_NUM   GPIO_PIN_0

/* 微秒级延时 (DWT 精确延时, 72MHz) */
static void DHT11_DelayUs(uint16_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000);
    while ((DWT->CYCCNT - start) < ticks);
}

/* 设置 DHT11 引脚为输出 */
static void DHT11_SetOutput(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = DHT11_PIN_NUM;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(DHT11_PORT, &GPIO_InitStruct);
}

/* 设置 DHT11 引脚为输入 */
static void DHT11_SetInput(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = DHT11_PIN_NUM;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(DHT11_PORT, &GPIO_InitStruct);
}

/* 读取一个 bit */
static uint8_t DHT11_ReadBit(void)
{
    uint32_t timeout = 0;
    /* 等待低电平结束 */
    while (HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN_NUM) == GPIO_PIN_RESET)
    {
        if (++timeout > 2000) return 0xFF;
    }
    /* 延时 40us 后判断电平 */
    DHT11_DelayUs(40);
    if (HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN_NUM) == GPIO_PIN_SET)
    {
        /* 等待高电平结束 */
        timeout = 0;
        while (HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN_NUM) == GPIO_PIN_SET)
        {
            if (++timeout > 2000) return 0xFF;
        }
        return 1;
    }
    return 0;
}

/* 读取一个字节 */
static uint8_t DHT11_ReadByte(void)
{
    uint8_t value = 0;
    for (int i = 0; i < 8; i++)
    {
        value <<= 1;
        uint8_t bit = DHT11_ReadBit();
        if (bit == 0xFF) return 0xFF; /* 超时 */
        if (bit) value |= 0x01;
    }
    return value;
}

/**
  * @brief  读取 DHT11 温湿度数据
  * @param  data: 数据指针
  * @retval 0: 成功, 1: 失败/超时
  */
uint8_t DHT11_Read(DHT11_Data *data)
{
    uint8_t buf[5] = {0};
    uint32_t timeout = 0;

    /* 步骤1: MCU 发送起始信号 (拉低 > 18ms) */
    DHT11_SetOutput();
    HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN_NUM, GPIO_PIN_RESET);
    HAL_Delay(20);
    HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN_NUM, GPIO_PIN_SET);
    DHT11_DelayUs(30);

    /* 步骤2: 切换为输入, 等待 DHT11 响应 */
    DHT11_SetInput();

    /* DHT11 拉低 80us */
    timeout = 0;
    while (HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN_NUM) == GPIO_PIN_RESET)
    {
        if (++timeout > 3000) return 1;
    }
    /* DHT11 拉高 80us */
    timeout = 0;
    while (HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN_NUM) == GPIO_PIN_SET)
    {
        if (++timeout > 3000) return 1;
    }

    /* 步骤3: 读取 5 字节数据 */
    for (int i = 0; i < 5; i++)
    {
        buf[i] = DHT11_ReadByte();
        if (buf[i] == 0xFF) return 1;
    }

    /* 校验 */
    if (buf[4] != (uint8_t)(buf[0] + buf[1] + buf[2] + buf[3]))
        return 1;

    data->humidity    = buf[0];
    data->temperature = buf[2];

    return 0;
}
