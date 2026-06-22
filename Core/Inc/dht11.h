#ifndef __DHT11_H__
#define __DHT11_H__

#include "main.h"

typedef struct {
    uint8_t temperature;
    uint8_t humidity;
} DHT11_Data;

uint8_t DHT11_Read(DHT11_Data *data);

#endif /* __DHT11_H__ */
