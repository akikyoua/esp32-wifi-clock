#ifndef _DISPLAY_H_
#define _DISPLAY_H_


#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"

extern unsigned char bleConn;
extern unsigned char wifiConn;

extern unsigned char secondMode;
extern unsigned char daylightness;
extern unsigned char nightlightness;
extern unsigned char nightMode;
extern unsigned char nightStart;
extern unsigned char nightEnd;


#define GPIO_OUTPUT_IO_0    26
#define GPIO_OUTPUT_IO_1    27
#define GPIO_OUTPUT_IO_2    32
#define GPIO_OUTPUT_IO_3    33
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<GPIO_OUTPUT_IO_0) | (1ULL<<GPIO_OUTPUT_IO_1) | (1ULL<<GPIO_OUTPUT_IO_2) | (1ULL<<GPIO_OUTPUT_IO_3))

#define GPIO_INPUT_IO_4    12
#define GPIO_INPUT_PIN_SEL  (1ULL<<GPIO_INPUT_IO_4)

#define DISM() gpio_get_level(GPIO_INPUT_IO_4)

#define DIN2H() gpio_set_level(GPIO_OUTPUT_IO_0, 1)
#define DIN2L() gpio_set_level(GPIO_OUTPUT_IO_0, 0)
#define SCK2H() gpio_set_level(GPIO_OUTPUT_IO_1, 1)
#define SCK2L() gpio_set_level(GPIO_OUTPUT_IO_1, 0)
#define DIN1H() gpio_set_level(GPIO_OUTPUT_IO_2, 1)
#define DIN1L() gpio_set_level(GPIO_OUTPUT_IO_2, 0)
#define SCK1H() gpio_set_level(GPIO_OUTPUT_IO_3, 1)
#define SCK1L() gpio_set_level(GPIO_OUTPUT_IO_3, 0)

#define WRITE_DATA_MODE_Z   0x40        // 地址自动加
#define WRITE_DATA_MODE_G   0x44        // 固定地址
#define START_DATA          0xC0        // 
#define DISPLAY_EN          0x8A        // 开显示
#define DISPLAY_DIS         0x80        // 关显示


void displayInit(void);
void displayTask(void* arg);

#endif

