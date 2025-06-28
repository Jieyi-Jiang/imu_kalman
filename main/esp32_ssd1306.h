#pragma once

#include<stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "u8g2.h"
#include "u8x8.h"
#include "stdlib.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif
#define u8 uint8_t
#define u32 uint32_t

#define MAX_LEN         128  
#define OLED_ADDRESS    0x3C 
#define OLED_CMD        0x00  
#define OLED_DATA       0x40 

/* USER CODE BEGIN Prototypes */
uint8_t u8x8_byte_hw_i2c(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);
uint8_t u8x8_gpio_and_delay(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);
void u8g2Init(u8g2_t *u8g2, i2c_port_num_t port_num);
void draw_u8g2_logo(u8g2_t *u8g2);
void testDrawPixelToFillScreen(u8g2_t *u8g2);

#ifdef __cplusplus
}
#endif
