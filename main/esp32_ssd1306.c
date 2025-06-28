#include "esp32_ssd1306.h"
#include "stdlib.h"
#include "driver/gpio.h"
#include "sys/unistd.h"
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include "driver/i2c_master.h"
#include "u8x8.h"
#include "u8g2.h"
// #define I2C_HOST  0

// #define EXAMPLE_LCD_PIXEL_CLOCK_HZ    (100 * 1000)
#define SSD1306_I2C_SPEED_HZ        (400*1000)

// #define EXAMPLE_PIN_NUM_SDA     5
// #define EXAMPLE_PIN_NUM_SCL     4
//使用OLED的硬件地址，i2c_master_write_to_device()函数内会转换为写地址
#define EXAMPLE_I2C_HW_ADDR     0x3C

#define I2C_MASTER_TIMEOUT_MS   1000

static i2c_port_num_t iic_port_num;



void u8g2Init(u8g2_t *u8g2, i2c_port_num_t port_num)
{
    iic_port_num = port_num;
	u8g2_Setup_ssd1306_i2c_128x64_noname_f(u8g2, U8G2_R0, u8x8_byte_hw_i2c, u8x8_gpio_and_delay); //
	u8g2_InitDisplay(u8g2);                                                                       //
	u8g2_SetPowerSave(u8g2, 0);                                                                   //
	u8g2_ClearBuffer(u8g2);
}

uint8_t u8x8_byte_hw_i2c(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
    /* u8g2/u8x8 will never send more than 32 bytes between START_TRANSFER and END_TRANSFER */

    static uint8_t buf[128];
    static uint8_t buf_idx;
    i2c_master_dev_handle_t dev_handle;
    i2c_master_bus_handle_t bus_handle;

    switch (msg)
    {
    case U8X8_MSG_BYTE_INIT:
    {
        // i2c_master_init();
        break;
    }
    case U8X8_MSG_BYTE_START_TRANSFER:
    {
        buf_idx = 0;
        break;
    }
    case U8X8_MSG_BYTE_SEND:
    {
        while (arg_int > 0)
        {
            buf[buf_idx++] = *(uint8_t*)arg_ptr++;
            arg_int--;
        }
        break;
    }
    case U8X8_MSG_BYTE_END_TRANSFER:
    {
        int ret;
        // ret = i2c_master_write_to_device(I2C_HOST, OLED_ADDRESS, buf, buf_idx, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
        ESP_ERROR_CHECK(i2c_master_get_bus_handle(iic_port_num, &bus_handle));
        i2c_device_config_t dev_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = OLED_ADDRESS,
            .scl_speed_hz = 100000,
            .scl_wait_us = 10000,
        };
        ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));
        ret = i2c_master_transmit(dev_handle, buf, buf_idx, 1000);
        ESP_ERROR_CHECK(i2c_master_bus_rm_device(dev_handle));
		if (ret!= ESP_OK) return 0;
		else break;
    }
    case U8X8_MSG_BYTE_SET_DC:
        break;

    default:
        return 0;
    }

    return 1;
}



uint8_t u8x8_gpio_and_delay(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
	//使用硬件IIC，部分分支可忽略不写
    switch (msg)
    {
    case U8X8_MSG_DELAY_100NANO: // delay arg_int * 100 nano seconds
        break;
    case U8X8_MSG_DELAY_10MICRO: // delay arg_int * 10 micro seconds
        break;
    case U8X8_MSG_DELAY_MILLI: // delay arg_int * 1 milli second
        break;
    case U8X8_MSG_DELAY_I2C: // arg_int is the I2C speed in 100KHz, e.g. 4 = 400 KHz
        break;                    // arg_int=1: delay by 5us, arg_int = 4: delay by 1.25us
    case U8X8_MSG_GPIO_I2C_CLOCK: // arg_int=0: Output low at I2C clock pin
        break;                    // arg_int=1: Input dir with pullup high for I2C clock pin
    case U8X8_MSG_GPIO_I2C_DATA:  // arg_int=0: Output low at I2C data pin
        break;                    // arg_int=1: Input dir with pullup high for I2C data pin
    case U8X8_MSG_GPIO_MENU_SELECT:
        // u8x8_SetGPIOResult(u8x8, /* get menu select pin state */ 0);
        break;
    case U8X8_MSG_GPIO_MENU_NEXT:
        // u8x8_SetGPIOResult(u8x8, /* get menu next pin state */ 0);
        break;
    case U8X8_MSG_GPIO_MENU_PREV:
        // u8x8_SetGPIOResult(u8x8, /* get menu prev pin state */ 0);
        break;
    case U8X8_MSG_GPIO_MENU_HOME:
        // u8x8_SetGPIOResult(u8x8, /* get menu home pin state */ 0);
        break;
    default:
        u8x8_SetGPIOResult(u8x8, 1); // default return value
        break;
    }
    return 1;
}

void draw_u8g2_logo(u8g2_t *u8g2)
{
	u8g2_ClearBuffer(u8g2);
    u8g2_SetFontMode(u8g2, 1); /*字体模式选择*/
    u8g2_SetFontDirection(u8g2, 0); /*字体方向选择*/
    u8g2_SetFont(u8g2, u8g2_font_inb24_mf); /*字库选择*/
    u8g2_DrawStr(u8g2, 0, 20, "U");

    u8g2_SetFontDirection(u8g2, 1);
    u8g2_SetFont(u8g2, u8g2_font_inb30_mn);
    u8g2_DrawStr(u8g2, 21,8,"8");

    u8g2_SetFontDirection(u8g2, 0);
    u8g2_SetFont(u8g2, u8g2_font_inb24_mf);
    u8g2_DrawStr(u8g2, 51,30,"g");
    u8g2_DrawStr(u8g2, 67,30,"\xb2");

    u8g2_DrawHLine(u8g2, 2, 35, 47);
    u8g2_DrawHLine(u8g2, 3, 36, 47);
    u8g2_DrawVLine(u8g2, 45, 32, 12);
    u8g2_DrawVLine(u8g2, 46, 33, 12);

    u8g2_SetFont(u8g2, u8g2_font_5x7_tr);
    u8g2_DrawStr(u8g2, 1,54,"github.com/olikraus/u8g2");
    u8g2_SendBuffer(u8g2);
}



