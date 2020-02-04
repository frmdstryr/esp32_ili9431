/**
 * @file disp_par.c
 *
 */

/*********************
 *      INCLUDES
 *********************/
#include "disp_par.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "freertos/task.h"
#include "../lvgl/lvgl.h"

/*********************
 *      DEFINES
 *********************/

// Mask for the 8 data bits to set pin directions
#define dir_mask ((1 << TFT_D0) | (1 << TFT_D1) | (1 << TFT_D2) | (1 << TFT_D3) | (1 << TFT_D4) | (1 << TFT_D5) | (1 << TFT_D6) | (1 << TFT_D7))

// Data bits and the write line are cleared to 0 in one step
#define clr_mask (dir_mask | (1 << TFT_WR))

// A lookup table is used to set the different bit patterns, this uses 1kByte of RAM
#define set_mask(C) xset_mask[C] // 63fps Sprite rendering test 33% faster, graphicstest only 1.8% faster than shifting in real time


/**********************
 *      TYPEDEFS
 **********************/

/**********************
 *  STATIC PROTOTYPES
 **********************/

/**********************
 *  STATIC VARIABLES
 **********************/
uint32_t xclr_mask, xdir_mask, xset_mask[256];


/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/
void disp_par_init(void) {

    // Create a bit set lookup table for data bus - wastes 1kbyte of RAM
    // but speeds things up dramatically (from TFT_eSPI)
    for (int32_t c = 0; c<256; c++) {
        xset_mask[c] = 0;
        if ( c & 0x01 ) xset_mask[c] |= (1 << TFT_D0);
        if ( c & 0x02 ) xset_mask[c] |= (1 << TFT_D1);
        if ( c & 0x04 ) xset_mask[c] |= (1 << TFT_D2);
        if ( c & 0x08 ) xset_mask[c] |= (1 << TFT_D3);
        if ( c & 0x10 ) xset_mask[c] |= (1 << TFT_D4);
        if ( c & 0x20 ) xset_mask[c] |= (1 << TFT_D5);
        if ( c & 0x40 ) xset_mask[c] |= (1 << TFT_D6);
        if ( c & 0x80 ) xset_mask[c] |= (1 << TFT_D7);
    }

    gpio_set_level(TFT_CS, 0); // CS IDLE
    gpio_set_level(TFT_DC, 1); // CS DATA
    gpio_set_level(TFT_WR, 1); // RW IDLE
    gpio_set_level(TFT_RD, 1); // RD IDLE

    gpio_set_direction(TFT_CS, GPIO_MODE_OUTPUT);
    gpio_set_direction(TFT_DC, GPIO_MODE_OUTPUT);
    gpio_set_direction(TFT_WR, GPIO_MODE_OUTPUT);
    gpio_set_direction(TFT_RD, GPIO_MODE_OUTPUT);

    GPIO.out_w1ts = set_mask(255); // Set data bus to 0xFF

    disp_par_port_mode(GPIO_MODE_OUTPUT);

}


/**
 * Set the direction of the port
 */
void disp_par_port_mode(gpio_mode_t mode) {
   // Supports GPIO 0 - 31 on ESP32 only
    gpio_config_t gpio;
    gpio.pin_bit_mask = dir_mask;
    gpio.mode         = GPIO_MODE_INPUT;
    gpio.pull_up_en   = GPIO_PULLUP_ENABLE;
    gpio.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio.intr_type    = GPIO_INTR_DISABLE;
    if (mode == GPIO_MODE_OUTPUT) gpio.mode = GPIO_MODE_OUTPUT;
    gpio_config(&gpio);
}

void disp_par_send_data(uint8_t * data, uint16_t length) {
    // Only works with 1 byte
    gpio_set_level(TFT_CS, 0);
    for (uint16_t i=0; i<length; i++){
        GPIO.out_w1tc = clr_mask;  // This sets TFT_WR to low as well
        GPIO.out_w1ts = set_mask(data[i]);
        gpio_set_level(TFT_WR, 1);
    }
    gpio_set_level(TFT_CS, 1);
}

uint8_t disp_par_read_data(uint8_t * data, uint16_t length) {
    uint8_t b = 0;
    disp_par_send_data(data, length);
    gpio_set_level(TFT_RD, 0);
    uint32_t reg;           // Read all GPIO pins 0-31
    reg = gpio_input_get(); // Read three times to allow for bus access time
    reg = gpio_input_get();
    reg = gpio_input_get(); // Data should be stable now
    gpio_set_level(TFT_RD, 1);

    // Check GPIO bits used and build value
    b  = (((reg>>TFT_D0)&1) << 0);
    b |= (((reg>>TFT_D1)&1) << 1);
    b |= (((reg>>TFT_D2)&1) << 2);
    b |= (((reg>>TFT_D3)&1) << 3);
    b |= (((reg>>TFT_D4)&1) << 4);
    b |= (((reg>>TFT_D5)&1) << 5);
    b |= (((reg>>TFT_D6)&1) << 6);
    b |= (((reg>>TFT_D7)&1) << 7);
    return b;
}

void disp_par_send_colors(uint8_t * data, uint16_t length) {
    gpio_set_level(TFT_CS, 0);
    for (uint16_t i=0; i<length; i++){
        GPIO.out_w1tc = clr_mask;
        GPIO.out_w1ts = set_mask(data[i]);
        gpio_set_level(TFT_WR, 1);
    }
    gpio_set_level(TFT_CS, 1);

    //lv_flush_ready();
}


/**********************
 *   STATIC FUNCTIONS
 **********************/
