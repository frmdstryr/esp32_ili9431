/**
 * @file disp_par.h
 *
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/
#include <stdint.h>
#include "driver/gpio.h"

/*********************
 *      DEFINES
 *********************/

#define TFT_CS  5
#define TFT_DC  4
#define TFT_WR  2
#define TFT_RD  0

// 8-bit parallel line
#define TFT_D0  12
#define TFT_D1  13
#define TFT_D2  14
#define TFT_D3  15
#define TFT_D4  16
#define TFT_D5  17
#define TFT_D6  18
#define TFT_D7  19

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 * GLOBAL PROTOTYPES
 **********************/
void disp_par_init(void);
void disp_par_port_mode(gpio_mode_t mode);
void disp_par_send_data(uint8_t * data, uint16_t length);
uint8_t disp_par_read_data(uint8_t * data, uint16_t length);
void disp_par_send_colors(uint8_t * data, uint16_t length);

/**********************
 *      MACROS
 **********************/


#ifdef __cplusplus
} /* extern "C" */
#endif
