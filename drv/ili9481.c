/**
 * @file ili9481.c
 *
 */

/*********************
 *      INCLUDES
 *********************/
#include "disp_par.h"
#include "ili9481.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/

/*The LCD needs a bunch of command/argument values to be initialized. They are stored in this struct. */
typedef struct {
    uint8_t cmd;
    uint8_t data[16];
    uint8_t databytes; //No of data in data; bit 7 = delay after set; 0xFF = end of cmds.
} lcd_init_cmd_t;

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void ili9481_send_cmd(uint8_t cmd);
static void ili9481_send_data(void * data, uint16_t length);
static void ili9481_send_color(void * data, uint16_t length);

/**********************
 *  STATIC VARIABLES
 **********************/

/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

void ili9481_init(void)
{
	lcd_init_cmd_t ili_init_cmds[]={
		{TFT_SLPOUT, {0}, 0x80},
        {0xD0, {0x07, 0x42, 0x18}, 3}, // Power setting
		{0xD1, {0x00, 0x07, 0x10}, 3}, // VCOM
        {0xD2, {0x01, 0x02}, 2}, // Norm mode
		{0xC0, {0x10, 0x3B, 0x00, 0x02, 0x11}, 5}, // Driving setting
		{0xC5, {0x03}, 1}, // Frame rate & inv
        // Gamma
		{0xC8, {0x00, 0x32, 0x36, 0x45, 0x06, 0x16,
                0x37, 0x75, 0x77, 0x54, 0x0C, 0x00}, 12},
		{0x3A, {0x55}, 1}, // Pixel format
        //{TFT_INVOFF, {0}, 0}, // Invert off
        {TFT_MADCTL, {0x4A}, 1},
		{TFT_CASET, {0x00, 0x00, 0x01, 0x3F}, 4},
		{TFT_PASET, {0x00, 0x00, 0x01, 0xDF}, 4},
		{TFT_DISPON, {0}, 0x80},
		{0, {0}, 0xff},
	};

	//Initialize non-SPI GPIOs
	gpio_set_direction(TFT_DC, GPIO_MODE_OUTPUT);
	gpio_set_direction(TFT_RST, GPIO_MODE_OUTPUT);

	//Reset the display
	gpio_set_level(TFT_RST, 0);
	vTaskDelay(100 / portTICK_RATE_MS);
	gpio_set_level(TFT_RST, 1);
	vTaskDelay(100 / portTICK_RATE_MS);


	printf("ILI9481 initialization.\n");


	//Send all the commands
	uint16_t cmd = 0;
	while (ili_init_cmds[cmd].databytes!=0xff) {
		ili9481_send_cmd(ili_init_cmds[cmd].cmd);
		ili9481_send_data(ili_init_cmds[cmd].data, ili_init_cmds[cmd].databytes&0x1F);
		if (ili_init_cmds[cmd].databytes & 0x80) {
			vTaskDelay(100 / portTICK_RATE_MS);
		}
		cmd++;
	}

	///Enable backlight
	printf("Initialization complete.\n");
}

void ili9481_fill(int32_t x1, int32_t y1, int32_t x2, int32_t y2, lv_color_t color)
{
	uint8_t data[4];

	/*Column addresses*/
	ili9481_send_cmd(0x2A);
	data[0] = (x1 >> 8) & 0xFF;
	data[1] = x1 & 0xFF;
	data[2] = (x2 >> 8) & 0xFF;
	data[3] = x2 & 0xFF;
	ili9481_send_data(data, 4);

	/*Page addresses*/
	ili9481_send_cmd(0x2B);
	data[0] = (y1 >> 8) & 0xFF;
	data[1] = y1 & 0xFF;
	data[2] = (y2 >> 8) & 0xFF;
	data[3] = y2 & 0xFF;
	ili9481_send_data(data, 4);

	/*Memory write*/
	ili9481_send_cmd(0x2C);

	uint32_t size = (x2 - x1 + 1) * (y2 - y1 + 1);
	uint16_t buf[LV_HOR_RES];

	uint32_t i;
	if(size < LV_HOR_RES) {
		for(i = 0; i < size; i++) buf[i] = color.full;

	} else {
		for(i = 0; i < LV_HOR_RES; i++) buf[i] = color.full;
	}

	while(size > LV_HOR_RES) {
		ili9481_send_color(buf, LV_HOR_RES * 2);
		size -= LV_HOR_RES;
	}

	ili9481_send_color(buf, size * 2);	/*Send the remaining data*/
}


void ili9481_flush(int32_t x1, int32_t y1, int32_t x2, int32_t y2, const lv_color_t * color_map)
{
	uint8_t data[4];

	/*Column addresses*/
	ili9481_send_cmd(0x2A);
	data[0] = (x1 >> 8) & 0xFF;
	data[1] = x1 & 0xFF;
	data[2] = (x2 >> 8) & 0xFF;
	data[3] = x2 & 0xFF;
	ili9481_send_data(data, 4);

	/*Page addresses*/
	ili9481_send_cmd(0x2B);
	data[0] = (y1 >> 8) & 0xFF;
	data[1] = y1 & 0xFF;
	data[2] = (y2 >> 8) & 0xFF;
	data[3] = y2 & 0xFF;
	ili9481_send_data(data, 4);

	/*Memory write*/
	ili9481_send_cmd(0x2C);


	uint32_t size = (x2 - x1 + 1) * (y2 - y1 + 1);


	ili9481_send_color((void*)color_map, size * 2);	/*Send the remaining data*/

	lv_flush_ready();

}

/**********************
 *   STATIC FUNCTIONS
 **********************/


static void ili9481_send_cmd(uint8_t cmd)
{
	gpio_set_level(TFT_DC, 0);	 /*Command mode*/
	disp_par_send_data(&cmd, 1);
}

static void ili9481_send_data(void * data, uint16_t length)
{
	gpio_set_level(TFT_DC, 1);	 /*Data mode*/
	disp_par_send_data(data, length);
}

static void ili9481_send_color(void * data, uint16_t length)
{
    gpio_set_level(TFT_DC, 1);   /*Data mode*/
    disp_par_send_colors(data, length);
}
