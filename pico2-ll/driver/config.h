#pragma once

#if defined(PIMORONI_PRESTO)

#define DPI_DATA_PIN_BASE  1
#define DPI_SYNC_PIN_BASE 19
#define DPI_CLOCK_PIN     22

#define DPI_MODE_CLOCK 15750000 // ish

#define DPI_MODE_H_FRONT_PORCH     4
#define DPI_MODE_H_SYNC_WIDTH     16
#define DPI_MODE_H_BACK_PORCH     30
#define DPI_MODE_H_ACTIVE_PIXELS 480

#define DPI_MODE_V_FRONT_PORCH     5
#define DPI_MODE_V_SYNC_WIDTH      8
#define DPI_MODE_V_BACK_PORCH      5
#define DPI_MODE_V_ACTIVE_LINES  480

#define DPI_SPI_INIT spi1
#define DPI_ST7701

#define DPI_BIT_REVERSE

#define FT6236_I2C i2c1
#define FT6236_SDA_PIN 30
#define FT6236_SCL_PIN 31
#define FT6236_INT_PIN 32
#define FT6236_ADDR 0x48

#define LCD_CS_PIN        28
#define LCD_DC_PIN        -1
#define LCD_SCK_PIN       26
#define LCD_MOSI_PIN      27
#define LCD_BACKLIGHT_PIN 45
#define LCD_RESET_PIN     44

#define SD_SCK  34
#define SD_MOSI 35
#define SD_MISO 36
#define SD_CS   39

#define DEFAULT_I2C_CLOCK 100000

#else
#error "No board configuration!"
#endif
