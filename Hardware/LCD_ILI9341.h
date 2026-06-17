#ifndef __LCD_ILI9341_H
#define __LCD_ILI9341_H

#include "stm32f10x.h"

/* ============================================================================
 * ILI9341 240x320 SPI TFT driver, landscape 320x240.
 *
 * Bus  : SPI2 hardware (18MHz, mode 3) - chosen to avoid SPI1 (master link).
 * Pins : SCK=PB13, MOSI=PB15 (SPI2 AF); DC/RS=PB10, CS=PB11, RST=PB12,
 *        BL/LED=PB0 (driven high = backlight on). MISO(PB14) not used.
 * Note : OLED is still in use on PB6-PB9, so the backlight was moved from
 *        the vendor default PB9 to PB0. The LCD pins (PB0, PB10-PB13, PB15)
 *        do not conflict with OLED (PB6-PB9), the rain ADC (PA0/1), the
 *        SPI1 master link (PA4-7) or USART1 (PA9/10).
 * ========================================================================== */

#define LCD_W   320U   /* landscape width  (long axis) */
#define LCD_H   240U   /* landscape height (short axis) */

/* RGB565 generic colors */
#define LCD_BLACK   0x0000
#define LCD_WHITE   0xFFFF
#define LCD_RED     0xF800
#define LCD_GREEN   0x07E0
#define LCD_BLUE    0x001F
#define LCD_YELLOW  0xFFE0
#define LCD_CYAN    0x07FF
#define LCD_ORANGE  0xFD20
#define LCD_GRAY    0x8410
#define LCD_DGRAY   0x2104   /* dark gray, for grid */
#define LCD_LGRAY   0xC618   /* light gray, for labels */
#define LCD_NAVY    0x041F   /* accent blue */

/* sz32 big-glyph indices (see LCD_Font.h) */
#define LCD_BIG_DOT     10U
#define LCD_BIG_COLON   11U
#define LCD_BIG_PERCENT 12U
#define LCD_BIG_DEGC    13U
#define LCD_BIG_MINUS   14U

void LCD_Init(void);
void LCD_Backlight(uint8_t on);
void LCD_Clear(uint16_t color);
void LCD_SetRegion(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);
void LCD_FillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
void LCD_DrawPoint(uint16_t x, uint16_t y, uint16_t color);
void LCD_DrawHLine(uint16_t x, uint16_t y, uint16_t w, uint16_t color);
void LCD_DrawVLine(uint16_t x, uint16_t y, uint16_t h, uint16_t color);
void LCD_DrawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color);
void LCD_DrawRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);

/* Text: 8x16 ASCII. If bc==fc the background is left untouched (transparent). */
void LCD_ShowChar(uint16_t x, uint16_t y, char ch, uint16_t fc, uint16_t bc);
void LCD_ShowString(uint16_t x, uint16_t y, const char *s, uint16_t fc, uint16_t bc);
/* Big 32x32 glyph by sz32 index (0-9 digits, LCD_BIG_* for symbols). */
void LCD_ShowBigGlyph(uint16_t x, uint16_t y, uint8_t idx, uint16_t fc, uint16_t bc);

#endif /* __LCD_ILI9341_H */
