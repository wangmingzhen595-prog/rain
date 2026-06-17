#include "LCD_ILI9341.h"
#include "LCD_Font.h"
#include "Delay.h"

/* ---- control pin macros (GPIOB) ---- */
#define LCD_LED_PIN   GPIO_Pin_0    /* BL: moved off PB9 to PB0 (OLED uses PB6-PB9) */
#define LCD_RS_PIN    GPIO_Pin_10   /* DC */
#define LCD_CS_PIN    GPIO_Pin_11
#define LCD_RST_PIN   GPIO_Pin_12
#define LCD_SCK_PIN   GPIO_Pin_13   /* SPI2 */
#define LCD_MISO_PIN  GPIO_Pin_14   /* SPI2 (unused) */
#define LCD_MOSI_PIN  GPIO_Pin_15   /* SPI2 */

#define LCD_CS_LOW    (GPIOB->BRR  = LCD_CS_PIN)
#define LCD_CS_HIGH   (GPIOB->BSRR = LCD_CS_PIN)
#define LCD_RS_LOW    (GPIOB->BRR  = LCD_RS_PIN)
#define LCD_RS_HIGH   (GPIOB->BSRR = LCD_RS_PIN)
#define LCD_RST_LOW   (GPIOB->BRR  = LCD_RST_PIN)
#define LCD_RST_HIGH  (GPIOB->BSRR = LCD_RST_PIN)
#define LCD_LED_LOW   (GPIOB->BRR  = LCD_LED_PIN)
#define LCD_LED_HIGH  (GPIOB->BSRR = LCD_LED_PIN)

/* ---- low level SPI2 ---- */
static void LCD_SPI2_Init(void)
{
    GPIO_InitTypeDef gpio;
    SPI_InitTypeDef  spi;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

    /* SCK/MOSI -> SPI2 alternate function push-pull */
    gpio.GPIO_Pin = LCD_SCK_PIN | LCD_MOSI_PIN;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &gpio);

    /* MISO -> input pull-up (unused for write-only) */
    gpio.GPIO_Pin = LCD_MISO_PIN;
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOB, &gpio);

    /* control lines (incl. backlight PB0) -> push-pull output */
    gpio.GPIO_Pin = LCD_LED_PIN | LCD_RS_PIN | LCD_CS_PIN | LCD_RST_PIN;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &gpio);

    spi.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    spi.SPI_Mode = SPI_Mode_Master;
    spi.SPI_DataSize = SPI_DataSize_8b;
    spi.SPI_CPOL = SPI_CPOL_High;
    spi.SPI_CPHA = SPI_CPHA_2Edge;
    spi.SPI_NSS = SPI_NSS_Soft;
    spi.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2; /* PCLK1/2 = 18MHz */
    spi.SPI_FirstBit = SPI_FirstBit_MSB;
    spi.SPI_CRCPolynomial = 7;
    SPI_Init(SPI2, &spi);
    SPI_Cmd(SPI2, ENABLE);
}

static void LCD_SPI_Send(uint8_t b)
{
    while ((SPI2->SR & SPI_I2S_FLAG_TXE) == RESET) { }
    SPI2->DR = b;
    while ((SPI2->SR & SPI_I2S_FLAG_RXNE) == RESET) { }
    (void)SPI2->DR;
}

static void LCD_WriteCmd(uint8_t cmd)
{
    LCD_CS_LOW;
    LCD_RS_LOW;
    LCD_SPI_Send(cmd);
    LCD_CS_HIGH;
}

static void LCD_WriteData8(uint8_t data)
{
    LCD_CS_LOW;
    LCD_RS_HIGH;
    LCD_SPI_Send(data);
    LCD_CS_HIGH;
}

static void LCD_WriteData16(uint16_t data)
{
    LCD_WriteData8((uint8_t)(data >> 8));
    LCD_WriteData8((uint8_t)data);
}

void LCD_SetRegion(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    LCD_WriteCmd(0x2A);
    LCD_WriteData16(x0);
    LCD_WriteData16(x1);
    LCD_WriteCmd(0x2B);
    LCD_WriteData16(y0);
    LCD_WriteData16(y1);
    LCD_WriteCmd(0x2C);
}

void LCD_DrawPoint(uint16_t x, uint16_t y, uint16_t color)
{
    LCD_SetRegion(x, y, x, y);
    LCD_WriteData16(color);
}

/* Fast block fill: set region once, stream pixels with CS held low. */
void LCD_FillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
    uint32_t n;
    uint8_t hi = (uint8_t)(color >> 8);
    uint8_t lo = (uint8_t)color;

    if (w == 0 || h == 0) return;
    LCD_SetRegion(x, y, (uint16_t)(x + w - 1), (uint16_t)(y + h - 1));

    LCD_CS_LOW;
    LCD_RS_HIGH;
    for (n = (uint32_t)w * h; n > 0; n--)
    {
        LCD_SPI_Send(hi);
        LCD_SPI_Send(lo);
    }
    LCD_CS_HIGH;
}

void LCD_Clear(uint16_t color)
{
    LCD_FillRect(0, 0, LCD_W, LCD_H, color);
}

void LCD_DrawHLine(uint16_t x, uint16_t y, uint16_t w, uint16_t color)
{
    LCD_FillRect(x, y, w, 1, color);
}

void LCD_DrawVLine(uint16_t x, uint16_t y, uint16_t h, uint16_t color)
{
    LCD_FillRect(x, y, 1, h, color);
}

void LCD_DrawRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
    if (w == 0 || h == 0) return;
    LCD_DrawHLine(x, y, w, color);
    LCD_DrawHLine(x, (uint16_t)(y + h - 1), w, color);
    LCD_DrawVLine(x, y, h, color);
    LCD_DrawVLine((uint16_t)(x + w - 1), y, h, color);
}

/* Bresenham line via single-point writes. */
void LCD_DrawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color)
{
    int dx = (int)x1 - (int)x0;
    int dy = (int)y1 - (int)y0;
    int x_inc = (dx >= 0) ? 1 : -1;
    int y_inc = (dy >= 0) ? 1 : -1;
    int index;

    if (dx < 0) dx = -dx;
    if (dy < 0) dy = -dy;

    if (dx > dy)
    {
        int error = (dy << 1) - dx;
        for (index = 0; index <= dx; index++)
        {
            LCD_DrawPoint(x0, y0, color);
            if (error >= 0) { error -= (dx << 1); y0 = (uint16_t)((int)y0 + y_inc); }
            error += (dy << 1);
            x0 = (uint16_t)((int)x0 + x_inc);
        }
    }
    else
    {
        int error = (dx << 1) - dy;
        for (index = 0; index <= dy; index++)
        {
            LCD_DrawPoint(x0, y0, color);
            if (error >= 0) { error -= (dy << 1); x0 = (uint16_t)((int)x0 + x_inc); }
            error += (dx << 1);
            y0 = (uint16_t)((int)y0 + y_inc);
        }
    }
}

/* ---- text ---- */
void LCD_ShowChar(uint16_t x, uint16_t y, char ch, uint16_t fc, uint16_t bc)
{
    uint8_t i, j;
    uint16_t k = (uint8_t)ch;

    if (k < 32) return;
    k -= 32;                       /* asc16 starts at space */
    for (i = 0; i < 16; i++)
    {
        uint8_t row = asc16[k * 16 + i];
        for (j = 0; j < 8; j++)
        {
            if (row & (0x80 >> j))      LCD_DrawPoint((uint16_t)(x + j), (uint16_t)(y + i), fc);
            else if (fc != bc)          LCD_DrawPoint((uint16_t)(x + j), (uint16_t)(y + i), bc);
        }
    }
}

void LCD_ShowString(uint16_t x, uint16_t y, const char *s, uint16_t fc, uint16_t bc)
{
    while (*s)
    {
        LCD_ShowChar(x, y, *s, fc, bc);
        x += 8;
        s++;
    }
}

/* Big 32x32 glyph (4 bytes/row) from sz32 by index. */
void LCD_ShowBigGlyph(uint16_t x, uint16_t y, uint8_t idx, uint16_t fc, uint16_t bc)
{
    uint8_t i, j, k;

    for (i = 0; i < 32; i++)
    {
        for (j = 0; j < 4; j++)
        {
            uint8_t c = sz32[idx * 32 * 4 + i * 4 + j];
            for (k = 0; k < 8; k++)
            {
                if (c & (0x80 >> k))   LCD_DrawPoint((uint16_t)(x + j * 8 + k), (uint16_t)(y + i), fc);
                else if (fc != bc)     LCD_DrawPoint((uint16_t)(x + j * 8 + k), (uint16_t)(y + i), bc);
            }
        }
    }
}

void LCD_Backlight(uint8_t on)
{
    if (on) LCD_LED_HIGH;
    else    LCD_LED_LOW;
}

void LCD_Init(void)
{
    LCD_SPI2_Init();

    /* hardware reset */
    LCD_RST_LOW;
    Delay_ms(100);
    LCD_RST_HIGH;
    Delay_ms(50);

    LCD_WriteCmd(0x11);            /* sleep out */
    LCD_WriteData8(0x00);

    LCD_WriteCmd(0xCF);
    LCD_WriteData8(0x00); LCD_WriteData8(0xC1); LCD_WriteData8(0x30);
    LCD_WriteCmd(0xED);
    LCD_WriteData8(0x64); LCD_WriteData8(0x03); LCD_WriteData8(0x12); LCD_WriteData8(0x81);
    LCD_WriteCmd(0xE8);
    LCD_WriteData8(0x85); LCD_WriteData8(0x11); LCD_WriteData8(0x78);
    LCD_WriteCmd(0xF6);
    LCD_WriteData8(0x01); LCD_WriteData8(0x30); LCD_WriteData8(0x00);
    LCD_WriteCmd(0xCB);
    LCD_WriteData8(0x39); LCD_WriteData8(0x2C); LCD_WriteData8(0x00);
    LCD_WriteData8(0x34); LCD_WriteData8(0x05);
    LCD_WriteCmd(0xF7);
    LCD_WriteData8(0x20);
    LCD_WriteCmd(0xEA);
    LCD_WriteData8(0x00); LCD_WriteData8(0x00);
    LCD_WriteCmd(0xC0); LCD_WriteData8(0x20);          /* power control 1 */
    LCD_WriteCmd(0xC1); LCD_WriteData8(0x11);          /* power control 2 */
    LCD_WriteCmd(0xC5); LCD_WriteData8(0x31); LCD_WriteData8(0x3C); /* VCOM */
    LCD_WriteCmd(0xC7); LCD_WriteData8(0xA9);
    LCD_WriteCmd(0x3A); LCD_WriteData8(0x55);          /* 16-bit/pixel */
    LCD_WriteCmd(0x36); LCD_WriteData8(0xE8);          /* MADCTL: landscape */

    LCD_WriteCmd(0xB1); LCD_WriteData8(0x00); LCD_WriteData8(0x18);
    LCD_WriteCmd(0xB4); LCD_WriteData8(0x00); LCD_WriteData8(0x00);
    LCD_WriteCmd(0xF2); LCD_WriteData8(0x00);
    LCD_WriteCmd(0x26); LCD_WriteData8(0x01);

    LCD_WriteCmd(0xE0);            /* positive gamma */
    LCD_WriteData8(0x0F); LCD_WriteData8(0x17); LCD_WriteData8(0x14); LCD_WriteData8(0x09);
    LCD_WriteData8(0x0C); LCD_WriteData8(0x06); LCD_WriteData8(0x43); LCD_WriteData8(0x75);
    LCD_WriteData8(0x36); LCD_WriteData8(0x08); LCD_WriteData8(0x13); LCD_WriteData8(0x05);
    LCD_WriteData8(0x10); LCD_WriteData8(0x0B); LCD_WriteData8(0x08);
    LCD_WriteCmd(0xE1);            /* negative gamma */
    LCD_WriteData8(0x00); LCD_WriteData8(0x1F); LCD_WriteData8(0x23); LCD_WriteData8(0x03);
    LCD_WriteData8(0x0E); LCD_WriteData8(0x04); LCD_WriteData8(0x39); LCD_WriteData8(0x25);
    LCD_WriteData8(0x4D); LCD_WriteData8(0x06); LCD_WriteData8(0x0D); LCD_WriteData8(0x0B);
    LCD_WriteData8(0x33); LCD_WriteData8(0x37); LCD_WriteData8(0x0F);

    LCD_WriteCmd(0x29);            /* display on */

    LCD_Backlight(1);
    LCD_Clear(LCD_BLACK);
}
