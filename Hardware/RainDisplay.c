#include "RainDisplay.h"
#include "LCD_ILI9341.h"

/* ---- layout constants (landscape 320x240) ---- */
#define TITLE_H        22U

#define BOX_X          4U
#define BOX_Y          26U
#define BOX_W          312U
#define BOX_H          146U

#define PLOT_X         7U          /* inner plot area */
#define PLOT_Y         29U
#define PLOT_W         306U
#define PLOT_H         140U
#define PLOT_BOTTOM    (PLOT_Y + PLOT_H - 1U)   /* y of ADC=0 */

#define ADC_FS         4095U       /* full scale = 3.3V */

#define STAT_ROW1_Y    180U
#define STAT_ROW2_Y    212U
#define STAT_COL1_X    6U
#define STAT_COL2_X    114U
#define STAT_COL3_X    222U

/* ---- semantic palette ---- */
#define COL_BG         LCD_BLACK
#define COL_TRACE      LCD_GREEN
#define COL_TRACE_PA1  LCD_ORANGE
#define COL_GRID       LCD_DGRAY
#define COL_BASE       LCD_GRAY
#define COL_THRESH     LCD_YELLOW
#define COL_PEAK       LCD_RED
#define COL_TITLE      LCD_WHITE
#define COL_ACCENT     LCD_NAVY
#define COL_LABEL      LCD_LGRAY
#define COL_VALUE      LCD_WHITE
#define COL_TOTAL      LCD_CYAN
#define COL_VOL        LCD_YELLOW

/* ---- small integer formatting helpers (no printf/float lib) ---- */

/* unsigned decimal, right-aligned not needed: returns chars written */
static uint8_t u32_to_str(uint32_t v, char *out)
{
    char tmp[11];
    uint8_t n = 0, i;
    if (v == 0) { out[0] = '0'; out[1] = '\0'; return 1; }
    while (v > 0 && n < 10) { tmp[n++] = (char)('0' + (v % 10)); v /= 10; }
    for (i = 0; i < n; i++) out[i] = tmp[n - 1 - i];
    out[n] = '\0';
    return n;
}

/* value scaled by 100 -> "int.dd" */
static void fixed2_to_str(uint32_t v100, char *out)
{
    uint8_t n = u32_to_str(v100 / 100U, out);
    out[n++] = '.';
    out[n++] = (char)('0' + (v100 / 10U) % 10U);
    out[n++] = (char)('0' + v100 % 10U);
    out[n]   = '\0';
}

/* draw "label value" then clear trailing cells up to field_chars wide */
static void draw_field(uint16_t x, uint16_t y, const char *label,
                       const char *value, uint16_t value_color,
                       uint8_t field_chars)
{
    uint16_t cx = x;
    uint8_t used = 0;

    while (*label) { LCD_ShowChar(cx, y, *label, COL_LABEL, COL_BG); cx += 8; label++; used++; }
    LCD_ShowChar(cx, y, ' ', COL_LABEL, COL_BG); cx += 8; used++;
    while (*value) { LCD_ShowChar(cx, y, *value, value_color, COL_BG); cx += 8; value++; used++; }
    /* pad the rest of the field with spaces so stale digits get erased */
    while (used < field_chars) { LCD_ShowChar(cx, y, ' ', COL_BG, COL_BG); cx += 8; used++; }
}

static void draw_gain_badge(char gain)
{
    uint16_t color = (gain == 'L') ? LCD_ORANGE : LCD_GREEN;
    /* badge box at top-right */
    LCD_FillRect(290, 3, 26, 16, color);
    LCD_ShowChar(290 + 9, 3, gain, LCD_BLACK, color);
}

/* ---- waveform plot ---- */

static uint16_t adc_to_y(uint16_t adc)
{
    uint32_t off;
    if (adc > ADC_FS) adc = ADC_FS;
    off = (uint32_t)adc * (PLOT_H - 1U) / ADC_FS;
    return (uint16_t)(PLOT_BOTTOM - off);
}

static void plot_draw_grid(void)
{
    uint8_t i;
    /* clear plot interior */
    LCD_FillRect(PLOT_X, PLOT_Y, PLOT_W, PLOT_H, COL_BG);
    /* 3 inner vertical + 2 inner horizontal divisions */
    for (i = 1; i < 4; i++)
        LCD_DrawVLine((uint16_t)(PLOT_X + i * (PLOT_W / 4U)), PLOT_Y, PLOT_H, COL_GRID);
    for (i = 1; i < 3; i++)
        LCD_DrawHLine(PLOT_X, (uint16_t)(PLOT_Y + i * (PLOT_H / 3U)), PLOT_W, COL_GRID);
}

void RainDisplay_DrawWaveform(const uint16_t *buf, uint16_t len,
                              uint16_t baseline, uint16_t threshold,
                              uint16_t peak_index, uint8_t pa1_takeover)
{
    uint16_t c;
    uint16_t prev_x = PLOT_X, prev_y;
    uint16_t y_base, y_thr;
    uint16_t trace_color = pa1_takeover ? COL_TRACE_PA1 : COL_TRACE;

    if (buf == 0 || len == 0) return;

    plot_draw_grid();

    /* reference lines */
    y_base = adc_to_y(baseline);
    y_thr  = adc_to_y(threshold);
    LCD_DrawHLine(PLOT_X, y_base, PLOT_W, COL_BASE);
    for (c = 0; c < PLOT_W; c += 6)             /* dashed threshold */
        LCD_DrawHLine((uint16_t)(PLOT_X + c), y_thr, 3, COL_THRESH);

    /* trace: map each screen column to a sample, connect with lines */
    prev_y = adc_to_y(buf[0]);
    for (c = 0; c < PLOT_W; c++)
    {
        uint16_t idx = (uint16_t)((uint32_t)c * (len - 1U) / (PLOT_W - 1U));
        uint16_t x = (uint16_t)(PLOT_X + c);
        uint16_t y = adc_to_y(buf[idx]);
        if (c == 0) { LCD_DrawPoint(x, y, trace_color); }
        else        { LCD_DrawLine(prev_x, prev_y, x, y, trace_color); }
        prev_x = x;
        prev_y = y;
    }

    /* peak marker */
    if (peak_index < len)
    {
        uint16_t px = (uint16_t)(PLOT_X + (uint32_t)peak_index * (PLOT_W - 1U) / (len - 1U));
        uint16_t py = adc_to_y(buf[peak_index]);
        LCD_DrawHLine((uint16_t)(px - 2), py, 5, COL_PEAK);
        LCD_DrawVLine(px, (uint16_t)(py - 2), 5, COL_PEAK);
    }
}

/* ---- static frame ---- */

void RainDisplay_DrawStatic(void)
{
    LCD_Clear(COL_BG);

    /* title bar */
    LCD_FillRect(0, 0, 4, TITLE_H, COL_ACCENT);
    LCD_ShowString(10, 3, "RAIN MONITOR", COL_TITLE, COL_BG);

    /* waveform border + empty grid (so the plot looks intentional at boot) */
    LCD_DrawRect(BOX_X, BOX_Y, BOX_W, BOX_H, LCD_GRAY);
    plot_draw_grid();

    /* static stat labels are drawn together with values in UpdateStats;
       here we just leave the area cleared by LCD_Clear. */
}

void RainDisplay_Init(void)
{
    LCD_Init();
    RainDisplay_DrawStatic();
}

void RainDisplay_UpdateStats(float volt, uint32_t integral_mv_us,
                             uint32_t drops, uint32_t last_vol_0p01mm3,
                             uint32_t total_0p01mm3, float intensity_mmh,
                             char gain)
{
    char buf[16];
    uint32_t v100;

    /* intensity in title bar: "12.3 mm/h" */
    v100 = (uint32_t)(intensity_mmh * 10.0f + 0.5f);     /* x10 -> one decimal */
    {
        char t[16];
        uint8_t n = u32_to_str(v100 / 10U, t);
        t[n++] = '.';
        t[n++] = (char)('0' + v100 % 10U);
        t[n] = '\0';
        LCD_ShowString(150, 3, "        ", COL_BG, COL_BG);  /* clear */
        LCD_ShowString(150, 3, t, COL_LABEL, COL_BG);
        LCD_ShowString(150 + (uint16_t)n * 8 + 4, 3, "mm/h", COL_LABEL, COL_BG);
    }
    draw_gain_badge(gain);

    /* row 1 */
    v100 = (uint32_t)(volt * 100.0f + 0.5f);
    fixed2_to_str(v100, buf);
    draw_field(STAT_COL1_X, STAT_ROW1_Y, "Volt", buf, COL_VALUE, 12);

    u32_to_str(integral_mv_us, buf);
    draw_field(STAT_COL2_X, STAT_ROW1_Y, "Imp", buf, COL_VALUE, 13);

    u32_to_str(drops, buf);
    draw_field(STAT_COL3_X, STAT_ROW1_Y, "Drop", buf, COL_VALUE, 11);

    /* row 2 */
    fixed2_to_str(last_vol_0p01mm3, buf);
    draw_field(STAT_COL1_X, STAT_ROW2_Y, "Vol", buf, COL_VOL, 12);

    fixed2_to_str(total_0p01mm3, buf);
    draw_field(STAT_COL2_X, STAT_ROW2_Y, "Tot", buf, COL_TOTAL, 20);
}

/* ---- self test ---- */

static uint16_t selftest_buf[PLOT_W];

void RainDisplay_SelfTest(void)
{
    uint16_t i;
    const uint16_t base = 300;
    const uint16_t amp  = 3400;          /* peak ~3.7k ADC */
    const int center = (int)PLOT_W / 3;  /* pulse early, like a real event */
    const int half   = 60;

    RainDisplay_DrawStatic();

    /* synthesize a parabolic pulse on a flat baseline */
    for (i = 0; i < PLOT_W; i++)
    {
        int d = (int)i - center;
        int v = base;
        if (d > -half && d < half)
        {
            int f = (half * half - d * d);          /* 0..half^2 */
            v = base + (int)((long)amp * f / (half * half));
        }
        selftest_buf[i] = (uint16_t)v;
    }

    RainDisplay_DrawWaveform(selftest_buf, PLOT_W,
                             base, 620, (uint16_t)center, 0);

    RainDisplay_UpdateStats(2.85f, 1367650U, 1234U, 3008U, 123456U, 12.3f, 'H');
}
