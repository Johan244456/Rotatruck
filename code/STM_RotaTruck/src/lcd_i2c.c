#include "lcd_i2c.h"
#include "board.h"
#include "stm32g4xx_hal.h"

#include <string.h>

extern I2C_HandleTypeDef hi2c_imu;   /* LCD shares I2C1 with the ISM330DHCX */

/* PCF8574 backpack wiring: P0=RS, P1=RW, P2=EN, P3=backlight, P4..P7=D4..D7.
 * RW (P1) is held low - this driver only ever writes. */
#define LCD_RS        0x01u
#define LCD_EN        0x04u
#define LCD_BACKLIGHT 0x08u
#define LCD_CMD_CLEAR 0x01u

#define LCD_COLS            16
#define LCD_MIN_REFRESH_MS  200U   /* cap full repaints to ~5 Hz */
#define LCD_OP_DONE         (2 * LCD_COLS + 1)   /* last op index (33) */

static uint8_t        s_addr_hal  = 0;
static bool           s_present   = false;
static const uint8_t  s_backlight = LCD_BACKLIGHT;

/* Non-blocking refresh state. */
static char     s_want[2][LCD_COLS];     /* latest requested content   */
static char     s_active[2][LCD_COLS];   /* snapshot being pushed out   */
static bool     s_dirty       = false;
static int      s_op          = -1;      /* -1 = idle, else 0..33       */
static uint32_t s_last_refresh_ms = 0;

/* ---------------------------------------------------------------------
 *  Low-level byte path.  No HAL_Delay: each I2C byte transfer (~150 us at
 *  100 kHz) is far longer than the HD44780's setup/EN/execute timing, so
 *  the transfers themselves provide the required spacing.
 * ---------------------------------------------------------------------*/
static bool lcd_write_raw(uint8_t value)
{
    if (!s_present) return false;
    return HAL_I2C_Master_Transmit(&hi2c_imu, s_addr_hal, &value, 1,
                                   LCD_I2C_TIMEOUT_MS) == HAL_OK;
}

static void lcd_write4(uint8_t nibble, uint8_t rs)
{
    uint8_t data = (uint8_t)((nibble & 0xF0u) | s_backlight | rs);
    lcd_write_raw((uint8_t)(data | LCD_EN));   /* EN high + data */
    lcd_write_raw(data);                       /* EN low -> latch */
}

static void lcd_send(uint8_t value, uint8_t rs)
{
    lcd_write4((uint8_t)(value & 0xF0u), rs);
    lcd_write4((uint8_t)((value << 4) & 0xF0u), rs);
}

static void pad16(char *dst, const char *src)
{
    int i = 0;
    while (src && src[i] && i < LCD_COLS) { dst[i] = src[i]; i++; }
    while (i < LCD_COLS) { dst[i] = ' '; i++; }
}

/* ---------------------------------------------------------------------
 *  Public API
 * ---------------------------------------------------------------------*/
static bool probe_addr(uint8_t addr_7bit)
{
    return HAL_I2C_IsDeviceReady(&hi2c_imu, (uint16_t)(addr_7bit << 1), 2,
                                 LCD_I2C_TIMEOUT_MS) == HAL_OK;
}

bool lcdBegin(void)
{
    if (probe_addr(LCD_I2C_ADDR_PRIMARY)) {
        s_addr_hal = (uint8_t)(LCD_I2C_ADDR_PRIMARY << 1);
    } else if (probe_addr(LCD_I2C_ADDR_FALLBACK)) {
        s_addr_hal = (uint8_t)(LCD_I2C_ADDR_FALLBACK << 1);
    } else {
        s_present = false;
        return false;
    }
    s_present = true;
    HAL_Delay(50);

    /* HD44780 4-bit power-on init — blocking, but runs once at boot.
     * The explicit delays here satisfy the wake-up timing. */
    lcd_write4(0x30u, 0); HAL_Delay(5);
    lcd_write4(0x30u, 0); HAL_Delay(5);
    lcd_write4(0x30u, 0); HAL_Delay(1);
    lcd_write4(0x20u, 0); HAL_Delay(1);
    lcd_send(0x28u, 0);            /* 4-bit, 2 lines, 5x8 font     */
    lcd_send(0x08u, 0);            /* display off                  */
    lcd_send(LCD_CMD_CLEAR, 0); HAL_Delay(2);
    lcd_send(0x06u, 0);            /* entry mode: increment        */
    lcd_send(0x0Cu, 0);            /* display on, cursor off       */

    memset(s_want,   ' ', sizeof s_want);
    memset(s_active, ' ', sizeof s_active);
    s_dirty = true;
    s_op    = -1;
    s_last_refresh_ms = 0;
    return true;
}

bool lcdIsPresent(void) { return s_present; }

void lcdSetLines(const char *line0, const char *line1)
{
    char n0[LCD_COLS], n1[LCD_COLS];
    pad16(n0, line0);
    pad16(n1, line1);
    if (memcmp(n0, s_want[0], LCD_COLS) != 0 ||
        memcmp(n1, s_want[1], LCD_COLS) != 0) {
        memcpy(s_want[0], n0, LCD_COLS);
        memcpy(s_want[1], n1, LCD_COLS);
        s_dirty = true;
    }
}

void lcdService(void)
{
    if (!s_present) return;

    if (s_op < 0) {
        /* Idle: start a fresh repaint only if content changed AND we've
         * waited out the rate limit. */
        if (!s_dirty) return;
        uint32_t now = HAL_GetTick();
        if ((uint32_t)(now - s_last_refresh_ms) < LCD_MIN_REFRESH_MS) return;
        s_last_refresh_ms = now;
        s_dirty = false;
        memcpy(s_active, s_want, sizeof s_active);
        s_op = 0;
    }

    /* Emit exactly one HD44780 byte this pass.
     * op 0 = move to line 0; ops 1..16 = line-0 chars; op 17 = move to
     * line 1; ops 18..33 = line-1 chars. */
    if (s_op == 0) {
        lcd_send(0x80u, 0);                                  /* DDRAM -> line 0 */
    } else if (s_op <= LCD_COLS) {
        lcd_send((uint8_t)s_active[0][s_op - 1], LCD_RS);
    } else if (s_op == LCD_COLS + 1) {
        lcd_send(0xC0u, 0);                                  /* DDRAM -> line 1 */
    } else {
        lcd_send((uint8_t)s_active[1][s_op - (LCD_COLS + 2)], LCD_RS);
    }

    if (++s_op > LCD_OP_DONE) s_op = -1;                     /* refresh complete */
}