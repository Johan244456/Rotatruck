#include "lcd_i2c.h"
#include "board.h"
#include "stm32f4xx_hal.h"

#include <stdio.h>
#include <string.h>

#if LCD_I2C_SEPARATE_BUS
/* Own bus (board.h: I2C3 on PA8 SCL / PC9 SDA); MSP in stm32f4xx_hal_msp.c. */
I2C_HandleTypeDef hi2c_lcd;
#define LCD_BUS (&hi2c_lcd)

/* Free a slave left holding SDA low (e.g. reset mid-transfer): up to 9 SCL
 * pulses with the pins as open-drain GPIO, then a STOP.  The pins are put
 * back on the I2C alternate function by HAL_I2C_Init's MspInit. */
static void lcd_bus_clear(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    GPIO_InitTypeDef g = {0};
    g.Mode  = GPIO_MODE_OUTPUT_OD;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    g.Pin = LCD_I2C_SCL_PIN; HAL_GPIO_Init(LCD_I2C_SCL_PORT, &g);
    g.Pin = LCD_I2C_SDA_PIN; HAL_GPIO_Init(LCD_I2C_SDA_PORT, &g);
    HAL_GPIO_WritePin(LCD_I2C_SDA_PORT, LCD_I2C_SDA_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LCD_I2C_SCL_PORT, LCD_I2C_SCL_PIN, GPIO_PIN_SET);
    for (int i = 0; i < 9 && HAL_GPIO_ReadPin(LCD_I2C_SDA_PORT, LCD_I2C_SDA_PIN) == GPIO_PIN_RESET; i++) {
        HAL_GPIO_WritePin(LCD_I2C_SCL_PORT, LCD_I2C_SCL_PIN, GPIO_PIN_RESET); HAL_Delay(1);
        HAL_GPIO_WritePin(LCD_I2C_SCL_PORT, LCD_I2C_SCL_PIN, GPIO_PIN_SET);   HAL_Delay(1);
    }
    HAL_GPIO_WritePin(LCD_I2C_SDA_PORT, LCD_I2C_SDA_PIN, GPIO_PIN_RESET); HAL_Delay(1);  /* STOP */
    HAL_GPIO_WritePin(LCD_I2C_SDA_PORT, LCD_I2C_SDA_PIN, GPIO_PIN_SET);   HAL_Delay(1);
}

/* STM32F4 errata (I2C analog filter): the BUSY flag can latch with both lines
 * high, and every transfer then fails at once with HAL_BUSY.  ST workaround:
 * peripheral off, walk SDA/SCL low and high as GPIO, back to AF, SWRST. */
static void lcd_bus_unstick(void)
{
    hi2c_lcd.Instance->CR1 &= ~I2C_CR1_PE;
    GPIO_InitTypeDef g = {0};
    g.Mode  = GPIO_MODE_OUTPUT_OD;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    g.Pin = LCD_I2C_SCL_PIN; HAL_GPIO_Init(LCD_I2C_SCL_PORT, &g);
    g.Pin = LCD_I2C_SDA_PIN; HAL_GPIO_Init(LCD_I2C_SDA_PORT, &g);
    HAL_GPIO_WritePin(LCD_I2C_SCL_PORT, LCD_I2C_SCL_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LCD_I2C_SDA_PORT, LCD_I2C_SDA_PIN, GPIO_PIN_SET);   HAL_Delay(1);
    HAL_GPIO_WritePin(LCD_I2C_SDA_PORT, LCD_I2C_SDA_PIN, GPIO_PIN_RESET); HAL_Delay(1);
    HAL_GPIO_WritePin(LCD_I2C_SCL_PORT, LCD_I2C_SCL_PIN, GPIO_PIN_RESET); HAL_Delay(1);
    HAL_GPIO_WritePin(LCD_I2C_SCL_PORT, LCD_I2C_SCL_PIN, GPIO_PIN_SET);   HAL_Delay(1);
    HAL_GPIO_WritePin(LCD_I2C_SDA_PORT, LCD_I2C_SDA_PIN, GPIO_PIN_SET);   HAL_Delay(1);
    g.Mode      = GPIO_MODE_AF_OD;
    g.Speed     = GPIO_SPEED_FREQ_HIGH;
    g.Alternate = LCD_I2C_GPIO_AF;
    g.Pin = LCD_I2C_SCL_PIN; HAL_GPIO_Init(LCD_I2C_SCL_PORT, &g);
    g.Pin = LCD_I2C_SDA_PIN; HAL_GPIO_Init(LCD_I2C_SDA_PORT, &g);
    hi2c_lcd.Instance->CR1 |= I2C_CR1_SWRST;
    hi2c_lcd.Instance->CR1 &= ~I2C_CR1_SWRST;
}

static bool lcd_bus_busy(void) { return (hi2c_lcd.Instance->SR2 & I2C_SR2_BUSY) != 0; }

static bool lcd_bus_init(void)
{
    lcd_bus_clear();
    if (hi2c_lcd.State != HAL_I2C_STATE_RESET) (void)HAL_I2C_DeInit(&hi2c_lcd);
    hi2c_lcd.Instance             = LCD_I2C_INSTANCE;
    hi2c_lcd.Init.ClockSpeed      = LCD_I2C_CLOCK_HZ;
    hi2c_lcd.Init.DutyCycle       = I2C_DUTYCYCLE_2;
    hi2c_lcd.Init.OwnAddress1     = 0;
    hi2c_lcd.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    hi2c_lcd.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c_lcd.Init.OwnAddress2     = 0;
    hi2c_lcd.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c_lcd.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c_lcd) != HAL_OK) return false;
    if (lcd_bus_busy()) {                       /* errata: BUSY latched -> unstick, re-init */
        lcd_bus_unstick();
        (void)HAL_I2C_DeInit(&hi2c_lcd);
        if (HAL_I2C_Init(&hi2c_lcd) != HAL_OK) return false;
    }
    return !lcd_bus_busy();
}
#else
extern I2C_HandleTypeDef hi2c_imu;   /* LCD shares I2C1 with the ISM330DHCX */
#define LCD_BUS (&hi2c_imu)
#endif

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
    return HAL_I2C_Master_Transmit(LCD_BUS, s_addr_hal, &value, 1,
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
    return HAL_I2C_IsDeviceReady(LCD_BUS, (uint16_t)(addr_7bit << 1), 2,
                                 LCD_I2C_TIMEOUT_MS) == HAL_OK;
}

bool lcdBegin(void)
{
#if LCD_I2C_SEPARATE_BUS
    if (!lcd_bus_init()) { s_present = false; return false; }
#endif
    s_present = false;
    uint8_t found = 0;
    if (probe_addr(LCD_I2C_ADDR_PRIMARY))       found = LCD_I2C_ADDR_PRIMARY;
    else if (probe_addr(LCD_I2C_ADDR_FALLBACK)) found = LCD_I2C_ADDR_FALLBACK;
    else {
        /* Any PCF8574 (0x20-0x27) / PCF8574A (0x38-0x3F) address - backpacks
         * with the A0-A2 jumpers bridged sit elsewhere in these ranges. */
        for (uint8_t a = 0x20u; a <= 0x3Fu && !found; a++) {
            if (a == 0x28u) a = 0x38u;
            if (probe_addr(a)) found = a;
        }
    }
    if (!found) return false;
    s_addr_hal = (uint8_t)(found << 1);
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

uint8_t lcdAddress(void) { return s_present ? (uint8_t)(s_addr_hal >> 1) : 0u; }

#if LCD_I2C_SEPARATE_BUS
/* ---- software (bit-banged) probe, independent of the I2C3 peripheral ----
 * Run twice: as wired (SCL PA8, SDA PC9) and with the two lines swapped. */
static GPIO_TypeDef *bb_scl_port, *bb_sda_port;
static uint16_t      bb_scl_pin,   bb_sda_pin;
static void bb_delay(void) { for (volatile int i = 0; i < 300; i++) { } }   /* ~5 us at 168 MHz */
static void bb_scl(int v) { HAL_GPIO_WritePin(bb_scl_port, bb_scl_pin, v ? GPIO_PIN_SET : GPIO_PIN_RESET); bb_delay(); }
static void bb_sda(int v) { HAL_GPIO_WritePin(bb_sda_port, bb_sda_pin, v ? GPIO_PIN_SET : GPIO_PIN_RESET); bb_delay(); }
static int  bb_sda_in(void) { return HAL_GPIO_ReadPin(bb_sda_port, bb_sda_pin) == GPIO_PIN_SET; }

static bool bb_probe(uint8_t addr7)
{
    bb_sda(1); bb_scl(1);
    bb_sda(0); bb_scl(0);                              /* START */
    uint8_t byte = (uint8_t)(addr7 << 1);              /* write */
    for (int i = 7; i >= 0; i--) { bb_sda((byte >> i) & 1); bb_scl(1); bb_scl(0); }
    bb_sda(1); bb_scl(1);                              /* release SDA, 9th clock */
    bool ack = !bb_sda_in();
    bb_scl(0);
    bb_sda(0); bb_scl(1); bb_sda(1);                   /* STOP */
    return ack;
}

static int bb_scan_once(void (*print)(const char *), const char *label)
{
    char b[96];
    int n = 0;
    for (uint8_t a = 0x08u; a < 0x78u; a++) {
        if (bb_probe(a)) { snprintf(b, sizeof b, "    0x%02X answers (%s)\r\n", a, label); print(b); n++; }
    }
    snprintf(b, sizeof b, "    %s: %d device(s)\r\n", label, n);
    print(b);
    return n;
}

static void bb_scan(void (*print)(const char *))
{
    hi2c_lcd.Instance->CR1 &= ~I2C_CR1_PE;
    GPIO_InitTypeDef g = {0};
    g.Mode  = GPIO_MODE_OUTPUT_OD;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    g.Pin = LCD_I2C_SCL_PIN; HAL_GPIO_Init(LCD_I2C_SCL_PORT, &g);
    g.Pin = LCD_I2C_SDA_PIN; HAL_GPIO_Init(LCD_I2C_SDA_PORT, &g);
    /* A line that is not idle-high (held low, or nothing connected to pull it up)
     * makes every address look like it answers - report it and skip the scan. */
    {
        /* inputs with the internal PULL-DOWN: only the backpack's pull-up can make them read high */
        GPIO_InitTypeDef t = {0};
        t.Mode = GPIO_MODE_INPUT; t.Pull = GPIO_PULLDOWN;
        t.Pin = LCD_I2C_SCL_PIN; HAL_GPIO_Init(LCD_I2C_SCL_PORT, &t);
        t.Pin = LCD_I2C_SDA_PIN; HAL_GPIO_Init(LCD_I2C_SDA_PORT, &t);
        HAL_Delay(2);
        const int scl = HAL_GPIO_ReadPin(LCD_I2C_SCL_PORT, LCD_I2C_SCL_PIN) == GPIO_PIN_SET;
        const int sda = HAL_GPIO_ReadPin(LCD_I2C_SDA_PORT, LCD_I2C_SDA_PIN) == GPIO_PIN_SET;
        g.Pin = LCD_I2C_SCL_PIN; HAL_GPIO_Init(LCD_I2C_SCL_PORT, &g);     /* back to open-drain, no pull */
        g.Pin = LCD_I2C_SDA_PIN; HAL_GPIO_Init(LCD_I2C_SDA_PORT, &g);
        if (!scl || !sda) {
            if (!scl) print("  -> SCL (PA8): no pull-up from the backpack - wire not connected to the "
                            "backpack's SCL pin, or shorted to GND.\r\n");
            if (!sda) print("  -> SDA (PC9): no pull-up from the backpack - wire not connected to the "
                            "backpack's SDA pin, or shorted to GND (e.g. on its GND pin).\r\n");
            (void)lcd_bus_init();
            return;
        }
    }
    print("  software scan (pins driven directly, no I2C peripheral):\r\n");
    bb_scl_port = LCD_I2C_SCL_PORT; bb_scl_pin = LCD_I2C_SCL_PIN;
    bb_sda_port = LCD_I2C_SDA_PORT; bb_sda_pin = LCD_I2C_SDA_PIN;
    int n = bb_scan_once(print, "as wired: SCL PA8, SDA PC9");
    bb_scl_port = LCD_I2C_SDA_PORT; bb_scl_pin = LCD_I2C_SDA_PIN;
    bb_sda_port = LCD_I2C_SCL_PORT; bb_sda_pin = LCD_I2C_SCL_PIN;
    int m = bb_scan_once(print, "SWAPPED: SCL PC9, SDA PA8");
    if (!n && m) print("  -> the LCD answers with SDA/SCL SWAPPED: swap the two wires.\r\n");
    if (!n && !m) print("  -> nothing answers either way: check LCD VCC 5V + GND, and that the\r\n"
                        "     wires really go to PA8 / PC9 (and the backpack is plugged onto the LCD).\r\n");
    (void)lcd_bus_init();                               /* back to the I2C3 peripheral */
}
#endif

/* Console `LCD`: bus state + every address that answers, via print(). */
void lcdDiagnose(void (*print)(const char *))
{
    char b[160];
#if LCD_I2C_SEPARATE_BUS
    if (hi2c_lcd.State == HAL_I2C_STATE_RESET && !lcd_bus_init()) {
        print("LCD bus: I2C3 init failed or BUSY stuck.\r\n");
    }
    snprintf(b, sizeof b, "LCD bus I2C3 (SCL PA8 = %d, SDA PC9 = %d, BUSY = %d):\r\n",
             (int)HAL_GPIO_ReadPin(LCD_I2C_SCL_PORT, LCD_I2C_SCL_PIN),
             (int)HAL_GPIO_ReadPin(LCD_I2C_SDA_PORT, LCD_I2C_SDA_PIN), (int)lcd_bus_busy());
    print(b);
    if (lcd_bus_busy()) { lcd_bus_unstick(); print("  BUSY was stuck - bus reset (errata workaround).\r\n"); }
#else
    print("LCD bus: shared I2C1 (see I2C):\r\n");
#endif
    int n = 0;
    for (uint8_t a = 0x08u; a < 0x78u; a++) {
        if (probe_addr(a)) {
            snprintf(b, sizeof b, "  0x%02X answers%s\r\n", a,
                     ((a >= 0x20u && a <= 0x27u) || (a >= 0x38u && a <= 0x3Fu)) ? "  (PCF8574 LCD backpack range)" : "");
            print(b);
            n++;
        }
    }
    snprintf(b, sizeof b, "  %d device(s) found by I2C3 (BUSY now %d).%s\r\n", n,
#if LCD_I2C_SEPARATE_BUS
             (int)lcd_bus_busy(),
#else
             0,
#endif
             n ? "" : "");
    print(b);
#if LCD_I2C_SEPARATE_BUS
    if (!n) bb_scan(print);
#endif
}

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