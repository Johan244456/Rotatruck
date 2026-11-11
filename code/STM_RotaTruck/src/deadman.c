/*
 *  deadman.c — handle safety switch on PE7.  See deadman.h.
 */

#include "deadman.h"
#include "board.h"
#include "command.h"
#include "stm32f4xx_hal.h"

static bool          s_pressed = false;      /* debounced state, starts RELEASED (safe) */
static GPIO_PinState s_raw_last;
static uint32_t      s_change_ms;

static bool level_is_pressed(GPIO_PinState level)
{
#if DEADMAN_ACTIVE_LOW
    return level == GPIO_PIN_RESET;
#else
    return level == GPIO_PIN_SET;
#endif
}

void deadmanSetup(void)
{
#if DEADMAN_ENABLE
    __HAL_RCC_GPIOE_CLK_ENABLE();   /* PE7 - main() only enables GPIOA..C */
    GPIO_InitTypeDef g = {0};
    g.Pin   = DEADMAN_PIN;
    g.Mode  = GPIO_MODE_INPUT;
    /* Pull towards RELEASED so an open circuit = stop (fail-safe). */
#if DEADMAN_ACTIVE_LOW
    g.Pull  = GPIO_PULLUP;
#else
    g.Pull  = GPIO_PULLDOWN;
#endif
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(DEADMAN_PORT, &g);
    HAL_Delay(5);       /* let the pull charge the wire before the first read */

    s_raw_last  = HAL_GPIO_ReadPin(DEADMAN_PORT, DEADMAN_PIN);
    s_change_ms = HAL_GetTick();
    s_pressed   = false;                      /* must be seen pressed first */
    uart_printf("SAFETY: handle switch on PE7, %s - %s now.\r\n",
                DEADMAN_ACTIVE_LOW ? "active-low" : "active-high",
                level_is_pressed(s_raw_last) ? "PRESSED" : "RELEASED");
#else
    s_pressed = true;
    uart_print("SAFETY: handle switch DISABLED in board.h (DEADMAN_ENABLE 0) - bench only!\r\n");
#endif
}

void deadmanPoll(void)
{
#if DEADMAN_ENABLE
    uint32_t now = HAL_GetTick();
    GPIO_PinState raw = HAL_GPIO_ReadPin(DEADMAN_PORT, DEADMAN_PIN);
    if (raw != s_raw_last) {
        s_raw_last  = raw;
        s_change_ms = now;
        return;
    }
    bool want = level_is_pressed(raw);
    if (want == s_pressed) return;
    /* Release is safety-critical: accept it after a short stable time.
     * Press needs longer so contact bounce can't re-enable the assist. */
    uint32_t need = want ? DEADMAN_PRESS_DEBOUNCE_MS : DEADMAN_RELEASE_DEBOUNCE_MS;
    if ((uint32_t)(now - s_change_ms) >= need) {
        s_pressed = want;
        uart_print(s_pressed ? "SAFETY: handle switch PRESSED - assist allowed.\r\n"
                             : "SAFETY: handle switch RELEASED - stopping gradually.\r\n");
    }
#endif
}

bool deadmanPressed(void) { return s_pressed; }
