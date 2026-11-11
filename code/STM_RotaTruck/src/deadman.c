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

/* gestures (board.h DEADMAN_HOLD_MS / CLICK_*) */
static uint32_t s_press_ms;                  /* debounced press time            */
static uint32_t s_release_ms;                /* debounced release time          */
static uint8_t  s_clicks;                    /* quick clicks in the current run */
static uint32_t s_first_click_ms;            /* press time of the run's 1st click */
static bool     s_triple;                    /* triple click pending            */
static uint8_t  s_short_run;                 /* 1-2 quick clicks that ended (hint) */
static uint32_t s_triple_first_ms;

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
    /* A click run ends when no further press follows within the gap.  Checked
     * first, every pass: the early returns below fire whenever the switch is
     * idle, which is exactly when a single click has to be recognised. */
    GPIO_PinState raw = HAL_GPIO_ReadPin(DEADMAN_PORT, DEADMAN_PIN);
    if (s_clicks && !s_pressed && !level_is_pressed(raw) &&     /* not a 2nd click starting */
        (uint32_t)(now - s_release_ms) > DEADMAN_CLICK_GAP_MS) {
        s_short_run = s_clicks;               /* 1 or 2 quick clicks, then nothing */
        s_clicks = 0;
    }
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
        /* time of the real edge = when the level changed, not when it was accepted */
        uint32_t edge = s_change_ms;
        if (s_pressed) {
            if (s_clicks && (uint32_t)(edge - s_release_ms) > DEADMAN_CLICK_GAP_MS) s_clicks = 0;
            if (s_clicks == 0) s_first_click_ms = edge;
            s_press_ms = edge;
        } else {
            s_release_ms = edge;
            if ((uint32_t)(edge - s_press_ms) <= DEADMAN_CLICK_MAX_MS) {
                if (++s_clicks >= 3) {
                    s_triple = true; s_triple_first_ms = s_first_click_ms; s_clicks = 0;
                }
            } else {
                s_clicks = 0;                 /* a hold ends any click run */
            }
        }
        uart_print(s_pressed ? "SAFETY: handle switch PRESSED - assist allowed.\r\n"
                             : "SAFETY: handle switch RELEASED - stopping gradually.\r\n");
    }
#endif
}

bool deadmanHeld(void)
{
    return s_pressed && (uint32_t)(HAL_GetTick() - s_press_ms) >= DEADMAN_HOLD_MS;
}

uint32_t deadmanPressStartMs(void) { return s_press_ms; }

uint8_t deadmanTakeShortClicks(void)
{
    uint8_t n = s_short_run;
    s_short_run = 0;
    return n;
}

bool deadmanTakeTripleClick(uint32_t *first_press_ms)
{
    if (!s_triple) return false;
    s_triple = false;
    if (first_press_ms) *first_press_ms = s_triple_first_ms;
    return true;
}

bool deadmanPressed(void) { return s_pressed; }
