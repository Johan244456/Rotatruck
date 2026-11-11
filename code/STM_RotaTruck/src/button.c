/*
 *  button.c — Nucleo B1 USER button on PC13, debounced, polled.
 *
 *  On every press (release->press edge) the power-assist toggles:
 *  ENABLED <-> DISABLED, together with the observer's gravity + friction
 *  compensation (same as the external D8 button).  Lets you feel the loaded
 *  cart with assist on vs off without touching the keyboard.
 *
 *  On the NUCLEO-G491RE the B1 button is ACTIVE-HIGH: the official BSP
 *  configures PC13 with an internal PULL-DOWN, so released reads LOW and
 *  pressing drives PC13 to VDD (HIGH).  (Earlier active-low/pull-up gave a
 *  constant HIGH — the `1 -> 1` symptom.)  If a board reads inverted, flip
 *  USER_BTN_PULL + USER_BTN_PRESSED together.
 */

#include "button.h"
#include "admittance.h"
#include "stm32f4xx_hal.h"

/* F407 DISC1: blue USER button B1 on PA0, active-high (pressed = VDD), with
 * its own pull-down + RC on the board (was PC13 on the Nucleo). */
#define USER_BTN_PORT     GPIOA
#define USER_BTN_PIN      GPIO_PIN_0
#define USER_BTN_PULL     GPIO_NOPULL       /* board has R + C on PA0 */
#define USER_BTN_PRESSED  GPIO_PIN_SET      /* active-high (pressed = VDD) */
#define BTN_DEBOUNCE_MS   40U

static GPIO_PinState s_raw_last    = GPIO_PIN_RESET;  /* released = low */
static GPIO_PinState s_stable      = GPIO_PIN_RESET;
static uint32_t      s_changed_ms  = 0;

void buttonSetup(void)
{
    /* GPIOA clock is already enabled in main's GPIO_Clocks_Enable(). */
    GPIO_InitTypeDef g = {0};
    g.Pin  = USER_BTN_PIN;
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = USER_BTN_PULL;
    HAL_GPIO_Init(USER_BTN_PORT, &g);

    s_raw_last   = HAL_GPIO_ReadPin(USER_BTN_PORT, USER_BTN_PIN);
    s_stable     = s_raw_last;
    s_changed_ms = HAL_GetTick();
}

void buttonPoll(void)
{
    GPIO_PinState raw = HAL_GPIO_ReadPin(USER_BTN_PORT, USER_BTN_PIN);
    uint32_t now = HAL_GetTick();

    if (raw != s_raw_last) {
        s_raw_last   = raw;
        s_changed_ms = now;                 /* restart the debounce window */
        return;
    }
    if ((uint32_t)(now - s_changed_ms) < BTN_DEBOUNCE_MS) return;

    if (raw != s_stable) {
        s_stable = raw;
        if (raw == USER_BTN_PRESSED) {      /* committed press edge */
            admittanceToggleWithComp();
        }
    }
}