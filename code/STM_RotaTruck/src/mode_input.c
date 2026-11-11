#include "mode_input.h"
#include "admittance.h"
#include "board.h"
#include "payload.h"
#include "stm32g4xx_hal.h"

#include <stdbool.h>

/*
 *  Physical front-panel buttons, POLLED with a per-button debounce state
 *  machine:
 *    PB6 (D10) -> payload +5 kg  (manual - the fallback when load detection
 *    PC7 (D9)  -> payload -5 kg   is not valid; a press also overrides a
 *                                 latched value, see payload.h)
 *    PA9 (D8)  -> toggle ASSIST on / off  (same as the on-board B1).  Switch
 *                 it ON at STANDING so the detected load is latched when you
 *                 tip back.
 *
 *  Polling (not EXTI) is deliberate.  On long breadboard wiring next to the
 *  motors / CAN, the *edges* are noisy and the contacts bounce on BOTH press
 *  and release, so an edge-interrupt fires several times per click (press +
 *  release + phantom edges).  Sampling the *level* instead is immune to all
 *  of that: a button only changes state after its level has been stable for
 *  the debounce time, and the action fires ONCE, on the released->pressed
 *  transition of that debounced state.  The main loop polls far faster than
 *  any human press.  Active-high wiring (button -> 3.3 V + pull-down); see
 *  UI_MODE_BUTTON_ACTIVE_LOW in board.h.
 */

typedef void (*btn_action_t)(void);

typedef struct {
    GPIO_TypeDef  *port;
    uint16_t       pin;
    btn_action_t   on_press;
    GPIO_PinState  raw_last;        /* last raw sample            */
    GPIO_PinState  stable;          /* debounced level            */
    uint32_t       last_change_ms;  /* when the raw level changed */
} Button;

/* --- Button actions ------------------------------------------------- */
static void act_payload_inc(void) { payloadAdjustManual(+UI_PAYLOAD_STEP_KG); }
static void act_payload_dec(void) { payloadAdjustManual(-UI_PAYLOAD_STEP_KG); }

/* Toggle the power-assist on / off (plus gravity + friction comp) — the
 * comparison test: push the loaded cart with assist ON, then OFF, and feel
 * the difference.  (Same action as the on-board B1 button.) */
static void act_assist_toggle(void)
{
    admittanceToggleWithComp();
}

static Button s_buttons[] = {
    {UI_PAYLOAD_INC_BUTTON_PORT, UI_PAYLOAD_INC_BUTTON_PIN, act_payload_inc,
     GPIO_PIN_RESET, GPIO_PIN_RESET, 0},
    {UI_PAYLOAD_DEC_BUTTON_PORT, UI_PAYLOAD_DEC_BUTTON_PIN, act_payload_dec,
     GPIO_PIN_RESET, GPIO_PIN_RESET, 0},
    {UI_MODE_TOGGLE_BUTTON_PORT, UI_MODE_TOGGLE_BUTTON_PIN, act_assist_toggle,
     GPIO_PIN_RESET, GPIO_PIN_RESET, 0},
};
#define BUTTON_COUNT (sizeof s_buttons / sizeof s_buttons[0])

static bool button_pressed(GPIO_PinState state)
{
#if UI_MODE_BUTTON_ACTIVE_LOW
    return state == GPIO_PIN_RESET;
#else
    return state == GPIO_PIN_SET;
#endif
}

static void setup_button_gpio(Button *button)
{
    GPIO_InitTypeDef g = {0};
    g.Pin   = button->pin;
    g.Mode  = GPIO_MODE_INPUT;        /* plain input — polled, no EXTI */
#if UI_MODE_BUTTON_ACTIVE_LOW
    g.Pull  = GPIO_PULLUP;
#else
    g.Pull  = GPIO_PULLDOWN;
#endif
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(button->port, &g);

    button->raw_last       = HAL_GPIO_ReadPin(button->port, button->pin);
    button->stable         = button->raw_last;
    button->last_change_ms = HAL_GetTick();
}

void modeInputSetup(void)
{
    for (unsigned i = 0; i < BUTTON_COUNT; i++) {
        setup_button_gpio(&s_buttons[i]);
    }
}

void modeInputUpdate(void)
{
    uint32_t now = HAL_GetTick();
    for (unsigned i = 0; i < BUTTON_COUNT; i++) {
        Button *b = &s_buttons[i];
        GPIO_PinState raw = HAL_GPIO_ReadPin(b->port, b->pin);

        if (raw != b->raw_last) {
            /* Level changed — (re)start the debounce window.  Bounce and
             * phantom glitches keep landing here and never settle. */
            b->raw_last       = raw;
            b->last_change_ms = now;
        } else if (raw != b->stable &&
                   (uint32_t)(now - b->last_change_ms) >= UI_MODE_BUTTON_DEBOUNCE_MS) {
            /* Level held steady long enough -> commit the new debounced
             * state.  Fire ONLY on released -> pressed (so the release does
             * nothing, and one press = one action). */
            GPIO_PinState prev = b->stable;
            b->stable = raw;
            if (!button_pressed(prev) && button_pressed(raw) && b->on_press) {
                b->on_press();
            }
        }
    }
}