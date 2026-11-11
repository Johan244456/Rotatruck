#ifndef BUTTON_H
#define BUTTON_H

/* =====================================================================
 *  button.h — Nucleo B1 USER button (PC13).
 *
 *  On-board Nucleo B1 USER button (PC13), polled and debounced. Each press
 *  toggles the power-assist on/off (with gravity + friction comp) - the same
 *  action as the external D8 button.
 * =====================================================================*/

#ifdef __cplusplus
extern "C" {
#endif

void buttonSetup(void);
void buttonPoll(void);

#ifdef __cplusplus
}
#endif

#endif /* BUTTON_H */