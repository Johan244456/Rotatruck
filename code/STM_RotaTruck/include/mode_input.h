#ifndef MODE_INPUT_H
#define MODE_INPUT_H

#ifdef __cplusplus
extern "C" {
#endif

/* Polled front-panel buttons: PB6 (D10) +5 kg, PC7 (D9) -5 kg,
 * PA9 (D8) toggle power-assist on/off. Debounced in the main loop, no EXTI. */
void modeInputSetup(void);
void modeInputUpdate(void);

#ifdef __cplusplus
}
#endif

#endif /* MODE_INPUT_H */
