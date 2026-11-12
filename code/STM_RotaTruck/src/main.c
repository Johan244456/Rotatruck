/*
 *  STM RotaTruck — ODrive S1 branch
 *  Power-assisted hand-truck firmware.  Motor control is delegated to
 *  two ODrive S1 boards over a shared CAN bus.
 *
 *  Target : Nucleo-G491RE (STM32G491RET6 @ 170 MHz)
 *  CAN    : CAN2 on PB12 (RX) / PB13 (TX), 250 kbit/s, classic frames
 *           Requires an external transceiver (e.g. SN65HVD230) — see
 *           docs/02_branch/ODRIVE_INTEGRATION.md for the wiring diagram.
 *  Sensors: ISM330DHCX IMU on I2C1 (PB8/PB9),
 *           2 x HX711 strain-gauge ADCs for automatic payload (loadcell.c).
 *  UI     : 16x2 LCD on same I2C1 bus; payload +/- buttons on PB6/PC7
 *           (manual fallback), assist toggle on PA9 (D8) and the on-board
 *           USER button PC13.
 *  Safety : handle (dead-man) switch on PE7 - released = wheels ramp
 *           down to 0 and are held (deadman.c, assist_control.c).
 *  Debug  : USART2 via the ST-Link VCP, 115200 8N1.
 */

#include "stm32f4xx_hal.h"
#include "board.h"
#include "motor_control.h"
#include "command.h"
#include "imu.h"
#include "assist_control.h"
#include "observer.h"
#include "admittance.h"
#include "autoseq.h"
#include "button.h"
#include "display.h"
#include "mode_input.h"
#include "loadcell.h"
#include "payload.h"
#include "deadman.h"

void SystemClock_Config(void);   /* clock_f4.c: 168 MHz, APB1 42 MHz (CAN timing) */
static void GPIO_Clocks_Enable(void);

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    GPIO_Clocks_Enable();

    /* commandSetup() brings up USART2 first so subsequent uart_print
     * calls from the other init paths actually show up on the console. */
    commandSetup();

    /* ODrive CAN bus — non-fatal if HAL init fails (uart_print message
     * inside motorControlSetup will say so).  ODrives may not be
     * powered yet; heartbeat detection happens lazily. */
    motorControlSetup();

    /* IMU is optional for this branch — present = full pose state
     * machine; absent = "dev mode" where motor commands are gated only
     * by the user's explicit `E 1` (enable). */
    if (imuBegin()) {
        uart_printf("IMU: using %s for pose + observer.\r\n", imuGetSourceName());
    } else {
        uart_print(IMU_ENABLE ? "IMU: none detected (check PB9/SDA, PB8/SCL, 3V, GND).\r\n"
                              : "IMU: DISABLED in board.h (IMU_ENABLE 0) - no pose control, handle switch only.\r\n");
    }
    imuScanBus();           /* list everything on I2C1 (IMU swap bring-up) */

    /* Status LCD — non-blocking, serviced from the main loop. */
    uart_print(displayInit()
               ? (LCD_I2C_SEPARATE_BUS ? "LCD: 16x2 I2C display detected (own bus I2C3, SCL PA8 / SDA PC9).\r\n"
                                       : "LCD: 16x2 I2C display detected (I2C1, shared with the IMU).\r\n")
               : (LCD_I2C_SEPARATE_BUS ? "LCD: not detected (own bus I2C3: SCL PA8, SDA PC9, VCC 5V, GND; address 0x27/0x3F).\r\n"
                                       : "LCD: not detected (check I2C address/wiring on PB9/PB8).\r\n"));

    /* Handle (dead-man) safety switch on PE7 - before the pose machine, which
     * ramps the wheels to a stop whenever it reads released. */
    deadmanSetup();

    assistSetup();

    /* Momentum-disturbance observer (reads ODrive Iq/velocity + IMU and
     * estimates the operator's push/twist force).  Read-only — commands no
     * motor torque on its own.  Reads the IMU once per loop pass and
     * rate-limits its own force estimate to OBSERVER_RATE_HZ internally;
     * the pose detector reuses this cached read. */
    observerInit();

    /* Power-assist torque loop — opt-in, disabled until `AS 1`. */
    admittanceInit();
    autoseqInit();          /* autonomous arm-and-calibrate (R A / R M)    */
    buttonSetup();          /* DISC1 blue USER button (PA0) toggles assist on/off */

    /* Front-panel buttons: payload +/- (manual fallback) and D8 assist toggle. */
    modeInputSetup();

    /* Automatic payload detection: 2 x HX711 strain gauges + IMU pose gating.
     * Uses the shared IMU layer, so after imuBegin(). */
    loadcellInit();

    /* Assist payload source: load detection latched on STANDING -> DRIVE
     * (when the assist was switched on at STANDING), else manual buttons. */
    payloadInit();

    uart_print("RotaTruck ODrive-S1 firmware online. Type 'H' for help.\r\n");

    while (1) {
        commandPoll();
        buttonPoll();       /* B1 press toggles assist enabled / disabled    */
        observerUpdate();   /* refreshes IMU + force estimate (self-throttled to ~100 Hz) */
        loadcellUpdate();   /* HX711s when ready (10 Hz) + load pose, non-blocking */
        deadmanPoll();      /* handle safety switch (PE7), debounced         */
        assistUpdate();     /* pose detect + brake/release / release-stop ramp */
        autoseqUpdate();    /* autonomous self-calibrate + arm on drive-entry */
        modeInputUpdate();  /* payload +/- buttons, D8 assist toggle       */
        payloadUpdate();    /* latch load on STANDING -> DRIVE, -> observer */
        admittanceStep();   /* power-assist torque loop (gated on pose)    */
        displayUpdate();    /* status LCD — non-blocking, one byte/pass     */
    }
}

static void GPIO_Clocks_Enable(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
}



/* Unrecoverable fault: kills interrupts and spins forever. No LED/UART
 * output - if the board seems dead at boot, suspect this. */
void Error_Handler(void)
{
    __disable_irq();
    while (1) { }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    (void)file; (void)line;
}
#endif
