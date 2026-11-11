/*
 *  imu_test_main.c — standalone IMU bring-up for the STM32F407G-DISC1.
 *
 *  Built ONLY by the [env:imu_test] PlatformIO environment (the full
 *  firmware env excludes this file).  Brings up the clock, USART2 and the
 *  ISM330DHCX on I2C1, scans the bus once, then prints one IMU line every
 *  100 ms (cart frame, same format as the main firmware's `I S` stream).
 *
 *  Output goes to BOTH:
 *    - RTT over the ST-Link (same mini-USB cable as flashing): run
 *      tools/rtt_monitor.sh.  Needed because the DISC1's ST-Link virtual
 *      COM port is NOT wired to the F407.
 *    - USART2 TX = PA2, RX = PA3, 115200 8N1, for a 3.3 V USB-serial
 *      adapter (adapter RX -> PA2, GND -> GND).
 *
 *  Handle (dead-man) switch: PE7 -> switch -> GND (deadman.c, board.h).
 *  Shown as handle=PRESSED/RELEASED on every line, and each debounced
 *  change is reported as a "SAFETY:" line.
 *
 *  IMU: SDA -> PB9, SCL -> PB8, 3V3, GND (board.h).  PB9 is also the
 *  on-board CS43L22 audio codec's SDA (with a 4.7k pull-up); its SCL is
 *  PB6, so it stays idle and does not disturb this bus.
 *
 *  ODrive CAN: CAN2, RX = PB12 (red), TX = PB13 (brown), 250 kbit/s, via
 *  the TJA transceiver powered from 5V (odrive_can.c, board.h).  Listen
 *  only - nothing is sent to the ODrives.  Each line ends with
 *  CAN L=<left state> R=<right state>  ("--" = no heartbeat yet), and
 *  heartbeat found/lost and bus-error changes get their own "CAN:" line.
 *  If the bus isn't idle at boot (transceiver off), init retries every 2 s.
 *
 *  Keys (typed in the RTT terminal, or adapter TX -> PA3):
 *    space = pause/resume stream,  s = I2C scan,  c = calibrate (hold still)
 */

#include "stm32f4xx_hal.h"
#include "board.h"
#include "command.h"
#include "imu.h"
#include "deadman.h"
#include "odrive_can.h"
#include "rtt.h"
#include <stdio.h>
#include <math.h>
#include <stdarg.h>
#include <string.h>

#define STREAM_PERIOD_MS 100U

static UART_HandleTypeDef huart_dbg;
static bool s_can_ok = false;

static void SystemClock_Config(void);

/* ---------------------------------------------------------------------
 *  uart_print / uart_printf  (imu.c prints through these; the full
 *  firmware gets them from command.c, which is not in this build)
 * -------------------------------------------------------------------*/
void uart_print(const char *s)
{
    size_t n = strlen(s);
    rttWrite(s, n);
    HAL_UART_Transmit(&huart_dbg, (uint8_t *)s, (uint16_t)n, 100);
}

void uart_printf(const char *fmt, ...)
{
    char buf[192];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);
    if (n <= 0) return;
    if (n >= (int)sizeof buf) n = (int)sizeof buf - 1;
    rttWrite(buf, (size_t)n);
    HAL_UART_Transmit(&huart_dbg, (uint8_t *)buf, (uint16_t)n, 100);
}

static void uart_init(void)
{
    huart_dbg.Instance          = DEBUG_UART_INSTANCE;
    huart_dbg.Init.BaudRate     = DEBUG_UART_BAUD;
    huart_dbg.Init.WordLength   = UART_WORDLENGTH_8B;
    huart_dbg.Init.StopBits     = UART_STOPBITS_1;
    huart_dbg.Init.Parity       = UART_PARITY_NONE;
    huart_dbg.Init.Mode         = UART_MODE_TX_RX;
    huart_dbg.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart_dbg.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart_dbg);
}

/* Non-blocking single-key read: RTT terminal first, then USART2 (polled). */
static int key_getc(void)
{
    int c = rttGetc();
    if (c >= 0) return c;
    if (__HAL_UART_GET_FLAG(&huart_dbg, UART_FLAG_ORE)) __HAL_UART_CLEAR_OREFLAG(&huart_dbg);
    if (!__HAL_UART_GET_FLAG(&huart_dbg, UART_FLAG_RXNE)) return -1;
    return (int)(huart_dbg.Instance->DR & 0xFFU);
}

/* ---------------------------------------------------------------------
 *  ODrive CAN (listen-only use of odrive_can.c)
 * -------------------------------------------------------------------*/
#define CAN_ALIVE_MS     500U
#define CAN_RETRY_MS    2000U

static const char *axis_state_name(odrive_axis_state_t st)
{
    switch (st) {
    case ODRIVE_AXIS_STATE_IDLE:                return "IDLE";
    case ODRIVE_AXIS_STATE_STARTUP_SEQUENCE:    return "STARTUP";
    case ODRIVE_AXIS_STATE_FULL_CALIBRATION_SEQUENCE:
    case ODRIVE_AXIS_STATE_MOTOR_CALIBRATION:
    case ODRIVE_AXIS_STATE_ENCODER_OFFSET_CALIBRATION:
                                                return "CALIB";
    case ODRIVE_AXIS_STATE_CLOSED_LOOP_CONTROL: return "CLOSED_LOOP";
    default:                                    return "OTHER";
    }
}

/* Short per-node status for the stream line. */
static const char *node_status(uint8_t node)
{
    const odrive_state_t *s = odriveGetState(node);
    if (!s || !s->seen)                   return "--";
    if (!odriveIsAlive(node, CAN_ALIVE_MS)) return "LOST";
    if (s->axis_error)                    return "ERR";
    return axis_state_name(s->axis_state);
}

static void can_start(bool verbose)
{
    s_can_ok = odriveCanInit();
    if (s_can_ok) {
        uart_print("CAN: CAN2 up on PB12(RX)/PB13(TX) at 250 kbit/s - "
                   "listening for ODrive heartbeats (node 0 = LEFT, 1 = RIGHT).\r\n");
    } else if (verbose) {
        uart_print("CAN: start failed - PB12 (RX) never saw an idle bus. "
                   "Check the transceiver's 5V/GND and the red RX wire. "
                   "Retrying every 2 s.\r\n");
    }
}

/* Retry init while down; report heartbeat found/lost and bus-error
 * level changes as separate lines. */
static void can_poll(void)
{
    static uint32_t last_try;
    static bool     alive[2];
    static int      level;          /* 0 ok, 1 warning, 2 passive, 3 bus-off */
    const uint8_t   nodes[2] = { ODRIVE_NODE_ID_LEFT, ODRIVE_NODE_ID_RIGHT };
    const char     *names[2] = { "LEFT", "RIGHT" };
    uint32_t        now = HAL_GetTick();

    if (!s_can_ok) {
        if ((uint32_t)(now - last_try) >= CAN_RETRY_MS) {
            last_try = now;
            can_start(false);
        }
        return;
    }

    for (int i = 0; i < 2; i++) {
        bool a = odriveIsAlive(nodes[i], CAN_ALIVE_MS);
        if (a != alive[i]) {
            alive[i] = a;
            const odrive_state_t *s = odriveGetState(nodes[i]);
            if (a) {
                uart_printf("CAN: ODrive node %u (%s) heartbeat OK - state=%s error=0x%08lX\r\n",
                            nodes[i], names[i], axis_state_name(s->axis_state),
                            (unsigned long)s->axis_error);
            } else {
                uart_printf("CAN: ODrive node %u (%s) heartbeat LOST\r\n",
                            nodes[i], names[i]);
            }
        }
    }

    uint8_t tec, rec; bool boff;
    odriveCanGetBusStatus(&tec, &rec, &boff);
    uint8_t worst = tec > rec ? tec : rec;
    int lv = boff ? 3 : worst >= 128 ? 2 : worst >= 96 ? 1 : 0;
    if (lv != level) {
        static const char *lvname[] = { "OK", "WARNING", "ERROR-PASSIVE", "BUS-OFF" };
        level = lv;
        uart_printf("CAN: bus %s (TEC=%u REC=%u)%s\r\n", lvname[lv], tec, rec,
                    lv ? " - check baud (250k), CANH/CANL, GND, termination" : "");
    }
}

static const char *can_suffix(void)
{
    static char buf[40];
    if (!s_can_ok) return "  CAN=off";
    snprintf(buf, sizeof buf, "  CAN L=%s R=%s",
             node_status(ODRIVE_NODE_ID_LEFT), node_status(ODRIVE_NODE_ID_RIGHT));
    return buf;
}

static void print_imu_line(void)
{
    if (!imuRead()) {
        uart_printf("IMU read failed (ISM330DHCX not responding)  handle=%s%s\r\n",
                    deadmanPressed() ? "PRESSED" : "RELEASED", can_suffix());
        return;
    }
    const float ax = imuGetAxG(), ay = imuGetAyG(), az = imuGetAzG();
    uart_printf("IMU a=[%+.3f %+.3f %+.3f]g |a|=%.3f  g=[%+7.2f %+7.2f %+7.2f]dps  pitch=%+6.1f  handle=%s%s\r\n",
                (double)ax, (double)ay, (double)az,
                (double)sqrtf(ax * ax + ay * ay + az * az),
                (double)imuGetGxDps(), (double)imuGetGyDps(), (double)imuGetGzDps(),
                (double)imuGetPitchDeg(),
                deadmanPressed() ? "PRESSED" : "RELEASED", can_suffix());
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    rttInit();
    uart_init();
    uart_print("\r\n=== RotaTruck F407 IMU test ===\r\n");

    if (imuBegin()) {
        uart_printf("IMU: %s OK.\r\n", imuGetSourceName());
    } else {
        uart_print("IMU: none detected (check PB9/SDA, PB8/SCL, 3V3, GND).\r\n");
    }
    imuScanBus();
    deadmanSetup();
    can_start(true);
    uart_print("Streaming at 10 Hz. Keys: space=pause  s=scan  c=calibrate\r\n");

    bool     streaming = true;
    uint32_t last = HAL_GetTick();

    while (1) {
        deadmanPoll();
        can_poll();
        switch (key_getc()) {
        case ' ': streaming = !streaming;
                  uart_print(streaming ? "-- resumed --\r\n" : "-- paused --\r\n"); break;
        case 's': imuScanBus(); break;
        case 'c': uart_print("Calibrating - keep still...\r\n");
                  imuCalibrate(200); uart_print("Done.\r\n"); break;
        default:  break;
        }

        uint32_t now = HAL_GetTick();
        if (streaming && (uint32_t)(now - last) >= STREAM_PERIOD_MS) {
            last = now;
            print_imu_line();
        }
    }
}

/* ---------------------------------------------------------------------
 *  168 MHz from the 8 MHz HSE crystal (X2 on the DISC1):
 *  8 / M8 * N336 / P2 = 168 MHz SYSCLK, Q7 = 48 MHz (USB).
 *  APB1 = 42 MHz (I2C1, USART2), APB2 = 84 MHz.
 * -------------------------------------------------------------------*/
static void SystemClock_Config(void)
{
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitTypeDef o = {0};
    o.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    o.HSEState       = RCC_HSE_ON;
    o.PLL.PLLState   = RCC_PLL_ON;
    o.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    o.PLL.PLLM       = 8;
    o.PLL.PLLN       = 336;
    o.PLL.PLLP       = RCC_PLLP_DIV2;
    o.PLL.PLLQ       = 7;
    if (HAL_RCC_OscConfig(&o) != HAL_OK) { while (1) { } }

    RCC_ClkInitTypeDef c = {0};
    c.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    c.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    c.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    c.APB1CLKDivider = RCC_HCLK_DIV4;
    c.APB2CLKDivider = RCC_HCLK_DIV2;
    if (HAL_RCC_ClockConfig(&c, FLASH_LATENCY_5) != HAL_OK) { while (1) { } }
}

/* ---------------------------------------------------------------------
 *  HAL MSP + interrupt glue (F4)
 * -------------------------------------------------------------------*/
void HAL_MspInit(void)
{
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();
}

void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance != IMU_I2C_INSTANCE) return;
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef g = {0};
    g.Mode      = GPIO_MODE_AF_OD;
    g.Pull      = GPIO_PULLUP;
    g.Speed     = GPIO_SPEED_FREQ_HIGH;
    g.Alternate = IMU_I2C_GPIO_AF;
    g.Pin = IMU_I2C_SCL_PIN; HAL_GPIO_Init(IMU_I2C_SCL_PORT, &g);
    g.Pin = IMU_I2C_SDA_PIN; HAL_GPIO_Init(IMU_I2C_SDA_PORT, &g);

    /* Enable the clock only after the pins are on AF, then pulse the
     * peripheral reset - avoids the F4 I2C "BUSY stuck after init" erratum. */
    __HAL_RCC_I2C1_CLK_ENABLE();
    __HAL_RCC_I2C1_FORCE_RESET();
    __HAL_RCC_I2C1_RELEASE_RESET();
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance != IMU_I2C_INSTANCE) return;
    __HAL_RCC_I2C1_CLK_DISABLE();
}

void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    if (huart->Instance != DEBUG_UART_INSTANCE) return;
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef g = {0};
    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_PULLUP;
    g.Speed     = GPIO_SPEED_FREQ_HIGH;
    g.Alternate = DEBUG_UART_AF;
    g.Pin = DEBUG_UART_TX_PIN; HAL_GPIO_Init(DEBUG_UART_TX_PORT, &g);
    g.Pin = DEBUG_UART_RX_PIN; HAL_GPIO_Init(DEBUG_UART_RX_PORT, &g);
}

void SysTick_Handler(void)
{
    HAL_IncTick();
}

void CAN2_RX0_IRQHandler(void)
{
    odriveCanRxISR();
}

void HardFault_Handler(void)
{
    while (1) { }
}
