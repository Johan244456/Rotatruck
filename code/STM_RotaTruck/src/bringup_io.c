/*
 *  bringup_io.c — console I/O + core handlers for the bring-up envs.
 *  See bringup_io.h.  Not part of the full firmware build.
 */

#include "bringup_io.h"
#include "board.h"
#include "command.h"     /* uart_print / uart_printf prototypes */
#include "rtt.h"
#include "stm32f4xx_hal.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

static UART_HandleTypeDef huart_dbg;

void uart_print(const char *s)
{
    size_t n = strlen(s);
    rttWrite(s, n);
    HAL_UART_Transmit(&huart_dbg, (uint8_t *)s, (uint16_t)n, 100);
}

void uart_printf(const char *fmt, ...)
{
    char buf[256];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);
    if (n <= 0) return;
    if (n >= (int)sizeof buf) n = (int)sizeof buf - 1;
    rttWrite(buf, (size_t)n);
    HAL_UART_Transmit(&huart_dbg, (uint8_t *)buf, (uint16_t)n, 100);
}

void bringupIoInit(void)
{
    rttInit();
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

int bringupGetc(void)
{
    int c = rttGetc();
    if (c >= 0) return c;
    if (__HAL_UART_GET_FLAG(&huart_dbg, UART_FLAG_ORE)) __HAL_UART_CLEAR_OREFLAG(&huart_dbg);
    if (!__HAL_UART_GET_FLAG(&huart_dbg, UART_FLAG_RXNE)) return -1;
    return (int)(huart_dbg.Instance->DR & 0xFFU);
}

bool bringupGetLine(char *buf, size_t len)
{
    static char   line[96];
    static size_t n;
    int c;
    while ((c = bringupGetc()) >= 0) {
        if (c == '\r' || c == '\n') {
            if (n == 0) continue;           /* empty line / 2nd half of CRLF */
            line[n] = '\0';
            n = 0;
            strncpy(buf, line, len - 1);
            buf[len - 1] = '\0';
            return true;
        }
        if (c < 32 || c > 126) continue;   /* junk byte some ports send on open */
        if (n < sizeof line - 1) line[n++] = (char)c;
    }
    return false;
}

void SysTick_Handler(void)
{
    HAL_IncTick();
}

void HardFault_Handler(void)
{
    while (1) { }
}
