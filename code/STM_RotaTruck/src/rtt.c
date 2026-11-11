/*
 *  rtt.c — minimal SEGGER-RTT-compatible control block (1 up, 1 down).
 *
 *  Memory layout matches SEGGER's SEGGER_RTT_CB so OpenOCD's
 *  `rtt setup <addr> <size> "SEGGER RTT"` + `rtt start` finds it in RAM.
 */

#include "rtt.h"
#include <string.h>

#define RTT_UP_SIZE    4096u
#define RTT_DOWN_SIZE  16u

typedef struct {
    const char       *name;
    char             *buf;
    unsigned          size;
    volatile unsigned wr;
    volatile unsigned rd;
    unsigned          flags;     /* 0 = non-blocking, skip if full */
} rtt_ring_t;

typedef struct {
    char       id[16];
    int        num_up;
    int        num_down;
    rtt_ring_t up[1];
    rtt_ring_t down[1];
} rtt_cb_t;

static char      s_up_buf[RTT_UP_SIZE];
static char      s_down_buf[RTT_DOWN_SIZE];
static rtt_cb_t  s_cb;

void rttInit(void)
{
    s_cb.num_up   = 1;
    s_cb.num_down = 1;
    s_cb.up[0]   = (rtt_ring_t){ "Terminal", s_up_buf,   RTT_UP_SIZE,   0, 0, 0 };
    s_cb.down[0] = (rtt_ring_t){ "Terminal", s_down_buf, RTT_DOWN_SIZE, 0, 0, 0 };
    /* ID written last so the host never finds a half-initialised block. */
    __asm volatile ("" ::: "memory");
    strcpy(s_cb.id, "SEGGER RTT");
}

void rttWrite(const char *s, size_t len)
{
    rtt_ring_t *r = &s_cb.up[0];
    unsigned wr = r->wr;
    unsigned rd = r->rd;
    unsigned avail = (rd > wr) ? (rd - wr - 1u) : (r->size - wr + rd - 1u);
    if (len > avail) return;                /* host not reading: drop the whole message */
    while (len--) {
        r->buf[wr] = *s++;
        if (++wr == r->size) wr = 0;
    }
    __asm volatile ("" ::: "memory");
    r->wr = wr;
}

int rttGetc(void)
{
    rtt_ring_t *r = &s_cb.down[0];
    unsigned rd = r->rd;
    if (rd == r->wr) return -1;
    int c = (unsigned char)r->buf[rd];
    if (++rd == r->size) rd = 0;
    r->rd = rd;
    return c;
}
