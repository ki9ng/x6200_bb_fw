/* Host stubs for the board services App/SDR calls. Logging is silenced unless
 * DSP_TEST_LOG is set; event subscriptions record the callback so a test can
 * drive RX/TX transitions itself. */
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include "bsp_log.h"
#include "app_event.h"

int Bsp_Log_Print(BspLogLevel level, const char *tag, const char *fmt, ...)
{
    if (!getenv("DSP_TEST_LOG")) return 0;
    va_list ap;
    va_start(ap, fmt);
    fprintf(stderr, "[%u %s] ", (unsigned)level, tag);
    vfprintf(stderr, fmt, ap);
    fputc('\n', stderr);
    va_end(ap);
    return 0;
}

AppEventRxTxFn stub_rx_tx_cb;
void *stub_rx_tx_user;

int App_Event_SubscribeRxTx(AppEventRxTxFn cb, void *user)
{
    stub_rx_tx_cb = cb;
    stub_rx_tx_user = user;
    return 0;
}
