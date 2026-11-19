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

/* AGC needs the spectrum noise estimate and the AM carrier level. Tests set
 * these directly. */
#include <stdbool.h>
bool  stub_noise_valid = true;
float stub_noise_db    = -100.0f;
float stub_am_carrier  = 0.0f;
bool  Sdr_Spectrum_Is_Noise_Valid(void) { return stub_noise_valid; }
float Sdr_Spectrum_Get_Demod_Noise_Est_Db(void) { return stub_noise_db; }
float Sdr_AM_GetCarrierLevel(void) { return stub_am_carrier; }

AppEventRxCfgFn stub_rx_cfg_cb;
void *stub_rx_cfg_user;
int App_Event_SubscribeRxCfg(AppEventRxCfgFn cb, void *user)
{
    stub_rx_cfg_cb = cb;
    stub_rx_cfg_user = user;
    return 0;
}
