/**
 * SPDX-License-Identifier: Apache-2.0
 * 
 */


 //Implementation of the BNO08X i2c Hardware Abstraction Layer

#include <bno08x_platform.h>


static bool _reset_occurred = false;

static uint8_t hardware_reset(void)
{
    return 0;
}

static void event_callback(void *cookie, sh2_AsyncEvent_t * pEvent)
{
 // If we see a reset, set a flag so that sensors will be reconfigured.
  if (pEvent->eventId == SH2_RESET) {
    _reset_occurred = true;
  }
}

void bno08x_i2c_hal_init(i2c_hal_t * pHal, bool defaultState)
{
    pHal->isDefault = defaultState;
    pHal->reset = hardware_reset;
    pHal->sh2_event_callback = event_callback;

    pHal->bno08x_i2c_hal.open = NULL;
    pHal->bno08x_i2c_hal.close = NULL;
    pHal->bno08x_i2c_hal.read = NULL;
    pHal->bno08x_i2c_hal.write = NULL;
    pHal->bno08x_i2c_hal.getTimeUs = NULL;

    // return &pHal->bno08x_i2c_hal;
}