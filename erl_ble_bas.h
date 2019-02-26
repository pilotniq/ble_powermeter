#ifndef ERL_BLE_BAS_H
#define ERL_BLE_BAS_H

#include "ble.h"

void erl_ble_bas_init();
void erl_ble_bas_on_ble_evt( ble_evt_t * p_ble_evt );
uint16_t erl_ble_bas_get_adcValue(void);
uint16_t erl_ble_bas_get_battery_mV(void);

#endif
