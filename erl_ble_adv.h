/*
  erl_ble_adv.h
  */

#include "ble.h"

#ifndef ERL_BLE_ADV_H
#define ERL_BLE_ADV_H

#include "ble.h"

uint32_t erl_ble_adv_init(void);
// call activate after the services are setup, and before adv_start
uint32_t erl_ble_adv_activate( const void *manuf_data, int manuf_data_size );
uint32_t erl_ble_adv_update_manuf_data( void *data, int size );
uint32_t erl_ble_adv_add_uuid( const ble_uuid_t *uuid );
uint32_t erl_ble_adv_start(void);
void erl_ble_adv_on_sys_evt(uint32_t sys_evt);

#endif
