/*
 * Hide all the BLE crap from the application
 */

#ifndef ERL_BLE_H
#define ERL_BLE_H

#include "ble.h"

// Prescaler 0: timer resolution 30.517 us, overflow after 512 seconds
#define APP_TIMER_PRESCALER              0  /**< Value of the RTC1 PRESCALER register. */

/*
 * type defintions
 */
typedef void (*ble_evt_func_p)( ble_evt_t * p_ble_evt, void *context );

void erl_ble_init( bool erase_bonds, /* SleepModeEnterFunc sleepModeEnter,*/
                   const char *manufacturerName, const char *deviceName,
                   int appTimerCount, ble_uuid128_t *baseUUID );
// uint32_t erl_ble_update_manuf_data( void *data, int size );
uint32_t erl_ble_add_ble_evt_func( ble_evt_func_p func, void *context );

// returns error code
// uint32_t erl_ble_bas_battery_level_update(uint8_t battery_level);

#endif
