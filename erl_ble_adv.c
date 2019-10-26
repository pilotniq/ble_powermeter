/*
 * erl_ble_adv.c
 */

#include "ble.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_dis.h"
#include "app_error.h"
#define NRF_LOG_MODULE_NAME "ERL_BLE_ADV"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#include "erl_ble.h"
#include "erl_ble_adv.h"

#define APP_ADV_FAST_INTERVAL                 300                                         /**< The advertising interval (in units of 0.625 ms. This value corresponds to 187.5 ms). */
// #define APP_ADV_TIMEOUT_IN_SECONDS       180                                         /**< The advertising timeout in units of seconds. */
#define APP_ADV_FAST_TIMEOUT_IN_SECONDS       180                                         /**< The advertising timeout in units of seconds. */
#define APP_ADV_SLOW_INTERVAL                3200                                       /**< slow advertising 1600=1 second */
#define APP_ADV_SLOW_TIMEOUT_IN_SECONDS  0

#define MAX_UUIDS 8

static int uuid_count = 1;
static ble_uuid_t m_adv_uuids[ MAX_UUIDS ] = {
                                  /*  {BLE_UUID_BATTERY_SERVICE, BLE_UUID_TYPE_BLE}, */
                                  {BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE} }; /**< Universally unique service identifiers. */

/*
 * static function prototypes
 */
static void init_advdata( ble_advdata_t *advdata, ble_advdata_t *srdata,
                          ble_advdata_manuf_data_t *manuf,
                          const void *manuf_data,
                          uint8_t manuf_data_size );
static void on_adv_evt(ble_adv_evt_t ble_adv_evt);

/*
 * Start of code
 */
 /**@brief Function for putting the chip into sleep mode.
  * TODO: This does not belong in the advertising module. Move away.
  * @note This function will not return.
  */
 static void sleep_mode_enter(void)
 {
   //    uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
   // uint32_t err_code;

   //APP_ERROR_CHECK(err_code);

     // Prepare wakeup buttons.
   //   err_code = bsp_btn_ble_sleep_mode_prepare();
   //  APP_ERROR_CHECK(err_code);

     // don't turn off, then we'd lose clock
     // Go to system-off mode (this function will not return; wakeup will cause a reset).
     // err_code = sd_power_system_off();
     // APP_ERROR_CHECK(err_code);
 }

static void init_advdata( ble_advdata_t *advdata, ble_advdata_t *srdata,
                          ble_advdata_manuf_data_t *manuf, const void *manuf_data,
                          uint8_t manuf_data_size )
{
  if( manuf != NULL )
  {
    // memset( manuf, 0, sizeof( *manuf ) );

    manuf->company_identifier = 0xffff;
    manuf->data.p_data = (uint8_t *) manuf_data;
    manuf->data.size = manuf_data_size;
  }

  // for unknown reasns, memset causes crash.
  // need the memset here, or crash later
  memset(advdata, 0, sizeof(*advdata));
  advdata->name_type               = /*BLE_ADVDATA_NO_NAME*/ /* BLE_ADVDATA_SHORT_NAME */BLE_ADVDATA_FULL_NAME;
  // advdata->short_name_len          = 4;
  advdata->include_appearance      = false;
  advdata->flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
  // advdata->uuids_complete.uuid_cnt = uuid_count;
  // advdata->uuids_complete.p_uuids  = m_adv_uuids;
  advdata->p_manuf_specific_data   = manuf;

  // put all setvice uuids in sr. should have more advance logic to fit parts
  // in proper package for optimized packing
  memset(srdata, 0, sizeof(*srdata));

  srdata->uuids_complete.uuid_cnt = uuid_count;
  srdata->uuids_complete.p_uuids  = m_adv_uuids;

  // srdata->uuids_more_available.uuid_cnt = uuid_count;
  // srdata->uuids_more_available.p_uuids  = m_adv_uuids;
}

/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
  // uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            NRF_LOG_INFO("Fast Advertising\r\n");
            // err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            // APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();
            break;

        default:
            break;
    }
}

static void on_ble_evt( ble_evt_t *evt, void *context )
{
  ble_advertising_on_ble_evt( evt );
}

void erl_ble_adv_on_sys_evt(uint32_t sys_evt)
{
  ble_advertising_on_sys_evt(sys_evt);
}

/**@brief Function for starting advertising.
 */
uint32_t erl_ble_adv_start(void)
{
    ret_code_t err_code;

    err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);

    return err_code;
}

uint32_t erl_ble_adv_update_manuf_data( void *data, int size )
{
  ble_advdata_t advdata, srdata; // static to see of crash affected
  ble_advdata_manuf_data_t manuf_data;

  init_advdata( &advdata, &srdata, &manuf_data, data, size );

  return ble_advdata_set( &advdata, &srdata );
}

static void init_options(ble_adv_modes_config_t *options)
{
  memset(options, 0, sizeof(*options));
  options->ble_adv_fast_enabled  = true;
  options->ble_adv_fast_interval = APP_ADV_FAST_INTERVAL;
  options->ble_adv_fast_timeout  = APP_ADV_FAST_TIMEOUT_IN_SECONDS;
  options->ble_adv_slow_enabled  = true;
  options->ble_adv_slow_interval = APP_ADV_SLOW_INTERVAL;
  options->ble_adv_slow_timeout  = APP_ADV_SLOW_TIMEOUT_IN_SECONDS;

}
 /**@brief Function for initializing the Advertising functionality.
  */
uint32_t erl_ble_adv_init(void)
{
  return NRF_SUCCESS;
}

/*
 * call after services are set up
 */
uint32_t erl_ble_adv_activate( const void *manuf_data, int manuf_data_size )
{
  uint32_t               err_code;
  ble_advdata_t          advdata, srdata;
  ble_adv_modes_config_t options;
  ble_advdata_manuf_data_t manuf;
  // uint8_t data[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08 };

  // Build advertising data struct to pass into @ref ble_advertising_init.
  init_advdata( &advdata, &srdata, &manuf, manuf_data, manuf_data_size);

  init_options( &options );

 // NOTE: don't do this on reinit
 err_code = erl_ble_add_ble_evt_func( on_ble_evt, NULL );
 APP_ERROR_CHECK(err_code);

 // commented out to see if it affects crash
 err_code = ble_advertising_init(&advdata, &srdata, &options, on_adv_evt, NULL);
 APP_ERROR_CHECK(err_code);

 return err_code;
}

#if 0
static uint32_t reinit()
{
  uint32_t               err_code;
  ble_advdata_t          advdata;
  ble_adv_modes_config_t options;
  ble_advdata_manuf_data_t manuf_data;
  uint8_t data[] = { 0x01, 0x02 };

  // Build advertising data struct to pass into @ref ble_advertising_init.
  init_advdata( &advdata, &manuf_data, data, sizeof( data ));

  init_options( &options );

  err_code = ble_advertising_init(&advdata, NULL, &options, on_adv_evt, NULL);
  APP_ERROR_CHECK(err_code);

  return err_code;
}
#endif
uint32_t erl_ble_adv_add_uuid( const ble_uuid_t *uuid )
{
  if( uuid_count == MAX_UUIDS )
    return NRF_ERROR_NULL; // TODO: Find better error code

  m_adv_uuids[ uuid_count ] = *uuid;

  uuid_count++;

  // can we re-initialize advertising? Try!
  return NRF_SUCCESS;
}
