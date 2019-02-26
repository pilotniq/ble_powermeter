/*
 * Battery service implementation
 *
 * Plan:
 *   -do an ADC read
 */
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_drv_adc.h"
#define NRF_LOG_MODULE_NAME "ERL_BLE_BAS"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "app_timer.h"
#include "app_scheduler.h"

#include "erl_ble.h"  // for APP_TIMER_PRESCALER
#include "erl_ble_adv.h"
#include "ble_bas.h"

// this samples battery voltage every 2s.
// keep for now, later increase to much less often to save battery
#define BATTERY_LEVEL_MEAS_INTERVAL      APP_TIMER_TICKS(2000, APP_TIMER_PRESCALER)  /**< Battery level measurement interval (ticks). */

/*
 * static function prototypes
 */
static void battery_level_update( void *data, uint16_t size );
static void erl_ble_bas_on_ble_evt( ble_evt_t *evt, void *context );

 APP_TIMER_DEF(m_battery_timer_id);                        /**< Battery timer. */
 static ble_bas_t m_bas;                                   /**< Structure used to identify the battery service. */
 static nrf_adc_value_t       adc_value; /**< ADC buffer. */
 static nrf_drv_adc_channel_t m_channel_config; //  = NRF_DRV_ADC_DEFAULT_CHANNEL(NRF_ADC_CONFIG_INPUT_0); /**< Channel instance. Default configuration used. */
static uint16_t battery_mV;
 /**
  * @brief ADC interrupt handler.
  */
 static void adc_event_handler(nrf_drv_adc_evt_t const * p_event)
 {
     if (p_event->type == NRF_DRV_ADC_EVT_DONE)
     {
       // take the adc value and do some calculations
       NRF_LOG_INFO( "ADC value: %d", adc_value );

       uint16_t voltage_mV;

       voltage_mV = ((uint32_t) adc_value) * 1200 / 1024;

       // reverse the voltage divider
       battery_mV = ((uint32_t) voltage_mV) * 122 / 22;

       app_sched_event_put( NULL, 0, battery_level_update );
       // TODO: take value calculated by ADC event handler
       // battery_level_update( battery_mV * 100 / 2400 );

       // battery_level_update(p_event->data.done.p_buffer[0]);
       /*
         uint32_t i;
         for (i = 0; i < p_event->data.done.size; i++)
         {
 	  // err_code = ble_bas_battery_level_
             NRF_LOG_INFO("Current sample value: %d\r\n", p_event->data.done.p_buffer[i]);
         }
         */
     }
 }
/**@brief Function for performing battery measurement and updating the Battery Level characteristic
 *        in Battery Service.
 */
static void battery_level_update( void *data, uint16_t size )
{
  uint32_t err_code;
  uint8_t level;
     // uint8_t  battery_level;
/*
     battery_level = (uint8_t)sensorsim_measure(&m_battery_sim_state,
                                                &m_battery_sim_cfg);
*/
  // battery_level = 50; // TODO: Calculate from ADC
  level = battery_mV * 100 / 3000;

     err_code = ble_bas_battery_level_update(&m_bas, level);

     if ((err_code != NRF_SUCCESS) &&
         (err_code != NRF_ERROR_INVALID_STATE) &&
         (err_code != BLE_ERROR_NO_TX_PACKETS) &&
         (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
        )
     {
         APP_ERROR_HANDLER(err_code);
     }
}

 /**@brief Function for handling the Battery measurement timer timeout.
  *
  * @details This function will be called each time the battery level measurement timer expires.
  *
  * @param[in] p_context  Pointer used for passing some arbitrary information (context) from the
  *                       app_start_timer() call to the timeout handler.
  */
static void battery_level_meas_timeout_handler(void * p_context)
{
  UNUSED_PARAMETER(p_context);

  ret_code_t ret;

  // initiate an ADC conversion. Event will be generated when it is done
  ret = nrf_drv_adc_buffer_convert( &adc_value, 1);
  APP_ERROR_CHECK(ret);

  nrf_drv_adc_sample();
}

 static void adc_config(void)
 {
     ret_code_t ret_code;
     nrf_drv_adc_config_t config = NRF_DRV_ADC_DEFAULT_CONFIG;

     ret_code = nrf_drv_adc_init(&config, adc_event_handler);
     APP_ERROR_CHECK(ret_code);

     memset(&m_channel_config, 0, sizeof(m_channel_config));

     m_channel_config.config.config.resolution = NRF_ADC_CONFIG_RES_10BIT;
     m_channel_config.config.config.input = NRF_ADC_CONFIG_SCALING_INPUT_FULL_SCALE;
     m_channel_config.config.config.reference = NRF_ADC_CONFIG_REF_VBG;
     // m_channel_config.config.config.ain = nrf_drv_adc_gpio_to_ain(2); // pin 2 is AIN3
     m_channel_config.config.config.ain = NRF_ADC_CONFIG_INPUT_3; // pin 2 is AIN3

     nrf_drv_adc_channel_enable(&m_channel_config);
 }
/*
static void erl_ble_bas_evt_func( ble_evt_t *evt, void *context )
{
  ble_bas_on_ble_evt( &m_bas, evt );
}
*/
void erl_ble_bas_init()
{
  ret_code_t err_code;
  ble_bas_init_t bas_init;
  ble_uuid_t uuid = {BLE_UUID_BATTERY_SERVICE, BLE_UUID_TYPE_BLE};

  adc_config();

  // Initialize Battery Service.
  memset(&bas_init, 0, sizeof(bas_init));

  bas_init.evt_handler          = NULL;
  bas_init.support_notification = true;
  bas_init.p_report_ref         = NULL;
  bas_init.initial_batt_level   = 100;

  // Here the sec level for the Battery Service can be changed/increased.
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_char_attr_md.cccd_write_perm);
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_char_attr_md.read_perm);
  BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&bas_init.battery_level_char_attr_md.write_perm);

  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_report_read_perm);

  err_code = ble_bas_init(&m_bas, &bas_init);
  APP_ERROR_CHECK(err_code);

  // erl_ble_bas_on_ble_evt( p_ble_evt );
  err_code = erl_ble_add_ble_evt_func( erl_ble_bas_on_ble_evt, &m_bas );
  APP_ERROR_CHECK(err_code);

  err_code = erl_ble_adv_add_uuid( &uuid );
  APP_ERROR_CHECK(err_code);

  // Create timers.
  err_code = app_timer_create(&m_battery_timer_id,
                              APP_TIMER_MODE_REPEATED,
                              battery_level_meas_timeout_handler);
  APP_ERROR_CHECK(err_code);

  // Start application timers.
  err_code = app_timer_start(m_battery_timer_id, BATTERY_LEVEL_MEAS_INTERVAL, NULL);
  APP_ERROR_CHECK(err_code);
}

static void erl_ble_bas_on_ble_evt( ble_evt_t *evt, void *context )
{
  ble_bas_on_ble_evt(&m_bas, evt);
}

uint16_t erl_ble_bas_get_adcValue(void)
{
  return adc_value;
}

uint16_t erl_ble_bas_get_battery_mV(void)
{
  return battery_mV;
}
