/**
 * Copyright (c) 2014 - 2017, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @example examples/ble_peripheral/ble_app_hrs/main.c
 *
 * @brief Heart Rate Service Sample Application main file.
 *
 * This file contains the source code for a sample application using the Heart Rate service
 * (and also Battery and Device Information services). This application uses the
 * @ref srvlib_conn_params module.
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ble.h"
#include "fds.h"

// #include "boards.h"
#include "app_timer.h"
#include "app_scheduler.h"
#include "nrf_delay.h"

#define NRF_LOG_MODULE_NAME "APP"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "erl_ble.h"
#include "erl_ble_adv.h"
#include "erl_ble_bas.h"
#include "ble_powermeter.h"

#define IS_SRVC_CHANGED_CHARACT_PRESENT  1                                           /**< Include or not the service_changed characteristic. if not enabled, the server's database cannot be changed for the lifetime of the device*/

#define DEVICE_NAME                      "BlueLightYear"                                /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME                "ErlandLewin"                       /**< Manufacturer. Will be passed to Device Information Service. */

#define DEAD_BEEF                        0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define APP_TIMER_OP_QUEUE_SIZE          4                                           /**< Size of timer operation queues. */

#define PIN_IR_IN   1

/*
 * Global variables
 */
 // UUID generated here: https://www.uuidgenerator.net/
 static ble_uuid128_t powerMeter_base_uuid =
  {{ 0x86, 0xb4, 0x0d, 0x45, 0x76, 0x88, 0x4b, 0x0f,
     0xa7, 0xbe, 0x54, 0x84, 0x6f, 0x90, 0x17, 0x8a }};

 /*
static uint32_t energy_counter_wh = 0;
static uint32_t previous_wh_timestamp = 0;
static uint32_t latest_wh_timestamp = 0;
*/
//APP_TIMER_DEF(m_rr_interval_timer_id);                    /**< RR interval timer. */                 /**< RR interval timer. */
//APP_TIMER_DEF(m_sensor_contact_timer_id);                 /**< Sensor contact detected timer. */

/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num   Line number of the failing ASSERT call.
 * @param[in] file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**@brief Function for starting application timers.
 */
static void application_timers_start(void)
{
//    uint32_t err_code;
// Start application timers.
//err_code = app_timer_start(m_battery_timer_id, BATTERY_LEVEL_MEAS_INTERVAL, NULL);
//APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the Connection Parameters module.
 */

/**@brief Function for the Power manager.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();

    APP_ERROR_CHECK(err_code);
}

/**@brief Function for application main entry.
 */
int main(void)
{
    uint32_t err_code;
    bool     erase_bonds = false;
#if 1
    ble_powermeter_t powerMeter;
    ble_powermeter_init_t powerMeter_init;
#endif
    struct
    {
      uint16_t counter;
      uint16_t battery_mV;
    } myManufData = { 0, 0 };

    // Initialize.
    err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEBUG("main entry\n");

    // 4th param is number of application timers
    erl_ble_init( erase_bonds, MANUFACTURER_NAME, DEVICE_NAME, 0, &powerMeter_base_uuid );

    err_code = erl_ble_adv_init();
    APP_ERROR_CHECK(err_code);

#if 1 // crashes
    powerMeter_init.evt_handler = NULL;
    powerMeter_init.support_notification = true;
    powerMeter_init.p_report_ref = NULL; // What is a Report Reference Descriptor?

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&powerMeter_init.power_char_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&powerMeter_init.power_char_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&powerMeter_init.power_char_attr_md.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&powerMeter_init.power_report_read_perm);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&powerMeter_init.energy_char_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&powerMeter_init.energy_char_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&powerMeter_init.energy_char_attr_md.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&powerMeter_init.energy_report_read_perm);

    err_code = ble_powermeter_init(&powerMeter, &powerMeter_init, &powerMeter_base_uuid);
    APP_ERROR_CHECK(err_code);
#endif
  // erl_ble_adv_init must be called after the services are set up
  err_code = erl_ble_adv_activate( &myManufData, sizeof( myManufData ) );
  APP_ERROR_CHECK(err_code);

  err_code = erl_ble_adv_start();
  APP_ERROR_CHECK(err_code);

#if 0
    // configure pin 1 as input.
    // when pin 1 goes high, save time stamp. Increment energy counter
    err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);

    // false = not high accuracy; don't invoke high frequency clock
    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_LOTOHI(false);
    in_config.pull = NRF_GPIO_PIN_NOPULL;

    err_code = nrf_drv_gpiote_in_init(PIN_IR_IN, &in_config, ir_pin_handler);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(PIN_IR_IN, true);
#endif
#if 0
    // nrf_drv_gpiote_out_config_t out_config = GPIOTE_CONFIG_OUT_SIMPLE(false);

    // err_code = nrf_drv_gpiote_out_init(PIN_OUT, &out_config);
    // APP_ERROR_CHECK(err_code);

    // in_config.pull = NRF_GPIO_PIN_PULLUP;

#endif
    // Start execution.
    NRF_LOG_INFO("Blue Light Year Start!\r\n");
    application_timers_start();
    // advertising_start();

    // Enter main loop.
    for (;;)
    {
        if (NRF_LOG_PROCESS() == false)
        {
          myManufData.counter++;
          myManufData.battery_mV = erl_ble_bas_get_battery_mV();

          // test updating manufacturer data
          err_code = erl_ble_adv_update_manuf_data( &myManufData, sizeof( myManufData ) );
          APP_ERROR_CHECK(err_code);

           app_sched_execute();
          power_manage();
        }
    }
}
