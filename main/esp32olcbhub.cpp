/** \copyright
 * Copyright (c) 2020, Mike Dunston
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are  permitted provided that the following conditions are met:
 *
 *  - Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 *  - Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \file esp32olcbhub.cpp
 *
 * Program entry point for the ESP32OlcbHub.
 *
 * @author Mike Dunston
 * @date 4 July 2020
 */
#include "sdkconfig.h"
#include "constants.hxx"
#include "fs.hxx"
#include "hardware.hxx"
#include "NodeRebootHelper.hxx"
#include "nvs_config.hxx"

#include <algorithm>
#include <driver/i2c.h>
#include <driver/uart.h>
#include <esp_err.h>
#include <esp_log.h>
#include <esp_ota_ops.h>
#include <esp_system.h>
#include <esp_task_wdt.h>
#include <esp32/rom/rtc.h>
#include <freertos_includes.h>
#include <openlcb/SimpleStack.hxx>

///////////////////////////////////////////////////////////////////////////////
// If compiling with IDF v4.2+ enable usage of select().
///////////////////////////////////////////////////////////////////////////////
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4,2,0)

///////////////////////////////////////////////////////////////////////////////
// Enable usage of select() for GridConnect connections.
///////////////////////////////////////////////////////////////////////////////
OVERRIDE_CONST_TRUE(gridconnect_tcp_use_select);
#endif // IDF v4.2+

///////////////////////////////////////////////////////////////////////////////
// This will generate newlines after GridConnect each packet being sent.
///////////////////////////////////////////////////////////////////////////////
OVERRIDE_CONST_TRUE(gc_generate_newlines);

///////////////////////////////////////////////////////////////////////////////
// Increase the GridConnect buffer size to improve performance by bundling more
// than one GridConnect packet into the same send() call to the socket.
///////////////////////////////////////////////////////////////////////////////
OVERRIDE_CONST_DEFERRED(gridconnect_buffer_size, CONFIG_LWIP_TCP_MSS);

///////////////////////////////////////////////////////////////////////////////
// Increase the time for the buffer to fill up before sending it out over the
// socket connection.
///////////////////////////////////////////////////////////////////////////////
OVERRIDE_CONST(gridconnect_buffer_delay_usec, 1500);

///////////////////////////////////////////////////////////////////////////////
// This limits the number of outbound GridConnect packets which limits the
// memory used by the BufferPort.
///////////////////////////////////////////////////////////////////////////////
OVERRIDE_CONST(gridconnect_bridge_max_outgoing_packets, 2);

///////////////////////////////////////////////////////////////////////////////
// This limits the number of inbound GridConnect packets which limits the
// memory used by the BufferPort.
///////////////////////////////////////////////////////////////////////////////
OVERRIDE_CONST(gridconnect_bridge_max_incoming_packets, 10);
///////////////////////////////////////////////////////////////////////////////
// Increase the listener backlog to improve concurrency.
///////////////////////////////////////////////////////////////////////////////
OVERRIDE_CONST(socket_listener_backlog, 3);

openlcb::SimpleCanStack *initialize_openlcb_stack(node_config_t config);
void initialize_openlcb_helpers(node_config_t config
                              , openlcb::SimpleCanStack *stack);
void initialize_stack(node_config_t *config, openlcb::SimpleCanStack *stack
                    , bool reset_events);

extern "C"
{

void *node_reboot(void *arg)
{
    Singleton<esp32olcbhub::NodeRebootHelper>::instance()->reboot();
    return nullptr;
}

void reboot()
{
    os_thread_create(nullptr, nullptr, uxTaskPriorityGet(NULL) + 1, 2048
                   , node_reboot, nullptr);
}

ssize_t os_get_free_heap()
{
    return heap_caps_get_free_size(MALLOC_CAP_8BIT);
}

static const char * const reset_reasons[] =
{
    "unknown",                  // NO_MEAN                  0
    "power on reset",           // POWERON_RESET            1
    "unknown",                  // no key                   2
    "software reset",           // SW_RESET                 3
    "watchdog reset (legacy)",  // OWDT_RESET               4
    "deep sleep reset",         // DEEPSLEEP_RESET          5
    "reset (SLC)",              // SDIO_RESET               6
    "watchdog reset (group0)",  // TG0WDT_SYS_RESET         7
    "watchdog reset (group1)",  // TG1WDT_SYS_RESET         8
    "RTC system reset",         // RTCWDT_SYS_RESET         9
    "Intrusion test reset",     // INTRUSION_RESET          10
    "WDT Timer group reset",    // TGWDT_CPU_RESET          11
    "software reset (CPU)",     // SW_CPU_RESET             12
    "RTC WDT reset",            // RTCWDT_CPU_RESET         13
    "software reset (CPU)",     // EXT_CPU_RESET            14
    "Brownout reset",           // RTCWDT_BROWN_OUT_RESET   15
    "RTC Reset (Normal)",       // RTCWDT_RTC_RESET         16
};

void i2c_setup(bool scan = true)
{
    i2c_config_t i2c_config;
    bzero(&i2c_config, sizeof(i2c_config_t));
    i2c_config.mode = I2C_MODE_MASTER;
    i2c_config.sda_io_num = I2C_SDA_PIN;
    i2c_config.sda_pullup_en = GPIO_PULLUP_ENABLE;
    i2c_config.scl_io_num = I2C_SCL_PIN;
    i2c_config.scl_pullup_en = GPIO_PULLUP_ENABLE;
    i2c_config.master.clk_speed = 100000;
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &i2c_config));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));

    if (scan)
    {
        LOG(INFO, "[I2C] Scanning for I2C devices...");
        // Scan the I2C bus and dump the output of devices that respond
        for (uint8_t addr = 3; addr < 0x7F; addr++)
        {
            LOG(VERBOSE, "[I2C] Scanning for device on address: %02x", addr);
            i2c_cmd_handle_t cmd = i2c_cmd_link_create();
            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
            i2c_master_stop(cmd);
            esp_err_t res = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(100));
            i2c_cmd_link_delete(cmd);
            if (res == ESP_OK)
            {
                LOG(INFO, "[I2C] I2C device %02x found", addr);
            }
            else if (res == ESP_ERR_TIMEOUT)
            {
                LOG(INFO, "[I2C] I2C device %02x timeout", addr);
            }
        }
    }
}

void app_main()
{
    // capture the reason for the CPU reset
    uint8_t reset_reason = rtc_get_reset_reason(PRO_CPU_NUM);
    uint8_t orig_reset_reason = reset_reason;
    // Ensure the reset reason it within bounds.
    if (reset_reason > ARRAYSIZE(reset_reasons))
    {
        reset_reason = 0;
    }
    // silence all but error messages by default
    esp_log_level_set("*", ESP_LOG_ERROR);

    GpioInit::hw_init();

    const esp_app_desc_t *app_data = esp_ota_get_app_description();
    LOG(INFO, "\n\n%s %s starting up (%d:%s)...", app_data->project_name
      , app_data->version, reset_reason, reset_reasons[reset_reason]);
    LOG(INFO, "Compiled on %s %s using ESP-IDF %s", app_data->date
      , app_data->time, app_data->idf_ver);
    LOG(INFO, "Running from: %s", esp_ota_get_running_partition()->label);
    LOG(INFO, "%s uses the OpenMRN library\n"
              "Copyright (c) 2019-2020, OpenMRN\n"
              "All rights reserved.", app_data->project_name);
    if (reset_reason != orig_reset_reason)
    {
        LOG(WARNING, "Reset reason mismatch: %d vs %d", reset_reason
          , orig_reset_reason);
    }
    nvs_init();

    // load non-CDI based config from NVS.
    bool cleanup_config_tree = false;
    bool reset_events = false;
    node_config_t config;
    if (load_config(&config) != ESP_OK)
    {
        default_config(&config);
        cleanup_config_tree = true;
    }

    // Check for factory reset button being held on startup
    if (FACTORY_RESET_Pin::instance()->is_clr())
    {
        LED_WIFI_Pin::set(true);
        LED_ACTIVITY_Pin::set(false);
        // Count down from the overall factory reset time.
        int8_t hold_time = FACTORY_RESET_HOLD_TIME;
        for (; hold_time > 0 && FACTORY_RESET_Pin::instance()->is_clr();
             hold_time--)
        {
            if (hold_time > FACTORY_RESET_EVENTS_HOLD_TIME)
            {
                LOG(WARNING
                  , "Event ID reset in %d seconds, factory reset in %d seconds."
                  , hold_time - FACTORY_RESET_EVENTS_HOLD_TIME, hold_time);
                LED_ACTIVITY_Pin::toggle();
            }
            else
            {
                LOG(WARNING, "Factory reset in %d seconds.", hold_time);
                LED_ACTIVITY_Pin::set(false);
            }
            usleep(SEC_TO_USEC(1));
            LED_WIFI_Pin::toggle();
        }
        if (FACTORY_RESET_Pin::instance()->is_clr() && hold_time <= 0)
        {
            // if the button is still being held and the hold time expired
            // start a full factory reset.
            LOG(WARNING, "Factory reset triggered!");
            default_config(&config);
            config.force_reset = true;
        }
        else if (hold_time <= FACTORY_RESET_EVENTS_HOLD_TIME)
        {
            // if the button is not being held and the hold time is less than
            // the event id reset count down trigger a reset of events.
            LOG(WARNING, "Reset of events triggered!");
            reset_events = true;
        }
        else
        {
            // The button was released prior to the event id reset limit, do
            // nothing.
            LOG(WARNING, "Factory reset aborted!");
        }
        // reset LEDs to default state.
        LED_WIFI_Pin::set(false);
        LED_ACTIVITY_Pin::set(false);
    }

    // Check for and reset factory reset flag.
    if (config.force_reset)
    {
        cleanup_config_tree = true;
        config.force_reset = false;
        save_config(&config);
    }

    dump_config(&config);
    mount_fs();
    recursive_dump_tree(LITTLE_FS_MOUNTPOINT, cleanup_config_tree);

    LOG(INFO, "[SNIP] version:%d, manufacturer:%s, model:%s, hw-v:%s, sw-v:%s"
      , openlcb::SNIP_STATIC_DATA.version
      , openlcb::SNIP_STATIC_DATA.manufacturer_name
      , openlcb::SNIP_STATIC_DATA.model_name
      , openlcb::SNIP_STATIC_DATA.hardware_version
      , openlcb::SNIP_STATIC_DATA.software_version);

    openlcb::SimpleCanStack *stack = initialize_openlcb_stack(config);
    initialize_openlcb_helpers(config, stack);
    initialize_stack(&config, stack, reset_events);

    // Check if the reset reason was due to brownout.
    if (reset_reason == RTCWDT_BROWN_OUT_RESET)
    {
        // Queue the brownout event to be sent.
        stack->executor()->add(new CallbackExecutable([&]()
        {
            LOG_ERROR("[Brownout] Detected a brownout reset, sending event");
            stack->send_event(openlcb::Defs::NODE_POWER_BROWNOUT_EVENT);
        }));
    }

    // disable the task WDT before passing ownership of the task to the stack.
    LOG(INFO, "[WDT] Disabling WDT for app_main");
    esp_task_wdt_delete(NULL);

    LOG(INFO, "[LCC] Starting LCC stack");
    stack->loop_executor();
}

} // extern "C"
