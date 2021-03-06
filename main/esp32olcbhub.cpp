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
#include "fs.hxx"
#include "hardware.hxx"
#include "NodeRebootHelper.hxx"
#include "nvs_config.hxx"

#include <algorithm>
#include <bootloader_hal.h>
#include <driver/i2c.h>
#include <driver/uart.h>
#include <esp_err.h>
#include <esp_log.h>
#include <esp_ota_ops.h>
#include <esp_system.h>
#include <esp_task_wdt.h>
#include <esp32/rom/rtc.h>
#include <freertos_includes.h>
#include <freertos_drivers/esp32/Esp32SocInfo.hxx>
#include <mutex>
#include <openlcb/SimpleStack.hxx>

///////////////////////////////////////////////////////////////////////////////
// Enable usage of select() for GridConnect connections.
///////////////////////////////////////////////////////////////////////////////
OVERRIDE_CONST_TRUE(gridconnect_tcp_use_select);

///////////////////////////////////////////////////////////////////////////////
// This will generate newlines after GridConnect each packet being sent.
///////////////////////////////////////////////////////////////////////////////
OVERRIDE_CONST_TRUE(gc_generate_newlines);

///////////////////////////////////////////////////////////////////////////////
// Increase the GridConnect buffer size to improve performance by bundling more
// than one GridConnect packet into the same send() call to the socket.
///////////////////////////////////////////////////////////////////////////////
OVERRIDE_CONST(gridconnect_buffer_size, CONFIG_LWIP_TCP_MSS);

///////////////////////////////////////////////////////////////////////////////
// Increase the time for the buffer to fill up before sending it out over the
// socket connection.
///////////////////////////////////////////////////////////////////////////////
OVERRIDE_CONST(gridconnect_buffer_delay_usec, 500);

///////////////////////////////////////////////////////////////////////////////
// This limits the number of outbound GridConnect packets which limits the
// memory used by the BufferPort.
///////////////////////////////////////////////////////////////////////////////
OVERRIDE_CONST(gridconnect_bridge_max_outgoing_packets, 10);

///////////////////////////////////////////////////////////////////////////////
// This limits the number of inbound GridConnect packets which limits the
// memory used by the BufferPort.
///////////////////////////////////////////////////////////////////////////////
OVERRIDE_CONST(gridconnect_bridge_max_incoming_packets, 10);

///////////////////////////////////////////////////////////////////////////////
// Increase the listener backlog to improve concurrency.
///////////////////////////////////////////////////////////////////////////////
OVERRIDE_CONST(socket_listener_backlog, 2);

///////////////////////////////////////////////////////////////////////////////
// Increase the CAN RX frame buffer size to reduce overruns when the hub has
// high load (ie: large datagram transport).
///////////////////////////////////////////////////////////////////////////////
OVERRIDE_CONST(can_rx_buffer_size, 64);

/// Number of seconds to hold the Factory Reset button to force clear all
/// stored configuration data.
static constexpr int8_t FACTORY_RESET_HOLD_TIME = 10;

/// Number of seconds to hold the Factory Reset button to force regeneration of
/// all Event IDs. NOTE: This will *NOT* clear WiFi configuration data.
static constexpr int8_t FACTORY_RESET_EVENTS_HOLD_TIME = 5;

namespace esp32olcbhub
{
void start_openlcb_stack(node_config_t *config, bool reset_events
                       , bool brownout_detected);
}
void start_bootloader_stack(uint64_t id);
void initialize_bootloader_vars(uint8_t reason);
void set_bootloader_requested();

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

void app_main()
{
    // silence all but error messages by default
    esp_log_level_set("*", ESP_LOG_ERROR);

    GpioInit::hw_init();

    const esp_app_desc_t *app_data = esp_ota_get_app_description();
    LOG(INFO, "\n\n%s %s starting up...", app_data->project_name,
        app_data->version);
    LOG(INFO, "Compiled on %s %s using ESP-IDF %s", app_data->date,
        app_data->time, app_data->idf_ver);
    LOG(INFO, "Running from: %s", esp_ota_get_running_partition()->label);
    LOG(INFO, "%s uses the OpenMRN library\n"
              "Copyright (c) 2019-2021, OpenMRN\n"
              "All rights reserved.", app_data->project_name);
    LOG(INFO, "[SNIP] version:%d, manufacturer:%s, model:%s, hw-v:%s, sw-v:%s",
        openlcb::SNIP_STATIC_DATA.version,
        openlcb::SNIP_STATIC_DATA.manufacturer_name,
        openlcb::SNIP_STATIC_DATA.model_name,
        openlcb::SNIP_STATIC_DATA.hardware_version,
        openlcb::SNIP_STATIC_DATA.software_version);
    uint8_t reset_reason = Esp32SocInfo::print_soc_info();
    initialize_bootloader_vars(reset_reason);
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

    // Check for factory reset button being held to GND and the USER button
    // not being held to GND. If this is detected the factory reset process
    // will be started.
    if (FACTORY_RESET_Pin::instance()->is_clr() && 
        USER_BUTTON_Pin::instance()->is_set())
    {
        LED_WIFI_Pin::instance()->set();
        LED_ACTIVITY_Pin::instance()->clr();
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
                LED_ACTIVITY_Pin::instance()->clr();
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
        LED_WIFI_Pin::instance()->clr();
        LED_ACTIVITY_Pin::instance()->clr();
    }
    else if (FACTORY_RESET_Pin::instance()->is_clr() && 
             USER_BUTTON_Pin::instance()->is_clr())
    {
        // If both the factory reset and user button are held to GND it is a
        // request to enter the bootloader mode.
        set_bootloader_requested();

        // give a visual indicator that the bootloader request has been ACK'd
        // turn on both WiFi and Activity LEDs, wait ~1sec, turn off WiFi LED,
        // wait ~1sec, turn off Activity LED.
        LED_WIFI_Pin::instance()->set();
        LED_ACTIVITY_Pin::instance()->set();
        vTaskDelay(pdMS_TO_TICKS(1000));
        LED_WIFI_Pin::instance()->clr();
        vTaskDelay(pdMS_TO_TICKS(1000));
        LED_ACTIVITY_Pin::instance()->clr();
    }

    // Check for and reset factory reset flag.
    if (config.force_reset)
    {
        cleanup_config_tree = true;
        config.force_reset = false;
        save_config(&config);
    }

    dump_config(&config);

    // If the bootloader has been requested and TWAI is enabled we can start
    // the bootloader stack. Otherwise start the full stack.
    if (request_bootloader())
    {
        start_bootloader_stack(config.node_id);
    }
    else
    {
        mount_fs(cleanup_config_tree);
        esp32olcbhub::start_openlcb_stack(&config, reset_events
                                        , reset_reason == RTCWDT_BROWN_OUT_RESET);
    }

    // At this point the OpenMRN stack is running in it's own task and we can
    // safely exit from this one. We do not need to cleanup as that will be
    // handled automatically by ESP-IDF.
}


std::mutex log_mux;
// OpenMRN log output method, overridden to add mutex guard around fwrite/fputc
// due to what appears to be a bug in esp-idf where it thinks a recursive mutex
// is being held and that it is in an ISR context.
void log_output(char* buf, int size)
{
    const std::lock_guard<std::mutex> lock(log_mux);
    // drop null/short messages
    if (size <= 0) return;

    // no error checking is done here, any error check logs would get lost if
    // there was a failure at this point anyway.
    fwrite(buf, 1, size, stdout);
    fputc('\n', stdout);
}

} // extern "C"
