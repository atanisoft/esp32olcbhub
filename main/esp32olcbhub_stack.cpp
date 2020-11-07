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
 * \file esp32olcbhub_stack.cpp
 *
 * OpenMRN Stack components initialization.
 *
 * @author Mike Dunston
 * @date 1 September 2020
 */

#include "cdi.hxx"
#include "sdkconfig.h"
#include "hardware.hxx"
#include "constants.hxx"
#include "nvs_config.hxx"

#include <freertos_includes.h>
#include <freertos_drivers/esp32/Esp32WiFiManager.hxx>
#include <freertos_drivers/esp32/Esp32Twai.hxx>

#include <openlcb/SimpleStack.hxx>
#include <utils/AutoSyncFileFlow.hxx>
#include <utils/constants.hxx>

// TODO: shift to SimpleStackBase to allow shift to TCP/IP native stack rather
// than GridConnect TCP/IP stack.
std::unique_ptr<openlcb::SimpleCanStack> stack;
std::unique_ptr<Esp32WiFiManager> wifi_manager;
Esp32Twai twai("/dev/twai", TWAI_RX_PIN, TWAI_TX_PIN);

extern esp32olcbhub::ConfigDef cfg;

openlcb::SimpleCanStack *initialize_openlcb_stack(node_config_t config)
{
    // Create the LCC stack.
    stack.reset(new openlcb::SimpleCanStack(config.node_id));

    if (ENABLE_PACKET_PRINTER)
    {
        stack->print_all_packets();
    }

    // If the wifi mode is enabled start the wifi manager and httpd.
    if (config.wifi_mode > WIFI_MODE_NULL && config.wifi_mode < WIFI_MODE_MAX)
    {
        // if the wifi mode is not SoftAP and we do not have a station SSID
        // force reset to SoftAP only.
        if (config.wifi_mode != WIFI_MODE_AP && strlen(config.sta_ssid) == 0)
        {
            reset_wifi_config_to_softap(&config);
        }

        wifi_manager.reset(
            new Esp32WiFiManager(config.wifi_mode != WIFI_MODE_AP ? config.sta_ssid : config.ap_ssid
                               , config.wifi_mode != WIFI_MODE_AP ? config.sta_pass : config.ap_pass
                               , stack.get()
                               , cfg.seg().wifi()
                               , config.hostname_prefix
                               , config.wifi_mode
                               , nullptr // TODO add config.sta_ip
                               , ip_addr_any
                               , 1
                               , config.ap_auth
                               , config.ap_pass));

        wifi_manager->register_network_up_callback(
        [](esp_interface_t iface, uint32_t ip)
        {
            LED_WIFI_Pin::set(true);
        });
        wifi_manager->register_network_down_callback(
        [](esp_interface_t iface)
        {
            LED_WIFI_Pin::set(false);
        });
    }

    if (TWAI_RX_PIN != GPIO_NUM_NC && TWAI_TX_PIN != GPIO_NUM_NC)
    {
        // Initialize the TWAI driver and attach it to the stack.
        twai.hw_init();
        stack->add_can_port_select("/dev/twai/twai0");
    }

    return stack.get();
}