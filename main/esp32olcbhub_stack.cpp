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

#include "sdkconfig.h"
#include "cdi.hxx"
#include "ConfigUpdateHelper.hxx"
#include "DelayRebootHelper.hxx"
#include "FactoryResetHelper.hxx"
#include "fs.hxx"
#include "hardware.hxx"
#include "HealthMonitor.hxx"
#include "NodeRebootHelper.hxx"
#include "nvs_config.hxx"
#include "web_server.hxx"

#include <CDIHelper.hxx>
#include <esp_task.h>
#include <freertos_includes.h>
#include <freertos_drivers/esp32/Esp32WiFiManager.hxx>
#include <freertos_drivers/esp32/Esp32Twai.hxx>

#include <openlcb/SimpleStack.hxx>
#include <utils/AutoSyncFileFlow.hxx>
#include <utils/format_utils.hxx>
#include <utils/constants.hxx>

static esp32olcbhub::ConfigDef cfg(0);

namespace openlcb
{
    // where the CDI file exists
    const char *const CDI_FILE = "/fs/cdi.xml";

    // This will stop openlcb from exporting the CDI memory space upon start.
    const char CDI_DATA[] = "";

    // Path to where OpenMRN should persist general configuration data.
    const char *const CONFIG_FILENAME = "/fs/config";

    // The size of the memory space to export over the above device.
    const size_t CONFIG_FILE_SIZE = cfg.seg().size() + cfg.seg().offset();

    // Default to store the dynamic SNIP data is stored in the same persistant
    // data file as general configuration data.
    const char *const SNIP_DYNAMIC_FILENAME = "/fs/config";

    /// Defines the identification information for the node. The arguments are:
    ///
    /// - 4 (version info, always 4 by the standard
    /// - Manufacturer name
    /// - Model name
    /// - Hardware version
    /// - Software version
    ///
    /// This data will be used for all purposes of the identification:
    ///
    /// - the generated cdi.xml will include this data
    /// - the Simple Node Ident Info Protocol will return this data
    /// - the ACDI memory space will contain this data.
    const SimpleNodeStaticValues SNIP_STATIC_DATA =
    {
        4,
        SNIP_PROJECT_PAGE,
        SNIP_PROJECT_NAME,
        SNIP_HW_VERSION,
        SNIP_SW_VERSION
    };
}

// TODO: shift to SimpleStackBase to allow shift to TCP/IP native stack rather
// than GridConnect TCP/IP stack.
std::unique_ptr<openlcb::SimpleCanStack> stack;
std::unique_ptr<Esp32WiFiManager> wifi_manager;

#if CONFIG_TWAI_ENABLED
Esp32Twai twai("/dev/twai", CONFIG_TWAI_RX_PIN, CONFIG_TWAI_TX_PIN);

static void twai_init_task(void *param)
{
  auto stack = static_cast<openlcb::SimpleCanStack *>(param);
  twai.hw_init();
  stack->add_can_port_select("/dev/twai/twai0");
  vTaskDelete(nullptr);
}
#endif // CONFIG_TWAI_ENABLED

std::unique_ptr<AutoSyncFileFlow> config_sync;
std::unique_ptr<FactoryResetHelper> factory_reset_helper;
std::unique_ptr<esp32olcbhub::DelayRebootHelper> delayed_reboot;
std::unique_ptr<esp32olcbhub::ConfigUpdateHelper> config_helper;
std::unique_ptr<esp32olcbhub::NodeRebootHelper> node_reboot_helper;
std::unique_ptr<esp32olcbhub::HealthMonitor> health_mon;

openlcb::SimpleCanStack *prepare_openlcb_stack(node_config_t *config, bool reset_events)
{
    // Create the LCC stack.
    stack.reset(new openlcb::SimpleCanStack(config->node_id));

    stack->set_tx_activity_led(LED_ACTIVITY_Pin::instance());

    // If the wifi mode is enabled start the wifi manager and httpd.
    if (config->wifi_mode > WIFI_MODE_NULL && config->wifi_mode < WIFI_MODE_MAX)
    {
        // if the wifi mode is not SoftAP and we do not have a station SSID
        // force reset to SoftAP only.
        if (config->wifi_mode != WIFI_MODE_AP && strlen(config->sta_ssid) == 0)
        {
            reset_wifi_config_to_softap(config);
        }

        wifi_manager.reset(
            new Esp32WiFiManager(config->wifi_mode != WIFI_MODE_AP ? config->sta_ssid : config->ap_ssid
                               , config->wifi_mode != WIFI_MODE_AP ? config->sta_pass : config->ap_pass
                               , stack.get()
                               , cfg.seg().wifi()
                               , config->hostname_prefix
                               , config->wifi_mode
                               , nullptr // TODO add config.sta_ip
                               , ip_addr_any
                               , 1
                               , config->ap_auth
                               , config->ap_ssid
                               , config->ap_pass));

        wifi_manager->wait_for_ssid_connect(config->sta_wait_for_connect);
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

#if CONFIG_OLCB_PRINT_ALL_PACKETS
    stack->print_all_packets();
#endif

#if CONFIG_TWAI_ENABLED
    // Initialize the TWAI driver from core 1 to ensure the TWAI driver is
    // tied to the core that the OpenMRN stack is *NOT* running on.
    xTaskCreatePinnedToCore(twai_init_task, "twai-init", 2048, stack.get()
                            , config_arduino_openmrn_task_priority(), nullptr
                            , APP_CPU_NUM);
#endif // CONFIG_TWAI_ENABLED

    // Initialize the factory reset helper.
    factory_reset_helper.reset(new FactoryResetHelper(config->node_id, cfg));

    // hook for delayed rebooter.
    delayed_reboot.reset(new esp32olcbhub::DelayRebootHelper(stack->service()));

    config_helper.reset(
        new esp32olcbhub::ConfigUpdateHelper(stack->executor()
                                           , stack->config_service()));

    health_mon.reset(new esp32olcbhub::HealthMonitor(stack->service()));

    // Create / update CDI, if the CDI is out of date a factory reset will be
    // forced.
    if (CDIHelper::create_config_descriptor_xml(cfg, openlcb::CDI_FILE
                                              , stack.get()))
    {
        LOG(WARNING, "[CDI] Forcing factory reset due to CDI update");
        unlink(openlcb::CONFIG_FILENAME);
    }

    // Create config file and initiate factory reset if it doesn't exist or is
    // otherwise corrupted.
    int config_fd =
        stack->create_config_file_if_needed(cfg.seg().internal_config()
                                          , CDI_VERSION
                                          , openlcb::CONFIG_FILE_SIZE);

    if (reset_events)
    {
        LOG(WARNING, "[CDI] Resetting event IDs");
        stack->factory_reset_all_events(cfg.seg().internal_config()
                                      , config->node_id, config_fd);
        fsync(config_fd);
    }

    // Create auto-sync hook since LittleFS will not persist the config until
    // fflush or file close.
    // NOTE: This can be removed if/when OpenMRN issues an fsync() as part of
    // processing MemoryConfigDefs::COMMAND_UPDATE_COMPLETE.
    config_sync.reset(
        new AutoSyncFileFlow(stack->service(), config_fd
                           , SEC_TO_USEC(CONFIG_OLCB_CONFIG_SYNC_SEC)));

    // Configure the node reboot helper to allow safe shutdown of file handles
    // and file systems etc.
    node_reboot_helper.reset(
        new esp32olcbhub::NodeRebootHelper(stack.get(), config_fd
                                         , config_sync.get()));

    // Initialize the webserver after the config file has been created/opened.
    if (config->wifi_mode > WIFI_MODE_NULL && config->wifi_mode < WIFI_MODE_MAX)
    {
        init_webserver(config, config_fd);
    }

    return stack.get();
}