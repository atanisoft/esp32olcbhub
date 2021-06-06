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
#include "DelayRebootHelper.hxx"
#include "EventBroadcastHelper.hxx"
#include "FactoryResetHelper.hxx"
#include "fs.hxx"
#include "hardware.hxx"
#include "HealthMonitor.hxx"
#include "NodeRebootHelper.hxx"
#include "nvs_config.hxx"
#include "web_server.hxx"

#include <CDIXMLGenerator.hxx>
#include <freertos_drivers/esp32/Esp32HardwareTwai.hxx>
#include <freertos_drivers/esp32/Esp32WiFiManager.hxx>
#include <nmranet_config.h>
#include <openlcb/MemoryConfigClient.hxx>
#include <openlcb/SimpleStack.hxx>
#include <openlcb/SimpleNodeInfo.hxx>
#include <utils/constants.hxx>
#include <utils/format_utils.hxx>
#include <utils/Uninitialized.hxx>

namespace esp32olcbhub
{

static ConfigDef cfg(0);
}

namespace openlcb
{
    /// Name of CDI.xml to generate dynamically.
    const char CDI_FILENAME[] = "/fs/cdi.xml";

    // Path to where OpenMRN should persist general configuration data.
    const char *const CONFIG_FILENAME = "/fs/config";

    // The size of the memory space to export over the above device.
    const size_t CONFIG_FILE_SIZE =
        esp32olcbhub::cfg.seg().size() + esp32olcbhub::cfg.seg().offset();

    // Default to store the dynamic SNIP data is stored in the same persistant
    // data file as general configuration data.
    const char *const SNIP_DYNAMIC_FILENAME = "/fs/config";

    /// This will stop openlcb from exporting the CDI memory space upon start.
    extern const char CDI_DATA[] = "";

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
} // namespace openlcb

namespace esp32olcbhub
{

static int config_fd;
uninitialized<openlcb::SimpleCanStack> stack;
uninitialized<Esp32WiFiManager> wifi_manager;
uninitialized<openlcb::MemoryConfigClient> memory_client;
uninitialized<FactoryResetHelper> factory_reset_helper;
uninitialized<EventBroadcastHelper> event_helper;
uninitialized<DelayRebootHelper> delayed_reboot;
uninitialized<HealthMonitor> health_mon;
uninitialized<NodeRebootHelper> node_reboot_helper;
Esp32HardwareTwai twai(TWAI_RX_PIN, TWAI_TX_PIN);

void factory_reset_events()
{
    LOG(WARNING, "[CDI] Resetting event IDs");
    stack->factory_reset_all_events(cfg.seg().internal_config()
                                  , stack->node()->node_id(), config_fd);
    fsync(config_fd);
}

void NodeRebootHelper::reboot()
{
    // make sure we are not called from the executor thread otherwise there
    // will be a deadlock
    HASSERT(os_thread_self() != stack->executor()->thread_handle());
    //shutdown_webserver();
    LOG(INFO, "[Reboot] Shutting down LCC executor...");
    stack->executor()->sync_run([&]()
    {
        close(config_fd);
        unmount_fs();
        // restart the node
        LOG(INFO, "[Reboot] Restarting!");
        esp_restart();
    });
}

void EventBroadcastHelper::send_event(uint64_t eventID)
{
    stack->executor()->add(new CallbackExecutable([&]()
    {
        stack->send_event(eventID);
    }));
}

template<const unsigned num, const char separator>
void inject_seperator(std::string & input)
{
    for (auto it = input.begin(); (num + 1) <= std::distance(it, input.end());
        ++it)
    {
        std::advance(it, num);
        it = input.insert(it, separator);
    }
}

ConfigUpdateListener::UpdateAction FactoryResetHelper::apply_configuration(
    int fd, bool initial_load, BarrierNotifiable *done)
{
    // nothing to do here as we do not load config
    AutoNotify n(done);
    LOG(VERBOSE, "[CFG] apply_configuration(%d, %d)", fd, initial_load);

    return ConfigUpdateListener::UpdateAction::UPDATED;
}

void FactoryResetHelper::factory_reset(int fd)
{
    LOG(VERBOSE, "[CFG] factory_reset(%d)", fd);
    // set the name of the node to the SNIP model name
    cfg.userinfo().name().write(fd, openlcb::SNIP_STATIC_DATA.model_name);
    string node_id = uint64_to_string_hex(stack->node()->node_id(), 12);
    std::replace(node_id.begin(), node_id.end(), ' ', '0');
    inject_seperator<2, '.'>(node_id);
    // set the node description to the node id in expanded hex format.
    cfg.userinfo().description().write(fd, node_id.c_str());
}

void start_openlcb_stack(node_config_t *config, bool reset_events
                       , bool brownout_detected)
{
    LOG(INFO, "[SNIP] version:%d, manufacturer:%s, model:%s, hw-v:%s, sw-v:%s"
      , openlcb::SNIP_STATIC_DATA.version
      , openlcb::SNIP_STATIC_DATA.manufacturer_name
      , openlcb::SNIP_STATIC_DATA.model_name
      , openlcb::SNIP_STATIC_DATA.hardware_version
      , openlcb::SNIP_STATIC_DATA.software_version);

    // Create the LCC stack.
    stack.emplace(config->node_id);
    stack->set_tx_activity_led(LED_ACTIVITY_Pin::instance());
#if CONFIG_OLCB_PRINT_ALL_PACKETS
    stack->print_all_packets();
#endif

    // Create / update CDI, if the CDI is out of date a factory reset will be
    // forced.
    bool reset_cdi = CDIXMLGenerator::create_config_descriptor_xml(
        cfg, openlcb::CDI_FILENAME, stack.get_mutable());
    if (reset_cdi)
    {
        LOG(WARNING, "[CDI] Forcing factory reset due to CDI update");
        unlink(openlcb::CONFIG_FILENAME);
    }

    memory_client.emplace(stack->node(), stack->memory_config_handler());
    wifi_manager.emplace(config->station_ssid, config->station_pass,
                         stack.get_mutable(), cfg.seg().wifi(),
                         config->wifi_mode, config->hostname_prefix,
                         config->sntp_server, config->timezone,
                         config->sntp_enabled, config->softap_channel,
                         config->softap_auth, config->softap_ssid,
                         config->softap_pass);

    wifi_manager->set_status_led(LED_WIFI_Pin::instance());
    init_webserver(wifi_manager.get_mutable(), memory_client.get_mutable(), config->node_id);
    factory_reset_helper.emplace();
    event_helper.emplace();
    delayed_reboot.emplace(stack->service());
    health_mon.emplace(stack->service());
    node_reboot_helper.emplace();

    // Create config file and initiate factory reset if it doesn't exist or is
    // otherwise corrupted.
    int config_fd =
        stack->create_config_file_if_needed(cfg.seg().internal_config()
                                          , CDI_VERSION
                                          , openlcb::CONFIG_FILE_SIZE);

    // Initialize the TWAI driver
    twai.hw_init();
    stack->add_can_port_select("/dev/twai/twai0");

    if (reset_events)
    {
        LOG(WARNING, "[CDI] Resetting event IDs");
        stack->factory_reset_all_events(cfg.seg().internal_config()
                                      , config->node_id, config_fd);
        fsync(config_fd);
    }

    if (brownout_detected)
    {
        // Queue the brownout event to be sent.
        LOG_ERROR("[Brownout] Detected a brownout reset, sending event");
        event_helper->send_event(openlcb::Defs::NODE_POWER_BROWNOUT_EVENT);
    }

    // Start the stack in the background using it's own task.
    stack->start_executor_thread("OpenMRN", 0, 4096);
}

} // namespace esp32olcbhub