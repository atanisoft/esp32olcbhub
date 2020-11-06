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
 * OpenMRN Stack extensions initialization.
 *
 * @author Mike Dunston
 * @date 1 September 2020
 */

#include "sdkconfig.h"
#include "cdi.hxx"
#include "CDIHelper.hxx"
#include "constants.hxx"
#include "ConfigUpdateHelper.hxx"
#include "DelayRebootHelper.hxx"
#include "NodeRebootHelper.hxx"
#include "HealthMonitor.hxx"
#include "hardware.hxx"
#include "nvs_config.hxx"
#include "fs.hxx"
#include "web_server.hxx"

#include <esp_task.h>
#include <openlcb/EventHandlerTemplates.hxx>
#include <openlcb/SimpleStack.hxx>

esp32olcbhub::ConfigDef cfg(0);

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

// when the io board starts up the first time the config is blank and needs to
// be reset to factory settings.
class FactoryResetHelper : public DefaultConfigUpdateListener
{
public:
    FactoryResetHelper(uint64_t node_id) : nodeId_(node_id)
    {

    }
    UpdateAction apply_configuration(int fd, bool initial_load,
                                     BarrierNotifiable *done) override
    {
        // nothing to do here as we do not load config
        AutoNotify n(done);
        LOG(VERBOSE, "[CFG] apply_configuration(%d, %d)", fd, initial_load);
        return UPDATED;
    }

    void factory_reset(int fd) override
    {
        LOG(VERBOSE, "[CFG] factory_reset(%d)", fd);
        cfg.userinfo().name().write(fd, SNIP_PROJECT_NAME);
        string node_id = uint64_to_string_hex(nodeId_, 12);
        std::replace(node_id.begin(), node_id.end(), ' ', '0');
        inject_seperator<2, '.'>(node_id);
        cfg.userinfo().description().write(fd, node_id.c_str());
    }
private:
    uint64_t nodeId_;
};

std::unique_ptr<AutoSyncFileFlow> config_sync;
std::unique_ptr<FactoryResetHelper> factory_reset_helper;
std::unique_ptr<esp32olcbhub::DelayRebootHelper> delayed_reboot;
std::unique_ptr<esp32olcbhub::ConfigUpdateHelper> config_helper;
std::unique_ptr<esp32olcbhub::NodeRebootHelper> node_reboot_helper;
std::unique_ptr<esp32olcbhub::HealthMonitor> health_mon;

void initialize_openlcb_helpers(node_config_t config, openlcb::SimpleCanStack *stack)
{
    // Initialize the factory reset helper.
    factory_reset_helper.reset(new FactoryResetHelper(config.node_id));

    // hook for delayed rebooter.
    delayed_reboot.reset(new esp32olcbhub::DelayRebootHelper(stack->service()));

    config_helper.reset(
        new esp32olcbhub::ConfigUpdateHelper(stack->executor()
                                           , stack->config_service()));

    health_mon.reset(new esp32olcbhub::HealthMonitor(stack->service()));
}

void initialize_stack(node_config_t *config, openlcb::SimpleCanStack *stack
                    , bool reset_events)
{
    // Create / update CDI, if the CDI is out of date a factory reset will be
    // forced.
    if (CDIHelper::create_config_descriptor_xml(cfg, openlcb::CDI_FILE
                                              , stack))
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
    config_sync.reset(
        new AutoSyncFileFlow(stack->service(), config_fd
                           , CFG_AUTO_SYNC_INTERVAL));
    // create hook point for node reboot.
    node_reboot_helper.reset(
        new esp32olcbhub::NodeRebootHelper(stack, config_fd
                                         , config_sync.get()));

    // Initialize the webserver after the config file has been created/opened.
    if (config->wifi_mode > WIFI_MODE_NULL && config->wifi_mode < WIFI_MODE_MAX)
    {
        init_webserver(config, config_fd);
    }
}