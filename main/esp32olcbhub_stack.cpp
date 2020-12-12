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
#include "FactoryResetHelper.hxx"
#include "fs.hxx"
#include "hardware.hxx"
#include "HealthMonitor.hxx"
#include "NodeRebootHelper.hxx"
#include "nvs_config.hxx"
#include "web_server.hxx"

#include <CDIHelper.hxx>
#include <freertos_drivers/esp32/Esp32WiFiManager.hxx>
#include <freertos_drivers/esp32/Esp32HardwareTwai.hxx>
#include <openlcb/SimpleStack.hxx>
#include <utils/constants.hxx>
#include <utils/format_utils.hxx>
#include <utils/Uninitialized.hxx>

static esp32olcbhub::ConfigDef cfg(0);

namespace openlcb
{
    // pre-generated CDI data.
    const char CDI_DATA[] = R"xmlpayload(<?xml version="1.0"?>
<cdi xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:noNamespaceSchemaLocation="http://openlcb.org/schema/cdi/1/1/cdi.xsd">
<identification>
<manufacturer>http://atanisoft.github.io/esp32olcbhub</manufacturer>
<model>ESP32OlcbHub</model>
<hardwareVersion>1.0</hardwareVersion>
<softwareVersion>1.0</softwareVersion>
</identification>
<acdi/>
<segment space='251' origin='1'>
<string size='63'>
<name>User Name</name>
<description>This name will appear in network browsers for the current node.</description>
</string>
<string size='64'>
<name>User Description</name>
<description>This description will appear in network browsers for the current node.</description>
</string>
</segment>
<segment space='253' origin='128'>
<group>
<name>Internal data</name>
<description>Do not change these settings.</description>
<int size='2'>
<name>Version</name>
</int>
<int size='2'>
<name>Next event ID</name>
</int>
</group>
<group>
<name>WiFi Configuration</name>
<int size='1'>
<name>WiFi mode</name>
<description>Configures the WiFi operating mode.</description>
<min>0</min>
<max>3</max>
<default>2</default>
<map><relation><property>0</property><value>Off</value></relation><relation><property>1</property><value>Station Only</value></relation><relation><property>2</property><value>SoftAP Only</value></relation><relation><property>3</property><value>SoftAP and Station</value></relation></map>
</int>
<string size='21'>
<name>Hostname prefix</name>
<description>Configures the hostname prefix used by the node.
Note: the node ID will be appended to this value.</description>
</string>
<string size='32'>
<name>Station SSID</name>
<description>Configures the SSID that the ESP32 will connect to.</description>
</string>
<string size='128'>
<name>Station password</name>
<description>Configures the password that the ESP32 will use for the station SSID.</description>
</string>
<string size='32'>
<name>SoftAP SSID</name>
<description>Configures the SSID that the ESP32 will use for the SoftAP.</description>
</string>
<string size='128'>
<name>SoftAP assword</name>
<description>Configures the password that the ESP32 will use for the SoftAP.</description>
</string>
<int size='1'>
<name>Authentication Mode</name>
<description>Configures the authentication mode of the SoftAP.</description>
<min>0</min>
<max>7</max>
<default>3</default>
<map><relation><property>0</property><value>Open</value></relation><relation><property>1</property><value>WEP</value></relation><relation><property>2</property><value>WPA</value></relation><relation><property>3</property><value>WPA2</value></relation><relation><property>4</property><value>WPA/WPA2</value></relation><relation><property>6</property><value>WPA3</value></relation><relation><property>7</property><value>WPA2/WPA3</value></relation></map>
</int>
<int size='1'>
<name>WiFi Channel</name>
<description>Configures the WiFi channel to use for the SoftAP.
Note: Some channels overlap eachother and may not provide optimal performance.Recommended channels are: 1, 6, 11 since these do not overlap.</description>
<min>1</min>
<max>14</max>
<default>1</default>
</int>
<int size='1'>
<name>Enable SNTP</name>
<description>Enabling this option will allow the ESP32 to poll an SNTP server at regular intervals to obtain the current time. The refresh interval roughly once per hour.</description>
<min>0</min>
<max>1</max>
<default>0</default>
<map><relation><property>0</property><value>Disabled</value></relation><relation><property>1</property><value>Enabled</value></relation></map>
</int>
<string size='64'>
<name>SNTP Server</name>
<description>Enter the SNTP Server address. Example: pool.ntp.org
Most of the time this does not need to be changed.</description>
</string>
<string size='64'>
<name>TimeZone</name>
<description>This is the timezone that the ESP32 should use, note it must be in POSIX notation. Note: The timezone is only configured when SNTP is also enabled.
A few common values:
PST8PDT,M3.2.0,M11.1.0 -- UTC-8 with automatic DST adjustment
MST7MDT,M3.2.0,M11.1.0 -- UTC-7 with automatic DST adjustment
CST6CDT,M3.2.0,M11.1.0 -- UTC-6 with automatic DST adjustment
EST5EDT,M3.2.0,M11.1.0 -- UTC-5 with automatic DST adjustment
A complete list can be seen here in the second column:
https://github.com/nayarsystems/posix_tz_db/blob/master/zones.csv</description>
</string>
<group>
<name>Hub Configuration</name>
<description>Configuration settings for an OpenLCB Hub</description>
<int size='1'>
<name>Enable</name>
<description>Configures this node as an OpenLCB hub which can accept connections from other nodes.
NOTE: This may cause some instability as the number of connected nodes increases.</description>
<min>0</min>
<max>1</max>
<default>0</default>
<map><relation><property>0</property><value>Disabled</value></relation><relation><property>1</property><value>Enabled</value></relation></map>
</int>
<int size='2'>
<name>Hub Listener Port</name>
<description>Defines the TCP/IP listener port this node will use when operating as a hub. Most of the time this does not need to be changed.</description>
<min>1</min>
<max>65535</max>
<default>12021</default>
</int>
<string size='64'>
<name>mDNS Service</name>
<description>mDNS or Bonjour service name, such as _openlcb-can._tcp</description>
</string>
<group offset='6'/>
</group>
<group>
<name>Uplink Configuration</name>
<description>Configures how this node will connect to other nodes.</description>
<int size='1'>
<name>Enable</name>
<description>Enables connecting to an OpenLCB Hub. In some cases it may be desirable to disable the uplink, such as a CAN only configuration.</description>
<min>0</min>
<max>1</max>
<default>1</default>
<map><relation><property>0</property><value>Disabled</value></relation><relation><property>1</property><value>Enabled</value></relation></map>
</int>
<string size='64'>
<name>mDNS Service</name>
<description>mDNS or Bonjour service name, such as _openlcb-can._tcp</description>
</string>
<string size='64'>
<name>IP Address</name>
<description>Enter the server IP address. Example: 192.168.0.55
Note: This will be used as a fallback when mDNS lookup is not successful.</description>
</string>
<int size='2'>
<name>Port Number</name>
<description>TCP port number of the server. Most of the time this does not need to be changed.</description>
<min>1</min>
<max>65535</max>
<default>12021</default>
</int>
</group>
<int size='1'>
<name>WiFi Power Savings Mode</name>
<description>When enabled this allows the ESP32 WiFi radio to use power savings mode which puts the radio to sleep except to receive beacon updates from the connected SSID. This should generally not need to be enabled unless you are powering the ESP32 from a battery.</description>
<min>0</min>
<max>1</max>
<default>0</default>
<map><relation><property>0</property><value>Disabled</value></relation><relation><property>1</property><value>Enabled</value></relation></map>
</int>
<int size='1'>
<name>WiFi Transmit Power</name>
<description>WiFi Radio transmit power in dBm. This can be used to limit the WiFi range. This option generally does not need to be changed.
NOTE: Setting this option to a very low value can cause communication failures.</description>
<min>8</min>
<max>78</max>
<default>78</default>
<map><relation><property>8</property><value>2 dBm</value></relation><relation><property>20</property><value>5 dBm</value></relation><relation><property>28</property><value>7 dBm</value></relation><relation><property>34</property><value>8 dBm</value></relation><relation><property>44</property><value>11 dBm</value></relation><relation><property>52</property><value>13 dBm</value></relation><relation><property>56</property><value>14 dBm</value></relation><relation><property>60</property><value>15 dBm</value></relation><relation><property>66</property><value>16 dBm</value></relation><relation><property>72</property><value>18 dBm</value></relation><relation><property>78</property><value>20 dBm</value></relation></map>
</int>
<int size='1'>
<name>Wait for successful SSID connection</name>
<description>Enabling this option will cause the node to restart when there is a failure (or timeout) during the SSID connection process.</description>
<min>0</min>
<max>1</max>
<default>1</default>
<map><relation><property>0</property><value>Disabled</value></relation><relation><property>1</property><value>Enabled</value></relation></map>
</int>
</group>
</segment>
<segment space='253'>
<name>Version information</name>
<int size='1'>
<name>ACDI User Data version</name>
<description>Set to 2 and do not change.</description>
</int>
</segment>
</cdi>
)xmlpayload";
    extern const size_t CDI_SIZE;
    const size_t CDI_SIZE = sizeof(CDI_DATA);

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

extern "C" void enter_bootloader()
{
    node_config_t config;
    if (load_config(&config) != ESP_OK)
    {
        default_config(&config);
    }
    config.bootloader_req = true;
    save_config(&config);
    LOG(INFO, "[Bootloader] Rebooting into bootloader");
    reboot();
}

uninitialized<openlcb::SimpleCanStack> stack;
uninitialized<Esp32WiFiManager> wifi_manager;
uninitialized<FactoryResetHelper> factory_reset_helper;
uninitialized<esp32olcbhub::DelayRebootHelper> delayed_reboot;
uninitialized<esp32olcbhub::NodeRebootHelper> node_reboot_helper;
uninitialized<esp32olcbhub::HealthMonitor> health_mon;
Esp32HardwareTwai twai(TWAI_RX_PIN, TWAI_TX_PIN);

#ifndef CONFIG_WIFI_STATION_SSID
#define CONFIG_WIFI_STATION_SSID ""
#endif

#ifndef CONFIG_WIFI_STATION_PASSWORD
#define CONFIG_WIFI_STATION_PASSWORD ""
#endif

#ifndef CONFIG_WIFI_SOFTAP_SSID
#define CONFIG_WIFI_SOFTAP_SSID "esp32olcbhub"
#endif

#ifndef CONFIG_WIFI_SOFTAP_PASSWORD
#define CONFIG_WIFI_SOFTAP_PASSWORD "esp32olcbhub"
#endif

#ifndef CONFIG_WIFI_RESTART_ON_SSID_CONNECT_FAILURE
#define CONFIG_WIFI_RESTART_ON_SSID_CONNECT_FAILURE 0
#endif

#ifndef WIFI_HOSTNAME_PREFIX
#define WIFI_HOSTNAME_PREFIX "esp32olcbhub_"
#endif

#ifndef CONFIG_WIFI_SOFTAP_CHANNEL
#define CONFIG_WIFI_SOFTAP_CHANNEL 1
#endif

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

    wifi_manager.emplace(stack.get_mutable(), cfg.seg().wifi()
                       , (wifi_mode_t)CONFIG_WIFI_MODE
                       , WIFI_HOSTNAME_PREFIX
                       , CONFIG_WIFI_STATION_SSID
                       , CONFIG_WIFI_STATION_PASSWORD
                       , nullptr        /* default to DHCP */
                       , ip_addr_any    /* default to DHCP DNS */
                       , CONFIG_WIFI_SOFTAP_SSID
                       , CONFIG_WIFI_SOFTAP_PASSWORD
                       , CONFIG_WIFI_SOFTAP_CHANNEL
                       , nullptr        /* default SoftAP IP */
                       , CONFIG_SNTP_SERVER, CONFIG_TIMEZONE
#if CONFIG_SNTP
                       , true);
#else
                       , false);
#endif
    wifi_manager->set_status_led(LED_WIFI_Pin::instance());

    // Initialize the factory reset helper.
    factory_reset_helper.emplace(cfg, config->node_id);

    // hook for delayed rebooter.
    delayed_reboot.emplace(stack->service());
    health_mon.emplace(stack.get_mutable());

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

    // Configure the node reboot helper to allow safe shutdown of file handles
    // and file systems etc.
    node_reboot_helper.emplace(stack.get_mutable(), config_fd);

    init_webserver(config_fd, stack.get_mutable());

#if CONFIG_OLCB_ENABLE_TWAI
    if (!config->disable_twai)
    {
        stack->executor()->add(new CallbackExecutable([]
        {
            // Initialize the TWAI driver
            twai.hw_init();
            stack->add_can_port_async("/dev/twai/twai0");
        }));
    }
#endif // CONFIG_OLCB_ENABLE_TWAI

    if (brownout_detected)
    {
        // Queue the brownout event to be sent.
        stack->executor()->add(new CallbackExecutable([]()
        {
            LOG_ERROR("[Brownout] Detected a brownout reset, sending event");
            stack->send_event(openlcb::Defs::NODE_POWER_BROWNOUT_EVENT);
        }));
    }

    // Start the stack in the background using it's own task.
    stack->start_executor_thread("OpenMRN"
                               , config_arduino_openmrn_task_priority()
                               , config_arduino_openmrn_stack_size());
}