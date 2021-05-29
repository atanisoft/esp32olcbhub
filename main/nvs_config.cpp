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
 * \file nvs_config.cpp
 *
 * NVS based configuration management for the ESP32OlcbHub.
 *
 * @author Mike Dunston
 * @date 4 July 2020
 */

#include "sdkconfig.h"
#include "nvs_config.hxx"
#include <nvs.h>
#include <nvs_flash.h>
#include <string>

// TODO: adjust format_utils.hxx not to require this line here.
using std::string;

#include <utils/format_utils.hxx>
#include <utils/logging.h>

/// NVS Persistence namespace.
static constexpr char NVS_NAMESPACE[] = "node";

/// NVS Persistence key.
static constexpr char NVS_CFG_KEY[] = "cfg";

esp_err_t load_config(node_config_t *config)
{
    LOG(INFO, "[NVS] Loading configuration");
    // load non-CDI based config from NVS
    nvs_handle_t nvs;
    size_t size = sizeof(node_config_t);
    esp_err_t res =
        ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_open(NVS_NAMESPACE, NVS_READWRITE
                                             , &nvs));
    if (res != ESP_OK)
    {
        LOG_ERROR("[NVS] Configuration load failed: %s (%d)"
                , esp_err_to_name(res), res);
        return res;
    }
    res = nvs_get_blob(nvs, NVS_CFG_KEY, config, &size);
    nvs_close(nvs);

    // if the size read in is not as expected reset the result code to failure.
    if (size != sizeof(node_config_t))
    {
        LOG_ERROR("[NVS] Configuration load failed (loaded size incorrect: "
                  "%zu vs %zu)", size, sizeof(node_config_t));
        res = ESP_FAIL;
    }
    return res;
}

esp_err_t save_config(node_config_t *config)
{
    nvs_handle_t nvs;
    esp_err_t res = ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs));
    if (res != ESP_OK)
    {
        LOG_ERROR("[NVS] Configuration save failed: %s (%d)", esp_err_to_name(res), res);
        return res;
    }
    res = ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_set_blob(nvs, NVS_CFG_KEY, config, sizeof(node_config_t)));
    if (res != ESP_OK)
    {
        LOG_ERROR("[NVS] Configuration save failed: %s (%d)", esp_err_to_name(res), res);
        return res;
    }
    res = ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_commit(nvs));
    nvs_close(nvs);
    if (res != ESP_OK)
    {
        LOG_ERROR("[NVS] Commit failed: %s (%d)", esp_err_to_name(res), res);
    }
    return res;
}

#ifndef CONFIG_OLCB_NODE_ID
#define CONFIG_OLCB_NODE_ID 0x050201030000
#endif

#ifndef CONFIG_WIFI_MODE
#define CONFIG_WIFI_MODE WIFI_MODE_AP
#endif

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

#ifndef CONFIG_WIFI_SOFTAP_AUTH
#define CONFIG_WIFI_SOFTAP_AUTH WIFI_AUTH_OPEN
#endif

#ifndef CONFIG_WIFI_HOSTNAME_PREFIX
#define CONFIG_WIFI_HOSTNAME_PREFIX "esp32olcbhub_"
#endif

#ifndef CONFIG_WIFI_SOFTAP_CHANNEL
#define CONFIG_WIFI_SOFTAP_CHANNEL 1
#endif

#ifndef CONFIG_SNTP_SERVER
#define CONFIG_SNTP_SERVER "pool.ntp.org"
#endif

#ifndef CONFIG_TIMEZONE
#define CONFIG_TIMEZONE "UTC0"
#endif

esp_err_t default_config(node_config_t *config)
{
    LOG(INFO, "[NVS] Initializing default configuration");
    memset(config, 0, sizeof(node_config_t));
    config->node_id = CONFIG_OLCB_NODE_ID;
    config->wifi_mode = (wifi_mode_t)CONFIG_WIFI_MODE;
    str_populate(config->hostname_prefix, CONFIG_WIFI_HOSTNAME_PREFIX);
    str_populate(config->station_ssid, CONFIG_WIFI_STATION_SSID);
    str_populate(config->station_pass, CONFIG_WIFI_STATION_PASSWORD);
    str_populate(config->softap_ssid, CONFIG_WIFI_SOFTAP_SSID);
    str_populate(config->softap_pass, CONFIG_WIFI_SOFTAP_PASSWORD);
    config->softap_auth = (wifi_auth_mode_t)CONFIG_WIFI_SOFTAP_AUTH;
#if defined(CONFIG_SNTP)
    config->sntp_enabled = true;
#else
    config->sntp_enabled = false;
#endif
    str_populate(config->sntp_server, CONFIG_SNTP_SERVER);
    str_populate(config->timezone, CONFIG_TIMEZONE);

    return save_config(config);
}

void nvs_init()
{
    // Initialize NVS before we do any other initialization as it may be
    // internally used by various components even if we disable it's usage in
    // the WiFi connection stack.
    LOG(INFO, "[NVS] Initializing NVS");
    if (ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_flash_init()) == ESP_ERR_NVS_NO_FREE_PAGES)
    {
        const esp_partition_t* partition =
        esp_partition_find_first(ESP_PARTITION_TYPE_DATA
                               , ESP_PARTITION_SUBTYPE_DATA_NVS, NULL);
        if (partition != NULL)
        {
            LOG(INFO, "[NVS] Erasing partition %s...", partition->label);
            ESP_ERROR_CHECK(esp_partition_erase_range(partition, 0,
                                                      partition->size));
            ESP_ERROR_CHECK(nvs_flash_init());
        }
    }
}

void dump_config(node_config_t *config)
{
    // display current configuration settings.
    LOG(INFO, "[NVS] Node ID: %s",
        uint64_to_string_hex(config->node_id).c_str());
    LOG(INFO, "[NVS] WiFi configuration:");
    LOG(INFO, "Mode: %s (%d)",
        config->wifi_mode == WIFI_MODE_NULL ? "Off" :
        config->wifi_mode == WIFI_MODE_AP ? "SoftAP" :
        config->wifi_mode == WIFI_MODE_STA ? "Station" : "Station / SoftAP",
        config->wifi_mode);
    LOG(INFO, "Hostname Prefix: %s", config->hostname_prefix);
    if (config->wifi_mode == WIFI_MODE_STA ||
        config->wifi_mode == WIFI_MODE_APSTA)
    {
        LOG(INFO, "Station SSID: %s", config->station_ssid);
    }
    if (config->wifi_mode == WIFI_MODE_AP ||
        config->wifi_mode == WIFI_MODE_APSTA)
    {
        LOG(INFO, "SoftAP SSID: %s", config->softap_ssid);
        LOG(INFO, "SoftAP Auth: %s (%d)",
            config->softap_auth == WIFI_AUTH_OPEN ? "Open" :
            config->softap_auth == WIFI_AUTH_WEP ? "WEP" :
            config->softap_auth == WIFI_AUTH_WPA_PSK ? "WPA" :
            config->softap_auth == WIFI_AUTH_WPA2_PSK ? "WPA2" :
            config->softap_auth == WIFI_AUTH_WPA_WPA2_PSK ? "WPA/WPA2" :
            config->softap_auth == WIFI_AUTH_WPA3_PSK ? "WPA3" : "WPA2/WPA3",
            config->softap_auth);
    }
    if (config->sntp_enabled)
    {
        LOG(INFO, "SNTP: %s / %s", config->sntp_server, config->timezone);
    }
    else
    {
        LOG(INFO, "SNTP: Off");
    }
}

bool set_node_id(uint64_t node_id)
{
    node_config_t config;
    load_config(&config);
    LOG(INFO, "[NVS] Setting Node ID to: %s"
      , uint64_to_string_hex(node_id).c_str());
    config.node_id = node_id;
    config.force_reset = true;
    return save_config(&config) == ESP_OK;
}

bool force_factory_reset()
{
    node_config_t config;
    load_config(&config);
    config.force_reset = true;

    return save_config(&config) == ESP_OK;
}
