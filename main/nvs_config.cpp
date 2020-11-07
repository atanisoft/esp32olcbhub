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

#include "constants.hxx"
#include "nvs_config.hxx"
#include <utils/format_utils.hxx>
#include <utils/logging.h>

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
    if (config->wifi_mode != WIFI_MODE_STA &&
        config->wifi_mode != WIFI_MODE_NULL &&
        config->ap_ssid[0] == '\0')
    {
        LOG_ERROR("[NVS] Configuration doesn't appear to be valid, AP SSID is "
                  "blank!");
        res = ESP_FAIL;
    }
    if (config->wifi_mode != WIFI_MODE_AP &&
        config->wifi_mode != WIFI_MODE_NULL &&
        config->sta_ssid[0] == '\0')
    {
        LOG_ERROR("[NVS] Configuration doesn't appear to be valid, Station "
                  "SSID is blank!");
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

esp_err_t default_config(node_config_t *config)
{
    LOG(INFO, "[NVS] Initializing default configuration");
    bzero(config, sizeof(node_config_t));
    config->node_id = DEFAULT_NODE_ID;
    config->wifi_mode = DEFAULT_WIFI_MODE;
    strcpy(config->sta_ssid, DEFAULT_SSID_NAME);
    strcpy(config->sta_pass, DEFAULT_SSID_PASS);
    strcpy(config->ap_ssid, DEFAULT_AP_NAME);
    strcpy(config->ap_pass, DEFAULT_AP_PASS);
    strcpy(config->hostname_prefix, DEFAULT_HOSTNAME_PREFIX);
    config->ap_auth = WIFI_AUTH_WPA2_PSK;
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
    uint8_t mac[6];
    LOG(INFO, "[NVS] Node ID: %s"
      , uint64_to_string_hex(config->node_id).c_str());
    switch(config->wifi_mode)
    {
        case WIFI_MODE_STA:
            bzero(&mac, 6);
            ESP_ERROR_CHECK_WITHOUT_ABORT(esp_read_mac(mac, ESP_MAC_WIFI_STA));
            LOG(INFO, "[NVS] WiFi mode: %d (Station)", config->wifi_mode);
            LOG(INFO, "[NVS] Station MAC: %s", mac_to_string(mac).c_str());
            LOG(INFO, "[NVS] Station SSID: %s", config->sta_ssid);
            break;
        case WIFI_MODE_AP:
            bzero(&mac, 6);
            ESP_ERROR_CHECK_WITHOUT_ABORT(
                esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP));
            LOG(INFO, "[NVS] WiFi mode: %d (SoftAP)", config->wifi_mode);
            LOG(INFO, "[NVS] SoftAP MAC: %s", mac_to_string(mac).c_str());
            LOG(INFO, "[NVS] SoftAP SSID: %s", config->ap_ssid);
            break;
        case WIFI_MODE_APSTA:
            bzero(&mac, 6);
            ESP_ERROR_CHECK_WITHOUT_ABORT(esp_read_mac(mac, ESP_MAC_WIFI_STA));
            LOG(INFO, "[NVS] WiFi mode: %d (Station + SoftAP)"
              , config->wifi_mode);
            LOG(INFO, "[NVS] Station MAC: %s", mac_to_string(mac).c_str());
            LOG(INFO, "[NVS] Station SSID: %s", config->ap_ssid);
            bzero(&mac, 6);
            ESP_ERROR_CHECK_WITHOUT_ABORT(
                esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP));
            LOG(INFO, "[NVS] SoftAP MAC: %s", mac_to_string(mac).c_str());
            LOG(INFO, "[NVS] SoftAP SSID: esp32s2io_%s"
              , uint64_to_string_hex(config->node_id).c_str());
            break;
        case WIFI_MODE_NULL:
        case WIFI_MODE_MAX:
            LOG(INFO, "[NVS] WiFi mode: %d (OFF)", config->wifi_mode);
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

bool reconfigure_wifi(wifi_mode_t mode, const string &ssid
                    , const string &password)
{
    if (ssid.length() > AP_SSID_PASS_LEN)
    {
        LOG_ERROR("[NVS] Requested SSID is longer than permitted: %zu (max:%d)"
                , ssid.length(), AP_SSID_PASS_LEN);
        return false;
    }
    if (password.length() > AP_SSID_PASS_LEN)
    {
        LOG_ERROR("[NVS] Requested PASSWORD is longer than permitted: %zu "
                  "(max:%d)", password.length(), AP_SSID_PASS_LEN);
        return false;
    }

    node_config_t config;
    load_config(&config);
    LOG(INFO, "[NVS] Setting wifi_mode to: %d (%s)", mode
      , mode == WIFI_MODE_NULL ? "Off" :
        mode == WIFI_MODE_STA ? "Station" :
        mode == WIFI_MODE_APSTA ? "Station + SoftAP" : "SoftAP");
    config.wifi_mode = mode;
    if (mode == WIFI_MODE_STA || mode == WIFI_MODE_APSTA)
    {
        LOG(INFO, "[NVS] Setting STATION ssid to: %s", ssid.c_str());
        strcpy(config.sta_ssid, ssid.c_str());
        strcpy(config.sta_pass, password.c_str());
    }
    else if (mode == WIFI_MODE_AP)
    {
        LOG(INFO, "[NVS] Setting AP ssid to: %s", ssid.c_str());
        strcpy(config.ap_ssid, ssid.c_str());
        strcpy(config.ap_pass, password.c_str());
    }
    return save_config(&config) == ESP_OK;
}

bool force_factory_reset()
{
    node_config_t config;
    load_config(&config);
    config.force_reset = true;

    return save_config(&config) == ESP_OK;
}

bool reset_wifi_config_to_softap(node_config_t *config)
{
    LOG(WARNING, "[NVS] Switching to SoftAP mode as the station SSID is blank!");
    config->wifi_mode = WIFI_MODE_AP;
    if (strlen(config->ap_ssid) == 0)
    {
        LOG(WARNING, "[NVS] SoftAP SSID is blank, resetting to %s"
          , DEFAULT_AP_NAME);
        strcpy(config->ap_ssid, DEFAULT_AP_NAME);
        strcpy(config->ap_pass, DEFAULT_AP_PASS);
    }
    return save_config(config) == ESP_OK;
}