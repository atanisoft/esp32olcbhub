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
 * \file nvs_config.hxx
 *
 * NVS based configuration management for the ESP32OlcbHub.
 *
 * @author Mike Dunston
 * @date 4 July 2020
 */

#ifndef NVS_CONFIG_HXX_
#define NVS_CONFIG_HXX_

#include <esp_err.h>
#include <esp_wifi_types.h>
#include <stdint.h>

typedef struct
{
    bool force_reset;
    uint64_t node_id;
    wifi_mode_t wifi_mode;
    char hostname_prefix[16];
    char station_ssid[33];
    char station_pass[33];
    char softap_ssid[33];
    char softap_pass[33];
    wifi_auth_mode_t softap_auth;
    uint8_t softap_channel;
    char sntp_server[33];
    char timezone[33];
    bool sntp_enabled;
    uint8_t reserved[29];
} node_config_t;

esp_err_t load_config(node_config_t *config);
esp_err_t save_config(node_config_t *config);
esp_err_t default_config(node_config_t *config);
void nvs_init();
void dump_config(node_config_t *config);
bool set_node_id(uint64_t node_id);
bool force_factory_reset();

#endif // NVS_CONFIG_HXX_