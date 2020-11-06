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
 * \file constants.hxx
 *
 * Static constants for the ESP32OlcbHub.
 *
 * @author Mike Dunston
 * @date 4 July 2020
 */

#ifndef CONSTANTS_HXX_
#define CONSTANTS_HXX_

#include <esp_wifi.h>
#include <hal/gpio_types.h>
#include <os/os.h>
#include <stdint.h>

/// Enables the printing of all packets in the CanHub. This is not enabled by
/// default due to performance implications and should only be used for
/// debugging.
static constexpr bool ENABLE_PACKET_PRINTER = false;

/// Number of seconds to hold the Factory Reset button to force clear all
/// stored configuration data.
static constexpr uint8_t FACTORY_RESET_HOLD_TIME = 10;

/// Number of seconds to hold the Factory Reset button to force regeneration of
/// all Event IDs. NOTE: This will *NOT* clear WiFi configuration data.
static constexpr uint8_t FACTORY_RESET_EVENTS_HOLD_TIME = 5;

/// I2C SDA Pin.
static constexpr gpio_num_t I2C_SDA_PIN = GPIO_NUM_21;

/// I2C SCL Pin.
static constexpr gpio_num_t I2C_SCL_PIN = GPIO_NUM_23;

/// Pin connected to an active LOW LED that will be "ON" when the WiFi is
/// ready/available to be used.
static constexpr gpio_num_t LED_WIFI_PIN = GPIO_NUM_22;

/// Pin connected to the TWAI/CAN Transceiver RX pin.
static constexpr gpio_num_t TWAI_RX_PIN = GPIO_NUM_4;

/// Pin connected to the TWAI/CAN Transceiver TX pin.
static constexpr gpio_num_t TWAI_TX_PIN = GPIO_NUM_5;

/// Pin connected to the Factory Reset button, active LOW.
static constexpr gpio_num_t FACTORY_RESET_PIN = GPIO_NUM_NC;

/// NVS Persistence namespace.
static constexpr char NVS_NAMESPACE[] = "nodecfg";

/// NVS Persistence key.
static constexpr char NVS_CFG_KEY[] = "cfg";

/// Default Node ID that will be assigned upon Factory Reset.
static constexpr uint64_t DEFAULT_NODE_ID = 0x050201030000;

/// Default WiFi operating mode that will be assigned upon Factory Reset.
static constexpr wifi_mode_t DEFAULT_WIFI_MODE = WIFI_MODE_AP;

/// Default WiFi SoftAP name that will be assigned upon Factory Reset.
static constexpr char DEFAULT_AP_NAME[] = "esp32olcbhub";

/// Default WiFi SoftAP password that will be assigned upon Factory Reset.
static constexpr char DEFAULT_AP_PASS[] = "esp32olcbhub";

/// Default WiFi hostname prefix that will be assigned upon Factory Reset.
static constexpr char DEFAULT_HOSTNAME_PREFIX[] = "esp32olcbhub_";

/// Partition name for the persistent filesystem.
static constexpr char LITTLE_FS_PARTITION[] = "fs";

/// Mount point for the persistent filesystem.
static constexpr char LITTLE_FS_MOUNTPOINT[] = "/fs";

/// Automatic persistence interval for the node configuration file.
static constexpr uint64_t CFG_AUTO_SYNC_INTERVAL = SEC_TO_USEC(5);

/// Statically embedded index.html start location.
extern const uint8_t indexHtmlGz[] asm("_binary_index_html_gz_start");

/// Statically embedded index.html size.
extern const size_t indexHtmlGz_size asm("index_html_gz_length");

/// Statically embedded cash.js start location.
extern const uint8_t cashJsGz[] asm("_binary_cash_min_js_gz_start");

/// Statically embedded cash.js size.
extern const size_t cashJsGz_size asm("cash_min_js_gz_length");

/// Statically embedded milligram.min.css start location.
extern const uint8_t milligramMinCssGz[] asm("_binary_milligram_min_css_gz_start");

/// Statically embedded milligram.min.css size.
extern const size_t milligramMinCssGz_size asm("milligram_min_css_gz_length");

/// Statically embedded normalize.min.css start location.
extern const uint8_t normalizeMinCssGz[] asm("_binary_normalize_min_css_gz_start");

/// Statically embedded normalize.min.css size.
extern const size_t normalizeMinCssGz_size asm("normalize_min_css_gz_length");

/// Cative portal landing page.
static constexpr const char * const CAPTIVE_PORTAL_HTML = R"!^!(
<html>
 <head>
  <title>%s v%s</title>
  <meta http-equiv="refresh" content="30;url='/captiveauth'" />
 </head>
 <body>
  <h1>Welcome to the %s configuration portal</h1>
  <h2>Navigate to any website and the %s configuration portal will be presented.</h2>
  <p>If this dialog does not automatically close, please click <a href="/captiveauth">here</a>.</p>
 </body>
</html>)!^!";

#endif // CONSTANTS_HXX_