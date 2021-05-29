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
 * \file esp32olcbhub_bootloader.cpp
 *
 * Firmware download bootloader support for the ESP32 IO Board.
 *
 * @author Mike Dunston
 * @date 30 November 2020
 */

#include "sdkconfig.h"
#include "hardware.hxx"

#include <bootloader_hal.h>
#include <freertos_drivers/esp32/Esp32BootloaderHal.hxx>

/// Variable used to indicate that the startup should go into the bootloader
/// rather than default startup mode.
static uint32_t RTC_NOINIT_ATTR bootloader_request;

static constexpr uint32_t BOOTLOADER_REQUEST_ENABLED = 1;
static constexpr uint32_t BOOTLOADER_REQUEST_DISABLED = 0;


extern "C"
{

void enter_bootloader()
{
    bootloader_request = BOOTLOADER_REQUEST_ENABLED;
    LOG(INFO, "[Bootloader] Rebooting into bootloader");
    reboot();
}

/// Initializes the node specific bootloader hardware (LEDs)
void bootloader_hw_set_to_safe(void)
{
    LOG(VERBOSE, "[Bootloader] bootloader_hw_set_to_safe");
    LED_WIFI_Pin::hw_init();
    LED_ACTIVITY_Pin::hw_init();
}

/// Verifies that the bootloader has been requested.
///
/// @return true if the bootloader has been requested, false otherwise.
bool request_bootloader(void)
{
    LOG(VERBOSE, "[Bootloader] request_bootloader");
    return bootloader_request == BOOTLOADER_REQUEST_ENABLED;
}

/// Updates the state of a status LED.
///
/// @param led is the LED to update.
/// @param value is the new state of the LED.
///
/// NOTE: Currently the following mapping is being used for the LEDs:
/// LED_ACTIVE -> Activity LED
/// LED_WRITING -> WiFi LED
/// LED_REQUEST -> Used only as a hook for printing bootloader startup.
void bootloader_led(enum BootloaderLed led, bool value)
{
    LOG(VERBOSE, "[Bootloader] bootloader_led(%d, %d)", led, value);
    if (led == LED_ACTIVE)
    {
        LED_ACTIVITY_Pin::instance()->write(value);
    }
    else if (led == LED_WRITING)
    {
        LED_WIFI_Pin::instance()->write(value);
    }
    else if (led == LED_REQUEST)
    {
        LOG(INFO, "[Bootloader] Preparing to receive firmware");
        LOG(INFO, "[Bootloader] Current partition: %s", current->label);
        LOG(INFO, "[Bootloader] Target partition: %s", target->label);
    }
}

} // extern "C"

/// Starts the ESP32 Bootloader "lean" stack.
///
/// @param id is the node identifier to use.
void start_bootloader_stack(uint64_t id)
{
    esp32_bootloader_run(id, TWAI_TX_PIN, TWAI_RX_PIN, false);
    bootloader_request = BOOTLOADER_REQUEST_DISABLED;
    esp_restart();
}

/// Initializes the ESP32 Bootloader request variable if the startup reason is
/// matching to a fresh startup.
///
/// @param reason is the SoC restart reason.
void initialize_bootloader_vars(uint8_t reason)
{
    // If this is the first power up of the node we need to reset the flag
    // since it will not be initialized automatically.
    if (reason == POWERON_RESET)
    {
        bootloader_request = BOOTLOADER_REQUEST_DISABLED;
    }
}

/// Sets the ESP32 Bootloader request flag as enabled.
void set_bootloader_requested()
{
    bootloader_request = BOOTLOADER_REQUEST_ENABLED;
}
