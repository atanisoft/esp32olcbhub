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
 * \file hardware.hxx
 *
 * Hardware representation for the ESP32OlcbHub.
 *
 * @author Mike Dunston
 * @date 4 July 2020
 */

#ifndef HARDWARE_HXX_
#define HARDWARE_HXX_

#include <freertos_drivers/arduino/DummyGPIO.hxx>
#include <freertos_drivers/esp32/Esp32Gpio.hxx>
#include <os/Gpio.hxx>
#include <utils/GpioInitializer.hxx>

#include "constants.hxx"

/// WiFi Active indicator LED. Active (ON) Low.
GPIO_PIN(LED_WIFI, GpioOutputSafeHighInvert, LED_WIFI_PIN);

/// Factory Reset Pin, pull LOW (GND) during startup to force reset of events
/// or all configuration based on how long it is held.
GPIO_PIN(FACTORY_RESET, GpioInputPU, FACTORY_RESET_PIN);

/// GPIO Pin initializer.
typedef GpioInitializer<LED_WIFI_Pin, FACTORY_RESET_Pin> GpioInit;

#endif // HARDWARE_HXX_