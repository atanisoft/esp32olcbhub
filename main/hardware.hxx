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

#include "sdkconfig.h"

/// WiFi Active indicator LED. Active (ON) Low.
GPIO_PIN(LED_WIFI, GpioOutputSafeHighInvert, CONFIG_LED_WIFI);

/// Activity indicator LED. Active (ON) Low.
GPIO_PIN(LED_ACTIVITY, GpioOutputSafeHighInvert, CONFIG_LED_STATUS);

#if CONFIG_FACTORY_RESET == GPIO_NUM_NC
/// Fake factory reset pin, it will always read high.
typedef DummyPinWithReadHigh FACTORY_RESET_Pin;
#else
/// Factory Reset Pin, pull LOW (GND) during startup to force reset of events
/// or all configuration based on how long it is held.
GPIO_PIN(FACTORY_RESET, GpioInputPU, CONFIG_FACTORY_RESET);
#endif // CONFIG_FACTORY_RESET == GPIO_NUM_NC

#if CONFIG_USER_BUTTON == GPIO_NUM_NC
/// Fake user input pin, it will always read high.
typedef DummyPinWithReadHigh USER_BUTTON_Pin;
#else
/// User Button Pin.
GPIO_PIN(USER_BUTTON, GpioInputPU, CONFIG_USER_BUTTON);
#endif // CONFIG_USER_BUTTON == GPIO_NUM_NC

/// GPIO Pin initializer.
typedef GpioInitializer<LED_WIFI_Pin, LED_ACTIVITY_Pin,
                        FACTORY_RESET_Pin, USER_BUTTON_Pin> GpioInit;

#ifndef CONFIG_TWAI_RX_PIN
#define CONFIG_TWAI_RX_PIN GPIO_NUM_4
#endif

#ifndef CONFIG_TWAI_TX_PIN
#define CONFIG_TWAI_TX_PIN GPIO_NUM_5
#endif

/// GPIO Pin connected to the TWAI (CAN) Transceiver RX pin.
static constexpr gpio_num_t TWAI_RX_PIN = (gpio_num_t)CONFIG_TWAI_RX_PIN;

/// GPIO Pin connected to the TWAI (CAN) Transceiver TX pin.
static constexpr gpio_num_t TWAI_TX_PIN = (gpio_num_t)CONFIG_TWAI_TX_PIN;

#endif // HARDWARE_HXX_