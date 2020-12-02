# ESP32 OpenLCB Hub

This program creates an OpenLCB hub that runs on the ESP32. The hub mode will need to be enabled via configuration update through the web interface *OR* via CDI.

## GPIO pin usage

GPIO pins are configured via `idf.py menuconfig` and have the following default values:

| PIN | Usage |
| --- | ----- |
| 4 | CAN RX |
| 5 | CAN TX |
| 19 | I2C SDA |
| 21 | I2C SCL |
| 22 | WiFi LED (Active LOW) |
| 23 | Activity LED (Active LOW) |
| 36 | User Button |
| 39 | Factory Reset |

The WiFi LED will be ON (LOW) when the SoftAP is UP or the ESP32 has received an IP address from the configured SSID.

## Firmware updates via OpenLCB

The ESP32 OpenLCB Hub supports receiving firmware over the CAN interface from JMRI or other bootloader clients. The WiFi interface is not supported at this time.

## Building

The ESP32 OpenLCB Hub requires ESP-IDF v4.0 or later. When cloning the code be sure to clone recursively:
`git clone --recursive git@github.com:atanisoft/esp32olcbhub.git`. If you receive an error related to littlefs it is likely that the esp_littlefs dependencies are not present, to fix this navigate to `components/esp_littlefs` and execute `git submodule update --init --recursive`.
