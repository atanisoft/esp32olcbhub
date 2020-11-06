# ESP32 OpenLCB Hub

This program creates an OpenLCB hub that runs on the ESP32. The hub mode will need to be enabled via configuration update through the web interface *OR* via CDI.

## Building

The ESP32 OpenLCB Hub requires ESP-IDF v4.0 or later. When checking out the code be sure to checkout recursively as:
`git clone --recursive git@github.com:atanisoft/esp32olcbhub.git`. If you receive an error related to littlefs it is likely that the esp_littlefs dependencies are not present, to fix this navigate to `components/esp_littlefs` and execute `git submodule update --init --recursive`.

## GPIO Pin connections

Pins are defined and configured in `main/constants.hxx` with the defaults below:
|PIN|Usage|
|---|-----|
| 4 | CAN RX |
| 5 | CAN TX |
| 21 | I2C SCL |
| 22 | WiFi LED (Active LOW) |
| 23 | I2C SDA |
| NC | Factory Reset |

The factory reset pin is not configured by default. The WiFi LED will be ON (LOW) when the SoftAP is UP or the ESP32 has received an IP address from the configured SSID.

If the CAN physical interface is not desired it can be disabled via `main/constants.hxx` by setting `TWAI_RX_PIN` and `TWAI_TX_PIN` to `GPIO_NUM_NC`.