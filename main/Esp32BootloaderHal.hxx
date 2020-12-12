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
 * \file Esp32BootloaderHal.hxx
 *
 * ESP32 specific implementation of the HAL (Hardware Abstraction Layer) used
 * by the OpenLCB bootloader.
 * 
 * Additional functions from bootloader_hal.h will be required to be defined by
 * the application code.
 *
 * @author Mike Dunston
 * @date 1 December 2020
 */

#include "sdkconfig.h"

// Enable streaming support for the bootloader
#define BOOTLOADER_STREAM
// Set the buffer size to half the sector size to minimize the flash writes.
#define WRITE_BUFFER_SIZE (CONFIG_WL_SECTOR_SIZE / 2)

#if __has_include(<driver/twai.h>)
#include <driver/twai.h>
#else
#include <driver/can.h>
#endif
#include <esp_ota_ops.h>
#include <openlcb/Bootloader.hxx>
#include <utils/Hub.hxx>
#include <utils/constants.hxx>

/// Mapping of known ESP32 chip id values.
static constexpr const char * ESP_CHIP_ID[] =
{
    "ESP32",            // 0 ESP_CHIP_ID_ESP32 / CHIP_ESP32
    "INVALID",          // 1 invalid (placeholder)
    "ESP32-S2"          // 2 ESP_CHIP_ID_ESP32S2 / CHIP_ESP32S2
    "INVALID",          // 3 invalid (placeholder)
    "ESP32-S3"          // 4 ESP_CHIP_ID_ESP32S3 / CHIP_ESP32S3
};

/// Chip identifier for the currently running firmware.
static esp_chip_id_t chip_id = ESP_CHIP_ID_INVALID;

/// Currently running application header information.
static struct app_header node_app_header;

/// Node ID to use for the bootloader.
static uint64_t NODE_ID;

/// This is the currently running partition on the ESP32.
static esp_partition_t *current;

/// This is the target partition on the ESP32 to write the firmware to.
static esp_partition_t *target;

/// OTA handle used to track the firmware update progress.
static esp_ota_handle_t ota_handle = 0;

/// Maximum time to wait for a TWAI frame to be received or transmitted before
/// giving up.
static constexpr BaseType_t MAX_TWAI_WAIT = pdMS_TO_TICKS(250);

/// Internal flag to indicate that we have initialized the TWAI peripheral and
/// should deinit it before exit.
static bool bootloader_twai_initialized = false;

#if __has_include(<driver/twai.h>)
/// TWAI driver timing configuration, 125kbps.
twai_timing_config_t t_config = TWAI_TIMING_CONFIG_125KBITS();

/// TWAI driver filter configuration, accept all frames.
twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

/// TWAI driver general configuration.
/// NOTE: tx_io, rx_io, tx_queue_len, rx_queue_len will be updated as part of
/// the init process.
twai_general_config_t g_config =
    TWAI_GENERAL_CONFIG_DEFAULT(TWAI_IO_UNUSED, TWAI_IO_UNUSED,
                                TWAI_MODE_NORMAL);
#else
/// CAN driver timing configuration, 125kbps.
can_timing_config_t t_config = CAN_TIMING_CONFIG_125KBITS();

/// CAN driver filter configuration, accept all frames.
can_filter_config_t f_config = CAN_FILTER_CONFIG_ACCEPT_ALL();

/// CAN driver general configuration.
/// NOTE: tx_io, rx_io, tx_queue_len, rx_queue_len will be updated as part of
/// the init process.
can_general_config_t can_general_config =
{
    .mode = CAN_MODE_NORMAL,
    .tx_io = (gpio_num_t)CAN_IO_UNUSED,
    .rx_io = (gpio_num_t)CAN_IO_UNUSED,
    .clkout_io = (gpio_num_t)CAN_IO_UNUSED,
    .bus_off_io = (gpio_num_t)CAN_IO_UNUSED,
    .tx_queue_len = 0,
    .rx_queue_len = 0,
    .alerts_enabled = CAN_ALERT_NONE,
    .clkout_divider = 0
};

// Compatibility defines mapping the new TWAI APIs to the old CAN APIs.
#define twai_driver_install can_driver_install
#define twai_start can_start
#define twai_driver_uninstall can_driver_uninstall
#define twai_stop can_stop
#define twai_transmit can_transmit
#define twai_receive can_receive
#define twai_message_t can_message_t
#endif // __has_include(<driver/twai.h>)

extern "C"
{

/// Initializes the TWAI device driver for the ESP32.
void bootloader_hw_init(void)
{
    LOG(VERBOSE, "[Bootloader] Configuring TWAI driver");
    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    LOG(VERBOSE, "[Bootloader] Starting TWAI driver");
    ESP_ERROR_CHECK(twai_start());
    bootloader_twai_initialized = true;
}

/// Bootloader HAL callback to transfer control to the default operating mode
/// of the node.
///
/// NOTE: Currently this will force a restart of the ESP32.
void application_entry(void)
{
    LOG(VERBOSE, "[Bootloader] application_entry");

    // restart the esp32 since we do not have a way to rerun app_main.
    esp_restart();
}

/// Bootloader HAL callback indicating that a reboot has been requested.
///
/// NOTE: This is currently a no-op on the ESP32 since the restart will be
/// handled either in @ref esp32_bootloader_run or by the node itself.
void bootloader_reboot(void)
{
    LOG(VERBOSE, "[Bootloader] Reboot requested");
}

/// Attempts to read one CAN frame from the TWAI device driver.
///
/// @param frame is the holder for the received CAN frame.
///
/// @returns true if a frame was received, false otherwise.
bool read_can_frame(struct can_frame *frame)
{
    twai_message_t rx_msg;
    bzero(&rx_msg, sizeof(twai_message_t));
    if (twai_receive(&rx_msg, MAX_TWAI_WAIT) == ESP_OK)
    {
        LOG(VERBOSE, "[Bootloader] CAN_RX");
        frame->can_id = rx_msg.identifier;
        frame->can_dlc = rx_msg.data_length_code;
        frame->can_err = 0;
        frame->can_eff = rx_msg.extd;
        frame->can_rtr = rx_msg.rtr;
        memcpy(frame->data, rx_msg.data, frame->can_dlc);
        return true;
    }
    return false;
}

/// Attempts to transmit one CAN frame to the TWAI device driver.
///
/// @param frame is the CAN frame to transmit.
///
/// @returns true if a frame was transmitted, false otherwise.
bool try_send_can_frame(const struct can_frame &frame)
{
    twai_message_t tx_msg;
    bzero(&tx_msg, sizeof(twai_message_t));
    tx_msg.identifier = frame.can_id;
    tx_msg.data_length_code = frame.can_dlc;
    tx_msg.extd = frame.can_eff;
    tx_msg.rtr = frame.can_rtr;
    memcpy(tx_msg.data, frame.data, frame.can_dlc);
    if (twai_transmit(&tx_msg, MAX_TWAI_WAIT) == ESP_OK)
    {
        LOG(VERBOSE, "[Bootloader] CAN_TX");
        return true;
    }
    return false;
}

/// Obtains the flash size for the bootloader HAL.
/// 
/// @param flash_min is the minimum address for the flash, defaults to zero.
/// @param flash_max is the last valid address for the flash, defaults to the
/// currently running partition size.
/// @param app_header is the application metadata header, currently unused.
void get_flash_boundaries(const void **flash_min, const void **flash_max,
    const struct app_header **app_header)
{
    LOG(VERBOSE, "[Bootloader] get_flash_boundaries(%d,%d)", 0
      , node_app_header.app_size);
    *((uint32_t *)flash_min) = 0;
    *((uint32_t *)flash_max) = node_app_header.app_size;
    *app_header = &node_app_header;
}

/// Obtains the flash page information for the bootloader HAL.
///
/// @param address is the address in flash being worked with.
/// @param page_start is the aligned address calculated based on the flash page
/// size.
/// @param page_length_bytes is the size of the flash page, defaults to 4096
/// bytes.
void get_flash_page_info(
    const void *address, const void **page_start, uint32_t *page_length_bytes)
{
    uint32_t value = (uint32_t)address;
    value &= ~(CONFIG_WL_SECTOR_SIZE - 1);
    *page_start = (const void *)value;
    *page_length_bytes = CONFIG_WL_SECTOR_SIZE;
    LOG(VERBOSE, "[Bootloader] get_flash_page_info(%d, %d)", value
      , *page_length_bytes);
}

/// Erases a flash page.
///
/// @param address is the flash page address to be erased.
///
/// NOTE: This is a NO-OP on the ESP32 since the esp_ota_write API will
/// internally erase the flash prior to writing the new data.
void erase_flash_page(const void *address)
{
#if LOGLEVEL >= VERBOSE
    // NO OP -- handled automatically as part of write.
    uint32_t addr = (uint32_t)address;
    LOG(VERBOSE, "[Bootloader] Erase: %d", addr);
#endif
}

/// Writes a single block of received firmware data to flash.
///
/// @param address is the address to write the firmware data to.
/// @param data is the raw byte data to be written.
/// @param size_bytes is the number of bytes in @param data to be written.
///
/// NOTE: When @param address evaluates to zero (first page), the first block
/// of data is inspected to ensure it is a valid firmware image for the ESP32.
/// If the firmware received is not compatible with the currently running ESP32
/// this method will abort and reboot the esp32.
/// @TODO: add abort flag to openlcb/Bootloader.hxx to cleanly abort the
/// transfer rather than abrupt shutdown causing the remote upload client to
/// "hang" waiting for an ACK.
void write_flash(const void *address, const void *data, uint32_t size_bytes)
{
    uint32_t addr = (uint32_t)address;
    LOG(VERBOSE, "[Bootloader] Write: %d, %d", addr, size_bytes);
    if (addr == 0)
    {
        // The first part of the received binary should have the image header,
        // segment header and app description. These are used as a first pass
        // validation of the received data to ensure it is a valid firmware.
        bool should_abort = false;
        uint8_t *ptr = (uint8_t *)data;
        esp_image_header_t image_header;
        memcpy(&image_header, ptr, sizeof(esp_image_header_t));
        // advance the pointer to after the image header.
        ptr += sizeof(esp_image_header_t);
        // If the image magic is correct we can proceed with validating the
        // basic details of the image.
        if (image_header.magic == ESP_IMAGE_HEADER_MAGIC)
        {
            // advance the pointer to after the image segment header.
            ptr += sizeof(esp_image_segment_header_t);

            // copy the app description from the first segment
            esp_app_desc_t app_desc;
            memcpy(&app_desc, ptr, sizeof(esp_app_desc_t));
            // validate the image magic byte and chip type to
            // ensure it matches the currently running chip.
            if (image_header.chip_id != ESP_CHIP_ID_INVALID &&
                image_header.chip_id == chip_id &&
                app_desc.magic_word == ESP_APP_DESC_MAGIC_WORD)
            {
                LOG(INFO,
R"!^!([Bootloader] Firmware details:
Name: %s (%s)
ESP-IDF version: %s
Compile timestamp: %s %s
Target chip-id: %s)!^!"
                  , app_desc.project_name, app_desc.version
                  , app_desc.idf_ver, app_desc.date, app_desc.time
                  , ESP_CHIP_ID[image_header.chip_id]);

                // start the OTA process at this point, if we have had a
                // previous failure this will reset the OTA process so we can
                // start fresh.
                esp_err_t err = ESP_ERROR_CHECK_WITHOUT_ABORT(
                    esp_ota_begin(target, OTA_SIZE_UNKNOWN, &ota_handle));
                should_abort = (err != ESP_OK);
            }
            else
            {
                LOG_ERROR("[Bootloader] Firmware does not appear to be valid "
                          "or is for a different chip (%s vs %s)."
                        , ESP_CHIP_ID[image_header.chip_id]
                        , ESP_CHIP_ID[chip_id]);
                should_abort = true;
            }
        }
        else
        {
            LOG_ERROR("[Bootloader] Image magic is incorrect: %d vs %d!"
                    , image_header.magic, ESP_IMAGE_HEADER_MAGIC);
            should_abort = true;
        }

        // It would be ideal to abort the firmware upload at this point but the
        // bootloader HAL does not offer a way to abort the transfer so instead
        // reboot the node.
        if (should_abort || ota_handle == 0)
        {
            esp_restart();
        }
    }
    bootloader_led(LED_WRITING, true);
    bootloader_led(LED_ACTIVE, false);
    ESP_ERROR_CHECK(esp_ota_write(ota_handle, data, size_bytes));
    bootloader_led(LED_WRITING, false);
    bootloader_led(LED_ACTIVE, true);
}

/// Attempts to finalize the OTA update.
///
/// When the bootloader HAL has received the final chunk of the firmware image
/// this method will be called to confirm the firmware has been successfully
/// written to flash and validated.
///
/// @return zero if the firmware was successfully updated, non-zero otherwise.
uint16_t flash_complete(void)
{
    LOG(INFO, "[Bootloader] Finalizing firmware update");
    esp_err_t res = ESP_ERROR_CHECK_WITHOUT_ABORT(esp_ota_end(ota_handle));
    if (res == ESP_OK)
    {
        LOG(INFO
          , "[Bootloader] Firmware appears valid, updating the next boot "
            "partition to %s.", target->label);
        res = ESP_ERROR_CHECK_WITHOUT_ABORT(esp_ota_set_boot_partition(target));
        if (res != ESP_OK)
        {
            LOG_ERROR("[Bootloader] Failed to update the boot partition!");
        }
    }
    else if (res == ESP_ERR_OTA_VALIDATE_FAILED)
    {
        LOG_ERROR("[Bootloader] Firmware image failed validation, aborting!");
    }
    return res != ESP_OK;
}

/// Calculates a checksum for the provided block of data.
///
/// @param data is the data to checksum.
/// @param size is the number of bytes in data.
/// @param checksum is the calculated checksum.
///
/// NOTE: For the ESP32 this is a no-op.
void checksum_data(const void* data, uint32_t size, uint32_t* checksum)
{
    LOG(VERBOSE, "[Bootloader] checksum_data(%d)", size);
    // Force the checksum to be zero since it is not currently used on the
    // ESP32. The startup of the node may validate the built-in SHA256 and
    // fallback to previous application binary if the SHA256 validation fails.
    memset(checksum, 0, 16);
}

/// Returns the node alias to use.
///
/// @returns the node alias.
///
/// NOTE: For the ESP32 this will return zero and the bootloader will calculate
/// a suitable alias.
uint16_t nmranet_alias(void)
{
    LOG(VERBOSE, "[Bootloader] nmranet_alias");
    // let the bootloader generate it based on nmranet_nodeid().
    return 0;
}

/// Returns the node id to use.
///
/// @returns the node identifier.
uint64_t nmranet_nodeid(void)
{
    LOG(VERBOSE, "[Bootloader] nmranet_nodeid");
    return NODE_ID;
}

} // extern "C"

/// Starts the ESP32 bootloader "lean" stack.
///
/// @param id is the node identifier to use.
/// @param tx is the GPIO pin connected to the CAN transceiver TX pin.
/// @param rx is the GPIO pin connected to the CAN transceiver RX pin.
/// @param reboot_on_exit controls if this method will automatically restart
/// the esp32 when the bootloader execution completes, defaults to true.
void esp32_bootloader_run(uint64_t id, gpio_num_t tx, gpio_num_t rx,
                          bool reboot_on_exit = true)
{
    NODE_ID = id;
    g_config.tx_io = tx;
    g_config.rx_io = rx;
    g_config.tx_queue_len = config_can_tx_buffer_size();
    g_config.rx_queue_len = config_can_rx_buffer_size();

    // Initialize the app header details based on the currently running
    // partition.
    bzero(&node_app_header, sizeof(struct app_header));
    current = (esp_partition_t *)esp_ota_get_running_partition();
    node_app_header.app_size = current->size;

    // Extract the currently running chip details so we can use it to confirm
    // the received firmware is for this chip.
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);

    // The CHIP_ESP32 define is defined as 1 whereas ESP_CHIP_ID_ESP32 is 0.
    // All other ESP_CHIP_ID_XXX maps to CHIP_XXXX directly at this time.
    if (chip_info.model == CHIP_ESP32)
    {
        chip_id = ESP_CHIP_ID_ESP32;
    }
    else
    {
        chip_id = (esp_chip_id_t)chip_info.model;
    }

    // Find the next OTA partition and confirm it is not the currently running
    // partition.
    target = (esp_partition_t *)esp_ota_get_next_update_partition(NULL);
    if (target != nullptr && target != current)
    {
        LOG(VERBOSE, "[Bootloader] calling bootloader_entry");
        bootloader_entry();
    }
    else
    {
        LOG_ERROR("[Bootloader] Unable to locate next OTA partition!");
    }

    if (bootloader_twai_initialized)
    {
        LOG(VERBOSE, "[Bootloader] Stopping TWAI driver");
        ESP_ERROR_CHECK(twai_stop());
        LOG(VERBOSE, "[Bootloader] Disabling TWAI driver");
        ESP_ERROR_CHECK(twai_driver_uninstall());
    }

    if (reboot_on_exit)
    {
        // If we reach here we should restart the node.
        LOG(INFO, "[Bootloader] Restarting!");
        esp_restart();
    }
}