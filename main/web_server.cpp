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
 * \file web_server.cpp
 *
 * Built-in webserver for the ESP32OlcbHub.
 *
 * @author Mike Dunston
 * @date 4 July 2020
 */

#include "constants.hxx"
#include "DelayRebootHelper.hxx"
#include "ConfigUpdateHelper.hxx"
#include "web_server.hxx"
#include <esp_ota_ops.h>
#include <utils/FdUtils.hxx>
#include <utils/FileUtils.hxx>
#include <utils/logging.h>

static std::unique_ptr<http::Httpd> http_server;
static MDNS mdns;
static int config_fd;
static node_config_t *node_cfg;

esp_ota_handle_t otaHandle;
esp_partition_t *ota_partition = nullptr;
HTTP_STREAM_HANDLER_IMPL(process_ota, request, filename, size, data, length, offset, final, abort_req)
{
    if (!offset)
    {
        ota_partition = (esp_partition_t *)esp_ota_get_next_update_partition(NULL);
        esp_err_t err = ESP_ERROR_CHECK_WITHOUT_ABORT(
            esp_ota_begin(ota_partition, size, &otaHandle));
        if (err != ESP_OK)
        {
            LOG_ERROR("[Web] OTA start failed, aborting!");
            request->set_status(http::HttpStatusCode::STATUS_SERVER_ERROR);
            *abort_req = true;
            return nullptr;
        }
        LOG(INFO, "[Web] OTA Update starting (%zu bytes, target:%s)...", size, ota_partition->label);
    }
    HASSERT(ota_partition);
    ESP_ERROR_CHECK(esp_ota_write(otaHandle, data, length));
    if (final)
    {
        esp_err_t err = ESP_ERROR_CHECK_WITHOUT_ABORT(esp_ota_end(otaHandle));
        if (err != ESP_OK)
        {
            LOG_ERROR("[Web] OTA end failed, aborting!");
            request->set_status(http::HttpStatusCode::STATUS_SERVER_ERROR);
            *abort_req = true;
            return nullptr;
        }
        LOG(INFO, "[Web] OTA binary received, setting boot partition: %s", ota_partition->label);
        err = ESP_ERROR_CHECK_WITHOUT_ABORT(
            esp_ota_set_boot_partition(ota_partition));
        if (err != ESP_OK)
        {
            LOG_ERROR("[Web] OTA end failed, aborting!");
            request->set_status(http::HttpStatusCode::STATUS_SERVER_ERROR);
            *abort_req = true;
            return nullptr;
        }
        LOG(INFO, "[Web] OTA Update Complete!");
        request->set_status(http::HttpStatusCode::STATUS_OK);
        Singleton<esp32olcbhub::DelayRebootHelper>::instance()->start();
        return new http::StringResponse("OTA Upload Successful, rebooting", http::MIME_TYPE_TEXT_PLAIN);
    }
    return nullptr;
}

// Helper which converts a string to a uint64 value.
uint64_t string_to_uint64(std::string value)
{
    // remove period characters if present
    value.erase(std::remove(value.begin(), value.end(), '.'), value.end());
    // convert the string to a uint64_t value
    return std::stoull(value, nullptr, 16);
}

HTTP_HANDLER_IMPL(process_cdi, request)
{
    http::AbstractHttpResponse *response = nullptr;
    size_t offs = request->param("offs", 0);
    std::string param_type = request->param("type");
    if (request->method() == http::HttpMethod::GET)
    {
        size_t size = request->param("size", 0);
        if (param_type == "string")
        {
            LOG(VERBOSE, "[Web] CDI STRING READ offs:%d, size:%d", offs, size);
            uint8_t buffer[256];
            bzero(&buffer, 256);
            HASSERT(size < 256);
            ERRNOCHECK("seek_config", lseek(config_fd, offs, SEEK_SET));
            FdUtils::repeated_read(config_fd, buffer, size);
            response = new http::StringResponse(string((char *)buffer), http::MIME_TYPE_TEXT_PLAIN);
        }
        else if (param_type == "int")
        {
            LOG(VERBOSE, "[Web] CDI INT READ offs:%d, size:%d", offs, size);
            ERRNOCHECK("seek_config", lseek(config_fd, offs, SEEK_SET));
            switch (size)
            {
            case 1:
            {
                uint8_t data8 = 0;
                FdUtils::repeated_read(config_fd, &data8, size);
                response =
                    new http::StringResponse(integer_to_string(data8), http::MIME_TYPE_TEXT_PLAIN);
            }
            break;
            case 2:
            {
                uint16_t data16 = 0;
                FdUtils::repeated_read(config_fd, &data16, size);
                response =
                    new http::StringResponse(integer_to_string(be16toh(data16)), http::MIME_TYPE_TEXT_PLAIN);
            }
            break;
            case 4:
            {
                uint32_t data32 = 0;
                FdUtils::repeated_read(config_fd, &data32, size);
                response =
                    new http::StringResponse(integer_to_string(be32toh(data32)), http::MIME_TYPE_TEXT_PLAIN);
            }
            break;
            }
        }
        else if (param_type == "eventid")
        {
            LOG(VERBOSE, "[Web] CDI EVENT READ offs:%d", offs);
            uint64_t data = 0;
            ERRNOCHECK("seek_config", lseek(config_fd, offs, SEEK_SET));
            FdUtils::repeated_read(config_fd, &data, sizeof(uint64_t));
            response =
                new http::StringResponse(uint64_to_string_hex(be64toh(data)), http::MIME_TYPE_TEXT_PLAIN);
        }
        else
        {
            LOG_ERROR("[Web] Bad request:\n%s", request->to_string().c_str());
            request->set_status(http::HttpStatusCode::STATUS_BAD_REQUEST);
            return nullptr;
        }
        LOG(VERBOSE, "[Web] CDI READ value:\n%s", response->to_string().c_str());
    }
    else if (request->method() == http::HttpMethod::PUT)
    {
        string value = request->param("value");
        size_t size = request->param("size", 0);
        if (param_type == "string")
        {
            LOG(VERBOSE, "[Web] CDI STRING WRITE offs:%d, value:%s", offs, value.c_str());
            ERRNOCHECK("seek_config", lseek(config_fd, offs, SEEK_SET));
            // make sure value is null terminated
            value += '\0';
            FdUtils::repeated_write(config_fd, value.data(), value.size());
        }
        else if (param_type == "int")
        {
            LOG(VERBOSE, "[Web] CDI INT WRITE offs:%d, size:%d, value:%s", offs, size, value.c_str());
            ERRNOCHECK("seek_config", lseek(config_fd, offs, SEEK_SET));
            switch (size)
            {
            case 1:
            {
                uint8_t data8 = std::stoi(value);
                FdUtils::repeated_write(config_fd, &data8, size);
            }
            break;
            case 2:
            {
                uint16_t data16 = htobe16(std::stoi(value));
                FdUtils::repeated_write(config_fd, &data16, size);
            }
            break;
            case 4:
            {
                uint32_t data32 = htobe32(std::stoul(value));
                FdUtils::repeated_write(config_fd, &data32, size);
            }
            break;
            }
        }
        else if (param_type == "eventid")
        {
            LOG(VERBOSE, "[Web] CDI EVENT WRITE offs:%d, value: %s", offs, value.c_str());
            uint64_t data = htobe64(string_to_uint64(value));
            ERRNOCHECK("seek_config", lseek(config_fd, offs, SEEK_SET));
            FdUtils::repeated_write(config_fd, &data, sizeof(uint64_t));
        }
        else
        {
            LOG_ERROR("[Web] Bad request:\n%s", request->to_string().c_str());
            request->set_status(http::HttpStatusCode::STATUS_BAD_REQUEST);
            return nullptr;
        }
        request->set_status(http::HttpStatusCode::STATUS_ACCEPTED);
        Singleton<esp32olcbhub::ConfigUpdateHelper>::instance()->trigger_update();
    }
    else
    {
        LOG_ERROR("[Web] Bad request:\n%s", request->to_string().c_str());
        request->set_status(http::HttpStatusCode::STATUS_BAD_REQUEST);
    }
    return response;
}

void init_webserver(node_config_t *config, int fd)
{
    config_fd = fd;
    node_cfg = config;
    LOG(INFO, "[Httpd] Initializing webserver");
    http_server.reset(new http::Httpd(&mdns));
    if (config->wifi_mode == WIFI_MODE_AP || config->wifi_mode == WIFI_MODE_APSTA)
    {
        const esp_app_desc_t *app_data = esp_ota_get_app_description();
        http_server->captive_portal(
            StringPrintf(CAPTIVE_PORTAL_HTML, app_data->project_name, app_data->version, app_data->project_name, app_data->project_name));
    }
    http_server->redirect_uri("/", "/index.html");
    http_server->static_uri("/index.html", indexHtmlGz, indexHtmlGz_size, http::MIME_TYPE_TEXT_HTML, http::HTTP_ENCODING_GZIP);
    http_server->static_uri("/cash.min.js", cashJsGz, cashJsGz_size, http::MIME_TYPE_TEXT_JAVASCRIPT, http::HTTP_ENCODING_GZIP);
    http_server->static_uri("/normalize.min.css", milligramMinCssGz, milligramMinCssGz_size, http::MIME_TYPE_TEXT_CSS, http::HTTP_ENCODING_GZIP);
    http_server->static_uri("/milligram.min.css", normalizeMinCssGz, normalizeMinCssGz_size, http::MIME_TYPE_TEXT_CSS, http::HTTP_ENCODING_GZIP);
    http_server->uri("/info", [&](http::HttpRequest *req) -> http::AbstractHttpResponse * {
        const esp_app_desc_t *app_data = esp_ota_get_app_description();
        const esp_partition_t *partition = esp_ota_get_running_partition();
        string info =
            StringPrintf("{\"build\":\"%s\","
                         "\"timestamp\":\"%s %s\",\"ota\":\"%s\",\"uptime\":%llu,"
                         "\"snip_name\":\"%s\",\"snip_hw\":\"%s\","
                         "\"snip_sw\":\"%s\",\"node_id\":\"%s\"}",
                         app_data->version, app_data->date, app_data->time, partition->label, esp_timer_get_time(),
                         openlcb::SNIP_STATIC_DATA.model_name, openlcb::SNIP_STATIC_DATA.hardware_version,
                         openlcb::SNIP_STATIC_DATA.software_version, uint64_to_string_hex(node_cfg->node_id).c_str());
        return new http::JsonResponse(info);
    });
    http_server->uri("/fs", http::HttpMethod::GET, [&](http::HttpRequest *request) -> http::AbstractHttpResponse * {
        string path = request->param("path");
        LOG(VERBOSE, "[Web] Searching for path: %s", path.c_str());
        struct stat statbuf;
        // verify that the requested path exists
        if (!stat(path.c_str(), &statbuf))
        {
            string data = read_file_to_string(path);
            string mimetype = http::MIME_TYPE_TEXT_PLAIN;
            if (path.find(".xml") != string::npos)
            {
                mimetype = http::MIME_TYPE_TEXT_XML;
                // CDI xml files have a trailing null, this can cause
                // issues in browsers parsing/rendering the XML data.
                if (request->param("remove_nulls", false))
                {
                    std::replace(data.begin(), data.end(), '\0', ' ');
                }
            }
            else if (path.find(".json") != string::npos)
            {
                mimetype = http::MIME_TYPE_APPLICATION_JSON;
            }
            return new http::StringResponse(data, mimetype);
        }
        LOG(INFO, "[Web] Path not found");
        request->set_status(http::HttpStatusCode::STATUS_NOT_FOUND);
        return nullptr;
    });
    http_server->uri("/wifi_scan", [&](http::HttpRequest *req) -> http::AbstractHttpResponse * {
        auto wifi = Singleton<openmrn_arduino::Esp32WiFiManager>::instance();
        string result = "[";
        SyncNotifiable n;
        wifi->start_ssid_scan(&n);
        n.wait_for_notification();
        size_t num_found = wifi->get_ssid_scan_result_count();
        vector<string> seen_ssids;
        for (int i = 0; i < num_found; i++)
        {
            auto entry = wifi->get_ssid_scan_result(i);
            if (std::find_if(seen_ssids.begin(), seen_ssids.end(), [entry](string &s) {
                    return s == (char *)entry.ssid;
                }) != seen_ssids.end())
            {
                // filter duplicate SSIDs
                continue;
            }
            seen_ssids.push_back((char *)entry.ssid);
            if (result.length() > 1)
            {
                result += ",";
            }
            LOG(VERBOSE, "auth:%d,rssi:%d,ssid:%s", entry.authmode, entry.rssi, entry.ssid);
            result += StringPrintf("{\"auth\":%d,\"rssi\":%d,\"ssid\":\"%s\"}",
                                   entry.authmode, entry.rssi,
                                   http::url_encode((char *)entry.ssid).c_str());
        }
        result += "]";
        wifi->clear_ssid_scan_results();
        return new http::JsonResponse(result);
    });
    http_server->uri("/factory_reset", [&](http::HttpRequest *req) -> http::AbstractHttpResponse * {
        LOG(INFO, "[Web] Factory Reset request received");
        if (force_factory_reset())
        {
            req->set_status(http::HttpStatusCode::STATUS_NO_CONTENT);
            Singleton<esp32olcbhub::DelayRebootHelper>::instance()->start();
        }
        else
        {
            req->set_status(http::HttpStatusCode::STATUS_SERVER_ERROR);
        }
        return nullptr;
    });
    http_server->uri("/cdi", process_cdi);
    http_server->uri("/node_id", http::HttpMethod::POST, [&](http::HttpRequest *req) -> http::AbstractHttpResponse * {
        if (!req->has_param("node_id"))
        {
            req->set_status(http::HttpStatusCode::STATUS_BAD_REQUEST);
        }
        else
        {
            uint64_t new_node_id = string_to_uint64(req->param("node_id"));
            if (new_node_id != 0 && set_node_id(string_to_uint64(req->param("node_id"))))
            {
                req->set_status(http::HttpStatusCode::STATUS_ACCEPTED);
                Singleton<esp32olcbhub::DelayRebootHelper>::instance()->start();
            }
            else
            {
                req->set_status(http::HttpStatusCode::STATUS_BAD_REQUEST);
            }
        }
        return nullptr;
    });
    http_server->uri("/ota", http::HttpMethod::POST, nullptr, process_ota);
    http_server->uri("/wifi", [&](http::HttpRequest *req) -> http::AbstractHttpResponse * {
        if (req->method() == http::HttpMethod::GET)
        {
            return new http::JsonResponse(StringPrintf("{\"mode\":%d,\"ssid\":\"%s\"}", node_cfg->wifi_mode, node_cfg->ap_ssid));
        }
        else
        {
            if (reconfigure_wifi((wifi_mode_t)req->param("mode", WIFI_MODE_STA), req->param("ssid"), req->param("pass")))
            {
                req->set_status(http::HttpStatusCode::STATUS_NO_CONTENT);
                Singleton<esp32olcbhub::DelayRebootHelper>::instance()->start();
            }
            else
            {
                req->set_status(http::HttpStatusCode::STATUS_BAD_REQUEST);
            }
        }
        return nullptr;
    });
}