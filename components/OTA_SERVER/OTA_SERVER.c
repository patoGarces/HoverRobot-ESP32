#include <stdio.h>
#include "OTA_SERVER.h"
#include "esp_http_server.h"
#include "esp_ota_ops.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <sys/param.h>
#include "cJSON.h"

static const char *TAG = "OTA_SERVER";

#define OTA_STABILIZED_MODE 3

extern const uint8_t index_html_start[] asm("_binary_index_html_start");
extern const uint8_t index_html_end[]   asm("_binary_index_html_end");

extern const uint8_t style_css_start[] asm("_binary_style_css_start");
extern const uint8_t style_css_end[]   asm("_binary_style_css_end");

static ota_mode_provider_t mode_provider = NULL;

void otaServerSetModeProvider(ota_mode_provider_t provider) {
    mode_provider = provider;
}

static bool isRobotStabilized(void) {
    uint8_t mode = mode_provider ? mode_provider() : 0;
    return mode == OTA_STABILIZED_MODE;
}

static esp_err_t rootGetHandler(httpd_req_t *req)
{
    size_t len = index_html_end - index_html_start;

    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, (const char *)index_html_start, len);

    return ESP_OK;
}

static esp_err_t infoGetHandler(httpd_req_t *req)
{
    char response[256];

    snprintf(response, sizeof(response),
        "{"
        "\"device\":\"TermoSmart\","
        "\"version\":\"1.0.0\""
        "}"
    );

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, response, HTTPD_RESP_USE_STRLEN);

    return ESP_OK;
}

static esp_err_t statusGetHandler(httpd_req_t *req) {
    cJSON *root = cJSON_CreateObject();

    uint8_t mode = mode_provider ? mode_provider() : 0;

    cJSON_AddStringToObject(root, "version", "0.0.0"); // VERSION_FIRMWARE);
    cJSON_AddNumberToObject(root, "uptime", esp_timer_get_time() / 1000000);
    cJSON_AddNumberToObject(root, "mode", mode);

    char *response = cJSON_PrintUnformatted(root);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, response);

    free(response);
    cJSON_Delete(root);

    return ESP_OK;
}

static esp_err_t otaCheckHandler(httpd_req_t *req) {
    if (isRobotStabilized()) {
        httpd_resp_set_status(req, "409 Conflict");
        ESP_LOGE("OTA", "Robot is stabilized. OTA rejected.");
        return httpd_resp_sendstr(req, "Robot is stabilized. OTA rejected.");
    }

    return httpd_resp_sendstr(req, "OTA ready");
}

static esp_err_t cssGetHandler(httpd_req_t *req)
{
    size_t len = style_css_end - style_css_start;

    httpd_resp_set_type(req, "text/css");
    httpd_resp_send(req, (const char *)style_css_start, len);

    return ESP_OK;
}

static esp_err_t otaPostHandler(httpd_req_t *req) {

    if (isRobotStabilized()) {
        ESP_LOGW(TAG, "Robot is stabilized. OTA rejected.");

        httpd_resp_set_status(req, "409 Conflict");
        return httpd_resp_sendstr(req, "Robot is stabilized. OTA rejected.");
    }

    ESP_LOGI("OTA", "Starting OTA upload");
    const esp_partition_t *update_partition =
        esp_ota_get_next_update_partition(NULL);

    if (!update_partition) {
        ESP_LOGE("OTA", "No OTA partition found");
        return ESP_FAIL;
    }

    esp_ota_handle_t ota_handle;

    esp_err_t err = esp_ota_begin(
        update_partition,
        OTA_SIZE_UNKNOWN,
        &ota_handle
    );

    if (err != ESP_OK) {
        ESP_LOGE("OTA", "ota begin failed: %s", esp_err_to_name(err));
        return ESP_FAIL;
    }
    char buffer[1024];

    size_t total_size = req->content_len;
    size_t remaining = req->content_len;
    size_t received_total = 0;

    int last_percent = -1;

    while (remaining > 0) {
        int received = httpd_req_recv(
            req,
            buffer,
            MIN(remaining, sizeof(buffer))
        );

        if (received <= 0) {

            ESP_LOGE("OTA", "Receive failed");

            esp_ota_end(ota_handle);

            return ESP_FAIL;
        }
        err = esp_ota_write(
            ota_handle,
            buffer,
            received
        );

        if (err != ESP_OK) {
            ESP_LOGE(
                "OTA",
                "Write failed: %s",
                esp_err_to_name(err)
            );
            esp_ota_end(ota_handle);
            return ESP_FAIL;
        }

        remaining -= received;
        received_total += received;
        int percent = (received_total * 100) / total_size;

        if (percent != last_percent) {
            ESP_LOGI(
                "OTA",
                "Upload: %d%% (%u/%u bytes)",
                percent,
                received_total,
                total_size
            );
            last_percent = percent;
        }
    }
    err = esp_ota_end(ota_handle);

    if (err != ESP_OK) {

        ESP_LOGE(
            "OTA",
            "OTA end failed: %s",
            esp_err_to_name(err)
        );

        return ESP_FAIL;
    }
    err = esp_ota_set_boot_partition(update_partition);

    if (err != ESP_OK) {

        ESP_LOGE(
            "OTA",
            "Set boot partition failed: %s",
            esp_err_to_name(err)
        );

        return ESP_FAIL;
    }
    ESP_LOGI("OTA", "Update complete, rebooting");

    httpd_resp_sendstr(
        req,
        "Update OK. Rebooting..."
    );

    vTaskDelay(pdMS_TO_TICKS(100));
    esp_restart();

    return ESP_OK;
}

void otaServerStart(void) {
    httpd_handle_t server = NULL;

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 8192;
    config.max_open_sockets = 7;
    config.lru_purge_enable = true;
    config.keep_alive_enable = false;

    esp_err_t err = httpd_start(&server, &config);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "No se pudo iniciar el servidor");
        return;
    }

    httpd_uri_t root = {
        .uri       = "/",
        .method    = HTTP_GET,
        .handler   = rootGetHandler,
        .user_ctx  = NULL
    };

    httpd_register_uri_handler(server, &root);

    httpd_uri_t ota = {
        .uri      = "/update",
        .method   = HTTP_POST,
        .handler  = otaPostHandler
    };

    httpd_register_uri_handler(server, &ota);

    httpd_uri_t css = {
        .uri       = "/style.css",
        .method    = HTTP_GET,
        .handler   = cssGetHandler,
        .user_ctx  = NULL
    };

    httpd_register_uri_handler(server, &css);

    httpd_uri_t status = {
        .uri       = "/status",
        .method    = HTTP_GET,
        .handler   = statusGetHandler,
        .user_ctx  = NULL
    };

    httpd_register_uri_handler(server, &status);

    httpd_uri_t info = {
        .uri       = "/info",
        .method    = HTTP_GET,
        .handler   = infoGetHandler,
        .user_ctx  = NULL
    };

    httpd_register_uri_handler(server, &info);

    httpd_uri_t otaCheck = {
        .uri = "/update/check",
        .method = HTTP_POST,
        .handler = otaCheckHandler,
        .user_ctx = NULL
    };

    httpd_register_uri_handler(server, &otaCheck);

    ESP_LOGI(TAG, "Servidor HTTP iniciado");
}