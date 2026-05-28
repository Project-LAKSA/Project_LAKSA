/* Embedded WiFi dashboard implementation.
 *
 * The ESP32 joins the configured WiFi network as a station and starts a small
 * HTTP server. The page polls JSON state and sends simple servo commands.
 */

#include "web_dashboard.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp32_config.h"
#include "esp_check.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/event_groups.h"
#include "nvs_flash.h"

#define BNO08X_TRANSPORT_NAME "SPI"

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1
#define HTTP_QUERY_BUF_LEN 96
#define HTTP_RESPONSE_BUF_LEN 1024

static const char *TAG = "web_dashboard";
static EventGroupHandle_t wifi_event_group;
static web_dashboard_t *active_dashboard;

static const char dashboard_html[] =
    "<!doctype html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>LAKSA Dashboard</title><style>"
    "body{margin:0;font-family:system-ui,-apple-system,Segoe UI,sans-serif;background:#10151f;color:#edf2f7}"
    "main{max-width:980px;margin:0 auto;padding:18px}h1{font-size:24px;margin:0 0 14px}"
    ".grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(260px,1fr));gap:12px}"
    "section{background:#17202d;border:1px solid #283548;border-radius:8px;padding:14px}"
    "h2{font-size:16px;margin:0 0 12px;color:#9cc5ff}.row{display:flex;justify-content:space-between;gap:10px;margin:8px 0}"
    ".value{font-variant-numeric:tabular-nums;color:#d6e4ff}.controls{display:grid;grid-template-columns:44px 1fr 44px;gap:8px;align-items:center}"
    "button{height:38px;border:0;border-radius:6px;background:#2f80ed;color:white;font-size:20px}"
    "input[type=range]{width:100%}.muted{color:#91a0b5}.bad{color:#ff8a8a}.ok{color:#86efac}"
    "</style></head><body><main><h1>LAKSA ESP32 Dashboard</h1><div class='grid'>"
    "<section><h2>Servos</h2><div id='servos'></div></section>"
    "<section><h2>IMU</h2><div class='row'><span>Transport</span><span id='transport' class='value'></span></div>"
    "<div id='imu'></div></section></div></main><script>"
    "const servoChannels=[13,15];"
    "function fmt(v,n=3){return Number(v||0).toFixed(n)}"
    "async function setServo(ch,angle){angle=Math.max(0,Math.min(180,Math.round(angle/10)*10));await fetch(`/api/servo?channel=${ch}&angle=${angle}`);refresh()}"
    "function servoView(s){return servoChannels.map(ch=>{let a=s['ch'+ch]??0;return `<div class='row'><span>CH ${ch}</span><span class='value'>${a}&deg;</span></div><div class='controls'><button onclick='setServo(${ch},${a}-10)'>-</button><input type='range' min='0' max='180' step='10' value='${a}' onchange='setServo(${ch},this.value)'><button onclick='setServo(${ch},${a}+10)'>+</button></div>`}).join('')}"
    "function quat(name,q,showAcc=true){let acc=showAcc?` acc ${fmt(q.accuracy,4)}`:'';return `<div class='row'><span>${name}</span><span class='${q.has?'ok':'bad'}'>${q.has?'Y':'N'}</span></div><div class='value'>i ${fmt(q.i,4)} j ${fmt(q.j,4)} k ${fmt(q.k,4)} r ${fmt(q.real,4)}${acc}</div>`}"
    "async function refresh(){try{let r=await fetch('/api/state');let d=await r.json();transport.textContent=d.imu.transport;servos.innerHTML=servoView(d.servos);imu.innerHTML=`<div class='row'><span>Accel</span><span class='${d.imu.accel.has?'ok':'bad'}'>${d.imu.accel.has?'Y':'N'}</span></div><div class='value'>x ${fmt(d.imu.accel.x)} y ${fmt(d.imu.accel.y)} z ${fmt(d.imu.accel.z)}</div>${quat('Rotation',d.imu.rotation)}${quat('Game rotation',d.imu.game_rotation,false)}`}"
    "catch(e){imu.innerHTML='<span class=bad>offline</span>'}}"
    "setInterval(refresh,500);refresh();</script></body></html>";

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();
        xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Dashboard URL: http://" IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static esp_err_t wifi_start(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_RETURN_ON_ERROR(err, TAG, "NVS init failed");

    wifi_event_group = xEventGroupCreate();
    if (wifi_event_group == NULL) {
        return ESP_ERR_NO_MEM;
    }

    ESP_RETURN_ON_ERROR(esp_netif_init(), TAG, "netif init failed");
    ESP_RETURN_ON_ERROR(esp_event_loop_create_default(), TAG, "event loop init failed");
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t init_config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_wifi_init(&init_config), TAG, "WiFi init failed");
    ESP_RETURN_ON_ERROR(esp_event_handler_instance_register(WIFI_EVENT,
                                                            ESP_EVENT_ANY_ID,
                                                            wifi_event_handler,
                                                            NULL,
                                                            NULL),
                        TAG,
                        "WiFi event handler failed");
    ESP_RETURN_ON_ERROR(esp_event_handler_instance_register(IP_EVENT,
                                                            IP_EVENT_STA_GOT_IP,
                                                            wifi_event_handler,
                                                            NULL,
                                                            NULL),
                        TAG,
                        "IP event handler failed");

    wifi_config_t wifi_config = {0};
    strlcpy((char *)wifi_config.sta.ssid, WIFI_STA_SSID, sizeof(wifi_config.sta.ssid));
    strlcpy((char *)wifi_config.sta.password, WIFI_STA_PASSWORD, sizeof(wifi_config.sta.password));
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_STA), TAG, "WiFi mode failed");
    ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_STA, &wifi_config), TAG, "WiFi config failed");
    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "WiFi start failed");

    EventBits_t bits = xEventGroupWaitBits(wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           pdMS_TO_TICKS(WIFI_CONNECT_TIMEOUT_MS));

    if ((bits & WIFI_CONNECTED_BIT) == 0) {
        return ESP_ERR_TIMEOUT;
    }

    return ESP_OK;
}

static esp_err_t index_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, dashboard_html, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t state_handler(httpd_req_t *req)
{
    web_dashboard_t *dashboard = (web_dashboard_t *)req->user_ctx;
    bno08x_adapter_vec3_t accel = {0};
    bno08x_adapter_quat_t rotation = {0};
    bno08x_adapter_quat_t game_rotation = {0};

    bool has_accel = false;
    bool has_rotation = false;
    bool has_game_rotation = false;
    uint8_t servo13 = 0;
    uint8_t servo15 = 0;

    if (dashboard->hardware_mutex != NULL) {
        xSemaphoreTake(dashboard->hardware_mutex, portMAX_DELAY);
    }

    has_accel = bno08x_adapter_get_acceleration(dashboard->imu, &accel) == ESP_OK;
    has_rotation = bno08x_adapter_get_rotation_vector(dashboard->imu, &rotation) == ESP_OK;
    has_game_rotation = bno08x_adapter_get_game_rotation_vector(dashboard->imu, &game_rotation) == ESP_OK;
    servo13 = dashboard->servo13_angle;
    servo15 = dashboard->servo15_angle;

    if (dashboard->hardware_mutex != NULL) {
        xSemaphoreGive(dashboard->hardware_mutex);
    }

    char response[HTTP_RESPONSE_BUF_LEN];
    int len = snprintf(response,
                       sizeof(response),
                       "{\"servos\":{\"ch13\":%u,\"ch15\":%u},"
                       "\"imu\":{\"transport\":\"%s\","
                       "\"accel\":{\"has\":%s,\"x\":%.3f,\"y\":%.3f,\"z\":%.3f},"
                       "\"rotation\":{\"has\":%s,\"i\":%.4f,\"j\":%.4f,\"k\":%.4f,\"real\":%.4f,\"accuracy\":%.4f},"
                       "\"game_rotation\":{\"has\":%s,\"i\":%.4f,\"j\":%.4f,\"k\":%.4f,\"real\":%.4f,\"accuracy\":%.4f}}}",
                       servo13,
                       servo15,
                       BNO08X_TRANSPORT_NAME,
                       has_accel ? "true" : "false",
                       accel.x,
                       accel.y,
                       accel.z,
                       has_rotation ? "true" : "false",
                       rotation.i,
                       rotation.j,
                       rotation.k,
                       rotation.real,
                       rotation.accuracy,
                       has_game_rotation ? "true" : "false",
                       game_rotation.i,
                       game_rotation.j,
                       game_rotation.k,
                       game_rotation.real,
                       game_rotation.accuracy);

    if (len < 0 || len >= (int)sizeof(response)) {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "state too large");
    }

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, response, len);
}

static esp_err_t servo_handler(httpd_req_t *req)
{
    web_dashboard_t *dashboard = (web_dashboard_t *)req->user_ctx;
    char query[HTTP_QUERY_BUF_LEN] = {0};
    char channel_text[8] = {0};
    char angle_text[8] = {0};

    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK ||
        httpd_query_key_value(query, "channel", channel_text, sizeof(channel_text)) != ESP_OK ||
        httpd_query_key_value(query, "angle", angle_text, sizeof(angle_text)) != ESP_OK) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "expected channel and angle");
    }

    int channel = atoi(channel_text);
    int angle = atoi(angle_text);
    if ((channel != 13 && channel != 15) || angle < 0 || angle > 180) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid servo command");
    }

    if (dashboard->hardware_mutex != NULL) {
        xSemaphoreTake(dashboard->hardware_mutex, portMAX_DELAY);
    }

    esp_err_t err = pca9685_set_servo_angle(dashboard->pca9685, (uint8_t)channel, (uint8_t)angle);
    if (err == ESP_OK) {
        if (channel == 13) {
            dashboard->servo13_angle = (uint8_t)angle;
        } else {
            dashboard->servo15_angle = (uint8_t)angle;
        }
    }

    if (dashboard->hardware_mutex != NULL) {
        xSemaphoreGive(dashboard->hardware_mutex);
    }

    if (err != ESP_OK) {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, esp_err_to_name(err));
    }

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, "{\"ok\":true}");
}

esp_err_t web_dashboard_start(web_dashboard_t *dashboard)
{
    if (dashboard == NULL || dashboard->pca9685 == NULL || dashboard->imu == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    active_dashboard = dashboard;
    ESP_RETURN_ON_ERROR(wifi_start(), TAG, "WiFi failed");

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.uri_match_fn = httpd_uri_match_wildcard;

    httpd_handle_t server = NULL;
    ESP_RETURN_ON_ERROR(httpd_start(&server, &config), TAG, "HTTP server failed");

    httpd_uri_t index_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = index_handler,
        .user_ctx = active_dashboard,
    };
    httpd_uri_t state_uri = {
        .uri = "/api/state",
        .method = HTTP_GET,
        .handler = state_handler,
        .user_ctx = active_dashboard,
    };
    httpd_uri_t servo_uri = {
        .uri = "/api/servo",
        .method = HTTP_GET,
        .handler = servo_handler,
        .user_ctx = active_dashboard,
    };

    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(server, &index_uri), TAG, "index route failed");
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(server, &state_uri), TAG, "state route failed");
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(server, &servo_uri), TAG, "servo route failed");

    return ESP_OK;
}
