#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "esp_camera.h"
#include "lwip/sockets.h"
#include "sensor.h"

#include "secrets.h"

#define CAM_PIN_PWDN 32
#define CAM_PIN_RESET -1
#define CAM_PIN_XCLK 0
#define CAM_PIN_SIOD 26
#define CAM_PIN_SIOC 27
#define CAM_PIN_D7 35
#define CAM_PIN_D6 34
#define CAM_PIN_D5 39
#define CAM_PIN_D4 36
#define CAM_PIN_D3 21
#define CAM_PIN_D2 19
#define CAM_PIN_D1 18
#define CAM_PIN_D0 5
#define CAM_PIN_VSYNC 25
#define CAM_PIN_HREF 23
#define CAM_PIN_PCLK 22
#define CAM_PIN_FLASH 4

#define TAG "ESP32-CAM"

static QueueHandle_t frameBuffer_queue;

static camera_config_t camera_config = {
    .pin_pwdn  = CAM_PIN_PWDN,
    .pin_reset = CAM_PIN_RESET,
    .pin_xclk = CAM_PIN_XCLK,
    .pin_sccb_sda = CAM_PIN_SIOD,
    .pin_sccb_scl = CAM_PIN_SIOC,

    .pin_d7 = CAM_PIN_D7,
    .pin_d6 = CAM_PIN_D6,
    .pin_d5 = CAM_PIN_D5,
    .pin_d4 = CAM_PIN_D4,
    .pin_d3 = CAM_PIN_D3,
    .pin_d2 = CAM_PIN_D2,
    .pin_d1 = CAM_PIN_D1,
    .pin_d0 = CAM_PIN_D0,
    .pin_vsync = CAM_PIN_VSYNC,
    .pin_href = CAM_PIN_HREF,
    .pin_pclk = CAM_PIN_PCLK,

    .xclk_freq_hz = 38000000, //slightly unstable but works
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_JPEG,
    .frame_size = FRAMESIZE_SVGA,
    
    .jpeg_quality = 15, //0-63, for OV series camera sensors, lower number means higher quality
    .fb_count = 3,
    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_LATEST
};

void camera_init(void *arg){
    ESP_LOGI(TAG, "camera_init running on core %d!", xPortGetCoreID());

    gpio_set_level(CAM_PIN_PWDN, 0);
    
    ESP_ERROR_CHECK(esp_camera_init(&camera_config));
    while (1) {
        // TickType_t start = xTaskGetTickCount();

        camera_fb_t *frameBuffer = esp_camera_fb_get();
        xQueueSend(frameBuffer_queue, &frameBuffer, portMAX_DELAY); 

        // TickType_t end = xTaskGetTickCount();
        // TickType_t delta = end - start;
        // ESP_LOGI(TAG, "Camera loop duration: %lu ticks (~%lu ms)", delta, delta * portTICK_PERIOD_MS);
    }
}

void udp_client_stream(void *args) {
    ESP_LOGI(TAG, "udp_client_stream running on core %d!", xPortGetCoreID());
    camera_fb_t * frameBuffer = NULL;

    struct sockaddr_in dest_addr = {
        .sin_addr.s_addr = inet_addr(UDP_SERVER_IP),
        .sin_family = AF_INET,
        .sin_port = htons(UDP_SERVER_PORT)
    };

    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    int udp_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (udp_socket < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        esp_restart();
    }
    
    while (1) {
        // TickType_t start = xTaskGetTickCount();
        if (xQueueReceive(frameBuffer_queue, &frameBuffer, portMAX_DELAY) != pdTRUE){
            continue;
        }

        if (!frameBuffer || !frameBuffer->buf || frameBuffer->len == 0) {
            ESP_LOGE(TAG, "Frame inválido. Saltando frame.");
            esp_camera_fb_return(frameBuffer);
            continue;
        }

        int err = sendto(udp_socket, frameBuffer->buf, frameBuffer->len, 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));

        if (err < 0) {
            ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
            esp_camera_fb_return(frameBuffer);
            continue;
            //esp_restart();
        }

        //ESP_LOGI(TAG, "Message sent");
        esp_camera_fb_return(frameBuffer);
        // TickType_t end = xTaskGetTickCount();
        // TickType_t delta = end - start;
        // ESP_LOGI(TAG, "udp_client_stream loop duration: %lu ticks (~%lu ms)", delta, delta * portTICK_PERIOD_MS);
    }

}

void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    ESP_LOGI(TAG, "WIFI event received: base=%s, id=%" PRId32, event_base, event_id);

    switch (event_id)
    {
        case WIFI_EVENT_STA_START:
            ESP_LOGI(TAG, "Connecting to wifi...");
            esp_wifi_connect();
            break;
        case WIFI_EVENT_STA_CONNECTED:
            ESP_LOGI(TAG, "Connected to wifi!");
            break;
        case WIFI_EVENT_STA_DISCONNECTED:
            ESP_LOGE(TAG, "LOST WIFI CONNECTION, REBOOTING");
            esp_restart();
            break;
        default:
            break;
    }

}

void ip_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    ESP_LOGI(TAG, "IP event received: base=%s, id=%" PRId32, event_base, event_id);

    switch (event_id)
    {
        case IP_EVENT_STA_GOT_IP:
            ESP_LOGI(TAG, "Got IP!");
            xTaskCreatePinnedToCore(camera_init, "camera_task", 4096, NULL, 1, NULL, 1);
            xTaskCreatePinnedToCore(udp_client_stream, "udp_task", 4096, NULL, 1, NULL, 0);
            break;
        case IP_EVENT_STA_LOST_IP:
            ESP_LOGE(TAG, "LOST IP, REBOOTING");
            esp_restart();
            break;
        default:
            break;
    }

}

void wifi_init_sta() {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    ESP_LOGI(TAG, "Created STA interface");

    wifi_init_config_t wifi_init_config = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&wifi_init_config);
    ESP_LOGI(TAG, "WIFI initializated");

    esp_event_handler_instance_t instance_wifi_event_handler;
    esp_event_handler_instance_t instance_ip_event_handler;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, &instance_wifi_event_handler));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, ESP_EVENT_ANY_ID, &ip_event_handler, NULL, &instance_ip_event_handler));
    ESP_LOGI(TAG, "WIFI and IP event handlers registered");


    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASSWORD,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_sta() finished");
}

void app_main(void)
{
    gpio_config_t pwdn_pin_config = {
        .pin_bit_mask = (1ULL<<CAM_PIN_PWDN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&pwdn_pin_config);

    // gpio_config_t flash_pin_config = {
    //     .pin_bit_mask = (1ULL<<CAM_PIN_FLASH),
    //     .mode = GPIO_MODE_OUTPUT,
    //     .pull_up_en = GPIO_PULLUP_DISABLE,
    //     .pull_down_en = GPIO_PULLDOWN_DISABLE,
    //     .intr_type = GPIO_INTR_DISABLE
    // };
    // gpio_config(&flash_pin_config);
    // gpio_set_level(CAM_PIN_FLASH, 1);

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    frameBuffer_queue = xQueueCreate( 3, sizeof( camera_fb_t* ) );

    wifi_init_sta();
}