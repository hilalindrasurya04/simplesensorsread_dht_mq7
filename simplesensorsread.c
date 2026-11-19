#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "nvs_flash.h"

#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "mqtt_client.h"

static const char *TAG = "IOT_NODE01";

//konfig jaringan broker
#define WIFI_SSID           "terkunci2"
#define WIFI_PASS           "buyung123"
#define MQTT_BROKER_URI     "mqtt://192.168.1.50:1883"
#define MQTT_USERNAME       "alexander"
#define MQTT_PASSWORD       "Jalansehat"
#define MQTT_TOPIC_PUB      "iot/alexander/esp32_node01/telemetry"

//konfig pin
#define DHT11_GPIO          GPIO_NUM_4
#define MQ7_ADC_CHANNEL     ADC_CHANNEL_6  // GPIO 34

static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT  BIT0

static esp_mqtt_client_handle_t mqtt_client = NULL;
static bool mqtt_connected = false;
static adc_oneshot_unit_handle_t adc1_handle;
static portMUX_TYPE dht11_mux = portMUX_INITIALIZER_UNLOCKED;

static int dht11_wait_level(int expected_level, int timeout_us) {
    int elapsed = 0;
    while (gpio_get_level(DHT11_GPIO) != expected_level) {
        if (elapsed >= timeout_us) {
            return -1;
        }
        esp_rom_delay_us(1);
        elapsed++;
    }
    return elapsed;
}

static esp_err_t dht11_read(float *temperature, float *humidity) {
    uint8_t data[5] = {0};

    gpio_set_direction(DHT11_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(DHT11_GPIO, 0);
    esp_rom_delay_us(20000);
    gpio_set_level(DHT11_GPIO, 1);
    esp_rom_delay_us(30);

    gpio_set_direction(DHT11_GPIO, GPIO_MODE_INPUT);
    gpio_set_pull_mode(DHT11_GPIO, GPIO_PULLUP_ONLY);

    taskENTER_CRITICAL(&dht11_mux);

    if (dht11_wait_level(0, 100) < 0 ||
        dht11_wait_level(1, 100) < 0 ||
        dht11_wait_level(0, 100) < 0) {
        taskEXIT_CRITICAL(&dht11_mux);
        return ESP_ERR_TIMEOUT;
    }

    for (int i = 0; i < 40; i++) {
        /* Setiap bit diawali ~50us LOW */
        if (dht11_wait_level(1, 70) < 0) {
            taskEXIT_CRITICAL(&dht11_mux);
            return ESP_ERR_TIMEOUT;
        }

        int high_duration = dht11_wait_level(0, 100);
        if (high_duration < 0) {
            taskEXIT_CRITICAL(&dht11_mux);
            return ESP_ERR_TIMEOUT;
        }

        data[i / 8] <<= 1;
        if (high_duration > 40) {
            data[i / 8] |= 1;
        }
    }

    taskEXIT_CRITICAL(&dht11_mux);

    uint8_t checksum = (data[0] + data[1] + data[2] + data[3]) & 0xFF;
    if (data[4] != checksum) {
        ESP_LOGE(TAG, "DHT11 Checksum error: calc=0x%02X, recv=0x%02X", checksum, data[4]);
        return ESP_ERR_INVALID_CRC;
    }

    *humidity    = (float)data[0] + ((float)data[1] * 0.1f);
    *temperature = (float)data[2] + ((float)data[3] * 0.1f);
    return ESP_OK;
}

static void adc_mq7_init(void) {
    adc_oneshot_unit_init_cfg_t init_config = { .unit_id = ADC_UNIT_1 };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc1_handle));

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = ADC_ATTEN_DB_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, MQ7_ADC_CHANNEL, &config));
}

static float mq7_read_co_ppm(void) {
    int raw_sum = 0;
    for (int i = 0; i < 16; i++) {
        int val = 0;
        adc_oneshot_read(adc1_handle, MQ7_ADC_CHANNEL, &val);
        raw_sum += val;
    }
    float raw_avg = raw_sum / 16.0f;

    float v_out = (raw_avg / 4095.0f) * 3.3f * 1.5f;
    if (v_out < 0.05f) v_out = 0.05f; // Proteksi pembagian nol

    float rs = ((5.0f - v_out) / v_out) * 10.0f;
    float ratio = rs / 10.0f;
    return 99.042f * powf(ratio, -1.518f);
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();
        ESP_LOGW(TAG, "Wi-Fi terputus, mencoba menghubungkan ulang...");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Mendapatkan IP: " IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "Terhubung ke Mosquitto Broker di Raspberry Pi");
            mqtt_connected = true;
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGW(TAG, "Terputus dari Mosquitto Broker");
            mqtt_connected = false;
            break;
        default:
            break;
    }
}

static void wifi_init_sta(void) {
    s_wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL);

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
}

static void mqtt_app_start(void) {
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = MQTT_BROKER_URI,
        .credentials.username = MQTT_USERNAME,
        .credentials.authentication.password = MQTT_PASSWORD,
        .credentials.client_id = "esp32_alexander_node01",
    };
    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(mqtt_client);
}

static void telemetry_task(void *pvParameters) {
    char payload[256];
    uint32_t seq = 0;
    float last_temp = 0.0f, last_hum = 0.0f;

    while (1) {
        if (mqtt_connected) {
            float temp = 0.0f, hum = 0.0f;
            if (dht11_read(&temp, &hum) == ESP_OK) {
                last_temp = temp;
                last_hum = hum;
            } else {
                ESP_LOGW(TAG, "Gagal membaca DHT11, menggunakan nilai valid terakhir");
            }

            float co_ppm = mq7_read_co_ppm();

            wifi_ap_record_t ap_info;
            int8_t rssi = (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) ? ap_info.rssi : 0;
            uint32_t uptime_sec = (uint32_t)(esp_timer_get_time() / 1000000ULL);

            snprintf(payload, sizeof(payload),
                     "{\"device_id\":\"esp32_node01\",\"seq\":%lu,\"uptime_s\":%lu,"
                     "\"rssi_dbm\":%d,\"sensors\":{\"temperature_c\":%.1f,"
                     "\"humidity_pct\":%.1f,\"co_ppm\":%.2f}}",
                     (unsigned long)++seq, (unsigned long)uptime_sec, rssi,
                     last_temp, last_hum, co_ppm);

            int msg_id = esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC_PUB, payload, 0, 1, 0);
            ESP_LOGI(TAG, "Published [msg_id=%d]: %s", msg_id, payload);
        }
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

void app_main(void) {
    ESP_ERROR_CHECK(nvs_flash_init());
    gpio_reset_pin(DHT11_GPIO);
    adc_mq7_init();

    wifi_init_sta();
    mqtt_app_start();

    xTaskCreate(telemetry_task, "telemetry_task", 4096, NULL, 5, NULL);
}