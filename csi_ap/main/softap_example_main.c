#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_mac.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_now.h"

#include "lwip/err.h"
#include "lwip/sys.h"

/* WiFi configuration macros */
#define EXAMPLE_ESP_WIFI_SSID      CONFIG_ESP_WIFI_SSID
#define EXAMPLE_ESP_WIFI_PASS      CONFIG_ESP_WIFI_PASSWORD
#define EXAMPLE_ESP_WIFI_CHANNEL   CONFIG_ESP_WIFI_CHANNEL
#define EXAMPLE_MAX_STA_CONN       CONFIG_ESP_MAX_STA_CONN

/* CSI Configuration */
#define CONFIG_LESS_INTERFERENCE_CHANNEL    11   // WiFi channel with less interference
#define CONFIG_SEND_FREQUENCY              100   // CSI send frequency in Hz
static const uint8_t CONFIG_CSI_SEND_MAC[] = {0x1a, 0x00, 0x00, 0x00, 0x00, 0x00};

static const char *TAG = "wifi softAP";
static const char *CSI_TAG = "csi_send";

/* Task handle for CSI sending task */
static TaskHandle_t csi_send_task_handle = NULL;

/* ESP-NOW broadcast address */
static const uint8_t broadcast_mac[ESP_NOW_ETH_ALEN] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

/* WiFi event handler function */
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                    int32_t event_id, void* event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        ESP_LOGI(TAG, "station "MACSTR" join, AID=%d",
                 MAC2STR(event->mac), event->aid);
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        ESP_LOGI(TAG, "station "MACSTR" leave, AID=%d, reason=%d",
                 MAC2STR(event->mac), event->aid, event->reason);
    }
}

/**
 * @brief Task for sending CSI packets at regular intervals
 * 
 * This task sends ESP-NOW packets at the configured frequency
 * to allow CSI measurements by the receiver.
 */
static void csi_send_task(void *pvParameters)
{
    ESP_LOGI(CSI_TAG, "CSI send task started");
    ESP_LOGI(CSI_TAG, "================ CSI SEND ================");
    ESP_LOGI(CSI_TAG, "wifi_channel: %d, send_frequency: %d, mac: " MACSTR,
             CONFIG_LESS_INTERFERENCE_CHANNEL, CONFIG_SEND_FREQUENCY, MAC2STR(CONFIG_CSI_SEND_MAC));
    
    // Counter for the packet sequence
    uint8_t count = 0;
    
    while (1) {
        // Send a packet using ESP-NOW
        esp_err_t ret = esp_now_send(broadcast_mac, &count, sizeof(uint8_t));
        if (ret != ESP_OK) {
            ESP_LOGW(CSI_TAG, "<%s> ESP-NOW send error", esp_err_to_name(ret));
        } else {
            if (count % 100 == 0) {  // Log only every 100 packets to reduce output
                ESP_LOGI(CSI_TAG, "Sent CSI packet #%d", count);
            }
        }
        
        // Increment counter
        count++;
        
        // Wait for the next interval
        vTaskDelay(1000 / CONFIG_SEND_FREQUENCY / portTICK_PERIOD_MS);
    }
}

/**
 * @brief Initialize ESP-NOW for CSI packet transmission
 */
static void esp_now_init_csi()
{
    // Initialize ESP-NOW
    ESP_ERROR_CHECK(esp_now_init());
    
    // Set PMK (Primary Master Key)
    ESP_ERROR_CHECK(esp_now_set_pmk((uint8_t *)"pmk1234567890123"));
    
    // Register broadcast peer
    esp_now_peer_info_t peer = {
        .channel   = CONFIG_LESS_INTERFERENCE_CHANNEL,
        .ifidx     = WIFI_IF_AP,     // Use AP interface for sending packets
        .encrypt   = false,          // No encryption for simplicity
        .peer_addr = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff},  // Broadcast address
    };
    
    // Add broadcast peer
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));
}

/**
 * @brief Initialize WiFi in AP mode with CSI capabilities
 */
void wifi_init_softap(void)
{
    // Initialize network interface
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    // Initialize WiFi with default config
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    
    // Get and print the default MAC address (before WiFi is started)
    uint8_t default_mac[6];
    esp_read_mac(default_mac, ESP_MAC_WIFI_SOFTAP);
    ESP_LOGI(TAG, "Default AP MAC Address: "MACSTR, MAC2STR(default_mac));

    // Register event handler
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));

    // Configure AP mode
    wifi_config_t wifi_config = {
        .ap = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .ssid_len = strlen(EXAMPLE_ESP_WIFI_SSID),
            .channel = CONFIG_LESS_INTERFERENCE_CHANNEL,  // Use the same channel as CSI
            .password = EXAMPLE_ESP_WIFI_PASS,
            .max_connection = EXAMPLE_MAX_STA_CONN,
#ifdef CONFIG_ESP_WIFI_SOFTAP_SAE_SUPPORT
            .authmode = WIFI_AUTH_WPA3_PSK,
            .sae_pwe_h2e = WPA3_SAE_PWE_BOTH,
#else /* CONFIG_ESP_WIFI_SOFTAP_SAE_SUPPORT */
            .authmode = WIFI_AUTH_WPA2_PSK,
#endif
            .pmf_cfg = {
                    .required = true,
            },
        },
    };
    
    // If no password, use open authentication
    if (strlen(EXAMPLE_ESP_WIFI_PASS) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    // Set WiFi mode and config
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    
    // Additional configurations for CSI
    ESP_ERROR_CHECK(esp_wifi_set_bandwidth(WIFI_IF_AP, WIFI_BW_HT40));  // Use 40MHz bandwidth for better CSI
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));  // Disable power saving for consistent transmission
    
    // Option: Set a specific MAC address for CSI
    // Uncomment if you want to use a specific MAC for the AP
    // ESP_ERROR_CHECK(esp_wifi_set_mac(WIFI_IF_AP, CONFIG_CSI_SEND_MAC));
    
    // Start WiFi
    ESP_ERROR_CHECK(esp_wifi_start());
    
    // Get and print the actual AP MAC address after WiFi is started
    uint8_t active_mac[6];
    ESP_ERROR_CHECK(esp_wifi_get_mac(WIFI_IF_AP, active_mac));
    ESP_LOGI(TAG, "Active AP MAC Address: "MACSTR, MAC2STR(active_mac));

    // Initialize ESP-NOW for CSI sending
    esp_now_init_csi();
    
    // Create a task for sending CSI packets
    xTaskCreate(csi_send_task, "csi_send_task", 4096, NULL, 5, &csi_send_task_handle);

    ESP_LOGI(TAG, "wifi_init_softap finished. SSID:%s password:%s channel:%d",
             EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS, CONFIG_LESS_INTERFERENCE_CHANNEL);
}

void app_main(void)
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "ESP_WIFI_MODE_AP");
    wifi_init_softap();
}