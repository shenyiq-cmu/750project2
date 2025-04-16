#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_mac.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_now.h"

#include "csi_sender.h"

static const char *CSI_TAG = "csi_send";
static TaskHandle_t csi_send_task_handle = NULL;
static uint32_t total_sent_count = 0;

/* ESP-NOW broadcast address */
static const uint8_t broadcast_mac[ESP_NOW_ETH_ALEN] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

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
    
    // Get current MAC to print in logs
    uint8_t mac[6];
    esp_wifi_get_mac(WIFI_IF_AP, mac);
    
    ESP_LOGI(CSI_TAG, "wifi_channel: %d, send_frequency: %d, mac: " MACSTR,
             CONFIG_LESS_INTERFERENCE_CHANNEL, CONFIG_SEND_FREQUENCY, MAC2STR(mac));
    
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
            total_sent_count++;
        }
        
        // Increment counter
        count++;
        
        // Wait for the next interval
        vTaskDelay(1000 / CONFIG_SEND_FREQUENCY / portTICK_PERIOD_MS);
    }
}

void csi_sender_init(void)
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
    
    ESP_LOGI(CSI_TAG, "CSI sender initialized");
}

void csi_sender_start(void)
{
    // Create a task for sending CSI packets
    xTaskCreate(csi_send_task, "csi_send_task", 4096, NULL, 5, &csi_send_task_handle);
}

uint32_t csi_sender_get_count(void)
{
    return total_sent_count;
}