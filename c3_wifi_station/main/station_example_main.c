/**
 * ESP32 WiFi Station with Packet Receiver
 * 
 * This implementation receives and processes control and data packets
 * from the AP scheduler.
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_mac.h"

#include "lwip/err.h"
#include "lwip/sys.h"

/* WiFi configuration */
#define EXAMPLE_ESP_WIFI_SSID      CONFIG_ESP_WIFI_SSID
#define EXAMPLE_ESP_WIFI_PASS      CONFIG_ESP_WIFI_PASSWORD
#define EXAMPLE_ESP_WIFI_CHANNEL   CONFIG_ESP_WIFI_CHANNEL  /* Match this with AP */
#define EXAMPLE_ESP_MAXIMUM_RETRY  CONFIG_ESP_MAXIMUM_RETRY

/* Packet receiver configuration */
#define MAX_CLASSES                3     // 3 classes (Class 1: 3s, Class 2: 5s, Class 3: 6s)
#define MAX_PACKET_SIZE           1400   // Maximum packet data size
#define PROMISCUOUS_FILTER_MASK   WIFI_PROMIS_FILTER_MASK_DATA  // Only receive data frames
#define RX_TASK_STACK_SIZE        4096
#define RX_TASK_PRIORITY          5

/* Class definitions */
typedef enum {
    CLASS_1 = 0,                 // Class 1: 3-second period
    CLASS_2 = 1,                 // Class 2: 5-second period
    CLASS_3 = 2,                 // Class 3: 6-second period
} class_id_t;

/* Data type definitions */
typedef enum {
    DATA_TYPE_INT8 = 0,          // 8-bit integer
    DATA_TYPE_INT16 = 1,         // 16-bit integer
    DATA_TYPE_INT32 = 2,         // 32-bit integer
    DATA_TYPE_FLOAT = 3,         // 32-bit float
    DATA_TYPE_DOUBLE = 4,        // 64-bit double
} data_type_t;

/* Control packet structure - received before data transmission */
typedef struct {
    uint8_t class_counts[MAX_CLASSES]; // Number of elements for each class
    data_type_t class_types[MAX_CLASSES]; // Data type for each class
} __attribute__((packed)) control_packet_t;

/* Data packet header structure */
typedef struct {
    uint8_t class_counts[MAX_CLASSES]; // Number of items for each class
    uint16_t total_size;          // Total size of all data in bytes
    uint32_t timestamp;           // Transmission timestamp
} __attribute__((packed)) data_packet_header_t;

/* Receiver context */
typedef struct {
    SemaphoreHandle_t mutex;          // Mutex for operations
    TaskHandle_t receiver_task;       // Receiver task handle
    
    // Class information from last control packet
    data_type_t class_types[MAX_CLASSES];
    uint8_t class_counts[MAX_CLASSES];
    
    bool control_packet_received;     // Flag to indicate if we've received a control packet
    
    // Statistics
    uint32_t packets_received;       // Total packets received
    uint32_t control_packets;        // Control packets received
    uint32_t data_packets;           // Data packets received
    uint32_t error_packets;          // Packets with errors
    uint32_t current_time_ms;        // Current time in milliseconds
} receiver_context_t;

/* FreeRTOS event group to signal when we are connected */
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static const char *TAG = "wifi-sta-receiver";

static int s_retry_num = 0;

/* Global receiver context */
static receiver_context_t receiver_ctx;

/* Function prototypes */
static void receiver_task(void *pvParameters);
static void wifi_promiscuous_rx_cb(void *buf, wifi_promiscuous_pkt_type_t type);
static void process_control_packet(const uint8_t *data, size_t length);
static void process_data_packet(const uint8_t *data, size_t length);

/* Get the current time in milliseconds */
static uint32_t get_current_time_ms(void)
{
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

/* Initialize the packet receiver */
static void receiver_init(void)
{
    // Initialize mutex
    receiver_ctx.mutex = xSemaphoreCreateMutex();
    if (receiver_ctx.mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return;
    }

    // Initialize statistics and flags
    receiver_ctx.packets_received = 0;
    receiver_ctx.control_packets = 0;
    receiver_ctx.data_packets = 0;
    receiver_ctx.error_packets = 0;
    receiver_ctx.current_time_ms = 0;
    receiver_ctx.control_packet_received = false;
    
    // Create receiver task
    BaseType_t ret = xTaskCreate(
        receiver_task,             // Function that implements the task
        "receiver_task",           // Text name for the task
        RX_TASK_STACK_SIZE,        // Stack size in words
        NULL,                      // Parameter passed into the task
        RX_TASK_PRIORITY,          // Priority
        &receiver_ctx.receiver_task // Used to pass out the task handle
    );
    
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create receiver task");
        return;
    }
    
    ESP_LOGI(TAG, "Packet receiver initialized");
}

/* WiFi event handler */
static void event_handler(void* arg, esp_event_base_t event_base,
                           int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT) {
        switch (event_id) {
            case WIFI_EVENT_STA_START:
                ESP_LOGI(TAG, "WiFi station started, connecting to AP");
                esp_wifi_connect();
                break;
                
            case WIFI_EVENT_STA_CONNECTED:
                ESP_LOGI(TAG, "Connected to AP successfully!");
                break;
                
            case WIFI_EVENT_STA_DISCONNECTED: {
                wifi_event_sta_disconnected_t *event = (wifi_event_sta_disconnected_t*) event_data;
                ESP_LOGW(TAG, "Disconnected from AP, reason: %d", event->reason);
                
                // Print SSID we tried to connect to
                char ssid[33] = {0};
                if (event->ssid_len > 0 && event->ssid_len < 33) {
                    memcpy(ssid, event->ssid, event->ssid_len);
                    ESP_LOGW(TAG, "Failed SSID: '%s'", ssid);
                }
                
                if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
                    esp_wifi_connect();
                    s_retry_num++;
                    ESP_LOGI(TAG, "Retry %d to connect to the AP", s_retry_num);
                } else {
                    xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
                    ESP_LOGI(TAG, "Failed to connect to AP after maximum retries");
                }
                break;
            }
                
            default:
                ESP_LOGI(TAG, "Other WiFi event: %ld", event_id);
                break;
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP address: " IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        
        // After successful connection, enable promiscuous mode for packet capture
        ESP_LOGI(TAG, "Enabling promiscuous mode for packet capture");
        
        // Set filter for all packet types initially, then narrow down
        wifi_promiscuous_filter_t filter = {
            .filter_mask = WIFI_PROMIS_FILTER_MASK_ALL
        };
        esp_wifi_set_promiscuous_filter(&filter);
        
        // Register callback and enable promiscuous mode
        esp_wifi_set_promiscuous_rx_cb(wifi_promiscuous_rx_cb);
        esp_wifi_set_promiscuous(true);
        
        ESP_LOGI(TAG, "Promiscuous mode enabled successfully");
    }
}

void wifi_init_sta(void)
{
    ESP_LOGI(TAG, "Initializing WiFi in station mode");
    
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Register event handlers
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        NULL));

    // Configure WiFi station with hardcoded SSID and password to ensure match
    // Replace these with your actual AP SSID and password
    const char* wifi_ssid = "myssid";  // MUST MATCH YOUR AP SSID EXACTLY
    const char* wifi_password = "mypassword";  // MUST MATCH YOUR AP PASSWORD EXACTLY
    
    wifi_config_t wifi_config = {0};
    
    // Copy SSID and password to config
    strncpy((char*)wifi_config.sta.ssid, wifi_ssid, sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char*)wifi_config.sta.password, wifi_password, sizeof(wifi_config.sta.password) - 1);
    
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    wifi_config.sta.pmf_cfg.capable = true;
    wifi_config.sta.pmf_cfg.required = false;
    
    // Print connection details for debugging
    ESP_LOGI(TAG, "Setting WiFi configuration:");
    ESP_LOGI(TAG, "  SSID: %s", wifi_config.sta.ssid);
    ESP_LOGI(TAG, "  Password: %s", "********");  // Don't log actual password
    
    // Set the station to the same channel as your AP (e.g., 1)
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    
    // Set fixed channel to match AP if needed
    // Uncomment and set to your AP's channel
    //ESP_ERROR_CHECK(esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE));
    
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi station initialization completed");

    /* Wait until either the connection is established or connection failed */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* Check which event happened */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Connected to SSID: %s", wifi_config.sta.ssid);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGE(TAG, "Failed to connect to SSID: %s", wifi_config.sta.ssid);
        
        // If we failed to connect normally, enable promiscuous mode anyway
        // This allows us to see packets even without an established connection
        ESP_LOGI(TAG, "Enabling promiscuous mode without connection");
        
        wifi_promiscuous_filter_t filter = {
            .filter_mask = WIFI_PROMIS_FILTER_MASK_ALL  // Capture all packet types initially
        };
        esp_wifi_set_promiscuous_filter(&filter);
        esp_wifi_set_promiscuous_rx_cb(wifi_promiscuous_rx_cb);
        esp_wifi_set_promiscuous(true);
        
        // Scan for available networks to find out what's actually there
        ESP_LOGI(TAG, "Scanning for available networks...");
        wifi_scan_config_t scan_config = {
            .ssid = NULL,
            .bssid = NULL,
            .channel = 0,
            .show_hidden = true,
            .scan_type = WIFI_SCAN_TYPE_ACTIVE,
            .scan_time.active.min = 100,
            .scan_time.active.max = 300,
        };
        esp_wifi_scan_start(&scan_config, true);
        
        // Get scan results
        uint16_t ap_count = 0;
        esp_wifi_scan_get_ap_num(&ap_count);
        
        if (ap_count > 0) {
            wifi_ap_record_t *ap_records = calloc(ap_count, sizeof(wifi_ap_record_t));
            if (ap_records) {
                ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&ap_count, ap_records));
                
                ESP_LOGI(TAG, "Found %d access points:", ap_count);
                for (int i = 0; i < ap_count; i++) {
                    ESP_LOGI(TAG, "  %d: SSID '%s', Channel %d, RSSI %d", 
                            i, ap_records[i].ssid, ap_records[i].primary, ap_records[i].rssi);
                }
                
                // Try to find and match channel with our target SSID
                for (int i = 0; i < ap_count; i++) {
                    if (strcmp((char*)ap_records[i].ssid, wifi_ssid) == 0) {
                        ESP_LOGI(TAG, "Found our target AP! Channel: %d, RSSI: %d", 
                                ap_records[i].primary, ap_records[i].rssi);
                        
                        // Set our channel to match the found AP
                        ESP_LOGI(TAG, "Setting channel to %d", ap_records[i].primary);
                        esp_wifi_set_channel(ap_records[i].primary, WIFI_SECOND_CHAN_NONE);
                        break;
                    }
                }
                
                free(ap_records);
            }
        } else {
            ESP_LOGI(TAG, "No access points found in scan");
        }
    } else {
        ESP_LOGE(TAG, "Unexpected event during connection");
    }
}

/* WiFi promiscuous mode callback */
static void wifi_promiscuous_rx_cb(void *buf, wifi_promiscuous_pkt_type_t type)
{
    if (type != WIFI_PKT_DATA) {
        return;  // We're only interested in data packets
    }
    
    const wifi_promiscuous_pkt_t *pkt = (wifi_promiscuous_pkt_t *)buf;
    const uint8_t *payload = pkt->payload;
    
    // Update current time
    receiver_ctx.current_time_ms = get_current_time_ms();
    
    // We need at least 24 bytes for the basic 802.11 header
    if (pkt->rx_ctrl.sig_len < 24) {
        return;
    }
    
    // Parse 802.11 header
    uint8_t frame_control_1 = payload[0];
    uint8_t frame_control_2 = payload[1];
    uint8_t frame_type = (frame_control_1 & 0x0C);
    //uint8_t frame_subtype = (frame_control_1 & 0xF0) >> 4;
    uint8_t from_ds = (frame_control_2 & 0x02) >> 1;
    uint8_t to_ds = (frame_control_2 & 0x01);
    
    // Check if this is a data frame (type = 0x08) with proper flags
    if (frame_type != 0x08 || from_ds != 1 || to_ds != 0) {
        return;  // Not a data frame from AP to station
    }
    
    // Extract MAC addresses from header to check if this packet is intended for us
    // (or is a broadcast packet)
    uint8_t destination_mac[6];
    memcpy(destination_mac, &payload[4], 6);

    // Check for broadcast (FF:FF:FF:FF:FF:FF) or our specific MAC
    bool is_broadcast = true;
    for (int i = 0; i < 6; i++) {
        if (destination_mac[i] != 0xFF) {
            is_broadcast = false;
            break;
        }
    }
    
    // Get our MAC address
    uint8_t our_mac[6];
    esp_wifi_get_mac(WIFI_IF_STA, our_mac);
    
    // Check if packet is for us (either broadcast or specific)
    bool is_for_us = is_broadcast;
    if (!is_for_us) {
        is_for_us = true;
        for (int i = 0; i < 6; i++) {
            if (destination_mac[i] != our_mac[i]) {
                is_for_us = false;
                break;
            }
        }
    }
    
    if (!is_for_us) {
        return;  // Packet not intended for us
    }
    
    // Increment packet count
    if (xSemaphoreTake(receiver_ctx.mutex, portMAX_DELAY) == pdTRUE) {
        receiver_ctx.packets_received++;
        xSemaphoreGive(receiver_ctx.mutex);
    }
    
    // Skip the 802.11 header (24 bytes) to get to our payload
    const uint8_t *data = payload + 24;
    size_t data_len = pkt->rx_ctrl.sig_len - 24;
    
    // Check packet size - we need at least sizeof(control_packet_t) or sizeof(data_packet_header_t)
    if (data_len < sizeof(control_packet_t)) {
        if (xSemaphoreTake(receiver_ctx.mutex, portMAX_DELAY) == pdTRUE) {
            receiver_ctx.error_packets++;
            xSemaphoreGive(receiver_ctx.mutex);
        }
        ESP_LOGW(TAG, "Received packet too small: %d bytes", data_len);
        return;
    }
    
    // Determine if this is a control or data packet
    // Control packet is smaller, so we check size compared to data packet header
    if (data_len == sizeof(control_packet_t)) {
        process_control_packet(data, data_len);
    } else if (data_len >= sizeof(data_packet_header_t)) {
        process_data_packet(data, data_len);
    } else {
        // Unexpected packet size
        if (xSemaphoreTake(receiver_ctx.mutex, portMAX_DELAY) == pdTRUE) {
            receiver_ctx.error_packets++;
            xSemaphoreGive(receiver_ctx.mutex);
        }
        ESP_LOGW(TAG, "Received packet with unexpected size: %d bytes", data_len);
    }
}

/* Process a control packet */
static void process_control_packet(const uint8_t *data, size_t length)
{
    // Make sure the packet is the right size
    if (length < sizeof(control_packet_t)) {
        ESP_LOGE(TAG, "Control packet too small: %d bytes", length);
        return;
    }
    
    // Cast to control packet structure
    const control_packet_t *control = (const control_packet_t *)data;
    
    // Store control packet information
    if (xSemaphoreTake(receiver_ctx.mutex, portMAX_DELAY) == pdTRUE) {
        // Copy class types and counts
        for (int i = 0; i < MAX_CLASSES; i++) {
            receiver_ctx.class_types[i] = control->class_types[i];
            receiver_ctx.class_counts[i] = control->class_counts[i];
        }
        
        receiver_ctx.control_packet_received = true;
        receiver_ctx.control_packets++;
        
        xSemaphoreGive(receiver_ctx.mutex);
    }
    
    ESP_LOGI(TAG, "Received control packet: Class1=%d(%d), Class2=%d(%d), Class3=%d(%d)",
             control->class_counts[0], control->class_types[0],
             control->class_counts[1], control->class_types[1],
             control->class_counts[2], control->class_types[2]);
}

/* Process a data packet */
static void process_data_packet(const uint8_t *data, size_t length)
{
    // Make sure the packet is the right size
    if (length < sizeof(data_packet_header_t)) {
        ESP_LOGE(TAG, "Data packet too small: %d bytes", length);
        return;
    }
    
    // Cast to data packet header structure
    const data_packet_header_t *header = (const data_packet_header_t *)data;
    
    // Make sure we have received a control packet first
    bool have_control_packet = false;
    if (xSemaphoreTake(receiver_ctx.mutex, portMAX_DELAY) == pdTRUE) {
        have_control_packet = receiver_ctx.control_packet_received;
        receiver_ctx.data_packets++;
        xSemaphoreGive(receiver_ctx.mutex);
    }
    
    if (!have_control_packet) {
        ESP_LOGW(TAG, "Received data packet before control packet - discarding");
        return;
    }
    
    // Get data pointer (after header)
    const uint8_t *payload = data + sizeof(data_packet_header_t);
    
    // Verify the total size matches what's in the header
    if (length - sizeof(data_packet_header_t) != header->total_size) {
        ESP_LOGW(TAG, "Data packet size mismatch: expected %d, got %d",
                 header->total_size, length - sizeof(data_packet_header_t));
    }
    
    // Calculate latency (time from transmission to reception)
    uint32_t current_time = get_current_time_ms();
    uint32_t latency = current_time - header->timestamp;
    
    ESP_LOGI(TAG, "Received data packet: Class1=%d, Class2=%d, Class3=%d, Size=%d, Latency=%lu ms",
             header->class_counts[0], header->class_counts[1], header->class_counts[2],
             header->total_size, latency);
    
    // Process the data for each class
    const uint8_t *class_data = payload;
    
    // Take mutex to get class types
    data_type_t class_types[MAX_CLASSES];
    if (xSemaphoreTake(receiver_ctx.mutex, portMAX_DELAY) == pdTRUE) {
        for (int i = 0; i < MAX_CLASSES; i++) {
            class_types[i] = receiver_ctx.class_types[i];
        }
        xSemaphoreGive(receiver_ctx.mutex);
    }
    
    // Process each class's data
    for (int class_id = 0; class_id < MAX_CLASSES; class_id++) {
        if (header->class_counts[class_id] == 0) {
            continue;  // No data for this class
        }
        
        // Calculate element size based on data type
        uint16_t element_size = 0;
        switch (class_types[class_id]) {
            case DATA_TYPE_INT8:   element_size = 1; break;
            case DATA_TYPE_INT16:  element_size = 2; break;
            case DATA_TYPE_INT32:
            case DATA_TYPE_FLOAT:  element_size = 4; break;
            case DATA_TYPE_DOUBLE: element_size = 8; break;
            default:               element_size = 4; break;
        }
        
        // Calculate total size for this class
        uint16_t class_size = element_size * header->class_counts[class_id];
        
        // Process based on data type
        ESP_LOGI(TAG, "  Class %d data (%d elements, type %d):",
                 class_id + 1, header->class_counts[class_id], class_types[class_id]);
        
        // Display a few samples of the data based on type
        switch (class_types[class_id]) {
            case DATA_TYPE_INT8: {
                const int8_t *values = (const int8_t *)class_data;
                int display_count = header->class_counts[class_id] > 5 ? 5 : header->class_counts[class_id];
                for (int i = 0; i < display_count; i++) {
                    ESP_LOGI(TAG, "    [%d] = %d", i, values[i]);
                }
                break;
            }
            
            case DATA_TYPE_INT16: {
                const int16_t *values = (const int16_t *)class_data;
                int display_count = header->class_counts[class_id] > 5 ? 5 : header->class_counts[class_id];
                for (int i = 0; i < display_count; i++) {
                    ESP_LOGI(TAG, "    [%d] = %d", i, values[i]);
                }
                break;
            }
            
            case DATA_TYPE_INT32: {
                const int32_t *values = (const int32_t *)class_data;
                int display_count = header->class_counts[class_id] > 5 ? 5 : header->class_counts[class_id];
                for (int i = 0; i < display_count; i++) {
                    ESP_LOGI(TAG, "    [%d] = %ld", i, values[i]);
                }
                break;
            }
            
            case DATA_TYPE_FLOAT: {
                const float *values = (const float *)class_data;
                int display_count = header->class_counts[class_id] > 5 ? 5 : header->class_counts[class_id];
                for (int i = 0; i < display_count; i++) {
                    ESP_LOGI(TAG, "    [%d] = %f", i, values[i]);
                }
                break;
            }
            
            case DATA_TYPE_DOUBLE: {
                const double *values = (const double *)class_data;
                int display_count = header->class_counts[class_id] > 5 ? 5 : header->class_counts[class_id];
                for (int i = 0; i < display_count; i++) {
                    ESP_LOGI(TAG, "    [%d] = %f", i, values[i]);
                }
                break;
            }
            
            default:
                ESP_LOGI(TAG, "    (Data type not handled)");
                break;
        }
        
        // If there are more elements than we displayed, indicate this
        if (header->class_counts[class_id] > 5) {
            ESP_LOGI(TAG, "    ... (%d more elements not shown)",
                     header->class_counts[class_id] - 5);
        }
        
        // Move to next class's data
        class_data += class_size;
    }
}

/* Main receiver task */
static void receiver_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Receiver task started");
    
    // This task just periodically prints statistics
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t stats_interval = pdMS_TO_TICKS(5000);  // 5 seconds
    
    while (1) {
        // Wait for the next interval
        vTaskDelayUntil(&last_wake_time, stats_interval);
        
        // Print statistics
        if (xSemaphoreTake(receiver_ctx.mutex, portMAX_DELAY) == pdTRUE) {
            ESP_LOGI(TAG, "Receiver Statistics:");
            ESP_LOGI(TAG, "  Packets received: %lu", receiver_ctx.packets_received);
            ESP_LOGI(TAG, "  Control packets: %lu", receiver_ctx.control_packets);
            ESP_LOGI(TAG, "  Data packets: %lu", receiver_ctx.data_packets);
            ESP_LOGI(TAG, "  Error packets: %lu", receiver_ctx.error_packets);
            xSemaphoreGive(receiver_ctx.mutex);
        }
    }
}

/* Main application entry point */
void app_main(void)
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize WiFi in station mode with minimal setup
    ESP_LOGI(TAG, "Initializing WiFi in promiscuous mode");
    
    // Initialize the TCP/IP stack
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    // Initialize WiFi with default config
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    
    // Set to the same channel as your AP
    ESP_ERROR_CHECK(esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE));
    
    // Enable promiscuous mode for packet capture
    ESP_LOGI(TAG, "Enabling promiscuous mode for packet capture");
    wifi_promiscuous_filter_t filter = {
        .filter_mask = WIFI_PROMIS_FILTER_MASK_ALL  // Capture all packets
    };
    esp_wifi_set_promiscuous_filter(&filter);
    esp_wifi_set_promiscuous_rx_cb(wifi_promiscuous_rx_cb);
    esp_wifi_set_promiscuous(true);
    
    // Initialize packet receiver
    ESP_LOGI(TAG, "Initializing packet receiver");
    receiver_init();
    
    ESP_LOGI(TAG, "Station in promiscuous mode, waiting for packets");
}