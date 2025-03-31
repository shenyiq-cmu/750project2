/**
 * ESP32 WiFi Access Point with Packet Receiver
 * 
 * This implementation creates a WiFi access point and then receives
 * and processes data packets from connected stations.
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
#define EXAMPLE_ESP_WIFI_CHANNEL   CONFIG_ESP_WIFI_CHANNEL
#define EXAMPLE_MAX_STA_CONN       CONFIG_ESP_MAX_STA_CONN

/* Packet receiver configuration */
#define MAX_CLASSES                3     // 3 classes (Class 1: 3s, Class 2: 5s, Class 3: 6s)
#define MAX_PACKET_SIZE           1400   // Maximum packet data size
#define PROMISCUOUS_FILTER_MASK   WIFI_PROMIS_FILTER_MASK_DATA  // Only receive data frames
#define RX_TASK_STACK_SIZE        4096
#define RX_TASK_PRIORITY          5

static const char *TAG = "wifi-ap-receiver";

/* Add a reception counter to track packet sequence */
static uint32_t rx_packet_counter = 0;

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

/* Updated data packet header (now includes all necessary information) */
typedef struct {
    uint8_t class_counts[MAX_CLASSES];      // Number of items for each class
    data_type_t class_types[MAX_CLASSES];   // Data type for each class
    uint16_t total_size;                    // Total size of all data in bytes
    uint32_t timestamp;                     // Transmission timestamp
} __attribute__((packed)) data_packet_header_t;

/* Receiver context */
typedef struct {
    SemaphoreHandle_t mutex;          // Mutex for operations
    TaskHandle_t receiver_task;       // Receiver task handle
    
    // Class information from last received packet
    data_type_t class_types[MAX_CLASSES];
    uint8_t class_counts[MAX_CLASSES];
    
    // Statistics
    uint32_t packets_received;       // Total packets received
    uint32_t data_packets;           // Data packets received
    uint32_t error_packets;          // Packets with errors
    uint32_t current_time_ms;        // Current time in milliseconds
} receiver_context_t;

/* Global receiver context */
static receiver_context_t receiver_ctx;

/* Function prototypes */
static void receiver_task(void *pvParameters);
static void wifi_promiscuous_rx_cb(void *buf, wifi_promiscuous_pkt_type_t type);
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
    receiver_ctx.data_packets = 0;
    receiver_ctx.error_packets = 0;
    receiver_ctx.current_time_ms = 0;
    
    // Initialize class types to sensible defaults
    for (int i = 0; i < MAX_CLASSES; i++) {
        receiver_ctx.class_types[i] = DATA_TYPE_INT32;  // Default to INT32
        receiver_ctx.class_counts[i] = 0;
    }
    
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

/* Initialize WiFi in AP mode */
void wifi_init_softap(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));

    // Configure with hardcoded SSID and password to ensure match with station
    const char* wifi_ssid = "myssid1";  // MUST MATCH STA SSID
    const char* wifi_password = "mypassword1";  // MUST MATCH STA PASSWORD
    
    wifi_config_t wifi_config = {
        .ap = {
            .channel = EXAMPLE_ESP_WIFI_CHANNEL,
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
    
    // Copy SSID and password to config
    strncpy((char*)wifi_config.ap.ssid, wifi_ssid, sizeof(wifi_config.ap.ssid) - 1);
    wifi_config.ap.ssid_len = strlen(wifi_ssid);
    strncpy((char*)wifi_config.ap.password, wifi_password, sizeof(wifi_config.ap.password) - 1);
    
    if (strlen(wifi_password) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    // Print connection details
    ESP_LOGI(TAG, "Setting WiFi configuration:");
    ESP_LOGI(TAG, "  SSID: %s", wifi_config.ap.ssid);
    ESP_LOGI(TAG, "  Password: %s", "********");  // Don't log actual password
    ESP_LOGI(TAG, "  Channel: %d", wifi_config.ap.channel);
    
    ESP_LOGI(TAG, "wifi_init_softap finished.");
}

/* Enable promiscuous mode for packet capturing */
static void enable_promiscuous_mode(void)
{
    ESP_LOGI(TAG, "Enabling promiscuous mode for packet capture");
    
    // Set filter for data packets
    wifi_promiscuous_filter_t filter = {
        .filter_mask = PROMISCUOUS_FILTER_MASK
    };
    esp_wifi_set_promiscuous_filter(&filter);
    
    // Register callback and enable promiscuous mode
    esp_wifi_set_promiscuous_rx_cb(wifi_promiscuous_rx_cb);
    esp_wifi_set_promiscuous(true);
    
    ESP_LOGI(TAG, "Promiscuous mode enabled successfully");
}

/* WiFi promiscuous mode callback */
/* WiFi promiscuous mode callback */
static void wifi_promiscuous_rx_cb(void *buf, wifi_promiscuous_pkt_type_t type)
{
    if (type != WIFI_PKT_DATA) {
        return;  // We're only interested in data packets
    }
    
    const wifi_promiscuous_pkt_t *pkt = (wifi_promiscuous_pkt_t *)buf;
    const uint8_t *payload = pkt->payload;
    const size_t pkt_len = pkt->rx_ctrl.sig_len;
    
    // Update current time
    receiver_ctx.current_time_ms = get_current_time_ms();
    
    // Basic validation - we need at least 24 bytes for the basic 802.11 header
    if (pkt_len < 24) {
        ESP_LOGD(TAG, "Packet too small for 802.11 header: %d bytes", pkt_len);
        return;
    }
    
    // We need at least enough data for header + our data packet header
    if (pkt_len < 24 + sizeof(data_packet_header_t)) {
        ESP_LOGD(TAG, "Packet too small for data packet header: %d bytes", pkt_len);
        return;
    }
    
    // Parse 802.11 header
    uint8_t frame_control_1 = payload[0];
    uint8_t frame_control_2 = payload[1];
    uint8_t frame_type = (frame_control_1 & 0x0C);
    uint8_t from_ds = (frame_control_2 & 0x02) >> 1;
    uint8_t to_ds = (frame_control_2 & 0x01);
    
    // Check if this is a data frame (type = 0x08) with proper flags
    // For AP receiving, we want packets from stations (to_ds=1, from_ds=0)
    if (frame_type != 0x08 || from_ds != 0 || to_ds != 1) {
        return;  // Not a data frame from station to AP
    }
    
    // Extract MAC addresses from header to check if this packet is intended for us
    uint8_t destination_mac[6];
    memcpy(destination_mac, &payload[4], 6);

    // Get our MAC address
    uint8_t our_mac[6];
    esp_wifi_get_mac(WIFI_IF_AP, our_mac);
    
    // Check if packet is for us or broadcast
    bool is_for_us = true;
    bool is_broadcast = true;
    
    for (int i = 0; i < 6; i++) {
        if (destination_mac[i] != our_mac[i]) {
            is_for_us = false;
        }
        if (destination_mac[i] != 0xFF) {
            is_broadcast = false;
        }
    }
    
    if (!is_for_us && !is_broadcast) {
        return;  // Packet not intended for us
    }
    
    // Skip the 802.11 header (24 bytes) to get to our payload
    const uint8_t *data = payload + 24;
    size_t data_len = pkt_len - 24;
    
    // Basic size validation for our header
    if (data_len < sizeof(data_packet_header_t)) {
        if (xSemaphoreTake(receiver_ctx.mutex, portMAX_DELAY) == pdTRUE) {
            receiver_ctx.error_packets++;
            xSemaphoreGive(receiver_ctx.mutex);
        }
        ESP_LOGW(TAG, "Received packet too small: %d bytes (expected at least %d)", 
                data_len, sizeof(data_packet_header_t));
        return;
    }
    
    // Do a basic validation of the data packet header
    const data_packet_header_t *header = (const data_packet_header_t *)data;
    
    // Validate total size is reasonable
    if (header->total_size > MAX_PACKET_SIZE) {
        ESP_LOGW(TAG, "Invalid total size in header: %d (max allowed: %d)", 
                 header->total_size, MAX_PACKET_SIZE);
        if (xSemaphoreTake(receiver_ctx.mutex, portMAX_DELAY) == pdTRUE) {
            receiver_ctx.error_packets++;
            xSemaphoreGive(receiver_ctx.mutex);
        }
        return;
    }
    
    // Validate class types
    bool valid_types = true;
    for (int i = 0; i < MAX_CLASSES; i++) {
        if (header->class_types[i] > DATA_TYPE_DOUBLE) {
            valid_types = false;
            break;
        }
    }
    
    if (!valid_types) {
        ESP_LOGW(TAG, "Invalid class types in header");
        if (xSemaphoreTake(receiver_ctx.mutex, portMAX_DELAY) == pdTRUE) {
            receiver_ctx.error_packets++;
            xSemaphoreGive(receiver_ctx.mutex);
        }
        return;
    }
    
    // Check if we have enough data for header + payload
    if (data_len < sizeof(data_packet_header_t) + header->total_size) {
        ESP_LOGW(TAG, "Insufficient data: header indicates %d data bytes, packet has %d bytes available",
                 header->total_size, data_len - sizeof(data_packet_header_t));
    }
    
    // Increment packet count only for valid packets
    if (xSemaphoreTake(receiver_ctx.mutex, portMAX_DELAY) == pdTRUE) {
        receiver_ctx.packets_received++;
        xSemaphoreGive(receiver_ctx.mutex);
    }
    
    // Process the data packet
    process_data_packet(data, data_len);
}

/* Process a data packet */
/* Process a data packet */
static void process_data_packet(const uint8_t *data, size_t length)
{
    // Make sure the packet is at least as large as our header
    if (length < sizeof(data_packet_header_t)) {
        ESP_LOGE(TAG, "Data packet too small: %d bytes (minimum size: %d)", 
                 length, sizeof(data_packet_header_t));
        return;
    }
    
    // Cast to data packet header structure
    const data_packet_header_t *header = (const data_packet_header_t *)data;
    
    // Validate total size
    if (header->total_size > MAX_PACKET_SIZE) {
        ESP_LOGE(TAG, "Invalid total size in header: %d (max allowed: %d)", 
                 header->total_size, MAX_PACKET_SIZE);
        return;
    }
    
    // Validate class types (must be between 0-4)
    for (int i = 0; i < MAX_CLASSES; i++) {
        if (header->class_types[i] > DATA_TYPE_DOUBLE) {
            ESP_LOGE(TAG, "Invalid class type for class %d: %d", i, header->class_types[i]);
            return;
        }
    }
    
    // Calculate expected total data size based on class counts and types
    uint16_t expected_size = 0;
    for (int i = 0; i < MAX_CLASSES; i++) {
        if (header->class_counts[i] > 0) {
            uint16_t element_size = 0;
            switch (header->class_types[i]) {
                case DATA_TYPE_INT8:   element_size = 1; break;
                case DATA_TYPE_INT16:  element_size = 2; break;
                case DATA_TYPE_INT32:  
                case DATA_TYPE_FLOAT:  element_size = 4; break;
                case DATA_TYPE_DOUBLE: element_size = 8; break;
                default:               element_size = 0; break;  // Should not happen after validation
            }
            expected_size += element_size * header->class_counts[i];
        }
    }
    
    // Verify the total size in header matches our calculation
    if (expected_size != header->total_size) {
        ESP_LOGW(TAG, "Size mismatch: header says %d, calculated %d", 
                 header->total_size, expected_size);
    }
    
    // Verify actual received data size matches or exceeds what we need
    if (length - sizeof(data_packet_header_t) < header->total_size) {
        ESP_LOGW(TAG, "Data packet size mismatch: expected %d, got %d",
                 header->total_size, length - sizeof(data_packet_header_t));
        // We'll continue processing what we can
    }

    // Print transmission summary (matching the station's output format)
    ESP_LOGI(TAG, "=============================================================");
    ESP_LOGI(TAG, "Received packet #%lu", rx_packet_counter);
    ESP_LOGI(TAG, "  Total data size: %d bytes", header->total_size);
    ESP_LOGI(TAG, "  Sent data packet: Class1=%ditem(type%d), Class2=%ditem(type%d), Class3=%ditem(type%d)",
                header->class_counts[0], header->class_types[0],
                header->class_counts[1], header->class_types[1],
                header->class_counts[2], header->class_types[2]);
    // ESP_LOGI(TAG, "  Transmission timestamp: %lu", header->timestamp);
    // ESP_LOGI(TAG, "  Reception timestamp: %lu", current_time);
    // ESP_LOGI(TAG, "  Latency: %lu ms", latency);
    
    // Store class information from the packet
    if (xSemaphoreTake(receiver_ctx.mutex, portMAX_DELAY) == pdTRUE) {
        // Copy class types and counts
        for (int i = 0; i < MAX_CLASSES; i++) {
            receiver_ctx.class_types[i] = header->class_types[i];
            receiver_ctx.class_counts[i] = header->class_counts[i];
        }
        
        receiver_ctx.data_packets++;
        xSemaphoreGive(receiver_ctx.mutex);
    }
    
    // Get data pointer (after header)
    const uint8_t *payload = data + sizeof(data_packet_header_t);
    
    // Calculate latency (time from transmission to reception)
    uint32_t current_time = get_current_time_ms();
    uint32_t packet_timestamp = header->timestamp;
    
    // Validate timestamp (avoid huge latency values)
    uint32_t latency;
    if (current_time >= packet_timestamp) {
        latency = current_time - packet_timestamp;
        // If latency is unreasonably large (> 30 seconds), it's probably a bad timestamp
        if (latency > 30000) {
            ESP_LOGW(TAG, "Suspicious latency value: %lu ms, using 0", latency);
            latency = 0;
        }
    } else {
        // Handle timestamp rollover or invalid timestamps
        ESP_LOGW(TAG, "Invalid timestamp: %lu > %lu, using 0", packet_timestamp, current_time);
        latency = 0;
    }
    
    ESP_LOGI(TAG, "Received data packet: Class1=%d(%d), Class2=%d(%d), Class3=%d(%d), Size=%d, Latency=%lu ms",
             header->class_counts[0], header->class_types[0],
             header->class_counts[1], header->class_types[1],
             header->class_counts[2], header->class_types[2],
             header->total_size, latency);
    
    // Process the data for each class
    const uint8_t *class_data = payload;
    
    // Process each class's data
    for (int class_id = 0; class_id < MAX_CLASSES; class_id++) {
        if (header->class_counts[class_id] == 0) {
            continue;  // No data for this class
        }
        
        // Calculate element size based on data type
        uint16_t element_size = 0;
        switch (header->class_types[class_id]) {
            case DATA_TYPE_INT8:   element_size = 1; break;
            case DATA_TYPE_INT16:  element_size = 2; break;
            case DATA_TYPE_INT32:
            case DATA_TYPE_FLOAT:  element_size = 4; break;
            case DATA_TYPE_DOUBLE: element_size = 8; break;
            default:
                ESP_LOGE(TAG, "Unexpected data type %d for class %d", 
                        header->class_types[class_id], class_id);
                continue;  // Skip this class
        }
        
        // Calculate total size for this class
        uint16_t class_size = element_size * header->class_counts[class_id];
        
        // Make sure we have enough data
        size_t remaining_bytes = (payload + header->total_size) - class_data;
        if (remaining_bytes < class_size) {
            ESP_LOGW(TAG, "Not enough data for class %d: need %d bytes, have %d",
                     class_id, class_size, remaining_bytes);
            break;  // Stop processing
        }
        
        // Process based on data type
        ESP_LOGI(TAG, "  Class %d data (%d elements, type %d):",
                 class_id + 1, header->class_counts[class_id], header->class_types[class_id]);
        
        
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
        // if (xSemaphoreTake(receiver_ctx.mutex, portMAX_DELAY) == pdTRUE) {
        //     ESP_LOGI(TAG, "Receiver Statistics:");
        //     ESP_LOGI(TAG, "  Packets received: %lu", receiver_ctx.packets_received);
        //     ESP_LOGI(TAG, "  Data packets: %lu", receiver_ctx.data_packets);
        //     ESP_LOGI(TAG, "  Error packets: %lu", receiver_ctx.error_packets);
            
        //     // Print current class configuration
        //     ESP_LOGI(TAG, "  Class Types: Class1=%d, Class2=%d, Class3=%d",
        //              receiver_ctx.class_types[0], receiver_ctx.class_types[1], receiver_ctx.class_types[2]);
        //     ESP_LOGI(TAG, "  Last Packet Counts: Class1=%d, Class2=%d, Class3=%d",
        //              receiver_ctx.class_counts[0], receiver_ctx.class_counts[1], receiver_ctx.class_counts[2]);
            
        //     xSemaphoreGive(receiver_ctx.mutex);
        // }
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

    // Initialize WiFi and create AP
    ESP_LOGI(TAG, "Starting WiFi in AP mode");
    wifi_init_softap();
    
    // After WiFi initialization, enable promiscuous mode for packet capture
    vTaskDelay(pdMS_TO_TICKS(1000));  // Short delay to ensure WiFi is fully initialized
    enable_promiscuous_mode();
    
    // Initialize packet receiver
    ESP_LOGI(TAG, "Initializing packet receiver");
    receiver_init();
    
    ESP_LOGI(TAG, "AP ready, waiting for packets");
}