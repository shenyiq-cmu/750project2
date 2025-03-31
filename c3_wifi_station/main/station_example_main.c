/**
 * ESP32 WiFi Station with Packet Scheduler and Sender
 * 
 * This implementation connects to a WiFi network first and then
 * sends data packets to the AP using the packet scheduler.
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

/* Include the terminal commands header */
#include "terminal_cmd.h"

/* WiFi configuration */
#define EXAMPLE_ESP_WIFI_SSID      CONFIG_ESP_WIFI_SSID
#define EXAMPLE_ESP_WIFI_PASS      CONFIG_ESP_WIFI_PASSWORD
#define EXAMPLE_ESP_MAXIMUM_RETRY  CONFIG_ESP_MAXIMUM_RETRY

/* Scheduler Configuration */
#define MAX_CLASSES              3     // 3 classes (Class 1: 3s, Class 2: 5s, Class 3: 6s)
#define MAX_PACKET_SIZE          1400  // Maximum packet data size
#define MAX_QUEUE_SIZE           50    // Maximum packets per queue
#define SCHEDULER_CHECK_INTERVAL_MS 50 // How often to check queues
#define DEADLINE_PROCESSING_THRESHOLD_MS 1000 // Process if deadline is within this threshold
#define MAX_TX_SIZE              1400  // Maximum data size for transmission buffer

/* FreeRTOS event group to signal when we are connected */
static EventGroupHandle_t s_wifi_event_group;

static uint32_t tx_packet_counter = 1;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static const char *TAG = "wifi-sta-sender";

static int s_retry_num = 0;

/* Internal queue packet structure */
typedef struct {
    class_id_t class_id;          // Class identifier (0, 1, 2)
    uint32_t deadline;            // Absolute deadline for this packet (in ms)
    data_type_t data_type;        // Type of data contained
    uint16_t data_count;          // Number of data elements
    uint16_t size;                // Actual data size in bytes (not include header)
    uint8_t data[MAX_PACKET_SIZE]; // Packet data
} queue_packet_t;

/* Node structure for the linked list queue */
typedef struct queue_node {
    queue_packet_t packet;
    struct queue_node *next;
} queue_node_t;

/* Queue structure */
typedef struct {
    queue_node_t *head;
    queue_node_t *tail;
    int count;
} packet_queue_t;

/* Updated data packet header (now includes all necessary information) */
typedef struct {
    uint8_t class_counts[MAX_CLASSES];      // Number of items for each class
    data_type_t class_types[MAX_CLASSES];   // Data type for each class
    uint16_t total_size;                    // Total size of all data in bytes
    uint32_t timestamp;                     // Transmission timestamp
} __attribute__((packed)) data_packet_header_t;

/* Scheduler context */
typedef struct {
    packet_queue_t packet_queues[MAX_CLASSES]; // Separate queue for each class
    SemaphoreHandle_t mutex;      // Mutex for operations
    TaskHandle_t scheduler_task;  // Scheduler task handle
    TaskHandle_t packet_creator_task; // Packet creator task handle
    
    // Class information
    data_type_t class_types[MAX_CLASSES];  // Data type for each class
    uint32_t class_periods[MAX_CLASSES];   // Period for each class (ms)
    uint32_t class_deadlines[MAX_CLASSES]; // Deadline for each class (ms)
    uint32_t processing_threshold;         // Deadline processing threshold (ms)
    
    // Statistics
    uint32_t packets_processed;   // Total packets processed
    uint32_t packets_transmitted; // Packets successfully transmitted
    uint32_t deadline_misses;     // Packets that missed deadlines
    uint32_t current_time_ms;     // Current time in milliseconds
} scheduler_context_t;


/* Global scheduler context */
static scheduler_context_t scheduler_ctx;

/* Function prototypes */
static void scheduler_task(void *pvParameters);
static void process_packets(void);
static esp_err_t send_data_packet(uint8_t *data, uint16_t size, uint8_t class_counts[MAX_CLASSES]);

/* Queue functions */
static void queue_init(packet_queue_t *queue) {
    queue->head = NULL;
    queue->tail = NULL;
    queue->count = 0;
}

/* Add packet to the end of queue */
static bool queue_enqueue(packet_queue_t *queue, queue_packet_t *packet) {
    if (queue->count >= MAX_QUEUE_SIZE) {
        return false;  // Queue is full
    }
    
    queue_node_t *new_node = (queue_node_t*)malloc(sizeof(queue_node_t));
    if (!new_node) {
        return false;  // Memory allocation failed
    }
    
    // Copy packet data
    memcpy(&new_node->packet, packet, sizeof(queue_packet_t));
    new_node->next = NULL;
    
    // Add to queue
    if (queue->count == 0) {
        // First packet
        queue->head = new_node;
        queue->tail = new_node;
    } else {
        // Add to end
        queue->tail->next = new_node;
        queue->tail = new_node;
    }
    
    queue->count++;
    return true;
}

/* Add packet to the front of queue */
static bool queue_enqueue_front(packet_queue_t *queue, queue_packet_t *packet) {
    if (queue->count >= MAX_QUEUE_SIZE) {
        return false;  // Queue is full
    }
    
    queue_node_t *new_node = (queue_node_t*)malloc(sizeof(queue_node_t));
    if (!new_node) {
        return false;  // Memory allocation failed
    }
    
    // Copy packet data
    memcpy(&new_node->packet, packet, sizeof(queue_packet_t));
    
    // Add to front of queue
    if (queue->count == 0) {
        // First packet
        new_node->next = NULL;
        queue->head = new_node;
        queue->tail = new_node;
    } else {
        // Add to front
        new_node->next = queue->head;
        queue->head = new_node;
    }
    
    queue->count++;
    return true;
}

/* Remove and return packet from the front of queue */
static bool queue_dequeue(packet_queue_t *queue, queue_packet_t *packet) {
    if (queue->count == 0) {
        return false;  // Queue is empty
    }
    
    queue_node_t *node = queue->head;
    
    // Copy packet data
    memcpy(packet, &node->packet, sizeof(queue_packet_t));
    
    // Update queue
    queue->head = node->next;
    queue->count--;
    
    if (queue->count == 0) {
        queue->tail = NULL;
    }
    
    // Free node
    free(node);
    return true;
}

/* Peek at the front packet without removing it */
static bool queue_peek(packet_queue_t *queue, queue_packet_t *packet) {
    if (queue->count == 0) {
        return false;  // Queue is empty
    }
    
    // Copy packet data
    memcpy(packet, &queue->head->packet, sizeof(queue_packet_t));
    return true;
}

/* Get the current time in milliseconds */
static uint32_t get_current_time_ms(void)
{
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}


/* Find earliest deadline among all packets in all queues */
static uint32_t find_earliest_deadline(void)
{
    uint32_t earliest_deadline = UINT32_MAX;
    
    if (xSemaphoreTake(scheduler_ctx.mutex, portMAX_DELAY) != pdTRUE) {
        return earliest_deadline;
    }
    
    // Check each class queue
    for (int class_id = 0; class_id < MAX_CLASSES; class_id++) {
        packet_queue_t *queue = &scheduler_ctx.packet_queues[class_id];
        
        // If queue is not empty, check the first packet's deadline
        if (queue->head != NULL) {
            uint32_t deadline = queue->head->packet.deadline;
            if (deadline < earliest_deadline) {
                earliest_deadline = deadline;
            }
        }
    }
    
    xSemaphoreGive(scheduler_ctx.mutex);
    return earliest_deadline;
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
                ESP_LOGI(TAG, "!!!Connected to AP successfully!!!!");
                break;
                
            case WIFI_EVENT_STA_DISCONNECTED:
                if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
                    esp_wifi_connect();
                    s_retry_num++;
                    ESP_LOGI(TAG, "Retry %d to connect to the AP", s_retry_num);
                } else {
                    xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
                    ESP_LOGI(TAG, "Failed to connect to AP after maximum retries");
                }
                break;
                
            default:
                ESP_LOGI(TAG, "Other WiFi event: %ld", event_id);
                break;
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP address: " IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

/* Initialize WiFi in station mode and connect to AP */
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
    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    // Configure WiFi station with hardcoded SSID and password to ensure match
    // These MUST match exactly with the AP configuration
    const char* wifi_ssid = "myssid1";  // MUST MATCH AP SSID
    const char* wifi_password = "mypassword1";  // MUST MATCH AP PASSWORD
    
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
    
    // Configure WiFi and start
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi station initialization completed, waiting for connection");

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
    } else {
        ESP_LOGE(TAG, "Unexpected event during connection");
    }
}


/* Set data type for a class */
esp_err_t scheduler_set_class_type(class_id_t class_id, data_type_t data_type)
{
    if (class_id >= MAX_CLASSES) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (xSemaphoreTake(scheduler_ctx.mutex, portMAX_DELAY) == pdTRUE) {
        scheduler_ctx.class_types[class_id] = data_type;
        xSemaphoreGive(scheduler_ctx.mutex);
        
        ESP_LOGI(TAG, "Set class %d data type to %d", class_id, data_type);
        return ESP_OK;
    }
    
    return ESP_FAIL;
}

/* Submit a packet to the scheduler */
esp_err_t scheduler_submit_packet(class_id_t class_id, const void *data, uint16_t count)
{
    // Validate class
    if (class_id >= MAX_CLASSES) {
        ESP_LOGE(TAG, "Invalid class: %d", class_id);
        return ESP_ERR_INVALID_ARG;
    }
    
    // Get data type for this class
    data_type_t data_type = scheduler_ctx.class_types[class_id];
    
    // Calculate size based on data type and count
    uint16_t element_size = 0;
    switch (data_type) {
        case DATA_TYPE_INT8:   element_size = 1; break;
        case DATA_TYPE_INT16:  element_size = 2; break;
        case DATA_TYPE_INT32:  element_size = 4; break;
        case DATA_TYPE_FLOAT:  element_size = 4; break;
        case DATA_TYPE_DOUBLE: element_size = 8; break;
    }
    
    uint16_t total_size = element_size * count;
    if (total_size > MAX_PACKET_SIZE) {
        ESP_LOGE(TAG, "Data too large: %d bytes (max: %d)", total_size, MAX_PACKET_SIZE);
        return ESP_ERR_INVALID_ARG;
    }
    
    // Create queue packet
    queue_packet_t packet = {0};
    
    // Set class info
    packet.class_id = class_id;
    packet.data_type = data_type;
    packet.data_count = count;
    packet.size = total_size;
    
    // Set deadline based on class - use configurable deadlines now
    uint32_t current_time = get_current_time_ms();
    packet.deadline = current_time + scheduler_ctx.class_deadlines[class_id];
    
    // Copy data
    if (data != NULL && total_size > 0) {
        memcpy(packet.data, data, total_size);
    }
    
    // Submit packet to the appropriate queue with mutex protection
    if (xSemaphoreTake(scheduler_ctx.mutex, portMAX_DELAY) == pdTRUE) {
        bool success = queue_enqueue(&scheduler_ctx.packet_queues[class_id], &packet);
        xSemaphoreGive(scheduler_ctx.mutex);
        
        if (!success) {
            ESP_LOGE(TAG, "Failed to queue packet: Queue %d full", class_id);
            return ESP_FAIL;
        }
    } else {
        return ESP_FAIL;
    }
    
    return ESP_OK;
}

/* Process and transmit packets with fixed class order */
static void process_packets(void)
{
    uint32_t current_time = get_current_time_ms();
    scheduler_ctx.current_time_ms = current_time;
    
    // Find earliest deadline first
    uint32_t earliest_deadline = find_earliest_deadline();
    
    // If no packets or deadline is not approaching, return without processing
    if (earliest_deadline == UINT32_MAX) {
        return; // No packets in any queue
    }
    
    // Check if we need to process now based on deadline threshold
    // Use the configurable threshold from scheduler context instead of the fixed macro
    if (earliest_deadline > current_time + scheduler_ctx.processing_threshold) {
        ESP_LOGD(TAG, "Earliest deadline not approaching yet: %lu, current time: %lu", 
                 earliest_deadline, current_time);
        return; // No urgency to process
    }
    
    ESP_LOGI(TAG, "Processing packets - earliest deadline approaching: %lu, current time: %lu", 
             earliest_deadline, current_time);
    
    // Allocate data buffer
    uint8_t *data_buffer = malloc(MAX_TX_SIZE);
    if (data_buffer == NULL) {
        ESP_LOGE(TAG, "Failed to allocate data buffer");
        return;
    }
    
    // Clear buffer
    memset(data_buffer, 0, MAX_TX_SIZE);
    uint8_t *data_ptr = data_buffer;
    uint16_t remaining_space = MAX_TX_SIZE;
    uint8_t class_counts[MAX_CLASSES] = {0};
    
    // Process packets in FIXED CLASS ORDER: Class 1 → Class 2 → Class 3
    // No matter what the deadlines are, we always maintain this order in the buffer
    for (int class_id = 0; class_id < MAX_CLASSES; class_id++) {
        queue_packet_t packet;
        bool packet_available = false;
        
        // Take mutex for queue operations
        if (xSemaphoreTake(scheduler_ctx.mutex, portMAX_DELAY) == pdTRUE) {
            // Process all available packets from this class that fit in buffer
            while (queue_peek(&scheduler_ctx.packet_queues[class_id], &packet)) {
                // Check if packet fits in remaining space
                if (packet.size > remaining_space) {
                    // Stop processing this class if packet doesn't fit
                    break;
                }
                
                // Dequeue the packet
                queue_dequeue(&scheduler_ctx.packet_queues[class_id], &packet);
                packet_available = true;
                
                // Skip packets that missed deadline
                if (current_time > packet.deadline) {
                    ESP_LOGW(TAG, "Class %d packet missed deadline: Deadline=%lu, Current=%lu",
                            class_id + 1, packet.deadline, current_time);
                    
                    scheduler_ctx.deadline_misses++;
                    scheduler_ctx.packets_processed++;
                    
                    // Skip this packet (don't include in buffer)
                    packet_available = false;
                    continue;
                }
                
                // We have a valid packet that fits, copy it to buffer
                if (packet_available) {
                    memcpy(data_ptr, packet.data, packet.size);
                    data_ptr += packet.size;
                    remaining_space -= packet.size;
                    
                    // Update class count (how many data items per class)
                    class_counts[class_id] += packet.data_count;
                    
                    scheduler_ctx.packets_processed++;
                    
                    ESP_LOGI(TAG, "Added Class %d packet to transmission: Size=%d, Deadline=%lu",
                            class_id + 1, packet.size, packet.deadline);
                    
                    // If buffer is nearly full, stop adding more packets
                    if (remaining_space < 100) {
                        break;
                    }
                }
            }
            
            xSemaphoreGive(scheduler_ctx.mutex);
        }
    }
    
    // Calculate actual data size
    uint16_t actual_data_size = MAX_TX_SIZE - remaining_space;
    
    // Count how many different classes were included
    ESP_LOGI(TAG, "==========Sending buffer #%lu...================", tx_packet_counter);
    ESP_LOGI(TAG, "  Total data size: %d bytes (%.1f%% of buffer capacity)",
            actual_data_size, (actual_data_size * 100.0) / MAX_TX_SIZE);

    
    // Send data if we have any
    if (actual_data_size > 0) {
        esp_err_t ret = send_data_packet(data_buffer, actual_data_size, class_counts);
        
        if (ret == ESP_OK) {
            if (xSemaphoreTake(scheduler_ctx.mutex, portMAX_DELAY) == pdTRUE) {
                // Count how many class types were transmitted
                scheduler_ctx.packets_transmitted += 
                    (class_counts[0] > 0 ? 1 : 0) + 
                    (class_counts[1] > 0 ? 1 : 0) + 
                    (class_counts[2] > 0 ? 1 : 0);
                xSemaphoreGive(scheduler_ctx.mutex);
            }
        }
    } else {
        ESP_LOGW(TAG, "No data to transmit after processing");
    }
    
    // Free the data buffer
    free(data_buffer);
}

/* Send the data packet with all class data and type information */
static esp_err_t send_data_packet(uint8_t *data, uint16_t size, uint8_t class_counts[MAX_CLASSES])
{
    // Increment the transmission counter
    tx_packet_counter++;

    // Verify size is valid
    if (size > MAX_TX_SIZE) {
        ESP_LOGE(TAG, "Data size %d exceeds maximum allowed %d", size, MAX_TX_SIZE);
        size = MAX_TX_SIZE;  // Truncate to maximum
    }
    
    // Create proper 802.11 data frame header
    const size_t HEADER_SIZE = 24;  // Basic 802.11 header size
    
    // Create data packet header
    data_packet_header_t header = {0};
    header.total_size = size;
    header.timestamp = get_current_time_ms();
    
    // Copy class counts and types
    for (int i = 0; i < MAX_CLASSES; i++) {
        header.class_counts[i] = class_counts[i];
        
        // Get class type from scheduler context
        if (xSemaphoreTake(scheduler_ctx.mutex, portMAX_DELAY) == pdTRUE) {
            header.class_types[i] = scheduler_ctx.class_types[i];
            xSemaphoreGive(scheduler_ctx.mutex);
        }
    }
    
    // Calculate total buffer size needed
    size_t packet_size = HEADER_SIZE + sizeof(data_packet_header_t) + size;
    
    // Allocate buffer for 802.11 header + our header + data
    uint8_t *packet_buffer = malloc(packet_size);
    if (packet_buffer == NULL) {
        ESP_LOGE(TAG, "Failed to allocate packet buffer");
        return ESP_FAIL;
    }
    
    // Clear buffer to ensure all padding bytes are zero
    memset(packet_buffer, 0, packet_size);
    
    // Setup 802.11 header
    // Frame Control: Data frame (0x08) with FromDS=0, ToDS=1 (Station to AP)
    packet_buffer[0] = 0x08;  // Data frame type
    packet_buffer[1] = 0x01;  // FromDS=0, ToDS=1
    
    // Set destination address (AP MAC)
    wifi_ap_record_t ap_info;
    if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
        memcpy(&packet_buffer[4], ap_info.bssid, 6);
    } else {
        // Fallback to broadcast
        memset(&packet_buffer[4], 0xFF, 6);
    }
    
    // Set source address - our own MAC address
    uint8_t mac[6];
    esp_wifi_get_mac(WIFI_IF_STA, mac);
    memcpy(&packet_buffer[10], mac, 6);
    
    // Set BSSID in the header as well
    if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
        memcpy(&packet_buffer[16], ap_info.bssid, 6);
    } else {
        // Fallback
        memset(&packet_buffer[16], 0xFF, 6);
    }
    
    // Debug: Log header size and data sizes
    ESP_LOGD(TAG, "Header size: %d, Data size: %d, Total packet size: %d", 
             sizeof(data_packet_header_t), size, packet_size);
    
    // Copy our header after the 802.11 header
    memcpy(packet_buffer + HEADER_SIZE, &header, sizeof(data_packet_header_t));
    
    // Copy data after our header
    if (data != NULL && size > 0) {
        memcpy(packet_buffer + HEADER_SIZE + sizeof(data_packet_header_t), data, size);
    }
    
    // Send packet
    //ESP_LOGI(TAG, "Sending buffer #%lu...", tx_packet_counter);
    esp_err_t ret = esp_wifi_80211_tx(WIFI_IF_STA, 
                                     packet_buffer, 
                                     packet_size, 
                                     true);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send data packet: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "  Sent data packet: Class1=%ditem(type%d), Class2=%ditem(type%d), Class3=%ditem(type%d), Size=%d bytes",
                header.class_counts[0], header.class_types[0],
                header.class_counts[1], header.class_types[1],
                header.class_counts[2], header.class_types[2],
                size);
        ESP_LOGI(TAG,"================================================");
    }
    
    // Free buffer
    free(packet_buffer);
    
    return ret;
}

/* Main scheduler task */
static void scheduler_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Scheduler task started");
    
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t check_interval = pdMS_TO_TICKS(SCHEDULER_CHECK_INTERVAL_MS);
    
    // After starting the task, wait for WiFi to fully initialize
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    while (1) {
        // Wait for the next check interval
        vTaskDelayUntil(&last_wake_time, check_interval);
        
        // Process packets if any deadlines are approaching
        process_packets();
    }
}

/* Create a test int32 packet */
void create_test_int32_packet(class_id_t class_id, uint16_t count)
{
    // Create array of int32 values
    int32_t *values = malloc(count * sizeof(int32_t));
    if (values == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for test data");
        return;
    }
    
    // Fill with sequential values
    for (int i = 0; i < count; i++) {
        values[i] = i;
    }
    
    // Make sure data type matches what we're sending
    //scheduler_set_class_type(class_id, DATA_TYPE_INT32);
    
    // Submit packet with this data
    scheduler_submit_packet(class_id, values, count);
    
    // Free the temporary buffer
    free(values);
}

/* Create a test float packet */
void create_test_float_packet(class_id_t class_id, uint16_t count)
{
    // Create array of float values
    float *values = malloc(count * sizeof(float));
    if (values == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for test data");
        return;
    }
    
    // Fill with values
    for (int i = 0; i < count; i++) {
        values[i] = i * 0.1f;
    }
    
    // Make sure data type matches what we're sending
    //scheduler_set_class_type(class_id, DATA_TYPE_FLOAT);
    
    // Submit packet with this data
    scheduler_submit_packet(class_id, values, count);
    
    // Free the temporary buffer
    free(values);
}

/* Create a test int16 packet */
void create_test_int16_packet(class_id_t class_id, uint16_t count)
{
    // Create array of int16 values
    int16_t *values = malloc(count * sizeof(int16_t));
    if (values == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for test data");
        return;
    }
    
    // Fill with sequential values
    for (int i = 0; i < count; i++) {
        values[i] = i * 10;
    }
    
    // Make sure data type matches what we're sending
    //scheduler_set_class_type(class_id, DATA_TYPE_INT16);
    
    // Submit packet with this data
    scheduler_submit_packet(class_id, values, count);
    
    // Free the temporary buffer
    free(values);
}

/* Print scheduler statistics */
void print_scheduler_stats(void)
{
    if (xSemaphoreTake(scheduler_ctx.mutex, portMAX_DELAY) != pdTRUE) {
        return;
    }
    
    // ESP_LOGI(TAG, "->Scheduler Statistics:");
    // ESP_LOGI(TAG, "  Packets processed: %lu", scheduler_ctx.packets_processed);
    // ESP_LOGI(TAG, "  Packets transmitted: %lu", scheduler_ctx.packets_transmitted);
    // ESP_LOGI(TAG, "  Deadline misses: %lu", scheduler_ctx.deadline_misses);
    
    // Queue status
    int queue_length[MAX_CLASSES];
    for (int i = 0; i < MAX_CLASSES; i++) {
        queue_length[i] = scheduler_ctx.packet_queues[i].count;
    }
    
    ESP_LOGI(TAG, "  Queue status: Class1=%d, Class2=%d, Class3=%d", 
            queue_length[0], queue_length[1], queue_length[2]);
    
    xSemaphoreGive(scheduler_ctx.mutex);
}

static void packet_creator_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Packet creator task started");
    
    // Track the last time a packet was created for each class
    TickType_t last_class_time[MAX_CLASSES];
    for (int i = 0; i < MAX_CLASSES; i++) {
        last_class_time[i] = xTaskGetTickCount();
    }
    
    // Define check interval (sleep time between checks)
    const TickType_t check_interval = pdMS_TO_TICKS(100);  // Check every 100ms
    
    // Data size counters to create varying sized packets
    uint16_t class_counts[MAX_CLASSES] = {5, 4, 6};
    
    while (1) {
        TickType_t current_time = xTaskGetTickCount();
        
        // Check if we need to create packets for any class based on their periods
        for (int class_id = 0; class_id < MAX_CLASSES; class_id++) {
            // Get current period for this class (with mutex protection)
            uint32_t period_ms = DEFAULT_CLASS1_PERIOD;  // Default value
            
            if (xSemaphoreTake(scheduler_ctx.mutex, portMAX_DELAY) == pdTRUE) {
                period_ms = scheduler_ctx.class_periods[class_id];
                xSemaphoreGive(scheduler_ctx.mutex);
            }
            
            TickType_t period_ticks = pdMS_TO_TICKS(period_ms);
            
            // Check if it's time to create a packet for this class
            if ((current_time - last_class_time[class_id]) >= period_ticks) {
                switch (class_id) {
                    case CLASS_1:
                        create_test_int32_packet(CLASS_1, class_counts[CLASS_1]);
                        class_counts[CLASS_1] = 5 + (class_counts[CLASS_1] % 5);
                        break;
                    case CLASS_2:
                        create_test_float_packet(CLASS_2, class_counts[CLASS_2]);
                        class_counts[CLASS_2] = 4 + (class_counts[CLASS_2] % 4);
                        break;
                    case CLASS_3:
                        create_test_int16_packet(CLASS_3, class_counts[CLASS_3]);
                        class_counts[CLASS_3] = 6 + (class_counts[CLASS_3] % 3);
                        break;
                }
                
                last_class_time[class_id] = current_time;
            }
        }
        
        // Print statistics every second
        static TickType_t last_stats_time = 0;
        if ((current_time - last_stats_time) >= pdMS_TO_TICKS(1000)) {
            print_scheduler_stats();
            last_stats_time = current_time;
        }
        
        // Sleep for a short interval before checking again
        vTaskDelay(check_interval);
    }
}


/* Initialize the packet scheduler */
/* Initialize the packet scheduler */
void scheduler_init(scheduler_config_t *config)
{
    // Initialize packet queues for each class
    for (int i = 0; i < MAX_CLASSES; i++) {
        queue_init(&scheduler_ctx.packet_queues[i]);
    }

    // Initialize mutex
    scheduler_ctx.mutex = xSemaphoreCreateMutex();
    if (scheduler_ctx.mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return;
    }

    // Set class types, periods, and deadlines from the configuration
    for (int i = 0; i < MAX_CLASSES; i++) {
        scheduler_ctx.class_types[i] = config->class_types[i];
        scheduler_ctx.class_periods[i] = config->class_periods[i];
        scheduler_ctx.class_deadlines[i] = config->class_deadlines[i];
    }
    
    // Set processing threshold from configuration - ONLY ONCE
    scheduler_ctx.processing_threshold = config->processing_threshold;

    // Initialize statistics
    scheduler_ctx.packets_processed = 0;
    scheduler_ctx.packets_transmitted = 0;
    scheduler_ctx.deadline_misses = 0;
    scheduler_ctx.current_time_ms = 0;
    
    // Create packet creator task
    BaseType_t ret = xTaskCreate(
        packet_creator_task,             // Function that implements the task
        "packet_creator_task",           // Text name for the task
        16384,                           // Stack size in words
        NULL,                            // Parameter passed into the task
        4,                               // Priority (lower than scheduler) 
        &scheduler_ctx.packet_creator_task // Used to pass out the task handle
    );
    
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create packet creator task");
        return;
    }
    
    // Create scheduler task
    ret = xTaskCreate(
        scheduler_task,                  // Function that implements the task
        "scheduler_task",                // Text name for the task
        16384,                           // Stack size in words
        NULL,                            // Parameter passed into the task
        5,                               // Priority (higher than creator)
        &scheduler_ctx.scheduler_task    // Used to pass out the task handle
    );
    
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create scheduler task");
        return;
    }
    
    // Log initialization details
    ESP_LOGI(TAG, "Packet scheduler initialized with the following configuration:");
    for (int i = 0; i < MAX_CLASSES; i++) {
        const char *type_str;
        switch (scheduler_ctx.class_types[i]) {
            case DATA_TYPE_INT8:   type_str = "INT8";   break;
            case DATA_TYPE_INT16:  type_str = "INT16";  break;
            case DATA_TYPE_INT32:  type_str = "INT32";  break;
            case DATA_TYPE_FLOAT:  type_str = "FLOAT";  break;
            case DATA_TYPE_DOUBLE: type_str = "DOUBLE"; break;
            default:               type_str = "UNKNOWN"; break;
        }
        
        ESP_LOGI(TAG, "Class %d: Type=%s, Period=%lu ms, Deadline=%lu ms", 
                i + 1, type_str, scheduler_ctx.class_periods[i], scheduler_ctx.class_deadlines[i]);
    }
    
    // Also log the processing threshold
    ESP_LOGI(TAG, "Processing threshold: %lu ms", scheduler_ctx.processing_threshold);
}

/* Modified main application entry point */
void app_main(void)
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize WiFi and connect to AP
    ESP_LOGI(TAG, "Starting WiFi in station mode");
    wifi_init_sta();
    
    // Short delay to allow WiFi to initialize fully
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    // Initialize configuration structure
    scheduler_config_t config = {0};
    
    // Initialize the terminal and get configuration from user
    ESP_LOGI(TAG, "Waiting for user configuration via terminal...");
    terminal_init_and_configure(&config);
    
    // Once user has completed configuration via terminal, initialize packet scheduler
    ESP_LOGI(TAG, "User configuration complete, initializing scheduler...");
    scheduler_init(&config);
    
    // Notify user that the system is now running
    printf("\n==================================================\n");
    printf("    ESP32 WiFi Packet Scheduler Now Running    \n");
    printf("==================================================\n");
    printf("System is running with configured parameters.\n");
    
    // Main thread has nothing more to do - all work is done in the tasks
    ESP_LOGI(TAG, "Main task complete, system running with configured parameters");
}