/**
 * ESP32 WiFi Access Point with Packet Scheduler
 * Using standard C queue implementation instead of FreeRTOS queues
 */

#include <string.h>
#include <portmacro.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_mac.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sys.h"

/* WiFi configuration */
#define EXAMPLE_ESP_WIFI_SSID      CONFIG_ESP_WIFI_SSID
#define EXAMPLE_ESP_WIFI_PASS      CONFIG_ESP_WIFI_PASSWORD
#define EXAMPLE_ESP_WIFI_CHANNEL   CONFIG_ESP_WIFI_CHANNEL
#define EXAMPLE_MAX_STA_CONN       CONFIG_ESP_MAX_STA_CONN

/* Scheduler Configuration */
#define MAX_CLASSES              3     // 3 classes (Class 1: 3s, Class 2: 5s, Class 3: 6s)
#define MAX_PACKET_SIZE          1400  // Maximum packet data size
#define MAX_QUEUE_SIZE           50    // Maximum packets per queue
#define SCHEDULER_CHECK_INTERVAL_MS 50 // How often to check queues
#define MAX_TX_SIZE              1400  // Maximum data size for transmission buffer

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

static const char *TAG = "wifi-ap-scheduler";

/* Internal queue packet structure */
typedef struct {
    class_id_t class_id;          // Class identifier (0, 1, 2)
    uint32_t deadline;            // Absolute deadline for this packet (in ms)
    data_type_t data_type;        // Type of data contained
    uint16_t data_count;          
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

/* Control packet structure - sent before data transmission */
typedef struct {
    uint8_t class_counts[MAX_CLASSES]; // Number of elements for each class
    data_type_t class_types[MAX_CLASSES]; // Data type for each class
} __attribute__((packed)) control_packet_t;

/* Data packet (for transmission not in queue) header structure */
typedef struct {
    uint8_t class_counts[MAX_CLASSES]; // Number of items for each class
    uint16_t total_size;          // Total size of all data in bytes
    uint32_t timestamp;           // Transmission timestamp
} __attribute__((packed)) data_packet_header_t;

/* Scheduler context */
typedef struct {
    packet_queue_t packet_queues[MAX_CLASSES]; // Separate queue for each class
    SemaphoreHandle_t mutex;      // Mutex for operations
    TaskHandle_t scheduler_task;  // Scheduler task handle
    
    // Class information
    data_type_t class_types[MAX_CLASSES]; // Data type for each class
    
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
static esp_err_t send_control_packet(void);
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

/* Return number of packets in queue */

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

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .ssid_len = strlen(EXAMPLE_ESP_WIFI_SSID),
            .channel = EXAMPLE_ESP_WIFI_CHANNEL,
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
    if (strlen(EXAMPLE_ESP_WIFI_PASS) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_softap finished. SSID:%s password:%s channel:%d",
             EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS, EXAMPLE_ESP_WIFI_CHANNEL);
}

/* Initialize the packet scheduler */
void scheduler_init(void)
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

    // Set default data types
    scheduler_ctx.class_types[CLASS_1] = DATA_TYPE_INT32;  // Class 1 (3s) - INT32
    scheduler_ctx.class_types[CLASS_2] = DATA_TYPE_FLOAT;  // Class 2 (5s) - FLOAT
    scheduler_ctx.class_types[CLASS_3] = DATA_TYPE_INT16;  // Class 3 (6s) - INT16

    // Initialize statistics
    scheduler_ctx.packets_processed = 0;
    scheduler_ctx.packets_transmitted = 0;
    scheduler_ctx.deadline_misses = 0;
    scheduler_ctx.current_time_ms = 0;
    
    // Create scheduler task
    BaseType_t ret = xTaskCreate(
        scheduler_task,              // Function that implements the task
        "scheduler_task",            // Text name for the task
        16384,                        // Stack size in words
        NULL,                        // Parameter passed into the task
        5,                           // Priority 
        &scheduler_ctx.scheduler_task // Used to pass out the task handle
    );
    
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create scheduler task");
        return;
    }
    
    ESP_LOGI(TAG, "Packet scheduler initialized with %d classes", MAX_CLASSES);
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

/* Get the current time in milliseconds */
static uint32_t get_current_time_ms(void)
{
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
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
    
    // Set deadline based on class
    uint32_t current_time = get_current_time_ms();
    switch (class_id) {
        case CLASS_1:
            packet.deadline = current_time + 3000;  // 3 seconds
            break;
        case CLASS_2:
            packet.deadline = current_time + 5000;  // 5 seconds
            break;
        case CLASS_3:
            packet.deadline = current_time + 6000;  // 6 seconds
            break;
    }
    
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
    
    ESP_LOGI(TAG, "Queued Class %d packet: Type=%d, Count=%d, Size=%d, Deadline=%lu",
             class_id + 1, data_type, count, total_size, packet.deadline);
    
    return ESP_OK;
}

/* Process and transmit packets from all classes */
static void process_packets(void)
{
    queue_packet_t candidate_packets[MAX_CLASSES];
    bool has_candidate[MAX_CLASSES] = {false};
    int total_packets = 0;
    uint32_t current_time = get_current_time_ms();
    
    // Update the current time
    scheduler_ctx.current_time_ms = current_time;
    
    // Take mutex for queue operations
    if (xSemaphoreTake(scheduler_ctx.mutex, portMAX_DELAY) != pdTRUE) {
        return;
    }
    
    // First, peek at the front packet from each queue to find earliest deadline
    for (int i = 0; i < MAX_CLASSES; i++) {
        if (queue_peek(&scheduler_ctx.packet_queues[i], &candidate_packets[i])) {
            has_candidate[i] = true;
            total_packets++;
        }
    }
    
    if (total_packets == 0) {
        xSemaphoreGive(scheduler_ctx.mutex);
        return;  // No packets to process
    }
    
    // Find which queue has the packet with earliest deadline
    int earliest_class = -1;
    uint32_t earliest_deadline = UINT32_MAX;
    
    for (int i = 0; i < MAX_CLASSES; i++) {
        if (has_candidate[i] && candidate_packets[i].deadline < earliest_deadline) {
            earliest_deadline = candidate_packets[i].deadline;
            earliest_class = i;
        }
    }
    
    if (earliest_class < 0) {
        xSemaphoreGive(scheduler_ctx.mutex);
        return;  // No valid candidate found
    }
    
    // Release mutex before further processing
    xSemaphoreGive(scheduler_ctx.mutex);
    
    // Always transmit if there are packets, regardless of deadline
    ESP_LOGI(TAG, "Processing packets: %d packets in queue", total_packets);
    
    // First send the control packet
    esp_err_t ret = send_control_packet();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send control packet");
        return;
    }
    
    // Now prepare the data packet
    uint8_t class_counts[MAX_CLASSES] = {0};
    
    // Calculate total size needed for the data buffer
    uint16_t total_data_size = 0;
    
    // Take mutex again for size calculation
    if (xSemaphoreTake(scheduler_ctx.mutex, portMAX_DELAY) != pdTRUE) {
        return;
    }
    
    for (int i = 0; i < MAX_CLASSES; i++) {
        queue_packet_t packet;
        if (queue_peek(&scheduler_ctx.packet_queues[i], &packet)) {
            total_data_size += packet.size;
        }
    }
    
    xSemaphoreGive(scheduler_ctx.mutex);
    
    // Handle case where total size exceeds max packet size
    bool size_exceeded = (total_data_size > MAX_TX_SIZE);
    
    if (size_exceeded) {
        ESP_LOGW(TAG, "Total data size %d exceeds maximum packet size %d, will send partial data",
                total_data_size, MAX_TX_SIZE);
    }
    
    // Allocate data buffer with appropriate size
    uint16_t buffer_size = size_exceeded ? MAX_TX_SIZE : total_data_size;
    uint8_t *data_buffer = malloc(buffer_size);
    if (data_buffer == NULL) {
        ESP_LOGE(TAG, "Failed to allocate data buffer");
        return;
    }
    
    // Position pointer for data population
    uint8_t *data_ptr = data_buffer;
    uint16_t remaining_space = buffer_size;
    
    // Dequeue and process packets from each class
    for (int class_id = 0; class_id < MAX_CLASSES; class_id++) {
        queue_packet_t packet;
        bool packet_available = false;
        
        // Take mutex for queue operations
        if (xSemaphoreTake(scheduler_ctx.mutex, portMAX_DELAY) == pdTRUE) {
            packet_available = queue_dequeue(&scheduler_ctx.packet_queues[class_id], &packet);
            xSemaphoreGive(scheduler_ctx.mutex);
        }
        
        if (packet_available) {
            // Check if packet missed deadline
            if (current_time > packet.deadline) {
                ESP_LOGW(TAG, "Class %d packet missed deadline: Deadline=%lu, Current=%lu",
                         class_id + 1, packet.deadline, current_time);
                
                if (xSemaphoreTake(scheduler_ctx.mutex, portMAX_DELAY) == pdTRUE) {
                    scheduler_ctx.deadline_misses++;
                    scheduler_ctx.packets_processed++;
                    xSemaphoreGive(scheduler_ctx.mutex);
                }
                
                continue;  // Skip this packet
            }
            
            // Check if we have enough space for this packet
            if (packet.size > remaining_space) {
                // Put packet back in queue if buffer's remaining size can't fit it
                if (xSemaphoreTake(scheduler_ctx.mutex, portMAX_DELAY) == pdTRUE) {
                    queue_enqueue_front(&scheduler_ctx.packet_queues[class_id], &packet);
                    xSemaphoreGive(scheduler_ctx.mutex);
                }
                
                ESP_LOGW(TAG, "Class %d packet size %d exceeds remaining space %d, will send in next batch",
                        class_id + 1, packet.size, remaining_space);
                continue;
            }
            
            // Copy packet data to buffer
            memcpy(data_ptr, packet.data, packet.size);
            data_ptr += packet.size;
            remaining_space -= packet.size;
            
            // Update class count
            class_counts[class_id] = packet.data_count;
            
            if (xSemaphoreTake(scheduler_ctx.mutex, portMAX_DELAY) == pdTRUE) {
                scheduler_ctx.packets_processed++;
                xSemaphoreGive(scheduler_ctx.mutex);
            }
        }
    }
    
    // Calculate actual data size
    uint16_t actual_data_size = buffer_size - remaining_space;
    
    // Only send if we have data to send
    if (actual_data_size > 0) {
        // Send the data packet
        ret = send_data_packet(data_buffer, actual_data_size, class_counts);
        
        if (ret == ESP_OK) {
            if (xSemaphoreTake(scheduler_ctx.mutex, portMAX_DELAY) == pdTRUE) {
                scheduler_ctx.packets_transmitted += 
                    (class_counts[0] > 0 ? 1 : 0) + 
                    (class_counts[1] > 0 ? 1 : 0) + 
                    (class_counts[2] > 0 ? 1 : 0);
                xSemaphoreGive(scheduler_ctx.mutex);
            }
        }
    }
    
    // Free the data buffer
    free(data_buffer);
}

/* Send the control packet with class type information */
static esp_err_t send_control_packet(void)
{
    // Create proper 802.11 data frame header
    // Frame Control Field: Data frame (0x08)
    // Duration: 0
    // Address fields: Set to broadcast
    const size_t HEADER_SIZE = 24;  // Basic 802.11 header size
    
    control_packet_t control_packet = {0};
    
    // Get class information
    if (xSemaphoreTake(scheduler_ctx.mutex, portMAX_DELAY) == pdTRUE) {
        // Set data types
        for (int i = 0; i < MAX_CLASSES; i++) {
            control_packet.class_types[i] = scheduler_ctx.class_types[i];
        }
        
        // Get counts for each class
        for (int i = 0; i < MAX_CLASSES; i++) {
            queue_packet_t packet;
            if (queue_peek(&scheduler_ctx.packet_queues[i], &packet)) {
                control_packet.class_counts[i] = packet.data_count;
            } else {
                control_packet.class_counts[i] = 0;
            }
        }
        
        xSemaphoreGive(scheduler_ctx.mutex);
    }
    
    // Calculate control packet size
    size_t control_size = sizeof(control_packet_t);
    
    // Create properly sized buffer with header
    size_t buffer_size = HEADER_SIZE + control_size;
    uint8_t *buffer = malloc(buffer_size);
    if (!buffer) {
        ESP_LOGE(TAG, "Failed to allocate buffer for control packet");
        return ESP_ERR_NO_MEM;
    }
    
    // Clear buffer
    memset(buffer, 0, buffer_size);
    
    // Setup 802.11 header
    // Frame Control: Data frame (0x08) with FromDS=1, ToDS=0
    buffer[0] = 0x08;  // Data frame type
    buffer[1] = 0x02;  // FromDS=1, ToDS=0
    
    // Set broadcast address (FF:FF:FF:FF:FF:FF) for destination
    memset(&buffer[4], 0xFF, 6);
    
    // Set source address - our own MAC address
    uint8_t mac[6];
    esp_wifi_get_mac(WIFI_IF_AP, mac);
    memcpy(&buffer[10], mac, 6);
    
    // Copy broadcast address for BSSID as well
    memset(&buffer[16], 0xFF, 6);
    
    // Copy control packet payload after the header
    memcpy(buffer + HEADER_SIZE, &control_packet, control_size);
    
    // Send control packet using ESP32's Wi-Fi functionality
    esp_err_t ret = esp_wifi_80211_tx(WIFI_IF_AP, buffer, buffer_size, true);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send control packet: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Sent control packet: Class1=%d(%d), Class2=%d(%d), Class3=%d(%d)",
                control_packet.class_counts[0], control_packet.class_types[0],
                control_packet.class_counts[1], control_packet.class_types[1],
                control_packet.class_counts[2], control_packet.class_types[2]);
    }
    
    // Free the buffer
    free(buffer);
    
    return ret;
}

/* Send the data packet with all class data */
static esp_err_t send_data_packet(uint8_t *data, uint16_t size, uint8_t class_counts[MAX_CLASSES])
{
    // Create proper 802.11 data frame header
    const size_t HEADER_SIZE = 24;  // Basic 802.11 header size
    
    // Create data packet header
    data_packet_header_t header = {0};
    header.total_size = size;
    header.timestamp = get_current_time_ms();
    
    // Copy class counts
    for (int i = 0; i < MAX_CLASSES; i++) {
        header.class_counts[i] = class_counts[i];
    }
    
    // Allocate buffer for 802.11 header + our header + data
    uint8_t *packet_buffer = malloc(HEADER_SIZE + sizeof(data_packet_header_t) + size);
    if (packet_buffer == NULL) {
        ESP_LOGE(TAG, "Failed to allocate packet buffer");
        return ESP_FAIL;
    }
    
    // Clear buffer
    memset(packet_buffer, 0, HEADER_SIZE + sizeof(data_packet_header_t) + size);
    
    // Setup 802.11 header
    // Frame Control: Data frame (0x08) with FromDS=1, ToDS=0
    packet_buffer[0] = 0x08;  // Data frame type
    packet_buffer[1] = 0x02;  // FromDS=1, ToDS=0
    
    // Set broadcast address (FF:FF:FF:FF:FF:FF) for destination
    memset(&packet_buffer[4], 0xFF, 6);
    
    // Set source address - our own MAC address
    uint8_t mac[6];
    esp_wifi_get_mac(WIFI_IF_AP, mac);
    memcpy(&packet_buffer[10], mac, 6);
    
    // Copy broadcast address for BSSID as well
    memset(&packet_buffer[16], 0xFF, 6);
    
    // Copy our header and data after the 802.11 header
    memcpy(packet_buffer + HEADER_SIZE, &header, sizeof(data_packet_header_t));
    memcpy(packet_buffer + HEADER_SIZE + sizeof(data_packet_header_t), data, size);
    
    // Send packet
    esp_err_t ret = esp_wifi_80211_tx(WIFI_IF_AP, 
                                     packet_buffer, 
                                     HEADER_SIZE + sizeof(data_packet_header_t) + size, 
                                     true);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send data packet: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Sent data packet: Class1=%d, Class2=%d, Class3=%d, Size=%d bytes",
                class_counts[0], class_counts[1], class_counts[2], size);
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
    
    while (1) {
        // Wait for the next check interval
        vTaskDelayUntil(&last_wake_time, check_interval);
        
        // Process packets
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
    
    // Fill with sine wave values
    for (int i = 0; i < count; i++) {
        values[i] = i * 0.1f;
    }
    
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
    
    ESP_LOGI(TAG, "->Scheduler Statistics:");
    ESP_LOGI(TAG, "  Packets processed: %lu", scheduler_ctx.packets_processed);
    ESP_LOGI(TAG, "  Packets transmitted: %lu", scheduler_ctx.packets_transmitted);
    ESP_LOGI(TAG, "  Deadline misses: %lu", scheduler_ctx.deadline_misses);
    
    // Queue status
    int queue_length[MAX_CLASSES];
    for (int i = 0; i < MAX_CLASSES; i++) {
        queue_length[i] = scheduler_ctx.packet_queues[i].count;
    }
    
    ESP_LOGI(TAG, "  Queue status: Class1=%d, Class2=%d, Class3=%d", 
            queue_length[0], queue_length[1], queue_length[2]);
    
    xSemaphoreGive(scheduler_ctx.mutex);
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

    // Initialize WiFi AP
    ESP_LOGI(TAG, "Initializing WiFi in AP mode");
    wifi_init_softap();
    
    // Initialize packet scheduler
    ESP_LOGI(TAG, "Initializing packet scheduler");
    scheduler_init();
    
    // Demo: Create test packets with different data types
    ESP_LOGI(TAG, "Submitting test packets");
    
    // Create test packets for each class
    create_test_int32_packet(CLASS_1, 10);  // Class 1: 10 INT32 values
    create_test_float_packet(CLASS_2, 8);   // Class 2: 8 FLOAT values
    
    // INT16 values for Class 3
    int16_t test_int16[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
    scheduler_submit_packet(CLASS_3, test_int16, 10);
    
    // Periodic task for statistics and additional test packets
    int counter = 0;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(50000));  // Every 50 seconds
        
        // Print statistics
        print_scheduler_stats();
        
        // Create additional test packets periodically
        counter++;
        if (counter % 3 == 0) {
            create_test_int32_packet(CLASS_1, 5);
        } else if (counter % 3 == 1) {
            create_test_float_packet(CLASS_2, 4);
        } else {
            int16_t more_data[] = {100, 200, 300, 400, 500};
            scheduler_submit_packet(CLASS_3, more_data, 5);
        }
    }
}