/**
 * ESP32 WiFi Station with Packet Scheduler and Sender
 * 
 * This implementation connects to a WiFi network first and then
 * sends control and data packets to the AP using the packet scheduler.
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
#include "msgqueue.h"
#include "types.h"

/* WiFi configuration */
#define WIFI_SSID      "new_ssid"
#define WIFI_PASS      "new_password"
#define MAXIMUM_RETRY  CONFIG_ESP_MAXIMUM_RETRY

/* Scheduler Configuration */
#define MAX_CLASSES              3     // 3 classes (Class 1: 3s, Class 2: 5s, Class 3: 6s)
#define SCHEDULER_CHECK_INTERVAL_MS 50 // How often to check queues
#define MAX_TX_SIZE              1400  // Maximum data size for transmission buffer
#define MAX_POINT_SIZE           20
// TODO: keyboard input for ddl and period
#define CLASS_DDL_1              6000
#define CLASS_DDL_2              10000
#define CLASS_DDL_3              10000
#define DDL_GAP                  100
#define CLASS1_DATA_COUNT        10
#define CLASS2_DATA_COUNT        8
#define CLASS3_DATA_COUNT        12

/* Packet type identifier */
#define PACKET_SIGNATURE  0xA5B6C7D0

/* FreeRTOS event group to signal when we are connected */
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static const char *TAG = "wifi-sta-sender";

static const uint16_t Type_Size[NUM_DATA_TYPE] = {sizeof(int8_t), sizeof(int16_t), sizeof(int32_t), sizeof(float), sizeof(double)};
static uint32_t Class_Deadlines[MAX_CLASSES] = {CLASS_DDL_1, CLASS_DDL_2, CLASS_DDL_3};

static int s_retry_num = 0;

/* Data packet (for transmission not in queue) header structure */
typedef struct {
    uint32_t signature;  
    packet_type_t packet_type;    // Always PACKET_TYPE_DATA
    uint32_t class_counts[MAX_CLASSES]; // Number of items for each class
    uint32_t data_counts[MAX_CLASSES]; // Number of elements for each class
    data_type_t class_types[MAX_CLASSES]; // Data type for each class
    uint32_t total_size;          // Total size of all data in bytes
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
    uint32_t points_processed;   // Total packets processed
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

/* Get the current time in milliseconds */
static uint32_t get_current_time_ms(void)
{
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
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
                
            case WIFI_EVENT_STA_DISCONNECTED:
                if (s_retry_num < MAXIMUM_RETRY) {
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
    // const char* wifi_ssid = "myssid";  // MUST MATCH AP SSID
    // const char* wifi_password = "mypassword";  // MUST MATCH AP PASSWORD
    
    wifi_config_t wifi_config = {0};
    
    // Copy SSID and password to config
    strncpy((char*)wifi_config.sta.ssid, WIFI_SSID, sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char*)wifi_config.sta.password, WIFI_PASS, sizeof(wifi_config.sta.password) - 1);
    
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
    scheduler_ctx.class_types[CLASS_2] = DATA_TYPE_INT32;  // Class 2 (5s) - FLOAT
    scheduler_ctx.class_types[CLASS_3] = DATA_TYPE_INT32;  // Class 3 (6s) - INT16

    // Initialize statistics
    scheduler_ctx.points_processed = 0;
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
    uint16_t element_size = Type_Size[(int)data_type];
    
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
            packet.deadline = current_time + CLASS_DDL_1;  // 3 seconds
            break;
        case CLASS_2:
            packet.deadline = current_time + CLASS_DDL_2;  // 5 seconds
            break;
        case CLASS_3:
            packet.deadline = current_time + CLASS_DDL_3;  // 6 seconds
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
    //TODO: optimize to check deadline once
    
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

    // TODO: Update Earliest ddl when generating tasks
    if(current_time + DDL_GAP < earliest_deadline){
        return;
    }

    
    // Always transmit if there are packets, regardless of deadline
    ESP_LOGI(TAG, "Processing packets: %d packets in queue", total_packets);
    
    // // First send the control packet
    // esp_err_t ret = send_control_packet();
    // if (ret != ESP_OK) {
    //     ESP_LOGE(TAG, "Failed to send control packet");
    //     return;
    // }
    
    // Now prepare the data packet
    uint8_t class_counts[MAX_CLASSES] = {0};
    
    // Calculate total size needed for the data buffer
    uint16_t total_data_size = 0;
    
    // Take mutex again for size calculation
    // TODO: fix size calculation
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
    // TODO: get rid of malloc
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
        int8_t point_count = 0;

        while(1){
            
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
                        scheduler_ctx.points_processed++;
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
                point_count++;
                
                if (xSemaphoreTake(scheduler_ctx.mutex, portMAX_DELAY) == pdTRUE) {
                    scheduler_ctx.points_processed++;
                    xSemaphoreGive(scheduler_ctx.mutex);
                }
            }
            else{
                break;
            }
        }

        // Update class count
        class_counts[class_id] = point_count;
        point_count = 0;

        

    }
    
    // Calculate actual data size
    uint16_t actual_data_size = buffer_size - remaining_space;
    
    // Only send if we have data to send
    if (actual_data_size > 0) {
        // Send the data packet
        esp_err_t ret = send_data_packet(data_buffer, actual_data_size, class_counts);
        
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



/* Send the data packet with all class data */
static esp_err_t send_data_packet(uint8_t *data, uint16_t size, uint8_t class_counts[MAX_CLASSES])
{
    // Create proper 802.11 data frame header
    const size_t HEADER_SIZE = 24;  // Basic 802.11 header size
    
    // Create data packet header
    data_packet_header_t header = {0};
    header.signature = PACKET_SIGNATURE;
    header.packet_type = PACKET_TYPE_DATA;  // Set packet type
    header.total_size = size;
    header.timestamp = get_current_time_ms();
    
    // Copy class counts
    for (int i = 0; i < MAX_CLASSES; i++) {
        header.class_counts[i] = class_counts[i];
    }
    // Manually set header data type and count info for now

    header.data_counts[0] = CLASS1_DATA_COUNT;
    header.data_counts[1] = CLASS2_DATA_COUNT;
    header.data_counts[2] = CLASS3_DATA_COUNT;
    header.class_types[0] = DATA_TYPE_INT32;
    header.class_types[1] = DATA_TYPE_INT32;
    header.class_types[2] = DATA_TYPE_INT32;

    
    // Allocate buffer for 802.11 header + our header + data
    uint8_t *packet_buffer = malloc(HEADER_SIZE + sizeof(data_packet_header_t) + size);
    if (packet_buffer == NULL) {
        ESP_LOGE(TAG, "Failed to allocate packet buffer");
        return ESP_FAIL;
    }
    
    // Clear buffer
    memset(packet_buffer, 0, HEADER_SIZE + sizeof(data_packet_header_t) + size);
    
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
    
    // Copy our header and data after the 802.11 header
    memcpy(packet_buffer + HEADER_SIZE, &header, sizeof(data_packet_header_t));
    memcpy(packet_buffer + HEADER_SIZE + sizeof(data_packet_header_t), data, size);

    for(int i = 0; i < size + sizeof(data_packet_header_t); i+=4){
        ESP_LOGE(TAG, "Send bytes %lx bytes", *((uint32_t*) (packet_buffer + HEADER_SIZE+i)));
    }
    
    // Send packet
    esp_err_t ret = esp_wifi_80211_tx(WIFI_IF_STA, 
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
    
    // After starting the task, immediately send a control packet
    // to inform AP about class configurations
    vTaskDelay(pdMS_TO_TICKS(1000));  // Wait for WiFi to fully initialize
    // send_control_packet();
    
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
    int32_t values[MAX_POINT_SIZE];
    
    // Fill with sequential values
    for (int i = 0; i < count; i++) {
        values[i] = i;
    }

    
    // Submit packet with this data
    scheduler_submit_packet(class_id, values, count);
    
}

/* Create a test float packet */
void create_test_float_packet(class_id_t class_id, uint16_t count)
{
    // Create array of float values
    float values[MAX_POINT_SIZE];
    // Fill with values
    for (int i = 0; i < count; i++) {
        values[i] = i * 0.1f;
    }
    
    // Submit packet with this data
    scheduler_submit_packet(class_id, values, count);
}

/* Create a test int16 packet */
void create_test_int16_packet(class_id_t class_id, uint16_t count)
{
    // Create array of int16 values
    int16_t values[MAX_POINT_SIZE];
    // Fill with sequential values
    for (int i = 0; i < count; i++) {
        values[i] = i * 10;
    }
    
    // Submit packet with this data
    scheduler_submit_packet(class_id, values, count);
    
}

/* Print scheduler statistics */
void print_scheduler_stats(void)
{
    if (xSemaphoreTake(scheduler_ctx.mutex, portMAX_DELAY) != pdTRUE) {
        return;
    }
    
    ESP_LOGI(TAG, "->Scheduler Statistics:");
    ESP_LOGI(TAG, "  Points processed: %lu", scheduler_ctx.points_processed);
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

    // Initialize WiFi and connect to AP
    ESP_LOGI(TAG, "Starting WiFi in station mode");
    wifi_init_sta();
    
    // Short delay to allow WiFi to initialize fully
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    // Initialize packet scheduler
    ESP_LOGI(TAG, "Initializing packet scheduler");
    scheduler_init();
    
    // // Set initial class types to match expected formats
    // scheduler_set_class_type(CLASS_1, DATA_TYPE_INT32);
    // scheduler_set_class_type(CLASS_2, DATA_TYPE_FLOAT);
    // scheduler_set_class_type(CLASS_3, DATA_TYPE_INT16);
    
    // Short delay to allow scheduler to initialize
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Demo: Create test packets with different data types
    ESP_LOGI(TAG, "Submitting initial test packets");
    
    // Create test packets for each class
    create_test_int32_packet(CLASS_1, CLASS1_DATA_COUNT);  // Class 1: 10 INT32 values
    create_test_float_packet(CLASS_2, CLASS2_DATA_COUNT);   // Class 2: 8 FLOAT values
    create_test_int16_packet(CLASS_3, CLASS3_DATA_COUNT);  // Class 3: 12 INT16 values
    
    // Periodic task for statistics and additional test packets
    int counter = 0;
    while (1) {
        // First create a more frequent schedule for initial testing
        vTaskDelay(pdMS_TO_TICKS(1000));

        // Print statistics
        print_scheduler_stats();
        
        // Create additional test packets periodically
        counter++;
        
        // Generate different test data for each class
        // Class 1 (INT32) - Period: 3 seconds
        if (counter % 3 == 0) {
            create_test_int32_packet(CLASS_1, CLASS1_DATA_COUNT);
        }
        
        // Class 2 (FLOAT) - Period: 5 seconds  
        if (counter % 5 == 0) {
            create_test_int32_packet(CLASS_2, CLASS2_DATA_COUNT);
        }
        
        // Class 3 (INT16) - Period: 6 seconds
        if (counter % 6 == 0) {
            create_test_int32_packet(CLASS_3, CLASS3_DATA_COUNT);
        }
    }
}