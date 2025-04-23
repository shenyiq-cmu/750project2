/**
 * @file terminal_cmd.h
 * @brief Terminal command definitions and interface for ESP32 scheduler
 */

#ifndef TERMINAL_CMD_H
#define TERMINAL_CMD_H

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "esp_console.h"
#include "esp_vfs_dev.h"
#include "linenoise/linenoise.h"
#include "esp_random.h"  // For esp_random() function
#include "esp_wifi.h"


/* Terminal Configuration */
#define UART_NUM            UART_NUM_0
#define UART_BAUD_RATE      115200
#define UART_BUF_SIZE       1024
#define MAX_CMDLINE_ARGS    8
#define MAX_CMDLINE_LENGTH  256

/* Class and Deadline Configuration */
#define MAX_CLASSES         4     // Number of classes
#define DEFAULT_CLASS1_PERIOD 3000    // Default: 3 seconds
#define DEFAULT_CLASS2_PERIOD 5000    // Default: 5 seconds
#define DEFAULT_CLASS3_PERIOD 6000    // Default: 6 seconds
#define DEFAULT_PROCESSING_THRESHOLD 1000  // Default: 1 second (1000ms)

/* Packet Count Configuration */
#define DEFAULT_CLASS1_COUNT 5    // Default packet count for Class 1
#define DEFAULT_CLASS2_COUNT 4    // Default packet count for Class 2
#define DEFAULT_CLASS3_COUNT 6    // Default packet count for Class 3
#define MIN_PACKET_COUNT     1    // Minimum packet count
#define MAX_PACKET_COUNT     100  // Maximum packet count

#define MIN_PERIOD          1000      // Minimum period: 1 second
#define MAX_PERIOD          10000     // Maximum period: 10 seconds
#define MIN_DEADLINE_FACTOR 0.8       // Deadline can be 80% of period at minimum
#define MAX_DEADLINE_FACTOR 1.2       // Deadline can be 120% of period at maximum
#define MIN_THRESHOLD       100       // Minimum processing threshold: 100ms
#define MAX_THRESHOLD       5000      // Maximum processing threshold: 5000ms (5s)

/* Data type options for command line */
#define TYPE_OPTION_INT8  "int8"
#define TYPE_OPTION_INT16 "int16"
#define TYPE_OPTION_INT32 "int32"
#define TYPE_OPTION_FLOAT "float"
#define TYPE_OPTION_DOUBLE "double"

/* Random Packet Configuration */
#define DEFAULT_RANDOM_PACKET_MIN_INTERVAL 500    // Default min interval: 500ms
#define DEFAULT_RANDOM_PACKET_MAX_INTERVAL 3000   // Default max interval: 3s
#define DEFAULT_RANDOM_PACKET_BURST_PERIOD 10000  // Default burst period: 10s
#define DEFAULT_RANDOM_PACKET_BURST_INTERVAL 50   // Default burst interval: 50ms
#define DEFAULT_RANDOM_PACKET_COUNT 10             // Default packet size
#define DEFAULT_RANDOM_PACKET_TYPE DATA_TYPE_INT32 // Default packet type
#define DEFAULT_RANDOM_PACKET_BURST_ENABLED true  // Default: Burst mode enabled

// WiFi default configuration
#define DEFAULT_WIFI_TX_POWER    80      // Default: 20dBm (value 80 = 20dBm)
#define DEFAULT_WIFI_PS_MODE     WIFI_PS_MIN_MODEM  // Default: Min modem
#define DEFAULT_WIFI_PROTOCOL    (WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N)  // Default: 11bgn

/* Adaptive TX power configuration */
#define RSSI_EXCELLENT    -5    // -15 dBm or better: excellent signal
#define RSSI_GOOD         -20    // -25 dBm or better: good signal
#define RSSI_FAIR         -89    // -35 dBm or better: fair signal
#define RSSI_POOR         -90   // -45 dBm or better: poor signal

#define TX_POWER_MIN      8      // 2 dBm (minimum)
#define TX_POWER_LOW      44     // 11 dBm
#define TX_POWER_MEDIUM   60     // 15 dBm
#define TX_POWER_HIGH     80     // 20 dBm (maximum)

/* Forward declarations for scheduler types */
typedef enum {
    CLASS_1 = 0,                 // Class 1
    CLASS_2 = 1,                 // Class 2
    CLASS_3 = 2,                 // Class 3
    CLASS_RANDOM = 3,            // Random packet class
} class_id_t;

typedef enum {
    DATA_TYPE_INT8 = 0,          // 8-bit integer
    DATA_TYPE_INT16 = 1,         // 16-bit integer
    DATA_TYPE_INT32 = 2,         // 32-bit integer
    DATA_TYPE_FLOAT = 3,         // 32-bit float
    DATA_TYPE_DOUBLE = 4,        // 64-bit double
} data_type_t;

/* Configuration structure to be passed back to the main program */
typedef struct {
    uint32_t class_periods[MAX_CLASSES];   // Period for each class (ms)
    uint32_t class_deadlines[MAX_CLASSES]; // Deadline for each class (ms)
    data_type_t class_types[MAX_CLASSES];  // Data type for each class
    uint16_t packet_counts[MAX_CLASSES];   // Packet count for each class
    uint32_t processing_threshold;         // Deadline processing threshold (ms)
    bool start_program;                    // Flag to indicate if program should start
    
    // Random packet generation parameters
    bool random_packet_enabled;            // Enable random packet generation
    uint32_t random_packet_min_interval;   // Minimum interval between random packets (ms)
    uint32_t random_packet_max_interval;   // Maximum interval between random packets (ms)
    uint32_t random_packet_burst_period;   // Period after which to switch to burst mode (ms)
    uint32_t random_packet_burst_interval; // Interval between packets in burst mode (ms)
    uint16_t random_packet_count;           // Number of data elements in random packet
    bool random_packet_burst_enabled;  // New field to enable/disable burst mode
    data_type_t random_packet_type;        // Data type for random packet

    // WiFi parameters
    int8_t wifi_tx_power;          // WiFi transmit power
    wifi_ps_type_t wifi_ps_mode;   // WiFi power save mode
    uint8_t wifi_protocol;         // WiFi protocol bitmap

    // adaptive tx power on rssi 
    bool auto_tx_power;            // Whether to automatically adjust TX power based on RSSI
    uint32_t auto_tx_power_interval;  // Interval (ms) for checking and adjusting TX power

} scheduler_config_t;

/**
 * @brief Initialize terminal and process configuration commands
 * 
 * @param[out] config Pointer to configuration structure to be filled
 * @return ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t terminal_init_and_configure(scheduler_config_t *config);

/**
 * @brief Process a single line of command input
 * 
 * @param[in] line Command line to process
 * @param[in,out] config Configuration to update based on command
 * @return true if program should start, false if more configuration needed
 */
bool process_command(char *line, scheduler_config_t *config);

#endif /* TERMINAL_CMD_H */