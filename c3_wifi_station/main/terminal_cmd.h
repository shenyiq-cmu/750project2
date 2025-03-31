/**
 * @file terminal_commands.h
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

/* Terminal Configuration */
#define UART_NUM            UART_NUM_0
#define UART_BAUD_RATE      115200
#define UART_BUF_SIZE       1024
#define MAX_CMDLINE_ARGS    8
#define MAX_CMDLINE_LENGTH  256

/* Class and Deadline Configuration */
#define MAX_CLASSES         3     // Number of classes
#define DEFAULT_CLASS1_PERIOD 3000    // Default: 3 seconds
#define DEFAULT_CLASS2_PERIOD 5000    // Default: 5 seconds
#define DEFAULT_CLASS3_PERIOD 6000    // Default: 6 seconds

#define MIN_PERIOD          100      // Minimum period: 100ms
#define MAX_PERIOD          10000     // Maximum period: 10 seconds
#define MIN_DEADLINE_FACTOR 0.8       // Deadline can be 80% of period at minimum
#define MAX_DEADLINE_FACTOR 1.2       // Deadline can be 120% of period at maximum

/* Forward declarations for scheduler types */
typedef enum {
    CLASS_1 = 0,                 // Class 1
    CLASS_2 = 1,                 // Class 2
    CLASS_3 = 2,                 // Class 3
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
    bool start_program;                     // Flag to indicate if program should start
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