/**
 * @file terminal_commands.c
 * @brief Implementation of terminal commands for ESP32 scheduler
 */

#include "terminal_cmd.h"
#include "esp_log.h"
#include "esp_random.h" // Add proper header for esp_random

static const char *TAG = "terminal"; // This will be used for ESP_LOG* calls

/* Command structure */
typedef struct {
    const char *command;
    const char *help;
    int (*function)(int argc, char **argv, scheduler_config_t *config);
} cmd_t;

/* Forward declarations for terminal commands */
static int cmd_help(int argc, char **argv, scheduler_config_t *config);
static int cmd_status(int argc, char **argv, scheduler_config_t *config);
static int cmd_set_class(int argc, char **argv, scheduler_config_t *config);
static int cmd_reset(int argc, char **argv, scheduler_config_t *config);
static int cmd_random(int argc, char **argv, scheduler_config_t *config);
static int cmd_start(int argc, char **argv, scheduler_config_t *config);

/* Helper function for generating random values */
static uint32_t random_range(uint32_t min, uint32_t max) 
{
    return min + (esp_random() % (max - min + 1));
}

/* Help command implementation */
static int cmd_help(int argc, char **argv, scheduler_config_t *config) 
{
    printf("\nAvailable commands:\n");
    printf("  %-10s - Display this help message\n", "help");
    printf("  %-10s - Show current class periods and deadlines\n", "status");
    printf("  %-10s - Set period and deadline for a class\n", "set");
    printf("  %-10s - Reset all classes to default values\n", "reset");
    printf("  %-10s - Set random periods and deadlines for all classes\n", "random");
    printf("  %-10s - Start the program with current configuration\n", "start");
    
    printf("\nClass-specific commands:\n");
    printf("  set <class> <period> <deadline>  - Set period and deadline for a class (1-3)\n");
    printf("                                     Use -a for auto-generated values\n");
    printf("  Example: set 1 4000 3500        - Set Class 1 period to 4s, deadline to 3.5s\n");
    printf("  Example: set 2 5000 -a          - Set Class 2 period to 5s, auto deadline\n");
    printf("  Example: set 3 -a -a            - Set Class 3 with auto period and deadline\n");
    
    printf("\nOnce you've configured all parameters, use 'start' to begin execution.\n");
    return 0;
}

/* Status command implementation */
static int cmd_status(int argc, char **argv, scheduler_config_t *config) 
{
    ESP_LOGI(TAG, "Displaying current class configuration");
    printf("\nCurrent Class Configuration:\n");
    for (int i = 0; i < MAX_CLASSES; i++) {
        const char *type_str;
        switch (config->class_types[i]) {
            case DATA_TYPE_INT8:   type_str = "INT8";   break;
            case DATA_TYPE_INT16:  type_str = "INT16";  break;
            case DATA_TYPE_INT32:  type_str = "INT32";  break;
            case DATA_TYPE_FLOAT:  type_str = "FLOAT";  break;
            case DATA_TYPE_DOUBLE: type_str = "DOUBLE"; break;
            default:               type_str = "UNKNOWN"; break;
        }
        
        printf("Class %d: Type=%s, Period=%lu ms, Deadline=%lu ms\n", 
               i + 1, type_str, config->class_periods[i], config->class_deadlines[i]);
    }
    
    return 0;
}

/* Set class parameters command */
static int cmd_set_class(int argc, char **argv, scheduler_config_t *config) 
{
    ESP_LOGI(TAG, "Setting class parameters");
    if (argc < 2) {
        printf("Usage: set <class> <period> <deadline>\n");
        printf("       Use -a for auto values\n");
        return 1;
    }
    
    // Parse class number
    int class_num = atoi(argv[1]);
    if (class_num < 1 || class_num > MAX_CLASSES) {
        printf("Error: Invalid class number. Must be between 1 and %d.\n", MAX_CLASSES);
        return 1;
    }
    
    class_id_t class_id = (class_id_t)(class_num - 1);  // Convert to 0-based index
    
    // Initialize with current values
    uint32_t period = config->class_periods[class_id];
    uint32_t deadline = config->class_deadlines[class_id];
    
    // Parse period
    if (argc >= 3) {
        if (strcmp(argv[2], "-a") == 0) {
            // Auto-generate period
            period = random_range(MIN_PERIOD, MAX_PERIOD);
            printf("Auto-generated period: %lu ms\n", period);
        } else {
            period = atoi(argv[2]);
            if (period < MIN_PERIOD || period > MAX_PERIOD) {
                printf("Warning: Period outside recommended range [%d-%d]. Clamping.\n", 
                       MIN_PERIOD, MAX_PERIOD);
                period = (period < MIN_PERIOD) ? MIN_PERIOD : MAX_PERIOD;
            }
        }
    } else {
        printf("Period unchanged: %lu ms\n", period);
    }
    
    // Parse deadline
    if (argc >= 4) {
        if (strcmp(argv[3], "-a") == 0) {
            // Auto-generate deadline based on period
            float factor = (MIN_DEADLINE_FACTOR + 
                          ((MAX_DEADLINE_FACTOR - MIN_DEADLINE_FACTOR) * 
                           (esp_random() % 100) / 100.0f));
            deadline = (uint32_t)(period * factor);
            printf("Auto-generated deadline: %lu ms (%.1f%% of period)\n", 
                   deadline, factor * 100);
        } else {
            deadline = atoi(argv[3]);
            
            // Calculate if the deadline is reasonable
            float factor = (float)deadline / period;
            if (factor < MIN_DEADLINE_FACTOR || factor > MAX_DEADLINE_FACTOR) {
                printf("Warning: Deadline factor (%.2f) outside recommended range [%.1f-%.1f].\n",
                       factor, MIN_DEADLINE_FACTOR, MAX_DEADLINE_FACTOR);
            }
        }
    } else if (argc == 3) {
        // If period changed but deadline not specified, adjust deadline proportionally
        deadline = (uint32_t)(period * ((float)deadline / config->class_periods[class_id]));
        printf("Auto-adjusted deadline: %lu ms\n", deadline);
    } else {
        printf("Deadline unchanged: %lu ms\n", deadline);
    }
    
    // Update values
    config->class_periods[class_id] = period;
    config->class_deadlines[class_id] = deadline;
    
    printf("Updated Class %d: Period=%lu ms, Deadline=%lu ms\n", 
           class_num, period, deadline);
    
    return 0;
}

/* Reset all classes to default values */
static int cmd_reset(int argc, char **argv, scheduler_config_t *config) 
{
    ESP_LOGI(TAG, "Resetting all classes to default values");
    // Reset to default values
    config->class_periods[CLASS_1] = DEFAULT_CLASS1_PERIOD;
    config->class_periods[CLASS_2] = DEFAULT_CLASS2_PERIOD;
    config->class_periods[CLASS_3] = DEFAULT_CLASS3_PERIOD;
    
    // Set deadlines equal to periods by default
    config->class_deadlines[CLASS_1] = DEFAULT_CLASS1_PERIOD;
    config->class_deadlines[CLASS_2] = DEFAULT_CLASS2_PERIOD;
    config->class_deadlines[CLASS_3] = DEFAULT_CLASS3_PERIOD;
    
    // Set default data types
    config->class_types[CLASS_1] = DATA_TYPE_INT32;  // Class 1 - INT32
    config->class_types[CLASS_2] = DATA_TYPE_FLOAT;  // Class 2 - FLOAT
    config->class_types[CLASS_3] = DATA_TYPE_INT16;  // Class 3 - INT16
    
    printf("All classes reset to default values.\n");
    
    return 0;
}

/* Set random values for all classes */
static int cmd_random(int argc, char **argv, scheduler_config_t *config) 
{
    ESP_LOGI(TAG, "Setting random values for all classes");
    printf("Setting random values for all classes:\n");
    
    for (int i = 0; i < MAX_CLASSES; i++) {
        // Generate random period
        uint32_t period = random_range(MIN_PERIOD, MAX_PERIOD);
        
        // Generate random deadline factor
        float factor = MIN_DEADLINE_FACTOR + 
                      ((MAX_DEADLINE_FACTOR - MIN_DEADLINE_FACTOR) * 
                       (esp_random() % 100) / 100.0f);
        
        uint32_t deadline = (uint32_t)(period * factor);
        
        // Update values
        config->class_periods[i] = period;
        config->class_deadlines[i] = deadline;
        
        printf("Class %d: Period=%lu ms, Deadline=%lu ms (%.1f%% of period)\n", 
               i + 1, period, deadline, factor * 100);
    }
    
    return 0;
}

/* Start the program with the current configuration */
static int cmd_start(int argc, char **argv, scheduler_config_t *config) 
{
    ESP_LOGI(TAG, "Starting program with current configuration");
    printf("\nStarting program with following configuration:\n");
    cmd_status(0, NULL, config);
    
    printf("\nProgram starting...\n");
    config->start_program = true;
    return 0;
}

/* Define the command array */
static const cmd_t commands[] = {
    {"help", "Print the list of commands", cmd_help},
    {"status", "Show current class periods and deadlines", cmd_status},
    {"set", "Set period and deadline for a class", cmd_set_class},
    {"reset", "Reset all classes to default values", cmd_reset},
    {"random", "Set random periods and deadlines for all classes", cmd_random},
    {"start", "Start program with current configuration", cmd_start},
    {NULL, NULL, NULL}
};

/* Process the received command */
bool process_command(char *line, scheduler_config_t *config) 
{
    if (line == NULL || strlen(line) == 0) {
        return false;
    }
    
    // Add to history
    linenoiseHistoryAdd(line);
    
    // Tokenize the command
    int argc = 0;
    char *argv[MAX_CMDLINE_ARGS];
    char *token = strtok(line, " ");
    
    while (token != NULL && argc < MAX_CMDLINE_ARGS) {
        argv[argc++] = token;
        token = strtok(NULL, " ");
    }
    
    // Find and execute the command
    for (const cmd_t *cmd = commands; cmd->command != NULL; cmd++) {
        if (strcmp(argv[0], cmd->command) == 0) {
            cmd->function(argc, argv, config);
            return config->start_program;
        }
    }
    
    printf("Unknown command: %s\n", argv[0]);
    cmd_help(0, NULL, config);
    return false;
}

/* Initialize terminal and process configuration */
esp_err_t terminal_init_and_configure(scheduler_config_t *config) 
{
    ESP_LOGI(TAG, "Initializing terminal interface");
    /* Configure UART parameters */
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    /* Install UART driver for interrupt-driven reads and writes */
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, UART_BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
    
    /* Configure VFS for linenoise */
    esp_vfs_dev_uart_port_set_rx_line_endings(UART_NUM, ESP_LINE_ENDINGS_CR);
    esp_vfs_dev_uart_port_set_tx_line_endings(UART_NUM, ESP_LINE_ENDINGS_CRLF);
    
    /* Tell VFS to use UART driver */
    esp_vfs_dev_uart_use_driver(UART_NUM);
    
    /* Configure linenoise */
    linenoiseSetMultiLine(0);
    linenoiseSetDumbMode(1);  // No advanced terminal functionality
    
    /* Initialize the console */
    esp_console_config_t console_config = {
        .max_cmdline_length = MAX_CMDLINE_LENGTH,
        .max_cmdline_args = MAX_CMDLINE_ARGS,
    };
    ESP_ERROR_CHECK(esp_console_init(&console_config));
    
    /* Welcome message */
    printf("\n\n==================================================\n");
    printf("    ESP32 WiFi Packet Scheduler Configuration    \n");
    printf("==================================================\n");
    printf("Configure the scheduler parameters and then enter 'start'.\n");
    printf("Type 'help' to view available commands\n\n");
    
    /* Initialize configuration with default values */
    config->start_program = false;
    
    // Set default periods
    config->class_periods[CLASS_1] = DEFAULT_CLASS1_PERIOD;
    config->class_periods[CLASS_2] = DEFAULT_CLASS2_PERIOD;
    config->class_periods[CLASS_3] = DEFAULT_CLASS3_PERIOD;
    
    // Set default deadlines equal to periods
    config->class_deadlines[CLASS_1] = DEFAULT_CLASS1_PERIOD;
    config->class_deadlines[CLASS_2] = DEFAULT_CLASS2_PERIOD;
    config->class_deadlines[CLASS_3] = DEFAULT_CLASS3_PERIOD;
    
    // Set default data types
    config->class_types[CLASS_1] = DATA_TYPE_INT32;  // Class 1 - INT32
    config->class_types[CLASS_2] = DATA_TYPE_FLOAT;  // Class 2 - FLOAT
    config->class_types[CLASS_3] = DATA_TYPE_INT16;  // Class 3 - INT16
    
    // Display current configuration
    cmd_status(0, NULL, config);
    
    // Main configuration loop
    char *line;
    while (!config->start_program) {
        /* Get a line using linenoise */
        line = linenoise("config> ");
        
        /* If line is not NULL, process the command */
        if (line != NULL) {
            bool should_start = process_command(line, config);
            linenoiseFree(line);
            
            if (should_start) {
                break;
            }
        }
        
        /* Small delay to prevent hogging CPU */
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    return ESP_OK;
}