/**
 * @file terminal_cmd.c
 * @brief Implementation of terminal commands for ESP32 scheduler
 */

#include "terminal_cmd.h"
#include "esp_log.h"
#include "esp_random.h"

static const char *TAG = "terminal";

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
static int cmd_threshold(int argc, char **argv, scheduler_config_t *config);
static int cmd_type(int argc, char **argv, scheduler_config_t *config);
static int cmd_packet_count(int argc, char **argv, scheduler_config_t *config);


/* Forward declarations for new terminal commands */
static int cmd_random_packet(int argc, char **argv, scheduler_config_t *config);
static int cmd_random_packet_type(int argc, char **argv, scheduler_config_t *config);
static int cmd_random_packet_count(int argc, char **argv, scheduler_config_t *config);
static int cmd_random_packet_burst(int argc, char **argv, scheduler_config_t *config);

/* Helper function for generating random values */
static uint32_t random_range(uint32_t min, uint32_t max) 
{
    return min + (esp_random() % (max - min + 1));
}

/* Command to set deadline for random packets */
static int cmd_random_packet_deadline(int argc, char **argv, scheduler_config_t *config)
{
    ESP_LOGI(TAG, "Setting random packet deadline");
    
    if (argc < 2) {
        printf("Usage: rdeadline <value_ms>\n");
        printf("       Use '-a' for auto value\n");
        printf("Current deadline: %lu ms\n", config->class_deadlines[CLASS_RANDOM]);
        return 1;
    }
    
    uint32_t deadline = config->class_deadlines[CLASS_RANDOM];
    
    if (strcmp(argv[1], "-a") == 0) {
        // Auto-generate deadline between 500-3000ms
        deadline = random_range(500, 3000);
        printf("Auto-generated deadline: %lu ms\n", deadline);
    } else {
        deadline = atoi(argv[1]);
        printf("Set deadline to %lu ms\n", deadline);
    }
    
    config->class_deadlines[CLASS_RANDOM] = deadline;
    
    return 0;
}

/* Command to enable/disable and configure random packet generation */
static int cmd_random_packet(int argc, char **argv, scheduler_config_t *config)
{
    ESP_LOGI(TAG, "Configuring random packet generation");
    
    if (argc < 2) {
        printf("Usage: random [on|off] [min_interval] [max_interval]\n");
        printf("       Use 'on' to enable, 'off' to disable\n");
        printf("       Specify intervals in milliseconds or '-a' for auto values\n");
        printf("Current status: %s\n", config->random_packet_enabled ? "enabled" : "disabled");
        printf("Min interval: %lu ms\n", config->random_packet_min_interval);
        printf("Max interval: %lu ms\n", config->random_packet_max_interval);
        return 1;
    }
    
    // Parse enable/disable
    if (strcmp(argv[1], "on") == 0) {
        config->random_packet_enabled = true;
        // Add this line to set a non-zero period for CLASS_RANDOM
        config->class_periods[CLASS_RANDOM] = 1;  // Just a placeholder, random task controls timing
        printf("Random packet generation enabled\n");
    } else if (strcmp(argv[1], "off") == 0) {
        config->random_packet_enabled = false;
        // Reset period to 0 when disabled
        config->class_periods[CLASS_RANDOM] = 0;
        printf("Random packet generation disabled\n");
        return 0;
    }
    
    // Parse min interval
    if (argc >= 3) {
        if (strcmp(argv[2], "-a") == 0) {
            // Auto-generate min interval
            config->random_packet_min_interval = random_range(100, 1000);
            printf("Auto-generated min interval: %lu ms\n", config->random_packet_min_interval);
        } else {
            config->random_packet_min_interval = atoi(argv[2]);
            printf("Set min interval to %lu ms\n", config->random_packet_min_interval);
        }
    }
    
    // Parse max interval
    if (argc >= 4) {
        if (strcmp(argv[3], "-a") == 0) {
            // Auto-generate max interval
            config->random_packet_max_interval = random_range(
                config->random_packet_min_interval + 500, 
                config->random_packet_min_interval + 5000);
            printf("Auto-generated max interval: %lu ms\n", config->random_packet_max_interval);
        } else {
            config->random_packet_max_interval = atoi(argv[3]);
            printf("Set max interval to %lu ms\n", config->random_packet_max_interval);
        }
    }
    
    // Validate intervals
    if (config->random_packet_min_interval >= config->random_packet_max_interval) {
        printf("Warning: Min interval (%lu) >= max interval (%lu). Setting max = min + 1000\n",
               config->random_packet_min_interval, config->random_packet_max_interval);
        config->random_packet_max_interval = config->random_packet_min_interval + 1000;
    }
    
    return 0;
}

/* Command to configure random packet type */
static int cmd_random_packet_type(int argc, char **argv, scheduler_config_t *config)
{
    ESP_LOGI(TAG, "Setting random packet data type");
    
    if (argc < 2) {
        printf("Usage: rtype <datatype>\n");
        printf("Available datatypes: int8, int16, int32, float, double\n");
        printf("Example: rtype int32\n");
        
        // Show current type
        const char *type_str;
        switch (config->random_packet_type) {
            case DATA_TYPE_INT8:   type_str = "INT8";   break;
            case DATA_TYPE_INT16:  type_str = "INT16";  break;
            case DATA_TYPE_INT32:  type_str = "INT32";  break;
            case DATA_TYPE_FLOAT:  type_str = "FLOAT";  break;
            case DATA_TYPE_DOUBLE: type_str = "DOUBLE"; break;
            default:               type_str = "UNKNOWN"; break;
        }
        printf("Current type: %s\n", type_str);
        return 1;
    }
    
    // Parse data type
    data_type_t new_type;
    const char *type_name = argv[1];
    
    if (strcasecmp(type_name, TYPE_OPTION_INT8) == 0) {
        new_type = DATA_TYPE_INT8;
    } else if (strcasecmp(type_name, TYPE_OPTION_INT16) == 0) {
        new_type = DATA_TYPE_INT16;
    } else if (strcasecmp(type_name, TYPE_OPTION_INT32) == 0) {
        new_type = DATA_TYPE_INT32;
    } else if (strcasecmp(type_name, TYPE_OPTION_FLOAT) == 0) {
        new_type = DATA_TYPE_FLOAT;
    } else if (strcasecmp(type_name, TYPE_OPTION_DOUBLE) == 0) {
        new_type = DATA_TYPE_DOUBLE;
    } else {
        printf("Error: Invalid data type '%s'.\n", type_name);
        printf("Available datatypes: int8, int16, int32, float, double\n");
        return 1;
    }
    
    // Update the data type
    config->random_packet_type = new_type;
    config->class_types[CLASS_RANDOM] = new_type;  // Also update in the class types array
    
    // Show confirmation
    const char *type_str;
    switch (new_type) {
        case DATA_TYPE_INT8:   type_str = "INT8";   break;
        case DATA_TYPE_INT16:  type_str = "INT16";  break;
        case DATA_TYPE_INT32:  type_str = "INT32";  break;
        case DATA_TYPE_FLOAT:  type_str = "FLOAT";  break;
        case DATA_TYPE_DOUBLE: type_str = "DOUBLE"; break;
        default:               type_str = "UNKNOWN"; break;
    }
    
    printf("Random packet type set to %s\n", type_str);
    
    return 0;
}

/* Command to configure random packet size */
static int cmd_random_packet_count(int argc, char **argv, scheduler_config_t *config)
{
    ESP_LOGI(TAG, "Setting random packet size");
    
    if (argc < 2) {
        printf("Usage: rsize <value>\n");
        printf("       Use '-a' for auto value\n");
        printf("Current size: %u elements\n", config->random_packet_count);
        return 1;
    }
    
    uint16_t size = config->random_packet_count;
    
    if (strcmp(argv[1], "-a") == 0) {
        // Auto-generate size
        size = random_range(5, 50);
        printf("Auto-generated packet size: %u elements\n", size);
    } else {
        // Parse user-provided value
        size = atoi(argv[1]);
        if (size < 1 || size > 200) {
            printf("Warning: Size outside recommended range [1-200]. Clamping.\n");
            size = (size < 1) ? 1 : 200;
        }
    }
    
    config->random_packet_count = size;
    printf("Random packet size set to %u elements\n", size);
    
    return 0;
}

/* Command to configure random packet burst parameters */
static int cmd_random_packet_burst(int argc, char **argv, scheduler_config_t *config)
{
    ESP_LOGI(TAG, "Setting random packet burst parameters");
    
    if (argc < 2) {
        printf("Usage: rburst [on|off] <period> <interval>\n");
        printf("       Use 'on' to enable, 'off' to disable burst mode\n");
        printf("       <period> is the time (ms) after which to switch to burst mode\n");
        printf("       <interval> is the interval (ms) between packets in burst mode\n");
        printf("       Use '-a' for auto values\n");
        printf("Current burst status: %s\n", config->random_packet_burst_enabled ? "ENABLED" : "DISABLED");
        printf("Current burst period: %lu ms\n", config->random_packet_burst_period);
        printf("Current burst interval: %lu ms\n", config->random_packet_burst_interval);
        return 1;
    }
    
    // Check if first argument is on/off
    if (strcasecmp(argv[1], "on") == 0) {
        config->random_packet_burst_enabled = true;
        printf("Burst mode enabled\n");
        
        // Shift arguments for period and interval
        argc--;
        argv++;
    } else if (strcasecmp(argv[1], "off") == 0) {
        config->random_packet_burst_enabled = false;
        printf("Burst mode disabled\n");
        
        // Shift arguments for period and interval
        argc--;
        argv++;
    }
    
    // Parse burst period if provided
    if (argc >= 2) {
        if (strcmp(argv[1], "-a") == 0) {
            // Auto-generate burst period
            config->random_packet_burst_period = random_range(5000, 20000);
            printf("Auto-generated burst period: %lu ms\n", config->random_packet_burst_period);
        } else {
            config->random_packet_burst_period = atoi(argv[1]);
            printf("Set burst period to %lu ms\n", config->random_packet_burst_period);
        }
    }
    
    // Parse burst interval if provided
    if (argc >= 3) {
        if (strcmp(argv[2], "-a") == 0) {
            // Auto-generate burst interval
            config->random_packet_burst_interval = random_range(20, 200);
            printf("Auto-generated burst interval: %lu ms\n", config->random_packet_burst_interval);
        } else {
            config->random_packet_burst_interval = atoi(argv[2]);
            printf("Set burst interval to %lu ms\n", config->random_packet_burst_interval);
        }
    }
    
    return 0;
}

/* Help command implementation */
static int cmd_help(int argc, char **argv, scheduler_config_t *config) 
{
    printf("\nAvailable commands:\n");
    printf("  %-10s - Display this help message\n", "help");
    printf("  %-10s - Show current class periods and deadlines\n", "status");
    printf("  %-10s - Set period and deadline for a class\n", "set");
    printf("  %-10s - Set data type for a class\n", "type");
    printf("  %-10s - Set packet count for a class\n", "count");
    printf("  %-10s - Set processing threshold\n", "threshold");
    printf("  %-10s - Reset all classes to default values\n", "reset");
    printf("  %-10s - Set random periods and deadlines for all classes\n", "random");
    printf("  %-10s - Start the program with current configuration\n", "start");
    
    printf("\nRandom packet commands:\n");
    printf("  %-10s - Enable the random packet (on/off) and packet generation\n", "rpacket");
    printf("  %-10s - Set random packet data type\n", "rtype");
    printf("  %-10s - Set random packet size\n", "rsize");
    printf("  %-10s - Configure random packet burst parameters\n", "rburst");
    printf("  %-10s - Set random packet deadline\n", "rdeadline");
    printf("  %-10s - Configure random packet burst parameters\n", "rburst");
    printf("  Example: rpacket on 500 2000  - Enable with min=500ms, max=2000ms\n");
    printf("  Example: rtype float         - Set type to FLOAT\n");
    printf("  Example: rsize 20            - Set size to 20 elements\n");
    printf("  Example: rdeadline 1500      - Set deadline to 1500ms\n");
    printf("  Example: rburst on 10000 50   - Enable burst mode, after 10s switch to 50ms intervals\n");
    printf("  Example: rburst off          - Disable burst mode\n");
    
    
    printf("\nClass-specific commands:\n");
    printf("  set <class> <period> <deadline>  - Set period and deadline for a class (1-3)\n");
    printf("                                     Use -a for auto-generated values\n");
    printf("  Example: set 1 4000 3500        - Set Class 1 period to 4s, deadline to 3.5s\n");
    printf("  Example: set 2 5000 -a          - Set Class 2 period to 5s, auto deadline\n");
    printf("  Example: set 3 -a -a            - Set Class 3 with auto period and deadline\n");
    
    printf("\nType command:\n");
    printf("  type <class> <datatype>         - Set data type for a class\n");
    printf("  Available types: int8, int16, int32, float, double\n");
    printf("  Example: type 1 int32           - Set Class 1 type to INT32\n");
    
    printf("\nCount command:\n");
    printf("  count <class> <value>           - Set packet count for a class\n");
    printf("  Example: count 1 10             - Set Class 1 packet count to 10\n");
    printf("  Example: count 2 -a             - Set Class 2 packet count to random value\n");
    
    printf("\nThreshold command:\n");
    printf("  threshold <value_ms>            - Set deadline processing threshold\n");
    printf("  Example: threshold 2000         - Set threshold to 2000ms (2s)\n");
    printf("  Example: threshold -a           - Set auto-generated threshold\n");
    
    printf("\nOnce you've configured all parameters, use 'start' to begin execution.\n");
    return 0;
}

/* Status command implementation */
static int cmd_status(int argc, char **argv, scheduler_config_t *config) 
{
    ESP_LOGI(TAG, "Displaying current class configuration");
    printf("\nCurrent Class Configuration:\n");
    for (int i = 0; i < MAX_CLASSES; i++) {  // Show only the 3 regular classes + random
        const char *type_str;
        switch (config->class_types[i]) {
            case DATA_TYPE_INT8:   type_str = "INT8";   break;
            case DATA_TYPE_INT16:  type_str = "INT16";  break;
            case DATA_TYPE_INT32:  type_str = "INT32";  break;
            case DATA_TYPE_FLOAT:  type_str = "FLOAT";  break;
            case DATA_TYPE_DOUBLE: type_str = "DOUBLE"; break;
            default:               type_str = "UNKNOWN"; break;
        }
        
        printf("Class %d: Type=%s, Period=%lu ms, Deadline=%lu ms, Count=%u\n", 
               i + 1, type_str, config->class_periods[i], config->class_deadlines[i],
               config->packet_counts[i]);
    }
    
    // Add threshold information
    printf("\nProcessing Threshold: %lu ms\n", config->processing_threshold);
    printf("(Tasks are processed when deadline is within this threshold)\n");
    
    // Add random packet information
    const char *type_str;
    switch (config->random_packet_type) {
        case DATA_TYPE_INT8:   type_str = "INT8";   break;
        case DATA_TYPE_INT16:  type_str = "INT16";  break;
        case DATA_TYPE_INT32:  type_str = "INT32";  break;
        case DATA_TYPE_FLOAT:  type_str = "FLOAT";  break;
        case DATA_TYPE_DOUBLE: type_str = "DOUBLE"; break;
        default:               type_str = "UNKNOWN"; break;
    }

    printf("\nRandom Packet Configuration: %s\n", 
       config->random_packet_enabled ? "ENABLED" : "DISABLED");
    printf("  Initial interval: %lu-%lu ms\n", 
        config->random_packet_min_interval, config->random_packet_max_interval);
    printf("  Burst mode: %s\n", config->random_packet_burst_enabled ? "ENABLED" : "DISABLED");
    if (config->random_packet_burst_enabled) {
        printf("  Burst settings: After %lu ms, switch to %lu ms intervals\n", 
            config->random_packet_burst_period, config->random_packet_burst_interval);
    }

    printf("  Packet: Type=%s, Size=%u elements\n", 
        type_str, config->random_packet_count);
    printf("  Deadline: %lu ms\n", config->class_deadlines[CLASS_RANDOM]);

    
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

/* Set packet count for a class */
static int cmd_packet_count(int argc, char **argv, scheduler_config_t *config)
{
    ESP_LOGI(TAG, "Setting packet count");
    
    if (argc < 3) {
        printf("Usage: count <class> <value>\n");
        printf("       Use '-a' for auto value\n");
        printf("Example: count 1 10     - Set Class 1 packet count to 10\n");
        printf("Example: count 2 -a     - Set Class 2 packet count to random value\n");
        return 1;
    }
    
    // Parse class number
    int class_num = atoi(argv[1]);
    if (class_num < 1 || class_num > MAX_CLASSES) {
        printf("Error: Invalid class number. Must be between 1 and %d.\n", MAX_CLASSES);
        return 1;
    }
    
    class_id_t class_id = (class_id_t)(class_num - 1);  // Convert to 0-based index
    
    // Get current count
    uint16_t count = config->packet_counts[class_id];
    
    // Parse new count
    if (strcmp(argv[2], "-a") == 0) {
        // Auto-generate count
        count = random_range(MIN_PACKET_COUNT, MAX_PACKET_COUNT);
        printf("Auto-generated packet count: %u\n", count);
    } else {
        // Parse user-provided value
        uint16_t new_count = atoi(argv[2]);
        if (new_count < MIN_PACKET_COUNT || new_count > MAX_PACKET_COUNT) {
            printf("Warning: Count outside recommended range [%d-%d]. Clamping.\n", 
                   MIN_PACKET_COUNT, MAX_PACKET_COUNT);
            count = (new_count < MIN_PACKET_COUNT) ? MIN_PACKET_COUNT : MAX_PACKET_COUNT;
        } else {
            count = new_count;
        }
    }
    
    // Update the count
    config->packet_counts[class_id] = count;
    printf("Updated Class %d packet count to %u\n", class_num, count);
    
    return 0;
}

/* Set class data type command */
static int cmd_type(int argc, char **argv, scheduler_config_t *config)
{
    ESP_LOGI(TAG, "Setting class data type");
    
    if (argc < 3) {
        printf("Usage: type <class> <datatype>\n");
        printf("Available datatypes: int8, int16, int32, float, double\n");
        printf("Example: type 1 int32\n");
        printf("Example: type 2 float\n");
        printf("Example: type 3 int16\n");
        return 1;
    }
    
    // Parse class number
    int class_num = atoi(argv[1]);
    if (class_num < 1 || class_num > MAX_CLASSES) {
        printf("Error: Invalid class number. Must be between 1 and %d.\n", MAX_CLASSES);
        return 1;
    }
    
    class_id_t class_id = (class_id_t)(class_num - 1);  // Convert to 0-based index
    
    // Parse data type
    data_type_t new_type;
    const char *type_name = argv[2];
    
    if (strcasecmp(type_name, TYPE_OPTION_INT8) == 0) {
        new_type = DATA_TYPE_INT8;
    } else if (strcasecmp(type_name, TYPE_OPTION_INT16) == 0) {
        new_type = DATA_TYPE_INT16;
    } else if (strcasecmp(type_name, TYPE_OPTION_INT32) == 0) {
        new_type = DATA_TYPE_INT32;
    } else if (strcasecmp(type_name, TYPE_OPTION_FLOAT) == 0) {
        new_type = DATA_TYPE_FLOAT;
    } else if (strcasecmp(type_name, TYPE_OPTION_DOUBLE) == 0) {
        new_type = DATA_TYPE_DOUBLE;
    } else {
        printf("Error: Invalid data type '%s'.\n", type_name);
        printf("Available datatypes: int8, int16, int32, float, double\n");
        return 1;
    }
    
    // Update the data type
    config->class_types[class_id] = new_type;
    
    // Show confirmation
    const char *type_str;
    switch (new_type) {
        case DATA_TYPE_INT8:   type_str = "INT8";   break;
        case DATA_TYPE_INT16:  type_str = "INT16";  break;
        case DATA_TYPE_INT32:  type_str = "INT32";  break;
        case DATA_TYPE_FLOAT:  type_str = "FLOAT";  break;
        case DATA_TYPE_DOUBLE: type_str = "DOUBLE"; break;
        default:               type_str = "UNKNOWN"; break;
    }
    
    printf("Updated Class %d: Type=%s\n", class_num, type_str);
    
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
    
    // Set default packet counts
    config->packet_counts[CLASS_1] = DEFAULT_CLASS1_COUNT;
    config->packet_counts[CLASS_2] = DEFAULT_CLASS2_COUNT;
    config->packet_counts[CLASS_3] = DEFAULT_CLASS3_COUNT;
    
    // Reset processing threshold
    config->processing_threshold = DEFAULT_PROCESSING_THRESHOLD;

    // Reset random packet parameters
    config->random_packet_enabled = false;
    config->random_packet_min_interval = DEFAULT_RANDOM_PACKET_MIN_INTERVAL;
    config->random_packet_max_interval = DEFAULT_RANDOM_PACKET_MAX_INTERVAL;
    config->random_packet_burst_period = DEFAULT_RANDOM_PACKET_BURST_PERIOD;
    config->random_packet_burst_interval = DEFAULT_RANDOM_PACKET_BURST_INTERVAL;
    config->random_packet_burst_enabled = DEFAULT_RANDOM_PACKET_BURST_ENABLED;
    config->random_packet_count = DEFAULT_RANDOM_PACKET_COUNT;
    config->random_packet_type = DEFAULT_RANDOM_PACKET_TYPE;
    
    printf("All classes reset to default values.\n");
    printf("Processing threshold reset to %lu ms.\n", config->processing_threshold);
    printf("Random packet generation disabled and reset to default values.\n");
    
    return 0;
}

/* Set threshold command implementation */
static int cmd_threshold(int argc, char **argv, scheduler_config_t *config)
{
    ESP_LOGI(TAG, "Setting processing threshold");
    
    if (argc < 2) {
        printf("Usage: threshold <value_ms>\n");
        printf("       Use '-a' for auto value\n");
        printf("Current threshold: %lu ms\n", config->processing_threshold);
        return 1;
    }
    
    uint32_t threshold = config->processing_threshold;
    
    if (strcmp(argv[1], "-a") == 0) {
        // Auto-generate threshold
        threshold = random_range(MIN_THRESHOLD, MAX_THRESHOLD);
        printf("Auto-generated threshold: %lu ms\n", threshold);
    } else {
        threshold = atoi(argv[1]);
        if (threshold < MIN_THRESHOLD || threshold > MAX_THRESHOLD) {
            printf("Warning: Threshold outside recommended range [%d-%d]. Clamping.\n", 
                   MIN_THRESHOLD, MAX_THRESHOLD);
            threshold = (threshold < MIN_THRESHOLD) ? MIN_THRESHOLD : MAX_THRESHOLD;
        }
    }
    
    config->processing_threshold = threshold;
    printf("Processing threshold set to %lu ms.\n", threshold);
    
    return 0;
}

/* Set random values for all classes */
static int cmd_random(int argc, char **argv, scheduler_config_t *config) 
{
    ESP_LOGI(TAG, "Setting random values for all classes");
    
    printf("Setting random values for all classes:\n");
    
    // These arrays define the possible data types and their names
    data_type_t possible_types[] = {
        DATA_TYPE_INT8, 
        DATA_TYPE_INT16, 
        DATA_TYPE_INT32,
        DATA_TYPE_FLOAT, 
        DATA_TYPE_DOUBLE
    };
    const char* type_names[] = {
        "INT8", 
        "INT16", 
        "INT32", 
        "FLOAT", 
        "DOUBLE"
    };
    const int num_types = sizeof(possible_types) / sizeof(possible_types[0]);
    
    for (int i = 0; i < MAX_CLASSES - 1; i++) {
        // Generate random period
        uint32_t period = random_range(MIN_PERIOD, MAX_PERIOD);
        
        // Generate random deadline factor
        float factor = MIN_DEADLINE_FACTOR + 
                      ((MAX_DEADLINE_FACTOR - MIN_DEADLINE_FACTOR) * 
                       (esp_random() % 100) / 100.0f);
        
        uint32_t deadline = (uint32_t)(period * factor);
        
        // Generate random data type
        int rand_type_index = esp_random() % num_types;
        data_type_t data_type = possible_types[rand_type_index];
        
        // Generate random packet count
        uint16_t packet_count = random_range(MIN_PACKET_COUNT, MAX_PACKET_COUNT);
        
        // Update values
        config->class_periods[i] = period;
        config->class_deadlines[i] = deadline;
        config->class_types[i] = data_type;
        config->packet_counts[i] = packet_count;
        
        printf("Class %d: Type=%s, Period=%lu ms, Deadline=%lu ms (%.1f%% of period), Count=%u\n", 
               i + 1, type_names[rand_type_index], period, deadline, factor * 100, packet_count);
    }
    
    // Also generate a random threshold
    uint32_t threshold = random_range(MIN_THRESHOLD, MAX_THRESHOLD);
    config->processing_threshold = threshold;
    printf("Processing threshold: %lu ms\n", threshold);
    
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
    {"type", "Set data type for a class", cmd_type},
    {"count", "Set packet count for a class", cmd_packet_count},
    {"threshold", "Set processing threshold", cmd_threshold},
    {"reset", "Reset all classes to default values", cmd_reset},
    {"random", "Set random periods and deadlines for all classes", cmd_random},
    {"start", "Start program with current configuration", cmd_start},
    {"rpacket", "Configure random packet generation", cmd_random_packet},
    {"rtype", "Set random packet data type", cmd_random_packet_type},
    {"rsize", "Set random packet size", cmd_random_packet_count},
    {"rburst", "Configure random packet burst parameters", cmd_random_packet_burst},
    {"rdeadline", "Set random packet deadline", cmd_random_packet_deadline},
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
    config->class_periods[CLASS_RANDOM] = 0;  // Not periodic, controlled by random mechanism
    
    // Set default deadlines equal to periods
    config->class_deadlines[CLASS_1] = DEFAULT_CLASS1_PERIOD;
    config->class_deadlines[CLASS_2] = DEFAULT_CLASS2_PERIOD;
    config->class_deadlines[CLASS_3] = DEFAULT_CLASS3_PERIOD;
    config->class_deadlines[CLASS_RANDOM] = 2000;  // Default deadline for random packets: 2s
    
    // Set default data types
    config->class_types[CLASS_1] = DATA_TYPE_INT32;  // Class 1 - INT32
    config->class_types[CLASS_2] = DATA_TYPE_FLOAT;  // Class 2 - FLOAT
    config->class_types[CLASS_3] = DATA_TYPE_INT16;  // Class 3 - INT16
    config->class_types[CLASS_RANDOM] = DATA_TYPE_INT32;  // Random - INT32
    
    // Set default packet counts
    config->packet_counts[CLASS_1] = DEFAULT_CLASS1_COUNT;
    config->packet_counts[CLASS_2] = DEFAULT_CLASS2_COUNT;
    config->packet_counts[CLASS_3] = DEFAULT_CLASS3_COUNT;
    config->packet_counts[CLASS_RANDOM] = 0;  // Not applicable for random packets
    
    // Set default processing threshold
    config->processing_threshold = DEFAULT_PROCESSING_THRESHOLD;

    // Initialize random packet parameters
    config->random_packet_enabled = false;  // Disabled by default
    config->random_packet_min_interval = DEFAULT_RANDOM_PACKET_MIN_INTERVAL;
    config->random_packet_max_interval = DEFAULT_RANDOM_PACKET_MAX_INTERVAL;
    config->random_packet_burst_period = DEFAULT_RANDOM_PACKET_BURST_PERIOD;
    config->random_packet_burst_interval = DEFAULT_RANDOM_PACKET_BURST_INTERVAL;
    config->random_packet_count = DEFAULT_RANDOM_PACKET_COUNT;
    config->random_packet_type = DEFAULT_RANDOM_PACKET_TYPE;
    config->random_packet_burst_enabled = DEFAULT_RANDOM_PACKET_BURST_ENABLED;
    
    // Display current configuration
    cmd_status(0, NULL, config);
    
    // Main configuration loopA
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