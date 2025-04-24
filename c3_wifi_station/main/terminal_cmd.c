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
static void cmd_adjust_tx_power_by_rssi(scheduler_config_t *config);
static int cmd_auto_tx_power(int argc, char **argv, scheduler_config_t *config); // adaptive tx power

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

/* Command to set WiFi TX power */
static int cmd_wifi_tx_power(int argc, char **argv, scheduler_config_t *config)
{
    ESP_LOGI(TAG, "Setting WiFi TX power");
    
    int8_t current_power = 0;
    esp_wifi_get_max_tx_power(&current_power);
    
    if (argc < 2) {
        printf("Usage: txpower <value>\n");
        printf("       Value range: 8-84 (2dBm-20dBm, in 0.25dBm units)\n");
        printf("       Use '-a' for auto value\n");
        printf("Current TX power: %d\n", current_power);
        return 1;
    }
    
    int8_t power = config->wifi_tx_power;
    
    if (strcmp(argv[1], "-a") == 0) {
        // Auto-generate power setting
        power = 80;  // 20dBm, maximum value
        printf("Auto-generated TX power: %d\n", power);
    } else {
        power = atoi(argv[1]);
        if (power < 8 || power > 84) {
            printf("Warning: TX power outside valid range [8-84]. Clamping.\n");
            power = (power < 8) ? 8 : 84;
        }
    }
    
    config->wifi_tx_power = power;
    
    // Apply setting immediately if WiFi is started
    esp_err_t ret = esp_wifi_set_max_tx_power(power);
    if (ret == ESP_OK) {
        printf("TX power set to %d (applied immediately)\n", power);
    } else {
        printf("TX power will be set to %d when WiFi starts\n", power);
    }
    
    return 0;
}

/* Command to set WiFi power save mode */
static int cmd_wifi_ps_mode(int argc, char **argv, scheduler_config_t *config)
{
    ESP_LOGI(TAG, "Setting WiFi power save mode");
    
    wifi_ps_type_t current_mode;
    esp_wifi_get_ps(&current_mode);
    
    if (argc < 2) {
        printf("Usage: psmode <mode>\n");
        printf("Available modes:\n");
        printf("  none   - No power save (WIFI_PS_NONE)\n");
        printf("  min    - Minimum power save (WIFI_PS_MIN_MODEM)\n");
        printf("  max    - Maximum power save (WIFI_PS_MAX_MODEM)\n");
        
        // Show current mode
        const char *mode_str;
        switch (current_mode) {
            case WIFI_PS_NONE: mode_str = "NONE"; break;
            case WIFI_PS_MIN_MODEM: mode_str = "MIN_MODEM"; break;
            case WIFI_PS_MAX_MODEM: mode_str = "MAX_MODEM"; break;
            default: mode_str = "UNKNOWN"; break;
        }
        printf("Current power save mode: %s\n", mode_str);
        return 1;
    }
    
    wifi_ps_type_t mode = config->wifi_ps_mode;
    
    if (strcasecmp(argv[1], "none") == 0) {
        mode = WIFI_PS_NONE;
    } else if (strcasecmp(argv[1], "min") == 0) {
        mode = WIFI_PS_MIN_MODEM;
    } else if (strcasecmp(argv[1], "max") == 0) {
        mode = WIFI_PS_MAX_MODEM;
    } else {
        printf("Error: Invalid power save mode '%s'.\n", argv[1]);
        printf("Available modes: none, min, max\n");
        return 1;
    }
    
    config->wifi_ps_mode = mode;
    
    // Apply setting immediately if WiFi is started
    esp_err_t ret = esp_wifi_set_ps(mode);
    if (ret == ESP_OK) {
        printf("Power save mode set to %s (applied immediately)\n", argv[1]);
    } else {
        printf("Power save mode will be set to %s when WiFi starts\n", argv[1]);
    }
    
    return 0;
}

/* Command to set WiFi protocol */
static int cmd_wifi_protocol(int argc, char **argv, scheduler_config_t *config)
{
    ESP_LOGI(TAG, "Setting WiFi protocol");
    
    uint8_t current_protocol;
    esp_wifi_get_protocol(WIFI_IF_STA, &current_protocol);
    
    if (argc < 2) {
        printf("Usage: protocol <mode>\n");
        printf("Available modes:\n");
        printf("  b      - 802.11b only\n");
        printf("  bg     - 802.11b/g\n");
        printf("  g      - 802.11g only (no 11b rates)\n");
        printf("  bgn    - 802.11b/g/n (default)\n");
        printf("  gn     - 802.11g/n (no 11b rates)\n");
        
        // Show current protocol
        printf("Current protocol: ");
        if (current_protocol & WIFI_PROTOCOL_11B) printf("802.11b ");
        if (current_protocol & WIFI_PROTOCOL_11G) printf("802.11g ");
        if (current_protocol & WIFI_PROTOCOL_11N) printf("802.11n ");
        printf("\n");
        
        // Show configured protocol
        printf("Configured protocol: ");
        if (config->wifi_protocol & WIFI_PROTOCOL_11B) printf("802.11b ");
        if (config->wifi_protocol & WIFI_PROTOCOL_11G) printf("802.11g ");
        if (config->wifi_protocol & WIFI_PROTOCOL_11N) printf("802.11n ");
        printf("\n");
        
        return 1;
    }
    
    uint8_t protocol = 0;
    bool disable_11b_rates = false;
    
    if (strcasecmp(argv[1], "b") == 0) {
        // 802.11b only
        protocol = WIFI_PROTOCOL_11B;
        disable_11b_rates = false;
    } else if (strcasecmp(argv[1], "bg") == 0) {
        // 802.11bg mode
        protocol = WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G;
        disable_11b_rates = false;
    } else if (strcasecmp(argv[1], "g") == 0) {
        // 802.11g only mode
        protocol = WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G;
        disable_11b_rates = true;
    } else if (strcasecmp(argv[1], "bgn") == 0) {
        // 802.11bgn mode
        protocol = WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N;
        disable_11b_rates = false;
    } else if (strcasecmp(argv[1], "gn") == 0) {
        // 802.11gn mode
        protocol = WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N;
        disable_11b_rates = true;
    } else {
        printf("Error: Invalid protocol mode '%s'.\n", argv[1]);
        printf("Available modes: b, bg, g, bgn, gn\n");
        return 1;
    }
    
    // Update the configuration
    config->wifi_protocol = protocol;
    config->disable_11b_rates = disable_11b_rates;  // You'll need to add this field to your config struct
    
    printf("WiFi protocol will be set to %s when WiFi starts\n", argv[1]);
    printf("Configured protocol: ");
    if (config->wifi_protocol & WIFI_PROTOCOL_11B) printf("802.11b ");
    if (config->wifi_protocol & WIFI_PROTOCOL_11G) printf("802.11g ");
    if (config->wifi_protocol & WIFI_PROTOCOL_11N) printf("802.11n ");
    printf("\n");
    
    if (disable_11b_rates) {
        printf("11b rates will be disabled (pure G mode)\n");
    }
    
    return 0;
}

/* Adjust TX power based on RSSI */
static void cmd_adjust_tx_power_by_rssi(scheduler_config_t *config)
{
    // Get current RSSI
    wifi_ap_record_t ap_info;
    esp_err_t err = esp_wifi_sta_get_ap_info(&ap_info);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get AP info for TX power adjustment (error %d: %s)", 
                 err, esp_err_to_name(err));
        return;
    }
    
    int8_t rssi = ap_info.rssi;
    ESP_LOGW(TAG, "Current RSSI: %d dBm", rssi);
    
    int8_t new_tx_power = config->wifi_tx_power; // Start with current setting
    
    // Determine appropriate TX power based on RSSI
    if (rssi >= RSSI_EXCELLENT) {
        // Excellent signal, use lowest power
        new_tx_power = TX_POWER_MIN;
    } else if (rssi >= RSSI_GOOD) {
        // Good signal, use low power
        new_tx_power = TX_POWER_LOW;
    } else if (rssi >= RSSI_FAIR) {
        // Fair signal, use medium power
        new_tx_power = TX_POWER_MEDIUM;
    } else {
        // Poor signal, use high power
        new_tx_power = TX_POWER_HIGH;
    }
    
    // Only change if different from current setting
    if (new_tx_power != config->wifi_tx_power) {
        ESP_LOGW(TAG, "Adjusting TX power based on RSSI %d dBm: %d -> %d", 
                 rssi, config->wifi_tx_power, new_tx_power);
        
        // Update config and apply setting
        config->wifi_tx_power = new_tx_power;
        esp_err_t ret = esp_wifi_set_max_tx_power(new_tx_power);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set TX power: %s", esp_err_to_name(ret));
        }
    }
}

/* Command to set auto TX power check interval */
static int cmd_auto_tx_interval(int argc, char **argv, scheduler_config_t *config)
{
    ESP_LOGI(TAG, "Setting auto TX power check interval");
    
    if (argc < 2) {
        printf("Usage: autotx_interval <value_ms>\n");
        printf("       Use '-a' for auto value\n");
        printf("Current interval: %lu ms\n", config->auto_tx_power_interval);
        return 1;
    }
    
    uint32_t interval = config->auto_tx_power_interval;
    
    if (strcmp(argv[1], "-a") == 0) {
        // Auto-generate interval between 1000-10000ms (1-10s)
        interval = random_range(1000, 10000);
        printf("Auto-generated interval: %lu ms\n", interval);
    } else {
        interval = atoi(argv[1]);
        if (interval < 500 || interval > 30000) {
            printf("Warning: Interval outside recommended range [500-30000]. Clamping.\n");
            interval = (interval < 500) ? 500 : 30000;
        }
    }
    
    config->auto_tx_power_interval = interval;
    printf("Auto TX power check interval set to %lu ms\n", interval);
    
    return 0;
}

/* Command to configure auto TX power adjustment */
static int cmd_auto_tx_power(int argc, char **argv, scheduler_config_t *config)
{
    ESP_LOGI(TAG, "Configuring auto TX power adjustment");
    
    if (argc < 2) {
        printf("Usage: autotx [on|off]\n");
        printf("       Use 'on' to enable, 'off' to disable automatic TX power adjustment\n");
        printf("Current status: %s\n", config->auto_tx_power ? "ENABLED" : "DISABLED");
        return 1;
    }
    
    // Parse enable/disable
    if (strcasecmp(argv[1], "on") == 0) {
        config->auto_tx_power = true;
        printf("Auto TX power adjustment enabled\n");
        
        // Check if WiFi is connected before trying to adjust TX power
        wifi_ap_record_t ap_info;
        if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
            // Immediately adjust TX power based on current RSSI
            cmd_adjust_tx_power_by_rssi(config);
        } else {
            printf("Note: WiFi not connected yet. TX power will be adjusted once connected.\n");
        }
    } else if (strcasecmp(argv[1], "off") == 0) {
        config->auto_tx_power = false;
        printf("Auto TX power adjustment disabled\n");
    } else {
        printf("Error: First argument must be 'on' or 'off'\n");
        return 1;
    }
    
    return 0;
}

static int cmd_verify_wifi(int argc, char **argv, scheduler_config_t *config)
{
    ESP_LOGI(TAG, "Verifying WiFi settings");
    esp_err_t ret = verify_wifi_settings(config);
    
    if (ret == ESP_OK) {
        printf("All WiFi settings were successfully verified.\n");
    } else if (ret == ESP_FAIL) {
        printf("Some WiFi settings don't match the configuration.\n");
        printf("Check the logs for details.\n");
    } else {
        printf("Error during verification: %s\n", esp_err_to_name(ret));
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

    printf("\nWiFi configuration commands:\n");
    printf("  %-10s - Set WiFi transmit power (8-84)\n", "txpower");
    printf("  %-10s - Set WiFi power save mode (none/min/max)\n", "psmode");
    printf("  %-10s - Set WiFi protocol (b/bg/bgn)\n", "protocol");
    printf("  %-10s - Enable/disable auto TX power adjustment\n", "autotx");
    printf("  %-10s - Set auto TX power check interval\n", "autotx_interval");
    printf("  Example: txpower 80     - Set TX power to 20dBm (maximum)\n");
    printf("  Example: psmode min     - Use minimum power save\n");
    printf("  Example: protocol bgn   - Use 802.11b/g/n protocols\n");
    printf("  Example: autotx on      - Enable automatic TX power adjustment\n");
    printf("  Example: autotx_interval 3000  - Check and adjust every 3 seconds\n");
    
    
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
    
    // Add WiFi configuration information
    printf("\nWiFi Configuration:\n");
    
    // Display configured TX power (from config)
    printf("  TX power setting: %d (0.25dBm units)\n", config->wifi_tx_power);
    
    // Get and display current TX power (from hardware)
    int8_t wifi_tx_power = 0;
    if(esp_wifi_get_max_tx_power(&wifi_tx_power) == ESP_OK) {
        printf("  Current TX power: %d\n", wifi_tx_power);
    }

    // Display configured power save mode (from config)
    const char *ps_mode_str;
    switch (config->wifi_ps_mode) {
        case WIFI_PS_NONE: ps_mode_str = "NONE"; break;
        case WIFI_PS_MIN_MODEM: ps_mode_str = "MIN_MODEM"; break;
        case WIFI_PS_MAX_MODEM: ps_mode_str = "MAX_MODEM"; break;
        default: ps_mode_str = "UNKNOWN"; break;
    }
    printf("  Power save mode setting: %s\n", ps_mode_str);
    
    // Get and display current power save mode (from hardware)
    wifi_ps_type_t ps_mode;
    if(esp_wifi_get_ps(&ps_mode) == ESP_OK) {
        const char *current_ps_str;
        switch (ps_mode) {
            case WIFI_PS_NONE: current_ps_str = "NONE"; break;
            case WIFI_PS_MIN_MODEM: current_ps_str = "MIN_MODEM"; break;
            case WIFI_PS_MAX_MODEM: current_ps_str = "MAX_MODEM"; break;
            default: current_ps_str = "UNKNOWN"; break;
        }
        printf("  Current power save mode: %s\n", current_ps_str);
    }

    // Display configured protocol (from config)
    printf("  Protocol setting: ");
    if (config->wifi_protocol & WIFI_PROTOCOL_11B) printf("802.11b ");
    if (config->wifi_protocol & WIFI_PROTOCOL_11G) printf("802.11g ");
    if (config->wifi_protocol & WIFI_PROTOCOL_11N) printf("802.11n ");
    printf("\n");
    
    // Get and display current protocol (from hardware)
    uint8_t getprotocol = 0;
    printf("  Current protocol: ");
    if (esp_wifi_get_protocol(WIFI_IF_STA, &getprotocol) == ESP_OK) {
        if (getprotocol & WIFI_PROTOCOL_11B) printf("802.11b ");
        if (getprotocol & WIFI_PROTOCOL_11G) printf("802.11g ");
        if (getprotocol & WIFI_PROTOCOL_11N) printf("802.11n ");
        printf("\n");
    } else {
        printf("Unknown (could not get protocol)\n");
    }

    // Display connection information
    wifi_ap_record_t ap_info;
    if(esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
        printf("  Connected to AP: %s\n", ap_info.ssid);
        printf("  AP RSSI: %d dBm\n", ap_info.rssi);
        printf("  AP Channel: %d\n", ap_info.primary);
    } else {
        printf("  Not connected to an AP\n");
    }

    // Display auto TX power adjustment information
    printf("  Auto TX power adjustment: %s\n", config->auto_tx_power ? "ENABLED" : "DISABLED");
    printf("  Auto TX power check interval: %lu ms\n", config->auto_tx_power_interval);
    
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

    // Reset WiFi parameters
    config->wifi_tx_power = DEFAULT_WIFI_TX_POWER;
    config->wifi_ps_mode = DEFAULT_WIFI_PS_MODE;
    config->wifi_protocol = DEFAULT_WIFI_PROTOCOL;
    
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
    {"txpower", "Set WiFi transmit power", cmd_wifi_tx_power},
    {"psmode", "Set WiFi power save mode", cmd_wifi_ps_mode},
    {"protocol", "Set WiFi protocol", cmd_wifi_protocol},
    {"autotx", "Configure automatic TX power adjustment", cmd_auto_tx_power},
    {"autotx_interval", "Set auto TX power check interval", cmd_auto_tx_interval},
    {"verify_wifi", "Verify current WiFi settings against configuration", cmd_verify_wifi},
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

    // Initialize WiFi parameters
    config->wifi_tx_power = DEFAULT_WIFI_TX_POWER;
    config->wifi_ps_mode = DEFAULT_WIFI_PS_MODE;
    config->wifi_protocol = DEFAULT_WIFI_PROTOCOL;
    config->disable_11b_rates = false;

    // Initialize auto TX power adjustment
    config->auto_tx_power = false;
    config->auto_tx_power_interval = 5000; // Default: check every 5 seconds
    
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

/**
 * @brief Get current WiFi settings and verify they match configuration
 * 
 * @param config Pointer to scheduler configuration structure
 * @return esp_err_t ESP_OK if all settings match, ESP_FAIL if any mismatch
 */
esp_err_t verify_wifi_settings(scheduler_config_t *config)
{
    ESP_LOGI(TAG, "Verifying WiFi settings against configuration...");
    
    bool all_match = true;
    
    // Check TX power
    int8_t current_tx_power = 0;
    esp_err_t ret = esp_wifi_get_max_tx_power(&current_tx_power);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get TX power: %s", esp_err_to_name(ret));
        return ret;
    }
    
    if (current_tx_power != config->wifi_tx_power) {
        ESP_LOGW(TAG, "TX power mismatch! Config: %d, Actual: %d", 
                config->wifi_tx_power, current_tx_power);
        all_match = false;
    } else {
        ESP_LOGI(TAG, "TX power verified: %d", current_tx_power);
    }
    
    // Check power save mode
    wifi_ps_type_t current_ps_mode;
    ret = esp_wifi_get_ps(&current_ps_mode);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get power save mode: %s", esp_err_to_name(ret));
        return ret;
    }
    
    if (current_ps_mode != config->wifi_ps_mode) {
        const char *config_mode_str, *actual_mode_str;
        
        switch (config->wifi_ps_mode) {
            case WIFI_PS_NONE: config_mode_str = "NONE"; break;
            case WIFI_PS_MIN_MODEM: config_mode_str = "MIN_MODEM"; break;
            case WIFI_PS_MAX_MODEM: config_mode_str = "MAX_MODEM"; break;
            default: config_mode_str = "UNKNOWN"; break;
        }
        
        switch (current_ps_mode) {
            case WIFI_PS_NONE: actual_mode_str = "NONE"; break;
            case WIFI_PS_MIN_MODEM: actual_mode_str = "MIN_MODEM"; break;
            case WIFI_PS_MAX_MODEM: actual_mode_str = "MAX_MODEM"; break;
            default: actual_mode_str = "UNKNOWN"; break;
        }
        
        ESP_LOGW(TAG, "Power save mode mismatch! Config: %s, Actual: %s", 
                config_mode_str, actual_mode_str);
        all_match = false;
    } else {
        const char *mode_str;
        switch (current_ps_mode) {
            case WIFI_PS_NONE: mode_str = "NONE"; break;
            case WIFI_PS_MIN_MODEM: mode_str = "MIN_MODEM"; break;
            case WIFI_PS_MAX_MODEM: mode_str = "MAX_MODEM"; break;
            default: mode_str = "UNKNOWN"; break;
        }
        ESP_LOGI(TAG, "Power save mode verified: %s", mode_str);
    }
    
    // Check protocol
    uint8_t current_protocol;
    ret = esp_wifi_get_protocol(WIFI_IF_STA, &current_protocol);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get WiFi protocol: %s", esp_err_to_name(ret));
        return ret;
    }
    
    if (current_protocol != config->wifi_protocol) {
        ESP_LOGW(TAG, "WiFi protocol mismatch! Config: 0x%02x, Actual: 0x%02x", 
                config->wifi_protocol, current_protocol);
        
        ESP_LOGW(TAG, "Config protocols: %s%s%s", 
                (config->wifi_protocol & WIFI_PROTOCOL_11B) ? "802.11b " : "",
                (config->wifi_protocol & WIFI_PROTOCOL_11G) ? "802.11g " : "",
                (config->wifi_protocol & WIFI_PROTOCOL_11N) ? "802.11n " : "");
        
        ESP_LOGW(TAG, "Actual protocols: %s%s%s", 
                (current_protocol & WIFI_PROTOCOL_11B) ? "802.11b " : "",
                (current_protocol & WIFI_PROTOCOL_11G) ? "802.11g " : "",
                (current_protocol & WIFI_PROTOCOL_11N) ? "802.11n " : "");
        
        all_match = false;
    } else {
        ESP_LOGI(TAG, "WiFi protocol verified: %s%s%s", 
                (current_protocol & WIFI_PROTOCOL_11B) ? "802.11b " : "",
                (current_protocol & WIFI_PROTOCOL_11G) ? "802.11g " : "",
                (current_protocol & WIFI_PROTOCOL_11N) ? "802.11n " : "");
    }
    
    // Check if we're connected to the configured SSID
    wifi_ap_record_t ap_info;
    ret = esp_wifi_sta_get_ap_info(&ap_info);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Currently connected to AP: %s", ap_info.ssid);
        ESP_LOGI(TAG, "RSSI: %d dBm", ap_info.rssi);
        ESP_LOGI(TAG, "Channel: %d", ap_info.primary);
        
        // If auto TX power is enabled, check if current TX power makes sense for RSSI
        if (config->auto_tx_power) {
            ESP_LOGI(TAG, "Auto TX power is enabled, checking if TX power matches RSSI...");
            
            int8_t expected_tx_power = 0;
            if (ap_info.rssi >= RSSI_EXCELLENT) {
                expected_tx_power = TX_POWER_MIN;
            } else if (ap_info.rssi >= RSSI_GOOD) {
                expected_tx_power = TX_POWER_LOW;
            } else if (ap_info.rssi >= RSSI_FAIR) {
                expected_tx_power = TX_POWER_MEDIUM;
            } else {
                expected_tx_power = TX_POWER_HIGH;
            }
            
            if (current_tx_power != expected_tx_power) {
                ESP_LOGW(TAG, "TX power doesn't match expected value for RSSI %d dBm", ap_info.rssi);
                ESP_LOGW(TAG, "Current: %d, Expected: %d", current_tx_power, expected_tx_power);
            } else {
                ESP_LOGI(TAG, "TX power correctly set for current RSSI");
            }
        }
    } else {
        ESP_LOGW(TAG, "Not connected to an AP, couldn't verify SSID");
    }
    
    if (all_match) {
        ESP_LOGI(TAG, "All WiFi settings match configuration!");
        return ESP_OK;
    } else {
        ESP_LOGW(TAG, "Some WiFi settings don't match configuration.");
        return ESP_FAIL;
    }
}