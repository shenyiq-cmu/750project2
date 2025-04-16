#ifndef CSI_COLLECTOR_H
#define CSI_COLLECTOR_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_wifi_types.h"

/* CSI Configuration */
#define CSI_RSSI_THRESHOLD         -85       // Only process CSI data with RSSI above this threshold
#define CSI_DISPLAY_INTERVAL_MS    10000     // Display summary every 10 seconds
#define OUTPUT_COMPACT_MODE        1         // Set to 1 for compact output, 0 for detailed

/* AP MAC Address Configuration - MAKE SURE THESE MATCH YOUR ACTUAL MAC ADDRESSES */
// Make sure these values exactly match the MAC shown in your AP logs
#define AP_MAC_ADDR                {0x48, 0x31, 0xb7, 0x01, 0x9d, 0x49}  // Your AP's MAC address
#define ESPNOW_MAC_ADDR            {0x1a, 0x00, 0x00, 0x00, 0x00, 0x00}  // ESP-NOW sender MAC

/* Debug Flags */
#define DEBUG_MAC_COMPARISON       1         // Set to 1 to print MAC comparison debug info

/**
 * @brief Initializes the CSI collection subsystem
 * 
 * Sets up CSI callbacks, configuration, and data structures
 */
void csi_init(void);

/**
 * @brief Returns total number of CSI packets collected
 * 
 * @return uint32_t Number of CSI packets
 */
uint32_t csi_get_total_count(void);

/**
 * @brief Returns number of CSI packets from the AP
 * 
 * @return uint32_t Number of AP CSI packets
 */
uint32_t csi_get_ap_count(void);

/**
 * @brief Returns number of CSI packets from ESP-NOW
 * 
 * @return uint32_t Number of ESP-NOW CSI packets
 */
uint32_t csi_get_espnow_count(void);

/**
 * @brief Gets latest RSSI value from AP
 * 
 * @return int8_t RSSI value in dBm
 */
int8_t csi_get_ap_rssi(void);

/**
 * @brief Gets latest RSSI value from ESP-NOW
 * 
 * @return int8_t RSSI value in dBm
 */
int8_t csi_get_espnow_rssi(void);

/**
 * @brief Prints CSI statistics to console
 * 
 * Displays summary statistics and detailed information about
 * collected CSI data, with special focus on AP and ESP-NOW packets.
 */
void csi_print_statistics(void);

#endif /* CSI_COLLECTOR_H */