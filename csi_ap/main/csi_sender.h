#ifndef CSI_SENDER_H
#define CSI_SENDER_H

#include <stdint.h>
#include <stdbool.h>

/* CSI Configuration */
#define CONFIG_LESS_INTERFERENCE_CHANNEL    11   // WiFi channel with less interference
#define CONFIG_SEND_FREQUENCY              100   // CSI send frequency in Hz
#define CONFIG_CSI_SEND_MAC               {0x1a, 0x00, 0x00, 0x00, 0x00, 0x00}  // ESP-NOW sender MAC

/**
 * @brief Initializes ESP-NOW for CSI sending
 * 
 * Sets up ESP-NOW and related configurations
 */
void csi_sender_init(void);

/**
 * @brief Starts sending periodic CSI packets
 * 
 * Starts a task that periodically sends ESP-NOW packets 
 * which can be used for CSI measurements by receivers
 */
void csi_sender_start(void);

/**
 * @brief Returns total number of sent CSI packets
 * 
 * @return uint32_t Number of sent packets
 */
uint32_t csi_sender_get_count(void);

#endif /* CSI_SENDER_H */