/**
 * @file packet_generators.h
 * @brief Test packet generation functions for different data types
 */

#ifndef PACKET_GENERATORS_H
#define PACKET_GENERATORS_H

#include "esp_system.h"
#include "esp_log.h"
#include "terminal_cmd.h"  // For class_id_t and data_type_t

/**
 * @brief Create a test packet of a specified data type
 * 
 * @param class_id Class identifier
 * @param count Number of data elements to create
 * @param data_type Type of data to create
 * @return esp_err_t ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t create_test_packet(class_id_t class_id, uint16_t count, data_type_t data_type);

/**
 * @brief Create a test packet of INT8 data
 * 
 * @param class_id Class identifier
 * @param count Number of data elements
 * @return esp_err_t ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t create_test_int8_packet(class_id_t class_id, uint16_t count);

/**
 * @brief Create a test packet of INT16 data
 * 
 * @param class_id Class identifier
 * @param count Number of data elements
 * @return esp_err_t ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t create_test_int16_packet(class_id_t class_id, uint16_t count);

/**
 * @brief Create a test packet of INT32 data
 * 
 * @param class_id Class identifier
 * @param count Number of data elements
 * @return esp_err_t ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t create_test_int32_packet(class_id_t class_id, uint16_t count);

/**
 * @brief Create a test packet of FLOAT data
 * 
 * @param class_id Class identifier
 * @param count Number of data elements
 * @return esp_err_t ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t create_test_float_packet(class_id_t class_id, uint16_t count);

/**
 * @brief Create a test packet of DOUBLE data
 * 
 * @param class_id Class identifier
 * @param count Number of data elements
 * @return esp_err_t ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t create_test_double_packet(class_id_t class_id, uint16_t count);

#endif /* PACKET_GENERATORS_H */