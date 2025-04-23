/**
 * @file packet_generator.c
 * @brief Implementation of test packet generation functions
 */

#include <string.h>
#include <stdlib.h>
#include "esp_log.h"
#include "packet_generator.h"

static const char *TAG = "packet-gen";

/* External function declaration for scheduler_submit_packet */
extern esp_err_t scheduler_submit_packet(class_id_t class_id, const void *data, uint16_t count);

/**
 * Create a test packet based on specified data type
 */
esp_err_t create_test_packet(class_id_t class_id, uint16_t count, data_type_t data_type)
{
    esp_err_t ret = ESP_OK;
    
    switch (data_type) {
        case DATA_TYPE_INT8:
            ret = create_test_int8_packet(class_id, count);
            break;
        case DATA_TYPE_INT16:
            ret = create_test_int16_packet(class_id, count);
            break;
        case DATA_TYPE_INT32:
            ret = create_test_int32_packet(class_id, count);
            break;
        case DATA_TYPE_FLOAT:
            ret = create_test_float_packet(class_id, count);
            break;
        case DATA_TYPE_DOUBLE:
            ret = create_test_double_packet(class_id, count);
            break;
        default:
            ESP_LOGW(TAG, "Unknown data type %d, using INT32 as fallback", data_type);
            ret = create_test_int32_packet(class_id, count);
            break;
    }
    
    return ret;
}

/**
 * Create a test INT8 packet
 */
esp_err_t create_test_int8_packet(class_id_t class_id, uint16_t count)
{
    // Create array of int8 values
    int8_t *values = malloc(count * sizeof(int8_t));
    if (values == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for INT8 test data");
        return ESP_FAIL;
    }
    
    // Fill with sequential values
    for (int i = 0; i < count; i++) {
        values[i] = i % 256;  // Keep values in the int8 range
    }
    
    // Submit packet with this data
    esp_err_t ret = scheduler_submit_packet(class_id, values, count);
    
    // Free the temporary buffer
    free(values);
    
    return ret;
}

/**
 * Create a test INT16 packet
 */
esp_err_t create_test_int16_packet(class_id_t class_id, uint16_t count)
{
    // Create array of int16 values
    int16_t *values = malloc(count * sizeof(int16_t));
    if (values == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for INT16 test data");
        return ESP_FAIL;
    }
    
    // Fill with sequential values
    for (int i = 0; i < count; i++) {
        values[i] = i * 10;
    }
    
    // Submit packet with this data
    esp_err_t ret = scheduler_submit_packet(class_id, values, count);
    
    // Free the temporary buffer
    free(values);
    
    return ret;
}

/**
 * Create a test INT32 packet
 */
esp_err_t create_test_int32_packet(class_id_t class_id, uint16_t count)
{
    // Create array of int32 values
    int32_t *values = malloc(count * sizeof(int32_t));
    if (values == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for INT32 test data");
        return ESP_FAIL;
    }
    
    // Fill with sequential values
    for (int i = 0; i < count; i++) {
        values[i] = i;
    }
    
    // Submit packet with this data
    esp_err_t ret = scheduler_submit_packet(class_id, values, count);
    
    // Free the temporary buffer
    free(values);
    
    return ret;
}

/**
 * Create a test FLOAT packet
 */
esp_err_t create_test_float_packet(class_id_t class_id, uint16_t count)
{
    // Create array of float values
    float *values = malloc(count * sizeof(float));
    if (values == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for FLOAT test data");
        return ESP_FAIL;
    }
    
    // Fill with values
    for (int i = 0; i < count; i++) {
        values[i] = i * 0.1f;
    }
    
    // Submit packet with this data
    esp_err_t ret = scheduler_submit_packet(class_id, values, count);
    
    // Free the temporary buffer
    free(values);
    
    return ret;
}

/**
 * Create a test DOUBLE packet
 */
esp_err_t create_test_double_packet(class_id_t class_id, uint16_t count)
{
    // Create array of double values
    double *values = malloc(count * sizeof(double));
    if (values == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for DOUBLE test data");
        return ESP_FAIL;
    }
    
    // Fill with values
    for (int i = 0; i < count; i++) {
        values[i] = i * 0.01;
    }
    
    // Submit packet with this data
    esp_err_t ret = scheduler_submit_packet(class_id, values, count);
    
    // Free the temporary buffer
    free(values);
    
    return ret;
}