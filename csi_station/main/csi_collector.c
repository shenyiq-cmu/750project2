#include <string.h>
#include <stdio.h>
#include <inttypes.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_err.h"

#include "csi_collector.h"

static const char *CSI_TAG = "wifi csi";

/* MAC addresses of interest */
static const uint8_t ap_mac[6] = AP_MAC_ADDR;
static const uint8_t espnow_mac[6] = ESPNOW_MAC_ADDR;

/* CSI data structure for storing filtered data */
typedef struct {
    uint8_t mac[6];
    int8_t rssi;
    uint32_t timestamp;
    uint8_t channel;
    uint8_t secondary_channel;
    uint8_t sig_mode;        // 0:non HT(11bg), 1:HT(11n), 3:VHT(11ac)
    uint8_t mcs;             // Modulation Coding Scheme
    uint8_t bandwidth;       // Bandwidth, 0:20MHz, 1:40MHz
    uint8_t smoothing;       // 1:smoothing, 0:not smoothing
    uint8_t not_sounding;    // 0:sounding, 1:not sounding
    uint8_t aggregation;     // 1:aggregation, 0:non aggregation
    uint8_t stbc;            // 1:STBC, 0:non STBC
    uint8_t fec_coding;      // 1:LDPC, 0:BCC
    uint8_t sgi;             // 1:Short GI, 0:Long GI
    int8_t noise_floor;      // Noise floor in dBm
    uint8_t ampdu_cnt;       // AMPDU count
    uint8_t rate;            // Rate, value is: 0 ~ 11
    uint8_t ant;             // Antenna number
    uint16_t len;
    bool is_ap;              // Flag if this is from our AP
    bool is_espnow;          // Flag if this is from ESP-NOW
    int8_t *buf;             // Pointer to CSI data buffer
} csi_entry_t;

// Circular buffer to store most recent CSI entries
#define MAX_CSI_ENTRIES 100
static csi_entry_t csi_entries[MAX_CSI_ENTRIES];
static int csi_entry_count = 0;
static int csi_entry_index = 0;
static uint32_t last_display_time = 0;
static uint32_t total_csi_count = 0;

// Counters for AP-specific data
static uint32_t ap_csi_count = 0;
static uint32_t espnow_csi_count = 0;
static int8_t last_ap_rssi = 0;
static int8_t last_espnow_rssi = 0;

/* Function to print MAC address */
static void print_mac(const uint8_t *mac, char *mac_str)
{
    sprintf(mac_str, "%02X:%02X:%02X:%02X:%02X:%02X", 
            mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

/* Function to check if a MAC matches our AP or ESP-NOW MAC */
static bool is_ap_mac(const uint8_t *mac)
{
    return memcmp(mac, ap_mac, 6) == 0;
}

static bool is_espnow_mac(const uint8_t *mac)
{
    return memcmp(mac, espnow_mac, 6) == 0;
}

/* Get string representation of signal mode */
static const char* get_sig_mode_str(uint8_t sig_mode)
{
    switch(sig_mode) {
        case 0: return "11bg";  // Non-HT
        case 1: return "11n";   // HT
        case 3: return "11ac";  // VHT
        default: return "Unknown";
    }
}

/* Get string representation of bandwidth */
static const char* get_bandwidth_str(uint8_t bandwidth)
{
    switch(bandwidth) {
        case 0: return "20MHz";
        case 1: return "40MHz";
        default: return "Unknown";
    }
}

/* Function to print CSI data in compact CSV format */
static void print_csi_csv_header()
{
    printf("Source,MAC,RSSI,Channel,SecChan,SigMode,BW,Rate,MCS,SGI,STBC,FEC,NF,AGG,CSI_Len\n");
}

static void print_csi_csv(csi_entry_t *entry)
{
    char mac_str[18];
    print_mac(entry->mac, mac_str);
    
    const char *source = "OTHER";
    if (entry->is_ap) {
        source = "AP";
    } else if (entry->is_espnow) {
        source = "ESP-NOW";
    }
    
    printf("%s,%s,%d,%d,%d,%s,%s,%d,%d,%d,%d,%s,%d,%d,%d\n",
           source,
           mac_str,
           entry->rssi,
           entry->channel,
           entry->secondary_channel,
           get_sig_mode_str(entry->sig_mode),
           get_bandwidth_str(entry->bandwidth),
           entry->rate,
           entry->mcs,
           entry->sgi,
           entry->stbc,
           entry->fec_coding ? "LDPC" : "BCC",
           entry->noise_floor,
           entry->aggregation,
           entry->len);
}

/* Function to display CSI detailed information */
static void display_csi_details(csi_entry_t *entry, const char *source_type)
{
    char mac_str[18];
    print_mac(entry->mac, mac_str);
    
    ESP_LOGI(CSI_TAG, "------- %s CSI Details -------", source_type);
    ESP_LOGI(CSI_TAG, "MAC: %s", mac_str);
    ESP_LOGI(CSI_TAG, "Signal | RSSI: %d dBm | Channel: %d (Secondary: %d) | Noise Floor: %d dBm",
             entry->rssi, entry->channel, entry->secondary_channel, entry->noise_floor);
             
    ESP_LOGI(CSI_TAG, "PHY    | Mode: %s | BW: %s | Rate: %d | MCS: %d | Antenna: %d",
             get_sig_mode_str(entry->sig_mode), get_bandwidth_str(entry->bandwidth),
             entry->rate, entry->mcs, entry->ant);
             
    ESP_LOGI(CSI_TAG, "Frame  | STBC: %s | FEC: %s | GI: %s | AGG: %s | Smooth: %s | Sound: %s",
             entry->stbc ? "Yes" : "No",
             entry->fec_coding ? "LDPC" : "BCC",
             entry->sgi ? "Short" : "Long",
             entry->aggregation ? "Yes" : "No",
             entry->smoothing ? "Yes" : "No",
             entry->not_sounding ? "No" : "Yes");
             
    // Print first few CSI values
    const int display_count = 10;
    ESP_LOGI(CSI_TAG, "CSI Data Length: %d bytes, First %d values: [%d", 
             entry->len, (display_count < entry->len) ? display_count : entry->len, 
             entry->len > 0 ? entry->buf[0] : 0);
             
    for (int i = 1; i < display_count && i < entry->len; i++) {
        printf(", %d", entry->buf[i]);
    }
    
    if (entry->len > display_count) {
        printf(", ...]\n");
    } else {
        printf("]\n");
    }
}

/* Function to find most recent entry for a specific MAC */
static csi_entry_t* find_latest_entry(const uint8_t *target_mac)
{
    for (int i = 0; i < csi_entry_count; i++) {
        int idx = (csi_entry_index - i - 1 + MAX_CSI_ENTRIES) % MAX_CSI_ENTRIES;
        if (memcmp(csi_entries[idx].mac, target_mac, 6) == 0) {
            return &csi_entries[idx];
        }
    }
    return NULL;
}

/* Function to store CSI entry in circular buffer */
static void store_csi_entry(const wifi_csi_info_t *info)
{
    const wifi_pkt_rx_ctrl_t *rx_ctrl = &info->rx_ctrl;
    
    // Check if this is from our AP or ESP-NOW sender
    bool from_ap = is_ap_mac(info->mac);
    bool from_espnow = is_espnow_mac(info->mac);
    
    // Update counters for AP and ESP-NOW packets
    if (from_ap) {
        ap_csi_count++;
        last_ap_rssi = rx_ctrl->rssi;
    } else if (from_espnow) {
        espnow_csi_count++;
        last_espnow_rssi = rx_ctrl->rssi;
    }
    
    // Free previous buffer if it exists
    if (csi_entries[csi_entry_index].buf != NULL) {
        free(csi_entries[csi_entry_index].buf);
        csi_entries[csi_entry_index].buf = NULL;
    }
    
    // Store entry in circular buffer
    memcpy(csi_entries[csi_entry_index].mac, info->mac, 6);
    csi_entries[csi_entry_index].rssi = rx_ctrl->rssi;
    csi_entries[csi_entry_index].timestamp = rx_ctrl->timestamp;
    csi_entries[csi_entry_index].channel = rx_ctrl->channel;
    csi_entries[csi_entry_index].secondary_channel = rx_ctrl->secondary_channel;
    csi_entries[csi_entry_index].sig_mode = rx_ctrl->sig_mode;
    csi_entries[csi_entry_index].mcs = rx_ctrl->mcs;
    csi_entries[csi_entry_index].bandwidth = rx_ctrl->cwb;
    csi_entries[csi_entry_index].smoothing = rx_ctrl->smoothing;
    csi_entries[csi_entry_index].not_sounding = rx_ctrl->not_sounding;
    csi_entries[csi_entry_index].aggregation = rx_ctrl->aggregation;
    csi_entries[csi_entry_index].stbc = rx_ctrl->stbc;
    csi_entries[csi_entry_index].fec_coding = rx_ctrl->fec_coding;
    csi_entries[csi_entry_index].sgi = rx_ctrl->sgi;
    csi_entries[csi_entry_index].noise_floor = rx_ctrl->noise_floor;
    csi_entries[csi_entry_index].ampdu_cnt = rx_ctrl->ampdu_cnt;
    csi_entries[csi_entry_index].rate = rx_ctrl->rate;
    csi_entries[csi_entry_index].ant = rx_ctrl->ant;
    csi_entries[csi_entry_index].len = info->len;
    csi_entries[csi_entry_index].is_ap = from_ap;
    csi_entries[csi_entry_index].is_espnow = from_espnow;
    
    // Allocate and copy CSI data buffer
    csi_entries[csi_entry_index].buf = (int8_t *)malloc(info->len);
    if (csi_entries[csi_entry_index].buf != NULL) {
        memcpy(csi_entries[csi_entry_index].buf, info->buf, info->len);
    } else {
        ESP_LOGE(CSI_TAG, "Failed to allocate memory for CSI data");
        csi_entries[csi_entry_index].len = 0;
    }
    
    // Update counters
    if (csi_entry_count < MAX_CSI_ENTRIES) {
        csi_entry_count++;
    }
    
    csi_entry_index = (csi_entry_index + 1) % MAX_CSI_ENTRIES;
    total_csi_count++;
    
    // Check if it's time to display statistics
    uint32_t current_time = esp_log_timestamp();
    if (current_time - last_display_time >= CSI_DISPLAY_INTERVAL_MS) {
        last_display_time = current_time;
        csi_print_statistics();
    }
    
    // Special notification for AP or ESP-NOW packets (always show these)
    if (from_ap || from_espnow) {
        char mac_str[18];
        print_mac(info->mac, mac_str);
        
        const char *source = from_ap ? "AP" : "ESP-NOW";
        
        ESP_LOGI(CSI_TAG, "*** %s CSI: #%"PRIu32" | MAC: %s | RSSI: %d | CH: %d | BW: %s | Mode: %s ***", 
                source,
                from_ap ? ap_csi_count : espnow_csi_count,
                mac_str,
                rx_ctrl->rssi, 
                rx_ctrl->channel,
                get_bandwidth_str(rx_ctrl->cwb),
                get_sig_mode_str(rx_ctrl->sig_mode));
    }
}

/* CSI callback function to process received CSI data */
static void wifi_csi_rx_cb(void *ctx, wifi_csi_info_t *info)
{
    if (!info || !info->buf) {
        ESP_LOGW(CSI_TAG, "<%s> wifi_csi_cb", esp_err_to_name(ESP_ERR_INVALID_ARG));
        return;
    }

    const wifi_pkt_rx_ctrl_t *rx_ctrl = &info->rx_ctrl;
    
    // Filter out weak signals
    if (rx_ctrl->rssi < CSI_RSSI_THRESHOLD) {
        return;
    }
    
    // For logging every 100th packet (that's not from our AP or ESP-NOW)
    if (!is_ap_mac(info->mac) && !is_espnow_mac(info->mac) && (total_csi_count % 100 == 0)) {
        char mac_str[18];
        print_mac(info->mac, mac_str);
        
        ESP_LOGI(CSI_TAG, "CSI #%"PRIu32" | MAC: %s | RSSI: %d | CH: %d | BW: %s | Mode: %s", 
                total_csi_count,
                mac_str,
                rx_ctrl->rssi, 
                rx_ctrl->channel,
                get_bandwidth_str(rx_ctrl->cwb),
                get_sig_mode_str(rx_ctrl->sig_mode));
    }
    
    // Store the CSI entry for analysis
    store_csi_entry(info);
}

/* Public API Implementations */

void csi_init(void)
{
    // Initialize CSI entry array
    memset(csi_entries, 0, sizeof(csi_entries));
    csi_entry_count = 0;
    csi_entry_index = 0;
    total_csi_count = 0;
    ap_csi_count = 0;
    espnow_csi_count = 0;
    
    // Enable promiscuous mode to receive all packets
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));
    
    // Set CSI configuration
    wifi_csi_config_t csi_config = {
        .lltf_en           = true,    // Enable Legacy Long Training Field
        .htltf_en          = true,    // Enable HT Long Training Field
        .stbc_htltf2_en    = true,    // Enable Space-Time Block Coding HT Long Training Field
        .ltf_merge_en      = true,    // Enable LTF merge
        .channel_filter_en = true,    // Enable channel filter
        .manu_scale        = false,   // Disable manual scaling
        .shift             = false,   // No shift
    };
    
    ESP_ERROR_CHECK(esp_wifi_set_csi_config(&csi_config));
    ESP_ERROR_CHECK(esp_wifi_set_csi_rx_cb(wifi_csi_rx_cb, NULL));
    ESP_ERROR_CHECK(esp_wifi_set_csi(true));
    
    // Print AP and ESP-NOW MAC addresses we're tracking
    char ap_mac_str[18];
    print_mac(ap_mac, ap_mac_str);
    
    char espnow_mac_str[18];
    print_mac(espnow_mac, espnow_mac_str);
    
    ESP_LOGI(CSI_TAG, "============ CSI CONFIG ============");
    ESP_LOGI(CSI_TAG, "CSI collection initialized");
    ESP_LOGI(CSI_TAG, "RSSI threshold: %d dBm", CSI_RSSI_THRESHOLD);
    ESP_LOGI(CSI_TAG, "Output mode: %s", OUTPUT_COMPACT_MODE ? "Compact CSV" : "Detailed");
    ESP_LOGI(CSI_TAG, "Statistics interval: %d ms", CSI_DISPLAY_INTERVAL_MS);
    ESP_LOGI(CSI_TAG, "Buffer size: %d entries", MAX_CSI_ENTRIES);
    ESP_LOGI(CSI_TAG, "Tracking AP MAC: %s", ap_mac_str);
    ESP_LOGI(CSI_TAG, "Tracking ESP-NOW MAC: %s", espnow_mac_str);
    ESP_LOGI(CSI_TAG, "CSI Config: Legacy LTF, HT LTF, STBC HT-LTF2, LTF merge, Channel filter");
    ESP_LOGI(CSI_TAG, "===================================");
    
    if (OUTPUT_COMPACT_MODE) {
        ESP_LOGI(CSI_TAG, "CSV column legend:");
        ESP_LOGI(CSI_TAG, "Source: AP/ESP-NOW/OTHER | MAC: Device MAC | RSSI: Signal strength | Channel: WiFi channel");
        ESP_LOGI(CSI_TAG, "SecChan: Secondary channel | SigMode: 11bg/11n/11ac | BW: Bandwidth");
        ESP_LOGI(CSI_TAG, "Rate: Transmission rate | MCS: Modulation & coding scheme | SGI: Short guard interval");
        ESP_LOGI(CSI_TAG, "STBC: Space-time block coding | FEC: Forward error correction | NF: Noise floor");
        ESP_LOGI(CSI_TAG, "AGG: Aggregation | CSI_Len: CSI data length");
    }
}

uint32_t csi_get_total_count(void)
{
    return total_csi_count;
}

uint32_t csi_get_ap_count(void)
{
    return ap_csi_count;
}

uint32_t csi_get_espnow_count(void)
{
    return espnow_csi_count;
}

int8_t csi_get_ap_rssi(void)
{
    return last_ap_rssi;
}

int8_t csi_get_espnow_rssi(void)
{
    return last_espnow_rssi;
}

void csi_print_statistics(void)
{
    if (csi_entry_count == 0) {
        ESP_LOGI(CSI_TAG, "No CSI data collected yet");
        return;
    }
    
    // Count unique MAC addresses
    uint8_t unique_macs[MAX_CSI_ENTRIES][6];
    int unique_mac_count = 0;
    int mac_counts[MAX_CSI_ENTRIES] = {0};
    int rssi_sums[MAX_CSI_ENTRIES] = {0};
    
    for (int i = 0; i < csi_entry_count; i++) {
        int entry_idx = (csi_entry_index + MAX_CSI_ENTRIES - csi_entry_count + i) % MAX_CSI_ENTRIES;
        csi_entry_t *entry = &csi_entries[entry_idx];
        
        bool found = false;
        for (int j = 0; j < unique_mac_count; j++) {
            if (memcmp(entry->mac, unique_macs[j], 6) == 0) {
                mac_counts[j]++;
                rssi_sums[j] += entry->rssi;
                found = true;
                break;
            }
        }
        
        if (!found && unique_mac_count < MAX_CSI_ENTRIES) {
            memcpy(unique_macs[unique_mac_count], entry->mac, 6);
            mac_counts[unique_mac_count] = 1;
            rssi_sums[unique_mac_count] = entry->rssi;
            unique_mac_count++;
        }
    }
    
    // Print statistics
    ESP_LOGI(CSI_TAG, "======== CSI STATISTICS ========");
    ESP_LOGI(CSI_TAG, "Total CSI packets: %"PRIu32" (From AP: %"PRIu32", ESP-NOW: %"PRIu32")", 
             total_csi_count, ap_csi_count, espnow_csi_count);
    ESP_LOGI(CSI_TAG, "Unique devices: %d", unique_mac_count);
    
    // Highlight AP and ESP-NOW details
    ESP_LOGI(CSI_TAG, "====== TARGET DEVICE INFO ======");
    
    char ap_mac_str[18];
    print_mac(ap_mac, ap_mac_str);
    ESP_LOGI(CSI_TAG, "AP MAC: %s, Packets: %"PRIu32", Last RSSI: %d", 
             ap_mac_str, ap_csi_count, last_ap_rssi);
             
    char espnow_mac_str[18];
    print_mac(espnow_mac, espnow_mac_str);
    ESP_LOGI(CSI_TAG, "ESP-NOW MAC: %s, Packets: %"PRIu32", Last RSSI: %d", 
             espnow_mac_str, espnow_csi_count, last_espnow_rssi);
             
    // Display most recent AP and ESP-NOW CSI details
    csi_entry_t *latest_ap = find_latest_entry(ap_mac);
    csi_entry_t *latest_espnow = find_latest_entry(espnow_mac);
    
    if (latest_ap) {
        display_csi_details(latest_ap, "AP");
    } else {
        ESP_LOGI(CSI_TAG, "No CSI data from AP yet");
    }
    
    if (latest_espnow) {
        display_csi_details(latest_espnow, "ESP-NOW");
    } else {
        ESP_LOGI(CSI_TAG, "No CSI data from ESP-NOW yet");
    }
    
    ESP_LOGI(CSI_TAG, "------------------------------------");
    
    // Print general device info
    if (OUTPUT_COMPACT_MODE) {
        print_csi_csv_header();
    }
    
    // Print per-device info and the most recent entry for each device
    for (int i = 0; i < unique_mac_count; i++) {
        char mac_str[18];
        print_mac(unique_macs[i], mac_str);
        
        float avg_rssi = (float)rssi_sums[i] / mac_counts[i];
        
        if (!OUTPUT_COMPACT_MODE) {
            ESP_LOGI(CSI_TAG, "Device: %s | Count: %d | Avg RSSI: %.1f", 
                    mac_str, mac_counts[i], avg_rssi);
        }
        
        // Find most recent entry for this MAC
        for (int j = 0; j < csi_entry_count; j++) {
            int entry_idx = (csi_entry_index + MAX_CSI_ENTRIES - j - 1) % MAX_CSI_ENTRIES;
            csi_entry_t *entry = &csi_entries[entry_idx];
            
            if (memcmp(entry->mac, unique_macs[i], 6) == 0) {
                if (OUTPUT_COMPACT_MODE) {
                    print_csi_csv(entry);
                }
                break;
            }
        }
    }
    
    ESP_LOGI(CSI_TAG, "===============================");
}