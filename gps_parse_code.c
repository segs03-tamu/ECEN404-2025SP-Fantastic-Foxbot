#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include <math.h>
#include "driver/gpio.h"
#include "driver/adc.h"
#include "driver/dac.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <unistd.h>

#include "driver/uart.h"
#include "mavlink.h"

// Constants
#define UART_NUM        UART_NUM_1
#define TX_PIN          17
#define RX_PIN          16
#define BAUD_RATE       57600      // Adjust to match PixHawk's baud rate
#define MAV_SYSTEM_ID   1          // PixHawk's system ID
#define ESP_SYSTEM_ID   255        // ESP32's system ID
#define MAV_COMP_ID     0          // Autopilot component ID
#define TAG             "MAVLINK_ESP32"

// Function prototypes
void init_uart();
void request_gps_stream();
void process_mavlink_message(mavlink_message_t *msg);
void receive_mavlink_task(void *pvParameters);

// UART Initialization
void init_uart() {
    uart_config_t uart_config = {
        .baud_rate = BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT
    };
    // Increase UART buffer sizes to prevent overflow
    uart_driver_install(UART_NUM, 2048, 2048, 0, NULL, 0);
    uart_param_config(UART_NUM, &uart_config);
    uart_set_pin(UART_NUM, TX_PIN, RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    ESP_LOGI(TAG, "UART initialized");
}

// Request GPS stream using MAV_CMD_SET_MESSAGE_INTERVAL
void request_gps_stream() {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    mavlink_msg_command_long_pack(
        ESP_SYSTEM_ID, MAV_COMP_ID, &msg,
        MAV_SYSTEM_ID, MAV_COMP_ID,
        MAV_CMD_SET_MESSAGE_INTERVAL,
        0, // confirmation
        MAVLINK_MSG_ID_GLOBAL_POSITION_INT, // message ID
        100000, // 100 ms interval (10Hz)
        0, 0, 0, 0, 0 // unused params
    );

    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    int bytes_sent = uart_write_bytes(UART_NUM, buf, len);
    if (bytes_sent > 0) {
        ESP_LOGI(TAG, "Sent GPS data request (%d bytes)", bytes_sent);
    } else {
        ESP_LOGE(TAG, "Failed to send GPS request");
    }
}

// Process parsed MAVLink messages
void process_mavlink_message(mavlink_message_t *msg) {
    ESP_LOGI(TAG, "Received message ID: %d", msg->msgid);
    switch (msg->msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT:
            ESP_LOGI(TAG, "Received HEARTBEAT");
            break;
        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
            mavlink_global_position_int_t gps;
            mavlink_msg_global_position_int_decode(msg, &gps);
            float lat = gps.lat / 1e7;
            float lon = gps.lon / 1e7;
            float alt = gps.alt / 1000.0;
            ESP_LOGI("GPS", "Lat: %.7f, Lon: %.7f, Alt: %.2f m", lat, lon, alt);
            break;
        }
        default:
            break;
    }
}

// Task to receive and parse MAVLink messages
void receive_mavlink_task(void *pvParameters) {
    uint8_t data[512];  // Increased buffer size to 512 bytes
    mavlink_message_t msg;
    mavlink_status_t status;
    static int good_messages = 0;  // Counter for successfully parsed messages
    static int bad_crcs = 0;       // Counter for bad CRC errors

    while (1) {
        // Read UART data with a 100ms timeout
        int len = uart_read_bytes(UART_NUM, data, sizeof(data), 100 / portTICK_PERIOD_MS);
        if (len > 0) {
            ESP_LOGI("MAVLINK_ESP32", "Received %d bytes", len);
            // Log first 16 bytes for inspection
            printf("First 16 bytes: ");
            for (int i = 0; i < len && i < 16; i++) {
                printf("0x%02X ", data[i]);
            }
            printf("\n");

            // Parse each byte with mavlink_parse_char
            for (int i = 0; i < len; i++) {
                if (data[i] == 0xFD) {
                    ESP_LOGI("MAVLINK_ESP32", "Received start byte 0xFD at index %d", i);
                }
                if (mavlink_parse_char(MAVLINK_COMM_0, data[i], &msg, &status)) {
                    good_messages++;  // Increment good message counter
                    ESP_LOGI("MAVLINK_ESP32", "Parsed message ID: %d", msg.msgid);
                    // Add your message processing function here
                    process_mavlink_message(&msg);
                } else {
                    // Log parser state for debugging
                    switch (status.parse_state) {
                        case MAVLINK_PARSE_STATE_GOT_BAD_CRC1:
                            bad_crcs++;  // Increment bad CRC counter
                            ESP_LOGW("MAVLINK_ESP32", "Parser: Bad CRC");
                            break;
                        default:
                            break;  // Other states can be logged if needed
                    }
                }
            }
            // Log message statistics after processing the buffer
            ESP_LOGI("MAVLINK_ESP32", "Good messages: %d, Bad CRCs: %d", good_messages, bad_crcs);
        } else {
            ESP_LOGW("MAVLINK_ESP32", "No data received");
        }
        vTaskDelay(pdMS_TO_TICKS(100)); // 100ms delay between reads
    }
}

// Main application entry point
void app_main() {
    init_uart();
    xTaskCreate(receive_mavlink_task, "mavlink_rx", 4096, NULL, 5, NULL);

    // Send GPS stream request multiple times
    for (int i = 0; i < 5; i++) {
        request_gps_stream();
        vTaskDelay(pdMS_TO_TICKS(500)); // 500ms delay between requests
    }

    ESP_LOGI(TAG, "Application initialized");
}











