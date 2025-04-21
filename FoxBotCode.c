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
#include <math.h>

#include "driver/uart.h"
#include "mavlink.h"


#include <inttypes.h>
#include <errno.h>
#include <sys/fcntl.h>
#include <sys/unistd.h>
#include <lwip/sockets.h>

#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_timer.h"





#define SAMPLE_PERIOD_MS		200
#define I2C_SDA_BM1             21  
#define I2C_SCL_IO              22  
#define I2C_FREQ_HZ             100000  
#define I2C_SDA_BM2             19
#define I2C_SDA_BM3             18

#define BMWakeup                4                /*Raising Edge to boot chip*/


#define I2C_PORT_NUM			I2C_NUM_1        /*!< I2C port number for master dev */
#define I2C_TX_BUF_DISABLE  	0                /*!< I2C master do not need buffer */
#define I2C_RX_BUF_DISABLE  	0                /*!< I2C master do not need buffer */

// I2C common protocol defines
#define WRITE_BIT                          I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT                           I2C_MASTER_READ  /*!< I2C master read */
#define ACK_CHECK_EN                       0x1              /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS                      0x0              /*!< I2C master will not check ack from slave */
#define ACK_VAL                            0x0              /*!< I2C ack value */
#define NACK_VAL                           0x1              /*!< I2C nack value */

// BQ7692003 Registers
#define BQ7692003PWR_I2C_ADDR		0x08
#define SYS_STAT        0x00
#define SYS_CTRL1       0x04
#define SYS_CTRL2       0x05
#define PROTECT1        0x06
#define PROTECT2        0x07
#define PROTECT3        0x08
#define VC1_HI          0x0C
#define VC1_LO          0x0D
#define VC2_HI          0x0E
#define VC2_LO          0x0F
#define VC3_HI          0x10
#define VC3_LO          0x11
#define VC4_HI          0x12
#define VC4_LO          0x13
#define VC5_HI          0x14
#define VC5_LO          0x15

#define CC_CFG          0x0B

#define CELLBAL1        0x01 


#define UV_TRIP         0x0A


#define BAT_HI          0x2A
#define BAT_LO          0x2B
#define CC_HI           0x32
#define CC_LO           0x33

#define ADCGAIN1        0x50
#define ADCOFFSET       0x51
#define ADCGAIN2        0x59



// Constants for the morse code bit
#define DAC_PIN DAC_CHANNEL_1   // GPIO25 (DAC1)
#define LED_PIN GPIO_NUM_33      // GPIO33
#define SINE_RESOLUTION 100     // Number of points in one sine wave cycle

#define FREQUENCY 2000      // Maximum frequency (Hz)

// Morse code timing (in milliseconds)
#define DOT_DURATION 100
#define DASH_DURATION 300
#define PART_SPACE_DURATION 100
#define LETTER_SPACE_DURATION 300
#define WORD_SPACE_DURATION 700

// Push to Talk gpio pin
#define PTT GPIO_NUM_32

// Bit 0-4 of DTMF decoder
#define Bit0 GPIO_NUM_13
#define Bit1 GPIO_NUM_14
#define Bit2 GPIO_NUM_27
#define Bit3 GPIO_NUM_26

// Number of pins used
#define NUM_PINS 4
const gpio_num_t pins[NUM_PINS] = {Bit0, Bit1, Bit2, Bit3};  

#define UART_PORT_NUM      UART_NUM_0  // Use UART0
#define TXD_PIN            GPIO_NUM_1  // GPIO pin for TX (default for UART0)
#define RXD_PIN            GPIO_NUM_3  // GPIO pin for RX (default for UART0)
#define BUF_SIZE           1024        // Buffer size for UART
#define BAUD_RATE          115200        // Lower baud rate

#define UART_NUM        UART_NUM_1
#define TX_PIN_PIX          17
#define RX_PIN_PIX          16
#define BAUD_RATE_PIX       57600      // Adjust to match PixHawk's baud rate
#define MAV_SYSTEM_ID   1          // PixHawk's system ID
#define ESP_SYSTEM_ID   255        // ESP32's system ID
#define MAV_COMP_ID     0          // Autopilot component ID
#define TAG_PIX             "MAVLINK_ESP32"

// Ultrasonic Sensors gpio pins
#define TRIGGER_PIN_1 23           // GPIO pin for Trigger 1
#define ECHO_PIN_1 39              // GPIO pin for Echo 1
#define ECHO_PIN_2 34              // GPIO pin for Echo 1

#define PI 3.14159265358979323846


// Function prototypes
void init_uart_pix();
void request_gps_stream();
void process_mavlink_message(mavlink_message_t *msg);
void receive_mavlink_task();

// UART Initialization
void init_uart_pix() {
    uart_config_t uart_config = {
        .baud_rate = BAUD_RATE_PIX,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT
    };
    // Increase UART buffer sizes to prevent overflow
    uart_driver_install(UART_NUM, 2048, 2048, 0, NULL, 0);
    uart_param_config(UART_NUM, &uart_config);
    uart_set_pin(UART_NUM, TX_PIN_PIX, RX_PIN_PIX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    ESP_LOGI(TAG_PIX, "UART initialized");
}



static const char *TAG = "i2c_restart";

// Function Responsible for reading given register
static esp_err_t i2c_master_read_slave_reg(i2c_port_t i2c_num, uint8_t i2c_addr, uint8_t i2c_reg, uint8_t* data_rd, size_t size)
{
    if (size == 0) {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    // first, send device address (indicating write) & register to be read
    i2c_master_write_byte(cmd, ( i2c_addr << 1 ), ACK_CHECK_EN);
    // send register we want
    i2c_master_write_byte(cmd, i2c_reg, ACK_CHECK_EN);
    // Send repeated start
    i2c_master_start(cmd);
    // now send device address (indicating read) & read data
    i2c_master_write_byte(cmd, ( i2c_addr << 1 ) | READ_BIT, ACK_CHECK_EN);
    if (size > 1) {
        i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

// Function responsible for writing to register
static esp_err_t i2c_master_write_slave_reg(i2c_port_t i2c_num, uint8_t i2c_addr, uint8_t i2c_reg, uint8_t* data_wr, size_t size) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);

    // Log the write operation details
    ESP_LOGI(TAG, "Writing to I2C device: 0x%02X", i2c_addr);
    ESP_LOGI(TAG, "Register address: 0x%02X", i2c_reg);
    ESP_LOGI(TAG, "Data to write: 0x%02X", *data_wr);

    // Send device address + write bit
    i2c_master_write_byte(cmd, (i2c_addr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);

    // Send register address
    i2c_master_write_byte(cmd, i2c_reg, ACK_CHECK_EN);

    // Write the data
    i2c_master_write(cmd, data_wr, size, ACK_CHECK_EN);

    i2c_master_stop(cmd);

    // Execute the I2C command
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C write failed: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "I2C write successful");
    }

    i2c_cmd_link_delete(cmd);
    return ret;
}

// Returns value of the register 
esp_err_t rdAFE( uint8_t reg, uint8_t *pdata, uint8_t count )
{
	return( i2c_master_read_slave_reg(I2C_PORT_NUM, BQ7692003PWR_I2C_ADDR,  reg, pdata, count ) );
}

// Write to register
esp_err_t wrAFE( uint8_t reg, uint8_t *pdata, uint8_t count )
{
	return( i2c_master_write_slave_reg( I2C_PORT_NUM, BQ7692003PWR_I2C_ADDR,  reg, pdata, count ) );
}

// Function which turns on the battery monitors with the correct timing
void wake_up_bq76920() {
    gpio_set_direction(BMWakeup, GPIO_MODE_OUTPUT);

    // Ensure TS1 is LOW initially
    gpio_set_level(BMWakeup, 0);
    vTaskDelay(pdMS_TO_TICKS(50));  // Wait 50ms

    // Create a rising edge by setting TS1 HIGH
    gpio_set_level(BMWakeup, 1);
    vTaskDelay(pdMS_TO_TICKS(50));  // Wait 50ms

    ESP_LOGI("BQ76920", "Wake-up pulse sent to TS1");
}


// Read register of battery monitor 
uint8_t read_register(uint8_t reg) {
    uint8_t reg_value = 0xFF;  // Default value in case of read failure
    esp_err_t err;

    // Step 1: Read the current value of the register
    err = rdAFE(reg, &reg_value, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read register 0x%02X: %s", reg, esp_err_to_name(err));
    }
    /*else {
        // Log the register address and its value in hex format
        ESP_LOGI(TAG, "Read register 0x%02X: 0x%02X", reg, reg_value);
    }*/
    return reg_value;
    vTaskDelay(pdMS_TO_TICKS(500));
}

// Function to read the cell voltage
float read_cell_voltage(uint8_t vc_hi_addr, uint8_t vc_low_addr) {
    // Read the values of VC_HI and VC_LOW registers
    uint8_t vc_hi = read_register(vc_hi_addr);
    uint8_t vc_low = read_register(vc_low_addr);
    //printf("This is the HI: 0x%02X, and this is the Lo: 0x%02X \n", vc_hi, vc_low);

    // Combine VC_HI and VC_LOW into a 16-bit value
    uint16_t combined_value = (vc_hi << 8) | vc_low;

    // Mask out bits <15:14> to use only bits <13:0>
    combined_value = combined_value & 0x3FFF; // 0x3FFF = 0b0011111111111111

    // Multiply the combined value by 375 microvolts (375e-6 volts)
    float voltage = combined_value * 375e-6;

    return voltage;
}

// Battery monitior testing
static void afe_init()
{ 

    printf("This is sys_STAT:  0x%02X \n", read_register(SYS_STAT));   
    
    printf("Cell 1 Voltage: %f V\n", read_cell_voltage(VC1_HI, VC1_LO));
    printf("Cell 2 Voltage: %f V\n", read_cell_voltage(VC2_HI, VC2_LO));
    printf("Cell 3 Voltage: %f V\n", read_cell_voltage(VC3_HI, VC3_LO));
    printf("Cell 4 Voltage: %f V\n", read_cell_voltage(VC4_HI, VC4_LO));
    printf("Cell 5 Voltage: %f V\n", read_cell_voltage(VC5_HI, VC5_LO));
    
}

// Initialization of I2C Communication for Battery monitors all share the same SCL line
void i2c_init() {
    i2c_config_t conf = {0};  
    conf.mode = I2C_MODE_MASTER;
    conf.scl_io_num = I2C_SCL_IO;   // Shared SCL
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_FREQ_HZ;
    
    // Initial SDA (default to first line)
    conf.sda_io_num = I2C_SDA_BM1;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;

    esp_err_t err = i2c_param_config(I2C_PORT_NUM, &conf);
    if (err != ESP_OK) {
        ESP_LOGE("I2C", "i2c_param_config failed: %s", esp_err_to_name(err));
        return;
    }

    err = i2c_driver_install(I2C_PORT_NUM, I2C_MODE_MASTER, 0, 0, 0);
    if (err != ESP_OK) {
        ESP_LOGE("I2C", "i2c_driver_install failed: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI("I2C", "I2C initialized successfully!");
    }
}

// Function to switch SDA line dynamically
void i2c_set_sda(gpio_num_t sda_pin) {
    i2c_driver_delete(I2C_PORT_NUM);  // Delete the current I2C driver

    i2c_config_t conf = {0};  
    conf.mode = I2C_MODE_MASTER;
    conf.scl_io_num = I2C_SCL_IO;  // Shared SCL line
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.sda_io_num = sda_pin;  // Change SDA dynamically
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_FREQ_HZ;

    esp_err_t err = i2c_param_config(I2C_PORT_NUM, &conf);
    if (err == ESP_OK) {
        i2c_driver_install(I2C_PORT_NUM, I2C_MODE_MASTER, 0, 0, 0);
    }
}


// Scanner to determine if battery monitor is observable to ESP
void i2c_scanner() {
    ESP_LOGI(TAG, "Scanning I2C bus...");
    for (uint8_t addr = 0x01; addr < 0x7F; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | WRITE_BIT, ACK_CHECK_EN);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(I2C_PORT_NUM, cmd, 1000 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Found device at address: 0x%02X", addr);
        }
    }
    ESP_LOGI(TAG, "Scan complete.");
}


// Sine wave lookup table
uint8_t sineTable[SINE_RESOLUTION];


// Morse code lookup table
const char *morseCode[37] = {
    ".-", "-...", "-.-.", "-..", ".", "..-.", "--.", "....", "..", ".---", "-.-", ".-..", "--", "-.", "---", ".--.", "--.-", ".-.", "...", "-", "..-", "...-", ".--", "-..-", "-.--", "--..", // A-Z
    "-----", ".----", "..---", "...--", "....-", ".....", "-....", "--...", "---..", "----.", // 0-9
    ".-.-.-" // .
};


// Generate sine wave lookup table
void generateSineTable() {
    // For loop for generating the sinewave
    for (int i = 0; i < SINE_RESOLUTION; i++) {
        sineTable[i] = (uint8_t)(127.5 + 127.5 * sin(2 * M_PI * i / SINE_RESOLUTION));
    }
}

// Initialize DAC
void dac_init() {
    dac_output_enable(DAC_PIN); // Enable DAC channel
}

// Output a value to the DAC
void dac_output(uint8_t value) {
    dac_output_voltage(DAC_PIN, value); // Set DAC output voltage (0-255)
}

// Initialize LED to show Morse code dots and dashes
void led_init() {
    gpio_reset_pin(LED_PIN); // Reset the LED pin
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT); // Set LED pin as output
}

// Initialize push to talk pin
void ptt_init() {
    gpio_reset_pin(PTT); // Reset the LED pin
    gpio_set_direction(PTT, GPIO_MODE_OUTPUT); // Set LED pin as output
}

// Play a Morse code symbol (dot or dash)
void playMorseSymbol(const char symbol, int frequency) {
    int duration = (symbol == '.') ? DOT_DURATION : DASH_DURATION;
    int delayTime = 1000000 / (frequency * SINE_RESOLUTION); // Microseconds per step

    // Turn on LED and play sine wave for the duration of the symbol
    gpio_set_level(LED_PIN, 1); // Set LED pin high
    for (int t = 0; t < duration; t += delayTime) {
        for (int i = 0; i < SINE_RESOLUTION; i++) {
            dac_output(sineTable[i]); // Output sine wave
            vTaskDelay(pdMS_TO_TICKS(delayTime / 1000)); // Delay for frequency control
        }
    }
    gpio_set_level(LED_PIN, 0); // Set LED pin low

    // Silence for the space between parts of the same letter
    vTaskDelay(pdMS_TO_TICKS(PART_SPACE_DURATION));
}

// Play a Morse code letter
void playMorseLetter(const char *code, char letter, int frequency) {
    // Print the letter and its Morse code
    printf("Letter: %c, Morse Code: %s\n", letter, code);

    while (*code) {
        //printf("%c ", *code); // Print each Morse symbol (dot or dash)
        playMorseSymbol(*code, frequency);
        code++;
    }
    
}

// Play a Morse code string
void playMorseString(const char *text, int frequency) {
    //gpio_set_level(PTT, 1);
    vTaskDelay(pdMS_TO_TICKS(1000)); 
    while (*text) {
        if (*text == ' ') {
            // Space between words
            printf("  (Word Space)\n");
            vTaskDelay(pdMS_TO_TICKS(WORD_SPACE_DURATION));
        } else {
            // Get Morse code for the character
            int index = (*text >= 'A' && *text <= 'Z') ? *text - 'A' :
                        (*text >= 'a' && *text <= 'z') ? *text - 'a' :
                        (*text >= '0' && *text <= '9') ? *text - '0' + 26:
                        (*text == '.') ? 36 :
                        -1;
            if (index >= 0) {
                playMorseLetter(morseCode[index], *text, frequency);
                // Space between letters
                vTaskDelay(pdMS_TO_TICKS(LETTER_SPACE_DURATION));
            }
        }
        text++;
    }

    vTaskDelay(pdMS_TO_TICKS(1000)); 
    //gpio_set_level(PTT, 0);
    dac_output_voltage(DAC_PIN, 0);
        
}

// Initialize DTMF bit pins
void bit_init() {
    // Configure the GPIO pins as inputs
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;  // Disable interrupt
    io_conf.mode = GPIO_MODE_INPUT;         // Set as input mode
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;  // Disable pull-down
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;      // Disable pull-up

    // Configure each pin separately
    io_conf.pin_bit_mask = (1ULL << Bit0);
    gpio_config(&io_conf);
    
    io_conf.pin_bit_mask = (1ULL << Bit2);
    gpio_config(&io_conf);
    
    io_conf.pin_bit_mask = (1ULL << Bit3);
    gpio_config(&io_conf);
    

    io_conf.pin_bit_mask = (1ULL << Bit1);
    gpio_config(&io_conf);
    
}

//Define variable for 
TaskHandle_t xTaskBitMonitor = NULL; // Declare a variable to store the task handle

// Function to initialize UART for MAVLink
void init_uart() {
    // Configure UART parameters
    uart_config_t uart_config = {
        .baud_rate = BAUD_RATE,       
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    // Install UART driver
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, BUF_SIZE, BUF_SIZE, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

// Intialize ultrasonic gpio pins
void init_ultrasonic(void) {
	int gpio_pad_select_gpio();
    gpio_pad_select_gpio(TRIGGER_PIN_1); // initializing gpio pin 18 - trigger pin of hcsr04
    gpio_set_direction(TRIGGER_PIN_1, GPIO_MODE_OUTPUT); // setting trigger pin as an output
    

    
    gpio_pad_select_gpio(ECHO_PIN_1); // initializing gpio pin 19 - echo pin of hcsr04
    gpio_set_direction(ECHO_PIN_1, GPIO_MODE_INPUT); // setting echo pin as an input

    gpio_pad_select_gpio(ECHO_PIN_2); // initializing gpio pin 19 - echo pin of hcsr04
    gpio_set_direction(ECHO_PIN_2, GPIO_MODE_INPUT); // setting echo pin as an input

}


void setup_uart() {
    const uart_config_t uart_config = {
        .baud_rate = 9600,     // Baud rate for Sabertooth communication - found per document
        .data_bits = UART_DATA_8_BITS, // general uart config
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM, &uart_config);
    uart_set_pin(UART_NUM, TX_PIN_PIX, RX_PIN_PIX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM, 1024, 0, 0, NULL, 0);
}


// Function to run to have all componets initalize correctly 
void every_init(){
    
    vTaskDelay(pdMS_TO_TICKS(500));

    // Establish I2C communcation
    i2c_init();
    vTaskDelay(pdMS_TO_TICKS(500));

    // Wake up the battery monitors 
    wake_up_bq76920();
    vTaskDelay(pdMS_TO_TICKS(500));

    // Check if battery monitors are responding
    i2c_scanner();
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // Initialize DAC
    dac_init(); 

    // Initialize LED
    led_init();

    // Intialize PTT transistor gpio
    ptt_init();

    //Intialize UART
    init_uart();

    init_uart_pix();
    //setup_uart();

    // Generate sine wave lookup table
    generateSineTable();

    // iniate the ultrasonic gpio pins
    init_ultrasonic();

    
}

// Send arm or disarm signal to the pixhawk
void send_arm_disarm(bool arm) {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    mavlink_msg_command_long_pack(
        ESP_SYSTEM_ID, MAV_COMP_ID, &msg,
        MAV_SYSTEM_ID, MAV_COMP_ID,
        MAV_CMD_COMPONENT_ARM_DISARM, // MAVLink command to ARM/DISARM
        0,  // Confirmation
        arm ? 1 : 0,  // Param1: 1 to arm, 0 to disarm
        0, 0, 0, 0, 0, 0  // Unused parameters
    );

    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    uart_write_bytes(UART_NUM, buf, len);
    
    ESP_LOGI(TAG_PIX, "Sent MAVLink %s command", arm ? "ARM" : "DISARM");
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
        ESP_LOGI(TAG_PIX, "Sent GPS data request (%d bytes)", bytes_sent);
    } else {
        ESP_LOGE(TAG_PIX, "Failed to send GPS request");
    }
}

// Process parsed MAVLink messages
void process_mavlink_message(mavlink_message_t *msg) {
    char gps_message_lat[10];
    char gps_message_lon[10];
    char gps_message_alt[10];
    // ESP_LOGI(TAG_PIX, "Received message ID: %d", msg->msgid);
    switch (msg->msgid) {
        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
            mavlink_global_position_int_t gps;
            mavlink_msg_global_position_int_decode(msg, &gps);
            float lat = gps.lat / 1e7;
            float lon = gps.lon / 1e7;
            float alt = gps.alt / 1000.0;
            ESP_LOGI("GPS", "Lat: %.7f, Lon: %.7f, Alt: %.2f m", lat, lon, alt);
            gpio_set_level(PTT, 1);
            vTaskDelay(pdMS_TO_TICKS(1000));
            snprintf(gps_message_lat, sizeof(gps_message_lat), "%.2f", lat);
            playMorseString(gps_message_lat, FREQUENCY);
            snprintf(gps_message_lon, sizeof(gps_message_lon), "%.2f", lon);
            playMorseString(gps_message_lon, FREQUENCY);
            //snprintf(gps_message_alt, sizeof(gps_message_alt), "%.2f", alt);
            //playMorseString(gps_message_alt, FREQUENCY);
            // Disable PTT gpio
            gpio_set_level(PTT, 0);
            dac_output_voltage(DAC_PIN, 0);

            break;

        }
        default:
            break;
    }
}

// Task to receive and parse MAVLink messages
void receive_mavlink_task() {
    uint8_t data[512];  // Increased buffer size to 512 bytes
    mavlink_message_t msg;
    mavlink_status_t status;
    static int good_messages = 0;  // Counter for successfully parsed messages
    static int bad_crcs = 0;       // Counter for bad CRC errors

    //while (1) {
        // Read UART data with a 100ms timeout
        int len = uart_read_bytes(UART_NUM, data, sizeof(data), 100 / portTICK_PERIOD_MS);
        if (len > 0) 
            // Parse each byte with mavlink_parse_char
            for (int i = 0; i < len; i++) {
                if (mavlink_parse_char(MAVLINK_COMM_0, data[i], &msg, &status)) {
                    process_mavlink_message(&msg);
                } else {
                    // Log parser state for debugging
                    switch (status.parse_state) {
                        case MAVLINK_PARSE_STATE_GOT_BAD_CRC1:
                            bad_crcs++;  // Increment bad CRC counter
                            break;
                        default:
                            break;  // Other states can be logged if needed
                    }
                }
            }
            // Log message statistics after processing the buffer

        vTaskDelay(pdMS_TO_TICKS(10000)); // 100ms delay between reads
    //}
}

// Returns current decimal value being outputted by the DTMF decoder
int get_DTMF_bit(){
    // Read the state of each GPIO pin and calculate the decimal value
    int decimal_value = 0;

    // Iterated through each of the bits and calculate the decimal value
    for (int i = 0; i < NUM_PINS; i++) {
        int state = gpio_get_level(pins[i]);
        decimal_value |= (state << i);  // Combine the bits to form the binary number
    }

    // Print the binary and decimal values
    printf("Binary: %d%d%d%d, Decimal: %d\n",
       gpio_get_level(pins[3]),  // MSB (Bit3)
       gpio_get_level(pins[2]),  // Bit2
       gpio_get_level(pins[1]),  // Bit1
       gpio_get_level(pins[0]),  // LSB (Bit0)
       decimal_value);
    
    // Returns final decimal value
    return(decimal_value);
}

// Checks status of all Battery monitors and return bool value
bool get_BM_Stat(){
    int stat = 0;

    // Set SDA line to BM1
    i2c_set_sda(I2C_SDA_BM1);
    // Read SYS_STAT register and adds to stat decimal 
    stat += (int)((read_register(SYS_STAT) >> 4) & 0x0F);

    /*
    // Set SDA line to BM2
    i2c_set_sda(I2C_SDA_BM2);
    // Read SYS_STAT register and adds to stat decimal 
    stat += (int)((read_register(SYS_STAT) >> 4) & 0x0F);
    */
    /*
    i2c_set_sda(I2C_SDA_BM3);
    stat += (int)((read_register(SYS_STAT) >> 4) & 0x0F);
    */

    // If the sum of the stat is zero means BM are all goof
    if(stat == 0){
        return true;
    }
    else{
        return false;
    }

}

// Return total voltage of battery connected to the BM
float get_BAT_VOLT(){
    float volt = 0;
    // Read each cell voltgae and adds them together
    volt += read_cell_voltage(VC1_HI, VC1_LO);
    volt += read_cell_voltage(VC2_HI, VC2_LO);
    volt += read_cell_voltage(VC5_HI, VC5_LO);
    return(volt);
}


// generic function used to measure distance using ultrasonic sensor and gpio pins
float get_distance_1(void) { // function to obtain distance for 1st ultrasonic sensor

    gpio_set_level(TRIGGER_PIN_1, 0);
    vTaskDelay(2 / portTICK_PERIOD_MS);
    gpio_set_level(TRIGGER_PIN_1, 1);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    gpio_set_level(TRIGGER_PIN_1, 0);

    int64_t start_time = esp_timer_get_time();
    while (gpio_get_level(ECHO_PIN_1) == 0) {
        start_time = esp_timer_get_time();
    }

    int64_t end_time = esp_timer_get_time();
    while (gpio_get_level(ECHO_PIN_1) == 1) {
        end_time = esp_timer_get_time();
    }

    int64_t time_diff = end_time - start_time;
    float distance = (time_diff * 0.0343) / 2;  // Convert time to distance cm
    return distance;
}

// generic function used to measure distance using ultrasonic sensor and gpio pins
float get_distance_2(void) { // function to obtain distance for 1st ultrasonic sensor

    gpio_set_level(TRIGGER_PIN_1, 0);
    vTaskDelay(2 / portTICK_PERIOD_MS);
    gpio_set_level(TRIGGER_PIN_1, 1);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    gpio_set_level(TRIGGER_PIN_1, 0);

    int64_t start_time = esp_timer_get_time();
    while (gpio_get_level(ECHO_PIN_2) == 0) {
        start_time = esp_timer_get_time();
    }

    int64_t end_time = esp_timer_get_time();
    while (gpio_get_level(ECHO_PIN_2) == 1) {
        end_time = esp_timer_get_time();
    }

    int64_t time_diff = end_time - start_time;
    float distance = (time_diff * 0.0343) / 2;  // Convert time to distance cm
    return distance;
}


// Determines if object is detected bool
bool obj_detect(){
    // Check if object is close for either sensor
    if (get_distance_1() < 150 || get_distance_2() < 150){
        return true;
    }
    return false;
}

   
// Note structure: Frequency in Hz, Duration in milliseconds
typedef struct {
    int frequency;
    int duration;
} Note;

// Twinkle Twinkle Little Star melody (in Hz and duration in ms)
Note melody[] = {
    {262, 500}, {262, 500}, {392, 500}, {392, 500}, {440, 500}, {440, 500}, {392, 1000}, // Twinkle Twinkle
    {349, 500}, {349, 500}, {330, 500}, {330, 500}, {294, 500}, {294, 500}, {262, 1000}, // Little Star
    {392, 500}, {392, 500}, {349, 500}, {349, 500}, {330, 500}, {330, 500}, {294, 1000}, // How I Wonder
    {392, 500}, {392, 500}, {349, 500}, {349, 500}, {330, 500}, {330, 500}, {294, 1000}, // What You Are
    {262, 500}, {262, 500}, {392, 500}, {392, 500}, {440, 500}, {440, 500}, {392, 1000}, // Twinkle Twinkle
    {349, 500}, {349, 500}, {330, 500}, {330, 500}, {294, 500}, {294, 500}, {262, 1000}  // Little Star
};

// Function to generate tone for the given frequency and duration
void playTone(int frequency, int duration) {
    if (frequency == 0) { // Rest note
        vTaskDelay(pdMS_TO_TICKS(duration));
        return;
    }

    int sampleRate = 40000;  // 40 kHz DAC sample rate
    int numSamples = sampleRate * duration / 1000;
    
    for (int i = 0; i < numSamples; i++) {
        float t = (float)i / sampleRate;
        uint8_t value = (uint8_t)(128 + 127 * sin(2 * PI * frequency * t));  // Generate sine wave
        dac_output_voltage(DAC_PIN, value);
        vTaskDelay(pdMS_TO_TICKS(1));  // Small delay for timing
    }
}

// Plays twinkle little star melody
void play_music(){
    // Calculates the number of notes (deicmal value)
    int numNotes = sizeof(melody) / sizeof(melody[0]);

    // iterate through whole melody 
    for (int i = 0; i < numNotes; i++) {
        // Play the tone for time
        playTone(melody[i].frequency, melody[i].duration);
        vTaskDelay(pdMS_TO_TICKS(50)); // Small pause between notes
    }
}

// Sophia's Call sign for Radio use
void call_sign(){
    playMorseString("KJ5HZT", FREQUENCY);
}

// Melody generated for recieving signal 
Note melody2[] = {
    //{ 932, 150 }, { 1245, 150}, { 1864, 150}
    { 880, 200 }, { 988, 200 }, { 1319, 300 }
    //{ 660, 200 }, { 880, 200 }, { 1100, 300 }
};

// Melody generated for recieving signal 
Note melody3[] = {
    { 932, 150 }, { 1245, 150}, { 1864, 150}
    
    //{ 660, 200 }, { 880, 200 }, { 1100, 300 }
};

// Play melody 2
void play_music2(){
    
    
    int numNotes = sizeof(melody2) / sizeof(melody2[0]);
    for (int i = 0; i < numNotes; i++) {
        playTone(melody2[i].frequency, melody2[i].duration);
        vTaskDelay(pdMS_TO_TICKS(50)); // Small pause between notes
    }

}

// Play melody 2
void play_music3(){
    
    
    int numNotes = sizeof(melody3) / sizeof(melody3[0]);
    for (int i = 0; i < numNotes; i++) {
        playTone(melody3[i].frequency, melody3[i].duration);
        vTaskDelay(pdMS_TO_TICKS(50)); // Small pause between notes
    }

}



// Flag variable to store detected action number
volatile int bit_action_flag = -1;

// Hold previous value of the Bit flag
volatile int bit_prev_flag = -1;


// Monitor Bits of gpio pins and determine if the reading is accurate and set a flag for trigger
void vTask_HealthMode(void *pvParameters) {
    // Internal count variable for the number of times read the same value
    
    // Infinte while to loop till an action flag is triggered
    while (1) {
            

            bit_action_flag = get_DTMF_bit();;

            // if condition that checks to see if bit_action_flag needs to be set 
            // checks if the previous flag is not the same as current, dont do action on zero, and make sure there was a count of 3
            if((bit_prev_flag != bit_action_flag) && !(bit_action_flag == 10)){
                // set bit flag to the new decimal value
                char message[20];  // Adjust the size as needed
                gpio_set_level(PTT, 1);
                playMorseString("Recieved", FREQUENCY);
                vTaskDelay(pdMS_TO_TICKS(1000));
                call_sign();
                gpio_set_level(PTT, 0);


                vTaskDelay(pdMS_TO_TICKS(3000));
                // If else block for each of the digits
                if(bit_action_flag == 1){   
                    printf("Mode 1 Activated: PTT ON and OFF Movement \n");
                    //send_arm_disarm(false);
                    vTaskDelay(pdMS_TO_TICKS(5000));
                    
                    // Enable push to talk
                    gpio_set_level(PTT, 1);

                    // Tell user Mode 1 was started
                    playMorseString("Mode 1 Start", FREQUENCY);
                    vTaskDelay(pdMS_TO_TICKS(1000));


                    // Disable Push to talk
                    gpio_set_level(PTT, 0);
                    vTaskDelay(pdMS_TO_TICKS(1000));
                    
                    // ***************************************** //
                    // Send are to pixhawk to start movement on path
                    send_arm_disarm(true); 
                    // Check if pixhawk is working and on

                    
                    // Loop to repeat signal ON and OFF
                    for(int i=0; i<5; ++i){
                        // Enable Push to talk 
                        gpio_set_level(PTT, 1);
                        playMorseString("Hello Catch Me", FREQUENCY);
                        vTaskDelay(pdMS_TO_TICKS(1000));
                        
                        gpio_set_level(PTT, 0);
                        printf("\n");
                        vTaskDelay(pdMS_TO_TICKS(10000)); 
                    }
                    gpio_set_level(PTT, 1);
                    call_sign();
                    gpio_set_level(PTT, 0);
                    
                    vTaskDelay(pdMS_TO_TICKS(1000)); 
                    send_arm_disarm(false); 
                    vTaskDelay(pdMS_TO_TICKS(5000));  

                    /*
                    vTaskDelay(pdMS_TO_TICKS(8000));
                    send_arm_disarm(true); 
                    
                    playMorseString("PIXARMED", FREQUENCY);
                    */
                }
                else if(bit_action_flag == 2){
                    printf("Mode 2 Activated: Intermittant PTT and Stationary\n");

                    // Enable push to talk
                    gpio_set_level(PTT, 1);

                    // Tell user Mode 1 was started
                    playMorseString("Mode 2 Start", FREQUENCY);
                    vTaskDelay(pdMS_TO_TICKS(1000));

                    

                    // Disable Push to talk
                    gpio_set_level(PTT, 0);
                    vTaskDelay(pdMS_TO_TICKS(1000));

                    // Loop to repeat signal ON and OFF
                    for(int i=0; i<5; ++i){

                        // Enable Push to talk 
                        gpio_set_level(PTT, 1);
                        playMorseString("Hello Catch Me", FREQUENCY);
                        vTaskDelay(pdMS_TO_TICKS(1000));
                        
                        gpio_set_level(PTT, 0);
                        printf("\n");
                        vTaskDelay(pdMS_TO_TICKS(10000)); 
                    }

                    gpio_set_level(PTT, 1);
                    call_sign();
                    gpio_set_level(PTT, 0);

                    vTaskDelay(pdMS_TO_TICKS(5000));          
                       
                }
                else if(bit_action_flag == 5){  
                    printf("Mode 5 Activated: Intermittant PTT, Intermittent movement\n");

                    // Enable push to talk
                    gpio_set_level(PTT, 1);

                    // Tell user Mode 1 was started
                    playMorseString("Mode 5 Start", FREQUENCY);
                    vTaskDelay(pdMS_TO_TICKS(1000));

                    // Disable Push to talk
                    gpio_set_level(PTT, 0);
                    vTaskDelay(pdMS_TO_TICKS(1000));

                    // Loop to repeat signal ON and OFF
                    for(int i=0; i<5; ++i){

                        send_arm_disarm(true); 
                        // Enable Push to talk 
                        gpio_set_level(PTT, 1);
                        playMorseString("Hello Catch Me", FREQUENCY);
                        vTaskDelay(pdMS_TO_TICKS(1000));
                        
                        gpio_set_level(PTT, 0);
                        printf("\n");
                        send_arm_disarm(false); 
                        vTaskDelay(pdMS_TO_TICKS(10000)); 
                    }
                    vTaskDelay(pdMS_TO_TICKS(5000));     
                    vTaskDelay(pdMS_TO_TICKS(3000));
                }
                else if(bit_action_flag == 3){      
                    printf("Mode 3 Activated: Constant transmission and NO Movement\n");
                    
                    // Enable push to talk
                    gpio_set_level(PTT, 1);

                    // Tell user Mode 1 was started
                    playMorseString("Mode 3 Start", FREQUENCY);
                    vTaskDelay(pdMS_TO_TICKS(1000));

                    // Call sign
                    call_sign();

                    // Disable Push to talk
                    gpio_set_level(PTT, 0);
                    vTaskDelay(pdMS_TO_TICKS(3000));
                    
                    gpio_set_level(PTT, 1); 
                    play_music();
                    vTaskDelay(pdMS_TO_TICKS(2000));
                    play_music();
                    vTaskDelay(pdMS_TO_TICKS(2000));
                    play_music();
                    call_sign();
                    gpio_set_level(PTT, 0); 
                    vTaskDelay(pdMS_TO_TICKS(5000));   
                    
                    
                }
                else if(bit_action_flag == 4){      
                    printf("Mode 4 Activated: Continuous PTT and continuous movement \n");

                    // Enable push to talk
                    gpio_set_level(PTT, 1);

                    // Tell user Mode 1 was started
                    playMorseString("Mode 4 Start", FREQUENCY);
                    vTaskDelay(pdMS_TO_TICKS(1000));

                    // Call sign
                    call_sign();

                    // Disable Push to talk
                    gpio_set_level(PTT, 0);
                    vTaskDelay(pdMS_TO_TICKS(1000));

                    // Send are to pixhawk to start movement on path
                    send_arm_disarm(true); 
                    // Check if pixhawk is working and on
                    
                    
                    // Disable Push to talk
                    gpio_set_level(PTT, 0);
                    vTaskDelay(pdMS_TO_TICKS(3000));
                    
                    gpio_set_level(PTT, 1); 
                    printf("Starting Music \n");
                    play_music();
                    vTaskDelay(pdMS_TO_TICKS(1000));
                    printf("Second Iteration \n");
                    play_music();
                    vTaskDelay(pdMS_TO_TICKS(1000));
                    printf("Third Iteration \n");
                    play_music();
                    call_sign();
                    gpio_set_level(PTT, 0); 
                    // Send are to pixhawk to stop movement 
                    send_arm_disarm(false); 
                    vTaskDelay(pdMS_TO_TICKS(5000)); 


                }
                else if(bit_action_flag == 6){      
                    // Only does BM1 right now
                    char tot_mess[40];


                    i2c_set_sda(I2C_SDA_BM1);
                    snprintf(message, sizeof(message), "%.2f", get_BAT_VOLT());
                    strcat(message, " V "); 
                    strcat(tot_mess, "B1 ");
                    strcat(tot_mess,  message);

                    vTaskDelay(pdMS_TO_TICKS(1000));

                    /*
                    // Only does BM1 right now
                    i2c_set_sda(I2C_SDA_BM2);
                    snprintf(message, sizeof(message), "%.2f", get_BAT_VOLT());
                    strcat(message, " V ");
                    strcat(tot_mess, "B2 ");
                    strcat(tot_mess,  message); 

                    vTaskDelay(pdMS_TO_TICKS(1000));
                    
                    // Only does BM1 right now
                    i2c_set_sda(I2C_SDA_BM3);
                    snprintf(message, sizeof(message), "%.2f", get_BAT_VOLT());
                    strcat(message, " V ");
                    strcat(tot_mess, "B3 ");
                    strcat(tot_mess,  message); 
                    */
                    gpio_set_level(PTT, 1); 
                    playMorseString(tot_mess, FREQUENCY);
                    vTaskDelay(pdMS_TO_TICKS(1000));
                    call_sign();
                    gpio_set_level(PTT, 0); 
                    vTaskDelay(pdMS_TO_TICKS(1000));

                }
                else if(bit_action_flag == 8){      
                    printf("This is GPS coordinates \n");
                    receive_mavlink_task();
                    // ******** NEED TO PUT CALL SIGN
                    vTaskDelay(pdMS_TO_TICKS(1000));


                }
                else if(bit_action_flag == 7){      
                    printf("This is ultrasonic sensor \n");
                    if(obj_detect()){
                        playMorseString("OBJ Detect", FREQUENCY);
                    }
                    else{
                        playMorseString("No OBJ", FREQUENCY);
                    }
                }
                else if(bit_action_flag == 9){      
                    printf("OFF \n");
                    gpio_set_level(PTT, 1); 
                    playMorseString("Turning OFF", FREQUENCY);
                    send_arm_disarm(false);
                    gpio_set_level(PTT, 0); 
                    break;
                }
                else{
                    printf("Not an actionable task \n");

                }
        
                // Wait time
                printf("This is back in health mode \n");
                // Turns on PTT
                gpio_set_level(PTT, 1);
                playMorseString("H mode", FREQUENCY);
                vTaskDelay(pdMS_TO_TICKS(1000));
                call_sign();
                gpio_set_level(PTT, 0);
                vTaskDelay(pdMS_TO_TICKS(5000));
            
                // Store value of action flag in a previous flag 
                bit_prev_flag = bit_action_flag;
                
            }
            // Delay before next reading
            vTaskDelay(pdMS_TO_TICKS(5000));
        }
        
}


void app_main(void)
{    
      
    bool flag = false;
    char message[20];
    int val =0;
    bit_init();
    // Wait for user to input one on DTMF
    while(1){
        val = get_DTMF_bit();
        if(val == 10){
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(5000));
    }


    vTaskDelay(pdMS_TO_TICKS(5000));
    // Intialize everything we need
    every_init();

    
    gpio_set_level(PTT, 1);
    playMorseString("Recieved", FREQUENCY);
    vTaskDelay(pdMS_TO_TICKS(1000));
    gpio_set_level(PTT, 0);
    

    if(get_BM_Stat()){
        printf("Battery Monitor System is good \n");
        gpio_set_level(PTT, 1);
        
        playMorseString("BM Good", FREQUENCY);
        vTaskDelay(pdMS_TO_TICKS(1000));
        call_sign();
        gpio_set_level(PTT, 0);
        vTaskDelay(pdMS_TO_TICKS(1000));
    
    }
    else{
        printf("Battery Monitor System is Bad \n");
        gpio_set_level(PTT, 1);
        
        playMorseString("BM Bad", FREQUENCY);
        vTaskDelay(pdMS_TO_TICKS(1000));
        call_sign();
        gpio_set_level(PTT, 0);

        flag = true;

        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    

    if(!flag){
        // arm pixhawk and wait till at home point 
        xTaskCreate(vTask_HealthMode, "Bit Monitor Task", 8192, NULL, 5, &xTaskBitMonitor);
    }
    else{
        for(int i=0; i<5; ++i){
            gpio_set_level(PTT, 1);
        
            playMorseString("System Error!!!!!", FREQUENCY);
            vTaskDelay(pdMS_TO_TICKS(1000));
            call_sign();
            gpio_set_level(PTT, 0);
            vTaskDelay(pdMS_TO_TICKS(5000));
        }

    }
    
    
    
}
