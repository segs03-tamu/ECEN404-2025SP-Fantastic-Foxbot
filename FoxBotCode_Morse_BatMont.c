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

#define SAMPLE_PERIOD_MS		200
#define I2C_SDA_IO              21  
#define I2C_SCL_IO              22  
#define I2C_FREQ_HZ             100000  

#define TS1_GPIO                18                /*Raising Edge to boot chip*/


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


#define ADCGAIN1        0x50
#define ADCOFFSET       0x51
#define ADCGAIN2        0x59


// Constants for the morse code bit
#define DAC_PIN DAC_CHANNEL_1   // GPIO25 (DAC1)
#define LED_PIN GPIO_NUM_2      // GPIO2 (Built-in LED on many ESP32 boards)
#define SINE_RESOLUTION 100     // Number of points in one sine wave cycle
#define ADC_MAX_VALUE 4095      // 12-bit ADC max value
#define VOLTAGE_MAX 3.3         // Maximum ADC input voltage

#define FREQUENCY 2000      // Maximum frequency (Hz)

// Morse code timing (in milliseconds)
/*
#define DOT_DURATION 100
#define DASH_DURATION 300
#define PART_SPACE_DURATION 100
#define LETTER_SPACE_DURATION 300
#define WORD_SPACE_DURATION 700
*/
#define DOT_DURATION 1000
#define DASH_DURATION 3000
#define PART_SPACE_DURATION 1000
#define LETTER_SPACE_DURATION 3000
#define WORD_SPACE_DURATION 7000


#define Trans GPIO_NUM_23


#define Bit0 GPIO_NUM_36
#define Bit1 GPIO_NUM_39
#define Bit2 GPIO_NUM_34
#define Bit3 GPIO_NUM_35

// Define the GPIO pins
#define NUM_PINS 4
const gpio_num_t pins[NUM_PINS] = {GPIO_NUM_36, GPIO_NUM_39, GPIO_NUM_34, GPIO_NUM_35};  // Replace with your GPIO pin numbers



static const char *TAG = "i2c_restart";


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

/**
 * @brief Test code to write i2c slave device with registered interface
 *        Master device write data to slave(both esp32),
 *        the data will be stored in slave buffer.
 *        We can read them out from slave buffer.
 * ____________________________________________________________________________________
 * | start | slave_addr + wr_bit + ack | register + ack | write n bytes + ack  | stop |
 * --------|---------------------------|----------------|----------------------|------|
 *
 */
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

/* Read contents of a register
---------------------------------------------------------------------------*/
esp_err_t rdAFE( uint8_t reg, uint8_t *pdata, uint8_t count )
{
	return( i2c_master_read_slave_reg(I2C_PORT_NUM, BQ7692003PWR_I2C_ADDR,  reg, pdata, count ) );
}

/* Write value to specified register
---------------------------------------------------------------------------*/
esp_err_t wrAFE( uint8_t reg, uint8_t *pdata, uint8_t count )
{
	return( i2c_master_write_slave_reg( I2C_PORT_NUM, BQ7692003PWR_I2C_ADDR,  reg, pdata, count ) );
}


void wake_up_bq76920() {
    gpio_set_direction(TS1_GPIO, GPIO_MODE_OUTPUT);

    // Ensure TS1 is LOW initially
    gpio_set_level(TS1_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(50));  // Wait 50ms

    // Create a rising edge by setting TS1 HIGH
    gpio_set_level(TS1_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(50));  // Wait 50ms

    ESP_LOGI("BQ76920", "Wake-up pulse sent to TS1");
}

void test_write_and_verify() {
    uint8_t data_to_write = 0x19;  // Example data to write
    uint8_t read_data = 0;

    // Write to CC_CFG
    esp_err_t err = wrAFE(CC_CFG, &data_to_write, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write to CC_CFG: %s", esp_err_to_name(err));
        return;
    }

    // Read back from CC_CFG
    err = rdAFE(CC_CFG, &read_data, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read from CC_CFG: %s", esp_err_to_name(err));
        return;
    }

    // Verify the written value
    if (read_data == data_to_write) {
        ESP_LOGI(TAG, "Write verified successfully. CC_CFG value: 0x%02X", read_data);
    } else {
        ESP_LOGE(TAG, "Write verification failed. Expected: 0x%02X, Actual: 0x%02X", data_to_write, read_data);
    }
}

void set_bit(uint8_t reg, uint8_t bit) {
    if (bit > 7) {
        ESP_LOGE(TAG, "Invalid bit position. Bit must be between 0 and 7.");
        return;
    }

    uint8_t reg_value;
    esp_err_t err;

    // Step 1: Read the current value of the register
    err = rdAFE(reg, &reg_value, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read register 0x%02X: %s", reg, esp_err_to_name(err));
        return;
    }
    ESP_LOGI(TAG, "Current register 0x%02X value: 0x%02X", reg, reg_value);

    // Step 2: Set the specified bit to 1
    reg_value |= (1 << bit);  // Set the bit at the specified position

    // Step 3: Write the modified value back to the register
    err = wrAFE(reg, &reg_value, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write register 0x%02X: %s", reg, esp_err_to_name(err));
        return;
    }
    ESP_LOGI(TAG, "Bit %d set. New register 0x%02X value: 0x%02X", bit, reg, reg_value);
}


/**
 * @brief Read the value of a specified register.
 * @param reg The register address (e.g., SYS_STAT, SYS_CTRL1, etc.).
 * @return The value of the register, or 0xFF if the read operation fails.
 */
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
    printf("This is the HI: 0x%02X, and this is the Lo: 0x%02X \n", vc_hi, vc_low);

    // Combine VC_HI and VC_LOW into a 16-bit value
    uint16_t combined_value = (vc_hi << 8) | vc_low;

    // Mask out bits <15:14> to use only bits <13:0>
    combined_value = combined_value & 0x3FFF; // 0x3FFF = 0b0011111111111111

    // Multiply the combined value by 375 microvolts (375e-6 volts)
    float voltage = combined_value * 375e-6;

    return voltage;
}
// Function to read the cell voltage
float read_bat_voltage(uint8_t vc_hi_addr, uint8_t vc_low_addr) {
    // Read the values of VC_HI and VC_LOW registers
    uint8_t vc_hi = read_register(vc_hi_addr);
    uint8_t vc_low = read_register(vc_low_addr);
    printf("This is the HI: 0x%02X, and this is the Lo: 0x%02X \n", vc_hi, vc_low);

    // Combine VC_HI and VC_LOW into a 16-bit value
    uint16_t combined_value = (vc_hi << 8) | vc_low;

    // Multiply the combined value by 375 microvolts (375e-6 volts)
    float voltage = combined_value * 375e-6;

    return voltage;
}

static void afe_init()
{ 

    printf("This is sys_STAT:  0x%02X \n", read_register(SYS_STAT));   
    float voltage = read_cell_voltage(VC1_HI, VC1_LO);
    printf("Cell 1 Voltage: %f V\n", voltage);
    printf("This is battery Voltage: %f V \n", read_bat_voltage(BAT_HI, BAT_LO));
    
}

void i2c_init() {
    i2c_config_t conf = {0};  
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_FREQ_HZ;
    
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

// Initialize LED
void led_init() {
    gpio_reset_pin(LED_PIN); // Reset the LED pin
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT); // Set LED pin as output
}

// Initialize transistor pin
void trans_init() {
    gpio_reset_pin(Trans); // Reset the LED pin
    gpio_set_direction(Trans, GPIO_MODE_OUTPUT); // Set LED pin as output
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
}


// Flag variable to store detected action number
volatile int bit_action_flag = -1;

// Hold previous value of the Bit flag
volatile int bit_prev_flag = -1;

// Monitor Bits of gpio pins and determine if the reading is accurate and set a flag for trigger
void vTaskMonitorBits(void *pvParameters) {
    // Internal count variable for the number of times read the same value
    int count = 1;

    // Decimal Value to represent the value of the bits in decimal
    int decimal_value = 0;

    // Variable responsible for holding the last value of the decimal value
    int prev_decimal_value = -1;

    // Infinte while to loop till an action flag is triggered
    while (1) {

        // Initial if statement to check if the bit_action_flag is in its orgrinal state -1
        if(bit_action_flag == -1){

            //reset the decimal value to zero at begining of every loop
            decimal_value = 0;

            // Read the state of each GPIO pin and calculate the decimal value
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
            
            // Check if the current value matches the previous value
            if (decimal_value == prev_decimal_value) {
                count++; // Increment the counter
            } 
            // Else reset the count to 1
            else {
                count = 1; // Reset the counter if the value changes
            }

            // Update the previous value
            prev_decimal_value = decimal_value;

            // if condition that checks to see if bit_action_flag needs to be set 
            // checks if the previous flag is not the same as current, dont do action on zero, and make sure there was a count of 3
            if(bit_prev_flag != decimal_value && decimal_value !=0 && (count >=3)){
                // set bit flag to the new decimal value
                bit_action_flag = decimal_value;
            }
            // Delay before next reading
            vTaskDelay(pdMS_TO_TICKS(5000));
        }
        else{
            // Suspend task if a flag is triggered
            vTaskSuspend(NULL);
        } 
    } 
}

// Initialize transistor pin
void bit_init() {
    // Configure the GPIO pins as inputs
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;  // Disable interrupt
    io_conf.mode = GPIO_MODE_INPUT;         // Set as input mode
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;  // Disable pull-down
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;      // Disable pull-up

    for (int i = 0; i < NUM_PINS; i++) {
        io_conf.pin_bit_mask = (1ULL << pins[i]);
        gpio_config(&io_conf);
    }
}

//Define variable for 
TaskHandle_t xTaskBitMonitor = NULL; // Declare a variable to store the task handle



void every_int(){
    /*
    vTaskDelay(pdMS_TO_TICKS(500));
    i2c_init();
    vTaskDelay(pdMS_TO_TICKS(500));
    wake_up_bq76920();
    vTaskDelay(pdMS_TO_TICKS(500));
    i2c_scanner();
    vTaskDelay(pdMS_TO_TICKS(500));
    
    //printf("This is working");
    
    afe_init();
    
      
    */

    // Initialize DAC
    dac_init(); 

    // Initialize LED
    led_init();

    // Intialize PTT transistor gpio
    trans_init();

    //Intialize gpio bits 0-3 
    bit_init();

    // Generate sine wave lookup table
    generateSineTable();

    
}

#define UART_NUM        UART_NUM_1  // Use UART1 for MAVLink
#define TX_PIN          17
#define RX_PIN          16
#define BAUD_RATE       57600
#define MAV_SYSTEM_ID   1   // Pixhawk system ID
#define ESP_SYSTEM_ID   255 // ESP32 system ID
#define MAV_COMP_ID     0   // Target component (autopilot)

// Debugging tag
static const char *TAGG = "MAVLINK_ESP32";

// Function to initialize UART for MAVLink
void init_uart() {
    uart_config_t uart_config = {
        .baud_rate = BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT
    };

    uart_driver_install(UART_NUM, 512, 0, 0, NULL, 0);
    uart_param_config(UART_NUM, &uart_config);
    uart_set_pin(UART_NUM, TX_PIN, RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    ESP_LOGI(TAGG, "UART initialized for MAVLink");
}

// Function to send an ARM/DISARM command to Pixhawk
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
    
    ESP_LOGI(TAGG, "Sent MAVLink %s command", arm ? "ARM" : "DISARM");
}


void request_mavlink_data() {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    mavlink_msg_command_long_pack(
        255, 0, &msg,   // ESP32 as sender
        1, 0,           // Pixhawk system ID and component ID
        MAV_CMD_SET_MESSAGE_INTERVAL, 
        0, 
        MAVLINK_MSG_ID_GLOBAL_POSITION_INT, // Request GPS data
        100000,  // Send every 100ms (10Hz)
        0, 0, 0, 0, 0  // Unused
    );

    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    uart_write_bytes(UART_NUM, buf, len);
    
    ESP_LOGI("MAVLINK", "Requested MAVLink GPS Data from Pixhawk");
}


// Function to process received MAVLink messages
void process_mavlink_message(mavlink_message_t *msg) {
    if (msg->msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT) {
        mavlink_global_position_int_t gps;
        mavlink_msg_global_position_int_decode(msg, &gps);

        // Convert latitude & longitude from 1E7 format to decimal degrees
        float lat = gps.lat / 1e7;
        float lon = gps.lon / 1e7;
        float alt = gps.alt / 1000.0;  // Convert mm to meters

        ESP_LOGI(TAGG, "GPS: Lat: %.7f, Lon: %.7f, Alt: %.2f m", lat, lon, alt);
    }
    else {
        ESP_LOGW(TAGG, "MAVLink message received, but not GPS data (ID: %d)", msg->msgid);
    }
}

// Task to receive and parse MAVLink messages
void receive_mavlink_task(void *pvParameters) {
    uint8_t data;
    mavlink_message_t msg;
    mavlink_status_t status;
    ESP_LOGI(TAGG, "GPS initialized for MAVLink: begin parsing");

    while (1) {

        int len = uart_read_bytes(UART_NUM, &data, 1, 100 / portTICK_PERIOD_MS);
        if (len > 0) {
            if (mavlink_parse_char(MAVLINK_COMM_0, data, &msg, &status)) {
                process_mavlink_message(&msg);
                ESP_LOGI(TAG, "Received MAVLink Message ID: %d", msg.msgid);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
// Responsible for determing what each digit does and how to react
void num_actions(int num_val){
    //char message[20];  // Adjust the size as needed

    //Enable push to talk
    gpio_set_level(Trans, 1);
    vTaskDelay(pdMS_TO_TICKS(1000));        //Pause for time     

    // If else block for each of the digits
    if(bit_action_flag == 1){      
        // playMorseString("One", FREQUENCY); 
        send_arm_disarm(true);  // ARM the Pixhawk

        /*
        printf("Executing Action %d, reading Cell1 Voltage \n", bit_action_flag);
        // Read the voltage of Cell 1
        float voltage = read_cell_voltage(VC1_HI, VC1_LO);
        // Convert the voltage value to a string
        snprintf(message, sizeof(message), "%.2f", voltage);

        // Log the voltage value for debugging
        printf("Cell 1 Voltage: %s V\n", message);
            
        // Play Morse code message
        playMorseString(message, FREQUENCY);
        */   
    }
    else if(bit_action_flag == 2){      
        // playMorseString("Two", FREQUENCY);  
        send_arm_disarm(false);  // ARM the Pixhawk
  
    }
    else if(bit_action_flag == 3){      
        playMorseString("Three", FREQUENCY);    
    }
    else if(bit_action_flag == 4){      
        playMorseString("Four", FREQUENCY);    
    }
    else if(bit_action_flag == 5){      
        playMorseString("Five", FREQUENCY);    
    }
    else if(bit_action_flag == 6){      
        playMorseString("Six", FREQUENCY);    
    }
    else if(bit_action_flag == 7){      
        playMorseString("Seven", FREQUENCY);    
    }
    else if(bit_action_flag == 8){      
        playMorseString("Eight", FREQUENCY);    
    }
    else if(bit_action_flag == 9){      
        playMorseString("Nine", FREQUENCY);    
    }
    else{
        printf("Not an actionable task \n");
        bit_action_flag = 0;
    }

    // Wait time
    vTaskDelay(pdMS_TO_TICKS(1000));

    // Disable PTT gpio
    gpio_set_level(Trans, 0);

    // Wait time
    vTaskDelay(pdMS_TO_TICKS(1000));

    // Store value of action flag in a previous flag 
    bit_prev_flag = bit_action_flag;

    // Reset the bit action flag
    bit_action_flag = -1;
        
    // Resume the TaskBitMonitor 
    vTaskResume(xTaskBitMonitor);
}

void app_main() {
    // Intialize everything we need
    every_int();
    init_uart();

    //Create task to monitor bits, defined xTaskBitMonitor
    xTaskCreate(vTaskMonitorBits, "Bit Monitor Task", 2048, NULL, 5, &xTaskBitMonitor);
    
    while (1) {

        // Checks if bit action is needed
        if(bit_action_flag != -1){
            num_actions(bit_action_flag);
        }

        // Delay between each reading of the bit action flag
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
    

}
