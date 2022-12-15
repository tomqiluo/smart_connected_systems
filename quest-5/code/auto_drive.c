#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_attr.h"

#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"

//STD C library
#include <string.h>
#include <stdlib.h>
#include <sys/param.h>
#include <math.h>

#include "esp_log.h"
//#include "led_strip.h"
#include "sdkconfig.h"
#include "driver/uart.h"
#include "esp_vfs_dev.h"

//RTOS
#include "freertos/event_groups.h"
#include "esp_system.h"

//GPIO task
#include "driver/gpio.h"

#include "driver/i2c.h"

#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "driver/i2c.h"

//You can get these value from the datasheet of servo you use, in general pulse width varies between 1000 to 2000 mocrosecond
#define SERVO_MIN_PULSEWIDTH 1000 //Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH 2000 //Maximum pulse width in microsecond
#define SERVO_MAX_DEGREE 45 //Maximum angle in degree upto which servo can rotate

#define WRITE_BIT                          I2C_MASTER_WRITE // i2c master write
#define READ_BIT                           I2C_MASTER_READ  // i2c master read
#define I2C_EXAMPLE_MASTER_SCL_IO          22   // gpio number for i2c clk
#define I2C_EXAMPLE_MASTER_SDA_IO          23   // gpio number for i2c data
#define I2C_EXAMPLE_MASTER_NUM             I2C_NUM_0  // i2c port
#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE  0    // i2c master no buffer needed
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE  0    // i2c master no buffer needed
#define I2C_EXAMPLE_MASTER_FREQ_HZ         400000     // i2c master clock freq
#define ACK_CHECK_EN 0x1            /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0           /*!< I2C master will not check ack from slave */

#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   64          //Multisampling

static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t channel = ADC_CHANNEL_6; 
static const adc_atten_t atten = ADC_ATTEN_DB_11;
static const adc_unit_t unit = ADC_UNIT_1;

float distance_m = 0.0;
float distance_m1 = 0.0;
float distance_m2 = 0.0;
int emergency_stop = 0;

static void mcpwm_example_gpio_initialize(void)
{
    printf("initializing mcpwm servo control gpio......\n");
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, 15);    //Steering servo
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, 21);     //ESC servo
}

void calibrateESC() {
  printf("TURN POWER ON\n");
  vTaskDelay(3000 / portTICK_PERIOD_MS);  // Give yourself time to turn on crawler (3s)
  printf("START CALIBRATING\n");
  mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 1400); // NEUTRAL signal in microseconds
  vTaskDelay(3000/ portTICK_PERIOD_MS); // Do for at least 3s, and leave in neutral state
  printf("FINISH CALIBRATING\n");
}

/*
 * @brief Use this function to calcute pulse width for per degree rotation
 *
 * @param  degree_of_rotation the angle in degree to which servo has to rotate
 *
 * @return
 *     - calculated pulse width
 */
static uint32_t servo_per_degree_init(uint32_t degree_of_rotation)
{
    uint32_t cal_pulsewidth = 0;
    cal_pulsewidth = (SERVO_MIN_PULSEWIDTH + (((SERVO_MAX_PULSEWIDTH - SERVO_MIN_PULSEWIDTH) * (degree_of_rotation)) / (SERVO_MAX_DEGREE)));
    return cal_pulsewidth;
}

// Utility  Functions //////////////////////////////////////////////////////////

// // Utility function to test for I2C device address -- not used in deploy
int testConnection(uint8_t devAddr, int32_t timeout) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (devAddr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
  i2c_master_stop(cmd);
  int err = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  return err;
}
// Utility function to scan for i2c device
static void i2c_scanner() {
  int32_t scanTimeout = 1000;
  printf("\n>> I2C scanning ..."  "\n");
  uint8_t count = 0;
  for (uint8_t i = 1; i < 127; i++) {
    if (testConnection(i, scanTimeout) == ESP_OK) {
      printf( "- Device found at address: 0x%X%s", i, "\n");
      count++;
    }
  }
  if (count == 0) {printf("- No I2C devices found!" "\n");}
}
//i2c init from Master init
static void i2c_master_init(){
  // Debug
  printf("\n>> i2c Config\n");
  int err;

  // Port configuration
  int i2c_master_port = I2C_EXAMPLE_MASTER_NUM;

  /// Define I2C configurations
  i2c_config_t conf;
  conf.mode = I2C_MODE_MASTER;                              // Master mode
  conf.sda_io_num = I2C_EXAMPLE_MASTER_SDA_IO;              // Default SDA pin
  conf.sda_pullup_en = GPIO_PULLUP_ENABLE;                  // Internal pullup
  conf.scl_io_num = I2C_EXAMPLE_MASTER_SCL_IO;              // Default SCL pin
  conf.scl_pullup_en = GPIO_PULLUP_ENABLE;                  // Internal pullup
  conf.master.clk_speed = I2C_EXAMPLE_MASTER_FREQ_HZ;       // CLK frequency
  conf.clk_flags = 0;
  err = i2c_param_config(i2c_master_port, &conf);           // Configure
  if (err == ESP_OK) {printf("- parameters: ok\n");}

  // Install I2C driver
  err = i2c_driver_install(i2c_master_port, conf.mode,
                     I2C_EXAMPLE_MASTER_RX_BUF_DISABLE,
                     I2C_EXAMPLE_MASTER_TX_BUF_DISABLE, 0);
  if (err == ESP_OK) {printf("- initialized: yes\n");}

  // Data in MSB mode
  i2c_set_data_mode(i2c_master_port, I2C_DATA_MODE_MSB_FIRST, I2C_DATA_MODE_MSB_FIRST);
}

////////////////////////////////////////////////////////////////////////////////
// Read and write to register Functions ///////////////////////////////////////////////////////////

int writeI2C(uint8_t slaveAddr, uint8_t reg, uint8_t *data, uint8_t bytes)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (slaveAddr << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_write(cmd, data, bytes, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}
int readI2C(uint8_t slaveAddr, uint8_t reg, uint8_t *buffer, uint8_t bytes)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (slaveAddr << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (slaveAddr << 1) | READ_BIT, ACK_CHECK_EN);
    if (bytes > 1)
    {
        i2c_master_read(cmd, buffer, bytes - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, buffer + bytes - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    int ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

// Write one byte to register (single byte write)
void writeRegister(uint8_t reg, uint8_t data) {
  // create i2c communication init
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);	// 1. Start (Master write start)
  i2c_master_write_byte(cmd, ( 0x62 << 1 ) | WRITE_BIT, I2C_MASTER_ACK); // (Master write slave add + write bit)
  // wait for salve to ack
  i2c_master_write_byte(cmd, reg, I2C_MASTER_ACK); // (Master write register address)
  // wait for slave to ack
  i2c_master_write_byte(cmd, data, I2C_MASTER_ACK);// master write data 
  // wait for slave to ack
  i2c_master_stop(cmd); // 11. Stop
  // i2c communication done and delete
  i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  // no return here since data is written by master onto slave
}

// Write one byte to register (single byte write)
void writeRegister1(uint8_t reg, uint8_t data) {
  // create i2c communication init
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);	// 1. Start (Master write start)
  i2c_master_write_byte(cmd, ( 0x63 << 1 ) | WRITE_BIT, I2C_MASTER_ACK); // (Master write slave add + write bit)
  // wait for salve to ack
  i2c_master_write_byte(cmd, reg, I2C_MASTER_ACK); // (Master write register address)
  // wait for slave to ack
  i2c_master_write_byte(cmd, data, I2C_MASTER_ACK);// master write data 
  // wait for slave to ack
  i2c_master_stop(cmd); // 11. Stop
  // i2c communication done and delete
  i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  // no return here since data is written by master onto slave
}

// Write one byte to register (single byte write)
void writeRegister2(uint8_t reg, uint8_t data) {
  // create i2c communication init
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);	// 1. Start (Master write start)
  i2c_master_write_byte(cmd, ( 0x64 << 1 ) | WRITE_BIT, I2C_MASTER_ACK); // (Master write slave add + write bit)
  // wait for salve to ack
  i2c_master_write_byte(cmd, reg, I2C_MASTER_ACK); // (Master write register address)
  // wait for slave to ack
  i2c_master_write_byte(cmd, data, I2C_MASTER_ACK);// master write data 
  // wait for slave to ack
  i2c_master_stop(cmd); // 11. Stop
  // i2c communication done and delete
  i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  // no return here since data is written by master onto slave
}

// // Read register (single byte read)
// uint16_t readRegister(uint8_t reg) {
//   uint8_t data1; //first byte MSB
//   uint8_t data2; //second byte LSB

//   i2c_cmd_handle_t cmd = i2c_cmd_link_create();
//   i2c_cmd_handle_t cmd1 = i2c_cmd_link_create();

//   // Start
//   i2c_master_start(cmd);
//   // Master write slave address + write bit
//   i2c_master_write_byte(cmd, ( LIDARLite_ADDRESS << 1 ) | WRITE_BIT, I2C_MASTER_ACK); 
//   // Master write register address + send ack
//   i2c_master_write_byte(cmd, reg, I2C_MASTER_ACK); 
//   //master stops
//   i2c_master_stop(cmd);
//   // This starts the I2C communication
//   i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS); 
//   i2c_cmd_link_delete(cmd);

//   //master starts
//   i2c_master_start(cmd1);
//   // Master write slave address + read bit
//   i2c_master_write_byte(cmd1, ( LIDARLite_ADDRESS << 1 ) | READ_BIT, I2C_MASTER_ACK);  
//   // Master reads in slave ack and data
//   i2c_master_read_byte(cmd1, &data1 , I2C_MASTER_ACK);
//   i2c_master_read_byte(cmd1, &data2 , I2C_MASTER_NACK);
//   // Master nacks and stops.
//   i2c_master_stop(cmd1);
//   // This starts the I2C communication
//   i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd1, 1000 / portTICK_RATE_MS); 
//   i2c_cmd_link_delete(cmd1);
  

//   uint16_t two_byte_data = (data1 << 8 | data2);
//   return two_byte_data;
// }

static void lidar(){
    printf("\n>> Polling Lidar\n");
    int steer = 0;
    while (1) {
        //write to register 0x00 the value 0x04
        writeRegister(0x00, 0x04);
        writeRegister1(0x00, 0x04);
        writeRegister2(0x00, 0x04);
        //writeI2C(0x62, 0x04, 0x00,1);
        //READ REGISTER 0X01 UNTIL LSB GOES LOW 
        //if LSB goes low then set flag to true
        int flag = 1;
        int flag1 = 1;
        int flag2 = 1;
        while(flag && flag1 && flag2){
            //uint16_t data = readRegister(0x01);
            uint8_t data[2];
            readI2C(0x62,0x01,data,2);
            uint8_t dataBytes[2];
            readI2C(0x63,0x01,dataBytes,2);
            uint8_t dataBytes1[2];
            readI2C(0x64,0x01, dataBytes1,2);
            // printf("DATA: %d\n", data);
            flag = *data & (1<<15);
            flag1 = *dataBytes & (1<<15);
            flag2 = *dataBytes1 & (1<<15);
            vTaskDelay(5);
        }


        //uint16_t distance = readRegister(RegisterHighLowB);
        uint8_t distance0[2];
        readI2C(0x62, 0x8f, distance0, 2);
        uint8_t distance1[2];
        readI2C(0x63, 0x10, distance1, 2);
        uint8_t distance2[2];
        readI2C(0x64, 0x10, distance2, 2);
        distance_m = (float)*distance0/100.0;
        distance_m1 = (float)*distance1/100.0;
        distance_m2 = (float)*distance2/100.0;
        // printf("Distance: %.2f m\n", distance_m);
        //printf("Distance1: %.2f m\n", distance_m1);
        //printf("Distance2: %.2f m\n", distance_m2);

        float diff = distance_m1 - distance_m2;
        if(diff > 0.1){
          steer = steer + 1;
        }
        else if(diff < -0.1){
          steer = steer - 1;
        }
        else{
          steer = 0;
        }
    }
}

void init(void){
    //1. mcpwm gpio initialization
    mcpwm_example_gpio_initialize();

    //2. initial mcpwm configuration
    printf("Configuring Initial Parameters of mcpwm......\n");
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 50;    //frequency = 50Hz, i.e. for every servo motor time period should be 20ms
    pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);    //Configure PWM0A & PWM0B with above settings

    calibrateESC();
}

/*
 * @brief Configure MCPWM module
 */
void steering_servo_task(void *arg)
{
    uint32_t angle, count;
    int steer = 0;
    while (1) {
        float diff = distance_m1 - distance_m2;
        // if(diff > 0.1){
        //   steer = steer + 1;
        // }
        // else if(diff < -0.1){
        //   steer = steer - 1;
        // }
        // else{
        //   steer = 0;
        // }
        if(diff > 0.1){
            mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1800);
            //printf("左： %f, %f\n", distance_m1, distance_m2);
            //vTaskDelay(500/ portTICK_PERIOD_MS);
        }
        else if(diff <  -0.1){
            mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1000);
            //printf("右： %f, %f\n", distance_m1, distance_m2);
            //vTaskDelay(500/ portTICK_PERIOD_MS);
        }
        else{
            mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1440);
            //printf("中： %f, %f\n", distance_m1, distance_m2);
        }
        vTaskDelay(500/ portTICK_PERIOD_MS);
    }
}

static void check_efuse(void)
{
    //Check TP is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        printf("eFuse Two Point: Supported\n");
    } else {
        printf("eFuse Two Point: NOT supported\n");
    }

    //Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        printf("eFuse Vref: Supported\n");
    } else {
        printf("eFuse Vref: NOT supported\n");
    }
}

static void print_char_val_type(esp_adc_cal_value_t val_type)
{
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        printf("Characterized using Two Point Value\n");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        printf("Characterized using eFuse Vref\n");
    } else {
        printf("Characterized using Default Vref\n");
    }
}

void IR(void)
{
    //Check if Two Point or Vref are burned into eFuse
    check_efuse();

    //Configure ADC
    if (unit == ADC_UNIT_1) {
        adc1_config_width(ADC_WIDTH_BIT_12);
        adc1_config_channel_atten(channel, atten);
    } else {
        adc2_config_channel_atten((adc2_channel_t)channel, atten);
    }

    //Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
    print_char_val_type(val_type);

    /* Install UART driver for interrupt-driven reads and writes */
    ESP_ERROR_CHECK( uart_driver_install(UART_NUM_0,
      256, 0, 0, NULL, 0) );

    /* Tell VFS to use UART driver */
    esp_vfs_dev_uart_use_driver(UART_NUM_0);

    // i2c_example_master_init();
    // i2c_scanner();

    //xTaskCreate(test_alpha_display, "test_alpha_display", 4096, NULL, 5, NULL);

    //Continuously sample ADC1
    while (1) {
        uint32_t adc_reading = 0;
        //Multisampling
        for (int i = 0; i < NO_OF_SAMPLES; i++) {
            if (unit == ADC_UNIT_1) {
                adc_reading += adc1_get_raw((adc1_channel_t)channel);
            } else {
                int raw;
                adc2_get_raw((adc2_channel_t)channel, ADC_WIDTH_BIT_10, &raw);
                adc_reading += raw;
            }
        }
        adc_reading /= NO_OF_SAMPLES;
        //Convert adc_reading to voltage in mV
        float voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
        // voltage_display = adc_reading / 2400 * 500;
        float adc_display = 75.5534 - 70.8301*log(voltage/1000 - 0.4);
        if((adc_reading > 1700)){
          emergency_stop = 1;
          mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 1300);
          vTaskDelay(pdMS_TO_TICKS(1000));
          //printf("停： %d\n", adc_reading);
        }
        else{
          emergency_stop = 0;
          mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 1520);
          //printf("走： %d\n", adc_reading);
        }
        //printf("Raw: %d\tVoltage: %f mV\tDistance: %f cm\n", adc_reading, voltage, adc_display);
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

float dt = 0.1;
float previous_error = 0;
float integral = 0;
float Kp = 10;
float Ki = 5;
float Kd = 5;
int speed = 1550;

void decoder(void)
{
    int pcnt_unit = PCNT_UNIT_0;
    /* Initialize LEDC to generate sample pulse signal */
    ledc_init();

    /* Initialize PCNT event queue and PCNT functions */
    pcnt_evt_queue = xQueueCreate(10, sizeof(pcnt_evt_t));
    pcnt_example_init(pcnt_unit);

    int16_t count = 0;
    pcnt_evt_t evt;
    int p_counter = 0;
    int counter = 0;
    float velocity;

    portBASE_TYPE res;
    while (1) {
        /* Wait for the event information passed from PCNT's interrupt handler.
         * Once received, decode the event type and print it on the serial monitor.
         */
        res = xQueueReceive(pcnt_evt_queue, &evt, 100 / portTICK_PERIOD_MS);
        pcnt_get_counter_value(pcnt_unit, &count);
        if (count >= p_counter) {
            counter = count - p_counter;
            p_counter = count;
        } else {
            counter = 100 - p_counter + count;
            p_counter = count;
        }
        velocity = counter * 0.2;


        float error = 0.5-velocity;
        integral = integral + error * dt;
        float derivative = (error - previous_error) / dt;
        float correction = Kp * error + Ki * integral + Kd * derivative;
        previous_error = error;

        if(emergency_stop == 0){
            mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, speed + correction);
        }
        else{
            mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 1400);
        }
        

        ESP_LOGI(TAG, "Current PID :%f", correction);
        ESP_LOGI(TAG, "Current speed value :%f", velocity);
        
    }
}

void app_main(void)
{
    init();
    i2c_master_init();
    i2c_scanner();
    
    printf("Testing servo motor.......\n");
    //xTaskCreate(esc_servo_task, "esc_servo_task", 4096, NULL, 5, NULL);
    xTaskCreate(steering_servo_task, "steering_servo_task", 4096, NULL, 5, NULL);
    xTaskCreate(IR, "IR", 4096, NULL, 8, NULL);
    xTaskCreate(lidar, "lidar", 4096, NULL, 6, NULL);
}