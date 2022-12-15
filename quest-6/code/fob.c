#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_attr.h"

#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"

#include <string.h>

#include "esp_log.h"
#include "driver/rmt.h"
#include "soc/rmt_reg.h"
#include "driver/uart.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"

//You can get these value from the datasheet of servo you use, in general pulse width varies between 1000 to 2000 mocrosecond
#define SERVO_MIN_PULSEWIDTH 1000 //Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH 2000 //Maximum pulse width in microsecond
#define SERVO_MAX_DEGREE 45 //Maximum angle in degree upto which servo can rotate

// RMT definitions
#define RMT_TX_CHANNEL    1     // RMT channel for transmitter
#define RMT_TX_GPIO_NUM   25    // GPIO number for transmitter signal -- A1
#define RMT_CLK_DIV       100   // RMT counter clock divider
#define RMT_TICK_10_US    (80000000/RMT_CLK_DIV/100000)   // RMT counter value for 10 us.(Source clock is APB clock)
#define rmt_item32_tIMEOUT_US   9500     // RMT receiver timeout value(us)

// UART definitions
#define UART_TX_GPIO_NUM 26 // A0
#define UART_RX_GPIO_NUM 34 // A2
#define BUF_SIZE (1024)

// Hardware interrupt definitions
#define GPIO_INPUT_IO_1       4
#define ESP_INTR_FLAG_DEFAULT 0
#define GPIO_INPUT_PIN_SEL    1ULL<<GPIO_INPUT_IO_1

// LED Output pins definitions
#define BLUEPIN   32
#define GREENPIN  17
#define REDPIN    14
#define ONBOARD   13

#define TIMER_DIVIDER         16    //  Hardware timer clock divider
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)  // to seconds
#define TIMER_INTERVAL_2_SEC  (2)
#define TIMER_INTERVAL_10_SEC (10)
#define TEST_WITH_RELOAD      1     // Testing will be done with auto reload

// Default ID/color
#define ID 3
#define COLOR 'E'

// Variables for my ID, minVal and status plus string fragments
char start = 0x1B;
char myID = (char) ID;
char myColor = (char) COLOR;
int len_out = 4;

// Mutex (for resources), and Queues (for button)
SemaphoreHandle_t mux = NULL;
static xQueueHandle gpio_evt_queue = NULL;
static xQueueHandle timer_queue;

// A simple structure to pass "events" to main task
typedef struct {
    int flag;     // flag for enabling stuff in timer task
} timer_event_t;

// System tags
static const char *TAG_SYSTEM = "system";       // For debug logs

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
    while (1) {
        
        switch((int)myColor){
            case 'L' :
                printf("Left\n");
                mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1800);
                vTaskDelay(1000/ portTICK_PERIOD_MS);
                break;
            case 'R' :
                printf("Right\n");
                mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 500);
                vTaskDelay(1000/ portTICK_PERIOD_MS);
                break;
            default:
                printf("Center\n");
                mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1440);
                vTaskDelay(1000/ portTICK_PERIOD_MS);
                break;
        }
    }
}


void esc_servo_task(void *arg)
{
    
    while (1) {
        // mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 1500);
        // vTaskDelay(3000/ portTICK_PERIOD_MS);
        // mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 1500);
        // vTaskDelay(1000/ portTICK_PERIOD_MS);
        // mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 1300);
        // vTaskDelay(1000/ portTICK_PERIOD_MS);
        // mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 1400);
        // vTaskDelay(1000/ portTICK_PERIOD_MS);
        // mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 1300);
        // vTaskDelay(3000/ portTICK_PERIOD_MS);
        // mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 1400);
        // vTaskDelay(3000/ portTICK_PERIOD_MS);
        switch((int)myColor){
            case 'C' :
                printf("Do Trick\n");
                for (int i=0; i<3; i++) {
                    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 1400);
                    vTaskDelay(500/ portTICK_PERIOD_MS);
                    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 1300);
                    vTaskDelay(500/ portTICK_PERIOD_MS);
                    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 1400);
                    vTaskDelay(500/ portTICK_PERIOD_MS);
                    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 1300);
                    vTaskDelay(500/ portTICK_PERIOD_MS);
                    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 1400);
                    vTaskDelay(3000/ portTICK_PERIOD_MS);
                    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 1500);
                    vTaskDelay(3000/ portTICK_PERIOD_MS);
                }
                break;
            default:
                printf("Default.\n");
                mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 1500);
                vTaskDelay(3000/ portTICK_PERIOD_MS);
                break;
        }
    }
}

// Button interrupt handler -- add to queue
static void IRAM_ATTR gpio_isr_handler(void* arg){
  uint32_t gpio_num = (uint32_t) arg;
  xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

// ISR handler
// void IRAM_ATTR timer_group0_isr(void *para) {

//     // Prepare basic event data, aka set flag
//     timer_event_t evt;
//     evt.flag = 1;

//     // Yellow is shorter
//     if (myColor == 'G') {
//       timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, TIMER_INTERVAL_2_SEC * TIMER_SCALE);
//     }
//     else {
//       timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, TIMER_INTERVAL_10_SEC * TIMER_SCALE);
//     }

//     // Clear the interrupt, Timer 0 in group 0
//     TIMERG0.int_clr_timers.t0 = 1;

//     // After the alarm triggers, we need to re-enable it to trigger it next time
//     TIMERG0.hw_timer[TIMER_0].config.alarm_en = TIMER_ALARM_EN;

//     // Send the event data back to the main program task
//     xQueueSendFromISR(timer_queue, &evt, NULL);
// }

// Utilities ///////////////////////////////////////////////////////////////////

// Checksum
char genCheckSum(char *p, int len) {
  char temp = 0;
  for (int i = 0; i < len; i++){
    temp = temp^p[i];
  }
  // printf("%X\n",temp);

  return temp;
}
bool checkCheckSum(uint8_t *p, int len) {
  char temp = (char) 0;
  bool isValid;
  for (int i = 0; i < len-1; i++){
    temp = temp^p[i];
  }
  // printf("Check: %02X ", temp);
  if (temp == p[len-1]) {
    isValid = true; }
  else {
    isValid = false; }
  return isValid;
}

// Init Functions //////////////////////////////////////////////////////////////
// RMT tx init
static void rmt_tx_init() {
    rmt_config_t rmt_tx;
    rmt_tx.channel = RMT_TX_CHANNEL;
    rmt_tx.gpio_num = RMT_TX_GPIO_NUM;
    rmt_tx.mem_block_num = 1;
    rmt_tx.clk_div = RMT_CLK_DIV;
    rmt_tx.tx_config.loop_en = false;
    rmt_tx.tx_config.carrier_duty_percent = 50;
    // Carrier Frequency of the IR receiver
    rmt_tx.tx_config.carrier_freq_hz = 38000;
    rmt_tx.tx_config.carrier_level = 1;
    rmt_tx.tx_config.carrier_en = 1;
    // Never idle -> aka ontinuous TX of 38kHz pulses
    rmt_tx.tx_config.idle_level = 1;
    rmt_tx.tx_config.idle_output_en = true;
    rmt_tx.rmt_mode = 0;
    rmt_config(&rmt_tx);
    rmt_driver_install(rmt_tx.channel, 0, 0);
}

// Configure UART
static void uart_init() {
  // Basic configs
  uart_config_t uart_config = {
      .baud_rate = 1200, // Slow BAUD rate
      .data_bits = UART_DATA_8_BITS,
      .parity    = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
  };
  uart_param_config(UART_NUM_1, &uart_config);

  // Set UART pins using UART0 default pins
  uart_set_pin(UART_NUM_1, UART_TX_GPIO_NUM, UART_RX_GPIO_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

  // Reverse receive logic line
  uart_set_line_inverse(UART_NUM_1,UART_SIGNAL_RXD_INV);

  // Install UART driver
  uart_driver_install(UART_NUM_1, BUF_SIZE * 2, 0, 0, NULL, 0);
}

// GPIO init for LEDs
static void led_init() {
    gpio_pad_select_gpio(BLUEPIN);
    gpio_pad_select_gpio(GREENPIN);
    gpio_pad_select_gpio(REDPIN);
    gpio_pad_select_gpio(ONBOARD);
    gpio_set_direction(BLUEPIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(GREENPIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(REDPIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(ONBOARD, GPIO_MODE_OUTPUT);
}

// Configure timer
// static void alarm_init() {
//     // Select and initialize basic parameters of the timer
//     timer_config_t config;
//     config.divider = TIMER_DIVIDER;
//     config.counter_dir = TIMER_COUNT_UP;
//     config.counter_en = TIMER_PAUSE;
//     config.alarm_en = TIMER_ALARM_EN;
//     config.intr_type = TIMER_INTR_LEVEL;
//     config.auto_reload = TEST_WITH_RELOAD;
//     timer_init(TIMER_GROUP_0, TIMER_0, &config);

//     // Timer's counter will initially start from value below
//     timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0x00000000ULL);

//     // Configure the alarm value and the interrupt on alarm
//     timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, TIMER_INTERVAL_10_SEC * TIMER_SCALE);
//     timer_enable_intr(TIMER_GROUP_0, TIMER_0);
//     timer_isr_register(TIMER_GROUP_0, TIMER_0, timer_group0_isr,
//         (void *) TIMER_0, ESP_INTR_FLAG_IRAM, NULL);

//     // Start timer
//     timer_start(TIMER_GROUP_0, TIMER_0);
// }

// Button interrupt init
static void button_init() {
    gpio_config_t io_conf;
    //interrupt of rising edge
    io_conf.intr_type = GPIO_PIN_INTR_POSEDGE;
    //bit mask of the pins, use GPIO4 here
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);
    gpio_intr_enable(GPIO_INPUT_IO_1 );
    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_LEVEL3);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_IO_1, gpio_isr_handler, (void*) GPIO_INPUT_IO_1);
    //create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    //start gpio task
}

////////////////////////////////////////////////////////////////////////////////

// Tasks ///////////////////////////////////////////////////////////////////////
// Button task -- rotate through myIDs
void button_task(){
  uint32_t io_num;
  while(1) {
    if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
      xSemaphoreTake(mux, portMAX_DELAY);
      if (myID == 3) {
        myID = 1;
      }
      else {
        myID++;
      }
      xSemaphoreGive(mux);
      printf("Button pressed.\n");
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }

}

// Send task -- sends payload | Start | myID | Start | myID
void send_task(){
  while(1) {

    char *data_out = (char *) malloc(len_out);

    xSemaphoreTake(mux, portMAX_DELAY);
    data_out[0] = start;
    data_out[1] = (char) myColor;
    data_out[2] = (char) myID;
    data_out[3] = genCheckSum(data_out,len_out-1);
    // ESP_LOG_BUFFER_HEXDUMP(TAG_SYSTEM, data_out, len_out, ESP_LOG_INFO);

    uart_write_bytes(UART_NUM_1, data_out, len_out);
    xSemaphoreGive(mux);

    vTaskDelay(5 / portTICK_PERIOD_MS);
  }
}

// Receives task -- looks for Start byte then stores received values
void recv_task(){
  // Buffer for input data
  uint8_t *data_in = (uint8_t *) malloc(BUF_SIZE);
  while (1) {
    int len_in = uart_read_bytes(UART_NUM_1, data_in, BUF_SIZE, 20 / portTICK_RATE_MS);
    if (len_in >0) {
      if (data_in[0] == start) {
        if (checkCheckSum(data_in,len_out)) {
          // ESP_LOG_BUFFER_HEXDUMP(TAG_SYSTEM, data_in, len_out, ESP_LOG_INFO);
          printf("data_in = %c\n", data_in[1]);
          myColor = data_in[1];
        }
      }
    }
    else{
    //  printf("Nothing received.\n");
    }
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
  free(data_in);
}

// LED task to light LED based on traffic state
void led_task(){
  while(1) {
    switch((int)myColor){
      case 'D' : // Red
        gpio_set_level(GREENPIN, 0);
        gpio_set_level(REDPIN, 1);
        gpio_set_level(BLUEPIN, 0);
        // printf("Current state: %c\n",status);
        break;
    //   case 'Y' : // Yellow
    //     gpio_set_level(GREENPIN, 1);
    //     gpio_set_level(REDPIN, 1);
    //     gpio_set_level(BLUEPIN, 0);
    //     // printf("Current state: %c\n",status);
    //     break;
    //   case 'G' : // Green
    //     gpio_set_level(GREENPIN, 1);
    //     gpio_set_level(REDPIN, 0);
    //     gpio_set_level(BLUEPIN, 0);
    //     // printf("Current state: %c\n",status);
    //     break;
        default:
            gpio_set_level(GREENPIN, 0);
            gpio_set_level(REDPIN, 0);
            gpio_set_level(BLUEPIN, 0);

    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

// LED task to blink onboard LED based on ID
void id_task(){
  while(1) {
    for (int i = 0; i < (int) myID; i++) {
      gpio_set_level(ONBOARD,1);
      vTaskDelay(200 / portTICK_PERIOD_MS);
      gpio_set_level(ONBOARD,0);
      vTaskDelay(200 / portTICK_PERIOD_MS);
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

// Timer task -- R (10 seconds), G (10 seconds), Y (2 seconds)
// static void timer_evt_task(void *arg) {
//     while (1) {
//         // Create dummy structure to store structure from queue
//         timer_event_t evt;

//         // Transfer from queue
//         xQueueReceive(timer_queue, &evt, portMAX_DELAY);

//         // Do something if triggered!
//         if (evt.flag == 1) {
//             // printf("Action!\n");
//             if (myColor == 'R') {
//               // printf("Color = G\n");
//               myColor = 'G';
//             }
//             else if (myColor == 'G') {
//               // printf("Color = Y\n");
//               myColor = 'Y';
//             }
//             else if (myColor == 'Y') {
//               // printf("Color = R\n");
//               myColor = 'R';
//             }
//         }
//     }
// }

void app_main(void)
{
    init();

    // Mutex for current values when sending
    mux = xSemaphoreCreateMutex();

    // Create a FIFO queue for timer-based events
    timer_queue = xQueueCreate(10, sizeof(timer_event_t));
    
    printf("Testing servo motor.......\n");
    xTaskCreate(esc_servo_task, "esc_servo_task", 4096, NULL, 5, NULL);
    xTaskCreate(steering_servo_task, "steering_servo_task", 4096, NULL, 5, NULL);

    // Create task to handle timer-based events
    // xTaskCreate(timer_evt_task, "timer_evt_task", 2048, NULL, 5, NULL);

    // Initialize all the things
    rmt_tx_init();
    uart_init();
    led_init();
    // alarm_init();
    button_init();

    // Create tasks for receive, send, set gpio, and button
    xTaskCreate(recv_task, "uart_rx_task", 1024*4, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(send_task, "uart_tx_task", 1024*2, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(led_task, "set_traffic_task", 1024*2, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(id_task, "set_id_task", 1024*2, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(button_task, "button_task", 1024*2, NULL, configMAX_PRIORITIES, NULL);
}