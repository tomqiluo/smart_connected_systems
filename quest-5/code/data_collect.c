#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
#include <stdio.h>
#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_attr.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"
#include "freertos/queue.h"
#include "driver/pcnt.h"
#include "esp_types.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"
#include <math.h>
#include "driver/i2c.h"
#include "./ADXL343.h"

#include "freertos/event_groups.h"
#include "addr_from_stdin.h"

#include <stdlib.h>
#include "sdkconfig.h"
#include "driver/uart.h"
#include "esp_vfs_dev.h"
#include "driver/gpio.h"

static const char *TAG = "multi_sensor";
static const char *payload = "Message from ESP32 ";

#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO          (21) // Define the output GPIO
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY               (1000) // Set duty to 50%. ((2 ** 13) - 1) * 50% = 4095
#define LEDC_FREQUENCY          (5000) // Frequency in Hertz. Set frequency at 5 kHz

#define PORT 5000

#define PCNT_H_LIM_VAL      100
#define PCNT_L_LIM_VAL     -10
#define PCNT_THRESH1_VAL    5
#define PCNT_THRESH0_VAL   -5
#define PCNT_INPUT_SIG_IO   27  // Pulse Input GPIO
#define PCNT_INPUT_CTRL_IO  5  // Control GPIO HIGH=count up, LOW=count down
//#define LEDC_OUTPUT_IO      4 // Output GPIO of a sample 1 Hz pulse generator

#define TIMER_DIVIDER         16    //  Hardware timer clock divider
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)  // to seconds
#define TIMER_INTERVAL_SEC   (0.1)    // Sample test interval for the first timer
#define TEST_WITH_RELOAD      1     // Testing will be done with auto reload

#define SERVO_MIN_PULSEWIDTH 1000 //Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH 2000 //Maximum pulse width in microsecond
#define SERVO_MAX_DEGREE 45 //Maximum angle in degree upto which servo can rotate

// Master I2C
#define I2C_EXAMPLE_MASTER_SCL_IO          22   // gpio number for i2c clk
#define I2C_EXAMPLE_MASTER_SDA_IO          23   // gpio number for i2c data
#define I2C_EXAMPLE_MASTER_NUM             I2C_NUM_0  // i2c port
#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE  0    // i2c master no buffer needed
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE  0    // i2c master no buffer needed
#define I2C_EXAMPLE_MASTER_FREQ_HZ         100000     // i2c master clock freq
#define WRITE_BIT                          I2C_MASTER_WRITE // i2c master write
#define READ_BIT                           I2C_MASTER_READ  // i2c master read
#define ACK_CHECK_EN                       true // i2c master will check ack
#define ACK_CHECK_DIS                      false// i2c master will not check ack
#define ACK_VAL                            0x00 // i2c ack value
#define NACK_VAL                           0xFF // i2c nack value

// ADXL343
#define SLAVE_ADDR                         ADXL343_ADDRESS // 0x53

int num = 0;
int num0 = 0;
int num1 = 0;
char outstring[80];
float left_lidar = 0;
float right_lidar = 0;

char start_str[] = "start";

xQueueHandle pcnt_evt_queue;   // A queue to handle pulse counter events

#define HOST_IP_ADDR "192.168.1.114"

#define PORT 6000 //CONFIG_EXAMPLE_PORT

/* A sample structure to pass events from the PCNT
 * interrupt handler to the main program.
 */
typedef struct {
    int unit;  // the PCNT unit that originated an interrupt
    uint32_t status; // information on the event type that caused the interrupt
} pcnt_evt_t;

// A simple structure to pass "events" to main task
typedef struct {
    int flag;     // flag for enabling stuff in main code
} timer_event_t;

// Initialize queue handler for timer-based events
xQueueHandle timer_queue;

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

void steering_servo_task(void *arg)
{
    //uint32_t angle, count;
    while (1) {
        if(num1 == 2){
            printf("Left\n");
            mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 2000);
            vTaskDelay(500/ portTICK_PERIOD_MS);
            printf("Center\n");
            mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1440);
            num1 = 0;
        }
        else if (num1 == 3){
            printf("Right\n");
            mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 800);
            vTaskDelay(500/ portTICK_PERIOD_MS);
            printf("Center\n");
            mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1440);
            num1 = 0;
        }
    }
}

void esc_servo_task(void *arg)
{
    while (1) {
        if(num0 == 1){
            printf("start\n");
            mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 1500);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
        else if (num0 == 0){
            mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 1400);
            vTaskDelay(1000/ portTICK_PERIOD_MS);
        }
        // mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 1500);
        // vTaskDelay(3000/ portTICK_PERIOD_MS);
        // mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 1400);
        // vTaskDelay(1000/ portTICK_PERIOD_MS);
        // mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 1300);
        // vTaskDelay(1000/ portTICK_PERIOD_MS);
        // mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 1400);
        // vTaskDelay(1000/ portTICK_PERIOD_MS);
        // mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 1300);
        // vTaskDelay(3000/ portTICK_PERIOD_MS);
        // mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 1400);
        // vTaskDelay(3000/ portTICK_PERIOD_MS);
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

static void IRAM_ATTR pcnt_example_intr_handler(void *arg)
{
    int pcnt_unit = (int)arg;
    pcnt_evt_t evt;
    evt.unit = pcnt_unit;
    /* Save the PCNT event type that caused an interrupt
       to pass it to the main program */
    pcnt_get_event_status(pcnt_unit, &evt.status);
    xQueueSendFromISR(pcnt_evt_queue, &evt, NULL);
}

/* Initialize PCNT functions:
 *  - configure and initialize PCNT
 *  - set up the input filter
 *  - set up the counter events to watch
 */
static void pcnt_example_init(int unit)
{
    /* Prepare configuration for the PCNT unit */
    pcnt_config_t pcnt_config = {
        // Set PCNT input signal and control GPIOs
        .pulse_gpio_num = PCNT_INPUT_SIG_IO,
        .ctrl_gpio_num = PCNT_INPUT_CTRL_IO,
        .channel = PCNT_CHANNEL_0,
        .unit = unit,
        // What to do on the positive / negative edge of pulse input?
        .pos_mode = PCNT_COUNT_INC,   // Count up on the positive edge
        .neg_mode = PCNT_COUNT_DIS,   // Keep the counter value on the negative edge
        // What to do when control input is low or high?
        .lctrl_mode = PCNT_MODE_REVERSE, // Reverse counting direction if low
        .hctrl_mode = PCNT_MODE_KEEP,    // Keep the primary counter mode if high
        // Set the maximum and minimum limit values to watch
        .counter_h_lim = PCNT_H_LIM_VAL,
        .counter_l_lim = PCNT_L_LIM_VAL,
    };
    /* Initialize PCNT unit */
    pcnt_unit_config(&pcnt_config);

    /* Configure and enable the input filter */
    pcnt_set_filter_value(unit, 100);
    pcnt_filter_enable(unit);

    /* Set threshold 0 and 1 values and enable events to watch */
    pcnt_set_event_value(unit, PCNT_EVT_THRES_1, PCNT_THRESH1_VAL);
    pcnt_event_enable(unit, PCNT_EVT_THRES_1);
    pcnt_set_event_value(unit, PCNT_EVT_THRES_0, PCNT_THRESH0_VAL);
    pcnt_event_enable(unit, PCNT_EVT_THRES_0);
    /* Enable events on zero, maximum and minimum limit values */
    pcnt_event_enable(unit, PCNT_EVT_ZERO);
    pcnt_event_enable(unit, PCNT_EVT_H_LIM);
    pcnt_event_enable(unit, PCNT_EVT_L_LIM);

    /* Initialize PCNT's counter */
    pcnt_counter_pause(unit);
    pcnt_counter_clear(unit);

    /* Install interrupt service and add isr callback handler */
    pcnt_isr_service_install(0);
    pcnt_isr_handler_add(unit, pcnt_example_intr_handler, (void *)unit);

    /* Everything is set up, now go to counting */
    pcnt_counter_resume(unit);
}

// ISR handler
void IRAM_ATTR timer_group0_isr(void *para) {

    // Prepare basic event data, aka set flag
    timer_event_t evt;
    evt.flag = 1;

    // Clear the interrupt, Timer 0 in group 0
    TIMERG0.int_clr_timers.t0 = 1;

    // After the alarm triggers, we need to re-enable it to trigger it next time
    TIMERG0.hw_timer[TIMER_0].config.alarm_en = TIMER_ALARM_EN;

    // Send the event data back to the main program task
    xQueueSendFromISR(timer_queue, &evt, NULL);
}

static void ledc_init(void)
{
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer;
    ledc_timer.speed_mode       = LEDC_LOW_SPEED_MODE;
    ledc_timer.timer_num        = LEDC_TIMER_1;
    ledc_timer.duty_resolution  = LEDC_TIMER_10_BIT;
    ledc_timer.freq_hz          = 1;  // set output frequency at 1 Hz
    ledc_timer.clk_cfg = LEDC_AUTO_CLK;
    ledc_timer_config(&ledc_timer);

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel;
    ledc_channel.speed_mode = LEDC_LOW_SPEED_MODE;
    ledc_channel.channel    = LEDC_CHANNEL_1;
    ledc_channel.timer_sel  = LEDC_TIMER_1;
    ledc_channel.intr_type  = LEDC_INTR_DISABLE;
    ledc_channel.gpio_num   = LEDC_OUTPUT_IO;
    ledc_channel.duty       = 100; // set duty at about 10%
    ledc_channel.hpoint     = 0;
    ledc_channel_config(&ledc_channel);
}

static void udp_server_task(void *pvParameters)
{
    char rx_buffer[128];
    char addr_str[128];
    int addr_family = (int)pvParameters;
    int ip_protocol = 0;
    struct sockaddr_in6 dest_addr;

    while (1) {

        if (addr_family == AF_INET) {
            struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
            dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
            dest_addr_ip4->sin_family = AF_INET;
            dest_addr_ip4->sin_port = htons(PORT);
            ip_protocol = IPPROTO_IP;
        } else if (addr_family == AF_INET6) {
            bzero(&dest_addr.sin6_addr.un, sizeof(dest_addr.sin6_addr.un));
            dest_addr.sin6_family = AF_INET6;
            dest_addr.sin6_port = htons(PORT);
            ip_protocol = IPPROTO_IPV6;
        }

        int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket created");

#if defined(CONFIG_LWIP_NETBUF_RECVINFO) && !defined(CONFIG_EXAMPLE_IPV6)
        int enable = 1;
        lwip_setsockopt(sock, IPPROTO_IP, IP_PKTINFO, &enable, sizeof(enable));
#endif

#if defined(CONFIG_EXAMPLE_IPV4) && defined(CONFIG_EXAMPLE_IPV6)
        if (addr_family == AF_INET6) {
            // Note that by default IPV6 binds to both protocols, it is must be disabled
            // if both protocols used at the same time (used in CI)
            int opt = 1;
            setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
            setsockopt(sock, IPPROTO_IPV6, IPV6_V6ONLY, &opt, sizeof(opt));
        }
#endif
        // Set timeout
        struct timeval timeout;
        timeout.tv_sec = 10;
        timeout.tv_usec = 0;
        setsockopt (sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof timeout);

        int err = bind(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (err < 0) {
            ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        }
        ESP_LOGI(TAG, "Socket bound, port %d", PORT);

        struct sockaddr_storage source_addr; // Large enough for both IPv4 or IPv6
        socklen_t socklen = sizeof(source_addr);

#if defined(CONFIG_LWIP_NETBUF_RECVINFO) && !defined(CONFIG_EXAMPLE_IPV6)
        struct iovec iov;
        struct msghdr msg;
        struct cmsghdr *cmsgtmp;
        u8_t cmsg_buf[CMSG_SPACE(sizeof(struct in_pktinfo))];

        iov.iov_base = rx_buffer;
        iov.iov_len = sizeof(rx_buffer);
        msg.msg_control = cmsg_buf;
        msg.msg_controllen = sizeof(cmsg_buf);
        msg.msg_flags = 0;
        msg.msg_iov = &iov;
        msg.msg_iovlen = 1;
        msg.msg_name = (struct sockaddr *)&source_addr;
        msg.msg_namelen = socklen;
#endif

        while (1) {
            ESP_LOGI(TAG, "Waiting for data");
#if defined(CONFIG_LWIP_NETBUF_RECVINFO) && !defined(CONFIG_EXAMPLE_IPV6)
            int len = recvmsg(sock, &msg, 0);
#else
            int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);
#endif
            // Error occurred during receiving
            if (len < 0) {
                ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
                break;
            }
            // Data received
            else {
                // Get the sender's ip address as string
                if (source_addr.ss_family == PF_INET) {
                    inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr, addr_str, sizeof(addr_str) - 1);
#if defined(CONFIG_LWIP_NETBUF_RECVINFO) && !defined(CONFIG_EXAMPLE_IPV6)
                    for ( cmsgtmp = CMSG_FIRSTHDR(&msg); cmsgtmp != NULL; cmsgtmp = CMSG_NXTHDR(&msg, cmsgtmp) ) {
                        if ( cmsgtmp->cmsg_level == IPPROTO_IP && cmsgtmp->cmsg_type == IP_PKTINFO ) {
                            struct in_pktinfo *pktinfo;
                            pktinfo = (struct in_pktinfo*)CMSG_DATA(cmsgtmp);
                            ESP_LOGI(TAG, "dest ip: %s\n", inet_ntoa(pktinfo->ipi_addr));
                        }
                    }
#endif
                } else if (source_addr.ss_family == PF_INET6) {
                    inet6_ntoa_r(((struct sockaddr_in6 *)&source_addr)->sin6_addr, addr_str, sizeof(addr_str) - 1);
                }

                rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string...
                ESP_LOGI(TAG, "Received %d bytes from %s:", len, addr_str);
                ESP_LOGI(TAG, "%s", rx_buffer);
                num = atoi(rx_buffer);
                if (num == 0 || num == 1) {
                    num0 = num;
                } else if (num == 2 || num == 3) {
                    num1 = num;
                }

                // if(strcmp(rx_buffer, start_str)){
                //     num = 1;
                // }
                // else{
                //     num = 0;
                // }

                int err = sendto(sock, rx_buffer, len, 0, (struct sockaddr *)&source_addr, sizeof(source_addr));
                if (err < 0) {
                    ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                    break;
                }
            }
        }

        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    }
    vTaskDelete(NULL);
}

///////////////////////
//decoder
// void decoder(void)
// {
//     int pcnt_unit = PCNT_UNIT_0;
//     /* Initialize LEDC to generate sample pulse signal */
//     //ledc_init();

//     /* Initialize PCNT event queue and PCNT functions */
//     pcnt_evt_queue = xQueueCreate(10, sizeof(pcnt_evt_t));
//     pcnt_example_init(pcnt_unit);

//     int16_t count = 0;
//     pcnt_evt_t evt;
//     int p_counter = 0;
//     int counter = 0;
//     float velocity;

//     portBASE_TYPE res;
//     while (1) {
//         // Create dummy structure to store structure from queue
//         timer_event_t evt;

//         // Transfer from queue
//         xQueueReceive(timer_queue, &evt, portMAX_DELAY);

//         // Do something if triggered!
//         if (evt.flag == 1) {
//             /* Wait for the event information passed from PCNT's interrupt handler.
//             * Once received, decode the event type and print it on the serial monitor.
//             */
            
//             pcnt_get_counter_value(pcnt_unit, &count);
//             if (count >= p_counter) {
//                 counter = count - p_counter;
//                 p_counter = count;
//             } else {
//                 counter = 100 - p_counter + count;
//                 p_counter = count;
//             }
//             velocity = counter * 0.2;
//             // ESP_LOGI(TAG, "Current speed value :%f", velocity);
//             printf("decoder count: %d\t decoder speed: %f\n", counter, velocity);
//         }
//     }
// }
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
        //ESP_LOGI(TAG, "Current speed value :%f", velocity);
        printf("%.2f\n", velocity);
    }
}


// Initialize timer 0 in group 0 for 1 sec alarm interval and auto reload
static void alarm_init() {
    /* Select and initialize basic parameters of the timer */
    timer_config_t config;
    config.divider = TIMER_DIVIDER;
    config.counter_dir = TIMER_COUNT_UP;
    config.counter_en = TIMER_PAUSE;
    config.alarm_en = TIMER_ALARM_EN;
    config.intr_type = TIMER_INTR_LEVEL;
    config.auto_reload = TEST_WITH_RELOAD;
    timer_init(TIMER_GROUP_0, TIMER_0, &config);

    // Timer's counter will initially start from value below
    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0x00000000ULL);

    // Configure the alarm value and the interrupt on alarm
    timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, TIMER_INTERVAL_SEC * TIMER_SCALE);
    timer_enable_intr(TIMER_GROUP_0, TIMER_0);
    timer_isr_register(TIMER_GROUP_0, TIMER_0, timer_group0_isr,
        (void *) TIMER_0, ESP_INTR_FLAG_IRAM, NULL);

    // Start timer
    timer_start(TIMER_GROUP_0, TIMER_0);
}

// Function to initiate i2c -- note the MSB declaration!
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

// Utility  Functions //////////////////////////////////////////////////////////

// Utility function to test for I2C device address -- not used in deploy
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
    // printf("0x%X%s",i,"\n");
    if (testConnection(i, scanTimeout) == ESP_OK) {
      printf( "- Device found at address: 0x%X%s", i, "\n");
      count++;
    }
  }
  if (count == 0) {printf("- No I2C devices found!" "\n");}
}

// ADXL343 Functions ///////////////////////////////////////////////////////////

// Get Device ID
int getDeviceID(uint8_t *data) {
  int ret;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, ADXL343_REG_DEVID, ACK_CHECK_EN);
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | READ_BIT, ACK_CHECK_EN);
  i2c_master_read_byte(cmd, data, ACK_CHECK_DIS);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  return ret;
}

// Write one byte to register
void writeRegister(uint8_t reg, uint8_t data) {
  // create i2c communication init
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);	// 1. Start (Master write start)
  i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | WRITE_BIT, I2C_MASTER_ACK); // (Master write slave add + write bit)
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

// Read register
uint8_t readRegister(uint8_t reg) {
  uint8_t data;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  // start
  i2c_master_start(cmd);
  // Master write slave address + write bit
  i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
  // Master write register address + send ack
  i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
  // Start Master again
  i2c_master_start(cmd);
  // Master write register address + read bit
  i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | READ_BIT, ACK_CHECK_EN);
  // Master reads in the ack and data
  i2c_master_read_byte(cmd, &data, I2C_MASTER_ACK);
  // Master stop
  i2c_master_stop(cmd);
  // Starts the I2C communication
  i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS); 
  i2c_cmd_link_delete(cmd);

  return data;
}

// read 16 bits (2 bytes)
int16_t read16(uint8_t reg) {
  uint8_t data1 = readRegister(reg);
  uint8_t data2 = readRegister(reg+1);
  uint16_t two_byte_data = (data2 << 8 | data1);
  return two_byte_data;
}

void setRange(range_t range) {
  /* Red the data format register to preserve bits */
  uint8_t format = readRegister(ADXL343_REG_DATA_FORMAT);

  /* Update the data rate */
  format &= ~0x0F;
  format |= range;

  /* Make sure that the FULL-RES bit is enabled for range scaling */
  format |= 0x08;

  /* Write the register back to the IC */
  writeRegister(ADXL343_REG_DATA_FORMAT, format);

}

range_t getRange(void) {
  /* Red the data format register to preserve bits */
  return (range_t)(readRegister(ADXL343_REG_DATA_FORMAT) & 0x03);
}

dataRate_t getDataRate(void) {
  return (dataRate_t)(readRegister(ADXL343_REG_BW_RATE) & 0x0F);
}

////////////////////////////////////////////////////////////////////////////////


// function to get acceleration
void getAccel(float * xp, float *yp, float *zp) {
    *xp = read16(ADXL343_REG_DATAX0) * ADXL343_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;
    *yp = read16(ADXL343_REG_DATAY0) * ADXL343_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;
    *zp = read16(ADXL343_REG_DATAZ0) * ADXL343_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;
    // printf("X: %.2f \t Y: %.2f \t Z: %.2f\n", *xp, *yp, *zp);
}

// function to print roll and pitch
void calcRP(float x, float y, float z){
  float roll = 0.00, pitch = 0.00;       //Roll & Pitch are the angles which rotate by the axis X and y
  roll = atan2(y , z) * 57.3;
  pitch = atan2((- x) , sqrt(y * y + z * z)) * 57.3;
  printf("roll: %.2f \t pitch: %.2f \n", roll, pitch);
}



float p_velocity = 0.00;
float velocity = 0.00;

// Task to continuously poll acceleration and calculate roll and pitch
static void test_adxl343() {
  printf("\n>> Polling ADAXL343\n");
  while (1) {
    // Create dummy structure to store structure from queue
        timer_event_t evt;

        // Transfer from queue
        xQueueReceive(timer_queue, &evt, portMAX_DELAY);

        // Do something if triggered!
        if (evt.flag == 1) {
            float xVal, yVal, zVal;
            getAccel(&xVal, &yVal, &zVal);
            velocity = p_velocity - (xVal+ 0.5)*0.1 ;
            printf("%f\t", velocity);
            // printf("%f\n", xVal);
            p_velocity = velocity;
        }
    
  }
}

static void udp_client_task(void *pvParameters)
{
    char rx_buffer[128];
    char host_ip[] = HOST_IP_ADDR;
    int addr_family = 0;
    int ip_protocol = 0;

    while (1) {

#if defined(CONFIG_EXAMPLE_IPV4)
        struct sockaddr_in dest_addr;
        dest_addr.sin_addr.s_addr = inet_addr(HOST_IP_ADDR);
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(PORT);
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;
#elif defined(CONFIG_EXAMPLE_IPV6)
        struct sockaddr_in6 dest_addr = { 0 };
        inet6_aton(HOST_IP_ADDR, &dest_addr.sin6_addr);
        dest_addr.sin6_family = AF_INET6;
        dest_addr.sin6_port = htons(PORT);
        dest_addr.sin6_scope_id = esp_netif_get_netif_impl_index(EXAMPLE_INTERFACE);
        addr_family = AF_INET6;
        ip_protocol = IPPROTO_IPV6;
#elif defined(CONFIG_EXAMPLE_SOCKET_IP_INPUT_STDIN)
        struct sockaddr_storage dest_addr = { 0 };
        ESP_ERROR_CHECK(get_addr_from_stdin(PORT, SOCK_DGRAM, &ip_protocol, &addr_family, &dest_addr));
#endif

        int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }

        // Set timeout
        struct timeval timeout;
        timeout.tv_sec = 10;
        timeout.tv_usec = 0;
        setsockopt (sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof timeout);

        ESP_LOGI(TAG, "Socket created, sending to %s:%d", HOST_IP_ADDR, PORT);

        while (1) {
            sprintf(outstring, "%f,%f", left_lidar, right_lidar);
            payload = outstring;
            int err = sendto(sock, payload, strlen(payload), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
            if (err < 0) {
                ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                break;
            }
            ESP_LOGI(TAG, "Message sent");

            struct sockaddr_storage source_addr; // Large enough for both IPv4 or IPv6
            socklen_t socklen = sizeof(source_addr);
            int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);

            // Error occurred during receiving
            if (len < 0) {
                ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
                break;
            }
            // Data received
            else {
                rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string
                ESP_LOGI(TAG, "Received %d bytes from %s:", len, host_ip);
                ESP_LOGI(TAG, "%s", rx_buffer);
                if (strncmp(rx_buffer, "OK: ", 4) == 0) {
                    ESP_LOGI(TAG, "Received expected message, reconnecting");
                    break;
                }
            }

            vTaskDelay(2000 / portTICK_PERIOD_MS);
        }

        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    }
    vTaskDelete(NULL);
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

// Read register (single byte read)
uint16_t readRegister(uint8_t reg) {
  uint8_t data1; //first byte MSB
  uint8_t data2; //second byte LSB

  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_cmd_handle_t cmd1 = i2c_cmd_link_create();

  // Start
  i2c_master_start(cmd);
  // Master write slave address + write bit
  i2c_master_write_byte(cmd, ( LIDARLite_ADDRESS << 1 ) | WRITE_BIT, I2C_MASTER_ACK); 
  // Master write register address + send ack
  i2c_master_write_byte(cmd, reg, I2C_MASTER_ACK); 
  //master stops
  i2c_master_stop(cmd);
  // This starts the I2C communication
  i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS); 
  i2c_cmd_link_delete(cmd);

  //master starts
  i2c_master_start(cmd1);
  // Master write slave address + read bit
  i2c_master_write_byte(cmd1, ( LIDARLite_ADDRESS << 1 ) | READ_BIT, I2C_MASTER_ACK);  
  // Master reads in slave ack and data
  i2c_master_read_byte(cmd1, &data1 , I2C_MASTER_ACK);
  i2c_master_read_byte(cmd1, &data2 , I2C_MASTER_NACK);
  // Master nacks and stops.
  i2c_master_stop(cmd1);
  // This starts the I2C communication
  i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd1, 1000 / portTICK_RATE_MS); 
  i2c_cmd_link_delete(cmd1);
  

  uint16_t two_byte_data = (data1 << 8 | data2);
  return two_byte_data;
}

static void test(){
    printf("\n>> Polling Lidar\n");
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
        float distance_m = (float)*distance0/100.0;
        float distance_m1 = (float)*distance1/100.0;
        float distance_m2 = (float)*distance2/100.0;
        printf("Distance: %.2f m\n", distance_m);
        printf("Distance1: %.2f m\n", distance_m1);
        printf("Distance2: %.2f m\n", distance_m2);
        left_lidar = distance_m1;
        right_lidar = distance_m2;
        
        
        vTaskDelay(1000 / portTICK_RATE_MS);
    }
}



void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect());

    init();

    timer_queue = xQueueCreate(10, sizeof(timer_event_t));

    // Routine
    i2c_master_init();
    i2c_scanner();

    // Check for ADXL343
    uint8_t deviceID;
    getDeviceID(&deviceID);
    if (deviceID == 0xE5) {
        printf("\n>> Found ADAXL343\n");
    }

    // Disable interrupts
    writeRegister(ADXL343_REG_INT_ENABLE, 0);

    // Set range
    setRange(ADXL343_RANGE_16_G);

    // Enable measurements
    writeRegister(ADXL343_REG_POWER_CTL, 0x08);

#ifdef CONFIG_EXAMPLE_IPV4
    xTaskCreate(udp_server_task, "udp_server", 4096, (void*)AF_INET, 5, NULL);
#endif
#ifdef CONFIG_EXAMPLE_IPV6
    xTaskCreate(udp_server_task, "udp_server", 4096, (void*)AF_INET6, 5, NULL);
#endif

    //xTaskCreate(decoder, "decoder", 4096, NULL, 5, NULL);
    //xTaskCreate(test_adxl343,"test_adxl343", 4096, NULL, 5, NULL);

    xTaskCreate(esc_servo_task, "esc_servo_task", 4096, NULL, 6, NULL);
    //xTaskCreate(led, "led", 4096, NULL, 8, NULL);
    xTaskCreate(steering_servo_task, "steering_servo_task", 4096, NULL, 6, NULL);

    // Initiate alarm using timer API
    alarm_init();

    xTaskCreate(udp_client_task, "udp_client", 4096, NULL, 5, NULL);
    xTaskCreate(lidar, "lidar", 4096, NULL, 6, NULL);
}
