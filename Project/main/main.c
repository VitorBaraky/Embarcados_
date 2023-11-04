// Project - Embedded systems - ESP32
// Group : Vitor Baraky and Victor Hugo
// ---------------------------------------- Library -------------------------------------------- //
#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/queue.h"
#include "driver/gptimer.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include <math.h>
#include <freertos/semphr.h>
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "soc/soc_caps.h"
#include "driver/uart.h"
#include "string.h"
// ----------------------------------------  Defining Global TAGs -------------------------------------------- //
static const char *TAG = "ESP_32";
static const char *TAG_GPIO = "GPIO";
static const char *TAG_TIMER = "TIMER";
static const char *TAG_PWM = "PWM";
static const char *TAG_ADC = "ADC";
static const char *TAG_UART = "UART";
//  ----------------------------------------  Defining global variables and struct ---------------------------- //
//  Inputs
#define GPIO_INPUT_IO_0 21
#define GPIO_INPUT_IO_1 22
#define GPIO_INPUT_IO_2 23
#define GPIO_INPUT_PIN_SEL ((1ULL << GPIO_INPUT_IO_0) | (1ULL << GPIO_INPUT_IO_1) | (1ULL << GPIO_INPUT_IO_2))
// Outputs
#define GPIO_OUTPUT_IO_0 2
#define GPIO_OUTPUT_PIN_SEL 1ULL << GPIO_OUTPUT_IO_0
#define ESP_INTR_FLAG_DEFAULT 0
// LED STATE variable
uint32_t led_state = 0; // variable responsable to register the LED state
// Defining event count struct
typedef struct
{
    uint64_t event_count;
    uint64_t a_count;
} queue_element_t;
int conta = 0;
// Defining real time clock struct
typedef struct
{
    uint8_t segundo;
    uint8_t minuto;
    uint8_t hora;
} horario;
horario relogio;
typedef struct
{
    bool mode;           // Validate pwm mode (Automatic or Manual)
    uint32_t duty_count; // Raw duty count
} pwm_modes;
typedef struct
{
    uint32_t raw;     // Raw value
    uint32_t voltage; // Calib voltage
} adc_type;
// PWM Global defines
#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL_0 LEDC_CHANNEL_0   // PWM CHANNEL 0
#define LEDC_CHANNEL_1 LEDC_CHANNEL_1   // PWM CHANNEL 1
#define LEDC_DUTY_RES LEDC_TIMER_13_BIT // PWM resolution
#define LEDC_FREQUENCY (5000)           // PWM frequency
// ADC Global defines
#define ADC1_CHAN0 ADC_CHANNEL_3
// UART Global variables
static const int RX_BUF_SIZE = 1024;
#define TXD_PIN (GPIO_NUM_5)
#define RXD_PIN (GPIO_NUM_4)
// ---------------------------------------- Starting SEMAPHOREs -------------------------------------------- //
static SemaphoreHandle_t semaphore_pwm = NULL;
static SemaphoreHandle_t semaphore_ADC = NULL;
// ---------------------------------------- Starting QUEUEs -------------------------------------------- //
// Starting GPIO Queue
static QueueHandle_t gpio_evt_queue = NULL;
// Starting Timer Queue
static QueueHandle_t Timer_evt_queue = NULL;
// Starting GPIO-PWM Queue
static QueueHandle_t gpio_pwm_queue = NULL;
// Starting ADC-TIMER Queue
static QueueHandle_t adc_timer_queue = NULL;
// Starting UART-TIMER Queue
static QueueHandle_t uart_timer_queue = NULL;
// ----------------------------------------  Interruptions -------------------------------------------- //
// GPIO interruption
static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    uint32_t gpio_num = (uint32_t)arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL); // Send GPIO number to Queue
}
// TIMER interruption
static bool IRAM_ATTR alarm_v1(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data)
{
    BaseType_t high_task_awoken = pdFALSE;
    // Retrieve count value and send to queue
    queue_element_t timer_element = {
        .event_count = edata->count_value,
        .a_count = edata->alarm_value};
    xQueueSendFromISR(Timer_evt_queue, &timer_element, NULL);
    gptimer_alarm_config_t alarm_config = {
        .alarm_count = edata->alarm_value + 100000, // alarm in next 100ms
    };
    gptimer_set_alarm_action(timer, &alarm_config);
    return (high_task_awoken == pdTRUE);
}
// ----------------------------------------  Functions -------------------------------------------- //
// Send Uart data function
int sendData(const char *logName, const char *data)
{
    const int len = strlen(data);
    const int txBytes = uart_write_bytes(UART_NUM_1, data, len);
    ESP_LOGI(logName, "Wrote %d bytes", txBytes);
    return txBytes;
}
// ----------------------------------------  TASKS -------------------------------------------- //
// GPIO task //
static void gpio_task(void *arg)
{
    // Set Log level
    esp_log_level_set(TAG_GPIO, ESP_LOG_NONE);
    // Input Config
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf); // Configuring specific inputs
    // Output Config
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf); // Configuring specific outputs
    uint32_t io_num;       // IO number received by the Queue
    // install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    // hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler, (void *)GPIO_INPUT_IO_0);
    gpio_isr_handler_add(GPIO_INPUT_IO_1, gpio_isr_handler, (void *)GPIO_INPUT_IO_1);
    gpio_isr_handler_add(GPIO_INPUT_IO_2, gpio_isr_handler, (void *)GPIO_INPUT_IO_2);
    // PWM init variables
    pwm_modes button_pwm;
    button_pwm.mode = 0;
    button_pwm.duty_count = 0;
    // GPIO and PWM loop
    for (;;)
    {
        // Unlock semaphor
        xSemaphoreTake(semaphore_pwm, portMAX_DELAY);
        // Receive GPIO number from Queue
        if (xQueueReceive(gpio_evt_queue, &io_num, 10 / (portTICK_PERIOD_MS)))
        {
            ESP_LOGI(TAG_GPIO, "GPIO[%" PRIu32 "] intr, val: %d\n", io_num, gpio_get_level(io_num));
            switch (io_num)
            {
            // Automatic PWM mode - PIN 21
            case GPIO_INPUT_IO_0:
                gpio_set_level(GPIO_OUTPUT_IO_0, 1);
                led_state = 1;
                button_pwm.mode = 0;
                button_pwm.duty_count = 0;
                break;
            // Manual PWM mode - PIN 22
            case GPIO_INPUT_IO_1: // 22 manual
                gpio_set_level(GPIO_OUTPUT_IO_0, 0);
                button_pwm.duty_count = 0;
                led_state = 0;
                button_pwm.mode = 1;
                break;
            // Manual Incrementation - PIN 23
            case GPIO_INPUT_IO_2:
                if (button_pwm.mode == 1)
                {
                    if (button_pwm.duty_count == 100)
                    {
                        button_pwm.duty_count = 0;
                    }
                    else
                    {
                        button_pwm.duty_count += 10;
                    }
                }
                // Evaluate LED state
                if (led_state)
                {
                    gpio_set_level(GPIO_OUTPUT_IO_0, 0);
                    led_state = 0;
                }
                else
                {
                    gpio_set_level(GPIO_OUTPUT_IO_0, 1);
                    led_state = 1;
                };
                break;
            default:
                break;
            }
        }
        // Send PWM Counter/Mode to queue
        xQueueSendToBack(gpio_pwm_queue, &button_pwm, NULL);
    }
}
// TIMER task //
static void timer_task(void *arg)
{
    // Set log Level
    esp_log_level_set(TAG_TIMER, ESP_LOG_INFO);
    // Timer configuration
    queue_element_t timer_element;
    adc_type adc1;
    horario relogioRTC;
    gptimer_handle_t gptimer = NULL;
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000, // 1MHz, 1 tick=1us
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer));
    // Timer Callback
    gptimer_event_callbacks_t call_Back = {
        .on_alarm = alarm_v1,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &call_Back, NULL));
    ESP_ERROR_CHECK(gptimer_enable(gptimer));
    ESP_LOGI(TAG, "Start timer, update alarm value dynamically");
    gptimer_alarm_config_t alarm_config1 = {
        .alarm_count = 100000, // period = 100ms
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_config1));
    ESP_ERROR_CHECK(gptimer_start(gptimer));
    // Clock Loop + Semaphor sync
    for (;;)
    {
        // Receive Timer counter
        if (xQueueReceive(Timer_evt_queue, &timer_element, portMAX_DELAY))
        {
            conta++;
            if (conta == 10)
            {
                relogio.segundo++;
                if (relogio.segundo == 60)
                {
                    relogio.minuto++;
                    relogio.segundo = 0;
                    if (relogio.minuto == 60)
                    {
                        relogio.hora++;
                        relogio.minuto = 0;
                        if (relogio.hora == 24)
                        {
                            relogio.hora = 0;
                        }
                    }
                }
                conta = 0;
                ESP_LOGI(TAG_TIMER, "Timer stopped, count=%llu , a_count=%llu, hora= %u, minuto= %u, segundo= %u", timer_element.event_count, timer_element.a_count, relogio.hora, relogio.minuto, relogio.segundo);
                while (xQueueReceive(adc_timer_queue, &adc1, 10 / (portTICK_PERIOD_MS)))
                {
                    ESP_LOGI(TAG_ADC, "ADC read --> Raw = %lu | Voltage = %lu ", adc1.raw, adc1.voltage);
                }
                if (xQueueReceive(uart_timer_queue, &relogioRTC, 10 / (portTICK_PERIOD_MS)))
                {
                    conta = 0;
                    relogio.hora = relogioRTC.hora;
                    relogio.minuto = relogioRTC.minuto;
                    relogio.segundo = relogioRTC.segundo;
                    relogioRTC.hora = 0;
                    relogioRTC.minuto = 0;
                    relogioRTC.segundo = 0;
                }
            }
            // Update semaphor state
            xSemaphoreGive(semaphore_pwm);
            xSemaphoreGive(semaphore_ADC);
        }
        else
        {
            ESP_LOGW(TAG_TIMER, "Missed one count event");
        }
    }
}
// PWM task
static void pwm_task(void *arg)
{
    // Set log level
    esp_log_level_set(TAG_PWM, ESP_LOG_NONE);
    pwm_modes button_pwm;
    int new_duty = 0;
    int duty_local = 0;
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t PWM_timer = {
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER,
        .duty_resolution = LEDC_DUTY_RES,
        .freq_hz = LEDC_FREQUENCY, // Set output frequency at 5 kHz
        .clk_cfg = LEDC_AUTO_CLK};
    ESP_ERROR_CHECK(ledc_timer_config(&PWM_timer));
    // PWM Channel 0
    ledc_channel_config_t PWM_1_channel = {
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = 16,
        .duty = 0, // Set duty to 0%
        .hpoint = 0};
    // PWM channel 1
    ledc_channel_config_t PWM_2_channel = {
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL_1,
        .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = 33,
        .duty = 0, // Set duty to 0%
        .hpoint = 0};
    ESP_ERROR_CHECK(ledc_channel_config(&PWM_1_channel));
    ESP_ERROR_CHECK(ledc_channel_config(&PWM_2_channel));
    // PWM modes
    for (;;)
    {
        // manual
        if (xQueueReceive(gpio_pwm_queue, &button_pwm, portMAX_DELAY))
        {
            if (button_pwm.mode == 1)
            {
                ESP_LOGI(TAG_PWM, "Manual mode");
                new_duty = (pow(2, 13) - 1) * button_pwm.duty_count / 100;
                ESP_LOGI(TAG_PWM, "New Manual Duty %u", new_duty);
                ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_0, new_duty));
                // Update duty to apply the new value
                ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_0));
                ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_1, new_duty));
                // Update duty to apply the new value
                ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_1));
            }
            // Automatic
            if (button_pwm.mode == 0)
            {
                ESP_LOGI(TAG_PWM, "Automatic mode");
                duty_local += 10;
                new_duty = (pow(2, 13) - 1) * duty_local / 100;
                ESP_LOGI(TAG_PWM, "New automatic Duty %u", new_duty);
                if (duty_local == 100)
                    duty_local = 0;
                ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_0, new_duty));
                // Update duty to apply the new value
                ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_0));
                ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_1, new_duty));
                // Update duty to apply the new value
                ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_1));
            }
        }
    }
}
// ADC Task
static void ADC_task(void *arg)
{
    // Set Log level
    esp_log_level_set(TAG_ADC, ESP_LOG_NONE);
    adc_type adc1;
    //-------------ADC1 Init---------------//
    adc_oneshot_unit_handle_t adc1_handle;
    adc_cali_handle_t adc1_cali_handle = NULL;
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    adc_oneshot_new_unit(&init_config1, &adc1_handle);
    //-------------ADC1 Config---------------//
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_12, // 12bit max resolution
        .atten = ADC_ATTEN_DB_11     // Allows a greater measurement range
    };
    adc_oneshot_config_channel(adc1_handle, ADC1_CHAN0, &config);
    //-------------ADC1 Calibration Init---------------//
    adc_cali_line_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN_DB_11,
        .bitwidth = ADC_BITWIDTH_12,
    };
    adc_cali_create_scheme_line_fitting(&cali_config, &adc1_cali_handle);
    // Reading loop
    for (;;)
    {
        // Unlock semaphor
        xSemaphoreTake(semaphore_ADC, portMAX_DELAY);
        adc_oneshot_read(adc1_handle, ADC1_CHAN0, &adc1.raw);
        adc_cali_raw_to_voltage(adc1_cali_handle, adc1.raw, &adc1.voltage);
        xQueueSendToBack(adc_timer_queue, &adc1, NULL);
    }
}
// UART task
static void uart_task(void *arg)
{
    // Set log level
    esp_log_level_set(TAG_UART, ESP_LOG_INFO);
    horario relogioRTC;
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    // RX data receive loop
    static const char *TAG_UART = "RX_UART";
    uint8_t *data = (uint8_t *)malloc(RX_BUF_SIZE + 1);
    while (1)
    {
        const int rxBytes = uart_read_bytes(UART_NUM_1, data, RX_BUF_SIZE, 1000 / portTICK_PERIOD_MS);
        if (rxBytes > 0)
        {
            data[rxBytes] = 0;
            relogioRTC.hora = (data[0] - '0') * 10 + (data[1] - '0');
            relogioRTC.minuto = (data[2] - '0') * 10 + (data[3] - '0');
            relogioRTC.segundo = (data[4] - '0') * 10 + (data[5] - '0');
            xQueueSendToBack(uart_timer_queue, &relogioRTC, NULL);
            ESP_LOGI(TAG_UART, "Read %d bytes: '%s'", rxBytes, data);
            ESP_LOG_BUFFER_HEXDUMP(TAG_UART, data, rxBytes, ESP_LOG_INFO);
        }
    }
    free(data);
}
// Main Task
void app_main(void)
{
    // Set log level
    esp_log_level_set(TAG, ESP_LOG_INFO);
    // ------- CHIP INFO -------- //
    // Set variables
    esp_chip_info_t chip_info;
    uint32_t flash_size;
    esp_chip_info(&chip_info);
    // Print LOG
    ESP_LOGI(TAG, "This is %s chip model %u with %d CPU core(s) WiFi%s%s%s, ",
             CONFIG_IDF_TARGET, chip_info.model, chip_info.cores,
             (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
             (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "",
             (chip_info.features & CHIP_FEATURE_IEEE802154) ? ", 802.15.4 (Zigbee/Thread)" : "");

    unsigned major_rev = chip_info.revision / 100;
    unsigned minor_rev = chip_info.revision % 100;
    ESP_LOGI(TAG, "silicon revision v%d.%d, ", major_rev, minor_rev);
    if (esp_flash_get_size(NULL, &flash_size) != ESP_OK)
    {
        ESP_LOGE(TAG, "Get flash size failed");
        return;
    }
    ESP_LOGI(TAG, "%" PRIu32 "MB %s flash\n", flash_size / (uint32_t)(1024 * 1024),
             (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    ESP_LOGI(TAG, "Minimum free heap size: %" PRIu32 " bytes\n", esp_get_minimum_free_heap_size());

    // ------- GPIO -------- //
    // Creating queue
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    gpio_pwm_queue = xQueueCreate(10, sizeof(pwm_modes));
    semaphore_pwm = xSemaphoreCreateBinary();
    semaphore_ADC = xSemaphoreCreateBinary();
    xTaskCreate(gpio_task, "gpio_task", 2048, NULL, 10, NULL);
    // ------- Timer -------- //
    // Creating TIMER Queue and task
    uart_timer_queue = xQueueCreate(10, sizeof(uint32_t)); // UART queue
    adc_timer_queue = xQueueCreate(10, sizeof(adc_type));  // ADC queue
    Timer_evt_queue = xQueueCreate(10, sizeof(queue_element_t));
    if (!Timer_evt_queue)
    {
        ESP_LOGE(TAG_TIMER, "Creating queue failed");
        return;
    }
    ESP_LOGI(TAG_TIMER, "Create timer handle");
    xTaskCreate(timer_task, "Timer_task", 2048, NULL, 10, NULL);
    // ------- PWM -------- //
    xTaskCreate(pwm_task, "pwm_task", 2048, NULL, 10, NULL);
    // ------- ADC -------- //
    xTaskCreate(ADC_task, "ADC_task", 2048, NULL, 10, NULL);
    // ------- UART -------- //
    xTaskCreate(uart_task, "uart_task", 2048, NULL, configMAX_PRIORITIES, NULL);
    // ------- Block loop -------- //
    int i = 0;
    while (1)
    {
        i++;
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        // ESP_LOGI(TAG,"The program is running %d ...", i);
    }
}
