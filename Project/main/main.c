// Project - Embedded systems - ESP32
// Group : Vitor Baraky and Victor Hugo
// Library
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
// Defining Global TAGs
 static const char* TAG = "ESP_32";
// Defining global variables
// Inputs
#define GPIO_INPUT_IO_0    21
#define GPIO_INPUT_IO_1    22
#define GPIO_INPUT_IO_2    23
#define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_INPUT_IO_0) | (1ULL<<GPIO_INPUT_IO_1)| (1ULL<<GPIO_INPUT_IO_2))
// Outputs
#define GPIO_OUTPUT_IO_0 2 
#define GPIO_OUTPUT_PIN_SEL 1ULL<<GPIO_OUTPUT_IO_0
#define ESP_INTR_FLAG_DEFAULT 0
// LED STATE variable
uint32_t led_state = 0; // variable responsable to register the LED state
// define event queue
static QueueHandle_t gpio_evt_queue = NULL;
// Interruption tasks
static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);  
}
// GPIO task
static void gpio_task(void* arg)
{
    uint32_t io_num; // IO number received by the Queue
    for(;;) {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            ESP_LOGI(TAG,"GPIO[%"PRIu32"] intr, val: %d\n", io_num, gpio_get_level(io_num));
            // LED activation logic
            switch (io_num)
            {
                case GPIO_INPUT_IO_0:
                    gpio_set_level(GPIO_OUTPUT_IO_0,1);
                    led_state = 1;
                break;
                case GPIO_INPUT_IO_1:
                    gpio_set_level(GPIO_OUTPUT_IO_0,0);
                    led_state = 0; 
                break;
                case GPIO_INPUT_IO_2:
                    // Evaluate LED state
                    if (led_state){
                        gpio_set_level(GPIO_OUTPUT_IO_0,0);
                        led_state = 0;
                    }
                    else{
                        gpio_set_level(GPIO_OUTPUT_IO_0,1);
                        led_state = 1;
                    };
                break;
                default:
                break;
            }
        }
    }
}
// Main task
void app_main(void)
{
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
    // Creating queue
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t)); //criação de fila
    xTaskCreate(gpio_task, "gpio_task", 2048, NULL, 10, NULL); //task do gpio
    // install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    // hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler, (void*) GPIO_INPUT_IO_0);
    gpio_isr_handler_add(GPIO_INPUT_IO_1, gpio_isr_handler, (void*) GPIO_INPUT_IO_1);
    gpio_isr_handler_add(GPIO_INPUT_IO_2, gpio_isr_handler, (void*) GPIO_INPUT_IO_2);
    // Set variables
    esp_chip_info_t chip_info;
    uint32_t flash_size;
    esp_chip_info(&chip_info);
    esp_log_level_set("ESP_32", ESP_LOG_INFO);
    // Print LOG
    ESP_LOGI(TAG, "This is %s chip model %u with %d CPU core(s) WiFi%s%s%s, ", 
                                                                CONFIG_IDF_TARGET, chip_info.model, chip_info.cores,
                                                                (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
                                                                (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "",
                                                                (chip_info.features & CHIP_FEATURE_IEEE802154) ? ", 802.15.4 (Zigbee/Thread)" : "" );

    unsigned major_rev = chip_info.revision / 100;
    unsigned minor_rev = chip_info.revision % 100;
    ESP_LOGI(TAG, "silicon revision v%d.%d, ", major_rev, minor_rev);
    if(esp_flash_get_size(NULL, &flash_size) != ESP_OK) {
        ESP_LOGE(TAG, "Get flash size failed");
        return;
    }
    ESP_LOGI(TAG,"%" PRIu32 "MB %s flash\n", flash_size / (uint32_t)(1024 * 1024),
           (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    ESP_LOGI(TAG,"Minimum free heap size: %" PRIu32 " bytes\n", esp_get_minimum_free_heap_size());
    // Block loop
    int i = 0;
    while(1)
    {
        i++;
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        ESP_LOGI(TAG,"The program is running %d ...", i);
    }

}
