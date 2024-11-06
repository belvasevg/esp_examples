#include <stdio.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_log.h"

void app_main(void)
{
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC_CHANNEL_0,ADC_ATTEN_DB_0);
    while(1)
    {
        int val = adc1_get_raw(ADC_CHANNEL_0);
        ESP_LOGI("HALL VALUE", "Hall Voltage: %d", val);
        vTaskDelay(1000/portTICK_PERIOD_MS);
    }
    
}