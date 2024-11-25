#include <stdio.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_log.h"

#define TEMT6000_TAG "TEMT6000"
#define TEMT6000 ADC_CHANNEL_7

void getValues(float *volts,float* voltsPercent,float* amps, float* microamps, float* lux)
{
    (*volts) = adc1_get_raw(TEMT6000)*5/4096.0;
    (*voltsPercent) = adc1_get_raw(TEMT6000)/4096.0*100;
    (*amps) = (*volts) / 10000;
    (*microamps) = (*amps) * 1000000;
    (*lux) = (*microamps) * 2.0;
}
void log_values(float *volts,float* voltsPercent,float* amps, float* microamps, float* lux)
{
    ESP_LOGI(TEMT6000_TAG,"Voltage:%.2f\nVoltage Percents: %.2f",(*volts),(*voltsPercent));
    ESP_LOGI(TEMT6000_TAG, "Amperes: %.3f\nMicroamperes: %.3f",(*amps),(*microamps));
    ESP_LOGI(TEMT6000_TAG, "Lux: %.3f",(*lux));
}

void app_main(void)
{
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(TEMT6000,ADC_ATTEN_DB_0);
    
    float volts, voltsPercent, amps, microamps,lux;
    while (1)
    {
        getValues(&volts,&voltsPercent,&amps,&microamps,&lux);
        log_values(&volts,&voltsPercent,&amps,&microamps,&lux);
        vTaskDelay(1000/portTICK_PERIOD_MS);
    }
}
