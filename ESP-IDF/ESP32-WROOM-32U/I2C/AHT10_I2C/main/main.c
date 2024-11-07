#include <stdio.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/adc.h"
#include "esp_log.h"

#define I2C_SDA_PORT 21
#define I2C_SCL_PORT 22
#define CLK_SPEED 100000

#define I2C_CFG_TAG "I2C CONFIG"
#define I2C_READ_TAG "I2C READ AHT10"

#define ACK_CHECK_EN 0x1
#define ACK_CHECK_DIS 0x0
#define ACK_VAL 0x0
#define NACK_VAL 0x1

#define AHT10_ADDR 0x38

SemaphoreHandle_t print_mux = NULL;

static esp_err_t i2c_mst_cfg(void)
{
    //конфигураци€ режима I2C
    i2c_config_t wroom32u_i2c_cfg = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_PORT,
        .scl_io_num = I2C_SCL_PORT,
        //при false подключить подт€гивающие резисторы 4,7 кќм
        //при true аппаратно подключаютс€ встроенные регисторы 10кќм
        .sda_pullup_en = false,
        .scl_pullup_en = false,
        .master.clk_speed = CLK_SPEED,
        .clk_flags = 0,
    };
    //конфигурирование шины I2C
    esp_err_t err = i2c_param_config(I2C_NUM_0, &wroom32u_i2c_cfg);

    if (err!=ESP_OK)
    {
        ESP_LOGE(I2C_CFG_TAG,"I2C param configure error! Error #%d (%s)",err,esp_err_to_name(err));
        return ESP_FAIL;
    }
    //установка драйвера I2C
    return i2c_driver_install(I2C_NUM_0, wroom32u_i2c_cfg.mode,0,0,0);
}
esp_err_t readAHT10_I2C(i2c_port_t i2c_num,uint8_t* data_hum, uint8_t* data_temp)
{
    esp_err_t error_code = ESP_ERR_NO_MEM;
    // онтейнер дл€ команд
    i2c_cmd_handle_t cmdLink = i2c_cmd_link_create();
    if (cmdLink)
    {
        //отправка стартового бита
        error_code = i2c_master_start(cmdLink);
        if (error_code != ESP_OK) goto end;
        //отправка адреса с бита чтени€
        error_code = i2c_master_write_byte(cmdLink,(AHT10_ADDR<<1)|I2C_MASTER_READ,ACK_CHECK_EN);
        if (error_code != ESP_OK) goto end;
        //чтение данных
        error_code = i2c_master_read_byte(cmdLink,data_hum,ACK_VAL);
        if (error_code != ESP_OK) goto end;
        error_code = i2c_master_read_byte(cmdLink, data_temp,NACK_VAL);
        if (error_code != ESP_OK) goto end;
        error_code = i2c_master_stop(cmdLink);
        if (error_code != ESP_OK) goto end;
        error_code = i2c_master_cmd_begin(i2c_num,cmdLink,1000/portTICK_PERIOD_MS);
    }
end:
    if (error_code != ESP_OK)
    {
        ESP_LOGE(I2C_READ_TAG,"I2C Transmit/Recieve Error! Error #%d (%s)",error_code,esp_err_to_name(error_code));
    }
    i2c_cmd_link_delete(cmdLink);
    return error_code;
}


void app_main(void)
{
    esp_err_t err = i2c_mst_cfg();
    if (err!=ESP_OK)
    {
        ESP_LOGE(I2C_CFG_TAG,"I2C driver install error! Error #%d (%s)",err,esp_err_to_name(err));
        return;       
    }
    else{
        ESP_LOGI(I2C_CFG_TAG, "I2C driver install success!");
    }
    uint8_t sensor_data_hum,sensor_data_temp;

    err = readAHT10_I2C(I2C_NUM_0,&sensor_data_hum,&sensor_data_temp);
    //xSemaphoreTake(print_mux,portMAX_DELAY);
    if (err == ESP_ERR_TIMEOUT){
        ESP_LOGE(I2C_READ_TAG, "I2C Timeout error!");
        return;
    }
    else if(err == ESP_OK){
        ESP_LOGI(I2C_READ_TAG,"Sensor data: %x",sensor_data_hum);
        ESP_LOGI(I2C_READ_TAG,"Sensor_data_temp: %x",sensor_data_temp);
    }
    else{
        ESP_LOGE(I2C_READ_TAG,"I2C read func error! Error: %d (%s)",err,esp_err_to_name(err));
        return;
    }
    //xSemaphoreGive(print_mux);
    vTaskDelay(1000/portTICK_PERIOD_MS);

}