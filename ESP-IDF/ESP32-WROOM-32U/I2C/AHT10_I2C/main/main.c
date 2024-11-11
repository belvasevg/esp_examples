#include <stdio.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/i2c.h"
#include "esp_log.h"

//параметры I2C bus
#define I2C_SDA_PORT            21
#define I2C_SCL_PORT            22
#define CLK_SPEED               100000

#define TAG_I2C_CONFIG          "I2C CONFIG"
#define TAG_AHT10               "AHT10"

//определение битов фиксации получения на slave и концы сообщений
#define ACK_CHECK_EN            0x1
#define ACK_CHECK_DIS           0x0
#define ACK_VAL                 0x0
#define NACK_VAL                0x1

#define AHT10_ADDR              0x38

#define CMD_INIT                0xE1
#define CMD_TRIGGER_MEASUREMENT 0xAC
#define CMD_SOFT_RESET          0xBA

#define AHT10_DATA_SIZE         6

static esp_err_t aht10_i2c_driver_install(void)
{
    i2c_config_t i2c_mst_cfg = {
        .mode = I2C_MODE_MASTER,
        .master.clk_speed = CLK_SPEED,
        .sda_io_num = I2C_SDA_PORT,
        .sda_pullup_en = false,
        .scl_io_num = I2C_SCL_PORT,
        .scl_pullup_en = false,
        .clk_flags = 0,
    };
    esp_err_t err = i2c_param_config(I2C_NUM_0,&i2c_mst_cfg);
    if(err != ESP_OK) goto return_error;
    return i2c_driver_install(I2C_NUM_0,i2c_mst_cfg.mode,0,0,0);
return_error:
    return err;
}

esp_err_t aht10_set_state(int state_cmd)
{
    esp_err_t err;
    i2c_cmd_handle_t cmdLink = i2c_cmd_link_create();
    if (cmdLink)
    {
        err = i2c_master_start(cmdLink);
        if (err!=ESP_OK) goto end;
        err = i2c_master_write_byte(cmdLink,((AHT10_ADDR<<1)|I2C_MASTER_WRITE),ACK_CHECK_EN);
        if (err!=ESP_OK) goto end;
        switch (state_cmd)
        {
            case 0:
            {
                err = i2c_master_write_byte(cmdLink,CMD_INIT,ACK_CHECK_EN);
                break;
            }
            case 1:
            {
                err = i2c_master_write_byte(cmdLink,CMD_TRIGGER_MEASUREMENT,ACK_CHECK_EN);
                break;
            }
            case 2:
            {
                err = i2c_master_write_byte(cmdLink,CMD_SOFT_RESET,ACK_CHECK_EN);
                break;
            }
        }
        if (err!=ESP_OK) goto end;
    }
    err = i2c_master_stop(cmdLink);
end:
    i2c_cmd_link_delete(cmdLink);
    return err;
}

esp_err_t read_AHT10_values(uint8_t* data_RX,int size_data_RX)
{
    esp_err_t err;
    i2c_cmd_handle_t cmdLink = i2c_cmd_link_create();
    if (cmdLink)
    {
        err = i2c_master_start(cmdLink);
        if (err!=ESP_OK) goto end;
        err = i2c_master_write_byte(cmdLink,((AHT10_ADDR<<1)|I2C_MASTER_READ),ACK_CHECK_EN);
        if (err!=ESP_OK) goto end;
        err = i2c_master_read(cmdLink,data_RX,(size_data_RX-1),ACK_VAL);
        if (err!=ESP_OK) goto end;
        err = i2c_master_read_byte(cmdLink,(data_RX+size_data_RX-1),NACK_VAL);
        if (err!=ESP_OK) goto end;
    }
    err = i2c_master_stop(cmdLink);
    for (int i=0;i<AHT10_DATA_SIZE;i++)
    {
        ESP_LOGI(TAG_AHT10, "AHT10 status is byte #%d = 0x%X", i,data_RX[i]);
    }
end:
    i2c_cmd_link_delete(cmdLink);
    return err;
}

uint32_t converted_value_from_AHT10(uint8_t* data,bool received_data_status)
{
    if (received_data_status)
    {
        return ((((uint32_t)data[1])<<12) | ((uint32_t)data[2]<<4) | (data[3]>>4));
    }
    else{
        return ((((uint32_t)data[3]&0x0F)<<16) | ((uint32_t)data[4]<<8) | data[5]);
    }
}

void app_main(void)
{
    uint8_t data_RW[AHT10_DATA_SIZE];
    uint32_t temp_u32,hum_u32;
    float temp_f,hum_f;
    esp_err_t err = aht10_i2c_driver_install();
    if (err!=ESP_OK)
    {
        ESP_LOGE(TAG_I2C_CONFIG,"I2C driver install ERROR! Error #%d(%s)",err,esp_err_to_name(err));
        return;
    }
    err = aht10_set_state(1);
    if (err!=ESP_OK)
    {
        ESP_LOGE(TAG_I2C_CONFIG,"I2C driver install ERROR! Error #%d(%s)",err,esp_err_to_name(err));
        return;
    }
    vTaskDelay(100);
    err = read_AHT10_values(data_RW,AHT10_DATA_SIZE);
    if (err!=ESP_OK)
    {
        ESP_LOGE(TAG_I2C_CONFIG,"I2C driver install ERROR! Error #%d(%s)",err,esp_err_to_name(err));
        return;
    }
    hum_u32 = converted_value_from_AHT10(data_RW,1);
    temp_u32 = converted_value_from_AHT10(data_RW,0);
    ESP_LOGI(TAG_AHT10,"Values in 32bit. Temp: %lu. Hum: %lu",(unsigned long)temp_u32,(unsigned long)hum_u32);
    hum_f = (float)(hum_u32*100/1048576.00);
    temp_f = (float)(temp_u32*200/1048576.00)-50;
    ESP_LOGI(TAG_AHT10,"Temp = %.2f\tHum = %.2f",temp_f,hum_f);
    err = aht10_set_state(2);
    if (err!=ESP_OK)
    {
        ESP_LOGE(TAG_I2C_CONFIG,"I2C driver install ERROR! Error #%d(%s)",err,esp_err_to_name(err));
        return;
    }
}