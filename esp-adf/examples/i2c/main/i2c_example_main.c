/* i2c - Example

   For other examples please check:
   https://github.com/espressif/esp-idf/tree/master/examples

   See README.md file to get detailed usage of this example.

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "sdkconfig.h"

static const char *TAG = "i2c-example";

#define _I2C_NUMBER(num) I2C_NUM_##num
#define I2C_NUMBER(num) _I2C_NUMBER(num)

#define DATA_LENGTH 512                  /*!< Data buffer length of test buffer */
#define RW_TEST_LENGTH 128               /*!< Data length for r/w test, [0,DATA_LENGTH] */
#define DELAY_TIME_BETWEEN_ITEMS_MS 1000 /*!< delay time between different test items */

#define I2C_MASTER_SCL_IO CONFIG_I2C_MASTER_SCL               /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO CONFIG_I2C_MASTER_SDA               /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUMBER(CONFIG_I2C_MASTER_PORT_NUM) /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ CONFIG_I2C_MASTER_FREQUENCY        /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */

#define WRITE_BIT I2C_MASTER_WRITE              /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ                /*!< I2C master read */
#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                             /*!< I2C ack value */
#define NACK_VAL 0x1                            /*!< I2C nack value */


#define CHIP_AUDIO_RS		0x00
#define AC101_ADDR			0x1a				/*!< Device address*/
#define SPKOUT_CTRL			0x58

static int AC101_Write_Reg(i2c_port_t i2c_num, uint8_t reg_addr, uint16_t data);
static uint16_t AC101_read_Reg(i2c_port_t i2c_num, uint8_t reg_addr);

SemaphoreHandle_t print_mux = NULL;

/**
 * @brief test code to read esp-i2c-slave
 *        We need to fill the buffer of esp slave device, then master can read them out.
 *
 * _______________________________________________________________________________________
 * | start | slave_addr + rd_bit +ack | read n-1 bytes + ack | read 1 byte + nack | stop |
 * --------|--------------------------|----------------------|--------------------|------|
 *
 */


/**
 * @brief Test code to write esp-i2c-slave
 *        Master device write data to slave(both esp32),
 *        the data will be stored in slave buffer.
 *        We can read them out from slave buffer.
 *
 * ___________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write n bytes + ack  | stop |
 * --------|---------------------------|----------------------|------|
 *
 */

/**
 * @brief test code to operate on BH1750 sensor
 *
 * 1. set operation mode(e.g One time L-resolution mode)
 * _________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write 1 byte + ack  | stop |
 * --------|---------------------------|---------------------|------|
 * 2. wait more than 24 ms
 * 3. read data
 * ______________________________________________________________________________________
 * | start | slave_addr + rd_bit + ack | read 1 byte + ack  | read 1 byte + nack | stop |
 * --------|---------------------------|--------------------|--------------------|------|
 */
 
 

 
static esp_err_t i2c_master_sensor_test(i2c_port_t i2c_num, uint16_t * data_1)
{

	int ret;

  	ret = AC101_Write_Reg(i2c_num, CHIP_AUDIO_RS, 0x123);

    if (ret != ESP_OK) {
        return ret;
    }
    vTaskDelay(30 / portTICK_RATE_MS);
 
    *data_1 = AC101_read_Reg(i2c_num, SPKOUT_CTRL);

 	return ret;
}

static int AC101_Write_Reg(i2c_port_t i2c_num, uint8_t reg_addr, uint16_t data)
{
    int res = 0;
  
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, AC101_ADDR, 1 /*ACK_CHECK_EN*/);
    i2c_master_write_byte(cmd, reg_addr, 1 /*ACK_CHECK_EN*/);
    i2c_master_write_byte(cmd, data, 1 /*ACK_CHECK_EN*/);
    i2c_master_stop(cmd);
    res = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    return res;
}

static uint16_t AC101_read_Reg(i2c_port_t i2c_num, uint8_t reg_addr)
{
    uint8_t data;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, AC101_ADDR, 1 /*ACK_CHECK_EN*/);
    i2c_master_write_byte(cmd, reg_addr, 1 /*ACK_CHECK_EN*/);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, AC101_ADDR | 0x01, 1 /*ACK_CHECK_EN*/);
    i2c_master_read_byte(cmd, &data, 0x01 /*NACK_VAL*/);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(0, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    return (int)data;
}

/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(void)
{
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = GPIO_NUM_32;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = GPIO_NUM_33;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(1, &conf);
    return i2c_driver_install(1, conf.mode, 0, 0, 0);
}

static void i2c_test_task(void *arg)
{
    int i = 0;
    int ret;
    uint32_t task_idx = (uint32_t)arg;
    uint8_t *data = (uint8_t *)malloc(DATA_LENGTH);
    uint16_t data_1 = 0;

	
	int cnt = 0;
    while (1) {
        ESP_LOGI(TAG, "TASK[%d] test cnt: %d", task_idx, cnt++);
        ret = i2c_master_sensor_test(1, &data_1);
 		xSemaphoreTake(print_mux, portMAX_DELAY);
 
        printf("*******************\n");
        printf("TASK[%d]\n", task_idx);
        printf("*******************\n");
        printf("SPKOUT_CTRL: %02x\n", data_1);
        printf("*******************\n");
        printf("ret: %02x\n", ret);

        xSemaphoreGive(print_mux);
        vTaskDelay((DELAY_TIME_BETWEEN_ITEMS_MS * (task_idx + 1)) / portTICK_RATE_MS);
        //---------------------------------------------------
        for (i = 0; i < DATA_LENGTH; i++) {
            data[i] = i;
        }
        xSemaphoreTake(print_mux, portMAX_DELAY);
        vTaskDelay((DELAY_TIME_BETWEEN_ITEMS_MS * (task_idx + 1)) / portTICK_RATE_MS);
 }
    vSemaphoreDelete(print_mux);
    vTaskDelete(NULL);
}

void app_main(void)
{
    print_mux = xSemaphoreCreateMutex();
    ESP_ERROR_CHECK(i2c_master_init());
    xTaskCreate(i2c_test_task, "i2c_test_task_0", 1024 * 2, (void *)0, 10, NULL);
}
