#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "esp_log.h"
#include "driver/i2c.h"

#define I2C_MASTER_SCL_IO           9
#define I2C_MASTER_SDA_IO           8
#define I2C_MASTER_NUM              0
#define I2C_MASTER_FREQ_HZ          400000
#define I2C_MASTER_TX_BUF_DISABLE   0
#define I2C_MASTER_RX_BUF_DISABLE   0
#define I2C_MASTER_TIMEOUT_MS       1000

#define LSM9DS1_M   0x1E
#define LSM9DS1_AG  0x6B
#define BNO  0x28

static const char* TAG = "TestModule";

static esp_err_t writeDevice(uint8_t dev_addr, uint8_t reg_addr, uint8_t data) {
  i2c_cmd_handle_t link = i2c_cmd_link_create();

  i2c_master_start(link);
  i2c_master_write_byte(link, dev_addr << 1 | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(link, reg_addr, true);
  i2c_master_write_byte(link, data, true);
  i2c_master_stop(link);

  esp_err_t res = i2c_master_cmd_begin(I2C_MASTER_NUM, link, I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);

  i2c_cmd_link_delete(link);
  return res;
}

static esp_err_t readDevice(uint8_t dev_addr, uint8_t reg_addr, uint8_t* data, size_t dataSize) {
  i2c_cmd_handle_t link = i2c_cmd_link_create();

  i2c_master_start(link);
  i2c_master_write_byte(link, dev_addr << 1 | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(link, reg_addr, true);

  i2c_master_start(link);
  i2c_master_write_byte(link, dev_addr << 1 | I2C_MASTER_READ, true);
  for(size_t x = 0; x < dataSize; x++) {
    i2c_master_read_byte(link, data + x, x != (dataSize-1) ? I2C_MASTER_ACK : I2C_MASTER_NACK);
  }
  i2c_master_stop(link);

  esp_err_t res = i2c_master_cmd_begin(I2C_MASTER_NUM, link, I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);

  i2c_cmd_link_delete(link);
  return res;
}

static void i2cInit() {
  i2c_config_t conf = {
    .mode = I2C_MODE_MASTER,
    .sda_io_num = I2C_MASTER_SDA_IO,
    .sda_pullup_en = GPIO_PULLUP_ENABLE,
    .scl_io_num = I2C_MASTER_SCL_IO,
    .scl_pullup_en = GPIO_PULLUP_ENABLE,
    .master.clk_speed = I2C_MASTER_FREQ_HZ,
  };

  ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
  ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, I2C_MODE_MASTER, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0));
}

void app_main(void) {
  i2cInit();
  ESP_LOGI(TAG, "I2C initialized successfully");

  //Ensure LSM device is correct
  uint8_t data;
  ESP_ERROR_CHECK(readDevice(LSM9DS1_AG, 0x0F, (uint8_t*) &data, sizeof(data)));
  if(data != 0x68) {
    ESP_LOGE(TAG, "BAD LSM DEVICE");
    exit(1);
  }
  //Bring LSM sensors out of power off state
  data = 0x20;
  ESP_ERROR_CHECK(writeDevice(LSM9DS1_AG, 0x10, data));

  //Ensure BNO is correct
  ESP_ERROR_CHECK(readDevice(BNO, 0x00, (uint8_t*) &data, sizeof(data)));
  if(data != 0xA0) {
    ESP_LOGE(TAG, "BAD BNO DEVICE");
    exit(1);
  }
  //Check self-test on BNO
  ESP_ERROR_CHECK(readDevice(BNO, 0x36, (uint8_t*) &data, sizeof(data)));
  data = 0xF & data;
  if(data != 0xF) {
    ESP_LOGE(TAG, "BNO failed self-test: 0x%X", data);
    exit(1);
  }
  //Set OP mode of BNO
  data = 0xC;
  ESP_ERROR_CHECK(writeDevice(BNO, 0x3D, data));

  while(1) {
    //Read LSM 
    int16_t accX, accY, accZ;
    ESP_ERROR_CHECK(readDevice(LSM9DS1_AG, 0x28, (uint8_t*) &accX, sizeof(accX)));
    ESP_ERROR_CHECK(readDevice(LSM9DS1_AG, 0x2A, (uint8_t*) &accY, sizeof(accY)));
    ESP_ERROR_CHECK(readDevice(LSM9DS1_AG, 0x2C, (uint8_t*) &accZ, sizeof(accZ)));
    

    //Read BNO
    int16_t bnoaccX, bnoaccY, bnoaccZ;
    ESP_ERROR_CHECK(readDevice(BNO, 0x8, (uint8_t*) &bnoaccX, sizeof(bnoaccX)));
    ESP_ERROR_CHECK(readDevice(BNO, 0xA, (uint8_t*) &bnoaccY, sizeof(bnoaccY)));
    ESP_ERROR_CHECK(readDevice(BNO, 0xC, (uint8_t*) &bnoaccZ, sizeof(bnoaccZ)));
    
    ESP_LOGI(TAG, "BNO Accel X: %d\tY: %d\tZ: %d" "\tLSM Accel X: %d\tY: %d\tZ: %d", bnoaccX, bnoaccY, bnoaccZ, accX, accY, accZ);

    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}
