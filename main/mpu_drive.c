#include "head.h"

static const char *TAG = "D_t";

#define I2C_SCL_IO        				    2              				  /*!< gpio number for I2C master clock */
#define I2C_SDA_IO      					14           				     /*!< gpio number for I2C master data  */
#define I2C_EXAMPLE_MASTER_NUM              I2C_NUM_0        /*!< I2C port number for master dev */
#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE   0                /*!< I2C master do not need buffer */
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE   0                /*!< I2C master do not need buffer */

#define MPU6050_SENSOR_ADDR                 0x68             /*!< slave address for MPU6050 sensor */
#define MPU6050_CMD_START                   0x41             /*!< Command to set measure mode */
#define MPU6050_WHO_AM_I                    0x75             /*!< Command to read WHO_AM_I reg */
#define WRITE_BIT                           I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT                            I2C_MASTER_READ  /*!< I2C master read */
#define ACK_CHECK_EN                        0x1              /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS                       0x0              /*!< I2C master will not check ack from slave */
#define ACK_VAL                             0x0              /*!< I2C ack value */
#define NACK_VAL                            0x1              /*!< I2C nack value */
#define LAST_NACK_VAL                       0x2              /*!< I2C last_nack value */

// MPU6050 Slave Device Address
const uint8_t MPU6050SlaveAddress = 0x68;

const uint8_t scl = 14;
const uint8_t sda = 12;
const uint8_t I2C_MASTER_NUM = I2C_NUM_0;
// sensitivity scale factor respective to full scale setting provided in datasheet
const uint16_t AccelScaleFactor = 16384;
const uint16_t GyroScaleFactor = 131;

// MPU6050 few configuration register addresses
const uint8_t MPU6050_REGISTER_SMPLRT_DIV   =  0x19;
const uint8_t MPU6050_REGISTER_USER_CTRL    =  0x6A;
const uint8_t MPU6050_REGISTER_PWR_MGMT_1   =  0x6B;
const uint8_t MPU6050_REGISTER_PWR_MGMT_2   =  0x6C;
const uint8_t MPU6050_REGISTER_CONFIG       =  0x1A;
const uint8_t MPU6050_REGISTER_GYRO_CONFIG  =  0x1B;
const uint8_t MPU6050_REGISTER_ACCEL_CONFIG =  0x1C;
const uint8_t MPU6050_REGISTER_FIFO_EN      =  0x23;
const uint8_t MPU6050_REGISTER_INT_ENABLE   =  0x38;
const uint8_t MPU6050_REGISTER_ACCEL_XOUT_H =  0x3B;
const uint8_t MPU6050_REGISTER_SIGNAL_PATH_RESET  = 0x68;
const uint8_t WHO_AM_I  = 0x75;

/**
 * @brief i2c master initialization
 */
static esp_err_t master_init()
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_SDA_IO;
    conf.sda_pullup_en = 1;
    conf.scl_io_num = I2C_SCL_IO;
    conf.scl_pullup_en = 1;
    conf.clk_stretch_tick = 300; // 300 ticks, Clock stretch is about 210us, you can make changes according to the actual situation.
    ESP_ERROR_CHECK(i2c_driver_install(i2c_master_port, conf.mode));
    ESP_ERROR_CHECK(i2c_param_config(i2c_master_port, &conf));
    ESP_LOGI(TAG,"i2c master initialization");
    return ESP_OK;
}

/**
 * @brief test code to write mpu6050
 *
 * 1. send data
 * ___________________________________________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write reg_address + ack | write data_len byte + ack  | stop |
 * --------|---------------------------|-------------------------|----------------------------|------|
 *
 * @param i2c_num I2C port number
 * @param reg_address slave reg address
 * @param data data to send
 * @param data_len data length
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 *     - ESP_FAIL Sending command error, slave doesn't ACK the transfer.
 *     - ESP_ERR_INVALID_STATE I2C driver not installed or not in master mode.
 *     - ESP_ERR_TIMEOUT Operation timeout because the bus is busy.
 */

static esp_err_t i2c_master_mpu6050_write(i2c_port_t i2c_num, uint8_t devAddress,uint8_t reg_address, uint8_t *data, size_t data_len)
{    ESP_LOGI(TAG,"i2c master write");
int ret;
i2c_cmd_handle_t cmd = i2c_cmd_link_create();
i2c_master_start(cmd);
i2c_master_write_byte(cmd, MPU6050_SENSOR_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
i2c_master_write_byte(cmd, reg_address, ACK_CHECK_EN);
i2c_master_write(cmd, data, data_len, ACK_CHECK_EN);
i2c_master_stop(cmd);
ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
i2c_cmd_link_delete(cmd);

return ret;
}

/**
 * @brief test code to read mpu6050
 *
 * 1. send reg address
 * ______________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write reg_address + ack | stop |
 * --------|---------------------------|-------------------------|------|
 *
 * 2. read data
 * ___________________________________________________________________________________
 * | start | slave_addr + wr_bit + ack | read data_len byte + ack(last nack)  | stop |
 * --------|---------------------------|--------------------------------------|------|
 *
 * @param i2c_num I2C port number
 * @param reg_address slave reg address
 * @param data data to read
 * @param data_len data length
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 *     - ESP_FAIL Sending command error, slave doesn't ACK the transfer.
 *     - ESP_ERR_INVALID_STATE I2C driver not installed or not in master mode.
 *     - ESP_ERR_TIMEOUT Operation timeout because the bus is busy.
 */
static esp_err_t i2c_master_mpu6050_read(i2c_port_t i2c_num, uint8_t devAddress,uint8_t reg_address, uint8_t *data, size_t data_len)
{        ESP_LOGE(TAG, "ESP_READ");

    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, devAddress << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_address, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    ESP_LOGE(TAG, "send reg address");

    if (ret != ESP_OK) {
        return ret;
    }

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, devAddress << 1 | READ_BIT, ACK_CHECK_EN);
    i2c_master_read(cmd, data, data_len, LAST_NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    ESP_LOGE(TAG, "read data");

    return ret;
}

static esp_err_t i2c_master_mpu6050_init(i2c_port_t i2c_num)
{
    uint8_t cmd_data;
    vTaskDelay(1000 / portTICK_RATE_MS);
    master_init();
    cmd_data =0x07;
    ESP_ERROR_CHECK(i2c_master_mpu6050_write(i2c_num,MPU6050SlaveAddress, MPU6050_REGISTER_SMPLRT_DIV, &cmd_data, 1));
    cmd_data =0x01;
    ESP_ERROR_CHECK(i2c_master_mpu6050_write(i2c_num,MPU6050SlaveAddress, MPU6050_REGISTER_PWR_MGMT_1, &cmd_data, 1));
    cmd_data =0x00;
    ESP_ERROR_CHECK(i2c_master_mpu6050_write(i2c_num,MPU6050SlaveAddress, MPU6050_REGISTER_PWR_MGMT_2, &cmd_data, 1));
    cmd_data =0x00;
    ESP_ERROR_CHECK(i2c_master_mpu6050_write(i2c_num,MPU6050SlaveAddress, MPU6050_REGISTER_CONFIG, &cmd_data, 1));
    cmd_data =0x00;
    ESP_ERROR_CHECK(i2c_master_mpu6050_write(i2c_num,MPU6050SlaveAddress, MPU6050_REGISTER_GYRO_CONFIG, &cmd_data, 1));//set +/-250 degree/second full scale
    cmd_data =0x00;
    ESP_ERROR_CHECK(i2c_master_mpu6050_write(i2c_num,MPU6050SlaveAddress, MPU6050_REGISTER_ACCEL_CONFIG, &cmd_data, 1));// set +/- 2g full scale
    cmd_data =0x00;
    ESP_ERROR_CHECK(i2c_master_mpu6050_write(i2c_num,MPU6050SlaveAddress, MPU6050_REGISTER_FIFO_EN, &cmd_data, 1));
    cmd_data =0x01;
    ESP_ERROR_CHECK(i2c_master_mpu6050_write(i2c_num,MPU6050SlaveAddress, MPU6050_REGISTER_INT_ENABLE, &cmd_data, 1));
    cmd_data =0x00;
    ESP_ERROR_CHECK(i2c_master_mpu6050_write(i2c_num,MPU6050SlaveAddress, MPU6050_REGISTER_SIGNAL_PATH_RESET, &cmd_data, 1));
    cmd_data =0x00;
    ESP_ERROR_CHECK(i2c_master_mpu6050_write(i2c_num,MPU6050SlaveAddress, MPU6050_REGISTER_USER_CTRL, &cmd_data, 1));
    ESP_LOGI	(TAG, "i2c_master_mpu6050_init");

    vTaskDelay(100 / portTICK_RATE_MS);
    return ESP_OK;
}

static void i2c_task(void *arg)
{
    uint8_t sensor_data[14];
    uint8_t who_am_i, i;
    int16_t AccelX, AccelY, AccelZ;
	double Ax, Ay, Az;
	double roll,pith;
    int ret;

	i2c_master_mpu6050_init(I2C_MASTER_NUM);

    while (1) {
        memset(sensor_data, 0, 14);
        ret = i2c_master_mpu6050_read(I2C_MASTER_NUM, MPU6050SlaveAddress,MPU6050_REGISTER_ACCEL_XOUT_H, sensor_data, 14);

        if (ret == ESP_OK) {

        	AccelX=(((int16_t)sensor_data[0] << 8) | sensor_data[1]);
        	AccelY=(((int16_t)sensor_data[2] << 8) | sensor_data[3]);
        	AccelZ=(((int16_t)sensor_data[4] << 8) | sensor_data[5]);

            //divide each with their sensitivity scale factor
            Ax = (double)AccelX/AccelScaleFactor;
            Ay = (double)AccelY/AccelScaleFactor;
            Az = (double)AccelZ/AccelScaleFactor;

            roll =  ((atan((Ay) / sqrt(pow((Ax), 2) + pow((Az), 2))) * 180 / M_PI));
            pith =  ((atan(-1 * (Ax) / sqrt(pow((Ay), 2) + pow((Az), 2))) * 180 / M_PI));
    		ESP_LOGI(TAG,"%i/%i/%i",AccelX,AccelY,AccelZ);
    		ESP_LOGI(TAG,"%.2lf/%.2lf",Ax,Ay);
    		ESP_LOGI(TAG,"%.2lf/%.2lf",roll,pith);


        } else {
            ESP_LOGE(TAG, "No ack, sensor not connected...skip...\n");
        }

        vTaskDelay(1000 / portTICK_RATE_MS);
    }

    i2c_driver_delete(I2C_EXAMPLE_MASTER_NUM);

}

esp_err_t mpu_start(uint8_t prior, uint16_t mem_us) {
    xTaskCreate(i2c_task, "i2c_task_example", 2048*2, NULL, 5, NULL);

	return ESP_OK;
}
