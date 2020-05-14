#include "head.h"
#include "mpu.h"
static const char *TAG = "MPU_TASK";

static esp_err_t i2c_master_mpu6050_write(i2c_port_t i2c_num,uint8_t devAddress, uint8_t reg_address, uint8_t *data, size_t data_len) {
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, MPU6050_SENSOR_ADDR << 1 | WRITE_BIT,
			ACK_CHECK_EN);
	i2c_master_write_byte(cmd, reg_address, ACK_CHECK_EN);
	i2c_master_write(cmd, data, data_len, ACK_CHECK_EN);
	i2c_master_stop(cmd);
	ESP_ERROR_CHECK(i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS));
	i2c_cmd_link_delete(cmd);
	return ESP_OK;
}

static esp_err_t i2c_master_mpu6050_read(i2c_port_t i2c_num, uint8_t devAddress,
		uint8_t reg_address, uint8_t *data, size_t data_len) {
	int ret;
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, devAddress << 1 | WRITE_BIT, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, reg_address, ACK_CHECK_EN);
	i2c_master_stop(cmd);
	ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);

	if (ret != ESP_OK) {
		ESP_LOGW(TAG,"ESP read fail");
		return ret;
	}

	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, devAddress << 1 | READ_BIT, ACK_CHECK_EN);
	i2c_master_read(cmd, data, data_len, LAST_NACK_VAL);
	i2c_master_stop(cmd);
	ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);

	return ret;
}

static esp_err_t i2c_master_init()
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_SDA_IO;
    conf.sda_pullup_en = 1;
    conf.scl_io_num = I2C_SCL_IO;
    conf.scl_pullup_en = 1;
    conf.clk_stretch_tick = 300;
    ESP_ERROR_CHECK(i2c_driver_install(i2c_master_port, conf.mode));
    ESP_ERROR_CHECK(i2c_param_config(i2c_master_port, &conf));
    ESP_LOGI(TAG,"i2c master initialization");
    return ESP_OK;
}


static esp_err_t i2c_master_mpu6050_init(i2c_port_t i2c_num)
{
    uint8_t cmd_data;
    vTaskDelay(100 / portTICK_RATE_MS);
    i2c_master_init();
    cmd_data =0x07;
    ESP_ERROR_CHECK(i2c_master_mpu6050_write(i2c_num,MPU6050SlaveAddress, MPU6050_REGISTER_SMPLRT_DIV, &cmd_data, 1));
    cmd_data =0x01;
    ESP_ERROR_CHECK(i2c_master_mpu6050_write(i2c_num,MPU6050SlaveAddress, MPU6050_REGISTER_PWR_MGMT_1, &cmd_data, 1));
    cmd_data =0x00;
    ESP_ERROR_CHECK(i2c_master_mpu6050_write(i2c_num,MPU6050SlaveAddress, MPU6050_REGISTER_PWR_MGMT_2, &cmd_data, 1));
    cmd_data =0x00;
    ESP_ERROR_CHECK(i2c_master_mpu6050_write(i2c_num,MPU6050SlaveAddress, MPU6050_REGISTER_CONFIG, &cmd_data, 1));
    cmd_data =0x00;
    ESP_ERROR_CHECK(i2c_master_mpu6050_write(i2c_num,MPU6050SlaveAddress, MPU6050_REGISTER_GYRO_CONFIG, &cmd_data, 1));
    cmd_data =0x00;
    ESP_ERROR_CHECK(i2c_master_mpu6050_write(i2c_num,MPU6050SlaveAddress, MPU6050_REGISTER_ACCEL_CONFIG, &cmd_data, 1));
    cmd_data =0x00;
    ESP_ERROR_CHECK(i2c_master_mpu6050_write(i2c_num,MPU6050SlaveAddress, MPU6050_REGISTER_FIFO_EN, &cmd_data, 1));
    cmd_data =0x01;
    ESP_ERROR_CHECK(i2c_master_mpu6050_write(i2c_num,MPU6050SlaveAddress, MPU6050_REGISTER_INT_ENABLE, &cmd_data, 1));
    cmd_data =0x00;
    ESP_ERROR_CHECK(i2c_master_mpu6050_write(i2c_num,MPU6050SlaveAddress, MPU6050_REGISTER_SIGNAL_PATH_RESET, &cmd_data, 1));
    cmd_data =0x00;
    ESP_ERROR_CHECK(i2c_master_mpu6050_write(i2c_num,MPU6050SlaveAddress, MPU6050_REGISTER_USER_CTRL, &cmd_data, 1));
    ESP_LOGI(TAG, "mpu6050_init ok");
    vTaskDelay(100 / portTICK_RATE_MS);
    return ESP_OK;
}


static size_t json_serilizete(double *roll,double *pith,char *str){
	size_t len=0;
	len=sprintf (str,"{ \"name\": \"%-1s\", \"roll\": %.1f, \"pith\": %.1f}\n", "acel",*roll,*pith);
	str[len]='\0';
	return len;
}

static void i2c_task(void *arg)
{
    uint8_t sensor_data[14];
    int16_t AccelX, AccelY, AccelZ;
	double Ax, Ay, Az;
	double roll,pith;
    int ret;
    char str[128];
    size_t len;
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

            roll =  ((atan2((Ay) , sqrt(pow((Ax), 2) + pow((Az), 2))) * 180 / M_PI));
            pith =  ((atan2(-1 * (Ax) , sqrt(pow((Ay), 2) + pow((Az), 2))) * 180 / M_PI));
            len=json_serilizete(&roll,&pith,&str);
            server_send_q_date(&str,len);

        } else {
            ESP_LOGE(TAG, "No ack, sensor not connected...skip...\n");
        }

        vTaskDelay(1000 / portTICK_RATE_MS);
    }

    i2c_driver_delete(I2C_MASTER_NUM);

}
esp_err_t mpu_start(uint8_t prior,uint16_t mem_us){
    xTaskCreate(i2c_task, "i2c_task", mem_us, NULL, prior, NULL);
    return ESP_OK;
}
