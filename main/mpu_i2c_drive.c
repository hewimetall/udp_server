

#include "head.h"

static const char *TAG = "mpu_i2c_drive";
#define I2C_MASTER_TX_BUF_DISABLE   0               /*!< I2C master do not need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0               /*!< I2C master do not need buffer */

#define MPU6050_SENSOR_ADDR                 0x68            /*!< slave address for MPU6050 sensor */
#define MPU6050_CMD_START                   0x41            /*!< Command to set measure mode */
#define MPU6050_WHO_AM_I                    0x75            /*!< Command to read WHO_AM_I reg */
#define WRITE_BIT                           I2C_MASTER_WRITE/*!< I2C master write */
#define READ_BIT                            I2C_MASTER_READ /*!< I2C master read */
#define ACK_CHECK_EN                        0x1             /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS                       0x0             /*!< I2C master will not check ack from slave */
#define ACK_VAL                             0x0             /*!< I2C ack value */
#define NACK_VAL                            0x1             /*!< I2C nack value */
#define LAST_NACK_VAL                       0x2             /*!< I2C last_nack value */

/**
 * Define the mpu6050 register address:
 */

#define PWR_MGMT_1      0x6B  /*!< Disable sleep mode */
#define WHO_AM_I        0x75  /*!< Command to read WHO_AM_I reg */

#define SMPLRT_DIV      0x19 /*!< Gyro Sample Rate dividing */
#define CONFIG          0x1A /*!< This register configures the external Frame Synchronization (FSYNC) pin sampling and the Digital Low Pass Filter (DLPF) setting for both the gyroscopes and accelerometers */
#define GYRO_CONFIG     0x1B /*!< This register configures Full Scale Range and Self-Test gyroscopes */
#define ACCEL_CONFIG    0x1C /*!< This register configures Full Scale Range and Self-Test accelerometers */


#define GYRO_RANGE		0x08 /* 2000 grad/sec */
#define ACCEL_RANGE		0x09 /* 8g */

#define ACCEL_XOUT_H    0x3B
#define ACCEL_XOUT_L    0x3C
#define ACCEL_YOUT_H    0x3D
#define ACCEL_YOUT_L    0x3E
#define ACCEL_ZOUT_H    0x3F
#define ACCEL_ZOUT_L    0x40

#define TEMP_OUT_H      0x41
#define TEMP_OUT_L      0x42

#define GYRO_XOUT_H     0x43
#define GYRO_XOUT_L     0x44
#define GYRO_YOUT_H     0x45
#define GYRO_YOUT_L     0x46
#define GYRO_ZOUT_H     0x47
#define GYRO_ZOUT_L     0x48


#define CONVERSIONG 3.9  // convert the raw readings to the g unit (1g = 9.8 m/s²) depends on sensor settings


void min_str_date(struct date_i2c *d1 ,struct date_i2c *d2 ){
	d1->x = d1->x -d2->x;
	d1->y = d1->y -d2->y;
	d1->z = d1->z -d2->z;
}
/* date pipline */
QueueHandle_t Qdata_mpu =NULL;

/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init()
{
	int i2c_master_port = I2C_MASTER_NUM;
	i2c_config_t conf;
	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = I2C_MASTER_SDA_IO;
	conf.sda_pullup_en = 1;
	conf.scl_io_num = I2C_MASTER_SCL_IO;
	conf.scl_pullup_en = 1;
	conf.clk_stretch_tick = 300; // 300 ticks, Clock stretch is about 210us, you can make changes according to the actual situation.
	ESP_ERROR_CHECK(i2c_driver_install(i2c_master_port, conf.mode));
	ESP_ERROR_CHECK(i2c_param_config(i2c_master_port, &conf));
	return ESP_OK;
}

/**
 * @brief write mpu6050
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
static esp_err_t i2c_master_mpu6050_write(i2c_port_t i2c_num, uint8_t reg_address, uint8_t *data, size_t data_len)
{
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
 * @brief code to read mpu6050
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


static esp_err_t i2c_master_mpu6050_read(i2c_port_t i2c_num, uint8_t reg_address, uint8_t *data, size_t data_len)
{
	int ret;
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, MPU6050_SENSOR_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, reg_address, ACK_CHECK_EN);
	i2c_master_stop(cmd);
	ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);

	if (ret != ESP_OK) {
		return ret;
	}

	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, MPU6050_SENSOR_ADDR << 1 | READ_BIT, ACK_CHECK_EN);
	i2c_master_read(cmd, data, data_len, LAST_NACK_VAL);
	i2c_master_stop(cmd);
	ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);

	return ret;
}

 esp_err_t i2c_master_mpu6050_init(i2c_port_t i2c_num)
{
	uint8_t cmd_data;
	vTaskDelay(100 / portTICK_RATE_MS);
	i2c_master_init();  // init i2c proto
	cmd_data = 0x00;    // reset mpu6050
	ESP_ERROR_CHECK(i2c_master_mpu6050_write(i2c_num, PWR_MGMT_1, &cmd_data, 1));
	cmd_data = 0x07;    // Set the SMPRT_DIV
	ESP_ERROR_CHECK(i2c_master_mpu6050_write(i2c_num, SMPLRT_DIV, &cmd_data, 1));
	cmd_data = 0x06;    // Set the Low Pass Filter set
	ESP_ERROR_CHECK(i2c_master_mpu6050_write(i2c_num, CONFIG, &cmd_data, 1));
	cmd_data = GYRO_RANGE;    // Set the GYRO range
	ESP_ERROR_CHECK(i2c_master_mpu6050_write(i2c_num, GYRO_CONFIG, &cmd_data, 1));
	cmd_data = ACCEL_RANGE;    // Set the ACCEL range
	ESP_ERROR_CHECK(i2c_master_mpu6050_write(i2c_num, ACCEL_CONFIG, &cmd_data, 1));
	return ESP_OK;
}

static  esp_err_t mpu_get_accel_data(float *ax,float  *ay,float *az,float d){
	int ret;
	uint8_t data_int[6];
	*ax=0,*ay=0,*az=0;
	memset(data_int, 0, 6);
	ret = i2c_master_mpu6050_read(I2C_MASTER_NUM, ACCEL_XOUT_H, data_int, 6);
	if (ret != ESP_OK) {
		ESP_LOGW(TAG,"Error");
		return ret;
	}
	*ax=((data_int[0] << 8) | data_int[1])/d;
	*ay=((data_int[2] << 8) | data_int[3])/d;
	*az=((data_int[4] << 8) | data_int[5])/d;

	return ESP_OK;
}

esp_err_t mpu_get_accel_angle(float *accAngleX,float *accAngleY){
	int err;
	float ax=0,ay=0,az=0;
	float x,y,z;
	err=mpu_get_accel_data(&ax,&ay,&az,16384.0);

	if(err !=ESP_OK){
		return ESP_FAIL;
	}
	*accAngleX = (atan(ax / sqrt(pow(ax, 2) + pow(az, 2))) * 180 / M_PI) ;
	*accAngleY = (atan(-1 * ax / sqrt(pow(ay, 2) + pow(az, 2))) * 180 / M_PI) ;

//	x= atan (ax/sqrt(ay*ay + az*az))*(180 /M_PI);
//	y= atan (*ay/sqrt(ax*ax + az*az))*(180 /M_PI);
//	z= atan (az/sqrt(ax*ax + az*az))*(180 /M_PI);

//	  angle_z_accel = atan(accel_z_scalled/(sqrt(accel_y_scalled*accel_y_scalled+accel_x_scalled*accel_x_scalled)))*(float)rad2degree;
//	  angle_y_accel = -atan(accel_y_scalled/(sqrt(accel_y_scalled*accel_y_scalled+accel_z_scalled*accel_z_scalled)))*(float)rad2degree;
//	  angle_x_accel = atan(accel_x_scalled/(sqrt(accel_x_scalled*accel_x_scalled+accel_z_scalled*accel_z_scalled)))*(float)rad2degree;

#ifdef DEBUG_INFO
	ESP_LOGI(TAG,"sensor accelerometers_X %d\n",accAngleX);
	ESP_LOGI(TAG,"sensor accelerometers_Y %d",accAngleY);
#endif
	return ESP_OK;

}

static esp_err_t mpu_get_gyro_data(float *x,float *y,float *z,float d){
	int ret;
	uint8_t data_int[6];
	memset(data_int, 0, 6);
	ret = i2c_master_mpu6050_read(I2C_MASTER_NUM, GYRO_XOUT_H, data_int, 6);
	if (ret != ESP_OK) {
		return ret;
	}

	*x=((data_int[0] << 8) | data_int[1])/d;
	*y=((data_int[2] << 8) | data_int[3])/d;
	*z=((data_int[4] << 8) | data_int[5])/d;

#ifdef DEBUG_INFO
	ESP_LOGI(TAG,"sensor gyroscopes X:%i",x);
	ESP_LOGI(TAG,"sensor gyroscopes Y:%i",y);
	ESP_LOGI(TAG,"sensor gyroscopes Z:%i",z);
#endif

	return ESP_OK;

}

esp_err_t mpu_get_gyro_angle(){
 /* */
	return ESP_OK;
}

esp_err_t mpu_get_temp_data(uint16_t *temp){
	int ret;
	uint8_t data_int[2];

	memset(data_int, 0, 2);
	ret = i2c_master_mpu6050_read(I2C_MASTER_NUM, TEMP_OUT_H, data_int, 2);
	*temp=36.53 + ((double)(int16_t)((data_int[0] << 8) | data_int[2]) / 340);

#ifdef DEBUG_INFO
	ESP_LOGI(TAG, "TEMP: %d.%d\n", (uint16_t)*temp, (uint16_t)(*temp * 100) % 100);
#endif

	return ret;

}

esp_err_t mpu_test_conn(){

	uint8_t who_am_i;
	int ret[2];
	memset(ret,0,2);


	do{

		i2c_master_mpu6050_read(I2C_MASTER_NUM, WHO_AM_I, &who_am_i, 1);

		if (0x68 != who_am_i) {
			ESP_LOGW("who_am_i != 0x68 ,who_am_i %x",who_am_i);
		}

	}while(0x68 != who_am_i);

	return ESP_OK;
}
