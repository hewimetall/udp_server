#include "head.h"
static const char *TAG = "mpu_drive";

struct date_i2c plus_date_str(struct date_i2c *d1,struct date_i2c *d2){
	struct date_i2c d3;
	d3.x=d1->x+d2->x;
	d3.y=d1->y+d2->y;
	d3.z=d1->z+d2->z;
	return d3;
}
struct date_i2c del_date_str(struct date_i2c *d1,struct date_i2c *d2){
	struct date_i2c d3;
	d3.x=d1->x/d2->x;
	d3.y=d1->y/d2->y;
	d3.z=d1->z/d2->z;
	return d3;
}
struct date_i2c min_date_str(struct date_i2c *d1,struct date_i2c *d2){
	struct date_i2c d3;
	d3.x=d1->x-d2->x;
	d3.y=d1->y-d2->y;
	d3.z=d1->z-d2->z;
	return d3;
}


size_t json_serilizete(struct date_i2c *date,char *str){
	size_t len=0;
	len=sprintf (str,"{ \"name\": \"%-1s\", \"X\": %.2f, \"Y\": %.2f, \"Z\": %.2f }\n", "acel",date->x,date->y,date->z);
	str[len]='\0';
	return len;
}

void i2c_task_display(void *arg)
{
	struct date_i2c accel_D;
	struct date_i2c gyro_D;

	struct date_i2c accel_D_int;
	struct date_i2c gyro_D_int;

	mpu_test_conn();

ESP_LOGI(TAG,"START CALIBRATE");
	/* calibrate date */
for(int i=0;i<10;i++){
	for (int k = 0; k < 100; ++k) {
		mpu_get_accel_angle(&accel_D.x,&accel_D.y);
		accel_D_int=plus_date_str(&accel_D,&accel_D_int);
	}
	accel_D_int.x /=100.;
	accel_D_int.y /=100.;
}

ESP_LOGI(TAG,"STOP CALIBRATE accel");
ESP_LOGI(TAG,"DATE CALIBRATE :\n\tX:%f\n\tY:%f",accel_D_int.x,accel_D_int.y);
//
//	while (1) {
//		who_am_i = 0;
//		ret[0] = mpu_get_accel_data(&accel_D);
//		ret[1] = mpu_get_gyro_data(&gyro_D);
//
//		if ((ret[0] && ret[1]) == ESP_OK) {
//			/* Send the entire structure to the queue created to hold 10 structures. */
//			xQueueSend( /* The handle of the queue. */
//					Qdata_mpu,
//					/* The address of the xMessage variable.  sizeof( struct AMessage )
//                        bytes are copied from here into the queue. */
//					( void * ) &accel_D,
//					/* Block time of 0 says don't block if the queue is already full.
//                        Check the value returned by xQueueSend() to know if the message
//                        was sent to the queue successfully. */
//					( TickType_t ) 10/portTICK_RATE_MS );
//		} else {
//			ESP_LOGE(TAG, "No ack, sensor not connected...skip...\n");
//		}
//
//		vTaskDelay(100 / portTICK_RATE_MS);
//	}

	i2c_driver_delete(I2C_MASTER_NUM);
	vTaskDelete(NULL);
}

//size_t mpu_get_date(char *json_str){
//	struct date_i2c date;
//	size_t len=0;
//
//	len=json_serilizete(&date,json_str);
////	while(Qdata_mpu == NULL){};
//	if( xQueueReceive( Qdata_mpu,
//			&( date ),
//			portMAX_DELAY ) == pdPASS)
//	{
//		len=json_serilizete(&date,json_str);
//	}
//	return len;
//}

esp_err_t mpu_init(){
	i2c_master_mpu6050_init(I2C_MASTER_NUM);
//	Qdata_mpu = xQueueCreate(4,sizeof(Date_ini ));
	return ESP_OK;
}

esp_err_t mpu_start(uint8_t prior,uint16_t mem_us){
	xTaskCreate(i2c_task_display, "mpu_monitor", mem_us, NULL, prior, NULL);

	return ESP_OK;
}
