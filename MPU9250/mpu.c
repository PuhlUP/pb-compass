#include "mpu.h"

int mpu_accel_read(struct axis *a){
	short data[3];

	if(mpu_get_accel_reg(data)) {
		log_e("mpu_get_accel_reg error\n");
		return -1;
	} else {
		a->x = data[0];
		a->y = data[1];
		a->z = data[2];
	}

	return 0;
}

int mpu_gyro_read(struct axis *a){
	short data[3];

	if(mpu_get_gyro_reg(data)) {
		log_e("mpu_get_gyro_reg error\n");
		return -1;
	} else {
		a->x = data[0];
		a->y = data[1];
		a->z = data[2];
	}

	return 0;
}

int mpu_compass_read(struct axis *a){
	short data[3];

	if(mpu_get_compass_reg(data)){
		log_e("mpu_get_compass_reg error\n");
		return -1;
	} else {
		a->x = data[0];
		a->y = data[1];
		a->z = data[2];
	}

	return 0;
}

int mpu_offset_gyro_accel_set(){
	long gBias[3]={0,0,0};
	long aBias[3]={0,0,0};
	FILE *offset_data_gyro, *offset_data_accel;

	offset_data_gyro = fopen("./offset_data_gyro","r");
	if(offset_data_gyro==NULL){
		perror("Cannot open offset_data_gyro file");
		return -1;
	}
	fscanf(offset_data_gyro, "g %ld %ld %ld\n", &gBias[0], &gBias[1], &gBias[2]);
	fclose(offset_data_gyro);

	if(mpu_set_gyro_bias_reg(gBias)){
		fprintf(stderr, "mpu_set_gyro_bias_reg error\n");
		return -1;
	}

	offset_data_accel = fopen("offset_data_accel","r");
	if(offset_data_accel==NULL){
		perror("Cannot open offset_data_accel file");
		return -1;
	}
	fscanf(offset_data_accel, "a %ld %ld %ld\n", &aBias[0], &aBias[1], &aBias[2]);
	fclose(offset_data_accel);

	if(mpu_set_accel_bias_6500_reg(aBias)){
		fprintf(stderr, "mpu_set_accel_bias_6500_reg error\n");
		return -1;
	}

	return 0;
}

int mpu_start(void) {
#ifndef I2C_BUS_NAME
#error "I2C_BUS_NAME not defined. See mpu.h:23"
#endif

	if(mpu_i2c_open()) {
		fprintf(stderr, "mpu_i2c_open fail.\n");
		return -1;
	}

	if(mpu_set_sensors(INV_XYZ_GYRO + INV_XYZ_ACCEL  + INV_XYZ_COMPASS)) {
		fprintf(stderr, "MPU axis activation fail.\n");
		return -1;
	}

	if(mpu_set_gyro_fsr(GYRO_FSR)) {
		fprintf(stderr, "MPU's gyro scale configuration fail.\n");
		return -1;
	}

	if(mpu_set_accel_fsr(ACCEL_FSR)) {
		fprintf(stderr, "MPU's accel scale configuration fail.\n");
		return -1;
	}

	if(mpu_set_sample_rate(MPU_SAMPLE_RATE)) {
		fprintf(stderr, "MPU sample rate configuration fail.\n");
		return -1;
	}

	if(mpu_set_compass_sample_rate(AK89xx_SAMPLE_RATE)) {
		fprintf(stderr, "MPU's compass sample rate configuration fail.\n");
		return -1;
	}

}
