#ifndef I2C_MPU_H
#define I2C_MPU_H

#include "i2c.h"

#ifndef I2C_BUS_NAME
#define I2C_BUS_NAME "/dev/i2c-0"
#endif

/*Requested by inv_mpu.h*/
int mpu_i2c_write(int addr, unsigned char reg, int buffersize, unsigned char *data);
int mpu_i2c_read(int addr, unsigned char reg, int buffersize, unsigned char *data);
int mpu_i2c_open(const char *filename);
void mpu_i2c_close();

/*Requested by compass.h*/
bool mpu_mag_drdy(void);

#endif // i2c_mpu.h included
