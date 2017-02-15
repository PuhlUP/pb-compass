#include "i2c_mpu.h"

#define AK89xx_DRDY 0x1

static i2c bus;

int mpu_i2c_write(int addr, unsigned char reg, int buffersize, unsigned char *data){
	unsigned char buffer[buffersize+1];
	buffer[0] = reg;
	int i;
	for (i = 0; i < buffersize; i++)
	{
		buffer[i+1] = data[i]; 
	}
	if(i2cWrite(bus, addr, buffer, buffersize+1)){
		return -1;
	}
	return 0;
}

int mpu_i2c_read(int addr, unsigned char reg, int buffersize, unsigned char *data){
	if(i2cWrite(bus, addr, &reg, 1)){
		return -1;
	}
	if(i2cRead(bus, addr, data, buffersize)){
		return -1;
	}
	return 0;
}

int mpu_i2c_open(){
	bus = i2cOpen(I2C_BUS_NAME);
	if(bus==NULL) return -1;

	return 0;
}

void mpu_i2c_close(){
	i2cClose(bus);
}

bool mpu_mag_drdy(void)
{
	unsigned char buffer = 0x49;

	if(i2cWrite(bus, 0x68, &buffer, 1)) {
		return false;
	}

	if(i2cRead(bus, 0x68, &buffer, 1)) {
		return false;
	}

	if(buffer & AK89xx_DRDY) {
		return true;
	} else {
		return false;
	}
}
