#ifndef COMPASS_H
#define COMPASS_H

#define ACCEL_ACTIVITY_THRESHOLD 16605
#define GYRO_FREQ 1000
#define GYRO_PERIOD (1.0/GYRO_FREQ)

#include <pthread.h>
#include <stdbool.h>
#include "i2c.h"
#include <math.h>

/*Device specific includes*/
#include "MPU9250/i2c_mpu.h"
#include "MPU9250/mpu.h"

#ifndef STRUCT_AXIS_DEFINED
#define STRUCT_AXIS_DEFINED
struct axis {
	union {
		double x;
		double pitch;
	};
	union {
		double y;
		double roll;
	};
	union {
		double z;
		double yaw;
	};
};
#endif

int compass_init(void);
//#define compass_start() your_compass_device_specific_init_function();
#define comapss_start() mpu_start()
double compass_read(void);
void compass_stop(void);

/* Read functions */
int accel_read(struct axis *a);
//#define accel_read(a) your_accel_device_specific_function(a)
#define accel_read(a) mpu_accel_read(a)
int gyro_read(struct axis *a);
//#define gyro_read(a) your_gyro_device_specific_function(a)
#define gyro_read(a) mpu_gyro_read(a)
int mag_read(struct axis *a);
//#define mag_read(a) your_mag_device_specific_function(a)
#define mag_read(a) mpu_compass_read(a)
bool mag_drdy(void);	// check for mag's new data
//#define mag_drdy() your_mag_device_specific_function()
#define mag_drdy() mpu_mag_drdy()

#endif // compass.h included
