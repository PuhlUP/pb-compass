#ifndef MPU_H
#define MPU_H

#include <stdio.h>
#include "inv_mpu.h"
#include "i2c_mpu.h"

#ifndef GYRO_FSR
#define GYRO_FSR 500	/*Options: 250, [500], 1000, 2000 (dps)*/
#endif
#ifndef ACCEL_FSR
#define ACCEL_FSR 2		/*Options: [2], 4, 8, 16 (g)*/
#endif
#ifndef MPU_SAMPLE_RATE
#define MPU_SAMPLE_RATE 10	/*Options are: TODO */
#endif
#ifndef AK89xx_SAMPLE_RATE
#define AK89xx_SAMPLE_RATE 100	/*Options are: TODO */
#endif

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

/*Requested by compass.h*/
int mpu_accel_read(struct axis *a);
int mpu_gyro_read(struct axis *a);
int mpu_compass_read(struct axis *a);
int mpu_start(void);

#endif
