#include "compass.h"

static struct axis mag_min, mag_max, tilt;		// z/yaw is the heading
pthread_mutex_t tilt_axis_mutex;
pthread_t head_thread_id;
static bool head_track;

int compass_init(void)
{
	tilt.x = tilt.y = tilt.z = 0;

#if defined compass_start
	if(compass_start()) {
		return -1;
	}
#endif

	if(pthread_mutex_init(&tilt_axis_mutex, NULL)) {
		return -2;
	}

	if(pthread_create(&tilt_thread_id, NULL, tilt_thread, NULL)) {
		return -3;
	}

	return 0;
}

void *head_track(void *args)
{
	struct axis accel, gyro, mag;
	head_track = true;
	unsigned char st1;

	while(head_track) {

		/*Tilt tracking*/
		accel_read(&accel);		/*Recieve accel data to determine activity status*/
		gyro_read(&gyro);

		if(sqrt(powf(accel.x,2) + powf(accel.y,2) + powf(accel.z,2)) > ACCEL_ACTIVITY_THRESHOLD) {
			/*Use gyro*/
			pthread_mutex_lock(&tilt_axis_mutex);

			/*Gyro tilt compesation (first order integration)*/
			tilt.roll += gyro.roll*(1/GYRO_FREQ);
			tilt.pitch += gyro.pitch*(1/GYRO_FREQ);

			pthread_mutex_unlick(&tilt_axis_mutex);
		} else {
			/*Use accel*/
			pthread_mutex_lock(&tilt_axis_mutex);

			/*Accel tilt compesation*/
			tilt.roll = atan(accel.y / sqrt(powf(accel.x,2) + powf(accel.z,2));
			tilt.pitch = atan(-accel.x / accel.z);

			pthread_mutex_unlick(&tilt_axis_mutex);

			/*Block here until accel report new activity,
			 * since roll and pitch will stay the same */
			while(sqrt(powf(accel.x,2) + powf(accel.y,2) + powf(accel.z,2)));
		}

		/*Head tracking*/
		if(mag_drdy()) {		/*DRDY set: mag data ready*/
			/*Use mag*/
			mag_read(&a);

			/*Soft-Iron Compesation: Data normalization*/
			a.x = 2 * (a.x - mag_min.x) / (mag_max.x - mag_min.x);
			a.y = 2 * (a.y - mag_min.y) / (mag_max.y - mag_min.y);
			a.z = 2 * (a.z - mag_min.z) / (mag_max.z - mag_min.z);

			pthread_mutex_lock(&tilt_axis_mutex);

			/*Tilt Compesation: Rotation matrix*/
			a.x = a.x * cos(-tilt.pitch)
				+ a.y * sin(-tilt.pitch) * sin(-tilt.roll)
				+ a.z * sin(tilt.pitch) * cos(tilt.roll);

			a.y = a.y * cos(-tilt.roll)
				- a.z * sin(tilt.roll);

			/*z is not needed for heading calculation
			a.z = a.x * sin(-tilt.pitch)
				+ a.y * cos(-tilt.pitch) * sin(tilt.roll)
				+ a.z * cos(-tilt.pitch) * cos(-c.roll);*/

			/*atan2 answer from radians to degrees*/
			tilt.z = 180 * atan2(y, x) / M_PI;		//TODO: + declination angle

			pthread_mutex_unlock(&tilt_axis_mutex);
		} else {
			/*Use gyro*/
			pthread_mutex_lock(&tilt_axis_mutex);

			tilt.z += a.yaw * (1/GYRO_FREQ);		//TODO: tilt comp for gyro data?

			pthread_mutex_unlock(&tilt_axis_mutex);
		}
	}

	return NULL;
}

double compass_read(void)
{
	double heading;

	pthread_mutex_lock(&tilt_axis_mutex);

	heading = tilt.z;

	pthread_mutex_unlock(&tilt_axis_mutex);

	return heading;
}

void compass_stop(void)
{
	tilt_track = false;

	pthread_join(tilt_thread_id);
	pthread_mutex_destroy(tilt_axis_mutex);
}
