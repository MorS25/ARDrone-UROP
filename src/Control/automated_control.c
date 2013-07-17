#include <Control/automated_control.h>
#include <Control/joypad_control.h>

#include <ardrone_tool/UI/ardrone_input.h>
#include <ardrone_api.h>
#include <SDL/SDL.h>

#include <math.h>

float goalX;
float goalY;
float goalZ;
float goalOri;

float32_t navdata_theta;
float32_t navdata_psi;
float32_t navdata_phi;

int32_t navdata_altitude;

float32_t navdata_vx;
float32_t navdata_vy;
float32_t navdata_vz;

/* Position of the drone, origin being starting position */
float pos_drone_y = 0;
float pos_drone_z = 0;

/* PID gains */
float kp_gaz = 0;
float ki_gaz = 0;
float kd_gaz = 0;

float kp_roll = 0;
float ki_roll = 0;
float kd_roll = 0;

float kp_pitch = 0;
float ki_pitch = 0;
float kd_pitch = 0;

float kp_yaw = 0;
float ki_yaw = 0;
float kd_yaw = 0;

/* PID Integrals */
float sum_gaz_error = 0;
float sum_roll_error = 0;
float sum_pitch_error = 0;
float sum_yaw_error = 0;

/* PID Derivatives */
float old_gaz_error = 0;
float old_roll_error = 0;
float old_pitch_error = 0;
float old_yaw_error = 0;

void set_navdata(float32_t theta, float32_t psi, float32_t phi, 
		           int32_t altitude, float32_t vx, float32_t vy, float32_t vz)
{
	navdata_altitude = altitude;

	navdata_vx = vx;
  navdata_vy = vy;
  navdata_vz = vz;

	navdata_theta = theta;
	navdata_psi = psi;
	navdata_phi = phi;
}
void set_goal(float x, float y, float z, float orientation)
{
	pos_drone_y = 0;
	pos_drone_z = 0;

	goalX = x + navdata_altitude;
	goalY = y;
	goalZ = z;
	goalOri = orientation;
}

float pid(float kp, float ki, float kd, 
					float error, float *sum, float *old)
{
	float r = kp * error + ki * (*sum + error) + kd * (error - *old);
	*sum += error;
	*old = error;
	return r;
}

C_RESULT automated_init(void)
{
	return C_OK;
}

C_RESULT automated_update(void)
{
	float a = 0;
	float gaz = 0;
	float roll = 0;
	float pitch = 0;
	float yaw = 0;

	printf("Altitude = %i\n", navdata_altitude);
	printf("vx = %f\n", navdata_vx);
	printf("vy = %f\n", navdata_vy);
	printf("vz = %f\n", navdata_vz);
	printf("theta = %f\n", navdata_theta);
	printf("phi = %f\n", navdata_phi);
	printf("psi = %f\n", navdata_psi);

	gaz = pid(kp_gaz, ki_gaz, kd_gaz, 
				goalX - navdata_altitude, &sum_gaz_error, &old_gaz_error);

	a = atan2(goalY - pos_drone_y, goalX - navdata_altitude) 
					- navdata_phi;
	roll = pid(kp_roll, ki_roll, kd_roll, 
					atan2(sin(a), cos(a)), &sum_roll_error, &old_roll_error);

	a = atan2(goalX - navdata_altitude, goalZ - pos_drone_z) 
					- navdata_theta;
	pitch = pid(kp_pitch, ki_pitch, kd_pitch, 
					atan2(sin(a), cos(a)), &sum_pitch_error, &old_pitch_error);

	a = goalOri - navdata_psi;
	yaw = pid(kp_yaw, ki_yaw, kd_yaw, 
					atan2(sin(a), cos(a)), &sum_yaw_error, &old_yaw_error);

	//ardrone_tool_set_progressive_cmd( 0, roll, pitch, gaz, yaw, 0.0, 0.0);

	/* Update pos_drone_y and pos_drone_z based on velocity */
  return C_OK;
}

C_RESULT automated_shutdown(void)
{
  printf("Shutdown\n");
  ardrone_tool_set_ui_pad_start(0);
  return C_OK;
}
