#ifndef AUTOMATED_CONTROL_H
#define AUTOMATED_CONTROL_H

#include <config.h>
#include "Navdata/navdata.h"

C_RESULT automated_init(void);
C_RESULT automated_update(void);
C_RESULT automated_shutdown(void);

float pid(float kp, float ki, float kd, 
					float error, float *sum, float *old);

void set_goal(float x, float y, float z, float orientation);

void set_navdata(float32_t theta, float32_t psi, float32_t phi, 
							int32_t altitude, float32_t vx, float32_t vy, float32_t vz);
#endif
