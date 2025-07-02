#pragma once

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define PI_VAL 3.1415926535897932384626433832795

extern float th_p_posterior, th_r_posterior, th_y_posterior;
extern float th_p_sen, th_r_sen, th_y_sen; 
extern float theta_pitch, theta_roll, theta_yaw;
// float velocity_x = 0.0, velocity_y = 0.0, velocity_z = 0.0;
extern float velocity_x, velocity_y, velocity_z;
extern float K_p, K_r, K_y;
extern float acce_scale_norm;

void kalman_filter(float acce_x, float acce_y, float acce_z, float gyro_x, float gyro_y, float gyro_z, float dt);

