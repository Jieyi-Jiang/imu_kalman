#pragma once

#include "zf_device_imu660ra.h"

#define STANDARD_GRAVITY_ACCELERATION  9.80665
#define G_ACCE  9.80665

extern float acce_scale;
extern float gyro_offset_x, gyro_offset_y, gyro_offset_z;
extern float acce_cal_x, acce_cal_y, acce_cal_z;
extern float acce_A[3][3];
extern float acce_bias[3][1];
extern float P_p_prior, P_r_prior, P_y_prior;
extern float P_p_posterior, P_r_posterior, P_y_posterior;
extern float R_p, R_r, R_y;

void calibrate_acce_scale();
void calibrate_gyro_zero_offest();
void eliminate_gravity_acceleration(float acce_x, float acce_y, float acce_z, float roll, float pitch);

inline void imu660ra_calibration()
{
    // calibrate_acce_scale(); 
    calibrate_gyro_zero_offest();
}
inline float imu660ra_get_gyro_x()
{
    return (imu660ra_gyro_transition(imu660ra_gyro_x) - gyro_offset_x);
}
inline float imu660ra_get_gyro_y()
{
    return (imu660ra_gyro_transition(imu660ra_gyro_y) - gyro_offset_y);
}

inline float imu660ra_get_gyro_z()
{
    return (imu660ra_gyro_transition(imu660ra_gyro_z) - gyro_offset_z);
}

inline float imu660ra_get_acce_x()
{
    return -imu660ra_acc_transition(imu660ra_acc_x) / acce_scale * STANDARD_GRAVITY_ACCELERATION;
}

inline float imu660ra_get_acce_y()
{
    return -imu660ra_acc_transition(imu660ra_acc_y) / acce_scale * STANDARD_GRAVITY_ACCELERATION;
}

inline float imu660ra_get_acce_z()
{
    return -imu660ra_acc_transition(imu660ra_acc_z) / acce_scale * STANDARD_GRAVITY_ACCELERATION;
}