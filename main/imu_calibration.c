#include <math.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "imu_calibration.h"
#include "kalman_filter.h"

#define IMU_CALIBRATION_ACCE_SAMPLE_NUM 400
#define IMU_CALIBRATION_GYRO_SAMPLE_NUM 400
float acce_scale = 0.994255;
float gyro_offset_x = 0.047866, gyro_offset_y = 0.055945, gyro_offset_z = -0.047561;
float acce_cal_x = 0.0, acce_cal_y = 0.0, acce_cal_z = 0.0;

float acce_A[3][3] = {
    {1.000050, 0.000257, 0.000210},
    {0.000257, 1.000965, 0.003950},
    {0.000210, 0.003950, 1.001645}};
float acce_bias[3][1] = {
    {0.012272},
    {-0.022023},
    {0.003605}};

void calibrate_acce_scale()
{
    float ax, ay, az;
    float sum_x = 0.0, sum_y = 0.0 , sum_z = 0.0;
    float average_x, average_y, average_z;
    for (int i = 0; i < 50; ++ i)
    {
        imu660ra_get_acc();
        ax = imu660ra_acc_transition(imu660ra_acc_x);
        ay = imu660ra_acc_transition(imu660ra_acc_y);
        az = imu660ra_acc_transition(imu660ra_acc_z);
        system_delay_ms(10);
    }
    for (int i = 0; i < IMU_CALIBRATION_ACCE_SAMPLE_NUM; ++ i)
    {
        imu660ra_get_acc();
        ax = imu660ra_acc_transition(imu660ra_acc_x);
        ay = imu660ra_acc_transition(imu660ra_acc_y);
        az = imu660ra_acc_transition(imu660ra_acc_z);
        sum_x += ax;
        sum_y += ay;
        sum_z += az;
        system_delay_ms(10);
    }
    average_x = sum_x / IMU_CALIBRATION_ACCE_SAMPLE_NUM;
    average_y = sum_y / IMU_CALIBRATION_ACCE_SAMPLE_NUM;
    average_z = sum_z / IMU_CALIBRATION_ACCE_SAMPLE_NUM;
    acce_scale = sqrt(average_x * average_x + average_y * average_y + average_z * average_z);
    printf("acce_scale = %f\n", acce_scale);
}

low_pass_filter_t acce_cal_x_filter = {
    .value = 0.0,
    .last_value = 0.0,
    .alpha = 0.5, // 平滑系数
};
low_pass_filter_t acce_cal_y_filter = {
    .value = 0.0,
    .last_value = 0.0,
    .alpha = 0.5, // 平滑系数
};
low_pass_filter_t acce_cal_z_filter = {
    .value = 0.0,
    .last_value = 0.0,
    .alpha = 0.5, // 平滑系数
};
void eliminate_gravity_acceleration(float acce_x, float acce_y, float acce_z, float roll, float pitch)
{
    acce_cal_x = G_ACCE * (acce_x + sin(pitch / 180.0 * M_PI));
    acce_cal_y = G_ACCE * (acce_y - cos(pitch / 180.0 * M_PI) * sin(roll / 180.0 * M_PI));
    acce_cal_z = G_ACCE * (acce_z - cos(pitch / 180.0 * M_PI) * cos(roll / 180.0 * M_PI));
    acce_cal_x = low_pass_filter(&acce_cal_x_filter, acce_cal_x);
    acce_cal_y = low_pass_filter(&acce_cal_y_filter, acce_cal_y);
    acce_cal_z = low_pass_filter(&acce_cal_z_filter, acce_cal_z);
    // printf("acce_cal_x = %f, acce_cal_y = %f,
}

void calibrate_gyro_zero_offest()
{
    float gyrx, gyry, gyrz;
    float sum_x = 0.0, sum_y = 0.0 , sum_z = 0.0;
    float average_x, average_y, average_z;
    for (int i = 0; i < 50; ++ i)
    {
        imu660ra_get_gyro();
        gyrx = imu660ra_gyro_transition(imu660ra_gyro_x);
        gyry = imu660ra_gyro_transition(imu660ra_gyro_y);
        gyrz = imu660ra_gyro_transition(imu660ra_gyro_z);
        system_delay_ms(10);
    }
    for (int i = 0; i < IMU_CALIBRATION_GYRO_SAMPLE_NUM; ++ i)
    {
        imu660ra_get_gyro();
        gyrx = imu660ra_gyro_transition(imu660ra_gyro_x);
        gyry = imu660ra_gyro_transition(imu660ra_gyro_y);
        gyrz = imu660ra_gyro_transition(imu660ra_gyro_z);
        sum_x += gyrx;
        sum_y += gyry;
        sum_z += gyrz;
        system_delay_ms(2);
    }
    average_x = sum_x / IMU_CALIBRATION_GYRO_SAMPLE_NUM;
    average_y = sum_y / IMU_CALIBRATION_GYRO_SAMPLE_NUM;
    average_z = sum_z / IMU_CALIBRATION_GYRO_SAMPLE_NUM;
    gyro_offset_x = average_x;
    gyro_offset_y = average_y;
    gyro_offset_z = average_z;
    printf("gyro_offset_x = %f\n", gyro_offset_x);
    printf("gyro_offset_y = %f\n", gyro_offset_y);
    printf("gyro_offset_z = %f\n", gyro_offset_z);
}