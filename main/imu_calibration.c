#include <math.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "imu_calibration.h"

#define IMU_CALIBRATION_ACCE_SAMPLE_NUM 400
#define IMU_CALIBRATION_GYRO_SAMPLE_NUM 400
float acce_scale = 0.994255;
float gyro_offset_x = 0.047866, gyro_offset_y = 0.055945, gyro_offset_z = -0.047561;
float acce_cal_x = 0.0, acce_cal_y = 0.0, acce_cal_z = 0.0;

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

void eliminate_gravity_acceleration(float acce_x, float acce_y, float acce_z, float roll, float pitch)
{
    acce_cal_x = acce_x + G_ACCE * sin(pitch / 180.0 * M_PI);
    acce_cal_y = acce_y - G_ACCE * cos(pitch / 180.0 * M_PI) * sin(roll / 180.0 * M_PI);
    acce_cal_z = acce_z - G_ACCE * cos(pitch / 180.0 * M_PI) * cos(roll / 180.0 * M_PI);
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
        system_delay_ms(10);
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