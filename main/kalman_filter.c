#include "kalman_filter.h"
#include "imu_calibration.h"

// 传感器测量值
// float gyro_x = 0.0, gyro_y = 0.0, gyro_z = 0.0;
// float acce_x = 0.0, acce_y = 0.0, acce_z = 0.0;
float th_p_sen = 0.0, th_r_sen = 0.0, th_y_sen = 0.0; 
// 过程噪声协方差矩阵（扰动协方差矩阵）-> Gyro
float Q_p = 0.01, Q_r = 0.01, Q_y = 0.01;
// 观测噪声协方差矩阵 -> Acce 
float R_p = 2.0, R_r = 2.0, R_y = .0;
// 状态协方差矩阵（误差协方差矩阵）
float P_p_prior = 0.0, P_r_prior = 0.0, P_y_prior = 0.0;
float P_p_posterior = 0.0, P_r_posterior = 0.0, P_y_posterior = 0.0;

// 状态向量
float th_p_prior = 0.0, th_r_prior = 0.0, th_y_prior = 0.0;
float th_p_posterior = 0.0, th_r_posterior = 0.0, th_y_posterior = 0.0;

// 欧拉角输出
float theta_pitch, theta_roll, theta_yaw;

// 卡尔曼增益
float K_p, K_r, K_y;

// 速度积分值
float velocity_x = 0.0, velocity_y = 0.0, velocity_z = 0.0;

typedef struct {
    float value;          // 当前值
    float last_value;     // 上一个值
    float alpha;          // 平滑系数，范围在0到1之间，越小越平滑
} low_pass_filter_t;


float low_pass_filter(low_pass_filter_t *filter, float new_value)
{
    filter->last_value = filter->value;
    filter->value = filter->alpha * new_value + (1 - filter->alpha) * filter->last_value;
    return filter->value;
}

float ragular_to_range_180(float angle)
{
    float ret = 0.0;
    if (angle > 180.0)
    {
        ret = angle - 360.0;
    }
    else if (angle < -180.0)
    {
        ret = angle + 360.0;
    }
    else
    {
        ret = angle;
    }
    return ret;
}

float bound_to_90(float angle)
{
    float ret = 0.0;
    if (angle > 90.0)
    {
        ret = 90.0;
    }
    else if (angle < -90.0)
    {
        ret = -90.0;
    }
    else
    {
        ret = angle;
    }
    return ret;
}

void acce_to_pitch(float acce_x, float acce_y, float acce_z)
{
    th_p_sen =  atan2(-acce_x, sqrt(acce_z*acce_z + acce_y*acce_y)) / PI_VAL * 180.0;
}

void acce_to_roll(float acce_x, float acce_y, float acce_z)
{
    th_r_sen = atan2(acce_y, acce_z) / PI_VAL * 180.0;
}

float acce_scale_norm;
low_pass_filter_t acce_scale_norm_filter = {
    .value = 0.0,
    .last_value = 0.0,
    .alpha = 0.3, // 平滑系数
};

low_pass_filter_t theta_pitch_filter = {
    .value = 0.0,
    .last_value = 0.0,
    .alpha = 0.3, // 平滑系数
};
low_pass_filter_t theta_roll_filter = {
    .value = 0.0,
    .last_value = 0.0,
    .alpha = 0.3, // 平滑系数
};
low_pass_filter_t theta_yaw_filter = {
    .value = 0.0,
    .last_value = 0.0,
    .alpha = 0.3, // 平滑系数
};

void kalman_filter(float acce_x, float acce_y, float acce_z, float gyro_x, float gyro_y, float gyro_z, float dt)
{
    float acce_scale = sqrt(acce_x * acce_x + acce_y * acce_y + acce_z * acce_z);
    // float K = 10000.0;
    acce_scale_norm = fabs(acce_scale - 1.0);
    acce_scale_norm = low_pass_filter(&acce_scale_norm_filter, acce_scale_norm);
    static int acce_scale_norm_cnt = 0;
    if (acce_scale_norm > 0.02)
    {
        R_p = 1e9;
        R_r = 1e9;
        acce_scale_norm_cnt = 0;
    }
    else if (acce_scale_norm > 0.01 && acce_scale_norm <= 0.02)
    {
        R_p = 1000.0;
        R_r = 1000.0;
        acce_scale_norm_cnt = 0;
    }
    else if (acce_scale_norm > 0.005 && acce_scale_norm <= 0.01)
    {
        R_p = 10.0;
        R_r = 10.0;
        acce_scale_norm_cnt = 0;
    }
    else
    {
        acce_scale_norm_cnt += 1;
    }

    if (acce_scale_norm_cnt > 50)
    {
        acce_scale_norm_cnt = 0;
        R_p = 2.0;
        R_r = 2.0;
    }
    // R_p = 1000;
    // R_r = 1000;

    // R_p = 10.0 + K * acce_scale_norm;
    // R_r = 10.0 + K * acce_scale_norm;

    acce_to_pitch(acce_x, acce_y, acce_z);
    acce_to_roll(acce_y, acce_y, acce_z);
    // float dt  = 0.01;
    // 更新先验协方差矩阵
    P_p_prior = P_p_posterior + Q_p;
    P_r_prior = P_r_posterior + Q_r;
    
    // 更新先验状态估计
    th_p_prior = bound_to_90(th_p_posterior + dt * gyro_y);
    th_r_prior = ragular_to_range_180(th_r_posterior + dt * gyro_x);
    
    // 更新卡尔曼增益
    K_p = P_p_prior / (P_p_prior + R_p);
    K_r = P_r_prior / (P_r_prior + R_r);
    // if (acce_scale_norm > 0.01)
    // {
    //     K_p = 1.0;
    //     K_r = 1.0;
    // }
    // else
    // {
    //     K_p = P_p_prior / (P_p_prior + R_p);
    //     K_r = P_r_prior / (P_r_prior + R_r);
    // }
    // 更新后验状态估计
    th_p_posterior = th_p_prior + K_p * (th_p_sen - th_p_prior);
    th_r_posterior = th_r_prior + K_r * (th_r_sen - th_r_prior);
    
    // 更新后验状态估计的协方差
    P_p_posterior = (1 - K_p) * P_p_prior;  
    P_r_posterior = (1 - K_r) * P_r_prior;
    
    // yaw 单纯用积分处理
    th_y_posterior = ragular_to_range_180(th_y_posterior + dt * gyro_z);

    // theta_pitch = low_pass_filter(&theta_pitch_filter, th_p_posterior);
    // theta_roll  = low_pass_filter(&theta_roll_filter, th_r_posterior);
    // theta_yaw   = low_pass_filter(&theta_yaw_filter, th_y_posterior);
    // th_p_posterior = theta_pitch;
    // th_r_posterior = theta_roll;
    // th_y_posterior = theta_yaw;
    theta_pitch = th_p_posterior;
    theta_roll  = th_r_posterior;
    theta_yaw   = th_y_posterior;
    // if (acce_cal_x > 0.02 || acce_cal_x < -0.02)
    // {
    //     velocity_x = velocity_x + dt *  acce_cal_x;
    // }
    // if (acce_cal_y > 0.02 || acce_cal_y < -0.02)
    // {
    //     velocity_y = velocity_y + dt *  acce_cal_y;
    // }
    // if (acce_cal_z > 0.02 || acce_cal_z < -0.02)
    // {
    //     velocity_z = velocity_z + dt *  acce_cal_z;
    // }
    return;
}   