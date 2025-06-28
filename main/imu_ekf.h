#pragma once

typedef struct _quaternion_t
{
    float w;
    float x;
    float y;
    float z;
} quaternion_t;

typedef struct _vector3_t
{
    float x;
    float y;
    float z;
} vector3_t;

typedef struct _imu_ekf_state_t
{
    quaternion_t q;
    vector3_t gyro_bias; 
}   imu_ekf_state_t;

typedef struct _imu_raw_date_t
{
    vector3_t acce;
    vector3_t gyro;
    float dt;
} imu_raw_date_t;

// ekf 初始化
void imu_ekf_init();
// // ekf 预测阶段
// void imu_ekf_predict(float dt);
// // ekf 测量更新阶段
// void imu_ekf_update(vector3_t acc, vector3_t gyro);

// 运行调用
void imu_ekf_run(vector3_t acc, vector3_t gyro, float dt);
// ekf 先验估计
void imu_ekf_update_prior_state();
// ekf 更新状态雅可比
void imu_ekf_update_state_jacobian();
// ekf 更新先验协方差
void imu_ekf_update_prior_covariance();
// ekf 更新观测雅可比
void imu_ekf_update_observation_jacobian();
// ekf 更新卡尔曼增益 
void imu_ekf_update_kalman_gain();
// ekf 更新后验状态估计
void imu_ekf_update_posterior_state();
// ekf 卡方检验
void imu_ekf_update_posterior_covariance();

// 获取四元数
void imu_ekf_get_orientation(quaternion_t *q);
// 获取欧拉角
void imu_ekf_get_euler_angle(float *pitch, float *roll, float *yaw);
// 设置陀螺仪偏置
void imu_ekf_set_gyro_bias(float x, float y, float z);

