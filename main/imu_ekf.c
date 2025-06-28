#include "imu_ekf.h"
#include "math.h"

#define Q_CONST_1 0.0001
#define Q_CONST_2 0.0001
#define R_CONST 0.01
#define FADING_FACTOR 0.9996


////////////////////////////// 状态量定义 /////////////////////////////
// IMU 原始数据
imu_raw_date_t raw_date = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, 0.0};
// 先验状态向量
imu_ekf_state_t state_neg = {{1.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
// 后验状态向量
imu_ekf_state_t state_pos = {{1.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
// acce 测量向量
vector3_t acce_m = {0.0, 0.0, 0.0};
// 测量误差向量
vector3_t acce_m_err = {0.0, 0.0, 0.0};
///////////////////////////// 雅可比矩阵定义 ///////////////////////////
// 状态雅可比矩阵
static float F_jor[7][7] = {
    {1.0, 0.0, 0.0, 0.0, /**/ 0.0, 0.0, 0.0}, /*0*/
    {0.0, 1.0, 0.0, 0.0, /**/ 0.0, 0.0, 0.0}, /*1*/
    {0.0, 0.0, 1.0, 0.0, /**/ 0.0, 0.0, 0.0}, /*2*/
    {0.0, 0.0, 0.0, 1.0, /**/ 0.0, 0.0, 0.0}, /*3*/
    //------------------------------------//
    {0.0, 0.0, 0.0, 0.0, /**/ 1.0, 0.0, 0.0}, /*4*/
    {0.0, 0.0, 0.0, 0.0, /**/ 0.0, 1.0, 0.0}, /*5*/
    {0.0, 0.0, 0.0, 0.0, /**/ 0.0, 0.0, 1.0}, /*6*/};
// 观测雅可比矩阵
static float H_jor[3][4] = {
    {0.0, 0.0, 0.0, 0.0}, /*0*/
    {0.0, 0.0, 0.0, 0.0}, /*1*/
    {0.0, 0.0, 0.0, 0.0}, /*2*/
    {0.0, 0.0, 0.0, 0.0}, /*3*/};
//////////////////////////// 协方差矩阵定义 ////////////////////////////
// 过程噪声协方差矩阵
// 假设相互独立
static float Q_c[7][7] = { 
    {Q_CONST_1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, /*0*/
    {0.0, Q_CONST_1, 0.0, 0.0, 0.0, 0.0, 0.0}, /*1*/
    {0.0, 0.0, Q_CONST_1, 0.0, 0.0, 0.0, 0.0}, /*2*/
    {0.0, 0.0, 0.0, Q_CONST_1, 0.0, 0.0, 0.0}, /*3*/
    {0.0, 0.0, 0.0, 0.0, Q_CONST_2, 0.0, 0.0}, /*4*/
    {0.0, 0.0, 0.0, 0.0, 0.0, Q_CONST_2, 0.0}, /*5*/
    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, Q_CONST_2}, /*6*/};
// 测量噪声协方差矩阵
// 假设相互独立
static float R_c[3][3] = { 
    {R_CONST, 0.0, 0.0}, /*0*/
    {0.0, R_CONST, 0.0}, /*1*/
    {0.0, 0.0, R_CONST}, /*2*/};
// 先验状态协方差矩阵
static float P_neg[7][7] = { 
    {1e5, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1}, /*0*/
    {0.1, 1e5, 0.1, 0.1, 0.1, 0.1, 0.1}, /*1*/
    {0.1, 0.1, 1e5, 0.1, 0.1, 0.1, 0.1}, /*2*/
    {0.1, 0.1, 0.1, 1e5, 0.1, 0.1, 0.1}, /*3*/
    {0.1, 0.1, 0.1, 0.1, 1e5, 0.1, 0.1}, /*4*/
    {0.1, 0.1, 0.1, 0.1, 0.1, 1e5, 0.1}, /*5*/
    {0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 1e5}, /*6*/};
// 后验状态协方差矩阵
static float P_pos[7][7] = { 
    {1e5, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1}, /*0*/
    {0.1, 1e5, 0.1, 0.1, 0.1, 0.1, 0.1}, /*1*/
    {0.1, 0.1, 1e5, 0.1, 0.1, 0.1, 0.1}, /*2*/
    {0.1, 0.1, 0.1, 1e5, 0.1, 0.1, 0.1}, /*3*/
    {0.1, 0.1, 0.1, 0.1, 1e5, 0.1, 0.1}, /*4*/
    {0.1, 0.1, 0.1, 0.1, 0.1, 1e5, 0.1}, /*5*/
    {0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 1e5}, /*6*/};

//////////////////////////// 优化参数定义 ////////////////////////////
// 渐消因子
static float lambda_fading[7][7] = { 
    {1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, /*0*/
    {0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0}, /*1*/
    {0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0}, /*2*/
    {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0}, /*3*/
    {0.0, 0.0, 0.0, 0.0, 1.0/FADING_FACTOR, 0.0, 0.0}, /*4*/
    {0.0, 0.0, 0.0, 0.0, 0.0, 1.0/FADING_FACTOR, 0.0}, /*5*/
    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0/FADING_FACTOR}, /*6*/};
// 接受因子，不使用加速度计更新四元数的 z 轴 
static float factor_rec[7][7] = { 
    {1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, /*0*/
    {0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0}, /*1*/
    {0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0}, /*2*/
    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, /*3*/
    {0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0}, /*4*/
    {0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0}, /*5*/
    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0}, /*6*/};
// 卡方检验协方差
static float cov_chi_square[3][3] = { 
{1.0, 0.0, 0.0}, /*0*/
{0.0, 1.0, 0.0}, /*1*/
{0.0, 0.0, 1.0}, /*2*/};


// 运行调用
// acce 为 scale 校正之后的
// gyro 为去零漂之前的
void imu_ekf_run(vector3_t acce, vector3_t gyro, float dt)
{
    raw_date.acce.x = acce.x;
    raw_date.acce.y = acce.y;
    raw_date.acce.z = acce.z;
    raw_date.gyro.x = gyro.x - state_pos.gyro_bias.x;
    raw_date.gyro.y = gyro.y - state_pos.gyro_bias.y;
    raw_date.gyro.z = gyro.z - state_pos.gyro_bias.z;
    raw_date.dt = dt;
    imu_ekf_update_prior_state();
}

// ekf 先验估计
void imu_ekf_update_prior_state()
{
    float q_0 = state_pos.q.w;
    float q_1 = state_pos.q.x;
    float q_2 = state_pos.q.y;
    float q_3 = state_pos.q.z;
    float w_x = raw_date.gyro.x;
    float w_y = raw_date.gyro.y;
    float w_z = raw_date.gyro.z;
    float dt  = raw_date.dt;

    state_neg.q.w = q_0 + 0.5 * dt * ( - q_1 * w_x - q_2 * w_y - q_3 * w_z );
    state_neg.q.x = q_1 + 0.5 * dt * (   q_0 * w_x + q_2 * w_z - q_3 * w_y );
    state_neg.q.y = q_2 + 0.5 * dt * (   q_0 * w_y - q_1 * w_z + q_3 * w_x );
    state_neg.q.z = q_3 + 0.5 * dt * (   q_0 * w_z + q_1 * w_y - q_2 * w_x );
    // bias_g 保持不变
}
// ekf 更新状态雅可比
void imu_ekf_update_state_jacobian()
{
    float q_0 = state_pos.q.w;
    float q_1 = state_pos.q.x;
    float q_2 = state_pos.q.y;
    float q_3 = state_pos.q.z;
    float w_x = raw_date.gyro.x;
    float w_y = raw_date.gyro.y;
    float w_z = raw_date.gyro.z;
    float dt  = raw_date.dt;

    F_jor[0][1] = 0.5 * dt * (-w_x);
    F_jor[0][2] = 0.5 * dt * (-w_y);
    F_jor[0][3] = 0.5 * dt * (-w_z); //  
    F_jor[1][0] = 0.5 * dt * ( w_x);
    F_jor[1][2] = 0.5 * dt * ( w_z);
    F_jor[1][3] = 0.5 * dt * (-w_y); // 
    F_jor[2][0] = 0.5 * dt * ( w_y);
    F_jor[2][1] = 0.5 * dt * (-w_z);
    F_jor[2][3] = 0.5 * dt * ( w_x); //
    F_jor[3][0] = 0.5 * dt * ( w_z);
    F_jor[3][1] = 0.5 * dt * ( w_y);
    F_jor[3][2] = 0.5 * dt * (-w_x); //

    F_jor[0][4] = 0.5 * dt * ( q_1);
    F_jor[0][5] = 0.5 * dt * ( q_2);
    F_jor[0][6] = 0.5 * dt * ( q_3); // 
    F_jor[1][4] = 0.5 * dt * (-q_0);
    F_jor[1][5] = 0.5 * dt * ( q_3);
    F_jor[1][6] = 0.5 * dt * (-q_2); // 
    F_jor[2][4] = 0.5 * dt * (-q_3);
    F_jor[2][5] = 0.5 * dt * (-q_0);
    F_jor[2][6] = 0.5 * dt * ( q_1); //
    F_jor[3][4] = 0.5 * dt * ( q_2);
    F_jor[3][5] = 0.5 * dt * (-q_1);
    F_jor[3][6] = 0.5 * dt * (-q_0); //

}
// ekf 更新先验协方差
void imu_ekf_update_prior_covariance()
{
    
}

// ekf 更新观测雅可比
void imu_ekf_update_observation_jacobian()
{

}

// ekf 更新卡尔曼增益 
void imu_ekf_update_kalman_gain()
{

}

// ekf 更新后验状态估计
void imu_ekf_update_posterior_state()
{

}

// ekf 卡方检验
void imu_ekf_update_posterior_covariance()
{

}