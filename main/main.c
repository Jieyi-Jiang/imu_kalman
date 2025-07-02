#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
// #include "esp_mac.h"

#include "esp_system.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "driver/gptimer.h"
#include "unity.h"

#include "esp32_ssd1306.h"
#include "zf_device_imu660ra.h"
#include "imu_calibration.h"
#include "kalman_filter.h"

#define I2C_MASTER_SCL_IO 2           /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 1           /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUM_0      /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 400 * 1000 /*!< I2C master clock frequency */

static const char *TAG = "MPU6050";
// static mpu6050_handle_t mpu6050 = NULL;
static i2c_master_bus_handle_t bus_handle;
static uint8_t timer_cut = 0;
// static bool display_flag = false;
u8g2_t u8g2;
/**
 * @brief i2c master initialization
 */
bool IRAM_ATTR test_timer_callback(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data)
{
    // printf("count: %d\n", i);
    timer_cut += 1;
    return true;
}

static void i2c_bus_init(void)
{
    ESP_LOGI(TAG, "i2c initializing ... -----------------------------");
    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_MASTER_NUM,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
        // .flags.enable_internal_pullup = false,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));
    ESP_LOGI(TAG, "i2c initialized ----------------------------------\n");
}

typedef struct
{
    float x;
    float y;
    float z;
} axis_data_t;

typedef union
{
    float f_val;
    uint8_t byte_vals;
} f_byte_union_t;

typedef struct
{
    float acc_x;
    float acc_y;
    float acc_z;
    float gyr_x;
    float gyr_y;
    float gyr_z;
    float sample_interval;  // ms
} imu_raw_data_t;

float cal_velocity(float ax, float ay, float az, float stemp_time_ms)
{
    static float v_x = 0, v_y = 0, v_z = 0, v_scale = 0;
    v_x += ax * stemp_time_ms * 1e-3;
    v_y += ay * stemp_time_ms * 1e-3;
    v_z += az * stemp_time_ms * 1e-3;
    v_scale = sqrtf(pow(v_x, 2) + pow(v_y, 2) + pow(v_z, 2));
    return v_scale;
}

void vTaskFunction(void *pvParameters)
{
    // 任务主体代码
    imu_raw_data_t *imu_data = pvParameters;
    char str_buff[100];
    u8g2_ClearBuffer(&u8g2);
    u8g2_SetFontMode(&u8g2, 1);             /*字体模式选择*/
    u8g2_SetFontDirection(&u8g2, 0);        /*字体方向选择*/
    u8g2_SetFont(&u8g2, u8g2_font_6x12_tf); /*字库选择*/
    while (true)
    {
        u8g2_ClearBuffer(&u8g2);
        u8g2_DrawStr(&u8g2, 0, 12, "Acce:");
        sprintf(str_buff, "%7.2f", imu_data->acc_x);
        u8g2_DrawStr(&u8g2, 0, 24, str_buff);
        sprintf(str_buff, "%7.2f", imu_data->acc_y);
        u8g2_DrawStr(&u8g2, 42, 24, str_buff);
        sprintf(str_buff, "%7.2f", imu_data->acc_z);
        u8g2_DrawStr(&u8g2, 84, 24, str_buff);
        u8g2_DrawStr(&u8g2, 0, 36, "Gyro:");
        sprintf(str_buff, "%7.2f", imu_data->gyr_x);
        u8g2_DrawStr(&u8g2, 0, 48, str_buff);
        sprintf(str_buff, "%7.2f", imu_data->gyr_y);
        u8g2_DrawStr(&u8g2, 42, 48, str_buff);
        sprintf(str_buff, "%7.2f", imu_data->gyr_z);
        u8g2_DrawStr(&u8g2, 84, 48, str_buff);
        sprintf(str_buff, "cost t: %.4f ms", imu_data->sample_interval);
        u8g2_DrawStr(&u8g2, 0, 60, str_buff);
        u8g2_SendBuffer(&u8g2);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void imu_data_sample(void *pvParameters)
{
    int64_t start_time = 0.0, end_time = 0.0, elapsed_time = 0.0;
    imu_raw_data_t *imu_data = pvParameters;
    start_time = esp_timer_get_time(); // 获取时间（微秒）
    while (1)
    {
        end_time = esp_timer_get_time();
        elapsed_time = end_time - start_time;
        start_time = esp_timer_get_time(); // 获取时间（微秒）
        imu_data->sample_interval = elapsed_time * 1e-3;
        imu660ra_get_acc();
        imu660ra_get_gyro();
        float acce_x = imu660ra_acc_transition(imu660ra_acc_x);
        float acce_y = imu660ra_acc_transition(imu660ra_acc_y);
        float acce_z = imu660ra_acc_transition(imu660ra_acc_z);
        imu_data->acc_x = -(acce_A[0][0] * acce_x + acce_A[0][1] * acce_y + acce_A[0][2] * acce_z - acce_bias[0][0]);
        imu_data->acc_y = -(acce_A[1][0] * acce_x + acce_A[1][1] * acce_y + acce_A[1][2] * acce_z - acce_bias[1][0]);
        imu_data->acc_z = -(acce_A[2][0] * acce_x + acce_A[2][1] * acce_y + acce_A[2][2] * acce_z - acce_bias[2][0]);

        // imu_data->acc_x = imu660ra_get_acce_x();
        // imu_data->acc_y = imu660ra_get_acce_y();
        // imu_data->acc_z = imu660ra_get_acce_z();
        // printf("%f, %f, %f, ", imu_data->acc_x, imu_data->acc_y, imu_data->acc_z);
        imu_data->gyr_x = imu660ra_get_gyro_x();
        imu_data->gyr_y = imu660ra_get_gyro_y();
        imu_data->gyr_z = imu660ra_get_gyro_z();
        // printf("%f, %f, %f, ", imu_data->gyr_x, imu_data->gyr_y, imu_data->gyr_z);
        float dt_s = imu_data->sample_interval * 1e-3; // ms to s
        kalman_filter(imu_data->acc_x, imu_data->acc_y, imu_data->acc_z, imu_data->gyr_x, imu_data->gyr_y, imu_data->gyr_z, dt_s);
        eliminate_gravity_acceleration(imu_data->acc_x, imu_data->acc_y, imu_data->acc_z, theta_roll, theta_pitch);
        velocity_x += dt_s * acce_cal_x;
        velocity_y += dt_s * acce_cal_y;
        velocity_z += dt_s * acce_cal_z;
        // printf("%f, %f, %f\n", th_r_posterior, th_p_posterior, th_y_posterior);
        // printf("%f, %f, %f, %f, %f, %f \n", th_r_posterior, th_p_posterior, th_y_posterior, th_r_sen, th_p_sen, th_y_sen);
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "timer initializing ...");
    gptimer_handle_t gptimer = NULL;
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000, // 1MHz, 1 tick=1us
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer));
    gptimer_event_callbacks_t cbs = {
        .on_alarm = test_timer_callback,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &cbs, NULL));
    ESP_ERROR_CHECK(gptimer_enable(gptimer));
    gptimer_alarm_config_t alarm_config = {
        .reload_count = 0,
        .alarm_count = 1000000, // period = 1s
        .flags.auto_reload_on_alarm = true,
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_config));
    ESP_ERROR_CHECK(gptimer_start(gptimer));
    ESP_LOGI(TAG, "timer initialized");

    ESP_LOGI(TAG, "i2c driver initializing ...");
    i2c_bus_init();
    ESP_LOGI(TAG, "i2c driver initialized");

    ESP_LOGI(TAG, "imu660ra initializing ...");
    while (1)
    {
        if (imu660ra_init(I2C_MASTER_NUM))
            printf("\r\nIMU660RA init error."); // IMU660RA 初始化失败
        else
            break;
    }
    imu660ra_calibration();
    ESP_LOGI(TAG, "imu660ra initialized");

    u8g2Init(&u8g2, I2C_MASTER_NUM);
    // char str_buff[100];
    imu_raw_data_t imu_data;
    // int64_t start_time, end_time, elapsed_time;
    ESP_LOGI(TAG, "thread initializing ...");
    BaseType_t xReturned;
    // TaskHandle_t xHandle_1 = NULL;
    TaskHandle_t xHandle_2 = NULL;
    // 屏幕显示进程
    // xReturned = xTaskCreate(vTaskFunction, "Task display", 4096, &imu_data, 1,&xHandle_1);
    // if(xReturned == pdPASS) {
    //     printf("create task display\n");
    // }
    xReturned = xTaskCreate(imu_data_sample, "Task imu sampling", 3072, &imu_data, 2, &xHandle_2);
    if (xReturned == pdPASS)
    {
        printf("create task imu sampling\n");
    }
    ESP_LOGI(TAG, "thread initialized");
    ESP_LOGI(TAG, "main loop start");
    // float acce_A[3][3] = {
    //     {1.000050, 0.000257, 0.000210},
    //     {0.000257, 1.000965, 0.003950},
    //     {0.000210, 0.003950, 1.001645}};
    // float acce_bias[3][1] = {
    //     {0.012272},
    //     {-0.022023},
    //     {0.003605}};
    while (1)
    {
        // float acce_x = imu660ra_acc_transition(imu660ra_acc_x);
        // float acce_y = imu660ra_acc_transition(imu660ra_acc_y);
        // float acce_z = imu660ra_acc_transition(imu660ra_acc_z);
        // float acce_cal_x = acce_A[0][0] * acce_x + acce_A[0][1] * acce_y + acce_A[0][2] * acce_z - acce_bias[0][0];
        // float acce_cal_z = acce_A[2][0] * acce_x + acce_A[2][1] * acce_y + acce_A[2][2] * acce_z - acce_bias[2][0];
        // float acce_cal_y = acce_A[1][0] * acce_x + acce_A[1][1] * acce_y + acce_A[1][2] * acce_z - acce_bias[1][0];

        // float acce_cal_scale = sqrtf(pow(acce_cal_x, 2) + pow(acce_cal_y, 2) + pow(acce_cal_z, 2));
        // printf("%f, %f, %f, %f\n",
        //        acce_cal_x, acce_cal_y, acce_cal_z, acce_cal_scale);

        // float acce_raw_scale = sqrtf(pow(imu_data.acc_x, 2) + pow(imu_data.acc_y, 2) + pow(imu_data.acc_z, 2));
        // printf("%f, %f, %f, %f\n",
        //        imu_data.acc_x, imu_data.acc_y, imu_data.acc_z, acce_raw_scale);

        printf("%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n",
                theta_roll, theta_pitch,    theta_yaw, 
                th_r_sen,   th_p_sen,       th_y_sen,
                R_p,        P_r_prior,      P_r_posterior, 
                K_p,        K_r,            acce_scale_norm,
                imu_data.sample_interval);

        // printf("%f, %f, %f\n",
        //        imu_data->gyr_x,
        //        imu_data->gyr_y,
        //        imu_data->gyr_z);

        // printf("%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n",
        //        th_r_posterior, th_p_posterior, th_y_posterior,
        //        imu_data.acc_x, imu_data.acc_y, imu_data.acc_z,
        //        acce_cal_x, acce_cal_y, acce_cal_z,
        //         velocity_x, velocity_y, velocity_z);

        // printf("%f, %f, %f, %f, %f, %f \n",
        //        P_r_prior, P_p_prior, P_y_prior, 
        //        P_r_posterior, P_p_posterior, P_y_posterior);
        // velocity_x, velocity_y, velocity_z

        fflush(stdout);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
