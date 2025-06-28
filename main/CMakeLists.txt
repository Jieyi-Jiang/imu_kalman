idf_component_register(
    SRCS 
        "main.c"
        "esp32_ssd1306.c"
        "imu_calibration.c"
        "kalman_filter.c"
    INCLUDE_DIRS 
        "."
)


# idf_component_register(SRCS "imu_ekf.c" "main.c"
#                     INCLUDE_DIRS ".")
