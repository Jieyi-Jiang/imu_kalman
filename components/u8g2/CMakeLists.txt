# idf_component_register(
#     SRCS "mpu6050.c"
#     INCLUDE_DIRS "include"
#     REQUIRES "driver"
# )
file(GLOB_RECURSE SOURCES "./src/*.c")
idf_component_register(
    SRCS 
        ${SOURCES}
    INCLUDE_DIRS 
        "./inc"
    REQUIRES "driver"
) 