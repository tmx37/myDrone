# Board bindings (callbacks + settings) of the drivers in Drivers/.
# Compiled into the parent target: they need main.h and the HAL headers.

target_include_directories(${LIB_PARENT_TARGET} PRIVATE
    Drv_BMP180_cfg/include
    Drv_HMC5883L_cfg/include
    Drv_HWSKYWALKER_ESC_cfg/include
    Drv_MPU60x0_cfg/include
)

target_sources(${LIB_PARENT_TARGET} PRIVATE
    Drv_BMP180_cfg/src/Drv_BMP180_Cfg.c
    Drv_HMC5883L_cfg/src/Drv_HMC5883L_Cfg.c
    Drv_HWSKYWALKER_ESC_cfg/src/Drv_HWSKYWALKER_ESC_Cfg.c
    Drv_MPU60x0_cfg/src/Drv_MPU60x0_Cfg.c
)
