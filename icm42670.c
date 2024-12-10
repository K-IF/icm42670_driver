/**********************************************************************************************************************
*                                       Kostas Ifantidis
***********************************************************************************************************************
*
***********************************************************************************************************************
* File Name          : icm42670_drv.c
* Date First Issued  : 01/09/2024
* Author(s)          : Kostas Ifantidis
* Description        : icm42670 sensor driver source file
* 
***********************************************************************************************************************/
/*
 * ELECTRICAL SPECS
 * VDD, VDDIO:                  1.7v-3.6v (1.8v typical)
 * Full-chip sleep mode:        3.5 μA
 * Low-Noise mode: (Gyro+Accel) 0.55 mA
 * Operating temperature:       (-40 to +85) ºC
 *
 * Supply Ramp Time:            3 ms
 * i2C max speed:               1 MHz
 *
 * FEATURES
 * 16bit ADC
 * Temperature sensor (internal)
 * ACCEL sensitivity: ±2g, ±4g, ±8g, ±16g
 * GYRO sensitivity ±250º/s, ±500º/s, ±1000º/s, ±2000º/s
 *
 *
 */


//*********************************************************************************************************************
// Function Name : None
// Description   : None
// Parameters    : None
// Return        : None
// Notes         : None
//*********************************************************************************************************************


//    INCLUDES
//*********************************************************************************************************************
#include <stdint.h>
#include <string.h>
#include "icm42670.h"

//    DEFINES
//*********************************************************************************************************************
#define I2C_FREQ_HZ                                 1000000 // 1MHz

// register structure definitions
#define ICM42670_MCLK_RDY_BITS                      0x08 // ICM42670_REG_MCLK_RDY<3>
#define ICM42670_MCLK_RDY_SHIFT                     3    // ICM42670_REG_MCLK_RDY<3>

#define ICM42670_SPI_AP_4WIRE_BITS                  0x04 // ICM42670_REG_DEVICE_CONFIG<2>
#define ICM42670_SPI_AP_4WIRE_SHIFT                 2    // ICM42670_REG_DEVICE_CONFIG<2>
#define ICM42670_SPI_MODE_BITS                      0x01 // ICM42670_REG_DEVICE_CONFIG<0>
#define ICM42670_SPI_MODE_SHIFT                     0    // ICM42670_REG_DEVICE_CONFIG<0>

#define ICM42670_SOFT_RESET_DEVICE_CONFIG_BITS      0x10 // ICM42670_REG_SIGNAL_PATH_RESET<4>
#define ICM42670_SOFT_RESET_DEVICE_CONFIG_SHIFT     4    // ICM42670_REG_SIGNAL_PATH_RESET<4>
#define ICM42670_FIFO_FLUSH_BITS                    0x04 // ICM42670_REG_SIGNAL_PATH_RESET<2>
#define ICM42670_FIFO_FLUSH_SHIFT                   2    // ICM42670_REG_SIGNAL_PATH_RESET<2>

#define ICM42670_I3C_DDR_SLEW_RATE_BITS             0x38 // ICM42670_REG_DRIVE_CONFIG1<5:3>
#define ICM42670_I3C_DDR_SLEW_RATE_SHIFT            3    // ICM42670_REG_DRIVE_CONFIG1<5:3>
#define ICM42670_I3C_SDR_SLEW_RATE_BITS             0x07 // ICM42670_REG_DRIVE_CONFIG1<2:0>
#define ICM42670_I3C_SDR_SLEW_RATE_SHIFT            0    // ICM42670_REG_DRIVE_CONFIG1<2:0>

#define ICM42670_I2C_DDR_SLEW_RATE_BITS             0x38 // ICM42670_REG_DRIVE_CONFIG2<5:3>
#define ICM42670_I2C_DDR_SLEW_RATE_SHIFT            3    // ICM42670_REG_DRIVE_CONFIG2<5:3>
#define ICM42670_I2C_SDR_SLEW_RATE_BITS             0x07 // ICM42670_REG_DRIVE_CONFIG2<2:0>
#define ICM42670_I2C_SDR_SLEW_RATE_SHIFT            0    // ICM42670_REG_DRIVE_CONFIG2<2:0>

#define ICM42670_SPI_SLEW_RATE_BITS                 0x07 // ICM42670_REG_DRIVE_CONFIG3<2:0>
#define ICM42670_SPI_SLEW_RATE_SHIFT                0    // ICM42670_REG_DRIVE_CONFIG3<2:0>

#define ICM42670_INT2_MODE_BITS                     0x20 // ICM42670_REG_INT_CONFIG<5>
#define ICM42670_INT2_MODE_SHIFT                    5    // ICM42670_REG_INT_CONFIG<5>
#define ICM42670_INT2_DRIVE_CIRCUIT_BITS            0x10 // ICM42670_REG_INT_CONFIG<4>
#define ICM42670_INT2_DRIVE_CIRCUIT_SHIFT           4    // ICM42670_REG_INT_CONFIG<4>
#define ICM42670_INT2_POLARITY_BITS                 0x08 // ICM42670_REG_INT_CONFIG<3>
#define ICM42670_INT2_POLARITY_SHIFT                3    // ICM42670_REG_INT_CONFIG<3>
#define ICM42670_INT1_MODE_BITS                     0x04 // ICM42670_REG_INT_CONFIG<2>
#define ICM42670_INT1_MODE_SHIFT                    2    // ICM42670_REG_INT_CONFIG<2>
#define ICM42670_INT1_DRIVE_CIRCUIT_BITS            0x02 // ICM42670_REG_INT_CONFIG<1>
#define ICM42670_INT1_DRIVE_CIRCUIT_SHIFT           1    // ICM42670_REG_INT_CONFIG<1>
#define ICM42670_INT1_POLARITY_BITS                 0x01 // ICM42670_REG_INT_CONFIG<0>
#define ICM42670_INT1_POLARITY_SHIFT                0    // ICM42670_REG_INT_CONFIG<0>
#define ICM42670_INT2_CFG_BITS_MASK                 0x38
#define ICM42670_INT1_CFG_BITS_MASK                 0x07

#define ICM42670_ACCEL_LP_CLK_SEL_BITS              0x80 // ICM42670_REG_PWR_MGMT0<7>
#define ICM42670_ACCEL_LP_CLK_SEL_SHIFT             7    // ICM42670_REG_PWR_MGMT0<7>
#define ICM42670_IDLE_BITS                          0x10 // ICM42670_REG_PWR_MGMT0<4>
#define ICM42670_IDLE_SHIFT                         4    // ICM42670_REG_PWR_MGMT0<4>
#define ICM42670_GYRO_MODE_BITS                     0x0C // ICM42670_REG_PWR_MGMT0<3:2>
#define ICM42670_GYRO_MODE_SHIFT                    2    // ICM42670_REG_PWR_MGMT0<3:2>
#define ICM42670_ACCEL_MODE_BITS                    0x03 // ICM42670_REG_PWR_MGMT0<1:0>
#define ICM42670_ACCEL_MODE_SHIFT                   0    // ICM42670_REG_PWR_MGMT0<1:0>

#define ICM42670_GYRO_UI_FS_SEL_BITS                0x60 // ICM42670_REG_GYRO_CONFIG0<6:5>
#define ICM42670_GYRO_UI_FS_SEL_SHIFT               5    // ICM42670_REG_GYRO_CONFIG0<6:5>
#define ICM42670_GYRO_ODR_BITS                      0x0F // ICM42670_REG_GYRO_CONFIG0<3:0>
#define ICM42670_GYRO_ODR_SHIFT                     0    // ICM42670_REG_GYRO_CONFIG0<3:0>

#define ICM42670_ACCEL_UI_FS_SEL_BITS               0x60 // ICM42670_REG_ACCEL_CONFIG0<6:5>
#define ICM42670_ACCEL_UI_FS_SEL_SHIFT              5    // ICM42670_REG_ACCEL_CONFIG0<6:5>
#define ICM42670_ACCEL_ODR_BITS                     0x0F // ICM42670_REG_ACCEL_CONFIG0<3:0>
#define ICM42670_ACCEL_ODR_SHIFT                    0    // ICM42670_REG_ACCEL_CONFIG0<3:0>

#define ICM42670_TEMP_FILT_BW_BITS                  0x70 // ICM42670_REG_TEMP_CONFIG0<6:4>
#define ICM42670_TEMP_FILT_BW_SHIFT                 4    // ICM42670_REG_TEMP_CONFIG0<6:4>

#define ICM42670_GYRO_UI_FILT_BW_BITS               0x07 // ICM42670_REG_GYRO_CONFIG1<2:0>
#define ICM42670_GYRO_UI_FILT_BW_SHIFT              0    // ICM42670_REG_GYRO_CONFIG1<2:0>

#define ICM42670_ACCEL_UI_AVG_BITS                  0x70 // ICM42670_REG_ACCEL_CONFIG1<6:4>
#define ICM42670_ACCEL_UI_AVG_SHIFT                 4    // ICM42670_REG_ACCEL_CONFIG1<6:4>
#define ICM42670_ACCEL_UI_FILT_BW_BITS              0x07 // ICM42670_REG_ACCEL_CONFIG1<2:0>
#define ICM42670_ACCEL_UI_FILT_BW_SHIFT             0    // ICM42670_REG_ACCEL_CONFIG1<2:0>

#define ICM42670_DMP_POWER_SAVE_EN_BITS             0x08 // ICM42670_REG_APEX_CONFIG0<3>
#define ICM42670_DMP_POWER_SAVE_EN_SHIFT            3    // ICM42670_REG_APEX_CONFIG0<3>
#define ICM42670_DMP_INIT_EN_BITS                   0x04 // ICM42670_REG_APEX_CONFIG0<2>
#define ICM42670_DMP_INIT_EN_SHIFT                  2    // ICM42670_REG_APEX_CONFIG0<2>
#define ICM42670_DMP_MEM_RESET_EN_BITS              0x01 // ICM42670_REG_APEX_CONFIG0<0>
#define ICM42670_DMP_MEM_RESET_EN_SHIFT             0    // ICM42670_REG_APEX_CONFIG0<0>

#define ICM42670_SMD_ENABLE_BITS                    0x40 // ICM42670_REG_APEX_CONFIG1<6>
#define ICM42670_SMD_ENABLE_SHIFT                   6    // ICM42670_REG_APEX_CONFIG1<6>
#define ICM42670_FF_ENABLE_BITS                     0x20 // ICM42670_REG_APEX_CONFIG1<5>
#define ICM42670_FF_ENABLE_SHIFT                    5    // ICM42670_REG_APEX_CONFIG1<5>
#define ICM42670_TILT_ENABLE_BITS                   0x10 // ICM42670_REG_APEX_CONFIG1<4>
#define ICM42670_TILT_ENABLE_SHIFT                  4    // ICM42670_REG_APEX_CONFIG1<4>
#define ICM42670_PED_ENABLE_BITS                    0x08 // ICM42670_REG_APEX_CONFIG1<3>
#define ICM42670_PED_ENABLE_SHIFT                   3    // ICM42670_REG_APEX_CONFIG1<3>
#define ICM42670_DMP_ODR_BITS                       0x03 // ICM42670_REG_APEX_CONFIG1<1:0>
#define ICM42670_DMP_ODR_SHIFT                      0    // ICM42670_REG_APEX_CONFIG1<1:0>

#define ICM42670_WOM_INT_DUR_BITS                   0x18 // ICM42670_REG_WOM_CONFIG<4:3>
#define ICM42670_WOM_INT_DUR_SHIFT                  3    // ICM42670_REG_WOM_CONFIG<4:3>
#define ICM42670_WOM_INT_MODE_BITS                  0x04 // ICM42670_REG_WOM_CONFIG<2>
#define ICM42670_WOM_INT_MODE_SHIFT                 2    // ICM42670_REG_WOM_CONFIG<2>
#define ICM42670_WOM_MODE_BITS                      0x02 // ICM42670_REG_WOM_CONFIG<1>
#define ICM42670_WOM_MODE_SHIFT                     1    // ICM42670_REG_WOM_CONFIG<1>
#define ICM42670_WOM_EN_BITS                        0x01 // ICM42670_REG_WOM_CONFIG<0>
#define ICM42670_WOM_EN_SHIFT                       0    // ICM42670_REG_WOM_CONFIG<0>

#define ICM42670_FIFO_MODE_BITS                     0x02 // ICM42670_REG_FIFO_CONFIG1<1>
#define ICM42670_FIFO_MODE_SHIFT                    1    // ICM42670_REG_FIFO_CONFIG1<1>
#define ICM42670_FIFO_BYPASS_BITS                   0x01 // ICM42670_REG_FIFO_CONFIG1<0>
#define ICM42670_FIFO_BYPASS_SHIFT                  0    // ICM42670_REG_FIFO_CONFIG1<0>

#define ICM42670_ST_INT1_EN_BITS                    0x80 // ICM42670_REG_INT_SOURCE0<7>
#define ICM42670_ST_INT1_EN_SHIFT                   7    // ICM42670_REG_INT_SOURCE0<7>
#define ICM42670_FSYNC_INT1_EN_BITS                 0x40 // ICM42670_REG_INT_SOURCE0<6>
#define ICM42670_FSYNC_INT1_EN_SHIFT                6    // ICM42670_REG_INT_SOURCE0<6>
#define ICM42670_PLL_RDY_INT1_EN_BITS               0x20 // ICM42670_REG_INT_SOURCE0<5>
#define ICM42670_PLL_RDY_INT1_EN_SHIFT              5    // ICM42670_REG_INT_SOURCE0<5>
#define ICM42670_RESET_DONE_INT1_EN_BITS            0x10 // ICM42670_REG_INT_SOURCE0<4>
#define ICM42670_RESET_DONE_INT1_EN_SHIFT           4    // ICM42670_REG_INT_SOURCE0<4>
#define ICM42670_DRDY_INT1_EN_BITS                  0x08 // ICM42670_REG_INT_SOURCE0<3>
#define ICM42670_DRDY_INT1_EN_SHIFT                 3    // ICM42670_REG_INT_SOURCE0<3>
#define ICM42670_FIFO_THS_INT1_EN_BITS              0x04 // ICM42670_REG_INT_SOURCE0<2>
#define ICM42670_FIFO_THS_INT1_EN_SHIFT             2    // ICM42670_REG_INT_SOURCE0<2>
#define ICM42670_FIFO_FULL_INT1_EN_BITS             0x02 // ICM42670_REG_INT_SOURCE0<1>
#define ICM42670_FIFO_FULL_INT1_EN_SHIFT            1    // ICM42670_REG_INT_SOURCE0<1>
#define ICM42670_AGC_RDY_INT1_EN_BITS               0x01 // ICM42670_REG_INT_SOURCE0<0>
#define ICM42670_AGC_RDY_INT1_EN_SHIFT              0    // ICM42670_REG_INT_SOURCE0<0>

#define ICM42670_I3C_PROTOCOL_ERROR_INT1_EN_BITS    0x40 // ICM42670_REG_INT_SOURCE1<6>
#define ICM42670_I3C_PROTOCOL_ERROR_INT1_EN_SHIFT   6    // ICM42670_REG_INT_SOURCE1<6>
#define ICM42670_SMD_INT1_EN_BITS                   0x08 // ICM42670_REG_INT_SOURCE1<3>
#define ICM42670_SMD_INT1_EN_SHIFT                  3    // ICM42670_REG_INT_SOURCE1<3>
#define ICM42670_WOM_Z_INT1_EN_BITS                 0x04 // ICM42670_REG_INT_SOURCE1<2>
#define ICM42670_WOM_Z_INT1_EN_SHIFT                2    // ICM42670_REG_INT_SOURCE1<2>
#define ICM42670_WOM_Y_INT1_EN_BITS                 0x02 // ICM42670_REG_INT_SOURCE1<1>
#define ICM42670_WOM_Y_INT1_EN_SHIFT                1    // ICM42670_REG_INT_SOURCE1<1>
#define ICM42670_WOM_X_INT1_EN_BITS                 0x01 // ICM42670_REG_INT_SOURCE1<0>
#define ICM42670_WOM_X_INT1_EN_SHIFT                0    // ICM42670_REG_INT_SOURCE1<0>

// ICM42670_REG_INT_SOURCE3 and ICM42670_REG_INT_SOURCE4 same as 0 and 1

#define ICM42670_DMP_IDLE_BITS                      0x04 // ICM42670_REG_APEX_DATA3<2>
#define ICM42670_DMP_IDLE_SHIFT                     2    // ICM42670_REG_APEX_DATA3<2>
#define ICM42670_ACTIVITY_CLASS_BITS                0x03 // ICM42670_REG_APEX_DATA3<1:0>
#define ICM42670_ACTIVITY_CLASS_SHIFT               0    // ICM42670_REG_APEX_DATA3<1:0>

#define ICM42670_FIFO_COUNT_FORMAT_BITS             0x40 // ICM42670_REG_INTF_CONFIG0<6>
#define ICM42670_FIFO_COUNT_FORMAT_SHIFT            6    // ICM42670_REG_INTF_CONFIG0<6>
#define ICM42670_FIFO_COUNT_ENDIAN_BITS             0x20 // ICM42670_REG_INTF_CONFIG0<5>
#define ICM42670_FIFO_COUNT_ENDIAN_SHIFT            5    // ICM42670_REG_INTF_CONFIG0<5>
#define ICM42670_SENSOR_DATA_ENDIAN_BITS            0x10 // ICM42670_REG_INTF_CONFIG0<4>
#define ICM42670_SENSOR_DATA_ENDIAN_SHIFT           4    // ICM42670_REG_INTF_CONFIG0<4>

#define ICM42670_I3C_SDR_EN_BITS                    0x08 // ICM42670_REG_INTF_CONFIG1<3>
#define ICM42670_I3C_SDR_EN_SHIFT                   3    // ICM42670_REG_INTF_CONFIG1<3>
#define ICM42670_I3C_DDR_EN_BITS                    0x04 // ICM42670_REG_INTF_CONFIG1<2>
#define ICM42670_I3C_DDR_EN_SHIFT                   2    // ICM42670_REG_INTF_CONFIG1<2>
#define ICM42670_CLKSEL_BITS                        0x03 // ICM42670_REG_INTF_CONFIG1<1:0>
#define ICM42670_CLKSEL_SHIFT                       0    // ICM42670_REG_INTF_CONFIG1<1:0>

#define ICM42670_DATA_RDY_INT_BITS                  0x01 // ICM42670_REG_INT_STATUS_DRDY<0>
#define ICM42670_DATA_RDY_INT_SHIFT                 0    // ICM42670_REG_INT_STATUS_DRDY<0>

#define ICM42670_ST_INT_BITS                        0x80 // ICM42670_REG_INT_STATUS<7>
#define ICM42670_ST_INT_SHIFT                       7    // ICM42670_REG_INT_STATUS<7>
#define ICM42670_FSYNC_INT_BITS                     0x40 // ICM42670_REG_INT_STATUS<6>
#define ICM42670_FSYNC_INT_SHIFT                    6    // ICM42670_REG_INT_STATUS<6>
#define ICM42670_PLL_RDY_INT_BITS                   0x20 // ICM42670_REG_INT_STATUS<5>
#define ICM42670_PLL_RDY_INT_SHIFT                  5    // ICM42670_REG_INT_STATUS<5>
#define ICM42670_RESET_DONE_INT_BITS                0x10 // ICM42670_REG_INT_STATUS<4>
#define ICM42670_RESET_DONE_INT_SHIFT               4    // ICM42670_REG_INT_STATUS<4>
#define ICM42670_FIFO_THS_INT_BITS                  0x04 // ICM42670_REG_INT_STATUS<2>
#define ICM42670_FIFO_THS_INT_SHIFT                 2    // ICM42670_REG_INT_STATUS<2>
#define ICM42670_FIFO_FULL_INT_BITS                 0x02 // ICM42670_REG_INT_STATUS<1>
#define ICM42670_FIFO_FULL_INT_SHIFT                1    // ICM42670_REG_INT_STATUS<1>
#define ICM42670_AGC_RDY_INT_BITS                   0x01 // ICM42670_REG_INT_STATUS<0>
#define ICM42670_AGC_RDY_INT_SHIFT                  0    // ICM42670_REG_INT_STATUS<0>

#define ICM42670_SMD_INT_BITS                       0x08 // ICM42670_REG_INT_STATUS2<3>
#define ICM42670_SMD_INT_SHIFT                      3    // ICM42670_REG_INT_STATUS2<3>
#define ICM42670_WOM_X_INT_BITS                     0x04 // ICM42670_REG_INT_STATUS2<2>
#define ICM42670_WOM_X_INT_SHIFT                    2    // ICM42670_REG_INT_STATUS2<2>
#define ICM42670_WOM_Y_INT_BITS                     0x02 // ICM42670_REG_INT_STATUS2<1>
#define ICM42670_WOM_Y_INT_SHIFT                    1    // ICM42670_REG_INT_STATUS2<1>
#define ICM42670_WOM_Z_INT_BITS                     0x01 // ICM42670_REG_INT_STATUS2<0>
#define ICM42670_WOM_Z_INT_SHIFT                    0    // ICM42670_REG_INT_STATUS2<0>

#define ICM42670_STEP_DET_INT_BITS                  0x20 // ICM42670_REG_INT_STATUS3<5>
#define ICM42670_STEP_DET_INT_SHIFT                 5    // ICM42670_REG_INT_STATUS3<5>
#define ICM42670_STEP_CNT_OVF_INT_BITS              0x10 // ICM42670_REG_INT_STATUS3<4>
#define ICM42670_STEP_CNT_OVF_INT_SHIFT             4    // ICM42670_REG_INT_STATUS3<4>
#define ICM42670_TILT_DET_INT_BITS                  0x08 // ICM42670_REG_INT_STATUS3<3>
#define ICM42670_TILT_DET_INT_SHIFT                 3    // ICM42670_REG_INT_STATUS3<3>
#define ICM42670_FF_DET_INT_BITS                    0x04 // ICM42670_REG_INT_STATUS3<2>
#define ICM42670_FF_DET_INT_SHIFT                   2    // ICM42670_REG_INT_STATUS3<2>
#define ICM42670_LOWG_DET_INT_BITS                  0x02 // ICM42670_REG_INT_STATUS3<1>
#define ICM42670_LOWG_DET_INT_SHIFT                 1    // ICM42670_REG_INT_STATUS3<1>

//    MACROS
//*********************************************************************************************************************
#if (defined(ICM42670_MUTEX_IS_ENABLED) && (ICM42670_MUTEX_IS_ENABLED == 1))
    #define ICM42670_TAKE_MUTEX(dev) icm42670_take_mutex(dev)
    #define ICM42670_GIVE_MUTEX(dev) icm42670_give_mutex(dev)
#else
    #define ICM42670_TAKE_MUTEX(dev) ((void)0)
    #define ICM42670_GIVE_MUTEX(dev) ((void)0)
#endif

#define ICM42670_CHECK_RET_VAL(x)                                                                                      \
    do                                                                                                                 \
    {                                                                                                                  \
        icm42670_err_t ret_var;                                                                                             \
        if ((ret_var = x) != ICM42670_NO_ERROR)                                                                             \
            return ret_var;                                                                                                 \
    }                                                                                                                  \
    while (0)

#define ICM42670_CHECK_ARG_VAL(val)                                                                                    \
    do                                                                                                                 \
    {                                                                                                                  \
        if (!(val))                                                                                                    \
            return ICM42670_INVALID_ARG_ERROR;                                                                         \
    }                                                                                                                  \
    while (0)

#define ICM42670_CHECK_RET_VAL_MUTEX(dev, x)                                                                           \
    do                                                                                                                 \
    {                                                                                                                  \
        icm42670_err_t ret_var;                                                                                             \
        if ((ret_var = x) != ICM42670_NO_ERROR)                                                                             \
        {                                                                                                              \
            ICM42670_GIVE_MUTEX(dev);                                                                                  \
            return ret_var;                                                                                                 \
        }                                                                                                              \
    }                                                                                                                  \
    while (0)



//    EXTERN FUNCTIONS
//*********************************************************************************************************************

//    FUNCTIONS PROTOTYPES
//*********************************************************************************************************************
static inline icm42670_err_t icm42670_write_reg(icm42670_t* dev, uint8_t reg, uint8_t* data);
static inline icm42670_err_t icm42670_read_reg(icm42670_t* dev, uint8_t reg, uint8_t* data);
static inline icm42670_err_t icm42670_read_register_16b(icm42670_t* dev, uint8_t upper_byte_reg, int16_t* data);
static inline icm42670_err_t icm42670_manipulate_register(icm42670_t* dev, uint8_t reg, uint8_t mask, uint8_t shift, uint8_t value);
static inline icm42670_err_t icm42670_write_mreg_register(icm42670_t* dev, icm42670_mreg_number_t mreg_num, uint8_t reg, uint8_t *data);
static inline icm42670_err_t icm42670_read_mreg_register(icm42670_t* dev, icm42670_mreg_number_t mreg_num, uint8_t reg, uint8_t *data);
static inline icm42670_err_t icm42670_manipulate_mreg_register(icm42670_t* dev, icm42670_mreg_number_t mreg_num, uint8_t reg, uint8_t mask, uint8_t shift, uint8_t value);
#if (defined(ICM42670_MUTEX_IS_ENABLED) && (ICM42670_MUTEX_IS_ENABLED == 1))
static inline icm42670_err_t icm42670_take_mutex(icm42670_t* dev);
static inline icm42670_err_t icm42670_give_mutex(icm42670_t* dev);
#endif

icm42670_err_t icm42670_init_object(icm42670_t* dev, uint8_t i2c_addr, uint32_t i2c_timeout, icm42670_i2c_write_t write_reg, icm42670_i2c_read_t read_reg, icm42670_gpio_irq_cb_t gpio_irq_cb, icm42670_msdelay_t ms_delay, icm42670_usdelay_t us_delay);
icm42670_err_t icm42670_deinit_object(icm42670_t* dev);
icm42670_err_t icm42670_init(icm42670_t* dev);
icm42670_err_t icm42670_reset(icm42670_t* dev);
icm42670_err_t icm42670_flush_fifo(icm42670_t* dev);
icm42670_err_t icm42670_set_low_power_clock(icm42670_t *dev, icm42670_lp_clock_source_t clock_source);
icm42670_err_t icm42670_read_raw_data(icm42670_t *dev, uint8_t reg, int16_t *data);
icm42670_err_t icm42670_read_temperature(icm42670_t *dev, float *temperature);
icm42670_err_t icm42670_read_devid(icm42670_t* dev, uint8_t* data);
icm42670_err_t icm42670_set_idle_pwr_mode(icm42670_t* dev, uint8_t enable_idle);
icm42670_err_t icm42670_set_gyro_pwr_mode(icm42670_t* dev, icm42670_gyro_pwr_mode_t pwr_mode);
icm42670_err_t icm42670_set_accel_pwr_mode(icm42670_t* dev, icm42670_accel_pwr_mode_t pwr_mode);
icm42670_err_t icm42670_set_gyro_fsr(icm42670_t *dev, icm42670_gyro_fsr_t range);
icm42670_err_t icm42670_set_gyro_odr(icm42670_t *dev, icm42670_gyro_odr_t odr);
icm42670_err_t icm42670_set_accel_fsr(icm42670_t *dev, icm42670_accel_fsr_t range);
icm42670_err_t icm42670_set_accel_odr(icm42670_t *dev, icm42670_accel_odr_t odr);
icm42670_err_t icm42670_set_temp_lpf(icm42670_t *dev, icm42670_temp_lfp_t lpf_bw);
icm42670_err_t icm42670_set_gyro_lpf(icm42670_t *dev, icm42670_gyro_lfp_t lpf_bw);
icm42670_err_t icm42670_set_accel_lpf(icm42670_t *dev, icm42670_accel_lfp_t lpf_bw);
icm42670_err_t icm42670_set_accel_avg(icm42670_t *dev, icm42670_accel_avg_t avg);
icm42670_err_t icm42670_config_int_pin(icm42670_t *dev, uint8_t int_pin, icm42670_int_config_t config);
icm42670_err_t icm42670_set_int_sources(icm42670_t *dev, uint8_t int_pin, icm42670_int_source_t sources);
icm42670_err_t icm42670_config_wom(icm42670_t *dev, icm42670_wom_config_t config);
icm42670_err_t icm42670_enable_wom(icm42670_t *dev, uint8_t enable);
icm42670_err_t icm42670_get_mclk_rdy(icm42670_t* dev, uint8_t* mclk_rdy);
icm42670_err_t icm42670_get_accel_odr(icm42670_t* dev, icm42670_accel_odr_t* odr);
icm42670_err_t icm42670_get_accel_avg(icm42670_t* dev, icm42670_accel_avg_t* avg);

//    ENUMERATORS
//*********************************************************************************************************************

//    STRUCTURES
//*********************************************************************************************************************

//    EXTERN VARIABLES
//*********************************************************************************************************************

//    VARIABLES
//*********************************************************************************************************************

//    FUNCTIONS DEFINITION
//*********************************************************************************************************************
/**
  * @brief  Write generic device register
  *
  * @param  dev         read / write interface definitions(ptr)
  * @param  reg         register to read
  * @param  data        pointer to buffer that store the data read(ptr)
  * @retval             interface status (MANDATORY: return 0 -> ICM42670_NO_ERROR)
  *
  */
static inline icm42670_err_t icm42670_write_reg(icm42670_t* dev, uint8_t reg, uint8_t* data)
{
    return dev->write_reg(dev, reg, data, 1);

}

/**
  * @brief  Read generic device register
  *
  * @param  dev         read / write interface definitions(ptr)
  * @param  reg         register to read
  * @param  data        pointer to buffer that store the data read(ptr)
  * @retval             interface status (MANDATORY: return 0 -> ICM42670_NO_ERROR)
  *
  */
static inline icm42670_err_t icm42670_read_reg(icm42670_t* dev, uint8_t reg, uint8_t* data)
{
    return dev->read_reg(dev, reg, data, 1);

}

/**
  * @brief  Read generic device 16b register
  *
  * @param  dev             read / write interface definitions(ptr)
  * @param  upper_byte_reg  register to read
  * @param  data            pointer to buffer that store the data read(ptr)
  * @retval                 interface status (MANDATORY: return 0 -> ICM42670_NO_ERROR)
  *
  */
static inline icm42670_err_t icm42670_read_register_16b(icm42670_t* dev, uint8_t upper_byte_reg, int16_t* data)
{
icm42670_err_t ret_error= ICM42670_NO_ERROR;
uint8_t high_byte_data, low_byte_data;

    ICM42670_CHECK_ARG_VAL(dev && data);

    ret_error= icm42670_read_reg(dev,  upper_byte_reg, &high_byte_data);
    ret_error= icm42670_read_reg(dev, (upper_byte_reg + 1), &low_byte_data);
    *data= (int16_t)((high_byte_data << 8) | low_byte_data);

    return ret_error;

}

/**
  * @brief  Manipulate register value
  *
  * @param  dev     read / write interface definitions(ptr)
  * @param  reg     register to manipulate
  * @param  mask    register mask value
  * @param  shift   number of bits to left shift
  * @param  value   register new value
  * @retval         interface status (MANDATORY: return 0 -> ICM42670_NO_ERROR)
  *
  */
static inline icm42670_err_t icm42670_manipulate_register(icm42670_t* dev, uint8_t reg, uint8_t mask, uint8_t shift, uint8_t value)
{
uint8_t data;

    ICM42670_CHECK_ARG_VAL(dev);

    ICM42670_TAKE_MUTEX(dev);
    ICM42670_CHECK_RET_VAL_MUTEX(dev, icm42670_read_reg(dev, reg, &data));
    data = (data & ~mask) | (value << shift);
    ICM42670_CHECK_RET_VAL_MUTEX(dev, icm42670_write_reg(dev, reg, &data));
    ICM42670_GIVE_MUTEX(dev);

    return ICM42670_NO_ERROR;

}

/**
  * @brief  Write generic MREG bank register
  *
  * @param  dev         read / write interface definitions(ptr)
  * @param  mreg_num    mregister to read
  * @param  reg         register to read
  * @param  data        pointer to buffer that store the data read(ptr)
  * @retval             interface status (MANDATORY: return 0 -> ICM42670_NO_ERROR)
  *
  */
static inline icm42670_err_t icm42670_write_mreg_register(icm42670_t* dev, icm42670_mreg_number_t mreg_num, uint8_t reg, uint8_t *data)
{
uint8_t mclk_rdy;

    ICM42670_CHECK_ARG_VAL(dev && data);

    ICM42670_CHECK_RET_VAL(icm42670_get_mclk_rdy(dev, &mclk_rdy));
    if (!mclk_rdy)
    {
        //Error log?!
        return ICM42670_INVALID_RESPONSE_ERROR;
    }

    ICM42670_TAKE_MUTEX(dev);
    ICM42670_CHECK_RET_VAL_MUTEX(dev, icm42670_write_reg(dev, ICM42670_REG_BLK_SEL_W, &mreg_num));
    ICM42670_CHECK_RET_VAL_MUTEX(dev, icm42670_write_reg(dev, ICM42670_REG_MADDR_W, &reg));
    ICM42670_CHECK_RET_VAL_MUTEX(dev, icm42670_write_reg(dev, ICM42670_REG_MADDR_W, data));
    dev->us_delay(10); // Wait for 10us until MREG write is complete
    ICM42670_GIVE_MUTEX(dev);

    return ICM42670_NO_ERROR;

}

/**
  * @brief  Read generic MREG bank register
  *
  * @param  dev         read / write interface definitions(ptr)
  * @param  mreg_num    mregister to read
  * @param  reg         register to read
  * @param  data        pointer to buffer that store the data read(ptr)
  * @retval             interface status (MANDATORY: return 0 -> ICM42670_NO_ERROR)
  *
  */
static inline icm42670_err_t icm42670_read_mreg_register(icm42670_t* dev, icm42670_mreg_number_t mreg_num, uint8_t reg, uint8_t *data)
{
uint8_t mclk_rdy;
icm42670_err_t ret_error= ICM42670_NO_ERROR;

    ICM42670_CHECK_ARG_VAL(dev && data);

    icm42670_get_mclk_rdy(dev, &mclk_rdy);
    if (!mclk_rdy)
    {
        //Error log?!
        return ICM42670_INVALID_RESPONSE_ERROR;
    }

    ICM42670_TAKE_MUTEX(dev);
    ret_error= icm42670_write_reg(dev, ICM42670_REG_BLK_SEL_R, &mreg_num);
    ICM42670_CHECK_RET_VAL_MUTEX(dev, ret_error);
    ret_error= icm42670_write_reg(dev, ICM42670_REG_MADDR_R, &reg);
    ICM42670_CHECK_RET_VAL_MUTEX(dev, ret_error);
    dev->us_delay(10); // Wait for 10us until MREG write is complete
    ret_error= icm42670_read_reg(dev, ICM42670_REG_M_R, data);
    dev->us_delay(10);
    ICM42670_GIVE_MUTEX(dev);

    return ret_error;

}

/**
  * @brief  Manipulate MREG register value
  *
  * @param  dev     read / write interface definitions(ptr)
  * @param  mreg    mreg register to manipulate
  * @param  mask    register mask value
  * @param  shift   number of bits to left shift
  * @param  value   register new value
  * @retval         interface status (MANDATORY: return 0 -> ICM42670_NO_ERROR)
  *
  */
static inline icm42670_err_t icm42670_manipulate_mreg_register(icm42670_t* dev, icm42670_mreg_number_t mreg_num, uint8_t reg, uint8_t mask, uint8_t shift, uint8_t value)
{
uint8_t data;

    ICM42670_CHECK_ARG_VAL(dev);

    ICM42670_CHECK_RET_VAL(icm42670_read_mreg_register(dev, mreg_num, reg, &data));
    data = (data & ~mask) | (value << shift);
    ICM42670_CHECK_RET_VAL(icm42670_write_mreg_register(dev, mreg_num, reg, &data));

    return ICM42670_NO_ERROR;

}

#if (defined(ICM42670_MUTEX_IS_ENABLED) && (ICM42670_MUTEX_IS_ENABLED == 1))
/**
  * @brief  Take mutex generic function
  *
  * @param  dev             read / write interface definitions(ptr)
  * @retval                 interface status (MANDATORY: return 0 -> ICM42670_NO_ERROR)
  *
  */
static inline icm42670_err_t icm42670_take_mutex(icm42670_t* dev)
{
    ICM42670_CHECK_ARG_VAL(dev);

    if (dev->mutex == ICM42670_MUTEX_IS_FREE)
    {
        dev->mutex= ICM42670_MUTEX_IS_TAKEN;
    }
    else
    {
        return ICM42670_MUTEX_IS_USED_ERROR;
    }

    return ICM42670_NO_ERROR;

}

/**
  * @brief  Take mutex generic function
  *
  * @param  dev     read / write interface definitions(ptr)
  * @retval         interface status (MANDATORY: return 0 -> ICM42670_NO_ERROR)
  *
  */
static inline icm42670_err_t icm42670_give_mutex(icm42670_t* dev)
{
    ICM42670_CHECK_ARG_VAL(dev);

    if (dev->mutex == ICM42670_MUTEX_IS_TAKEN)
    {
        dev->mutex= ICM42670_MUTEX_IS_FREE;
    }
    else
    {
        return ICM42670_MUTEX_IS_USED_ERROR;
    }

    return ICM42670_NO_ERROR;

}
#endif

//*********************************************************************************************************************
/**
 * @brief Initialize device descriptor
 *
 * @param dev           Device descriptor
 * @param i2c_addr      I2C device address, `ICM42670_I2C_ADDR_...` const
 * @param write_reg     i2c write reg callback function
 * @param read_reg      i2c read reg callback function
 * @param gpio_irq_cb   gpio interrupt callback function (to be used in gpio irq)
 * @param ms_delay      milliseconds delay callback function
 * @param us_delay      microseconds delay callback function
 * @return              `ICM42670_NO_ERROR` on success
 */
icm42670_err_t icm42670_init_object
(
    icm42670_t* dev,
    uint8_t  i2c_addr,
    uint32_t i2c_timeout,
    icm42670_i2c_write_t write_reg,
    icm42670_i2c_read_t read_reg,
    icm42670_gpio_irq_cb_t gpio_irq_cb,
    icm42670_msdelay_t ms_delay,
    icm42670_usdelay_t us_delay
)
{
    ICM42670_CHECK_ARG_VAL(dev);

    if ((i2c_addr != ICM42670_I2C_ADDR_GND) && (i2c_addr != ICM42670_I2C_ADDR_VCC))
    {
        // Error log?
        return ICM42670_INVALID_ARG_ERROR;
    }

    dev->i2c_addr= i2c_addr;
    dev->i2c_timeout= i2c_timeout;
    dev->write_reg= write_reg;
    dev->read_reg= read_reg;
    dev->ms_delay= ms_delay;
    dev->us_delay= us_delay;
    dev->mutex= ICM42670_MUTEX_IS_FREE;

    return ICM42670_NO_ERROR;

}

/**
 * @brief Deinitialize device descriptor
 *
 * @param dev   Device descriptor
 * @return      `ICM42670_NO_ERROR` on success
 */
icm42670_err_t icm42670_deinit_object(icm42670_t* dev)
{
    ICM42670_CHECK_ARG_VAL(dev);

    memset(dev, 0, sizeof(icm42670_t));

    return ICM42670_NO_ERROR;

}

/**
 * @brief Initialize device
 *
 * @param dev Device descriptor
 * @return `ICM42670_NO_ERROR` on success
 */
icm42670_err_t icm42670_init(icm42670_t* dev)
{
uint8_t reg_value= 0;
uint8_t mclk_rdy= 0;

    ICM42670_CHECK_ARG_VAL(dev);

    ICM42670_TAKE_MUTEX(dev);
    ICM42670_CHECK_RET_VAL_MUTEX(dev, icm42670_read_reg(dev, ICM42670_REG_WHO_AM_I, &reg_value));
    ICM42670_GIVE_MUTEX(dev);
    if (reg_value != ICM42670_ID)
    {
        // Err log? "Error initializing ICM42670, who_am_i register did not return 0x67"
        return ICM42670_DEVICE_DISCOVERY_FAILED_ERROR;
    }
    // Err log? "Init: Chip ICM42670 detected"

    // flush FIFO
    ICM42670_CHECK_RET_VAL(icm42670_flush_fifo(dev));
    // perform signal path reset
    ICM42670_CHECK_RET_VAL(icm42670_reset(dev));
    // Err log? "Init: Soft-Reset performed"
    // wait 10ms
    dev->ms_delay(10);
    // set device in IDLE power state
    ICM42670_CHECK_RET_VAL(icm42670_set_idle_pwr_mode(dev, 1));
    // wait 10ms
    dev->ms_delay(10);
    // check if internal clock is running
    ICM42670_CHECK_RET_VAL(icm42670_get_mclk_rdy(dev, &mclk_rdy));
    if (!mclk_rdy)
    {
        //Error log?! "Error initializing icm42670, Internal clock not running"
        return ICM42670_INVALID_RESPONSE_ERROR;
    }
    // Err log? "Init: Internal clock running"

    return ICM42670_NO_ERROR;

}

/**
 * @brief Wipes the FIFO
 *
 * @param dev Device descriptor
 * @return `ICM42670_NO_ERROR` on success
 */
icm42670_err_t icm42670_flush_fifo(icm42670_t* dev)
{
uint8_t reg_value = 1 << ICM42670_FIFO_FLUSH_SHIFT;

    ICM42670_CHECK_ARG_VAL(dev);

    ICM42670_TAKE_MUTEX(dev);
    ICM42670_CHECK_RET_VAL_MUTEX(dev, icm42670_write_reg(dev, ICM42670_REG_SIGNAL_PATH_RESET, &reg_value));
    ICM42670_GIVE_MUTEX(dev);
    dev->us_delay(2); // flush is done within 1.5us

    return ICM42670_NO_ERROR;
}

/**
 * @brief Performs a soft-reset
 *
 * @param dev Device descriptor
 * @return `ICM42670_NO_ERROR` on success
 */
icm42670_err_t icm42670_reset(icm42670_t* dev)
{
uint8_t reg_value = 1 << ICM42670_SOFT_RESET_DEVICE_CONFIG_SHIFT;

    ICM42670_CHECK_ARG_VAL(dev);

    ICM42670_TAKE_MUTEX(dev);
    ICM42670_CHECK_RET_VAL_MUTEX(dev, icm42670_write_reg(dev, ICM42670_REG_SIGNAL_PATH_RESET, &reg_value));
    ICM42670_GIVE_MUTEX(dev);

    return ICM42670_NO_ERROR;

}

/**
  * @brief  Read device ID number
  *
  * @param  dev   read / write interface definitions(ptr)
  * @param  data  pointer to buffer that store the data read(ptr)
  * @retval       interface status (MANDATORY: return 0 -> ICM42670_NO_ERROR)
  *
  */
icm42670_err_t icm42670_read_devid(icm42670_t* dev, uint8_t* data)
{
    return icm42670_read_reg(dev, ICM42670_REG_WHO_AM_I, data);

}

/**
 * @brief Set device power mode
 *
 * @param dev           Device descriptor
 * @param enable_idle   Values: 0, 1. (0-> disable idle mode) (1-> enable idle mode)
 * @return              `ICM42670_NO_ERROR` on success
 */
icm42670_err_t icm42670_set_idle_pwr_mode(icm42670_t* dev, uint8_t enable_idle)
{
    ICM42670_CHECK_ARG_VAL(dev);

    ICM42670_CHECK_RET_VAL(icm42670_manipulate_register(dev, ICM42670_REG_PWR_MGMT0, ICM42670_IDLE_BITS, ICM42670_IDLE_SHIFT, enable_idle));

    return ICM42670_NO_ERROR;

}

/**
 * @brief Set gyro power mode
 *
 * @param dev Device descriptor
 * @param pwr_mode struct of type icm42670_gyro_pwr_mode_t
 * @return `ICM42670_NO_ERROR` on success
 */
icm42670_err_t icm42670_set_gyro_pwr_mode(icm42670_t* dev, icm42670_gyro_pwr_mode_t pwr_mode)
{
    ICM42670_CHECK_ARG_VAL(dev);

    ICM42670_CHECK_RET_VAL(icm42670_manipulate_register(dev, ICM42670_REG_PWR_MGMT0, ICM42670_GYRO_MODE_BITS, ICM42670_GYRO_MODE_SHIFT, pwr_mode));
    // no register writes should be performed within the next 200us
    dev->us_delay(300);

    return ICM42670_NO_ERROR;

}

/**
 * @brief Set accel power mode
 *
 * @param dev Device descriptor
 * @param pwr_mode struct of type icm42670_accel_pwr_mode_t
 * @return `ICM42670_NO_ERROR` on success
 */
icm42670_err_t icm42670_set_accel_pwr_mode(icm42670_t* dev, icm42670_accel_pwr_mode_t pwr_mode)
{
icm42670_accel_odr_t odr;
icm42670_accel_avg_t avg;

    ICM42670_CHECK_ARG_VAL(dev);

    // certain odr and avg settings are not allowed in LP or LN mode
    ICM42670_CHECK_RET_VAL(icm42670_get_accel_odr(dev, &odr));
    ICM42670_CHECK_RET_VAL(icm42670_get_accel_avg(dev, &avg));

    if ((pwr_mode == ICM42670_ACCEL_ENABLE_LP_MODE)
        && ((odr == ICM42670_ACCEL_ODR_800HZ) || (odr == ICM42670_ACCEL_ODR_1_6KHZ)
            || ((odr == ICM42670_ACCEL_ODR_200HZ) && (avg == ICM42670_ACCEL_AVG_64X))))
    {
        // Err log? "Accel ODR and AVG settings invalid for Low-power mode"
        return ICM42670_INVALID_ARG_ERROR;
    }

    if ((pwr_mode == ICM42670_ACCEL_ENABLE_LN_MODE)
        && ((odr == ICM42670_ACCEL_ODR_6_25HZ) || (odr == ICM42670_ACCEL_ODR_3_125HZ)
            || (odr == ICM42670_ACCEL_ODR_1_5625HZ)))
    {
        // Err log? "Accel ODR settings invalid for Low-noise mode"
        return ICM42670_INVALID_ARG_ERROR;
    }

    ICM42670_CHECK_RET_VAL(icm42670_manipulate_register(dev, ICM42670_REG_PWR_MGMT0, ICM42670_ACCEL_MODE_BITS, ICM42670_ACCEL_MODE_SHIFT, pwr_mode));

    return ICM42670_NO_ERROR;
}

/**
 * @brief Get the status of the internal clock
 *
 * @param dev Device object struct
 * @param mclk_rdy true if internal clock is running
 * @return `ICM42670_NO_ERROR` on success
 */
icm42670_err_t icm42670_get_mclk_rdy(icm42670_t* dev, uint8_t* mclk_rdy)
{
icm42670_err_t ret_error= ICM42670_NO_ERROR;

    ICM42670_CHECK_ARG_VAL(dev && mclk_rdy);


    ret_error= icm42670_read_reg(dev, ICM42670_REG_MCLK_RDY, mclk_rdy);
    ICM42670_GIVE_MUTEX(dev);

    //Get value of MCLK_RDY bit
    *mclk_rdy= ((*mclk_rdy & ICM42670_MCLK_RDY_BITS) >> ICM42670_MCLK_RDY_SHIFT);

    return ret_error;

}

/**
 * @brief Get the output data rate (ODR) of the accel
 *
 * @param dev Device descriptor
 * @param odr pointer to icm42670_accel_odr_t
 * @return `ICM42670_NO_ERROR` on success
 */
icm42670_err_t icm42670_get_accel_odr(icm42670_t* dev, icm42670_accel_odr_t* odr)
{
uint8_t reg_value;

    ICM42670_CHECK_ARG_VAL(dev && odr);

    ICM42670_TAKE_MUTEX(dev);
    ICM42670_CHECK_RET_VAL_MUTEX(dev, icm42670_read_reg(dev, ICM42670_REG_ACCEL_CONFIG0, &reg_value));
    ICM42670_GIVE_MUTEX(dev);
    *odr = (reg_value & ICM42670_ACCEL_ODR_BITS) >> ICM42670_ACCEL_ODR_SHIFT;

    return ICM42670_NO_ERROR;
}

/**
 * @brief Get the status of the accel averaging
 *
 * @param dev Device descriptor
 * @param avg pointer to icm42670_accel_avg_t
 * @return `ICM42670_NO_ERROR` on success
 */
icm42670_err_t icm42670_get_accel_avg(icm42670_t* dev, icm42670_accel_avg_t* avg)
{
uint8_t reg_value;

    ICM42670_CHECK_ARG_VAL(dev && avg);

    ICM42670_TAKE_MUTEX(dev);
    ICM42670_CHECK_RET_VAL_MUTEX(dev, icm42670_read_reg(dev, ICM42670_REG_ACCEL_CONFIG1, &reg_value));
    ICM42670_GIVE_MUTEX(dev);
    *avg = (reg_value & ICM42670_ACCEL_UI_AVG_BITS) >> ICM42670_ACCEL_UI_AVG_SHIFT;

    return ICM42670_NO_ERROR;
}

/**
 * @brief Set clock source in LP mode
 *
 * @param dev Device descriptor
 * @param clock_source struct of type icm42670_lp_clock_source_t
 * @return `ICM42670_NO_ERROR` on success
 */
icm42670_err_t icm42670_set_low_power_clock(icm42670_t *dev, icm42670_lp_clock_source_t clock_source)
{
    ICM42670_CHECK_ARG_VAL(dev);

    ICM42670_CHECK_RET_VAL(icm42670_manipulate_register(dev, ICM42670_REG_PWR_MGMT0, ICM42670_ACCEL_LP_CLK_SEL_BITS, ICM42670_ACCEL_LP_CLK_SEL_SHIFT, clock_source));

    return ICM42670_NO_ERROR;
}

/**
 * @brief Read 16-bit raw data registers (accelerometer and gyro values)
 *
 * @param dev Device descriptor
 * @param data_register data register to read from
 * @param[out] data accel or gyro data
 * @return `ICM42670_NO_ERROR` on success
 */
icm42670_err_t icm42670_read_raw_data(icm42670_t *dev, uint8_t reg, int16_t *data)
{
    ICM42670_CHECK_ARG_VAL(dev && data);


    ICM42670_TAKE_MUTEX(dev);
    ICM42670_CHECK_RET_VAL_MUTEX(dev, icm42670_read_register_16b(dev, reg, data));
    ICM42670_GIVE_MUTEX(dev);

    return ICM42670_NO_ERROR;
}

/**
 * @brief Read temperature from device
 *
 * @param dev Device descriptor
 * @param[out] temperature temperature, degree C
 * @return `ICM42670_NO_ERROR` on success
 */
icm42670_err_t icm42670_read_temperature(icm42670_t *dev, float *temperature)
{
int16_t reg_value;

    ICM42670_CHECK_ARG_VAL(dev && temperature);

    ICM42670_CHECK_RET_VAL(icm42670_read_raw_data(dev, ICM42670_REG_TEMP_DATA1, &reg_value));
    *temperature = (reg_value / 128.0) + 25;

    return ICM42670_NO_ERROR;

}

/**
 * @brief Set the measurement FSR (Full Scale Range) of the gyro
 *
 * @param dev Device descriptor
 * @param range struct of type icm42670_gyro_fsr_t
 * @return `ICM42670_NO_ERROR` on success
 */
icm42670_err_t icm42670_set_gyro_fsr(icm42670_t *dev, icm42670_gyro_fsr_t range)
{
    ICM42670_CHECK_ARG_VAL(dev);
    return icm42670_manipulate_register(dev, ICM42670_REG_GYRO_CONFIG0, ICM42670_GYRO_UI_FS_SEL_BITS,
        ICM42670_GYRO_UI_FS_SEL_SHIFT, range);
}

/**
 * @brief Set the measurement ODR (Output Data Rate) of the gyro
 *
 * @param dev Device descriptor
 * @param odr struct of type icm42670_gyro_odr_t
 * @return `ICM42670_NO_ERROR` on success
 */
icm42670_err_t icm42670_set_gyro_odr(icm42670_t *dev, icm42670_gyro_odr_t odr)
{
    ICM42670_CHECK_ARG_VAL(dev);
    return icm42670_manipulate_register(dev, ICM42670_REG_GYRO_CONFIG0, ICM42670_GYRO_ODR_BITS, ICM42670_GYRO_ODR_SHIFT, odr);
}

/**
 * @brief Set the measurement FSR (Full Scale Range) of the accelerometer
 *
 * @param dev Device descriptor
 * @param range struct of type icm42670_accel_fsr_t
 * @return `ICM42670_NO_ERROR` on success
 */
icm42670_err_t icm42670_set_accel_fsr(icm42670_t *dev, icm42670_accel_fsr_t range)
{
    ICM42670_CHECK_ARG_VAL(dev);
    return icm42670_manipulate_register(dev, ICM42670_REG_ACCEL_CONFIG0, ICM42670_ACCEL_UI_FS_SEL_BITS,
        ICM42670_ACCEL_UI_FS_SEL_SHIFT, range);
}

/**
 * @brief Set the measurement ODR (Output Data Rate) of the accelerometer
 *
 * @param dev Device descriptor
 * @param odr struct of type icm42670_accel_odr_t
 * @return `ICM42670_NO_ERROR` on success
 */
icm42670_err_t icm42670_set_accel_odr(icm42670_t *dev, icm42670_accel_odr_t odr)
{
    ICM42670_CHECK_ARG_VAL(dev);
    return icm42670_manipulate_register(dev, ICM42670_REG_ACCEL_CONFIG0, ICM42670_ACCEL_ODR_BITS, ICM42670_ACCEL_ODR_SHIFT, odr);
}

/**
 * @brief Set the digital Low-Pass-Filter (LPF) of the temperature sensor
 *
 * @param dev Device descriptor
 * @param lpf_bw struct of type icm42670_temp_lfp_t (bandwidth)
 * @return `ICM42670_NO_ERROR` on success
 */
icm42670_err_t icm42670_set_temp_lpf(icm42670_t *dev, icm42670_temp_lfp_t lpf_bw)
{
    ICM42670_CHECK_ARG_VAL(dev);
    return icm42670_manipulate_register(dev, ICM42670_REG_TEMP_CONFIG0, ICM42670_TEMP_FILT_BW_BITS, ICM42670_TEMP_FILT_BW_SHIFT,
        lpf_bw);
}

/**
 * @brief Set the digital Low-Pass-Filter (LPF) of the gyro
 *
 * @param dev Device descriptor
 * @param lpf_bw struct of type icm42670_gyro_lfp_t (bandwidth)
 * @return `ICM42670_NO_ERROR` on success
 */
icm42670_err_t icm42670_set_gyro_lpf(icm42670_t *dev, icm42670_gyro_lfp_t lpf_bw)
{
    ICM42670_CHECK_ARG_VAL(dev);
    return icm42670_manipulate_register(dev, ICM42670_REG_GYRO_CONFIG1, ICM42670_GYRO_UI_FILT_BW_BITS,
        ICM42670_GYRO_UI_FILT_BW_SHIFT, lpf_bw);
}

/**
 * @brief Set the digital Low-Pass-Filter (LPF) of the accelerometer
 *
 * @param dev Device descriptor
 * @param lpf_bw struct of type icm42670_accel_lfp_t (bandwidth)
 * @return `ICM42670_NO_ERROR` on success
 */
icm42670_err_t icm42670_set_accel_lpf(icm42670_t *dev, icm42670_accel_lfp_t lpf_bw)
{
    ICM42670_CHECK_ARG_VAL(dev);
    return icm42670_manipulate_register(dev, ICM42670_REG_ACCEL_CONFIG1, ICM42670_ACCEL_UI_FILT_BW_BITS,
        ICM42670_ACCEL_UI_FILT_BW_SHIFT, lpf_bw);
}

/**
 * @brief Set the averaging filter of the accelerometer (ONLY IN LOW POWER MODE (LPM))
 *        This field can not be changed, when accel sensor is in LPM!
 *
 * @param dev Device descriptor
 * @param avg struct of type icm42670_accel_avg_t (averaging)
 * @return `ICM42670_NO_ERROR` on success
 */
icm42670_err_t icm42670_set_accel_avg(icm42670_t *dev, icm42670_accel_avg_t avg)
{
    ICM42670_CHECK_ARG_VAL(dev);
    return icm42670_manipulate_register(dev, ICM42670_REG_ACCEL_CONFIG1, ICM42670_ACCEL_UI_AVG_BITS, ICM42670_ACCEL_UI_AVG_SHIFT,
        avg);
}

/**
 * @brief Enable or Disable Wake on Motion (WoM)
 *
 * @param dev Device descriptor
 * @param enable true (1) to enable, false (0) to disable
 * @return `ICM42670_NO_ERROR` on success
 */
icm42670_err_t icm42670_enable_wom(icm42670_t *dev, uint8_t enable)
{
    ICM42670_CHECK_ARG_VAL(dev);
    return icm42670_manipulate_register(dev, ICM42670_REG_WOM_CONFIG, ICM42670_WOM_EN_BITS, ICM42670_WOM_EN_SHIFT, (1 & enable));
}









/**
 * @brief Configures the behaviour of an interrupt pin
 *
 * @param dev Device descriptor
 * @param int_pin interrupt pin (1 or 2)
 * @param config struct of type icm42670_int_config_t
 * @return `ICM42670_NO_ERROR` on success
 */
icm42670_err_t icm42670_config_int_pin(icm42670_t *dev, uint8_t int_pin, icm42670_int_config_t config)
{
uint8_t reg_value = config.mode << 2 | config.drive << 1 | config.polarity;

    ICM42670_CHECK_ARG_VAL(dev && (int_pin < 3) && (int_pin > 0));

    if (int_pin == 2)
    {
        return icm42670_manipulate_register(dev, ICM42670_REG_INT_CONFIG, ICM42670_INT2_CFG_BITS_MASK, ICM42670_INT2_POLARITY_SHIFT, reg_value);
    }
    else
    {
        return icm42670_manipulate_register(dev, ICM42670_REG_INT_CONFIG, ICM42670_INT1_CFG_BITS_MASK, ICM42670_INT1_POLARITY_SHIFT, reg_value);
    }

}

/**
 * @brief Configures the sources for an interrupt
 *
 * @param dev Device descriptor
 * @param int_pin interrupt pin (1 or 2)
 * @param sources struct of type icm42670_int_source_t
 * @return `ICM42670_NO_ERROR` on success
 */
icm42670_err_t icm42670_set_int_sources(icm42670_t *dev, uint8_t int_pin, icm42670_int_source_t sources)
{
uint8_t reg1_val = 0, reg2_val = 0;

    ICM42670_CHECK_ARG_VAL(dev && int_pin < 3 && int_pin > 0);

    if (sources.self_test_done)
        reg1_val = reg1_val | (1 << ICM42670_ST_INT1_EN_SHIFT);
    if (sources.fsync)
        reg1_val = reg1_val | (1 << ICM42670_FSYNC_INT1_EN_SHIFT);
    if (sources.pll_ready)
        reg1_val = reg1_val | (1 << ICM42670_PLL_RDY_INT1_EN_SHIFT);
    if (sources.reset_done)
        reg1_val = reg1_val | (1 << ICM42670_RESET_DONE_INT1_EN_SHIFT);
    if (sources.data_ready)
        reg1_val = reg1_val | (1 << ICM42670_DRDY_INT1_EN_SHIFT);
    if (sources.fifo_threshold)
        reg1_val = reg1_val | (1 << ICM42670_FIFO_THS_INT1_EN_SHIFT);
    if (sources.fifo_full)
        reg1_val = reg1_val | (1 << ICM42670_FIFO_FULL_INT1_EN_SHIFT);
    if (sources.agc_ready)
        reg1_val = reg1_val | (1 << ICM42670_AGC_RDY_INT1_EN_SHIFT);
    if (sources.i3c_error)
        reg2_val = reg2_val | (1 << ICM42670_I3C_PROTOCOL_ERROR_INT1_EN_SHIFT);
    if (sources.smd)
        reg2_val = reg2_val | (1 << ICM42670_SMD_INT1_EN_SHIFT);
    if (sources.wom_z)
        reg2_val = reg2_val | (1 << ICM42670_WOM_Z_INT1_EN_SHIFT);
    if (sources.wom_y)
        reg2_val = reg2_val | (1 << ICM42670_WOM_Y_INT1_EN_SHIFT);
    if (sources.wom_x)
        reg2_val = reg2_val | (1 << ICM42670_WOM_X_INT1_EN_SHIFT);

    if (int_pin == 1)
    {
        ICM42670_TAKE_MUTEX(dev);
        ICM42670_CHECK_RET_VAL_MUTEX(dev, icm42670_write_reg(dev, ICM42670_REG_INT_SOURCE0, &reg1_val));
        ICM42670_CHECK_RET_VAL_MUTEX(dev, icm42670_write_reg(dev, ICM42670_REG_INT_SOURCE1, &reg2_val));
        ICM42670_GIVE_MUTEX(dev);
    }
    else
    {
        ICM42670_TAKE_MUTEX(dev);
        ICM42670_CHECK_RET_VAL_MUTEX(dev, icm42670_write_reg(dev, ICM42670_REG_INT_SOURCE3, &reg1_val));
        ICM42670_CHECK_RET_VAL_MUTEX(dev, icm42670_write_reg(dev, ICM42670_REG_INT_SOURCE4, &reg2_val));
        ICM42670_GIVE_MUTEX(dev);
    }

    return ICM42670_NO_ERROR;

}

/**
 * @brief Configures the Wake on Motion (WoM) behaviour
 *        WoM can only be configured if WoM is not enabled
 *
 * @param dev Device descriptor
 * @param config struct of type icm42670_wom_config_t
 * @return `ICM42670_NO_ERROR` on success
 */
icm42670_err_t icm42670_config_wom(icm42670_t *dev, icm42670_wom_config_t config)
{
    ICM42670_CHECK_ARG_VAL(dev);

    ICM42670_CHECK_RET_VAL(icm42670_manipulate_register(dev, ICM42670_REG_WOM_CONFIG, ICM42670_WOM_INT_DUR_BITS, ICM42670_WOM_INT_DUR_SHIFT, config.trigger));
    ICM42670_CHECK_RET_VAL(icm42670_manipulate_register(dev, ICM42670_REG_WOM_CONFIG, ICM42670_WOM_INT_MODE_BITS, ICM42670_WOM_INT_MODE_SHIFT, config.logical_mode));
    ICM42670_CHECK_RET_VAL(icm42670_manipulate_register(dev, ICM42670_REG_WOM_CONFIG, ICM42670_WOM_MODE_BITS, ICM42670_WOM_MODE_SHIFT, config.reference));

    // WoM threshold values
    ICM42670_CHECK_RET_VAL(icm42670_write_mreg_register(dev, ICM42670_MREG1_RW, ICM42670_REG_ACCEL_WOM_X_THR, &config.wom_x_threshold));
    ICM42670_CHECK_RET_VAL(icm42670_write_mreg_register(dev, ICM42670_MREG1_RW, ICM42670_REG_ACCEL_WOM_Y_THR, &config.wom_y_threshold));
    ICM42670_CHECK_RET_VAL(icm42670_write_mreg_register(dev, ICM42670_MREG1_RW, ICM42670_REG_ACCEL_WOM_Z_THR, &config.wom_z_threshold));

    return ICM42670_NO_ERROR;

}









