/**********************************************************************************************************************
*                                       Kostas Ifantidis
***********************************************************************************************************************
*
***********************************************************************************************************************
* File Name          : icm42670_drv.h
* Date First Issued  : 01/09/2024
* Author(s)          : Kostas Ifantidis
* Description        : icm42670 sensor driver source file
* 
***********************************************************************************************************************/

#ifndef ICM42670_H_
#define ICM42670_H_

//    INCLUDES
//*********************************************************************************************************************
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

//    MACROS
//*********************************************************************************************************************

//    DEFINES
//*********************************************************************************************************************
// Configuration defines
#define ICM42670_MUTEX_IS_ENABLED           (0U)

#define ICM42670_I2C_ADDR_GND               (uint8_t)(0x68 << 1)
#define ICM42670_I2C_ADDR_VCC               (uint8_t)(0x69 << 1)

//#define ICM42670_I2C_ADDR                 ICM42670_I2C_ADDR_GND //Define this defined inside icm42670_hal or user code.
#define ICM42670_ID                         0x67

// Registers USER BANK 0
#define ICM42670_REG_MCLK_RDY               0x00
#define ICM42670_REG_DEVICE_CONFIG          0x01
#define ICM42670_REG_SIGNAL_PATH_RESET      0x02
#define ICM42670_REG_DRIVE_CONFIG1          0x03
#define ICM42670_REG_DRIVE_CONFIG2          0x04
#define ICM42670_REG_DRIVE_CONFIG3          0x05
#define ICM42670_REG_INT_CONFIG             0x06
#define ICM42670_REG_TEMP_DATA1             0x09
#define ICM42670_REG_TEMP_DATA0             0x0A
#define ICM42670_REG_ACCEL_DATA_X1          0x0B
#define ICM42670_REG_ACCEL_DATA_X0          0x0C
#define ICM42670_REG_ACCEL_DATA_Y1          0x0D
#define ICM42670_REG_ACCEL_DATA_Y0          0x0E
#define ICM42670_REG_ACCEL_DATA_Z1          0x0F
#define ICM42670_REG_ACCEL_DATA_Z0          0x10
#define ICM42670_REG_GYRO_DATA_X1           0x11
#define ICM42670_REG_GYRO_DATA_X0           0x12
#define ICM42670_REG_GYRO_DATA_Y1           0x13
#define ICM42670_REG_GYRO_DATA_Y0           0x14
#define ICM42670_REG_GYRO_DATA_Z1           0x15
#define ICM42670_REG_GYRO_DATA_Z0           0x16
#define ICM42670_REG_TMST_FSYNCH            0x17
#define ICM42670_REG_TMST_FSYNCL            0x18
#define ICM42670_REG_APEX_DATA4             0x1D
#define ICM42670_REG_APEX_DATA5             0x1E
#define ICM42670_REG_PWR_MGMT0              0x1F
#define ICM42670_REG_GYRO_CONFIG0           0x20
#define ICM42670_REG_ACCEL_CONFIG0          0x21
#define ICM42670_REG_TEMP_CONFIG0           0x22
#define ICM42670_REG_GYRO_CONFIG1           0x23
#define ICM42670_REG_ACCEL_CONFIG1          0x24
#define ICM42670_REG_APEX_CONFIG0           0x25
#define ICM42670_REG_APEX_CONFIG1           0x26
#define ICM42670_REG_WOM_CONFIG             0x27
#define ICM42670_REG_FIFO_CONFIG1           0x28
#define ICM42670_REG_FIFO_CONFIG2           0x29
#define ICM42670_REG_FIFO_CONFIG3           0x2A
#define ICM42670_REG_INT_SOURCE0            0x2B
#define ICM42670_REG_INT_SOURCE1            0x2C
#define ICM42670_REG_INT_SOURCE3            0x2D
#define ICM42670_REG_INT_SOURCE4            0x2E
#define ICM42670_REG_FIFO_LOST_PKT0         0x2F
#define ICM42670_REG_FIFO_LOST_PKT1         0x30
#define ICM42670_REG_APEX_DATA0             0x31
#define ICM42670_REG_APEX_DATA1             0x32
#define ICM42670_REG_APEX_DATA2             0x33
#define ICM42670_REG_APEX_DATA3             0x34
#define ICM42670_REG_INTF_CONFIG0           0x35
#define ICM42670_REG_INTF_CONFIG1           0x36
#define ICM42670_REG_INT_STATUS_DRDY        0x39
#define ICM42670_REG_INT_STATUS             0x3A
#define ICM42670_REG_INT_STATUS2            0x3B
#define ICM42670_REG_INT_STATUS3            0x3C
#define ICM42670_REG_FIFO_COUNTH            0x3D
#define ICM42670_REG_FIFO_COUNTL            0x3E
#define ICM42670_REG_FIFO_DATA              0x3F
#define ICM42670_REG_WHO_AM_I               0x75
#define ICM42670_REG_BLK_SEL_W              0x79
#define ICM42670_REG_MADDR_W                0x7A
#define ICM42670_REG_M_W                    0x7B
#define ICM42670_REG_BLK_SEL_R              0x7C
#define ICM42670_REG_MADDR_R                0x7D
#define ICM42670_REG_M_R                    0x7E

// MREG1 registers
#define ICM42670_REG_TMST_CONFIG1           0x00
#define ICM42670_REG_FIFO_CONFIG5           0x01
#define ICM42670_REG_FIFO_CONFIG6           0x02
#define ICM42670_REG_FSYNC_CONFIG           0x03
#define ICM42670_REG_INT_CONFIG0            0x04
#define ICM42670_REG_INT_CONFIG1            0x05
#define ICM42670_REG_SENSOR_CONFIG3         0x06
#define ICM42670_REG_ST_CONFIG              0x13
#define ICM42670_REG_SELFTEST               0x14
#define ICM42670_REG_INTF_CONFIG6           0x23
#define ICM42670_REG_INTF_CONFIG10          0x25
#define ICM42670_REG_INTF_CONFIG7           0x28
#define ICM42670_REG_OTP_CONFIG             0x2B
#define ICM42670_REG_INT_SOURCE6            0x2F
#define ICM42670_REG_INT_SOURCE7            0x30
#define ICM42670_REG_INT_SOURCE8            0x31
#define ICM42670_REG_INT_SOURCE9            0x32
#define ICM42670_REG_INT_SOURCE10           0x33
#define ICM42670_REG_APEX_CONFIG2           0x44
#define ICM42670_REG_APEX_CONFIG3           0x45
#define ICM42670_REG_APEX_CONFIG4           0x46
#define ICM42670_REG_APEX_CONFIG5           0x47
#define ICM42670_REG_APEX_CONFIG9           0x48
#define ICM42670_REG_APEX_CONFIG10          0x49
#define ICM42670_REG_APEX_CONFIG11          0x4A
#define ICM42670_REG_ACCEL_WOM_X_THR        0x4B
#define ICM42670_REG_ACCEL_WOM_Y_THR        0x4C
#define ICM42670_REG_ACCEL_WOM_Z_THR        0x4D
#define ICM42670_REG_OFFSET_USER0           0x4E
#define ICM42670_REG_OFFSET_USER1           0x4F
#define ICM42670_REG_OFFSET_USER2           0x50
#define ICM42670_REG_OFFSET_USER3           0x51
#define ICM42670_REG_OFFSET_USER4           0x52
#define ICM42670_REG_OFFSET_USER5           0x53
#define ICM42670_REG_OFFSET_USER6           0x54
#define ICM42670_REG_OFFSET_USER7           0x55
#define ICM42670_REG_OFFSET_USER8           0x56
#define ICM42670_REG_ST_STATUS1             0x63
#define ICM42670_REG_ST_STATUS2             0x64
#define ICM42670_REG_FDR_CONFIG             0x66
#define ICM42670_REG_APEX_CONFIG12          0x67

// MREG2 registers
#define ICM42670_REG_OTP_CTRL7              0x06

// MREG3 registers
#define ICM42670_REG_XA_ST_DATA             0x00
#define ICM42670_REG_YA_ST_DATA             0x01
#define ICM42670_REG_ZA_ST_DATA             0x02
#define ICM42670_REG_XG_ST_DATA             0x03
#define ICM42670_REG_YG_ST_DATA             0x04
#define ICM42670_REG_ZG_ST_DATA             0x05

//    VARIABLE TYPEDEFS
//*********************************************************************************************************************
typedef int32_t icm42670_err_t;

//    ENUMERATION TYPEDEFS
//*********************************************************************************************************************
typedef enum
{
    ICM42670_NO_ERROR = 0,
    ICM42670_DEFAULT_ERROR,
    ICM42670_INVALID_ARG_ERROR,
    ICM42670_INVALID_RESPONSE_ERROR,
    ICM42670_DEVICE_DISCOVERY_FAILED_ERROR,

    ICM42670_MUTEX_IS_USED_ERROR,

    ICM42670_NUMOF_ERRORS

} icm42670_errors_t;

typedef enum
{
    ICM42670_MUTEX_IS_FREE= 0,
    ICM42670_MUTEX_IS_TAKEN,

} icm42670_mutex_t;

/* Gyro power mode */
typedef enum
{
    ICM42670_GYRO_DISABLE        = 0x00,
    ICM42670_GYRO_STANDBY        = 0x01,
    ICM42670_GYRO_ENABLE_LN_MODE = 0x03
} icm42670_gyro_pwr_mode_t;

/* Accelerometer power mode */
typedef enum
{
    ICM42670_ACCEL_DISABLE        = 0x00,
    ICM42670_ACCEL_ENABLE_LP_MODE = 0x02,
    ICM42670_ACCEL_ENABLE_LN_MODE = 0x03
} icm42670_accel_pwr_mode_t;

/* Accelerometer low power mode clock source */
typedef enum
{
    ICM42670_LP_CLK_WUO = 0,
    ICM42670_LP_CLK_RCO = 1
} icm42670_lp_clock_source_t;

/* Gyro FSR (full scale range) */
typedef enum
{
    ICM42670_GYRO_RANGE_2000DPS = 0x00,
    ICM42670_GYRO_RANGE_1000DPS = 0x01,
    ICM42670_GYRO_RANGE_500DPS  = 0x02,
    ICM42670_GYRO_RANGE_250DPS  = 0x03
} icm42670_gyro_fsr_t;

/* Gyro ODR (output data rate) */
typedef enum
{
    ICM42670_GYRO_ODR_12_5HZ = 0x0C,
    ICM42670_GYRO_ODR_25HZ   = 0x0B,
    ICM42670_GYRO_ODR_50HZ   = 0x0A,
    ICM42670_GYRO_ODR_100HZ  = 0x09,
    ICM42670_GYRO_ODR_200HZ  = 0x08,
    ICM42670_GYRO_ODR_400HZ  = 0x07,
    ICM42670_GYRO_ODR_800HZ  = 0x06,
    ICM42670_GYRO_ODR_1_6KHZ = 0x05
} icm42670_gyro_odr_t;

/* Accelerometer FSR (full scale range) */
typedef enum
{
    ICM42670_ACCEL_RANGE_16G = 0x00,
    ICM42670_ACCEL_RANGE_8G  = 0x01,
    ICM42670_ACCEL_RANGE_4G  = 0x02,
    ICM42670_ACCEL_RANGE_2G  = 0x03
} icm42670_accel_fsr_t;

/* Accelerometer ODR (output data rate) */
typedef enum
{
    ICM42670_ACCEL_ODR_1_5625HZ = 0x0F,
    ICM42670_ACCEL_ODR_3_125HZ  = 0x0E,
    ICM42670_ACCEL_ODR_6_25HZ   = 0x0D,
    ICM42670_ACCEL_ODR_12_5HZ   = 0x0C,
    ICM42670_ACCEL_ODR_25HZ     = 0x0B,
    ICM42670_ACCEL_ODR_50HZ     = 0x0A,
    ICM42670_ACCEL_ODR_100HZ    = 0x09,
    ICM42670_ACCEL_ODR_200HZ    = 0x08,
    ICM42670_ACCEL_ODR_400HZ    = 0x07,
    ICM42670_ACCEL_ODR_800HZ    = 0x06,
    ICM42670_ACCEL_ODR_1_6KHZ   = 0x05
} icm42670_accel_odr_t;

/* Temperature LPF (low pass filter) */
typedef enum
{
    ICM42670_TEMP_LFP_BYPASSED = 0x00,
    ICM42670_TEMP_LFP_180HZ    = 0x01,
    ICM42670_TEMP_LFP_72HZ     = 0x02,
    ICM42670_TEMP_LFP_34HZ     = 0x03,
    ICM42670_TEMP_LFP_16HZ     = 0x04,
    ICM42670_TEMP_LFP_8HZ      = 0x05,
    ICM42670_TEMP_LFP_4HZ      = 0x06
} icm42670_temp_lfp_t;

/* Gyro LPF (low pass filter) */
typedef enum
{
    ICM42670_GYRO_LFP_BYPASSED = 0x00,
    ICM42670_GYRO_LFP_180HZ    = 0x01,
    ICM42670_GYRO_LFP_121HZ    = 0x02,
    ICM42670_GYRO_LFP_73HZ     = 0x03,
    ICM42670_GYRO_LFP_53HZ     = 0x04,
    ICM42670_GYRO_LFP_34HZ     = 0x05,
    ICM42670_GYRO_LFP_25HZ     = 0x06,
    ICM42670_GYRO_LFP_16HZ     = 0x07
} icm42670_gyro_lfp_t;

/* Accelerometer LPF (low pass filter) */
typedef enum
{
    ICM42670_ACCEL_LFP_BYPASSED = 0x00,
    ICM42670_ACCEL_LFP_180HZ    = 0x01,
    ICM42670_ACCEL_LFP_121HZ    = 0x02,
    ICM42670_ACCEL_LFP_73HZ     = 0x03,
    ICM42670_ACCEL_LFP_53HZ     = 0x04,
    ICM42670_ACCEL_LFP_34HZ     = 0x05,
    ICM42670_ACCEL_LFP_25HZ     = 0x06,
    ICM42670_ACCEL_LFP_16HZ     = 0x07
} icm42670_accel_lfp_t;

/* Accelerometer averaging (for low power mode) */
typedef enum
{
    ICM42670_ACCEL_AVG_2X  = 0x00,
    ICM42670_ACCEL_AVG_4X  = 0x01,
    ICM42670_ACCEL_AVG_8X  = 0x02,
    ICM42670_ACCEL_AVG_16X = 0x03,
    ICM42670_ACCEL_AVG_32X = 0x04,
    ICM42670_ACCEL_AVG_64X = 0x05
} icm42670_accel_avg_t;

/* Interrupt pin signal mode */
typedef enum
{
    ICM42670_INT_MODE_PULSED = 0,
    ICM42670_INT_MODE_LATCHED = 1
} icm42670_int_mode_t;

/* Interrupt pin signal type */
typedef enum
{
    ICM42670_INT_DRIVE_OPEN_DRAIN = 0,
    ICM42670_INT_DRIVE_PUSH_PULL = 1
} icm42670_int_drive_t;

/* Interrupt pin signal polarity */
typedef enum
{
    ICM42670_INT_POLARITY_ACTIVE_LOW = 0,
    ICM42670_INT_POLARITY_ACTIVE_HIGH = 1
} icm42670_int_polarity_t;

/* Wake on Motion interrupt assertion */
typedef enum
{
    ICM42670_WOM_INT_DUR_FIRST  = 0x00,
    ICM42670_WOM_INT_DUR_SECOND = 0x01,
    ICM42670_WOM_INT_DUR_THIRD  = 0x02,
    ICM42670_WOM_INT_DUR_FOURTH = 0x03
} icm42670_wom_int_dur_t;

/* Wake on Motion interrupt logical trigger */
typedef enum
{
    ICM42670_WOM_INT_MODE_ALL_OR = 0,
    ICM42670_WOM_INT_MODE_ALL_AND = 1
} icm42670_wom_int_mode_t;

/* Wake on Motion reference sample */
typedef enum
{
    ICM42670_WOM_MODE_REF_INITIAL = 0,
    ICM42670_WOM_MODE_REF_LAST = 1
} icm42670_wom_mode_t;

/* MREG 1-3 access */
typedef enum
{
    ICM42670_MREG1_RW = 0x00,
    ICM42670_MREG2_RW = 0x28,
    ICM42670_MREG3_RW = 0x50
} icm42670_mreg_number_t;


//    STRUCTURE TYPEDEFS
//*********************************************************************************************************************
/* Interrupt pin configuration */
typedef struct
{
    icm42670_int_mode_t mode;
    icm42670_int_drive_t drive;
    icm42670_int_polarity_t polarity;

} icm42670_int_config_t;

/* Interrupt source */
typedef struct
{
    uint8_t self_test_done;
    uint8_t fsync;
    uint8_t pll_ready;
    uint8_t reset_done;
    uint8_t data_ready;
    uint8_t fifo_threshold;
    uint8_t fifo_full;
    uint8_t agc_ready;
    uint8_t i3c_error;
    uint8_t smd;
    uint8_t wom_z;
    uint8_t wom_y;
    uint8_t wom_x;

} icm42670_int_source_t;

/* Wake on Motion configuration */
typedef struct
{
    icm42670_wom_int_dur_t trigger;
    icm42670_wom_int_mode_t logical_mode;
    icm42670_wom_mode_t reference;
    uint8_t wom_x_threshold; // 8-bit value between 0 and 1g (Resolution 1g/256=~3.9 mg)
    uint8_t wom_y_threshold; // 8-bit value between 0 and 1g (Resolution 1g/256=~3.9 mg)
    uint8_t wom_z_threshold; // 8-bit value between 0 and 1g (Resolution 1g/256=~3.9 mg)

} icm42670_wom_config_t;

typedef struct icm42670_struct icm42670_t;

/* Function pointer typedefs */
typedef icm42670_err_t (*icm42670_i2c_write_t)(icm42670_t* dev, uint8_t reg, uint8_t* data, uint16_t data_len);
typedef icm42670_err_t (*icm42670_i2c_read_t)(icm42670_t* dev, uint8_t reg, uint8_t* data, uint16_t data_len);
typedef void (*icm42670_gpio_irq_cb_t)(void* params);
typedef void (*icm42670_msdelay_t)(uint32_t ms);
typedef void (*icm42670_usdelay_t)(uint32_t us);

/* Device descriptor */
struct icm42670_struct
{
    uint8_t  i2c_addr;
    uint32_t i2c_timeout;
    /* Mandatory driver fields */
    icm42670_i2c_write_t write_reg;
    icm42670_i2c_read_t read_reg;
    icm42670_gpio_irq_cb_t gpio_irq_cb;
    icm42670_msdelay_t ms_delay;
    icm42670_usdelay_t us_delay;
    /* Proprietary driver fields */
    icm42670_mutex_t mutex;

};

//    GLOBAL FUNCTIONS
//*********************************************************************************************************************
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


//    GLOBAL VARIABLES
//*********************************************************************************************************************

//    STATIC INLINE FUNCTIONS
//*********************************************************************************************************************


#ifdef __cplusplus
}
#endif

#endif //ICM42670_H_
