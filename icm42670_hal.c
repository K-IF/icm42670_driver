/**********************************************************************************************************************
*                                       Kostas Ifantidis
***********************************************************************************************************************
*
***********************************************************************************************************************
* File Name          : icm42670_hal.c
* Date First Issued  : 01/09/2024
* Author(s)          : Kostas Ifantidis
* Description        : icm42670 hal interface. This file needs to be filled with
*                      necessary functions in order for the icm42670.c driver to work. 
* 
***********************************************************************************************************************/

//    INCLUDES
//*********************************************************************************************************************
#include <stdint.h>
#include "icm42670.h"
#include "icm42670_hal.h"
#include "i2c.h"

//    DEFINES
//*********************************************************************************************************************

#define ICM42670_I2C_ADDR       ICM42670_I2C_ADDR_GND
#define ICM42670_I2C_TIMEOUT    1000


#define CHECK_ARG_LOCK_ON_ERROR(VAL)                                                                                   \
    do                                                                                                                 \
    {                                                                                                                  \
        __asm__("nop");                                                                                                \
    }                                                                                                                  \
    while (VAL)

//    EXTERN FUNCTIONS
//*********************************************************************************************************************

//    FUNCTIONS PROTOTYPES
//*********************************************************************************************************************
void init_accelerometer(void);
void deinit_accelerometer(void);
void run_accelerometer(void);
void goto_sleep_accelerometer(void);
void icm42670_test(void);

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

icm42670_err_t icm42670_hal_i2c_read_reg(icm42670_t* dev, uint8_t reg, uint8_t* data, uint16_t data_len)
{

    return HAL_I2C_Mem_Read(&hi2c2, dev->i2c_addr, reg, I2C_MEMADD_SIZE_8BIT, data, data_len, dev->i2c_timeout);

}

icm42670_err_t icm42670_hal_i2c_write_reg(icm42670_t* dev, uint8_t reg, uint8_t* data, uint16_t data_len)
{

    return HAL_I2C_Mem_Write(&hi2c2, dev->i2c_addr, reg, I2C_MEMADD_SIZE_8BIT, data, data_len, dev->i2c_timeout);

}

void icm42670_hal_ms_delay(uint32_t msec)
{
    __asm__("nop");
    //Add code here...
}

void icm42670_hal_us_delay(uint32_t usec)
{
    __asm__("nop");
    //Add code here...
}

void init_accelerometer(void)
{
    __asm__("null");
    //Add code here...
}

void deinit_accelerometer(void)
{
    __asm__("null");
    //Add code here...
}

void run_accelerometer(void)
{
    __asm__("null");
    //Add code here...
}

void goto_sleep_accelerometer(void)
{
    __asm__("null");
    //Add code here...
}


//*********************************************************************************************************************
// EXAMPLE CODE
//*********************************************************************************************************************
#include <string.h>
#include <stdio.h>
#include "i2c.h"
#include "usart.h"

void debug_data_via_uart_in_ascii(void);


uint8_t reg_arr[10]=
{
    ICM42670_REG_ACCEL_DATA_X1,
    ICM42670_REG_ACCEL_DATA_Y1,
    ICM42670_REG_ACCEL_DATA_Z1,
    ICM42670_REG_GYRO_DATA_X1,
    ICM42670_REG_GYRO_DATA_Y1,
    ICM42670_REG_GYRO_DATA_Z1,
};

char text_arr[10][20]=
{
    "raw_acel_X: ",
    "raw_acel_Y: ",
    "raw_acel_Z: ",
    "raw_GYRO_X: ",
    "raw_GYRO_Y: ",
    "raw_GYRO_Z: ",
};

icm42670_t icm_dev= {0};
volatile float temperature;
volatile int16_t raw_reading;
volatile uint8_t data_register;

volatile int16_t raw_data[12]= {0};
uint8_t data_buffer[50]= {0};

void icm42670_test(void)
{
icm42670_err_t ret_err= ICM42670_NO_ERROR;

    MX_USART1_UART_Init();
    MX_I2C2_Init();

    ret_err= icm42670_init_object(&icm_dev, ICM42670_I2C_ADDR, ICM42670_I2C_TIMEOUT, icm42670_hal_i2c_write_reg, icm42670_hal_i2c_read_reg, (void*) 0, icm42670_hal_ms_delay, icm42670_hal_us_delay);
    CHECK_ARG_LOCK_ON_ERROR(ret_err);
    icm42670_init(&icm_dev);

    // enable accelerometer and gyro in low-noise (LN) mode
    icm42670_set_gyro_pwr_mode(&icm_dev, ICM42670_GYRO_ENABLE_LN_MODE);
    icm42670_set_accel_pwr_mode(&icm_dev, ICM42670_ACCEL_ENABLE_LN_MODE);

    /* OPTIONAL */
    // enable low-pass-filters on accelerometer and gyro
    icm42670_set_accel_lpf(&icm_dev, ICM42670_ACCEL_LFP_53HZ);
    icm42670_set_gyro_lpf(&icm_dev, ICM42670_GYRO_LFP_53HZ);
    // set output data rate (ODR)
    icm42670_set_accel_odr(&icm_dev, ICM42670_ACCEL_ODR_200HZ);
    icm42670_set_gyro_odr(&icm_dev, ICM42670_GYRO_ODR_200HZ);
    // set full scale range (FSR)
    icm42670_set_accel_fsr(&icm_dev, ICM42670_ACCEL_RANGE_16G);
    icm42670_set_gyro_fsr(&icm_dev, ICM42670_GYRO_RANGE_2000DPS);


    // now poll selected accelerometer or gyro raw value directly from registers
    while (1)
    {
        for (uint8_t i= 0; i<6; i++)
        {
            icm42670_read_raw_data(&icm_dev, reg_arr[i], &raw_data[i]);
            HAL_UART_Transmit(&huart1, (uint8_t*)&raw_data[i], 2, HAL_MAX_DELAY);
            HAL_Delay(5);

        }

        raw_data[6]= 3200;
        raw_data[7]= 6699;
        for (uint8_t i= 6; i<8; i++)
        {
            HAL_UART_Transmit(&huart1, (uint8_t*)&raw_data[i], 2, HAL_MAX_DELAY);
            HAL_Delay(5);
        }

        HAL_UART_Transmit(&huart1, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);
//        debug_data_via_uart_in_ascii();

        HAL_Delay(500);

    }

}

void debug_data_via_uart_in_ascii(void)
{
    for (uint8_t i= 0; i<6; i++)
    {
//        icm42670_read_raw_data(&icm_dev, reg_arr[i], (int16_t*)&raw_reading);
        if (i == 2 || i == 5)
        {
            sprintf((char*)data_buffer, "%s%d\r\r", text_arr[i], raw_data[i]);
        }
        else
        {
            sprintf((char*)data_buffer, "%s%d\r", text_arr[i], raw_data[i]);
        }
        HAL_UART_Transmit(&huart1, data_buffer, strlen((char*)data_buffer), HAL_MAX_DELAY);
        HAL_Delay(5);
    }

}















