#include <string.h>
#include <stdio.h>
#include "i2c.h"
#include "usart.h"

#define CHECK_ARG_LOCK_ON_ERROR(VAL)     \
    do                                   \
    {                                    \
        __asm__("nop");                  \
    }                                    \
    while (VAL)
        
#define DEBUG_IN_ASCII  (0U)

void debug_data_via_uart_in_ascii(void);
void transmit_raw_data_with_delim(void);

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
        }
        
        //Print data via uart
#if (defined(DEBUG_IN_ASCII) && (DEBUG_IN_ASCII == 1))            
        debug_data_via_uart_in_ascii();
#else
        transmit_raw_data_with_delim();
        
#endif

        //500ms wait
        HAL_Delay(500);

    }

}

void debug_data_via_uart_in_ascii(void)
{
    for (uint8_t i= 0; i<6; i++)
    {
        if (i == 2 || i == 5)
        {
            sprintf((char*)data_buffer, "%s%d\r\r", text_arr[i], raw_data[i]);
        }
        else
        {
            sprintf((char*)data_buffer, "%s%d\r", text_arr[i], raw_data[i]);
        }
        HAL_UART_Transmit(&huart1, data_buffer, strlen((char*)data_buffer), HAL_MAX_DELAY);
    }

}

void transmit_raw_data_with_delim(void)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)&raw_data[0], (2*6), HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart1, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);    
    
}
    
    













