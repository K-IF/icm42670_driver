/**********************************************************************************************************************
*                                       Kostas Ifantidis
***********************************************************************************************************************
*
***********************************************************************************************************************
* File Name          : icm42670_hal.h
* Date First Issued  : 01/09/2024
* Author(s)          : Kostas Ifantidis
* Description        : icm42670 hal interface header file.
* 
***********************************************************************************************************************/

#ifndef ICM42670_HAL_H_
#define ICM42670_HAL_H_

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

//    VARIABLE TYPEDEFS
//*********************************************************************************************************************

//    ENUMERATION TYPEDEFS
//*********************************************************************************************************************

//    STRUCTURE TYPEDEFS
//*********************************************************************************************************************

//    GLOBAL FUNCTIONS
//*********************************************************************************************************************
void init_accelerometer(void);
void deinit_accelerometer(void);
void run_accelerometer(void);
void goto_sleep_accelerometer(void);
void icm42670_test(void);

//    GLOBAL VARIABLES
//*********************************************************************************************************************

//    STATIC INLINE FUNCTIONS
//*********************************************************************************************************************


#ifdef __cplusplus
}
#endif

#endif //ICM42670_HAL_H_
