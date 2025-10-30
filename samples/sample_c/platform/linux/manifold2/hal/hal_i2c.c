/**
 ********************************************************************
 * @file    hal_i2c.c
 * @brief
 *
 * @copyright (c) 2018 DJI. All rights reserved.
 *
 * All information contained herein is, and remains, the property of DJI.
 * The intellectual and technical concepts contained herein are proprietary
 * to DJI and may be covered by U.S. and foreign patents, patents in process,
 * and protected by trade secret or copyright law.  Dissemination of this
 * information, including but not limited to data and other proprietary
 * material(s) incorporated within the information, in any form, is strictly
 * prohibited without the express written consent of DJI.
 *
 * If you receive this source code without DJI’s authorization, you may not
 * further disseminate the information, and you must immediately remove the
 * source code and notify DJI of its removal. DJI reserves the right to pursue
 * legal actions against you for any loss(es) or damage(s) caused by your
 * failure to do so.
 *
 *********************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "hal_i2c.h"
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <errno.h>
#include "uart_i2c_bridge.h"

/* Private constants ---------------------------------------------------------*/
#define I2C_DEVICE_RESET_TIME_US       (25 * 1000)
#define I2C_DEVICE_RESET_GPIO_NUM      (4)
#define DJI_SYSTEM_CMD_STR_MAX_SIZE    (64)

/* Private types -------------------------------------------------------------*/
typedef struct {
    int32_t i2cFd;
} T_I2cHandleStruct;

/* Private values -------------------------------------------------------------*/

/* Exported functions definition ---------------------------------------------*/
T_DjiReturnCode HalI2c_Init(T_DjiHalI2cConfig i2cConfig, T_DjiI2cHandle *i2cHandle)
{
    int ret;
    ret = uart_i2c_bridge_init();
    if (ret != 0) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }
    ret = uart_i2c_reset();
    if (ret != 0) {
        uart_i2c_bridge_close();
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }
    usleep(100*1000); // Wait for device to be ready after reset
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode HalI2c_DeInit(T_DjiI2cHandle i2cHandle)
{
    uart_i2c_bridge_close();
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode HalI2c_WriteData(T_DjiI2cHandle i2cHandle, uint16_t devAddress, const uint8_t *buf,
                                 uint32_t len, uint32_t *realLen)
{
    int ret;

    //打印日志
    // printf("I2C Write Addr: 0x%02X, Len: %u\n", devAddress, len);

    ret=uart_i2c_write(devAddress, buf, len);

    if (ret != 0) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    if (realLen != NULL) {
        *realLen = len;
    }

    //dump data
    // for (uint32_t i = 0; i < len; i++) {
    //     printf("0x%02X ", buf[i]);
    // }
    // printf("\n");

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode HalI2c_ReadData(T_DjiI2cHandle i2cHandle, uint16_t devAddress, uint8_t *buf,
                                uint32_t len, uint32_t *realLen)
{
    int ret;

    // //打印日志
    // printf("I2C Read Addr: 0x%02X, Len: %u\n", devAddress, len);

    ret=uart_i2c_read(devAddress, buf, len);

    if (ret < 0) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    if (realLen != NULL) {
        *realLen = len;
    }

    //dump data
    // for (uint32_t i = 0; i < len; i++) {
    //     printf("0x%02X ", buf[i]);
    // }
    // printf("\n");

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

/****************** (C) COPYRIGHT DJI Innovations *****END OF FILE****/
