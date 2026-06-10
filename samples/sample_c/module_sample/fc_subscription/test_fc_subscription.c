/**
 ********************************************************************
 * @file    test_fc_subscription.c
 * @brief
 * killall /usr/local/bin/dj
 * @copyright (c) 2021 DJI. All rights reserved.
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
#include <utils/util_misc.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include "test_fc_subscription.h"
#include "dji_logger.h"
#include "dji_platform.h"
#include "widget_interaction_test/test_widget_interaction.h"

#include "sbgc32.h"

/* Private constants ---------------------------------------------------------*/
#define FC_SUBSCRIPTION_TASK_FREQ (1)
#define FC_SUBSCRIPTION_TASK_STACK_SIZE (2048)

/* Private types -------------------------------------------------------------*/

/* Private functions declaration ---------------------------------------------*/
static void *UserFcSubscription_Task(void *arg);
static T_DjiReturnCode DjiTest_FcSubscriptionReceiveQuaternionCallback(const uint8_t *data, uint16_t dataSize,
                                                                       const T_DjiDataTimestamp *timestamp);

static T_DjiReturnCode GimbalCallback(const uint8_t *data, uint16_t dataSize,
                                      const T_DjiDataTimestamp *timestamp);

static int CompareGpsTimeWithSystemTime(T_DjiFcSubscriptionGpsDate gpsDate, T_DjiFcSubscriptionGpsTime gpsTime);
static void ResetSystemTimeWithGpsTime(T_DjiFcSubscriptionGpsDate gpsDate, T_DjiFcSubscriptionGpsTime gpsTime);
static bool DjiParseGpsUtcTime(T_DjiFcSubscriptionGpsDate gpsDate, T_DjiFcSubscriptionGpsTime gpsTime,
                               struct tm *utcTm);
static time_t DjiTimegm(struct tm *utcTm);

/* Private variables ---------------------------------------------------------*/
static T_DjiTaskHandle s_userFcSubscriptionThread;
static bool s_userFcSubscriptionDataShow = false;
static uint8_t s_totalSatelliteNumberUsed = 0;
static uint32_t s_userFcSubscriptionDataCnt = 0;

static sbgcGeneral_t SBGC32_Device;
static sbgcControl_t Control;

/* Exported functions definition ---------------------------------------------*/
T_DjiReturnCode DjiTest_FcSubscriptionStartService(void)
{
    T_DjiReturnCode djiStat;
    T_DjiOsalHandler *osalHandler = NULL;

    osalHandler = DjiPlatform_GetOsalHandler();
    djiStat = DjiFcSubscription_Init();
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
        USER_LOG_ERROR("init data subscription module error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    // Init SBGC32
    SBGC32_Init(&SBGC32_Device);
    memset(&Control, 0, sizeof(Control));

    // djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION, DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ,
    //                                            DjiTest_FcSubscriptionReceiveQuaternionCallback);
    // if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    //     USER_LOG_ERROR("Subscribe topic quaternion error.");
    //     return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    // } else {
    //     USER_LOG_DEBUG("Subscribe topic quaternion success.");
    // }

    // djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_VELOCITY, DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ,
    //                                            NULL);
    // if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    //     USER_LOG_ERROR("Subscribe topic velocity error.");
    //     return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    // } else {
    //     USER_LOG_DEBUG("Subscribe topic velocity success.");
    // }

    // djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_POSITION, DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ,
    //                                            NULL);
    // if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    //     USER_LOG_ERROR("Subscribe topic gps position error.");
    //     return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    // } else {
    //     USER_LOG_DEBUG("Subscribe topic gps position success.");
    // }

    // djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_DETAILS, DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ,
    //                                            NULL);
    // if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    //     USER_LOG_ERROR("Subscribe topic gps details error.");
    //     return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    // } else {
    //     USER_LOG_DEBUG("Subscribe topic gps details success.");
    // }

    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_DATE, DJI_DATA_SUBSCRIPTION_TOPIC_5_HZ,
                                               NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
        USER_LOG_ERROR("Subscribe topic gps date error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }
    else
    {
        USER_LOG_DEBUG("Subscribe topic gps date success.");
    }

    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_TIME, DJI_DATA_SUBSCRIPTION_TOPIC_5_HZ,
                                               NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
        USER_LOG_ERROR("Subscribe topic gps time error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }
    else
    {
        USER_LOG_DEBUG("Subscribe topic gps time success.");
    }

    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_GIMBAL_ANGLES, DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ,
                                               GimbalCallback);

    if (osalHandler->TaskCreate("user_subscription_task", UserFcSubscription_Task,
                                FC_SUBSCRIPTION_TASK_STACK_SIZE, NULL, &s_userFcSubscriptionThread) !=
        DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("user data subscription task create error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode DjiTest_FcSubscriptionRunSample(void)
{
    T_DjiReturnCode djiStat;
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    T_DjiFcSubscriptionVelocity velocity = {0};
    T_DjiDataTimestamp timestamp = {0};
    T_DjiFcSubscriptionGpsPosition gpsPosition = {0};
    T_DjiFcSubscriptionSingleBatteryInfo singleBatteryInfo = {0};

    USER_LOG_INFO("Fc subscription sample start");
    s_userFcSubscriptionDataShow = true;

    USER_LOG_INFO("--> Step 1: Init fc subscription module");
    djiStat = DjiFcSubscription_Init();
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
        USER_LOG_ERROR("init data subscription module error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    USER_LOG_INFO("--> Step 2: Subscribe the topics of quaternion, velocity and gps position");
    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION, DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ,
                                               DjiTest_FcSubscriptionReceiveQuaternionCallback);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
        USER_LOG_ERROR("Subscribe topic quaternion error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_VELOCITY, DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ,
                                               NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
        USER_LOG_ERROR("Subscribe topic velocity error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_POSITION, DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ,
                                               NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
        USER_LOG_ERROR("Subscribe topic gps position error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    USER_LOG_INFO("--> Step 3: Get latest value of the subscribed topics in the next 10 seconds\r\n");

    for (int i = 0; i < 10; ++i)
    {
        osalHandler->TaskSleepMs(1000 / FC_SUBSCRIPTION_TASK_FREQ);
        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_VELOCITY,
                                                          (uint8_t *)&velocity,
                                                          sizeof(T_DjiFcSubscriptionVelocity),
                                                          &timestamp);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
        {
            USER_LOG_ERROR("get value of topic velocity error.");
        }
        else
        {
            USER_LOG_INFO("velocity: x = %f y = %f z = %f healthFlag = %d, timestamp ms = %d us = %d.", velocity.data.x,
                          velocity.data.y,
                          velocity.data.z, velocity.health, timestamp.millisecond, timestamp.microsecond);
        }

        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_POSITION,
                                                          (uint8_t *)&gpsPosition,
                                                          sizeof(T_DjiFcSubscriptionGpsPosition),
                                                          &timestamp);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
        {
            USER_LOG_ERROR("get value of topic gps position error.");
        }
        else
        {
            USER_LOG_INFO("gps position: x = %d y = %d z = %d.", gpsPosition.x, gpsPosition.y, gpsPosition.z);
        }

        // Attention: if you want to subscribe the single battery info on M300 RTK, you need connect USB cable to
        // OSDK device or use topic DJI_FC_SUBSCRIPTION_TOPIC_BATTERY_INFO instead.
        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_BATTERY_SINGLE_INFO_INDEX1,
                                                          (uint8_t *)&singleBatteryInfo,
                                                          sizeof(T_DjiFcSubscriptionSingleBatteryInfo),
                                                          &timestamp);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
        {
            USER_LOG_ERROR("get value of topic battery single info index1 error.");
        }
        else
        {
            USER_LOG_INFO(
                "battery single info index1: capacity percent = %ld% voltage = %ldV temperature = %.2f degree.",
                singleBatteryInfo.batteryCapacityPercent,
                singleBatteryInfo.currentVoltage / 1000,
                (dji_f32_t)singleBatteryInfo.batteryTemperature / 10);
        }

        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_BATTERY_SINGLE_INFO_INDEX2,
                                                          (uint8_t *)&singleBatteryInfo,
                                                          sizeof(T_DjiFcSubscriptionSingleBatteryInfo),
                                                          &timestamp);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
        {
            USER_LOG_ERROR("get value of topic battery single info index2 error.");
        }
        else
        {
            USER_LOG_INFO(
                "battery single info index2: capacity percent = %ld% voltage = %ldV temperature = %.2f degree.\r\n",
                singleBatteryInfo.batteryCapacityPercent,
                singleBatteryInfo.currentVoltage / 1000,
                (dji_f32_t)singleBatteryInfo.batteryTemperature / 10);
        }
    }

    USER_LOG_INFO("--> Step 4: Unsubscribe the topics of quaternion, velocity and gps position");
    djiStat = DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
        USER_LOG_ERROR("UnSubscribe topic quaternion error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_VELOCITY);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
        USER_LOG_ERROR("UnSubscribe topic quaternion error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_POSITION);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
        USER_LOG_ERROR("UnSubscribe topic quaternion error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    USER_LOG_INFO("--> Step 5: Deinit fc subscription module");

    djiStat = DjiFcSubscription_DeInit();
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
        USER_LOG_ERROR("Deinit fc subscription error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    s_userFcSubscriptionDataShow = false;
    USER_LOG_INFO("Fc subscription sample end");

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode DjiTest_FcSubscriptionDataShowTrigger(void)
{
    s_userFcSubscriptionDataShow = !s_userFcSubscriptionDataShow;

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode DjiTest_FcSubscriptionGetTotalSatelliteNumber(uint8_t *number)
{
    *number = s_totalSatelliteNumberUsed;

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

/* Private functions definition-----------------------------------------------*/
#ifndef __CC_ARM
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-noreturn"
#pragma GCC diagnostic ignored "-Wreturn-type"
#endif

static bool DjiParseGpsUtcTime(T_DjiFcSubscriptionGpsDate gpsDate, T_DjiFcSubscriptionGpsTime gpsTime,
                               struct tm *utcTm)
{
    int year = gpsDate / 10000;
    int month = (gpsDate / 100) % 100;
    int day = gpsDate % 100;
    int hour = gpsTime / 10000;
    int minute = (gpsTime / 100) % 100;
    int second = gpsTime % 100;

    if (utcTm == NULL)
    {
        return false;
    }

    if (year < 1970 || month < 1 || month > 12 || day < 1 || day > 31 ||
        hour < 0 || hour > 23 || minute < 0 || minute > 59 || second < 0 || second > 60)
    {
        USER_LOG_ERROR("invalid gps date/time: %d %d", gpsDate, gpsTime);
        return false;
    }

    memset(utcTm, 0, sizeof(*utcTm));
    utcTm->tm_year = year - 1900;
    utcTm->tm_mon = month - 1;
    utcTm->tm_mday = day;
    utcTm->tm_hour = hour;
    utcTm->tm_min = minute;
    utcTm->tm_sec = second;
    utcTm->tm_isdst = 0;

    return true;
}

static time_t DjiTimegm(struct tm *utcTm)
{
#if defined(__linux__) || defined(__GLIBC__)
    return timegm(utcTm);
#else
    char *oldTz = getenv("TZ");
    char *oldTzCopy = oldTz != NULL ? strdup(oldTz) : NULL;
    time_t result;

    setenv("TZ", "UTC", 1);
    tzset();
    result = mktime(utcTm);

    if (oldTzCopy != NULL)
    {
        setenv("TZ", oldTzCopy, 1);
        free(oldTzCopy);
    }
    else
    {
        unsetenv("TZ");
    }
    tzset();

    return result;
#endif
}

static int CompareGpsTimeWithSystemTime(T_DjiFcSubscriptionGpsDate gpsDate, T_DjiFcSubscriptionGpsTime gpsTime)
{
    struct tm utcTm;
    time_t gpsUtc;
    time_t now;
    long long diff;

    if (!DjiParseGpsUtcTime(gpsDate, gpsTime, &utcTm))
    {
        return 0;
    }

    gpsUtc = DjiTimegm(&utcTm);
    if (gpsUtc == (time_t)-1)
    {
        USER_LOG_ERROR("convert gps time to utc failed");
        return 0;
    }

    now = time(NULL);
    diff = (long long)now - (long long)gpsUtc;

    return (int)(diff >= 0 ? diff : -diff);
}

static void ResetSystemTimeWithGpsTime(T_DjiFcSubscriptionGpsDate gpsDate, T_DjiFcSubscriptionGpsTime gpsTime)
{
    struct tm utcTm;
    time_t gpsUtc;
    struct timeval tv;

    if (!DjiParseGpsUtcTime(gpsDate, gpsTime, &utcTm))
    {
        return;
    }

    gpsUtc = DjiTimegm(&utcTm);
    if (gpsUtc == (time_t)-1)
    {
        USER_LOG_ERROR("convert gps time to utc failed");
        return;
    }

    tv.tv_sec = gpsUtc;
    tv.tv_usec = 0;

    if (settimeofday(&tv, NULL) != 0)
    {
        USER_LOG_ERROR("settimeofday failed, errno=%d", errno);
        return;
    }

    USER_LOG_INFO("system time updated using gps utc time: %d %d", gpsDate, gpsTime);
}

static void *UserFcSubscription_Task(void *arg)
{
    T_DjiReturnCode djiStat;
    T_DjiFcSubscriptionVelocity velocity = {0};
    T_DjiDataTimestamp timestamp = {0};
    T_DjiFcSubscriptionGpsPosition gpsPosition = {0};
    T_DjiFcSubscriptionGpsDetails gpsDetails = {0};
    T_DjiOsalHandler *osalHandler = NULL;
    T_DjiDataTimestamp timestampGps = {0};
    T_DjiFcSubscriptionGpsDate gpsDate = 0;
    T_DjiFcSubscriptionGpsTime gpsTime = 0;
    USER_UTIL_UNUSED(arg);
    osalHandler = DjiPlatform_GetOsalHandler();

    while (1)
    {
        osalHandler->TaskSleepMs(1000 / FC_SUBSCRIPTION_TASK_FREQ);

        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_DATE,
                                                          (uint8_t *)&gpsDate,
                                                          sizeof(T_DjiFcSubscriptionGpsDate),
                                                          &timestampGps);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
        {
            USER_LOG_ERROR("get value of GPS date error.");
            continue;
        }
        // USER_LOG_INFO("DATE:%d",gpsDate);
        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_TIME,
                                                          (uint8_t *)&gpsTime,
                                                          sizeof(T_DjiFcSubscriptionGpsTime),
                                                          &timestampGps);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
        {
            USER_LOG_ERROR("get value of GPS time error.");
            continue;
        }
        // USER_LOG_INFO("TIME:%d",gpsTime);
        //若获取到的GPS时间(gpsDate:yyyymmdd,gpsTime:hhmmss)与系统时间相差超过30s,则根据GPS时间重设系统时间
        if(CompareGpsTimeWithSystemTime(gpsDate, gpsTime) > 30)
        {
            ResetSystemTimeWithGpsTime(gpsDate, gpsTime);
        }


        /* djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_VELOCITY,
                                                          (uint8_t *)&velocity,
                                                          sizeof(T_DjiFcSubscriptionVelocity),
                                                          &timestamp);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
        {
            USER_LOG_ERROR("get value of topic velocity error.");
        }

        if (s_userFcSubscriptionDataShow == true)
        {
            USER_LOG_INFO("velocity: x %f y %f z %f, healthFlag %d.", velocity.data.x, velocity.data.y,
                          velocity.data.z, velocity.health);
        }

        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_POSITION,
                                                          (uint8_t *)&gpsPosition,
                                                          sizeof(T_DjiFcSubscriptionGpsPosition),
                                                          &timestamp);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
        {
            USER_LOG_ERROR("get value of topic gps position error.");
        }

        if (s_userFcSubscriptionDataShow == true)
        {
            USER_LOG_INFO("gps position: x %d y %d z %d.", gpsPosition.x, gpsPosition.y, gpsPosition.z);
        }

        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_DETAILS,
                                                          (uint8_t *)&gpsDetails,
                                                          sizeof(T_DjiFcSubscriptionGpsDetails),
                                                          &timestamp);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
        {
            USER_LOG_ERROR("get value of topic gps details error.");
        }

        if (s_userFcSubscriptionDataShow == true)
        {
            USER_LOG_INFO("gps total satellite number used: %d %d %d.",
                          gpsDetails.gpsSatelliteNumberUsed,
                          gpsDetails.glonassSatelliteNumberUsed,
                          gpsDetails.totalSatelliteNumberUsed);
            s_totalSatelliteNumberUsed = gpsDetails.totalSatelliteNumberUsed;
        } */
    }
}

#ifndef __CC_ARM
#pragma GCC diagnostic pop
#endif

static T_DjiReturnCode DjiTest_FcSubscriptionReceiveQuaternionCallback(const uint8_t *data, uint16_t dataSize,
                                                                       const T_DjiDataTimestamp *timestamp)
{
    T_DjiFcSubscriptionQuaternion *quaternion = (T_DjiFcSubscriptionQuaternion *)data;
    dji_f64_t pitch, yaw, roll;

    USER_UTIL_UNUSED(dataSize);

    pitch = (dji_f64_t)asinf(-2 * quaternion->q1 * quaternion->q3 + 2 * quaternion->q0 * quaternion->q2) * 57.3;
    roll = (dji_f64_t)atan2f(2 * quaternion->q2 * quaternion->q3 + 2 * quaternion->q0 * quaternion->q1,
                             -2 * quaternion->q1 * quaternion->q1 - 2 * quaternion->q2 * quaternion->q2 + 1) *
           57.3;
    yaw = (dji_f64_t)atan2f(2 * quaternion->q1 * quaternion->q2 + 2 * quaternion->q0 * quaternion->q3,
                            -2 * quaternion->q2 * quaternion->q2 - 2 * quaternion->q3 * quaternion->q3 + 1) *
          57.3;

    if (s_userFcSubscriptionDataShow == true)
    {
        if (s_userFcSubscriptionDataCnt++ % DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ == 0)
        {
            USER_LOG_INFO("receive quaternion data.");
            USER_LOG_INFO("timestamp: millisecond %u microsecond %u.", timestamp->millisecond,
                          timestamp->microsecond);
            USER_LOG_INFO("quaternion: %f %f %f %f.", quaternion->q0, quaternion->q1, quaternion->q2,
                          quaternion->q3);

            USER_LOG_INFO("euler angles: pitch = %.2f roll = %.2f yaw = %.2f.\r\n", pitch, roll, yaw);
            DjiTest_WidgetLogAppend("pitch = %.2f roll = %.2f yaw = %.2f.", pitch, roll, yaw);
        }
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

/****************** (C) COPYRIGHT DJI Innovations *****END OF FILE****/

static T_DjiReturnCode GimbalCallback(const uint8_t *data, uint16_t dataSize,
                                      const T_DjiDataTimestamp *timestamp)
{
    static T_DjiFcSubscriptionGimbalAngles lastGimbalAngles = {0};
    T_DjiFcSubscriptionGimbalAngles *gimbalAngles = (T_DjiFcSubscriptionGimbalAngles *)data;
    dji_f32_t pitch_speed;
    sbgcGetAngles_t GetAngles = {0};

    USER_UTIL_UNUSED(dataSize);
    USER_UTIL_UNUSED(timestamp);

    if (lastGimbalAngles.x != gimbalAngles->x)
    {
        // 获取当前sbgc云台角度
        SBGC32_GetAngles(&SBGC32_Device, &GetAngles);

        // pitch_speed = (gimbalAngles->x - lastGimbalAngles.x) / 0.02; // 50Hz
        lastGimbalAngles.x = gimbalAngles->x;
        Control.mode[PITCH] = CtrlMODE_ANGLE | CtrlFLAG_MIX_FOLLOW;
        Control.AxisC[PITCH].angle = sbgcAngleToDegree(gimbalAngles->x); // unit: degree
        Control.AxisC[PITCH].speed = sbgcSpeedToValue(SBGC_MAX_SPEED);      // unit: degree/s
        SBGC32_Control(&SBGC32_Device, &Control);
    }

    // USER_LOG_INFO("gimbal angles: pitch = %.2f roll = %.2f yaw = %.2f.\r\n",
    //               gimbalAngles->x, gimbalAngles->y, gimbalAngles->z);

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}