/*
 $License:
    Copyright (C) 2011-2012 InvenSense Corporation, All Rights Reserved.
 $
 */
/**
 *  @addtogroup  DRIVERS Sensor Driver Layer
 *  @brief       Hardware drivers to communicate with sensors via I2C.
 *
 *  @{
 *      @file       inv_mpu_dmp_motion_driver.h
 *      @brief      DMP image and interface functions.
 *      @details    All functions are preceded by the dmp_ prefix to
 *                  differentiate among MPL and general driver function calls.
 */

#ifndef _INV_MPU_DMP_MOTION_DRIVER_H_
#define _INV_MPU_DMP_MOTION_DRIVER_H_

#define TAP_X               (0x01)
#define TAP_Y               (0x02)
#define TAP_Z               (0x04)
#define TAP_XYZ             (0x07)

#define TAP_X_UP            (0x01)
#define TAP_X_DOWN          (0x02)
#define TAP_Y_UP            (0x03)
#define TAP_Y_DOWN          (0x04)
#define TAP_Z_UP            (0x05)
#define TAP_Z_DOWN          (0x06)

#define ANDROID_ORIENT_PORTRAIT             (0x00)
#define ANDROID_ORIENT_LANDSCAPE            (0x01)
#define ANDROID_ORIENT_REVERSE_PORTRAIT     (0x02)
#define ANDROID_ORIENT_REVERSE_LANDSCAPE    (0x03)

#define DMP_INT_GESTURE     (0x01)
#define DMP_INT_CONTINUOUS  (0x02)

#define DMP_FEATURE_TAP             (0x001)
#define DMP_FEATURE_ANDROID_ORIENT  (0x002)
#define DMP_FEATURE_LP_QUAT         (0x004)
#define DMP_FEATURE_PEDOMETER       (0x008)
#define DMP_FEATURE_6X_LP_QUAT      (0x010)
#define DMP_FEATURE_GYRO_CAL        (0x020)
#define DMP_FEATURE_SEND_RAW_ACCEL  (0x040)
#define DMP_FEATURE_SEND_RAW_GYRO   (0x080)
#define DMP_FEATURE_SEND_CAL_GYRO   (0x100)

#define INV_WXYZ_QUAT       (0x100)

#endif  /* _INV_MPU_DMP_MOTION_DRIVER_H_ */
