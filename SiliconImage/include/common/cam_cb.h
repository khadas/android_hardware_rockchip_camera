/******************************************************************************
 *
 * Copyright 2010, Dream Chip Technologies GmbH. All rights reserved.
 * No part of this work may be reproduced, modified, distributed, transmitted,
 * transcribed, or translated into any language or computer format, in any form
 * or by any means without written permission of:
 * Dream Chip Technologies GmbH, Steinriede 10, 30827 Garbsen / Berenbostel,
 * Germany
 *
 *****************************************************************************/
/**
 * @file cam_cb.h
 *
 * @brief Interface description for image sensor specific implementation (iss).
 *
 *****************************************************************************/
/**
 * @page module_name_page Module Name
 * Describe here what this module does.
 *
 * For a detailed list of functions and implementation detail refer to:
 * - @ref module_name
 *
 * @defgroup cam_cb   Common Camera Type Definitions
 * @{
 *
 */
#ifndef __CAM_CB_H__
#define __CAM_CB_H__

#ifdef __cplusplus
extern "C"
{
#endif

typedef enum CamSensorMode_e
{
    CAMERIC_SENSOR_MODE_INVALID = -1,
    CAM_SENSOR_MODE_FRAME   = 0,
    CAM_SENSOR_MODE_FILED   = 1
} CamSensorMode_t;

typedef enum CamSensorFiledStat_e
{
    CAMERIC_SENSOR_FIELD_INVALID = -1,
    CAM_SENSOR_FIELD_ODD   = 0,
    CAM_SENSOR_FIELD_EVEN   = 1
} CamSensorFiledStat_t;

typedef bool (*getSensorFiledStatCb_t)(void *user, CamSensorFiledStat_t *mode);
typedef bool (*getSensorModeCb_t)(void *user, CamSensorMode_t *mode);
typedef struct CamSensorCb2_s
{
    getSensorFiledStatCb_t  getSensorFiledStatCb;      /**< Notification callback */
    getSensorModeCb_t       getSensorModeCb;      /**< Notification callback */
    void*                  pUserContext;        /**< Pointer to user context to pass to callback */
} CamSensorCb2_t;

#ifdef __cplusplus
}
#endif

/* @} cam_types */

#endif /* __CAM_TYPES_H__ */

