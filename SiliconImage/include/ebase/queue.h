/******************************************************************************
 *
 * The copyright in this software is owned by Rockchip and/or its licensors.
 * This software is made available subject to the conditions of the license 
 * terms to be determined and negotiated by Rockchip and you.
 * THIS SOFTWARE IS PROVIDED TO YOU ON AN "AS IS" BASIS and ROCKCHP AND/OR 
 * ITS LICENSORS DISCLAIMS ANY AND ALL WARRANTIES AND REPRESENTATIONS WITH 
 * RESPECT TO SUCH SOFTWARE, WHETHER EXPRESS,IMPLIED, STATUTORY OR OTHERWISE, 
 * INCLUDING WITHOUT LIMITATION, ANY IMPLIED WARRANTIES OF TITLE, NON-INFRINGEMENT, 
 * MERCHANTABILITY, SATISFACTROY QUALITY, ACCURACY OR FITNESS FOR A PARTICULAR PURPOSE. 
 * Except as expressively authorized by Rockchip and/or its licensors, you may not 
 * (a) disclose, distribute, sell, sub-license, or transfer this software to any third party, 
 * in whole or part; (b) modify this software, in whole or part; (c) decompile, reverse-engineer, 
 * dissemble, or attempt to derive any source code from the software.
 *
 *****************************************************************************/
/**
 * @file queue.h
 *
 * @brief
 *   Extended data types: Queue
 *
 *****************************************************************************/
/**
 * @defgroup module_ext_queue Queue
 *
 * @{
 *
 *****************************************************************************/
#ifndef __QUEUE_H__
#define __QUEUE_H__

#include "types.h"
#include "ext_types.h"
#include "list.h"


/**
 * @brief Structure that represents an element in the queue.
 */
typedef struct
{
    GList *head;        /**< Head element of queue */
    GList *tail;        /**< Tail  element of queue  */
    uint32_t length;    /**< Length  of queue  */
} GQueue;


/*****************************************************************************/
/**
 * @brief
 *
 * @param
 *
 * @return
 * @retval
 *
 *****************************************************************************/
void* queuePopHead(GQueue* queue);


/*****************************************************************************/
/**
 * @brief
 *
 * @param
 *
 * @return
 * @retval
 *
 *****************************************************************************/
void queuePushHead(GQueue* queue, void* data);

/*****************************************************************************/
/**
 * @brief
 *
 * @param
 *
 * @return
 * @retval
 *
 *****************************************************************************/
bool_t queueIsEmpty(GQueue* queue);

/*****************************************************************************/
/**
 * @brief
 *
 * @param
 *
 * @return
 * @retval
 *
 *****************************************************************************/
GQueue* queueNew(void);

/*****************************************************************************/
/**
 * @brief
 *
 * @param
 *
 * @return
 * @retval
 *
 *****************************************************************************/
void queueFree(GQueue* queue);

/* @} module_ext_queue */

#endif /* __QUEUE_H__ */
