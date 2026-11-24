#ifndef __USBD_CONF_H
#define __USBD_CONF_H

#include "stm32g4xx_hal.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define USBD_MAX_NUM_INTERFACES     1
#define USBD_MAX_NUM_CONFIGURATION  1
#define USBD_MAX_STR_DESC_SIZ       512
#define USBD_DEBUG_LEVEL            0
#define USBD_SELF_POWERED           1
#define MAX_STATIC_ALLOC_SIZE       512

void *USBD_static_malloc(uint32_t size);
void USBD_static_free(void *p);

#endif