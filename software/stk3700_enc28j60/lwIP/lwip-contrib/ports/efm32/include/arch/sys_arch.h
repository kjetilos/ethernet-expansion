#ifndef _LWIP_ARCH_SYS_ARCH_H_
#define _LWIP_ARCH_SYS_ARCH_H_

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

typedef xSemaphoreHandle sys_sem_t;
typedef xSemaphoreHandle sys_mutex_t;
typedef xQueueHandle sys_mbox_t;
typedef xTaskHandle sys_thread_t;

#endif /* _LWIP_ARCH_SYS_ARCH_H_ */
