#include "arch/sys_arch.h"

#include "lwip/opt.h"

#include "lwip/debug.h"
#include "lwip/def.h"
#include "lwip/sys.h"
#include "lwip/mem.h"
#include "lwip/stats.h"

/* Very crude mechanism used to determine if the critical section handling
functions are being called from an interrupt context or not.  This relies on
the interrupt handler setting this variable manually. */
portBASE_TYPE xInsideISR = pdFALSE;

/*---------------------------------------------------------------------------*
 * Routine:  sys_mbox_new
 *---------------------------------------------------------------------------*
 * Description:
 *      Creates a new mailbox
 * Inputs:
 *      int size                -- Size of elements in the mailbox
 * Outputs:
 *      sys_mbox_t              -- Handle to new mailbox
 *---------------------------------------------------------------------------*/
err_t sys_mbox_new( sys_mbox_t *pxMailBox, int iSize )
{
  err_t xReturn = ERR_MEM;

  *pxMailBox = xQueueCreate( iSize, sizeof( void * ) );

  if( *pxMailBox != NULL )
  {
    xReturn = ERR_OK;
    SYS_STATS_INC_USED( mbox );
  }

  return xReturn;
}


/*---------------------------------------------------------------------------*
 * Routine:  sys_mbox_free
 *---------------------------------------------------------------------------*
 * Description:
 *      Deallocates a mailbox. If there are messages still present in the
 *      mailbox when the mailbox is deallocated, it is an indication of a
 *      programming error in lwIP and the developer should be notified.
 * Inputs:
 *      sys_mbox_t mbox         -- Handle of mailbox
 * Outputs:
 *      sys_mbox_t              -- Handle to new mailbox
 *---------------------------------------------------------------------------*/
void sys_mbox_free( sys_mbox_t *pxMailBox )
{
  unsigned long ulMessagesWaiting;

  ulMessagesWaiting = uxQueueMessagesWaiting( *pxMailBox );
  configASSERT( ( ulMessagesWaiting == 0 ) );

  #if SYS_STATS
  {
    if( ulMessagesWaiting != 0UL )
    {
      SYS_STATS_INC( mbox.err );
    }

    SYS_STATS_DEC( mbox.used );
  }
  #endif /* SYS_STATS */

  vQueueDelete( *pxMailBox );
}

/*---------------------------------------------------------------------------*
 * Routine:  sys_mbox_post
 *---------------------------------------------------------------------------*
 * Description:
 *      Post the "msg" to the mailbox.
 * Inputs:
 *      sys_mbox_t mbox         -- Handle of mailbox
 *      void *data              -- Pointer to data to post
 *---------------------------------------------------------------------------*/
void sys_mbox_post( sys_mbox_t *pxMailBox, void *pxMessageToPost )
{
  while( xQueueSendToBack( *pxMailBox, &pxMessageToPost, portMAX_DELAY ) != pdTRUE );
}

/*---------------------------------------------------------------------------*
 * Routine:  sys_mbox_trypost
 *---------------------------------------------------------------------------*
 * Description:
 *      Try to post the "msg" to the mailbox.  Returns immediately with
 *      error if cannot.
 * Inputs:
 *      sys_mbox_t mbox         -- Handle of mailbox
 *      void *msg               -- Pointer to data to post
 * Outputs:
 *      err_t                   -- ERR_OK if message posted, else ERR_MEM
 *                                  if not.
 *---------------------------------------------------------------------------*/
err_t sys_mbox_trypost( sys_mbox_t *pxMailBox, void *pxMessageToPost )
{
  err_t xReturn;
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

  if ( xInsideISR != pdFALSE )
  {
    xReturn = xQueueSendFromISR( *pxMailBox, &pxMessageToPost, &xHigherPriorityTaskWoken );
  }
  else
  {
    xReturn = xQueueSend( *pxMailBox, &pxMessageToPost, ( portTickType ) 0 );
  }

  if( xReturn == pdPASS )
  {
    xReturn = ERR_OK;
  }
  else
  {
    /* The queue was already full. */
    xReturn = ERR_MEM;
    SYS_STATS_INC( mbox.err );
  }

  return xReturn;
}

/*---------------------------------------------------------------------------*
 * Routine:  sys_arch_mbox_tryfetch
 *---------------------------------------------------------------------------*
 * Wait for a new message to arrive in the mbox
 * @param mbox mbox to get a message from
 * @param msg pointer where the message is stored
 * @param timeout maximum time (in milliseconds) to wait for a message
 * @return 0 (milliseconds) if a message has been received
 *         or SYS_MBOX_EMPTY if the mailbox is empty
 *---------------------------------------------------------------------------*/
u32_t sys_arch_mbox_tryfetch(sys_mbox_t *mbox, void **msg)
{
  void *dummyptr;

  if (msg == NULL)
  {
    msg = &dummyptr;
  }

  if (pdTRUE == xQueueReceive(*mbox, &(*msg), ( portTickType )0))
  {
    return 0;
  }
  else
  {
    return SYS_MBOX_EMPTY;
  }
}

int sys_mbox_valid(sys_mbox_t *mbox)
{
  return (*mbox == NULL) ? 0 : 1;
}

void sys_mbox_set_invalid(sys_mbox_t *mbox)
{
  *mbox = NULL;
}

/*---------------------------------------------------------------------------*
 * Routine:  sys_arch_mbox_fetch
 *---------------------------------------------------------------------------*
 * Description:
 *      Blocks the thread until a message arrives in the mailbox, but does
 *      not block the thread longer than "timeout" milliseconds (similar to
 *      the sys_arch_sem_wait() function). The "msg" argument is a result
 *      parameter that is set by the function (i.e., by doing "*msg =
 *      ptr"). The "msg" parameter maybe NULL to indicate that the message
 *      should be dropped.
 *
 *      The return values are the same as for the sys_arch_sem_wait() function:
 *      Number of milliseconds spent waiting or SYS_ARCH_TIMEOUT if there was a
 *      timeout.
 *
 *      Note that a function with a similar name, sys_mbox_fetch(), is
 *      implemented by lwIP.
 * Inputs:
 *      sys_mbox_t mbox         -- Handle of mailbox
 *      void **msg              -- Pointer to pointer to msg received
 *      u32_t timeout           -- Number of milliseconds until timeout
 * Outputs:
 *      u32_t                   -- SYS_ARCH_TIMEOUT if timeout, else number
 *                                  of milliseconds until received.
 *---------------------------------------------------------------------------*/
u32_t sys_arch_mbox_fetch( sys_mbox_t *pxMailBox, void **ppvBuffer, u32_t ulTimeOut )
{
void *pvDummy;
portTickType xStartTime, xEndTime, xElapsed;
unsigned long ulReturn;

  xStartTime = xTaskGetTickCount();

  if( NULL == ppvBuffer )
  {
    ppvBuffer = &pvDummy;
  }

  if( ulTimeOut != 0UL )
  {
    configASSERT( xInsideISR == ( portBASE_TYPE ) 0 );

    if( pdTRUE == xQueueReceive( *pxMailBox, &( *ppvBuffer ), ulTimeOut/ portTICK_RATE_MS ) )
    {
      xEndTime = xTaskGetTickCount();
      xElapsed = ( xEndTime - xStartTime ) * portTICK_RATE_MS;

      ulReturn = xElapsed;
    }
    else
    {
      /* Timed out. */
      *ppvBuffer = NULL;
      ulReturn = SYS_ARCH_TIMEOUT;
    }
  }
  else
  {
    while( pdTRUE != xQueueReceive( *pxMailBox, &( *ppvBuffer ), portMAX_DELAY ) );
    xEndTime = xTaskGetTickCount();
    xElapsed = ( xEndTime - xStartTime ) * portTICK_RATE_MS;

    if( xElapsed == 0UL )
    {
      xElapsed = 1UL;
    }

    ulReturn = xElapsed;
  }

  return ulReturn;
}

/** Create a new mutex
 * @param mutex pointer to the mutex to create
 * @return a new mutex */
err_t sys_mutex_new( sys_mutex_t *pxMutex )
{
  err_t xReturn = ERR_MEM;

  *pxMutex = xSemaphoreCreateMutex();

  if( *pxMutex != NULL )
  {
    xReturn = ERR_OK;
    SYS_STATS_INC_USED( mutex );
  }
  else
  {
    SYS_STATS_INC( mutex.err );
  }

  return xReturn;
}

/** Lock a mutex
 * @param mutex the mutex to lock */
void sys_mutex_lock( sys_mutex_t *pxMutex )
{
  while( xSemaphoreTake( *pxMutex, portMAX_DELAY ) != pdPASS );
}

/** Unlock a mutex
 * @param mutex the mutex to unlock */
void sys_mutex_unlock(sys_mutex_t *pxMutex )
{
  xSemaphoreGive( *pxMutex );
}

/**
 * Initialize the sys module
 */
void sys_init(void)
{
}

u32_t sys_now(void)
{
  return xTaskGetTickCount() / portTICK_PERIOD_MS;
}

/*---------------------------------------------------------------------------*
 * Routine:  sys_thread_new
 *---------------------------------------------------------------------------*
 * Description:
 *      Starts a new thread with priority "prio" that will begin its
 *      execution in the function "thread()". The "arg" argument will be
 *      passed as an argument to the thread() function. The id of the new
 *      thread is returned. Both the id and the priority are system
 *      dependent.
 * Inputs:
 *      char *name              -- Name of thread
 *      void (* thread)(void *arg) -- Pointer to function to run.
 *      void *arg               -- Argument passed into function
 *      int stacksize           -- Required stack amount in bytes
 *      int prio                -- Thread priority
 * Outputs:
 *      sys_thread_t            -- Pointer to per-thread timeouts.
 *---------------------------------------------------------------------------*/
sys_thread_t sys_thread_new( const char *pcName, void( *pxThread )( void *pvParameters ), void *pvArg, int iStackSize, int iPriority )
{
  xTaskHandle xCreatedTask;
  portBASE_TYPE xResult;
  sys_thread_t xReturn;

  xResult = xTaskCreate(pxThread, pcName, (uint16_t) iStackSize, pvArg, iPriority, &xCreatedTask);

  if( xResult == pdPASS )
  {
    xReturn = xCreatedTask;
  }
  else
  {
    xReturn = NULL;
  }

  return xReturn;
}

int sys_sem_valid(sys_sem_t *sem)
{
  return (*sem == NULL) ? 0 : 1;
}

void sys_sem_set_invalid(sys_sem_t *sem)
{
  *sem = NULL;
}

/*---------------------------------------------------------------------------*
 * Routine:  sys_sem_new
 *---------------------------------------------------------------------------*
 * Create a new semaphore
 * @param sem pointer to the semaphore to create
 * @param count initial count of the semaphore
 * @return ERR_OK if successful, another err_t otherwise
 *---------------------------------------------------------------------------*/
err_t sys_sem_new(sys_sem_t *sem, u8_t count)
{
  portENTER_CRITICAL();
  vSemaphoreCreateBinary(*sem);

  if (count == 0) {
    xSemaphoreTake(*sem, 1);
  }
  portEXIT_CRITICAL();

  if(*sem != NULL ){
#if SYS_STATS
    SYS_STATS_INC(sem.used);
#endif
    return ERR_OK;
  } else {
#if SYS_STATS
    SYS_STATS_INC(sem.err);
#endif
    return ERR_MEM;
  }
}

/*---------------------------------------------------------------------------*
 * Routine:  sys_arch_sem_wait
 *---------------------------------------------------------------------------*
 * Wait for a semaphore for the specified timeout
 * @param sem the semaphore to wait for
 * @param timeout timeout in milliseconds to wait (0 = wait forever)
 * @return time (in milliseconds) waited for the semaphore
 *         or SYS_ARCH_TIMEOUT on timeout
 *---------------------------------------------------------------------------*/
u32_t sys_arch_sem_wait(sys_sem_t *sem, u32_t timeout)
{
  portTickType StartTime, EndTime;
  u32_t Elapsed;

  StartTime = xTaskGetTickCount();

  if (timeout != 0)
  {
    if ( xSemaphoreTake( *sem, timeout/portTICK_RATE_MS ) == pdTRUE) // TODO: Define a proper macro to count ms
    {
      // Return time blocked.
      EndTime = xTaskGetTickCount();
      Elapsed = portTICK_RATE_MS*(EndTime - StartTime); // TODO: Define a proper macro to count ms
      if (Elapsed == 0)
      {
        Elapsed = 1*portTICK_RATE_MS;
      }
      return (Elapsed);
    }
    else
    {
      // Timed out
      return SYS_ARCH_TIMEOUT;
    }
  }
  else
  {
    // must block without a timeout
    if (xSemaphoreTake( *sem, portMAX_DELAY ) != pdTRUE)
    {
#if SYS_STATS
      SYS_STATS_INC(sem.err);
#endif /* SYS_STATS */
    }

    // Return time blocked.
    EndTime = xTaskGetTickCount();
    Elapsed = portTICK_RATE_MS*(EndTime - StartTime);// TODO: Define a proper macro to count ms
    if (Elapsed == 0)
    {
      Elapsed = portTICK_RATE_MS*1;
    }

    // return time blocked
    return (Elapsed);
  }
}

/*---------------------------------------------------------------------------*
 * Routine:  sys_sem_signal
 *---------------------------------------------------------------------------*
 * Signals a semaphore
 * @param sem the semaphore to signal
 *---------------------------------------------------------------------------*/
void sys_sem_signal(sys_sem_t * sem)
{
  xSemaphoreGive(*sem);
}

/*---------------------------------------------------------------------------*
 * Routine:  sys_sem_free
 *---------------------------------------------------------------------------*
 * Delete a semaphore
 * @param sem semaphore to delete
 *---------------------------------------------------------------------------*/
void sys_sem_free(sys_sem_t * sem)
{
#if SYS_STATS
  SYS_STATS_DEC(sem.used);
#endif /* SYS_STATS */

  vSemaphoreDelete(*sem);
}

