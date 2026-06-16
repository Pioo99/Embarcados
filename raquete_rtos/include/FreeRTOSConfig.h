#pragma once

/* ── Kernel ─────────────────────────────────────────────────────── */
#define configUSE_PREEMPTION          1
#define configUSE_IDLE_HOOK           1
#define configUSE_TICK_HOOK           0
#define configCPU_CLOCK_HZ            96000000UL  /* BlackPill @96 MHz */
#define configTICK_RATE_HZ            1000        /* tick de 1 ms */
#define configMAX_PRIORITIES          5
#define configMINIMAL_STACK_SIZE      128
#define configTOTAL_HEAP_SIZE         ( 16 * 1024 )
#define configMAX_TASK_NAME_LEN       10
#define configUSE_16_BIT_TICKS        0
#define configUSE_MUTEXES             1
#define configUSE_COUNTING_SEMAPHORES 1
#define configQUEUE_REGISTRY_SIZE     10
#define configUSE_MALLOC_FAILED_HOOK  1   /* DIAGNÓSTICO */
#define configCHECK_FOR_STACK_OVERFLOW 2  /* DIAGNÓSTICO */

/* ── INCLUDEs de API ────────────────────────────────────────────── */
#define INCLUDE_vTaskDelay             1
#define INCLUDE_vTaskDelayUntil        1
#define INCLUDE_xTaskGetTickCount      1
#define INCLUDE_xTaskGetSchedulerState 1

/* ── Prioridades de interrupção (Cortex-M4, 4 bits) ─────────────── */
#define configPRIO_BITS                              4
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY      15
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5
#define configKERNEL_INTERRUPT_PRIORITY \
    ( configLIBRARY_LOWEST_INTERRUPT_PRIORITY << ( 8 - configPRIO_BITS ) )
#define configMAX_SYSCALL_INTERRUPT_PRIORITY \
    ( configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << ( 8 - configPRIO_BITS ) )

#define configASSERT( x ) if( ( x ) == 0 ) { taskDISABLE_INTERRUPTS(); for( ;; ); }

/* ── Mapeia handlers do port para o vetor CMSIS ─────────────────── */
#define vPortSVCHandler     SVC_Handler
#define xPortPendSVHandler  PendSV_Handler
#define xPortSysTickHandler SysTick_Handler
