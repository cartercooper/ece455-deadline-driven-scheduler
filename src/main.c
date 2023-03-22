#include <stdint.h>
#include <stdio.h>
#include "stm32f4_discovery.h"

#include "stm32f4xx.h"
#include "../FreeRTOS_Source/include/FreeRTOS.h"
#include "../FreeRTOS_Source/include/queue.h"
#include "../FreeRTOS_Source/include/semphr.h"
#include "../FreeRTOS_Source/include/task.h"
#include "../FreeRTOS_Source/include/timers.h"


//TEST BENCH 1
#define TASK1_PERIOD (500)
#define TASK2_PERIOD (500)
#define TASK3_PERIOD (750)
#define TASK1_EXEC (95)
#define TASK2_EXEC (150)
#define TASK3_EXEC (250)

//TEST BENCH 2
//#define TASK1_PERIOD (250)
//#define TASK2_PERIOD (500)
//#define TASK3_PERIOD (750)
//#define TASK1_EXEC (95)
//#define TASK2_EXEC (150)
//#define TASK3_EXEC (250)

//TEST BENCH 3
//#define TASK1_PERIOD (500)
//#define TASK2_PERIOD (500)
//#define TASK3_PERIOD (500)
//#define TASK1_EXEC (100)
//#define TASK2_EXEC (200)
//#define TASK3_EXEC (200)



#define PRIORITY_UNSCHEDULED 0
#define PRIORITY_SCHEDULED 2

#define PRIORITY_SCHEDULER (4)
#define PRIORITY_MONITOR (2)
#define PRIORITY_GENERATOR (3)

#define mainQUEUE_LENGTH 10

#define WAIT_FOR_QUEUE 10

#define orange  0
#define green  	1
#define red  	2
#define blue  	3

#define orange_led	LED3
#define green_led	LED4
#define red_led		LED5
#define blue_led	LED6

typedef enum {PERIODIC,APERIODIC} task_type_t;

typedef struct {
	TaskHandle_t t_handle;
	task_type_t type;
	uint32_t task_id;
	uint32_t release_time;
	uint32_t absolute_deadline;
	uint32_t completion_time;
} dd_task_t;

typedef struct {
	dd_task_t task;
	struct dd_task_list *next_task;
} dd_task_list_t;

static void prvSetupHardware( void );

static void deadlineDrivenScheduler_FTASK( void *pvParameters );
TaskHandle_t handle_deadlineDrivenScheduler_FTASK;

static void monitorTask_FTASK( void *pvParameters );

static void DDT_GEN_1_FTASK( void *pvParameters );
static void DDT_RUN_1_FTASK( void *pvParameters );

static void DDT_GEN_2_FTASK( void *pvParameters );
static void DDT_RUN_2_FTASK( void *pvParameters );

static void DDT_GEN_3_FTASK( void *pvParameters );
static void DDT_RUN_3_FTASK( void *pvParameters );


void create_dd_task
(
	task_type_t type,
	uint32_t absolute_deadline,
	uint32_t task_id
);

const TaskFunction_t TASK_MAP[3] =
{
		DDT_RUN_1_FTASK,
		DDT_RUN_2_FTASK,
		DDT_RUN_3_FTASK

};

const char* PC_NAME_MAP[3] =
{
		"RUN1",
		"RUN2",
		"RUN3"
};

void delete_dd_task(uint32_t task_id);
dd_task_list_t** get_active_dd_task_list(void);
dd_task_list_t** get_complete_dd_task_list(void);
dd_task_list_t** get_overdue_dd_task_list(void);

xQueueHandle activeTaskQueueHandle = 0;
xQueueHandle completedTaskQueueHandle = 0;
xQueueHandle overdueTaskQueueHandle = 0;
xQueueHandle runningTaskQueueHandle = 0;


/*-----------------------------------------------------------*/

int main(void)
{
	prvSetupHardware();

	activeTaskQueueHandle = xQueueCreate( 	mainQUEUE_LENGTH, sizeof( dd_task_t ) );
	vQueueAddToRegistry( activeTaskQueueHandle, "ActiveTasks" );

	completedTaskQueueHandle = xQueueCreate( 	mainQUEUE_LENGTH, sizeof( dd_task_t ) );
	vQueueAddToRegistry( completedTaskQueueHandle, "CompletedTasks" );

	overdueTaskQueueHandle = xQueueCreate( 	mainQUEUE_LENGTH, sizeof( dd_task_t ) );
	vQueueAddToRegistry( overdueTaskQueueHandle, "OverdueTasks" );

	runningTaskQueueHandle = xQueueCreate( 	mainQUEUE_LENGTH, sizeof( dd_task_t ) );
	vQueueAddToRegistry( runningTaskQueueHandle, "RunningTasks" );

	xTaskCreate( deadlineDrivenScheduler_FTASK, "DDS", configMINIMAL_STACK_SIZE, NULL, PRIORITY_SCHEDULER, &handle_deadlineDrivenScheduler_FTASK);
	xTaskCreate( monitorTask_FTASK, "MT", configMINIMAL_STACK_SIZE, NULL, PRIORITY_MONITOR, NULL);

	xTaskCreate( DDT_GEN_1_FTASK, "DDTG1", configMINIMAL_STACK_SIZE, NULL, PRIORITY_GENERATOR, NULL);
	xTaskCreate( DDT_GEN_2_FTASK, "DDTG2", configMINIMAL_STACK_SIZE, NULL, PRIORITY_GENERATOR, NULL);
	xTaskCreate( DDT_GEN_3_FTASK, "DDTG3", configMINIMAL_STACK_SIZE, NULL, PRIORITY_GENERATOR, NULL);

	vTaskStartScheduler();

	return 0;
}

void create_dd_task(task_type_t type, uint32_t absolute_deadline, uint32_t task_id)
{

	TickType_t currentTicks = xTaskGetTickCount();

	TaskHandle_t taskHandleToPass;

	xTaskCreate
	(
		TASK_MAP[task_id-1],
		PC_NAME_MAP[task_id-1],
		configMINIMAL_STACK_SIZE,
		NULL,
		PRIORITY_UNSCHEDULED,
		&(taskHandleToPass)
	);

	dd_task_t taskToCreate =
	{
			.t_handle = taskHandleToPass,
			.type = type,
			.task_id = task_id,
			.release_time = currentTicks,
			.absolute_deadline = currentTicks + absolute_deadline,
			.completion_time = 0
	};

	vTaskSuspend(taskHandleToPass);

	xQueueSend(activeTaskQueueHandle, &taskToCreate, (TickType_t) WAIT_FOR_QUEUE);

	vTaskResume(handle_deadlineDrivenScheduler_FTASK);
}


/*-----------------------------------------------------------*/

static void deadlineDrivenScheduler_FTASK( void *pvParameters )
{
	while(1)
	{

		dd_task_t tasksToRun[mainQUEUE_LENGTH];
		dd_task_t tempTask;
		dd_task_t minTask;

		TickType_t currentTicks = xTaskGetTickCount();

		int i = 0;
		int minIndex = 0;

		while( (xQueueReceive(activeTaskQueueHandle, &tempTask, (TickType_t) WAIT_FOR_QUEUE)) && (i < mainQUEUE_LENGTH))
		{
//			if(tempTask.absolute_deadline < currentTicks)
//			{
//				xQueueSend(overdueTaskQueueHandle, &tempTask, (TickType_t) WAIT_FOR_QUEUE);
//			}
//			else
//			{
				tasksToRun[i] = tempTask;
//			}
			i++;
		}

		minTask = tempTask;

		for(int j = 0; j < i; j++)
		{
			if(tasksToRun[j].absolute_deadline < minTask.absolute_deadline)
			{
				minTask = tasksToRun[j];
				minIndex = j;
			}
		}


		for(int k = 0; k < i; k++)
		{
			if(k != minIndex)
			{
				xQueueSend(activeTaskQueueHandle, &tasksToRun[k], (TickType_t) WAIT_FOR_QUEUE);
			}
		}

		vTaskPrioritySet(minTask.t_handle, PRIORITY_SCHEDULED);

//		xQueueSend(runningTaskQueueHandle, &minTask, (TickType_t) WAIT_FOR_QUEUE);

		vTaskResume(minTask.t_handle);

		vTaskSuspend(NULL);
	}
}

/*-----------------------------------------------------------*/

static void monitorTask_FTASK( void *pvParameters )
{
	while(1)
	{
		int activeTasks = uxQueueMessagesWaiting(activeTaskQueueHandle);
		if(activeTasks > 2)
		{
			vTaskResume(handle_deadlineDrivenScheduler_FTASK);
		}

//		dd_task_t overdueTask;
//		while(xQueueReceive(overdueTaskQueueHandle, &overdueTask, (TickType_t) WAIT_FOR_QUEUE))
//		{
//			printf("overdue and deleted: %d\n", (int)overdueTask.task_id);
//			vTaskDelete(overdueTask.t_handle);
//		}

//		dd_task_t completedTask;
//		while(xQueueReceive(completedTaskQueueHandle, &completedTask, (TickType_t) WAIT_FOR_QUEUE))
//		{
//			printf("completed and deleted: %d, at %d\n", (int)completedTask.task_id, (int)completedTask.completion_time);
//			vTaskDelete(completedTask.t_handle);
//		}

		vTaskDelay(50 / portTICK_RATE_MS);
	}
}

/*-----------------------------------------------------------*/

static void DDT_GEN_1_FTASK( void *pvParameters )
{
	while(1)
	{
		TickType_t currentTicks = xTaskGetTickCount();
		printf("T1 R T: %d\n", (int)currentTicks);
		create_dd_task(PERIODIC, TASK1_PERIOD, (uint32_t) 1);

		vTaskDelay(TASK1_PERIOD);
	}
}

static void DDT_RUN_1_FTASK( void *pvParameters )
{
	while(1)
	{
		STM_EVAL_LEDToggle(green_led);

		TickType_t currentTicks = xTaskGetTickCount();



		while(xTaskGetTickCount() < (currentTicks + TASK1_EXEC));
		currentTicks = xTaskGetTickCount();
		printf("T1 C T: %d\n", (int)currentTicks);
//		vTaskDelay(currentTicks + TASK1_EXEC);
//		dd_task_t tempTask;

//		if(xQueueReceive(runningTaskQueueHandle, &tempTask, (TickType_t) WAIT_FOR_QUEUE))
//		{
//			tempTask.completion_time = xTaskGetTickCount();
//			xQueueSend(completedTaskQueueHandle, &tempTask, (TickType_t) WAIT_FOR_QUEUE);
//		}
//		vTaskSuspend(NULL);
		vTaskResume(handle_deadlineDrivenScheduler_FTASK);
		vTaskDelete(NULL);
	}
}

/*-----------------------------------------------------------*/

static void DDT_GEN_2_FTASK( void *pvParameters )
{
	while(1)
	{
		TickType_t currentTicks = xTaskGetTickCount();
		printf("T2 R T: %d\n", (int)currentTicks);
		create_dd_task(PERIODIC, TASK2_PERIOD, (uint32_t) 2);

		vTaskDelay(TASK2_PERIOD);
	}
}

static void DDT_RUN_2_FTASK( void *pvParameters )
{
	while(1)
	{
		STM_EVAL_LEDToggle(red_led);

		TickType_t currentTicks = xTaskGetTickCount();

//		printf("task2 release: %d\n", (int)currentTicks);

		while(xTaskGetTickCount() < (currentTicks + TASK2_EXEC));
		currentTicks = xTaskGetTickCount();
		printf("T2 C T: %d\n", (int)currentTicks);
//		vTaskDelay(currentTicks + TASK2_EXEC);
//		dd_task_t tempTask;
//
//		if(xQueueReceive(runningTaskQueueHandle, &tempTask, (TickType_t) WAIT_FOR_QUEUE))
//		{
//			tempTask.completion_time = xTaskGetTickCount();
//			xQueueSend(completedTaskQueueHandle, &tempTask, (TickType_t) WAIT_FOR_QUEUE);
//		}
//		vTaskSuspend(NULL);
		vTaskResume(handle_deadlineDrivenScheduler_FTASK);
		vTaskDelete(NULL);
	}
}

/*-----------------------------------------------------------*/

static void DDT_GEN_3_FTASK( void *pvParameters )
{
	while(1)
	{
		TickType_t currentTicks = xTaskGetTickCount();
		printf("T3 R T: %d\n", (int)currentTicks);
		create_dd_task(PERIODIC, TASK3_PERIOD, (uint32_t) 3);

		vTaskDelay(TASK3_PERIOD);
	}
}

static void DDT_RUN_3_FTASK( void *pvParameters )
{
	while(1)
	{
		STM_EVAL_LEDToggle(blue_led);

		TickType_t currentTicks = xTaskGetTickCount();

		while(xTaskGetTickCount() < (currentTicks + TASK3_EXEC));
		currentTicks = xTaskGetTickCount();
		printf("T3 C T: %d\n", (int)currentTicks);
//		vTaskDelay(currentTicks + TASK3_EXEC);
//		dd_task_t tempTask;
//
//		if(xQueueReceive(runningTaskQueueHandle, &tempTask, (TickType_t) WAIT_FOR_QUEUE))
//		{
//			tempTask.completion_time = xTaskGetTickCount();
//			xQueueSend(completedTaskQueueHandle, &tempTask, (TickType_t) WAIT_FOR_QUEUE);
//		}
//		vTaskSuspend(NULL);
		vTaskResume(handle_deadlineDrivenScheduler_FTASK);
		vTaskDelete(NULL);
	}
}

/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook( void )
{
	/* The malloc failed hook is enabled by setting
	configUSE_MALLOC_FAILED_HOOK to 1 in FreeRTOSConfig.h.

	Called if a call to pvPortMalloc() fails because there is insufficient
	free memory available in the FreeRTOS heap.  pvPortMalloc() is called
	internally by FreeRTOS API functions that create tasks, queues, software
	timers, and semaphores.  The size of the FreeRTOS heap is set by the
	configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( xTaskHandle pxTask, signed char *pcTaskName )
{
	( void ) pcTaskName;
	( void ) pxTask;

	/* Run time stack overflow checking is performed if
	configconfigCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected.  pxCurrentTCB can be
	inspected in the debugger if the task name passed into this function is
	corrupt. */
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
volatile size_t xFreeStackSpace;

	/* The idle task hook is enabled by setting configUSE_IDLE_HOOK to 1 in
	FreeRTOSConfig.h.

	This function is called on each cycle of the idle task.  In this case it
	does nothing useful, other than report the amount of FreeRTOS heap that
	remains unallocated. */
	xFreeStackSpace = xPortGetFreeHeapSize();

	if( xFreeStackSpace > 100 )
	{
		/* By now, the kernel has allocated everything it is going to, so
		if there is a lot of heap remaining unallocated then
		the value of configTOTAL_HEAP_SIZE in FreeRTOSConfig.h can be
		reduced accordingly. */
	}
}
/*-----------------------------------------------------------*/

static void prvSetupHardware( void )
{
	STM_EVAL_LEDInit(orange_led);
	STM_EVAL_LEDInit(green_led);
	STM_EVAL_LEDInit(red_led);
	STM_EVAL_LEDInit(blue_led);

	/* Ensure all priority bits are assigned as preemption priority bits.
	http://www.freertos.org/RTOS-Cortex-M3-M4.html */
	NVIC_SetPriorityGrouping( 0 );

	/* TODO: Setup the clocks, etc. here, if they were not configured before
	main() was called. */
}

