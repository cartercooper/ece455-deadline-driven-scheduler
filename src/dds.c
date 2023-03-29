static void deadlineDrivenScheduler_FTASK(void *pvParameters)
{
    while (1)
    {
        dd_task_t tempTask;
        dd_task_t minTask;

        TickType_t currentTicks = xTaskGetTickCount();

        QueueHandle_t tempQueue = xQueueCreate(mainQUEUE_LENGTH, sizeof(dd_task_t));
        BaseType_t queueStatus;

        // Move tasks from activeTaskQueueHandle to the temporary queue
        while ((queueStatus = xQueueReceive(activeTaskQueueHandle, &tempTask, (TickType_t)WAIT_FOR_QUEUE)) == pdTRUE)
        {
            xQueueSend(tempQueue, &tempTask, (TickType_t)WAIT_FOR_QUEUE);
        }

        // Find the task with the closest deadline
        xQueuePeek(tempQueue, &minTask, (TickType_t)0);

        while (xQueueReceive(tempQueue, &tempTask, (TickType_t)0))
        {
            if (tempTask.absolute_deadline < minTask.absolute_deadline)
            {
                minTask = tempTask;
            }
            else
            {
                xQueueSend(activeTaskQueueHandle, &tempTask, (TickType_t)WAIT_FOR_QUEUE);
            }
        }

        // Cleanup temporary queue
        vQueueDelete(tempQueue);

        // Set priority and resume the task with the closest deadline
        vTaskPrioritySet(minTask.t_handle, PRIORITY_SCHEDULED);
        vTaskResume(minTask.t_handle);

        vTaskSuspend(NULL);
    }
}
