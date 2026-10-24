#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void app_main(void)
{
    printf("--- Proyecto AGV Color Iniciado ---\n");
    
    while (1) {
        printf("Esperando lectura de sensores...\n");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}