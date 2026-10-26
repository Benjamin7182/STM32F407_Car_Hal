#include "led.h"

void vLedTask(void *pvParameters){
	while(1){
		HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
		vTaskDelay(pdMS_TO_TICKS(200));
	}
}

void vBuzzerTask(void *pvParameters){
	while(1){
		HAL_GPIO_TogglePin(Buzzer_GPIO_Port,Buzzer_Pin);
		vTaskDelay(pdMS_TO_TICKS(200));
	}
}