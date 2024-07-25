#define LED_GLOBALS
#include "includes.h"

void led_task( void *pvParameters )
{
	TickType_t xLastWakeTime;
	const TickType_t xPeriod1 = pdMS_TO_TICKS( 250 );
	
	xLastWakeTime = xTaskGetTickCount();
	
	uint8_t state = 0;
	
	for( ;; )
	{
		if( switch_flag == 1 )
		{
			HAL_GPIO_WritePin( GPIOH, GPIO_PIN_10, GPIO_PIN_RESET );
			
			if( error == false )
			{
				HAL_GPIO_WritePin( GPIOH, GPIO_PIN_12, GPIO_PIN_RESET );
				HAL_GPIO_TogglePin( GPIOH, GPIO_PIN_11 );
			}
			
			/* If an error occurs */
			else
			{
				/* Motor error */
				if( motor_error == true )
				{
					state = 1;
					
					HAL_GPIO_WritePin( GPIOH, GPIO_PIN_11, GPIO_PIN_RESET );
					HAL_GPIO_TogglePin( GPIOH, GPIO_PIN_12 );
				}
				
				/* Gyroscope error */
				else if( gyro_error == true )
				{
					state = 2;
					if(state != 2)
					{
						if( HAL_GPIO_ReadPin( GPIOH, GPIO_PIN_11 ) == GPIO_PIN_SET )
							HAL_GPIO_WritePin( GPIOH, GPIO_PIN_11, GPIO_PIN_RESET );
						if( HAL_GPIO_ReadPin( GPIOH, GPIO_PIN_12 ) == GPIO_PIN_SET )
							HAL_GPIO_WritePin( GPIOH, GPIO_PIN_12, GPIO_PIN_RESET );
					}
					
					HAL_GPIO_TogglePin( GPIOH, GPIO_PIN_11 );
					HAL_GPIO_TogglePin( GPIOH, GPIO_PIN_12 );
				}
			}
		}
		else if( switch_flag == 0 )
		{
			HAL_GPIO_TogglePin( GPIOH, GPIO_PIN_10 );
		}
		
		vTaskDelayUntil( &xLastWakeTime, xPeriod1 );
	}
}
