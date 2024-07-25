#define SYSTEM_GLOBALS
#include "includes.h"

void vStartTimerCallback( TimerHandle_t xStartTimer )
{
	switch_flag = 1;
	xTaskCreate( real_time_monitoring_task, "Monitor", 512, NULL, 5, NULL );
}

void system_init( void )
{
	xStartTimer = xTimerCreate( "Switch Timer",
												 pdMS_TO_TICKS( 25000 ),
												 pdFALSE,
												 ( void * ) 0,
												 vStartTimerCallback );
												 
	xResetTimer = xTimerCreate( "Reset Timer",
												 pdMS_TO_TICKS( 3000 ),
												 pdFALSE,
												 ( void * ) 1,
												 vResetTimerCallback );
	
	xTimerStart( xStartTimer, 0 );
}

/* Real-time inspection task */
void real_time_monitoring_task( void *pvParameters )
{
	TickType_t xLastWakeTime;
	const TickType_t xPeriod = pdMS_TO_TICKS( 1000 );
	
	xLastWakeTime = xTaskGetTickCount();
	
	for( ;; )
	{
		/* Check whether the motors are online */
		for( uint8_t index = 0; index < motor_count; index++ )
		{
			if( motor_info_list[index]->used == true )
			{
				if( motor_info_list[index]->online_reply == true )
				{
					motor_info_list[index]->online_reply = false;
					
					/* Clear the motor error flag */
					error_motor[index] = false;
				}
				else
				{
					/* Mark error */
					error = true;
					motor_error = true;
					error_motor[index] = true;
				}
			}
		}
		
		/* Check if the gyroscope is online */
		if( gyro_online_reply == true )
		{
			gyro_online_reply = false;
			
			gyro_error = false;
		}
		else
		{
			error = true;
			gyro_error = true;
		}
		
		/* Check if the remote is online */
		if( remote_online_reply == true )
		{
			remote_online_reply = false;
			
			remote_error = false;
		}
		else
		{
			error = true;
			remote_error = true;
		}
		
		/* Real-time detection of whether the error has been eliminated */
		if( error == true )
		{
			
			/* Whether the motor error has been eliminated */
			if( motor_error == true )
			{
				motor_error = false;
				
				for( uint8_t index = 0; index < motor_count; index++ )
				{
					if( motor_info_list[index]->used == true )
					{
						/* If do not receive a real-time response from the motor */
						if( error_motor[index] == true )
						{
							/* Mark the motor error flag */
							motor_error = true;
							
							test_flag = 1;
						}
					}
				}
			}
			
			
		}
		
		if( motor_error == false && gyro_error == false && remote_error == false )
		{
			error = false;
		}
		
		vTaskDelayUntil( &xLastWakeTime, xPeriod );
	}
}

/**
  * @brief  算出列表末尾四个字节前的数据累加值
  * @param  列表首地址
  * @param  长度（字节）
  * @retval 累加值
  * @attention  
  */
int32_t summation( uint8_t *pBuffer, uint32_t lenght )
{
	int32_t result = 0;
	
	for( uint8_t index = 0; index < lenght - 4; index++ )
	{
		result += pBuffer[index];
	}
	
	return result;
}

/**
  * @brief  附加校验值到列表最后四个字节
  * @param  列表首地址
  * @param  长度（字节）
  * @retval void
  * @attention  
  */
void append_accumulation_check_sum( uint8_t *pBuffer, uint32_t lenght )
{
	int32_t result = 0;
	
	result = summation( pBuffer, lenght );
	
	pBuffer[lenght - 4] = result;
	pBuffer[lenght - 3] = result >> 8;
	pBuffer[lenght - 2] = result >> 16;
	pBuffer[lenght - 1] = result >> 24;
}

/**
  * @brief  校验数据是否无误
  * @param  列表首地址
  * @param  长度（字节）
  * @retval true or false
  * @attention  
  */
uint8_t verify_accumulation_check_sum( uint8_t *pBuffer, uint32_t lenght )
{
	int32_t buffer_result;
	int32_t accumulation_result;
	
	buffer_result = pBuffer[lenght - 4] | pBuffer[lenght - 3] << 8 | pBuffer[lenght - 2] << 16 | pBuffer[lenght - 1] << 24;
	
	accumulation_result = summation( pBuffer, lenght );
	
	if( buffer_result == accumulation_result )
	{
		return true;
	}
	else
	{
		return false;
	}
}
