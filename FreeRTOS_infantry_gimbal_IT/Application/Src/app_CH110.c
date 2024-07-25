#define APP_CH110_GLOBALS
#include "includes.h"

void CH110_task( void *pvParameters )
{
	uint8_t rx_buf[CH110_data_size];
	uint32_t StdId;
	
	uint8_t first_received = false;
	float roll_offset = 0;
	float pitch_offset = 0;
	float yaw_offset = 0;
	float roll_previous = 0;
	float pitch_previous = 0;
	float yaw_previous = 0;
	float roll_res1 = 0, roll_res2 = 0;
	float pitch_res1 = 0, pitch_res2 = 0;
	float yaw_res1 = 0, yaw_res2 = 0;
	float roll_change = 0, pitch_change = 0, yaw_change = 0;
	
	for( ;; )
	{
		xQueueReceive( CH110MsgQueue, rx_buf, portMAX_DELAY );
		
		StdId = rx_buf[0] << 24 | rx_buf[1] << 16 | rx_buf[2] << 8 | rx_buf[3];
		
		/* 欧拉角 */
		if( StdId == 0x388 )
		{
			CH110_data.roll		= ( (int16_t)(rx_buf[5] << 8 | rx_buf[4]) / 100.0f ) - roll_offset;
			CH110_data.pitch	= ( (int16_t)(rx_buf[7] << 8 | rx_buf[6]) / 100.0f ) - pitch_offset;
			CH110_data.yaw 		= ( (int16_t)(rx_buf[9] << 8 | rx_buf[8]) / 100.0f ) - yaw_offset;
			
			/* 第一次接收数据 */
			if( first_received == false )
			{
				roll_offset = CH110_data.roll;
				pitch_offset = CH110_data.pitch;
				yaw_offset = CH110_data.yaw;
				
				first_received = true;
			}
			else
			{
				/* Calculate the actual change angle of the roll axis */
				if( CH110_data.roll - roll_previous > 0 )
				{
					roll_res1 = CH110_data.roll - roll_previous - 360;
					roll_res2 = CH110_data.roll - roll_previous;
					if( abs( roll_res1 ) < abs( roll_res2 ) )
						roll_change = roll_res1;
					else
						roll_change = roll_res2;
				}
				else
				{
					roll_res1 = CH110_data.roll - roll_previous + 360;
					roll_res2 = CH110_data.roll - roll_previous;
					if( abs( roll_res1 ) < abs( roll_res2 ) )
						roll_change = roll_res1;
					else
						roll_change = roll_res2;
				}
				
				/* Calculate the actual change angle of the pitch axis */
				if( CH110_data.pitch - pitch_previous > 0 )
				{
					pitch_res1 = CH110_data.pitch - pitch_previous - 360;
					pitch_res2 = CH110_data.pitch - pitch_previous;
					if( abs( pitch_res1 ) < abs( pitch_res2 ) )
						pitch_change = pitch_res1;
					else
						pitch_change = pitch_res2;
				}
				else
				{
					pitch_res1 = CH110_data.pitch - pitch_previous + 360;
					pitch_res2 = CH110_data.pitch - pitch_previous;
					if( abs( pitch_res1 ) < abs( pitch_res2 ) )
						pitch_change = pitch_res1;
					else
						pitch_change = pitch_res2;
				}
				
				/* Calculate the actual change angle of the yaw axis */
				if( CH110_data.yaw - yaw_previous > 0 )
				{
					yaw_res1 = CH110_data.yaw - yaw_previous - 360;
					yaw_res2 = CH110_data.yaw - yaw_previous;
					if( abs( yaw_res1 ) < abs( yaw_res2 ) )
						yaw_change = yaw_res1;
					else
						yaw_change = yaw_res2;
				}
				else
				{
					yaw_res1 = CH110_data.yaw - yaw_previous + 360;
					yaw_res2 = CH110_data.yaw - yaw_previous;
					if( abs( yaw_res1 ) < abs( yaw_res2 ) )
						yaw_change = yaw_res1;
					else
						yaw_change = yaw_res2;
				}
				
				/* Calculate the cumulative change angle */
				roll_cumulative_change_angle += roll_change;
				pitch_cumulative_change_angle += pitch_change;
				yaw_cumulative_change_angle += yaw_change;
				
				/* Record this Euler angle for the next calculation */
				roll_previous = CH110_data.roll;
				pitch_previous = CH110_data.pitch;
				yaw_previous = CH110_data.yaw;
			}
		}
		
		/* 角速度 */
		else if( StdId == 0x288 )
		{
			CH110_data.X_axisAngularVelocity = ( int16_t ) ( rx_buf[5] << 8 | rx_buf[4] );
			CH110_data.Y_axisAngularVelocity = ( int16_t ) ( rx_buf[7] << 8 | rx_buf[6] );
			CH110_data.Z_axisAngularVelocity = ( int16_t ) ( rx_buf[9] << 8 | rx_buf[8] );
		}
	}
}

