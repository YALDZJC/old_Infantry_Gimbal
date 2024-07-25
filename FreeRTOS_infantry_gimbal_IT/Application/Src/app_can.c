#define APP_CAN_GLOBALS
#include "includes.h"

uint8_t test;

int speed_min = 9000;
int speed_max = 0;

/* statement */
void angle_change_clac( uint8_t index );

void can_task( void *pvParameters )
{
	uint8_t rx_buf[motor_data_size];
	uint32_t StdId;
	
	for( ;; )
	{
		xQueueReceive( CanMsgQueue, rx_buf, portMAX_DELAY );
		
		StdId = rx_buf[0] << 24 | rx_buf[1] << 16 | rx_buf[2] << 8 | rx_buf[3];
		
		for( uint8_t index = 0; index < motor_count; index++ )
		{
			if( motor_info_list[index]->RecId == StdId )
			{
				motor_info_list[index]->mechanical_angle			= rx_buf[4] << 8 | rx_buf[5];
				motor_info_list[index]->rpm										= rx_buf[6] << 8 | rx_buf[7];
				motor_info_list[index]->actual_torque_current	= rx_buf[8] << 8 | rx_buf[9];
				motor_info_list[index]->temperature						= rx_buf[10];
				
				/* Determine whether the motor has sent the first message */
				if( motor_info_list[index]->first_received == false )
				{
					motor_info_list[index]->mechanical_angle_previous = motor_info_list[index]->mechanical_angle;
					
					motor_info_list[index]->first_received = true;
				}
				/* Calculate the cumulative change angle */
				else
				{
					#ifdef enable_angle_sum_clac_0x201
					if( motor_info_list[index]->RecId == 0x201 )
						angle_change_clac( index );
					#endif
					#ifdef enable_angle_sum_clac_0x202
					if( motor_info_list[index]->RecId == 0x202 )
						angle_change_clac( index );
					#endif
					#ifdef enable_angle_sum_clac_0x203
					if( motor_info_list[index]->RecId == 0x203 )
						angle_change_clac( index );
					#endif
					#ifdef enable_angle_sum_clac_0x204
					if( motor_info_list[index]->RecId == 0x204 )
						angle_change_clac( index );
					#endif
					#ifdef enable_angle_sum_clac_0x205
					if( motor_info_list[index]->RecId == 0x205 )
						angle_change_clac( index );
					#endif
					#ifdef enable_angle_sum_clac_0x206
					if( motor_info_list[index]->RecId == 0x206 )
						angle_change_clac( index );
					#endif
					#ifdef enable_angle_sum_clac_0x207
					if( motor_info_list[index]->RecId == 0x207 )
						angle_change_clac( index );
					#endif
					#ifdef enable_angle_sum_clac_0x208
					if( motor_info_list[index]->RecId == 0x208 )
						angle_change_clac( index );
					#endif
					#ifdef enable_angle_sum_clac_0x209
					if( motor_info_list[index]->RecId == 0x209 )
						angle_change_clac( index );
					#endif
					
					break;
				}
			}
		}
		if(motor_info_list[3]->rpm < speed_min)
			speed_min = motor_info_list[3]->rpm;
		if(motor_info_list[3]->rpm > speed_max)
			speed_max = motor_info_list[3]->rpm;
	}
}

/* 用于算出电机相对于上一次所改变的角度 */
void angle_change_clac( uint8_t index )
{
	int16_t res1 = 0, res2 = 0;
	
	if( motor_info_list[index]->mechanical_angle - motor_info_list[index]->mechanical_angle_previous > 0 )
	{
		res1 = motor_info_list[index]->mechanical_angle - motor_info_list[index]->mechanical_angle_previous - 8192;
		res2 = motor_info_list[index]->mechanical_angle - motor_info_list[index]->mechanical_angle_previous;
		if( abs( res1 ) < abs( res2 ) )
			motor_info_list[index]->angle_change_sum += res1;
		else
			motor_info_list[index]->angle_change_sum += res2;
	}
	else
	{
		res1 = motor_info_list[index]->mechanical_angle - motor_info_list[index]->mechanical_angle_previous + 8192;
		res2 = motor_info_list[index]->mechanical_angle - motor_info_list[index]->mechanical_angle_previous;
		if( abs( res1 ) < abs( res2 ) )
			motor_info_list[index]->angle_change_sum += res1;
		else
			motor_info_list[index]->angle_change_sum += res2;
	}
	
	motor_info_list[index]->mechanical_angle_previous = motor_info_list[index]->mechanical_angle;
}
