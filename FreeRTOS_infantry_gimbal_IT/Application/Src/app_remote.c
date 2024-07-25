#define REMOTE_GLOBALS
#include "includes.h"

double angle = 0;

/* Remote control data processing tasks */
void remote_task( void *pvParameters )
{
	uint8_t rx_buf[remote_data_size];
	
	for( ;; )
	{
		xQueueReceive( RemoteMsgQueue, rx_buf, portMAX_DELAY );
		
		RecMsg.remote.ch0 = ( rx_buf[0]      | rx_buf[1] << 8 )                   & 0x07FF;
		RecMsg.remote.ch1 = ( rx_buf[1] >> 3 | rx_buf[2] << 5 )                   & 0x07FF;
		RecMsg.remote.ch2 = ( rx_buf[2] >> 6 | rx_buf[3] << 2 | rx_buf[4] << 10 ) & 0x07FF;
		RecMsg.remote.ch3 = ( rx_buf[4] >> 1 | rx_buf[5] << 7)                    & 0x07FF;
		
		RecMsg.remote.s1  = ( ( rx_buf[5] >> 4 ) & 0x000C ) >> 2;
		RecMsg.remote.s2  = ( ( rx_buf[5] >> 4 ) & 0x0003 );
		
		RecMsg.mouse.x_axis = ( rx_buf[6]  | rx_buf[7] << 8 );
		RecMsg.mouse.y_axis = ( rx_buf[8]  | rx_buf[9] << 8 );
		RecMsg.mouse.z_axis = ( rx_buf[10] | rx_buf[11] << 8 );
		
		RecMsg.mouse.press_left  = rx_buf[12];
		RecMsg.mouse.press_right = rx_buf[13];
		
		RecMsg.KeyBoard.key_code = ( rx_buf[14] | rx_buf[15] << 8 );
	}
}

int32_t remote_do_something( void )
{
	if( RecMsg.remote.ch0 > 1000 && RecMsg.remote.ch0 < 1048 )
		return (int32_t) 0;
	else
		return ( RecMsg.remote.ch0 - 1024 ) * 1.551515;
}


/**
  * @brief  Calculate the motor speed based on the values of channel 3 and channel 2
  * @param  Target value variable of the upper left motor (id:0x201)
  * @param  Target value variable of the lower left motor (id:0x202)
  * @param  Target value variable of the lower right motor (id:0x203)
  * @param  Target value variable of the upper right motor (id:0x204)
  * @retval void
  * @attention  
  */
void speed_control( int16_t* upper_left, int16_t* lower_left, int16_t* lower_right, int16_t* upper_right )
{
	
	if( mode == translation && ( RecMsg.remote.ch2 < 900 || RecMsg.remote.ch2 > 1148 ) || ( RecMsg.remote.ch3 < 900 || RecMsg.remote.ch3 > 1148 ) )
	{
		int16_t a = RecMsg.remote.ch3 - 1024;
		int16_t b = RecMsg.remote.ch2 - 1024;
		speed = sqrt(a*a + b*b) * 12.954545;
		if( speed > 8550 )
		{
			speed = 8550;
		}
	//		if( RecMsg.remote.ch3 > 1024 )
	//		{
	//			if( motor_info_list[5]->mechanical_angle > motor_0x205_left_value || motor_info_list[5]->mechanical_angle < motor_0x205_right_value )
	//				*upper_left = (int16_t) speed;
	//			else
	//				*upper_left = (int16_t) -speed;
	//		}
	//		else if( RecMsg.remote.ch3 < 1024 )
	//		{
	//			if( motor_info_list[5]->mechanical_angle > motor_0x205_left_value || motor_info_list[5]->mechanical_angle < motor_0x205_right_value )
	//				*upper_left = (int16_t) -speed;
	//			else
	//				*upper_left = (int16_t) speed;
	//		}
	//		else
	//			*upper_left = (int16_t) 0;
	//	}
	//	else
	//		*upper_left = (int16_t) 0;
		*upper_left = (int16_t) speed;
		*lower_left = (int16_t) speed;
		*lower_right = (int16_t) speed;
		*upper_right = (int16_t) speed;
	}
	else if( mode == rotation && ( RecMsg.remote.ch0 < 900 || RecMsg.remote.ch0 > 1148 ) )
	{
		speed = ( RecMsg.remote.ch0 - 1024 ) * 12.954545;
		
		*upper_left = (int16_t) speed;
		*lower_left = (int16_t) speed;
		*lower_right = (int16_t) -speed;
		*upper_right = (int16_t) -speed;
	}
	else if( mode == niubi && ( RecMsg.remote.ch1 < 900 || RecMsg.remote.ch1 > 1148 ) )
	{
		speed = ( RecMsg.remote.ch1 - 1024 ) * 12.954545;
		
		*upper_left = (int16_t) speed;
		*lower_left = (int16_t) speed;
		*lower_right = (int16_t) speed;
		*upper_right = (int16_t) speed;
	}
	else
	{
		*upper_left = (int16_t) 0;
		*lower_left = (int16_t) 0;
		*lower_right = (int16_t) 0;
		*upper_right = (int16_t) 0;
	}
}

///**
//  * @brief  Calculate the motor rotation angle through the values of channel 3 and channel 2
//  * @param  Target value variable of the upper left motor (id:0x205)
//  * @param  Target value variable of the lower left motor (id:0x206)
//  * @param  Target value variable of the lower right motor (id:0x207)
//  * @param  Target value variable of the upper right motor (id:0x208)
//  * @retval void
//  * @attention  
//  */
//void Remote_angle_calculation( uint16_t* upper_left, uint16_t* lower_left, uint16_t* lower_right, uint16_t* upper_right	)
//{
//	int32_t local_upper_left = *upper_left;
//	int32_t local_lower_left = *lower_left;
//	int32_t local_lower_right = *lower_right;
//	int32_t local_upper_right = *upper_right;
//	
////	int32_t local_turn = 0;
//	
//	static int32_t local_upper_left_previous = 0;
//	static int32_t local_lower_left_previous = 0;
//	static int32_t local_lower_right_previous = 0;
//	static int32_t local_upper_right_previous = 0;
//	
//	static uint8_t flag = 1;
//	if( flag )
//	{
//		local_upper_left_previous = local_upper_left;
//		local_lower_left_previous = local_lower_left;
//		local_lower_right_previous = local_lower_right;
//		local_upper_right_previous = local_upper_right;
//		flag = 0;
//	}
//	
//	if( ( RecMsg.remote.ch3 < 900 || RecMsg.remote.ch3 > 1148 ) || ( RecMsg.remote.ch2 < 900 || RecMsg.remote.ch2 > 1148 ) )
//	{
//		angle = ( atan2( (double) (RecMsg.remote.ch2 - 1024), (double) (RecMsg.remote.ch3 - 1024) ) * 180 / PI );
//	
//		local_upper_left = initial_absolute_angle_0x205 + angle * 22.755556f;
//		local_lower_left = initial_absolute_angle_0x206 + angle * 22.755556f;
//		local_lower_right = initial_absolute_angle_0x207 + angle * 22.755556f;
//		local_upper_right = initial_absolute_angle_0x208 + angle * 22.755556f;
//		mode = translation;
//	}
//	else if( RecMsg.remote.ch0 < 900 || RecMsg.remote.ch0 > 1148 )
//	{
//		local_upper_left = initial_absolute_angle_0x205 + 1024;
//		local_lower_left = initial_absolute_angle_0x206 - 1024;
//		local_lower_right = initial_absolute_angle_0x207 + 1024;
//		local_upper_right = initial_absolute_angle_0x208 - 1024;
//		mode = rotation;
//	}
	
	/* test */
//	if( RecMsg.remote.s1 == 3 )
//	{
//		local_upper_left = initial_absolute_angle_0x205 - 1024 + gyro_data.yaw;
//		local_lower_left = initial_absolute_angle_0x206 + 1024 + gyro_data.yaw;
//		local_lower_right = initial_absolute_angle_0x207 + 1024 + gyro_data.yaw;
//		local_upper_right = initial_absolute_angle_0x208 - 1024 + gyro_data.yaw;
//		mode = niubi;
//	}
	
	
	
	
	
//		if( RecMsg.remote.ch0 < 900 || RecMsg.remote.ch0 > 1148 )
//		{
//			local_turn = ( RecMsg.remote.ch0 - 1024 ) * 1.551515f;
//		}
//	
//	if( ( angle < 45 && angle > -45 ) || ( angle < -135 && angle > 135 ) )
//	{
//		local_upper_left = initial_absolute_angle_0x205 + angle * 22.755556 + local_turn;
//		local_lower_left = initial_absolute_angle_0x206 + angle * 22.755556 - local_turn;
//		local_lower_right = initial_absolute_angle_0x207 + angle * 22.755556 - local_turn;
//		local_upper_right = initial_absolute_angle_0x208 + angle * 22.755556 + local_turn;
//	}
//	else if( ( angle > 45 && angle < 135 ) || ( angle > -135 && angle < -45 ) )
//	{
//		local_upper_left = initial_absolute_angle_0x205 + angle * 22.755556 + local_turn;
//		local_lower_left = initial_absolute_angle_0x206 + angle * 22.755556 + local_turn;
//		local_lower_right = initial_absolute_angle_0x207 + angle * 22.755556 - local_turn;
//		local_upper_right = initial_absolute_angle_0x208 + angle * 22.755556 - local_turn;
//	}
	
	/* 这段挪到上面 */
//	local_upper_left = initial_absolute_angle_0x205 + angle * 22.755556f;
//	local_lower_left = initial_absolute_angle_0x206 + angle * 22.755556f;
//	local_lower_right = initial_absolute_angle_0x207 + angle * 22.755556f;
//	local_upper_right = initial_absolute_angle_0x208 + angle * 22.755556f;
	
//	int16_t difference_upper_left = local_upper_left - local_upper_left_previous;
//	if( difference_upper_left < -2048 || difference_upper_left > 2048 )
//		local_upper_left = local_upper_left + 4096;
//	
//	int16_t difference_lower_left = local_lower_left - local_lower_left_previous;
//	if( difference_lower_left < -2048 || difference_lower_left > 2048 )
//		local_lower_left = local_lower_left + 4096;
//	
//	int16_t difference_lower_right = local_lower_right - local_lower_right_previous;
//	if( difference_lower_right < -2048 || difference_lower_right > 2048 )
//		local_lower_right = local_lower_right + 4096;
//	
//	int16_t difference_upper_right = local_upper_right - local_upper_right_previous;
//	if( difference_upper_right < -2048 || difference_upper_right > 2048 )
//		local_upper_right = local_upper_right + 4096;
	
//	if( local_upper_left > 8192 )
//	{
//		local_upper_left = local_upper_left % 8192;
//	}
//	if( local_lower_left > 8192 )
//	{
//		local_lower_left = local_lower_left % 8192;
//	}
//	if( local_lower_right > 8192 )
//	{
//		local_lower_right = local_lower_right % 8192;
//	}
//	if( local_upper_right > 8192 )
//	{
//		local_upper_right = local_upper_right % 8192;
//	}
//	
//		if( local_upper_left < 0 )
//	{
//		local_upper_left = 8192 + ( local_upper_left % 8192 );
//	}
//	if( local_lower_left < 0 )
//	{
//		local_lower_left = 8192 + ( local_lower_left % 8192 );
//	}
//	if( local_lower_right < 0 )
//	{
//		local_lower_right = 8192 + ( local_lower_right % 8192 );
//	}
//	if( local_upper_right < 0 )
//	{
//		local_upper_right = 8192 + ( local_upper_right % 8192 );
//	}
//	
//	*upper_left = (uint16_t) local_upper_left;
//	*lower_left = (uint16_t) local_lower_left;
//	*lower_right = (uint16_t) local_lower_right;
//	*upper_right = (uint16_t) local_upper_right;
//	
//	local_upper_left_previous = local_upper_left;
//	local_lower_left_previous = local_lower_left;
//	local_lower_right_previous = local_lower_right;
//	local_upper_right_previous = local_upper_right;
//}
