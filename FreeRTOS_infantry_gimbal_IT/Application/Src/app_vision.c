#define APP_VISION_GLOBALS
#include "includes.h"

uint8_t bullet_speed = 10;
uint8_t rx_buf[vision_receive_data_size];

//选择直瞄还是反返小
enum vision_mode_enum
{
	armor = 0,
	robot = 1,
};
uint8_t vision_mode = robot;


void vision_recive_task( void *pvParameters )
{
	
//	uint8_t rx_buf[vision_receive_data_size];
	
	for( ;; )
	{
		xQueueReceive( VisionMsgQueue, rx_buf, portMAX_DELAY );

		/* 帧头判断 */
		if (rx_buf[0] == 0x5B && rx_buf[vision_receive_data_size - 1] == 0x5D)
		{
			vision_struct.armor_pitch = (int16_t)((rx_buf[1] - 48) * 1000) + (int16_t)((rx_buf[2] - 48) * 100) + (int16_t)((rx_buf[3] - 48) * 10) + (int16_t)(rx_buf[4] - 48);
			vision_struct.armor_yaw = (int16_t)((rx_buf[6] - 48) * 1000) + (int16_t)((rx_buf[7] - 48) * 100) + (int16_t)((rx_buf[8] - 48) * 10) + (int16_t)(rx_buf[9] - 48);
			vision_struct.robot_pitch = (int16_t)((rx_buf[11] - 48) * 1000) + (int16_t)((rx_buf[12] - 48) * 100) + (int16_t)((rx_buf[13] - 48) * 10) + (int16_t)(rx_buf[14] - 48);
			vision_struct.robot_yaw = (int16_t)((rx_buf[16] - 48) * 1000) + (int16_t)((rx_buf[17] - 48) * 100) + (int16_t)((rx_buf[18] - 48) * 10) + (int16_t)(rx_buf[19] - 48);
			/* 视觉识别位 */
			vision_ready = 1;
			/* 视觉开火位 */
//			vision_fire = rx_buf[21] - 48;
			
			if( (vision_enable == 1/*||windmill_enable ==1*/) && vision_ready == 1 )
			{
				if (vision_mode == armor)
				{
				vision_struct.vision_pitch_increment_angle = vision_struct.armor_pitch;
				vision_struct.vision_yaw_increment_angle = vision_struct.armor_yaw;
				}
				
				if (vision_mode == robot)
				{
				vision_struct.vision_pitch_increment_angle = vision_struct.robot_pitch;
				vision_struct.vision_yaw_increment_angle = vision_struct.robot_yaw;

//				vision_fire_flag = 0;
				}
				
				yaw_target = vision_struct.vision_pitch_increment_angle;
				pitch_target = vision_struct.vision_yaw_increment_angle;
			}
			
		}
	}
}

void vision_send_task( void *pvParameters )
{
	TickType_t xLastWakeTime;
	const TickType_t xPeriod = pdMS_TO_TICKS( 10 );
	
	xLastWakeTime = xTaskGetTickCount();
	
	int16_t pitch_angle = 0;
	int32_t yaw_angle = 0;
	
	for( ;; )
	{
		if( __HAL_UART_GET_FLAG( &vision_uart, UART_FLAG_ORE ) != RESET )
		{
			__HAL_UART_CLEAR_OREFLAG( &vision_uart );
			HAL_UARTEx_ReceiveToIdle_IT( &vision_uart, vision_rx_buf, vision_receive_data_size );
		}
		
		#ifndef infantry_2
		pitch_angle = motor_info_list[1]->angle_change_sum;
		#else
		pitch_angle = motor_info_list[0]->angle_change_sum;
		#endif
		yaw_angle = yaw_cumulative_change_angle * 22.755555f;
		
		yaw_angle = (int16_t)(CH110_data.yaw * 10.0f);
		yaw_angle = yaw_angle + 1800;

		vision_send_buf[0] = 0xFF;
		vision_send_buf[1] = yaw_angle >> 8;
		vision_send_buf[2] = yaw_angle;
		
		uart_msg_send( &vision_uart, vision_send_buf, vision_send_data_size );
		
		vTaskDelayUntil( &xLastWakeTime, xPeriod );
	}
	
	
}

//void vision_recive_task( void *pvParameters )
//{
//	
////	uint8_t rx_buf[vision_receive_data_size];
//	
//	for( ;; )
//	{
//		xQueueReceive( VisionMsgQueue, rx_buf, portMAX_DELAY );

//		/* 帧头判断 */
//		if( rx_buf[0] == 0x39 || rx_buf[1] == 0x39 )
//		{
//			/* pitch轴绝对位置 */
//			vision_struct.vision_pitch_increment_angle = rx_buf[2] << 8 | rx_buf[3];
//			/* yaw轴绝对位置 */
//			vision_struct.vision_yaw_increment_angle = rx_buf[4] << 24 | rx_buf[5] << 16 | rx_buf[6] << 8 | rx_buf[7];
//			/* 视觉识别位 */
//			vision_ready = rx_buf[8];
//			/* 视觉开火位 */
//			vision_fire = rx_buf[9];
//			
//			if( (vision_enable == 1/*||windmill_enable ==1*/) && vision_ready == 1 )
//			{
//				yaw_target = vision_struct.vision_yaw_increment_angle;
//				pitch_target = vision_struct.vision_pitch_increment_angle;
//			}
//			
//		}
//	}
//}

//void vision_send_task( void *pvParameters )
//{
//	TickType_t xLastWakeTime;
//	const TickType_t xPeriod = pdMS_TO_TICKS( 10 );
//	
//	xLastWakeTime = xTaskGetTickCount();
//	
//	int16_t pitch_angle = 0;
//	int32_t yaw_angle = 0;
//	
//	for( ;; )
//	{
//		if( __HAL_UART_GET_FLAG( &vision_uart, UART_FLAG_ORE ) != RESET )
//		{
//			__HAL_UART_CLEAR_OREFLAG( &vision_uart );
//			HAL_UARTEx_ReceiveToIdle_IT( &vision_uart, vision_rx_buf, vision_receive_data_size );
//		}
//		
//		#ifndef infantry_2
//		pitch_angle = motor_info_list[1]->angle_change_sum;
//		#else
//		pitch_angle = motor_info_list[0]->angle_change_sum;
//		#endif
//		yaw_angle = yaw_cumulative_change_angle * 22.755555f;
//		
//		vision_send_buf[0] = 0x39;
//		vision_send_buf[1] = 0x39;
//		vision_send_buf[2] = pitch_angle >> 8;
//		vision_send_buf[3] = pitch_angle;
//		vision_send_buf[4] = yaw_angle >> 24;
//		vision_send_buf[5] = yaw_angle >> 16;
//		vision_send_buf[6] = yaw_angle >> 8;
//		vision_send_buf[7] = yaw_angle;
//		vision_send_buf[8] = bullet_speed;
//		vision_send_buf[10] = rotate_latch_flag;
//		vision_send_buf[11] = 0xff;
////		vision_send_buf[11] = windmill_enable;
//		if( ext_game_robot_status.robot_id == 1 || ext_game_robot_status.robot_id == 2 || ext_game_robot_status.robot_id == 3 || ext_game_robot_status.robot_id == 4 || \
//				ext_game_robot_status.robot_id == 5 || ext_game_robot_status.robot_id == 7 )
//		{
//			vision_send_buf[9] = 0x42;
//		}
//		else if( ext_game_robot_status.robot_id == 101 || ext_game_robot_status.robot_id == 102 || ext_game_robot_status.robot_id == 103 || \
//						 ext_game_robot_status.robot_id == 104 || ext_game_robot_status.robot_id == 105 || ext_game_robot_status.robot_id == 107 )
//		{
//			vision_send_buf[9] = 0x52;
//		}
//		
//		uart_msg_send( &vision_uart, vision_send_buf, vision_send_data_size );
//		
//		vTaskDelayUntil( &xLastWakeTime, xPeriod );
//	}
//	
//	
//}
