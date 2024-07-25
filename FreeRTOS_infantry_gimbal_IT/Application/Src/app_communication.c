#define APP_COM_GLOBALS
#include "includes.h"

/* 通信任务 */
void communication_send_task( void *pvParameters )
{
	
	for( ;; )
	{	
		/* 遥控部分 */
		communication_send_buf[0] = RecMsg.remote.ch0;
		communication_send_buf[1] = RecMsg.remote.ch0 >> 8;
		communication_send_buf[2] = RecMsg.remote.ch1;
		communication_send_buf[3] = RecMsg.remote.ch1 >> 8;
		communication_send_buf[4] = RecMsg.remote.ch2;
		communication_send_buf[5] = RecMsg.remote.ch2 >> 8;
		communication_send_buf[6] = RecMsg.remote.ch3;
		communication_send_buf[7] = RecMsg.remote.ch3 >> 8;
		communication_send_buf[8] = RecMsg.remote.s1;
		communication_send_buf[9] = RecMsg.remote.s2;
		
		/* 陀螺仪yaw轴部分 */
		communication_send_buf[10] = ((int32_t)yaw_cumulative_change_angle);
		communication_send_buf[11] = ((int32_t)yaw_cumulative_change_angle) >> 8;
		communication_send_buf[12] = ((int32_t)yaw_cumulative_change_angle) >> 16;
		communication_send_buf[13] = ((int32_t)yaw_cumulative_change_angle) >> 24;
		
		/* 云台与底盘夹角部分 */
		communication_send_buf[14] = theta;
		communication_send_buf[15] = theta >> 8;
		
		/* 键鼠部分 */
		communication_send_buf[16] = RecMsg.mouse.x_axis;
		communication_send_buf[17] = RecMsg.mouse.x_axis >> 8;
		communication_send_buf[18] = RecMsg.mouse.y_axis;
		communication_send_buf[19] = RecMsg.mouse.y_axis >> 8;
		communication_send_buf[20] = RecMsg.mouse.press_left;
		communication_send_buf[21] = RecMsg.mouse.press_right;
		communication_send_buf[22] = RecMsg.KeyBoard.key_code; 
		communication_send_buf[23] = RecMsg.KeyBoard.key_code >> 8;
		
		/* 附加校验值 */
		append_accumulation_check_sum( communication_send_buf, communication_send_data_size );
		
		/* 发送数据到下板 */
		uart_msg_send( &communication_uart, communication_send_buf, communication_send_data_size );
		
		vTaskDelay( pdMS_TO_TICKS( 2 ) );
	}
}

void communication_receive_task( void *pvParameters )
{
	uint8_t rx_buf[communication_receive_data_size];
	
	for( ;; )
	{
		xQueueReceive( CommunicationMsgQueue, rx_buf, portMAX_DELAY );
		
		if( verify_accumulation_check_sum( rx_buf, communication_receive_data_size ) )
		{
			ext_power_heat_data.shooter_id1_17mm_cooling_heat = rx_buf[1] << 8 | rx_buf[0];
			ext_game_robot_status.robot_id = rx_buf[3] << 8 | rx_buf[2];
			ext_game_robot_status.shooter_id1_17mm_speed_limit = rx_buf[5] << 8 | rx_buf[4]; /* 17mm枪管1射速上限 */
			ext_game_robot_status.shooter_id1_17mm_cooling_limit = rx_buf[7] | rx_buf[6]; /* 17mm枪管1热量上限 */
			rotate_latch_flag =rx_buf[8] ;
//			ext_shoot_data.bullet_speed = (rx_buf[9] | rx_buf[10] << 8) / 1000;
		}
	}
}


