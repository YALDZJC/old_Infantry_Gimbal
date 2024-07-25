#ifdef APP_COM_GLOBALS
#define APP_COM_EXTERN
#else
#define APP_COM_EXTERN extern
#endif

APP_COM_EXTERN uint8_t communication_send_buf[communication_send_data_size];
APP_COM_EXTERN int32_t yaw_motor_angle_sum;
APP_COM_EXTERN uint8_t rotate_latch_flag;


typedef struct{
	uint8_t robot_id;
	uint16_t shooter_id1_17mm_speed_limit;
	uint16_t shooter_id1_17mm_cooling_limit;
}ext_game_robot_status_t;

typedef __packed struct
{
 uint8_t bullet_type;
uint8_t shooter_id;
 uint8_t bullet_freq;
 float bullet_speed;
}ext_shoot_data_t;


APP_COM_EXTERN ext_game_robot_status_t ext_game_robot_status;
APP_COM_EXTERN ext_shoot_data_t ext_shoot_data;

void communication_send_task( void *pvParameters );
void communication_receive_task( void *pvParameters );
