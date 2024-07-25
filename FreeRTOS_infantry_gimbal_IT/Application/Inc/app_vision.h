#ifdef APP_VISION_GLOBALS
#define APP_VISION_EXTERN
#else
#define APP_VISION_EXTERN extern
#endif

typedef struct{
	int16_t vision_pitch_increment_angle;
	int32_t vision_yaw_increment_angle;
	int16_t armor_pitch;
	int16_t armor_yaw;
	int16_t robot_pitch;
	int16_t robot_yaw;
}vision_struct_t;

APP_VISION_EXTERN uint8_t vision_fire;
APP_VISION_EXTERN uint8_t vision_ready;

APP_VISION_EXTERN uint8_t vision_send_buf[12];

APP_VISION_EXTERN vision_struct_t vision_struct;

/* 视觉调试标识符 */
APP_VISION_EXTERN uint8_t vision_enable , windmill_enable;//视觉启动，能量机关启动

APP_VISION_EXTERN uint8_t bullet_speed ;

void vision_recive_task( void *pvParameters );
void vision_send_task( void *pvParameters );
