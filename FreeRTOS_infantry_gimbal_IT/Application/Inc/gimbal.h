#ifdef GIMBAL_GLOBALS
#define GIMBAL_EXTERN
#else
#define GIMBAL_EXTERN extern
#endif

GIMBAL_EXTERN uint16_t theta;

/* yaw轴和pitch轴的旋转角度 */
GIMBAL_EXTERN float yaw_target;
GIMBAL_EXTERN float pitch_target;
void gimbal_task( void *pvParameters );
void vResetTimerCallback( TimerHandle_t xResetTimer );
