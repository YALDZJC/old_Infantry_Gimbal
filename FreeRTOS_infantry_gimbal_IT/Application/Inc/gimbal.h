#ifdef GIMBAL_GLOBALS
#define GIMBAL_EXTERN
#else
#define GIMBAL_EXTERN extern
#endif

GIMBAL_EXTERN uint16_t theta;

/* yaw���pitch�����ת�Ƕ� */
GIMBAL_EXTERN float yaw_target;
GIMBAL_EXTERN float pitch_target;
void gimbal_task( void *pvParameters );
void vResetTimerCallback( TimerHandle_t xResetTimer );
