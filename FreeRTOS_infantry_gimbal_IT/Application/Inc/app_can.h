#ifdef APP_CAN_GLOBALS
#define APP_CAN_EXTERN
#else
#define APP_CAN_EXTERN extern
#endif

void can_task( void *pvParameters );
