#ifdef LED_GLOBALS
#define LED_EXTERN
#else
#define LED_EXTERN extern
#endif

void led_task( void *pvParameters );
