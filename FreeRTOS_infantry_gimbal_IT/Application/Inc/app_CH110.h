#ifdef APP_CH110_GLOBALS
#define APP_CH110_EXTERN
#else
#define APP_CH110_EXTERN extern
#endif

typedef struct{
	float roll; /* x÷· */
	float pitch;/* y÷· */
	float yaw;	/* z÷· */
	int16_t X_axisAngularVelocity;
	int16_t Y_axisAngularVelocity;
	int16_t Z_axisAngularVelocity;
}CH110_data_t;

APP_CH110_EXTERN CH110_data_t CH110_data;
APP_CH110_EXTERN float roll_cumulative_change_angle;
APP_CH110_EXTERN float pitch_cumulative_change_angle;
APP_CH110_EXTERN float yaw_cumulative_change_angle;

void CH110_task( void *pvParameters );
