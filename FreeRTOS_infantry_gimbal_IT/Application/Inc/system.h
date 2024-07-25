#ifdef SYSTEM_GLOBALS
#define SYSTEM_EXTERN
#else
#define SYSTEM_EXTERN extern
#endif

SYSTEM_EXTERN uint8_t test_flag;

enum left_or_right
{
	left,
	right,
};

SYSTEM_EXTERN TimerHandle_t xStartTimer;
SYSTEM_EXTERN TimerHandle_t xResetTimer;

/* Used to judge whether the motor needs to rotate forward or reverse */
SYSTEM_EXTERN uint16_t motor_0x205_left_value;
SYSTEM_EXTERN uint16_t motor_0x205_right_value;
SYSTEM_EXTERN uint16_t motor_0x206_left_value;
SYSTEM_EXTERN uint16_t motor_0x206_right_value;
SYSTEM_EXTERN uint16_t motor_0x207_left_value;
SYSTEM_EXTERN uint16_t motor_0x207_right_value;
SYSTEM_EXTERN uint16_t motor_0x208_left_value;
SYSTEM_EXTERN uint16_t motor_0x208_right_value;

SYSTEM_EXTERN uint8_t switch_flag;

SYSTEM_EXTERN bool error;

SYSTEM_EXTERN bool motor_error;
SYSTEM_EXTERN bool error_motor[motor_count];

SYSTEM_EXTERN bool gyro_online_reply;
SYSTEM_EXTERN bool gyro_error;

SYSTEM_EXTERN bool remote_online_reply;
SYSTEM_EXTERN bool remote_error;

SYSTEM_EXTERN bool communication_online_reply;
SYSTEM_EXTERN bool communication_error;

void system_init( void );
void motor_init(void);
void get_motor_total_change_angle( uint32_t RecId, int32_t* var );
void real_time_monitoring_task( void *pvParameters );

void append_accumulation_check_sum( uint8_t *pBuffer, uint32_t lenght );
uint8_t verify_accumulation_check_sum( uint8_t *pBuffer, uint32_t lenght );
