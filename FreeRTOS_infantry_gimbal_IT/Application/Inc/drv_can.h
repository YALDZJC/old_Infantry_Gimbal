#ifdef CAN_GLOBALS
#define CAN_EXTERN
#else
#define CAN_EXTERN extern
#endif

CAN_EXTERN uint8_t tx_data_0x200[8];
CAN_EXTERN uint8_t tx_data_0x1FF[8];
CAN_EXTERN uint8_t tx_data_0x2FF[8];

void can_manage_init( void );
void motor_control_send( CAN_HandleTypeDef CanHandle );
void motor_control_send_0x2FF( CAN_HandleTypeDef CanHandle );
