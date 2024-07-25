#ifdef REMOTE_GLOBALS
#define REMOTE_EXTERN
#else
#define REMOTE_EXTERN extern
#endif

typedef struct{
	
	struct{
		uint16_t ch0;
		uint16_t ch1;
		uint16_t ch2;
		uint16_t ch3;
		uint8_t s1;
		uint8_t s2;
	}remote;
	
	struct{
		int16_t x_axis;
		int16_t y_axis;
		int16_t z_axis;
		uint8_t press_left;
		uint8_t press_right;
	}mouse;
	
	union{
		uint16_t key_code;
		struct{
			uint16_t W_key:1;//前进
			uint16_t S_key:1;//后退
			uint16_t A_key:1;//左移
			uint16_t D_key:1;//右移
			uint16_t SHIFT_key:1;//超电全速前进
			uint16_t CTRL_key:1;//复位
			uint16_t Q_key:1;//小陀螺
			uint16_t E_key:1;//关弹仓
			uint16_t R_key:1;//开弹仓
			uint16_t F_key:1;//反向
			uint16_t G_key:1;//开摩擦轮
			uint16_t Z_key:1;//减速
			uint16_t X_key:1;//加速
			uint16_t C_key:1;//
			uint16_t V_key:1;//
			uint16_t B_key:1;//关摩擦轮
		}key;
	}KeyBoard;
	
}RecMsg_t;

REMOTE_EXTERN RecMsg_t RecMsg;
REMOTE_EXTERN double speed;
REMOTE_EXTERN uint8_t mode;

void remote_task( void *pvParameters );
int32_t remote_do_something( void );
//int16_t speed_control( void );
void speed_control( int16_t* upper_left, int16_t* lower_left, int16_t* lower_right, int16_t* upper_right );

void Remote_angle_calculation( uint16_t* upper_left, uint16_t* lower_left, uint16_t* lower_right, uint16_t* upper_right	);
