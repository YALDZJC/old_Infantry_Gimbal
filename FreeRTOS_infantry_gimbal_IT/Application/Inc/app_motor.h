#ifdef MOTOR_GLOBALS
#define MOTOR_EXTERN
#else
#define MOTOR_EXTERN extern
#endif

enum PID_type_t{
	nothing,
	rotating_speed,
	relative_angle,
	absolute_angle,
	relative_angle_cascade,
	absolute_angle_cascade,
	gyro_angle_cascade,
	CH110_gyro_angle_cascade,
	vision_angle_cascade,
};

struct motor_info_t{
	enum PID_type_t PID_type;
	void* PID_structure_addr;
	uint32_t RecId;
	uint32_t SendId;
	int16_t final_output;
	uint16_t mechanical_angle;
	uint16_t mechanical_angle_previous;
	uint16_t rpm;
	uint16_t actual_torque_current;
	uint8_t  temperature;
	
	bool first_received;
	bool online_reply;
	bool used;
	
	int32_t angle_change_sum;
};

struct rotating_speed_t{
	int16_t target_value;  //0-8550
	int16_t actual_RPM;
	int16_t output;
	int16_t error;
	int16_t error_previous;
	int32_t cumulative_error;
};

struct relative_angle_t{
	int32_t  target_value;  //0-8192
	uint16_t mechanical_angle;
	uint16_t mechanical_angle_previous;
	int16_t  output;
	int32_t  error;
	int32_t  error_previous;
	int32_t cumulative_error;
	int32_t  spindle_angle_change_sum;
};

struct absolute_angle_t{
	uint16_t target_value;  //0-8191
	uint16_t mechanical_angle;
	int16_t  output;
	int16_t  error;
	int16_t  error_previous;
	int32_t  cumulative_error;	
};

struct relative_angle_cascade_t{
	int32_t  primary_target_value;  //8192ÖÆ
	uint16_t mechanical_angle;
	uint16_t mechanical_angle_previous;
	int16_t  primary_output;
	int32_t  primary_error;
	int32_t  primary_error_previous;
	int32_t  primary_cumulative_error;
	
	int16_t  secondary_target_value;
	int16_t  angular_velocity;
	int16_t  secondary_output;
	int32_t  secondary_error;
	int32_t  secondary_error_previous;
	int32_t  secondary_cumulative_error;
	
	int32_t  spindle_angle_change_sum;
	uint8_t  used;
};

struct absolute_angle_cascade_t{
	
	uint16_t primary_target_value;  //0-8192
	uint16_t mechanical_angle;
	int16_t  primary_output;
	int16_t  primary_error;
	int16_t  primary_error_previous;
	int32_t  primary_cumulative_error;

	int16_t  secondary_target_value;
	int16_t  angular_velocity;
	int16_t  secondary_output;
	int16_t  secondary_error;
	int16_t  secondary_error_previous;
	int32_t  secondary_cumulative_error;
};

struct gyro_angle_cascade_t{
	
	float		 primary_target_value;
	int16_t  primary_output;
	float		 primary_error;
	float 	 primary_error_previous;
	float		 primary_cumulative_error;

	int16_t  secondary_target_value;
	int16_t  secondary_output;
	int16_t  secondary_error;
	int16_t  secondary_error_previous;
	int32_t  secondary_cumulative_error;
};

struct vision_angle_cascade_t{
	
	int32_t primary_target_value;
	int16_t primary_output;
	int32_t primary_error;
	int32_t primary_error_previous;
	int32_t primary_cumulative_error;
	
	int16_t secondary_target_value;
	int16_t secondary_output;
	int16_t secondary_error;
	int16_t secondary_error_previous;
	int32_t secondary_cumulative_error;
};

MOTOR_EXTERN struct motor_info_t* motor_info_list[motor_count];
MOTOR_EXTERN struct rotating_speed_t* rotating_speed_list[motor_count];
MOTOR_EXTERN struct relative_angle_t* relative_angle_list[motor_count];
MOTOR_EXTERN struct absolute_angle_t* absolute_angle_list[motor_count];
MOTOR_EXTERN struct relative_angle_cascade_t* relative_angle_cascade_list[motor_count];
MOTOR_EXTERN struct absolute_angle_cascade_t* absolute_angle_cascade_list[motor_count];
MOTOR_EXTERN struct gyro_angle_cascade_t* gyro_angle_cascade_list[motor_count];
MOTOR_EXTERN struct vision_angle_cascade_t* vision_angle_cascade_list[motor_count];

void add_motor( uint32_t RecId, uint32_t SendId );
int8_t request_structure_memory( uint8_t index, enum PID_type_t type );
int8_t release_structure_memory( uint8_t index);
uint16_t absolute_angle_remainder( int32_t value );
void motor_angle_sum_clear( uint32_t RecId );
int32_t get_motor_angle_sum( uint32_t RecId );
