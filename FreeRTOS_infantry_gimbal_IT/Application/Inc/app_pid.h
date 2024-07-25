#ifdef PID_GLOBALS
#define PID_EXTERN
#else
#define PID_EXTERN extern
#endif

enum direction{
	Both,
	CW,
	CCW,
};

enum euler_axis{
	roll,
	pitch,
	yaw,
};

enum speed_loop_data_source{
	gyro,
	motor,
};

typedef struct{
	float kp;
	float ki;
	float kd;
	int32_t cumulative_err_max;
	int16_t output_max;
}PID_param_struct_t;

typedef PID_param_struct_t primary_PID_param_struct_t;
typedef PID_param_struct_t secondary_PID_param_struct_t;

int8_t rotating_speed_calculation( uint32_t RecId, int16_t target_value, int16_t target_value_max, PID_param_struct_t structure );
int8_t relative_angle_calculation( uint32_t RecId, int32_t target_value, int32_t CW_angle_max, int32_t CCW_angle_max, PID_param_struct_t structure );
int8_t absolute_angle_calculation( uint32_t RecId, uint16_t target_value, enum direction direct, PID_param_struct_t structure );
int8_t relative_angle_cascade_calculation( uint32_t RecId, 
																						int32_t target_value, 
																						int32_t CW_angle_max, 
																						int32_t CCW_angle_max, 
												 primary_PID_param_struct_t primary_structure, 
											 secondary_PID_param_struct_t secondary_structure );
int8_t absolute_angle_cascade_calculation( uint32_t RecId, 
																					 uint16_t target_value, 
																							 enum direction direct, 
												 primary_PID_param_struct_t primary_structure, 
											 secondary_PID_param_struct_t secondary_structure );
int8_t gyro_angle_cascade_calculation( uint32_t RecId, 
																			 int32_t target_value, 
																			 enum euler_axis angular_velocity_axis, 
																			 enum speed_loop_data_source speed_src, 
																			 bool output_inversion,
																			 primary_PID_param_struct_t primary_structure, 
																			 secondary_PID_param_struct_t secondary_structure );
int8_t CH110_gyro_angle_cascade_calculation( uint32_t RecId, 
																						int32_t target_value, 
																						enum euler_axis angular_velocity_axis, 
																						enum speed_loop_data_source speed_src, 
																						bool output_inversion, 
																						primary_PID_param_struct_t primary_structure, 
																						secondary_PID_param_struct_t secondary_structure );
int8_t vision_angle_cascade_caculation( uint32_t RecId,
																				int32_t target_value,
																				enum euler_axis angular_velocity_axis,
																				enum speed_loop_data_source speed_src,
																				bool output_inversion,
																				primary_PID_param_struct_t primary_structure,
																				secondary_PID_param_struct_t secondary_structure );
