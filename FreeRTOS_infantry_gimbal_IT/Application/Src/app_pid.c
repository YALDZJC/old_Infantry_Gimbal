#define PID_GLOBALS
#include "includes.h"

int8_t RecId_find( uint32_t RecId )
{
	for( uint8_t index = 0; index < motor_count; index++ )
	{
		if( motor_info_list[index]->RecId == RecId )
		{
			return index;
		}
	}
	return INDEX_ERROR;
}

int16_t limit_int16_t( int16_t amt, int16_t limit )
{
	if( amt < -limit )
		return -limit;
	else if( amt > limit )
		return limit;
	else
		return amt;
}

int32_t cumulative_limit_int16_t( int32_t amt, int16_t add, int32_t limit )
{
	if( (amt + add) < -limit )
		return -limit;
	else if( (amt+add) > limit )
		return limit;
	return amt + add;
}

int32_t cumulative_limit_int32_t( int32_t amt, int32_t add, int32_t limit )
{
	if( (amt + add) < -limit )
		return -limit;
	else if( (amt+add) > limit )
		return limit;
	return amt + add;
}

int16_t output_limit( int32_t amt, int16_t limit )
{
	if( amt < -limit )
		return -limit;
	else if( amt > limit )
		return limit;
	else
		return amt;
}

int8_t rotating_speed_calculation( uint32_t RecId, int16_t target_value, int16_t target_value_max, PID_param_struct_t structure )
{
	struct rotating_speed_t* pStruture = NULL;
	int32_t local_pid_result = 0;
	
	int8_t index = RecId_find( RecId );
	
	/* Determine whether the specified RecId exists */
	if( index != -1 )
	{
		/* Determine whether the PID type is the specified type */
		if( motor_info_list[index]->PID_type != rotating_speed )
		{
			/* Release the memory space of the original PID type structure */
			release_structure_memory( index );
			
			/* Apply for the memory space of the new PID type structure */
			request_structure_memory( index, rotating_speed );
		}		
	}
	
	/* The specified RecId does not exist */
	else
	{
		return INDEX_ERROR;
	}
	
	/* Get the PID structure address of this motor */
	pStruture = motor_info_list[index]->PID_structure_addr;
	
	/* Store the target value in the structure */
	pStruture->target_value = target_value;
	
	/* Target limit */
	pStruture->target_value = limit_int16_t( pStruture->target_value, target_value_max );
	
	/* Convert the speed value returned by the motor into positive and negative values */
	if( motor_info_list[index]->rpm < 32767 )
	{
		pStruture->actual_RPM = motor_info_list[index]->rpm;
	}
	else if( motor_info_list[index]->rpm > 32767 )
	{
		pStruture->actual_RPM = motor_info_list[index]->rpm - 65535;
	}
	
	/* Calculation error */
	pStruture->error = pStruture->target_value - pStruture->actual_RPM;
	
	/* Integral limit */
	if( structure.cumulative_err_max != 0 )
	{
		pStruture->cumulative_error = cumulative_limit_int16_t( pStruture->cumulative_error, pStruture->error, structure.cumulative_err_max );
	}
	
	/* Calculate the result of this PID operation */
	local_pid_result = ( pStruture->error * structure.kp ) + \
										 ( pStruture->cumulative_error * structure.ki ) + \
									 ( ( pStruture->error - pStruture->error_previous ) * structure.kd );
	
	/* Update error_previous value */
	pStruture->error_previous = pStruture->error;
	
	/* Output limiter */
	pStruture->output = output_limit( local_pid_result, structure.output_max );
	
	/* Put the output value into the motor information structure to wait to be sent */
	motor_info_list[index]->final_output = pStruture->output;
	
	return LIB_OK;
}






int8_t relative_angle_calculation( uint32_t RecId, int32_t target_value, int32_t CW_angle_max, int32_t CCW_angle_max, PID_param_struct_t structure )
{
	struct relative_angle_t* pStruture = NULL;
	int32_t local_pid_result = 0;
	int32_t res1 = 0, res2 = 0;
	int32_t mechanical_angle_change = 0;
	int32_t real_mechanical_angle_change = 0;
	
	int8_t index = RecId_find( RecId );
	
	/* Determine whether the specified RecId exists */
	if( index != INDEX_ERROR )
	{
		/* Determine whether the PID type is the specified type */
		if( motor_info_list[index]->PID_type != relative_angle )
		{
			/* Release the memory space of the original PID type structure */
			release_structure_memory( index );
			
			/* Apply for the memory space of the new PID type structure */
			request_structure_memory( index, relative_angle );
		}		
	}
	
	/* The specified RecId does not exist */
	else
	{
		return INDEX_ERROR;
	}
	
	/* Get the PID structure address of this motor */
	pStruture = motor_info_list[index]->PID_structure_addr;
	
	/* Store the target value in the structure */
	pStruture->target_value = target_value;
	
	/* Store the mechanical angle value into the structure */
	pStruture->mechanical_angle = motor_info_list[index]->mechanical_angle;
	
	/* Calculate the angle change value */
	mechanical_angle_change = pStruture->mechanical_angle - pStruture->mechanical_angle_previous;
	
	if( mechanical_angle_change > 0 )
	{
		res1 = mechanical_angle_change - 8192;
		res2 = mechanical_angle_change;
	}
	else
	{
		res1 = mechanical_angle_change + 8192;
		res2 = mechanical_angle_change;
	}
	
	if( abs( res1 ) < abs( res2 ) )
	{
		real_mechanical_angle_change = res1;
	}
	else
	{
		real_mechanical_angle_change = res2;
	}
	
	/* Calculate the cumulative change angle */
	pStruture->spindle_angle_change_sum += real_mechanical_angle_change;
	
	/* Update the last angle value */
	pStruture->mechanical_angle_previous = pStruture->mechanical_angle;
	
	/* Determine whether you need to limit the angle */
	if( CW_angle_max || CCW_angle_max )
	{
		if( pStruture->target_value > CW_angle_max )
		{
			pStruture->target_value = CW_angle_max;
		}
		else if( pStruture->target_value < -CCW_angle_max )
		{
			pStruture->target_value = -CCW_angle_max;
		}
	}
	
	/* Calculation error */
	pStruture->error = pStruture->target_value - pStruture->spindle_angle_change_sum;
	
	/* Integral limit */
	if( structure.cumulative_err_max != 0 )
	{
		pStruture->cumulative_error = cumulative_limit_int32_t( pStruture->cumulative_error, pStruture->error, structure.cumulative_err_max );
	}
	
	/* Calculate the result of this PID operation */
	local_pid_result = ( pStruture->error * structure.kp ) + \
										 ( pStruture->cumulative_error * structure.ki ) + \
									 ( ( pStruture->error - pStruture->error_previous ) * structure.kd );
	
	/* Update error_previous value */
	pStruture->error_previous = pStruture->error;
	
	/* Output limiter */
	pStruture->output = output_limit( local_pid_result, structure.output_max );
	
	/* Put the output value into the motor information structure to wait to be sent */
	motor_info_list[index]->final_output = pStruture->output;	
	
	return LIB_OK;
}









int8_t absolute_angle_calculation( uint32_t RecId, uint16_t target_value, enum direction direct, PID_param_struct_t structure )
{
	struct absolute_angle_t* pStruture = NULL;
	int32_t local_pid_result = 0;
	uint16_t big_value = 0;
	uint16_t small_value = 0;
	
	int8_t index = RecId_find( RecId );
	
	/* Determine whether the specified RecId exists */
	if( index != INDEX_ERROR )
	{
		/* Determine whether the PID type is the specified type */
		if( motor_info_list[index]->PID_type != absolute_angle )
		{
			/* Release the memory space of the original PID type structure */
			release_structure_memory( index );
			
			/* Apply for the memory space of the new PID type structure */
			request_structure_memory( index, absolute_angle );
		}		
	}
	
	/* The specified RecId does not exist */
	else
	{
		return INDEX_ERROR;
	}
	
	/* Get the PID structure address of this motor */
	pStruture = motor_info_list[index]->PID_structure_addr;
	
	/* Determine whether the value exceeds the controllable range */
	if( target_value > 8192 )
	{
		return NUMERICAL_ERROR;
	}
	else
	{
		/* Store the target value in the structure */
		pStruture->target_value = target_value;
	}
	
	/* Store the mechanical angle value into the structure */
	pStruture->mechanical_angle = motor_info_list[index]->mechanical_angle;
	
	switch( direct )
	{
		/* The motor turns to the target position in the shortest distance */
		case Both:
			if( pStruture->target_value < pStruture->mechanical_angle )
			{
				small_value = pStruture->target_value;
				big_value = pStruture->mechanical_angle;
			}
			else
			{
				small_value = pStruture->mechanical_angle;
				big_value = pStruture->target_value;
			}
			
			if( big_value - small_value < 8192 - abs( small_value - big_value ) )
			{
				if( pStruture->target_value < pStruture->mechanical_angle )
				{
					pStruture->error = -( big_value - small_value );
				}
				else
				{
					pStruture->error = big_value - small_value;
				}
			}
			else
			{
				if( pStruture->target_value < pStruture->mechanical_angle )
				{
					pStruture->error = 8192 - abs( small_value - big_value );
				}
				else
				{
					pStruture->error = -( 8192 - abs( small_value - big_value ) );
				}
			}
			break;
			
		/* The motor turns to the target position in a clockwise direction */
		case CW:
			if( pStruture->target_value < pStruture->mechanical_angle )
			{
				small_value = pStruture->target_value;
				big_value = pStruture->mechanical_angle;
			}
			else
			{
				small_value = pStruture->mechanical_angle;
				big_value = pStruture->target_value;
			}
			
			if( pStruture->target_value < pStruture->mechanical_angle )
			{
				pStruture->error = 8192 - abs( small_value - big_value );
			}
			else
			{
				pStruture->error = big_value - small_value;
			}
			break;
			
		/* The motor turns to the target position in a counterclockwise direction */
		case CCW:
			if( pStruture->target_value < pStruture->mechanical_angle )
			{
				small_value = pStruture->target_value;
				big_value = pStruture->mechanical_angle;
			}
			else
			{
				small_value = pStruture->mechanical_angle;
				big_value = pStruture->target_value;
			}
			
			if( pStruture->target_value < pStruture->mechanical_angle )
			{
				pStruture->error = -( big_value - small_value );
			}
			else
			{
				pStruture->error = -( 8192 - abs( small_value - big_value ) );
			}
			break;
			
		default:
			return DIRECT_ERROR;
	}
	
	/* Integral limit */
	if( structure.cumulative_err_max != 0 )
	{
		pStruture->cumulative_error = cumulative_limit_int16_t( pStruture->cumulative_error, pStruture->error, structure.cumulative_err_max );
	}
	
	/* Calculate the result of this PID operation */
	local_pid_result = ( pStruture->error * structure.kp ) + \
										 ( pStruture->cumulative_error * structure.ki ) + \
									 ( ( pStruture->error - pStruture->error_previous ) * structure.kd );
	
	/* Update error_previous value */
	pStruture->error_previous = pStruture->error;
	
	/* Output limiter */
	pStruture->output = output_limit( local_pid_result, structure.output_max );
	
	/* Put the output value into the motor information structure to wait to be sent */
	motor_info_list[index]->final_output = pStruture->output;
	
	return LIB_OK;
}

int8_t absolute_angle_cascade_calculation( uint32_t RecId, 
																					 uint16_t target_value, 
																							 enum direction direct, 
												 primary_PID_param_struct_t primary_structure, 
											 secondary_PID_param_struct_t secondary_structure )
{
	struct absolute_angle_cascade_t* pStruture = NULL;
	int32_t primary_pid_result = 0;
	int32_t secondary_pid_result = 0;
	uint16_t big_value = 0;
	uint16_t small_value = 0;
	
	int8_t index = RecId_find( RecId );
	
	/* Determine whether the specified RecId exists */
	if( index != INDEX_ERROR )
	{
		/* Determine whether the PID type is the specified type */
		if( motor_info_list[index]->PID_type != absolute_angle_cascade )
		{
			/* Release the memory space of the original PID type structure */
			release_structure_memory( index );
			
			/* Apply for the memory space of the new PID type structure */
			request_structure_memory( index, absolute_angle_cascade );
		}		
	}
	
	/* The specified RecId does not exist */
	else
	{
		return INDEX_ERROR;
	}
	
	/* Get the PID structure address of this motor */
	pStruture = motor_info_list[index]->PID_structure_addr;
	
	/* Determine whether the value exceeds the controllable range */
	if( target_value > 8192 )
	{
		return NUMERICAL_ERROR;
	}
	else
	{
		/* Store the target value in the structure */
		pStruture->primary_target_value = target_value;
	}
	
	/* Store the mechanical angle value into the structure */
	pStruture->mechanical_angle = motor_info_list[index]->mechanical_angle;
	
	switch( direct )
	{
		/* The motor turns to the target position in the shortest distance */
		case Both:
			if( pStruture->primary_target_value < pStruture->mechanical_angle )
			{
				small_value = pStruture->primary_target_value;
				big_value = pStruture->mechanical_angle;
			}
			else
			{
				small_value = pStruture->mechanical_angle;
				big_value = pStruture->primary_target_value;
			}
			
			if( big_value - small_value < 8192 - abs( small_value - big_value ) )
			{
				if( pStruture->primary_target_value < pStruture->mechanical_angle )
				{
					pStruture->primary_error = -( big_value - small_value );
				}
				else
				{
					pStruture->primary_error = big_value - small_value;
				}
			}
			else
			{
				if( pStruture->primary_target_value < pStruture->mechanical_angle )
				{
					pStruture->primary_error = 8192 - abs( small_value - big_value );
				}
				else
				{
					pStruture->primary_error = -( 8192 - abs( small_value - big_value ) );
				}
			}
			break;
			
		/* The motor turns to the target position in a clockwise direction */
		case CW:
			if( pStruture->primary_target_value < pStruture->mechanical_angle )
			{
				small_value = pStruture->primary_target_value;
				big_value = pStruture->mechanical_angle;
			}
			else
			{
				small_value = pStruture->mechanical_angle;
				big_value = pStruture->primary_target_value;
			}
			
			if( pStruture->primary_target_value < pStruture->mechanical_angle )
			{
				pStruture->primary_error = 8192 - abs( small_value - big_value );
			}
			else
			{
				pStruture->primary_error = big_value - small_value;
			}
			break;
			
		/* The motor turns to the target position in a counterclockwise direction */
		case CCW:
			if( pStruture->primary_target_value < pStruture->mechanical_angle )
			{
				small_value = pStruture->primary_target_value;
				big_value = pStruture->mechanical_angle;
			}
			else
			{
				small_value = pStruture->mechanical_angle;
				big_value = pStruture->primary_target_value;
			}
			
			if( pStruture->primary_target_value < pStruture->mechanical_angle )
			{
				pStruture->primary_error = -( big_value - small_value );
			}
			else
			{
				pStruture->primary_error = -( 8192 - abs( small_value - big_value ) );
			}
			break;
			
		default:
			return DIRECT_ERROR;
	}
	
	/* Integral limit */
	if( primary_structure.cumulative_err_max != 0 )
	{
		pStruture->primary_cumulative_error = cumulative_limit_int16_t( pStruture->primary_cumulative_error, pStruture->primary_error, primary_structure.cumulative_err_max );
	}
	
	/* Calculate the result of this PID operation */
	primary_pid_result = ( pStruture->primary_error * primary_structure.kp ) + \
											 ( pStruture->primary_cumulative_error * primary_structure.ki ) + \
										 ( ( pStruture->primary_error - pStruture->primary_error_previous ) * primary_structure.kd );	
	
	/* Update error_previous value */
	pStruture->primary_error_previous = pStruture->primary_error;
	
	/* Output limiter */
	pStruture->primary_output = output_limit( primary_pid_result, primary_structure.output_max );
	
	/*******************************************primary calculation end*************************************************/
	
	pStruture->secondary_target_value = pStruture->primary_output;
	
	/* Convert the speed value returned by the motor into positive and negative values */
	if( motor_info_list[index]->rpm < 32767 )
	{
		pStruture->secondary_error = pStruture->secondary_target_value - motor_info_list[index]->rpm;
	}
	else if( motor_info_list[index]->rpm > 32767 )
	{
		pStruture->secondary_error = pStruture->secondary_target_value - ( motor_info_list[index]->rpm - 65535 );
	}
	
	/* Integral limit */
	if( secondary_structure.cumulative_err_max != 0 )
	{
		pStruture->secondary_cumulative_error = cumulative_limit_int16_t( pStruture->secondary_cumulative_error, pStruture->secondary_error, secondary_structure.cumulative_err_max );
	}
	
	/* Calculate the result of this PID operation */
	secondary_pid_result = ( pStruture->secondary_error * secondary_structure.kp ) + \
												 ( pStruture->secondary_cumulative_error * secondary_structure.ki ) + \
											 ( ( pStruture->secondary_error - pStruture->secondary_error_previous ) * secondary_structure.kd );
	
	/* Update error_previous value */
	pStruture->secondary_error_previous = pStruture->secondary_error;
	
	/* Output limiter */
	pStruture->secondary_output = output_limit( secondary_pid_result, secondary_structure.output_max );	
	
	/* Put the output value into the motor information structure to wait to be sent */
	motor_info_list[index]->final_output = pStruture->secondary_output;
	
	return LIB_OK;
}




int8_t relative_angle_cascade_calculation( uint32_t RecId, 
																						int32_t target_value, 
																						int32_t CW_angle_max, 
																						int32_t CCW_angle_max, 
												 primary_PID_param_struct_t primary_structure, 
											 secondary_PID_param_struct_t secondary_structure )
{
	struct relative_angle_cascade_t* pStruture = NULL;
	int32_t primary_pid_result = 0;
	int32_t secondary_pid_result = 0;
	int32_t res1 = 0, res2 = 0;
	int32_t mechanical_angle_change = 0;
	int32_t real_mechanical_angle_change = 0;
	
	int8_t index = RecId_find( RecId );
	
	/* Determine whether the specified RecId exists */
	if( index != INDEX_ERROR )
	{
		/* Determine whether the PID type is the specified type */
		if( motor_info_list[index]->PID_type != relative_angle_cascade )
		{
			/* Release the memory space of the original PID type structure */
			release_structure_memory( index );
			
			/* Apply for the memory space of the new PID type structure */
			request_structure_memory( index, relative_angle_cascade );
		}
	}
	
	/* The specified RecId does not exist */
	else
	{
		return INDEX_ERROR;
	}
	
	/* Get the PID structure address of this motor */
	pStruture = motor_info_list[index]->PID_structure_addr;
	
	/* Store the target value in the structure */
	pStruture->primary_target_value = target_value;
	
	/* Store the mechanical angle value into the structure */
	pStruture->mechanical_angle = motor_info_list[index]->mechanical_angle;
	
	if( !pStruture->used )
	{
		pStruture->mechanical_angle_previous = pStruture->mechanical_angle;
		pStruture->used = 1;
	}
	
	/* Calculate the angle change value */
	mechanical_angle_change = pStruture->mechanical_angle - pStruture->mechanical_angle_previous;
	
	if( mechanical_angle_change > 0 )
	{
		res1 = mechanical_angle_change - 8192;
		res2 = mechanical_angle_change;
	}
	else
	{
		res1 = mechanical_angle_change + 8192;
		res2 = mechanical_angle_change;
	}
	
	if( abs( res1 ) < abs( res2 ) )
	{
		real_mechanical_angle_change = res1;
	}
	else
	{
		real_mechanical_angle_change = res2;
	}
	
	/* Calculate the cumulative change angle */
	pStruture->spindle_angle_change_sum += real_mechanical_angle_change;
	
	/* Update the last angle value */
	pStruture->mechanical_angle_previous = pStruture->mechanical_angle;
	
	/* Determine whether you need to limit the angle */
	if( CW_angle_max || CCW_angle_max )
	{
		if( pStruture->primary_target_value > CW_angle_max )
		{
			pStruture->primary_target_value = CW_angle_max;
		}
		else if( pStruture->primary_target_value < -CCW_angle_max )
		{
			pStruture->primary_target_value = -CCW_angle_max;
		}
	}
	
	/* Calculation error */
	pStruture->primary_error = pStruture->primary_target_value - pStruture->spindle_angle_change_sum;
	
	/* Integral limit */
	if( primary_structure.cumulative_err_max != 0 )
	{
		pStruture->primary_cumulative_error = cumulative_limit_int32_t( pStruture->primary_cumulative_error, pStruture->primary_error, primary_structure.cumulative_err_max );
	}
	
	/* Calculate the result of this PID operation */
	primary_pid_result = ( pStruture->primary_error * primary_structure.kp ) + \
											 ( pStruture->primary_cumulative_error * primary_structure.ki ) + \
										 ( ( pStruture->primary_error - pStruture->primary_error_previous ) * primary_structure.kd );
	
	/* Update error_previous value */
	pStruture->primary_error_previous = pStruture->primary_error;
	
	/* Output limiter */
	pStruture->primary_output = output_limit( primary_pid_result, primary_structure.output_max );
	
	/*******************************************primary calculation end*************************************************/
	
	pStruture->secondary_target_value = pStruture->primary_output;
	
	/* Convert the speed value returned by the motor into positive and negative values */
	if( motor_info_list[index]->rpm < 32767 )
	{
		pStruture->secondary_error = pStruture->secondary_target_value - motor_info_list[index]->rpm;
	}
	else if( motor_info_list[index]->rpm > 32767 )
	{
		pStruture->secondary_error = pStruture->secondary_target_value - ( motor_info_list[index]->rpm - 65535 );
	}
	
	/* Integral limit */
	if( secondary_structure.cumulative_err_max != 0 )
	{
		pStruture->secondary_cumulative_error = cumulative_limit_int16_t( pStruture->secondary_cumulative_error, pStruture->secondary_error, secondary_structure.cumulative_err_max );
	}
	
	/* Calculate the result of this PID operation */
	secondary_pid_result = ( pStruture->secondary_error * secondary_structure.kp ) + \
												 ( pStruture->secondary_cumulative_error * secondary_structure.ki ) + \
											 ( ( pStruture->secondary_error - pStruture->secondary_error_previous ) * secondary_structure.kd );
	
	/* Update error_previous value */
	pStruture->secondary_error_previous = pStruture->secondary_error;
	
	/* Output limiter */
	pStruture->secondary_output = output_limit( secondary_pid_result, secondary_structure.output_max );	
	
	/* Put the output value into the motor information structure to wait to be sent */
	motor_info_list[index]->final_output = pStruture->secondary_output;
	
	return LIB_OK;
}

//int8_t gyro_angle_cascade_calculation( uint32_t RecId, 
//																			 int32_t target_value, 
//																			 enum euler_axis angular_velocity_axis, 
//																			 enum speed_loop_data_source speed_src, 
//																			 bool output_inversion,
//																			 primary_PID_param_struct_t primary_structure, 
//																			 secondary_PID_param_struct_t secondary_structure )
//{
//	struct gyro_angle_cascade_t* pStruture = NULL;
//	int32_t primary_pid_result = 0;
//	int32_t secondary_pid_result = 0;
//	
//	int8_t index = RecId_find( RecId );
//	
//	/* Determine whether the specified RecId exists */
//	if( index != INDEX_ERROR )
//	{
//		/* Determine whether the PID type is the specified type */
//		if( motor_info_list[index]->PID_type != gyro_angle_cascade )
//		{
//			/* Release the memory space of the original PID type structure */
//			release_structure_memory( index );
//			
//			/* Apply for the memory space of the new PID type structure */
//			request_structure_memory( index, gyro_angle_cascade );
//		}
//	}
//	
//	/* The specified RecId does not exist */
//	else
//	{
//		return INDEX_ERROR;
//	}
//	
//	/* Get the PID structure address of this motor */
//	pStruture = motor_info_list[index]->PID_structure_addr;
//	
////	/* Determine whether the value exceeds the controllable range */
////	if( target_value > 8192 )
////	{
////		return NUMERICAL_ERROR;
////	}
////	else
////	{
////		/* Store the target value in the structure */
////		pStruture->primary_target_value = target_value;
////	
//	
//	/* Store the target value in the structure */
//	pStruture->primary_target_value = target_value;
//	
//	/* Determine which axis to use */
//	if( angular_velocity_axis == roll )
//	{
//		/* Calculation error */
//		/*Here convert the gyroscope value to 0-8192 */
//		pStruture->primary_error = pStruture->primary_target_value - ( (int32_t) roll_cumulative_change_angle * 22.755555 );
//	}
//	
//	/* Determine which axis to use */
//	else if( angular_velocity_axis == pitch )
//	{
//		/* Calculation error */
//		/*Here convert the gyroscope value to 0-8192 */
//		pStruture->primary_error = pStruture->primary_target_value - ( (int32_t) pitch_cumulative_change_angle * 22.755555 );
//	}
//	
//	/* Determine which axis to use */
//	else if( angular_velocity_axis == yaw )
//	{
//		/* Calculation error */
//		/*Here convert the gyroscope value to 0-8192 */
//		pStruture->primary_error = pStruture->primary_target_value - ( (int32_t) yaw_cumulative_change_angle * 22.755555 );
//	}
//	
//	/* Integral limit */
//	if( primary_structure.cumulative_err_max != 0 )
//	{
//		pStruture->primary_cumulative_error = cumulative_limit_int16_t( pStruture->primary_cumulative_error, pStruture->primary_error, primary_structure.cumulative_err_max );
//	}
//	
//	/* Calculate the result of this PID operation */
//	primary_pid_result = ( pStruture->primary_error * primary_structure.kp ) + \
//											 ( pStruture->primary_cumulative_error * primary_structure.ki ) + \
//										 ( ( pStruture->primary_error - pStruture->primary_error_previous ) * primary_structure.kd );	
//	
//	/* Update error_previous value */
//	pStruture->primary_error_previous = pStruture->primary_error;
//	
//	/* Output limiter */
//	pStruture->primary_output = output_limit( primary_pid_result, primary_structure.output_max );
//	
//	/*******************************************primary calculation end*************************************************/
//	
//	pStruture->secondary_target_value = pStruture->primary_output;
//	
////	/* Convert the speed value returned by the motor into positive and negative values */
////	if( motor_info_list[index]->rpm < 32767 )
////	{
////		pStruture->secondary_error = pStruture->secondary_target_value - motor_info_list[index]->rpm;
////	}
////	else if( motor_info_list[index]->rpm > 32767 )
////	{
////		pStruture->secondary_error = pStruture->secondary_target_value - ( motor_info_list[index]->rpm - 65535 );
////	}
//	
//	
//	/* Convert the speed value returned by the motor into positive and negative values */
//	if( speed_src == motor )
//	{
//		if( motor_info_list[index]->rpm < 32767 )
//		{
//			pStruture->secondary_error = pStruture->secondary_target_value - motor_info_list[index]->rpm;
//		}
//		else if( motor_info_list[index]->rpm > 32767 )
//		{
//			pStruture->secondary_error = pStruture->secondary_target_value - ( motor_info_list[index]->rpm - 65535 );
//		}
//	}
//	else if( speed_src == gyro )
//	{
//		if( angular_velocity_axis == roll )
//		{
//			pStruture->secondary_error = pStruture->secondary_target_value - CH110_data.X_axisAngularVelocity;
//		}
//		else if( angular_velocity_axis == pitch )
//		{
//			pStruture->secondary_error = pStruture->secondary_target_value - CH110_data.Y_axisAngularVelocity;
//		}
//		else if( angular_velocity_axis == yaw )
//		{
//			pStruture->secondary_error = pStruture->secondary_target_value - CH110_data.Z_axisAngularVelocity;
//		}
//	}
//	
//	
//	
//	
//	/* Integral limit */
//	if( secondary_structure.cumulative_err_max != 0 )
//	{
//		pStruture->secondary_cumulative_error = cumulative_limit_int16_t( pStruture->secondary_cumulative_error, pStruture->secondary_error, secondary_structure.cumulative_err_max );
//	}
//	
//	/* Calculate the result of this PID operation */
//	secondary_pid_result = ( pStruture->secondary_error * secondary_structure.kp ) + \
//												 ( pStruture->secondary_cumulative_error * secondary_structure.ki ) + \
//											 ( ( pStruture->secondary_error - pStruture->secondary_error_previous ) * secondary_structure.kd );
//	
//	/* Update error_previous value */
//	pStruture->secondary_error_previous = pStruture->secondary_error;
//	
//	/* Output limiter */
//	pStruture->secondary_output = output_limit( secondary_pid_result, secondary_structure.output_max );	
//	
//	/* Put the output value into the motor information structure to wait to be sent */
//	if( output_inversion == false )
//	{
//		motor_info_list[index]->final_output = pStruture->secondary_output;
//	}
//	else
//	{
//		motor_info_list[index]->final_output = -pStruture->secondary_output;
//	}
//	
//	return LIB_OK;
//}

int8_t CH110_gyro_angle_cascade_calculation( uint32_t RecId, 
																						int32_t target_value, 
																						enum euler_axis angular_velocity_axis, 
																						enum speed_loop_data_source speed_src, 
																						bool output_inversion,
																						primary_PID_param_struct_t primary_structure, 
																						secondary_PID_param_struct_t secondary_structure )
{
	struct gyro_angle_cascade_t* pStruture = NULL;
	int32_t primary_pid_result = 0;
	int32_t secondary_pid_result = 0;
	
	int8_t index = RecId_find( RecId );
	
	/* Determine whether the specified RecId exists */
	if( index != INDEX_ERROR )
	{
		/* Determine whether the PID type is the specified type */
		if( motor_info_list[index]->PID_type != gyro_angle_cascade )
		{
			/* Release the memory space of the original PID type structure */
			release_structure_memory( index );
			
			/* Apply for the memory space of the new PID type structure */
			request_structure_memory( index, gyro_angle_cascade );
		}
	}
	
	/* The specified RecId does not exist */
	else
	{
		return INDEX_ERROR;
	}
	
	/* Get the PID structure address of this motor */
	pStruture = motor_info_list[index]->PID_structure_addr;
	
//	/* Determine whether the value exceeds the controllable range */
//	if( target_value > 8192 )
//	{
//		return NUMERICAL_ERROR;
//	}
//	else
//	{
//		/* Store the target value in the structure */
//		pStruture->primary_target_value = target_value;
//	
	
	/* Store the target value in the structure */
	pStruture->primary_target_value = target_value;
	
	/* Determine which axis to use */
	if( angular_velocity_axis == roll )
	{
		/* Calculation error */
		/*Here convert the gyroscope value to 0-8192 */
			pStruture->primary_error = pStruture->primary_target_value - ( ( int32_t ) ( roll_cumulative_change_angle * 22.755555f ) );
	}
	
	/* Determine which axis to use */
	else if( angular_velocity_axis == pitch )
	{
		/* Calculation error */
		/*Here convert the gyroscope value to 0-8192 */
			pStruture->primary_error = pStruture->primary_target_value - ( ( int32_t ) ( pitch_cumulative_change_angle * 22.755555f ) );
	}
	
	/* Determine which axis to use */
	else if( angular_velocity_axis == yaw )
	{
		/* Calculation error */
		/*Here convert the gyroscope value to 0-8192 */
			pStruture->primary_error = pStruture->primary_target_value - ( ( int32_t ) ( yaw_cumulative_change_angle * 22.755555f ) );
	}
	
	/* Integral limit */
	if( primary_structure.cumulative_err_max != 0 )
	{
		pStruture->primary_cumulative_error = cumulative_limit_int16_t( pStruture->primary_cumulative_error, pStruture->primary_error, primary_structure.cumulative_err_max );
	}
	
	/* Calculate the result of this PID operation */
	primary_pid_result = ( pStruture->primary_error * primary_structure.kp ) + \
											 ( pStruture->primary_cumulative_error * primary_structure.ki ) + \
										 ( ( pStruture->primary_error - pStruture->primary_error_previous ) * primary_structure.kd );
	
	/* Update error_previous value */
	pStruture->primary_error_previous = pStruture->primary_error;
	
	/* Output limiter */
	pStruture->primary_output = output_limit( primary_pid_result, primary_structure.output_max );
	
	/*******************************************primary calculation end*************************************************/
	
	pStruture->secondary_target_value = pStruture->primary_output;
	
	/* Convert the speed value returned by the motor into positive and negative values */
	if( speed_src == motor )
	{
		if( motor_info_list[index]->rpm < 32767 )
		{
			pStruture->secondary_error = pStruture->secondary_target_value - motor_info_list[index]->rpm;
		}
		else if( motor_info_list[index]->rpm > 32767 )
		{
			pStruture->secondary_error = pStruture->secondary_target_value - ( motor_info_list[index]->rpm - 65535 );
		}
	}
	else if( speed_src == gyro )
	{
		if( angular_velocity_axis == roll )
		{
			pStruture->secondary_error = pStruture->secondary_target_value - CH110_data.X_axisAngularVelocity;
		}
		else if( angular_velocity_axis == pitch )
		{
			pStruture->secondary_error = pStruture->secondary_target_value - CH110_data.Y_axisAngularVelocity;
		}
		else if( angular_velocity_axis == yaw )
		{
			pStruture->secondary_error = pStruture->secondary_target_value - CH110_data.Z_axisAngularVelocity;
		}
	}
	
	/* Integral limit */
	if( secondary_structure.cumulative_err_max != 0 )
	{
		pStruture->secondary_cumulative_error = cumulative_limit_int16_t( pStruture->secondary_cumulative_error, pStruture->secondary_error, secondary_structure.cumulative_err_max );
	}
	
	/* Calculate the result of this PID operation */
	secondary_pid_result = ( pStruture->secondary_error * secondary_structure.kp ) + \
												 ( pStruture->secondary_cumulative_error * secondary_structure.ki ) + \
											 ( ( pStruture->secondary_error - pStruture->secondary_error_previous ) * secondary_structure.kd );
	
	/* Update error_previous value */
	pStruture->secondary_error_previous = pStruture->secondary_error;
	
	/* Output limiter */
	pStruture->secondary_output = output_limit( secondary_pid_result, secondary_structure.output_max );	
	
	/* Put the output value into the motor information structure to wait to be sent */
	if( output_inversion == false )
	{
		motor_info_list[index]->final_output = pStruture->secondary_output;
	}
	else
	{
		motor_info_list[index]->final_output = -pStruture->secondary_output;
	}
	
	return LIB_OK;
}

int8_t vision_angle_cascade_caculation( uint32_t RecId,
																				int32_t target_value,
																				enum euler_axis angular_velocity_axis,
																				enum speed_loop_data_source speed_src,
																				bool output_inversion,
																				primary_PID_param_struct_t primary_structure,
																				secondary_PID_param_struct_t secondary_structure )
{
	struct vision_angle_cascade_t* pStruture = NULL;
	int32_t primary_pid_result = 0;
	int32_t secondary_pid_result = 0;
	
	int8_t index = RecId_find( RecId );
	
	/* Determine whether the specified RecId exists */
	if( index != INDEX_ERROR )
	{
		/* Determine whether the PID type is the specified type */
		if( motor_info_list[index]->PID_type != gyro_angle_cascade )
		{
			/* Release the memory space of the original PID type structure */
			release_structure_memory( index );
			
			/* Apply for the memory space of the new PID type structure */
			request_structure_memory( index, gyro_angle_cascade );
		}
	}
	
	/* The specified RecId does not exist */
	else
	{
		return INDEX_ERROR;
	}
	
	/* Get the PID structure address of this motor */
	pStruture = motor_info_list[index]->PID_structure_addr;
	
	/* Store the target value in the structure */
	pStruture->primary_target_value = target_value;
	
	/* Determine which axis to use */
	if( angular_velocity_axis == yaw )
	{
		/* Calculation error */
		/*Here convert the gyroscope value to 0-8192 */
			pStruture->primary_error = pStruture->primary_target_value - vision_struct.vision_yaw_increment_angle;
	}
	
	/* Determine which axis to use */
	else if( angular_velocity_axis == pitch )
	{
		/* Calculation error */
		/*Here convert the gyroscope value to 0-8192 */
			pStruture->primary_error = pStruture->primary_target_value - ( int32_t ) vision_struct.vision_pitch_increment_angle ;
	}
	
	/* Integral limit */
	if( primary_structure.cumulative_err_max != 0 )
	{
		pStruture->primary_cumulative_error = cumulative_limit_int16_t( pStruture->primary_cumulative_error, pStruture->primary_error, primary_structure.cumulative_err_max );
	}
	
	/* Calculate the result of this PID operation */
	primary_pid_result = ( pStruture->primary_error * primary_structure.kp ) + \
											 ( pStruture->primary_cumulative_error * primary_structure.ki ) + \
										 ( ( pStruture->primary_error - pStruture->primary_error_previous ) * primary_structure.kd );
	
	/* Update error_previous value */
	pStruture->primary_error_previous = pStruture->primary_error;
	
	/* Output limiter */
	pStruture->primary_output = output_limit( primary_pid_result, primary_structure.output_max );
	
	/*******************************************primary calculation end*************************************************/
	
	pStruture->secondary_target_value = pStruture->primary_output;
	
	/* Convert the speed value returned by the motor into positive and negative values */
	if( speed_src == motor )
	{
		if( motor_info_list[index]->rpm < 32767 )
		{
			pStruture->secondary_error = pStruture->secondary_target_value - motor_info_list[index]->rpm;
		}
		else if( motor_info_list[index]->rpm > 32767 )
		{
			pStruture->secondary_error = pStruture->secondary_target_value - ( motor_info_list[index]->rpm - 65535 );
		}
	}
	else if( speed_src == gyro )
	{
		if( angular_velocity_axis == roll )
		{
			pStruture->secondary_error = pStruture->secondary_target_value - CH110_data.X_axisAngularVelocity;
		}
		else if( angular_velocity_axis == pitch )
		{
			pStruture->secondary_error = pStruture->secondary_target_value - CH110_data.Y_axisAngularVelocity;
		}
		else if( angular_velocity_axis == yaw )
		{
			pStruture->secondary_error = pStruture->secondary_target_value - CH110_data.Z_axisAngularVelocity;
		}
	}
	
	/* Integral limit */
	if( secondary_structure.cumulative_err_max != 0 )
	{
		pStruture->secondary_cumulative_error = cumulative_limit_int16_t( pStruture->secondary_cumulative_error, pStruture->secondary_error, secondary_structure.cumulative_err_max );
	}
	
	/* Calculate the result of this PID operation */
	secondary_pid_result = ( pStruture->secondary_error * secondary_structure.kp ) + \
												 ( pStruture->secondary_cumulative_error * secondary_structure.ki ) + \
											 ( ( pStruture->secondary_error - pStruture->secondary_error_previous ) * secondary_structure.kd );
	
	/* Update error_previous value */
	pStruture->secondary_error_previous = pStruture->secondary_error;
	
	/* Output limiter */
	pStruture->secondary_output = output_limit( secondary_pid_result, secondary_structure.output_max );	
	
	/* Put the output value into the motor information structure to wait to be sent */
	if( output_inversion == false )
	{
		motor_info_list[index]->final_output = pStruture->secondary_output;
	}
	else
	{
		motor_info_list[index]->final_output = -pStruture->secondary_output;
	}
	
	return LIB_OK;
}
