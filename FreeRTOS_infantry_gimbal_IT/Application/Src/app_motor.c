/*
** This file is BullShit --@Author
*/

#define MOTOR_GLOBALS
#include "includes.h"

uint8_t rotating_speed_count;
uint8_t relative_angle_count;
uint8_t absolute_angle_count;
uint8_t relative_angle_cascade_count;
uint8_t absolute_angle_cascade_count;
uint8_t gyro_angle_cascade_count;
uint8_t vision_angle_cascade_count;

void add_motor( uint32_t RecId, uint32_t SendId )
{
	static uint8_t motor_info_count;

	/* Apply for memory space for storing motor information for newly added motors */
	motor_info_list[motor_info_count] = ( struct motor_info_t* )pvPortMalloc( sizeof( struct motor_info_t ) );
	
	/* Store the motor receiving address */
	motor_info_list[motor_info_count]->RecId = RecId;
	
	/* Store the motor sending address */
	motor_info_list[motor_info_count]->SendId = SendId;
	
	/* Mark this motor used */
	motor_info_list[motor_info_count]->used = true;
	
	/* Array index increment */
	motor_info_count++;
}

int8_t request_structure_memory( uint8_t index, enum PID_type_t type )
{	
	switch( type )
	{
		case rotating_speed:
			
			/* Apply for memory space to store the data needed for PID operation */
			rotating_speed_list[rotating_speed_count] = ( struct rotating_speed_t* )pvPortMalloc( sizeof( struct rotating_speed_t ) );
			
			/* Clear the data in the newly applied memory space */
			memset( ( void* )rotating_speed_list[rotating_speed_count], 0, sizeof( struct rotating_speed_t ) );
			
			/* Apply for memory space to store the data needed for PID operation */
			motor_info_list[index]->PID_structure_addr = ( void* )rotating_speed_list[rotating_speed_count];
			
			/* Store the PID control method to be used by the motor */
			motor_info_list[index]->PID_type = rotating_speed;
			
			/* Array index increment */
			rotating_speed_count++;
			break;
		
		case relative_angle:
			/* Same as above */
			relative_angle_list[relative_angle_count] = ( struct relative_angle_t* )pvPortMalloc( sizeof( struct relative_angle_t ) );

			memset( ( void* )relative_angle_list[relative_angle_count], 0, sizeof( struct relative_angle_t ) );
			
			motor_info_list[index]->PID_structure_addr = ( void* )relative_angle_list[relative_angle_count];
			motor_info_list[index]->PID_type = relative_angle;
			relative_angle_count++;
			break;
		
		case absolute_angle:
			/* Same as above */
			absolute_angle_list[absolute_angle_count] = ( struct absolute_angle_t* )pvPortMalloc( sizeof( struct absolute_angle_t ) );
			
			memset( ( void* )absolute_angle_list[absolute_angle_count], 0, sizeof( struct absolute_angle_t ) );
			
			motor_info_list[index]->PID_structure_addr = ( void* )absolute_angle_list[absolute_angle_count];
			motor_info_list[index]->PID_type = absolute_angle;
			absolute_angle_count++;
			break;
		
		case relative_angle_cascade:
			/* Same as above */
			relative_angle_cascade_list[relative_angle_cascade_count] = ( struct relative_angle_cascade_t* )pvPortMalloc( sizeof( struct relative_angle_cascade_t ) );
			
			memset( ( void* )relative_angle_cascade_list[relative_angle_cascade_count], 0, sizeof( struct relative_angle_cascade_t ) );
			
			motor_info_list[index]->PID_structure_addr = ( void* )relative_angle_cascade_list[relative_angle_cascade_count];
			motor_info_list[index]->PID_type = relative_angle_cascade;
			relative_angle_cascade_count++;
			break;
		
		case absolute_angle_cascade:
			/* Same as above */
			absolute_angle_cascade_list[absolute_angle_cascade_count] = ( struct absolute_angle_cascade_t* )pvPortMalloc( sizeof( struct absolute_angle_cascade_t ) );

			memset( ( void* )absolute_angle_cascade_list[absolute_angle_cascade_count], 0, sizeof( struct absolute_angle_cascade_t ) );
			
			motor_info_list[index]->PID_structure_addr = ( void* )absolute_angle_cascade_list[absolute_angle_cascade_count];
			motor_info_list[index]->PID_type = absolute_angle_cascade;
			absolute_angle_cascade_count++;
			break;
		
		case gyro_angle_cascade:
			/* Same as above */
			gyro_angle_cascade_list[gyro_angle_cascade_count] = ( struct gyro_angle_cascade_t* )pvPortMalloc( sizeof( struct gyro_angle_cascade_t ) );
			
			memset( ( void* )gyro_angle_cascade_list[gyro_angle_cascade_count], 0, sizeof( struct gyro_angle_cascade_t ) );
			
			motor_info_list[index]->PID_structure_addr = ( void* )gyro_angle_cascade_list[gyro_angle_cascade_count];
			motor_info_list[index]->PID_type = gyro_angle_cascade;
			gyro_angle_cascade_count++;
			break;
		
		case vision_angle_cascade:
			/* Same as above */
			vision_angle_cascade_list[vision_angle_cascade_count] = ( struct vision_angle_cascade_t* )pvPortMalloc( sizeof( struct vision_angle_cascade_t ) );
		
			memset( ( void* )vision_angle_cascade_list[vision_angle_cascade_count], 0, sizeof( struct vision_angle_cascade_t ) );
			
			motor_info_list[index]->PID_structure_addr = ( void* )vision_angle_cascade_list[vision_angle_cascade_count];
			motor_info_list[index]->PID_type = vision_angle_cascade;
			vision_angle_cascade_count++;
		
		default:
			return TYPE_ERROR;
	}
	return LIB_OK;
}

int8_t release_structure_memory( uint8_t index)
{
	enum PID_type_t type;
	type = motor_info_list[index]->PID_type;
	
	switch( type )
	{
		case rotating_speed:
			rotating_speed_count--;
			break;
		
		case relative_angle:
			relative_angle_count--;
			break;
		
		case absolute_angle:
			absolute_angle_count--;
			break;
		
		case relative_angle_cascade:
			relative_angle_cascade_count--;
			break;
		
		case absolute_angle_cascade:
			absolute_angle_cascade_count--;
			break;
		
		case gyro_angle_cascade:
			gyro_angle_cascade_count--;
			break;
		
		default:
			return TYPE_ERROR;
	}
	
	void *addr = motor_info_list[index]->PID_structure_addr;
	
	/* Release memory */
	vPortFree( addr );
	
	/* Clear the address of the freed memory */
	motor_info_list[index]->PID_structure_addr = NULL;
	
	/* Clear PID type */
	motor_info_list[index]->PID_type = nothing;
	
	return LIB_OK;
}

/* 将需要旋转的角度转换为电机角度 */
uint16_t absolute_angle_remainder( int32_t value )
{
	if( value > 8192 )
		return value % 8192;
	else if( value < 0 )
		return 8192 + ( value % 8192 );
	else
		return value;
}

/* 清除指定ID电机的累计旋转角度 */
void motor_angle_sum_clear( uint32_t RecId )
{
	for( uint8_t index = 0; index < motor_count; index++ )
	{
		if( motor_info_list[index]->RecId == RecId )
			motor_info_list[index]->angle_change_sum = 0;
	}
}

/* 获取指定ID电机的累计旋转角度（不知为何极其的慢） */
int32_t get_motor_angle_sum( uint32_t RecId )
{
	for( uint8_t index = 0; index < motor_count; index++ )
	{
		if( motor_info_list[index]->RecId == RecId )
		{
			return motor_info_list[index]->angle_change_sum;
		}
	}
}
