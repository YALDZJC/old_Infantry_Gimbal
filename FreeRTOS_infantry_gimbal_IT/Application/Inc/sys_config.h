//#define infantry_2

/* config */

/* important!!! Please find "Define" in "C/C++" in "Options for target" and add "__chassis__" or "__gimbal__" */
//#ifdef __gimbal__
#define bool uint8_t
#define true 1
#define false 0

#define PI 3.14159265f

#define INDEX_ERROR -1	
#define TYPE_ERROR -2
#define DIRECT_ERROR -3
#define NUMERICAL_ERROR -4
#define LIB_OK 1

#define translation 1
#define rotation 2
#define niubi 3

#define cos45 0.707107f
#define sin45 0.707107f

#define motor_count 5

#define REMOTE_DEAD_BAND	10

//#define SHOOT_HEAT_DEAD_BAND 10

#define enable_angle_sum_clac_0x201
//#define enable_angle_sum_clac_0x202
//#define enable_angle_sum_clac_0x203
//#define enable_angle_sum_clac_0x204
#define enable_angle_sum_clac_0x205
#define enable_angle_sum_clac_0x206
//#define enable_angle_sum_clac_0x207
//#define enable_angle_sum_clac_0x208
//#define enable_angle_sum_clac_0x209

#define remote_uart						huart3    /* UART_HandleTypeDef */
#define remote_data_size			18        /* Number of bytes received & Depth of message queue "RemoteMsgQueue" */
#define remote_queue_lenght		15				/* The length of the message queue "RemoteMsgQueue" */

#define motor_can1						hcan1			/* CAN_HandleTypeDef */
#define motor_data_size				12        /* Depth of the message queue "CanMsgQueue" */
#define motor_queue_lenght		15				/* The length of the message queue "CanMsgQueue" */

#define CH110_can							hcan2			/* CAN_HandleTypeDef */
#define CH110_data_size				12        /* Depth of the message queue "CH110MsgQueue" */
#define CH110_queue_lenght		15				/* The length of the message queue "CH110MsgQueue" */

#define vision_uart								huart6
#define vision_receive_data_size	23
#define vision_send_data_size			12
#define vision_queue_lenght				15

#define communication_uart							huart1	//ËÄÏß
#define communication_receive_data_size	15
#define communication_send_data_size		28
#define communication_queue_lenght			15

#ifndef infantry_2

#define initial_absolute_angle_0x205 3040
#define initial_absolute_angle_0x206 1303

#else

#define initial_absolute_angle_0x205 7880
#define initial_absolute_angle_0x206 4444

#endif /* infantry_2 */
