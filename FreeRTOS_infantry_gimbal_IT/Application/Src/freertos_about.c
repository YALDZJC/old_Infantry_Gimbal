#define FREERTOS_ABOUT_GLOBALS
#include "includes.h"

void freertos_init(void)
{
	xCAN1_Semaphore = xSemaphoreCreateMutex();
	
	CanMsgQueue = xQueueCreate( motor_queue_lenght, motor_data_size );
	RemoteMsgQueue = xQueueCreate( remote_queue_lenght, remote_data_size );
	CH110MsgQueue = xQueueCreate( CH110_queue_lenght, CH110_data_size );
	CommunicationMsgQueue = xQueueCreate( communication_queue_lenght, communication_receive_data_size );
	VisionMsgQueue = xQueueCreate( vision_queue_lenght, vision_receive_data_size );
}
