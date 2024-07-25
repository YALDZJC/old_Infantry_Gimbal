#define UART_GLOBALS
#include "includes.h"

uint8_t size;

void uart_manage_init( void )
{
	HAL_UARTEx_ReceiveToIdle_IT( &remote_uart, remote_rx_buf, remote_data_size );
	HAL_UARTEx_ReceiveToIdle_IT( &communication_uart, communication_rx_buf, communication_receive_data_size );
	HAL_UARTEx_ReceiveToIdle_IT( &vision_uart, vision_rx_buf, vision_receive_data_size );
	
	RecMsg.remote.ch0 = 1024;
	RecMsg.remote.ch1 = 1024;
	RecMsg.remote.ch2 = 1024;
	RecMsg.remote.ch3 = 1024;
}

void uart_msg_send( UART_HandleTypeDef *huart, uint8_t *pBuffer, uint16_t Lenght )
{
	taskENTER_CRITICAL();
	HAL_UART_Transmit_IT( huart, pBuffer, Lenght );
	taskEXIT_CRITICAL();
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	BaseType_t xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;
	
	/* ң�������ݴ��� */
	if( huart->Instance == remote_uart.Instance )
	{
		/* ����ң�������ݵ�"RemoteMsgQueue"���� */
		xQueueSendFromISR( RemoteMsgQueue, remote_rx_buf, &xHigherPriorityTaskWoken );
		
		/* ������������ */
		HAL_UARTEx_ReceiveToIdle_IT( &remote_uart, remote_rx_buf, remote_data_size );
		
		/* ң�������߱�־λ */
		remote_online_reply = true;
	}
	else if( huart->Instance == communication_uart.Instance )
	{
		/* ���͵��̴������������ݵ�"CommunicationMsgQueue"���� */
		xQueueSendFromISR( CommunicationMsgQueue, communication_rx_buf, &xHigherPriorityTaskWoken );
		
		/* ������������ */
		HAL_UARTEx_ReceiveToIdle_IT( &communication_uart, communication_rx_buf, communication_receive_data_size );
		
		/* ���ͨ�����߱�־λ */
		communication_online_reply = true;
	}
	else if( huart->Instance == vision_uart.Instance )
	{
		xQueueSendFromISR( VisionMsgQueue, vision_rx_buf, &xHigherPriorityTaskWoken );
		
		HAL_UARTEx_ReceiveToIdle_IT( &vision_uart, vision_rx_buf, vision_receive_data_size );
	}
	
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}
