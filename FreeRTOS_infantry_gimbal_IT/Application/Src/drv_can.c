#define CAN_GLOBALS
#include "includes.h"
extern float buchang;
float temp;
void can_manage_init( void )
{
	/* can1 */
	CAN_FilterTypeDef CAN_FilterInitStructure = {0};
	
	CAN_FilterInitStructure.FilterIdHigh         = 0x0000;
	CAN_FilterInitStructure.FilterIdLow          = 0x0000;
	CAN_FilterInitStructure.FilterMaskIdHigh     = 0x0000;
	CAN_FilterInitStructure.FilterMaskIdLow      = 0x0000;
	CAN_FilterInitStructure.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	CAN_FilterInitStructure.FilterBank           = 0;
	CAN_FilterInitStructure.FilterMode           = CAN_FILTERMODE_IDMASK;
	CAN_FilterInitStructure.FilterScale          = CAN_FILTERSCALE_32BIT;
	CAN_FilterInitStructure.FilterActivation     = CAN_FILTER_ENABLE;
	CAN_FilterInitStructure.SlaveStartFilterBank = 14;
	
	HAL_CAN_ConfigFilter( &motor_can1, &CAN_FilterInitStructure );
	HAL_CAN_ActivateNotification( &motor_can1, CAN_IT_RX_FIFO0_MSG_PENDING );
	HAL_CAN_Start( &motor_can1 );
	
	/* can2 */	
	CAN_FilterInitStructure.FilterIdHigh         = 0x0000;
	CAN_FilterInitStructure.FilterIdLow          = 0x0000;
	CAN_FilterInitStructure.FilterMaskIdHigh     = 0x0000;
	CAN_FilterInitStructure.FilterMaskIdLow      = 0x0000;
	CAN_FilterInitStructure.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	CAN_FilterInitStructure.FilterBank           = 15;
	CAN_FilterInitStructure.FilterMode           = CAN_FILTERMODE_IDMASK;
	CAN_FilterInitStructure.FilterScale          = CAN_FILTERSCALE_32BIT;
	CAN_FilterInitStructure.FilterActivation     = CAN_FILTER_ENABLE;
	CAN_FilterInitStructure.SlaveStartFilterBank = 14;
	
	HAL_CAN_ConfigFilter( &CH110_can, &CAN_FilterInitStructure );
	HAL_CAN_ActivateNotification( &CH110_can, CAN_IT_RX_FIFO0_MSG_PENDING );
	HAL_CAN_Start( &CH110_can );
}

void HAL_CAN_RxFifo0MsgPendingCallback( CAN_HandleTypeDef *hcan )
{
	BaseType_t xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;
	
	if( hcan->Instance == motor_can1.Instance )
	{
		CAN_RxHeaderTypeDef rx_header;
		uint8_t rx_buf[8];
		uint8_t tx_buf[motor_data_size];
		
		HAL_CAN_GetRxMessage( &motor_can1, CAN_RX_FIFO0, &rx_header, rx_buf );
		
		for( uint8_t index = 0; index < motor_count; index++ )
		{
			if( motor_info_list[index]->RecId == rx_header.StdId )
			{
				/* Mark motor online */
				motor_info_list[index]->online_reply = true;
			}
		}
		
		tx_buf[0] = rx_header.StdId >> 24;
		tx_buf[1] = rx_header.StdId >> 16;
		tx_buf[2] = rx_header.StdId >> 8;
		tx_buf[3] = rx_header.StdId >> 0;
		tx_buf[4] = rx_buf[0];
		tx_buf[5] = rx_buf[1];
		tx_buf[6] = rx_buf[2];
		tx_buf[7] = rx_buf[3];
		tx_buf[8] = rx_buf[4];
		tx_buf[9] = rx_buf[5];
		tx_buf[10] = rx_buf[6];
		tx_buf[11] = rx_buf[7];
		
		xQueueSendFromISR( CanMsgQueue, tx_buf, &xHigherPriorityTaskWoken );
	}
	else if( hcan->Instance == CH110_can.Instance )
	{
		CAN_RxHeaderTypeDef rx_header;
		uint8_t rx_buf[8];
		uint8_t tx_buf[motor_data_size];
		
		HAL_CAN_GetRxMessage( &CH110_can, CAN_RX_FIFO0, &rx_header, rx_buf );
		
		tx_buf[0] = rx_header.StdId >> 24;
		tx_buf[1] = rx_header.StdId >> 16;
		tx_buf[2] = rx_header.StdId >> 8;
		tx_buf[3] = rx_header.StdId >> 0;
		tx_buf[4] = rx_buf[0];
		tx_buf[5] = rx_buf[1];
		tx_buf[6] = rx_buf[2];
		tx_buf[7] = rx_buf[3];
		tx_buf[8] = rx_buf[4];
		tx_buf[9] = rx_buf[5];
		tx_buf[10] = rx_buf[6];
		tx_buf[11] = rx_buf[7];
		
		xQueueSendFromISR( CH110MsgQueue, tx_buf, &xHigherPriorityTaskWoken );
		
		gyro_online_reply = true;
	}
	
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

static void __0x200_Encode( uint8_t index )
{
  
	if( motor_info_list[index]->RecId == 0x201 ){
		tx_data_0x200[0] = motor_info_list[index]->final_output >> 8;
		tx_data_0x200[1] = motor_info_list[index]->final_output;
	}
	else if( motor_info_list[index]->RecId == 0x202 ){
		tx_data_0x200[2] = motor_info_list[index]->final_output >> 8;
		tx_data_0x200[3] = motor_info_list[index]->final_output;
	}
	else if( motor_info_list[index]->RecId == 0x203 ){
		tx_data_0x200[4] = motor_info_list[index]->final_output >> 8;
		tx_data_0x200[5] = motor_info_list[index]->final_output;
	}
	else if( motor_info_list[index]->RecId == 0x204 ){
		tx_data_0x200[6] = motor_info_list[index]->final_output >> 8;
		tx_data_0x200[7] = motor_info_list[index]->final_output;
	}
}

static void __0x1FF_Encode( uint8_t index )
{
//    temp=motor_info_list[1]->final_output;

//		if(motor_info_list[1]->final_output>30000)motor_info_list[1]->final_output=30000;
//		if(motor_info_list[1]->final_output<-30000)motor_info_list[1]->final_output=-30000;
	if( motor_info_list[index]->RecId == 0x205 ){
		tx_data_0x1FF[0] = motor_info_list[index]->final_output >> 8;
		tx_data_0x1FF[1] = motor_info_list[index]->final_output;
	}
	else if( motor_info_list[index]->RecId == 0x206 ){
		tx_data_0x1FF[2] = motor_info_list[index]->final_output >> 8;
		tx_data_0x1FF[3] = motor_info_list[index]->final_output;
	}
	else if( motor_info_list[index]->RecId == 0x207 ){
		tx_data_0x1FF[4] = motor_info_list[index]->final_output >> 8;
		tx_data_0x1FF[5] = motor_info_list[index]->final_output;
	}
	else if( motor_info_list[index]->RecId == 0x208 ){
		tx_data_0x1FF[6] = motor_info_list[index]->final_output >> 8;
		tx_data_0x1FF[7] = motor_info_list[index]->final_output;
	}
}

static void __0x2FF_Encode( uint8_t index )
{
  
	if( motor_info_list[index]->RecId == 0x209 ){
		tx_data_0x2FF[0] = motor_info_list[index]->final_output >> 8;
		tx_data_0x2FF[1] = motor_info_list[index]->final_output;
	}
	else if( motor_info_list[index]->RecId == 0x20A ){
		tx_data_0x2FF[2] = motor_info_list[index]->final_output >> 8;
		tx_data_0x2FF[3] = motor_info_list[index]->final_output;
	}
	else if( motor_info_list[index]->RecId == 0x20B ){
		tx_data_0x2FF[4] = motor_info_list[index]->final_output >> 8;
		tx_data_0x2FF[5] = motor_info_list[index]->final_output;
	}
}

void motor_control_send( CAN_HandleTypeDef CanHandle )
{
	xSemaphoreTake( xCAN1_Semaphore, portMAX_DELAY );
	CAN_TxHeaderTypeDef tx_header[2];
	
	tx_header[0].StdId = 0x200;
	tx_header[0].ExtId = 0;
	tx_header[0].IDE = CAN_ID_STD;
	tx_header[0].RTR = CAN_RTR_DATA;
	tx_header[0].DLC = 8;
	tx_header[0].TransmitGlobalTime = DISABLE;
	
	tx_header[1].StdId = 0x1FF;
	tx_header[1].ExtId = 0;
	tx_header[1].IDE = CAN_ID_STD;
	tx_header[1].RTR = CAN_RTR_DATA;
	tx_header[1].DLC = 8;
	tx_header[1].TransmitGlobalTime = DISABLE;
	
	for( uint8_t index = 0; index < motor_count; index++ )
	{
		if( motor_info_list[index]->SendId == 0x200 )
			__0x200_Encode( index );
		
		else if( motor_info_list[index]->SendId == 0x1FF )
			__0x1FF_Encode( index );
	}
	
	if( HAL_CAN_GetTxMailboxesFreeLevel( &CanHandle ) )
		HAL_CAN_AddTxMessage( &CanHandle, &tx_header[0], tx_data_0x200, ( uint32_t*) CAN_TX_MAILBOX0 );
	
	if( HAL_CAN_GetTxMailboxesFreeLevel( &CanHandle ) )
		HAL_CAN_AddTxMessage( &CanHandle, &tx_header[1], tx_data_0x1FF, ( uint32_t*) CAN_TX_MAILBOX0 );
	
	xSemaphoreGive( xCAN1_Semaphore );
}

//void motor_control_send_0x2FF( CAN_HandleTypeDef CanHandle )
//{
//	xSemaphoreTake( xCAN2_Semaphore, portMAX_DELAY );
//	CAN_TxHeaderTypeDef tx_header;
//	
//	tx_header.StdId = 0x2FF;
//	tx_header.ExtId = 0;
//	tx_header.IDE = CAN_ID_STD;
//	tx_header.RTR = CAN_RTR_DATA;
//	tx_header.DLC = 8;
//	tx_header.TransmitGlobalTime = DISABLE;
//	
//	for( uint8_t index = 0; index < motor_count; index++ )
//	{
//		if( motor_info_list[index]->SendId == 0x2FF )
//			__0x2FF_Encode( index );
//	}
//	
//	if( HAL_CAN_GetTxMailboxesFreeLevel( &CanHandle ) )
//		HAL_CAN_AddTxMessage( &CanHandle, &tx_header, tx_data_0x2FF, ( uint32_t*) CAN_TX_MAILBOX0 );
//	
//	xSemaphoreGive( xCAN2_Semaphore );
//}
