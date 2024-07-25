#ifdef UART_GLOBALS
#define UART_EXTERN
#else
#define UART_EXTERN extern
#endif

UART_EXTERN uint8_t remote_rx_buf[remote_data_size];
UART_EXTERN uint8_t communication_rx_buf[communication_receive_data_size];
UART_EXTERN uint8_t vision_rx_buf[vision_receive_data_size];

void uart_manage_init( void );
void uart_msg_send( UART_HandleTypeDef *huart, uint8_t *pBuffer, uint16_t Lenght );
