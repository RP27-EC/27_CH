#include "board_protocol.h"
/*------------------------板间通信测试代码---------------*/
//#define STRUCT_TEST

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart7;
extern UART_HandleTypeDef huart10;

int num = 60;//校验字节
uint8_t rx_info_buff[100];//观察通过校验的数据

uint8_t tx_info_buff[100];//发送的包
uint8_t crc_tx[60];//发送数据字节
bool is_pass_crc;//crc通过标志位

TestStruct test_struct=			//测试结构体发送,60字节
{
	.u8_1=0xA5,
};
//测试结构体内容初始化，置于device.c
void board_test_init()
{
	tx_info_buff [0] = 0xA5;
    for(uint8_t i = 1; i < num; i++)
    {
        tx_info_buff[i] = i + 1;  
    }
}
uint32_t lenghth;

void board_send()
{	
	lenghth = sizeof (TestStruct );
	tx_info_buff [1]++;
	if(tx_info_buff [1] >= 255)
		tx_info_buff [1] = 0;
	
	#ifdef STRUCT_TEST 
	
	test_struct .u8_2 = tx_info_buff[1];
	
	Append_CRC8_Check_Sum ((uint8_t*)&test_struct ,3);
	
	Append_CRC16_Check_Sum ( (uint8_t*)&test_struct,lenghth);
	
	HAL_UART_Transmit_DMA(&huart10 , (uint8_t*)&test_struct , lenghth);
	
	#else
	memcpy (crc_tx  ,tx_info_buff  ,num);
	
	Append_CRC8_Check_Sum (crc_tx ,3);
	
	Append_CRC16_Check_Sum (crc_tx ,num);
	
	HAL_UART_Transmit_DMA(&huart10 , crc_tx  , num);
	
	#endif
}

bool board_recieve(uint8_t *rxBuf)
{
	
	if (rxBuf == NULL) {
        return false;
    }
	if(Verify_CRC8_Check_Sum (rxBuf ,3) == true)
	{
		#ifdef STRUCT_TEST
		if(Verify_CRC16_Check_Sum (rxBuf ,lenghth ) == true)
		{
			memcpy (&rx_info_buff  ,rxBuf ,lenghth);
			return true;
		}
		#else
		if(Verify_CRC16_Check_Sum (rxBuf ,num ) == true)
		{
			memcpy (&rx_info_buff  ,rxBuf ,num);
			return true;
		}
		#endif
	}
	else return false;
}

void USART1_rxDataHandler(uint8_t *rxBuf)
{
	is_pass_crc=board_recieve (rxBuf);
}

void USART7_rxDataHandler(uint8_t *rxBuf)
{
	is_pass_crc=board_recieve (rxBuf);
}

void USART10_rxDataHandler(uint8_t *rxBuf)
{
	is_pass_crc=board_recieve (rxBuf);
}
