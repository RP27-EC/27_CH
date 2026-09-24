#include "can_protocol.h"
#include "Chassis_Motor.h"
#include "gimbal_Motor.h"
#include "cap_protocol.h"
#include "judge.h"
#include "cap.h"

/**
 *  @brief  CAN1 接收数据
 */
void CAN1_rxDataHandler(uint32_t rxId, uint8_t *rxBuf)
{
	switch (rxId)
	{
		case 0x012:
		Yaw_Motor.rx(&Yaw_Motor, rxBuf);
		break;
		default:
			break;
	}
}



/**
 *  @brief  CAN2 接收数据
 */
void CAN2_rxDataHandler(uint32_t rxId, uint8_t *rxBuf)
{
	switch (rxId)
	{
		case 0x001:
		Yaw_Motor.rx(&Yaw_Motor, rxBuf);
		break;
		default:
			break;
	}
}


/**
 *  @brief  CAN3 接收数据
 */
void CAN3_rxDataHandler(uint32_t rxId, uint8_t *rxBuf)
{
	switch (rxId)
	{
		default:
			break;
	}
}
