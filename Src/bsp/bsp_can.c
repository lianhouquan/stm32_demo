/**
 ***************************************(C) COPYRIGHT 2019 USTC**************************************
 * @file       bsp_can.c
 * @brief      this file contains sd card basic operating function
 * @note         
 * @Version    V1.0.0
 * @Date          
 ***************************************(C) COPYRIGHT 2018 USTC**************************************
 */

#include "bsp_can.h"
#include "can.h"
#include "string.h"
#include "stm32f4xx_hal.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

moto_measure_t moto_chassis[8] = {0};	//记录四个底盘电机信息
uint8_t                  RxData[MOTO_RX_FIFO_SIZE];

/**
  * @brief  Configures the CAN, transmit and receive by polling
  * @param  None
  * @retval PASSED if the reception is well done, FAILED in other case
  */
HAL_StatusTypeDef can_filter_init(CAN_HandleTypeDef* hcan)
{
  CAN_FilterTypeDef  sFilterConfig;

  /*##-2- Configure the CAN Filter ###########################################*/
  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 14;
  
	if(hcan == &hcan1)
	{
	sFilterConfig.FilterBank = 0;
	}
	if(hcan == &hcan2)
	{
	sFilterConfig.FilterBank = 14;
	}
	
  if(HAL_CAN_ConfigFilter(hcan, &sFilterConfig) != HAL_OK)
  {
    /* Filter configuration Error */
    Error_Handler();
  }

  /*##-3- Start the CAN peripheral ###########################################*/
  if (HAL_CAN_Start(hcan) != HAL_OK)
  {
    /* Start Error */
    Error_Handler();
  }
	
  /*##-4- Activate CAN RX notification #######################################*/
  if (HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
    /* Notification Error */
    Error_Handler();
  }

	return HAL_OK;
}


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef   RxHeader;
	
  if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
  {
    /* Reception Error */
    Error_Handler();
  }
	
	switch (RxHeader.StdId)
	{
		case CAN_3510Moto1_ID:
		case CAN_3510Moto2_ID:
		case CAN_3510Moto3_ID:
		case CAN_3510Moto4_ID:
		{
			uint8_t index = 0;
			index=RxHeader.StdId-CAN_3510Moto1_ID;//得到电机下标号
			
			
			//舍弃前几次的值作为偏移,更新电机数据
			moto_chassis[index].msg_cnt++ <= 50	?	get_moto_offset(&moto_chassis[index], RxData) : get_moto_measure(&moto_chassis[index], RxData);
			break;
		}
			
		default : break;
	}
	
}


/*
  * @Func			void get_moto_measure(moto_measure_t *ptr, CAN_HandleTypeDef* hcan)
  * @Brief    接收云台电机,3508电机通过CAN发过来的信息
  * @Param		
  * @Retval		None
  * @Date     2015/11/24
*/
void get_moto_measure(moto_measure_t *ptr, uint8_t* Rxdate)
{
	ptr->last_angle = ptr->angle;
	ptr->angle = (uint16_t)(Rxdate[0]<<8 | Rxdate[1]) ;
	ptr->speed_rpm  = (int16_t)(Rxdate[2]<<8 | Rxdate[3]);
	ptr->given_current = (int16_t)(Rxdate[4]<<8 | Rxdate[5])/-5;
	ptr->hall = Rxdate[6];
	if(ptr->angle - ptr->last_angle > 4096)
		ptr->round_cnt --;
	else if (ptr->angle - ptr->last_angle < -4096)
		ptr->round_cnt ++;
	ptr->total_angle = ptr->round_cnt * 8192 + ptr->angle - ptr->offset_angle;
}

/*this function should be called after system+can init */
void get_moto_offset(moto_measure_t *ptr, uint8_t* Rxdate)
{
	ptr->angle = (uint16_t)(Rxdate[0]<<8 | Rxdate[1]) ;
	ptr->offset_angle = ptr->angle;
}


//电机控制任务的变量
CAN_TxHeaderTypeDef CAN1_TxHeader;//发送的句柄
uint32_t WhichMailBox;//告知从哪个邮箱发出
uint8_t CanSendBuff[8];
void set_moto_current(CAN_HandleTypeDef* hcan, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4)
{

	CAN1_TxHeader.DLC=8;//写数据长度
	CAN1_TxHeader.IDE=CAN_ID_STD;//标准帧
	CAN1_TxHeader.RTR=CAN_RTR_DATA;//数据帧
	CAN1_TxHeader.StdId=0x200;//发送ID
	CAN1_TxHeader.ExtId=0x0012;
	
	CanSendBuff[0]=iq1 >> 8;
	CanSendBuff[1]=iq1;
	CanSendBuff[2]=iq2 >> 8;
	CanSendBuff[3]=iq2;
	CanSendBuff[4]=iq3 >> 8;
	CanSendBuff[5]=iq3;
	CanSendBuff[6]=iq4 >> 8;
	CanSendBuff[7]=iq4;
	HAL_CAN_AddTxMessage(hcan,&CAN1_TxHeader,CanSendBuff,&WhichMailBox);

	
}	




