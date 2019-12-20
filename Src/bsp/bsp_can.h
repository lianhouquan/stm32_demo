/**
 ***************************************(C) COPYRIGHT 2019 USTC**************************************
 * @file       bsp_can.h
 * @brief      this file contains sd card basic operating function
 * @note         
 * @Version    V1.0.0
 * @Date             
 ***************************************(C) COPYRIGHT 2019 USTC**************************************
 */
  
#ifndef _BSP__CAN_H
#define _BSP__CAN_H

#include "stm32f4xx_HAL.h"


#define MOTO_CAN_RX_ID       (0x259)
#define MOTO_RX_FIFO_SIZE    (22)
#define MOTO_TX_FIFO_SIZE    (22)

//���ֵ����CAN  ����ID
typedef enum
{
	//add by langgo
	CAN_3510Moto_ALL_ID = 0x200,
	CAN_3510Moto1_ID = 0x201,
	CAN_3510Moto2_ID = 0x202,
	CAN_3510Moto3_ID = 0x203,
	CAN_3510Moto4_ID = 0x204,

}CAN_Message_ID;


/*���յ�����̨����Ĳ����ṹ��*/
typedef struct{
	int16_t	 	speed_rpm;  //���ת��
  int16_t  	real_current; 
  int16_t  	given_current;//�������ĵ���
  uint8_t  	hall;					//����¶�
	uint16_t 	angle;				//abs angle range:[0,8191]
	uint16_t 	last_angle;	//abs angle range:[0,8191]
	uint16_t	offset_angle;
	int32_t		round_cnt;
	int32_t		total_angle;
	uint32_t			msg_cnt;
}moto_measure_t;


extern moto_measure_t moto_chassis[8];	//��¼�ĸ����̵����Ϣ
//��������

//��ʼ��
HAL_StatusTypeDef can_filter_init(CAN_HandleTypeDef* hcan);

//��ȡ��Ϣ
void get_moto_offset(moto_measure_t *ptr, uint8_t* Rxdate);
void get_moto_measure(moto_measure_t *ptr, uint8_t* Rxdate);

//�������
void set_moto_current(CAN_HandleTypeDef* hcan, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);
#endif
