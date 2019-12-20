/**
 ***************************************(C) COPYRIGHT 2019 USTC**************************************
 * @file       bsp_uart.c
 * @brief      this file contains rc data receive and processing function
 * @note       
 * @Version    V1.0.0
 * @Date       
 ***************************************(C) COPYRIGHT 2019 USTC***************************************
 */
                                                                                                              
#include "string.h"
#include "stdlib.h"
#include "bsp_uart.h"
#include "usart.h"
uint8_t   dbus_buf[DBUS_BUFLEN];
RC_Ctl_t RC_DATE;

/**
  * @brief       handle received rc data
  * @param[out]  rc:   structure to save handled rc data
  * @param[in]   buff: the buff which saved raw rc data
  * @retval 
  */
void rc_callback_handler(uint8_t *buff)
{
  RC_DATE.rc_ch0 = ((uint16_t) buff[0] | (uint16_t)buff[1] <<8) & 0x07FF;
	RC_DATE.rc_ch0-=1024;
  RC_DATE.rc_ch1 = ((uint16_t)buff[1] >>3 | (uint16_t)buff[2] <<5) & 0x07FF;
	RC_DATE.rc_ch1-=1024;
  RC_DATE.rc_ch2 = ((uint16_t)buff[2] >>6 | (uint16_t)buff[3] <<2 | (uint16_t)buff[4] <<10) & 0x07FF;
	RC_DATE.rc_ch2-=1024;
  RC_DATE.rc_ch3 = ((uint16_t)buff[4] >>1 | (uint16_t)buff[5] <<7) & 0x07FF;
	RC_DATE.rc_ch3-=1024;
	
  RC_DATE.rc_s1 = ((buff[5] >> 4) & 0x000C) >> 2;
  RC_DATE.rc_s2 = (buff[5] >> 4) & 0x0003;

	RC_DATE.mouse_x = ((int16_t)buff[6]) | ((int16_t)buff[7] << 8);
	RC_DATE.mouse_y = ((int16_t)buff[8]) | ((int16_t)buff[9] << 8);
	RC_DATE.mouse_z = ((int16_t)buff[10]) | ((int16_t)buff[11] << 8);
	RC_DATE.mouse_press_l = buff[12];
	RC_DATE.mouse_press_r = buff[13];
	RC_DATE.key_v = ((int16_t)buff[14]);// | ((int16_t)pData[15] << 8);
	//HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2,RC_DATE->rc->ch0>0?1:0);
	//如果数值超界，则舍弃
	if ((abs(RC_DATE.rc_ch0) > 660) || \
      (abs(RC_DATE.rc_ch1) > 660) || \
      (abs(RC_DATE.rc_ch2) > 660) || \
      (abs(RC_DATE.rc_ch3) > 660))
  {
    //memset(&RC_DATE, 0, sizeof(RC_Ctl_t));
		return;
  }
}

#define max_speed 5000
void cal_speed(int16_t sped[])
{
//遥控器数据左右方向是ch0,前后方向是ch1，航向CH2
	int16_t vty,vtz,oumiga;
	oumiga=RC_DATE.rc_ch2;
	vty=RC_DATE.rc_ch0;
	vtz=RC_DATE.rc_ch1;
	
		sped[0]=10*(oumiga+vty-vtz);//-660 ~ 660   -16384 ~ 16384
		sped[1]=10*(oumiga+vty+vtz);
		sped[2]=10*(oumiga-vty+vtz);
		sped[3]=10*(oumiga-vty-vtz);
	
	if(sped[0]>max_speed)sped[0]=max_speed;
	if(sped[1]>max_speed)sped[1]=max_speed;
	if(sped[2]>max_speed)sped[2]=max_speed;
	if(sped[3]>max_speed)sped[3]=max_speed;
	if(sped[0]<-max_speed)sped[0]=-max_speed;
	if(sped[1]<-max_speed)sped[1]=-max_speed;
	if(sped[2]<-max_speed)sped[2]=-max_speed;
	if(sped[3]<-max_speed)sped[3]=-max_speed;

}
/**
  * @brief      clear idle it flag after uart receive a frame data
  * @param[in]  huart: uart IRQHandler id
  * @retval  
  */
static void uart_rx_idle_callback(UART_HandleTypeDef* huart)
{
	/* clear idle it flag avoid idle interrupt all the time */
	__HAL_UART_CLEAR_IDLEFLAG(huart);

	/* handle received data in idle interrupt */
	if (huart == &DBUS_HUART)
	{
		/* handle dbus data dbus_buf from DMA */
		if ((uint8_t)(huart->pRxBuffPtr-dbus_buf) == DBUS_BUFLEN)
		{
			rc_callback_handler(dbus_buf);	
		}
		
		huart->pRxBuffPtr=dbus_buf;
	}
}

/**
  * @brief      callback this function when uart interrupt 
  * @param[in]  huart: uart IRQHandler id
  * @retval  
  */
void uart_receive_handler(UART_HandleTypeDef *huart)
{
	if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) && 
			__HAL_UART_GET_IT_SOURCE(huart, UART_IT_IDLE))
	{
		uart_rx_idle_callback(huart);	
		HAL_UART_Receive_IT(&DBUS_HUART,dbus_buf,DBUS_MAX_LEN);
	}
	
}

/**
  * @brief   initialize dbus uart device 
  * @param   
  * @retval  
  */
void dbus_uart_init(void)
{
	/* open uart idle it */
	__HAL_UART_CLEAR_IDLEFLAG(&DBUS_HUART);
	__HAL_UART_ENABLE_IT(&DBUS_HUART, UART_IT_IDLE|UART_IT_RXNE);
	HAL_UART_Receive_IT(&DBUS_HUART,dbus_buf,DBUS_MAX_LEN);
	//uart_receive_dma_no_it(&DBUS_HUART, dbus_buf, DBUS_MAX_LEN);
}
