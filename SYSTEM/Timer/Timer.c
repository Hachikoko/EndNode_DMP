#include "Timer.h"
#include "NRF24L01.h"
#include "delay.h"
#include "main.h"
#include "MPU9250.h"
#include "string.h"
#include "fliter.h"
#include "link_queue.h"
#include "uart.h"

extern u8 node_index_for_base_station;
extern u8 node_index_for_end_node;
extern Link_Queue * ptr_link_queue;
extern char ret_words[100];
extern unsigned int absolute_frame_num;

void Timer7_init(void){
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;
	

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7,ENABLE);
	
	TIM_TimeBaseInitStruct.TIM_ClockDivision =  TIM_CKD_DIV1;
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_Period = 40 * node_index_for_base_station;
	TIM_TimeBaseInitStruct.TIM_Prescaler = 8399;
	TIM_TimeBaseInit(TIM7,&TIM_TimeBaseInitStruct);          //计数器初始化
	
	NVIC_InitStruct.NVIC_IRQChannel = TIM7_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_InitStruct);
	
	TIM_ClearITPendingBit(TIM7,TIM_IT_Update); //清除中断标示位
	TIM_ITConfig(TIM7,TIM_IT_Update,ENABLE );                //中断使能
	
	TIM_Cmd(TIM7,DISABLE); 
	
	return;
}

void TIM7_IRQHandler(void){
	u8 buf[32];
	u8* temp = buf;
	NODE_DATA* ptemp = &(ptr_link_queue->front->next->data);
	memset(buf,0,32);
	if(TIM_GetITStatus(TIM7,TIM_IT_Update)==SET) //溢出中断
	{
		TIM_Cmd(TIM7,DISABLE); 
//		middle_average_filter(ptr_link_queue,&flited_result);
		buf[0] = 'D';
		buf[1] = node_index_for_end_node;
		temp += 2;
		*((unsigned int *)temp) = absolute_frame_num;
		temp += 4;
		*((short *)temp) = ptemp->acc_x;
		temp += 2;
		*((short *)temp) = ptemp->acc_y;
		temp += 2;
		*((short *)temp) = ptemp->acc_z;
		temp += 2;
		*((short *)temp) = ptemp->gyr_x;
		temp += 2;
		*((short *)temp) = ptemp->gyr_y;
		temp += 2;
		*((short *)temp) = ptemp->gyr_z;
		temp += 2;
		*((short *)temp) = ptemp->mag_x;
		temp += 2;
		*((short *)temp) = ptemp->mag_y;
		temp += 2;
		*((short *)temp) = ptemp->mag_z;
		temp += 2;
		*((short *)temp) = ptemp->q0;
		temp += 2;
		*((short *)temp) = ptemp->q1;
		temp += 2;
		*((short *)temp) = ptemp->q2;
		temp += 2;
		*((short *)temp) = ptemp->q3;
		if(TX_OK==Wireless_Send_Data(buf)){
		}else {
			Uart_string_send(" . 节点射频模块发射失败......\r\n");
		}
//		int i;
//		for(i = 0;i < 32;i++){
//			Uart_send(buf[i]);
//		}
		#ifdef TEST_MPU
//		sprintf(ret_words,"mag_x:%d,mag_y:%d,mag_z:%d,",flited_result.mx,flited_result.my,flited_result.mz);
//		Uart1_SendString((u8*)ret_words);
		#endif
		
		TIM_ClearITPendingBit(TIM7,TIM_IT_Update); //清除中断标志位
	}
}

