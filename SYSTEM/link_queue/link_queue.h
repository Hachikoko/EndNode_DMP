#ifndef _LINK_QUEUE_H_
#define _LINK_QUEUE_H_

#include "stm32f4xx.h"

typedef struct {
	short acc_x;
	short acc_y;
	short acc_z;
	
	short gyr_x;
	short gyr_y;
	short gyr_z;
	
	short mag_x;
	short mag_y;
	short mag_z;
	
	short acc_line_x;
	short acc_line_y;
	short acc_line_z;
	
	short q0;
	short q1;
	short q2;
	short q3;
	
	short euler_x;
	short euler_y;
	short euler_z;
} NODE_DATA;

typedef struct node{
	NODE_DATA data;
	struct node * next;
}Queue_Node;

typedef struct link_queue{
	struct node * front;
	struct node * rear;
	u32 length;
}Link_Queue;


Link_Queue* creat_link_queue(void);
Queue_Node* creat_link_node(void);
u8 pop_node(Link_Queue* link_queue);
u8 push_back_node(Link_Queue* link_queue,NODE_DATA* ptr_data);


#endif

