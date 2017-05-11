#include "sys.h"
#include "link_queue.h"
#include "stdlib.h"
#include "string.h"
#include "usart.h"

#define FLITER_RAW_N 3

u8 test_words[30];

Link_Queue* creat_link_queue(){
	Queue_Node* head = (Queue_Node*)malloc(sizeof(Queue_Node));
	memset(head,0,sizeof(Queue_Node));
	head->next = NULL;
	Link_Queue* temp_link_queue = (Link_Queue*) malloc(sizeof(Link_Queue));
	memset(temp_link_queue,0,sizeof(Link_Queue));
	temp_link_queue->front = temp_link_queue->rear = head;
	temp_link_queue->length = 1;
	return temp_link_queue;
}

Queue_Node* creat_link_node(void){
	Queue_Node* temp_node = (Queue_Node*)malloc(sizeof(Queue_Node));
	memset(temp_node,0,sizeof(Queue_Node));
	temp_node->next = NULL;
	return temp_node;
}

u8 pop_node(Link_Queue* link_queue){
	Queue_Node* temp_node = link_queue->front;
	link_queue->front = temp_node->next;
	free(temp_node);
	link_queue->length -= 1;
	
	return 0;
}

u8 push_back_node(Link_Queue* link_queue,NODE_DATA* ptr_data){
	
	Queue_Node* temp_node = (Queue_Node*)malloc(sizeof(Queue_Node));
	#ifdef TEST_LINK_QUEUE
	Uart1_SendString("after push_back_node malloc\r\n");
	#endif
	memset(temp_node,0,sizeof(Queue_Node));
	temp_node->data = *(ptr_data);
	temp_node->next = NULL;
	
	
	link_queue->rear->next = temp_node;
	link_queue->rear = temp_node;
	link_queue->length += 1;
	
	while(FLITER_RAW_N < link_queue->length){
		pop_node(link_queue);
	}
	
	#ifdef TEST_LINK_QUEUE
	sprintf((char*)test_words,"node number:%d\r\n",link_queue->length);
	Uart1_SendString(test_words);
	#endif
	
	return 0;
}
