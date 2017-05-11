
/**********************************************************************
File    : uart.h
Purpose : 
**********************************************************************/
#ifndef __UART_H__
#define __UART_H__
/****************************** Includes *****************************/
/****************************** Defines *******************************/
/***************************** Prototypes *****************************/
void USART_Config(void);
int Uart_send ( int ch );      // Primary UART for QUAT/ACCEL/GYRO data
int Uart_string_send(const char * src);

#endif // __UART_H__


