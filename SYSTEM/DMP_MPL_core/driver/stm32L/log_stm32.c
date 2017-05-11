/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.
 $
 */
/*******************************************************************************
 * $Id: $
 *******************************************************************************/

/**
 *  @defgroup MSP430_System_Layer MSP430 System Layer
 *  @brief  MSP430 System Layer APIs.
 *          To interface with any platform, eMPL needs access to various
 *          system layer functions.
 *
 *  @{
 *      @file   log_msp430.c
 *      @brief  Logging facility for the TI MSP430.
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>

#include "packet.h"
#include "log.h"
#include "stm32f4xx.h"
#include "uart.h"

//超核上位机
#include "packet1.h"

#define BUF_SIZE        (256)
#define PACKET_LENGTH   (23)

#define PACKET_DEBUG    (1)
#define PACKET_QUAT     (2)
#define PACKET_DATA     (3)

typedef struct result_data{
	
	double euler[3];       	//欧拉角
	double q[4]; 			//四元数 
}Fliter_Result_Data;

typedef struct mpu9250_raw_data{
	short ax;			//加速度
	short ay;
	short az;
	short gx;			//陀螺仪
	short gy;
	short gz;
	short mx;			//磁力计
	short my;
	short mz;
}MPU9250_RAW_DATD;

//数据拆分宏定义，在发送大于1字节的数据类型时，比如int16、float等，需要把数据拆分成单独字节进行发送
#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)		) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )

u8 data_to_send[50];	//发送数据缓存


/*<FUNC>***********************************************************************
* 函数名称: BspFixToDou
* 功能描述: 将指定的定点数 转化为 浮点数
* 算法描述: 无
* 输入参数:  ucType 0表示无符号 1表示有符号
*			ucInteger 表示整数占几个bit
*		    ucdecimal 表示小数占几个bit
*		    llfix 为待转化的定点数
* 输出参数: 无
* 返 回 值: 无 
************************************************************************/ 

void BspFixToDou(u8 ucType, u8 ucInteger, u8 ucdecimal, u32 llfix, double *pdbRet)
{
	u32 lltemp = llfix & ((((u32)(1)<<(ucType + ucInteger + ucdecimal))-1));
	if(0 == llfix)
	{
		*pdbRet = 0.0;
	}
	if(lltemp & (((u32)(1)<<(ucInteger + ucdecimal)))) /* 有符号数并且是负数 */
	{
		*pdbRet = -(double)(((u32)(1)<<(ucType + ucInteger + ucdecimal)) - lltemp)/(double)((u32)(1)<<ucdecimal);
	}
	else 		 /* 无符号数或者有符号数的正数*/
	{
		*pdbRet = (double)((double)lltemp/(double)((u32)1<<ucdecimal));
	}
}

//Send_Data函数是协议中所有发送数据功能使用到的发送函数
//移植时，用户应根据自身应用的情况，根据使用的通信方式，实现此函数
void ANO_DT_Send_Data(u8 *dataToSend , u8 length)
{
	for(int i = 0; i < length; i++){
		Uart_send(data_to_send[i]);
	}
}

void ANO_DT_Send_Status(float angle_rol, float angle_pit, float angle_yaw, s32 alt, u8 fly_model, u8 armed)
{
	u8 _cnt=0;
	vs16 _temp;
	vs32 _temp2 = alt;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x01;
	data_to_send[_cnt++]=0;
	
	_temp = (int)(angle_rol*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(angle_pit*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(angle_yaw*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[_cnt++]=BYTE3(_temp2);
	data_to_send[_cnt++]=BYTE2(_temp2);
	data_to_send[_cnt++]=BYTE1(_temp2);
	data_to_send[_cnt++]=BYTE0(_temp2);
	
	data_to_send[_cnt++] = fly_model;
	
	data_to_send[_cnt++] = armed;
	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}

/**
 *  @brief      Prints a variable argument log message.
 *  USB output will be formatted as follows:\n
 *  packet[0]       = $\n
 *  packet[1]       = packet type (1: debug, 2: quat, 3: data)\n
 *  packet[2]       = \n for debug packets: log priority\n
 *                    for quaternion packets: unused\n
 *                    for data packets: packet content (accel, gyro, etc)\n
 *  packet[3-20]    = data\n
 *  packet[21]      = \\r\n
 *  packet[22]      = \\n
 *  @param[in]  priority    Log priority (based on Android).
 *  @param[in]  tag         File specific string.
 *  @param[in]  fmt         String of text with optional format tags.
 *
 *  @return     0 if successful.
 */
int _MLPrintLog (int priority, const char* tag, const char* fmt, ...)
{
    va_list args;
    int length, ii, i;
    char buf[BUF_SIZE], out[PACKET_LENGTH], this_length;

    /* This can be modified to exit for unsupported priorities. */
    switch (priority) {
    case MPL_LOG_UNKNOWN:
    case MPL_LOG_DEFAULT:
    case MPL_LOG_VERBOSE:
    case MPL_LOG_DEBUG:
    case MPL_LOG_INFO:
    case MPL_LOG_WARN:
    case MPL_LOG_ERROR:
    case MPL_LOG_SILENT:
        break;
    default:
        return 0;
    }

    va_start(args, fmt);

    length = vsprintf(buf, fmt, args);
    if (length <= 0) {
        va_end(args);
        return length;
    }

    memset(out, 0, PACKET_LENGTH);
    out[0] = '$';
    out[1] = PACKET_DEBUG;
    out[2] = priority;
    out[21] = '\r';
    out[22] = '\n';
    for (ii = 0; ii < length; ii += (PACKET_LENGTH-5)) {
#define min(a,b) ((a < b) ? a : b)
        this_length = min(length-ii, PACKET_LENGTH-5);
        memset(out+3, 0, 18);
        memcpy(out+3, buf+ii, this_length);
        for (i=0; i<PACKET_LENGTH; i++) {
          Uart_send(out[i]);
        }
    }
    
            
    va_end(args);

    return 0;
}
static int index = 0;

void eMPL_send_quat(long *quat)
{
    char out[PACKET_LENGTH];
	char char_data[100];
	char * temp = char_data;
    int i,n;
	double q0,q1,q2,q3;
    if (!quat)
        return;
    memset(out, 0, PACKET_LENGTH);
	memset(char_data, 0, 100);
	BspFixToDou(1, 1, 30, quat[0], &q0);
	BspFixToDou(1, 1, 30, quat[1], &q1);
	BspFixToDou(1, 1, 30, quat[2], &q2);
	BspFixToDou(1, 1, 30, quat[3], &q3);
//	sprintf(char_data,"q0:%f,q1:%f,q2:%f,q3:%f;\r\n",q0,q1,q2,q3);
//	n = strlen(char_data);
	n = 32;
	temp[0] = 'D';
	temp += 1;
	*(int*)(temp) = index++;
	temp += 4;
	temp[0] = 11;
	temp += 1;
	*(short*)temp = 0;
	temp += 2;
	*(short*)temp = 0;
	temp += 2;
	*(short*)temp = 0;
	temp += 2;
	
	*(short*)temp = 0;
	temp += 2;
	*(short*)temp = 0;
	temp += 2;
	*(short*)temp = 0;
	temp += 2;
	
	*(short*)temp = 0;
	temp += 2;
	*(short*)temp = 0;
	temp += 2;
	*(short*)temp = 0;
	temp += 2;
	
	*(short*)temp = (short)(q0 *10000.0f);
	temp += 2;
	*(short*)temp = (short)(q1 *10000.0f);
	temp += 2;
	*(short*)temp = (short)(q2 *10000.0f);
	temp += 2;
	*(short*)temp = (short)(q3 *10000.0f);
	
	for (i=0; i<n; i++) {
      Uart_send(char_data[i]);
    }
	
//	sprintf(char_data,"q0:%f,q1:%f,q2:%f,q3:%f;\r\n",q0,q1,q2,q3);
//	n = strlen(char_data);
//	for (i=0; i<n; i++) {
//      Uart_send(char_data[i]);
//    }
	
//    out[0] = '$';
//    out[1] = PACKET_QUAT;
//    out[3] = (char)(quat[0] >> 24);
//    out[4] = (char)(quat[0] >> 16);
//    out[5] = (char)(quat[0] >> 8);
//    out[6] = (char)quat[0];
//    out[7] = (char)(quat[1] >> 24);
//    out[8] = (char)(quat[1] >> 16);
//    out[9] = (char)(quat[1] >> 8);
//    out[10] = (char)quat[1];
//    out[11] = (char)(quat[2] >> 24);
//    out[12] = (char)(quat[2] >> 16);
//    out[13] = (char)(quat[2] >> 8);
//    out[14] = (char)quat[2];
//    out[15] = (char)(quat[3] >> 24);
//    out[16] = (char)(quat[3] >> 16);
//    out[17] = (char)(quat[3] >> 8);
//    out[18] = (char)quat[3];
//    out[21] = '\r';
//    out[22] = '\n';
    
//    for (i=0; i<PACKET_LENGTH; i++) {
//      Uart_send(out[i]);
//    }
}

void eMPL_send_data(unsigned char type, long *data)
{
    char out[PACKET_LENGTH];
	char char_data[100];
	double x,y,z;
    int i,n;
    if (!data)
        return;
    memset(out, 0, PACKET_LENGTH);
    out[0] = '$';
    out[1] = PACKET_DATA;
    out[2] = type;
    out[21] = '\r';
    out[22] = '\n';
    switch (type) {
    /* Two bytes per-element. */
    case PACKET_DATA_ROT:
        out[3] = (char)(data[0] >> 24);
        out[4] = (char)(data[0] >> 16);
        out[5] = (char)(data[1] >> 24);
        out[6] = (char)(data[1] >> 16);
        out[7] = (char)(data[2] >> 24);
        out[8] = (char)(data[2] >> 16);
        out[9] = (char)(data[3] >> 24);
        out[10] = (char)(data[3] >> 16);
        out[11] = (char)(data[4] >> 24);
        out[12] = (char)(data[4] >> 16);
        out[13] = (char)(data[5] >> 24);
        out[14] = (char)(data[5] >> 16);
        out[15] = (char)(data[6] >> 24);
        out[16] = (char)(data[6] >> 16);
        out[17] = (char)(data[7] >> 24);
        out[18] = (char)(data[7] >> 16);
        out[19] = (char)(data[8] >> 24);
        out[20] = (char)(data[8] >> 16);
        break;
    /* Four bytes per-element. */
    /* Four elements. */
    case PACKET_DATA_QUAT:
        out[15] = (char)(data[3] >> 24);
        out[16] = (char)(data[3] >> 16);
        out[17] = (char)(data[3] >> 8);
        out[18] = (char)data[3];
    /* Three elements. */
    case PACKET_DATA_ACCEL:
    case PACKET_DATA_GYRO:
    case PACKET_DATA_COMPASS:
    case PACKET_DATA_EULER:   //注意: pitch:data[0] roll:data[1] yaw:data[2]
		
		BspFixToDou(1, 15, 16, data[0], &x);
		BspFixToDou(1, 15, 16, data[1], &y);
		BspFixToDou(1, 15, 16, data[2], &z);

		ANO_DT_Send_Status(y, x, z, 10000, 1, 0);
//		sprintf(char_data,"x:%f,y:%f,z:%f;\r\n",x,y,z);
//		n = strlen(char_data);
//		for (i=0; i<n; i++) {
//			Uart_send(char_data[i]);
//		}
		
        out[3] = (char)(data[0] >> 24);
        out[4] = (char)(data[0] >> 16);
        out[5] = (char)(data[0] >> 8);
        out[6] = (char)data[0];
        out[7] = (char)(data[1] >> 24);
        out[8] = (char)(data[1] >> 16);
        out[9] = (char)(data[1] >> 8);
        out[10] = (char)data[1];
        out[11] = (char)(data[2] >> 24);
        out[12] = (char)(data[2] >> 16);
        out[13] = (char)(data[2] >> 8);
        out[14] = (char)data[2];
        break;
    case PACKET_DATA_HEADING:
        out[3] = (char)(data[0] >> 24);
        out[4] = (char)(data[0] >> 16);
        out[5] = (char)(data[0] >> 8);
        out[6] = (char)data[0];
        break;
    default:
        return;
    }
//    for (i=0; i<PACKET_LENGTH; i++) {
//      Uart_send(out[i]);
//    }
}

void Uranus_send(long*alte){
	char data_bef_to_send[60];
	double x,y,z;
	short s_x,s_y,s_z;
	uint16_t currectCrc;
	char*temp;
	BspFixToDou(1, 15, 16, alte[0], &x);//注意: pitch:data[0] roll:data[1] yaw:data[2]
	BspFixToDou(1, 15, 16, alte[1], &y);
	BspFixToDou(1, 15, 16, alte[2], &z);
	
	s_x = (short)(100 * x);
	s_y = (short)(100 * y);
	s_z = (short)(10 * z);
		
	
	data_bef_to_send[0] = 0x5A;
	data_bef_to_send[1] = 0xA5;
	data_bef_to_send[2] = 35;
	data_bef_to_send[3] = 0;
	data_bef_to_send[4] = 0x90;
	data_bef_to_send[5] = 0x00;
	data_bef_to_send[6] = 0xA0;
	data_bef_to_send[7] = 0; 
	data_bef_to_send[8] = 0; 
	data_bef_to_send[9] = 0; 
	data_bef_to_send[10] = 0; 
	data_bef_to_send[11] = 0; 
	data_bef_to_send[12] = 0; 
	
	data_bef_to_send[13] = 0xB0;
	data_bef_to_send[14] = 0; 
	data_bef_to_send[15] = 0; 
	data_bef_to_send[16] = 0; 
	data_bef_to_send[17] = 0; 
	data_bef_to_send[18] = 0; 
	data_bef_to_send[19] = 0;
	
	data_bef_to_send[20] = 0xC0;
	data_bef_to_send[21] = 0; 
	data_bef_to_send[22] = 0; 
	data_bef_to_send[23] = 0; 
	data_bef_to_send[24] = 0; 
	data_bef_to_send[25] = 0; 
	data_bef_to_send[26] = 0;
	
	data_bef_to_send[27] = 0xD0;
	*((short*)(data_bef_to_send+28)) = s_y;
	*((short*)(data_bef_to_send+30)) = s_x;
	*((short*)(data_bef_to_send+32)) = s_z;
	
	

	data_bef_to_send[34] = 0xF0;
	*((int*)(data_bef_to_send+35)) = 0;
	
	crc16_update(&currectCrc,data_bef_to_send,39);

	data_bef_to_send[0] = 0x5A;
	data_bef_to_send[1] = 0xA5;
	data_bef_to_send[2] = 35;
	data_bef_to_send[3] = 0;
	data_bef_to_send[4] = *((char*)&currectCrc);
	data_bef_to_send[5] = *(((char*)&currectCrc) + 1);
	data_bef_to_send[6] = 0x90;
	data_bef_to_send[7] = 0x00;
	data_bef_to_send[8] = 0xA0;
	data_bef_to_send[9] = 0; 
	data_bef_to_send[10] = 0; 
	data_bef_to_send[11] = 0; 
	data_bef_to_send[12] = 0; 
	data_bef_to_send[13] = 0; 
	data_bef_to_send[14] = 0; 
	
	data_bef_to_send[15] = 0xB0;
	data_bef_to_send[16] = 0; 
	data_bef_to_send[17] = 0; 
	data_bef_to_send[18] = 0; 
	data_bef_to_send[19] = 0; 
	data_bef_to_send[20] = 0; 
	data_bef_to_send[21] = 0;
	
	data_bef_to_send[22] = 0xC0;
	data_bef_to_send[23] = 0; 
	data_bef_to_send[24] = 0; 
	data_bef_to_send[25] = 0; 
	data_bef_to_send[26] = 0; 
	data_bef_to_send[27] = 0; 
	data_bef_to_send[28] = 0;
	
	data_bef_to_send[29] = 0xD0;
	*((short*)(data_bef_to_send+30)) = s_y;
	*((short*)(data_bef_to_send+32)) = s_x;
	*((short*)(data_bef_to_send+34)) = s_z;
	
	
	data_bef_to_send[36] = 0xF0;
	*((int*)(data_bef_to_send+37)) = 0;
	
	for (int i=0; i<41; i++) {
      Uart_send(data_bef_to_send[i]);
    }
}

/**
 * @}
**/


