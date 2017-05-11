#include "MPU9250.h"
#include "IIC.h"
#include "delay.h"
#include "usart.h"
#include "main.h"
#include "sys.h"

extern char ret_words[100];

u8 MPU9250_Init(void){
	u8 res_MPU,res_AKM;
	char ID[20];
	IIC_Init();
	MPU_Write_Byte(MPU9250_PWR_MGMT_1,0X80);	//复位MPU9250
	delay_ms(100);
	MPU_Write_Byte(MPU9250_PWR_MGMT_1,0X00);	//唤醒MPU6050 
	MPU_Set_Gyro_Fsr(MPU9250_GYRO_FULL_SCALE_500DPS);			//陀螺仪传感器,±500dps
	MPU_Set_Accel_Fsr(MPU9250_FULL_SCALE_4G);					//加速度传感器,±2g
	MPU_Write_Byte(MPU9250_INT_ENABLE,0X00);	//关闭所有中断
	MPU_Write_Byte(MPU9250_USER_CTRL,0X00);	//I2C主模式关闭
	MPU_Write_Byte(MPU9250_FIFO_EN,0X00);	//关闭FIFO
	MPU_Write_Byte(MPU9250_INT_PIN_CFG,0X82);	//INT引脚低电平有效 开启bypass IIC
	MPU_Write_Byte(MPU9250_PWR_MGMT_1,0X01);	//设置CLKSEL,PLL X轴为参考
	MPU_Write_Byte(MPU9250_PWR_MGMT_2,0X00);	//加速度与陀螺仪都工作
	AKM_Write_Byte(MPU9250_MAG_RSV,0x01);
	AKM_Write_Byte(MPU9250_MAG_CNTL,0x12);		//电子罗盘工作
	MPU_Set_Rate(800);						//设置采样率为50Hz
	res_MPU = MPU_Read_Byte(MPU9250_WHO_AM_I);
	res_AKM = AKM_Read_Byte(MPU9250_MAG_WIA);
#ifdef TEST_MPU
	sprintf(ID,"%x,%x\r\n",res_MPU,res_AKM);
	Uart1_SendString((u8*)ID);
#endif
	if(res_MPU == MPU9250_ID && res_AKM == MPU9250_AKM_ID)//器件ID正确
	{
		return 0;
 	}else return 1;
}

//void MPU_Get_9_Axis(MPU9250_DATD * mpu_data){
//	u8 ret = 0;
//	
//	int temp_data;
//	int full_data = 32768;
//	MPU9250_RAW_DATD mpu_raw_data;
//	
//	ret = MPU_Get_Accelerometer(&(mpu_raw_data.ax),&(mpu_raw_data.ay),&(mpu_raw_data.az));
//	if(ret != 0){
//		Uart1_SendString((u8*)"get accelerometer fail!\r\n");
//		return;
//	}
//	
//	ret = MPU_Get_Gyroscope(&(mpu_raw_data.gx),&(mpu_raw_data.gy),&(mpu_raw_data.gz));
//	if(ret != 0){
//		Uart1_SendString((u8*)"get gyroscope fail!\r\n");
//		return;
//	}
//	
//	ret = MPU_Get_Gyroscope(&(mpu_raw_data.mx),&(mpu_raw_data.my),&(mpu_raw_data.mz));
//	if(ret != 0){
//		Uart1_SendString((u8*)"get gyroscope fail!\r\n");
//		return;
//	}
//	
//	temp_data = mpu_raw_data.ax;
//	mpu_data->ax = (((float)temp_data)/((float)full_data)) * 4.0;
//	temp_data = mpu_raw_data.ay;
//	mpu_data->ay = (((float)temp_data)/((float)full_data)) * 4.0;
//	temp_data = mpu_raw_data.az;
//	mpu_data->az = (((float)temp_data)/((float)full_data)) * 4.0;
//	
//	temp_data = mpu_raw_data.gx;
//	mpu_data->gx = (((float)temp_data)/((float)full_data)) * 500.0;
//	temp_data = mpu_raw_data.gy;
//	mpu_data->gy = (((float)temp_data)/((float)full_data)) * 500.0;
//	temp_data = mpu_raw_data.gz;
//	mpu_data->gz = (((float)temp_data)/((float)full_data)) * 500.0;
//	
//	mpu_data->mx = mpu_raw_data.mx * 0.15;
//	mpu_data->my = mpu_raw_data.my * 0.15;
//	mpu_data->mz = mpu_raw_data.mz * 0.15;
//	
//}

u8 MPU_get_ACC_offset(MPU9250_RAW_DATD* acc_offset){
	u8 buf[6],ret;
	ret = MPU_Read_Len(MPU9250_ADDR,MPU9250_ACCEL_XOUT_H,6,buf);
	
}
u8 MPU_get_GYR_offset(MPU9250_RAW_DATD* gry_offset){
	u8 buf[6],ret;
}

void MPU_Get_9_Axis_Raw(MPU9250_RAW_DATD * mpu_raw_data){
	u8 ret = 0;
	
	ret = MPU_Get_Accelerometer(&(mpu_raw_data->ax),&(mpu_raw_data->ay),&(mpu_raw_data->az));
	if(ret != 0){
		Uart1_SendString((u8*)"get accelerometer fail!\r\n");
		return;
	}
	
	ret = MPU_Get_Gyroscope(&(mpu_raw_data->gx),&(mpu_raw_data->gy),&(mpu_raw_data->gz));
	if(ret != 0){
		Uart1_SendString((u8*)"get gyroscope fail!\r\n");
		return;
	}
	
	ret = MPU_Get_Magnetometer(&(mpu_raw_data->mx),&(mpu_raw_data->my),&(mpu_raw_data->mz));
	if(ret != 0){
		Uart1_SendString((u8*)"get magnetometer fail!\r\n");
		return;
	}
	
}

//得到加速度值(原始值)
//gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
u8 MPU_Get_Accelerometer(short *ax,short *ay,short *az)
{
    u8 buf[6],res;  
	res=MPU_Read_Len(MPU9250_ADDR,MPU9250_ACCEL_XOUT_H,6,buf);
	if(res==0)
	{
		*ax=((u16)buf[0]<<8)|buf[1];  
		*ay=((u16)buf[2]<<8)|buf[3];  
		*az=((u16)buf[4]<<8)|buf[5];
	} 	
    return res;;
}


//得到陀螺仪值(原始值)
//gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
u8 MPU_Get_Gyroscope(short *gx,short *gy,short *gz)
{
    u8 buf[6],res;  
	res=MPU_Read_Len(MPU9250_ADDR,MPU9250_GYRO_XOUT_H,6,buf);
	if(res==0)
	{
		*gx=((u16)buf[0]<<8)|buf[1];  
		*gy=((u16)buf[2]<<8)|buf[3];  
		*gz=((u16)buf[4]<<8)|buf[5];
	} 	
    return res;;
}

//得到电子罗盘值(原始值)
//mx,my,mz:电子罗盘x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
u8 MPU_Get_Magnetometer (short *mx,short *my,short *mz)
{
    u8 buf[6],res;
	u8 x_axis,y_axis,z_axis; 
	
	x_axis = AKM_Read_Byte(MPU9250_MAG_ASAX);
	y_axis = AKM_Read_Byte(MPU9250_MAG_ASAY);
	z_axis = AKM_Read_Byte(MPU9250_MAG_ASAZ);
	
	#ifdef TEST_MPU
	sprintf(ret_words,"x_axis:%d,y_axis:%d,z_axis:%d\r\n",x_axis,y_axis,z_axis);
	Uart1_SendString((u8*)ret_words);
	#endif
	
	if(AKM_Read_Byte(MPU9250_MAG_ST1) == MPU9250_MAG_DATA_IS_READY){
		buf[0]= AKM_Read_Byte(MPU9250_MAG_XOUT_L);
		if(AKM_Read_Byte(MPU9250_MAG_ST2) != MPU9250_MAG_NO_HOFL){
			res =  1;
		}
		
		buf[1]= AKM_Read_Byte(MPU9250_MAG_XOUT_H);
		if(AKM_Read_Byte(MPU9250_MAG_ST2) != MPU9250_MAG_NO_HOFL){
			res =  1;
		}
		
		buf[2]= AKM_Read_Byte(MPU9250_MAG_YOUT_L);
		if(AKM_Read_Byte(MPU9250_MAG_ST2) != MPU9250_MAG_NO_HOFL){
			res =  1;
		}
				
		buf[3]= AKM_Read_Byte(MPU9250_MAG_YOUT_H);
		if(AKM_Read_Byte(MPU9250_MAG_ST2) != MPU9250_MAG_NO_HOFL){
			res =  1;
		}
		
		buf[4]= AKM_Read_Byte(MPU9250_MAG_ZOUT_L);
		if(AKM_Read_Byte(MPU9250_MAG_ST2) != MPU9250_MAG_NO_HOFL){
			res =  1;
		}
		
		buf[5]= AKM_Read_Byte(MPU9250_MAG_ZOUT_H);
		if(AKM_Read_Byte(MPU9250_MAG_ST2) != MPU9250_MAG_NO_HOFL){
			res =  1;
		}
		
		if (1 == res){
			return res;
		}
		
		*mx=((u16)buf[1]<<8)|buf[0];  
		*mx *= (((x_axis - 128) >> 8) + 1);
		*my=((u16)buf[3]<<8)|buf[2];  
		*my *= (((y_axis - 128) >> 8) + 1);
		*mz=((u16)buf[5]<<8)|buf[4];
		*mz *= (((z_axis - 128) >> 8) + 1);
	}


	
	#ifdef TEST_MPU
	sprintf(ret_words,"mx:%d,my:%d,mz:%d\r\n",*mx,*my,*mz);
	Uart1_SendString((u8*)ret_words);
	#endif

    return res;
}

//设置MPU6050的数字低通滤波器
//lpf:数字低通滤波频率(Hz)
//返回值:0,设置成功
//    其他,设置失败 
u8 MPU_Set_LPF(u16 lpf)
{
	u8 data=0;
	if(lpf>=188)data=1;
	else if(lpf>=98)data=2;
	else if(lpf>=42)data=3;
	else if(lpf>=20)data=4;
	else if(lpf>=10)data=5;
	else data=6; 
	return MPU_Write_Byte(MPU9250_CONFIG,data);//设置数字低通滤波器  
}
//设置MPU6050的采样率(假定Fs=1KHz)
//rate:4~1000(Hz)
//返回值:0,设置成功
//    其他,设置失败 
u8 MPU_Set_Rate(u16 rate)
{
	u8 data;
	if(rate>1000)rate=1000;
	if(rate<4)rate=4;
	data=1000/rate-1;
	data=MPU_Write_Byte(MPU9250_SMPLRT_DIV,data);	//设置数字低通滤波器
 	return MPU_Set_LPF(rate/2);	//自动设置LPF为采样率的一半
}


u8 MPU_Set_Gyro_Fsr(u8 fsr)
{
	return MPU_Write_Byte(MPU9250_GYRO_CONFIG,fsr<<3);//设置陀螺仪满量程范围  
}

//设置MPU6050加速度传感器满量程范围
//fsr:0,±2g;1,±4g;2,±8g;3,±16g
//返回值:0,设置成功
//    其他,设置失败 
u8 MPU_Set_Accel_Fsr(u8 fsr)
{
	return MPU_Write_Byte(MPU9250_ACCEL_CONFIG,fsr<<3);//设置加速度传感器满量程范围  
}

//IIC读罗盘一个字节 
//reg:寄存器地址 
//返回值:读到的数据
u8 AKM_Read_Byte(u8 reg)
{
	u8 res;
    IIC_Start(); 
	IIC_Send_Byte((MPU9250_MAG_ADDRESS<<1)|0);//发送器件地址+写命令	
	IIC_Wait_Ack();		//等待应答 
    IIC_Send_Byte(reg);	//写寄存器地址
    IIC_Wait_Ack();		//等待应答
    IIC_Start();
	IIC_Send_Byte((MPU9250_MAG_ADDRESS<<1)|1);//发送器件地址+读命令	
    IIC_Wait_Ack();		//等待应答 
	res=IIC_Read_Byte(0);//读取数据,发送nACK 
    IIC_Stop();			//产生一个停止条件 
	return res;		
}

//IIC写罗盘一个字节 
//reg:寄存器地址
//data:数据
//返回值:0,正常
//    其他,错误代码
u8 AKM_Write_Byte(u8 reg,u8 data) 				 
{ 
    IIC_Start(); 
	IIC_Send_Byte((MPU9250_MAG_ADDRESS<<1)|0);//发送器件地址+写命令	
	if(IIC_Wait_Ack())	//等待应答
	{
		IIC_Stop();		 
		return 1;		
	}
    IIC_Send_Byte(reg);	//写寄存器地址
    IIC_Wait_Ack();		//等待应答 
	IIC_Send_Byte(data);//发送数据
	if(IIC_Wait_Ack())	//等待ACK
	{
		IIC_Stop();	 
		return 1;		 
	}		 
    IIC_Stop();	 
	return 0;
}


//IIC读一个字节 
//reg:寄存器地址 
//返回值:读到的数据
u8 MPU_Read_Byte(u8 reg)
{
	u8 res;
    IIC_Start(); 
	IIC_Send_Byte((MPU9250_ADDR<<1)|0);//发送器件地址+写命令	
	IIC_Wait_Ack();		//等待应答 
    IIC_Send_Byte(reg);	//写寄存器地址
    IIC_Wait_Ack();		//等待应答
    IIC_Start();
	IIC_Send_Byte((MPU9250_ADDR<<1)|1);//发送器件地址+读命令	
    IIC_Wait_Ack();		//等待应答 
	res=IIC_Read_Byte(0);//读取数据,发送nACK 
    IIC_Stop();			//产生一个停止条件 
	return res;		
}



//IIC连续读
//addr:器件地址
//reg:要读取的寄存器地址
//len:要读取的长度
//buf:读取到的数据存储区
//返回值:0,正常
//    其他,错误代码
u8 MPU_Read_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{ 
 	IIC_Start(); 
	IIC_Send_Byte((addr<<1)|0);//发送器件地址+写命令	
	if(IIC_Wait_Ack())	//等待应答
	{
		IIC_Stop();		 
		return 1;		
	}
    IIC_Send_Byte(reg);	//写寄存器地址
    IIC_Wait_Ack();		//等待应答
    IIC_Start();
	IIC_Send_Byte((addr<<1)|1);//发送器件地址+读命令	
    IIC_Wait_Ack();		//等待应答 
	while(len)
	{
		if(len==1)*buf=IIC_Read_Byte(0);//读数据,发送nACK 
		else *buf=IIC_Read_Byte(1);		//读数据,发送ACK  
		len--;
		buf++; 
	}    
    IIC_Stop();	//产生一个停止条件 
	return 0;	
}


//IIC写一个字节 
//reg:寄存器地址
//data:数据
//返回值:0,正常
//    其他,错误代码
u8 MPU_Write_Byte(u8 reg,u8 data) 				 
{ 
    IIC_Start(); 
	IIC_Send_Byte((MPU9250_ADDR<<1)|0);//发送器件地址+写命令	
	if(IIC_Wait_Ack())	//等待应答
	{
		IIC_Stop();		 
		return 1;		
	}
    IIC_Send_Byte(reg);	//写寄存器地址
    IIC_Wait_Ack();		//等待应答 
	IIC_Send_Byte(data);//发送数据
	if(IIC_Wait_Ack())	//等待ACK
	{
		IIC_Stop();	 
		return 1;		 
	}		 
    IIC_Stop();	 
	return 0;
}


//IIC连续写
//addr:器件地址 
//reg:寄存器地址
//len:写入长度
//buf:数据区
//返回值:0,正常
//    其他,错误代码
u8 MPU_Write_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{
	u8 i; 
    IIC_Start(); 
	IIC_Send_Byte((addr<<1)|0);//发送器件地址+写命令	
	if(IIC_Wait_Ack())	//等待应答
	{
		IIC_Stop();		 
		return 1;		
	}
    IIC_Send_Byte(reg);	//写寄存器地址
    IIC_Wait_Ack();		//等待应答
	for(i=0;i<len;i++)
	{
		IIC_Send_Byte(buf[i]);	//发送数据
		if(IIC_Wait_Ack())		//等待ACK
		{
			IIC_Stop();	 
			return 1;		 
		}		
	}    
    IIC_Stop();	 
	return 0;	
} 

//串口1发送1个字符 
//c:要发送的字符
void usart1_send_char(u8 c)
{

	USART_SendData(USART1,c); 
	while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET);	
    

} 

//传送数据给匿名四轴上位机软件(V2.6版本)
//fun:功能字. 0XA0~0XAF
//data:数据缓存区,最多28字节!!
//len:data区有效数据个数
void usart1_niming_report(u8 fun,u8*data,u8 len)
{
	u8 send_buf[32];
	u8 i;
	if(len>28)return;	//最多28字节数据 
	send_buf[len+3]=0;	//校验数置零
	send_buf[0]=0X88;	//帧头
	send_buf[1]=fun;	//功能字
	send_buf[2]=len;	//数据长度
	for(i=0;i<len;i++)send_buf[3+i]=data[i];	//复制数据
	for(i=0;i<len+3;i++)send_buf[len+3]+=send_buf[i];	//计算校验和	
	for(i=0;i<len+4;i++)usart1_send_char(send_buf[i]);	//发送数据到串口1 
}

//发送加速度传感器数据和陀螺仪数据
//aacx,aacy,aacz:x,y,z三个方向上面的加速度值
//gyrox,gyroy,gyroz:x,y,z三个方向上面的陀螺仪值
void MPU9250_send_data(short aacx,short aacy,short aacz,short gyrox,short gyroy,short gyroz,short meox,short meoy,short meoz)
{
	u8 tbuf[18]; 
	tbuf[0]=(aacx>>8)&0XFF;
	tbuf[1]=aacx&0XFF;
	tbuf[2]=(aacy>>8)&0XFF;
	tbuf[3]=aacy&0XFF;
	tbuf[4]=(aacz>>8)&0XFF;
	tbuf[5]=aacz&0XFF; 
	tbuf[6]=(gyrox>>8)&0XFF;
	tbuf[7]=gyrox&0XFF;
	tbuf[8]=(gyroy>>8)&0XFF;
	tbuf[9]=gyroy&0XFF;
	tbuf[10]=(gyroz>>8)&0XFF;
	tbuf[11]=gyroz&0XFF;
	tbuf[12]=(meox>>8)&0XFF;
	tbuf[13]=meox&0XFF;
	tbuf[14]=(meoy>>8)&0XFF;
	tbuf[15]=meoy&0XFF;
	tbuf[16]=(meoz>>8)&0XFF;
	tbuf[17]=meoz&0XFF;
	usart1_niming_report(0XA1,tbuf,18);//自定义帧,0XA1
}	
