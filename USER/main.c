#include "sys.h"
#include "delay.h"
#include "led.h"
#include "beep.h"
#include "key.h"
#include "lcd.h"
#include "hc05.h" 	 
//#include "usart3.h" 	
#include "Main.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include <string.h>
#include <stdlib.h>
#include "inv_mpu.h"
#include "mpu6050.h"
#include "arm_math.h"
#include "stdio.h"
#include "inv_mpu_dmp_motion_driver.h" 
#include "usart.h"

// data definition
#define proNum 100
#define frequence 50
#define dt 0.02f
#define capacity 256
#define peakCapacity 10
#define delta_thres 20

#define MIN(A,B) ((A)<(B) ? (A):(B))

double minPeakValue=0.25;

double bArr[9]={1.8321602336960946e-4,0.0,-7.328640934784378e-4,0.0,0.0010992961402176568,0.0,-7.328640934784378e-4,0.0,1.8321602336960946e-4};
double aArr[9]={1.0,-7.224163591308814,22.958620298726956,-41.926271928661706,48.12271256334232,-35.55142228952962,16.509171846911332,-4.406124281448874,0.5174781997880404};
u8 M=8;  // M = len(bArr) - 1; N=len(aArr) - 1
u8 N=8;
u8 flag;
u8 length,accBufBegin,accBufEnd,yawBufBegin,yawBufEnd;
float32_t accBuf[3][capacity];
float32_t yawBuf[capacity];
float32_t global_x=0,global_y=0;



typedef struct peak
{
	u8 location;       // ??
	float32_t value;       // ??
	u8 delta;    // ?????????
	u8 valid;
}Peak,*PPeak;

// function declaration
void saveYaw(unsigned char rxBuffer[]);
void saveAcc(unsigned char rxBuffer[]);
void showYaw(unsigned char rxBuffer[]);
void IIRFilter(float32_t xArr[],float32_t yArr[],u8 tmpLength);
u8 findPeaks(float32_t filteredAcc[], float32_t peaksInfo[3][peakCapacity],u8 tmpLength);
void HC05_Role_Show(void);
void HC05_Sta_Show(void);


/////////
int main(void)
{  

	u8 i;
	u8 report=1;			//默认开启上报
	u8 key;
	float32_t mean;
	float32_t peaksInfo[3][peakCapacity];
	u8 peakNum;
	float32_t yawSin[capacity],yawCos[capacity];
	float32_t distance;
	float32_t stepFreq,stepAV,stepLength;
	u8 posStart, posEnd;
	float32_t yawSinMean,yawCosMean;
	float32_t degree;
	float32_t accNorm[capacity];
	float32_t accNormFiltered[capacity];
	short aacx,aacy,aacz;		//加速度传感器原始数据
	short tempx,tempy,tempz;
	short gyrox,gyroy,gyroz;	//陀螺仪原始数据
	//float pitch,roll,yaw; 		//欧拉角
	u8 length=proNum;
	
	SysTick_init(72,10);//设置时钟频率
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	uart1_init(115200);//接PC的串口
	//Initial_UART2(115200);//接JY-901模块的串口	
	LED_Init();
	LCD_Init();
	POINT_COLOR=RED;
	MPU_Init();					//初始化MPU6050(包括初始化IIC、复位MPU、设置MPU参数等)
	KEY_Init();					//初始化按键
	LED0=0;
	delay_ms(1000);
 	while(HC05_Init()) 		//初始化ATK-HC05模块  
	{
		
		LCD_ShowString(30,90,200,16,16,"ATK-HC05 Error!"); 
		delay_ms(500);
		LCD_ShowString(30,90,200,16,16,"Please Check!!!"); 
		delay_ms(100);
	}
	LCD_ShowString(30,110,200,16,16,"ATK-HC05 Standby!");
	HC05_Role_Show();
	delay_ms(100);
	USART3_RX_STA=0;
	while(mpu_dmp_init())
	{
		LCD_ShowString(30,130,200,16,16,"MPU6050 Error");
		delay_ms(200);
		LCD_Fill(30,130,239,130+16,WHITE);
 		delay_ms(200);
	}
	distance=0;
	//u3_printf("%.3f,%.3f,%.3f\r\n",global_x,global_y,distance);
	
	while(1)
	{
			u8 cur=accBufBegin;
			u8 tmpLength=length;		
		if(key==KEY0_PRES)
		{
			report=!report;
			if(report)LCD_ShowString(30,170,200,16,16,"UPLOAD ON ");
			else LCD_ShowString(30,170,200,16,16,"UPLOAD OFF");
		}

			mean=0;
			for(;;)
			{			
				MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//得到加速度传感器数据
				if(tempx==aacx&&tempy==aacy&&tempz==aacz)continue;//检测是否有新数据产生,若无，则重新进入循环
				tempx=aacx;tempy=aacy;tempz=aacz;
				accBufEnd++;
				aacx=aacx*4*9.8f/65536;
				aacy=aacy*4*9.8f/65536;
				aacz=aacz*4*9.8f/65536;
				memcpy(&accBuf[0][accBufEnd],&aacx,2);
				memcpy(&accBuf[1][accBufEnd],&aacy,2);
				memcpy(&accBuf[2][accBufEnd],&aacz,2);
			  MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//得到陀螺仪数据
				gyroy=gyroy*500/65536;
				memcpy(&yawBuf[yawBufEnd],&gyroy,2);
				//tmpLength=accBufEnd-accBufBegin;
				if(tmpLength==proNum)
				{
					for(i=1;i<=tmpLength;i++)
					{
						arm_sqrt_f32(accBuf[0][cur]*accBuf[0][cur]+accBuf[1][cur]*accBuf[1][cur]+accBuf[2][cur]*accBuf[2][cur],&accNorm[i]);
						mean+=accNorm[i];
						cur++;
					}
					accBufBegin+=tmpLength;
					break;
				}
			}	
			
		
			for(i=1;i<=tmpLength;i++)
			{
				accNorm[i]-=mean;
			}

			IIRFilter(accNorm,accNormFiltered,tmpLength);
			
			peakNum=findPeaks(accNormFiltered,peaksInfo,tmpLength);
			
			if (peakNum==0)
			{
				// No peak, indicating standing still. Just drop the data.
				accBufBegin+=tmpLength;
				yawBufBegin+=tmpLength;
				length-=tmpLength;
				flag=0;
				continue;
			}
			
			if (peakNum==1)
			{
				// can't calculate the step with only one peak.
				flag=0;
				continue;
			}
			
			
			for(i=0,cur=yawBufBegin;i<tmpLength;i++,cur++)
			{
				degree=yawBuf[cur];  //((int)yawBuf[cur]%360)+yawBuf[cur]-(int)yawBuf[cur];
				if(degree>=0 && degree<90)
				{
					yawSin[i]=degree;
					yawCos[i]=degree;
				}
				else if(degree>=90 && degree<180)
				{
					yawSin[i]=180-degree;
					yawCos[i]=degree;
				}
				else if(degree>=180 && degree<270)
				{
					yawSin[i]=180-degree;
					yawCos[i]=360-degree;
				}
				else
				{
					yawSin[i]=degree-360;
					yawCos[i]=360-degree;
				}
			}
			
			for (i=0; i<peakNum-1;i++)
			{
				posStart=(u8)peaksInfo[0][i];
				posEnd=(u8)peaksInfo[0][i+1];
				arm_mean_f32(&yawSin[posStart],posEnd-posStart,&yawSinMean);
				arm_mean_f32(&yawCos[posStart],posEnd-posStart,&yawCosMean);
				
				stepFreq=1.0f/(peaksInfo[2][i+1]*dt);
				arm_var_f32(&accNorm[posStart],posEnd-posStart,&stepAV);
				stepLength=0.2844f+0.2231f*stepFreq+0.0426f*stepAV;
				
				distance+=stepLength;
				
				global_x+=stepLength*arm_cos_f32(yawCosMean/180*PI);
				global_y+=stepLength*arm_sin_f32(yawSinMean/180*PI);
				u3_printf("%.3f,%.3f,%.3f\r\n",global_x,global_y,stepLength);
			}
			
			i=(peaksInfo[0][peakNum-1]+peaksInfo[0][peakNum-2])/2;
			accBufBegin+=i;
			yawBufBegin+=i;
			length-=i;
			

			flag=0;
		}
	
}

//CopeSerialData为串口2中断调用函数，串口每收到一个数据，调用一次这个函数。
//void CopeSerial2Data(unsigned char ucData)
//{
//	static unsigned char ucRxBuffer[11];
//	static unsigned char ucRxCnt = 0;	
//	
//	static u8 accCnt=0;
//	static u8 yawCnt=0;
//	//LED_REVERSE();					//接收到数据，LED灯闪烁一下
//	//USB_TxWrite(&ucData,1);			//向USB-HID端口转发收到的串口数据，可直接用接上位机看到模块输出的数据。
//	ucRxBuffer[ucRxCnt++]=ucData;	//将收到的数据存入缓冲区中
//	if (ucRxBuffer[0]!=0x55) //数据头不对，则重新开始寻找0x55数据头
//	{
//		ucRxCnt=0;
//		return;
//	}
//	if (ucRxCnt<11) {return;}//数据不满11个，则返回
//	else
//	{
//		switch(ucRxBuffer[1])//判断数据是哪种数据，然后将其拷贝到对应的结构体中，有些数据包需要通过上位机打开对应的输出后，才能接收到这个数据包的数据
//		{
//			case 0x50:
//				//delay_ms(100);
//				break;
//				//memcpy(&stcTime,&ucRxBuffer[2],8);break;//memcpy为编译器自带的内存拷贝函数，需引用"string.h"，将接收缓冲区的字符拷贝到数据结构体里面，从而实现数据的解析。
//			case 0x51:	
//				saveAcc(ucRxBuffer);
//				accCnt++;
//				break;
//			case 0x52:
//				saveYaw(ucRxBuffer);
//				yawCnt++;
//				break;
//			case 0x53:
//				//showYaw(ucRxBuffer);
//				break;
//			default:
//				break;
//		}
//		ucRxCnt=0;//清空缓存区
//		if((accCnt>=proNum) & (yawCnt >= proNum))
//		{
//			flag=1;
//			accCnt-=proNum;
//			yawCnt-=proNum;
//		}
//		return ;
//	}
//}

//void CopeSerial1Data(unsigned char ucData)
//{	
//	UART2_Put_Char(ucData);//转发串口1收到的数据给串口2（JY模块）
//}






void IIRFilter(float32_t xArr[],float32_t yArr[],u8 tmpLength)
{
	int i,j;
	
	for(i = 0; i < tmpLength; i++){
		float32_t yFront = 0;
		float32_t yBehind = 0;
		for(j = 0; j <= M & j <= i; j++){
			yFront = yFront + bArr[j] * xArr[i - j];
		}
		for(j = 1; j <= N & j <= i; j++){
			yBehind = yBehind + aArr[j] * yArr[i - j];
		}
		yArr[i] =(yFront - yBehind);
	}
	return ;
}

u8 findPeaks(float32_t filteredAcc[], float32_t peaksInfo[3][peakCapacity],u8 tmpLength)
{
	Peak peaks[20];
	u8 i,j,cnt=0;
	
	for(i=1;i<tmpLength-1;i++)
	{
		if(filteredAcc[i]>=filteredAcc[i-1]&&filteredAcc[i]>=filteredAcc[i+1]&&filteredAcc[i]>minPeakValue)
		{
			peaks[cnt].location=i;
			peaks[cnt].value=filteredAcc[i];
			peaks[cnt].valid=1;
			cnt++;
		}
	}
	
	for(i=1;i<cnt;i++)
	{
		peaks[i].delta=peaks[i].location-peaks[i-1].location;
		if (peaks[i].delta<delta_thres)
		{
			if(peaks[i].value>peaks[i-1].value)
			{
				peaks[i].delta+=peaks[i-1].delta;
			}
			else
			{
				peaks[i].location=peaks[i-1].location;
				peaks[i].value=peaks[i-1].value;
				peaks[i].delta=peaks[i-1].delta;
			}
			peaks[i-1].valid=0;
		}
	}
	
	j=0;
	for(i=0;i<cnt;i++)
	{
		if(peaks[i].valid)
		{
			peaksInfo[0][j]=peaks[i].location;
			peaksInfo[1][j]=peaks[i].value;
			peaksInfo[2][j]=peaks[i].delta;
			j++;
		}
	}
	return j;  // number of valid peaks
}

void saveAcc(unsigned char rxBuffer[11])
{
	short res[3];
//	char str[50];
	u8 i;
	memcpy(res,&rxBuffer[2],6);
	for(i=0;i<3;i++)
	{
		accBuf[i][accBufEnd]=(float32_t)res[i]/32768*16*9.8f;
	}
//	sprintf(str,"Acc: %.2f  %.2f  %.2f",accBuf[0][accBufEnd],accBuf[1][accBufEnd],accBuf[2][accBufEnd]);
//	LCD_ShowString(30,130,200,16,16,str);
	accBufEnd++;
	length++;
}

void saveYaw(unsigned char rxBuffer[11])
{
	short res;
//	char str[50];
	float32_t wz,degree;
	u8 tmp=yawBufEnd;
	memcpy(&res,&rxBuffer[6],2);
	wz=(float32_t)res/32768*2000;
	degree=yawBuf[--tmp]+wz*dt;
	if(degree>=0)
	{
		yawBuf[yawBufEnd]=((int)degree%360)+degree-(int)degree;
	}
	else
	{
		yawBuf[yawBufEnd]=((int)degree%360)+360+degree-(int)degree;
	}
//	sprintf(str,"Wz: %.2f   Yaw: %.2f",wz,yawBuf[yawBufEnd]);
//	LCD_ShowString(30,160,200,16,16,str);
	yawBufEnd++;
}

void showYaw(unsigned char rxBuffer[11])
{
	short res;
	char str[50];
	float32_t yaw;
	memcpy(&res,&rxBuffer[6],2);
	yaw=(float32_t)res/32786*180;
	sprintf(str,"rawYaw: %.2f",yaw);
	LCD_ShowString(30,180,200,16,16,str);
}

//显示ATK-HC05模块的主从状态
void HC05_Role_Show(void)
{
	if(HC05_Get_Role()==1)LCD_ShowString(30,140,200,16,16,"ROLE:Master");	//主机
	else LCD_ShowString(30,140,200,16,16,"ROLE:Slave ");			 		//从机
}
//显示ATK-HC05模块的连接状态
void HC05_Sta_Show(void)
{												 
	if(HC05_LED)LCD_ShowString(120,140,120,16,16,"STA:Connected ");			//连接成功
	else LCD_ShowString(120,140,120,16,16,"STA:Disconnect");	 			//未连接				 
}	 

