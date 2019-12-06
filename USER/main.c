#include "sys.h"
#include "delay.h"
#include "led.h"
#include "beep.h"
#include "key.h"
#include "lcd.h"
#include "hc05.h" 	 
#include "usart3.h" 	

#include "Main.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "UART1.h"
#include "UART2.h"
#include <string.h>
#include <stdlib.h>


#include "arm_math.h"
#include "stdio.h"



// data definition
#define proNum 100
#define frequence 50
#define dt 0.02
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


//// IIR
//float32_t testInput[10]={1,2,3,4,5,6,7,8,9,10};
//float32_t testOutput[10];
//float32_t testCustom[10];
//#define numStages 2
//float32_t IIRState[4*numStages];
//float32_t scaleValue=0.115582005640396334733566163777140900493*0.115582005640396334733566163777140900493;
//float32_t IIRCoeffs[5*numStages]={1,0,-1,1.900268344036148970843669303576461970806,-0.911213008049482686701026068476494401693,
//									1,0,-1,1.687640724660825819469778252823743969202,-0.769190930108339165904851597588276490569};
//arm_biquad_casd_df1_inst_f32 S;

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

//CopeSerialData为串口2中断调用函数，串口每收到一个数据，调用一次这个函数。
void CopeSerial2Data(unsigned char ucData)
{
	static unsigned char ucRxBuffer[11];
	static unsigned char ucRxCnt = 0;	
	
	static u8 accCnt=0;
	static u8 yawCnt=0;
	//LED_REVERSE();					//接收到数据，LED灯闪烁一下
	//USB_TxWrite(&ucData,1);			//向USB-HID端口转发收到的串口数据，可直接用接上位机看到模块输出的数据。
	ucRxBuffer[ucRxCnt++]=ucData;	//将收到的数据存入缓冲区中
	if (ucRxBuffer[0]!=0x55) //数据头不对，则重新开始寻找0x55数据头
	{
		ucRxCnt=0;
		return;
	}
	if (ucRxCnt<11) {return;}//数据不满11个，则返回
	else
	{
		switch(ucRxBuffer[1])//判断数据是哪种数据，然后将其拷贝到对应的结构体中，有些数据包需要通过上位机打开对应的输出后，才能接收到这个数据包的数据
		{
			case 0x50:
				//delay_ms(100);
				break;
				//memcpy(&stcTime,&ucRxBuffer[2],8);break;//memcpy为编译器自带的内存拷贝函数，需引用"string.h"，将接收缓冲区的字符拷贝到数据结构体里面，从而实现数据的解析。
			case 0x51:	
				saveAcc(ucRxBuffer);
				accCnt++;
				break;
			case 0x52:
				saveYaw(ucRxBuffer);
				yawCnt++;
				break;
			case 0x53:
				//showYaw(ucRxBuffer);
				break;
			default:
				break;
		}
		ucRxCnt=0;//清空缓存区
		if((accCnt>=proNum) & (yawCnt >= proNum))
		{
			flag=1;
			accCnt-=proNum;
			yawCnt-=proNum;
		}
		return ;
	}
}

void CopeSerial1Data(unsigned char ucData)
{	
	UART2_Put_Char(ucData);//转发串口1收到的数据给串口2（JY模块）
}





int main(void)
{  

	u8 i;
	
	
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

	
//	// IIR
//	arm_biquad_cascade_df1_init_f32(&S,numStages,(float32_t*)IIRCoeffs,(float32_t*)IIRState);   //reset state each time.
//	arm_biquad_cascade_df1_f32(&S,testInput,testOutput,10);
//	for(i=0;i<10;i++)
//	{
//		testOutput[i]*=scaleValue;
//	}
//	
//	for(i = 0; i < 10; i++){
//		double yFront = 0;
//		double yBehind = 0;
//		double res;
//		for(j = 0; j <= M & j <= i; j++){
//			yFront = yFront + bArr[j] * testInput[i - j];
//		}
//		for(j = 1; j <= N & j <= i; j++){
//			yBehind = yBehind + aArr[j] * testCustom[i - j];
//		}
//		res=(yFront - yBehind) / aArr[0];
//		testCustom[i] = res;
//	}
	
	SysTick_init(72,10);//设置时钟频率
	//NVIC_Configuration();
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	Initial_UART1(115200);//接PC的串口
	Initial_UART2(115200);//接JY-901模块的串口	
	LED_Init();
	
	LCD_Init();
	POINT_COLOR=RED;
//	LCD_ShowString(30,50,200,16,16,"Explorer STM32F4");	
	
	LED0=0;
	delay_ms(1000);//等等JY-91初始化完成
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
	
	distance=0;
	u3_printf("%.3f,%.3f,%.3f\r\n",global_x,global_y,distance);
	
	while(1)
	{
		if(flag)
		{
			u8 cur=accBufBegin;
			u8 tmpLength=length;
			mean=0;
			
			LED0=!LED0;      // reverse
			LED1=!HC05_LED;  // low invalid
			
			for(i=0;i<tmpLength;i++)
			{
				arm_sqrt_f32(accBuf[0][cur]*accBuf[0][cur]+accBuf[1][cur]*accBuf[1][cur]+accBuf[2][cur]*accBuf[2][cur],&accNorm[i]);
				mean+=accNorm[i];
				cur++;
			}
			mean/=tmpLength;
			for(i=0;i<tmpLength;i++)
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
				
				stepFreq=1.0/(peaksInfo[2][i+1]*dt);
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
	};
}

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
		accBuf[i][accBufEnd]=(float32_t)res[i]/32768*16*9.8;
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

//public static double[] IIRFilter(double[] xArr, double[] bArr, double[] aArr){
//        int lenB = bArr.length;
//        int lenA = aArr.length;
//        int lenX = xArr.length;
//        int M = lenB - 1;
//        int N = lenA - 1;
//        double[] yArr = new double[lenX];
//        for(int i = 0; i < lenX; i++){
//            double yFront = 0;
//            for(int j = 0; j <= M && j <= i; j++){
//                yFront = yFront + bArr[j] * xArr[i - j];
//            }
//            double yBehind = 0;
//            for(int j = 1; j <= N && j <= i; j++){
//                yBehind = yBehind + aArr[j] * yArr[i - j];
//            }
//            yArr[i] = (yFront - yBehind) / aArr[0];
//        }
//        return yArr;

//float SquareRootFloat(float number)
//{

//    long i;

//    float x, y;

//    const float f = 1.5F;

//    x = number * 0.5F;

//    y  = number;

//    i  = * ( long * ) &y;

//    i  = 0x5f3759df - ( i >> 1 );  //???

//  //i  = 0x5f375a86 - ( i >> 1 );  //Lomont

//    y  = * ( float * ) &i;

//    y  = y * ( f - ( x * y * y ) );

//    y  = y * ( f - ( x * y * y ) );

//    return number * y;
//}

//int findPeaks(float filteredAcc[], long index[], float peaksInfo[3][peakCapacity])
//{
//	PPeak dummyHead=(PPeak)malloc(sizeof(Peak));
//	PPeak cur=dummyHead;
//	PPeak ppeak;
//	PPeak p1,p2,p3;
//	int i;
//	int cnt=0;
//	for(i=1; i<length-1;i++)
//	{
//		if(filteredAcc[i]>=filteredAcc[i-1]&&filteredAcc[i]>=filteredAcc[i+1]&&filteredAcc[i]>minPeakValue)
//		{
//			ppeak=(PPeak)malloc(sizeof(PPeak));
//			cur->next=ppeak;
//			cur=cur->next;
//			cur->location=i;
//			cur->value=filteredAcc[i];
//			cur->time=index[i];
//		}
//	}
//	
//	// ????p1,p2,p3???????????
//	p1=dummyHead;
//	p2=dummyHead->next;
//	if(p2==NULL)                       /////////////////////// NULL 
//		return 0;
//	else
//	{
//		p2->delta_time=p2->time-index[0];       // ????delta_time???
//		p3=p2->next;
//	}
//	while(p3!=NULL)
//	{
//		p3->delta_time=p3->time-p2->time;
//		if(p3->delta_time<threshold)  // 400
//		{
//			if(p3->value<=p2->value)
//			{
//				p2->next=p3->next;
//				p3=p3->next;
//			}
//			else
//			{
//				p3->delta_time+=p2->delta_time;
//				p1->next=p3;
//				p2=p3;
//				p3=p3->next;
//			}
//		}
//		else
//		{
//			p1=p2;
//			p2=p3;
//			p3=p3->next;
//		}
//	}
//	
//	
//	cur=dummyHead->next;
//	
//	while(cur!=NULL)
//	{
//		peaksInfo[0][cnt]=cur->location;
//		peaksInfo[1][cnt]=cur->value;
//		peaksInfo[2][cnt]=cur->delta_time;
//		cur=cur->next;
//		cnt+=1;
//	}

//	p1=dummyHead;
//	do
//	{
//		p2=p1->next;
//		free(p1);
//		p1=p2;
//	}
//	while(p2!=NULL);
//	return cnt;
//}

//float meanCalculation(int start, int end, float arr[])
//{
//	float ret=0;
//	int i;
//	for (i=start;i<end;i++)  // ????
//	{
//		ret+=arr[i];
//	}
//	return ret/(end-start);
//}

//float varCalculation(int start, int end, float arr[])
//{
//	float var,mean,sumSquare=0;
//	int i;
//	mean=meanCalculation(start,end,arr);
//	for (i=start;i<end;i++)
//	{
//		sumSquare=sumSquare+(arr[i]-mean)*(arr[i]-mean);
//	}
//	var=sumSquare/(end-start);
//	return var;
//}
