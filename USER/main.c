#include "sys.h"
#include "delay.h"
#include "led.h"
#include "beep.h"
#include "key.h"

#include "Main.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "UART1.h"
#include "UART2.h"
#include <string.h>

#include "JY901.h"



// data definition
#define proNum 150
#define frequence 50
#define dt 1.0/frequence
#define capacity 256
#define peakCapacity 50
#define threshold 400
#define data_thres 150
double minPeakValue=0.25;

float aArr[9]={0.0002,0,-0.0007,0,0.0011,0 -0.0007,0,0.0002};
float bArr[9]={1.0000,-7.2242,22.9586,-41.9263,48.1227,-35.5514,16.5092,-4.4061,0.5175};
int M=8;  // M = len(bArr) - 1; N=len(aArr) - 1
int N=8;
u8 flag;
u8 length,accBufBegin,accBufEnd,yawBufBegin,yawBufEnd;
float accBuf[capacity][3];
float yawBuf[capacity]={0};
float global_x=0,global_y=0;

typedef struct peak
{
	int location;       // ??
	double value;       // ??
	long time;          // ????
	long delta_time;    // ?????????
	struct peak *next;
}Peak,*PPeak;

// function declaration
void saveYaw(unsigned char rxBuffer[]);
void saveAcc(unsigned char rxBuffer[]);
float varCalculation(int start, int end, float arr[]);
void IIRFilter(float xArr[]);
float SquareRootFloat(float number);
int findPeaks(float filteredAcc[], long index[], float peaksInfo[3][peakCapacity]);
float meanCalculation(int start, int end, float arr[]);


/////////
float accNormFilterd[capacity];
float accNorm[capacity];

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
			default:
				break;
		}
		ucRxCnt=0;//清空缓存区
		if((accCnt>=proNum) & (yawCnt >= proNum))
		{
			flag=1;
			accCnt=0;
			yawCnt=0;
		}
		return ;
	}
}

void CopeSerial1Data(unsigned char ucData)
{	
	UART2_Put_Char(ucData);//转发串口1收到的数据给串口2（JY模块）
}

void saveAcc(unsigned char rxBuffer[11])
{
	static short res[3];
	int i;
	memcpy(res,&rxBuffer[2],6);
	for(i=0;i<3;i++)
	{
		accBuf[accBufEnd][i]=(float)res[i]/32768*16;
	}
	accBufEnd++;
	length++;
}

void saveYaw(unsigned char rxBuffer[11])
{
	static short res;
	static float wz;
	memcpy(&res,&rxBuffer[6],2);
	wz=(float)res/32768*2000;
	yawBuf[yawBufEnd]=yawBuf[yawBufEnd-1]+wz*dt;
	yawBufEnd++;
}


int main(void)
{  
	char str[100];
	u16 times=0;
	u8 t;
	u8 len;
	unsigned char i;
	
	
	float mean;
	float peaksInfo[3][peakCapacity];
	int peakNum;
	float accNormFilterd[capacity];
	float distance;
	float stepFreq,stepAV,stepLength;
	int posStart, posEnd;
	float yawSin,yawCos;
	
	//USB_Config();		//配置USB-HID
	SysTick_init(72,10);//设置时钟频率
	//NVIC_Configuration();
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	Initial_UART1(115200);//接PC的串口
	Initial_UART2(115200);//接JY-901模块的串口	
	
	//LED_ON();
	delay_ms(1000);//等等JY-91初始化完成
	
	while(1)
	{
		if(flag)
		{
			int i;
			u8 cur=accBufBegin;
			mean=0;
			distance=0;
			for(i=0;i<length;i++)
			{
				accNorm[i]=SquareRootFloat(accBuf[cur][0]*accBuf[cur][0]+accBuf[cur][1]*accBuf[cur][1]+accBuf[cur][2]*accBuf[cur][2]);
				mean=i/(float)(i+1)*mean+1/(float)(i+1)*accNorm[i];  // mean[n]=(n-1)/(n)*mean[n-1]+1/n*a[n]
				cur++;
			}
			for(i=0;i<length;i++)
			{
				accNorm[i]-=mean;
			}

			IIRFilter(NULL);
			
//			peakNum=findPeaks(accNormFilterd,index,peaksInfo);
//			
//			if (peakNum<1)
//			{
//				// No peak, indicating standing still. Just drop the data.
//				
//				
//			}
//			
//			for (int i=0; i<peakNum-1;i++)
//			{
//				posStart=(int)peaksInfo[0][i];
//				posEnd=(int)peaksInfo[0][i+1];
//				yawSin=meanCalculation(posStart,posEnd,finalYaw[1]);
//				yawCos=meanCalculation(posStart,posEnd,finalYaw[2]);
//				
//				stepFreq=1000.0/peaksInfo[2][i+1];
//				stepAV=varCalculation(posStart,posEnd,accNorm);
//				stepLength=0.2844+0.2231*stepFreq+0.0426*stepAV;
//				
//				distance+=stepLength;
//				
//				global_x+=stepLength*cos(yawCos);
//				global_y+=stepLength*sin(yawSin);
//			}
//			
//			// ?????????
//			// delIdx=(peaksInfo[peakNum-1]+peakInfo[peakNum-2])/2;
//			// for(int i=0; i<length-delIdx;i++)
//			// {
//				// acc[i]=acc[i+delIdx];
//				// yaw[i]=yaw[i+delIdx];
//			// }
//			
//			
//			// ???????????yaw,acc;
//			flag=0;
//			length=lengthBackup+length-delIdx;
//			lengthBackup=0;
			
		}
	};
}

float SquareRootFloat(float number)
{

    long i;

    float x, y;

    const float f = 1.5F;

    x = number * 0.5F;

    y  = number;

    i  = * ( long * ) &y;

    i  = 0x5f3759df - ( i >> 1 );  //???

  //i  = 0x5f375a86 - ( i >> 1 );  //Lomont

    y  = * ( float * ) &i;

    y  = y * ( f - ( x * y * y ) );

    y  = y * ( f - ( x * y * y ) );

    return number * y;
}

void IIRFilter(float xArr[])
{
	int i,j;
	for(i = 0; i < length; i++){
		double yFront = 0;
		double yBehind = 0;
		float res;
		for(j = 0; j <= M & j <= i; j++){
			yFront = yFront + bArr[j] * accNorm[i - j];
		}
		for(j = 1; j <= N & j <= i; j++){
			yBehind = yBehind + aArr[j] * accNormFilterd[i - j];
		}
		res=(yFront - yBehind) / aArr[0];
		accNormFilterd[i] = res;
	}
	return ;
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

int findPeaks(float filteredAcc[], long index[], float peaksInfo[3][peakCapacity])
{
	Peak dummyHead;
	PPeak cur=&dummyHead;
	PPeak p1,p2,p3;
	int i;
	int cnt=0;
	for(i=1; i<length-1;i++)
	{
		if(filteredAcc[i]>=filteredAcc[i-1]&&filteredAcc[i]>=filteredAcc[i+1]&&filteredAcc[i]>minPeakValue)
		{
			Peak peak;
			cur->next=&peak;
			cur=cur->next;
			cur->location=i;
			cur->value=filteredAcc[i];
			cur->time=index[i];
		}
	}
	
	// ????p1,p2,p3???????????
	p1=&dummyHead;
	p2=dummyHead.next;
	if(p2==NULL)                       /////////////////////// NULL 
		return 0;
	else
	{
		p2->delta_time=p2->time-index[0];       // ????delta_time???
		p3=p2->next;
	}
	while(p3)
	{
		p3->delta_time=p3->time-p2->time;
		if(p3->delta_time<threshold)  // 400
		{
			if(p3->value<=p2->value)
			{
				p2->next=p3->next;
				p3=p3->next;
			}
			else
			{
				p3->delta_time+=p2->delta_time;
				p1->next=p3;
				p2=p3;
				p3=p3->next;
			}
		}
		else
		{
			p1=p2;
			p2=p3;
			p3=p3->next;
		}
	}
	
	
	cur=dummyHead.next;
	
	while(cur)
	{
		peaksInfo[0][cnt]=cur->location;
		peaksInfo[1][cnt]=cur->value;
		peaksInfo[2][cnt]=cur->delta_time;
		cur=cur->next;
		cnt+=1;
	}
	return cnt;
}

float meanCalculation(int start, int end, float arr[])
{
	float ret=0;
	int i;
	for (i=start;i<end;i++)  // ????
	{
		ret+=arr[i];
	}
	return ret/(end-start);
}

float varCalculation(int start, int end, float arr[])
{
	float var,mean,sumSquare=0;
	int i;
	mean=meanCalculation(start,end,arr);
	for (i=start;i<end;i++)
	{
		sumSquare=sumSquare+(arr[i]-mean)*(arr[i]-mean);
	}
	var=sumSquare/(end-start);
	return var;
}