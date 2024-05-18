#include "AD7606.h"
#include "main.h"
#include "tim.h"
#include "usart.h"
#define AD_CS_SET AD_CS_GPIO_Port->BSRR = AD_CS_Pin
#define AD_RD_SET AD_RD_GPIO_Port->BSRR = AD_RD_Pin

#define AD_CS_CLR AD_CS_GPIO_Port->BSRR = (uint32_t)AD_CS_Pin << 16u
#define AD_RD_CLR AD_RD_GPIO_Port->BSRR = (uint32_t)AD_RD_Pin << 16u

#define DATA_PORT GPIOF->IDR
#define Uart_SendBuf_Num 20
int16_t ADC_Code[8];

float ADC_Volt[8] = {0};
float Filt_Dat[8];
float Volt_bias[8] = {0};
float Meg_val[8] = {0};
struct AD7606_FIFO ad7606_fifo; //环形缓存区
float *Sendbuf;
uint8_t CalBias_flag = 0;
uint64_t cnt = 0;


void AD7606_Reset(void)
{
	HAL_GPIO_WritePin(AD_RESET_GPIO_Port, AD_RESET_Pin, 0);
	HAL_GPIO_WritePin(AD_RESET_GPIO_Port, AD_RESET_Pin, 1);
	HAL_GPIO_WritePin(AD_RESET_GPIO_Port, AD_RESET_Pin, 1);
	HAL_GPIO_WritePin(AD_RESET_GPIO_Port, AD_RESET_Pin, 1);
	HAL_GPIO_WritePin(AD_RESET_GPIO_Port, AD_RESET_Pin, 0);
}

void AD7606_SetOS(uint8_t osmode)
{
	switch (osmode)
	{
	case NO_OS:
	{
		HAL_GPIO_WritePin(AD_OS1_GPIO_Port, AD_OS1_Pin, 0);
		HAL_GPIO_WritePin(AD_OS2_GPIO_Port, AD_OS2_Pin, 0);
		HAL_GPIO_WritePin(AD_OS3_GPIO_Port, AD_OS3_Pin, 0);
	}
	break;
	case OS_2:
	{
		HAL_GPIO_WritePin(AD_OS1_GPIO_Port, AD_OS1_Pin, 1);
		HAL_GPIO_WritePin(AD_OS2_GPIO_Port, AD_OS2_Pin, 0);
		HAL_GPIO_WritePin(AD_OS3_GPIO_Port, AD_OS3_Pin, 0);
	}
	break;
	case OS_4:
	{
		HAL_GPIO_WritePin(AD_OS1_GPIO_Port, AD_OS1_Pin, 0);
		HAL_GPIO_WritePin(AD_OS2_GPIO_Port, AD_OS2_Pin, 1);
		HAL_GPIO_WritePin(AD_OS3_GPIO_Port, AD_OS3_Pin, 0);
	}
	break;
	case OS_8:
	{
		HAL_GPIO_WritePin(AD_OS1_GPIO_Port, AD_OS1_Pin, 1);
		HAL_GPIO_WritePin(AD_OS2_GPIO_Port, AD_OS2_Pin, 1);
		HAL_GPIO_WritePin(AD_OS3_GPIO_Port, AD_OS3_Pin, 0);
	}
	break;
	case OS_16:
	{
		HAL_GPIO_WritePin(AD_OS1_GPIO_Port, AD_OS1_Pin, 0);
		HAL_GPIO_WritePin(AD_OS2_GPIO_Port, AD_OS2_Pin, 0);
		HAL_GPIO_WritePin(AD_OS3_GPIO_Port, AD_OS3_Pin, 1);
	}
	break;
	case OS_32:
	{
		HAL_GPIO_WritePin(AD_OS1_GPIO_Port, AD_OS1_Pin, 1);
		HAL_GPIO_WritePin(AD_OS2_GPIO_Port, AD_OS2_Pin, 0);
		HAL_GPIO_WritePin(AD_OS3_GPIO_Port, AD_OS3_Pin, 1);
	}
	break;
	case OS_64:
	{
		HAL_GPIO_WritePin(AD_OS1_GPIO_Port, AD_OS1_Pin, 0);
		HAL_GPIO_WritePin(AD_OS2_GPIO_Port, AD_OS2_Pin, 1);
		HAL_GPIO_WritePin(AD_OS3_GPIO_Port, AD_OS3_Pin, 1);
	}
	break;
	default:
		break;
	}
}
uint8_t text[4];
struct _JustFloat JustFloat;
void AD7606_Init(void)
{
	//	text[2] = 0xFF;
	//	text[3] = 0xFF;
	//
	JustFloat.tail[0] = 0X00;
	JustFloat.tail[1] = 0X00;
	JustFloat.tail[2] = 0X80;
	JustFloat.tail[3] = 0X7F;

	for (int16_t i = 0; i < AD_FIFO_SIZE; i++)
	{
		ad7606_fifo.Buf[i][8].s[0] = 0x00;
		ad7606_fifo.Buf[i][8].s[1] = 0x00;
		ad7606_fifo.Buf[i][8].s[2] = 0x80;
		ad7606_fifo.Buf[i][8].s[3] = 0x7F;
	}

	ad7606_fifo.Fifo_read = 0;
	ad7606_fifo.Fifo_wirte = 0;
	ad7606_fifo.Fifo_count = 0;
	ad7606_fifo.isFull = 0;

	HAL_Delay(1);
	HAL_GPIO_WritePin(AD_RESET_GPIO_Port, AD_RESET_Pin, 0);
	HAL_GPIO_WritePin(AD_RANGE_GPIO_Port, AD_RANGE_Pin, 0);
	AD7606_SetOS(OS_64);
	AD_RD_SET;
	HAL_Delay(1);
	AD7606_Reset();
}

uint16_t AD7606_RESULT()
{
	uint16_t dat;
	AD_RD_CLR;
	__NOP();
	dat = DATA_PORT;
	AD_RD_SET;
	return dat;
}
uint8_t str[4] = {0};

void ADC_CodeToVolt_CONV(int16_t *Code)
{
	const float Volt_Step = 0.152; // mv
	for (uint8_t i = 0; i < 8; i++)
	{
		ADC_Volt[i] = Code[i] * Volt_Step;
	}
}

void MoveAverageFilter(float *dat, uint8_t Window)
{
	static uint8_t num = 0;
	static float sum[8] = {0};

	num++;
	if (num >= 250)
	{
		num = 250;
	}
	for (uint8_t i = 0; i < 8; i++)
	{
		if (num <= Window)
		{
			sum[i] += dat[i];
			Filt_Dat[i] = sum[i] / num;
		}
		else
		{
			sum[i] -= sum[i] / Window;
			sum[i] += dat[i];
			Filt_Dat[i] = sum[i] / Window;
		}
	}
}

float Volt_Meg_Fitting(float *volt)
{
	const float coef = 0.1543;

	for (uint8_t i = 0; i < 8; i++)
	{
		volt[i] = volt[i] - Volt_bias[i];
		Meg_val[i] = coef * volt[i];
	}
}
// void AverageFilter(uint16_t *dat)
//{

//	num++;
//	if(num>=250)
//	{
//		num = 250;
//	}
//	for(uint8_t i=0;i<8;i++)
//	{
//			if(num<=N)
//		{
//			sum[i] += dat[i];
//			Filt_Dat[i] = sum[i]/num;
//		}
//		else
//		{
//			sum[i] -=sum[i]/N;
//			sum[i] += dat[i];
//			Filt_Dat[i] = sum[i]/N;
//		}
//	}

//}

void Calculate_VBias(float *Volt)
{
	static uint16_t n = 0;
	static float sum[8] = {0};

	if (n < 200)
	{
		for (uint8_t i = 0; i < 8; i++)
		{
			sum[i] += Volt[i];
		}
		n++;
	}
	else
	{
		for (uint8_t i = 0; i < 8; i++)
		{
			Volt_bias[i] += sum[i] / n;
			sum[i] = 0;
		}
		n = 0;
		CalBias_flag = 0;
	}
}
uint32_t samp_cnt;
void AD7606_ReadNowADC(void)
{samp_cnt++;
	static uint8_t cnt = 0;
	const uint8_t send_Num = 5;
	AD_CS_CLR;
	ADC_Code[0] = AD7606_RESULT(); /* 读第1路 */
	ADC_Code[1] = AD7606_RESULT(); /* 读第2路 */
	ADC_Code[2] = AD7606_RESULT(); /* 读第3路 */
	ADC_Code[3] = AD7606_RESULT(); /* 读第4路 */
	ADC_Code[4] = AD7606_RESULT(); /* 读第5路 */
	ADC_Code[5] = AD7606_RESULT(); /* 读第6路 */
	ADC_Code[6] = AD7606_RESULT(); /* 读第7路 */
	ADC_Code[7] = AD7606_RESULT(); /* 读第8路 */
	AD_CS_SET;

	ADC_CodeToVolt_CONV(ADC_Code);

	MoveAverageFilter(ADC_Volt, 5);

	if (CalBias_flag == 1)
	{
		Calculate_VBias(Filt_Dat);
	}

	Volt_Meg_Fitting(Filt_Dat);

	AD7606_FIFO_Write(Meg_val);
	cnt++;
	if (cnt >= send_Num)
	{
		AD7606_FIFO_Read(send_Num);
		cnt = 0;
	}
}

void AD7606_StopConv()
{
	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4);
	HAL_GPIO_WritePin(AD_CONV_GPIO_Port, AD_CONV_Pin, 1);
}
/*
void AD7606_StartConv(float Sps)
{
//	AD7606_StopConv();

	AD7606_SetOS(NO_OS);
	HAL_GPIO_WritePin(AD_RANGE_GPIO_Port,AD_RANGE_Pin,0);
	AD7606_Reset();
	HAL_GPIO_WritePin(AD_CONV_GPIO_Port,AD_CONV_Pin,1);
	HAL_GPIO_WritePin(AD_CONV_GPIO_Port,AD_CONV_Pin,0);
	HAL_GPIO_WritePin(AD_CONV_GPIO_Port,AD_CONV_Pin,0);
	HAL_GPIO_WritePin(AD_CONV_GPIO_Port,AD_CONV_Pin,0);
	HAL_GPIO_WritePin(AD_CONV_GPIO_Port,AD_CONV_Pin,1);

	ad7606_fifo.Fifo_read=0;
	ad7606_fifo.Fifo_wirte=0;
	ad7606_fifo.Fifo_count=0;
	ad7606_fifo.isFull=0;
	 if(Sps < 20000)
	 {
		Set_PWM_Freq(&htim2,Sps);
	 }
	 else
	 {
		Set_PWM_Freq(&htim2,20000);
	 }
}
*/

uint8_t AD7606_FIFO_HasNewData(void)
{
	if (ad7606_fifo.Fifo_count > 0)
	{
		return 1;
	}
	return 0;
}

uint8_t AD7606_FIFOFull(void)
{
	return ad7606_fifo.isFull;
}

void AD7606_FIFO_Write(float *Buf)
{
	for (uint8_t i = 0; i < 8; i++)
	{
		ad7606_fifo.Buf[ad7606_fifo.Fifo_wirte][i].dat = Buf[i];
	}

	if (++ad7606_fifo.Fifo_wirte >= AD_FIFO_SIZE)
	{
		ad7606_fifo.Fifo_wirte = 0;
	}

	if (ad7606_fifo.Fifo_count < AD_FIFO_SIZE)
	{
		ad7606_fifo.Fifo_count++;
	}
	else
	{
		ad7606_fifo.isFull = 1;
	}
}

void AD7606_FIFO_Read(uint16_t Num) // Num必须是500的因数
{
	Sendbuf = &ad7606_fifo.Buf[ad7606_fifo.Fifo_read][0].dat;
	ad7606_fifo.Fifo_read += Num;

	if (ad7606_fifo.Fifo_read >= AD_FIFO_SIZE)
	{
		ad7606_fifo.Fifo_read = ad7606_fifo.Fifo_read % AD_FIFO_SIZE;
	}
	USART_TX_DMA_Send(&huart1, &hdma_usart1_tx, (uint8_t *)Sendbuf, Num * Data_Size * 4);

	ad7606_fifo.Fifo_count -= Num;
}
