#ifndef __AD7606_H__
#define __AD7606_H__
#include "stdint.h"


#define AD_FIFO_SIZE 500
#define Data_Size 9


union U
{
    char s[4];
    float dat;
};
struct _JustFloat
{
	float adc;
	float adc_filt;
	uint8_t tail[4];
};

struct AD7606_FIFO
{
	uint16_t Fifo_wirte;
	uint16_t Fifo_read;
	uint16_t Fifo_count;
	uint8_t isFull;
  union U Buf[AD_FIFO_SIZE][Data_Size];

};

enum AD7606_OSMode
{
	 NO_OS=1,
	 OS_2,
	 OS_4,
	 OS_8,
	 OS_16,
	 OS_32,
	 OS_64,
};

extern struct AD7606_FIFO ad7606_fifo;
extern uint8_t CalBias_flag;
void AD7606_ReadNowADC(void);
void AD7606_StartConv(float Sps);
void AD7606_StopConv();
void AD7606_Init(void);
void AD7606_FIFO_Write(float *Buf);
void AD7606_FIFO_Read(uint16_t Num);
#endif