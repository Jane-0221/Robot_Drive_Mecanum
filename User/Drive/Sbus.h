#ifndef __SBUS_H__
#define __SBUS_H__
#include "stdint.h"


typedef struct
{
	uint16_t CH1;// 通道1数据
	uint16_t CH2;// 通道2数据
	uint16_t CH3;// 通道3数据
	uint16_t CH4;// 通道4数据
	uint16_t CH5;// 通道5数据
	uint16_t CH6;// 通道6数据
	uint16_t CH7;// 通道7数据
	uint16_t CH8;// 通道8数据
	uint16_t CH9;// 通道9数据
	uint16_t CH10;// 通道10数据
	uint16_t CH11;// 通道11数据
	uint16_t CH12;// 通道12数据
	uint16_t CH13;// 通道13数据
	uint16_t CH14;// 通道14数据
	uint16_t CH15;// 通道15数据
	uint16_t CH16;// 通道16数据
	uint8_t ConnectState;// 遥控器连接状态：0=未连接，1=已连接
}SBUS_CH_Struct;

extern SBUS_CH_Struct SBUS_CH;
extern uint8_t sbus_data_buffer[256];
void update_sbus(volatile const uint8_t *sbus_buf,SBUS_CH_Struct *SBUS_CH);
uint8_t SBUS_UpdateIfNew(void);

// 存储256字节数据的函数声明
void store_sbus_data(const uint8_t *data, uint16_t size);

// 获取存储的数据缓冲区指针
uint8_t* get_sbus_data_buffer(void);

// 获取缓冲区大小
uint16_t get_sbus_buffer_size(void);

#endif //
