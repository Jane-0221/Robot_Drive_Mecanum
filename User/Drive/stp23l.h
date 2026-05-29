#ifndef STP23L_H
#define STP23L_H

#include "stdint.h"

/************************* 严格遵循STP23L官方手册宏定义 *************************/
#define STP23L_HEADER            0xAAU        // 数据包起始符
#define STP23L_DEV_ADDR          0x00U        // 设备地址（保留位）
#define STP23L_PACK_TOTAL_LEN    195U         // 整包总字节：10包头+184数据+1校验
#define STP23L_HEAD_LEN          10U          // 包头长度
#define STP23L_DATA_FIELD_LEN    0x00B8U      // 数据域长度184字节
#define STP23L_POINT_NUM         12U          // 一帧12个测量点
#define STP23L_POINT_BYTES       15U          // 每个测点15字节
#define STP23L_TS_BYTES          4U           // 时间戳4字节
#define STP23L_HEADER_BYTES      4U           // 4个AA起始符

/************************* 手册核心测距指标（硬过滤） *************************/
#define STP23L_CONFIDENCE_THRESH 80U          // 置信度阈值（0~100，可配置）
#define STP23L_DIST_BLIND        30           // 手册标注盲区：最小有效距离30mm
#define STP23L_DIST_MAX          7500         // 手册标注最大有效距离7500mm

/************************* 命令码宏定义 *************************/
#define STP23L_CMD_GET_DIST      0x02U        // 获取测量数据（核心）
#define STP23L_CMD_RESET         0x0DU        // 复位
#define STP23L_CMD_STOP          0x0FU        // 停止传输
#define STP23L_CMD_ACK           0x10U        // 应答
#define STP23L_CMD_VERSION       0x14U        // 获取传感器信息

/************************* 测点结构体（严格匹配手册） *************************/
typedef struct
{
    int16_t  distance;   // 距离(mm) 2B小端
    uint16_t noise;      // 环境噪声 2B小端
    uint32_t peak;       // 接收强度 4B小端
    uint8_t  confidence; // 置信度 1B（噪声+接收强度融合）
    uint32_t intg;       // 积分次数 4B小端
    int16_t  reftof;     // 温度表征值 2B小端
}STP23L_PointDef;

/************************* 全局解析数据结构体（extern供外部调用） *************************/
typedef struct
{
    uint8_t         parse_ok;    // 解析完成标志：1-帧解析完成，0-未完成
    uint8_t         parse_err;   // 解析错误标志：1-校验/帧错误，0-正常
    STP23L_PointDef points[STP23L_POINT_NUM]; // 12个测点原始数据
    uint32_t        timestamp;   // 帧时间戳
    uint32_t        recv_cnt;    // 成功接收帧数
    uint8_t         cmd_code;    // 当前命令码
}STP23L_DataDef;

extern STP23L_DataDef stp23l_data;
extern uint8_t stp23l_raw_data[256];
/************************* 核心函数声明 *************************/
void STP23L_ParseData(uint8_t *buf, uint16_t size); // 整包解析函数
uint8_t STP23L_ParseLatest(void);
void STP23L_Reset(void);                           // 解析复位
extern void store_stp23l_data(const uint8_t *data, uint16_t size); // 存储STP23L原始数据
static inline void STP23L_ClearOkFlag(void)        // 清除解析完成标志
{
    stp23l_data.parse_ok = 0;
}
int16_t STP23L_GetFinalDistPerFrame(void);         // 每帧获取一个最终距离值

#endif // STP23L_H
