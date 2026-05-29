#include "stp23l.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stddef.h>
#include <string.h>
/************************* 全局解析数据实例化 *************************/
STP23L_DataDef stp23l_data = {0};
uint8_t stp23l_raw_data[256];
static uint8_t stp23l_parse_buffer[256];
static volatile uint16_t stp23l_raw_size = 0U;
static volatile uint8_t stp23l_data_pending = 0U;

/************************* 静态辅助函数：查找4AA包头 *************************/
static uint16_t STP23L_FindHeader(uint8_t *buf, uint16_t size)
{
    for(uint16_t i = 0; i <= size - STP23L_HEADER_BYTES; i++)
    {
        if((buf[i] == STP23L_HEADER) && (buf[i+1] == STP23L_HEADER) &&
           (buf[i+2] == STP23L_HEADER) && (buf[i+3] == STP23L_HEADER))
        {
            return i;
        }
    }
    return 0xFFFF; // 未找到包头
}
static void STP23L_StoreRaw(const uint8_t *data,
                            uint16_t size,
                            uint8_t *raw_data,
                            volatile uint16_t *raw_size,
                            volatile uint8_t *data_pending)
{
    if (data == NULL || size == 0) {
        return;
    }

    uint16_t copy_size = (size > 256U) ? 256U : size;

    memcpy(raw_data, data, copy_size);
    *raw_size = copy_size;
    *data_pending = 1U;
}

void store_stp23l_data(const uint8_t *data, uint16_t size)
{
    STP23L_StoreRaw(data, size, stp23l_raw_data, &stp23l_raw_size, &stp23l_data_pending);
}

void STP23L_Reset(void)
{
    stp23l_data.parse_ok = 0;
    stp23l_data.parse_err = 0;
}

uint8_t STP23L_ParseLatest(void)
{
    uint16_t size;

    if (stp23l_data_pending == 0U)
    {
        return 0U;
    }

    taskENTER_CRITICAL();
    size = stp23l_raw_size;
    memcpy(stp23l_parse_buffer, stp23l_raw_data, size);
    stp23l_data_pending = 0U;
    taskEXIT_CRITICAL();

    STP23L_ParseData(stp23l_parse_buffer, size);
    return 1U;
}

void STP23L_ParseData(uint8_t *buf, uint16_t size)
{
    if(buf == NULL || size < STP23L_HEAD_LEN)
    {
        stp23l_data.parse_err = 1;
        return;
    }

    uint16_t buf_idx = 0;
    uint8_t crc_calc = 0;
    uint16_t header_pos = 0;

    while(buf_idx <= size - STP23L_HEADER_BYTES)
    {
        header_pos = STP23L_FindHeader(&buf[buf_idx], size - buf_idx);
        if(header_pos == 0xFFFF) break;
        buf_idx += header_pos;

        // 校验整包完整性
        if((size - buf_idx) < STP23L_PACK_TOTAL_LEN)
        {
            stp23l_data.parse_err = 1;
            break;
        }

        // 解析包头字段
        uint8_t dev_addr = buf[buf_idx + 4];
        uint8_t cmd_code = buf[buf_idx + 5];
        uint16_t offset = (buf[buf_idx +7]<<8) | buf[buf_idx +6];
        uint16_t data_len = (buf[buf_idx +9]<<8) | buf[buf_idx +8];

        // 协议硬校验：仅处理获取测量数据命令
        if(dev_addr != STP23L_DEV_ADDR || cmd_code != STP23L_CMD_GET_DIST ||
           data_len != STP23L_DATA_FIELD_LEN || offset != 0x0000)
        {
            buf_idx++;
            stp23l_data.parse_err = 1;
            continue;
        }

        // 计算校验码（除去4AA，低8位累加）
        crc_calc = 0;
        for(uint16_t i = 4; i < STP23L_PACK_TOTAL_LEN -1; i++)
        {
            crc_calc += buf[buf_idx + i];
        }
        if(crc_calc != buf[buf_idx + STP23L_PACK_TOTAL_LEN -1])
        {
            buf_idx += STP23L_HEADER_BYTES;
            stp23l_data.parse_err = 1;
            continue;
        }

        // 解析12个测量点数据（小端模式）
        stp23l_data.cmd_code = cmd_code;
        uint16_t data_offset = buf_idx + STP23L_HEAD_LEN;
        for(uint8_t i = 0; i < STP23L_POINT_NUM; i++)
        {
            uint16_t p_offset = data_offset + i * STP23L_POINT_BYTES;
            stp23l_data.points[i].distance = (buf[p_offset+1]<<8) | buf[p_offset];
            stp23l_data.points[i].noise    = (buf[p_offset+3]<<8) | buf[p_offset+2];
            stp23l_data.points[i].peak     = (buf[p_offset+7]<<24) | (buf[p_offset+6]<<16) | (buf[p_offset+5]<<8) | buf[p_offset+4];
            stp23l_data.points[i].confidence = buf[p_offset+8];
            stp23l_data.points[i].intg     = (buf[p_offset+12]<<24) | (buf[p_offset+11]<<16) | (buf[p_offset+10]<<8) | buf[p_offset+9];
            stp23l_data.points[i].reftof   = (buf[p_offset+14]<<8) | buf[p_offset+13];
        }

        // 解析帧时间戳
        uint16_t ts_offset = buf_idx + STP23L_HEAD_LEN + STP23L_POINT_NUM * STP23L_POINT_BYTES;
        stp23l_data.timestamp = (buf[ts_offset+3]<<24) | (buf[ts_offset+2]<<16) | (buf[ts_offset+1]<<8) | buf[ts_offset];

        // 帧解析完成置位
        stp23l_data.parse_ok = 1;
        stp23l_data.recv_cnt++;
        stp23l_data.parse_err = 0;

        buf_idx += STP23L_PACK_TOTAL_LEN;
    }
}

int16_t STP23L_GetFinalDistPerFrame(void)
{
    static int16_t last_valid_dist = 0; // 静态缓存上一帧有效距离，防突变
    int32_t valid_dist_sum = 0;         // 有效距离累加和（32位避免溢出）
    uint8_t valid_point_cnt = 0;        // 有效测点数量
    int16_t final_dist = 0;             // 本帧最终距离值

    // 步骤1：遍历12个测点，按手册标准过滤有效点
    for(uint8_t i = 0; i < STP23L_POINT_NUM; i++)
    {
        if((stp23l_data.points[i].confidence >= STP23L_CONFIDENCE_THRESH) &&  // 置信度达标
           (stp23l_data.points[i].distance >= STP23L_DIST_BLIND) &&            // 超出盲区
           (stp23l_data.points[i].distance <= STP23L_DIST_MAX))               // 未超量程
        {
            valid_dist_sum += stp23l_data.points[i].distance;
            valid_point_cnt++;
        }
    }

    // 步骤2：根据有效点数量计算本帧最终距离
    if(valid_point_cnt > 0)
    {
        if(valid_point_cnt == 1)
        {
            final_dist = (int16_t)valid_dist_sum; // 单有效点直接返回
        }
        else
        {
            final_dist = (int16_t)(valid_dist_sum / valid_point_cnt); // 多有效点取平均
        }
        last_valid_dist = final_dist; // 更新历史有效距离
    }
    else
    {
        final_dist = last_valid_dist; // 无有效点，返回历史值防突变
    }

    return final_dist; // 每帧返回一个最终距离值
}
