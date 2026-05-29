#include "Robstride04.h"
#include <string.h>
#include "fdcan.h"

// 外部CAN句柄声明（保留原有硬件句柄命名，仅修改函数参数）

// RS04电机参数范围
#define P_MIN -12.57f      // RS04: -12.57rad
#define P_MAX 12.57f       // RS04: 12.57rad
#define V_MIN -15.0f       // RS04: -15rad/s
#define V_MAX 15.0f        // RS04: 15rad/s
#define KP_MIN 0.0f
#define KP_MAX 5000.0f     // RS04: 0~5000
#define KD_MIN 0.0f
#define KD_MAX 100.0f      // RS04: 0~100
#define T_MIN -120.0f      // RS04: -120Nm
#define T_MAX 120.0f       // RS04: 120Nm
#define CSP_LIMIT_SPD_MAX 20.0f

const uint16_t Index_List[] = {0X7005, 0X7006, 0X700A, 0X700B, 0X7010, 0X7011, 0X7014, 0X7016, 0X7017, 0X7018, 0x7019, 0x701A, 0x701B, 0x701C, 0x701D};

uint32_t Mailbox; // 邮箱变量

// 如果没有定义CAN相关类型，定义它们
#ifndef CAN_TxHeaderTypeDef
typedef struct {
    uint32_t StdId;
    uint32_t ExtId;
    uint32_t IDE;
    uint32_t RTR;
    uint32_t DLC;
} CAN_TxHeaderTypeDef;
#endif

#ifndef CAN_ID_EXT
#define CAN_ID_EXT 1
#endif

#ifndef CAN_ID_STD
#define CAN_ID_STD 0
#endif

#ifndef CAN_RTR_DATA
#define CAN_RTR_DATA 0
#endif

/*******************************************************************************
* 函数功能  : uint16_t型转float型浮点数
*******************************************************************************/
float uint16_to_float_lz(uint16_t x, float x_min, float x_max, uint8_t bits)
{
    uint32_t span = (1 << bits) - 1;
    x &= span;
    float offset = x_max - x_min;
    return offset * x / span + x_min;
}

/*******************************************************************************
* 函数功能  : float浮点数转int型
*******************************************************************************/
uint16_t float_to_uint_lz(float x, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    if (x > x_max) x = x_max;
    else if (x < x_min) x = x_min;
    return (uint16_t)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

/*******************************************************************************
* 函数功能  : uint8_t数组转float浮点数
*******************************************************************************/
float Byte_to_float(uint8_t* bytedata)
{
    uint32_t data = (uint32_t)bytedata[7] << 24 | (uint32_t)bytedata[6] << 16 | (uint32_t)bytedata[5] << 8 | bytedata[4];
    float data_float = *(float*)(&data);
    return data_float;
}

/*******************************************************************************
* 函数功能  : MIT错位码转换至私有模式错位码
*******************************************************************************/
uint8_t mapFaults(uint16_t fault16)
{
    uint8_t fault8 = 0;
    
    // 根据RS04说明书第55页的故障位定义进行映射
    if (fault16 & (1 << 16)) fault8 |= (1 << 0); // A相电流采样过流 -> 欠压故障(位0)
    if (fault16 & (1 << 14)) fault8 |= (1 << 4); // 电机堵转过载算法保护 -> 过载故障(位4)
    if (fault16 & (1 << 9))  fault8 |= (1 << 5); // 位置初始化故障 -> 未标定(位5)
    if (fault16 & (1 << 8))  fault8 |= (1 << 1); // 硬件识别故障 -> 驱动故障(位1)
    if (fault16 & (1 << 7))  fault8 |= (1 << 5); // 编码器未标定 -> 未标定(位5)
    if (fault16 & (1 << 5))  fault8 |= (1 << 2); // C相电流采样过流 -> 过温(位2)
    if (fault16 & (1 << 4))  fault8 |= (1 << 2); // B相电流采样过流 -> 过温(位2)
    if (fault16 & (1 << 3))  fault8 |= (1 << 3); // 过压故障 -> 磁编码故障(位3)
    
    return fault8;
}

/*******************************************************************************
* 函数功能  : RobStride电机初始化
*******************************************************************************/
void RobStride_Motor_Init(RobStride_Motor_t* motor, uint8_t CAN_Id, bool MIT_mode)
{
    motor->CAN_ID = CAN_Id;
    motor->Master_CAN_ID = 0xFD;
    motor->Motor_Set_All.set_motor_mode = move_control_mode;
    motor->MIT_Mode = MIT_mode;
    motor->MIT_Type = operationControl;
    
    // 初始化数据读写结构
    data_read_write_init(&motor->drw);
}

void RobStride_Motor_Init_Offset(RobStride_Motor_t* motor, uint8_t CAN_Id, bool MIT_mode)
{
    RobStride_Motor_Init(motor, CAN_Id, MIT_mode);
}

/*******************************************************************************
* 函数功能  : 接收处理函数
*******************************************************************************/
void RobStride_Motor_Analysis(RobStride_Motor_t* motor, uint8_t* DataFrame, uint32_t ID_ExtId)
{
    if (motor->MIT_Mode)
    {
        // MIT协议使用标准帧，主机ID为0xFD
        if ((ID_ExtId & 0xFF) == 0xFD)
        {
            // 检查是否为故障反馈帧
            if (DataFrame[3] == 0x00 && DataFrame[4] == 0x00 && DataFrame[5] == 0x00 &&
                DataFrame[6] == 0x00 && DataFrame[7] == 0x00)
            {
                // 故障信息在Byte1-2中
                uint16_t fault16 = (uint16_t)(DataFrame[1] << 8) | DataFrame[2];
                motor->error_code = mapFaults(fault16);
            }
            else
            {
                // 正常数据帧解析
                motor->Pos_Info.Angle = uint16_to_float_lz((uint16_t)(DataFrame[1] << 8) | DataFrame[2], P_MIN, P_MAX, 16);
                
                // 速度：12位
                uint16_t speed_raw = (uint16_t)(DataFrame[3] << 4) | (DataFrame[4] >> 4);
                motor->Pos_Info.Speed = uint16_to_float_lz(speed_raw, V_MIN, V_MAX, 12);
                
                // 力矩：12位
                uint16_t torque_raw = (uint16_t)((DataFrame[4] & 0x0F) << 8) | DataFrame[5];
                motor->Pos_Info.Torque = uint16_to_float_lz(torque_raw, T_MIN, T_MAX, 12);
                
                motor->Pos_Info.Temp = (float)((uint16_t)(DataFrame[6] << 8) | DataFrame[7]) * 0.1f;
            }
        }
        else if ((ID_ExtId & 0xFF) == 0xFE)
        {
            // 获取ID应答帧
            memcpy(&motor->Unique_ID, DataFrame, 8);
        }
    }
    else
    {
        // 私有协议解析
        if (((ID_ExtId >> 8) & 0xFF) == motor->CAN_ID)
        {
            uint8_t communication_type = (uint8_t)((ID_ExtId >> 24) & 0x3F);
            
            if (communication_type == 2)
            {
                // 通信类型2：电机反馈数据
                motor->Pos_Info.Angle = uint16_to_float_lz((uint16_t)(DataFrame[0] << 8) | DataFrame[1], P_MIN, P_MAX, 16);
                motor->Pos_Info.Speed = uint16_to_float_lz((uint16_t)(DataFrame[2] << 8) | DataFrame[3], V_MIN, V_MAX, 16);
                motor->Pos_Info.Torque = uint16_to_float_lz((uint16_t)(DataFrame[4] << 8) | DataFrame[5], T_MIN, T_MAX, 16);
                motor->Pos_Info.Temp = (float)((uint16_t)(DataFrame[6] << 8) | DataFrame[7]) * 0.1f;
                motor->error_code = (uint8_t)((ID_ExtId >> 16) & 0x3F);
                motor->Pos_Info.pattern = (uint8_t)((ID_ExtId >> 22) & 0x03);
            }
            else if (communication_type == 17)
            {
                // 通信类型17：参数读取应答
                uint16_t index = (uint16_t)(DataFrame[1] << 8) | DataFrame[0];
                
                for (uint8_t index_num = 0; index_num <= 13; index_num++)
                {
                    if (index == Index_List[index_num])
                    {
                        switch (index_num)
                        {
                            case 0:
                                motor->drw.run_mode.data = (float)DataFrame[4];
                                break;
                            case 1:
                                motor->drw.iq_ref.data = Byte_to_float(DataFrame);
                                break;
                            case 2:
                                motor->drw.spd_ref.data = Byte_to_float(DataFrame);
                                break;
                            case 3:
                                motor->drw.imit_torque.data = Byte_to_float(DataFrame);
                                break;
                            case 4:
                                motor->drw.cur_kp.data = Byte_to_float(DataFrame);
                                break;
                            case 5:
                                motor->drw.cur_ki.data = Byte_to_float(DataFrame);
                                break;
                            case 6:
                                motor->drw.cur_filt_gain.data = Byte_to_float(DataFrame);
                                break;
                            case 7:
                                motor->drw.loc_ref.data = Byte_to_float(DataFrame);
                                break;
                            case 8:
                                motor->drw.limit_spd.data = Byte_to_float(DataFrame);
                                break;
                            case 9:
                                motor->drw.limit_cur.data = Byte_to_float(DataFrame);
                                break;
                            case 10:
                                motor->drw.mechPos.data = Byte_to_float(DataFrame);
                                motor->Pos_Info.Angle = motor->drw.mechPos.data;
                                break;
                            case 11:
                                motor->drw.iqf.data = Byte_to_float(DataFrame);
                                break;
                            case 12:
                                motor->drw.mechVel.data = Byte_to_float(DataFrame);
                                motor->Pos_Info.Speed = motor->drw.mechVel.data;
                                break;
                            case 13:
                                motor->drw.VBUS.data = Byte_to_float(DataFrame);
                                break;
                        }
                        break;
                    }
                }
            }
            else if ((ID_ExtId & 0xFF) == 0xFE)
            {
                // 获取ID应答帧
                motor->CAN_ID = (uint8_t)((ID_ExtId >> 8) & 0xFF);
                memcpy(&motor->Unique_ID, DataFrame, 8);
            }
        }
    }
}

/*******************************************************************************
* 函数功能  : RobStride电机获取设备ID和MCU
* 修改点：参数改为hcan_t* hcan
*******************************************************************************/
void RobStride_Get_CAN_ID(RobStride_Motor_t* motor, hcan_t* hcan)
{
    uint8_t txdata[8] = {0};
    CAN_TxHeaderTypeDef TxMessage;
    
    TxMessage.IDE = CAN_ID_EXT;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 8;
    TxMessage.ExtId = Communication_Type_Get_ID << 24 | (uint32_t)motor->Master_CAN_ID << 8 | motor->CAN_ID;
    
    // 替换为hcan参数
    canx_send_ext_data(hcan, TxMessage.ExtId, txdata, TxMessage.DLC);
}

/*******************************************************************************
* 函数功能  : RobStride电机运控模式
* 修改点：参数改为hcan_t* hcan
*******************************************************************************/
void RobStride_Motor_move_control(RobStride_Motor_t* motor, hcan_t* hcan, float Torque, float Angle, float Speed, float Kp, float Kd)
{
    uint8_t txdata[8] = {0};
    CAN_TxHeaderTypeDef TxMessage;
    
    motor->Motor_Set_All.set_Torque = Torque;
    motor->Motor_Set_All.set_angle = Angle;
    motor->Motor_Set_All.set_speed = Speed;
    motor->Motor_Set_All.set_Kp = Kp;
    motor->Motor_Set_All.set_Kd = Kd;
    
    if (motor->drw.run_mode.data != 0)
    {
        Set_RobStride_Motor_parameter(motor, hcan, 0X7005, move_control_mode, 'j');
        Get_RobStride_Motor_parameter(motor, hcan, 0x7005);
        Enable_Motor(motor, hcan);
        motor->Motor_Set_All.set_motor_mode = move_control_mode;
    }
    
    if (motor->Pos_Info.pattern != 2)
    {
        Enable_Motor(motor, hcan);
    }
    
    TxMessage.IDE = CAN_ID_EXT;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 8;
    TxMessage.ExtId = (uint32_t)Communication_Type_MotionControl << 24 |
                      (uint32_t)float_to_uint_lz(motor->Motor_Set_All.set_Torque, T_MIN, T_MAX, 16) << 8 |
                      motor->CAN_ID;
    
    uint16_t angle_uint = float_to_uint_lz(motor->Motor_Set_All.set_angle, P_MIN, P_MAX, 16);
    uint16_t speed_uint = float_to_uint_lz(motor->Motor_Set_All.set_speed, V_MIN, V_MAX, 16);
    uint16_t kp_uint = float_to_uint_lz(motor->Motor_Set_All.set_Kp, KP_MIN, KP_MAX, 16);
    uint16_t kd_uint = float_to_uint_lz(motor->Motor_Set_All.set_Kd, KD_MIN, KD_MAX, 16);
    
    txdata[0] = (uint8_t)(angle_uint >> 8);
    txdata[1] = (uint8_t)(angle_uint & 0xFF);
    txdata[2] = (uint8_t)(speed_uint >> 8);
    txdata[3] = (uint8_t)(speed_uint & 0xFF);
    txdata[4] = (uint8_t)(kp_uint >> 8);
    txdata[5] = (uint8_t)(kp_uint & 0xFF);
    txdata[6] = (uint8_t)(kd_uint >> 8);
    txdata[7] = (uint8_t)(kd_uint & 0xFF);
    
    // 替换为hcan参数
    canx_send_ext_data(hcan, TxMessage.ExtId, txdata, TxMessage.DLC);
}

/*******************************************************************************
* 函数功能  : RS04 MIT模式使能
* 修改点：参数改为hcan_t* hcan
*******************************************************************************/
void RobStride_Motor_MIT_Enable(RobStride_Motor_t* motor, hcan_t* hcan)
{
    uint8_t txdata[8] = {0};
    CAN_TxHeaderTypeDef txMsg;
    
    txMsg.StdId = motor->CAN_ID;
    txMsg.IDE = CAN_ID_STD;
    txMsg.RTR = CAN_RTR_DATA;
    txMsg.DLC = 8;
    
    txdata[0] = 0xFF;
    txdata[1] = 0xFF;
    txdata[2] = 0xFF;
    txdata[3] = 0xFF;
    txdata[4] = 0xFF;
    txdata[5] = 0xFF;
    txdata[6] = 0xFF;
    txdata[7] = MIT_CMD_ENABLE;
    
    // 替换为hcan参数
    canx_send_data(hcan, (uint16_t)txMsg.StdId, txdata, txMsg.DLC);
}

/*******************************************************************************
* 函数功能  : RS04 MIT模式失能
* 修改点：参数改为hcan_t* hcan
*******************************************************************************/
void RobStride_Motor_MIT_Disable(RobStride_Motor_t* motor, hcan_t* hcan)
{
    uint8_t txdata[8] = {0};
    CAN_TxHeaderTypeDef txMsg;
    
    txMsg.StdId = motor->CAN_ID;
    txMsg.IDE = CAN_ID_STD;
    txMsg.RTR = CAN_RTR_DATA;
    txMsg.DLC = 8;
    
    txdata[0] = 0xFF;
    txdata[1] = 0xFF;
    txdata[2] = 0xFF;
    txdata[3] = 0xFF;
    txdata[4] = 0xFF;
    txdata[5] = 0xFF;
    txdata[6] = 0xFF;
    txdata[7] = MIT_CMD_DISABLE;
    
    // 替换为hcan参数
    canx_send_data(hcan, (uint16_t)txMsg.StdId, txdata, txMsg.DLC);
}

/*******************************************************************************
* 函数功能  : RS04 MIT模式清除或检查错误
* 修改点：参数改为hcan_t* hcan
*******************************************************************************/
void RobStride_Motor_MIT_ClearOrCheckError(RobStride_Motor_t* motor, hcan_t* hcan, uint8_t F_CMD)
{
    uint8_t txdata[8] = {0};
    CAN_TxHeaderTypeDef txMsg;
    
    txMsg.StdId = motor->CAN_ID;
    txMsg.IDE = CAN_ID_STD;
    txMsg.RTR = CAN_RTR_DATA;
    txMsg.DLC = 8;
    
    txdata[0] = 0xFF;
    txdata[1] = 0xFF;
    txdata[2] = 0xFF;
    txdata[3] = 0xFF;
    txdata[4] = 0xFF;
    txdata[5] = 0xFF;
    txdata[6] = F_CMD;
    txdata[7] = MIT_CMD_CLEAR_ERR;
    
    // 替换为hcan参数
    canx_send_data(hcan, (uint16_t)txMsg.StdId, txdata, txMsg.DLC);
}

/*******************************************************************************
* 函数功能  : RS04 MIT设置电机运行模式
* 修改点：参数改为hcan_t* hcan
*******************************************************************************/
void RobStride_Motor_MIT_SetMotorType(RobStride_Motor_t* motor, hcan_t* hcan, uint8_t F_CMD)
{
    uint8_t txdata[8] = {0};
    CAN_TxHeaderTypeDef txMsg;
    
    txMsg.StdId = motor->CAN_ID;
    txMsg.IDE = CAN_ID_STD;
    txMsg.RTR = CAN_RTR_DATA;
    txMsg.DLC = 8;
    
    txdata[0] = 0xFF;
    txdata[1] = 0xFF;
    txdata[2] = 0xFF;
    txdata[3] = 0xFF;
    txdata[4] = 0xFF;
    txdata[5] = 0xFF;
    txdata[6] = F_CMD;
    txdata[7] = MIT_CMD_SET_MODE;
    
    // 替换为hcan参数
    canx_send_data(hcan, (uint16_t)txMsg.StdId, txdata, txMsg.DLC);
}

/*******************************************************************************
* 函数功能  : RS04 MIT设置电机ID
* 修改点：参数改为hcan_t* hcan
*******************************************************************************/
void RobStride_Motor_MIT_SetMotorId(RobStride_Motor_t* motor, hcan_t* hcan, uint8_t F_CMD)
{
    uint8_t txdata[8] = {0};
    CAN_TxHeaderTypeDef txMsg;
    
    txMsg.StdId = motor->CAN_ID;
    txMsg.IDE = CAN_ID_STD;
    txMsg.RTR = CAN_RTR_DATA;
    txMsg.DLC = 8;
    
    txdata[0] = 0xFF;
    txdata[1] = 0xFF;
    txdata[2] = 0xFF;
    txdata[3] = 0xFF;
    txdata[4] = 0xFF;
    txdata[5] = 0xFF;
    txdata[6] = F_CMD;
    txdata[7] = 0xFA; // 设置ID命令
    
    // 替换为hcan参数
    canx_send_data(hcan, (uint16_t)txMsg.StdId, txdata, txMsg.DLC);
}

/*******************************************************************************
* 函数功能  : RS04 MIT运控模式控制指令
* 修改点：参数改为hcan_t* hcan
*******************************************************************************/
void RobStride_Motor_MIT_Control(RobStride_Motor_t* motor, hcan_t* hcan, float Angle, float Speed, float Kp, float Kd, float Torque)
{
    uint8_t txdata[8] = {0};
    CAN_TxHeaderTypeDef txMsg;
    
    txMsg.StdId = motor->CAN_ID;
    txMsg.IDE = CAN_ID_STD;
    txMsg.RTR = CAN_RTR_DATA;
    txMsg.DLC = 8;
    
    uint16_t angle_uint = float_to_uint_lz(Angle, P_MIN, P_MAX, 16);
    txdata[0] = (uint8_t)(angle_uint >> 8);
    txdata[1] = (uint8_t)(angle_uint & 0xFF);
    
    uint16_t speed_uint = float_to_uint_lz(Speed, V_MIN, V_MAX, 12);
    txdata[2] = (uint8_t)(speed_uint >> 4);
    txdata[3] = (uint8_t)((speed_uint << 4) | (float_to_uint_lz(Kp, KP_MIN, KP_MAX, 12) >> 8));
    
    uint16_t kp_uint = float_to_uint_lz(Kp, KP_MIN, KP_MAX, 12);
    txdata[4] = (uint8_t)(kp_uint & 0xFF);
    
    uint16_t kd_uint = float_to_uint_lz(Kd, KD_MIN, KD_MAX, 12);
    txdata[5] = (uint8_t)(kd_uint >> 4);
    txdata[6] = (uint8_t)((kd_uint << 4) | (float_to_uint_lz(Torque, T_MIN, T_MAX, 12) >> 8));
    
    uint16_t torque_uint = float_to_uint_lz(Torque, T_MIN, T_MAX, 12);
    txdata[7] = (uint8_t)(torque_uint & 0xFF);
    
    // 替换为hcan参数
    canx_send_data(hcan, (uint16_t)txMsg.StdId, txdata, txMsg.DLC);
}

/*******************************************************************************
* 函数功能  : RS04 MIT位置模式控制指令
* 修改点：参数改为hcan_t* hcan
*******************************************************************************/
void RobStride_Motor_MIT_PositionControl(RobStride_Motor_t* motor, hcan_t* hcan, float position_rad, float speed_rad_per_s)
{
    uint8_t txdata[8] = {0};
    CAN_TxHeaderTypeDef txMsg;
    
    txMsg.StdId = (1 << 8) | motor->CAN_ID;
    txMsg.IDE = CAN_ID_STD;
    txMsg.RTR = CAN_RTR_DATA;
    txMsg.DLC = 8;
    
    memcpy(&txdata[0], &position_rad, 4);
    memcpy(&txdata[4], &speed_rad_per_s, 4);
    
    // 替换为hcan参数
    canx_send_data(hcan, (uint16_t)txMsg.StdId, txdata, txMsg.DLC);
}

/*******************************************************************************
* 函数功能  : RS04 MIT速度模式控制指令
* 修改点：参数改为hcan_t* hcan
*******************************************************************************/
void RobStride_Motor_MIT_SpeedControl(RobStride_Motor_t* motor, hcan_t* hcan, float speed_rad_per_s, float current_limit)
{
    uint8_t txdata[8] = {0};
    CAN_TxHeaderTypeDef txMsg;
    
    txMsg.StdId = (2 << 8) | motor->CAN_ID;
    txMsg.IDE = CAN_ID_STD;
    txMsg.RTR = CAN_RTR_DATA;
    txMsg.DLC = 8;
    
    memcpy(&txdata[0], &speed_rad_per_s, 4);
    memcpy(&txdata[4], &current_limit, 4);
    
    // 替换为hcan参数
    canx_send_data(hcan, (uint16_t)txMsg.StdId, txdata, txMsg.DLC);
}

/*******************************************************************************
* 函数功能  : RS04 MIT零点设置模式
* 修改点：参数改为hcan_t* hcan
*******************************************************************************/
void RobStride_Motor_MIT_SetZeroPos(RobStride_Motor_t* motor, hcan_t* hcan)
{
    uint8_t txdata[8] = {0};
    CAN_TxHeaderTypeDef txMsg;
    
    txMsg.StdId = motor->CAN_ID;
    txMsg.IDE = CAN_ID_STD;
    txMsg.RTR = CAN_RTR_DATA;
    txMsg.DLC = 8;
    
    txdata[0] = 0xFF;
    txdata[1] = 0xFF;
    txdata[2] = 0xFF;
    txdata[3] = 0xFF;
    txdata[4] = 0xFF;
    txdata[5] = 0xFF;
    txdata[6] = 0xFF;
    txdata[7] = MIT_CMD_SET_ZERO;
    
    // 替换为hcan参数
    canx_send_data(hcan, (uint16_t)txMsg.StdId, txdata, txMsg.DLC);
}

/*******************************************************************************
* 函数功能  : RS04 MIT设置协议类型
* 修改点：参数改为hcan_t* hcan
*******************************************************************************/
void RobStride_Motor_MIT_SetProtocol(RobStride_Motor_t* motor, hcan_t* hcan, uint8_t protocol_type)
{
    uint8_t txdata[8] = {0};
    CAN_TxHeaderTypeDef txMsg;
    
    txMsg.StdId = motor->CAN_ID;
    txMsg.IDE = CAN_ID_STD;
    txMsg.RTR = CAN_RTR_DATA;
    txMsg.DLC = 8;
    
    txdata[0] = 0xFF;
    txdata[1] = 0xFF;
    txdata[2] = 0xFF;
    txdata[3] = 0xFF;
    txdata[4] = 0xFF;
    txdata[5] = 0xFF;
    txdata[6] = protocol_type;
    txdata[7] = MIT_CMD_SET_PROTOCOL;
    
    // 替换为hcan参数
    canx_send_data(hcan, (uint16_t)txMsg.StdId, txdata, txMsg.DLC);
}

/*******************************************************************************
* 函数功能  : RobStride电机位置模式(PP插补位置模式控制)
* 修改点：参数改为hcan_t* hcan
*******************************************************************************/
void RobStride_Motor_Pos_control(RobStride_Motor_t* motor, hcan_t* hcan, float Speed, float Angle)
{
    motor->Motor_Set_All.set_speed = Speed;
    motor->Motor_Set_All.set_angle = Angle;
    
    if (motor->drw.run_mode.data != 1)
    {
        Set_RobStride_Motor_parameter(motor, hcan, 0X7005, Pos_control_mode, 'j');
        Get_RobStride_Motor_parameter(motor, hcan, 0x7005);
        motor->Motor_Set_All.set_motor_mode = Pos_control_mode;
        Enable_Motor(motor, hcan);
        Set_RobStride_Motor_parameter(motor, hcan, 0X7024, motor->Motor_Set_All.set_limit_speed, 'p');
        Set_RobStride_Motor_parameter(motor, hcan, 0X7025, motor->Motor_Set_All.set_acceleration, 'p');
    }
    
    Set_RobStride_Motor_parameter(motor, hcan, 0X7016, motor->Motor_Set_All.set_angle, 'p');
}

/*******************************************************************************
* 函数功能  : RobStride电机位置模式(CSP位置模式控制)
* 修改点：参数改为hcan_t* hcan
*******************************************************************************/
void RobStride_Motor_CSP_control(RobStride_Motor_t* motor, hcan_t* hcan, float Angle, float limit_spd)
{
    if (motor->MIT_Mode)
    {
        RobStride_Motor_MIT_PositionControl(motor, hcan, Angle, limit_spd);
    }
    else
    {
        motor->Motor_Set_All.set_angle = Angle;

        if (limit_spd != limit_spd)
        {
            limit_spd = 0.0f;
        }

        if (limit_spd < 0.0f)
        {
            limit_spd = -limit_spd;
        }

        if (limit_spd > CSP_LIMIT_SPD_MAX)
        {
            limit_spd = CSP_LIMIT_SPD_MAX;
        }

        motor->Motor_Set_All.set_limit_speed = limit_spd;
        
        if (motor->drw.run_mode.data != CSP_control_mode)
        {
            Set_RobStride_Motor_parameter(motor, hcan, 0X7005, CSP_control_mode, 'j');
            Get_RobStride_Motor_parameter(motor, hcan, 0x7005);
            Enable_Motor(motor, hcan);
        }

        if (motor->drw.limit_spd.data != motor->Motor_Set_All.set_limit_speed)
        {
            Set_RobStride_Motor_parameter(motor, hcan, 0X7017, motor->Motor_Set_All.set_limit_speed, 'p');
        }
        
        Set_RobStride_Motor_parameter(motor, hcan, 0X7016, motor->Motor_Set_All.set_angle, 'p');
    }
}

/*******************************************************************************
* 函数功能  : RobStride电机速度模式
* 修改点：参数改为hcan_t* hcan
*******************************************************************************/
void RobStride_Motor_Speed_control(RobStride_Motor_t* motor, hcan_t* hcan, float Speed, float limit_cur)
{
    motor->Motor_Set_All.set_speed = Speed;
    motor->Motor_Set_All.set_limit_cur = limit_cur;
    
    if (motor->drw.run_mode.data != 2)
    {
        Set_RobStride_Motor_parameter(motor, hcan, 0X7005, Speed_control_mode, 'j');
        Get_RobStride_Motor_parameter(motor, hcan, 0x7005);
        Enable_Motor(motor, hcan);
        motor->Motor_Set_All.set_motor_mode = Speed_control_mode;
        Set_RobStride_Motor_parameter(motor, hcan, 0X7018, motor->Motor_Set_All.set_limit_cur, 'p');
        Set_RobStride_Motor_parameter(motor, hcan, 0X7022, 10, 'p');
    }
    
    Set_RobStride_Motor_parameter(motor, hcan, 0X700A, motor->Motor_Set_All.set_speed, 'p');
}

/*******************************************************************************
* 函数功能  : RobStride电机电流模式
* 修改点：参数改为hcan_t* hcan
*******************************************************************************/
void RobStride_Motor_current_control(RobStride_Motor_t* motor, hcan_t* hcan, float current)
{
    motor->Motor_Set_All.set_current = current;
    motor->output = motor->Motor_Set_All.set_current;
    
    if (motor->Motor_Set_All.set_motor_mode != 3)
    {
        Set_RobStride_Motor_parameter(motor, hcan, 0X7005, Elect_control_mode, 'j');
        Get_RobStride_Motor_parameter(motor, hcan, 0x7005);
        motor->Motor_Set_All.set_motor_mode = Elect_control_mode;
        Enable_Motor(motor, hcan);
    }
    
    Set_RobStride_Motor_parameter(motor, hcan, 0X7006, motor->Motor_Set_All.set_current, 'p');
}

/*******************************************************************************
* 函数功能  : RobStride电机零点模式
* 修改点：参数改为hcan_t* hcan
*******************************************************************************/
void RobStride_Motor_Set_Zero_control(RobStride_Motor_t* motor, hcan_t* hcan)
{
    Set_RobStride_Motor_parameter(motor, hcan, 0X7005, Set_Zero_mode, 'j');
}

/*******************************************************************************
* 函数功能  : RobStride电机使能
* 修改点：参数改为hcan_t* hcan
*******************************************************************************/
void Enable_Motor(RobStride_Motor_t* motor, hcan_t* hcan)
{
    if (motor->MIT_Mode)
    {
        RobStride_Motor_MIT_Enable(motor, hcan);
    }
    else
    {
        uint8_t txdata[8] = {0};
        CAN_TxHeaderTypeDef TxMessage;
        
        TxMessage.IDE = CAN_ID_EXT;
        TxMessage.RTR = CAN_RTR_DATA;
        TxMessage.DLC = 8;
        TxMessage.ExtId = (uint32_t)Communication_Type_MotorEnable << 24 | 
                          (uint32_t)motor->Master_CAN_ID << 8 | 
                          motor->CAN_ID;
        
        // 替换为hcan参数
        canx_send_ext_data(hcan, TxMessage.ExtId, txdata, TxMessage.DLC);
    }
}

/*******************************************************************************
* 函数功能  : RobStride电机失能
* 修改点：参数改为hcan_t* hcan
*******************************************************************************/
void Disenable_Motor(RobStride_Motor_t* motor, hcan_t* hcan, uint8_t clear_error)
{
    if (motor->MIT_Mode)
    {
        RobStride_Motor_MIT_Disable(motor, hcan);
    }
    else
    {
        uint8_t txdata[8] = {0};
        CAN_TxHeaderTypeDef TxMessage;
        
        txdata[0] = clear_error;
        TxMessage.IDE = CAN_ID_EXT;
        TxMessage.RTR = CAN_RTR_DATA;
        TxMessage.DLC = 8;
        TxMessage.ExtId = (uint32_t)Communication_Type_MotorStop << 24 | 
                          (uint32_t)motor->Master_CAN_ID << 8 | 
                          motor->CAN_ID;
        
        // 替换为hcan参数
        canx_send_ext_data(hcan, TxMessage.ExtId, txdata, TxMessage.DLC);
        Set_RobStride_Motor_parameter(motor, hcan, 0X7005, move_control_mode, 'j');
    }
}

/*******************************************************************************
* 函数功能  : RobStride电机写入参数
* 修改点：参数改为hcan_t* hcan
*******************************************************************************/
void Set_RobStride_Motor_parameter(RobStride_Motor_t* motor, hcan_t* hcan, uint16_t Index, float Value, char Value_mode)
{
    uint8_t txdata[8] = {0};
    CAN_TxHeaderTypeDef TxMessage;
    
    TxMessage.IDE = CAN_ID_EXT;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 8;
    TxMessage.ExtId = (uint32_t)Communication_Type_SetSingleParameter << 24 | 
                      (uint32_t)motor->Master_CAN_ID << 8 | 
                      motor->CAN_ID;
    
    txdata[0] = (uint8_t)(Index & 0xFF);
    txdata[1] = (uint8_t)(Index >> 8);
    txdata[2] = 0x00;
    txdata[3] = 0x00;
    
    if (Value_mode == 'p')
    {
        memcpy(&txdata[4], &Value, 4);
        if (Index == 0x7016U)
        {
            motor->drw.loc_ref.data = Value;
        }
        else if (Index == 0x7017U)
        {
            motor->drw.limit_spd.data = Value;
        }
    }
    else if (Value_mode == 'j')
    {
        motor->Motor_Set_All.set_motor_mode = (uint8_t)Value;
        if (Index == 0x7005U)
        {
            /* 本地同步运行模式，避免等待参数回读期间重复发送整套切模指令。 */
            motor->drw.run_mode.data = Value;
        }
        txdata[4] = (uint8_t)Value;
        txdata[5] = 0x00;
        txdata[6] = 0x00;
        txdata[7] = 0x00;
    }
    
    // 替换为hcan参数
    canx_send_ext_data(hcan, TxMessage.ExtId, txdata, TxMessage.DLC);
}

/*******************************************************************************
* 函数功能  : RobStride电机单个参数读取
* 修改点：参数改为hcan_t* hcan
*******************************************************************************/
void Get_RobStride_Motor_parameter(RobStride_Motor_t* motor, hcan_t* hcan, uint16_t Index)
{
    uint8_t txdata[8] = {0};
    CAN_TxHeaderTypeDef TxMessage;
    
    txdata[0] = (uint8_t)(Index & 0xFF);
    txdata[1] = (uint8_t)(Index >> 8);
    
    TxMessage.IDE = CAN_ID_EXT;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 8;
    TxMessage.ExtId = (uint32_t)Communication_Type_GetSingleParameter << 24 | 
                      (uint32_t)motor->Master_CAN_ID << 8 | 
                      motor->CAN_ID;
    
    // 替换为hcan参数
    canx_send_ext_data(hcan, TxMessage.ExtId, txdata, TxMessage.DLC);
}

/*******************************************************************************
* 函数功能  : RobStride电机设置CAN_ID
* 修改点：参数改为hcan_t* hcan
*******************************************************************************/
void Set_CAN_ID(RobStride_Motor_t* motor, hcan_t* hcan, uint8_t Set_CAN_ID)
{
    Disenable_Motor(motor, hcan, 0);
    
    uint8_t txdata[8] = {0};
    CAN_TxHeaderTypeDef TxMessage;
    
    TxMessage.IDE = CAN_ID_EXT;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 8;
    TxMessage.ExtId = (uint32_t)Communication_Type_Can_ID << 24 | 
                      (uint32_t)Set_CAN_ID << 16 | 
                      (uint32_t)motor->Master_CAN_ID << 8 | 
                      motor->CAN_ID;
    
    // 替换为hcan参数
    canx_send_ext_data(hcan, TxMessage.ExtId, txdata, TxMessage.DLC);
}

/*******************************************************************************
* 函数功能  : RobStride电机设置机械零点
* 修改点：参数改为hcan_t* hcan
*******************************************************************************/
void Set_ZeroPos(RobStride_Motor_t* motor, hcan_t* hcan)
{
    Disenable_Motor(motor, hcan, 0);
    
    uint8_t txdata[8] = {0};
    CAN_TxHeaderTypeDef TxMessage;
    
    TxMessage.IDE = CAN_ID_EXT;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 8;
    TxMessage.ExtId = (uint32_t)Communication_Type_SetPosZero << 24 | 
                      (uint32_t)motor->Master_CAN_ID << 8 | 
                      motor->CAN_ID;
    
    txdata[0] = 1;
    // 替换为hcan参数
    canx_send_ext_data(hcan, TxMessage.ExtId, txdata, TxMessage.DLC);
    
    Enable_Motor(motor, hcan);
}

/*******************************************************************************
* 函数功能  : RobStride电机数据保存
* 修改点：参数改为hcan_t* hcan
*******************************************************************************/
void RobStride_Motor_MotorDataSave(RobStride_Motor_t* motor, hcan_t* hcan)
{
    uint8_t txdata[8] = {0};
    CAN_TxHeaderTypeDef TxMessage;
    
    TxMessage.IDE = CAN_ID_EXT;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 8;
    TxMessage.ExtId = (uint32_t)Communication_Type_MotorDataSave << 24 | 
                      (uint32_t)motor->Master_CAN_ID << 8 | 
                      motor->CAN_ID;
    
    txdata[0] = 0x01;
    txdata[1] = 0x02;
    txdata[2] = 0x03;
    txdata[3] = 0x04;
    txdata[4] = 0x05;
    txdata[5] = 0x06;
    txdata[6] = 0x07;
    txdata[7] = 0x08;
    
    // 替换为hcan参数
    canx_send_ext_data(hcan, TxMessage.ExtId, txdata, TxMessage.DLC);
}

/*******************************************************************************
* 函数功能  : RobStride电机波特率修改
* 修改点：参数改为hcan_t* hcan
*******************************************************************************/
void RobStride_Motor_BaudRateChange(RobStride_Motor_t* motor, hcan_t* hcan, uint8_t F_CMD)
{
    uint8_t txdata[8] = {0};
    CAN_TxHeaderTypeDef TxMessage;
    
    TxMessage.IDE = CAN_ID_EXT;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 8;
    TxMessage.ExtId = (uint32_t)Communication_Type_BaudRateChange << 24 | 
                      (uint32_t)motor->Master_CAN_ID << 8 | 
                      motor->CAN_ID;
    
    txdata[0] = 0x01;
    txdata[1] = 0x02;
    txdata[2] = 0x03;
    txdata[3] = 0x04;
    txdata[4] = 0x05;
    txdata[5] = 0x06;
    txdata[6] = F_CMD;
    txdata[7] = 0x08;
    
    // 替换为hcan参数
    canx_send_ext_data(hcan, TxMessage.ExtId, txdata, TxMessage.DLC);
}

/*******************************************************************************
* 函数功能  : RobStride电机主动上报设置
* 修改点：参数改为hcan_t* hcan
*******************************************************************************/
void RobStride_Motor_ProactiveEscalationSet(RobStride_Motor_t* motor, hcan_t* hcan, uint8_t F_CMD)
{
    uint8_t txdata[8] = {0};
    CAN_TxHeaderTypeDef TxMessage;
    
    TxMessage.IDE = CAN_ID_EXT;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 8;
    TxMessage.ExtId = (uint32_t)Communication_Type_ProactiveEscalationSet << 24 | 
                      (uint32_t)motor->Master_CAN_ID << 8 | 
                      motor->CAN_ID;
    
    txdata[0] = 0x01;
    txdata[1] = 0x02;
    txdata[2] = 0x03;
    txdata[3] = 0x04;
    txdata[4] = 0x05;
    txdata[5] = 0x06;
    txdata[6] = F_CMD;
    txdata[7] = 0x08;
    
    // 替换为hcan参数
    canx_send_ext_data(hcan, TxMessage.ExtId, txdata, TxMessage.DLC);
}

/*******************************************************************************
* 函数功能  : RobStride电机协议修改
* 修改点：参数改为hcan_t* hcan
*******************************************************************************/
void RobStride_Motor_MotorModeSet(RobStride_Motor_t* motor, hcan_t* hcan, uint8_t F_CMD)
{
    uint8_t txdata[8] = {0};
    CAN_TxHeaderTypeDef TxMessage;
    
    TxMessage.IDE = CAN_ID_EXT;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 8;
    TxMessage.ExtId = (uint32_t)Communication_Type_MotorModeSet << 24 | 
                      (uint32_t)motor->Master_CAN_ID << 8 | 
                      motor->CAN_ID;
    
    txdata[0] = 0x01;
    txdata[1] = 0x02;
    txdata[2] = 0x03;
    txdata[3] = 0x04;
    txdata[4] = 0x05;
    txdata[5] = 0x06;
    txdata[6] = F_CMD;
    txdata[7] = 0x08;
    
    // 替换为hcan参数
    canx_send_ext_data(hcan, TxMessage.ExtId, txdata, TxMessage.DLC);
}

/*******************************************************************************
* 函数功能  : 数据读写结构体初始化
*******************************************************************************/
void data_read_write_init(data_read_write_t* drw)
{
    drw->run_mode.index = Index_List[0];
    drw->iq_ref.index = Index_List[1];
    drw->spd_ref.index = Index_List[2];
    drw->imit_torque.index = Index_List[3];
    drw->cur_kp.index = Index_List[4];
    drw->cur_ki.index = Index_List[5];
    drw->cur_filt_gain.index = Index_List[6];
    drw->loc_ref.index = Index_List[7];
    drw->limit_spd.index = Index_List[8];
    drw->limit_cur.index = Index_List[9];
    drw->mechPos.index = Index_List[10];
    drw->iqf.index = Index_List[11];
    drw->mechVel.index = Index_List[12];
    drw->VBUS.index = Index_List[13];
    drw->rotation.index = Index_List[14];
}

/*******************************************************************************
* 函数功能  : 获取MIT模式状态
*******************************************************************************/
bool Get_MIT_Mode(RobStride_Motor_t* motor)
{
    return motor->MIT_Mode;
}

/*******************************************************************************
* 函数功能  : 获取MIT类型
*******************************************************************************/
enum MIT_TYPE get_MIT_Type(RobStride_Motor_t* motor)
{
    return motor->MIT_Type;
}
