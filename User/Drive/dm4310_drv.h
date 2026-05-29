#ifndef __DM4310_DRV_H__
#define __DM4310_DRV_H__
#include "main.h"
#include "fdcan.h"
#include "can_receive_send.h"
#define MIT_MODE 0x000
#define POS_MODE 0x100
#define SPEED_MODE 0x200
// 4310
#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -30.0f
#define V_MAX 30.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -10.0f
#define T_MAX 10.0f
// 8009
#define P_MIN2 -12.5f
#define P_MAX2 12.5f
#define V_MIN2 -45.0f
#define V_MAX2 45.0f
#define KP_MIN2 0.0f
#define KP_MAX2 500.0f
#define KD_MIN2 0.0f
#define KD_MAX2 5.0f
#define T_MIN2 -54.0f
#define T_MAX2 54.0f
// 10010l
#define P_MIN3 -12.5f
#define P_MAX3 12.5f
#define V_MIN3 -25.0f
#define V_MAX3 25.0f
#define KP_MIN3 0.0f
#define KP_MAX3 500.0f
#define KD_MIN3 0.0f
#define KD_MAX3 5.0f
#define T_MIN3 -200.0f
#define T_MAX3 200.0f
// 达妙参数到遥控器的转换
#define SBUS_POS 3.1415926f / 732.0000000f
typedef struct
{
	uint16_t id;
	uint16_t state;
	int p_int;
	int v_int;
	int t_int;
	int kp_int;
	int kd_int;
	float pos; // 电机位置
	float vel; // 电机速度
	float tor; // 扭矩
	float Kp;
	float Kd;
	float Tmos; // Mos的平均温度
	float Tcoil;
} motor_fbpara_t;
typedef struct
{
	uint16_t set_mode;
	float pos_set;
	float vel_set;
	float tor_set;
	float cur_set;
	float kp_set;
	float kd_set;
} motor_ctrl_t;

typedef struct
{
	uint8_t read_flag;
	uint8_t write_flag;
	uint8_t save_flag;

	float UV_Value;	  // 低压保护值
	float KT_Value;	  // 扭矩系数
	float OT_Value;	  // 过温保护值
	float OC_Value;	  // 过流保护值
	float ACC;		  // 加速度
	float DEC;		  // 减速度
	float MAX_SPD;	  // 最大速度
	uint32_t MST_ID;  // 反馈ID
	uint32_t ESC_ID;  // 接收ID
	uint32_t TIMEOUT; // 超时警报时间
	uint32_t cmode;	  // 控制模式
	float Damp;		  // 电机粘滞系数
	float Inertia;	  // 电机转动惯量
	uint32_t hw_ver;  // 保留
	uint32_t sw_ver;  // 软件版本号
	uint32_t SN;	  // 保留
	uint32_t NPP;	  // 电机极对数
	float Rs;		  // 电阻
	float Ls;		  // 电感
	float Flux;		  // 磁链
	float Gr;		  // 齿轮减速比
	float PMAX;		  // 位置映射范围
	float VMAX;		  // 速度映射范围
	float TMAX;		  // 扭矩映射范围
	float I_BW;		  // 电流环控制带宽
	float KP_ASR;	  // 速度环Kp
	float KI_ASR;	  // 速度环Ki
	float KP_APR;	  // 位置环Kp
	float KI_APR;	  // 位置环Ki
	float OV_Value;	  // 过压保护值
	float GREF;		  // 齿轮力矩效率
	float Deta;		  // 速度环阻尼系数
	float V_BW;		  // 速度环滤波带宽
	float IQ_cl;	  // 电流环增强系数
	float VL_cl;	  // 速度环增强系数
	uint32_t can_br;  // CAN波特率代码
	uint32_t sub_ver; // 子版本号
	float u_off;	  // u相偏置
	float v_off;	  // v相偏置
	float k1;		  // 补偿因子1
	float k2;		  // 补偿因子2
	float m_off;	  // 角度偏移
	float dir;		  // 方向
	float p_m;		  // 电机位置
	float x_out;	  // 输出轴位置
} esc_inf_t;

typedef enum
{
	Motor1,
	Motor2,
	Motor3,
	Motor4,
	Motor5,
	Motor6,
	Motor7,
	Motor8,
	Motor9,
	Motor10,
	num
} motor_num;

typedef enum
{
	mit_mode = 1,
	pos_mode = 2,
	spd_mode = 3,
	psi_mode = 4
} mode_e;

typedef struct
{
	uint16_t mode;
	motor_fbpara_t para; // 反馈的数据
	motor_ctrl_t ctrl;	 // 发送的数据

	uint16_t mst_id;
	esc_inf_t tmp;
} Arm_Motor_t;

typedef struct
{

	float MT04;
	float MT05;
	float MT06;
} POS_Motor;

typedef struct
{

	float MT04;
	float MT05;
	float MT06;

} VEL_Motor;

typedef struct
{
	float tor_rootpitch;
	float tor_rootyaw;
	float tor_middleyaw;
	float tor_middlerow;
	float tor_endyaw;
	float tor_endrow;
} TOR_Motor;
extern Arm_Motor_t arm_motor[num];
extern POS_Motor pos_motor;
extern VEL_Motor vel_motor;
extern TOR_Motor tor_motor;

extern float Hex_To_Float(uint32_t *Byte, int num); // 十六进制到浮点数
extern uint32_t FloatTohex(float HEX);				// 浮点数到十六进制转换

extern float uint_to_float(int x_int, float x_min, float x_max, int bits);
extern int float_to_uint(float x_float, float x_min, float x_max, int bits);

extern void arm_motor_init(Arm_Motor_t *motor, uint16_t id, uint16_t mode);
extern void enable_motor_mode(hcan_t *hcan, uint16_t motor_id, uint16_t mode_id);
extern void disable_motor_mode(hcan_t *hcan, uint16_t motor_id, uint16_t mode_id);
extern void damiao_fbdata(Arm_Motor_t *motor, uint8_t *rx_data);
extern void dm4310_fbdata(Arm_Motor_t *motor, uint8_t *rx_data, uint32_t data_len);
extern void dm8009_fbdata(Arm_Motor_t *motor, uint8_t *rx_data, uint32_t data_len);
extern void dm10010l_fbdata(Arm_Motor_t *motor, uint8_t *rx_data, uint32_t data_len);

void set_DM_pos_vel(float pos, float vel, uint16_t motor_id);
void set_DM_vel(float vel, uint16_t motor_id);
void set_DM_mit(float pos, float vel, float torq, float kp, float kd, uint16_t motor_id);
void set_DM_mode(uint16_t motor_id, uint16_t mode_id);

// 关节电机
extern void mit_ctrl(hcan_t *hcan, uint16_t motor_id, float pos, float vel, float kp, float kd, float torq);
extern void pos_speed_ctrl(hcan_t *hcan, uint16_t motor_id, float pos, float vel);
extern void speed_ctrl(hcan_t *hcan, uint16_t motor_id, float _vel);

// 电机使能相关
extern void CAN_Send_Enter(hcan_t *hcan, uint16_t motor_id);
extern void CAN_Send_Exit(hcan_t *hcan, uint16_t motor_id);
extern void CAN_Send_Clear_Error(hcan_t *hcan, uint16_t motor_id);
extern void CAN_Send_Save_Zero(hcan_t *hcan, uint16_t motor_id);

#endif /* __DM4310_DRV_H__ */
