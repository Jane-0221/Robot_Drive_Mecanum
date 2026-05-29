#include "Kalman_Filter_c.h"
#include "main.h"
// #include "USB_VirCom.h"
// #include "NUC_communication.h"

// 偏航角（Yaw）自动瞄准卡尔曼滤波器实例
KF_t yaw_auto_kf;
// 俯仰角（Pitch）自动瞄准卡尔曼滤波器实例
KF_t pitch_auto_kf;
/* 鼠标滤波相关 */
// 鼠标X轴卡尔曼滤波器实例
KF_t mouse_x_kf_fliter;
// 鼠标Y轴卡尔曼滤波器实例
KF_t mouse_y_kf_fliter;

/**
  * @name   KalmanCreate
  * @brief  创建一个卡尔曼滤波器实例并初始化参数
  * @param  p:   卡尔曼滤波器结构体指针（待初始化）
  *         T_Q: 系统过程噪声协方差（反映模型预测的不确定性）
  *         T_R: 测量噪声协方差（反映传感器测量的不确定性）
  * @retval 无返回值；T_Q和T_R的绝对值大小无绝对标准，需根据R/Q的比值调整预测参数，
  *         建议先固定其中一个为1，调试另一个参数
  * @attention R越大，代表越信任传感器的测量值，Q无穷大时仅使用测量值；
  *            反之，R越小代表越信任模型预测值，Q为0时仅使用模型预测值
  */
void KalmanCreate(extKalman_t *p,float T_Q,float T_R)
{
    p->X_last = (float)0;
    p->P_last = 0;
    p->Q = T_Q;
    p->R = T_R;
    p->A = 1;
    p->B = 0;
    p->H = 1;
    p->X_mid = p->X_last;
}

/**
  * @name   KalmanFilter
  * @brief  卡尔曼滤波核心计算函数（标准一阶卡尔曼实现）
  * @param  p:   已初始化的卡尔曼滤波器结构体指针
  *         dat: 待滤波的原始测量数据（如传感器输入）
  * @retval 滤波后的最优估计值
  * @attention Z(k) 是系统输入（测量值），X(k|k) 是卡尔曼滤波后的输出（最优估计值）；
  *            A=1、B=0、H=1、I=1 为一阶卡尔曼默认参数；
  *            W(K)、V(k) 是过程/测量高斯白噪声，叠加在测量值上，可忽略；
  *            以下是卡尔曼滤波的5个核心公式（代码对应实现）：
  *            1. 状态预测：x(k|k-1) = A*X(k-1|k-1)+B*U(k)+W(K)
  *            2. 协方差预测：p(k|k-1) = A*p(k-1|k-1)*A'+Q
  *            3. 卡尔曼增益：kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)
  *            4. 状态更新：x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))
  *            5. 协方差更新：p(k|k) = (I-kg(k)*H)*P(k|k-1)
  */
float KalmanFilter(extKalman_t* p,float dat)
{
    p->X_mid =p->A*p->X_last;                     // 对应公式(1) 状态预测方程
    p->P_mid = p->A*p->P_last+p->Q;               // 对应公式(2) 协方差预测方程
    p->kg = p->P_mid/(p->P_mid+p->R);             // 对应公式(4) 卡尔曼增益计算
    p->X_now = p->X_mid + p->kg*(dat-p->X_mid);   // 对应公式(3) 状态更新方程
    p->P_now = (1-p->kg)*p->P_mid;                // 对应公式(5) 协方差更新方程
    p->P_last = p->P_now;                         // 更新协方差，供下次迭代使用
    p->X_last = p->X_now;                         // 更新状态值，供下次迭代使用
    return p->X_now;							  // 输出最优估计值x(k|k)
}

/**
  * @name   Kalman_Init
  * @brief  卡尔曼滤波器全局初始化函数
  * @param  无
  * @retval 无
  * @attention 初始化偏航角、俯仰角的卡尔曼滤波器参数（Q=1，R=1）；
  *            鼠标滤波相关初始化暂注释，可根据需求启用
  */
void Kalman_Init(void)
{
    // 初始化偏航角自动瞄准卡尔曼滤波器（Q=1，R=1）
    KalmanCreate(&yaw_auto_kf.Angle_KF , 1,1);
    // 初始化俯仰角自动瞄准卡尔曼滤波器（Q=1，R=1）
    KalmanCreate(&pitch_auto_kf.Angle_KF , 1,1);
    // 初始化鼠标X轴卡尔曼滤波器（Q=1，R=10），暂注释
//	KalmanCreate(&mouse_x_kf_fliter.Angle_KF, 1,10);// 鼠标X轴卡尔曼滤波
//	KalmanCreate(&mouse_y_kf_fliter.Angle_KF, 1,10);// 鼠标Y轴卡尔曼滤波
}

/**
  * @name   AutoAim_Algorithm
  * @brief  自动瞄准卡尔曼滤波算法（偏航角/Yaw专用）
  * @param  str: 卡尔曼滤波器结构体指针（对应目标轴的滤波器实例）
  *         input: 原始测量输入值（如传感器采集的偏航角）
  * @retval 滤波后的最优估计值（当前仅对角度进行卡尔曼预测）
  */
float AutoAim_Algorithm(KF_t *str,float input)//yaw
{
		float res;
		str->Angle =KalmanFilter(&str->Angle_KF,input);
		str->Out = str->Angle;
		res = str->Out;// 当前仅对角度值进行卡尔曼预测
	return res;
}

// /**
//   * @name   AutoAim_pitch_Algorithm
//   * @brief  自动瞄准俯仰角算法（暂未实现卡尔曼滤波）
//   * @param  str: 卡尔曼滤波器结构体指针
//   * @retval 原始俯仰角值（直接读取NUC通信的pitch值）
//   * @attention 该函数暂仅读取原始值，无滤波处理，可后续补充卡尔曼逻辑
//   */
// float AutoAim_pitch_Algorithm(KF_t *str)//pitch
// {
// 	float res;
// 	str->Angle =fromNUC.pitch;
// 	 /*获取2帧输出*/
// 	str->Out = str->Angle;
// 	res = str->Out;
// 	return res;
// }