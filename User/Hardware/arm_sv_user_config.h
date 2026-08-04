#ifndef ARM_SV_USER_CONFIG_H
#define ARM_SV_USER_CONFIG_H

/*
 * ======================== 用户动作参数区 ========================
 * 后续只需要修改本文件中的数字，不要修改 arm_sv.c 里的轨迹计算公式。
 * 所有角度直接使用度数，例如 90 表示 +90°，-45 表示反方向 45°。
 * 所有时间使用毫秒，例如 5000 表示 5 秒，30000 表示 30 秒。
 * ================================================================
 */

/* 1=上电后自动执行一次动作；0=关闭自动动作，只接受外部/DAP命令。 */
#define ARM_SV_USER_ACTION_ENABLE 1

/* 上电后等待多久再开始动作，留出人员撤离和电源稳定时间。 */
#define ARM_SV_USER_START_DELAY_MS 3000

/*
 * 每段姿态运动的最短时间。数值越小动作越快。
 * 实际时间还受最大速度和最大加速度限制，必要时控制器会自动延长。
 */
#define ARM_SV_USER_MOVE_TIME_MS 5000

/* 前伸到位后保持多久，用来模拟完成抓取，再开始放下。 */
#define ARM_SV_USER_GRAB_HOLD_MS 1500

/* 最大速度和最大加速度，单位分别为 度/秒、度/秒²。 */
#define ARM_SV_USER_MAX_SPEED_DEG_S 35
#define ARM_SV_USER_MAX_ACCEL_DEG_S2 69

/*
 * 本次测试只控制实体1~4号舵机，数组顺序就是实体标签1、2、3、4。
 * 向前抓取姿态：2号回到中间0位，使主臂向前；3、4号分别使用
 * +20°和-20°做末端方向补偿。实体6号保持0°，本次不作为夹爪动作。
 */
#define ARM_SV_USER_SERVO1_FORWARD_GRAB_DEG 0
#define ARM_SV_USER_SERVO2_FORWARD_GRAB_DEG 0
#define ARM_SV_USER_SERVO3_FORWARD_GRAB_DEG 20
#define ARM_SV_USER_SERVO4_FORWARD_GRAB_DEG (-20)

/*
 * 放下姿态：2号从中间0位转到-90°，3、4号同时回到0°。
 * 动作结束后保持该姿态，不自动回零，也不重复运行。
 * 启动时也采用这个已知姿态，避免烧录复位后先跳到别的位置。
 */
#define ARM_SV_USER_SERVO1_PUT_DOWN_DEG 0
#define ARM_SV_USER_SERVO2_PUT_DOWN_DEG (-90)
#define ARM_SV_USER_SERVO3_PUT_DOWN_DEG 0
#define ARM_SV_USER_SERVO4_PUT_DOWN_DEG 0

#endif
