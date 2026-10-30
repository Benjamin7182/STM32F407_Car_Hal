#ifndef __CAR_H_
#define __CAR_H_

#include "headfile.h"

/* WHEELTEC Mini 阿克曼小车 D版 物理机械参数 (单位: m) */
#define AKM_WHEEL_SPACING      0.162f  /* 后轮轮距 W = 0.162m (Akm_wheelspacing) */
#define AKM_AXLE_SPACING       0.144f  /* 前后轮轴距 L = 0.144m (Akm_axlespacing) */
#define AKM_MIN_TURN_RADIUS    0.350f  /* 最小转弯半径 R_min = 0.350m */

/* 遥控速度档位定义 (单位: m/s) */
#define GEAR_1_MAX_SPEED       0.10f   /* 1档: 0.10 m/s (~294 脉冲/10ms) */
#define GEAR_2_MAX_SPEED       0.20f   /* 2档: 0.20 m/s (~588 脉冲/10ms) */
#define GEAR_3_MAX_SPEED       0.35f   /* 3档: 0.35 m/s (~1028 脉冲/10ms) */

typedef struct {
    float target_speed_l;
    float encl;
    float target_speed_r;
    float encr;
} VofaPacket_t;

extern QueueHandle_t xVofaQueue;

/* 正运动学解算结果 (用于上报 ROS 里程计 odom) */
extern float actual_speed_l;      /* 左后轮实际线速度 (m/s) */
extern float actual_speed_r;      /* 右后轮实际线速度 (m/s) */
extern float chassis_vx;          /* 车体中心前向线速度 Vx (m/s) */
extern float chassis_vz;          /* 车体中心偏航角速度 Vz (rad/s) */
extern int16_t chassis_vx_mms;    /* 车体前向线速度 (mm/s, 对应 ROS X_speed) */
extern int16_t chassis_vy_mms;    /* 车体横向线速度 (mm/s, 阿克曼恒为 0) */
extern int16_t chassis_vz_mrads;  /* 车体偏航角速度 (mrad/s, 对应 ROS Z_speed) */

/* 正逆运动学解算接口 */
void Kinematics_Forward(void);
float Vz_to_Akm_Angle(float vx_ms, float vz_rads);
void Kinematics_Inverse_Angle(float vx_ms, float angle_r_rad);
void Kinematics_Inverse_Vz(float vx_ms, float vz_rads);

/* WHEELTEC ROS 串口通信协议定义 (24字节上报 / 11字节接收) */
#define ROS_FRAME_HEADER       0x7B    /* 帧头 '{' */
#define ROS_FRAME_TAIL         0x7D    /* 帧尾 '}' */
#define ROS_SEND_SIZE          24      /* STM32 -> ROS 上报帧长度 */
#define ROS_RECV_SIZE          11      /* ROS -> STM32 控制帧长度 */

extern volatile uint8_t ros_ctrl_active;
extern volatile float ros_target_vx;
extern volatile float ros_target_vz;
extern uint8_t ros_rx1_byte;
extern uint8_t ros_rx3_byte;

void ROS_Parse_Byte(uint8_t byte, uint8_t port_id);
void ROS_Data_Task(void *pvParameters);

void Car_Remote_Task(void *pvParameters);
void PID_task(void *pvParameters);
void OledTask(void *pvParameters);
void VofaTask(void *pvParameters);
extern int current_pwm_l;

#endif

