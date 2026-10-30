#ifndef __CAR_H_
#define __CAR_H_

#include "headfile.h"

// 速度档位定义 (单位: m/s，对齐原厂标准库 RC_Velocity: 250, 500, 1000 mm/s)
#define GEAR_1_MAX_SPEED   0.25f  // 1档: 0.25 m/s (250 mm/s，室内慢速)
#define GEAR_2_MAX_SPEED   0.50f  // 2档: 0.50 m/s (500 mm/s，原厂标准 RC_Velocity)
#define GEAR_3_MAX_SPEED   1.00f  // 3档: 1.00 m/s (1000 mm/s，高速巡航)

typedef struct {
    float target_speed_l;
    float encl;
		float target_speed_r;
    float encr;
} VofaPacket_t;

extern QueueHandle_t xVofaQueue;

void Car_Remote_Task(void *pvParameters);
void PID_task(void *pvParameters);
void OledTask(void *pvParameters);
void VofaTask(void *pvParameters);
extern int current_pwm_l;
#endif
