#ifndef VARIABLE_H
#define VARIABLE_H
#include <Arduino.h>

int32_t motorPulseBL = 0;
int32_t motorPulseFL = 0;
int32_t motorPulseFR = 0;
int32_t motorPulseBR = 0;

int32_t rotInFL = 0;
int32_t rotInFR = 0;
int32_t rotInBL = 0;
int32_t rotInBR = 0;

float WHEEL_PPR_1 = 384;
float WHEEL_PPR_2 = 384;
float WHEEL_PPR_3 = 384;
float WHEEL_PPR_4 = 384;

float cur_locomotion_L = 0;
float cur_locomotion_R = 0;
float cur_locomotion_B = 0;
float temp_cur_locomotion_L = 0;
float temp_cur_locomotion_R = 0;
float temp_cur_locomotion_B = 0;

double last_timer_speed,t_speed;

double locomotion_FL_vel = 0;
double locomotion_FR_vel = 0;
double locomotion_BL_vel = 0;
double locomotion_BR_vel = 0;

double locomotion_FL_target_rate = 0;
double locomotion_FR_target_rate = 0;
double locomotion_BL_target_rate = 0;
double locomotion_BR_target_rate = 0;

#endif