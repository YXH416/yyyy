/**
  ******************************************************************************
  * @file    motor.h
  * @brief   D36A����STEP/DIR/EN���������ӿ�
  ******************************************************************************
  * TIMA1_CCP1��PA24���50%ռ�ձ�STEP��PA13����DIR��PA12����EN��
  * ���������ɶ�ʱ�������жϾ�ȷͳ��������������������رռƲ��жϡ�
  ******************************************************************************
  */
#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>
#include "ti_msp_dl_config.h"

/* STEP��ʱ��������1MHz����ʱ�ӣ�Ƶ��ͨ���Զ���װֵ���㡣 */
#define MOTOR_STEP_TIMER         PWM_0_INST
#define MOTOR_STEP_CC_INDEX      DL_TIMER_CC_1_INDEX
/* STEP��ʱ��������1MHz����ʱ�ӣ�Ƶ��ͨ���Զ���װֵ���㡣 */
#define MOTOR_STEP_TIMER_CLK_HZ  1000000U
/* D36Aϸ�ֱ����벦��һ�£�16ϸ��ʱһȦ��Ҫ200*16=3200��STEP���塣 */
#define D36A_MICROSTEP           16U
#define MOTOR_STEPS_PER_REV      (200U * D36A_MICROSTEP)
#define MOTOR_MIN_FREQ_HZ        16U
#define MOTOR_MAX_FREQ_HZ        5000U

/* ʵ�ʽ��ߣ�STEP=PA24��DIR=PA13��EN=PA12����ǰ����ʹ�øߵ�ƽʹ�ܡ� */
#define MOTOR_DIR_PORT           GPIOA
#define MOTOR_DIR_PIN            DL_GPIO_PIN_13
#define MOTOR_EN_PORT            GPIOA
#define MOTOR_EN_PIN             DL_GPIO_PIN_12

/* ���Ŷ��壻�����̽�ʹ����1�� */
typedef enum { MOTOR_AXIS_X = 0 } MotorAxis_t;
typedef enum { MOTOR_OK = 0, MOTOR_ERROR, MOTOR_BUSY } MotorStatus_t;

/* ��ʼ�����GPIO��ֹͣSTEP�����ʹ��TIMA1�жϡ� */
void Motor_Init(void);
MotorStatus_t Motor_SetDirection(MotorAxis_t axis, uint8_t high_level);
uint8_t Motor_GetDirection(MotorAxis_t axis);
/* �������У���ָ��Ƶ�����steps�����壻�������ٴ������᷵��MOTOR_BUSY�� */
MotorStatus_t Motor_Start(MotorAxis_t axis, uint32_t steps,
                          uint32_t frequency_hz);
/* �������У�����ʵ��2����ͳ��ʣ�����壬ֱ������ֹͣ������ */
void Motor_StartContinuous(uint32_t frequency_hz, uint8_t high_level);
void Motor_Stop(MotorAxis_t axis);
void Motor_StopAll(void);
uint8_t Motor_IsBusy(MotorAxis_t axis);
uint32_t Motor_GetRemainingSteps(MotorAxis_t axis);

#endif


