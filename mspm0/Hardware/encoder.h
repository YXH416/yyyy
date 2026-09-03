/**
  ******************************************************************************
  * @file    encoder.h
  * @brief   MS42CG���������A/B����������ZȦ����PWM���ԽǶȽӿ�
  ******************************************************************************
  * A=PA1/TIMG8_CCP0��B=PA0/TIMG8_CCP1����TIMG8Ӳ��QEI�ı�Ƶ������
  * PWM=PB20/TIMG12_CCP0������ϲ���ͬʱ�������ں͸ߵ�ƽʱ�䡣
  * Z=PA25/GPIO�������жϣ�ÿ�ξ�����е��׼��ʱ��QEI�����ۼ��з���Ȧ����
  ******************************************************************************
  */
#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

typedef enum { ENCODER_AXIS_X = 0 } EncoderAxis_t;

/* ��ʼ��QEI��PWM����������ۼ�״̬�� */
void Encoder_Init(void);
/* �̶����ڵ��ã���չ16λQEI��������ά��PWM�źų�ʱ�� */
void Encoder_Tick(uint32_t elapsed_ms);
/* ��ȡ����������Ķ�Ȧ�з��ż����� */
int32_t Encoder_GetCount(EncoderAxis_t axis);
float Encoder_GetAngle(EncoderAxis_t axis);
/* �������ε��ü�ļ����������ٶȣ���λΪ��/�롣 */
float Encoder_CalcSpeedDps(EncoderAxis_t axis, uint32_t period_ms);
void Encoder_SetZero(EncoderAxis_t axis);
void Encoder_SetZeroAll(void);
/* ��ȡ����Z�����ۼƵ��з��ž�Ȧ����������+1��������-1�� */
int32_t Encoder_GetZCount(EncoderAxis_t axis);
/* ��ȡ���һ��Z������ʱ��A/B��Լ��������ڼ��ÿȦ�Ƿ�ӽ�4000������ */
int32_t Encoder_GetCountAtLastZ(EncoderAxis_t axis);
/* ��ȡ0~360��PWM���ԽǶȣ�����0��ʾδ���ߡ���ʱ�򲶻�������Ч�� */
uint8_t Encoder_GetPwmAngle(EncoderAxis_t axis, float *angle_deg);

#endif


