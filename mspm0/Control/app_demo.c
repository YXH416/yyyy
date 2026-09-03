/**
  ******************************************************************************
  * @file    app_demo.c
  * @brief   �ĸ������ѧʵ��ʹ����������
  ******************************************************************************
  * ʵ��1��֤STEP/DIR����������ʵ��2��֤A/B��PWM��������ȡ��
  * ʵ��3�Զ�ִ�бջ�������ʵ��4ͨ��������������Ŀ��Ƕȡ�
  ******************************************************************************
  */
#include <stdio.h>
#include <string.h>
#include "board.h"
#include "app_demo.h"
#include "closed_loop.h"
#include "demo_config.h"
#include "encoder.h"
#include "motor.h"

/* ��5ms�ж�ά��������ʱ�����Լ���ʵ��ķ�����״̬������ */
static volatile uint32_t s_ms;
static char s_line[40];
static uint8_t s_line_len;
static uint32_t s_last_action;
static uint32_t s_last_output;
static uint8_t s_demo1_state;
static uint8_t s_demo3_state;
static uint8_t s_demo2_state;

/* ���ջ�������ת���������û�������˵���� */
static const char *Demo_FaultName(CL_Fault_t fault)
{
    if (fault == CL_FAULT_NONE) return "����";
    if (fault == CL_FAULT_NO_ENCODER) return "�ޱ���������!������������/����";
    if (fault == CL_FAULT_DIRECTION) return "������!���AXIS_X_POSITIVE_DIR_LEVEL��D1";
    return "��������";
}

/* ��ӡʵ��4֧�ֵ�ȫ�����ᴮ����� */
/*==========================================================================*/
/*                          ʵ��4������ָ�����                              */
/*==========================================================================*/
/**
  * �����ϵ����1���ֵ�ǰλ�ã�ͨ������ָ�������1ת��ָ���Ƕȣ�
  *       Ҳ���Բ�ѯ״̬����ǰλ�����㡢����ֹͣ������Ϻͷ�ת������
  *
  * ����������
  *   1. ��������Ϊ115200-8-N-1���ַ�����ĩβ������س����У�
  *   2. ����A1 90����1ת��+90�ȣ�����A1 -45.5ת��-45.5�ȣ�
  *   3. ����S��ѯĿ�ꡢʵ�ʡ���PWM���ԽǶȺ��з�����Ȧ����
  *   4. ����Z�ѵ�ǰλ����Ϊ������㣬����X����ֹͣ��
  *   5. �����ų�����C1����ϣ������෴�ҵ����ֹͣʱ����D1��
  *
  * �۲�㣺
  *   1. Ŀ��Ƕ�������������Ķ�Ȧ�Ƕȣ�����PWM��0~360�Ⱦ��ԽǶȣ�
  *   2. �·�Ŀ���ջ���ֶ�η�STEP���壬ʵ�ʽǶ��𽥱ƽ�Ŀ�ꣻ
  *   3. �й���ʱ�ܾ���Ŀ�꣬�����ȼ���������������������ߡ�
  */
static void Demo4_PrintHelp(void)
{
    printf("\r\n================ ʵ��4 ����ָ��˵�� ================\r\n");
    printf("�ô������ַ�������ָ��(��β�ӻس�����):\r\n");
    printf("  A1 <�Ƕ�>  ��1ת��ָ���Ƕ�, ��: A1 90 �� A1 -45.5\r\n");
    printf("  S          ��ѯ��1״̬(��PWM���ԽǶȺ��з���ZȦ��)\r\n");
    printf("  Z          ��1��ǰλ����Ϊ���\r\n");
    printf("  X          ����ֹͣ��1\r\n");
    printf("  C1         �����1����\r\n");
    printf("  D1         ��ת��1�������ƽ(������ʱ��)\r\n");
    printf("  H          ���´�ӡ��˵��\r\n");
    printf("====================================================\r\n\r\n");
}

/* �ɼ�һ�αջ���PWM���ԽǶȺ���Ȧ��״̬����ӡ�� */
static void Demo4_PrintAxis(void)
{
    CL_Snapshot_t snap;
    float pwm_angle = 0.0f;
    uint8_t pwm_ok;
    CL_GetSnapshot(MOTOR_AXIS_X, &snap);
    pwm_ok = Encoder_GetPwmAngle(ENCODER_AXIS_X, &pwm_angle);
    printf("��1: Ŀ��=%.2f�� ʵ��=%.2f�� ���=%.2f�� [%s] ����:%s ",
        snap.target_angle_deg, snap.current_angle_deg,
        snap.target_angle_deg - snap.current_angle_deg,
        (snap.reached != 0U) ? "�ѵ�λ" :
            ((snap.active != 0U) ? "�˶���" : "����"),
        Demo_FaultName(snap.fault));
    if (pwm_ok != 0U) printf("PWM���ԽǶ�=%.2f�� ", pwm_angle);
    else printf("PWM���ź� ");
    printf("ZȦ��=%ld\r\n", (long)Encoder_GetZCount(ENCODER_AXIS_X));
}

/**
  * ����һ��ASCII����Ȱ�Сдת��Ϊ��д��������ʶ��Ƕȡ���ѯ��
  * ���㡢ֹͣ������ϡ���ת����Ͱ������
  */
static void Demo4_HandleCommand(char *line)
{
    char *p;
    float angle;
    for (p = line; *p != '\0'; p++) {
        if (*p >= 'a' && *p <= 'z') *p = (char)(*p - 'a' + 'A');
    }
    if (sscanf(line, "A1 %f", &angle) == 1) {
        if (CL_SetTargetAngle(MOTOR_AXIS_X, angle) == MOTOR_OK)
            printf("OK: ��1 Ŀ��Ƕ� %.2f��\r\n", angle);
        else
            printf("ʧ��! ��������й���, �ȷ� C1 �������\r\n");
    } else if (strcmp(line, "S") == 0) {
        Demo4_PrintAxis();
    } else if (strcmp(line, "Z") == 0) {
        CL_SetZeroAll();
        printf("OK: ��1���ڵ�ǰλ������\r\n");
    } else if (strcmp(line, "X") == 0) {
        CL_StopAll();
        printf("OK: ��1��ֹͣ\r\n");
    } else if (strcmp(line, "C1") == 0) {
        CL_ClearFault(MOTOR_AXIS_X);
        printf("OK: ��1���������\r\n");
    } else if (strcmp(line, "D1") == 0) {
        if (CL_TogglePositiveDirLevel(MOTOR_AXIS_X) == MOTOR_OK)
            printf("OK: ��1�������ƽ�ѷ�ת\r\n");
        else
            printf("ʧ��! ����˶��в��ܷ�ת����\r\n");
    } else if (strcmp(line, "H") == 0 || strcmp(line, "?") == 0) {
        Demo4_PrintHelp();
    } else if (line[0] != '\0') {
        printf("δָ֪��: %s (�� H �鿴ָ��˵��)\r\n", line);
    }
}

/* ʵ��2�ĵ��ַ�������0ֹͣ��1˳ʱ������ת��2��ʱ������ת�� */
/*==========================================================================*/
/*                          ʵ��2����������ȡ                                */
/*==========================================================================*/
/**
  * ������¼��λ����Ĭ��ֹͣ�����ڷ��͵��ַ����ɿ�����1��
  *       1=˳ʱ��������ת��2=��ʱ��������ת��0=ֹͣ��
  *       ���۵��ת������ֹͣ������ÿDEMO2_PRINT_MS��ӡһ�α��������ݡ�
  *
  * Ϊʲô����������ת��
  *   D36Aʹ�ܺ�ֹͣSTEP����ʱ�����Ȼͨ�����ᣬ���ʺ�ǿ������ת����
  *   ����ɳ����������������۲�A/B�������ٶȺ�PWM���ԽǶȡ�
  *
  * �۲�㣺
  *   1. ����1��2ʱ��A/B�������ٶȵ���������Ӧ���෴��
  *   2. A/B�����ź�Ӳ��4��Ƶ�󣬵��תһ��ȦӦ�仯Լ4000������
  *   3. A/B�Ƕ�������ϵ����Ķ�Ȧ�Ƕȣ�PWM�Ƕ���0~360�Ⱦ��ԽǶȣ�
  *   4. ������Z��PA25��ÿ��Z�����ذ�QEI�����ۼ��з���Ȧ����
  *   5. �������������Ԥ���෴���޸�ENCODER_AXIS_X_SIGN��
  *
  * ������ת��ʵ�֣�
  *   TIMA1ֱ���������Ӳ��PWM��ֹͣ�����ʱ�رն�ʱ����ǿ��STEPΪ�͡�
  */
static void Demo2_SetMotion(uint8_t motion)
{
    if (motion == 0U) {
        Motor_StopAll();
        s_demo2_state = 0U;
        printf("[ʵ��2] ��ֹͣ��EN��Ϊ�ߵ�ƽ������������ʹ��/���ᣩ\r\n");
    } else if (motion == s_demo2_state) {
        printf("[ʵ��2] �Ѿ���%s������ת\r\n", motion == 1U ? "˳ʱ��" : "��ʱ��");
    } else {
        Motor_StartContinuous(DEMO2_FREQ_HZ, motion == 1U ? 0U : 1U);
        s_demo2_state = motion;
        printf("[ʵ��2] ��ʼ%s������ת��Ƶ��=%u Hz������0ֹͣ\r\n",
            motion == 1U ? "˳ʱ��" : "��ʱ��", (unsigned)DEMO2_FREQ_HZ);
    }
}

/*
 * ��ѯUART0����FIFO��
 * ʵ��2�յ����ַ�����ִ�У�ʵ��4�Իس�����Ϊһ������Ľ�����־��
 */
static void Demo_PollUart(void)
{
    while (!DL_UART_Main_isRXFIFOEmpty(UART_0_INST)) {
        char ch = (char)DL_UART_Main_receiveData(UART_0_INST);
#if (DEMO_SELECT == 2)
        if (ch == '0') Demo2_SetMotion(0U);
        else if (ch == '1') Demo2_SetMotion(1U);
        else if (ch == '2') Demo2_SetMotion(2U);
        else if (ch != '\r' && ch != '\n' && ch != ' ' && ch != '\t')
            printf("[ʵ��2] δָ֪�� '%c'���뷢�� 1��2 �� 0\r\n", ch);
#elif (DEMO_SELECT == 4)
        if (ch == '\r' || ch == '\n') {
            if (s_line_len != 0U) {
                s_line[s_line_len] = '\0';
                Demo4_HandleCommand(s_line);
                s_line_len = 0U;
            }
        } else if (s_line_len < sizeof(s_line) - 1U) {
            s_line[s_line_len++] = ch;
        } else {
            s_line_len = 0U;
        }
#else
        (void)ch;
#endif
    }
}

/* ��ʼ��ʵ��״̬��ģʽ3/4�������ñջ�������ӡ��ǰ���ú�ʹ��˵���� */
void Demo_Init(void)
{
    s_ms = 0U;
    s_last_action = 0U;
    s_last_output = 0U;
    s_demo1_state = 0U;
    s_demo2_state = 0U;
    s_demo3_state = 0U;
#if (DEMO_SELECT == 3) || (DEMO_SELECT == 4)
    CL_Init();
#endif
    printf("\r\n\r\n");
    printf("****************************************************\r\n");
    printf("*  MS42CG + D36A ��������ջ����� С������          *\r\n");
    printf("*  ����: MSPM0G3507  ����: 115200-8-N-1             *\r\n");
    printf("****************************************************\r\n");
    printf("��ǰϸ������ D36A_MICROSTEP = %u (��غͲ���һ��!)\r\n",
        (unsigned)D36A_MICROSTEP);
    printf("������� MOTOR_COUNT = 1 (ֻʹ����1)\r\n");
    printf("��ǰ����: ʵ��%d ", DEMO_SELECT);
#if (DEMO_SELECT == 1)
    printf("- ����ת��ʵ��\r\n");
    printf("�������ʼ����ת����ע��: װ�ڻ�����ʱ���СDEMO1_STEPS!\r\n\r\n");
#elif (DEMO_SELECT == 2)
    printf("- ��������ȡʵ��\r\n");
    printf("���Ĭ��ֹͣ������1=˳ʱ��������ת, 2=��ʱ��������ת, 0=ֹͣ��\r\n");
    printf("ʵ��2ʹ�õ��ַ�ָ��, ����Ҫ�س����У����������ݻ������ӡ��\r\n\r\n");
#elif (DEMO_SELECT == 3)
    printf("- �ջ�����ʵ��\r\n");
    printf("��1���� 0 <-> ��%.1f�� ���Զ�������\r\n", DEMO3_TARGET_DEG);
#if (DEMO3_OUTPUT_MODE == 1)
    printf("������ {B����1:����2:...}$ ���θ�ʽ���, ����������λ���鿴���ߡ�\r\n\r\n");
#else
    printf("\r\n");
#endif
#else
    printf("- ����ָ��ʵ��\r\n");
    Demo4_PrintHelp();
#endif
}

/* ʵʱ5ms��������չQEI������PWM��ʱ����ִ��һ�αջ����ߡ� */
void Demo_Tick5ms(void)
{
    s_ms += CL_PERIOD_MS;
    Encoder_Tick(CL_PERIOD_MS);
#if (DEMO_SELECT == 3) || (DEMO_SELECT == 4)
    CL_Process();
#endif
}

/**
  * ��ѭ��ʵ����ȣ�
  * ģʽ1��������������ģʽ2���ڴ�ӡ��������ģʽ3ÿ4���л��ջ�Ŀ�ꣻ
  * ģʽ4ֻ��Ӧ���ڽǶ�����ջ���������5ms��������ƽ���
  */
void Demo_Process(void)
{
    Demo_PollUart();
/*==========================================================================*/
/*                          ʵ��1������ת��                                  */
/*==========================================================================*/
/**
  * ������1��������������ת��DEMO1_STEPS�����壬ѭ��������
  *       ���æʱ��ѭ�������ظ���������ǰ����ν�����Ž�����һ�ζ�����
  *
  * �۲�㣺
  *   1. �޸�DEMO1_FREQ_HZ���Ըı�STEPƵ�ʣ�Ҳ���ǵ��ת�٣�
  *   2. �޸�D36A����ϸ��ʱ�����ȶϵ磬��ͬ���޸�D36A_MICROSTEP��
  *      �������ͺ겻һ�£���ͬ��������Ӧ��ʵ��ת�Ǿͻ����
  *   3. ��������ֻ�����Ѿ����������壬����������λ�ã�
  *   4. �����ת���ع�����ɶ������ɿ���λ�ò����Զ���������
  *      ����ǿ������Ƶ�ȱ�㣬ʵ��3�ıջ������λ������Զ�������
  *
  * ��ȫ��ʾ��װ�ڻ������״β���ʱӦ��СDEMO1_STEPS����ֹײ��е��λ��
  */
#if (DEMO_SELECT == 1)
    if (Motor_IsBusy(MOTOR_AXIS_X) == 0U && (s_ms - s_last_action) >= 1000U) {
        uint8_t direction = (s_demo1_state == 0U) ? 0U : 1U;
        printf(s_demo1_state == 0U ? "[ʵ��1] ��ת %u ��, Ƶ�� %u Hz\r\n" :
                                     "[ʵ��1] ��ת %u ��, Ƶ�� %u Hz\r\n",
            (unsigned)DEMO1_STEPS, (unsigned)DEMO1_FREQ_HZ);
        (void)Motor_SetDirection(MOTOR_AXIS_X, direction);
        (void)Motor_Start(MOTOR_AXIS_X, DEMO1_STEPS, DEMO1_FREQ_HZ);
        s_demo1_state = (uint8_t)!s_demo1_state;
        s_last_action = s_ms;
    }
#elif (DEMO_SELECT == 2)
    if ((s_ms - s_last_output) >= DEMO2_PRINT_MS) {
        float pwm_angle = 0.0f;
        uint8_t pwm_ok = Encoder_GetPwmAngle(ENCODER_AXIS_X, &pwm_angle);
        printf("״̬=%s | ��1: AB����=%ld �Ƕ�=%.2f�� �ٶ�=%.1f��/�� ",
            s_demo2_state == 1U ? "˳ʱ��" :
                (s_demo2_state == 2U ? "��ʱ��" : "ֹͣ"),
            (long)Encoder_GetCount(ENCODER_AXIS_X),
            Encoder_GetAngle(ENCODER_AXIS_X),
            Encoder_CalcSpeedDps(ENCODER_AXIS_X, DEMO2_PRINT_MS));
        if (pwm_ok != 0U) printf("PWM���ԽǶ�=%.2f�� ", pwm_angle);
        else printf("PWM���ź� ");
        printf("ZȦ��=%ld\r\n\r\n", (long)Encoder_GetZCount(ENCODER_AXIS_X));
        s_last_output = s_ms;
    }
/*==========================================================================*/
/*                          ʵ��3���ջ�����                                  */
/*==========================================================================*/
/**
  * ������1ÿ��DEMO3_SWITCH_MS�л�һ��Ŀ�꣬����
  *       +DEMO3_TARGET_DEG -> 0 -> -DEMO3_TARGET_DEG -> 0��˳��ѭ��������
  *
  * �۲�㣺
  *   1. �ı�ģʽ�¿��Կ�������𽥱�С��������CL_TOLERANCE_COUNTS�ݲ
  *   2. DEMO3_OUTPUT_MODE=1ʱ���{BĿ��:ʵ��:���}$��������λ�������ߣ�
  *   3. �ڰ�ȫ��Χ����΢�赲���Ŷ�������ɿ���ջ������������Ŀ�ꣻ
  *   4. ������ת�����������������ᱨ�޷������ϣ�
  *      ������������Գ��෴����仯���ᱨ������ϣ�
  *   5. ��һ�δ��������Խ����DEMO3_TARGET_DEG��Ϊ5�ȣ���ȷ��û����λ���档
  *
  * ���ƹ��̣�ÿ5ms��ȡA/Bλ�ã���ǰ������ν��������¼�����
  *           ��ѡ���򡢲�����Ƶ�ʷ�����һ�Σ�ֱ���ȶ���λ��
  */
#elif (DEMO_SELECT == 3)
    if ((s_ms - s_last_action) >= DEMO3_SWITCH_MS) {
        float target;
        if (CL_GetFault(MOTOR_AXIS_X) != CL_FAULT_NONE)
            printf("[ʵ��3] ��1����: %s\r\n", Demo_FaultName(CL_GetFault(MOTOR_AXIS_X)));
        if (s_demo3_state == 0U) target = DEMO3_TARGET_DEG;
        else if (s_demo3_state == 1U) target = 0.0f;
        else if (s_demo3_state == 2U) target = -DEMO3_TARGET_DEG;
        else target = 0.0f;
        (void)CL_SetTargetAngle(MOTOR_AXIS_X, target);
        s_demo3_state = (uint8_t)((s_demo3_state + 1U) % 4U);
        s_last_action = s_ms;
    }
    if ((s_ms - s_last_output) >= DEMO3_OUTPUT_MS) {
        CL_Snapshot_t snap;
        CL_GetSnapshot(MOTOR_AXIS_X, &snap);
#if (DEMO3_OUTPUT_MODE == 1)
        printf("{B%.2f:%.2f:%.2f}$", snap.target_angle_deg,
            snap.current_angle_deg, snap.target_angle_deg - snap.current_angle_deg);
#else
        static uint8_t s_div;
        if (++s_div >= 10U) {
            s_div = 0U;
            printf("��1: Ŀ��=%7.2f ʵ��=%7.2f ���=%6.2f %s\r\n",
                snap.target_angle_deg, snap.current_angle_deg,
                snap.target_angle_deg - snap.current_angle_deg,
                snap.reached != 0U ? "�ѵ�λ" : "�˶���");
        }
#endif
        s_last_output = s_ms;
    }
#endif
}


