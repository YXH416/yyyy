/**
 * board.c - �弶�������� / Board-level utilities
 * SysTick��ʱ������printf�ض����
 * Delay functions, printf retarget to UART
 */
#include "ti_msp_dl_config.h"
#include "board.h"

volatile unsigned long tick_ms;
volatile uint32_t start_time;


void SysTick_Init(void)
{
    DL_SYSTICK_config(CPUCLK_FREQ/1000);
    NVIC_SetPriority(SysTick_IRQn, 0);
}

// ��ȡSysTick��ǰ����ֵ / Read current SysTick count
uint32_t Systick_getTick(void)
{
	return (SysTick->VAL);
}


// ms��������ʱ / Blocking delay in ms
void delay_ms(uint32_t ms)
{
	// ����������ʱ��Χ��ض� / Clamp to max possible delay
	//if( ms > SysTickMAX_COUNT/(SysTickFre/1000) ) ms = SysTickMAX_COUNT/(SysTickFre/1000);
	for(int i=0;i<1000;i++)
	{
		delay_us(ms);
	}
}


// us��������ʱ / Blocking delay in us
void delay_us(uint32_t us)
{
	// �ضϳ���Χֵ / Clamp if exceeds max
	if( us > SysTickMAX_COUNT/(SysTickFre/1000000) ) us = SysTickMAX_COUNT/(SysTickFre/1000000);

	us = us*(SysTickFre/1000000); // ��λת��Ϊ����ֵ / Convert to tick count

	// ��¼�Ѿ��߹���ʱ�� / Accumulated elapsed ticks
	uint32_t runningtime = 0;

	// ��¼��ʼʱ�̵ļ���ֵ / Capture starting tick
	uint32_t InserTick = Systick_getTick();

	// ��ѯ��ʵʱˢ�� / Live tick during polling
	uint32_t tick = 0;

	uint8_t countflag = 0;
	// �ȴ���ʱ���� / Wait for delay to expire
	while(1)
	{
		tick = Systick_getTick();// ˢ�µ�ǰ����ֵ / Refresh current tick

		// �������緭ת���л����㷽ʽ / Handle wrap-around
		if( tick > InserTick ) countflag = 1;

		if( countflag ) runningtime = InserTick + SysTickMAX_COUNT - tick;
		else runningtime = InserTick - tick;

		if( runningtime>=us ) break;
	}

}

void delay_1us(unsigned long __us){ delay_us(__us); }
void delay_1ms(unsigned long ms){ delay_ms(ms); }

#if !defined(__MICROLIB)
// δʹ��΢��ʱ��Ҫ�ֶ���ȫ�ײ㺯�� / Needed when not using microlib
#if (__ARMCLIB_VERSION <= 6000000)
// AC5��������Ҫ����FILE�ṹ�� / AC5 compiler needs FILE struct
struct __FILE
{
	int handle;
};
#endif

FILE __stdout;

// ���ð�����ģʽ / Disable semihosting
void _sys_exit(int x)
{
	x = x;
}
#endif

// printf�ض��򵽴���0 / Redirect printf to UART0
int fputc(int ch, FILE *stream)
{
	// æ��ֱ�����ڿ����ٷ��� / Wait until UART is ready
	while( DL_UART_isBusy(UART_0_INST) == true );

	DL_UART_Main_transmitData(UART_0_INST, ch);

	return ch;
}
/*
 * TI���п��printf����fputc�����ͨ�ַ���%s��
 * ��fputs����Ѿ�ת����ɵ��������������͸������ַ�����
 * �����ӿڱ����ض���ͬһ��UART�����������������������ֶ�ʧ��
 */
int fputs(const char *text, FILE *stream)
{
    while (*text != '\0') {
        if (fputc((unsigned char)*text++, stream) == EOF) return EOF;
    }
    return 0;
}


