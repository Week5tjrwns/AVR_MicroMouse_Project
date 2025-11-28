#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>

#include "StepMotor.h"
#include "Sensor.h"
#include "Algorithm.h"

static inline void disable_jtag(void) {
	MCUCSR |= (1 << JTD);	
	MCUCSR |= (1 << JTD);	
} // JTAG 기능 끄기

int main(void)
{
	disable_jtag();
	StepMotor_Init();
	InitializeSensor();

	_delay_ms(1000);  // 전원 켜지고 안정화될 때까지 대기

	// DFS 알고리즘으로 주행 시작
	DFS();

	while (1) {
		// DFS 함수 안에서 계속 돌기 때문에 여기는 실행 안 됨
	}
}

// 센서값 터미널 측정용 테스트 코드
/*
#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>      // itoa 사용
#include "Sensor.h"      // 센서 함수

// UART1 초기화 함수
static void UART1_Init(void)
{
	// 16MHz 9600bps 설정시 UBRR값 103
	UBRR1H = 0;
	UBRR1L = 103;

	UCSR1B = (1 << TXEN1);                        // 송신 기능 활성화
	UCSR1C = (1 << UCSZ11) | (1 << UCSZ10);       // 8비트 데이터 전송 설정
}

static void UART1_PutChar(char c)
{
	while (!(UCSR1A & (1 << UDRE1)));   // 전송 버퍼 비어질 때까지 대기
	UDR1 = c;
}

static void UART1_PutString(const char *s)
{
	while (*s) UART1_PutChar(*s++);
}

static void UART1_PutU16(uint16_t v)
{
	char buf[6];   // 문자열 변환용 버퍼
	itoa(v, buf, 10);
	UART1_PutString(buf);
}

int main(void)
{
	// 센서 초기화
	InitializeSensor();
	UART1_Init();

	// 상태 확인용 LED 포트 설정
	DDRF |= (1<<PF4)|(1<<PF5)|(1<<PF6);

	while (1)
	{
		uint16_t f = readSensor(FRONT_SENSOR);
		uint16_t l = readSensor(LEFT_SENSOR);
		uint16_t r = readSensor(RIGHT_SENSOR);

		// 터미널에 센서값 출력
		UART1_PutString("F:");
		UART1_PutU16(f);
		UART1_PutString(" L:");
		UART1_PutU16(l);
		UART1_PutString(" R:");
		UART1_PutU16(r);
		UART1_PutString("\r\n");

		// LED로 센서 상태 간략 확인
		PORTF |= (1<<PF4)|(1<<PF5)|(1<<PF6);  // 전부 끄기
		if (f >= 1)  PORTF &= ~(1<<PF5);      // 앞 센서 감지되면 켜기
		if (l >= 1)  PORTF &= ~(1<<PF4);
		if (r >= 1)  PORTF &= ~(1<<PF6);

		_delay_ms(200);   // 0.2초마다 출력
	}
}*/
