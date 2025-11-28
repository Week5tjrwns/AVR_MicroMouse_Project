#define F_CPU 16000000UL // CPU 클럭 16MHz
#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include "Sensor.h"

// 내부 ADC 제어 함수
static void adc_init(void)   // ADC 기능 초기화
{
	ADMUX  = ADC_VREF_TYPE;                           // 기준 전압을 AVCC로 설정하고 데이터 우측 정렬
	ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0); // ADC 활성화 및 분주비 128 설정
}

static unsigned int read_adc(uint8_t ch)   // 특정 채널 ADC 값 읽기
{
	// ADC 채널 선택 하위 5비트 변경
	ADMUX = (ADMUX & 0xE0) | (ch & 0x1F);

	// 첫 번째 변환은 안정화를 위해 버림
	ADCSRA |= (1<<ADSC);
	while(ADCSRA & (1<<ADSC));

	// 데이터 안정화를 위해 4회 측정 후 평균값 반환
	uint32_t acc = 0;
	for(uint8_t i = 0; i < 4; i++){
		ADCSRA |= (1<<ADSC);
		while(ADCSRA & (1<<ADSC));
		acc += ADC;
	}
	return (unsigned int)(acc / 4);
}

// 외부에서 호출하는 함수

// 센서 초기화 함수
void InitializeSensor(void)
{
	// 적외선 LED 포트 출력 설정 및 초기화
	DDRB  |= (1<<PB5)|(1<<PB6)|(1<<PB7);
	PORTB &= ~((1<<PB5)|(1<<PB6)|(1<<PB7));

	// 수광부 포트 입력 설정 풀업 저항 미사용
	DDRF  &= ~((1<<PF0)|(1<<PF1)|(1<<PF2));
	PORTF &= ~((1<<PF0)|(1<<PF1)|(1<<PF2));

	DDRF  |= (1<<PF4)|(1<<PF5)|(1<<PF6);   // 출력 포트 설정
	PORTF |= (1<<PF4)|(1<<PF5)|(1<<PF6);   // Active Low 방식이므로 1을 출력하여 LED 끄기


	adc_init();
}

// 특정 센서의 값을 읽어서 반환
unsigned int readSensor(uint8_t si)
{
	unsigned int ret = 0;

	switch(si){
		case FRONT_SENSOR:      // 전방 센서
		PORTB |= (1<<PB5);  // 전방 LED 켜기
		_delay_us(50);      // 빛 반사 안정화를 위해 잠시 대기
		ret = read_adc(0);  // ADC0 채널 읽기
		PORTB &= ~(1<<PB5); // LED 끄기
		break;

		case LEFT_SENSOR:       // 좌측 센서
		PORTB |= (1<<PB6);  // 좌측 LED 켜기
		_delay_us(50);
		ret = read_adc(1);  // ADC1 채널 읽기
		PORTB &= ~(1<<PB6); // 끄기
		break;

		case RIGHT_SENSOR:      // 우측 센서
		PORTB |= (1<<PB7);  // 우측 LED 켜기
		_delay_us(50);
		ret = read_adc(2);  // ADC2 채널 읽기
		PORTB &= ~(1<<PB7); // 끄기
		break;

		default:
		ret = 0;
		break;
	}

	return ret;  // 0에서 1023 사이의 값 반환
}
