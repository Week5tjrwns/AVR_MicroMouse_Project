#ifndef _SENSOR_H_
#define _SENSOR_H_

#include <stdint.h>

// 센서 식별 번호 정의
#define FRONT_SENSOR 0
#define LEFT_SENSOR  1
#define RIGHT_SENSOR 2

// ADC 기준 전압 설정
#define ADC_VREF_TYPE 0x40

// 센서 초기화 함수
void InitializeSensor(void);

// 센서값 읽기 함수
unsigned int readSensor(uint8_t si);

// 세 방향 센서값을 한 번에 읽는 함수
static inline void readSensorAll(uint16_t *front, uint16_t *left, uint16_t *right)
{
	if (front) *front = readSensor(FRONT_SENSOR);
	if (left)  *left  = readSensor(LEFT_SENSOR);
	if (right) *right = readSensor(RIGHT_SENSOR);
}

#endif
