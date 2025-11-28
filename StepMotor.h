#ifndef STEPMOTOR_H_
#define STEPMOTOR_H_

#include <stdint.h>

// 모터 핀 설정 및 변수 초기화
void StepMotor_Init(void);

// 좌우 모터 각각 1스텝 구동 함수
// dL과 dR이 양수면 전진 음수면 후진
void StepMotor_Step(int8_t dL, int8_t dR);

// 직선 주행 함수
// delay_us는 스텝 사이의 지연 시간으로 속도 조절
void StepMotor_MoveLinear(uint32_t steps, int8_t dir, uint16_t delay_us);

// 제자리 회전 함수
// dir이 1이면 시계 방향 마이너스 1이면 반시계 방향
void StepMotor_Rotate(uint32_t steps, int8_t dir, uint16_t delay_us);

// 각도를 지정하여 회전하는 함수
void StepMotor_RotateDeg(uint16_t deg, int8_t dir, uint16_t delay_us);

#endif
