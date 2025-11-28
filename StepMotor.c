#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include "StepMotor.h"

// 90도 회전할 때 필요한 스텝수 실험을 통해 캘리브레이션한 값
#define ROT_STEPS_90 200U   // 약 200스텝이면 90도 회전

// 1-2상 여자 방식 구동 테이블
// PD4부터 7은 왼쪽 모터 PE4부터 7은 오른쪽 모터 연결
static volatile uint8_t PL = 0;
static volatile uint8_t PR = 0;

static const uint8_t T[8] = {
	0x50, 0x10, 0x90, 0x80,
	0xC0, 0x20, 0x60, 0x40
};

void StepMotor_Init(void)
{
	DDRD |= 0xF0;   // PD4에서 7번 핀 출력 설정 왼쪽 바퀴
	DDRE |= 0xF0;   // PE4에서 7번 핀 출력 설정 오른쪽 바퀴

	PL = 0;
	PR = 0;

	PORTD &= ~0xF0;
	PORTE &= ~0xF0;
}

// dL이 양수면 왼쪽 전진 음수면 왼쪽 후진
// dR이 양수면 오른쪽 전진 음수면 오른쪽 후진
void StepMotor_Step(int8_t dL, int8_t dR)
{
	// 왼쪽 모터는 배선이 정방향이라 그대로 계산
	if (dL > 0)      PL = (PL + 1) & 7;
	else if (dL < 0) PL = (PL - 1) & 7;

	// 오른쪽 모터는 하드웨어 배선이 반대로 되어 있어서 인덱스 방향을 뒤집음
	if (dR > 0)      PR = (PR - 1) & 7;  // 전진 명령이지만 인덱스는 감소
	else if (dR < 0) PR = (PR + 1) & 7;  // 후진 명령이지만 인덱스는 증가

	// 마스킹 후 출력 내보내기
	PORTD = (PORTD & 0x0F) | T[PL];
	PORTE = (PORTE & 0x0F) | T[PR];
}

// 직선 주행 함수 dir이 1이면 전진 마이너스 1이면 후진
void StepMotor_MoveLinear(uint32_t steps, int8_t dir, uint16_t delay_us)
{
	for (uint32_t i = 0; i < steps; i++) {
		StepMotor_Step(dir, dir);
		_delay_us(1);
		for (uint16_t t = 0; t < delay_us; t++)
		_delay_us(1);
	}
}

// 제자리 회전 함수 steps만큼 회전하고 dir이 1이면 시계방향
void StepMotor_Rotate(uint32_t steps, int8_t dir, uint16_t delay_us)
{
	int8_t dL = 0, dR = 0;

	if (dir > 0) {        // 시계 방향 회전
		dL = 1;
		dR = -1;
		} else if (dir < 0) { // 반시계 방향 회전
		dL = -1;
		dR = 1;
	}

	for (uint32_t i = 0; i < steps; i++) {
		StepMotor_Step(dL, dR);
		_delay_us(1);
		for (uint16_t t = 0; t < delay_us; t++)
		_delay_us(1);
	}
}

// 각도를 입력받아 회전하는 함수 deg는 도 단위
void StepMotor_RotateDeg(uint16_t deg, int8_t dir, uint16_t delay_us)
{
	// 입력받은 각도를 스텝 수로 변환
	uint32_t steps = ((uint32_t)deg * ROT_STEPS_90) / 90U;
	StepMotor_Rotate(steps, dir, delay_us);
}
