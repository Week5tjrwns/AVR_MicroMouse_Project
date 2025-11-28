[Project] ATmega128 Micromouse (Custom Perfboard)

1. 프로젝트 개요

목표: ATmega128을 이용한 자율 주행 마이크로마우스 제작 및 미로 탐색 알고리즘 구현

기간: 마이크로프로세서 응용 설계 및 실험. 2025/09/07 ~ ing

특징: 상용 모터 드라이버 쉴드 없이 만능기판(Perfboard)에 직접 회로를 배선하여 하드웨어 구성

개발 환경: Microchip Studio (AVR Studio), C언어



2. 하드웨어 구성 (Hardware)

MCU: ATmega128A (외부 클럭 16MHz)

Motor: Stepper Motor (SLA7026 Driver, 1-2상 여자 방식 구동)

Sensor: IR Sensor (EL-7L) x 3 (좌/우/전방, ADC 인터페이스)



3. 소프트웨어 핵심 로직 (Implementation)

A. 미로 탐색 알고리즘 (Algorithm.c)

DFS (깊이 우선 탐색):

stackX, stackY 배열을 이용한 스택(Stack) 자료구조 구현.

visited[ ][ ] 배열로 방문 여부를 체크하고, 막다른 길에서는 스택을 Pop하여 되돌아오는 백트래킹(Backtracking) 로직 적용.

탐색 우선순위: Left -> Front -> Right

B. 주행 제어 및 보정 (Algorithm.c, StepMotor.c)

PID 제어 (P-Control):

좌/우 센서 값의 오차(err = L - R)를 계산하여 주행 경로를 중앙으로 보정.

현재는 Kp=1, Ki=0, Kd=0으로 설정하여 P 제어 위주로 튜닝.

안전 유턴 로직 (Do_Uturn_GATED):

사방이 막혔을 때 즉시 회전하지 않고 모터를 정지.

센서 값을 40회 연속 샘플링 후 평균을 내어 벽 유무를 재확인한 뒤, 확실할 때만 180도 회전 수행 (센서 오판 방지).

C. 센서 데이터 필터링 (Sensor.c, Algorithm.c)

ADC 평균화: read_adc() 함수 내부에서 센서값을 4회 측정하여 평균값을 사용,  노이즈 감소.

히스테리시스 (Hysteresis): 전방 벽 인식 시 FRONT_ON(켜짐)과 FRONT_OFF(꺼짐) 임계값에 차이를 두어 경계선에서의 데이터 떨림 방지.

커밋 스텝 (Commit Steps): 90도 회전 직후에는 자세가 불안정하므로, TURN_COMMIT_BLOCKS 구간 동안은 강제 진입하여 통로 진입 유도.



4. 트러블 슈팅 (Troubleshooting)

1) JTAG 포트 충돌 문제 (main.c)

이슈: 포트 F(PF4~7)를 센서 제어용으로 할당했으나 동작하지 않음.

원인: ATmega128 출고 시 JTAG 인터페이스가 기본 활성화(Enable) 되어 있어 GPIO 제어 불가.

해결: MCUCSR 레지스터의 JTD 비트를 High로 설정하여 JTAG을 비활성화하는 disable_jtag() 함수 구현.

2) 스텝모터 배선 오류 해결 (StepMotor.c)

이슈: 오른쪽 모터의 상(Phase) 배선 순서가 반대로 연결됨(초기 하드웨어 불량).

해결: 소프트웨어 드라이버에서 오른쪽 모터 인덱스(PR) 제어 방향을 역으로 매핑하여 해결.

EX) if (dR > 0) PR = (PR - 1) & 7; // 전진 명령 시 인덱스 감소



5. 파일 구조

main.c: 시스템 초기화(JTAG 해제 등) 및 DFS 메인 루프 실행

Algorithm.c/h: 미로 탐색 로직, PID 제어기, 유턴/회전 판단 로직

Sensor.c/h: ADC 초기화 및 IR 센서값 필터링 처리

StepMotor.c/h: 스텝모터 구동 시퀀스 및 1-2상 여자 제어
