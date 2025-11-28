#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>

#include "StepMotor.h"   // 모터 제어 함수 포함
#include "Sensor.h"      // 센서 읽기 함수 포함
#include "Algorithm.h"

static inline void StepMotor_PauseStepping(void)  {}
static inline void StepMotor_ResumeStepping(void) {}

/* 튜닝 상수 설정
   임계값은 가까울수록 값이 둔하게 작용*/
#ifndef THRESH_FRONT
#define THRESH_FRONT        200
#endif
#ifndef THRESH_FRONT_EMER
#define THRESH_FRONT_EMER   450
#endif
#ifndef THRESH_LEFT
#define THRESH_LEFT         130
#endif
#ifndef THRESH_RIGHT
#define THRESH_RIGHT        300
#endif

// 한 블록의 스텝 길이 재판정 주기를 위해 약간 축소 설정
#ifndef BLOCK_STEPS
#define BLOCK_STEPS         300UL
#endif

// 직진 주행 속도 값이 클수록 느려짐
#ifndef BASE_DELAY_US
#define BASE_DELAY_US       2500
#endif
#ifndef SLOW_DELAY_US
#define SLOW_DELAY_US       2200
#endif

// 회전 속도 설정
#ifndef ROT_SPEED_US
#define ROT_SPEED_US        2000
#endif

// 보정 스텝 단위 설정 코너에서 튕기는 현상 완화
#ifndef CORR_UNIT
#define CORR_UNIT           70
#endif

// 정면 센서 히스테리시스 설정 민감한 반응 억제
#ifndef FRONT_ON
#define FRONT_ON            (THRESH_FRONT + 20)
#endif
#ifndef FRONT_OFF
#define FRONT_OFF           (THRESH_FRONT - 10)
#endif
static uint8_t g_front_blk = 0;
static inline uint8_t front_blocked_hyst(uint16_t f){
  if (f >= FRONT_ON)       g_front_blk = 1;
  else if (f <= FRONT_OFF) g_front_blk = 0;
  return g_front_blk;
}

/* 90도 회전 시 코너 진입 안정화 설정 유턴에는 적용하지 않음
   1 회전 후 최소 N블록은 무조건 진입
   2 회전 직후 일정 스텝 동안 정면 센서 무시하여 오판 방지
   3 회전 직후 코너 안쪽으로 일정 스텝 밀어넣기 */
#ifndef TURN_COMMIT_BLOCKS
#define TURN_COMMIT_BLOCKS  2
#endif
#ifndef FRONT_MASK_STEPS_AFTER_TURN
#define FRONT_MASK_STEPS_AFTER_TURN  180
#endif
static uint8_t  g_turn_commit = 0;   // 회전 후 무조건 진입해야 하는 블록 수
static uint16_t g_front_mask  = 0;   // 회전 후 정면 센서를 무시하는 스텝 수

static inline void SeedForwardSteps(uint16_t n){
  for(uint16_t i=0;i<n;i++){
    StepMotor_Step(1,1);
    _delay_us(BASE_DELAY_US);
  }
}

/* PID 제어 구조체 */
typedef struct {
  int16_t kp, ki, kd;
  int32_t integral;
  int16_t prev_err;
} PID_t;

// 중앙 정렬 보정 현재 P 제어만 사용
static PID_t pid_center = { .kp=1, .ki=0, .kd=0, .integral=0, .prev_err=0 };

static inline int16_t PID_Update(PID_t *pid, int16_t err){
  pid->integral += err;
  int16_t deriv = (int16_t)(err - pid->prev_err);
  pid->prev_err = err;
  int32_t out = (int32_t)pid->kp*err + (int32_t)pid->ki*pid->integral + (int32_t)pid->kd*deriv;
  if(out >  300) out =  300;
  if(out < -300) out = -300;
  return (int16_t)out;
}

/* 유틸리티 함수 */
static inline void LED_show_walls(uint8_t wl, uint8_t wf, uint8_t wr){
  // LED 포트 출력 설정 Active Low 방식이므로 반전 필요
  PORTF |= (1<<PF4)|(1<<PF5)|(1<<PF6);
  if(wl) PORTF &= ~(1<<PF4);
  if(wf) PORTF &= ~(1<<PF5);
  if(wr) PORTF &= ~(1<<PF6);
}

/* 함수 원형 선언 */
void Forward_PID_OneBlock(void);

/* 유턴 게이트 함수 사방이 막혔을 때만 호출되어 판단 수행
   절차 모터 정지 후 센서값 평균 측정하고 모두 막혔을 때만 회전 */
static void Do_Uturn_GATED(void){
  // 모터 완전 정지
  StepMotor_PauseStepping();

  // 센서값 40회 연속 측정 후 평균 계산
  uint32_t sf=0, sl=0, sr=0;
  for(uint8_t i=0;i<40;i++){
    sf += readSensor(FRONT_SENSOR);
    sl += readSensor(LEFT_SENSOR);
    sr += readSensor(RIGHT_SENSOR);
  }
  uint16_t mf = (uint16_t)(sf/40);
  uint16_t ml = (uint16_t)(sl/40);
  uint16_t mr = (uint16_t)(sr/40);

  uint8_t wf = (mf >= THRESH_FRONT);
  uint8_t wl = (ml >= THRESH_LEFT);
  uint8_t wr = (mr >= THRESH_RIGHT);
  LED_show_walls(wl, wf, wr);

  // 모터 동작 재개
  StepMotor_ResumeStepping();

  // 최종 판단 수행 세 방향 모두 막혔을 때만 180도 회전
  if(wf && wl && wr){
    StepMotor_RotateDeg(180, +1, ROT_SPEED_US);  // 유턴 수행
    Forward_PID_OneBlock();                      // 한 블록 전진 후 다시 상황 판단
  }else{
    Forward_PID_OneBlock();                      // 센서 오판이면 그냥 전진
  }
}

/* 한 블록 PID 직진 주행 함수 */
void Forward_PID_OneBlock(void){
#ifndef FRONT_LOOKAHEAD_STEPS
#define FRONT_LOOKAHEAD_STEPS 60
#endif
  static uint16_t lookahead_cnt = 0;

  int32_t corr_acc = 0;
  uint8_t slow = 0;

  for(uint32_t i=0;i<BLOCK_STEPS;i++){
    uint16_t l = readSensor(LEFT_SENSOR);
    uint16_t r = readSensor(RIGHT_SENSOR);
    uint16_t f = readSensor(FRONT_SENSOR);

    // 회전 직후에는 옆 벽을 앞 벽으로 오인하지 않도록 센서 무시
    uint8_t fb_now;
    if(g_front_mask > 0){
      fb_now = 0;
      g_front_mask--;
    }else{
      fb_now = front_blocked_hyst(f);
    }

    // 바로 앞에 벽이 있으면 안전을 위해 정지
    if(f >= THRESH_FRONT_EMER) break;

    // 히스테리시스 및 룩어헤드 적용하여 미리 감속
    if (fb_now && lookahead_cnt==0) lookahead_cnt = FRONT_LOOKAHEAD_STEPS;
    if (lookahead_cnt > 0){
      slow = 0;
      lookahead_cnt--;
    }else{
      slow = fb_now;
    }

    // 오차 계산 왼쪽이 가까우면 양수이므로 오른쪽으로 회전 필요
    int16_t err = (int16_t)l - (int16_t)r;
    int16_t u   = PID_Update(&pid_center, err);
    corr_acc += u;

    // 좌우 모터 동시에 1스텝 전진
    StepMotor_Step(1, 1);
    if(slow) _delay_us(SLOW_DELAY_US);
    else     _delay_us(BASE_DELAY_US);

    // 오른쪽으로 방향 보정 왼쪽 바퀴만 구동
    while(corr_acc > +CORR_UNIT){
      StepMotor_Step(1, 0);
      if(slow) _delay_us(SLOW_DELAY_US);
      else     _delay_us(BASE_DELAY_US);
      corr_acc -= CORR_UNIT;
    }
    // 왼쪽으로 방향 보정 오른쪽 바퀴만 구동
    while(corr_acc < -CORR_UNIT){
      StepMotor_Step(0, 1);
      if(slow) _delay_us(SLOW_DELAY_US);
      else     _delay_us(BASE_DELAY_US);
      corr_acc += CORR_UNIT;
    }
  }
}

/* 방향 정의 및 DFS 관련 상수 */
#define DIR_N 0
#define DIR_E 1
#define DIR_S 2
#define DIR_W 3

#define MAZE_W    16
#define MAZE_H    16
#define STACK_MAX (MAZE_W*MAZE_H)

#define START_X (MAZE_W/2)
#define START_Y (MAZE_H/2)

static const int8_t dx[4] = { 0,  1,  0, -1};
static const int8_t dy[4] = {-1,  0,  1,  0};

static uint8_t visited[MAZE_H][MAZE_W];
static int8_t  stackX[STACK_MAX];
static int8_t  stackY[STACK_MAX];

static inline uint8_t leftOf (uint8_t d){ return (d+3)&3; }
static inline uint8_t rightOf(uint8_t d){ return (d+1)&3; }

/* 방향 회전 함수 90도 회전 시 안정화 동작 수행 유턴은 별도 처리 */
static void TurnTo(uint8_t *curDir, uint8_t targetDir){
  uint8_t diff = (targetDir + 4 - *curDir) & 3;
  if(diff==0){
    /* 방향 유지 */
  }else if(diff==1){
    StepMotor_RotateDeg(90, +1, ROT_SPEED_US);   // 우회전 수행 후 안정화 동작
    SeedForwardSteps(120);
    g_turn_commit = TURN_COMMIT_BLOCKS;
    g_front_mask  = FRONT_MASK_STEPS_AFTER_TURN;
    *curDir = targetDir;
  }else if(diff==3){
    StepMotor_RotateDeg(90, -1, ROT_SPEED_US);   // 좌회전 수행 후 안정화 동작
    SeedForwardSteps(120);
    g_turn_commit = TURN_COMMIT_BLOCKS;
    g_front_mask  = FRONT_MASK_STEPS_AFTER_TURN;
    *curDir = targetDir;
  }else{  // 180도 회전 요청 시 유턴 게이트 함수 호출
    Do_Uturn_GATED();
    /* 게이트 함수 내부에서 실제 회전 여부를 결정하므로 
       여기서는 헤딩 방향을 직접 수정하지 않음 */
  }
}

/* 왼쪽 전방 오른쪽 순서로 탐색하는 DFS 알고리즘 */
void DFS(void){
  // 방문 기록 초기화
  for(uint8_t y=0;y<MAZE_H;y++)
    for(uint8_t x=0;x<MAZE_W;x++)
      visited[y][x]=0;

  int8_t  curX = START_X;
  int8_t  curY = START_Y;
  uint8_t dir  = DIR_N;

  int16_t sp=0;
  visited[curY][curX]=1;
  stackX[sp]=curX; stackY[sp]=curY; sp++;

  while(1){
    // 현재 센서값 측정
    uint16_t f = readSensor(FRONT_SENSOR);
    uint16_t l = readSensor(LEFT_SENSOR);
    uint16_t r = readSensor(RIGHT_SENSOR);

    // DFS 판단 시 회전 직후 정면 센서값 무시 처리
    uint8_t wallF = (g_front_mask>0) ? 0 : front_blocked_hyst(f);
    uint8_t wallL = (l >= THRESH_LEFT);
    uint8_t wallR = (r >= THRESH_RIGHT);
    LED_show_walls(wallL, wallF, wallR);

    // 왼쪽 전방 오른쪽 순서로 이동 가능한 미방문 구역 확인
    uint8_t moved = 0;
    for(uint8_t k=0;k<3;k++){
      uint8_t candDir = (k==0)? leftOf(dir) : (k==1)? dir : rightOf(dir);

      uint8_t blocked = 1;
      if      (candDir==dir)            blocked = wallF;
      else if(candDir==leftOf(dir))   blocked = wallL;
      else if(candDir==rightOf(dir))  blocked = wallR;

      if(blocked) continue;

      int8_t nx = curX + dx[candDir];
      int8_t ny = curY + dy[candDir];
      if(nx<0 || nx>=MAZE_W || ny<0 || ny>=MAZE_H) continue;
      if(visited[ny][nx]) continue;

      TurnTo(&dir, candDir);          // 90도 회전 시 안정화 동작 수행
      Forward_PID_OneBlock();

      curX = nx; curY = ny;
      visited[ny][nx] = 1;
      if(sp < STACK_MAX){ stackX[sp]=curX; stackY[sp]=curY; sp++; } 
      // 스택에 현재 Y좌표 저장
      stackY[sp-1] = curY;

      if(g_turn_commit) g_turn_commit--;   // 커밋 스텝 차감
      moved = 1;
      break;
    }
    if(moved) continue;

    // 정면만 막히고 좌우 중 하나라도 열린 경우 유턴하지 않고 전진하며 재탐색
    if(wallF && !(wallL && wallR)){
      Forward_PID_OneBlock();
      if(g_turn_commit) g_turn_commit--;
      continue;
    }

    // 사방이 막힌 경우 유턴 게이트 함수 호출하여 신중하게 회전
    if(wallF && wallL && wallR){
      Do_Uturn_GATED();                // 실제 유턴 판단은 여기서 수행
      continue;
    }

    // 이동 불가 상태지만 커밋 스텝이 남아있으면 강제 전진
    if(g_turn_commit){
      Forward_PID_OneBlock();
      g_turn_commit--;
      continue;
    }

    // 갈 곳이 없으면 스택에서 좌표를 꺼내 되돌아감
    if(sp <= 1) break;  // 시작점으로 돌아오면 종료

    int8_t px = stackX[sp-2];
    int8_t py = stackY[sp-2];

    uint8_t backDir;
    if      (px==curX && py==curY-1) backDir = DIR_N;
    else if(px==curX && py==curY+1) backDir = DIR_S;
    else if(px==curX-1 && py==curY) backDir = DIR_W;
    else                             backDir = DIR_E;

    TurnTo(&dir, backDir);           // 180도 회전이 필요한 경우 게이트 함수를 통해 안전하게 수행
    Forward_PID_OneBlock();

    curX = px; curY = py;
    sp--;
  }

  while(1){  }
}
