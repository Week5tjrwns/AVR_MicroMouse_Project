#ifndef ALGORITHM_H_
#define ALGORITHM_H_

#include <stdint.h>

void Forward_PID_Until_Obstacle(uint32_t max_steps);

// PID 제어로 한 블록 직진
void Forward_PID_OneBlock(void);

// 미로 탐색 메인 알고리즘 실행
void DFS(void);

#endif
