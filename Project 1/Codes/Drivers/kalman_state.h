/*
 * kalman_state.h
 */
#include "main.h"

#ifndef KALMAN_STATE_H_
#define KALMAN_STATE_H_

#ifndef kalman_state
#define kalman_state

typedef struct kalman_states{
	float q, r, x, p, k;
}kalman_state;

void new_kalman_state(*kalman_state, q, r, x, p, k);
float update(*kalman_state, measurement);

#endif /* KALMAN_STATE_H_ */
