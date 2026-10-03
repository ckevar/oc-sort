#ifndef _AOS_KALMAN_H_
#define _AOS_KALMAN_H_

#include "include/kalman.h"

void kalman_predict(float *Q, float *state, float *P);
void kalman_update(float *R, float *state, float *P, float *innovation);

#endif
