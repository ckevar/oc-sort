#ifndef _SOA_KALMAN_H_
#define _SOA_KALMAN_H_

#include "include/kalman.h"
#include "soa/track.h"

void kalman_predict_soa(float *Q, struct Tracks *trks, int trk_i);
void kalman_update(float *R, float *y, struct Tracks *trk, int trk_i);

#endif


