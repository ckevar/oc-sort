#ifndef _GENERIC_KALMAN_H_
#define _GENERIC_KALMAN_H_

#define KF_NUM_STATES       7
#define KF_NUM_COV_COMPACT  10 // Instead of using a 7x7 matrix it uses a 10 1D array
#define KF_NUM_MEASUREMENTS 4
#define KF_NUM_CLASSES      1

void gkalman_predict_Pi(float *P, int i);
void gkalman_fast_gain(float *K, float *P, float *R);
void gkalman_update_P_with_K(float   *P, float *K, float *R);

#endif
