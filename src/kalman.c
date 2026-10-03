#include "include/kalman.h"

// Measurment Noise (default)
//   R[0] = R[1] = 1.0; 
//   R[2] = R[3] = 10.0;
//
// Process Noise (default)
//   Q[0] = Q[1] = 1.0;  
//   Q[2] = Q[3] = 1.0;
//   Q[4] = Q[5] = 0.01;
//   Q[6] = 0.01*0.01;
//
// Covariance Matrix
//   +-                        +
//   | P0  0  0  0 p7*  0   0  |
//   |  0 P1  0  0  0  p8*  0  |
//   |  0  0 P2  0  0   0  p9* |
//   |  0  0  0 P3  0   0   0  |
//   | P7  0  0  0 P4   0   0  |
//   |  0 P8  0  0  0  P5   0  |
//   |  0  0 P9  0  0   0  P6  |
//   +-                       -+
// NOTE: p7* = P7, p8* = P8, p9* = P9
 
void gkalman_fast_gain(float *K, float *P, float *R) {
    K[0] = P[0] / (P[0] + R[0]);
    K[1] = P[1] / (P[1] + R[1]);
    K[2] = P[2] / (P[2] + R[2]);
    K[3] = P[3] / (P[3] + R[3]);

    K[4] = P[7] / (P[0] + R[0]);
    K[5] = P[8] / (P[1] + R[1]);
    K[6] = P[9] / (P[2] + R[2]);
}

void gkalman_predict_Pi(float *P, int i) {
    int j = i + 4;
    int ij = i + 7;

    P[i] += P[j] + 2.0f * P[ij];
    P[ij] += P[j];
}

void _update_Pij_with_k(float *P, float Rii, float *K, unsigned i) {
    unsigned j = i + 4;
    unsigned ij = i + 7;
    float Pii = P[i];
    float Pij = P[ij];
    float Kii = K[i];
    float Kji = K[j];

    float _1_Kii = 1 - Kii;
    float KiKjiRii = Kii * Kji * Rii;
    float Pji_KjiPii = Pij - Kji * Pii;

    P[i] = Pii * _1_Kii*_1_Kii + Kii*Kii * Rii;
    P[ij] = Pji_KjiPii * _1_Kii + KiKjiRii;
    P[j] = Kji * (-Pij - Pji_KjiPii) + P[j] + Kji*Kji * Rii; 
}

void _update_P33_with_k(float *P, float R33, float K33) {
    P[3] = P[3] * (1 - K33) * (1 - K33) + K33 * K33 * R33;
}

void gkalman_update_P_with_K(float *P, float *K, float *R) {
    _update_Pij_with_k(P, R[0], K, 0);
    _update_Pij_with_k(P, R[1], K, 1);
    _update_Pij_with_k(P, R[2], K, 2);
    _update_P33_with_k(P, R[3], K[3]);                              // NOTE: invoking a function is faster than unfolding
                                                                    // the array here.
}


