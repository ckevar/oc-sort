#include "include/kalman.h"

CVKalmanFilter::CVKalmanFilter(void) {

    // Measurment Noise
    R[0] = R[1] = 1.0; 
    R[2] = R[3] = 10.0;
    
    // Process Noise
    Q[0] = Q[1] = 1.0;  
    Q[2] = Q[3] = 1.0;
    Q[4] = Q[5] = 0.01;
    Q[6] = 0.01*0.01;

}

void CVKalmanFilter::compute_K_fast(float *K, float P[][KF_NUM_STATES], float *R) {
    K[0] = P[0][0] / (P[0][0] + R[0]);
    K[1] = P[1][1] / (P[1][1] + R[1]);
    K[2] = P[2][2] / (P[2][2] + R[2]);
    K[3] = P[3][3] / (P[3][3] + R[3]);

    K[4] = P[4][0] / (P[0][0] + R[0]);
    K[5] = P[5][1] / (P[1][1] + R[1]);
    K[6] = P[6][2] / (P[2][2] + R[2]);
}

void CVKalmanFilter::predict_Pi(float P[][KF_NUM_STATES], int i) {
    int j = i + 4;
    P[i][i] += P[j][j] + P[i][j] + P[j][i];
    P[i][j] += P[j][j];
    P[j][i] += P[j][j];
}

void CVKalmanFilter::update_Pij_with_k(float P[][KF_NUM_STATES], float Rii, float *K, unsigned i) {
    unsigned j = i + 4;
    float Pii = P[i][i];
    float Pij = P[i][j];
    float Kii = K[i];
    float Kji = K[j];

    float _1_Kii = 1 - Kii;
    float KiKjiRii = Kii * Kji * Rii;
    float Pji_KjiPii = P[j][i] - Kji * Pii;                         // NOTE: Pji is not faster thatn P[j][i]

    P[i][i] = Pii * _1_Kii*_1_Kii + Kii*Kii * Rii;
    P[i][j] = (-Kji * Pii + Pij) * _1_Kii + KiKjiRii;               // NOTE: KjiPii is not faster than Kji * Pii
    P[j][i] = Pji_KjiPii * _1_Kii + KiKjiRii;
    P[j][j] = Kji * (-Pij - Pji_KjiPii) + P[j][j] + Kji*Kji * Rii;  // NOTE: pre-computed Pjj is not faster that P[j][j]
}

void CVKalmanFilter::update_P33_with_k(float P[][KF_NUM_STATES], float R33, float K33) {
    P[3][3] = P[3][3] * (1 - K33) * (1 - K33) + K33 * K33 * R33;
}

void CVKalmanFilter::update_P_with_K(float P[][KF_NUM_STATES], float *K, float *R) {
    update_Pij_with_k(P, R[0], K, 0);
    update_Pij_with_k(P, R[1], K, 1);
    update_Pij_with_k(P, R[2], K, 2);
    update_P33_with_k(P, R[3], K[3]);                              // NOTE: invoking a function is faster than unfolding
                                                                    // the array here.
}


