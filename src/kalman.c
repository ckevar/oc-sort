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

void CVKalmanFilter::predict_Pi(float P[][KF_NUM_STATES], int i) {
    int j = i + 4;
    P[i][i] += P[j][j] + P[i][j] + P[j][i];
    P[i][j] += P[j][j];
    P[j][i] += P[j][j];
}




