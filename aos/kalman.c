#include "aos/kalman.h"

#include <cstring>

enum {
    IDX_X = 0,
    IDX_Y,
    IDX_S,
    IDX_R,
    IDX_dX,
    IDX_dY,
    IDX_dS
};

// --- Predict ---
void kalman_predict(float *Q, float *state, float *P) {

    state[IDX_X] += state[IDX_dX];
    state[IDX_Y] += state[IDX_dY];
    state[IDX_S] += state[IDX_dS];

    gkalman_predict_Pi(P, 0);
    gkalman_predict_Pi(P, 1);
    gkalman_predict_Pi(P, 2);
    
    for(int j = 0; j < KF_NUM_STATES; j++) 
        P[j] += Q[j];

    // NOTE: ---
    // Apparently we dont use P[4][4], P[5][5], nor P[6][6]. They keep stacking
    // up, but their values are not relevant in the ultimate calculations of P
    // or K, they seem to be a closed loop.
    // ---------

}
// --- End Predict ---

// --- Update ----
void _update_state_with_K(float *state, float *K, float *innovation) {
    state[IDX_X] += K[IDX_X] * innovation[IDX_X];
    state[IDX_Y] += K[IDX_Y] * innovation[IDX_Y];
    state[IDX_S] += K[IDX_S] * innovation[IDX_S];
    state[IDX_R] += K[IDX_R] * innovation[IDX_R];

    state[IDX_dX] += K[IDX_dX] * innovation[IDX_X];
    state[IDX_dY] += K[IDX_dY] * innovation[IDX_Y];
    state[IDX_dS] += K[IDX_dS] * innovation[IDX_S];
}


void kalman_update(
    float *R, 
    float *state, 
    float *P, 
    float *innovation) 
{
    float K[KF_NUM_STATES]; // NOTE: This is local, because This class is a manager
                            
    gkalman_fast_gain(K, P, R);
    _update_state_with_K(state, K, innovation);
    gkalman_update_P_with_K(P, K, R);
}
// --- END Update ---

