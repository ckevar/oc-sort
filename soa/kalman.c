#include "soa/kalman.h"

/* Debugging Header */
#include <cstdio>
/* END Debugging Header */

#include <cstring>

// This is hardcoded kalman for a 7-state CV model used in SORT and OC-SORT. where:
// 1. state = [x, y, scale/area, aspect ratio, dx, dy, dscare/darea]
// 2. The measured parameters are:
//    z = [x, y, scale/area, aspect ratio]
// 3. Matrices R, Q, are identity matrices.
// 4. P starts as identity but due to F matrix, it's not identiy but its 
//    deterministic and easy to compute.
//



// --- Predict --


void CVKalmanFilterSoA::predict_soa(struct Tracks *trks, int trk_i) {
   
    trks->x[trk_i] += trks->dx[trk_i];
    trks->y[trk_i] += trks->dy[trk_i];
    trks->s[trk_i] += trks->ds[trk_i];
    // trks.r[trk_i]  = trks.r[trk_i];
    // trks.dx[trk_i] = trks.dx[trk_i];
    // trks.dy[trk_i] = trks.dy[trk_i];
    // trks.ds[trk_i] = trks.ds[trk_i];

    // --- Predict Covariance ---
    predict_Pi(trks->covariance[trk_i], 0);   
    predict_Pi(trks->covariance[trk_i], 1);   
    predict_Pi(trks->covariance[trk_i], 2);
    for (int j = 0; j < KF_NUM_STATES; j++)
        trks->covariance[trk_i][j][j] += Q[j];
    
    // NOTE: ---
    // Apparently we dont use P[4][4], P[5][5], nor P[6][6]. They keep stacking
    // up, but their values are not relevant in the ultimate calculations of P
    // or K, they seem to be a closed loop.
    // ---------

}

// --- End Predict ---

// --- Update ---

void _update_state_with_K(struct Tracks *trks, int i, float *K, float *y) {
    trks->x[i] += K[0] * y[0];
    trks->y[i] += K[1] * y[1];
    trks->s[i] += K[2] * y[2];
    trks->r[i] += K[3] * y[3];

    trks->dx[i] += K[4] * y[0];
    trks->dy[i] += K[5] * y[1];
    trks->ds[i] += K[6] * y[2];
}



void CVKalmanFilterSoA::update(float *y, struct Tracks *trks, int trk_i) {
    // Input:
    // - y: Innovation Array, the difference between measured (z) and estimated (x): z - x,
    // - trks: a struct of arrays, holding all tracks,
    // - i: index of the track to be updated,
    //
    
    float K[KF_NUM_STATES];
    // 1. Update state X
    // 1.1. Compute fast K:
    compute_K_fast(K, trks->covariance[trk_i], R);

    // 1.2. Update Track States
    _update_state_with_K(trks, trk_i, K, y);

    // 2. Update P having (K), there's another method that we can update P without K.
    update_P_with_K(trks->covariance[trk_i], K, R);
}
// --- End Update



