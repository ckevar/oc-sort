#include "soa/kalman.h"

/* Debugging Header */
#include <cstdio>
/* END Debugging Header */

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
    // --- Predict state ---
    // NOTE: This is the OC-SORT way to avoid overshooting, it works but it 
    // might not be the best approach. The overshooting happens because upon 
    // track creation the bounding box is started from the bounding box, this 
    // leads the delta area `ds` to be predicted as really high, however in the
    // update this is shrink down, because the are cannot grow that fast, and in
    // the next prediction this speed gets shrunk down so negative that the 
    // kalman filter just explodes.
    if (trks->s[trk_i] + trks->ds[trk_i] <= 0.0f) 
        trks->ds[trk_i] = 0.0f;
    
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
void _compute_K_fast(float *K, float P[][KF_NUM_STATES], float *R) {
    K[0] = P[0][0] / (P[0][0] + R[0]);
    K[1] = P[1][1] / (P[1][1] + R[1]);
    K[2] = P[2][2] / (P[2][2] + R[2]);
    K[3] = P[3][3] / (P[3][3] + R[3]);

    K[4] = P[4][0] / (P[0][0] + R[0]);
    K[5] = P[5][1] / (P[1][1] + R[1]);
    K[6] = P[6][2] / (P[2][2] + R[2]);
}


void _update_state_with_K(struct Tracks *trks, int i, float *K, float *y) {
    trks->x[i] += K[0] * y[0];
    trks->y[i] += K[1] * y[1];
    trks->s[i] += K[2] * y[2];
    trks->r[i] += K[3] * y[3];

    trks->dx[i] += K[4] * y[0];
    trks->dy[i] += K[5] * y[1];
    trks->ds[i] += K[6] * y[2];
}

void _update_Pij_with_k(float P[][KF_NUM_STATES], float Rii, float *K, unsigned i) {
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

void _update_P33_with_k(float P[][KF_NUM_STATES], float R33, float K33) {
    P[3][3] = P[3][3] * (1 - K33) * (1 - K33) + K33 * K33 * R33;
}


void _update_P_with_K(float P[][KF_NUM_STATES], float *R, float *K) {
    _update_Pij_with_k(P, R[0], K, 0);
    _update_Pij_with_k(P, R[1], K, 1);
    _update_Pij_with_k(P, R[2], K, 2);
    _update_P33_with_k(P, R[3], K[3]);                              // NOTE: invoking a function is faster than unfolding
                                                                    // the array here.
}
void CVKalmanFilterSoA::update(float *y, struct Tracks *trks, int trk_i) {
    // Input:
    // - y: the difference between measured (z) and estimated (x): z - x,
    // - trks: a struct of arrays, holding all tracks,
    // - i: index of the track to be updated,
    //
    
    float K[KF_NUM_STATES];
    // 1. Update state X
    // 1.1. Compute fast K:
    _compute_K_fast(K, trks->covariance[trk_i], R);
    if (18 == trks->track_id[trk_i]) {
        printf("K_kf: %f %f %f %f %f %f %f\n", K[0], K[1], K[2], K[3], K[4], K[5], K[6]);
    }

    // 1.2. Update Track States
    _update_state_with_K(trks, trk_i, K, y);

    // 2. Update P having (K), there's another method that we can update P without K.
    _update_P_with_K(trks->covariance[trk_i], R, K);
}
// --- End Update
