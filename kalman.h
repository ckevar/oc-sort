#ifndef _KALMAN_H_
#define _KALMAN_H_

#include "track.h"

#define KF_NUM_STATES          7
#define KF_NUM_MEASUREMENTS    4

class CVKalmanFilter {
    public:
        CVKalmanFilter(void);
        void predict_soa(struct Tracks *trks, int trk_i);
        void update(float *y, struct Tracks *trks, int trk_i);
    private:
        float Q[KF_NUM_STATES];
        float R[KF_NUM_MEASUREMENTS];
};

#endif


