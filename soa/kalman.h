#ifndef _SOA_KALMAN_H_
#define _SOA_KALMAN_H_

#include "include/kalman.h"
#include "soa/track.h"

class CVKalmanFilterSoA:public CVKalmanFilter {
    public:
        CVKalmanFilterSoA(void) : CVKalmanFilter() {};
        void predict_soa(struct Tracks *trks, int trk_i);
        void update(float *y, struct Tracks *trks, int trk_i);
};

#endif


