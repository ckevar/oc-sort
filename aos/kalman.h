#ifndef _AOS_KALMAN_H_
#define _AOS_KALMAN_H_

#include "include/kalman.h"

class CVKalmanFilterAoS: public CVKalmanFilter {
    public:
        CVKalmanFilterAoS(void): CVKalmanFilter() {};
        void predict(float *state, float P[][KF_NUM_STATES]);
        void update(float *state, float P[][KF_NUM_STATES], float *innovation);
};

#endif
