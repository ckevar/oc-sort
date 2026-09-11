#ifndef _KALMAN_H_
#define _KALMAN_H_

#define KF_NUM_STATES       7
#define KF_NUM_MEASUREMENTS 4

class CVKalmanFilter {
    public:
        CVKalmanFilter(void);
        void predict(struct Track *t);

    protected:
        float Q[KF_NUM_STATES];
        float R[KF_NUM_MEASUREMENTS];
        void predict_Pi(float [][KF_NUM_STATES], int i);
};


#endif
