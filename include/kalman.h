#ifndef _KALMAN_H_
#define _KALMAN_H_

#define KF_NUM_STATES       7
#define KF_NUM_COV_COMPACT  10 // Instead of using a 7x7 matrix it uses a 10 1D array
#define KF_NUM_MEASUREMENTS 4

class CVKalmanFilter {
    public:
        CVKalmanFilter(void);
        void predict(struct Track *t);

    protected:
        float Q[KF_NUM_STATES];
        float R[KF_NUM_MEASUREMENTS];
        /* Legacy
        void predict_Pi(float P[][KF_NUM_STATES], int i);
        void compute_K_fast(float *K, float P[][KF_NUM_STATES], float *R);
        void update_P_with_K(float P[][KF_NUM_STATES], float *K, float *R);
        void update_Pij_with_k(float P[][KF_NUM_STATES], float Rii, float *K, unsigned i);
        void update_P33_with_k(float P[][KF_NUM_STATES], float R33, float K33);
        */
        void predict_Pi(float *P, int i);
        void compute_K_fast(float *K, float *P, float *R);
        void update_P_with_K(float   *P, float *K, float *R);
        void update_Pij_with_k(float *P, float Rii, float *K, unsigned i);
        void update_P33_with_k(float *P, float R33, float K33);
 
};


#endif
