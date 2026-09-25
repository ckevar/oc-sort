#ifndef _AOS_TRACK_H_
#define _AOS_TRACK_H_

#include <stdint.h>

#include "include/configuration.h"

#ifndef KF_NUM_STATES
#define KF_NUM_STATES   7
#endif

#ifndef KF_NUM_COV_COMPACT
#define KF_NUM_COV_COMPACT  10
#endif

struct Track {
    union {
        struct {float x, y, s, r, dx, dy, ds;};
        float state[KF_NUM_STATES];
        float xysrbox[4];
    };

    /* Kalman's */
    // float covariance[KF_NUM_STATES][KF_NUM_STATES];     // NOTE/TODO: This is a waste of memory, saving a 7x7(=49)
                                            // when deployed, only 7 + 3 + 3 (=13 positions are used)
                                            // as depicted in the following matrix: 
                                            //
                                            // +-              +
                                            // | 1 0 0 0 1 0 0 |
                                            // | 0 1 0 0 0 1 0 |
                                            // | 0 0 1 0 0 0 1 |
                                            // | 0 0 0 1 0 0 0 |
                                            // | 1 0 0 0 2 0 0 |
                                            // | 0 1 0 0 0 2 0 |
                                            // | 0 0 1 0 0 0 2 |
                                            // +-             -+
                                            //
                                            // 1s means taken, 0s means they are never touched, and
                                            // there's also the most funny thing that 2s even they
                                            // are calculated, it seems that they are never used.

    float covariance[KF_NUM_COV_COMPACT];     // NOTE/TODO: This is a waste of memory, saving a 7x7(=49)

    // required for for re-runable Kalman
    uint8_t kf_observed_flag;
    
    union {
        struct {float frozen_x, frozen_y, frozen_s, frozen_ds;};
        float frozen_state[3];               // Given the nature of this kalman, only the initial states
                                            // are required to be predicted, the remaining states are
                                            // maintained constant when frozen.
    };
    // float frozen_covariance[KF_NUM_STATES][KF_NUM_STATES];
    float frozen_covariance[KF_NUM_COV_COMPACT];

    // xyxy
    union {
        struct {float x1, y1, x2, y2;};
        float xyxybox[4];
    };

    float vx;
    float vy;

    // History away from tracks state
    float observations[MAX_OBSERVATIONS][OBS_LENGTH];// bounding box[xyxy] | age
    float *latest_obs;                               // Points the to the last observations in `observations`
    float *momentum_obs;                             // Points to `latest_obs` or the 3rd previous observations in `observations`

    uint8_t latest_obs_available;                   // Remember that this has to be initialized as zero upon creation of a track is new. In FPGA this can be represented by a large number instead of an array of uint8_t
    uint16_t class_id;

    // Age could be uint8_t, but be careful that you could break alignment, 
    // meaning sizeof(Tracks) has to be an even number.
    uint16_t age;   

    // When OC-SORT is parallelized getting unique ids for evaluation might be a problem.
    uint32_t track_id; 
    
    // time_since_update can be uint8_t, since this is related to the age of
    // disappearing.
    uint16_t time_since_update;
    uint16_t hit_streak;

};



void xysr2xyxy_aos(struct Track *t);



#endif
