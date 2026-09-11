#ifndef _SOA_TRACKLET_H_
#define _SOA_TRACKLET_H_

#include <stdint.h>
#include "include/configuration.h"

struct Tracks {
    float x[MAX_TRACKS];                    // Center x
    float y[MAX_TRACKS];                    // Center y
    float s[MAX_TRACKS];                    // Area
    float r[MAX_TRACKS];                    // Ratio
    float dx[MAX_TRACKS];
    float dy[MAX_TRACKS];
    float ds[MAX_TRACKS];

    /* Kalman's */
    float covariance[MAX_TRACKS][7][7];     // NOTE/TODO: This is a waste of memory, saving a 7x7(=49)
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
    // required for for re-runable Kalman
    uint8_t kf_observed_flag[MAX_TRACKS];
    float frozen_x[MAX_TRACKS];
    float frozen_y[MAX_TRACKS];
    float frozen_s[MAX_TRACKS];
    float frozen_covariance[MAX_TRACKS][7][7];

    // xyxy
    float x1[MAX_TRACKS];
    float x2[MAX_TRACKS];
    float y1[MAX_TRACKS];
    float y2[MAX_TRACKS];


    float vx[MAX_TRACKS];
    float vy[MAX_TRACKS];

    // History away from tracks state
    float observations[MAX_TRACKS][MAX_OBSERVATIONS][OBS_LENGTH];// bounding box | age
    float latest_obs[MAX_TRACKS][OBS_LENGTH];                   // the most recent matched detection (literally the bounding box of the detection) holds x1,y1,x2,y2
    float history_obs[MAX_TRACKS][OBS_LENGTH];
    uint8_t latest_obs_available[MAX_TRACKS];                   // Remember that this has to be initialized as zero upon creation of a track is new. In FPGA this can be represented by a large number instead of an array of uint8_t
    float momentum_obs[MAX_TRACKS][OBS_NET_LENGTH];             // the last kth observation (the last direction of motion).
    uint16_t class_id[MAX_TRACKS];

    // Age could be uint8_t, but be careful that you could break alignment, 
    // meaning sizeof(Tracks) has to be an even number.
    uint16_t age[MAX_TRACKS];   

    // When OC-SORT is parallelized getting unique ids for evaluation might be a problem.
    uint32_t track_id[MAX_TRACKS]; 
    
    // time_since_update can be uint8_t, since this is related to the age of
    // disappearing.
    uint16_t time_since_update[MAX_TRACKS];

    uint16_t hit_streak[MAX_TRACKS];

};




#endif
