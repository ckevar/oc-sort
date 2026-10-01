#ifndef _SOA_OCSORT_H_
#define _SOA_OCSORT_H_

#include "include/detections.h"
#include "include/ocsort.h"

#include "soa/detections.h"
#include "soa/track.h"

#include "soa/kalman.h"

// --- OC-SORT refactoring

class OCSortSoA: public OCSort {
    public:
        OCSortSoA(OCSORTcfg config) : OCSort(config) {};
        int update(struct Detection *detsAoS, uint16_t dets_len);

    private:
        /* Detections */
        struct DetectionSoA dets;  

        /* Track */
        static struct Tracks trks;
        void predict_trakcs(void);
        void update_track(int trk_idx, int det_idx);
        void update_track_state(int trk_idx, int det_idx);
        void update_track_observations(int trk_idx, int det_idx);
        void predict_tracks(void);
        void freeze_track_state(int trk_idx);
        void unfreeze_track_state(int trk_idx, int det_idx);
        CVKalmanFilterSoA kf;


        /* First Association */
        void cost_stage1(void);
        void association_stage1(void);

        /* Second Association */
        int cost_stage2(void);
        void association_stage2(void);

        /* Track Management */
        void update_unmatched_tracks(void);
        void init_tracks(void);
        int export_and_prune_tracks(void);
        void reallocate_track(unsigned dest_i, unsigned src_i);

};

#endif
