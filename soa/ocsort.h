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
        struct Tracks trks;
        void update_trk(int trk_idx, int det_idx);
        void update_trk_state(int trk_idx, int det_idx);
        void update_trk_observations(int trk_idx, int det_idx);
        void predict_trks(void);
        void freeze_state(int i);
        void unfreeze_state(int i, int j);
        CVKalmanFilterSoA kf;


        /* First Association */
        void compute_first_cost(void);
        void first_association(void);

        /* Second Association */
        int compute_second_cost(void);
        void second_association(void);

        /* Track Management */
        void update_unmatched_tracks(void);
        void create_new_tracks(void);
        int export_and_prune_tracks(void);
        void trackcpy(unsigned dest_i, unsigned src_i);

};

#endif
