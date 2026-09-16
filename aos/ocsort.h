#ifndef _AOS_OCSORT_H_
#define _AOS_OCSORT_H_

#include "include/configuration.h"
#include "include/ocsort.h"

#include "aos/detections.h"
#include "aos/track.h"
#include "aos/kalman.h"

// DEBUGING purposes {
#define OCSORT_SOA_NOT_IMPLEMENTED() \
    fprintf(stderr, "[TODO:] %s() is not implemented yet.\n", __func__)
// }

class OCSortAoS: public OCSort {
    private:
        void load_track_template(void);

    public: 
        OCSortAoS(OCSORTcfg config): OCSort(config) {load_track_template(); };
        int update(struct Detection *raw_dets, uint16_t raw_dets_len);

    private:
        /* Detections */
        struct DetectionAoS dets[MAX_DETECTIONS];

        /* Track */
        struct Track trks[MAX_TRACKS];
        struct Track TRK_TEMPLATE;
        void predict_trks(void);
        void update_trk(int trk_idx, int det_idx);  // wrapper
        void freeze_state(struct Track *t);
        void unfreeze_state(struct Track *t, struct DetectionAoS *d);
        void update_trk_state(struct Track *t, struct DetectionAoS *d);
        void update_trk_observations(struct Track *t, float *det_raw);
        CVKalmanFilterAoS kf;

        // First Association
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
