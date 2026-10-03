#ifndef _AOS_OCSORT_H_
#define _AOS_OCSORT_H_

#include "include/configuration.h"
#include "include/ocsort.h"

#include "aos/detections.h"
#include "aos/track.h"
#include "aos/kalman.h"

class OCSortAoS: public OCSort {
    private:
        void load_track_template(void);

    public: 
        OCSortAoS(OCSORTcfg config, const char *class_cfg_pathfile): OCSort(config, class_cfg_pathfile
                ) {load_track_template(); };
        int update(struct Detection *raw_dets, uint16_t raw_dets_len);

    private:
        /* Detections */
        struct DetectionAoS dets[MAX_DETECTIONS];

        /* Track */
        static struct Track trks[MAX_TRACKS];
        struct Track TRK_TEMPLATE;
        void predict_tracks(void);
        void update_track(int trk_idx, int det_idx);  // wrapper
        void freeze_track_state(struct Track *t);
        void unfreeze_track_state(struct Track *t, struct DetectionAoS *d);
        void update_track_state(struct Track *t, struct DetectionAoS *d);
        void update_track_observations(struct Track *t, float *det_raw);

        // First Association
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
