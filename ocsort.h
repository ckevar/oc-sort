#ifndef _OCSORT_H_
#define _OCSORT_H_

#include "detections.h"
#include "track.h"
#include "kalman.h"

// --- OC-SORT refactoring

typedef struct {
    // TODO: Some of the int values here might be changed to a limit number
    int max_age;
    int min_hits;
    float iou_threshold;
    float det_thresh;
    int delta_t;
    float inertia;
} OCSORTcfg;

class OCSort {
    public:
        OCSort(OCSORTcfg config);
        int update(struct Detection *detsAoS, uint16_t dets_len);
        char isValid(void);
        float bbox_out[MAX_TRACKS][5];

    private:
        /* Initialization */
        OCSORTcfg cfg;
        char cfg_valid;
        char config_check(OCSORTcfg *config);

        /* Detections */
        struct DetectionSoA dets;  
        uint16_t dets_len;

        /* Track */
        int ID_manager;
        struct Tracks trks;
        int active_trks;
        void update_trk(int trk_idx, int det_idx);
        void predict_trks(void);
        void kf_update_trk(int trk_idx, int det_idx);
        CVKalmanFilter kf;

        /* Association */
        int matched[MAX_TRACKS + 1]; // trks and detections
        int unmatched_trks[MAX_TRACKS];
        int unmatched_dets[MAX_DETECTIONS];
        unsigned unmatched_trks_count, unmatched_dets_count;
        float cost_matrix[MAX_TRACKS * MAX_DETECTIONS];
        float iou_matrix[MAX_TRACKS * MAX_DETECTIONS];

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

        /* Misc */
        int frame_count; // TODO: do we need this?
};

void MK_OCSORT_DEFAULT_CONFIG(OCSORTcfg *cfg);

#endif
