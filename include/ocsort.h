#ifndef _OCSORT_H_
#define _OCSORT_H_

#include "include/configuration.h"
#include "include/detections.h"

#include <cstdint>

typedef struct {
    // TODO: Some of the int values here might be changed to a limit number
    int max_age;
    int min_hits;
    float iou_threshold;
    float iou_lower_bound;
    float det_thresh;
    int delta_t;    // For mod operations is better to keep them as `int`
    float inertia;
} OCSORTcfg;

class OCSort {
    public:
        OCSort(OCSORTcfg config);
        char isValid(void);
        static float bbox_out[MAX_TRACKS][5];

    protected:
         /* Initialization */
        OCSORTcfg cfg;
        char cfg_valid;
        char config_check(OCSORTcfg *config);

        /* Detections */
        uint16_t dets_len;

        /* Track */
        int ID_manager;
        int active_trks;

        /* Association */
        static int matched[MAX_TRACKS + 1]; // trks and detections
        static int unmatched_trks[MAX_TRACKS];
        int unmatched_dets[MAX_DETECTIONS];
        unsigned unmatched_trks_count, unmatched_dets_count;
        static float cost_matrix[MAX_TRACKS * MAX_DETECTIONS];
        static float iou_matrix[MAX_TRACKS * MAX_DETECTIONS];

        /* Misc */
        int frame_count;

};

/* Misc */

void MK_OCSORT_DEFAULT_CONFIG(OCSORTcfg *cfg);
int prune_low_conf_dets(float th, struct Detection *dets, int dets_len);

#endif
