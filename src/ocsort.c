#include "include/ocsort.h"
#include <cstdio>

float OCSort::bbox_out[MAX_TRACKS][5];
int OCSort::matched[MAX_TRACKS + 1]; // trks and detections
int OCSort::unmatched_trks[MAX_TRACKS];
float OCSort::cost_matrix[MAX_TRACKS * MAX_DETECTIONS];
float OCSort::iou_matrix[MAX_TRACKS * MAX_DETECTIONS];

OCSort::OCSort(OCSORTcfg config) {
    cfg = config;
    if(0 == config_check(&config)) {
        cfg_valid = 1;
        cfg = config;
    } else {
        cfg_valid = 0;
        cfg = {0};
    }

    active_trks = 0;
    frame_count = 0;
    ID_manager = 1;
}

char OCSort::config_check(OCSORTcfg *config) {
    if (config->delta_t > MAX_OBSERVATIONS) {
        fprintf(stderr, 
                "[ERROR:] delta_t (%d) is larger than MAX_OBSERVATIONS (%d)." \
                " If you need that amount of observations, increase MAX_OBSERVATIONS", 
                config->delta_t, MAX_OBSERVATIONS);
        return 1;
    }

    return 0;
}


char OCSort::isValid(void) {
    return cfg_valid;
}

/* Misc */

void MK_OCSORT_DEFAULT_CONFIG(OCSORTcfg *cfg) {
    cfg->max_age        = 30;
    cfg->min_hits      = 3;
    cfg->iou_threshold  = 0.3;
    cfg->det_thresh     = 0.5;
    cfg->delta_t        = 3;
    cfg->inertia        = 0.2;
}

int prune_low_conf_dets(float th, struct Detection *dets, int dets_len) {
    for (int i = 0; i < dets_len; i++) {
        if (dets[i].score < th) {
            dets_len--;
            dets[i] = dets[dets_len];
            i--;
        }
    }

    return dets_len;
}


