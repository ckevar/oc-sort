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
    float det_thresh;
    int delta_t;
    float inertia;
} OCSORTcfg;

class OCSort {
    public:
        OCSort(OCSORTcfg config);
        char isValid(void);
        float bbox_out[MAX_TRACKS][5];

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


        /* Misc */
        int frame_count;

};

/* Misc */

void MK_OCSORT_DEFAULT_CONFIG(OCSORTcfg *cfg);
int prune_low_conf_dets(float th, struct Detection *dets, int dets_len);

#endif
