#ifndef _OCSORT_H_
#define _OCSORT_H_

#include "include/configuration.h"

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

        /* Track */
        int ID_manager;
        int active_trks;

        /* Misc */
        int frame_count;

};

void MK_OCSORT_DEFAULT_CONFIG(OCSORTcfg *cfg);

#endif
