#ifndef _AOS_DETECTIONS_H_
#define _AOS_DETECTIONS_H_

#include "include/detections.h"

struct DetectionAoS {
    union {
        struct {float x, y, area, ratio;};
        float xysrbox[4];
    };
    struct Detection *raw;
};

void 
dets_xyxy2xysr(
    struct DetectionAoS *dest,  
    struct Detection *src, 
    int len) ;

#endif
