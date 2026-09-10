#ifndef _DETECTIONS_H_
#define _DETECTIONS_H_

#include "configuration.h"

struct Detection {
    float frame_id;
    float x1, y1, x2, y2;
    float score;
    float class_id;
};

struct DetectionSoA {
    float x[MAX_DETECTIONS];
    float y[MAX_DETECTIONS];
    float area[MAX_DETECTIONS];
    float ratio[MAX_DETECTIONS];
    struct Detection *raw;
};


// Casts Array-of-Struct Detections into Struct-of-Arrays
void det_AoS2SoA(struct DetectionSoA *dest, struct Detection *src, int dets_len);


#endif
