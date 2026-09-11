#ifndef _SOA_DETECTIONS_H_
#define _SOA_DETECTIONS_H_

#include "include/detections.h"
#include "include/configuration.h"

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
