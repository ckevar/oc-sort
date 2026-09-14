#ifndef _DETECTIONS_H_
#define _DETECTIONS_H_

#include "bbox.h"

struct Detection {
    float frame_id;
    union {
        struct {float x1, y1, x2, y2;};
        float xyxybox[4];
    };
    float score;
    float class_id;
};

#endif
