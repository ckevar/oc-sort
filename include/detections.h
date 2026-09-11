#ifndef _DETECTIONS_H_
#define _DETECTIONS_H_

struct Detection {
    float frame_id;
    float x1, y1, x2, y2;
    float score;
    float class_id;
};

#endif
