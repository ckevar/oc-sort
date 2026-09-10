#include "detections.h"

#include <cstdio>

void det_AoS2SoA(struct DetectionSoA *dest, struct Detection *src, int dets_len) {
    // cats Detectectioins that are in Array of Structures into structure of
    // arrays.
    float w, h;
    dest->raw = src;
    for (int i = 0; i < dets_len; i ++) {
        w = src[i].x2 - src[i].x1;
        h = src[i].y2 - src[i].y1;
        dest->area[i] = w * h;
        dest->x[i] = src[i].x1 + (w * 0.5f);
        dest->y[i] = src[i].y1 + (h * 0.5f);
        dest->ratio[i] = w / (h + 1e-6);
    }
}


