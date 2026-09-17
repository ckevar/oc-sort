#include "aos/track.h"
#include <cmath>

void xysr2xyxy_aos(struct Track *t) {
    float w, h;
    w = sqrtf(t->s * t->r);
    h = t->s / w;
    w /= 2.0f;
    h /= 2.0f;
    t->x1 = t->x - w;
    t->x2 = t->x + w;
    t->y1 = t->y - h;
    t->y2 = t->y + h;

}
