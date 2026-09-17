
#include "aos/detections.h"
#include "include/detections.h"

void 
dets_xyxy2xysr(
    struct DetectionAoS *dest,  
    struct Detection *src, 
    int len) 
{
    float w, h;
    for (int i = 0; i < len; i++) {
        w = src[i].x2 - src[i].x1;
        h = src[i].y2 - src[i].y1;
        dest[i].area = w * h;
        dest[i].x = src[i].x1 + (w * 0.5f);
        dest[i].y = src[i].y1 + (h * 0.5f);
        dest[i].ratio = w / (h + 1e-6);
        dest[i].raw = &src[i]; 
    }

}
