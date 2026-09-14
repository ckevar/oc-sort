#ifndef _BBOX_H_
#define _BBOX_H_

struct XYXYBox {
    float x1;   // top
    float y1;   // left
    float x2;   // bottom
    float y2;   // right
};

struct XYSRBox {
    float x;    // x-center 
    float y;    // y-center
    float s;    // area
    float r;    // ratio 
};

#endif
