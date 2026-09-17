#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "include/utils.h"
#include "include/detections.h"
#include "include/motmanager.h"

#include "soa/ocsort.h"
#include "aos/ocsort.h"

long int
count_detections_in_frame(
    struct Detection *dets, 
    int frame_id,
    long offset,
    long max_dets_len)
{
    struct Detection *tail;
    struct Detection *head;
    long int len = 0;

    head = dets + offset;
    tail = head;

    if (head >= (dets + max_dets_len)) {
        return -1;
    }

    for(; (int) head->frame_id == frame_id; head++);

    len = head - tail;
    return len;
}

template <>
void MOTManager::run(OCSortAoS& ocsort, struct Detection *dets, long int dets_len) {
    int dets_offset = 0; // this has to be long long 
    long int frame_dets_len;
    struct Detection *frame_dets = NULL;
    int trk_count;
    
    for(int frame_id = 0; 1; frame_id++) {
        frame_dets_len = count_detections_in_frame(dets, frame_id, dets_offset, dets_len);

        if (frame_dets_len < 0) {
            return;
        }

        frame_dets = dets + dets_offset; 

        // TODO: catch time here
        trk_count = ocsort.update(frame_dets, frame_dets_len);
        // TODO: end time here
        //
        save_trks(frame_id, (float *)ocsort.bbox_out, trk_count);

        dets_offset += frame_dets_len;
        // if (frame_id == 134) break;
    }
}

int main(int argc, char *argv[]) {
    long int dets_len;
    struct Detection *dets = NULL;
    OCSORTcfg ocsort_cfg;
    MOTManager mot("track-test", KITTI_FMT);
    
    if (argc < 1) {
        fprintf(stderr, "We need a detections binary file.\n");
        return 1;
    }

    MK_OCSORT_DEFAULT_CONFIG(&ocsort_cfg);
    OCSortAoS ocsort(ocsort_cfg);

    if(ocsort.isValid() == 0) {
        return 1;
    }

    dets_len = dets_open(argv[1], 0, (float **)&dets);
    if (dets_len <= 0)
        return 1;
    
    mot.run(ocsort, dets, dets_len);

    free(dets);
    return 0;
}


