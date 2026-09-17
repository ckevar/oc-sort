#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>

#include "include/utils.h"
#include "include/detections.h"
#include "include/motmanager.h"

#include "soa/ocsort.h"

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
void MOTManager::run(OCSortSoA& ocsort, struct Detection *dets, long int dets_len) {
    int dets_offset = 0; // this has to be long long 
    long int frame_dets_len;
    struct Detection *frame_dets = NULL;
    int trk_count, frame_id;

    // Timing
    struct timespec tstart, tend;
    long elapsedTimePerFrame = 0;
    
    for(frame_id = 0; 1; frame_id++) {
        frame_dets_len = count_detections_in_frame(dets, frame_id, dets_offset, dets_len);

        if (frame_dets_len < 0) {
            break;
        }

        frame_dets = dets + dets_offset; 
    
        clock_gettime(CLOCK_MONOTONIC, &tstart);
        trk_count = ocsort.update(frame_dets, frame_dets_len);
        clock_gettime(CLOCK_MONOTONIC, &tend);

        save_trks(frame_id, (float *)ocsort.bbox_out, trk_count);

        dets_offset += frame_dets_len;

        // Timing ns
        elapsedTimePerFrame += (tend.tv_sec - tstart.tv_sec) * 1000000000L +
            (tend.tv_nsec - tstart.tv_nsec);

        // if (frame_id == 2) break;
    }
    elapsedTimePerFrame = elapsedTimePerFrame / ((long) frame_id);
    printf("Average Time SoA: %ldns\n", elapsedTimePerFrame);
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
    
    printf("Creating OC-SORT..\n");
    MK_OCSORT_DEFAULT_CONFIG(&ocsort_cfg);
    OCSortSoA ocsort(ocsort_cfg);

    if(ocsort.isValid() == 0) {
        return 1;
    }
    
    printf("Loading detections\n");
    dets_len = dets_open(argv[1], 0, (float **)&dets);
    if (dets_len <= 0)
        return 1;
    
    printf("Running MOT... \n");
    mot.run(ocsort, dets, dets_len);

    free(dets);
    return 0;
}


