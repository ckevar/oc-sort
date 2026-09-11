#include "include/motmanager.h"

MOTManager::MOTManager(const char *exp_name, uint8_t fmt) {
    log_file = fopen(exp_name, "w");
    sequence_format = fmt;
}


MOTManager::~MOTManager(void) {
    fclose(log_file);
}

void MOTManager::flush_trks(void) {
    fflush(log_file);
}

void MOTManager::close_trks(void) {
    fclose(log_file);
}

void MOTManager::save_trks(int frame_id, float *bbox, int len) {
    if (0 == len) {
        fprintf(stderr, "Warning: frame %d is empty.\n", frame_id);
        return;
    } 

    switch (sequence_format) {
    case MOT17_FMT:
        // FORMAT:
        // frame_id, track_id, left, top, width, height, ...
        for (int i = 0; i < len; i++) {
            fprintf(log_file, "%d,%d,%.2f,%.2f,%.2f,%.2f,1,-1,-1,-1\n", 
                    frame_id, (int) bbox[0], 
                    bbox[1], bbox[2], bbox[3] - bbox[1], bbox[4] - bbox[2]);
            bbox += 5;
        }
        break;
    case KITTI_FMT:
        // FORMAT:
        // frame_id, track_id, ... left, top, right, buttom
        bbox = bbox + 5 * (len - 1);    // NOTE: This is just because the python
                                        // implementation of OC-SORT outputs from
                                        // the highest id to the lowest id per 
                                        // frame.
        for (int i = 0; i < len; i++) {
            fprintf(log_file, "%d %d pedestrian 0 0 -10 %.2f %.2f %.2f %.2f -10 -10 -10 -100    0 -1000 -1000 -10\n",
                    frame_id, (int) bbox[0],
                    bbox[1], bbox[2], bbox[3], bbox[4]);
            bbox -= 5;                  // in normal mode, this is bbox += 5;
        }
        break;
    default:
        fprintf(stderr, "Warning: Unknown data type. Supported Data type `MOT17` or `KITTI`.\n");
        return;
    } 
    fflush(log_file);
}
