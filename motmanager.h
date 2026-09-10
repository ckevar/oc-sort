#ifndef _MOT_MANAGER_H_
#define _MOT_MANAGER_H_

#include <cstdio>
#include <cstdint>

enum {MOT17_FMT, KITTI_FMT};

class MOTManager {
    public:
        MOTManager(const char *exp_name, uint8_t fmt);

        template <typename Tracker>
        void run(Tracker& tracker, struct Detection *dets, long int dets_len);

        void save_trks(int frame_id, float *bbox, int len);
        void flush_trks(void);
        void close_trks(void);
        ~MOTManager(void);

    private:
        FILE *log_file;
        uint8_t sequence_format;
};

#endif
