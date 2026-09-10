#include "kalman.h"
#include "track.h"
#include "utils.h"

#include <cstdio>
#include <cstdlib>
#include <climits>
#include <ctime>
#include <cmath>

#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <cstring>

#define NUM_FRAMES      500
#define FRAME_WIDTH     1920
#define FRAME_HEIGHT    1080
#define PROCESS_NOISE   3.0

// TODO:
// 1. Create a fake trajectory, this could be done out of detections
// 2. Add noise to it,
// 3. Use kalman filter

void mk_trajectory(unsigned num_frames) {

    srand(time(0));

    // 1. pick a random position, initialize
    float x = (float) (rand() % FRAME_WIDTH);
    float y = (float) (rand() % FRAME_HEIGHT);
    float w = (float) (rand() % 100 + 50);
    float h = (float) (rand() % 200 + 100);
    float vx = (float) (rand()) / RAND_MAX;
    float vy = (float) (rand()) / RAND_MAX;

    // 2. Simulate frames
    for (float fid = 0.0; fid < (float) num_frames; fid += 1.0) {
        float cam_shift_x = 2.0 * sin(fid / 10.0);
        float cam_shift_y = 0.5 * cos(fid / 10.0);
        float noisex = (float)(rand()) / RAND_MAX * PROCESS_NOISE;
        float noisey = (float)(rand()) / RAND_MAX * PROCESS_NOISE;
        float noisew = ((float)(rand()) / RAND_MAX - 0.5);
        float noiseh = ((float)(rand()) / RAND_MAX - 0.5);
        x += vx + noisex - cam_shift_x;
        y += vy + noisey - cam_shift_y;
        w += noisew;
        h += noiseh;

        // Boundary
        if (x < 0.0 || x > (float)FRAME_WIDTH) vx *= -1.0;
        if (y < 0.0 || y > (float)FRAME_HEIGHT) vy *= -1.0;
        printf("%f %f %f %f\n", x, y, w, h);
    }
}

void init_trk_idx(struct Tracks *trks, unsigned i, float *z) {
    // This is how OC-SORT starts the tracks.
    trks->x[i] = z[0] + z[2] / 2.0;
    trks->y[i] = z[1] + z[3] / 2.0;
    trks->s[i] = z[2] * z[3]; 
    trks->r[i] = z[2] / (z[3] + 1e-6);

    for (int j = 0; j < 7; j++) 
        trks->covariance[i][j][j] =  1.0;

    trks->covariance[i][4][4] *= 1000.0;
    trks->covariance[i][5][5] *= 1000.0;
    trks->covariance[i][6][6] *= 1000.0; // NOTE: there's an initial huge uncertainty which might be causing identity switches when tracking objects.

    for (int j = 0; j < 7; j++) 
        trks->covariance[i][j][j] *=  10.0;

}

long int meas_open(char **meas, char *filename) {
    int fd = open(filename, O_RDONLY, S_IRUSR);
    struct stat sb;
    if( -1 == fd) {
        fprintf(stderr, "[ERROR:] Opening file.\n");
        return -1;
    }

    if (-1 == fstat(fd, &sb)) {
        fprintf(stderr, "[ERROR:] Couldn't get file size.\n");
        return -1;
    }

    char *meas_file = (char *) mmap(NULL, sb.st_size, PROT_READ, MAP_PRIVATE, fd, 0);
    *meas = (char *) malloc(sizeof(char) * sb.st_size);

    if (NULL == meas)  {
        munmap(meas_file, sb.st_size);
        close(fd);
        return -1;
    }

    memcpy(*meas, meas_file, sb.st_size);
    munmap(meas_file, sb.st_size);
    close(fd);

    return sb.st_size;
}
void print_covariance(float P[][7]) {
    for (int i = 0; i < 7; i ++) {
        for(int j = 0; j < 7; j++) {
            printf("%f ", P[i][j]);
        }
        printf("\n");
    }
}

#define LOAD_DETS(out, ptr0, ptr1) \
    out[0] = strtof(ptr0, &ptr1); \
    out[1] = strtof(ptr1, &ptr0); \
    out[2] = strtof(ptr0, &ptr1); \
    out[3] = strtof(ptr1, &ptr0)

#define TRKi  0

void run_kalman(char *filename) {
    struct Tracks trks;
    CVKalmanFilter kf;
    char *measurements;
    char *EOL0, *EOL1;
    long int measurements_sz;
    float z[4] = {0.0, 0.0, 0.0, 0.0}; // z = {x, y, w, h}
    // timing variables
    struct timespec tstart, tend;
    double ttotal_us;

    measurements_sz = meas_open(&measurements, filename);
    if (measurements_sz < 0){
        return;
    }

    EOL0 = measurements;
    LOAD_DETS(z, EOL0, EOL1); // this is kind of confusing, i know.
                              // but EOL0 holds the value of the last pointer
                              // just because the numbers of columns are pairs.

    init_trk_idx(&trks, TRKi, z);

    while(1) {
        float y[4];

        LOAD_DETS(z, EOL0, EOL1);

        if ((0.0 == z[0]) || (0.0 == z[1]) || (0.0 == z[2]) || (0.0 == z[3])) {
            break;
        }

        kf.predict_soa(&trks, TRKi);
        
        float r = z[2] * z[3];          // r = w * h
        float s = z[2] / (z[3] + 1e-6); // s = w / (h + 1e-6)
                                        //
                                        // Solve for w and H:
                                        //  w = s * (h + 1e-6)  ... [1]
                                        //  r = s * h^2         
                                        //  h = sqrt(r/s)       ... [2]


        z[0] += z[2] / 2.0;
        z[1] += z[3] / 2.0;
 
        y[0] = z[0] - trks.x[TRKi];
        y[1] = z[1] - trks.y[TRKi];
        y[2] = s    - trks.s[TRKi];
        y[3] = r    - trks.r[TRKi];

        clock_gettime(CLOCK_MONOTONIC, &tstart);
        kf.update(y, &trks, TRKi);
        clock_gettime(CLOCK_MONOTONIC, &tend);

        ttotal_us = (double)(tend.tv_sec - tstart.tv_sec) * 1000000.0 + \
                            (tend.tv_nsec - tstart.tv_nsec) / 1000.0;
        printf("%f\n", ttotal_us);

        // r = sqrt(trks.r[TRKi]/trks.s[TRKi]);    // h: r = height
        // s = trks.s[TRKi] * r;                   // w: s = width
        // printf("%f %f %f %f\n", trks.x[TRKi] - s / 2.0, trks.y[TRKi] - r / 2.0, s, r);
    }


    free(measurements);
}


int main(int argc, char *argv[]) {
    if (argc < 2) {
        fprintf(stderr, "[Warning:] Trajectory is being Generated.\n");
        mk_trajectory(NUM_FRAMES);
        fprintf(stderr, "[Warning:] Trajectory Generated.\n");
        return 0;
    }

    run_kalman(argv[1]);

    return 0;
}

