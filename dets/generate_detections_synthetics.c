// generate_synthetic.c
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <time.h>

#pragma pack(push, 1)
typedef struct {
    float frame_id;
    float x1, y1, x2, y2;
    float conf;
    float cls;
} DetectionRecord;
#pragma pack(pop)

typedef struct {
    float x, y, w, h;
    float vx, vy;
} SimTrack;

static float randf(float min, float max) {
    return min + (float)rand() / ((float)RAND_MAX / (max - min));
}

int main(int argc, char **argv) {
    if (argc < 4) {
        fprintf(stderr, "Usage: %s <output.bin> <num_frames> <dets_per_frame>\n", argv[0]);
        return 1;
    }

    const char *out_path = argv[1];
    int num_frames = atoi(argv[2]);
    int num_dets = atoi(argv[3]);

    FILE *f = fopen(out_path, "wb");
    if (!f) {
        perror("Failed to open output file");
        return 1;
    }

    srand(1337);

    // Initialize base targets across a 1920x1080 canvas
    SimTrack *tracks = malloc(num_dets * sizeof(SimTrack));
    for (int i = 0; i < num_dets; i++) {
        tracks[i].w = randf(30.0f, 120.0f);
        tracks[i].h = randf(50.0f, 200.0f);
        tracks[i].x = randf(0.0f, 1920.0f - tracks[i].w);
        tracks[i].y = randf(0.0f, 1080.0f - tracks[i].h);
        tracks[i].vx = randf(-2.0f, 2.0f);
        tracks[i].vy = randf(-2.0f, 2.0f);
    }

    DetectionRecord *batch = malloc(num_dets * sizeof(DetectionRecord));

    for (int frame = 0; frame < num_frames; frame++) {
        for (int i = 0; i < num_dets; i++) {
            // Update position with bounce
            tracks[i].x += tracks[i].vx;
            tracks[i].y += tracks[i].vy;

            if (tracks[i].x < 0 || tracks[i].x + tracks[i].w > 1920.0f) tracks[i].vx *= -1.0f;
            if (tracks[i].y < 0 || tracks[i].y + tracks[i].h > 1080.0f) tracks[i].vy *= -1.0f;

            // Add slight detection jitter
            float jitter_x = randf(-1.5f, 1.5f);
            float jitter_y = randf(-1.5f, 1.5f);

            batch[i].frame_id = (float)frame;
            batch[i].x1 = tracks[i].x + jitter_x;
            batch[i].y1 = tracks[i].y + jitter_y;
            batch[i].x2 = batch[i].x1 + tracks[i].w;
            batch[i].y2 = batch[i].y1 + tracks[i].h;
            batch[i].conf = randf(0.65f, 0.99f);
            batch[i].cls = 0.0f; // e.g., Car / Pedestrian
        }

        fwrite(batch, sizeof(DetectionRecord), num_dets, f);
    }

    free(batch);
    free(tracks);
    fclose(f);
    printf("Wrote %d detections (%zu bytes) to %s\n", 
           num_frames * num_dets, 
           (size_t)(num_frames * num_dets * sizeof(DetectionRecord)), 
           out_path);
    return 0;
}
