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
    float cls;
} SimTrack;

static float randf(float min, float max) {
    return min + (float)rand() / ((float)RAND_MAX / (max - min));
}

#ifdef KITTI_CLASS_DISTRIBUTION
#define CLASS_DISTRIBUTION_USR 1
#endif

#ifdef MOT17_CLASS_DISTRIBUTION
#define CLASS_DISTRIBUTION_USR 2
#endif

#ifdef WAYMOv2_CLASS_DISTRIBUTION
#define CLASS_DISTRIBUTION_USR 3
#endif


#if CLASS_DISTRIBUTION_USR == 1

    #define MOST_COMMON_CLASS_INDEX 0
    #define CAR_NUM         579
    #define PEDESTRIAN_NUM  167
    #define VAN_NUM         57
    #define CYCLIST_NUM     37
    #define PERSON_NUM      34
    #define TRUCK_NUM       13
    #define TRAM_NUM        12
    
    const float CLASS_DISTRIBUTION[]={
        CAR_NUM, 
        PEDESTRIAN_NUM, 
        VAN_NUM,
        CYCLIST_NUM,
        PERSON_NUM,
        TRUCK_NUM,
        TRAM_NUM
    };

#elif CLASS_DISTRIBUTION_USR == 2

    #define MOST_COMMON_CLASS_INDEX 0
    #define PEDESTRIAN_NUM  546
    #define CAR_NUM         34
    #define BICYCLE_NUM     19
    #define ST_PERSON_NUM   19
    #define PER_ON_VEH_NUM  7
    #define NON_MOTOR_NUM   1
    #define MOTORBIKE_NUM   1
    
    const float CLASS_DISTRIBUTION[]={
        PEDESTRIAN_NUM,
        CAR_NUM,
        BICYCLE_NUM,    
        ST_PERSON_NUM,  
        PER_ON_VEH_NUM, 
        NON_MOTOR_NUM, 
        MOTORBIKE_NUM,  
    };

#elif CLASS_DISTRIBUTION_USR == 3

    #define MOST_COMMON_CLASS_INDEX 0
    #define CAR_NUM         34198
    #define PEDESTRIAN_NUM  9965
    #define BICYCLE_NUM     381
    
    const float CLASS_DISTRIBUTION[]={
        CAR_NUM,
        PEDESTRIAN_NUM,
        BICYCLE_NUM,    
    };

#else

    #error "Error: You must define either KITTI_CLASS_DISTRIBUTION or MOT_CLASS_DISTRIBUTION."

#endif


#define NUM_CLASS() (sizeof(CLASS_DISTRIBUTION) / sizeof(int))

float class_fetcher(void) {
    // Yields a class based on the distribution of the dataset. It uses
    // montecarlo to generate code
    static float max_count = CLASS_DISTRIBUTION[MOST_COMMON_CLASS_INDEX];
    static float num_cls = (float) NUM_CLASS();

    float cls_count;
    int cls_index;
    
    do {

        cls_count = randf(0.0f, max_count);
        cls_index = (int) randf(0.0f, num_cls);
        // printf("count %f, index %d\n", cls_count, cls_index);

    } while(CLASS_DISTRIBUTION[cls_index] <= cls_count);

    // printf("PICKED: count %f, index %d\n", cls_count, cls_index);
    // printf("count ceil %f, index %d\n", CLASS_DISTRIBUTION[cls_index], cls_index);
    return (float) cls_index;
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
        tracks[i].cls = class_fetcher();
        printf("%f\n", tracks[i].cls);
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
            batch[i].cls = tracks[i].cls;
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
