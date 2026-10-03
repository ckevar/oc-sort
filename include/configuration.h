#ifndef _OC_SORT_CONFIG_H_
#define _OC_SORT_CONFIG_H_

/* Detections Configurations */
#define MAX_DETECTIONS      3100

/* Tracks Configurations */
#define MAX_TRACKS          3100
#define MAX_OBSERVATIONS    3   // Larger than max occlusion (30)
#define OBS_LENGTH          5   // Gross length: Bbox | age
                                // NOTE: `age` allows to keep track of the bbox
 

/* OC-SORT Configurations */
#define MAX_NUM_CLASSES     1   // 1: Monolithic tracking, > 1: Parallel tracking
#define OBS_AGE_INDEX       4   // Age is stored as a helper
#define OBS_NET_LENGTH      4   // BBox is the most important thing XYXY


#ifndef M_PI_F
#define M_PI_F              3.14159265358979323846f
#endif

/* Class Configuration */
#define NUM_CLASS_CFG_PARAMS    13  // number of fields of Struct ClassConfig

struct ClassConfig {
    int core_id;
    int is_pedestrian;
    float Q[7];         // Process Noise, noise per state
    float R[4];         // Measurement Noise, noise per measured state
};


int load_class_config(struct ClassConfig *ccfg, const char *filepath);


#endif
 
