#ifndef _OC_SORT_CONFIG_H_
#define _OC_SORT_CONFIG_H_

/* Detections Configurations */
#define MAX_DETECTIONS      500

/* Tracks Configurations */
#define MAX_TRACKS          500
#define MAX_OBSERVATIONS    3   // Larger than max occlusion (30)
#define OBS_LENGTH          5   // Gross length: Bbox | age
                                // NOTE: `age` allows to keep track of the bbox
 

/* OC-SORT Configurations */
#define OBS_AGE_INDEX       4   // Age is stored as a helper
#define OBS_NET_LENGTH      4   // BBox is the most important thing XYXY
#endif
 
