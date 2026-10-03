#include "include/configuration.h"
#include <cstdio>

void discard_comment_line(FILE *f) {
    
}

int load_class_config(struct ClassConfig *ccfg, const char *filepath) {

    FILE *f = fopen(filepath, "r");
    int ret, i;
    
    if (!f) return -1;

    for (i = 0; i < MAX_NUM_CLASSES; i++) {
        
        ret = fgetc(f);

        if('#' == ret) {
            while((ret = fgetc(f)) != '\n' && ret != EOF);
            if (EOF == ret) {
                fclose(f);
                return i;
            }
            i--;
            continue;
        }
        fseek(f, -1, SEEK_CUR);

        ret = fscanf(f, "%d %d %f %f %f %f %f %f %f %f %f %f %f",
                &ccfg[i].core_id,
                &ccfg[i].is_pedestrian,
                &ccfg[i].R[0], &ccfg[i].R[1],
                &ccfg[i].R[2], &ccfg[i].R[3],
                &ccfg[i].Q[0], &ccfg[i].Q[1],
                &ccfg[i].Q[2], &ccfg[i].Q[3],
                &ccfg[i].Q[4], &ccfg[i].Q[5],
                &ccfg[i].Q[6]);
        printf("Number of read parameters %d, Q[6] %f\n", ret, ccfg[i].Q[6]);
        if (ret < NUM_CLASS_CFG_PARAMS) {
            fclose(f);
            return -1;
        }
    }

    fclose(f);
    return i;
}
