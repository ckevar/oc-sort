#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <stdlib.h>
#include <string.h>
#include "utils.h"

#define DEFAULT_DET_NUM_FIELDS  7
#define DEFAULT_DET_SIZE        (DEFAULT_DET_NUM_FIELDS * sizeof(float))

long int dets_open(char *det_filename, int det_size, float **dets) {
    // Loads the binary detection file
    int fd = open(det_filename, O_RDONLY, S_IRUSR);
    struct stat sb;

    if (-1 == fd) {
        fprintf(stderr, "[ERROR:] Opening file.\n");
        return -1;
    }

    if (-1 == fstat(fd, &sb)) {
        fprintf(stderr, "[ERROR:] Couldn't get file size.\n");
        return -1;
    }

    if (0 == det_size) {
        det_size = DEFAULT_DET_SIZE;
    }

    if (sb.st_size % det_size != 0) {
        fprintf(stderr, 
                "[ERRPR:] File is corrupted or number of fields are not standard\n" \
                "standard detection length is: \n" \
                "  +----------+---------------+---------+---------@\n" \
                "  | Frame ID | Bounding Box* | Confid. | Class   |\n" \
                "  +==========+===============+=========+=========+\n" \
                "  | float    | float         | float   | float   |\n" \
                "  +----------+---------------+---------+---------+\n" \
                "  | 4bytes   | 4 x 4bytes    | 4bytes  | 4bytes  | = 28 bytes\n" \
                "  +----------+---------------+---------+---------+\n" \
                " *bounding box: [x1, y1, x2, y2], array of float32\n");
        dets = NULL;
        close(fd);
        return -1;

    }
    
    float *dets_file = (float *) mmap(NULL, sb.st_size, PROT_READ, MAP_PRIVATE, fd, 0);

    *dets = (float *) malloc(sizeof(float) * sb.st_size);
    if (NULL == dets) {
        munmap(dets_file, sb.st_size);
        close(fd);
        return -1;
    }

    memcpy(*dets, dets_file, sb.st_size);

    munmap(dets_file, sb.st_size);
    close(fd);

    return sb.st_size / det_size;
}

