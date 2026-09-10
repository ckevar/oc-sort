#include "hungarian.h"
#include <iostream>
#include <vector>
#include <climits>
#include <cstring>
#include <stdlib.h>
#include <time.h>

float *mk_cost3x3(void) {
    static float c[9];
    c[0] = 8.0; c[1] = 4.0; c[2] = 7.0;
    c[3] = 5.0; c[4] = 2.0; c[5] = 3.0;
    c[6] = 9.0; c[7] = 4.0; c[8] = 8.0;
    return c;
}

float *mk_cost2x3(void) {
    static float c[6];
    c[0] = 8.0; c[1] = 4.0; c[2] = 7.0;
    c[3] = 5.0; c[4] = 2.0; c[5] = 3.0;
    return c;
}

float *mk_cost3x2(void) {
    static float c[6];
    c[0] = 8.0; c[1] = 4.0; 
    c[2] = 5.0; c[3] = 2.0; 
    c[4] = 9.0; c[5] = 4.0; 
    return c;
}

float *mk_cost4x2(void) {
    static float c[8];
    //     det0       det1
    c[0] = 8.0; c[1] = 4.0;     // trk0
    c[2] = 5.0; c[3] = 2.0;     // trk1
    c[4] = 9.0; c[5] = 4.0;     // trk2
    c[6] = 10.0; c[7] = 5.0;    // trk3
    return c;
}


void print_cost(float const *cost, int const n, int const m) {
    int i, nm;

    nm = n*m;
    std::cout << "--------------------" << std::endl;
    std::cout << "Cost Matrix:" << std::endl << "--------------------" << std::endl;
    for (i = 0; i < nm; i++) {
        if (i % m == 0) {
            std::cout << std::endl;
        }
        std::cout << cost[i] << " ";
    }
    std::cout << std::endl << "--------------------" << std::endl;
}

/*
void associator_engine(int *raw_matched, unsigned n, unsigned m, char isTransposed) {
    unsigned len = isTransposed ? n:m;
    unsigned i;
    int unmatched_trks[5];
    int unmatched_dets[5];
    unsigned unmatched_trks_count = 0;
    unsigned unmatched_dets_count = 0;
    int row, trk_idx, det_idx;
    float match_cost;
    float match_iou;

    for(i = 1; i <= len; i++) {
        row = raw_matched[i];
        if (0 == row) {
            if (isTransposed) unmatched_trks[unmatched_trks_count++] = i - 1;
            else              unmatched_dets[unmatched_dets_count++] = i - 1;
        } else {
            if (isTransposed) {
                trk_idx = i - 1;
                det_idx = row - 1;
            } else {
                trk_idx = row - 1;
                det_idx = i - 1;
            }

            matrix_idx = trk_idx * m + det_idx;
            match_cost = cost_matrix[matrix_idx];
            match_iou = iou_matrix[matrix_idx];

            if (match_cost > cfg.threshold || match_iou > iou_thresh) {
                unmatched_trks[unmatched_trks_count++] = trk_idx;
                unmatched_dets[unmatched_dets_count++] = det_idx;
            } else {
                // kalman update
            }
            
        }
    }
}
*/

void parse_output(int *raw_matched, unsigned len, char isTransposed) {
    unsigned i;
    int unmatched_trks[5];
    int unmatched_dets[5];
    // int matched[5];
    unsigned unmatched_trks_count = 0;
    unsigned unmatched_dets_count = 0;
    unsigned matched_counts = 0;
    int row;

    if (isTransposed) {
        // ith: tracks, matches[ith]: dets
        for(i = 1; i <= len; i++) { 
            row = raw_matched[i];
            printf("%d %d\n", row, i);
            if (0 == row) {
                unmatched_trks[unmatched_trks_count] = i - 1;
                unmatched_trks_count++;
            } else {
                // matched[matched_counts] = {i - 1, row - 1}; // Do we need this?
                matched_counts++;
            }
        }

    } else {
        // ith: dets, matches[ith]: tracks
        for(i = 1; i <= len; i++) {
            if(0 == raw_matched[i]) {
                unmatched_dets[unmatched_dets_count] = i - 1;
                unmatched_dets_count++;
            } else {
                // matched[matched_counts] = {row - 1, i - 1}; // Do we need this?
                matched_counts++;
            }
        }
    }
}

int main(int argc, char *argv[]) {
  float *cost;
  int matched[5]; // the size of this is (max(n, m) + 1)
    uint8_t transposed_flag;

  cost = mk_cost4x2();

  print_cost(cost, 4, 2);

  transposed_flag = flinearsolver(matched, cost, 4, 2);
  printf("Transposed %d\n", transposed_flag);
  parse_output(matched, 4, transposed_flag);
  
  return 0;
}
