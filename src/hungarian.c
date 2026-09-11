#include <climits>
#include <iostream>
#include <cstdint>
#include <cstring>
#include <cfloat>

/* Solver for floating cost */

void init_farrays(float *u, float *v, int *p, int *way, int m, int n) {
    // NOTE: m >= n
    int i;
    for(i = 0; i <= n; i++) {
        u[i] = 0.0;
        v[i] = 0.0;
        p[i] = 0;
        way[i] = 0;
    }
    for(; i <= m; i++) {
        v[i] = 0.0;
        p[i] = 0;
        way[i] = 0;
    }
}

void _flinearsolver(int *p, float *cost, int const n, int const m) {
    float u[n + 1], v[m + 1], minv[m + 1];
    int way[m + 1];
    uint8_t unused[m + 1];
    int i, j, i0, j0, j1, i0_j;
    float delta, cur;

    init_farrays(u, v, p, way, m, n);
    for (i = 1; i <= n; i++) {
        p[0] = i;
        j0 = 0;

        for(j = 0; j <= m; j++) {
            minv[j] = FLT_MAX;
            unused[j] = 1;
        }

        do {
            unused[j0] = 0;
            i0 = p[j0];
            delta = FLT_MAX;
            for(j = 1; j <= m; j++) {
                if(unused[j]) {
                    i0_j = m * (i0 - 1) + j - 1;
                    cur = cost[i0_j] - u[i0] - v[j];
                    if (cur < minv[j])
                        minv[j] = cur, way[j] = j0;
                    if (minv[j] < delta)
                        delta = minv[j], j1 = j;
                }    
            }

            for (j = 0; j <= m; j++) {
                if (unused[j])
                    minv[j] -= delta;
                else
                    u[p[j]] += delta, v[j] -= delta;
            }
            j0 = j1;
        } while(p[j0]);

        do {
            j1 = way[j0];
            p[j0] = p[j1];
            j0 = j1;
        } while(j0);
    }

    /*
     * This section requires trans as input, to know if the cost matrix was 
     * transposed or not.
#ifdef PRINT_ASSOCIATION
    std::cout << "#--- ASSOCIATION ---#" << std::endl;
    if (trans)
        std::cout << "ROW COL (transposed)" << std::endl;
    else
        std::cout << "col row" << std::endl;

    for(i = 1; i <= m; i++) {
        std::cout << i << "  " << p[i] << std::endl;
    }
#endif
    */

    // return -v[0]; For association, the cost isn't the interest of when associating

}

// Define the absolute maximum objects your tracker will ever handle per frame
/* --- FPGA - Vitis ---
#define MAX_N 100 
#define MAX_M 100

float _flinearsolver_transposed(int *out, const float *cost, int n, int m) {
    // 1. HARDWARE FIX: Statically sized arrays for BRAM synthesis
    float costT[MAX_M * MAX_N]; 
    int outT[MAX_N + 1];        
    
    int i, j;
    float min_cost;

    // Transpose
    for(i = 0; i < n; i++) {
        // 2. HARDWARE FIX: Tell Vitis the max loop bounds so it can schedule the silicon
        #pragma HLS LOOP_TRIPCOUNT min=1 max=MAX_N 
        
        for (j = 0; j < m; j++) {
            #pragma HLS LOOP_TRIPCOUNT min=1 max=MAX_M
            
            // 3. HARDWARE FIX: Pipeline the inner loop for massive speed
            #pragma HLS PIPELINE II=1 
            
            costT[n * j + i] = cost[m * i + j];
        }
    }

    // Solve the cost matrix
    min_cost = _flinearsolver(outT, costT, m, n, 1);

    // Untranspose output
    // for(i = 1; i <= n; i++) {
    //     #pragma HLS LOOP_TRIPCOUNT min=1 max=MAX_N
    //     #pragma HLS PIPELINE II=1
    //     
    //     out[outT[i]] = i;
    // }

    // return min_cost;
}
 */

void _flinearsolver_transposed(int *out, float *cost, int n, int m) {
    float costT[m * n]; // For FPGA - VITIS, this requires to be statically
                        // allocated (even for computers, static allication 
                        // can give us some timing improvements.
                        // TODO: check if statically allocating this, we can 
                        // achieve some improvements.
    // int outT[n + 1];
    int i, j;
    
    // Transpose
    for(i = 0; i < n; i++) {
        for (j = 0; j < m; j++) {
            costT[n * j + i] = cost[m * i + j];
        }
    }
    
    // Solve the cost matrix
    // _flinearsolver(outT, costT, m, n, 1);
    _flinearsolver(out, costT, m, n);

    // Untranspose output:
    // cols rows, meaning out is output[cols] = rows[cols]
    
    // for(i = 1; i <= n; i++) {
    //    out[outT[i]] = i;
    // }

}

uint8_t flinearsolver(int *out, float *cost, int n, int m) {
    // *************************************
    // * NOTE:                             *
    // * This solver requires that m >= n. *
    // *************************************
    if (n > m) {
        _flinearsolver_transposed(out, cost, n, m);
        return 1;
    }

    _flinearsolver(out, cost, n, m);
    return 0;
}

