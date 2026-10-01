#include "include/hungarian.h"

#include "soa/ocsort.h"
#include "soa/track.h"

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

struct Tracks OCSortSoA::trks;

// --- Begin Prediction
void k_previous_obs(
    float *output,
    float input[][OBS_LENGTH], 
    int16_t cur_age, 
    int16_t k, 
    float *last_obs) 
{
    int16_t dt, target_age, observation_age;
    float *slot;

    for(dt = k; dt > 0; dt--) {
        target_age = cur_age - dt;
        slot = input[target_age % k]; // int idx = (target_age % k + k) % k;
        observation_age = (int16_t) slot[OBS_AGE_INDEX];
        if (observation_age == target_age) {
            output[0] = (slot[2] + slot[0]) / 2.0f;
            output[1] = (slot[3] + slot[1]) / 2.0f;
            return;
        }
    }
    output[0] = (last_obs[2] + last_obs[0]) / 2.0f;
    output[1] = (last_obs[3] + last_obs[1]) / 2.0f;
}

inline void get_k_previous_observation(struct Tracks *t, int trk_idx, uint16_t k) {
    k_previous_obs(
        t->centerxy[trk_idx],
        t->observations[trk_idx],
        t->age[trk_idx],
        k,
        t->latest_obs[trk_idx]);
}

void xysr_to_xyxy_soa(struct Tracks *trks, int trk_idx) {
    float w, h;
    w = sqrtf(trks->s[trk_idx] * trks->r[trk_idx]);
    h = trks->s[trk_idx] / w;
    w /= 2.0f;
    h /= 2.0f;
    trks->x1[trk_idx] = trks->x[trk_idx] - w;
    trks->x2[trk_idx] = trks->x[trk_idx] + w;
    trks->y1[trk_idx] = trks->y[trk_idx] - h;
    trks->y2[trk_idx] = trks->y[trk_idx] + h;
}

void OCSortSoA::predict_tracks(void) {
    
    for (int ti = 0; ti < active_trks; ti++) {
        
        // --- Predict state ---
        // NOTE: This is the OC-SORT way to avoid overshooting, it works but it 
        // might not be the best approach. The overshooting happens because upon 
        // track creation the bounding box is started from the bounding box, this 
        // leads the delta area `ds` to be predicted as really high, however in the
        // update this is shrink down, because the are cannot grow that fast, and in
        // the next prediction this speed gets shrunk down so negative that the 
        // kalman filter just explodes.
        if (trks.s[ti] + trks.ds[ti] <= 0.0f) {
            trks.ds[ti] = 0.0f;
        }

        kf.predict_soa(&trks, ti);
        
        trks.age[ti] += 1;
        trks.time_since_update[ti] += 1;
    
        if (trks.time_since_update[ti] > 1) {
            // this means previously ~time_since_update~ = 0;
            trks.hit_streak[ti] = 0;
        }

        xysr_to_xyxy_soa(&trks, ti);
        get_k_previous_observation(&trks, ti, cfg.delta_t);
    }
}

// --- End Prediction


// --- Begin First Association Cost

float area_of_intersection_SoA(
    struct Detection *d,
    struct Tracks *t, 
    int trk_idx)
{
    float xmin, ymin, xmax, ymax;
    xmin = d->x1 > t->x1[trk_idx] ? d->x1 : t->x1[trk_idx];
    ymin = d->y1 > t->y1[trk_idx] ? d->y1 : t->y1[trk_idx];
    xmax = d->x2 < t->x2[trk_idx] ? d->x2 : t->x2[trk_idx];
    ymax = d->y2 < t->y2[trk_idx] ? d->y2 : t->y2[trk_idx]; 
    
    xmax -= xmin;
    ymax -= ymin;

    if ((xmax <= 0.0f) || (ymax <= 0.0f)) return 0.0f;
    return xmax * ymax;
}

inline void 
iou_SoA(
    float *iou, 
    struct DetectionSoA *d,  int dj, 
    struct Tracks *t, int ti) 
{
    // ---
    // Computes IoU between detection and predicted bounding box.
    // ---
    float inter_area = area_of_intersection_SoA(&d->raw[dj], t, ti);
    *iou = inter_area / (d->area[dj] + t->s[ti] - inter_area);
}

inline float iou_with_areas(float *xyxy1, float area1, float *xyxy2, float area2) {
    float inter_area = area_of_intersection(xyxy1, xyxy2);
    return inter_area / (area1 + area2 - inter_area);
}

inline void 
compute_momentum_cost(
    float *angle_diff,
    struct DetectionSoA *d, int dj, 
    struct Tracks *t, int ti)
{
    // This functions ranges from -0.5 to 0.5

    // Intention of motion
    float delta_x = d->x[dj] - t->centerxy[ti][0];
    float delta_y = d->y[dj] - t->centerxy[ti][1];
    float norm = sqrtf(delta_x * delta_x + delta_y * delta_y) + 1e-6f;

    // Momentum Similarity
    *angle_diff = t->vx[ti] * delta_x + t->vy[ti] * delta_y;
    *angle_diff = *angle_diff / norm;


    *angle_diff = fminf(1.0f, fmaxf(-1.0f, *angle_diff));
    *angle_diff = acosf(*angle_diff);
    *angle_diff = 0.5f  - (*angle_diff / M_PI_F);
}

// --- End First Association Cost

/******************************/
/*    OC-SORT Refactoring     */
/******************************/


// TODO: change the structure name Detection to AoS_Detection, so we standardize
//       `struct Detection` for SoA data structure.
int OCSortSoA::update(struct Detection *AoSdets, uint16_t AoSdets_len) {

    frame_count++;

    // printf("F%d: dets in count %d:\n----\n", frame_count - 1, AoSdets_len);

    if (0 == AoSdets_len) { 
        return 0;
    }

    // frame_count++; BUG?
    // this is were the official implementation counts frames, which is kind of
    // a safe guard, avoiding the blow up of covariance.
    
    dets_len = prune_low_conf_dets(cfg.det_thresh, AoSdets, AoSdets_len);

    unmatched_dets_count = 0;
    unmatched_trks_count = 0;
    
    det_AoS2SoA(&dets, AoSdets, dets_len);
    
    predict_tracks();

    cost_stage1();
    association_stage1();

    if ((unmatched_trks_count > 0) && (unmatched_dets_count > 0)) {
        if (cost_stage2()) {
            association_stage2();
        }
    }

    
    update_unmatched_tracks();
    init_tracks();
    
    // printf("  Active Tracks: %d\n  unmatched detections: %d\n  Unmatched Tracks %d\n  det len %d\n", 
    //        active_trks, unmatched_dets_count, unmatched_trks_count, dets_len);
    return export_and_prune_tracks();
}

void OCSortSoA::cost_stage1(void) {
    int ti, dj, row_index;
    float iou, angle_diff;
    float *row_cost;
    float *row_iou;
    
    /* TODO:
    fprintf(stderr, "[WARNING@FIRST COST]: Even though a computer can solve floating points; however, when deployed on FPGAs, these values have to be integers\n");
    */

    if (0 == active_trks) {
        return;
    }
    
    // cost matrix is nxm = active_trks x dets_len
    for (ti = 0; ti < active_trks; ti++) {

        row_index = ti * dets_len;
        row_cost = &cost_matrix[row_index];
        row_iou =  &iou_matrix[row_index];

        for (dj = 0; dj < dets_len; dj++) {

            //--- IoU 
            iou_SoA(&iou, &dets, dj, &trks, ti);  // Range: 0.0 to 1.0
            iou = (iou < cfg.iou_lower_bound) ? 0.0f : iou; // Tiny IoU alters
                                                            // the cost in a way 
                                                            // that creates IDsw
                                                            // unlike python's 
                                                            // implementation that
                                                            // relies on a greedy
                                                            // algorithm this one
                                                            // uses hungarian one.

            //--- Momentum Cost
            compute_momentum_cost(&angle_diff, &dets, dj, &trks, ti);     // Range: -0.5 to 0.5
            angle_diff = angle_diff * cfg.inertia * dets.raw[dj].score;  // Range: -0.1 to 0.1
            
            //--- Fill the matrices
            // Then, cost matrix's range: -0.1 to 1.1, inverted: -1.1 to 0.1
            //
            // NOTE: 
            // 1. The hungarian implementation can't handle negative numbers, 
            //    so it is biased adding 1.2 (for safety reasons).
            // 2. This also suggest we can quantize the cost, so it becomes a integer 
            //    based matrix, which is smaller and faster to compute on constraint 
            //    devices.
            
            row_iou[dj] = iou;
            row_cost[dj] = -(iou + angle_diff) + 1.2f;
        }
    }

}

int OCSortSoA::cost_stage2(void) {
    unsigned int ti, dj, det_idx;
    float iou, iou_max, t_latest_area;
    float *row_cost;
    float *t_latest_obs;

    /* TODO:
    fprintf(stderr, "[WARNING@SECOND COST]: Even though a computer can solve floating points; however, when deployed on FPGAs, these values have to be integers\n");
    */

    iou_max = 0.0f;
    for (ti = 0; ti < unmatched_trks_count; ti++) {

        t_latest_obs = trks.latest_obs[unmatched_trks[ti]];
        t_latest_area = (t_latest_obs[2] - t_latest_obs[0]) * (t_latest_obs[3] - t_latest_obs[1]);
        row_cost = &cost_matrix[ti * unmatched_dets_count];

        for(dj = 0; dj < unmatched_dets_count; dj++) {
            det_idx = unmatched_dets[dj];

            iou = iou_with_areas(dets.raw[det_idx].xyxybox, dets.area[det_idx], 
                    t_latest_obs, t_latest_area);
            row_cost[dj] = -iou + 1.0f; // biasing this is important to solve the hungarian
            if (iou > iou_max) iou_max = iou;
        }
    }   
    
    // NOTE: The second stage is only executed if the iou_max is larger than iou_max
    if (iou_max > cfg.iou_threshold) 
        return 1;
    return 0;
}

// --- Freezing / Unfreezing ---
void OCSortSoA::freeze_track_state(int trk_idx) {
    trks.frozen_x[trk_idx] = trks.x[trk_idx];
    trks.frozen_y[trk_idx] = trks.y[trk_idx];
    trks.frozen_s[trk_idx] = trks.s[trk_idx];
    trks.frozen_ds[trk_idx] = trks.ds[trk_idx]; // Even though it doesn't change within the
                                    // kalman filter because they are predicted
                                    // it changes due to guarding upon every 
                                    // prediction
    
    // NOTE: This are maintained because they aren't predicted
    // trks.frozen_r[trk_idx]  = trks.r[trk_idx];
    // trks.frozen_dx[trk_idx] = trks.dx[trk_idx];
    // trks.frozen_dy[trk_idx] = trks.dy[trk_idx];
    memcpy(trks.frozen_covariance[trk_idx], 
           trks.covariance[trk_idx], 
           KF_NUM_COV_COMPACT * sizeof(float));
}

void OCSortSoA::unfreeze_track_state(int trk_idx, int det_idx) {
    // 1. copy back the frozen values
    trks.x[trk_idx] = trks.frozen_x[trk_idx];
    trks.y[trk_idx] = trks.frozen_y[trk_idx];
    trks.s[trk_idx] = trks.frozen_s[trk_idx];
    trks.ds[trk_idx] = trks.frozen_ds[trk_idx];

    // NOTE: Remaining states (r, dx, dy, ds) are never modified during prediction
    // because of the constant velocity model.
    memcpy(trks.covariance[trk_idx], 
           trks.frozen_covariance[trk_idx], 
           KF_NUM_COV_COMPACT * sizeof(float));

    float time_gap = (float) trks.time_since_update[trk_idx]; 
    // NOTE: in the oficial implementation, the `history obs` stores the xysr, 
    // which requires sqrt operations and divisions, instead we are saving
    // the xyxy, this only needs differences and a couple of divisions
    
    // Box 1
    float w1 = trks.latest_obs[trk_idx][2] - trks.latest_obs[trk_idx][0];
    float h1 = trks.latest_obs[trk_idx][3] - trks.latest_obs[trk_idx][1];
    float x1 = trks.latest_obs[trk_idx][0] + w1 / 2.0f;
    float y1 = trks.latest_obs[trk_idx][1] + h1 / 2.0f;

    // Box 2
    float dw = dets.raw[det_idx].x2 - dets.raw[det_idx].x1;
    float dh = dets.raw[det_idx].y2 - dets.raw[det_idx].y1;
    float dx = dets.raw[det_idx].x1 + dw / 2.0f;
    float dy = dets.raw[det_idx].y1 + dh / 2.0f;
    
    dx = (dx - x1) / time_gap;
    dy = (dy - y1) / time_gap;
    dw = (dw - w1) / time_gap;
    dh = (dh - h1) / time_gap;

    float dz[4]; // innovation array
    
    for(int k = 0; k < time_gap; k++) {
        float x = x1 + (k + 1) * dx;
        float y = y1 + (k + 1) * dy;
        float w = w1 + (k + 1) * dw;
        float h = h1 + (k + 1) * dh;
        float s = w * h;
        float r = w / h;

        dz[0] = x - trks.x[trk_idx];
        dz[1] = y - trks.y[trk_idx];
        dz[2] = s - trks.s[trk_idx];
        dz[3] = r - trks.r[trk_idx];

        kf.update(dz, &trks, trk_idx);
        if (k < (time_gap - 1)) {
            kf.predict_soa(&trks, trk_idx);
        }

    }
}
// -- END Freezing / Unfreezing

void OCSortSoA::update_track_state(int trk_idx, int det_idx) {
    float dz[4];    // Innovation array
    
    // Compute innovation
    dz[0] = dets.x[det_idx]     - trks.x[trk_idx];
    dz[1] = dets.y[det_idx]     - trks.y[trk_idx];
    dz[2] = dets.area[det_idx]  - trks.s[trk_idx];
    dz[3] = dets.ratio[det_idx] - trks.r[trk_idx];

    // update xywr bounding box
    kf.update(dz, &trks, trk_idx);

    // update (re-calculate) xyxy bounding box
    xysr_to_xyxy_soa(&trks, trk_idx);

}


void compute_trk_velocities(
    struct Tracks *trk, unsigned ti,
    struct DetectionSoA *dets, unsigned di)
{
    float dx = dets->x[di] - trk->centerxy[ti][0];
    float dy = dets->y[di] - trk->centerxy[ti][1];
    float norm = sqrtf(dx * dx + dy * dy);

    trk->vx[ti] = dx / norm;
    trk->vy[ti] = dy / norm;
}

void OCSortSoA::update_track_observations(int trk_idx, int det_idx) {
    int age_index;
    
    // NOTE: observations is age-based.
    age_index = trks.age[trk_idx] % cfg.delta_t;

    memcpy(&trks.observations[trk_idx][age_index],
        dets.raw[det_idx].xyxybox,
        OBS_NET_LENGTH * sizeof(float));
    
    trks.observations[trk_idx][age_index][OBS_AGE_INDEX] = (float) trks.age[trk_idx];
    trks.latest_obs[trk_idx] = (float *) &trks.observations[trk_idx][age_index];

}


void OCSortSoA::update_track(int trk_idx, int det_idx) {
    if (det_idx < 0) {
        if ((1 == trks.time_since_update[trk_idx]) && (trks.age[trk_idx] > 1)) {
            freeze_track_state(trk_idx);
            trks.kf_observed_flag[trk_idx] = 1;
        }
        return;

    }
    
    // Check Previous Observation
    if (trks.latest_obs_available[trk_idx]) { 
        compute_trk_velocities(&trks, trk_idx, &dets, det_idx);
    }
    trks.latest_obs_available[trk_idx] = 1;

    // Check if there is something frozen
    if (trks.kf_observed_flag[trk_idx]) {
        unfreeze_track_state(trk_idx, det_idx);
        trks.kf_observed_flag[trk_idx] = 0;
    }

    // --- NOTE: ---
    // Python's version require three addtional updates but they seem unnecesary
    // 1. history_observations array: Only used in the "public" version of oc-sort, but never called. It stores the lastest bounding boxes infinetely.
    // 2. hist array: there is a hist array here that stores the arrays, but it seems to be for debugging purposes only, because it is never actually used.
    // 3. hits counter: updated but never used.
    // --- END ---
    
    update_track_state(trk_idx, det_idx);
    update_track_observations(trk_idx, det_idx);

    trks.time_since_update[trk_idx] = 0;
    trks.hit_streak[trk_idx]++;
}

void OCSortSoA::association_stage1(void) {
    unsigned i, len;
    int row, trk_idx, det_idx, matrix_idx;
    char isTransposed;

    if (0 == active_trks) {
        for (unmatched_dets_count = 0; unmatched_dets_count < dets_len; unmatched_dets_count++) {
            unmatched_dets[unmatched_dets_count] = unmatched_dets_count;
        }
        return;
    }

    // NOTE: in OC-SORT, there's a trick here to check if we need to compute
    // the hungarian filter based on the IoU threshold. This new matrix has 
    // the same shape as the cost matrix and it is build on 
    
    isTransposed = flinearsolver(matched, cost_matrix, active_trks, dets_len);
    len = isTransposed ? active_trks:dets_len;

    for(i = 1; i <= len; i++) {
        row = matched[i];

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

            matrix_idx = trk_idx * dets_len + det_idx;
            if (iou_matrix[matrix_idx] < cfg.iou_threshold) {
                unmatched_trks[unmatched_trks_count++] = trk_idx;
                unmatched_dets[unmatched_dets_count++] = det_idx;
            } else {
                update_track(trk_idx, det_idx);
            }
        }
    }
    
}

void OCSortSoA::association_stage2(void) {
    unsigned i, len;
    int row, trk_idx, det_idx, matrix_idx;
    char isTransposed;
    float iou;
 
    // NOTE: unlike the previous association, the trick is not used here, but 
    // it will probably be useful too.
    
    isTransposed = flinearsolver(matched, cost_matrix, unmatched_trks_count, unmatched_dets_count);
    len = isTransposed ? unmatched_trks_count : unmatched_dets_count;

    for (i = 1; i <= len; i++) {
        row = matched[i];

        if (row > 0) {
            if (isTransposed) {
                trk_idx = i - 1;
                det_idx = row - 1;
            } else {
                trk_idx = row - 1;
                det_idx = i - 1;
            }
            matrix_idx = trk_idx * unmatched_dets_count + det_idx;
            iou = 1.0f - cost_matrix[matrix_idx];
            if (iou >= cfg.iou_threshold) { 
                update_track(unmatched_trks[trk_idx], unmatched_dets[det_idx]); 
                unmatched_trks[trk_idx] = -1;
                unmatched_dets[det_idx] = -1;
            }
        }

    }

    trk_idx = 0;
    for (i = 0; i < unmatched_trks_count; i++) {
        if(unmatched_trks[i] != -1) {
            unmatched_trks[trk_idx++] = unmatched_trks[i];
            // NOTE: This can be improved by checking if the last element or elements
            // are higher than -1
            // while(unmatched_trks[--unmatched_trks_count] != -1); // This condition might be wrong
            // However, there is a risk of memory leakage if a value higher than -1 is never found.
        }
    }
    unmatched_trks_count = trk_idx;

    det_idx = 0;
    for (i = 0; i < unmatched_dets_count; i++) {
        if(unmatched_dets[i] != -1) {
            unmatched_dets[det_idx++] = unmatched_dets[i];
        }
    }

    unmatched_dets_count = det_idx;

}

void OCSortSoA::update_unmatched_tracks(void) {
    for (unsigned i = 0; i < unmatched_trks_count; i++) {
        update_track(unmatched_trks[i], -1);
    }
}

void OCSortSoA::init_tracks(void) {
    unsigned i, j, det_idx;
    
    for (i = 0; i < unmatched_dets_count; i++) {
        // 1. Initialize Track's state
        // ----------------------------
        det_idx = unmatched_dets[i];
        trks.x[active_trks] = dets.x[det_idx];
        trks.y[active_trks] = dets.y[det_idx];
        trks.s[active_trks] = dets.area[det_idx];
        trks.r[active_trks] = dets.ratio[det_idx];
        trks.dx[active_trks] = 0.0f;
        trks.dy[active_trks] = 0.0f;
        trks.ds[active_trks] = 0.0f;
        trks.x1[active_trks] = dets.raw[det_idx].x1;
        trks.x2[active_trks] = dets.raw[det_idx].x2;
        trks.y1[active_trks] = dets.raw[det_idx].y1;
        trks.y2[active_trks] = dets.raw[det_idx].y2;
 
        // 2. Initialize Track's covariance
        // ----------------------------
        /* Legacy
        memset(trks.covariance + active_trks, 0, sizeof(float) * KF_NUM_STATES * KF_NUM_STATES);    // Sets all zeros.
        for (j = 0; j < KF_NUM_STATES; j++) trks.covariance[active_trks][j][j] = 10.0f;  // Creates identity matrix.
        trks.covariance[active_trks][4][4] *= 1000.0f;                        // Speeds have
        trks.covariance[active_trks][5][5] *= 1000.0f;                        // higher 
        trks.covariance[active_trks][6][6] *= 1000.0f;                        // variance.
        */
        memset(trks.covariance + active_trks, 0, sizeof(float) * KF_NUM_COV_COMPACT);    // Sets all zeros.
        for (j = 0; j < KF_NUM_STATES; j++) trks.covariance[active_trks][j] = 10.0f;  // Creates identity matrix.
        trks.covariance[active_trks][4] *= 1000.0f;                        // Speeds have
        trks.covariance[active_trks][5] *= 1000.0f;                        // higher 
        trks.covariance[active_trks][6] *= 1000.0f;                        // variance.
 

        // 3. Initialize Track's Meta
        // ----------------------------
        trks.time_since_update[active_trks] = 0;
        trks.track_id[active_trks] = ID_manager;
        ID_manager++;

        trks.hit_streak[active_trks] = 0;
        trks.age[active_trks] = 0;

        for (j = 0; j < MAX_OBSERVATIONS; j++) {
            // trks.observations[active_trks][j][0...3] = -1.0;     // dont care.
            trks.observations[active_trks][j][OBS_AGE_INDEX] = -10.0f;
        }

        // trks.centerxy[active_trks][0] = -1.0f;               
        // trks.centerxy[active_trks][1] = -1.0f;
        trks.latest_obs[active_trks] = trks.observations[active_trks][0];
        trks.latest_obs[active_trks][0] = -1.0f;
        trks.latest_obs[active_trks][1] = -1.0f;
        trks.latest_obs[active_trks][2] = -1.0f;
        trks.latest_obs[active_trks][3] = -1.0f;

        trks.latest_obs_available[active_trks] = 0;
        // NOTE: future trks.class_id[active_trks] = 0;
        trks.vx[active_trks] = 0.0f; // NOTE/TODO: Do we need to start these
        trks.vy[active_trks] = 0.0f; // Aren't they computed when they are needed?
        trks.kf_observed_flag[active_trks] = 0;

        active_trks++;
    }

}

void OCSortSoA::reallocate_track(unsigned dest_i, unsigned src_i) {
    trks.x[dest_i] = trks.x[src_i];
    trks.y[dest_i] = trks.y[src_i];
    trks.s[dest_i] = trks.s[src_i];
    trks.r[dest_i] = trks.r[src_i];
    trks.dx[dest_i] = trks.dx[src_i];
    trks.dy[dest_i] = trks.dy[src_i];
    trks.ds[dest_i] = trks.ds[src_i];
    trks.frozen_x[dest_i] = trks.frozen_x[src_i];
    trks.frozen_y[dest_i] = trks.frozen_y[src_i];
    trks.frozen_s[dest_i] = trks.frozen_s[src_i];
    trks.frozen_ds[dest_i] = trks.frozen_ds[src_i];

    trks.x1[dest_i] = trks.x1[src_i];
    trks.x2[dest_i] = trks.x2[src_i];
    trks.y1[dest_i] = trks.y1[src_i];
    trks.y2[dest_i] = trks.y2[src_i];
    
    memcpy(trks.covariance[dest_i], 
            trks.covariance[src_i], 
            sizeof(float) * KF_NUM_COV_COMPACT);
    memcpy(trks.frozen_covariance[dest_i],
            trks.frozen_covariance[src_i],
            sizeof(float) * KF_NUM_COV_COMPACT);

    memcpy(trks.observations[dest_i],
            trks.observations[src_i],
            sizeof(float) * OBS_LENGTH * MAX_OBSERVATIONS);

    int slot = (trks.latest_obs[src_i] - (float *)trks.observations[src_i]) / OBS_LENGTH;
    trks.latest_obs[dest_i] = trks.observations[dest_i][slot];

    trks.time_since_update[dest_i] = trks.time_since_update[src_i];
    trks.track_id[dest_i]          = trks.track_id[src_i];
    trks.hit_streak[dest_i]        = trks.hit_streak[src_i];
    trks.age[dest_i]               = trks.age[src_i];

    trks.latest_obs_available[dest_i]   = trks.latest_obs_available[src_i];
    trks.vx[dest_i]                     = trks.vx[src_i];
    trks.vy[dest_i]                     = trks.vy[src_i];
    trks.kf_observed_flag[dest_i]       = trks.kf_observed_flag[src_i];
    trks.class_id[dest_i]               = trks.class_id[src_i];
}

int OCSortSoA::export_and_prune_tracks(void) {
    int output_len = 0;
    
    for (int ti = 0; ti < active_trks; ti++) {
            
        // Export valid tracks
        if ((trks.time_since_update[ti] < 1) && ((trks.hit_streak[ti] >= cfg.min_hits) || frame_count <= cfg.min_hits)) {
            bbox_out[output_len][0] = (float) trks.track_id[ti];
            if (trks.latest_obs_available[ti]) {
                memcpy(&bbox_out[output_len][1], trks.latest_obs[ti], 4 * sizeof(float));
            } else {
                bbox_out[output_len][1] = trks.x1[ti];
                bbox_out[output_len][2] = trks.y1[ti];
                bbox_out[output_len][3] = trks.x2[ti];
                bbox_out[output_len][4] = trks.y2[ti];
            }
            output_len++;
        }

        // Prune dead tracks
        if (trks.time_since_update[ti] > cfg.max_age) {
            active_trks--;
            reallocate_track(ti, active_trks);
            ti--;
        }
    }

    return output_len;
}



