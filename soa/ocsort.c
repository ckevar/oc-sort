#include "include/hungarian.h"

#include "soa/ocsort.h"
#include "soa/track.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

// --- Begin Prediction
float *k_previous_obs(
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
            return slot;
        }
    }
    return last_obs;
}

inline void get_k_previous_observation(struct Tracks *t, int i, uint16_t k) {
    t->momentum_obs[i] =  k_previous_obs(
        t->observations[i],
        t->age[i],
        k,
        t->latest_obs[i]);

}

void xysr_to_xyxy_soa(struct Tracks *trks, int i) {
    float w, h;
    w = sqrtf(trks->s[i] * trks->r[i]);
    h = trks->s[i] / w;
    w /= 2.0f;
    h /= 2.0f;
    trks->x1[i] = trks->x[i] - w;
    trks->x2[i] = trks->x[i] + w;
    trks->y1[i] = trks->y[i] - h;
    trks->y2[i] = trks->y[i] + h;
}

void OCSortSoA::predict_trks(void) {
    
    for (int i = 0; i < active_trks; i++) {
        
        kf.predict_soa(&trks, i);
        
        trks.age[i] += 1;
        trks.time_since_update[i] += 1;
    
        if (trks.time_since_update[i] > 1) {
            // this means previously ~time_since_update~ = 0;
            trks.hit_streak[i] = 0;
        }


        xysr_to_xyxy_soa(&trks, i);
        
        /*
        if(trks.track_id[i] == 18) {
            printf("XYSR: [%f, %f, %f, %f]\n", trks.x[i], trks.y[i], trks.s[i], trks.r[i]);
        }
        */

        get_k_previous_observation(&trks, i, cfg.delta_t);
    }
}

// --- End Prediction


// --- Begin First Association Cost

float compute_inter_area(
    struct Detection *d,
    struct Tracks *t, 
    int i)
{
    float xmin, ymin, xmax, ymax;
    xmin = d->x1 > t->x1[i] ? d->x1 : t->x1[i];
    ymin = d->y1 > t->y1[i] ? d->y1 : t->y1[i];
    xmax = d->x2 < t->x2[i] ? d->x2 : t->x2[i];
    ymax = d->y2 < t->y2[i] ? d->y2 : t->y2[i]; 
    
    xmax -= xmin;
    ymax -= ymin;

    if ((xmax <= 0.0f) || (ymax <= 0.0f)) return 0.0f;
    return xmax * ymax;
}

inline void 
compute_iou(
    float *iou, 
    struct DetectionSoA *d,  int j, 
    struct Tracks *t, int i) 
{
    // ---
    // Computes IoU between detection and predicted bounding box.
    // ---
    float inter_area = compute_inter_area(&d->raw[j], t, i);
    *iou = inter_area / (d->area[j] + t->s[i] - inter_area);
}

float compute_inter_area_latest_obs(struct Detection *d, float *latest_obs) {
    float xmin, ymin, xmax, ymax;
    xmin = d->x1 > latest_obs[0] ? d->x1 : latest_obs[0];
    ymin = d->y1 > latest_obs[1] ? d->y1 : latest_obs[1];
    xmax = d->x2 < latest_obs[2] ? d->x2 : latest_obs[2];
    ymax = d->y2 < latest_obs[3] ? d->y2 : latest_obs[3];

    xmax -= xmin;
    ymax -= ymin;

    if ((xmax <= 0.0f) || (ymax <= 0.0f)) return 0.0f;

    return xmax * ymax;
}


inline float compute_iou_lastest_obs(struct DetectionSoA *d, int j, struct Tracks *t, int i) {
    // ---
    // Computes IoU between detections and lastest matched bounding box of a track.
    // NOTE/TODO: It might be the case that this function can be merged with the 
    // compute_iou function, probably not tho, but at least the inter_area, when we 
    // change this for AoS instead of SoA Tracks.
    // ---
    float inter_area = compute_inter_area_latest_obs(&d->raw[j], t->latest_obs[i]);
    float t_latest_area = (t->latest_obs[i][2] - t->latest_obs[i][0]) * (t->latest_obs[i][3] - t->latest_obs[i][1]);
    float iou = inter_area / (d->area[j] + t_latest_area - inter_area);
    return iou;
}

// NOTE: in frame 30, for some reason, the track index starts in the index 1 instead of zero
inline void 
compute_momentum_cost(
    float *angle_diff,
    struct DetectionSoA *d, int j, 
    struct Tracks *t, int i)
{
    // This functions ranges from -0.5 to 0.5

    // Intention of motion
    float delta_x = d->x[j] - (t->momentum_obs[i][2] + t->momentum_obs[i][0]) / 2.0;
    float delta_y = d->y[j] - (t->momentum_obs[i][3] + t->momentum_obs[i][1]) / 2.0;
    float norm = sqrtf(delta_x * delta_x + delta_y * delta_y) + 1e-6f;

    delta_x /= norm;
    delta_y /= norm;

    // Momentum Similarity
    *angle_diff = t->vx[i] * delta_x + t->vy[i] * delta_y;
    *angle_diff = acos(*angle_diff);
    *angle_diff = 0.5f  - (*angle_diff / M_PI);
}

// --- End First Association Cost

/******************************/
/*    OC-SORT Refactoring     */
/******************************/


// TODO: change the structure name Detection to AoS_Detection, so we standardize
//       `struct Detection` for SoA data structure.
int OCSortSoA::update(struct Detection *AoSdets, uint16_t AoSdets_len) {

    frame_count++;

    printf("F%d:\n----\n", frame_count - 1);

    if (0 == AoSdets_len) { 
        return 0;
    }
    
    dets_len = prune_low_conf_dets(cfg.det_thresh, AoSdets, AoSdets_len);

    // Re-check if there are any left detections left after triming low score ones
    if (0 == dets_len) {  
        return 0;
    }

    unmatched_dets_count = 0;
    unmatched_trks_count = 0;
    
    det_AoS2SoA(&dets, AoSdets, dets_len);
    
    predict_trks();

    compute_first_cost();

    first_association();

    if ((unmatched_trks_count > 0) && (unmatched_dets_count > 0)) {

        if (compute_second_cost()) {
            second_association();
        }
    }

    
    update_unmatched_tracks();
    create_new_tracks();
    
    /*
    printf(" Trks AFTER updated\n");
    for(int u = 0; u < active_trks; u++) {
        printf("T%d: A(%f, %f)\n", trks.track_id[u], trks.x1[u], trks.y1[u]);
    }
    */

    /*
    printf("F%d: uT %d, uD %d: Tid%d\n", frame_count, unmatched_trks_count, unmatched_dets_count, trks.track_id[unmatched_trks[0]]);
    printf(" T16@F9: A(%f, %f), D:A(%f, %f)\n" , 
            trks.x1[unmatched_trks[0]], trks.y1[unmatched_trks[0]], 
            dets.raw[unmatched_dets[0]].x1, dets.raw[unmatched_dets[0]].y1);
    */
    
    return export_and_prune_tracks();
}

void OCSortSoA::compute_first_cost(void) {
    size_t j, index;
    int i;
    float iou, angle_diff;
    
    /* TODO:
    fprintf(stderr, "[WARNING@FIRST COST]: Check resulting cost, they have to be larger than zero at all cost.\n");
    fprintf(stderr, "[WARNING@FIRST COST]: Even though a computer can solve floating points; however, when deployed on FPGAs, these values have to be integers\n");
    */

    if (0 == active_trks) {
        return;
    }

    // cost matrix is nxm = active_trks x dets_len
    for (i = 0; i < active_trks; i++) {

        // printf("%d: ", trks.track_id[i]);
        // printf("%f %f %f %f", trks.x1[i], trks.y1[i], trks.x2[i], trks.y2[i]); 
        // printf("%f %f", trks.vy[i], trks.vx[i]);
        // printf("%f %f %f %f", trks.momentum_obs[i][0], trks.momentum_obs[i][1], trks.momentum_obs[i][2], trks.momentum_obs[i][3]);
        for (j = 0; j < dets_len; j++) {

            //--- IoU 
            // printf("%f ", dets.raw[j].x1);
            compute_iou(&iou, &dets, j, &trks, i);  // Range: 0.0 to 1.0

            //--- Momentum Cost
            compute_momentum_cost(&angle_diff, &dets, j, &trks, i);     // Range: -0.5 to 0.5
            angle_diff = angle_diff * cfg.inertia * dets.raw[j].score;  // Range: -0.2 to 0.2
            // printf("%f ", iou);
            
            //--- Fill the matrices
            index = (i * dets_len) + j; 
            // Then, cost matrix's range: -0.2 to 1.2, inverted: -1.2 to 0.2
            // NOTE: The hungarian implementation can't handle negative numbers, 
            // so it is biased adding 1.3 (for safety reasons).
            // This also suggest we can quantize the cost, so it becomes a integer 
            // based matrix, which is smaller and faster to compute on constraint 
            // devices.
            cost_matrix[index] = -(iou + angle_diff) + 1.3;
            iou_matrix[index] = iou;
        }
        // printf("\n");
    }

    return;
 
}

int OCSortSoA::compute_second_cost(void) {
    size_t i, j, index;
    float iou, iou_max;

    /* TODO:
    fprintf(stderr, "[WARNING@SECOND COST]: Even though a computer can solve floating points; however, when deployed on FPGAs, these values have to be integers\n");
    */

    iou_max = 0.0f;
    
    for (i = 0; i < unmatched_trks_count; i++) {
        for(j = 0; j < unmatched_dets_count; j++) {
            iou = compute_iou_lastest_obs(&dets, unmatched_dets[j], &trks, unmatched_trks[i]);
            index = (i * unmatched_dets_count) + j;
            cost_matrix[index] = -iou + 1.0f; // biasing this is important to solve the hungarian
            if (iou > iou_max) iou_max = iou;
        }
    }   
    
    // NOTE: The second stage is only executed if the iou_max is larger than iou_max
    if (iou_max > cfg.iou_threshold) 
        return 1;
    return 0;
}

void kalman_freeze(struct Tracks *t, unsigned i) {
    t->frozen_x[i] = t->x[i];
    t->frozen_y[i] = t->y[i];
    t->frozen_s[i] = t->s[i];
    printf("  Freezing T%d: %f %f %f\n", t->track_id[i], t->x[i], t->y[i], t->s[i]);

    // NOTE: This are maintained because they aren't predicted
    // t->frozen_r[i] = t->r[i];
    // t->frozen_dx[i] = t->dx[i];
    // t->frozen_dy[i] = t->dy[i];
    // t->frozen_ds[i] = t->ds[i];
    memcpy(&t->frozen_covariance[i], &t->covariance[i], 7 * 7 * sizeof(float));
}

void kalman_unfreeze(CVKalmanFilterSoA *kf, struct Tracks *t, unsigned i, struct DetectionSoA *d, unsigned j) {
    // 1. copy back the frozen values

    printf("    BR: x %f, y %f, s %f, r %f, dx %f, dy %f, ds %f\n", t->x[i], t->y[i], t->s[i], t->r[i], t->dx[i], t->dy[i], t->ds[i]);
    t->x[i] = t->frozen_x[i];
    t->y[i] = t->frozen_y[i];
    t->s[i] = t->frozen_s[i];
    printf("    AR: x %f, y %f, s %f, r %f, dx %f, dy %f, ds %f\n", t->x[i], t->y[i], t->s[i], t->r[i], t->dx[i], t->dy[i], t->ds[i]);
    // NOTE: Remaining states (r, dx, dy, ds) are never modified during prediction
    // because of the constant velocity model.
    memcpy(&t->covariance[i], &t->frozen_covariance[i], 7 * 7 * sizeof(float));

    float time_gap = (float) t->time_since_update[i]; // minus one because, time_since_updates starts counting from prediction
    // NOTE: in the oficial implementation, the `history obs` stores the xysr, 
    // which requires sqrt operations and divisions, instead we are saving
    // the xyxy, this only needs differences and a couple of divisions
    
    // Box 1
    float w1 = t->history_obs[i][2] - t->history_obs[i][0];
    float h1 = t->history_obs[i][3] - t->history_obs[i][1];
    float x1 = t->history_obs[i][0] + w1 / 2.0f;
    float y1 = t->history_obs[i][1] + h1 / 2.0f;
    // printf("Box1: x1 %f, y1 %f, w1 = %f, h1 %f\n", x1, y1, w1, h1);

    // Box 2
    float dw = d->raw[j].x2 - d->raw[j].x1;
    float dh = d->raw[j].y2 - d->raw[j].y1;
    float dx = d->raw[j].x1 + dw / 2.0f;
    float dy = d->raw[j].y1 + dh / 2.0f;
    // printf("Box2: x2 %f, y2 %f, w2 = %f, h2 %f\n", dx, dy, dw, dh);
    
    dx = (dx - x1) / time_gap;
    dy = (dy - y1) / time_gap;
    dw = (dw - w1) / time_gap;
    dh = (dh - h1) / time_gap;
    // printf("time gap %f\n", time_gap);

    float dz[4]; // innovation array
    
    printf("unfolding kalman: dx %f, dy %f, dw = %f, dh %f, tgap %f\n", dx, dy, dw, dh, time_gap);
    for(int k = 0; k < time_gap; k++) {
        float x = x1 + (k + 1) * dx;
        float y = y1 + (k + 1) * dy;
        float w = w1 + (k + 1) * dw;
        float h = h1 + (k + 1) * dh;
        float s = w * h;
        float r = w / h;

        dz[0] = x - t->x[i];
        dz[1] = y - t->y[i];
        dz[2] = s - t->s[i];
        dz[3] = r - t->r[i];

        printf("  K=%d\n", k);
        // printf("    NB: x %f, y %f, s %f, r %f\n", x, y, s, r);
        printf("    CS: x %f, y %f, s %f, r %f, dx %f, dy %f, ds %f\n", t->x[i], t->y[i], t->s[i], t->r[i], t->dx[i], t->dy[i], t->ds[i]);
 
        kf->update(dz, t, i);
        printf("    US: %f %f %f %f %f %f %f\n", t->x[i], t->y[i], t->s[i], t->r[i], t->dx[i], t->dy[i], t->ds[i]);
        if (k < (time_gap - 1)) {
            kf->predict_soa(t, i);
            printf("    PS: %f %f %f %f %f %f %f\n", t->x[i], t->y[i], t->s[i], t->r[i], t->dx[i], t->dy[i], t->ds[i]);
        }
        printf("\n");

    }
    printf("Updated State: %f %f %f %f %f %f %f\n", t->x[i], t->y[i], t->s[i], t->r[i], t->dx[i], t->dy[i], t->ds[i]);

}

void OCSortSoA::kf_update_trk(int trk_idx, int det_idx) {
    float dz[4];    // Innovation array
    float *det_raw;
    
    if (det_idx < 0) {                          // This checks if there is a current observed state
        //if (trks.kf_observed_flag[trk_idx]) {   // This checks if there was a previous observed state
        if ((trks.time_since_update[trk_idx] == 1) && (trks.age[trk_idx] > 1)) {   // This checks if there was a previous observed state
            kalman_freeze(&trks, trk_idx);
            trks.kf_observed_flag[trk_idx] = 1;
        }
        return;
    }

    if (trks.kf_observed_flag[trk_idx]) {
        kalman_unfreeze(&kf, &trks, trk_idx, &dets, det_idx);
        trks.kf_observed_flag[trk_idx] = 0;
    }


    det_raw = (float *)&dets.raw[det_idx];

    memcpy(&trks.history_obs[trk_idx], 
            det_raw + 1, // -> Ignores the first field (frame ID)
            OBS_NET_LENGTH * sizeof(float));

    
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

void speed_direction(
    struct Tracks *trk,
    unsigned trk_idx,
    struct DetectionSoA *dets,
    unsigned det_idx,
    float *prev_box)
{
    float c1x = (prev_box[0] + prev_box[2]) / 2.0;
    float c1y = (prev_box[1] + prev_box[3]) / 2.0;

    float dx = dets->x[det_idx] - c1x;
    float dy = dets->y[det_idx] - c1y;

    float norm = sqrtf(dx * dx + dy * dy);
    trk->vx[trk_idx] = dx / norm;
    trk->vy[trk_idx] = dy / norm;
}


void compute_trk_velocities(
    struct Tracks *trks, 
    int trk_idx, 
    struct DetectionSoA *dets, 
    int det_idx,
    int k)
{
    float *previous_box, *slot;
    int dt, target_age, cur_age, obs_age;
    
    previous_box = trks->latest_obs[trk_idx];
    cur_age = trks->age[trk_idx];

    // Search if there's any previous bbox
    for (dt = k; dt > 0; dt--) {
        target_age = cur_age - dt;

        slot = trks->observations[trk_idx][target_age % k];
        obs_age = (int) slot[OBS_AGE_INDEX];
        
        if(obs_age == target_age) {
            previous_box = slot;
            break;
        }
    }

    speed_direction(trks, trk_idx, dets, det_idx, previous_box);
}

void update_trk_observations(
    struct Tracks *trks,
    int trk_idx,
    float *det_raw,
    int k)
{
    int age_index;
    
    // 1. Copy to last observation
    memcpy(&trks->latest_obs[trk_idx], 
        det_raw + 1,                        // -> Ignores the first field (frame ID)
        OBS_NET_LENGTH * sizeof(float));    // -> Only copies the bounding box and score
                       
    // 2. Log to observations: 
    // NOTE: observations is age-based.
    age_index = trks->age[trk_idx] % k;
    memcpy(&trks->observations[trk_idx][age_index],
        det_raw + 1,                        // -> Ignores the first field (frame ID)
        OBS_NET_LENGTH * sizeof(float));

    trks->observations[trk_idx][age_index][OBS_AGE_INDEX] = (float) trks->age[trk_idx];

}

void OCSortSoA::update_trk(int trk_idx, int det_idx) {
    if (det_idx < 0) {
        kf_update_trk(trk_idx, -1);
        return;
    }

    if (trks.latest_obs_available[trk_idx]) { // no previous observation
        compute_trk_velocities(&trks, trk_idx, &dets, det_idx, cfg.delta_t);
    }

    trks.latest_obs_available[trk_idx] = 1;
    update_trk_observations(
        &trks, 
        trk_idx, 
        (float *)(&dets.raw[det_idx]),
        cfg.delta_t
    ); 

    // --- NOTE: ---
    // Python's version require three addtional updates but they seem unnecesary
    // 1. history_observations array: Only used in the "public" version of oc-sort, but never called. It stores the lastest bounding boxes infinetely.
    // 2. hist array: there is a hist array here that stores the arrays, but it seems to be for debugging purposes only, because it is never actually used.
    // 3. hits counter: updated but never used.
    // --- END ---
    
    kf_update_trk(trk_idx, det_idx);
    trks.time_since_update[trk_idx] = 0;
    trks.hit_streak[trk_idx]++;
}

void OCSortSoA::first_association(void) {
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
               update_trk(trk_idx, det_idx);
            }
        }
    }
    
}

void OCSortSoA::second_association(void) {
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
                update_trk(unmatched_trks[trk_idx], unmatched_dets[det_idx]); 
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
    for (unsigned i = 0; i < unmatched_trks_count; i++)
        update_trk(unmatched_trks[i], -1);
}

void OCSortSoA::create_new_tracks(void) {
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
        memset(trks.covariance + active_trks, 0, sizeof(float) * 7 * 7);    // Sets all zeros.
        for (j = 0; j < 7; j++) trks.covariance[active_trks][j][j] = 10.0f;  // Creates identity matrix.
        trks.covariance[active_trks][4][4] *= 1000.0f;                        // Speeds have
        trks.covariance[active_trks][5][5] *= 1000.0f;                        // higher 
        trks.covariance[active_trks][6][6] *= 1000.0f;                        // variance.
                                                                            //

        // 3. Initialize Track's Meta
        // ----------------------------
        trks.time_since_update[active_trks] = 0;

        trks.track_id[active_trks] = ID_manager;
        ID_manager++;

        trks.hit_streak[active_trks] = 0;
        trks.age[active_trks] = 0;

        for (int j = 0; j < MAX_OBSERVATIONS; j++) {
            // trks.observations[active_trks][j][0...3] = -1.0;     // dont care.
            trks.observations[active_trks][j][OBS_AGE_INDEX] = -10.0f;
        }

        // NOTE: we only care about the center `x` and `y` to compute
        // the speed, for everything else... there is mastercard hahaha
        // trks.momentum_obs[active_trks][0] = -1.0f;               
        // trks.momentum_obs[active_trks][1] = -1.0f;  
        // trks.latest_obs[active_trks][0] = -1.0f;
        // trks.latest_obs[active_trks][1] = -1.0f;
        // trks.latest_obs[active_trks][2] = -1.0f;
        // trks.latest_obs[active_trks][3] = -1.0f;

        trks.latest_obs_available[active_trks] = 0;
        // NOTE: future trks.class_id[active_trks] = 0;
        trks.vx[active_trks] = 0.0f; // NOTE/TODO: Do we need to start these
        trks.vy[active_trks] = 0.0f; // Aren't they computed when they are needed?
        trks.kf_observed_flag[active_trks] = 0;

        active_trks++;
    }
}

void OCSortSoA::trackcpy(unsigned dest_i, unsigned src_i) {
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

    trks.x1[dest_i] = trks.x1[src_i];
    trks.x2[dest_i] = trks.x2[src_i];
    trks.y1[dest_i] = trks.y1[src_i];
    trks.y2[dest_i] = trks.y2[src_i];
    
    memcpy(trks.covariance[dest_i], trks.covariance[src_i], sizeof(float) * 7 * 7);

    trks.time_since_update[dest_i] = trks.time_since_update[src_i];
    trks.track_id[dest_i]          = trks.track_id[src_i];
    trks.hit_streak[dest_i]        = trks.hit_streak[src_i];
    trks.age[dest_i]               = trks.age[src_i];

    memcpy(&trks.latest_obs[dest_i], trks.latest_obs[src_i], sizeof(float) * OBS_LENGTH);

    trks.latest_obs_available[dest_i]   = trks.latest_obs_available[src_i];
    trks.vx[dest_i]                     = trks.vx[src_i];
    trks.vy[dest_i]                     = trks.vy[src_i];
    trks.kf_observed_flag[dest_i]       = trks.kf_observed_flag[src_i];
    trks.class_id[dest_i]               = trks.class_id[src_i];
    
    memcpy(&trks.history_obs[dest_i], &trks.history_obs[src_i], sizeof(float) * OBS_LENGTH);
    
}

int OCSortSoA::export_and_prune_tracks(void) {
    int output_len = 0;

    for (int i = 0; i < active_trks; i++) {
    
        // Export valid tracks
        if ((trks.time_since_update[i] < 1) && ((trks.hit_streak[i] >= cfg.min_hits) || frame_count <= cfg.min_hits)) {
            bbox_out[output_len][0] = (float) trks.track_id[i];
            if (trks.latest_obs_available[i]) {
                memcpy(&bbox_out[output_len][1], &trks.latest_obs[i], 4 * sizeof(float));
            } else {
                bbox_out[output_len][1] = trks.x1[i];
                bbox_out[output_len][2] = trks.y1[i];
                bbox_out[output_len][3] = trks.x2[i];
                bbox_out[output_len][4] = trks.y2[i];
            }
            output_len++;
        }

        // Prune dead tracks
        if (trks.time_since_update[i] > cfg.max_age) {
            // printf("RM T%d %dyo\n", trks.track_id[i], trks.time_since_update[i]);
            active_trks--;
            trackcpy(i, active_trks);
            i--;
        }
    }

    return output_len;
}



