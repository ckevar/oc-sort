#include "include/hungarian.h"

#include "aos/ocsort.h"

#include <math.h>
#include <cstdio>
#include <cstring>

struct Track OCSortAoS::trks[MAX_TRACKS];

int OCSortAoS::update(struct Detection *raw_dets, uint16_t raw_dets_len) {
    frame_count++;
    // printf("\n> F%03d | \n-------+\n", frame_count - 1);

    if(0 == raw_dets_len) {
        return 0;
    }

    dets_len = prune_low_conf_dets(cfg.det_thresh, raw_dets, raw_dets_len);

    unmatched_dets_count = 0;
    unmatched_trks_count = 0;

    dets_xyxy2xysr(dets, raw_dets, dets_len);

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

    // printf("  Active Tracks: %d\n  unmatched detections: %d\n  Unmatched Tracks %d\n  det len %d\n", active_trks, unmatched_dets_count, unmatched_trks_count, dets_len);
    return export_and_prune_tracks();
    
}

// ------+
// BEGIN | Track Prediction
// ------+

inline void get_k_previous_observation(struct Track *t, uint16_t k) {
    int16_t dt, cur_age, target_age, observation_age;
    cur_age = t->age;

    for (dt = k; dt > 0; dt--) {
        target_age = cur_age - dt;
        t->momentum_obs = t->observations[target_age % k]; // int idx = (target_age %k + k) % k;
        observation_age = (int16_t) t->momentum_obs[OBS_AGE_INDEX];
        if (observation_age == target_age) {
            return;
        }
    }

    t->momentum_obs = t->latest_obs;
}

void OCSortAoS::predict_trks(void) {
    
    for (int i = 0; i < active_trks; i++) {
        // TODO: optimize this "if" condition
        if (trks[i].s + trks[i].ds <= 0.0f) {
            trks[i].ds = 0.0f;
        }

        kf.predict(trks[i].state, trks[i].covariance);

        trks[i].age += 1;
        trks[i].time_since_update += 1;

        if (trks[i].time_since_update > 1) {
            trks[i].hit_streak = 0;
        }

        xysr2xyxy_aos(&trks[i]);

        get_k_previous_observation(&trks[i], cfg.delta_t);
    }
} 

// ----+
// END | Track Prediction
// ----+

// ------+
// BEGIN | Compute First Cost
// ------+

// float compute_inter_area(struct Detection *d, float *xyxy2) {
float compute_inter_area(float *xyxy1, float *xyxy2) {
    float xmin, ymin, xmax, ymax;
    xmin = xyxy1[0] > xyxy2[0] ? xyxy1[0] : xyxy2[0];
    ymin = xyxy1[1] > xyxy2[1] ? xyxy1[1] : xyxy2[1];
    xmax = xyxy1[2] < xyxy2[2] ? xyxy1[2] : xyxy2[2];
    ymax = xyxy1[3] < xyxy2[3] ? xyxy1[3] : xyxy2[3]; 
    
    xmax -= xmin;
    ymax -= ymin;

    if ((xmax <= 0.0f) || (ymax <= 0.0f)) return 0.0f;
    return xmax * ymax;
}

inline void compute_iou(float *iou, struct DetectionAoS *d, struct Track *t) {
    float inter_area = compute_inter_area(d->raw->xyxybox, t->xyxybox);
    *iou = inter_area / (d->area + t->s - inter_area);
}

inline float compute_iou_lastest_obs(struct DetectionAoS *d, float *latest_obs) {
    float inter_area =  compute_inter_area(d->raw->xyxybox, latest_obs);
    float t_latest_area = (latest_obs[2] - latest_obs[0]) * (latest_obs[3] - latest_obs[1]);
    float iou = inter_area / (d->area + t_latest_area - inter_area);
    return iou;
}

inline void 
compute_momentum_cost(
    float *angle_diff, 
    struct DetectionAoS *d, 
    struct Track *t) 
{
    // This functions ranges from -0.5 to 0.5

    // Intention of motion
    float delta_x = d->x - (t->momentum_obs[2] + t->momentum_obs[0]) / 2.0f;
    float delta_y = d->y - (t->momentum_obs[3] + t->momentum_obs[1]) / 2.0f;
    float norm = sqrtf(delta_x * delta_x + delta_y * delta_y) + 1e-6f;

    delta_x /= norm;
    delta_y /= norm;

    // Momentum Similarity
    *angle_diff = t->vx * delta_x + t->vy* delta_y;

    // TRUE angle_diff {
    *angle_diff = fminf(1.0f, fmaxf(-1.0f, *angle_diff)); //
    *angle_diff = acosf(*angle_diff);
    *angle_diff = 0.5f  - (*angle_diff / M_PI_F); // = asinf(*angle_diff) / M_PI_F;
    // }
    

    // angle_diff approximations
    // *angle_diff = fminf(1.0f, fmaxf(-1.0f, *angle_diff));
    // float x2 = (*angle_diff) * (*angle_diff);

    // 1. Taylor Expansion
       // *angle_diff = (*angle_diff) * (0.318310f + x2 * (0.053051f + x2 * 0.0477464f));

    // 2. Taylor expansion tails capped
       // *angle_diff = (*angle_diff) * (0.318310f + x2 * (0.053051f + x2 * 0.0477464f)) / 0.838216f;

    // 3. Taylor expansion: average fitting:
    //   *angle_diff = (*angle_diff) * (0.318310f + x2 * (0.053052f + x2 * 0.128638f));

}

void OCSortAoS::compute_first_cost(void) {
    size_t j, index;
    int i;
    float iou, angle_diff;

    if (0 == active_trks) {
        return;
    }

    for(i = 0; i < active_trks; i++) {
        for (j = 0; j < dets_len; j++) {
            //--- IoU
            compute_iou(&iou, &dets[j], &trks[i]);
            iou = (iou < cfg.iou_lower_bound) ? 0.0f : iou;
            index = (i * dets_len) + j;
            iou_matrix[index] = iou;

            // if (iou < cfg.iou_threshold) {
            //    cost_matrix[index] = -iou + 1.2f;
            // } else {
            //--- Momentum Cost
            compute_momentum_cost(&angle_diff, &dets[j], &trks[i]);
            angle_diff = angle_diff * cfg.inertia * dets[j].raw->score;

            //-- Fill the matrices
            cost_matrix[index] = -(iou + angle_diff) + 1.2f;
            iou_matrix[index] = iou;
            // }
        }
    }
}

// ----+
// END | Compute First Cost
// ----+


// ------+
// BEGIN | First Association 
// ------+


void OCSortAoS::update_trk_state(struct Track *t, struct DetectionAoS *d) {
    float dz[4];        // Innovation array

    // Compute Innovation
    dz[0] = d->x - t->x;
    dz[1] = d->y - t->y;
    dz[2] = d->area - t->s;
    dz[3] = d->ratio - t->r;
    
    // Update XYSR BBox Track State
    kf.update(t->state, t->covariance, dz);

    // Update (re-calculate) XYXY BBox
    xysr2xyxy_aos(t);

}

void speed_direction(struct Track *t, float *det_xysrbox, float *prev_box) {
    float c1x = (prev_box[0] + prev_box[2]) / 2.0;
    float c1y = (prev_box[1] + prev_box[3]) / 2.0;

    float dx = det_xysrbox[0] - c1x;
    float dy = det_xysrbox[1] - c1y;

    float norm = sqrtf(dx * dx + dy * dy);
    t->vx = dx / norm;
    t->vy = dy / norm;

}

void compute_trk_velocities(struct Track *t, float *det_xysrbox, int k) {
    
    float *previous_box, *slot;
    int dt, target_age, cur_age, obs_age;

    previous_box = t->latest_obs;
    cur_age = t->age;

    // Search if there's any previous bbox
    for (dt = k; dt > 0; dt--) {
        target_age = cur_age - dt;
        slot = t->observations[target_age % k];
        obs_age = (int) slot[OBS_AGE_INDEX];

        if(obs_age == target_age) {
            previous_box = slot;
            break;
        }
    }

    speed_direction(t, det_xysrbox, previous_box);
}

void OCSortAoS::update_trk_observations(struct Track *t, float *det_raw) {
    int age_index;

    // NOTE: Observation is age-based.
    age_index = t->age % cfg.delta_t;
    memcpy(t->observations[age_index], 
            det_raw,
            sizeof(float) * OBS_NET_LENGTH);

    t->observations[age_index][OBS_AGE_INDEX] = (float) t->age;
    t->latest_obs = (float *) &t->observations[age_index]; 
}

void OCSortAoS::freeze_state(struct Track *t) {
    memcpy(t->frozen_state, t->xysrbox, sizeof(t->frozen_state));   // Only 4 states are frozen
    t->frozen_ds = t->ds;

    // NOTE: This are maintained because they aren't predicted
    // t->frozen_r  = t->r;
    // t->frozen_dx = t->dx;
    // t->frozen_dy = t->dy;
    memcpy(t->frozen_covariance, 
           t->covariance, 
           KF_NUM_COV_COMPACT * sizeof(float));
}

void OCSortAoS::unfreeze_state(struct Track *t, struct DetectionAoS *d) {
    // 1. copy back the frozen values
    memcpy(t->xysrbox, t->frozen_state, sizeof(t->frozen_state));
    t->ds = t->frozen_ds;
    // NOTE: Remaining states (r, dx, dy) are never modified during prediction
    // because of the constant velocity model.

    memcpy(t->covariance, 
           t->frozen_covariance, 
           KF_NUM_COV_COMPACT * sizeof(float));
    
    float time_gap = (float) t->time_since_update; 
    // NOTE: in the oficial implementation, the `history obs` stores the xysr, 
    // which requires sqrt operations and divisions, instead we are saving
    // the xyxy, this only needs differences and a couple of divisions.
    // Additionally, `history_obs` was redudant, because the only observation of
    // interest is the last one, thus, `latest_obs` the same used for other
    // operations.
    
    // Box 1
    float w1 = t->latest_obs[2] - t->latest_obs[0];
    float h1 = t->latest_obs[3] - t->latest_obs[1];
    float x1 = t->latest_obs[0] + w1 / 2.0f;
    float y1 = t->latest_obs[1] + h1 / 2.0f;

    // Box 2
    float dw = d->raw->x2 - d->raw->x1;
    float dh = d->raw->y2 - d->raw->y1;
    float dx = d->raw->x1 + dw / 2.0f;
    float dy = d->raw->y1 + dh / 2.0f;
    
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

        dz[0] = x - t->x;
        dz[1] = y - t->y;
        dz[2] = s - t->s;
        dz[3] = r - t->r;

        kf.update(t->state, t->covariance, dz);
        if (k < (time_gap - 1)) {
            kf.predict(t->state, t->covariance);
        }

    }

}


void OCSortAoS::update_trk(int trk_idx, int det_idx) {
    struct Track *t = &trks[trk_idx];
    struct DetectionAoS *d;

    if (det_idx < 0) {
        if ((1 == t->time_since_update) && (t->age > 1)) {
            freeze_state(t);
            t->kf_observed_flag = 1;
        }
        return;
    }
    d = &dets[det_idx];
    
    // Check Previous Observation
    if(t->latest_obs_available) {
        compute_trk_velocities(t, d->xysrbox, cfg.delta_t);
    }
    t->latest_obs_available = 1;

    // Check if there is something frozen
    if (t->kf_observed_flag) {
        unfreeze_state(t, d);
        t->kf_observed_flag = 0;
    }

    // --- NOTE: ---
    // Python's version require three addtional updates but they seem unnecesary
    // 1. history_observations array: Only used in the "public" version of oc-sort, but never called. It stores the lastest bounding boxes infinetely.
    // 2. hist array: there is a hist array here that stores the arrays, but it seems to be for debugging purposes only, because it is never actually used.
    // 3. hits counter: updated but never used.
    // --- END ---
    
    update_trk_state(t, d);
    update_trk_observations(t, d->raw->xyxybox);

    t->time_since_update = 0;
    t->hit_streak++;
}

void OCSortAoS::first_association(void) {
    int i, len;
    int row, trk_idx, det_idx, matrix_idx;
    char isTransposed;

    if(0 == active_trks) {
        for(unmatched_dets_count = 0; unmatched_dets_count < dets_len; unmatched_dets_count++) {
            unmatched_dets[unmatched_dets_count] = unmatched_dets_count;
        }
        return;
    }

    // NOTE/TODO: in OC-SORT, there's a trick here to check if we need to compute
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
                // NOTE/TODO: we might want first extract and then perform the update
                // because loading cache lines can degrade the performance.
                update_trk(trk_idx, det_idx);
            }
        }
    }
 
}

// ----+
// END | First Association
// ----+

// ------+
// BEGIN | Compute Second Cost
// ------+



int OCSortAoS::compute_second_cost(void) {
    size_t i, j, index;
    float iou, iou_max;

    /* TODO:
    fprintf(stderr, "[WARNING@SECOND COST]: Even though a computer can solve floating points; however, when deployed on FPGAs, these values have to be integers\n");
    */
    iou_max = 0.0f;

    for (i = 0; i < unmatched_trks_count; i++) {
        for (j = 0; j < unmatched_dets_count; j++) {
            iou = compute_iou_lastest_obs(&dets[unmatched_dets[j]], trks[unmatched_trks[i]].latest_obs);
            index = (i * unmatched_dets_count) + j;
            cost_matrix[index] = -iou + 1.0f; // Biasing this is important to solve hungarian
            iou_max = fmaxf(iou_max, iou);
        }
    }

    if (iou_max > cfg.iou_threshold) 
        return 1;
    return 0;

}

// ----+
// END | Compute Second Cost
// ----+


// ------+
// BEGIN | Second Association
// ------+

void OCSortAoS::second_association(void) {
    unsigned i, len;
    int row, trk_idx, det_idx, matrix_idx;
    char isTransposed;
    float iou;

    // NOTE: Unlike the previous association, the trick of pre-solving the matrix
    // is not used here (in the official implementation) but it will probably be useful too. But given the fact that this is for real-time systems, actually saving time here ocassionally is pointless.
    

    isTransposed = flinearsolver(matched, cost_matrix, unmatched_trks_count, unmatched_dets_count);
    len = isTransposed? unmatched_trks_count : unmatched_dets_count;

    for (i = 1; i <= len; i++) {
        row = matched[i];
        if(row > 0) {
            if (isTransposed) {
                trk_idx = i - 1;
                det_idx = row - 1;
            } else {
                trk_idx = row - 1;
                det_idx = i - 1;
            }
            matrix_idx = trk_idx * unmatched_dets_count + det_idx;
            iou = 1.0f - cost_matrix[matrix_idx];
            if(iou >= cfg.iou_threshold) {
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

// ----+
// END | Second Association
// ----+

// ------+
// BEGIN | Finishers
// ------+

void OCSortAoS::update_unmatched_tracks(void) {
    for(unsigned i = 0; i < unmatched_trks_count; i++) {
        update_trk(unmatched_trks[i], -1);
    }
}

void OCSortAoS::load_track_template(void) {
    int j;
    // TODO: This can be set using memcopy or memset
    TRK_TEMPLATE.x  = 0.0f;
    TRK_TEMPLATE.y  = 0.0f;
    TRK_TEMPLATE.s  = 0.0f;
    TRK_TEMPLATE.r  = 0.0f;
    TRK_TEMPLATE.dx = 0.0f;
    TRK_TEMPLATE.dy = 0.0f;
    TRK_TEMPLATE.ds = 0.0f;
    TRK_TEMPLATE.x1 = 0.0f;
    TRK_TEMPLATE.x2 = 0.0f;
    TRK_TEMPLATE.y1 = 0.0f;
    TRK_TEMPLATE.y2 = 0.0f;

    // 2. Initialize Track's covariance
    // ----------------------------
    /* Legacy
    memset(TRK_TEMPLATE.covariance, 0, sizeof(float) * KF_NUM_STATES * KF_NUM_STATES);      // Sets all zeros.
    for (j = 0; j < KF_NUM_STATES; j++) TRK_TEMPLATE.covariance[j][j] = 10.0f;  // Creates identity matrix.
    TRK_TEMPLATE.covariance[4][4] *= 1000.0f;                       // Speeds have
    TRK_TEMPLATE.covariance[5][5] *= 1000.0f;                       // higher 
    TRK_TEMPLATE.covariance[6][6] *= 1000.0f;                       // variance.
    */
    memset(TRK_TEMPLATE.covariance, 0, sizeof(float) * KF_NUM_COV_COMPACT); // Sets all zeros.
    for (j = 0; j < KF_NUM_STATES; j++) TRK_TEMPLATE.covariance[j] = 10.0f; // Creates identity matrix.
    TRK_TEMPLATE.covariance[4] *= 1000.0f;                       // Speeds have
    TRK_TEMPLATE.covariance[5] *= 1000.0f;                       // higher 
    TRK_TEMPLATE.covariance[6] *= 1000.0f;                       // variance.
    
 
    // 3. Initialize Track's Meta
    // ----------------------------
    TRK_TEMPLATE.time_since_update = 0;
    TRK_TEMPLATE.track_id = -1;

    TRK_TEMPLATE.hit_streak = 0;
    TRK_TEMPLATE.age = 0;

    for (j = 0; j < MAX_OBSERVATIONS; j++) {
        // TRK_TEMPLATE.observations[active_trks][j][0...3] = -1.0; // dont care.
        TRK_TEMPLATE.observations[j][OBS_AGE_INDEX] = -10.0f;
    }

    // NOTE: we only care about the center `x` and `y` to compute
    // the speed, for everything else... there is mastercard hahaha
    // TRK_TEMPLATE.momentum_obs[0] = -1.0f;               
    // TRK_TEMPLATE.momentum_obs[1] = -1.0f;
    TRK_TEMPLATE.latest_obs = TRK_TEMPLATE.observations[0];
    TRK_TEMPLATE.latest_obs[0] = -1.0f;
    TRK_TEMPLATE.latest_obs[1] = -1.0f;
    TRK_TEMPLATE.latest_obs[2] = -1.0f;
    TRK_TEMPLATE.latest_obs[3] = -1.0f;

    TRK_TEMPLATE.latest_obs_available = 0;
    TRK_TEMPLATE.vx = 0.0f; // NOTE/TODO: Do we need to start these
    TRK_TEMPLATE.vy = 0.0f; // Aren't they computed when they are needed?
    TRK_TEMPLATE.kf_observed_flag = 0;
}

void OCSortAoS::create_new_tracks(void) {
    unsigned i, det_idx;
    
    for(i = 0; i < unmatched_dets_count; i++) {

        det_idx = unmatched_dets[i];

        trks[active_trks] = TRK_TEMPLATE;
    
        memcpy(trks[active_trks].xysrbox, 
                dets[det_idx].xysrbox, 
                sizeof(TRK_TEMPLATE.xysrbox));

        memcpy(trks[active_trks].xyxybox, 
                dets[det_idx].raw->xyxybox, 
                sizeof(TRK_TEMPLATE.xyxybox));

        trks[active_trks].latest_obs = trks[active_trks].observations[0];
                                                      
        // NOTE: future trks[active_trks].class_id = 0;
        trks[active_trks].track_id = ID_manager;
        ID_manager++;
        active_trks++;
    }
}

void OCSortAoS::trackcpy(unsigned dest_i, unsigned src_i) {
    trks[dest_i] = trks[src_i];
    int offset = trks[src_i].latest_obs - (float *)trks[src_i].observations;
    trks[dest_i].latest_obs = (float *)trks[dest_i].observations + offset;
}

int OCSortAoS::export_and_prune_tracks(void) {
    int output_len = 0;
    struct Track *t;
    
    for (int i = 0; i < active_trks; i++) {
        t = &trks[i];

        // Export Valid Tracks
        if ((t->time_since_update < 1) && ((t->hit_streak >= cfg.min_hits) || frame_count <= cfg.min_hits)) {
            bbox_out[output_len][0] = (float) t->track_id;
            if(t->latest_obs_available) {
                memcpy(&bbox_out[output_len][1], t->latest_obs, sizeof(float) * 4);
            } else {
                memcpy(&bbox_out[output_len][1], t->xyxybox, sizeof(float) * 4);
            }

            output_len++;
        }

        // Prune dead tracks
        if(t->time_since_update > cfg.max_age) {
            active_trks--;
            trackcpy(i, active_trks);
            i--;
        }
    }

    return output_len;
}

// ----+
// END | Finishers
// ----+


