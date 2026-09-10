import os
os.environ["OMP_NUM_THREADS"] = "1"
os.environ["OPENBLAS_NUM_THREADS"] = "1"
os.environ["MKL_NUM_THREADS"] = "1"
os.environ["VECLIB_MAXIMUM_THREAD"] = "1"
os.environ["NUMEXPR_NUM_THREADS"] = "1"

from trackers.newOC_SORT.trackers.ocsort_tracker.vocsort import OCSort
from utils import load_detector, store_results

from sequenceloader import SequenceLoader

import cv2
import time
import torch
import argparse
import numpy as np

# Tracker Parameters
class Args(object):
    det_thresh = 0.5
    max_age = 30
    min_hits = 3
    iou_threshold = 0.3
    delta_t = 3
    inertia = 0.2
    def __init__(self, args):
        if hasattr(args, "det_thresh"): self.det_thresh = args.det_thresh
        if hasattr(args, "max_age"): self.max_age = args.max_age
        if hasattr(args, "min_hits"): self.min_hits = args.min_hits
        if hasattr(args, "iou_threshold"): self.iou_threshold = args.iou_threshold
        if hasattr(args, "delta_t"): self.delta_t = args.delta_t
        if hasattr(args, "inertia"): self.inertia = args.inertia

def concat_LTRBboxes_confidence_classes(detections, min_height=0):
    cls = detections.boxes.cls.unsqueeze(1)
    confs = detections.boxes.conf.unsqueeze(1)
    out = torch.cat([detections.boxes.xyxy, confs, cls], dim=1)
    out = out[(out[:, 3] - out[:, 1]) > min_height, :]
    return out.cpu()

def parse_args():

    parser = argparse.ArgumentParser(description="C-BIoU Tracker")

    # Tracker Related
    parser.add_argument("--load_detector", help="Path to detector (supports yolo only).",
                        default=None, required=True)
    parser.add_argument("--det_thresh", help="High-score threshold.",
                        default=0.5, type=float) # for massive amount of detections
    parser.add_argument("--max_age", help="Frames to keep lost tracks.",
                        default=30, type=int) # to keep large amount of tracklets and overcrowd the tracker
    parser.add_argument("--min_hits", type=int, default=3)
    parser.add_argument("--iou_threshold", type=float, default=0.3)
    parser.add_argument("--delta_t", type=int, default=3)
    parser.add_argument("--inertia", type=float, default=0.2)
    
    # Experiment Related
    parser.add_argument("--experiment_name", help="Results directory (not path)",
                        default=None, required=True)
    parser.add_argument("--data_dir", help="Dataset directory. Supported: MOT17, KITTI and WaymoV2-MOT",
                        default=None, required=True)

    parser.add_argument("--data_type", help="Format. Suported MOT17 and KITTI only.",
                        default="MOT17")

    parser.add_argument("--overwrite", help="If True, it processes the entire dataset from scratch. If False, it resumes from the cache file, if there exists a cache file.",
                        default=False)

    args = parser.parse_args()

    return args


def store_results_bin(sequence_name, results):
    filename = "/tmp/dets.bin"
    print("NOTE: Keeping in mind the sequence_name is being ignored.")
    print(f"NOTE: this is being stored in {filename}.")
    results = np.stack(results).astype('float32')
    print(type(results))
    print(results.shape)
    
    with open(filename, 'wb') as f:
        f.write(results.tobytes())

    exit(1)

def run(sequence, detector, experiment_name):
    print(f"Processing sequence {sequence['sequence_name']}")
    frame_rate = (sequence["update_ms"] / 1000) if sequence["update_ms"] else 30

    results = []
    total_frame = 0
    frame_height, frame_width = sequence["image_size"]
    total_et = 0

    file = open(experiment_name+"micro_profile.txt", "a")

    for frame_id in sequence["image_filenames"]:
        frame = cv2.imread(sequence["image_filenames"][frame_id], cv2.IMREAD_COLOR)
        start_time = time.time()

        # detections = detector(frame, verbose=False, max_det=1000, conf=0.001, iou=0.99)[0]
        detections = detector(frame, verbose=False)[0]
        detections = concat_LTRBboxes_confidence_classes(detections)
        detections = detections.cpu().numpy()

        # Get tracking results
        for t in detections:
            results.append([
                frame_id, t[0], t[1], t[2], t[3], t[4], t[5]])

        total_et += time.time() - start_time

        # Logging entire pipeline elapsed time
        total_frame += 1

    file.close()
    print(f"Time elapsed: {total_et}, FPS: {total_frame/total_et}\n")
    
    # Store results.
    store_results_bin(sequence["output_file"], results)

if "__main__" == __name__:
    args = parse_args()

    sequences = SequenceLoader(args.data_dir,
                               args.data_type,
                               args.experiment_name,
                               args.overwrite)

    detector = load_detector(args.load_detector)
    
    # Create micro-profiling file
    file = open(args.experiment_name+"micro_profile.txt", "w");
    file.write("#N et\n")
    file.close()

    seq_len = len(sequences)
    for i, seq in sequences.next_sequence():
        print(f"{i}/{seq_len}: Processing sequence {seq['sequence_name']}")
        run(seq, detector, args.experiment_name)
