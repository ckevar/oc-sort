# 
# It benchmarks across several features
# 1. timing on synthethic data
# 2. profiling on synthetic data with 500 agents per frame, on CPU number 2
# 3. run on detected KITTI agents, stored in /tmp/KITTI-det/00*.bin, CPU number 2
#

#EXP_NAME="01-baseline"
#COMMENT="baseline"

EXP_NAME="02-No_ArcCosine"
COMMENT="ArcCosine is replaced by a taylor series approximantion with a coefficient adjustment."

function time_me {
    local BIN_UNDER_TEST="$1"
    local exp_name="$2"
   

    for det in $(ls dets/dets_*.bin | sort -t_ -nk2,2); do
        echo -n "."
        echo -n "$det " >> "$exp_name"
        taskset -c 2 $BIN_UNDER_TEST $det >> "$exp_name"
    done 

}

function profile_me {
    local BIN_NAME="$1"
    local OUT_DIR="$2"
    local det_file="${3:-dets/dets_500.bin}"

    valgrind \
        --tool=cachegrind \
        --cache-sim=yes \
        --branch-sim=yes \
        --cachegrind-out-file="$OUT_DIR/cachegrind.$BIN_NAME.out" \
        ./$BIN_NAME.bin "$det_file"

    cg_annotate \
        --show=Dr,D1mr,Bcm \
        "$OUT_DIR/cachegrind.$BIN_NAME.out" > "$OUT_DIR/profile.$BIN_NAME.src"

    mv track-test "$OUT_DIR/tracks.$BIN_NAME"

}

function track_dataset {
    
    local L_BIN_NAME="$1"
    local L_DATASET_NAME="$2"
    local L_DATASET_DIR="$3"
    local L_REPS="${4:-10}"

    OUT_DIR="$DIR_NAME/$DATASET_NAME-$L_BIN_NAME"

    if [ -d "$OUT_DIR" ]; then
        rm "$OUT_DIR"/*
    else
        mkdir -p "$OUT_DIR"
    fi

    # recompile for WARMUP FRAMES TO BE 0
    make WARMUP_FRAMES_USR=0

    for i in $(seq 1 $L_REPS); do
        echo -n "Iteration $i..."
        for SEQ in $(ls $L_DATASET_DIR); do
            SEQ_PATH="$L_DATASET_DIR/$SEQ"
            echo -n "."
            echo -n "$SEQ " >> $OUT_DIR/timing.log
            taskset -c 2 ./$L_BIN_NAME.bin "$SEQ_PATH" >> $OUT_DIR/timing.log
            SEQ_NAME=${SEQ%.*}
            mv track-test "$OUT_DIR/$SEQ_NAME.txt"
        done
        echo ""
    done
}


DIR_NAME="benchmark/results/$EXP_NAME"
mkdir -p "$DIR_NAME"

echo "Running $EXP_NAME, RESULTS: $DIR_NAME"
echo "$COMMENT" >> "$DIR_NAME/readme.txt"

## Timing
echo "Timing on synthetic data"
TIMING_FILE="$DIR_NAME/timing.log"
MAX_TIMING_RUNS=10
if [ -f "$TIMING_FILE" ]; then
    rm "$TIMING_FILE"
fi
 
make WARMUP_FRAMES_USR=80
for i in $(seq 1 $MAX_TIMING_RUNS); do 
    echo -n "iteration $i..."
    time_me ./soa.bin "$TIMING_FILE"
    time_me ./aos.bin "$TIMING_FILE"
    echo ""
done

## Profiling
echo "Profiling on "
profile_me aos "$DIR_NAME"
profile_me soa "$DIR_NAME"

# Track Dataset
# So far, the binary detections on kitti are in /tmp
DATASET_NAME="KITTI"
track_dataset aos "$DATASET_NAME" "/tmp/$DATASET_NAME-det" $MAX_TIMING_RUNS
track_dataset soa "$DATASET_NAME" "/tmp/$DATASET_NAME-det" $MAX_TIMING_RUNS

