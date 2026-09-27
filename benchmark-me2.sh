EXP_NAME="BASELINE"

function time_me {
    local BIN_UNDER_TEST="$1"
    local exp_name="$2"
    
   
    for det in $(ls dets/dets_*.bin | sort -t_ -nk2,2); do
        echo -n "."
        echo -n "$det " >> "$exp_name"
        $BIN_UNDER_TEST $det >> "$exp_name"
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

DIR_NAME="benchmark/results/$EXP_NAME"
mkdir -p "$DIR_NAME"


# Timing
TIMING_FILE="$DIR_NAME/timing.log"
MAX_TIMING_RUNS=10
if [ -f "$TIMING_FILE" ]; then
    rm "$TIMING_FILE"
fi
 
for i in $(seq 1 $MAX_TIMING_RUNS); do 
    echo -n "iteration $i..."
    time_me ./soa.bin "$TIMING_FILE"
    time_me ./aos.bin "$TIMING_FILE"
    echo ""
done

# Profiling
profile_me aos "$DIR_NAME"
profile_me soa "$DIR_NAME"

