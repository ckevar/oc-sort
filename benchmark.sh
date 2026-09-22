time_me() {
    local BIN_UNDER_TEST="$1"
    local exp_name="$2"
    
    if [ -f "$exp_name.log" ]; then
        rm "$exp_name.log"
    fi

    for det in $(ls dets/dets_*.bin | sort -t_ -nk2,2); do
        $BIN_UNDER_TEST "$det" | tee -a "$exp_name.log"
    done
}

EXP_NAME="BASELINE"

# Timing
# MAX_TIMING_RUNS=10
# for i in $(seq 1 $MAX_TIMING_RUNS); do 
#time_me ./soa.bin "soa-$EXP_NAME-$i-timing"
#time_me ./aos.bin "aos-$EXP_NAME-$i-timing"
# done

# Profiling
det_file="dets/dets_500.bin"
#valgrind --tool=cachegrind --branch-sim=yes --cache-sim=yes --cachegrind-out-file="benchmark/profiling/cachegrind.aos.$EXP_NAME" ./aos.bin "$det_file"
#valgrind --tool=cachegrind --branch-sim=yes --cache-sim=yes --cachegrind-out-file="benchmark/profiling/cachegrind.soa.$EXP_NAME" ./soa.bin "$det_file"
cg_annotate --show=Dr,D1mr,Bcm "benchmark/profiling/cachegrind.aos.$EXP_NAME" > "benchmark/profiling/aos-profile.$EXP_NAME"
cg_annotate --show=Dr,D1mr,Bcm "benchmark/profiling/cachegrind.soa.$EXP_NAME" > "benchmark/profiling/soa-profile.$EXP_NAME"


