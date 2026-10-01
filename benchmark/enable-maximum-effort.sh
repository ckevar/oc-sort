OPTION="$1"

function enable_max_performance {
    echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor
    echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/energy_performance_preference
    echo 100 | sudo tee         /sys/devices/system/cpu/intel_pstate/min_perf_pct

    echo "CPU2's Performance enabled"
    cat /sys/devices/system/cpu/cpu2/cpufreq/scaling_governor
    cat /sys/devices/system/cpu/cpu2/cpufreq/scaling_cur_freq
}

function restore_performance {
    echo powersave | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor
    echo balance_performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/energy_performance_preference
    echo 16 | sudo tee /sys/devices/system/cpu/intel_pstate/min_perf_pct

    echo "CPU2's Performance restored"
    cat /sys/devices/system/cpu/cpu2/cpufreq/scaling_governor
    cat /sys/devices/system/cpu/cpu2/cpufreq/scaling_cur_freq
}

echo "OPTION $OPTION"
if [[ "$OPTION" == "0" ]]; then
    echo "Restoring performance features..."
    restore_performance

else
    echo "Enabling max performance..."
    enable_max_performance
fi

