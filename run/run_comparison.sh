#!/bin/bash
# ============================================================================
# DBX-DIMM vs AsyncDIMM Performance Comparison — Batch Simulation Script
# Results saved to log_comp/
#
# Usage:
#   ./run_comparison.sh          # default: 4 parallel jobs
#   ./run_comparison.sh 8        # 8 parallel jobs
#   ./run_comparison.sh 1        # sequential (no parallelism)
# ============================================================================

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

MAX_JOBS=${1:-4}

BINARY="./ramulator2"
DBX_CONFIG="dbxdimm_ndp_test.yaml"
ASYNC_CONFIG="asyncdimm_config.yaml"
LOG_DIR="log_comp"

WORKLOADS=("COPY" "AXPY" "AXPBY" "AXPBYPCZ" "XMY" "DOT")
SIZES=("8K" "32K" "128K" "512K" "2M" "8M")

GEMV_SIZES=("8K" "16K" "32K" "64K" "128K")

mkdir -p "$LOG_DIR"

if [ ! -x "$BINARY" ]; then
    echo "ERROR: $BINARY not found or not executable"
    exit 1
fi

export LD_LIBRARY_PATH="$SCRIPT_DIR:$LD_LIBRARY_PATH"

total=$(( (${#WORKLOADS[@]} * ${#SIZES[@]} + ${#GEMV_SIZES[@]}) * 2 ))
launched=0
failed=0

echo "=== DBX-DIMM vs AsyncDIMM Comparison ==="
echo "Workloads:     ${WORKLOADS[*]}"
echo "Sizes:         ${SIZES[*]}"
echo "Total runs:    $total"
echo "Parallel jobs: $MAX_JOBS"
echo ""

# Run a single simulation in background
run_sim() {
    local label="$1"
    local config="$2"
    local trace="$3"
    local logfile="$4"
    local extra_param="$5"

    if [ ! -f "$trace" ]; then
        echo "[SKIP] $label: trace not found ($trace)"
        return 0
    fi

    if [ -n "$extra_param" ]; then
        $BINARY -f "$config" \
            -p "$extra_param" \
            -p "Frontend.core0_trace=$trace" \
            > "$logfile" 2>&1
    else
        $BINARY -f "$config" \
            -p "Frontend.core0_trace=$trace" \
            > "$logfile" 2>&1
    fi
    local rc=$?
    if [ $rc -eq 0 ]; then
        echo "[DONE] $label"
    else
        echo "[FAIL] $label (exit code $rc)"
    fi
    return $rc
}

# Launch all simulations with job-level parallelism
for wl in "${WORKLOADS[@]}"; do
    for sz in "${SIZES[@]}"; do

        # ---- DBX-DIMM ----
        dbx_trace="./trace_single_bk/pch_ndp/pch_ndp_x4_${sz}_${wl}.txt"
        dbx_log="${LOG_DIR}/dbx_${wl}_${sz}.log"
        launched=$((launched + 1))
        echo "[$launched/$total] Launching DBX-DIMM ${wl} ${sz} ..."
        run_sim "DBX ${wl}_${sz}" "$DBX_CONFIG" "$dbx_trace" "$dbx_log" \
            "Frontend.core0_trace=$dbx_trace" &

        # Throttle: wait if we hit MAX_JOBS
        while [ "$(jobs -rp | wc -l)" -ge "$MAX_JOBS" ]; do
            wait -n 2>/dev/null || true
        done

        # ---- AsyncDIMM ----
        async_trace="./trace_single_bk/asyncdimm_nma/asyncdimm_nma_${sz}_${wl}.txt"
        async_log="${LOG_DIR}/async_${wl}_${sz}.log"
        launched=$((launched + 1))
        echo "[$launched/$total] Launching AsyncDIMM ${wl} ${sz} ..."
        run_sim "AsyncDIMM ${wl}_${sz}" "$ASYNC_CONFIG" "$async_trace" "$async_log" "" &

        while [ "$(jobs -rp | wc -l)" -ge "$MAX_JOBS" ]; do
            wait -n 2>/dev/null || true
        done

    done
done

# ---- GEMV (uses different sizes) ----
for sz in "${GEMV_SIZES[@]}"; do

    # ---- DBX-DIMM GEMV ----
    dbx_trace="./trace_single_bk/pch_ndp/pch_ndp_x4_${sz}_GEMV.txt"
    dbx_log="${LOG_DIR}/dbx_GEMV_${sz}.log"
    launched=$((launched + 1))
    echo "[$launched/$total] Launching DBX-DIMM GEMV ${sz} ..."
    run_sim "DBX GEMV_${sz}" "$DBX_CONFIG" "$dbx_trace" "$dbx_log" \
        "Frontend.core0_trace=$dbx_trace" &

    while [ "$(jobs -rp | wc -l)" -ge "$MAX_JOBS" ]; do
        wait -n 2>/dev/null || true
    done

    # ---- AsyncDIMM GEMV ----
    async_trace="./trace_single_bk/asyncdimm_nma/asyncdimm_nma_${sz}_GEMV.txt"
    async_log="${LOG_DIR}/async_GEMV_${sz}.log"
    launched=$((launched + 1))
    echo "[$launched/$total] Launching AsyncDIMM GEMV ${sz} ..."
    run_sim "AsyncDIMM GEMV_${sz}" "$ASYNC_CONFIG" "$async_trace" "$async_log" "" &

    while [ "$(jobs -rp | wc -l)" -ge "$MAX_JOBS" ]; do
        wait -n 2>/dev/null || true
    done

done

# Wait for all remaining background jobs
echo ""
echo "Waiting for remaining jobs to finish ..."
wait

echo ""
echo "=== All $total simulations complete ==="
echo "Logs saved to: $LOG_DIR/"
echo ""
echo "Next: run 'python3 parse_results.py' to extract metrics"
