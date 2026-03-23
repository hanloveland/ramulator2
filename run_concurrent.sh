#!/bin/bash
# ============================================================================
# DBX-DIMM vs AsyncDIMM — Concurrent Mode (Host + NDP) Comparison
# Fixed NDP workload: GEMV 128K
# Host workloads: SPEC CPU2006 benchmark traces
# Results saved to log_concurrent/
#
# Usage:
#   ./run_concurrent.sh                  # default: 4 jobs, 50K host requests
#   ./run_concurrent.sh -j 8             # 8 parallel jobs
#   ./run_concurrent.sh -n 100000        # limit host trace to 100K requests
#   ./run_concurrent.sh -j 8 -n 50000   # 8 jobs, 50K host requests
#   ./run_concurrent.sh --hn-interval 16 --hn-quota 4 --hn-cap 8
#                                         # override AsyncDIMM H/N Arbiter fairness params
# ============================================================================

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

# Default values
MAX_JOBS=4
HOST_MAX_INST=50000

# H/N Arbiter fairness parameters (AsyncDIMM only, -1 = use YAML default)
HN_CMD_INTERVAL=-1    # (A) REQ ticks before forced CMD switch
HN_CMD_QUOTA=-1       # (A) Minimum CMD ticks per forced switch
HN_REQ_CAP=-1         # (C) REQ consecutive service cap

# Parse command-line options
while [[ $# -gt 0 ]]; do
    case "$1" in
        -j|--jobs)
            MAX_JOBS="$2"
            shift 2
            ;;
        -n|--num-host-inst)
            HOST_MAX_INST="$2"
            shift 2
            ;;
        --hn-interval)
            HN_CMD_INTERVAL="$2"
            shift 2
            ;;
        --hn-quota)
            HN_CMD_QUOTA="$2"
            shift 2
            ;;
        --hn-cap)
            HN_REQ_CAP="$2"
            shift 2
            ;;
        *)
            # Legacy: bare number = MAX_JOBS
            if [[ "$1" =~ ^[0-9]+$ ]]; then
                MAX_JOBS="$1"
                shift
            else
                echo "Unknown option: $1"
                echo "Usage: $0 [-j jobs] [-n host_max_inst] [--hn-interval N] [--hn-quota N] [--hn-cap N]"
                exit 1
            fi
            ;;
    esac
done

# Build AsyncDIMM fairness override params
ASYNC_FAIRNESS_PARAMS=()
if [ "$HN_CMD_INTERVAL" -ge 0 ] 2>/dev/null; then
    ASYNC_FAIRNESS_PARAMS+=(-p "MemorySystem.hn_cmd_service_interval=${HN_CMD_INTERVAL}")
fi
if [ "$HN_CMD_QUOTA" -ge 0 ] 2>/dev/null; then
    ASYNC_FAIRNESS_PARAMS+=(-p "MemorySystem.hn_cmd_service_quota=${HN_CMD_QUOTA}")
fi
if [ "$HN_REQ_CAP" -ge 0 ] 2>/dev/null; then
    ASYNC_FAIRNESS_PARAMS+=(-p "MemorySystem.hn_req_continuous_cap=${HN_REQ_CAP}")
fi

BINARY="./ramulator2"
DBX_CONFIG="dbxdimm_ndp_test.yaml"
ASYNC_CONFIG="asyncdimm_config.yaml"
LOG_DIR="log_concurrent"
SPEC_DIR="../spec_trace"

# Fixed NDP workload
NDP_WORKLOAD="GEMV"
NDP_SIZE="128K"
DBX_NDP_TRACE="./trace_single_bk/pch_ndp/pch_ndp_x4_${NDP_SIZE}_${NDP_WORKLOAD}.txt"
ASYNC_NDP_TRACE="./trace_single_bk/asyncdimm_nma/asyncdimm_nma_${NDP_SIZE}_${NDP_WORKLOAD}.txt"

# Representative SPEC benchmarks (memory-intensive + moderate)
SPEC_BENCHMARKS=(
    "429.mcf"           # pointer chasing, very memory-intensive
    "462.libquantum"    # streaming, memory-intensive
    "433.milc"          # matrix ops, memory-intensive
    "470.lbm"           # Lattice-Boltzmann, memory-intensive
    "471.omnetpp"       # moderate memory
    "403.gcc"           # moderate memory
    "401.bzip2"         # moderate memory
    "483.xalancbmk"     # moderate memory
)

mkdir -p "$LOG_DIR"

if [ ! -x "$BINARY" ]; then
    echo "ERROR: $BINARY not found or not executable"
    exit 1
fi

# Verify traces exist
if [ ! -f "$DBX_NDP_TRACE" ]; then
    echo "ERROR: DBX-DIMM NDP trace not found: $DBX_NDP_TRACE"
    echo "Run 'python3 gen_trace.py' first."
    exit 1
fi
if [ ! -f "$ASYNC_NDP_TRACE" ]; then
    echo "ERROR: AsyncDIMM NDP trace not found: $ASYNC_NDP_TRACE"
    echo "Run 'python3 gen_trace.py' first."
    exit 1
fi

export LD_LIBRARY_PATH="$SCRIPT_DIR:$LD_LIBRARY_PATH"

total=$((${#SPEC_BENCHMARKS[@]} * 2))
launched=0

echo "=== DBX-DIMM vs AsyncDIMM: Concurrent Mode (Host + NDP) ==="
echo "NDP Workload:    ${NDP_WORKLOAD} ${NDP_SIZE}"
echo "Host Traces:     ${SPEC_BENCHMARKS[*]}"
echo "Host max_inst:   ${HOST_MAX_INST}"
echo "Total runs:      $total"
echo "Parallel jobs:   $MAX_JOBS"
echo ""

run_sim() {
    local label="$1"
    local config="$2"
    local logfile="$3"
    shift 3
    # Remaining args are -p overrides
    local params=("$@")

    $BINARY -f "$config" "${params[@]}" > "$logfile" 2>&1
    local rc=$?
    if [ $rc -eq 0 ]; then
        echo "[DONE] $label"
    else
        echo "[FAIL] $label (exit code $rc)"
    fi
    return $rc
}

for bench in "${SPEC_BENCHMARKS[@]}"; do
    spec_trace="${SPEC_DIR}/${bench}"
    if [ ! -f "$spec_trace" ]; then
        echo "[SKIP] $bench: trace not found ($spec_trace)"
        launched=$((launched + 2))
        continue
    fi

    # ---- DBX-DIMM Concurrent ----
    dbx_log="${LOG_DIR}/dbx_concurrent_${bench}.log"
    launched=$((launched + 1))
    echo "[$launched/$total] Launching DBX-DIMM concurrent ${bench} + ${NDP_WORKLOAD}_${NDP_SIZE} ..."
    run_sim "DBX concurrent ${bench}" "$DBX_CONFIG" "$dbx_log" \
        -p "Frontend.max_inst=${HOST_MAX_INST}" \
        -p "Frontend.core0_trace=${spec_trace}" \
        -p "Frontend.core0_is_ndp_trace=false" \
        -p "MemorySystem.trace_core_enable=true" \
        -p "MemorySystem.trace_ndp_type=true" \
        -p "MemorySystem.trace_path=${DBX_NDP_TRACE}" \
        -p "MemorySystem.trace_core_mshr_size=32" &

    while [ "$(jobs -rp | wc -l)" -ge "$MAX_JOBS" ]; do
        wait -n 2>/dev/null || true
    done

    # ---- AsyncDIMM Concurrent ----
    async_log="${LOG_DIR}/async_concurrent_${bench}.log"
    launched=$((launched + 1))
    echo "[$launched/$total] Launching AsyncDIMM concurrent ${bench} + ${NDP_WORKLOAD}_${NDP_SIZE} ..."
    run_sim "AsyncDIMM concurrent ${bench}" "$ASYNC_CONFIG" "$async_log" \
        -p "Frontend.max_inst=${HOST_MAX_INST}" \
        -p "Frontend.core0_trace=${spec_trace}" \
        -p "Frontend.core0_is_ndp_trace=false" \
        -p "Frontend.core0_repeat=1" \
        -p "MemorySystem.concurrent_mode_enable=true" \
        -p "MemorySystem.trace_core_enable=true" \
        -p "MemorySystem.trace_nma_type=true" \
        -p "MemorySystem.trace_repeat=0" \
        -p "MemorySystem.trace_path=${ASYNC_NDP_TRACE}" \
        "${ASYNC_FAIRNESS_PARAMS[@]}" &

    while [ "$(jobs -rp | wc -l)" -ge "$MAX_JOBS" ]; do
        wait -n 2>/dev/null || true
    done
done

echo ""
echo "Waiting for remaining jobs to finish ..."
wait

echo ""
echo "=== All $total concurrent simulations complete ==="
echo "Logs saved to: $LOG_DIR/"
echo ""
echo "Next: run 'python3 parse_concurrent.py' to extract metrics"
