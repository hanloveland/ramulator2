#!/bin/bash
# ============================================================================
# DBX-DIMM Full Experiment Runner
#
# Three experiments with consistency checks:
#   Exp1: Base Standalone  — 1 DIMM, x4 (Baseline / AsyncDIMM-N / DBX-N)
#   Exp2: Scaling DRAM     — 1 DIMM (Baseline / DBX x4 / x8 / x16)
#   Exp3: Scaling DIMMs    — x4 (1 / 2 / 4 / 8 / 16 DIMM DBX-N)
#
# Usage:
#   ./run_all_experiments.sh              # run all 3 experiments
#   ./run_all_experiments.sh -e 1         # Exp1 only
#   ./run_all_experiments.sh -e 2         # Exp2 only
#   ./run_all_experiments.sh -e 3         # Exp3 only
#   ./run_all_experiments.sh -j 16        # 16 parallel jobs
#   ./run_all_experiments.sh -s           # skip simulation, parse only
# ============================================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

# ---------- defaults ----------
MAX_JOBS=8
SKIP_SIM=false
RUN_EXP=""   # empty = all

BINARY="./ramulator2"
TRACE_ROOT="./generated_traces"
LOG_ROOT="./run/log_experiments"

# Configs
CFG_BASELINE="configuration/ddr5_baseline_ncore_config.yaml"
CFG_DBX_NDP="configuration/dbxdimm_ndp_test.yaml"
CFG_ASYNCDIMM="configuration/asyncdimm_nma_test.yaml"

# Workload / size definitions
BLAS_WL=("COPY" "AXPY" "AXPBY" "AXPBYPCZ" "XMY" "DOT")
BLAS_SZ=("8K" "32K" "128K" "512K" "2M" "8M")
GEMV_SZ=("8K" "16K" "32K" "64K" "128K")

# ---------- argument parsing ----------
while [[ $# -gt 0 ]]; do
    case "$1" in
        -j) MAX_JOBS="$2"; shift 2 ;;
        -e) RUN_EXP="$2";  shift 2 ;;
        -s) SKIP_SIM=true; shift   ;;
        -h|--help)
            echo "Usage: $0 [-j JOBS] [-e EXP_NUM] [-s]"
            echo "  -j JOBS     Parallel jobs (default: 8)"
            echo "  -e 1|2|3    Run specific experiment only"
            echo "  -s          Skip simulation, parse existing logs"
            exit 0 ;;
        *) echo "Unknown option: $1"; exit 1 ;;
    esac
done

mkdir -p "$LOG_ROOT"

# ---------- helpers ----------
job_throttle() {
    while [ "$(jobs -rp | wc -l)" -ge "$MAX_JOBS" ]; do
        wait -n 2>/dev/null || true
    done
}

run_one() {
    local label="$1" config="$2" trace="$3" logfile="$4"
    shift 4
    # remaining args are -p overrides

    if [ ! -f "$trace" ]; then
        echo "[SKIP] $label: trace not found"
        return 2
    fi

    "$BINARY" -f "$config" "$@" > "$logfile" 2>&1
    local rc=$?
    if [ $rc -eq 0 ]; then echo "[DONE] $label"
    else echo "[FAIL] $label (exit $rc)"; fi
    return $rc
}

extract_val() {
    local logfile="$1" pattern="$2"
    grep -oP "$pattern" "$logfile" 2>/dev/null | head -1 || echo "N/A"
}

# ============================================================================
# Simulation Phase
# ============================================================================
if [ "$SKIP_SIM" = false ]; then
    if [ ! -x "$BINARY" ]; then echo "ERROR: $BINARY not found"; exit 1; fi
    export LD_LIBRARY_PATH="${SCRIPT_DIR}:${LD_LIBRARY_PATH:-}"

    total=0; launched=0

    # ----------------------------------------------------------------
    # Exp1: Base Standalone — 1 DIMM, x4
    # ----------------------------------------------------------------
    if [ -z "$RUN_EXP" ] || [ "$RUN_EXP" = "1" ]; then
        EXP1_LOG="$LOG_ROOT/exp1"
        mkdir -p "$EXP1_LOG"
        echo ""
        echo "========================================"
        echo " Exp1: Base Standalone (1D, x4)"
        echo "========================================"

        for wl in "${BLAS_WL[@]}"; do
            for sz in "${BLAS_SZ[@]}"; do
                # Baseline (conventional DDR5)
                trace="$TRACE_ROOT/exp1_base_standalone/baseline/baseline_${sz}_${wl}.txt"
                run_one "E1_base_${wl}_${sz}" "$CFG_BASELINE" "$trace" "$EXP1_LOG/base_${wl}_${sz}.log" \
                    -p "Frontend.core0_trace=$trace" \
                    -p "Frontend.max_inst=500000000" \
                    -p "MemorySystem.trace_core_enable=true" \
                    -p "MemorySystem.trace_ndp_type=false" \
                    -p "MemorySystem.trace_path=$trace" &
                job_throttle

                # DBX host-only (pCH, no NDP)
                trace="$TRACE_ROOT/exp1_base_standalone/pch_non_ndp/pch_non_ndp_x4_${sz}_${wl}.txt"
                run_one "E1_dbx_${wl}_${sz}" "$CFG_DBX_NDP" "$trace" "$EXP1_LOG/dbx_${wl}_${sz}.log" \
                    -p "Frontend.core0_trace=$trace" \
                    -p "Frontend.core0_is_ndp_trace=false" \
                    -p "MemorySystem.trace_core_enable=true" \
                    -p "MemorySystem.trace_ndp_type=false" \
                    -p "MemorySystem.trace_path=$trace" \
                    -p "MemorySystem.DRAM.org.preset=DDR5_16Gb_DBX_x4" \
                    -p "MemorySystem.DRAM.org.real_dq=4" &
                job_throttle

                # DBX-N (pCH + NDP)
                trace="$TRACE_ROOT/exp1_base_standalone/pch_ndp/pch_ndp_x4_${sz}_${wl}.txt"
                run_one "E1_dbxn_${wl}_${sz}" "$CFG_DBX_NDP" "$trace" "$EXP1_LOG/dbxn_${wl}_${sz}.log" \
                    -p "Frontend.core0_trace=$trace" \
                    -p "MemorySystem.DRAM.org.preset=DDR5_16Gb_DBX_x4" \
                    -p "MemorySystem.DRAM.org.real_dq=4" &
                job_throttle

                # AsyncDIMM-N
                trace="$TRACE_ROOT/exp1_base_standalone/asyncdimm_nma/asyncdimm_nma_${sz}_${wl}.txt"
                run_one "E1_async_${wl}_${sz}" "$CFG_ASYNCDIMM" "$trace" "$EXP1_LOG/async_${wl}_${sz}.log" \
                    -p "MemorySystem.trace_path=$trace" &
                job_throttle
            done
        done

        # GEMV
        for sz in "${GEMV_SZ[@]}"; do
            trace="$TRACE_ROOT/exp1_base_standalone/baseline/baseline_${sz}_GEMV.txt"
            run_one "E1_base_GEMV_${sz}" "$CFG_BASELINE" "$trace" "$EXP1_LOG/base_GEMV_${sz}.log" \
                -p "Frontend.core0_trace=$trace" \
                -p "Frontend.max_inst=500000000" \
                -p "MemorySystem.trace_core_enable=true" \
                -p "MemorySystem.trace_ndp_type=false" \
                -p "MemorySystem.trace_path=$trace" &
            job_throttle

            trace="$TRACE_ROOT/exp1_base_standalone/pch_non_ndp/pch_non_ndp_x4_${sz}_GEMV.txt"
            run_one "E1_dbx_GEMV_${sz}" "$CFG_DBX_NDP" "$trace" "$EXP1_LOG/dbx_GEMV_${sz}.log" \
                -p "Frontend.core0_trace=$trace" \
                -p "Frontend.core0_is_ndp_trace=false" \
                -p "MemorySystem.trace_core_enable=true" \
                -p "MemorySystem.trace_ndp_type=false" \
                -p "MemorySystem.trace_path=$trace" \
                -p "MemorySystem.DRAM.org.preset=DDR5_16Gb_DBX_x4" \
                -p "MemorySystem.DRAM.org.real_dq=4" &
            job_throttle

            trace="$TRACE_ROOT/exp1_base_standalone/pch_ndp/pch_ndp_x4_${sz}_GEMV.txt"
            run_one "E1_dbxn_GEMV_${sz}" "$CFG_DBX_NDP" "$trace" "$EXP1_LOG/dbxn_GEMV_${sz}.log" \
                -p "Frontend.core0_trace=$trace" \
                -p "MemorySystem.DRAM.org.preset=DDR5_16Gb_DBX_x4" \
                -p "MemorySystem.DRAM.org.real_dq=4" &
            job_throttle

            trace="$TRACE_ROOT/exp1_base_standalone/asyncdimm_nma/asyncdimm_nma_${sz}_GEMV.txt"
            run_one "E1_async_GEMV_${sz}" "$CFG_ASYNCDIMM" "$trace" "$EXP1_LOG/async_GEMV_${sz}.log" \
                -p "MemorySystem.trace_path=$trace" &
            job_throttle
        done

        echo "Exp1: waiting for jobs..."
        wait
        echo "Exp1: all simulations complete."
    fi

    # ----------------------------------------------------------------
    # Exp2: Scaling DRAM — 1 DIMM, x4 / x8 / x16
    # ----------------------------------------------------------------
    if [ -z "$RUN_EXP" ] || [ "$RUN_EXP" = "2" ]; then
        EXP2_LOG="$LOG_ROOT/exp2"
        mkdir -p "$EXP2_LOG"
        echo ""
        echo "========================================"
        echo " Exp2: Scaling DRAM (1D, x4/x8/x16)"
        echo "========================================"

        declare -A E2_DQ=( [x4]=4 [x8]=8 [x16]=16 )
        declare -A E2_PRESET=( [x4]="DDR5_16Gb_DBX_x4" [x8]="DDR5_16Gb_DBX_x8" [x16]="DDR5_16Gb_DBX_x16" )
        E2_SCALINGS=("x4" "x8" "x16")

        for wl in "${BLAS_WL[@]}"; do
            for sz in "${BLAS_SZ[@]}"; do
                for sc in "${E2_SCALINGS[@]}"; do
                    # DBX-N with scaling
                    trace="$TRACE_ROOT/exp2_scaling_dram/pch_ndp/pch_ndp_${sc}_${sz}_${wl}.txt"
                    run_one "E2_${sc}_${wl}_${sz}" "$CFG_DBX_NDP" "$trace" "$EXP2_LOG/${sc}_${wl}_${sz}.log" \
                        -p "Frontend.core0_trace=$trace" \
                        -p "MemorySystem.DRAM.org.preset=${E2_PRESET[$sc]}" \
                        -p "MemorySystem.DRAM.org.real_dq=${E2_DQ[$sc]}" &
                    job_throttle
                done
            done
        done

        for sz in "${GEMV_SZ[@]}"; do
            for sc in "${E2_SCALINGS[@]}"; do
                trace="$TRACE_ROOT/exp2_scaling_dram/pch_ndp/pch_ndp_${sc}_${sz}_GEMV.txt"
                run_one "E2_${sc}_GEMV_${sz}" "$CFG_DBX_NDP" "$trace" "$EXP2_LOG/${sc}_GEMV_${sz}.log" \
                    -p "Frontend.core0_trace=$trace" \
                    -p "MemorySystem.DRAM.org.preset=${E2_PRESET[$sc]}" \
                    -p "MemorySystem.DRAM.org.real_dq=${E2_DQ[$sc]}" &
                job_throttle
            done
        done

        echo "Exp2: waiting for jobs..."
        wait
        echo "Exp2: all simulations complete."
    fi

    # ----------------------------------------------------------------
    # Exp3: Scaling DIMMs — x4, 1/2/4/8/16 DIMM
    # ----------------------------------------------------------------
    if [ -z "$RUN_EXP" ] || [ "$RUN_EXP" = "3" ]; then
        EXP3_LOG="$LOG_ROOT/exp3"
        mkdir -p "$EXP3_LOG"
        echo ""
        echo "========================================"
        echo " Exp3: Scaling DIMMs (x4, 1-16D)"
        echo "========================================"

        DIMM_COUNTS=(1 2 4 8 16)

        for wl in "${BLAS_WL[@]}"; do
            for sz in "${BLAS_SZ[@]}"; do
                for nd in "${DIMM_COUNTS[@]}"; do
                    dtag=""; [ "$nd" -gt 1 ] && dtag="_d${nd}"
                    ch=$((nd * 2))
                    trace="$TRACE_ROOT/exp3_scaling_dimm/pch_ndp/pch_ndp_x4${dtag}_${sz}_${wl}.txt"
                    run_one "E3_d${nd}_${wl}_${sz}" "$CFG_DBX_NDP" "$trace" "$EXP3_LOG/d${nd}_${wl}_${sz}.log" \
                        -p "Frontend.core0_trace=$trace" \
                        -p "MemorySystem.DRAM.org.preset=DDR5_16Gb_DBX_x4" \
                        -p "MemorySystem.DRAM.org.real_dq=4" \
                        -p "MemorySystem.DRAM.org.channel=$ch" &
                    job_throttle
                done
            done
        done

        for sz in "${GEMV_SZ[@]}"; do
            for nd in "${DIMM_COUNTS[@]}"; do
                dtag=""; [ "$nd" -gt 1 ] && dtag="_d${nd}"
                ch=$((nd * 2))
                trace="$TRACE_ROOT/exp3_scaling_dimm/pch_ndp/pch_ndp_x4${dtag}_${sz}_GEMV.txt"
                run_one "E3_d${nd}_GEMV_${sz}" "$CFG_DBX_NDP" "$trace" "$EXP3_LOG/d${nd}_GEMV_${sz}.log" \
                    -p "Frontend.core0_trace=$trace" \
                    -p "MemorySystem.DRAM.org.preset=DDR5_16Gb_DBX_x4" \
                    -p "MemorySystem.DRAM.org.real_dq=4" \
                    -p "MemorySystem.DRAM.org.channel=$ch" &
                job_throttle
            done
        done

        echo "Exp3: waiting for jobs..."
        wait
        echo "Exp3: all simulations complete."
    fi
fi

# ============================================================================
# Result Parsing
# ============================================================================
echo ""
echo "========================================================================"
echo " Result Summary"
echo "========================================================================"

# ---------- Exp1 Results ----------
if [ -z "$RUN_EXP" ] || [ "$RUN_EXP" = "1" ]; then
    EXP1_LOG="$LOG_ROOT/exp1"
    echo ""
    echo "--- Exp1: Base Standalone (1D, x4) ---"
    echo ""
    printf "%-10s %6s | %12s %12s %12s %12s\n" \
        "Workload" "Size" "Baseline" "DBX" "DBX-N" "AsyncDIMM-N"
    printf "%s\n" "$(printf '%.0s-' {1..75})"

    e1_err=0
    for wl in "${BLAS_WL[@]}" "GEMV"; do
        if [ "$wl" = "GEMV" ]; then sizes=("${GEMV_SZ[@]}"); else sizes=("${BLAS_SZ[@]}"); fi
        for sz in "${sizes[@]}"; do
            cyc_base=$(extract_val "$EXP1_LOG/base_${wl}_${sz}.log" 'Total simulation cycles: \K\d+')
            cyc_dbx=$(extract_val "$EXP1_LOG/dbx_${wl}_${sz}.log" 'Total simulation cycles: \K\d+')
            cyc_dbxn=$(extract_val "$EXP1_LOG/dbxn_${wl}_${sz}.log" 'Total simulation cycles: \K\d+')
            cyc_async=$(extract_val "$EXP1_LOG/async_${wl}_${sz}.log" 'Total simulation cycles: \K\d+')
            printf "%-10s %6s | %12s %12s %12s %12s\n" "$wl" "$sz" "$cyc_base" "$cyc_dbx" "$cyc_dbxn" "$cyc_async"
            for v in "$cyc_base" "$cyc_dbx" "$cyc_dbxn" "$cyc_async"; do
                [ "$v" = "N/A" ] && e1_err=$((e1_err + 1))
            done
        done
    done
    echo ""
    [ $e1_err -gt 0 ] && echo "Exp1: $e1_err N/A entries (check logs)" || echo "Exp1: ALL OK"
fi

# ---------- Exp2 Results ----------
if [ -z "$RUN_EXP" ] || [ "$RUN_EXP" = "2" ]; then
    EXP2_LOG="$LOG_ROOT/exp2"
    echo ""
    echo "--- Exp2: Scaling DRAM (1D, x4/x8/x16) — Cycles ---"
    echo ""
    printf "%-10s %6s | %12s %12s %12s\n" "Workload" "Size" "x4" "x8" "x16"
    printf "%s\n" "$(printf '%.0s-' {1..58})"

    e2_err=0
    for wl in "${BLAS_WL[@]}" "GEMV"; do
        if [ "$wl" = "GEMV" ]; then sizes=("${GEMV_SZ[@]}"); else sizes=("${BLAS_SZ[@]}"); fi
        for sz in "${sizes[@]}"; do
            cyc_x4=$(extract_val "$EXP2_LOG/x4_${wl}_${sz}.log" 'Total simulation cycles: \K\d+')
            cyc_x8=$(extract_val "$EXP2_LOG/x8_${wl}_${sz}.log" 'Total simulation cycles: \K\d+')
            cyc_x16=$(extract_val "$EXP2_LOG/x16_${wl}_${sz}.log" 'Total simulation cycles: \K\d+')
            printf "%-10s %6s | %12s %12s %12s\n" "$wl" "$sz" "$cyc_x4" "$cyc_x8" "$cyc_x16"
            for v in "$cyc_x4" "$cyc_x8" "$cyc_x16"; do
                [ "$v" = "N/A" ] && e2_err=$((e2_err + 1))
            done
        done
    done

    echo ""
    echo "--- Exp2: NDP Bandwidth (GB/s) ---"
    echo ""
    printf "%-10s %6s | %12s %12s %12s\n" "Workload" "Size" "x4" "x8" "x16"
    printf "%s\n" "$(printf '%.0s-' {1..58})"
    for wl in "${BLAS_WL[@]}" "GEMV"; do
        if [ "$wl" = "GEMV" ]; then sizes=("${GEMV_SZ[@]}"); else sizes=("${BLAS_SZ[@]}"); fi
        for sz in "${sizes[@]}"; do
            bw_x4=$(grep 'DB<->DRAM NDP' "$EXP2_LOG/x4_${wl}_${sz}.log" 2>/dev/null | head -1 | awk '{print $(NF-1)}') || bw_x4="N/A"
            bw_x8=$(grep 'DB<->DRAM NDP' "$EXP2_LOG/x8_${wl}_${sz}.log" 2>/dev/null | head -1 | awk '{print $(NF-1)}') || bw_x8="N/A"
            bw_x16=$(grep 'DB<->DRAM NDP' "$EXP2_LOG/x16_${wl}_${sz}.log" 2>/dev/null | head -1 | awk '{print $(NF-1)}') || bw_x16="N/A"
            printf "%-10s %6s | %12s %12s %12s\n" "$wl" "$sz" "$bw_x4" "$bw_x8" "$bw_x16"
        done
    done
    echo ""
    [ $e2_err -gt 0 ] && echo "Exp2: $e2_err N/A entries" || echo "Exp2: ALL OK"
fi

# ---------- Exp3 Results ----------
if [ -z "$RUN_EXP" ] || [ "$RUN_EXP" = "3" ]; then
    EXP3_LOG="$LOG_ROOT/exp3"
    echo ""
    echo "--- Exp3: Scaling DIMMs (x4) — Cycles ---"
    echo ""
    printf "%-10s %6s | %12s %12s %12s %12s %12s\n" "Workload" "Size" "1D" "2D" "4D" "8D" "16D"
    printf "%s\n" "$(printf '%.0s-' {1..85})"

    e3_err=0
    for wl in "${BLAS_WL[@]}" "GEMV"; do
        if [ "$wl" = "GEMV" ]; then sizes=("${GEMV_SZ[@]}"); else sizes=("${BLAS_SZ[@]}"); fi
        for sz in "${sizes[@]}"; do
            vals=()
            for nd in 1 2 4 8 16; do
                v=$(extract_val "$EXP3_LOG/d${nd}_${wl}_${sz}.log" 'Total simulation cycles: \K\d+')
                vals+=("$v")
                [ "$v" = "N/A" ] && e3_err=$((e3_err + 1))
            done
            printf "%-10s %6s | %12s %12s %12s %12s %12s\n" "$wl" "$sz" "${vals[@]}"
        done
    done

    echo ""
    echo "--- Exp3: NDP Bandwidth (GB/s) ---"
    echo ""
    printf "%-10s %6s | %12s %12s %12s %12s %12s\n" "Workload" "Size" "1D" "2D" "4D" "8D" "16D"
    printf "%s\n" "$(printf '%.0s-' {1..85})"
    for wl in "${BLAS_WL[@]}" "GEMV"; do
        if [ "$wl" = "GEMV" ]; then sizes=("${GEMV_SZ[@]}"); else sizes=("${BLAS_SZ[@]}"); fi
        for sz in "${sizes[@]}"; do
            vals=()
            for nd in 1 2 4 8 16; do
                bw=$(grep 'DB<->DRAM NDP' "$EXP3_LOG/d${nd}_${wl}_${sz}.log" 2>/dev/null | head -1 | awk '{print $(NF-1)}') || bw="N/A"
                vals+=("$bw")
            done
            printf "%-10s %6s | %12s %12s %12s %12s %12s\n" "$wl" "$sz" "${vals[@]}"
        done
    done

    # Total RD count consistency (fixed total data)
    echo ""
    echo "--- Exp3: Total RD Count (sum across all CH, expect constant) ---"
    echo ""
    printf "%-10s %6s | %12s %12s %12s %12s %12s | %s\n" "Workload" "Size" "1D" "2D" "4D" "8D" "16D" "Status"
    printf "%s\n" "$(printf '%.0s-' {1..100})"
    for wl in "${BLAS_WL[@]}"; do
        if [ "$wl" = "GEMV" ]; then continue; fi
        for sz in "${BLAS_SZ[@]}"; do
            vals=()
            for nd in 1 2 4 8 16; do
                logf="$EXP3_LOG/d${nd}_${wl}_${sz}.log"
                ch_count=$((nd * 2))
                total_rd=0
                for ((c=0; c<ch_count; c++)); do
                    rd=$(grep -oP "s_num_ndp_dram_rd_${c}: \\K\\d+" "$logf" 2>/dev/null || echo "0")
                    total_rd=$((total_rd + rd))
                done
                vals+=("$total_rd")
            done
            # Check all values equal
            status="OK"
            for v in "${vals[@]}"; do
                if [ "$v" != "${vals[0]}" ]; then status="MISMATCH"; e3_err=$((e3_err + 1)); break; fi
            done
            printf "%-10s %6s | %12s %12s %12s %12s %12s | %s\n" "$wl" "$sz" "${vals[@]}" "$status"
        done
    done
    echo ""
    [ $e3_err -gt 0 ] && echo "Exp3: $e3_err issue(s)" || echo "Exp3: ALL OK"
fi

echo ""
echo "========================================================================"
echo " Done. Logs: $LOG_ROOT/"
echo "========================================================================"
