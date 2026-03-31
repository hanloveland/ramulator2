#!/bin/bash
# ============================================================================
# DBX-DIMM Full Experiment Runner
#
# Three experiments with consistency checks:
#   Exp1: Base Standalone  — 1 DIMM, x4 (Baseline / AsyncDIMM-N / DBX-N)
#   Exp2: Scaling DRAM     — 1 DIMM (Baseline / DBX x4 / x8 / x16)
#   Exp3: Scaling DIMMs    — x4 (1 / 2 / 4 / 8 / 16 DIMM DBX-N)
#   Exp4: Concurrent       — Host SPEC + tcore GEMV concurrent execution
#
# Usage:
#   ./run_all_experiments.sh              # run all experiments
#   ./run_all_experiments.sh -e 1         # Exp1 only
#   ./run_all_experiments.sh -e 2         # Exp2 only
#   ./run_all_experiments.sh -e 3         # Exp3 only
#   ./run_all_experiments.sh -e 4         # Exp4 only
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
TRACE_ROOT="./generated_traces_p"
LOG_ROOT="./run/log_experiments_p3"
MAX_INST=5000000000   # 5 billion — enough for all traces to complete

# Configs
CFG_BASELINE="configuration/ddr5_baseline_ncore_config.yaml"
CFG_DBX_NDP="configuration/dbxdimm_ndp_test.yaml"
CFG_ASYNCDIMM="configuration/asyncdimm_nma_test.yaml"

# Workload / size definitions
BLAS_WL=("COPY" "AXPY" "AXPBY" "AXPBYPCZ" "XMY" "DOT")
BLAS_SZ=("8K" "32K" "128K" "512K" "2M" "8M")
GEMV_SZ=("8K" "16K" "32K" "64K" "128K")

# Exp4: Concurrent mode — SPEC host + tcore GEMV
SPEC_TRACE_ROOT="../spec_trace"
SPEC_BENCHMARKS=("429.mcf" "470.lbm" "462.libquantum" "401.bzip2" "433.milc")
E4_GEMV_SZ=("128K")
E4_CONFIGS=("base" "baseT" "dbx" "dbxT" "dbxN" "asyncN")

# ---------- argument parsing ----------
while [[ $# -gt 0 ]]; do
    case "$1" in
        -j) MAX_JOBS="$2"; shift 2 ;;
        -e) RUN_EXP="$2";  shift 2 ;;
        -s) SKIP_SIM=true; shift   ;;
        -h|--help)
            echo "Usage: $0 [-j JOBS] [-e EXP_NUM] [-s]"
            echo "  -j JOBS     Parallel jobs (default: 8)"
            echo "  -e 1|2|3|4  Run specific experiment only"
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
                # Baseline (conventional DDR5) — Frontend loads trace directly
                trace="$TRACE_ROOT/exp1_base_standalone/baseline/baseline_${sz}_${wl}.txt"
                run_one "E1_base_${wl}_${sz}" "$CFG_BASELINE" "$trace" "$EXP1_LOG/base_${wl}_${sz}.log" \
                    -p "Frontend.core0_trace=$trace" \
                    -p "Frontend.max_inst=$MAX_INST" \
                    -p "MemorySystem.trace_core_enable=false" &
                job_throttle

                # DBX host-only (pCH, no NDP) — Frontend loads trace directly
                trace="$TRACE_ROOT/exp1_base_standalone/pch_non_ndp/pch_non_ndp_x4_${sz}_${wl}.txt"
                run_one "E1_dbx_${wl}_${sz}" "$CFG_DBX_NDP" "$trace" "$EXP1_LOG/dbx_${wl}_${sz}.log" \
                    -p "Frontend.core0_trace=$trace" \
                    -p "Frontend.core0_is_ndp_trace=false" \
                    -p "Frontend.max_inst=$MAX_INST" \
                    -p "MemorySystem.trace_core_enable=false" \
                    -p "MemorySystem.DRAM.org.preset=DDR5_16Gb_DBX_x4" \
                    -p "MemorySystem.DRAM.org.real_dq=4" &
                job_throttle

                # DBX-N (pCH + NDP) — Frontend loads NDP trace directly
                trace="$TRACE_ROOT/exp1_base_standalone/pch_ndp/pch_ndp_x4_${sz}_${wl}.txt"
                run_one "E1_dbxn_${wl}_${sz}" "$CFG_DBX_NDP" "$trace" "$EXP1_LOG/dbxn_${wl}_${sz}.log" \
                    -p "Frontend.core0_trace=$trace" \
                    -p "Frontend.max_inst=$MAX_INST" \
                    -p "MemorySystem.trace_core_enable=false" \
                    -p "MemorySystem.DRAM.org.preset=DDR5_16Gb_DBX_x4" \
                    -p "MemorySystem.DRAM.org.real_dq=4" &
                job_throttle

                # AsyncDIMM-N (NDP Only) — Frontend loads NMA trace, no concurrent mode
                trace="$TRACE_ROOT/exp1_base_standalone/asyncdimm_nma/asyncdimm_nma_${sz}_${wl}.txt"
                run_one "E1_async_${wl}_${sz}" "$CFG_ASYNCDIMM" "$trace" "$EXP1_LOG/async_${wl}_${sz}.log" \
                    -p "Frontend.core0_trace=$trace" \
                    -p "Frontend.max_inst=$MAX_INST" \
                    -p "MemorySystem.concurrent_mode_enable=false" \
                    -p "MemorySystem.trace_core_enable=false" &
                job_throttle
            done
        done

        # GEMV
        for sz in "${GEMV_SZ[@]}"; do
            trace="$TRACE_ROOT/exp1_base_standalone/baseline/baseline_${sz}_GEMV.txt"
            run_one "E1_base_GEMV_${sz}" "$CFG_BASELINE" "$trace" "$EXP1_LOG/base_GEMV_${sz}.log" \
                -p "Frontend.core0_trace=$trace" \
                -p "Frontend.max_inst=$MAX_INST" \
                -p "MemorySystem.trace_core_enable=false" &
            job_throttle

            trace="$TRACE_ROOT/exp1_base_standalone/pch_non_ndp/pch_non_ndp_x4_${sz}_GEMV.txt"
            run_one "E1_dbx_GEMV_${sz}" "$CFG_DBX_NDP" "$trace" "$EXP1_LOG/dbx_GEMV_${sz}.log" \
                -p "Frontend.core0_trace=$trace" \
                -p "Frontend.core0_is_ndp_trace=false" \
                -p "Frontend.max_inst=$MAX_INST" \
                -p "MemorySystem.trace_core_enable=false" \
                -p "MemorySystem.DRAM.org.preset=DDR5_16Gb_DBX_x4" \
                -p "MemorySystem.DRAM.org.real_dq=4" &
            job_throttle

            trace="$TRACE_ROOT/exp1_base_standalone/pch_ndp/pch_ndp_x4_${sz}_GEMV.txt"
            run_one "E1_dbxn_GEMV_${sz}" "$CFG_DBX_NDP" "$trace" "$EXP1_LOG/dbxn_GEMV_${sz}.log" \
                -p "Frontend.core0_trace=$trace" \
                -p "Frontend.max_inst=$MAX_INST" \
                -p "MemorySystem.trace_core_enable=false" \
                -p "MemorySystem.DRAM.org.preset=DDR5_16Gb_DBX_x4" \
                -p "MemorySystem.DRAM.org.real_dq=4" &
            job_throttle

            trace="$TRACE_ROOT/exp1_base_standalone/asyncdimm_nma/asyncdimm_nma_${sz}_GEMV.txt"
            run_one "E1_async_GEMV_${sz}" "$CFG_ASYNCDIMM" "$trace" "$EXP1_LOG/async_GEMV_${sz}.log" \
                -p "Frontend.core0_trace=$trace" \
                -p "Frontend.max_inst=$MAX_INST" \
                -p "MemorySystem.concurrent_mode_enable=false" \
                -p "MemorySystem.trace_core_enable=false" &
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
                    # DBX-N with scaling — Frontend loads NDP trace directly
                    trace="$TRACE_ROOT/exp2_scaling_dram/pch_ndp/pch_ndp_${sc}_${sz}_${wl}.txt"
                    run_one "E2_${sc}_${wl}_${sz}" "$CFG_DBX_NDP" "$trace" "$EXP2_LOG/${sc}_${wl}_${sz}.log" \
                        -p "Frontend.core0_trace=$trace" \
                        -p "Frontend.max_inst=$MAX_INST" \
                        -p "MemorySystem.trace_core_enable=false" \
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
                    -p "Frontend.max_inst=$MAX_INST" \
                    -p "MemorySystem.trace_core_enable=false" \
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
                        -p "Frontend.max_inst=$MAX_INST" \
                        -p "MemorySystem.trace_core_enable=false" \
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
                    -p "Frontend.max_inst=$MAX_INST" \
                    -p "MemorySystem.trace_core_enable=false" \
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

    # ----------------------------------------------------------------
    # Exp4: Concurrent — Host SPEC + tcore GEMV
    # Configs: base, base-T, dbx, dbx-T, dbx-N, async-N
    # ----------------------------------------------------------------
    if [ -z "$RUN_EXP" ] || [ "$RUN_EXP" = "4" ]; then
        EXP4_LOG="$LOG_ROOT/exp4"
        mkdir -p "$EXP4_LOG"
        echo ""
        echo "========================================"
        echo " Exp4: Concurrent (SPEC Host + GEMV)"
        echo "========================================"

        for bench in "${SPEC_BENCHMARKS[@]}"; do
            spec_trace="${SPEC_TRACE_ROOT}/${bench}"
            if [ ! -f "$spec_trace" ]; then
                echo "[SKIP] SPEC trace not found: $spec_trace"
                continue
            fi
            btag="${bench//./}"   # 429mcf

            for gsz in "${E4_GEMV_SZ[@]}"; do

                # --- base: Host SPEC only (baseline DDR5, no tcore) ---
                run_one "E4_base_${btag}_G${gsz}" "$CFG_BASELINE" "$spec_trace" \
                    "$EXP4_LOG/base_${btag}_G${gsz}.log" \
                    -p "Frontend.core0_trace=$spec_trace" \
                    -p "Frontend.max_inst=$MAX_INST" \
                    -p "MemorySystem.trace_core_enable=false" &
                job_throttle

                # --- base-T: Host SPEC + tcore GEMV (baseline DDR5, non-NDP LD/ST) ---
                tcore_trace="$TRACE_ROOT/exp1_base_standalone/baseline/baseline_${gsz}_GEMV.txt"
                run_one "E4_baseT_${btag}_G${gsz}" "$CFG_BASELINE" "$spec_trace" \
                    "$EXP4_LOG/baseT_${btag}_G${gsz}.log" \
                    -p "Frontend.core0_trace=$spec_trace" \
                    -p "Frontend.max_inst=$MAX_INST" \
                    -p "MemorySystem.trace_core_enable=true" \
                    -p "MemorySystem.trace_ndp_type=false" \
                    -p "MemorySystem.trace_path=$tcore_trace" &
                job_throttle

                # --- dbx: Host SPEC only (DBX-DIMM pCH, no tcore) ---
                run_one "E4_dbx_${btag}_G${gsz}" "$CFG_DBX_NDP" "$spec_trace" \
                    "$EXP4_LOG/dbx_${btag}_G${gsz}.log" \
                    -p "Frontend.core0_trace=$spec_trace" \
                    -p "Frontend.core0_is_ndp_trace=false" \
                    -p "Frontend.max_inst=$MAX_INST" \
                    -p "MemorySystem.trace_core_enable=false" \
                    -p "MemorySystem.DRAM.org.preset=DDR5_16Gb_DBX_x4" \
                    -p "MemorySystem.DRAM.org.real_dq=4" &
                job_throttle

                # --- dbx-T: Host SPEC + tcore GEMV non-NDP (DBX host-side LD/ST) ---
                tcore_trace="$TRACE_ROOT/exp1_base_standalone/pch_non_ndp/pch_non_ndp_x4_${gsz}_GEMV.txt"
                run_one "E4_dbxT_${btag}_G${gsz}" "$CFG_DBX_NDP" "$spec_trace" \
                    "$EXP4_LOG/dbxT_${btag}_G${gsz}.log" \
                    -p "Frontend.core0_trace=$spec_trace" \
                    -p "Frontend.core0_is_ndp_trace=false" \
                    -p "Frontend.max_inst=$MAX_INST" \
                    -p "MemorySystem.trace_core_enable=true" \
                    -p "MemorySystem.trace_ndp_type=false" \
                    -p "MemorySystem.trace_path=$tcore_trace" \
                    -p "MemorySystem.DRAM.org.preset=DDR5_16Gb_DBX_x4" \
                    -p "MemorySystem.DRAM.org.real_dq=4" &
                job_throttle

                # --- dbx-N: Host SPEC + tcore GEMV NDP (DBX NDP concurrent) ---
                tcore_trace="$TRACE_ROOT/exp1_base_standalone/pch_ndp/pch_ndp_x4_${gsz}_GEMV.txt"
                run_one "E4_dbxN_${btag}_G${gsz}" "$CFG_DBX_NDP" "$spec_trace" \
                    "$EXP4_LOG/dbxN_${btag}_G${gsz}.log" \
                    -p "Frontend.core0_trace=$spec_trace" \
                    -p "Frontend.core0_is_ndp_trace=false" \
                    -p "Frontend.max_inst=$MAX_INST" \
                    -p "MemorySystem.trace_core_enable=true" \
                    -p "MemorySystem.trace_ndp_type=true" \
                    -p "MemorySystem.trace_path=$tcore_trace" \
                    -p "MemorySystem.DRAM.org.preset=DDR5_16Gb_DBX_x4" \
                    -p "MemorySystem.DRAM.org.real_dq=4" &
                job_throttle

                # --- async-N: Host SPEC + tcore GEMV NDP (AsyncDIMM Concurrent Mode) ---
                tcore_trace="$TRACE_ROOT/exp1_base_standalone/asyncdimm_nma/asyncdimm_nma_${gsz}_GEMV.txt"
                run_one "E4_asyncN_${btag}_G${gsz}" "$CFG_ASYNCDIMM" "$spec_trace" \
                    "$EXP4_LOG/asyncN_${btag}_G${gsz}.log" \
                    -p "Frontend.core0_trace=$spec_trace" \
                    -p "Frontend.max_inst=$MAX_INST" \
                    -p "MemorySystem.trace_core_enable=true" \
                    -p "MemorySystem.trace_nma_type=true" \
                    -p "MemorySystem.trace_path=$tcore_trace" \
                    -p "MemorySystem.concurrent_mode_enable=true" &
                job_throttle

            done
        done

        echo "Exp4: waiting for jobs..."
        wait
        echo "Exp4: all simulations complete."
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
    echo "--- Exp1: Base Standalone (1D, x4) — Cycles ---"
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

    echo "--- Exp1: Base Standalone (1D, x4) — Bandwidth (GB/s) ---"
    echo ""
    printf "%-10s %6s | %12s %12s %12s %12s\n" \
        "Workload" "Size" "Baseline" "DBX" "DBX-N" "AsyncDIMM-N"
    printf "%s\n" "$(printf '%.0s-' {1..75})"

    for wl in "${BLAS_WL[@]}" "GEMV"; do
        if [ "$wl" = "GEMV" ]; then sizes=("${GEMV_SZ[@]}"); else sizes=("${BLAS_SZ[@]}"); fi
        for sz in "${sizes[@]}"; do
            bw_base=$(extract_val "$EXP1_LOG/base_${wl}_${sz}.log" 'Host<->DB/DRAM\s+:\s+\K[0-9.]+')
            bw_dbx=$(extract_val "$EXP1_LOG/dbx_${wl}_${sz}.log" 'Host<->DB\s+:\s+\K[0-9.]+')
            bw_dbxn=$(extract_val "$EXP1_LOG/dbxn_${wl}_${sz}.log" 'DB<->DRAM\s+:\s+\K[0-9.]+')
            bw_async=$(extract_val "$EXP1_LOG/async_${wl}_${sz}.log" 'Total Bandwidth\s+:\s+\K[0-9.]+')
            printf "%-10s %6s | %12s %12s %12s %12s\n" "$wl" "$sz" "$bw_base" "$bw_dbx" "$bw_dbxn" "$bw_async"
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

# ---------- Exp4 Results ----------
if [ -z "$RUN_EXP" ] || [ "$RUN_EXP" = "4" ]; then
    EXP4_LOG="$LOG_ROOT/exp4"
    echo ""
    echo "--- Exp4: Concurrent (SPEC Host + GEMV) — Simulation Cycles ---"
    echo ""

    e4_err=0
    E4_CFG_LABELS=("base" "base-T" "dbx" "dbx-T" "dbx-N" "async-N")
    E4_CFG_TAGS=("base" "baseT" "dbx" "dbxT" "dbxN" "asyncN")

    for bench in "${SPEC_BENCHMARKS[@]}"; do
        btag="${bench//./}"
        echo ""
        echo "  [$bench]"
        printf "  %-6s | %12s %12s %12s %12s %12s %12s\n" \
            "GEMV" "${E4_CFG_LABELS[@]}"
        printf "  %s\n" "$(printf '%.0s-' {1..90})"

        for gsz in "${E4_GEMV_SZ[@]}"; do
            vals=()
            for ctag in "${E4_CFG_TAGS[@]}"; do
                logf="$EXP4_LOG/${ctag}_${btag}_G${gsz}.log"
                v=$(grep -oP 'memory_system_cycles: \K\d+' "$logf" 2>/dev/null | head -1) || v=""
                [ -z "$v" ] && v="N/A" && e4_err=$((e4_err + 1))
                vals+=("$v")
            done
            printf "  %-6s | %12s %12s %12s %12s %12s %12s\n" "$gsz" "${vals[@]}"
        done
    done

    # Slowdown table
    echo ""
    echo "--- Exp4: Host Slowdown (cycles / base cycles) ---"
    echo ""
    for bench in "${SPEC_BENCHMARKS[@]}"; do
        btag="${bench//./}"
        echo ""
        echo "  [$bench]"
        printf "  %-6s | %10s %10s %10s %10s %10s\n" \
            "GEMV" "base-T" "dbx" "dbx-T" "dbx-N" "async-N"
        printf "  %s\n" "$(printf '%.0s-' {1..72})"

        for gsz in "${E4_GEMV_SZ[@]}"; do
            base_log="$EXP4_LOG/base_${btag}_G${gsz}.log"
            base_cyc=$(grep -oP 'memory_system_cycles: \K\d+' "$base_log" 2>/dev/null | head -1) || base_cyc=""

            vals=()
            for ctag in "baseT" "dbx" "dbxT" "dbxN" "asyncN"; do
                logf="$EXP4_LOG/${ctag}_${btag}_G${gsz}.log"
                cyc=$(grep -oP 'memory_system_cycles: \K\d+' "$logf" 2>/dev/null | head -1) || cyc=""
                if [ -n "$base_cyc" ] && [ -n "$cyc" ] && [ "$base_cyc" -gt 0 ] 2>/dev/null; then
                    ratio=$(python3 -c "print(f'{${cyc}/${base_cyc}:.3f}x')")
                    vals+=("$ratio")
                else
                    vals+=("N/A")
                fi
            done
            printf "  %-6s | %10s %10s %10s %10s %10s\n" "$gsz" "${vals[@]}"
        done
    done

    # Host stall detection
    echo ""
    echo "--- Exp4: Host Stall Terminations ---"
    stall_count=0
    for bench in "${SPEC_BENCHMARKS[@]}"; do
        btag="${bench//./}"
        for gsz in "${E4_GEMV_SZ[@]}"; do
            for ctag in "${E4_CFG_TAGS[@]}"; do
                logf="$EXP4_LOG/${ctag}_${btag}_G${gsz}.log"
                if grep -q "Host stalled" "$logf" 2>/dev/null; then
                    cyc=$(grep -oP 'memory_system_cycles: \K\d+' "$logf" 2>/dev/null | head -1) || cyc="?"
                    echo "  [STALL] ${ctag} ${bench} G${gsz} — terminated at cycle $cyc"
                    stall_count=$((stall_count + 1))
                fi
            done
        done
    done
    [ $stall_count -eq 0 ] && echo "  (none)"

    echo ""
    [ $e4_err -gt 0 ] && echo "Exp4: $e4_err N/A entries (check logs)" || echo "Exp4: ALL OK"
fi

echo ""
echo "========================================================================"
echo " Done. Logs: $LOG_ROOT/"
echo "========================================================================"
