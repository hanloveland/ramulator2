#!/bin/bash
# ============================================================================
# DBX-DIMM NDP Standalone — Batch Simulation & Consistency Check Script
#
# Usage:
#   ./run_dbx_ndp.sh              # default: 4 parallel jobs
#   ./run_dbx_ndp.sh -j 8         # 8 parallel jobs
#   ./run_dbx_ndp.sh -j 1         # sequential
#   ./run_dbx_ndp.sh -s           # skip simulation, only parse existing logs
#   ./run_dbx_ndp.sh -o log_test  # custom output directory
# ============================================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

# ---------- defaults ----------
MAX_JOBS=4
LOG_DIR="log_ndp"
SKIP_SIM=false

BINARY="./ramulator2"
CONFIG="dbxdimm_ndp_test.yaml"
TRACE_DIR="./trace_test/pch_ndp"

# BLAS workloads: 6 sizes
BLAS_WORKLOADS=("COPY" "AXPY" "AXPBY" "AXPBYPCZ" "XMY" "DOT")
BLAS_SIZES=("8K" "32K" "128K" "512K" "2M" "8M")

# GEMV workload: 5 sizes
GEMV_SIZES=("8K" "16K" "32K" "64K" "128K")

# ---------- argument parsing ----------
while [[ $# -gt 0 ]]; do
    case "$1" in
        -j) MAX_JOBS="$2"; shift 2 ;;
        -o) LOG_DIR="$2";  shift 2 ;;
        -s) SKIP_SIM=true; shift   ;;
        -h|--help)
            echo "Usage: $0 [-j JOBS] [-o LOG_DIR] [-s]"
            echo "  -j JOBS     Number of parallel jobs (default: 4)"
            echo "  -o LOG_DIR  Output log directory (default: log_ndp)"
            echo "  -s          Skip simulation, only parse existing logs"
            exit 0 ;;
        *) echo "Unknown option: $1"; exit 1 ;;
    esac
done

mkdir -p "$LOG_DIR"

# ---------- compute total runs ----------
total=$(( ${#BLAS_WORKLOADS[@]} * ${#BLAS_SIZES[@]} + ${#GEMV_SIZES[@]} ))
launched=0
failed=0
skipped=0

echo "============================================"
echo " DBX-DIMM NDP Standalone Batch Simulation"
echo "============================================"
echo "Config:        $CONFIG"
echo "Trace dir:     $TRACE_DIR"
echo "BLAS:          ${BLAS_WORKLOADS[*]}"
echo "BLAS sizes:    ${BLAS_SIZES[*]}"
echo "GEMV sizes:    ${GEMV_SIZES[*]}"
echo "Total runs:    $total"
echo "Parallel jobs: $MAX_JOBS"
echo "Log dir:       $LOG_DIR"
echo ""

# ---------- single simulation runner ----------
run_sim() {
    local wl="$1"
    local sz="$2"
    local trace="$3"
    local logfile="$4"

    if [ ! -f "$trace" ]; then
        echo "[SKIP] ${wl}_${sz}: trace not found ($trace)"
        return 2
    fi

    $BINARY -f "$CONFIG" \
        -p "Frontend.core0_trace=$trace" \
        > "$logfile" 2>&1
    local rc=$?

    if [ $rc -eq 0 ]; then
        echo "[DONE] ${wl}_${sz}"
    else
        echo "[FAIL] ${wl}_${sz} (exit code $rc)"
    fi
    return $rc
}

# ---------- run simulations ----------
if [ "$SKIP_SIM" = false ]; then

    if [ ! -x "$BINARY" ]; then
        echo "ERROR: $BINARY not found or not executable"
        exit 1
    fi

    export LD_LIBRARY_PATH="${SCRIPT_DIR}:${LD_LIBRARY_PATH:-}"

    # BLAS workloads
    for wl in "${BLAS_WORKLOADS[@]}"; do
        for sz in "${BLAS_SIZES[@]}"; do
            trace="${TRACE_DIR}/pch_ndp_x4_${sz}_${wl}.txt"
            logfile="${LOG_DIR}/dbx_${wl}_${sz}.log"
            launched=$((launched + 1))
            echo "[$launched/$total] Launching ${wl} ${sz} ..."
            run_sim "$wl" "$sz" "$trace" "$logfile" &

            while [ "$(jobs -rp | wc -l)" -ge "$MAX_JOBS" ]; do
                wait -n 2>/dev/null || true
            done
        done
    done

    # GEMV workloads
    for sz in "${GEMV_SIZES[@]}"; do
        trace="${TRACE_DIR}/pch_ndp_x4_${sz}_GEMV.txt"
        logfile="${LOG_DIR}/dbx_GEMV_${sz}.log"
        launched=$((launched + 1))
        echo "[$launched/$total] Launching GEMV ${sz} ..."
        run_sim "GEMV" "$sz" "$trace" "$logfile" &

        while [ "$(jobs -rp | wc -l)" -ge "$MAX_JOBS" ]; do
            wait -n 2>/dev/null || true
        done
    done

    echo ""
    echo "Waiting for remaining jobs ..."
    wait

    echo ""
    echo "=== All $total simulations complete ==="
    echo ""
fi

# ---------- result consistency check ----------
echo "============================================"
echo " Result Consistency Analysis"
echo "============================================"
echo ""

# Header
printf "%-12s %8s %10s | %8s %8s %8s | %8s %8s %8s | %8s %8s | %7s\n" \
    "Workload" "Size" "TotalCyc" \
    "NDP_RD_0" "NDP_WR_0" "DB_RD_0" \
    "NDP_RD_1" "NDP_WR_1" "DB_RD_1" \
    "NDP_BW" "TotalBW" \
    "DescHit"
printf "%s\n" "$(printf '%.0s-' {1..130})"

# Collect all log files
ALL_LOGS=()
for wl in "${BLAS_WORKLOADS[@]}"; do
    for sz in "${BLAS_SIZES[@]}"; do
        ALL_LOGS+=("${wl}_${sz}|${LOG_DIR}/dbx_${wl}_${sz}.log")
    done
done
for sz in "${GEMV_SIZES[@]}"; do
    ALL_LOGS+=("GEMV_${sz}|${LOG_DIR}/dbx_GEMV_${sz}.log")
done

error_count=0

for entry in "${ALL_LOGS[@]}"; do
    label="${entry%%|*}"
    logfile="${entry##*|}"
    wl="${label%%_*}"
    sz="${label#*_}"

    if [ ! -f "$logfile" ]; then
        printf "%-12s %8s  [LOG NOT FOUND]\n" "$wl" "$sz"
        error_count=$((error_count + 1))
        continue
    fi

    # Check for simulation error
    if ! grep -q "All Request Done" "$logfile" 2>/dev/null; then
        printf "%-12s %8s  [SIM NOT COMPLETED]\n" "$wl" "$sz"
        error_count=$((error_count + 1))
        continue
    fi

    # Extract total simulation cycles
    total_cyc=$(grep -oP 'Total simulation cycles: \K\d+' "$logfile" 2>/dev/null || echo "N/A")

    # Extract NDP DRAM RD/WR per channel
    ndp_rd_0=$(grep -oP 's_num_ndp_dram_rd_0: \K\d+' "$logfile" 2>/dev/null || echo "N/A")
    ndp_wr_0=$(grep -oP 's_num_ndp_dram_wr_0: \K\d+' "$logfile" 2>/dev/null || echo "N/A")
    ndp_db_rd_0=$(grep -oP 's_num_ndp_db_rd_0: \K\d+' "$logfile" 2>/dev/null || echo "N/A")

    ndp_rd_1=$(grep -oP 's_num_ndp_dram_rd_1: \K\d+' "$logfile" 2>/dev/null || echo "N/A")
    ndp_wr_1=$(grep -oP 's_num_ndp_dram_wr_1: \K\d+' "$logfile" 2>/dev/null || echo "N/A")
    ndp_db_rd_1=$(grep -oP 's_num_ndp_db_rd_1: \K\d+' "$logfile" 2>/dev/null || echo "N/A")

    # Extract bandwidth
    ndp_bw=$(grep -oP 'Total NDP Bandwidth\s+: \K[\d.]+' "$logfile" 2>/dev/null || echo "N/A")
    total_bw=$(grep -oP 'Total DB <-> DRAMs Bandwidth\s+: \K[\d.]+' "$logfile" 2>/dev/null || echo "N/A")

    # Extract descriptor cache hit rate (average across PCHs)
    hit_rates=$(grep -oP '\d+\.\d+%' "$logfile" 2>/dev/null | head -8)
    if [ -n "$hit_rates" ]; then
        # Average of all PCH hit rates
        avg_hit=$(echo "$hit_rates" | awk '{ gsub(/%/,""); sum+=$1; n++ } END { if(n>0) printf "%.1f%%", sum/n; else print "N/A" }')
    else
        avg_hit="N/A"
    fi

    printf "%-12s %8s %10s | %8s %8s %8s | %8s %8s %8s | %8s %8s | %7s" \
        "$wl" "$sz" "$total_cyc" \
        "$ndp_rd_0" "$ndp_wr_0" "$ndp_db_rd_0" \
        "$ndp_rd_1" "$ndp_wr_1" "$ndp_db_rd_1" \
        "$ndp_bw" "$total_bw" \
        "$avg_hit"

    # Consistency checks
    warnings=""

    # Check CH0 == CH1 symmetry (NDP RD/WR should be equal)
    if [ "$ndp_rd_0" != "N/A" ] && [ "$ndp_rd_1" != "N/A" ]; then
        if [ "$ndp_rd_0" != "$ndp_rd_1" ]; then
            warnings+=" [CH_RD_ASYM]"
        fi
    fi
    if [ "$ndp_wr_0" != "N/A" ] && [ "$ndp_wr_1" != "N/A" ]; then
        if [ "$ndp_wr_0" != "$ndp_wr_1" ]; then
            warnings+=" [CH_WR_ASYM]"
        fi
    fi

    # Check NDP RD > 0 (all workloads should have DRAM reads)
    if [ "$ndp_rd_0" = "0" ] 2>/dev/null; then
        warnings+=" [NO_NDP_RD]"
    fi

    # Check descriptor cache hit rate warnings
    if echo "$hit_rates" | grep -qP '^0\.00%$' 2>/dev/null; then
        warnings+=" [DESC_MISS]"
    fi

    if [ -n "$warnings" ]; then
        printf "  %s" "$warnings"
        error_count=$((error_count + 1))
    fi
    printf "\n"
done

echo ""
printf "%s\n" "$(printf '%.0s-' {1..130})"
echo ""

# ---------- DRAM RD/WR scaling check (BLAS workloads) ----------
echo "=== BLAS Workload DRAM RD/WR Scaling Check (4x per size step) ==="
echo ""
printf "%-12s | %8s → %-8s | %10s → %-10s | %6s\n" \
    "Workload" "Size1" "Size2" "RD1" "RD2" "Ratio"
printf "%s\n" "$(printf '%.0s-' {1..75})"

for wl in "${BLAS_WORKLOADS[@]}"; do
    prev_sz=""
    prev_rd=""
    for sz in "${BLAS_SIZES[@]}"; do
        logfile="${LOG_DIR}/dbx_${wl}_${sz}.log"
        if [ ! -f "$logfile" ]; then continue; fi
        cur_rd=$(grep -oP 's_num_ndp_dram_rd_0: \K\d+' "$logfile" 2>/dev/null || echo "")
        if [ -n "$prev_rd" ] && [ -n "$cur_rd" ] && [ "$prev_rd" -gt 0 ] 2>/dev/null; then
            ratio=$(awk "BEGIN { printf \"%.2f\", $cur_rd / $prev_rd }")
            flag=""
            # Check if ratio is approximately 4.0 (within 1%)
            check=$(awk "BEGIN { diff = $ratio - 4.0; if (diff < 0) diff = -diff; print (diff > 0.04) ? \"WARN\" : \"OK\" }")
            if [ "$check" = "WARN" ]; then flag=" [!]"; fi
            printf "%-12s | %8s → %-8s | %10s → %-10s | %6s%s\n" \
                "$wl" "$prev_sz" "$sz" "$prev_rd" "$cur_rd" "${ratio}x" "$flag"
        fi
        prev_sz="$sz"
        prev_rd="$cur_rd"
    done
done

echo ""

# ---------- NDP state distribution ----------
echo "=== NDP State Distribution (PCH[0] cycles) ==="
echo ""
printf "%-12s %8s | %10s %10s %10s %10s %10s\n" \
    "Workload" "Size" "RUN" "BAR" "WAIT" "FETCH_STL" "BEFORE_RUN"
printf "%s\n" "$(printf '%.0s-' {1..85})"

for entry in "${ALL_LOGS[@]}"; do
    label="${entry%%|*}"
    logfile="${entry##*|}"
    wl="${label%%_*}"
    sz="${label#*_}"

    if [ ! -f "$logfile" ]; then continue; fi

    ndp_run=$(grep '^\[NDP_RUN\]' "$logfile" 2>/dev/null | head -1 | awk -F'|' '{gsub(/[^0-9]/,"",$2); print $2}')
    ndp_bar=$(grep '^\[NDP_BAR\]' "$logfile" 2>/dev/null | head -1 | awk -F'|' '{gsub(/[^0-9]/,"",$2); print $2}')
    ndp_wait=$(grep '^\[NDP_WAIT\]' "$logfile" 2>/dev/null | head -1 | awk -F'|' '{gsub(/[^0-9]/,"",$2); print $2}')
    ndp_fetch=$(grep '^\[NDP_FETCH_STALL\]' "$logfile" 2>/dev/null | head -1 | awk -F'|' '{gsub(/[^0-9]/,"",$2); print $2}')
    ndp_before=$(grep '^\[NDP_BEFORE_RUN\]' "$logfile" 2>/dev/null | head -1 | awk -F'|' '{gsub(/[^0-9]/,"",$2); print $2}')

    printf "%-12s %8s | %10s %10s %10s %10s %10s\n" \
        "$wl" "$sz" \
        "${ndp_run:-N/A}" "${ndp_bar:-N/A}" "${ndp_wait:-N/A}" \
        "${ndp_fetch:-N/A}" "${ndp_before:-N/A}"
done

echo ""

# ---------- summary ----------
if [ $error_count -gt 0 ]; then
    echo "WARNING: $error_count consistency issue(s) detected. Review flagged entries above."
else
    echo "All consistency checks passed."
fi
echo ""
echo "Logs saved to: $LOG_DIR/"
