#!/bin/bash
# ============================================================================
# DBX-DIMM All Workloads — x4/x8/x16 Scaling Consistency Check
#
# Runs ALL workloads (BLAS + GEMV) across all sizes and DRAM scalings,
# then verifies consistency:
#   1) Simulation completion
#   2) CH0 == CH1 symmetry
#   3) Per-workload RD/WR ratio
#   4) Cross-size 4x scaling
#   5) Cross-scaling (x4/x8, x8/x16) RD ratio
#   6) Expected RD count (BLAS only, GEMV skip)
#   7) NDP state distribution
#
# Usage:
#   ./run_all_scaling_test.sh              # default: 4 parallel jobs
#   ./run_all_scaling_test.sh -j 8         # 8 parallel jobs
#   ./run_all_scaling_test.sh -s           # skip simulation, only parse logs
#   ./run_all_scaling_test.sh -w COPY      # single workload only
#   ./run_all_scaling_test.sh -w GEMV      # GEMV only
# ============================================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

# ---------- defaults ----------
MAX_JOBS=4
LOG_DIR="run/log_all_scaling"
SKIP_SIM=false
FILTER_WL=""

BINARY="./ramulator2"
CONFIG="configuration/dbxdimm_ndp_test.yaml"
TRACE_DIR="./trace_test/pch_ndp"

# BLAS workloads and sizes
BLAS_WORKLOADS=("COPY" "AXPY" "AXPBY" "AXPBYPCZ" "XMY" "DOT")
BLAS_SIZES=("8K" "32K" "128K" "512K" "2M" "8M")

# GEMV workload and sizes
GEMV_SIZES=("8K" "16K" "32K" "64K" "128K")

# Scaling configurations
declare -A REAL_DQ=( [x4]=4  [x8]=8  [x16]=16 )
declare -A PRESET=(  [x4]="DDR5_16Gb_DBX_x4" [x8]="DDR5_16Gb_DBX_x8" [x16]="DDR5_16Gb_DBX_x16" )
SCALINGS=("x4" "x8" "x16")

# Per-workload RD/WR multipliers (relative to base = n_bg * opsize * iteration * 4pch)
# rd_mult: how many RD streams per iteration body
# wr_mult: how many WR streams per iteration body
# DOT: wr is scalar (opsize=0 → 1 col per BG, only BG0)
declare -A WL_RD_MULT=( [COPY]=1 [AXPY]=2 [AXPBY]=2 [AXPBYPCZ]=3 [XMY]=2 [DOT]=2 )
declare -A WL_WR_MULT=( [COPY]=1 [AXPY]=1 [AXPBY]=1 [AXPBYPCZ]=1 [XMY]=1 [DOT]=0 )
# DOT wr_mult=0 means special handling (scalar write, not proportional)

# ---------- argument parsing ----------
while [[ $# -gt 0 ]]; do
    case "$1" in
        -j) MAX_JOBS="$2"; shift 2 ;;
        -s) SKIP_SIM=true; shift   ;;
        -w) FILTER_WL="$2"; shift 2 ;;
        -h|--help)
            echo "Usage: $0 [-j JOBS] [-s] [-w WORKLOAD]"
            echo "  -j JOBS      Number of parallel jobs (default: 4)"
            echo "  -s           Skip simulation, only parse existing logs"
            echo "  -w WORKLOAD  Filter to single workload (e.g., COPY, GEMV)"
            exit 0 ;;
        *) echo "Unknown option: $1"; exit 1 ;;
    esac
done

mkdir -p "$LOG_DIR"

# ---------- build job list ----------
# Each entry: "WORKLOAD|SIZE"
JOB_LIST=()

for wl in "${BLAS_WORKLOADS[@]}"; do
    if [ -n "$FILTER_WL" ] && [ "$FILTER_WL" != "$wl" ]; then continue; fi
    for sz in "${BLAS_SIZES[@]}"; do
        JOB_LIST+=("${wl}|${sz}")
    done
done

if [ -z "$FILTER_WL" ] || [ "$FILTER_WL" = "GEMV" ]; then
    for sz in "${GEMV_SIZES[@]}"; do
        JOB_LIST+=("GEMV|${sz}")
    done
fi

total=$(( ${#JOB_LIST[@]} * ${#SCALINGS[@]} ))
launched=0

echo "============================================"
echo " All Workloads x4/x8/x16 Scaling Test"
echo "============================================"
echo "Config:     $CONFIG"
echo "Trace dir:  $TRACE_DIR"
echo "BLAS:       ${BLAS_WORKLOADS[*]}"
echo "BLAS sizes: ${BLAS_SIZES[*]}"
echo "GEMV sizes: ${GEMV_SIZES[*]}"
echo "Scalings:   ${SCALINGS[*]}"
if [ -n "$FILTER_WL" ]; then
    echo "Filter:     $FILTER_WL only"
fi
echo "Total runs: $total"
echo "Parallel:   $MAX_JOBS"
echo "Log dir:    $LOG_DIR"
echo ""

# ---------- single simulation runner ----------
run_sim() {
    local wl="$1"
    local sc="$2"
    local sz="$3"
    local trace="$4"
    local logfile="$5"
    local dq="$6"
    local preset="$7"

    if [ ! -f "$trace" ]; then
        echo "[SKIP] ${wl}_${sc}_${sz}: trace not found"
        return 2
    fi

    $BINARY -f "$CONFIG" \
        -p "Frontend.core0_trace=$trace" \
        -p "MemorySystem.DRAM.org.preset=$preset" \
        -p "MemorySystem.DRAM.org.real_dq=$dq" \
        > "$logfile" 2>&1
    local rc=$?

    if [ $rc -eq 0 ]; then
        echo "[DONE] ${wl}_${sc}_${sz}"
    else
        echo "[FAIL] ${wl}_${sc}_${sz} (exit $rc)"
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

    for entry in "${JOB_LIST[@]}"; do
        wl="${entry%%|*}"
        sz="${entry##*|}"
        for sc in "${SCALINGS[@]}"; do
            trace="${TRACE_DIR}/pch_ndp_${sc}_${sz}_${wl}.txt"
            logfile="${LOG_DIR}/${wl}_${sc}_${sz}.log"
            launched=$((launched + 1))
            echo "[$launched/$total] ${wl} ${sc} ${sz} ..."
            run_sim "$wl" "$sc" "$sz" "$trace" "$logfile" "${REAL_DQ[$sc]}" "${PRESET[$sc]}" &

            while [ "$(jobs -rp | wc -l)" -ge "$MAX_JOBS" ]; do
                wait -n 2>/dev/null || true
            done
        done
    done

    echo ""
    echo "Waiting for remaining jobs ..."
    wait
    echo ""
    echo "=== All $total simulations complete ==="
    echo ""
fi

# ============================================================================
# Helper: extract metrics from a log file
# Sets: cycles, rd0, wr0, rd1, wr1, ndp_bw, host_bw, avg_hit
# ============================================================================
extract_metrics() {
    local logfile="$1"
    cycles=$(grep -oP 'Total simulation cycles: \K\d+' "$logfile" 2>/dev/null || echo "N/A")
    rd0=$(grep -oP 's_num_ndp_dram_rd_0: \K\d+' "$logfile" 2>/dev/null || echo "N/A")
    wr0=$(grep -oP 's_num_ndp_dram_wr_0: \K\d+' "$logfile" 2>/dev/null || echo "N/A")
    rd1=$(grep -oP 's_num_ndp_dram_rd_1: \K\d+' "$logfile" 2>/dev/null || echo "N/A")
    wr1=$(grep -oP 's_num_ndp_dram_wr_1: \K\d+' "$logfile" 2>/dev/null || echo "N/A")
    ndp_bw=$(grep 'DB<->DRAM NDP' "$logfile" 2>/dev/null | head -1 | awk '{print $(NF-1)}') || ndp_bw="N/A"
    host_bw=$(grep 'DB<->DRAM HOST' "$logfile" 2>/dev/null | head -1 | awk '{print $(NF-1)}') || host_bw="N/A"
    local hit_lines
    hit_lines=$(grep -A10 'Hit Rate' "$logfile" 2>/dev/null | grep -oP '\d+\.\d+%' | head -8) || hit_lines=""
    if [ -n "$hit_lines" ]; then
        avg_hit=$(echo "$hit_lines" | awk '{ gsub(/%/,""); sum+=$1; n++ } END { if(n>0) printf "%.1f%%", sum/n; else print "N/A" }')
    else
        avg_hit="N/A"
    fi
}

# ============================================================================
# Helper: compute cal_it_v2 expected base accesses per CH
# Args: scaling, size_str → sets: exp_base (= n_bg * opsize * iteration * 4pch)
# ============================================================================
calc_expected_base() {
    local sc="$1"
    local sz="$2"

    case "$sz" in
        8K)   vec=8192 ;;
        32K)  vec=32768 ;;
        128K) vec=131072 ;;
        512K) vec=524288 ;;
        2M)   vec=2097152 ;;
        8M)   vec=8388608 ;;
        *)    exp_base=-1; return ;;
    esac

    local row_sz base_op n_bg max_sz iteration opsize ratio_d

    case "$sc" in
        x4)
            row_sz=8192; base_op=128; n_bg=8
            max_sz=$((n_bg * row_sz / 2))
            if [ "$vec" -gt "$max_sz" ]; then
                iteration=$((vec / max_sz))
                opsize=$((base_op / 2))
            else
                ratio_d=$((max_sz / vec))
                iteration=1
                opsize=$((base_op / 2 / ratio_d))
            fi
            ;;
        x8)
            row_sz=8192; base_op=64; n_bg=8
            max_sz=$((n_bg * row_sz))
            if [ "$vec" -gt "$max_sz" ]; then
                iteration=$((vec / max_sz))
                opsize=$base_op
            else
                ratio_d=$((max_sz / vec))
                iteration=1
                opsize=$((base_op / ratio_d))
            fi
            ;;
        x16)
            row_sz=16384; base_op=64; n_bg=4
            max_sz=$((n_bg * row_sz))
            if [ "$vec" -gt "$max_sz" ]; then
                iteration=$((vec / max_sz))
                opsize=$base_op
            else
                ratio_d=$((max_sz / vec))
                iteration=1
                opsize=$((base_op / ratio_d))
            fi
            ;;
    esac

    exp_base=$((n_bg * opsize * iteration * 4))
}

# ============================================================================
#  1. Per-Run Results
# ============================================================================
echo "============================================"
echo " 1. Per-Run Results"
echo "============================================"
echo ""

printf "%-10s %-5s %6s | %10s | %8s %8s %8s %8s | %10s %10s | %7s\n" \
    "Workload" "DQ" "Size" "Cycles" \
    "NDP_RD0" "NDP_WR0" "NDP_RD1" "NDP_WR1" \
    "NDP_BW" "Host_BW" \
    "DescHR"
printf "%s\n" "$(printf '%.0s-' {1..120})"

declare -A RES_RD0 RES_WR0 RES_RD1 RES_WR1 RES_CYCLES
error_count=0

for entry in "${JOB_LIST[@]}"; do
    wl="${entry%%|*}"
    sz="${entry##*|}"
    for sc in "${SCALINGS[@]}"; do
        key="${wl}_${sc}_${sz}"
        logfile="${LOG_DIR}/${wl}_${sc}_${sz}.log"

        if [ ! -f "$logfile" ]; then
            printf "%-10s %-5s %6s | [LOG NOT FOUND]\n" "$wl" "$sc" "$sz"
            error_count=$((error_count + 1))
            continue
        fi

        if ! grep -q "All Request Done" "$logfile" 2>/dev/null; then
            printf "%-10s %-5s %6s | [SIM NOT COMPLETED]\n" "$wl" "$sc" "$sz"
            error_count=$((error_count + 1))
            continue
        fi

        extract_metrics "$logfile"

        RES_RD0[$key]="$rd0"
        RES_WR0[$key]="$wr0"
        RES_RD1[$key]="$rd1"
        RES_WR1[$key]="$wr1"
        RES_CYCLES[$key]="$cycles"

        warnings=""
        if [ "$rd0" != "N/A" ] && [ "$rd1" != "N/A" ] && [ "$rd0" != "$rd1" ]; then
            warnings+=" [CH_RD_ASYM]"
        fi
        if [ "$wr0" != "N/A" ] && [ "$wr1" != "N/A" ] && [ "$wr0" != "$wr1" ]; then
            warnings+=" [CH_WR_ASYM]"
        fi
        if [ "$rd0" = "0" ] 2>/dev/null; then
            warnings+=" [NO_NDP_RD]"
        fi
        if [ -n "$warnings" ]; then
            error_count=$((error_count + 1))
        fi

        printf "%-10s %-5s %6s | %10s | %8s %8s %8s %8s | %10s %10s | %7s%s\n" \
            "$wl" "$sc" "$sz" "$cycles" \
            "$rd0" "$wr0" "$rd1" "$wr1" \
            "$ndp_bw" "$host_bw" \
            "$avg_hit" "$warnings"
    done
done

# ============================================================================
#  2. RD/WR Ratio Check (per workload semantics)
# ============================================================================
echo ""
echo "============================================"
echo " 2. RD/WR Ratio Check (per workload)"
echo "============================================"
echo ""
echo "Expected: COPY=1:1, AXPY/AXPBY/XMY=2:1, AXPBYPCZ=3:1, DOT=2:scalar, GEMV=complex"
echo ""
printf "%-10s %-5s %6s | %8s %8s | %8s | %s\n" \
    "Workload" "DQ" "Size" "RD0" "WR0" "RD/WR" "Status"
printf "%s\n" "$(printf '%.0s-' {1..70})"

for entry in "${JOB_LIST[@]}"; do
    wl="${entry%%|*}"
    sz="${entry##*|}"
    for sc in "${SCALINGS[@]}"; do
        key="${wl}_${sc}_${sz}"
        rd0="${RES_RD0[$key]:-N/A}"
        wr0="${RES_WR0[$key]:-N/A}"

        if [ "$rd0" = "N/A" ] || [ "$wr0" = "N/A" ]; then
            printf "%-10s %-5s %6s | %8s %8s | %8s | SKIP\n" "$wl" "$sc" "$sz" "$rd0" "$wr0" "N/A"
            continue
        fi

        if [ "$wl" = "GEMV" ]; then
            # GEMV: complex tiling, just verify RD > WR and both > 0
            if [ "$rd0" -gt 0 ] && [ "$wr0" -gt 0 ] 2>/dev/null; then
                ratio_str=$(awk "BEGIN { printf \"%.1f\", $rd0 / $wr0 }")
                printf "%-10s %-5s %6s | %8s %8s | %8s | OK (complex)\n" "$wl" "$sc" "$sz" "$rd0" "$wr0" "$ratio_str"
            else
                printf "%-10s %-5s %6s | %8s %8s | %8s | FAIL\n" "$wl" "$sc" "$sz" "$rd0" "$wr0" "N/A"
                error_count=$((error_count + 1))
            fi
            continue
        fi

        if [ "$wl" = "DOT" ]; then
            # DOT: 2 RD streams, WR is scalar (very small)
            # Just check RD > WR significantly
            if [ "$rd0" -gt 0 ] && [ "$wr0" -ge 0 ] 2>/dev/null; then
                if [ "$wr0" -gt 0 ]; then
                    ratio_str=$(awk "BEGIN { printf \"%.1f\", $rd0 / $wr0 }")
                else
                    ratio_str="inf"
                fi
                status="OK (scalar WR)"
            else
                ratio_str="N/A"
                status="FAIL"
                error_count=$((error_count + 1))
            fi
            printf "%-10s %-5s %6s | %8s %8s | %8s | %s\n" "$wl" "$sc" "$sz" "$rd0" "$wr0" "$ratio_str" "$status"
            continue
        fi

        # BLAS (not DOT, not GEMV): check integer RD/WR ratio
        expected_ratio="${WL_RD_MULT[$wl]:-1}"
        if [ "$wr0" -gt 0 ] 2>/dev/null; then
            actual_ratio=$((rd0 / wr0))
            ratio_str="${actual_ratio}"
            if [ "$actual_ratio" -eq "$expected_ratio" ]; then
                status="OK"
            else
                status="FAIL (exp=${expected_ratio})"
                error_count=$((error_count + 1))
            fi
        else
            ratio_str="div0"
            status="FAIL (wr0=0)"
            error_count=$((error_count + 1))
        fi

        printf "%-10s %-5s %6s | %8s %8s | %8s | %s\n" "$wl" "$sc" "$sz" "$rd0" "$wr0" "$ratio_str" "$status"
    done
done

# ============================================================================
#  3. Cross-Size Scaling (expect 4x RD per size step, BLAS only)
# ============================================================================
echo ""
echo "============================================"
echo " 3. Cross-Size Scaling (expect 4x per step)"
echo "============================================"
echo ""
printf "%-10s %-5s | %8s -> %-8s | %10s -> %-10s | %6s | %s\n" \
    "Workload" "DQ" "Size1" "Size2" "RD1" "RD2" "Ratio" "Status"
printf "%s\n" "$(printf '%.0s-' {1..90})"

for entry in "${JOB_LIST[@]}"; do
    wl="${entry%%|*}"
    sz="${entry##*|}"
    # skip — we iterate per workload+scaling below
    true
done

# Iterate properly: per workload, per scaling, compare adjacent sizes
prev_printed_wl=""
for wl_entry in "${BLAS_WORKLOADS[@]}" "GEMV"; do
    if [ -n "$FILTER_WL" ] && [ "$FILTER_WL" != "$wl_entry" ]; then continue; fi

    if [ "$wl_entry" = "GEMV" ]; then
        sizes_arr=("${GEMV_SIZES[@]}")
    else
        sizes_arr=("${BLAS_SIZES[@]}")
    fi

    for sc in "${SCALINGS[@]}"; do
        prev_sz=""
        prev_rd=""
        for sz in "${sizes_arr[@]}"; do
            key="${wl_entry}_${sc}_${sz}"
            cur_rd="${RES_RD0[$key]:-}"
            if [ -n "$prev_rd" ] && [ -n "$cur_rd" ] && [ "$prev_rd" -gt 0 ] 2>/dev/null; then
                ratio=$(awk "BEGIN { printf \"%.2f\", $cur_rd / $prev_rd }")
                # GEMV scaling is not necessarily 4x, allow wider tolerance
                if [ "$wl_entry" = "GEMV" ]; then
                    check="OK"
                else
                    check=$(awk "BEGIN { diff = $ratio - 4.0; if(diff<0) diff=-diff; print (diff > 0.05) ? \"WARN\" : \"OK\" }")
                fi
                if [ "$check" = "WARN" ]; then error_count=$((error_count + 1)); fi
                printf "%-10s %-5s | %8s -> %-8s | %10s -> %-10s | %5sx | %s\n" \
                    "$wl_entry" "$sc" "$prev_sz" "$sz" "$prev_rd" "$cur_rd" "$ratio" "$check"
            fi
            prev_sz="$sz"
            prev_rd="$cur_rd"
        done
    done
done

# ============================================================================
#  4. Cross-Scaling RD Ratio (x4 vs x8 vs x16)
# ============================================================================
echo ""
echo "============================================"
echo " 4. Cross-Scaling RD Ratio (x4 vs x8 vs x16)"
echo "============================================"
echo ""
echo "BLAS same data: x4->x8 ~2.0, x8->x16 ~2.0 (opsize halves or BG halves)"
echo "GEMV: ratios may differ due to tiling differences"
echo ""
printf "%-10s %6s | %10s %10s %10s | %8s %8s | %s\n" \
    "Workload" "Size" "RD_x4" "RD_x8" "RD_x16" "x4/x8" "x8/x16" "Status"
printf "%s\n" "$(printf '%.0s-' {1..90})"

for wl_entry in "${BLAS_WORKLOADS[@]}" "GEMV"; do
    if [ -n "$FILTER_WL" ] && [ "$FILTER_WL" != "$wl_entry" ]; then continue; fi

    if [ "$wl_entry" = "GEMV" ]; then
        sizes_arr=("${GEMV_SIZES[@]}")
    else
        sizes_arr=("${BLAS_SIZES[@]}")
    fi

    for sz in "${sizes_arr[@]}"; do
        rd_x4="${RES_RD0[${wl_entry}_x4_${sz}]:-N/A}"
        rd_x8="${RES_RD0[${wl_entry}_x8_${sz}]:-N/A}"
        rd_x16="${RES_RD0[${wl_entry}_x16_${sz}]:-N/A}"

        ratio_4_8="N/A"
        ratio_8_16="N/A"
        status=""

        if [ "$rd_x4" != "N/A" ] && [ "$rd_x8" != "N/A" ] && [ "$rd_x8" -gt 0 ] 2>/dev/null; then
            ratio_4_8=$(awk "BEGIN { printf \"%.2f\", $rd_x4 / $rd_x8 }")
            if [ "$wl_entry" != "GEMV" ]; then
                check=$(awk "BEGIN { diff = $ratio_4_8 - 2.0; if(diff<0) diff=-diff; print (diff > 0.1) ? \"WARN\" : \"OK\" }")
                if [ "$check" = "WARN" ]; then
                    status+=" [x4/x8!~2.0]"
                    error_count=$((error_count + 1))
                fi
            fi
        fi

        if [ "$rd_x8" != "N/A" ] && [ "$rd_x16" != "N/A" ] && [ "$rd_x16" -gt 0 ] 2>/dev/null; then
            ratio_8_16=$(awk "BEGIN { printf \"%.2f\", $rd_x8 / $rd_x16 }")
            if [ "$wl_entry" != "GEMV" ]; then
                check=$(awk "BEGIN { diff = $ratio_8_16 - 2.0; if(diff<0) diff=-diff; print (diff > 0.1) ? \"WARN\" : \"OK\" }")
                if [ "$check" = "WARN" ]; then
                    status+=" [x8/x16!~2.0]"
                    error_count=$((error_count + 1))
                fi
            fi
        fi

        if [ -z "$status" ]; then status="OK"; fi

        printf "%-10s %6s | %10s %10s %10s | %8s %8s | %s\n" \
            "$wl_entry" "$sz" "$rd_x4" "$rd_x8" "$rd_x16" "$ratio_4_8" "$ratio_8_16" "$status"
    done
done

# ============================================================================
#  5. Expected RD Count (BLAS only, GEMV skipped)
# ============================================================================
echo ""
echo "============================================"
echo " 5. Expected RD Count (BLAS workloads)"
echo "============================================"
echo ""
echo "RD per CH = rd_mult x n_bg x opsize x iteration x 4pch"
echo "  COPY=1x, AXPY/AXPBY/XMY=2x, AXPBYPCZ=3x, DOT=2x"
echo ""
printf "%-10s %-5s %6s | %10s %10s | %s\n" \
    "Workload" "DQ" "Size" "Expected" "Actual" "Status"
printf "%s\n" "$(printf '%.0s-' {1..62})"

for wl in "${BLAS_WORKLOADS[@]}"; do
    if [ -n "$FILTER_WL" ] && [ "$FILTER_WL" != "$wl" ]; then continue; fi

    rd_mult="${WL_RD_MULT[$wl]:-1}"

    for sc in "${SCALINGS[@]}"; do
        for sz in "${BLAS_SIZES[@]}"; do
            key="${wl}_${sc}_${sz}"
            actual="${RES_RD0[$key]:-N/A}"

            calc_expected_base "$sc" "$sz"
            expected=$((exp_base * rd_mult))

            if [ "$actual" = "N/A" ]; then
                status="SKIP"
            elif [ "$actual" -eq "$expected" ] 2>/dev/null; then
                status="OK"
            else
                status="FAIL (exp=$expected)"
                error_count=$((error_count + 1))
            fi

            printf "%-10s %-5s %6s | %10d %10s | %s\n" "$wl" "$sc" "$sz" "$expected" "$actual" "$status"
        done
    done
done

# ============================================================================
#  6. NDP State Distribution (PCH[0])
# ============================================================================
echo ""
echo "============================================"
echo " 6. NDP State Distribution (PCH[0] cycles)"
echo "============================================"
echo ""
printf "%-10s %-5s %6s | %10s %10s %10s %10s %10s\n" \
    "Workload" "DQ" "Size" "RUN" "BAR" "WAIT" "FETCH_STL" "DONE"
printf "%s\n" "$(printf '%.0s-' {1..80})"

for entry in "${JOB_LIST[@]}"; do
    wl="${entry%%|*}"
    sz="${entry##*|}"
    for sc in "${SCALINGS[@]}"; do
        logfile="${LOG_DIR}/${wl}_${sc}_${sz}.log"
        if [ ! -f "$logfile" ]; then continue; fi

        ndp_run=$(grep '^\[NDP_RUN\]' "$logfile" 2>/dev/null | head -1 | awk -F'|' '{gsub(/[^0-9]/,"",$2); print $2}') || ndp_run="N/A"
        ndp_bar=$(grep '^\[NDP_BAR\]' "$logfile" 2>/dev/null | head -1 | awk -F'|' '{gsub(/[^0-9]/,"",$2); print $2}') || ndp_bar="N/A"
        ndp_wait=$(grep '^\[NDP_WAIT\]' "$logfile" 2>/dev/null | head -1 | awk -F'|' '{gsub(/[^0-9]/,"",$2); print $2}') || ndp_wait="N/A"
        ndp_fetch=$(grep '^\[NDP_FETCH_STALL\]' "$logfile" 2>/dev/null | head -1 | awk -F'|' '{gsub(/[^0-9]/,"",$2); print $2}') || ndp_fetch="N/A"
        ndp_done=$(grep '^\[NDP_DONE\]' "$logfile" 2>/dev/null | head -1 | awk -F'|' '{gsub(/[^0-9]/,"",$2); print $2}') || ndp_done="N/A"

        printf "%-10s %-5s %6s | %10s %10s %10s %10s %10s\n" \
            "$wl" "$sc" "$sz" \
            "${ndp_run:-N/A}" "${ndp_bar:-N/A}" "${ndp_wait:-N/A}" \
            "${ndp_fetch:-N/A}" "${ndp_done:-N/A}"
    done
done

# ============================================================================
#  7. Bandwidth Summary (per workload, best scaling)
# ============================================================================
echo ""
echo "============================================"
echo " 7. NDP Bandwidth Summary (GB/s, largest size)"
echo "============================================"
echo ""
printf "%-10s | %12s %12s %12s\n" "Workload" "x4" "x8" "x16"
printf "%s\n" "$(printf '%.0s-' {1..52})"

for wl_entry in "${BLAS_WORKLOADS[@]}" "GEMV"; do
    if [ -n "$FILTER_WL" ] && [ "$FILTER_WL" != "$wl_entry" ]; then continue; fi

    if [ "$wl_entry" = "GEMV" ]; then
        largest_sz="128K"
    else
        largest_sz="8M"
    fi

    bw_x4="N/A"; bw_x8="N/A"; bw_x16="N/A"
    for sc in "${SCALINGS[@]}"; do
        logfile="${LOG_DIR}/${wl_entry}_${sc}_${largest_sz}.log"
        if [ -f "$logfile" ]; then
            bw=$(grep 'DB<->DRAM NDP' "$logfile" 2>/dev/null | head -1 | awk '{print $(NF-1)}') || bw="N/A"
            case "$sc" in
                x4) bw_x4="$bw" ;;
                x8) bw_x8="$bw" ;;
                x16) bw_x16="$bw" ;;
            esac
        fi
    done

    printf "%-10s | %12s %12s %12s\n" "$wl_entry" "$bw_x4" "$bw_x8" "$bw_x16"
done

# ============================================================================
#  Summary
# ============================================================================
echo ""
echo "============================================"
echo " Summary"
echo "============================================"
echo ""
if [ $error_count -gt 0 ]; then
    echo "WARNING: $error_count consistency issue(s) detected."
    echo "Review [FAIL], [WARN], or flagged entries above."
else
    echo "All consistency checks PASSED."
fi
echo ""
echo "Logs: $LOG_DIR/"
