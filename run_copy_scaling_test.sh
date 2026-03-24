#!/bin/bash
# ============================================================================
# DBX-DIMM COPY Workload — x4/x8/x16 Scaling Consistency Check
#
# Runs COPY workload across all sizes (8K~8M) and DRAM scalings (x4/x8/x16),
# then verifies consistency of NDP DRAM RD/WR counts and cross-scaling ratios.
#
# Usage:
#   ./run_copy_scaling_test.sh              # default: 4 parallel jobs
#   ./run_copy_scaling_test.sh -j 8         # 8 parallel jobs
#   ./run_copy_scaling_test.sh -s           # skip simulation, only parse logs
# ============================================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

# ---------- defaults ----------
MAX_JOBS=4
LOG_DIR="run/log_copy_scaling"
SKIP_SIM=false

BINARY="./ramulator2"
CONFIG="configuration/dbxdimm_ndp_test.yaml"
TRACE_DIR="./trace_test/pch_ndp"

SIZES=("8K" "32K" "128K" "512K" "2M" "8M")

# scaling → (real_dq, preset)
declare -A REAL_DQ=( [x4]=4  [x8]=8  [x16]=16 )
declare -A PRESET=(  [x4]="DDR5_16Gb_DBX_x4" [x8]="DDR5_16Gb_DBX_x8" [x16]="DDR5_16Gb_DBX_x16" )
SCALINGS=("x4" "x8" "x16")

# ---------- argument parsing ----------
while [[ $# -gt 0 ]]; do
    case "$1" in
        -j) MAX_JOBS="$2"; shift 2 ;;
        -s) SKIP_SIM=true; shift   ;;
        -h|--help)
            echo "Usage: $0 [-j JOBS] [-s]"
            echo "  -j JOBS  Number of parallel jobs (default: 4)"
            echo "  -s       Skip simulation, only parse existing logs"
            exit 0 ;;
        *) echo "Unknown option: $1"; exit 1 ;;
    esac
done

mkdir -p "$LOG_DIR"

total=$(( ${#SIZES[@]} * ${#SCALINGS[@]} ))
launched=0

echo "============================================"
echo " COPY Workload x4/x8/x16 Scaling Test"
echo "============================================"
echo "Config:     $CONFIG"
echo "Trace dir:  $TRACE_DIR"
echo "Sizes:      ${SIZES[*]}"
echo "Scalings:   ${SCALINGS[*]}"
echo "Total runs: $total"
echo "Parallel:   $MAX_JOBS"
echo "Log dir:    $LOG_DIR"
echo ""

# ---------- single simulation runner ----------
run_sim() {
    local sc="$1"
    local sz="$2"
    local trace="$3"
    local logfile="$4"
    local dq="$5"
    local preset="$6"

    if [ ! -f "$trace" ]; then
        echo "[SKIP] COPY_${sc}_${sz}: trace not found ($trace)"
        return 2
    fi

    $BINARY -f "$CONFIG" \
        -p "Frontend.core0_trace=$trace" \
        -p "MemorySystem.DRAM.org.preset=$preset" \
        -p "MemorySystem.DRAM.org.real_dq=$dq" \
        > "$logfile" 2>&1
    local rc=$?

    if [ $rc -eq 0 ]; then
        echo "[DONE] COPY_${sc}_${sz}"
    else
        echo "[FAIL] COPY_${sc}_${sz} (exit $rc)"
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

    for sc in "${SCALINGS[@]}"; do
        for sz in "${SIZES[@]}"; do
            trace="${TRACE_DIR}/pch_ndp_${sc}_${sz}_COPY.txt"
            logfile="${LOG_DIR}/COPY_${sc}_${sz}.log"
            launched=$((launched + 1))
            echo "[$launched/$total] COPY ${sc} ${sz} ..."
            run_sim "$sc" "$sz" "$trace" "$logfile" "${REAL_DQ[$sc]}" "${PRESET[$sc]}" &

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
#  Result Parsing & Consistency Check
# ============================================================================

echo "============================================"
echo " 1. Per-Run Results"
echo "============================================"
echo ""

printf "%-5s %6s | %10s | %8s %8s %8s %8s | %12s %12s | %7s\n" \
    "DQ" "Size" "Cycles" \
    "NDP_RD0" "NDP_WR0" "NDP_RD1" "NDP_WR1" \
    "NDP_BW" "Host_BW" \
    "DescHR"
printf "%s\n" "$(printf '%.0s-' {1..115})"

# Associative arrays for cross-checks
declare -A RES_CYCLES RES_RD0 RES_WR0 RES_RD1 RES_WR1 RES_NDP_BW
error_count=0

for sc in "${SCALINGS[@]}"; do
    for sz in "${SIZES[@]}"; do
        key="${sc}_${sz}"
        logfile="${LOG_DIR}/COPY_${sc}_${sz}.log"

        if [ ! -f "$logfile" ]; then
            printf "%-5s %6s | [LOG NOT FOUND]\n" "$sc" "$sz"
            error_count=$((error_count + 1))
            continue
        fi

        if ! grep -q "All Request Done" "$logfile" 2>/dev/null; then
            printf "%-5s %6s | [SIM NOT COMPLETED]\n" "$sc" "$sz"
            error_count=$((error_count + 1))
            continue
        fi

        cycles=$(grep -oP 'Total simulation cycles: \K\d+' "$logfile" 2>/dev/null || echo "N/A")
        rd0=$(grep -oP 's_num_ndp_dram_rd_0: \K\d+' "$logfile" 2>/dev/null || echo "N/A")
        wr0=$(grep -oP 's_num_ndp_dram_wr_0: \K\d+' "$logfile" 2>/dev/null || echo "N/A")
        rd1=$(grep -oP 's_num_ndp_dram_rd_1: \K\d+' "$logfile" 2>/dev/null || echo "N/A")
        wr1=$(grep -oP 's_num_ndp_dram_wr_1: \K\d+' "$logfile" 2>/dev/null || echo "N/A")
        ndp_bw=$(grep 'DB<->DRAM NDP' "$logfile" 2>/dev/null | head -1 | awk '{print $(NF-1)}') || ndp_bw="N/A"
        host_bw=$(grep 'DB<->DRAM HOST' "$logfile" 2>/dev/null | head -1 | awk '{print $(NF-1)}') || host_bw="N/A"

        # Descriptor cache hit rate (average)
        hit_lines=$(grep -A10 'Hit Rate' "$logfile" 2>/dev/null | grep -oP '\d+\.\d+%' | head -8) || hit_lines=""
        if [ -n "$hit_lines" ]; then
            avg_hit=$(echo "$hit_lines" | awk '{ gsub(/%/,""); sum+=$1; n++ } END { if(n>0) printf "%.1f%%", sum/n; else print "N/A" }')
        else
            avg_hit="N/A"
        fi

        RES_CYCLES[$key]="$cycles"
        RES_RD0[$key]="$rd0"
        RES_WR0[$key]="$wr0"
        RES_RD1[$key]="$rd1"
        RES_WR1[$key]="$wr1"
        RES_NDP_BW[$key]="$ndp_bw"

        warnings=""
        if [ "$rd0" != "N/A" ] && [ "$rd1" != "N/A" ] && [ "$rd0" != "$rd1" ]; then
            warnings+=" [CH_RD_ASYM]"
        fi
        if [ "$wr0" != "N/A" ] && [ "$wr1" != "N/A" ] && [ "$wr0" != "$wr1" ]; then
            warnings+=" [CH_WR_ASYM]"
        fi
        if [ "$rd0" != "N/A" ] && [ "$wr0" != "N/A" ] && [ "$rd0" != "$wr0" ]; then
            warnings+=" [RD!=WR]"
        fi
        if [ "$rd0" = "0" ] 2>/dev/null; then
            warnings+=" [NO_NDP_RD]"
        fi
        if [ -n "$warnings" ]; then
            error_count=$((error_count + 1))
        fi

        printf "%-5s %6s | %10s | %8s %8s %8s %8s | %12s %12s | %7s%s\n" \
            "$sc" "$sz" "$cycles" \
            "$rd0" "$wr0" "$rd1" "$wr1" \
            "$ndp_bw" "$host_bw" \
            "$avg_hit" "$warnings"
    done
done

# ============================================================================
echo ""
echo "============================================"
echo " 2. COPY RD == WR Check (per channel)"
echo "============================================"
echo ""
printf "%-5s %6s | %8s %8s | %s\n" "DQ" "Size" "RD0" "WR0" "Status"
printf "%s\n" "$(printf '%.0s-' {1..50})"

for sc in "${SCALINGS[@]}"; do
    for sz in "${SIZES[@]}"; do
        key="${sc}_${sz}"
        rd0="${RES_RD0[$key]:-N/A}"
        wr0="${RES_WR0[$key]:-N/A}"
        if [ "$rd0" = "N/A" ] || [ "$wr0" = "N/A" ]; then
            status="SKIP"
        elif [ "$rd0" = "$wr0" ]; then
            status="OK"
        else
            status="FAIL"
            error_count=$((error_count + 1))
        fi
        printf "%-5s %6s | %8s %8s | %s\n" "$sc" "$sz" "$rd0" "$wr0" "$status"
    done
done

# ============================================================================
echo ""
echo "============================================"
echo " 3. Cross-Size Scaling (expect 4x RD per size step)"
echo "============================================"
echo ""
printf "%-5s | %8s -> %-8s | %10s -> %-10s | %6s | %s\n" \
    "DQ" "Size1" "Size2" "RD1" "RD2" "Ratio" "Status"
printf "%s\n" "$(printf '%.0s-' {1..80})"

for sc in "${SCALINGS[@]}"; do
    prev_sz=""
    prev_rd=""
    for sz in "${SIZES[@]}"; do
        key="${sc}_${sz}"
        cur_rd="${RES_RD0[$key]:-}"
        if [ -n "$prev_rd" ] && [ -n "$cur_rd" ] && [ "$prev_rd" -gt 0 ] 2>/dev/null; then
            ratio=$(awk "BEGIN { printf \"%.2f\", $cur_rd / $prev_rd }")
            check=$(awk "BEGIN { diff = $ratio - 4.0; if(diff<0) diff=-diff; print (diff > 0.05) ? \"WARN\" : \"OK\" }")
            if [ "$check" = "WARN" ]; then error_count=$((error_count + 1)); fi
            printf "%-5s | %8s -> %-8s | %10s -> %-10s | %5sx | %s\n" \
                "$sc" "$prev_sz" "$sz" "$prev_rd" "$cur_rd" "$ratio" "$check"
        fi
        prev_sz="$sz"
        prev_rd="$cur_rd"
    done
done

# ============================================================================
echo ""
echo "============================================"
echo " 4. Cross-Scaling RD Ratio (x4 vs x8 vs x16)"
echo "============================================"
echo ""
echo "COPY same data: x4->x8 expect RD ratio ~2.0, x8->x16 expect ~2.0"
echo "  x4: 8BG x 128col, x8: 8BG x 64col, x16: 4BG x 64col"
echo ""
printf "%6s | %10s %10s %10s | %8s %8s | %s\n" \
    "Size" "RD_x4" "RD_x8" "RD_x16" "x4/x8" "x8/x16" "Status"
printf "%s\n" "$(printf '%.0s-' {1..80})"

for sz in "${SIZES[@]}"; do
    rd_x4="${RES_RD0[x4_${sz}]:-N/A}"
    rd_x8="${RES_RD0[x8_${sz}]:-N/A}"
    rd_x16="${RES_RD0[x16_${sz}]:-N/A}"

    ratio_4_8="N/A"
    ratio_8_16="N/A"
    status=""

    if [ "$rd_x4" != "N/A" ] && [ "$rd_x8" != "N/A" ] && [ "$rd_x8" -gt 0 ] 2>/dev/null; then
        ratio_4_8=$(awk "BEGIN { printf \"%.2f\", $rd_x4 / $rd_x8 }")
        check=$(awk "BEGIN { diff = $ratio_4_8 - 2.0; if(diff<0) diff=-diff; print (diff > 0.1) ? \"WARN\" : \"OK\" }")
        if [ "$check" = "WARN" ]; then
            status+=" [x4/x8!~2.0]"
            error_count=$((error_count + 1))
        fi
    fi

    if [ "$rd_x8" != "N/A" ] && [ "$rd_x16" != "N/A" ] && [ "$rd_x16" -gt 0 ] 2>/dev/null; then
        ratio_8_16=$(awk "BEGIN { printf \"%.2f\", $rd_x8 / $rd_x16 }")
        check=$(awk "BEGIN { diff = $ratio_8_16 - 2.0; if(diff<0) diff=-diff; print (diff > 0.1) ? \"WARN\" : \"OK\" }")
        if [ "$check" = "WARN" ]; then
            status+=" [x8/x16!~2.0]"
            error_count=$((error_count + 1))
        fi
    fi

    if [ -z "$status" ]; then status="OK"; fi

    printf "%6s | %10s %10s %10s | %8s %8s | %s\n" \
        "$sz" "$rd_x4" "$rd_x8" "$rd_x16" "$ratio_4_8" "$ratio_8_16" "$status"
done

# ============================================================================
echo ""
echo "============================================"
echo " 5. Expected RD Count Verification"
echo "============================================"
echo ""
echo "COPY per CH = num_working_bg x opsize x iteration x num_pch(4)"
echo "  cal_it_v2: x4(128->halved), x8(64), x16(64)"
echo ""
printf "%-5s %6s | %10s %10s | %s\n" \
    "DQ" "Size" "Expected" "Actual" "Status"
printf "%s\n" "$(printf '%.0s-' {1..55})"

for sc in "${SCALINGS[@]}"; do
    for sz in "${SIZES[@]}"; do
        key="${sc}_${sz}"
        actual="${RES_RD0[$key]:-N/A}"

        case "$sz" in
            8K)   vec=8192 ;;
            32K)  vec=32768 ;;
            128K) vec=131072 ;;
            512K) vec=524288 ;;
            2M)   vec=2097152 ;;
            8M)   vec=8388608 ;;
        esac

        # Replicate cal_it_v2 logic
        case "$sc" in
            x4)
                row_sz=8192; base_op=128; n_bg=8
                max_sz=$((n_bg * row_sz / 2))  # 32768
                if [ "$vec" -gt "$max_sz" ]; then
                    iteration=$((vec / max_sz))
                    opsize=$((base_op / 2))  # 64
                else
                    ratio_d=$((max_sz / vec))
                    iteration=1
                    opsize=$((base_op / 2 / ratio_d))
                fi
                ;;
            x8)
                row_sz=8192; base_op=64; n_bg=8
                max_sz=$((n_bg * row_sz))  # 65536
                if [ "$vec" -gt "$max_sz" ]; then
                    iteration=$((vec / max_sz))
                    opsize=$base_op  # 64
                else
                    ratio_d=$((max_sz / vec))
                    iteration=1
                    opsize=$((base_op / ratio_d))
                fi
                ;;
            x16)
                row_sz=16384; base_op=64; n_bg=4
                max_sz=$((n_bg * row_sz))  # 65536
                if [ "$vec" -gt "$max_sz" ]; then
                    iteration=$((vec / max_sz))
                    opsize=$base_op  # 64
                else
                    ratio_d=$((max_sz / vec))
                    iteration=1
                    opsize=$((base_op / ratio_d))
                fi
                ;;
        esac

        # cal_it_v2 returns (opsize - 1) as 0-indexed, so actual accesses = opsize
        # Expected RD per CH = n_bg * opsize * iteration * 4_pch
        expected=$((n_bg * opsize * iteration * 4))

        if [ "$actual" = "N/A" ]; then
            status="SKIP"
        elif [ "$actual" -eq "$expected" ] 2>/dev/null; then
            status="OK"
        else
            status="FAIL (exp=$expected)"
            error_count=$((error_count + 1))
        fi

        printf "%-5s %6s | %10d %10s | %s\n" "$sc" "$sz" "$expected" "$actual" "$status"
    done
done

# ============================================================================
echo ""
echo "============================================"
echo " 6. NDP State Distribution (PCH[0] cycles)"
echo "============================================"
echo ""
printf "%-5s %6s | %10s %10s %10s %10s %10s %10s\n" \
    "DQ" "Size" "RUN" "BAR" "WAIT" "FETCH_STL" "BEFORE_RUN" "DONE"
printf "%s\n" "$(printf '%.0s-' {1..80})"

for sc in "${SCALINGS[@]}"; do
    for sz in "${SIZES[@]}"; do
        logfile="${LOG_DIR}/COPY_${sc}_${sz}.log"
        if [ ! -f "$logfile" ]; then continue; fi

        ndp_run=$(grep '^\[NDP_RUN\]' "$logfile" 2>/dev/null | head -1 | awk -F'|' '{gsub(/[^0-9]/,"",$2); print $2}')
        ndp_bar=$(grep '^\[NDP_BAR\]' "$logfile" 2>/dev/null | head -1 | awk -F'|' '{gsub(/[^0-9]/,"",$2); print $2}')
        ndp_wait=$(grep '^\[NDP_WAIT\]' "$logfile" 2>/dev/null | head -1 | awk -F'|' '{gsub(/[^0-9]/,"",$2); print $2}')
        ndp_fetch=$(grep '^\[NDP_FETCH_STALL\]' "$logfile" 2>/dev/null | head -1 | awk -F'|' '{gsub(/[^0-9]/,"",$2); print $2}')
        ndp_before=$(grep '^\[NDP_BEFORE_RUN\]' "$logfile" 2>/dev/null | head -1 | awk -F'|' '{gsub(/[^0-9]/,"",$2); print $2}')
        ndp_done=$(grep '^\[NDP_DONE\]' "$logfile" 2>/dev/null | head -1 | awk -F'|' '{gsub(/[^0-9]/,"",$2); print $2}')

        printf "%-5s %6s | %10s %10s %10s %10s %10s %10s\n" \
            "$sc" "$sz" \
            "${ndp_run:-N/A}" "${ndp_bar:-N/A}" "${ndp_wait:-N/A}" \
            "${ndp_fetch:-N/A}" "${ndp_before:-N/A}" "${ndp_done:-N/A}"
    done
done

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
