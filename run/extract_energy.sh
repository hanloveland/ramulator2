#!/bin/bash
#
# extract_energy.sh — Extract energy from experiment logs
#
# Usage: ./extract_energy.sh [LOG_DIR]
#   LOG_DIR: path to log_experiments/ (default: ./log_experiments)
#
# Outputs:
#   1) Per-experiment Total Energy comparison CSV
#   2) Per-experiment Energy Breakdown CSV (Background, Command, DQ, NDP)
#

set -euo pipefail

LOG_DIR="${1:-./log_experiments}"

if [[ ! -d "$LOG_DIR" ]]; then
    echo "Error: $LOG_DIR not found"
    exit 1
fi

OUTDIR="energy_results"
mkdir -p "$OUTDIR"

###############################################################################
# Helper: extract final summary energy from a single log file
# Outputs: bg_nj cmd_nj dq_nj ndp_nj total_nj total_w
###############################################################################
extract_energy() {
    local logfile="$1"
    if [[ ! -f "$logfile" ]]; then
        echo "N/A N/A N/A N/A N/A N/A"
        return
    fi

    # Extract the final summary lines (last occurrence of each)
    local bg cmd dq ndp total power

    bg=$(grep "DRAM Background Energy" "$logfile" | tail -1 | grep -oE '[0-9]+\.?[0-9]*e?[+-]?[0-9]*' | tail -1)
    cmd=$(grep "DRAM Command Energy" "$logfile" | tail -1 | grep -oE '[0-9]+\.?[0-9]*e?[+-]?[0-9]*' | tail -1)
    dq=$(grep "DRAM DQ Energy" "$logfile" | tail -1 | grep -oE '[0-9]+\.?[0-9]*e?[+-]?[0-9]*' | tail -1)
    total=$(grep "Total DRAM Energy" "$logfile" | tail -1 | grep -oE '[0-9]+\.?[0-9]*e?[+-]?[0-9]*' | tail -1)
    power=$(grep "Total DRAM Power" "$logfile" | tail -1 | grep -oE '[0-9]+\.?[0-9]*e?[+-]?[0-9]*' | tail -1)

    # NDP Ops Energy — summary line (not per-channel), last occurrence
    ndp=$(grep "NDP Ops Energy" "$logfile" | tail -1 | grep -oE '[0-9]+\.?[0-9]*e?[+-]?[0-9]*' | tail -1)
    [[ -z "$ndp" ]] && ndp="0"

    [[ -z "$bg" ]] && bg="N/A"
    [[ -z "$cmd" ]] && cmd="N/A"
    [[ -z "$dq" ]] && dq="N/A"
    [[ -z "$total" ]] && total="N/A"
    [[ -z "$power" ]] && power="N/A"

    echo "$bg $cmd $dq $ndp $total $power"
}

###############################################################################
# Format number: scientific notation → human-readable (nJ, μJ, mJ)
###############################################################################
fmt_nj() {
    python3 -c "
import sys
v = sys.argv[1]
if v == 'N/A':
    print('%12s' % 'N/A')
else:
    val = float(v)
    if val >= 1e6:
        print('%10.2f mJ' % (val/1e6))
    elif val >= 1e3:
        print('%10.2f uJ' % (val/1e3))
    else:
        print('%10.2f nJ' % val)
" "$1"
}

###############################################################################
# Experiment 1: Base Standalone (1 DIMM, x4)
# Configs: base, dbx, dbxn, async
###############################################################################
extract_exp1() {
    local exp_dir="$LOG_DIR/exp1"
    [[ ! -d "$exp_dir" ]] && { echo "Exp1: $exp_dir not found, skipping"; return; }

    local BLAS_WL=("COPY" "AXPY" "AXPBY" "AXPBYPCZ" "XMY" "DOT")
    local GEMV_WL=("GEMV")
    local BLAS_SIZES=("8K" "32K" "128K" "512K" "2M" "8M")
    local GEMV_SIZES=("8K" "16K" "32K" "64K" "128K")
    local CONFIGS=("base" "dbx" "dbxn" "async")
    local LABELS=("Baseline" "DBX" "DBX-N" "AsyncDIMM-N")

    local csv="$OUTDIR/exp1_total_energy.csv"
    echo "=== Experiment 1: Base Standalone (1D x4) — Total Energy ==="

    # Header
    header="Workload,Size"
    for label in "${LABELS[@]}"; do
        header+=",${label}_Total(nJ),${label}_Power(W)"
    done
    echo "$header" > "$csv"

    # BLAS workloads
    for wl in "${BLAS_WL[@]}"; do
        for sz in "${BLAS_SIZES[@]}"; do
            row="$wl,$sz"
            for cfg in "${CONFIGS[@]}"; do
                local logfile="$exp_dir/${cfg}_${wl}_${sz}.log"
                read -r bg cmd dq ndp total power <<< "$(extract_energy "$logfile")"
                row+=",$total,$power"
            done
            echo "$row" >> "$csv"
        done
    done

    # GEMV workloads
    for wl in "${GEMV_WL[@]}"; do
        for sz in "${GEMV_SIZES[@]}"; do
            row="$wl,$sz"
            for cfg in "${CONFIGS[@]}"; do
                local logfile="$exp_dir/${cfg}_${wl}_${sz}.log"
                read -r bg cmd dq ndp total power <<< "$(extract_energy "$logfile")"
                row+=",$total,$power"
            done
            echo "$row" >> "$csv"
        done
    done
    echo "  -> $csv"

    # Breakdown CSV
    local csv_bd="$OUTDIR/exp1_energy_breakdown.csv"
    echo "Workload,Size,Config,Background(nJ),Command(nJ),DQ(nJ),NDP(nJ),Total(nJ),Power(W)" > "$csv_bd"

    for wl in "${BLAS_WL[@]}" "${GEMV_WL[@]}"; do
        if [[ "$wl" == "GEMV" ]]; then
            local sizes=("${GEMV_SIZES[@]}")
        else
            local sizes=("${BLAS_SIZES[@]}")
        fi
        for sz in "${sizes[@]}"; do
            for i in "${!CONFIGS[@]}"; do
                local cfg="${CONFIGS[$i]}"
                local label="${LABELS[$i]}"
                local logfile="$exp_dir/${cfg}_${wl}_${sz}.log"
                read -r bg cmd dq ndp total power <<< "$(extract_energy "$logfile")"
                echo "$wl,$sz,$label,$bg,$cmd,$dq,$ndp,$total,$power" >> "$csv_bd"
            done
        done
    done
    echo "  -> $csv_bd"
}

###############################################################################
# Experiment 2: Scaling DRAM Device (1 DIMM, x4/x8/x16)
###############################################################################
extract_exp2() {
    local exp_dir="$LOG_DIR/exp2"
    [[ ! -d "$exp_dir" ]] && { echo "Exp2: $exp_dir not found, skipping"; return; }

    local BLAS_WL=("COPY" "AXPY" "AXPBY" "AXPBYPCZ" "XMY" "DOT")
    local GEMV_WL=("GEMV")
    local BLAS_SIZES=("8K" "32K" "128K" "512K" "2M" "8M")
    local GEMV_SIZES=("8K" "16K" "32K" "64K" "128K")
    local SCALINGS=("x4" "x8" "x16")

    local csv="$OUTDIR/exp2_total_energy.csv"
    echo ""
    echo "=== Experiment 2: Scaling DRAM (1D, x4/x8/x16) — Total Energy ==="

    header="Workload,Size"
    for sc in "${SCALINGS[@]}"; do
        header+=",${sc}_Total(nJ),${sc}_Power(W)"
    done
    echo "$header" > "$csv"

    for wl in "${BLAS_WL[@]}"; do
        for sz in "${BLAS_SIZES[@]}"; do
            row="$wl,$sz"
            for sc in "${SCALINGS[@]}"; do
                local logfile="$exp_dir/${sc}_${wl}_${sz}.log"
                read -r bg cmd dq ndp total power <<< "$(extract_energy "$logfile")"
                row+=",$total,$power"
            done
            echo "$row" >> "$csv"
        done
    done
    for wl in "${GEMV_WL[@]}"; do
        for sz in "${GEMV_SIZES[@]}"; do
            row="$wl,$sz"
            for sc in "${SCALINGS[@]}"; do
                local logfile="$exp_dir/${sc}_${wl}_${sz}.log"
                read -r bg cmd dq ndp total power <<< "$(extract_energy "$logfile")"
                row+=",$total,$power"
            done
            echo "$row" >> "$csv"
        done
    done
    echo "  -> $csv"

    # Breakdown CSV
    local csv_bd="$OUTDIR/exp2_energy_breakdown.csv"
    echo "Workload,Size,Scaling,Background(nJ),Command(nJ),DQ(nJ),NDP(nJ),Total(nJ),Power(W)" > "$csv_bd"

    for wl in "${BLAS_WL[@]}" "${GEMV_WL[@]}"; do
        if [[ "$wl" == "GEMV" ]]; then
            local sizes=("${GEMV_SIZES[@]}")
        else
            local sizes=("${BLAS_SIZES[@]}")
        fi
        for sz in "${sizes[@]}"; do
            for sc in "${SCALINGS[@]}"; do
                local logfile="$exp_dir/${sc}_${wl}_${sz}.log"
                read -r bg cmd dq ndp total power <<< "$(extract_energy "$logfile")"
                echo "$wl,$sz,$sc,$bg,$cmd,$dq,$ndp,$total,$power" >> "$csv_bd"
            done
        done
    done
    echo "  -> $csv_bd"
}

###############################################################################
# Experiment 3: Scaling DIMMs (x4, 1/2/4/8/16 DIMM)
###############################################################################
extract_exp3() {
    local exp_dir="$LOG_DIR/exp3"
    [[ ! -d "$exp_dir" ]] && { echo "Exp3: $exp_dir not found, skipping"; return; }

    local BLAS_WL=("COPY" "AXPY" "AXPBY" "AXPBYPCZ" "XMY" "DOT")
    local GEMV_WL=("GEMV")
    local BLAS_SIZES=("8K" "32K" "128K" "512K" "2M" "8M")
    local GEMV_SIZES=("8K" "16K" "32K" "64K" "128K")
    local DIMMS=("d1" "d2" "d4" "d8" "d16")
    local DIMM_LABELS=("1D" "2D" "4D" "8D" "16D")

    local csv="$OUTDIR/exp3_total_energy.csv"
    echo ""
    echo "=== Experiment 3: Scaling DIMMs (x4, 1-16 DIMM) — Total Energy ==="

    header="Workload,Size"
    for label in "${DIMM_LABELS[@]}"; do
        header+=",${label}_Total(nJ),${label}_Power(W)"
    done
    echo "$header" > "$csv"

    for wl in "${BLAS_WL[@]}"; do
        for sz in "${BLAS_SIZES[@]}"; do
            row="$wl,$sz"
            for dm in "${DIMMS[@]}"; do
                local logfile="$exp_dir/${dm}_${wl}_${sz}.log"
                read -r bg cmd dq ndp total power <<< "$(extract_energy "$logfile")"
                row+=",$total,$power"
            done
            echo "$row" >> "$csv"
        done
    done
    for wl in "${GEMV_WL[@]}"; do
        for sz in "${GEMV_SIZES[@]}"; do
            row="$wl,$sz"
            for dm in "${DIMMS[@]}"; do
                local logfile="$exp_dir/${dm}_${wl}_${sz}.log"
                read -r bg cmd dq ndp total power <<< "$(extract_energy "$logfile")"
                row+=",$total,$power"
            done
            echo "$row" >> "$csv"
        done
    done
    echo "  -> $csv"

    # Breakdown CSV
    local csv_bd="$OUTDIR/exp3_energy_breakdown.csv"
    echo "Workload,Size,DIMMs,Background(nJ),Command(nJ),DQ(nJ),NDP(nJ),Total(nJ),Power(W)" > "$csv_bd"

    for wl in "${BLAS_WL[@]}" "${GEMV_WL[@]}"; do
        if [[ "$wl" == "GEMV" ]]; then
            local sizes=("${GEMV_SIZES[@]}")
        else
            local sizes=("${BLAS_SIZES[@]}")
        fi
        for sz in "${sizes[@]}"; do
            for i in "${!DIMMS[@]}"; do
                local dm="${DIMMS[$i]}"
                local label="${DIMM_LABELS[$i]}"
                local logfile="$exp_dir/${dm}_${wl}_${sz}.log"
                read -r bg cmd dq ndp total power <<< "$(extract_energy "$logfile")"
                echo "$wl,$sz,$label,$bg,$cmd,$dq,$ndp,$total,$power" >> "$csv_bd"
            done
        done
    done
    echo "  -> $csv_bd"
}

###############################################################################
# Experiment 4: Concurrent (SPEC Host + tcore GEMV 128K)
# Configs: base, baseT, dbx, dbxT, dbxN, asyncN
###############################################################################
extract_exp4() {
    local exp_dir="$LOG_DIR/exp4"
    [[ ! -d "$exp_dir" ]] && { echo "Exp4: $exp_dir not found, skipping"; return; }

    local BENCHMARKS=("429mcf" "470lbm" "462libquantum" "401bzip2" "433milc")
    local BENCH_LABELS=("429.mcf" "470.lbm" "462.libquantum" "401.bzip2" "433.milc")
    local GEMV_SZ="128K"
    local CFG_TAGS=("base" "baseT" "dbx" "dbxT" "dbxN" "asyncN")
    local CFG_LABELS=("base" "base-T" "dbx" "dbx-T" "dbx-N" "async-N")

    local csv="$OUTDIR/exp4_total_energy.csv"
    echo ""
    echo "=== Experiment 4: Concurrent (SPEC Host + GEMV ${GEMV_SZ}) — Total Energy ==="

    # Header
    header="Benchmark"
    for label in "${CFG_LABELS[@]}"; do
        header+=",${label}_Total(nJ),${label}_Power(W)"
    done
    echo "$header" > "$csv"

    for i in "${!BENCHMARKS[@]}"; do
        local btag="${BENCHMARKS[$i]}"
        local blabel="${BENCH_LABELS[$i]}"
        row="$blabel"
        for ctag in "${CFG_TAGS[@]}"; do
            local logfile="$exp_dir/${ctag}_${btag}_G${GEMV_SZ}.log"
            read -r bg cmd dq ndp total power <<< "$(extract_energy "$logfile")"
            row+=",$total,$power"
        done
        echo "$row" >> "$csv"
    done
    echo "  -> $csv"

    # Breakdown CSV
    local csv_bd="$OUTDIR/exp4_energy_breakdown.csv"
    echo "Benchmark,Config,Background(nJ),Command(nJ),DQ(nJ),NDP(nJ),Total(nJ),Power(W)" > "$csv_bd"

    for i in "${!BENCHMARKS[@]}"; do
        local btag="${BENCHMARKS[$i]}"
        local blabel="${BENCH_LABELS[$i]}"
        for j in "${!CFG_TAGS[@]}"; do
            local ctag="${CFG_TAGS[$j]}"
            local clabel="${CFG_LABELS[$j]}"
            local logfile="$exp_dir/${ctag}_${btag}_G${GEMV_SZ}.log"
            read -r bg cmd dq ndp total power <<< "$(extract_energy "$logfile")"
            echo "$blabel,$clabel,$bg,$cmd,$dq,$ndp,$total,$power" >> "$csv_bd"
        done
    done
    echo "  -> $csv_bd"
}

###############################################################################
# Print summary table to stdout
###############################################################################
print_summary() {
    echo ""
    echo "=============================================================================="
    echo " Energy Summary"
    echo "=============================================================================="

    python3 << 'PYEOF'
import csv, os, sys

outdir = "energy_results"

def load_csv(path):
    if not os.path.exists(path):
        return None
    with open(path) as f:
        return list(csv.DictReader(f))

def fmt(val):
    if val == "N/A" or val == "":
        return "%12s" % "N/A"
    v = float(val)
    if v >= 1e6:
        return "%9.2f mJ" % (v/1e6)
    elif v >= 1e3:
        return "%9.2f uJ" % (v/1e3)
    else:
        return "%9.2f nJ" % v

def fmt_pct(val, total):
    """Format as percentage of total"""
    if val == "N/A" or val == "" or total == "N/A" or total == "":
        return "%6s" % "N/A"
    v, t = float(val), float(total)
    if t == 0:
        return "%6s" % "N/A"
    return "%5.1f%%" % (v / t * 100)

def print_breakdown(bd_rows, group_key, group_val, config_key, config_val):
    """Print energy breakdown for a specific (group, config) from breakdown CSV."""
    matches = [r for r in bd_rows
                if r.get(group_key,"") == group_val and r.get(config_key,"") == config_val]
    if not matches:
        return
    r = matches[0]
    bg, cmd, dq, ndp, total = r.get("Background(nJ)","N/A"), r.get("Command(nJ)","N/A"), \
                               r.get("DQ(nJ)","N/A"), r.get("NDP(nJ)","N/A"), r.get("Total(nJ)","N/A")
    print(f"    Background: {fmt(bg)} ({fmt_pct(bg, total)}), "
          f"Command: {fmt(cmd)} ({fmt_pct(cmd, total)}), "
          f"DQ: {fmt(dq)} ({fmt_pct(dq, total)}), "
          f"NDP: {fmt(ndp)} ({fmt_pct(ndp, total)}), "
          f"Total: {fmt(total)}")

# =========================================================================
# Exp1: Total Energy
# =========================================================================
rows = load_csv(f"{outdir}/exp1_total_energy.csv")
bd_rows = load_csv(f"{outdir}/exp1_energy_breakdown.csv")
if rows:
    print("\n--- Exp1: Base Standalone (1D x4) — Total Energy ---")
    print(f"{'Workload':>10s} {'Size':>5s} | {'Baseline':>12s} {'DBX':>12s} {'DBX-N':>12s} {'AsyncDIMM-N':>12s}")
    print("-" * 70)
    for r in rows:
        vals = [fmt(r.get(f"{c}_Total(nJ)","N/A")) for c in ["Baseline","DBX","DBX-N","AsyncDIMM-N"]]
        print(f"{r['Workload']:>10s} {r['Size']:>5s} | {vals[0]} {vals[1]} {vals[2]} {vals[3]}")

# Exp1: Breakdown
if bd_rows:
    print("\n--- Exp1: Base Standalone (1D x4) — Energy Breakdown ---")
    configs = ["Baseline", "DBX", "DBX-N", "AsyncDIMM-N"]
    # Group by workload+size, show breakdown per config
    seen = []
    for r in bd_rows:
        key = (r["Workload"], r["Size"])
        if key not in seen:
            seen.append(key)
    for wl, sz in seen:
        print(f"\n  {wl} {sz}:")
        print(f"    {'Config':>12s} | {'Background':>12s} {'%':>6s} | {'Command':>12s} {'%':>6s} | {'DQ':>12s} {'%':>6s} | {'NDP':>12s} {'%':>6s} | {'Total':>12s}")
        print(f"    {'-'*95}")
        for cfg in configs:
            matches = [r for r in bd_rows if r["Workload"]==wl and r["Size"]==sz and r["Config"]==cfg]
            if not matches: continue
            r = matches[0]
            bg, cmd, dq, ndp, total = r["Background(nJ)"], r["Command(nJ)"], r["DQ(nJ)"], r["NDP(nJ)"], r["Total(nJ)"]
            print(f"    {cfg:>12s} | {fmt(bg)} {fmt_pct(bg,total):>6s} | {fmt(cmd)} {fmt_pct(cmd,total):>6s} | {fmt(dq)} {fmt_pct(dq,total):>6s} | {fmt(ndp)} {fmt_pct(ndp,total):>6s} | {fmt(total)}")

# =========================================================================
# Exp2: Total Energy
# =========================================================================
rows = load_csv(f"{outdir}/exp2_total_energy.csv")
bd_rows = load_csv(f"{outdir}/exp2_energy_breakdown.csv")
if rows:
    print("\n--- Exp2: Scaling DRAM (1D, x4/x8/x16) — Total Energy ---")
    print(f"{'Workload':>10s} {'Size':>5s} | {'x4':>12s} {'x8':>12s} {'x16':>12s}")
    print("-" * 55)
    for r in rows:
        vals = [fmt(r.get(f"{c}_Total(nJ)","N/A")) for c in ["x4","x8","x16"]]
        print(f"{r['Workload']:>10s} {r['Size']:>5s} | {vals[0]} {vals[1]} {vals[2]}")

# Exp2: Breakdown
if bd_rows:
    print("\n--- Exp2: Scaling DRAM (1D, x4/x8/x16) — Energy Breakdown ---")
    scalings = ["x4", "x8", "x16"]
    seen = []
    for r in bd_rows:
        key = (r["Workload"], r["Size"])
        if key not in seen:
            seen.append(key)
    for wl, sz in seen:
        print(f"\n  {wl} {sz}:")
        print(f"    {'Scaling':>8s} | {'Background':>12s} {'%':>6s} | {'Command':>12s} {'%':>6s} | {'DQ':>12s} {'%':>6s} | {'NDP':>12s} {'%':>6s} | {'Total':>12s}")
        print(f"    {'-'*95}")
        for sc in scalings:
            matches = [r for r in bd_rows if r["Workload"]==wl and r["Size"]==sz and r["Scaling"]==sc]
            if not matches: continue
            r = matches[0]
            bg, cmd, dq, ndp, total = r["Background(nJ)"], r["Command(nJ)"], r["DQ(nJ)"], r["NDP(nJ)"], r["Total(nJ)"]
            print(f"    {sc:>8s} | {fmt(bg)} {fmt_pct(bg,total):>6s} | {fmt(cmd)} {fmt_pct(cmd,total):>6s} | {fmt(dq)} {fmt_pct(dq,total):>6s} | {fmt(ndp)} {fmt_pct(ndp,total):>6s} | {fmt(total)}")

# =========================================================================
# Exp3: Total Energy
# =========================================================================
rows = load_csv(f"{outdir}/exp3_total_energy.csv")
bd_rows = load_csv(f"{outdir}/exp3_energy_breakdown.csv")
if rows:
    print("\n--- Exp3: Scaling DIMMs (x4, 1-16 DIMM) — Total Energy ---")
    print(f"{'Workload':>10s} {'Size':>5s} | {'1D':>12s} {'2D':>12s} {'4D':>12s} {'8D':>12s} {'16D':>12s}")
    print("-" * 82)
    for r in rows:
        vals = [fmt(r.get(f"{c}_Total(nJ)","N/A")) for c in ["1D","2D","4D","8D","16D"]]
        print(f"{r['Workload']:>10s} {r['Size']:>5s} | {vals[0]} {vals[1]} {vals[2]} {vals[3]} {vals[4]}")

# Exp3: Breakdown
if bd_rows:
    print("\n--- Exp3: Scaling DIMMs (x4, 1-16 DIMM) — Energy Breakdown ---")
    dimms = ["1D", "2D", "4D", "8D", "16D"]
    seen = []
    for r in bd_rows:
        key = (r["Workload"], r["Size"])
        if key not in seen:
            seen.append(key)
    for wl, sz in seen:
        print(f"\n  {wl} {sz}:")
        print(f"    {'DIMMs':>6s} | {'Background':>12s} {'%':>6s} | {'Command':>12s} {'%':>6s} | {'DQ':>12s} {'%':>6s} | {'NDP':>12s} {'%':>6s} | {'Total':>12s}")
        print(f"    {'-'*95}")
        for dm in dimms:
            matches = [r for r in bd_rows if r["Workload"]==wl and r["Size"]==sz and r["DIMMs"]==dm]
            if not matches: continue
            r = matches[0]
            bg, cmd, dq, ndp, total = r["Background(nJ)"], r["Command(nJ)"], r["DQ(nJ)"], r["NDP(nJ)"], r["Total(nJ)"]
            print(f"    {dm:>6s} | {fmt(bg)} {fmt_pct(bg,total):>6s} | {fmt(cmd)} {fmt_pct(cmd,total):>6s} | {fmt(dq)} {fmt_pct(dq,total):>6s} | {fmt(ndp)} {fmt_pct(ndp,total):>6s} | {fmt(total)}")

# =========================================================================
# Exp4: Total Energy
# =========================================================================
rows = load_csv(f"{outdir}/exp4_total_energy.csv")
bd_rows = load_csv(f"{outdir}/exp4_energy_breakdown.csv")
if rows:
    print("\n--- Exp4: Concurrent (SPEC Host + GEMV 128K) — Total Energy ---")
    cfgs = ["base","base-T","dbx","dbx-T","dbx-N","async-N"]
    print(f"{'Benchmark':>18s} |" + "".join(f"{c:>12s}" for c in cfgs))
    print("-" * 92)
    for r in rows:
        vals = [fmt(r.get(f"{c}_Total(nJ)","N/A")) for c in cfgs]
        print(f"{r['Benchmark']:>18s} |" + "".join(vals))

# Exp4: Breakdown
if bd_rows:
    print("\n--- Exp4: Concurrent (SPEC Host + GEMV 128K) — Energy Breakdown ---")
    cfgs = ["base","base-T","dbx","dbx-T","dbx-N","async-N"]
    benchmarks = []
    for r in bd_rows:
        b = r["Benchmark"]
        if b not in benchmarks:
            benchmarks.append(b)
    for bench in benchmarks:
        print(f"\n  {bench}:")
        print(f"    {'Config':>8s} | {'Background':>12s} {'%':>6s} | {'Command':>12s} {'%':>6s} | {'DQ':>12s} {'%':>6s} | {'NDP':>12s} {'%':>6s} | {'Total':>12s}")
        print(f"    {'-'*95}")
        for cfg in cfgs:
            matches = [r for r in bd_rows if r["Benchmark"]==bench and r["Config"]==cfg]
            if not matches: continue
            r = matches[0]
            bg, cmd, dq, ndp, total = r["Background(nJ)"], r["Command(nJ)"], r["DQ(nJ)"], r["NDP(nJ)"], r["Total(nJ)"]
            print(f"    {cfg:>8s} | {fmt(bg)} {fmt_pct(bg,total):>6s} | {fmt(cmd)} {fmt_pct(cmd,total):>6s} | {fmt(dq)} {fmt_pct(dq,total):>6s} | {fmt(ndp)} {fmt_pct(ndp,total):>6s} | {fmt(total)}")

PYEOF
}

###############################################################################
# Main
###############################################################################
echo "Extracting energy from: $LOG_DIR"
echo "Output directory: $OUTDIR"
echo ""

extract_exp1
extract_exp2
extract_exp3
extract_exp4
print_summary

echo ""
echo "=== CSV files generated in $OUTDIR/ ==="
ls -la "$OUTDIR/"*.csv 2>/dev/null
