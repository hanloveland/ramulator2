#!/usr/bin/env python3
"""
Parse DBX-DIMM vs AsyncDIMM concurrent mode (Host + NDP) simulation logs.

Compares:
  1. NDP-only baseline (from log_comp/) vs concurrent (from log_concurrent/)
  2. Host read latency during concurrent execution
  3. Bandwidth and energy
  4. Host vs NDP bandwidth breakdown

Fixed NDP: GEMV 128K
Host: SPEC CPU2006 benchmarks
"""

import os
import re
import csv
import sys

CONCURRENT_LOG_DIR = "log_concurrent"
BASELINE_LOG_DIR = "log_comp"
OUTPUT_CSV = "concurrent_results.csv"

NDP_WORKLOAD = "GEMV"
NDP_SIZE = "128K"

SPEC_BENCHMARKS = [
    "429.mcf",
    "462.libquantum",
    "433.milc",
    "470.lbm",
    "471.omnetpp",
    "403.gcc",
    "401.bzip2",
    "483.xalancbmk",
]


def parse_log(filepath, arch):
    """Parse a simulation log and return metrics dict."""
    result = {
        "total_cycles": None,
        "ndp_span_cycles": None,
        "bandwidth_gbps": None,
        "host_bandwidth_gbps": None,
        "ndp_bandwidth_gbps": None,
        "total_energy_nj": None,
        "host_reads": None,
        "host_writes": None,
        "host_completed_reads": None,
        "host_avg_read_latency": None,
        "host_p50_latency": None,
        "host_p95_latency": None,
        "host_p99_latency": None,
        # DBX-DIMM bandwidth (GB/s): Host<->DB, DB<->DRAM
        # Main window
        "bw_host_db": None, "bw_host_db_host": None,
        "bw_host_db_d2pa": None, "bw_host_db_ndp": None,
        "bw_db_dram": None, "bw_db_dram_host": None,
        "bw_db_dram_d2pa": None, "bw_db_dram_ndp": None,
        # tcore window
        "bw_tc_host_db": None, "bw_tc_host_db_host": None,
        "bw_tc_host_db_d2pa": None, "bw_tc_host_db_ndp": None,
        "bw_tc_db_dram": None, "bw_tc_db_dram_host": None,
        "bw_tc_db_dram_d2pa": None, "bw_tc_db_dram_ndp": None,
        # AsyncDIMM bandwidth (GB/s): Host<->NMA, NMA<->DRAM
        # Main window
        "bw_host_nma": None, "bw_host_nma_bypass": None, "bw_host_nma_offload": None,
        "bw_nma_dram": None, "bw_nma_dram_bypass": None,
        "bw_nma_dram_offload": None, "bw_nma_dram_nma": None,
        # tcore window
        "bw_tc_host_nma": None, "bw_tc_host_nma_bypass": None, "bw_tc_host_nma_offload": None,
        "bw_tc_nma_dram": None, "bw_tc_nma_dram_bypass": None,
        "bw_tc_nma_dram_offload": None, "bw_tc_nma_dram_nma": None
    }

    if not os.path.exists(filepath):
        return result

    with open(filepath, "r") as f:
        content = f.read()

    # 1. Total cycles
    m = re.search(r"memory_system_cycles:\s+(\d+)", content)
    if m:
        result["total_cycles"] = int(m.group(1))

    # 2. NDP span (max across all instances)
    spans = re.findall(r"Total NDP span:\s+(\d+)\s+cycles", content)
    if spans:
        result["ndp_span_cycles"] = max(int(s) for s in spans)

    # AsyncDIMM: compute from NMA state cycles if no "Total NDP span"
    if result["ndp_span_cycles"] is None:
        run_vals = re.findall(r"RUN:\s+(\d+)", content)
        bar_vals = re.findall(r"BAR:\s+(\d+)", content)
        wait_vals = re.findall(r"WAIT:\s+(\d+)", content)
        done_vals = re.findall(r"DONE:\s+(\d+)", content)
        issue_vals = re.findall(r"ISSUE_START:\s+(\d+)", content)
        if run_vals:
            n = len(run_vals)
            rank_spans = []
            for i in range(n):
                span = int(run_vals[i])
                if i < len(bar_vals):
                    span += int(bar_vals[i])
                if i < len(wait_vals):
                    span += int(wait_vals[i])
                if i < len(done_vals):
                    span += int(done_vals[i])
                if i < len(issue_vals):
                    span += int(issue_vals[i])
                rank_spans.append(span)
            active = [s for s in rank_spans if s > 0]
            if active:
                result["ndp_span_cycles"] = max(active)

    # 3. Bandwidth parsing — architecture-specific formats
    #
    # DBX-DIMM: Host<->DB (HOST/D2PA/NDP), DB<->DRAM (HOST/D2PA/NDP)
    # AsyncDIMM: Host<->NMA (Bypass/Offload), NMA<->DRAM (Bypass/Offload/NMA)

    if arch == "dbx":
        bw_patterns = [
            # Main window (DBX-DIMM)
            ("bw_host_db",       r"(?<!\w)Host<->DB\s*:\s+([\d.]+)"),
            ("bw_host_db_host",  r"(?<!\w)Host<->DB HOST\s*:\s+([\d.]+)"),
            ("bw_host_db_d2pa",  r"(?<!\w)Host<->DB D2PA\s*:\s+([\d.]+)"),
            ("bw_host_db_ndp",   r"(?<!\w)Host<->DB NDP\s*:\s+([\d.]+)"),
            ("bw_db_dram",       r"(?<!\w)DB<->DRAM\s*:\s+([\d.]+)"),
            ("bw_db_dram_host",  r"(?<!\w)DB<->DRAM HOST\s*:\s+([\d.]+)"),
            ("bw_db_dram_d2pa",  r"(?<!\w)DB<->DRAM D2PA\s*:\s+([\d.]+)"),
            ("bw_db_dram_ndp",   r"(?<!\w)DB<->DRAM NDP\s*:\s+([\d.]+)"),
            # tcore window (DBX-DIMM)
            ("bw_tc_host_db",       r"tcore Host<->DB\s*:\s+([\d.]+)"),
            ("bw_tc_host_db_host",  r"tcore Host<->DB HOST\s*:\s+([\d.]+)"),
            ("bw_tc_host_db_d2pa",  r"tcore Host<->DB D2PA\s*:\s+([\d.]+)"),
            ("bw_tc_host_db_ndp",   r"tcore Host<->DB NDP\s*:\s+([\d.]+)"),
            ("bw_tc_db_dram",       r"tcore DB<->DRAM\s*:\s+([\d.]+)"),
            ("bw_tc_db_dram_host",  r"tcore DB<->DRAM HOST\s*:\s+([\d.]+)"),
            ("bw_tc_db_dram_d2pa",  r"tcore DB<->DRAM D2PA\s*:\s+([\d.]+)"),
            ("bw_tc_db_dram_ndp",   r"tcore DB<->DRAM NDP\s*:\s+([\d.]+)"),
        ]
    else:
        bw_patterns = [
            # Main window (AsyncDIMM)
            ("bw_host_nma",         r"(?<!\w)Host <-> NMA\s*:\s+([\d.]+)"),
            ("bw_host_nma_bypass",  r"(?<!\w)Host <-> NMA: Bypass\s*:\s+([\d.]+)"),
            ("bw_host_nma_offload", r"(?<!\w)Host <-> NMA: Offload\s*:\s+([\d.]+)"),
            ("bw_nma_dram",         r"(?<!\w)NMA <-> DRAM\s*:\s+([\d.]+)"),
            ("bw_nma_dram_bypass",  r"(?<!\w)NMA <-> DRAM: Bypass\s*:\s+([\d.]+)"),
            ("bw_nma_dram_offload", r"(?<!\w)NMA <-> DRAM: Offload\s*:\s+([\d.]+)"),
            ("bw_nma_dram_nma",     r"(?<!\w)NMA <-> DRAM: NMA\s*:\s+([\d.]+)"),
            # tcore window (AsyncDIMM)
            ("bw_tc_host_nma",         r"tcore Host <-> NMA\s*:\s+([\d.]+)"),
            ("bw_tc_host_nma_bypass",  r"tcore Host <-> NMA: Bypass\s*:\s+([\d.]+)"),
            ("bw_tc_host_nma_offload", r"tcore Host <-> NMA: Offload\s*:\s+([\d.]+)"),
            ("bw_tc_nma_dram",         r"tcore NMA <-> DRAM\s*:\s+([\d.]+)"),
            ("bw_tc_nma_dram_bypass",  r"tcore NMA <-> DRAM: Bypass\s*:\s+([\d.]+)"),
            ("bw_tc_nma_dram_offload", r"tcore NMA <-> DRAM: Offload\s*:\s+([\d.]+)"),
            ("bw_tc_nma_dram_nma",     r"tcore NMA <-> DRAM: NMA\s*:\s+([\d.]+)"),
        ]
    for key, pat in bw_patterns:
        m = re.search(pat, content)
        if m:
            result[key] = float(m.group(1))

    # Populate common keys for cross-architecture comparison
    # host_bandwidth: Main window host-side bus total (non-tcore frontend traffic)
    # ndp_bandwidth:  tcore window DRAM-side bus total (NDP computation DRAM traffic)
    if arch == "dbx":
        # Host BW = Main Host<->DB, NDP BW = tcore DB<->DRAM
        if result.get("bw_host_db") is not None:
            result["host_bandwidth_gbps"] = result["bw_host_db"]
        if result.get("bw_tc_db_dram") is not None:
            result["ndp_bandwidth_gbps"] = result["bw_tc_db_dram"]
        if result.get("bw_host_db") is not None:
            result["bandwidth_gbps"] = result["bw_host_db"]
    else:
        # Host BW = Main Host<->NMA, NDP BW = tcore NMA<->DRAM
        if result.get("bw_host_nma") is not None:
            result["host_bandwidth_gbps"] = result["bw_host_nma"]
        if result.get("bw_tc_nma_dram") is not None:
            result["ndp_bandwidth_gbps"] = result["bw_tc_nma_dram"]
        if result.get("bw_host_nma") is not None:
            result["bandwidth_gbps"] = result["bw_host_nma"]

    # Fallback for older logs without structured BW format
    if result["bandwidth_gbps"] is None:
        if arch == "dbx":
            m = re.search(r"Total NDP Bandwidth\s*:\s+([\d.]+)", content)
        else:
            m = re.search(r"Total Bandwidth\s*:\s+([\d.]+)", content)
        if m:
            result["bandwidth_gbps"] = float(m.group(1))

    # 4. Energy
    m = re.search(r"Total DRAM Energy \(nJ\)\s*:\s+([\d.eE+\-]+)", content)
    if m:
        result["total_energy_nj"] = float(m.group(1))

    # 5. Host access stats (from Frontend report)
    m = re.search(r"Core 0:.*?Reads=(\d+),\s*Writes=(\d+),\s*Completed_Reads=(\d+),\s*Avg_Read_Latency=([\d.]+)", content)
    if m:
        result["host_reads"] = int(m.group(1))
        result["host_writes"] = int(m.group(2))
        result["host_completed_reads"] = int(m.group(3))
        result["host_avg_read_latency"] = float(m.group(4))

    # 6. Latency percentiles (from Read Latency Report)
    m = re.search(r"READ latency samples:\s+(\d+)", content)
    if m and int(m.group(1)) > 0:
        m_avg = re.search(r"avg:\s+([\d.]+)\s+cycles", content)
        if m_avg:
            result["host_avg_read_latency"] = float(m_avg.group(1))
        m_p50 = re.search(r"p50:\s+(\d+)\s+cycles", content)
        if m_p50:
            result["host_p50_latency"] = int(m_p50.group(1))
        m_p95 = re.search(r"p95:\s+(\d+)\s+cycles", content)
        if m_p95:
            result["host_p95_latency"] = int(m_p95.group(1))
        m_p99 = re.search(r"p99:\s+(\d+)\s+cycles", content)
        if m_p99:
            result["host_p99_latency"] = int(m_p99.group(1))

    return result


def fmt_int(v, w=12):
    return f"{v:>{w},}" if v is not None else f"{'N/A':>{w}}"

def fmt_float(v, w=10, d=2):
    return f"{v:>{w}.{d}f}" if v is not None else f"{'N/A':>{w}}"

def fmt_ratio(a, b, w=8):
    if a and b and b != 0:
        return f"{a/b:.2f}x"
    return "N/A"


def main():
    rows = []

    # Parse concurrent results
    for bench in SPEC_BENCHMARKS:
        for arch, prefix in [("dbx", "dbx"), ("async", "async")]:
            logfile = os.path.join(CONCURRENT_LOG_DIR, f"{prefix}_concurrent_{bench}.log")
            metrics = parse_log(logfile, arch)
            arch_name = "DBX-DIMM" if arch == "dbx" else "AsyncDIMM"
            rows.append({
                "mode": "concurrent",
                "arch": arch_name,
                "host_bench": bench,
                **metrics,
            })

    # Parse NDP-only baselines (GEMV 128K from log_comp/)
    for arch, prefix in [("dbx", "dbx"), ("async", "async")]:
        logfile = os.path.join(BASELINE_LOG_DIR, f"{prefix}_{NDP_WORKLOAD}_{NDP_SIZE}.log")
        metrics = parse_log(logfile, arch)
        arch_name = "DBX-DIMM" if arch == "dbx" else "AsyncDIMM"
        rows.append({
            "mode": "ndp_only",
            "arch": arch_name,
            "host_bench": "none",
            **metrics,
        })

    # Write CSV
    fields = ["mode", "arch", "host_bench", "total_cycles", "ndp_span_cycles",
              "bandwidth_gbps", "host_bandwidth_gbps", "ndp_bandwidth_gbps",
              "total_energy_nj",
              "host_reads", "host_completed_reads",
              "host_avg_read_latency", "host_p50_latency", "host_p95_latency", "host_p99_latency",
              # DBX-DIMM bandwidth
              "bw_host_db", "bw_host_db_host", "bw_host_db_d2pa", "bw_host_db_ndp",
              "bw_db_dram", "bw_db_dram_host", "bw_db_dram_d2pa", "bw_db_dram_ndp",
              "bw_tc_host_db", "bw_tc_host_db_host", "bw_tc_host_db_d2pa", "bw_tc_host_db_ndp",
              "bw_tc_db_dram", "bw_tc_db_dram_host", "bw_tc_db_dram_d2pa", "bw_tc_db_dram_ndp",
              # AsyncDIMM bandwidth
              "bw_host_nma", "bw_host_nma_bypass", "bw_host_nma_offload",
              "bw_nma_dram", "bw_nma_dram_bypass", "bw_nma_dram_offload", "bw_nma_dram_nma",
              "bw_tc_host_nma", "bw_tc_host_nma_bypass", "bw_tc_host_nma_offload",
              "bw_tc_nma_dram", "bw_tc_nma_dram_bypass", "bw_tc_nma_dram_offload", "bw_tc_nma_dram_nma"]
    with open(OUTPUT_CSV, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields, extrasaction='ignore')
        writer.writeheader()
        writer.writerows(rows)

    print(f"CSV written to: {OUTPUT_CSV}")
    print(f"Total entries: {len(rows)}")
    print()

    # ============================================================
    # Table 1: NDP-Only Baseline
    # ============================================================
    dbx_base = next((r for r in rows if r["mode"] == "ndp_only" and r["arch"] == "DBX-DIMM"), None)
    async_base = next((r for r in rows if r["mode"] == "ndp_only" and r["arch"] == "AsyncDIMM"), None)

    print("=" * 90)
    print(f"  NDP-Only Baseline: {NDP_WORKLOAD} {NDP_SIZE}")
    print("-" * 90)
    print(f"  {'Arch':<12} {'NDP Span':>14} {'Bandwidth':>10} {'Energy':>14}")
    print("-" * 90)
    if dbx_base:
        dc = dbx_base["ndp_span_cycles"] or dbx_base["total_cycles"]
        print(f"  {'DBX-DIMM':<12} {fmt_int(dc, 14)} {fmt_float(dbx_base['bandwidth_gbps'])} {fmt_float(dbx_base['total_energy_nj'], 14, 0)}")
    if async_base:
        ac = async_base["ndp_span_cycles"] or async_base["total_cycles"]
        print(f"  {'AsyncDIMM':<12} {fmt_int(ac, 14)} {fmt_float(async_base['bandwidth_gbps'])} {fmt_float(async_base['total_energy_nj'], 14, 0)}")
    print("=" * 90)
    print()

    # ============================================================
    # Table 2: Concurrent Mode — NDP Performance Impact
    # ============================================================
    dbx_base_span = (dbx_base["ndp_span_cycles"] or dbx_base["total_cycles"]) if dbx_base else None
    async_base_span = (async_base["ndp_span_cycles"] or async_base["total_cycles"]) if async_base else None

    print("=" * 120)
    print(f"  Concurrent Mode: NDP ({NDP_WORKLOAD} {NDP_SIZE}) Performance with Host Access")
    print("-" * 120)
    print(f"  {'Benchmark':<16} | {'DBX NDP Span':>14} {'Slowdown':>10} | {'Async NDP Span':>14} {'Slowdown':>10} | {'DBX vs Async':>12}")
    print("-" * 120)

    for bench in SPEC_BENCHMARKS:
        dbx = next((r for r in rows if r["mode"] == "concurrent" and r["arch"] == "DBX-DIMM"
                     and r["host_bench"] == bench), None)
        asc = next((r for r in rows if r["mode"] == "concurrent" and r["arch"] == "AsyncDIMM"
                     and r["host_bench"] == bench), None)

        if not dbx and not asc:
            continue

        dc = (dbx["ndp_span_cycles"] or dbx["total_cycles"]) if dbx else None
        ac = (asc["ndp_span_cycles"] or asc["total_cycles"]) if asc else None

        dbx_slow = fmt_ratio(dc, dbx_base_span)
        async_slow = fmt_ratio(ac, async_base_span)

        # Compare: AsyncDIMM span / DBX-DIMM span (>1 = DBX faster)
        compare = fmt_ratio(ac, dc)

        print(f"  {bench:<16} | {fmt_int(dc, 14)} {dbx_slow:>10} | {fmt_int(ac, 14)} {async_slow:>10} | {compare:>12}")

    print("-" * 120)
    print("  Slowdown = concurrent_NDP_span / ndp_only_NDP_span (>1x = slower due to host interference)")
    print()

    # ============================================================
    # Table 3: Concurrent Mode — Host Read Latency
    # ============================================================
    print("=" * 130)
    print(f"  Concurrent Mode: Host Read Latency (cycles) with NDP ({NDP_WORKLOAD} {NDP_SIZE})")
    print("-" * 130)
    print(f"  {'Benchmark':<16} | {'DBX Avg':>10} {'DBX P50':>10} {'DBX P99':>10} {'DBX Reads':>10} |"
          f" {'Async Avg':>10} {'Async P50':>10} {'Async P99':>10} {'Async Reads':>10}")
    print("-" * 130)

    for bench in SPEC_BENCHMARKS:
        dbx = next((r for r in rows if r["mode"] == "concurrent" and r["arch"] == "DBX-DIMM"
                     and r["host_bench"] == bench), None)
        asc = next((r for r in rows if r["mode"] == "concurrent" and r["arch"] == "AsyncDIMM"
                     and r["host_bench"] == bench), None)

        if not dbx and not asc:
            continue

        d_avg = fmt_float(dbx["host_avg_read_latency"] if dbx else None, 10, 1)
        d_p50 = fmt_int(dbx["host_p50_latency"] if dbx else None, 10)
        d_p99 = fmt_int(dbx["host_p99_latency"] if dbx else None, 10)
        d_rds = fmt_int(dbx["host_completed_reads"] if dbx else None, 10)

        a_avg = fmt_float(asc["host_avg_read_latency"] if asc else None, 10, 1)
        a_p50 = fmt_int(asc["host_p50_latency"] if asc else None, 10)
        a_p99 = fmt_int(asc["host_p99_latency"] if asc else None, 10)
        a_rds = fmt_int(asc["host_completed_reads"] if asc else None, 10)

        print(f"  {bench:<16} | {d_avg} {d_p50} {d_p99} {d_rds} | {a_avg} {a_p50} {a_p99} {a_rds}")

    print("-" * 130)
    print()

    # ============================================================
    # Table 4: Energy Comparison
    # ============================================================
    print("=" * 100)
    print(f"  Concurrent Mode: Energy Comparison (nJ)")
    print("-" * 100)
    print(f"  {'Benchmark':<16} | {'DBX Energy':>14} {'Async Energy':>14} {'Ratio':>8} |"
          f" {'DBX BW':>10} {'Async BW':>10}")
    print("-" * 100)

    for bench in SPEC_BENCHMARKS:
        dbx = next((r for r in rows if r["mode"] == "concurrent" and r["arch"] == "DBX-DIMM"
                     and r["host_bench"] == bench), None)
        asc = next((r for r in rows if r["mode"] == "concurrent" and r["arch"] == "AsyncDIMM"
                     and r["host_bench"] == bench), None)

        if not dbx and not asc:
            continue

        de = dbx["total_energy_nj"] if dbx else None
        ae = asc["total_energy_nj"] if asc else None
        eratio = fmt_ratio(ae, de)

        print(f"  {bench:<16} | {fmt_float(de, 14, 0)} {fmt_float(ae, 14, 0)} {eratio:>8} |"
              f" {fmt_float(dbx['bandwidth_gbps'] if dbx else None, 10)} {fmt_float(asc['bandwidth_gbps'] if asc else None, 10)}")

    print("-" * 100)
    print("  E Ratio = AsyncDIMM_energy / DBX-DIMM_energy")
    print()

    # ============================================================
    # Table 5: Bandwidth Breakdown per benchmark (tcore window)
    # ============================================================
    # DBX-DIMM counter labels (tcore)
    dbx_bw_keys = [
        ("bw_tc_host_db",      "Host<->DB"),
        ("bw_tc_host_db_host", "  HOST"),
        ("bw_tc_host_db_d2pa", "  D2PA"),
        ("bw_tc_host_db_ndp",  "  NDP"),
        ("bw_tc_db_dram",      "DB<->DRAM"),
        ("bw_tc_db_dram_host", "  HOST"),
        ("bw_tc_db_dram_d2pa", "  D2PA"),
        ("bw_tc_db_dram_ndp",  "  NDP"),
    ]
    # AsyncDIMM counter labels (tcore)
    async_bw_keys = [
        ("bw_tc_host_nma",         "Host<->NMA"),
        ("bw_tc_host_nma_bypass",  "  Bypass"),
        ("bw_tc_host_nma_offload", "  Offload"),
        ("bw_tc_nma_dram",         "NMA<->DRAM"),
        ("bw_tc_nma_dram_bypass",  "  Bypass"),
        ("bw_tc_nma_dram_offload", "  Offload"),
        ("bw_tc_nma_dram_nma",     "  NMA"),
    ]

    for bench in SPEC_BENCHMARKS:
        dbx = next((r for r in rows if r["mode"] == "concurrent" and r["arch"] == "DBX-DIMM"
                     and r["host_bench"] == bench), None)
        asc = next((r for r in rows if r["mode"] == "concurrent" and r["arch"] == "AsyncDIMM"
                     and r["host_bench"] == bench), None)

        if not dbx and not asc:
            continue

        print("=" * 90)
        print(f"  Bandwidth Breakdown: {bench} + {NDP_WORKLOAD} {NDP_SIZE} (tcore window, GB/s)")
        print("-" * 90)
        print(f"  {'DBX-DIMM':<20} {'GB/s':>10}   |   {'AsyncDIMM':<20} {'GB/s':>10}")
        print("-" * 90)

        max_rows = max(len(dbx_bw_keys), len(async_bw_keys))
        for i in range(max_rows):
            # DBX-DIMM column
            if i < len(dbx_bw_keys):
                dk, dl = dbx_bw_keys[i]
                d_val = fmt_float(dbx[dk] if dbx else None, 10)
                d_label = f"  {dl:<20} {d_val}"
            else:
                d_label = " " * 32

            # AsyncDIMM column
            if i < len(async_bw_keys):
                ak, al = async_bw_keys[i]
                a_val = fmt_float(asc[ak] if asc else None, 10)
                a_label = f"{al:<20} {a_val}"
            else:
                a_label = ""

            print(f"{d_label}   |   {a_label}")

        print("-" * 90)
        print()

    # ============================================================
    # Table 5b: Bandwidth Summary (Host BW vs NDP BW)
    # ============================================================
    print("=" * 140)
    print(f"  Bandwidth Summary (tcore window, GB/s): Host BW vs NDP BW")
    print("-" * 140)
    print(f"  {'Benchmark':<16} | {'DBX Host':>10} {'DBX NDP':>10} {'DBX Total':>10} |"
          f" {'Async Host':>10} {'Async NDP':>10} {'Async Tot':>10} |"
          f" {'Host Ratio':>10} {'NDP Ratio':>10}")
    print("-" * 140)

    for bench in SPEC_BENCHMARKS:
        dbx = next((r for r in rows if r["mode"] == "concurrent" and r["arch"] == "DBX-DIMM"
                     and r["host_bench"] == bench), None)
        asc = next((r for r in rows if r["mode"] == "concurrent" and r["arch"] == "AsyncDIMM"
                     and r["host_bench"] == bench), None)

        if not dbx and not asc:
            continue

        # DBX: Host BW = Main Host<->DB, NDP BW = tcore DB<->DRAM
        d_hbw = dbx["bw_host_db"] if dbx else None
        d_nbw = dbx["bw_tc_db_dram"] if dbx else None
        d_tbw = dbx["bw_tc_host_db"] if dbx else None
        # Async: Host BW = Main Host<->NMA, NDP BW = tcore NMA<->DRAM
        a_hbw = asc["bw_host_nma"] if asc else None
        a_nbw = asc["bw_tc_nma_dram"] if asc else None
        a_tbw = asc["bw_tc_host_nma"] if asc else None

        h_ratio = fmt_ratio(a_hbw, d_hbw, 10)
        n_ratio = fmt_ratio(a_nbw, d_nbw, 10)

        print(f"  {bench:<16} | {fmt_float(d_hbw, 10)} {fmt_float(d_nbw, 10)} {fmt_float(d_tbw, 10)} |"
              f" {fmt_float(a_hbw, 10)} {fmt_float(a_nbw, 10)} {fmt_float(a_tbw, 10)} |"
              f" {h_ratio:>10} {n_ratio:>10}")

    print("-" * 140)
    print("  DBX: Host BW = Main Host<->DB, NDP BW = tcore DB<->DRAM")
    print("  Async: Host BW = Main Host<->NMA, NDP BW = tcore NMA<->DRAM")
    print("  Ratio = AsyncDIMM / DBX-DIMM (>1x = AsyncDIMM higher)")
    print()

    # ============================================================
    # Table 6: Memory System Cycles Comparison
    # ============================================================
    print("=" * 100)
    print(f"  Concurrent Mode: Memory System Cycles")
    print("-" * 100)
    print(f"  {'Benchmark':<16} | {'DBX Cycles':>14} {'Async Cycles':>14} {'Ratio':>8} |"
          f" {'DBX NDP Span':>14} {'Async NDP Span':>14}")
    print("-" * 100)

    for bench in SPEC_BENCHMARKS:
        dbx = next((r for r in rows if r["mode"] == "concurrent" and r["arch"] == "DBX-DIMM"
                     and r["host_bench"] == bench), None)
        asc = next((r for r in rows if r["mode"] == "concurrent" and r["arch"] == "AsyncDIMM"
                     and r["host_bench"] == bench), None)

        if not dbx and not asc:
            continue

        dc = dbx["total_cycles"] if dbx else None
        ac = asc["total_cycles"] if asc else None
        cratio = fmt_ratio(ac, dc)
        d_span = (dbx["ndp_span_cycles"] or dbx["total_cycles"]) if dbx else None
        a_span = (asc["ndp_span_cycles"] or asc["total_cycles"]) if asc else None

        print(f"  {bench:<16} | {fmt_int(dc, 14)} {fmt_int(ac, 14)} {cratio:>8} |"
              f" {fmt_int(d_span, 14)} {fmt_int(a_span, 14)}")

    # Also show baselines
    if dbx_base or async_base:
        print("-" * 100)
        dc = dbx_base["total_cycles"] if dbx_base else None
        ac = async_base["total_cycles"] if async_base else None
        cratio = fmt_ratio(ac, dc)
        d_span = (dbx_base["ndp_span_cycles"] or dbx_base["total_cycles"]) if dbx_base else None
        a_span = (async_base["ndp_span_cycles"] or async_base["total_cycles"]) if async_base else None
        print(f"  {'NDP-Only':.<16} | {fmt_int(dc, 14)} {fmt_int(ac, 14)} {cratio:>8} |"
              f" {fmt_int(d_span, 14)} {fmt_int(a_span, 14)}")

    print("-" * 100)
    print("  Ratio = AsyncDIMM_cycles / DBX-DIMM_cycles (>1x = AsyncDIMM slower)")
    print()


if __name__ == "__main__":
    main()
