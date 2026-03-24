#!/usr/bin/env python3
"""
Parse DBX-DIMM vs AsyncDIMM simulation logs from log_comp/ and produce CSV + summary table.

Extracted metrics:
  - Total cycles (memory_system_cycles)
  - Total NDP span (max across pCHs/ranks)
  - Bandwidth (GB/s)
  - Total DRAM Energy (nJ)
"""

import os
import re
import csv
import sys

LOG_DIR = "log_comp"
OUTPUT_CSV = "comparison_results.csv"

WORKLOADS = ["COPY", "AXPY", "AXPBY", "AXPBYPCZ", "XMY", "DOT", "GEMV"]
SIZES = ["8K", "32K", "128K", "512K", "2M", "8M"]
GEMV_SIZES = ["8K", "16K", "32K", "64K", "128K"]


def parse_log(filepath, arch):
    """Parse a single simulation log and return a dict of metrics."""
    result = {
        "total_cycles": None,
        "ndp_span_cycles": None,
        "bandwidth_gbps": None,
        "total_energy_nj": None,
    }

    if not os.path.exists(filepath):
        return result

    with open(filepath, "r") as f:
        content = f.read()

    # 1. Total cycles: memory_system_cycles
    m = re.search(r"memory_system_cycles:\s+(\d+)", content)
    if m:
        result["total_cycles"] = int(m.group(1))

    # 2. Total NDP span (max across all pCH/rank instances)
    spans = re.findall(r"Total NDP span:\s+(\d+)\s+cycles", content)
    if spans:
        result["ndp_span_cycles"] = max(int(s) for s in spans)

    # For AsyncDIMM: compute NDP span from NMA state cycles if no "Total NDP span"
    if result["ndp_span_cycles"] is None:
        # Sum non-IDLE NMA state cycles per rank, take max
        # Pattern: "  RUN:         XXXX"
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
            # Only count ranks that actually executed (span > 0)
            active_spans = [s for s in rank_spans if s > 0]
            if active_spans:
                result["ndp_span_cycles"] = max(active_spans)

    # 3. Bandwidth
    if arch == "dbx":
        # DBX-DIMM: "- Total NDP Bandwidth              : 129.087"
        m = re.search(r"Total NDP Bandwidth\s*:\s+([\d.]+)", content)
        if m:
            result["bandwidth_gbps"] = float(m.group(1))
    else:
        # AsyncDIMM: "- Total Bandwidth                  : 134.345"
        m = re.search(r"Total Bandwidth\s*:\s+([\d.]+)", content)
        if m:
            result["bandwidth_gbps"] = float(m.group(1))

    # 4. Total DRAM Energy
    m = re.search(r"Total DRAM Energy \(nJ\)\s*:\s+([\d.eE+\-]+)", content)
    if m:
        result["total_energy_nj"] = float(m.group(1))

    return result


def main():
    rows = []

    for wl in WORKLOADS:
        sizes = GEMV_SIZES if wl == "GEMV" else SIZES
        for sz in sizes:
            for arch, prefix in [("dbx", "dbx"), ("async", "async")]:
                logfile = os.path.join(LOG_DIR, f"{prefix}_{wl}_{sz}.log")
                metrics = parse_log(logfile, arch)
                arch_name = "DBX-DIMM" if arch == "dbx" else "AsyncDIMM"
                rows.append({
                    "arch": arch_name,
                    "workload": wl,
                    "size": sz,
                    "total_cycles": metrics["total_cycles"],
                    "ndp_span_cycles": metrics["ndp_span_cycles"],
                    "bandwidth_gbps": metrics["bandwidth_gbps"],
                    "total_energy_nj": metrics["total_energy_nj"],
                })

    # Write CSV
    fields = ["arch", "workload", "size", "total_cycles", "ndp_span_cycles",
              "bandwidth_gbps", "total_energy_nj"]
    with open(OUTPUT_CSV, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)

    print(f"CSV written to: {OUTPUT_CSV}")
    print(f"Total entries: {len(rows)}")
    print()

    # Print summary comparison table
    print("=" * 110)
    print(f"{'Workload':<10} {'Size':<6} | {'DBX Cycles':>12} {'Async Cycles':>12} {'Speedup':>8} | "
          f"{'DBX BW':>8} {'Async BW':>8} | {'DBX Energy':>12} {'Async Energy':>12} {'E Ratio':>8}")
    print("-" * 110)

    for wl in WORKLOADS:
        sizes = GEMV_SIZES if wl == "GEMV" else SIZES
        for sz in sizes:
            dbx = next((r for r in rows if r["arch"] == "DBX-DIMM" and r["workload"] == wl and r["size"] == sz), None)
            asc = next((r for r in rows if r["arch"] == "AsyncDIMM" and r["workload"] == wl and r["size"] == sz), None)

            if not dbx or not asc:
                continue

            dc = dbx["ndp_span_cycles"] or dbx["total_cycles"]
            ac = asc["ndp_span_cycles"] or asc["total_cycles"]
            dbw = dbx["bandwidth_gbps"]
            abw = asc["bandwidth_gbps"]
            de = dbx["total_energy_nj"]
            ae = asc["total_energy_nj"]

            speedup = f"{ac/dc:.2f}x" if dc and ac else "N/A"
            eratio = f"{ae/de:.2f}x" if de and ae else "N/A"

            dc_s = f"{dc:>12,}" if dc else f"{'N/A':>12}"
            ac_s = f"{ac:>12,}" if ac else f"{'N/A':>12}"
            dbw_s = f"{dbw:>8.2f}" if dbw else f"{'N/A':>8}"
            abw_s = f"{abw:>8.2f}" if abw else f"{'N/A':>8}"
            de_s = f"{de:>12.1f}" if de else f"{'N/A':>12}"
            ae_s = f"{ae:>12.1f}" if ae else f"{'N/A':>12}"

            print(f"{wl:<10} {sz:<6} | {dc_s} {ac_s} {speedup:>8} | "
                  f"{dbw_s} {abw_s} | {de_s} {ae_s} {eratio:>8}")

        print("-" * 110)

    print()
    print("Speedup = AsyncDIMM_cycles / DBX-DIMM_cycles (>1 means DBX-DIMM is faster)")
    print("E Ratio = AsyncDIMM_energy / DBX-DIMM_energy (>1 means DBX-DIMM is more efficient)")


if __name__ == "__main__":
    main()
