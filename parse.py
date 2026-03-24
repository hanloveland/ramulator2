#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

import argparse
import csv
import json
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Iterable, List, Dict, Any, Tuple


TRACE_HINTS_DEFAULT = [
    "_8K_AXPBY.txt",
    "_32K_AXPBY.txt", 
    "_128K_AXPBY.txt",
    "_512K_AXPBY.txt",  
    "_2M_AXPBY.txt",
    "_8M_AXPBY.txt",
    "_8K_AXPBYPCZ.txt",
    "_32K_AXPBYPCZ.txt", 
    "_128K_AXPBYPCZ.txt",
    "_512K_AXPBYPCZ.txt",  
    "_2M_AXPBYPCZ.txt",
    "_8M_AXPBYPCZ.txt",
    "_8K_AXPY.txt",
    "_32K_AXPY.txt", 
    "_128K_AXPY.txt",
    "_512K_AXPY.txt",  
    "_2M_AXPY.txt",
    "_8M_AXPY.txt",
    "_8K_COPY.txt",
    "_32K_COPY.txt", 
    "_128K_COPY.txt",
    "_512K_COPY.txt",  
    "_2M_COPY.txt",
    "_8M_COPY.txt",
    "_8K_XMY.txt",
    "_32K_XMY.txt", 
    "_128K_XMY.txt",
    "_512K_XMY.txt",  
    "_2M_XMY.txt",
    "_8M_XMY.txt",
    "_8K_DOT.txt",
    "_32K_DOT.txt", 
    "_128K_DOT.txt",
    "_512K_DOT.txt",  
    "_2M_DOT.txt",
    "_8M_DOT.txt",
    "_8K_GEMV.txt",
    "_16K_GEMV.txt",
    "_32K_GEMV.txt",
    "_64K_GEMV.txt",
    "_128K_GEMV.txt"
]

CONFIGS_WIDE = [
    "baseline",
    "pch_non_ndp_x4",
    "pch_non_ndp_x8",
    "pch_non_ndp_x16",
    "pch_ndp_x4",
    "pch_ndp_x8",
    "pch_ndp_x16",
]

NUM_RE = re.compile(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?")


@dataclass
class Rule:
    metric: str          # output metric name (e.g., memory_system_cycles)
    keyword: str         # substring to find in line
    pattern: Optional[re.Pattern]  # regex; if has group(1) use it, else last number


def parse_rule_string(s: str) -> Rule:
    """
    Format: "metric|keyword|regex"
    regex can be omitted: "metric|keyword"
    """
    parts = s.split("|")
    if len(parts) < 2:
        raise ValueError(f"Invalid --rules entry: {s} (need at least 'metric|keyword')")
    metric = parts[0].strip()
    keyword = parts[1].strip()
    regex = parts[2].strip() if len(parts) >= 3 else ""
    pat = re.compile(regex) if regex else None
    return Rule(metric=metric, keyword=keyword, pattern=pat)


def load_rules_from_json(path: Path) -> List[Rule]:
    """
    JSON format:
    {
      "rules": [
        {"metric":"memory_system_cycles","keyword":"memory_system_cycles",
         "regex":"memory_system_cycles\\s*[:=]\\s*([0-9.eE+-]+)"}
      ]
    }
    """
    data = json.loads(path.read_text(encoding="utf-8"))
    rules_data = data.get("rules", [])
    rules: List[Rule] = []
    for r in rules_data:
        metric = str(r["metric"]).strip()
        keyword = str(r["keyword"]).strip()
        regex = str(r.get("regex", "")).strip()
        pat = re.compile(regex) if regex else None
        rules.append(Rule(metric=metric, keyword=keyword, pattern=pat))
    if not rules:
        raise ValueError(f"No rules found in {path}")
    return rules


def collect_files(log_root: Path, extensions: List[str]) -> List[Path]:
    if not log_root.exists():
        raise FileNotFoundError(f"log_root not found: {log_root}")
    files = [p for p in log_root.rglob("*") if p.is_file()]
    if extensions:
        extset = set(extensions)
        files = [p for p in files if p.suffix in extset]
    return files


def infer_trace_from_path(p: Path, traces: Iterable[str]) -> str:
    s = str(p)
    for t in traces:
        if t in s:
            return t
    return "UNKNOWN_TRACE"


def infer_config_from_path(p: Path, configs: Iterable[str]) -> str:
    s = str(p)
    # prefer longer matches
    configs_sorted = sorted(configs, key=len, reverse=True)
    for c in configs_sorted:
        if c in s:
            return c
    return "UNKNOWN_CONFIG"


def extract_value_from_line(line: str, rule: Rule) -> Optional[float]:
    if rule.pattern is not None:
        m = rule.pattern.search(line)
        if not m:
            return None
        if m.lastindex and m.lastindex >= 1:
            token = m.group(1)
        else:
            nums = NUM_RE.findall(line)
            token = nums[-1] if nums else None
        if token is None:
            return None
        try:
            return float(token)
        except ValueError:
            return None

    nums = NUM_RE.findall(line)
    if not nums:
        return None
    try:
        return float(nums[-1])
    except ValueError:
        return None


def parse_metrics_in_file(path: Path, rules: List[Rule]) -> Dict[str, Optional[float]]:
    """
    For each rule, find first line containing keyword; parse value.
    Return {metric: value}.
    """
    values: Dict[str, Optional[float]] = {r.metric: None for r in rules}
    remaining = set(r.metric for r in rules)

    try:
        with path.open("r", encoding="utf-8", errors="replace") as f:
            for line in f:
                if not remaining:
                    break
                for r in rules:
                    if r.metric not in remaining:
                        continue
                    if r.keyword in line:
                        values[r.metric] = extract_value_from_line(line, r)
                        remaining.remove(r.metric)
    except OSError:
        # keep Nones
        return values

    return values


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--log_root", type=str, default="log")
    ap.add_argument("--out", type=str, default="wide_metrics.csv")

    ap.add_argument("--rules", type=str, action="append", default=[],
                    help='Rule: "metric|keyword|regex". regex optional. Can repeat.')
    ap.add_argument("--rules_file", type=str, default="",
                    help="JSON file with rules (alternative to --rules).")

    ap.add_argument("--traces", type=str, nargs="*", default=TRACE_HINTS_DEFAULT,
                    help="Trace hints used to detect trace from filename/path.")
    ap.add_argument("--extensions", type=str, nargs="*", default=[],
                    help="If set, parse only these file extensions (e.g., .log .txt).")

    ap.add_argument("--configs", type=str, nargs="*", default=CONFIGS_WIDE,
                    help="Configs for wide columns (default: baseline + pch_*_x{4,8,16}).")
    ap.add_argument("--keep_unknown", action="store_true",
                    help="Keep files even if trace/config is UNKNOWN (default: skip).")

    args = ap.parse_args()

    # Load rules
    if args.rules_file:
        rules = load_rules_from_json(Path(args.rules_file))
    else:
        if args.rules:
            rules = [parse_rule_string(s) for s in args.rules]
        else:
            # default: original keyword
            rules = [Rule(metric="memory_system_cycles", keyword="memory_system_cycles", pattern=None)]

    log_root = Path(args.log_root)
    out_path = Path(args.out)

    files = collect_files(log_root, args.extensions)

    # Data structure:
    # wide[trace][config][metric] = value
    wide: Dict[str, Dict[str, Dict[str, Optional[float]]]] = {}

    # For tie-break: store mtime for each (trace, config) to pick newest
    seen_mtime: Dict[Tuple[str, str], float] = {}

    # print(args.traces)
    # print(files)
    for p in files:
        trace = infer_trace_from_path(p, args.traces)
        config = infer_config_from_path(p, args.configs)
        if not args.keep_unknown:
            if trace == "UNKNOWN_TRACE" or config == "UNKNOWN_CONFIG":
                continue
            if config not in args.configs:
                continue

        metrics = parse_metrics_in_file(p, rules)

        # If none found at all, skip
        if all(v is None for v in metrics.values()):
            continue

        key = (trace, config)
        mtime = p.stat().st_mtime

        # pick newest file for the same (trace, config)
        if key in seen_mtime and mtime <= seen_mtime[key]:
            continue
        seen_mtime[key] = mtime

        wide.setdefault(trace, {})
        wide[trace][config] = metrics

    # Header: trace + (config.metric) columns
    metric_names = [r.metric for r in rules]
    header: List[str] = ["trace"]
    for cfg in args.configs:
        for m in metric_names:
            header.append(f"{cfg}.{m}")

    out_path.parent.mkdir(parents=True, exist_ok=True)
    ordered_traces = []
    seen = set()

    # 1) TRACE_HINTS_DEFAULT 순서대로
    for t in args.traces:
        if t in wide:
            ordered_traces.append(t)
            seen.add(t)

    # 2) 나머지 trace는 알파벳 순으로 뒤에 추가
    for t in sorted(wide.keys()):
        if t not in seen:
            ordered_traces.append(t)

    # print(ordered_traces)
    with out_path.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(header)

        for trace in ordered_traces:
            row: List[Any] = [trace]
            for cfg in args.configs:
                metrics = wide.get(trace, {}).get(cfg, {})
                for m in metric_names:
                    row.append(metrics.get(m))
            w.writerow(row)
    # with out_path.open("w", newline="", encoding="utf-8") as f:
    #     w = csv.writer(f)
    #     w.writerow(header)

    #     for trace in sorted(wide.keys()):
    #         row: List[Any] = [trace]
    #         for cfg in args.configs:
    #             metrics = wide.get(trace, {}).get(cfg, {})
    #             for m in metric_names:
    #                 row.append(metrics.get(m))
    #         w.writerow(row)

    print(f"[OK] Wrote wide CSV for {len(wide)} traces -> {out_path}")
    print(f"[INFO] Configs: {', '.join(args.configs)}")
    print(f"[INFO] Metrics: {', '.join(metric_names)}")


if __name__ == "__main__":
    main()