#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
from pathlib import Path

# ---------------------------------------------------------
# 1️⃣ 파일 이름 키워드 필터링
# ---------------------------------------------------------
def find_files(root: Path, keywords, mode="any", pattern="*"):
    matched = []
    for p in root.rglob(pattern):
        if not p.is_file():
            continue
        name = p.name
        if (mode == "any" and any(k in name for k in keywords)) or \
           (mode == "all" and all(k in name for k in keywords)):
            matched.append(p)
    return matched


# ---------------------------------------------------------
# 2️⃣ 파일 내용에서 특정 키워드 포함 라인 → 특정 컬럼 추출
# ---------------------------------------------------------
def extract_from_file(file_path: Path, keyword: str, col_idx: int, sep: str = "ws", mode="last"):
    # print(file_path.index(1))
    # print(dir(file_path))
    matches = []
    try:
        with file_path.open("r", encoding="utf-8", errors="ignore") as f:
            for line in f:
                if keyword in line:
                    if sep == "ws":
                        fields = line.strip().split()
                    else:
                        fields = line.strip().split(sep)
                    if 1 <= col_idx <= len(fields):
                        matches.append(fields[col_idx - 1])
    except Exception as e:
        print(f"⚠️  Error reading {file_path}: {e}")
        return None

    if not matches:
        return None
    if mode == "first":
        return matches[0]
    elif mode == "all":
        return ",".join(matches)
    else:
        return matches[-1]  # last

if __name__ == "__main__":
    root_path = './log_tmp'

    root = Path(root_path)
    if not root.exists():
        raise SystemExit(f" Directory not found: {root}")

    # files = find_files(root,fname_keywords,mode='all',pattern='*.txt.log')

    vec_size_list=['_8K', '_32K', '_128K', '_512K', '_2M', '_8M']
    mat_size_list=['_8K', '_16K', '_32K', '_64K', '_128K']
    vec_becch_list=['_AXPBY.txt', '_AXPBYPCZ.txt', '_AXPY.txt', '_COPY.txt', '_XMY.txt', '_DOT.txt']
    mat_bench_list=['_GEMV.txt']
    
    # base_keyword='ndp_4x'
    # base_keyword='ndp_8x'
    base_keyword='ndp_16x'
    # base_keyword='baseline'
    files = []
    for b in vec_becch_list:
        for s in vec_size_list:
            keyword=[base_keyword]
            keyword.append(b)
            keyword.append(s)
            file = find_files(root,keyword,mode='all',pattern='*.txt.log')
            if not file:
                print(f"No files matched filename keywords ({keyword}). ")
            else:
                files.append(file)
    for b in mat_bench_list:
        for s in mat_size_list:
            keyword=[base_keyword]
            keyword.append(b)
            keyword.append(s)
            file = find_files(root,keyword,mode='all',pattern='*.txt.log')
            if not file:
                print(f"No files matched filename keywords ({keyword}). ")
            else:
                files.append(file)            

    if not files:
        print("No files matched filename keywords.")
        exit(1)

    print(f"Found {len(files)} matching files.\n")
    results = []

    value_keyword='Total DRAM Power (W)'
    for f in files:
        print(f[0])
        value = extract_from_file(Path(f[0]), value_keyword, 7, sep='ws', mode="first")
        results.append((f[0].name, value))
        print(f"{f[0].name:<60} → {value}")

    print("\n=== SUMMARY ===")
    for name, val in results:
        print(f"{name}: {val}")