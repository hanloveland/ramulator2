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
    root_path = './log_blas4/'

    # root = Path(root_path)
    # if not root.exists():
    #     raise SystemExit(f" Directory not found: {root}")

    # files = find_files(root,fname_keywords,mode='all',pattern='*.txt.log')

    vec_size_list=['_8K', '_32K', '_128K', '_512K', '_2M', '_8M']
    mat_size_list=['_8K', '_16K', '_32K', '_64K', '_128K']
    vec_becch_list=['_AXPBY.txt', '_AXPBYPCZ.txt', '_AXPY.txt', '_COPY.txt', '_XMY.txt', '_DOT.txt']
    mat_bench_list=['_GEMV.txt']
    
    # base_keyword='ndp_4x'
    # base_keyword='ndp_8x'
    # base_keyword='ndp_16x'
    # base_keyword='baseline'
    # value_keyword='memory_system_cycles'
    # value_keyword='Total DRAM Power (W)'
    # column_idx=7    
    # keyword_list = [('memory_system_cycles',2), ('Total DRAM Power (W)',7), ('Total DRAM Energy (nJ)',7)]
    # keyword_list = [('DRAM Background Energy (nJ)',7), ('DRAM Command Energy (nJ)',0), ('DRAM DQ Energy (nJ)',7)]
    # keyword_list = [('avg_host_read_latency',2,"first")]
    # keyword_list = [('DB<->DRAM',3,"first")]
    # keyword_list = [('Host<->DB',3,"first")]    
    # keyword_list = [('Host<->DB/DRAM',3, "first")]    
    keyword_list = [('memory_system_cycles',2, "first")]
    # keyword_list = [('Total DRAM Power (W)',7)]
    # keyword_list = [('Total DRAM Energy (nJ)',7)]
    # keyword_list = [('NDP Ops Energy (nJ)',7)]
    # keyword_list = [('DRAM Background Energy (nJ)',7, "last")]
    # keyword_list = [('DRAM Command Energy (nJ)',7, "last")]
    # keyword_list = [('DRAM DQ Energy (nJ)',7,"last")]
    # keyword_list = [('NDP Ops Energy (nJ)',7,"last")]
    # keyword_list = [('NDP Ops Power(W)',6,"last")]
    
    
    all_result_list=[]
    keyword_array = [
        ("baseline", "baseline"),
        ("pch_non_ndp", "pch_non_ndp_x4"),
        ("pch_non_ndp", "pch_non_ndp_x8"),
        ("pch_non_ndp", "pch_non_ndp_x16"),
        ("pch_ndp", "pch_ndp_x4"),
        ("pch_ndp", "pch_ndp_x8"),
        ("pch_ndp", "pch_ndp_x16")
    ]
    for search_value_key, key_idx, search_mode in keyword_list:
        print(f"Serach Key {search_value_key}")
        for sub_path, file_keyword in keyword_array:
            files = []
            path_name = root_path + sub_path + "/"
            root = Path(path_name)
            if not root.exists():
                raise SystemExit(f" Directory not found: {root}")        
            for b in vec_becch_list:
                for s in vec_size_list:
                    keyword=[file_keyword]
                    keyword.append(b)
                    keyword.append(s)
                    file = find_files(root,keyword,mode='all',pattern='*.txt.log')
                    if not file:
                        print(f"No files matched filename keywords ({keyword}) at {path_name}.")
                    else:
                        files.append(file)
            for b in mat_bench_list:
                for s in mat_size_list:
                    keyword=[file_keyword]
                    keyword.append(b)
                    keyword.append(s)
                    file = find_files(root,keyword,mode='all',pattern='*.txt.log')
                    if not file:
                        print(f"No files matched filename keywords ({keyword}) at {path_name}.")
                    else:
                        files.append(file)            
    
            if not files:
                print("No files matched filename keywords.")
                exit(1)
    
            # print(f"Found {len(files)} matching files.\n")
            results = []
    
            for f in files:
                # print(f[0])
                value = extract_from_file(Path(f[0]), search_value_key, key_idx, sep='ws', mode=search_mode)
                results.append((f[0].name, value))
                # print(f"{f[0].name:<60} → {value}")
    
            all_result_list.append(results)
        print(f"\n=== SUMMARY ({search_value_key}) ===")
        # print(all_result_list)
        for i in range(len(all_result_list[0])):
            trace_name = all_result_list[0][i][0].split("_")        
            vec_name = trace_name[4].split(".")
            # print(f"{vec_name[0]} {trace_name[3]} {all_result_list[0][i][1]}")        
            # print(f"{vec_name[0]} {trace_name[3]} {all_result_list[0][i][1]} {all_result_list[1][i][1]} {all_result_list[2][i][1]}")     
            print(f"{vec_name[0]} {trace_name[3]} {all_result_list[0][i][1]} {all_result_list[1][i][1]} {all_result_list[2][i][1]} {all_result_list[3][i][1]} {all_result_list[4][i][1]} {all_result_list[5][i][1]} {all_result_list[6][i][1]}")        
            # print(f"{all_result_list[0][i][0]} {all_result_list[0][i][1]} {all_result_list[1][i][1]} {all_result_list[2][i][1]} {all_result_list[3][i][1]} {all_result_list[4][i][1]} {all_result_list[5][i][1]} {all_result_list[6][i][1]}")