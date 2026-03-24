#!/usr/bin/bash

# cmd="./build/ramulator2 -f ddr5_baseline_ncore_config.yaml > op1_log/baseline_power.log &"
# eval $cmd 

cmd1="./build/ramulator2 -f ddr5_pch_ncore_config.yaml -p MemorySystem.Controller.prefetch_buffer_size=8 > dbx_pre_bf_sz8.log &"
cmd2="./build/ramulator2 -f ddr5_pch_ncore_config.yaml -p MemorySystem.Controller.prefetch_buffer_size=16 > dbx_pre_bf_sz16.log &"
cmd3="./build/ramulator2 -f ddr5_pch_ncore_config.yaml -p MemorySystem.Controller.prefetch_buffer_size=32 > dbx_pre_bf_sz32.log &"
eval $cmd1 
eval $cmd2 
eval $cmd3
wait

# echo "Run 8K GEMV"
# cmd1="./build/ramulator2 -f ddr5_pch_ncore_config.yaml -p Frontend.core0_trace=./multi_dimm/1dimm/pch_ndp/pch_ndp_x4_8K_GEMV.txt -p MemorySystem.DRAM.org.channel=2 > multi_dimm_log/1dimm_8K.log &"
# cmd2="./build/ramulator2 -f ddr5_pch_ncore_config.yaml -p Frontend.core0_trace=./multi_dimm/2dimm/pch_ndp/pch_ndp_x4_8K_GEMV.txt -p MemorySystem.DRAM.org.channel=4 > multi_dimm_log/2dimm_8K.log &"
# cmd3="./build/ramulator2 -f ddr5_pch_ncore_config.yaml -p Frontend.core0_trace=./multi_dimm/4dimm/pch_ndp/pch_ndp_x4_8K_GEMV.txt -p MemorySystem.DRAM.org.channel=8 > multi_dimm_log/4dimm_8K.log &"
# cmd4="./build/ramulator2 -f ddr5_pch_ncore_config.yaml -p Frontend.core0_trace=./multi_dimm/8dimm/pch_ndp/pch_ndp_x4_8K_GEMV.txt -p MemorySystem.DRAM.org.channel=16 > multi_dimm_log/8dimm_8K.log &"
# cmd5="./build/ramulator2 -f ddr5_pch_ncore_config.yaml -p Frontend.core0_trace=./multi_dimm/16dimm/pch_ndp/pch_ndp_x4_8K_GEMV.txt -p MemorySystem.DRAM.org.channel=32 > multi_dimm_log/16dimm_8K.log &"
# eval $cmd1 
# eval $cmd2 
# eval $cmd3 
# eval $cmd4 
# eval $cmd5 
# wait

# echo "Run 16K GEMV"
# cmd1="./build/ramulator2 -f ddr5_pch_ncore_config.yaml -p Frontend.core0_trace=./multi_dimm/1dimm/pch_ndp/pch_ndp_x4_16K_GEMV.txt -p MemorySystem.DRAM.org.channel=2 > multi_dimm_log/1dimm_16K.log &"
# cmd2="./build/ramulator2 -f ddr5_pch_ncore_config.yaml -p Frontend.core0_trace=./multi_dimm/2dimm/pch_ndp/pch_ndp_x4_16K_GEMV.txt -p MemorySystem.DRAM.org.channel=4 > multi_dimm_log/2dimm_16K.log &"
# cmd3="./build/ramulator2 -f ddr5_pch_ncore_config.yaml -p Frontend.core0_trace=./multi_dimm/4dimm/pch_ndp/pch_ndp_x4_16K_GEMV.txt -p MemorySystem.DRAM.org.channel=8 > multi_dimm_log/4dimm_16K.log &"
# cmd4="./build/ramulator2 -f ddr5_pch_ncore_config.yaml -p Frontend.core0_trace=./multi_dimm/8dimm/pch_ndp/pch_ndp_x4_16K_GEMV.txt -p MemorySystem.DRAM.org.channel=16 > multi_dimm_log/8dimm_16K.log &"
# cmd5="./build/ramulator2 -f ddr5_pch_ncore_config.yaml -p Frontend.core0_trace=./multi_dimm/16dimm/pch_ndp/pch_ndp_x4_16K_GEMV.txt -p MemorySystem.DRAM.org.channel=32 > multi_dimm_log/16dimm_16K.log &"
# eval $cmd1 
# eval $cmd2 
# eval $cmd3 
# eval $cmd4 
# eval $cmd5 
# wait

# echo "Run 32K GEMV"
# cmd1="./build/ramulator2 -f ddr5_pch_ncore_config.yaml -p Frontend.core0_trace=./multi_dimm/1dimm/pch_ndp/pch_ndp_x4_32K_GEMV.txt -p MemorySystem.DRAM.org.channel=2 > multi_dimm_log/1dimm_32K.log &"
# cmd2="./build/ramulator2 -f ddr5_pch_ncore_config.yaml -p Frontend.core0_trace=./multi_dimm/2dimm/pch_ndp/pch_ndp_x4_32K_GEMV.txt -p MemorySystem.DRAM.org.channel=4 > multi_dimm_log/2dimm_32K.log &"
# cmd3="./build/ramulator2 -f ddr5_pch_ncore_config.yaml -p Frontend.core0_trace=./multi_dimm/4dimm/pch_ndp/pch_ndp_x4_32K_GEMV.txt -p MemorySystem.DRAM.org.channel=8 > multi_dimm_log/4dimm_32K.log &"
# cmd4="./build/ramulator2 -f ddr5_pch_ncore_config.yaml -p Frontend.core0_trace=./multi_dimm/8dimm/pch_ndp/pch_ndp_x4_32K_GEMV.txt -p MemorySystem.DRAM.org.channel=16 > multi_dimm_log/8dimm_32K.log &"
# cmd5="./build/ramulator2 -f ddr5_pch_ncore_config.yaml -p Frontend.core0_trace=./multi_dimm/16dimm/pch_ndp/pch_ndp_x4_32K_GEMV.txt -p MemorySystem.DRAM.org.channel=32 > multi_dimm_log/16dimm_32K.log &"
# eval $cmd1 
# eval $cmd2 
# eval $cmd3 
# eval $cmd4 
# eval $cmd5 
# wait

# echo "Run 64K GEMV"
# cmd1="./build/ramulator2 -f ddr5_pch_ncore_config.yaml -p Frontend.core0_trace=./multi_dimm/1dimm/pch_ndp/pch_ndp_x4_64K_GEMV.txt -p MemorySystem.DRAM.org.channel=2 > multi_dimm_log/1dimm_64K.log &"
# cmd2="./build/ramulator2 -f ddr5_pch_ncore_config.yaml -p Frontend.core0_trace=./multi_dimm/2dimm/pch_ndp/pch_ndp_x4_64K_GEMV.txt -p MemorySystem.DRAM.org.channel=4 > multi_dimm_log/2dimm_64K.log &"
# cmd3="./build/ramulator2 -f ddr5_pch_ncore_config.yaml -p Frontend.core0_trace=./multi_dimm/4dimm/pch_ndp/pch_ndp_x4_64K_GEMV.txt -p MemorySystem.DRAM.org.channel=8 > multi_dimm_log/4dimm_64K.log &"
# cmd4="./build/ramulator2 -f ddr5_pch_ncore_config.yaml -p Frontend.core0_trace=./multi_dimm/8dimm/pch_ndp/pch_ndp_x4_64K_GEMV.txt -p MemorySystem.DRAM.org.channel=16 > multi_dimm_log/8dimm_64K.log &"
# cmd5="./build/ramulator2 -f ddr5_pch_ncore_config.yaml -p Frontend.core0_trace=./multi_dimm/16dimm/pch_ndp/pch_ndp_x4_64K_GEMV.txt -p MemorySystem.DRAM.org.channel=32 > multi_dimm_log/16dimm_64K.log &"
# eval $cmd1 
# eval $cmd2 
# eval $cmd3 
# eval $cmd4 
# eval $cmd5 
# wait

# echo "Run 128K GEMV"
# cmd1="./build/ramulator2 -f ddr5_pch_ncore_config.yaml -p Frontend.core0_trace=./multi_dimm/1dimm/pch_ndp/pch_ndp_x4_128K_GEMV.txt -p MemorySystem.DRAM.org.channel=2 > multi_dimm_log/1dimm_128K.log &"
# cmd2="./build/ramulator2 -f ddr5_pch_ncore_config.yaml -p Frontend.core0_trace=./multi_dimm/2dimm/pch_ndp/pch_ndp_x4_128K_GEMV.txt -p MemorySystem.DRAM.org.channel=4 > multi_dimm_log/2dimm_128K.log &"
# cmd3="./build/ramulator2 -f ddr5_pch_ncore_config.yaml -p Frontend.core0_trace=./multi_dimm/4dimm/pch_ndp/pch_ndp_x4_128K_GEMV.txt -p MemorySystem.DRAM.org.channel=8 > multi_dimm_log/4dimm_128K.log &"
# cmd4="./build/ramulator2 -f ddr5_pch_ncore_config.yaml -p Frontend.core0_trace=./multi_dimm/8dimm/pch_ndp/pch_ndp_x4_128K_GEMV.txt -p MemorySystem.DRAM.org.channel=16 > multi_dimm_log/8dimm_128K.log &"
# cmd5="./build/ramulator2 -f ddr5_pch_ncore_config.yaml -p Frontend.core0_trace=./multi_dimm/16dimm/pch_ndp/pch_ndp_x4_128K_GEMV.txt -p MemorySystem.DRAM.org.channel=32 > multi_dimm_log/16dimm_128K.log &"

# eval $cmd1 
# eval $cmd2 
# eval $cmd3 
# eval $cmd4 
# eval $cmd5 
# wait

# cmd="./build/ramulator2 -f ddr5_pch_ncore_config.yaml -p Frontend.core0_trace=./multi_dimm/1dimm/pch_ndp/pch_ndp_x4_128K_GEMV.txt -p MemorySystem.DRAM.org.channel=2 > multi_dimm_log/1dimm.log &"
# eval $cmd 

# wait
# cmd="./build/ramulator2 -f ddr5_pch_ncore_config.yaml -p Frontend.core0_trace=./multi_dimm/2dimm/pch_ndp/pch_ndp_x4_128K_GEMV.txt -p MemorySystem.DRAM.org.channel=4 > multi_dimm_log/2dimm.log &"
# eval $cmd 

# cmd="./build/ramulator2 -f ddr5_pch_ncore_config.yaml -p Frontend.core0_trace=./multi_dimm/4dimm/pch_ndp/pch_ndp_x4_128K_GEMV.txt -p MemorySystem.DRAM.org.channel=8 > multi_dimm_log/4dimm.log &"
# eval $cmd 

# cmd="./build/ramulator2 -f ddr5_pch_ncore_config.yaml -p Frontend.core0_trace=./multi_dimm/8dimm/pch_ndp/pch_ndp_x4_128K_GEMV.txt -p MemorySystem.DRAM.org.channel=16 > multi_dimm_log/8dimm.log &"
# eval $cmd 

# cmd="./build/ramulator2 -f ddr5_pch_ncore_config.yaml -p Frontend.core0_trace=./multi_dimm/16dimm/pch_ndp/pch_ndp_x4_128K_GEMV.txt -p MemorySystem.DRAM.org.channel=32 > multi_dimm_log/16dimm.log &"
# eval $cmd 

# cmd="./build/ramulator2 -f ddr5_baseline_ncore_config.yaml -p MemorySystem.Controller.adaptive_row_cap=128 > op1_log/baseline_cap_128.log &"
# eval $cmd 

# cmd="./build/ramulator2 -f ddr5_baseline_ncore_config.yaml -p MemorySystem.Controller.adaptive_row_cap=16 > op1_log/baseline_cap_16.log &"
# eval $cmd 

# cmd="./build/ramulator2 -f ddr5_baseline_ncore_config.yaml -p MemorySystem.Controller.adaptive_row_cap=8 > op1_log/baseline_cap_8.log &"
# eval $cmd 

# cmd="./build/ramulator2 -f ddr5_baseline_ncore_config.yaml -p MemorySystem.Controller.adaptive_row_cap=4 > op1_log/baseline_cap_4.log &"
# eval $cmd 

# cmd="./build/ramulator2 -f ddr5_pch_ncore_config.yaml -p MemorySystem.Controller.adaptive_row_cap=128 > op1_log/pch_non_ndp_cap_2_128.log &"
# eval $cmd 

# cmd="./build/ramulator2 -f ddr5_pch_ncore_config.yaml -p MemorySystem.Controller.adaptive_row_cap=32 > op1_log/pch_non_ndp_cap_2_32.log &"
# eval $cmd 

# cmd="./build/ramulator2 -f ddr5_pch_ncore_config.yaml -p MemorySystem.Controller.adaptive_row_cap=16 > op1_log/pch_non_ndp_cap_2_16.log &"
# eval $cmd 

# cmd="./build/ramulator2 -f ddr5_pch_ncore_config.yaml -p MemorySystem.Controller.adaptive_row_cap=8 > op1_log/pch_non_ndp_cap_2_8.log &"
# eval $cmd 

# cmd="./build/ramulator2 -f ddr5_pch_ncore_config.yaml -p MemorySystem.Controller.adaptive_row_cap=4 > op1_log/pch_non_ndp_cap_2_4.log &"
# eval $cmd 

# cmd="./build/ramulator2 -f ddr5_pch_ncore_config.yaml -p MemorySystem.Controller.adaptive_row_cap=2 > op1_log/pch_non_ndp_cap_2_2.log &"
# eval $cmd 


# cmd="./build/ramulator2 -f ddr5_pch_ncore_config.yaml > op1_log/pch_non_ndp_1.log &"
# conf_file="ddr5_pch_ncore_config.yaml"
# log_path="log_conexc"

# CAP_OP="MemorySystem.Controller.m_ndp_row_hit_low_cap="
# TOKEN_OP="MemorySystem.Controller.pre_rw_per_token="
# # CAP Swap 4-32
# base_token=$TOKEN_OP"16"
# cap4=$CAP_OP"4"
# cmd="./build/ramulator2 -f $conf_file -p $base_token -p $cap4 1> $log_path"/cap4_token16.out" 2> $log_path"/cap4_token16.err" &"
# eval $cmd 

# cap8=$CAP_OP"8"
# cmd="./build/ramulator2 -f $conf_file -p $base_token -p $cap8 1> $log_path"/cap8_token16.out" 2> $log_path"/cap8_token16.err" &"
# eval $cmd 

# cap16=$CAP_OP"16"
# cmd="./build/ramulator2 -f $conf_file -p $base_token -p $cap16 1> $log_path"/cap16_token16.out" 2> $log_path"/cap16_token16.err" &"
# eval $cmd 

# cap32=$CAP_OP"32"
# cmd="./build/ramulator2 -f $conf_file -p $base_token -p $cap32 1> $log_path"/cap32_token16.out" 2> $log_path"/cap32_token16.err" &"
# eval $cmd 
# # Token Swap 1-32
# base_cap=$CAP_OP"16"
# token1=$TOKEN_OP"1"
# cmd="./build/ramulator2 -f $conf_file -p $base_cap -p $token1 1> $log_path"/cap16_token1.out" 2> $log_path"/cap16_token1.err" &"
# echo $cmd
# eval $cmd 

# token2=$TOKEN_OP"2"
# cmd="./build/ramulator2 -f $conf_file -p $base_cap -p $token2 1> $log_path"/cap16_token2.out" 2> $log_path"/cap16_token2.err" &"
# echo $cmd
# eval $cmd 

# token4=$TOKEN_OP"4"
# cmd="./build/ramulator2 -f $conf_file -p $base_cap -p $token4 1> $log_path"/cap16_token4.out" 2> $log_path"/cap16_token4.err" &"
# echo $cmd
# eval $cmd 

# token8=$TOKEN_OP"8"
# cmd="./build/ramulator2 -f $conf_file -p $base_cap -p $token8 1> $log_path"/cap16_token8.out" 2> $log_path"/cap16_token8.err" &"
# echo $cmd
# eval $cmd 

# token16=$TOKEN_OP"16"
# cmd="./build/ramulator2 -f $conf_file -p $base_cap -p $token16 1> $log_path"/cap16_token16.out" 2> $log_path"/cap16_token16.err" &"
# echo $cmd
# eval $cmd 

# token32=$TOKEN_OP"32"
# cmd="./build/ramulator2 -f $conf_file -p $base_cap -p $token32 1> $log_path"/cap16_token32.out" 2> $log_path"/cap16_token32.err" &"
# echo $cmd
# eval $cmd 

# wait 