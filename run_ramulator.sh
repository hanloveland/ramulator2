#!/usr/bin/bash

trace_list=(
"random_5M_R8W2"
"random_5M_R5W5"
"random_5M_R2W8"
"stream_5M_R8W2"
"stream_5M_R5W5"
"stream_5M_R2W8"    
)

speed_grade_list=(
"DDR5_3200AN"
"DDR5_4800B"
"DDR5_6400AN"
"DDR5_7200AN"
"DDR5_8000AN"
"DDR5_8800AN"
)

# Run Example
# ./ramulator2 -f ddr5_pch_config.yaml -p Frontend.path=./perf_comparison/traces/stream_5M_R8W2_ramulatorv2.trace

# for trace in ${trace_list[@]}; do
# trace_path="Frontend.path=./perf_comparison/traces/"${trace}"_ramulatorv2.trace"
# option="RoCoBaRaPcCh_prefetch_off"
# log_path="log/proposed_"$option"_"$trace"_.log"
# echo $trace_path
# ./ramulator2 -f ddr5_pch_config.yaml -p  $trace_path -p MemorySystem.DRAM.use_db_fetch=false > $log_path    
# done

for speed_grade in ${speed_grade_list[@]}; do
    for trace in ${trace_list[@]}; do
    trace_path="Frontend.path=./perf_comparison/traces/"${trace}"_ramulatorv2.trace"
    speed_option="MemorySystem.DRAM.timing.preset="$speed_grade
    option="RoCoBaRaPcCh_prefetch_off"
    conf_file="ddr5_pch_config.yaml"
    log_path="log_RoCoBaRa/proposed_"$speed_grade"_"$option"_"$trace"_.log"
    echo "========== Run Ramulator ============"
    echo "Trace: "$trace
    echo "Speed Graede: "$speed_grade
    echo "configuration file: "$conf_file
    echo "Option : "$option
    ./ramulator2 -f $conf_file -p  $trace_path -p $speed_option -p MemorySystem.DRAM.use_db_fetch=false > $log_path &
    done
    wait
done

for speed_grade in ${speed_grade_list[@]}; do
    for trace in ${trace_list[@]}; do
    trace_path="Frontend.path=./perf_comparison/traces/"${trace}"_ramulatorv2.trace"
    speed_option="MemorySystem.DRAM.timing.preset="$speed_grade
    option="RoCoBaRaPcCh_prefetch_on"
    conf_file="ddr5_pch_config.yaml"
    log_path="log_RoCoBaRa/proposed_"$speed_grade"_"$option"_"$trace"_.log"
    echo "========== Run Ramulator ============"
    echo "Trace: "$trace
    echo "Speed Graede: "$speed_grade
    echo "configuration file: "$conf_file
    echo "Option : "$option
    ./ramulator2 -f $conf_file -p  $trace_path -p $speed_option -p MemorySystem.DRAM.use_db_fetch=true > $log_path &
    done
    wait
done


for speed_grade in ${speed_grade_list[@]}; do
    for trace in ${trace_list[@]}; do
    trace_path="Frontend.path=./perf_comparison/traces/"${trace}"_ramulatorv2.trace"
    speed_option="MemorySystem.DRAM.timing.preset="$speed_grade
    option="RoCoBaRaCh"
    conf_file="ddr5_config.yaml"
    log_path="log_RoCoBaRa/baseline_"$speed_grade"_"$option"_"$trace"_.log"
    echo "========== Run Ramulator ============"
    echo "Trace: "$trace
    echo "Speed Graede: "$speed_grade
    echo "configuration file: "$conf_file
    echo "Option : "$option
    ./ramulator2 -f $conf_file -p  $trace_path -p $speed_option > $log_path &
    done
    wait
done

