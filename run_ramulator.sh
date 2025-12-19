#!/usr/bin/bash

trace_list=(
"stream_5M_R2W8"    
"random_5M_R8W2"
"random_5M_R5W5"
"random_5M_R2W8"
"stream_5M_R8W2"
"stream_5M_R5W5"
)

# "DDR5_3200AN"
# "DDR5_6400AN"
# "DDR5_7200AN"
# "DDR5_8000AN"
# "DDR5_8800AN"
speed_grade_list=(
"DDR5_4800B"
)

dbx_org_list=(
"DDR5_16Gb_DBX_x4"
)
# "DDR5_16Gb_DBX_x8"
# "DDR5_16Gb_DBX_x16"

ndp_trace_list=(
"ndp_128K_AXPBY.txt"  
"ndp_32K_AXPBY.txt"  
"ndp_512K_AXPBY.txt"
"ndp_64K_AXPBY.txt"  
"ndp_8K_AXPBY.txt" 
"ndp_8M_AXPBY.txt"
)

# AXPBY, AXPBYPCZ,  AXPY, COPY, XMY, DOT, GEMV
trace_list=(
"8K_AXPBY.txt"
"32K_AXPBY.txt" 
"128K_AXPBY.txt"
"512K_AXPBY.txt"  
"2M_AXPBY.txt"
"8M_AXPBY.txt"
"8K_AXPBYPCZ.txt"
"32K_AXPBYPCZ.txt" 
"128K_AXPBYPCZ.txt"
"512K_AXPBYPCZ.txt"  
"2M_AXPBYPCZ.txt"
"8M_AXPBYPCZ.txt"
"8K_AXPY.txt"
"32K_AXPY.txt" 
"128K_AXPY.txt"
"512K_AXPY.txt"  
"2M_AXPY.txt"
"8M_AXPY.txt"
"8K_COPY.txt"
"32K_COPY.txt" 
"128K_COPY.txt"
"512K_COPY.txt"  
"2M_COPY.txt"
"8M_COPY.txt"
"8K_XMY.txt"
"32K_XMY.txt" 
"128K_XMY.txt"
"512K_XMY.txt"  
"2M_XMY.txt"
"8M_XMY.txt"
"8K_DOT.txt"
"32K_DOT.txt" 
"128K_DOT.txt"
"512K_DOT.txt"  
"2M_DOT.txt"
"8M_DOT.txt"
"8K_GEMV.txt"
"16K_GEMV.txt"
"32K_GEMV.txt"
"64K_GEMV.txt"
"128K_GEMV.txt"
)

address_mapping_list=(
"ChRaBaRoCo"
"RoCoBaRaCh"
"RoBaRaCoCh"
"RoCoRaBaCh"
"RoRaCoBaCh"
"RoRaBaCoCh"
)

dbx_address_mapping_list=(
# "RoBaRaCoPcCh"
# "RoCoBaRaPcCh"
"RoRaCoBaPcCh"
# "RoRabkCoBgPcCh"
)

LOG_PATH="./log_tmp"
# Run Example
# ./ramulator2 -f ddr5_pch_config.yaml -p Frontend.path=./perf_comparison/traces/stream_5M_R8W2_ramulatorv2.trace

# Remove exist log folder and make log folder
if [ -d $LOG_PATH ]; then 
    echo "exist log folder and remove log folder!!"
    rm -rf $LOG_PATH
    echo "make New $LOG_PATH folder"
else
    echo "Does not exist log folder and make new log folder!!"

fi

#TRACE TYPE
BASELINE="baseline"
PCHNDP="pch_ndp"
PCHNONNDP="pch_non_ndp"
mkdir $LOG_PATH
mkdir $LOG_PATH/$BASELINE
mkdir $LOG_PATH/$PCHNDP
mkdir $LOG_PATH/$PCHNONNDP
## Nomral Access Test ##

pch_trace_type_list=("ndp" "non_ndp")

num_process=0
echo "====================================="
echo "====================================="
for speed_grade in ${speed_grade_list[@]}; do
    echo " -- Speed Grade: "$speed_grade
    for trace_type in ${pch_trace_type_list[@]}; do
        trace_prefix="${PCHNONNDP}/${PCHNONNDP}"
        trace_option="Frontend.core0_is_ndp_trace=false"
        log_prefix="${PCHNONNDP}"
        if [[ $trace_type == "ndp" ]]; then
            trace_prefix="${PCHNDP}/${PCHNDP}"
            trace_option="Frontend.core0_is_ndp_trace=true"
            log_prefix="${PCHNDP}"
        fi             
        for dbx_org in ${dbx_org_list[@]}; do
            echo " -- DBX Organization: "$dbx_org
            dbx_org_option="MemorySystem.DRAM.org.preset="$dbx_org            
            echo "====================================="
            echo "====================================="
            real_dq="4"
            current_preset="DDR5_4800x4"
            trace_prefix_name="${trace_prefix}_x4"
            log_prefix_name="${log_prefix}_x4"
            if [[ $dbx_org == "DDR5_16Gb_DBX_x8" ]]; then 
                real_dq="8"
                current_preset="DDR5_4800x8"
                trace_prefix_name="${trace_prefix}_x8"
                log_prefix_name="${log_prefix}_x8"
            elif [[ $dbx_org == "DDR5_16Gb_DBX_x16" ]]; then 
                real_dq="16"
                current_preset="DDR5_4800x16"
                trace_prefix_name="${trace_prefix}_x16"
                log_prefix_name="${log_prefix}_16"
            fi
            speed_option="MemorySystem.DRAM.timing.preset="$speed_grade
            real_dq_option="MemorySystem.DRAM.org.real_dq="$real_dq
            current_option="MemorySystem.DRAM.current.preset="$current_preset
            conf_file="ddr5_pch_ncore_config.yaml"
            echo " -- configuration file: "$conf_file
            echo " -- DBX ORG : "$dbx_org_option
            echo " -- Real DQ on DIMM : "$real_dq_option
            echo " -- Current: "$current_option
            for trace in ${trace_list[@]}; do
                trace_path="Frontend.core0_trace=./trace/${trace_prefix_name}_${trace}"
                log_path="${LOG_PATH}/$log_prefix/$speed_grade_${log_prefix_name}_${trace}.log"
                echo "========== Run Ramulator ============"
                echo "Trace: "$trace
                cmd="./build/ramulator2 -f $conf_file -p  $trace_path -p $trace_option -p $speed_option -p $dbx_org_option -p $real_dq_option -p $current_option > $log_path &"
                echo $cmd
                eval $cmd
                ((num_process++))
                if [[ $num_process -ge 32 ]]; then 
                    wait -n
                    ((num_process--))
                fi
             done
        echo "====================================="
        echo "====================================="        
        done
    done
done
wait

echo "====================================="
echo "====================================="
num_process=0
for speed_grade in ${speed_grade_list[@]}; do
    echo " -- Speed Grade: "$speed_grade
    for trace in ${non_ndp_trace_list[@]}; do
        conf_file="ddr5_config.yaml"
        echo " -- configuration file: "$conf_file
        trace_path="Frontend.path=./trace/non_ndp/"${trace}
        log_path="${LOG_PATH}/non_ndp/${speed_grade}_${trace}.log"
        echo "========== Run Ramulator ============"
        echo "Trace: "$trace
        cmd="./ramulator2 -f $conf_file -p  $trace_path > $log_path &"
        echo $cmd
        # eval $cmd
        echo "====================================="
        echo "====================================="   
        ((num_process++))
        if [[ $num_process == 32 ]]; then 
            num_process=0
            wait
        fi
    done
done

ndp_8x_trace_list=(
"ndp_8x_8K_AXPBY.txt"
"ndp_8x_32K_AXPBY.txt" 
"ndp_8x_128K_AXPBY.txt"
"ndp_8x_512K_AXPBY.txt"  
"ndp_8x_2M_AXPBY.txt"
"ndp_8x_8M_AXPBY.txt"
"ndp_8x_8K_AXPBYPCZ.txt"
"ndp_8x_32K_AXPBYPCZ.txt" 
"ndp_8x_128K_AXPBYPCZ.txt"
"ndp_8x_512K_AXPBYPCZ.txt"  
"ndp_8x_2M_AXPBYPCZ.txt"
"ndp_8x_8M_AXPBYPCZ.txt"
"ndp_8x_8K_AXPY.txt"
"ndp_8x_32K_AXPY.txt" 
"ndp_8x_128K_AXPY.txt"
"ndp_8x_512K_AXPY.txt"  
"ndp_8x_2M_AXPY.txt"
"ndp_8x_8M_AXPY.txt"
"ndp_8x_8K_COPY.txt"
"ndp_8x_32K_COPY.txt" 
"ndp_8x_128K_COPY.txt"
"ndp_8x_512K_COPY.txt"  
"ndp_8x_2M_COPY.txt"
"ndp_8x_8M_COPY.txt"
"ndp_8x_8K_XMY.txt"
"ndp_8x_32K_XMY.txt" 
"ndp_8x_128K_XMY.txt"
"ndp_8x_512K_XMY.txt"  
"ndp_8x_2M_XMY.txt"
"ndp_8x_8M_XMY.txt"
"ndp_8x_8K_DOT.txt"
"ndp_8x_32K_DOT.txt" 
"ndp_8x_128K_DOT.txt"
"ndp_8x_512K_DOT.txt"  
"ndp_8x_2M_DOT.txt"
"ndp_8x_8M_DOT.txt"
"ndp_8x_8K_GEMV.txt"
"ndp_8x_16K_GEMV.txt"
"ndp_8x_32K_GEMV.txt"
"ndp_8x_64K_GEMV.txt"
"ndp_8x_128K_GEMV.txt"
)


ndp_16x_trace_list=(
"ndp_16x_8K_AXPBY.txt"
"ndp_16x_32K_AXPBY.txt" 
"ndp_16x_128K_AXPBY.txt"
"ndp_16x_512K_AXPBY.txt"  
"ndp_16x_2M_AXPBY.txt"
"ndp_16x_8M_AXPBY.txt"
"ndp_16x_8K_AXPBYPCZ.txt"
"ndp_16x_32K_AXPBYPCZ.txt" 
"ndp_16x_128K_AXPBYPCZ.txt"
"ndp_16x_512K_AXPBYPCZ.txt"  
"ndp_16x_2M_AXPBYPCZ.txt"
"ndp_16x_8M_AXPBYPCZ.txt"
"ndp_16x_8K_AXPY.txt"
"ndp_16x_32K_AXPY.txt" 
"ndp_16x_128K_AXPY.txt"
"ndp_16x_512K_AXPY.txt"  
"ndp_16x_2M_AXPY.txt"
"ndp_16x_8M_AXPY.txt"
"ndp_16x_8K_COPY.txt"
"ndp_16x_32K_COPY.txt" 
"ndp_16x_128K_COPY.txt"
"ndp_16x_512K_COPY.txt"  
"ndp_16x_2M_COPY.txt"
"ndp_16x_8M_COPY.txt"
"ndp_16x_8K_XMY.txt"
"ndp_16x_32K_XMY.txt" 
"ndp_16x_128K_XMY.txt"
"ndp_16x_512K_XMY.txt"  
"ndp_16x_2M_XMY.txt"
"ndp_16x_8M_XMY.txt"
"ndp_16x_8K_DOT.txt"
"ndp_16x_32K_DOT.txt" 
"ndp_16x_128K_DOT.txt"
"ndp_16x_512K_DOT.txt"  
"ndp_16x_2M_DOT.txt"
"ndp_16x_8M_DOT.txt"
"ndp_16x_8K_GEMV.txt"
"ndp_16x_16K_GEMV.txt"
"ndp_16x_32K_GEMV.txt"
"ndp_16x_64K_GEMV.txt"
"ndp_16x_128K_GEMV.txt"
)

non_ndp_trace_list=(
"baseline_8K_AXPBY.txt" 
"baseline_32K_AXPBY.txt"
"baseline_128K_AXPBY.txt" 
"baseline_512K_AXPBY.txt" 
"baseline_2M_AXPBY.txt"
"baseline_8M_AXPBY.txt"
"baseline_8K_AXPBYPCZ.txt" 
"baseline_32K_AXPBYPCZ.txt"
"baseline_128K_AXPBYPCZ.txt" 
"baseline_512K_AXPBYPCZ.txt" 
"baseline_2M_AXPBYPCZ.txt"
"baseline_8M_AXPBYPCZ.txt"
"baseline_8K_DOT.txt" 
"baseline_32K_DOT.txt"
"baseline_128K_DOT.txt" 
"baseline_512K_DOT.txt" 
"baseline_2M_DOT.txt"
"baseline_8M_DOT.txt"
"baseline_8K_COPY.txt" 
"baseline_32K_COPY.txt"
"baseline_128K_COPY.txt" 
"baseline_512K_COPY.txt" 
"baseline_2M_COPY.txt"
"baseline_8M_COPY.txt"
"baseline_8K_AXPY.txt" 
"baseline_32K_AXPY.txt"
"baseline_128K_AXPY.txt" 
"baseline_512K_AXPY.txt" 
"baseline_2M_AXPY.txt"
"baseline_8M_AXPY.txt"
"baseline_8K_XMY.txt" 
"baseline_32K_XMY.txt"
"baseline_128K_XMY.txt" 
"baseline_512K_XMY.txt" 
"baseline_2M_XMY.txt"
"baseline_8M_XMY.txt"
"baseline_8K_GEMV.txt"
"baseline_16K_GEMV.txt"
"baseline_32K_GEMV.txt"
"baseline_128K_GEMV.txt"
"baseline_64K_GEMV.txt"
)

# for speed_grade in ${speed_grade_list[@]}; do
#     for trace in ${non_ndp_trace_list[@]}; do
#     trace_path="Frontend.path=./trace/non_ndp/"${trace}
#     speed_option="MemorySystem.DRAM.timing.preset="$speed_grade
#     option="RoCoBaRaCh"
#     conf_file="ddr5_config.yaml"
#     log_path="log/baseline_"$speed_grade"_"$option"_"$trace"_.log"
#     echo "========== Run Ramulator ============"
#     echo "Trace: "$trace
#     echo "Speed Graede: "$speed_grade
#     echo "configuration file: "$conf_file
#     echo "Option : "$option
#     ./ramulator2 -f $conf_file -p  $trace_path -p $speed_option > $log_path &
#     done
#     wait
# done

# exit

# for speed_grade in ${speed_grade_list[@]}; do
#     for trace in ${trace_list[@]}; do
#     trace_path="Frontend.path=./perf_comparison/traces/"${trace}"_ramulatorv2.trace"
#     speed_option="MemorySystem.DRAM.timing.preset="$speed_grade
#     option="RoCoBaRaPcCh_prefetch_off"
#     conf_file="ddr5_pch_config.yaml"
#     log_path="log_RoCoBaRa/proposed_"$speed_grade"_"$option"_"$trace"_.log"
#     echo "========== Run Ramulator ============"
#     echo "Trace: "$trace
#     echo "Speed Graede: "$speed_grade
#     echo "configuration file: "$conf_file
#     echo "Option : "$option
#     ./ramulator2 -f $conf_file -p  $trace_path -p $speed_option -p MemorySystem.DRAM.use_db_fetch=false > $log_path &
#     done
#     wait
# done

# for speed_grade in ${speed_grade_list[@]}; do
#     for trace in ${trace_list[@]}; do
#     trace_path="Frontend.path=./perf_comparison/traces/"${trace}"_ramulatorv2.trace"
#     speed_option="MemorySystem.DRAM.timing.preset="$speed_grade
#     option="RoCoBaRaPcCh_prefetch_on"
#     conf_file="ddr5_pch_config.yaml"
#     log_path="log_RoCoBaRa/proposed_"$speed_grade"_"$option"_"$trace"_.log"
#     echo "========== Run Ramulator ============"
#     echo "Trace: "$trace
#     echo "Speed Graede: "$speed_grade
#     echo "configuration file: "$conf_file
#     echo "Option : "$option
#     ./ramulator2 -f $conf_file -p  $trace_path -p $speed_option -p MemorySystem.DRAM.use_db_fetch=true > $log_path &
#     done
#     wait
# done


# for speed_grade in ${speed_grade_list[@]}; do
#     for trace in ${trace_list[@]}; do
#     trace_path="Frontend.path=./perf_comparison/traces/"${trace}"_ramulatorv2.trace"
#     speed_option="MemorySystem.DRAM.timing.preset="$speed_grade
#     option="RoCoBaRaCh"
#     conf_file="ddr5_config.yaml"
#     log_path="log_RoCoBaRa/baseline_"$speed_grade"_"$option"_"$trace"_.log"
#     echo "========== Run Ramulator ============"
#     echo "Trace: "$trace
#     echo "Speed Graede: "$speed_grade
#     echo "configuration file: "$conf_file
#     echo "Option : "$option
#     ./ramulator2 -f $conf_file -p  $trace_path -p $speed_option > $log_path &
#     done
#     wait
# done

# for speed_grade in ${speed_grade_list[@]}; do
#     for dbx_org in ${dbx_org_list[@]}; do
#         for trace in ${trace_list[@]}; do
#             # for address_mapping in ${dbx_address_mapping_list[@]}; do
#             address_mapping="RoRaCoBaPcCh"
#             trace_path="Frontend.path=./perf_comparison/traces/"${trace}"_ramulatorv2.trace"
#             speed_option="MemorySystem.DRAM.timing.preset="$speed_grade
#             address_mapping_option="MemorySystem.AddrMapper.impl="$address_mapping
#             dbx_org_option="MemorySystem.DRAM.org.preset="$dbx_org
#             real_dq="4"
#             current_preset="DDR5_4800x4"
#             if [[ $dbx_org == "DDR5_16Gb_DBX_x8" ]]; then 
#                 real_dq="8"
#                 current_preset="DDR5_4800x8"
#             elif [[ $dbx_org == "DDR5_16Gb_DBX_x16" ]]; then 
#                 real_dq="16"
#                 current_preset="DDR5_4800x16"
#             fi
#             real_dq_option="MemorySystem.DRAM.org.real_dq="$real_dq
#             current_option="MemorySystem.DRAM.current.preset="$current_preset
#             conf_file="ddr5_pch_config.yaml"
#             log_path="log/proposed_"$dbx_org"_"$speed_grade"_"$address_mapping"_"$trace"_.log"
#             echo "========== Run Ramulator ============"
#             echo "Trace: "$trace
#             echo "Speed Graede: "$speed_grade
#             echo "configuration file: "$conf_file
#             echo "Option : "$address_mapping_option
#             echo "DBX ORG : "$dbx_org_option
#             echo "Real DQ on DIMM : "$real_dq_option
#             echo "Current: "$current_option
#             ./ramulator2 -f $conf_file -p  $trace_path -p $speed_option -p $address_mapping_option -p $dbx_org_option -p $real_dq_option -p $current_option > $log_path &
#             # done
#             # wait
#         done 
#         wait
#     done
#     wait
# done

# for speed_grade in ${speed_grade_list[@]}; do
#     for trace in ${trace_list[@]}; do
#     trace_path="Frontend.path=./perf_comparison/traces/"${trace}"_ramulatorv2.trace"
#     speed_option="MemorySystem.DRAM.timing.preset="$speed_grade
#     option="RoCoBaRaPcCh_prefetch_on"
#     conf_file="ddr5_pch_config.yaml"
#     log_path="log_RoCoBaRa/proposed_"$speed_grade"_"$option"_"$trace"_.log"
#     echo "========== Run Ramulator ============"
#     echo "Trace: "$trace
#     echo "Speed Graede: "$speed_grade
#     echo "configuration file: "$conf_file
#     echo "Option : "$option
#     ./ramulator2 -f $conf_file -p  $trace_path -p $speed_option -p MemorySystem.DRAM.use_db_fetch=true > $log_path &
#     done
#     wait
# done