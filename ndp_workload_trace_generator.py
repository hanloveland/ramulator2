
import math
import os
import shutil
import random
from pathlib import Path

# 1 (x4), 2 (x8), 4 (x16)
DRAM_SCALING=1

# Default :  DRAM_SCALING=1
NUM_DIMM = 1
NUM_CHANNEL = int(2 * NUM_DIMM)
NUM_PSEUDOCHANNEL = 4

# Fixed total data size reference: 1 DIMM baseline
# input_size labels (8K, 32K, ...) represent per-PCH data for 1 DIMM.
# total_data = input_size_bytes × BASE_NUM_PCH_PER_DIMM
# When NUM_DIMM > 1, per-PCH data shrinks proportionally.
NUM_CH_PER_DIMM = 2
BASE_NUM_PCH_PER_DIMM = NUM_CH_PER_DIMM * NUM_PSEUDOCHANNEL  # 2 CH × 4 PCH = 8
NUM_RANK = 4 # Basline DRAM
NUM_BANKGROUP = 8
NUM_BANK = 4
NUM_ROW = 65536
NUM_COL = 128 # 1KB/8B    
NDP_ACC_GRA = 1 # Same with Normal Access
NUM_NORMAL_COL = 128 
# DBX Address Mapping 
CHANNEL_BITS    = int(math.log2(NUM_CHANNEL))                 # 2 Channel
PCHANNEL_BITS   = int(math.log2(NUM_PSEUDOCHANNEL))           # 4 Pseudo Channel  
RANK_BITS       = 0                                           # 1 Rank
BANKGROUP_BITS  = int(math.log2(NUM_BANKGROUP))               # 4 or 8 Bank Group  
BANK_BITS       = int(math.log2(NUM_BANK))                    # 4 Bank   
ROW_BITS        = int(math.log2(NUM_ROW))                     # 64K Row
COLUMN_BITS     = int(math.log2(NUM_COL))                     # 11 - 4 # 2K Column (1KB/8B=128)
BURST_SIZE      = 64                                          # Prefetch Size 64
GRANULARITY     = int(math.log2(BURST_SIZE))                  # 64B 

# Normal Addrss Mapping Not Change 
NORMAL_CHANNEL_BITS    = int(math.log2(NUM_CHANNEL))          # 2 Channel
NORMAL_RANK_BITS       = int(math.log2(NUM_RANK))             # 4 Rank
NORMAL_BANKGROUP_BITS  = int(math.log2(NUM_BANKGROUP))        # 8 Bank Group  
NORMAL_BANK_BITS       = int(math.log2(NUM_BANK))             # 4 Bank   
NORMAL_ROW_BITS        = int(math.log2(NUM_ROW))              # 64K Row
NORMAL_COLUMN_BITS     = int(math.log2(NUM_NORMAL_COL))       # 128 columns (log2(2048)-log2(prefetch=16)=7)
NORMAL_BURST_SIZE      = 64                                   # Prefetch Size 64
NORMAL_GRANULARITY     = int(math.log2(NORMAL_BURST_SIZE))    # 64B 

NORMAL_SCALE_FACTOR = 4 #  PCH per CH 

MAX_INST        = 1024 # 8K/8B
NDP_ROW         = 65535
# DRAM Spill: descriptor overflow → DRAM에 저장, HSNC가 prefetch
NDP_DESC_SPILL_ROW_BASE = 60000  # Reserved DRAM row for descriptor spill
NDP_DESC_SPILL_BG       = 0      # BG for descriptor spill region
PCH_DESC_BUFFER_SIZE    = 64     # On-chip pch_desc_buffer capacity

# DBX Addres Space 
# Host-side NDP Controller Address Space (BG,BK)
HSNU_CTR_REG_BK = 3
HSNU_CTR_REG_BG = 7
# Host-side NDP Access Information Buffer Address Space (BG,BK)
HSNU_CTR_BUF_BK = 3
HSNU_CTR_BUF_BG = 6
# DIMM-side NDP Contrller Address Space (BG,BK)
NDP_CTRL_BK     = 3
NDP_CTRL_BG     = 5
# Address Space of NDP Instruction Memory on NDP Unit  (BG,BK)
NDP_INS_MEM_BK  = 3 
NDP_INS_MEM_BG  = 4 
# Address Space of NDP Data Memory on NDP Unit  (BG,BK)
NDP_DAT0_MEM_BK = 3
NDP_DAT0_MEM_BG = 3 # BG0-BG3
NDP_DAT1_MEM_BK = -1 # Not Used when DRAM x4 
NDP_DAT1_MEM_BG = -1 # Not Used when DRAM x4 
NDP_DAT2_MEM_BK = -1 # Not Used when DRAM x4 
NDP_DAT2_MEM_BG = -1 # Not Used when DRAM x4 
NDP_DAT3_MEM_BK = -1 # Not Used when DRAM x4 
NDP_DAT3_MEM_BG = -1 # Not Used when DRAM x4 
NDP_TARGET_BK = 3    # To decoule host access and DRAM access

# NDPInst Wildcard: bit[42]=1 → match by id only, ignore bg/bk in matching
# Usage: inst(opcode, opsize, id, bg, bk, op1, op2, op3, wildcard=1)
NDP_INST_WILDCARD = 1

SPLIT_HIGH_BANK = False
LINEAR_ACCESS = False
GEN_BGCH = False
FIX_BANK=True
if SPLIT_HIGH_BANK:
    PCH_ADDRESS_SCHEME = "BhRoBlBgCoRaPcCH"
else:
    # PCH_ADDRESS_SCHEME = "RoCoBaBgRaPcCH"
    PCH_ADDRESS_SCHEME = "RoBaBgCoRaPcCH"
    # PCH_ADDRESS_SCHEME = "BaRoCoBgRaPcCH"
    # PCH_ADDRESS_SCHEME = "RoBaBgRaCoPcCH"
    
if SPLIT_HIGH_BANK:
    NORMAL_ADDRESS_SCHEME = "BhRoBlBgCoRaCh"
else:
    # NORMAL_ADDRESS_SCHEME = "RoCoBaRaCh"
    # NORMAL_ADDRESS_SCHEME = "RoBaRaCoCh"
    # NORMAL_ADDRESS_SCHEME = "RoRaCoBaCh"
    NORMAL_ADDRESS_SCHEME = "RoBaCoRaCh"

GEN_PCH_NORMAL_MODE = True

ndp_inst_opcode = {
    "LOAD"           :0,
    "LOAD_ADD"       :1,
    "LOAD_MUL"       :2,
    "ADD"            :3,
    "MUL"            :4,
    "V_RED"          :5,
    "S_RED"          :6,
    "MAC"            :7,
    "SCALE_ADD"      :8,
    "T_ADD"          :16,
    "T_MUL"          :17,
    "T_V_RED"        :18,
    "T_S_RED"        :19,
    "T_MAC"          :20,
    "WBD"            :32,
    "BARRIER"        :48,
    "EXIT"           :49,
    "SELF_EXEC_ON"   :50,
    "SELF_EXEC_OFF"  :51,
    "LOOP"           :52,
    "SET_BASE"       :53,
    "INC_BASE"       :54
}

ndp_acc_inst_opcode = {
    # Data access opcodes (mode applicable)
    "RD"         :0,
    "WR"         :1,
    # Control opcodes (mode not applicable)
    "BAR"        :2,
    "WAIT_RES"   :3,
    "LOOP_START" :4,   # Legacy (unused)
    "LOOP_END"   :5,   # Legacy (unused)
    "WAIT"       :6,
    "SET_BASE"   :8,   # NEW: base_reg[reg_idx] = base_value
    "INC_BASE"   :9,   # NEW: base_reg[reg_idx] += inc_value
    "SET_LOOP"   :10,  # NEW: loop_cnt_reg[cnt_reg_idx] = loop_count
    "LOOP"       :11,  # NEW: if loop_cnt_reg[cnt_reg_idx] > 0: cnt--, PC=jump_pc
    "DONE"       :15
}

input_size_byte_list = {
    "8K" :  8192,
    "32K":  32768,
    "128K": 131072,
    "512K": 524288,
    "2M": 2097152,
    "8M": 8388608
}

input_size_list = [
    "8K",
    "32K",
    "128K",
    "512K",
    "2M",
    "8M"
]

mat_input_size_byte_list = {
    "8K" :  8192,
    "16K":  16384,
    "32K": 32768,
    "64K": 65536,
    "128K": 131072,
}

mat_input_size_list = [
    "8K",
    "16K",
    "32K",
    "64K",
    "128K",
]

workload_list = {
    "AXPBY",
    "AXPBYPCZ",
    "AXPY",
    "COPY",
    "XMY",
    "DOT",
    "SCAL",
    "GEMV"
}

# =========================================================================
#  SLS (SparseLengthsSum) — Recommendation System Workload
# =========================================================================
SLS_NUM_TABLES          = 128           # T: number of embedding tables
SLS_ENTRIES_PER_TABLE   = 1_000_000     # N: entries per table
SLS_EMBED_DIM           = 128           # D: embedding dimension (FP16)
SLS_VECTOR_BYTES        = 256           # D × 2 bytes (FP16)
SLS_COLS_PER_VECTOR     = 4             # 256B / 64B burst
SLS_VECTORS_PER_ROW     = 32            # 128 cols / 4 cols per vector (8KB row)
SLS_ZIPF_ALPHA          = 1.05          # Criteo empirical distribution
SLS_ZIPF_SEED           = 42            # Reproducibility

# Mapper-accurate bit widths for DDR5_16Gb_x4 (ch=2, rk=4) + RoBaCoRaCh
# Col bits = log2(2048) - log2(prefetch=16) = 11 - 4 = 7
SLS_CH_BITS  = 1    # log2(2 channels)
SLS_RA_BITS  = 2    # log2(4 ranks)
SLS_CO_BITS  = 7    # 128 columns after prefetch adjustment
SLS_BG_BITS  = 3    # log2(8 bank groups)
SLS_BK_BITS  = 2    # log2(4 banks)
SLS_RO_BITS  = 16   # log2(65536 rows)
SLS_TX_OFFSET = 6   # log2(prefetch=16 × channel_width=32 / 8) = log2(64)

sls_config_list = {
    "SLS-S": {"B": 1,  "L": 16},    # Single query
    "SLS-M": {"B": 16, "L": 16},    # Batched inference
    "SLS-L": {"B": 64, "L": 64},    # High-throughput serving
}
sls_config_names = ["SLS-S", "SLS-M", "SLS-L"]

def config_scale_factor(scaling_factor):
    global NUM_CHANNEL
    global NUM_PSEUDOCHANNEL
    global NUM_BANKGROUP
    global NUM_BANK
    global NUM_ROW
    global NUM_COL
    global NDP_ACC_GRA    
    if scaling_factor == 1:
        NUM_CHANNEL = int(2 * NUM_DIMM)
        NUM_PSEUDOCHANNEL = 4
        NUM_BANKGROUP = 8
        NUM_BANK = 4
        NUM_ROW = 65536
        NUM_COL = 128 # 1KB/8B    
        NDP_ACC_GRA = 1 # Same with Normal Access
    elif scaling_factor == 2:
        NUM_CHANNEL = int(2 * NUM_DIMM)
        NUM_PSEUDOCHANNEL = 4
        NUM_BANKGROUP = 8
        NUM_BANK = 4
        NUM_ROW = 65536
        NUM_COL = 128 # 1KB/128B
        NDP_ACC_GRA = 2 # 2x then normal access
    elif scaling_factor == 4:
        NUM_CHANNEL = int(2 * NUM_DIMM)
        NUM_PSEUDOCHANNEL = 4
        NUM_BANKGROUP = 4
        NUM_BANK = 4
        NUM_ROW = 65536
        NUM_COL = 256 # 2KB/8B
        NDP_ACC_GRA = 4 # 4x then normal access
    else:
        print(f"Error: Wrong DRAM Type {scaling_factor}")
        exit(1)    

    global CHANNEL_BITS
    global PCHANNEL_BITS
    global BANKGROUP_BITS
    global BANK_BITS
    global ROW_BITS
    global COLUMN_BITS

    CHANNEL_BITS    = int(math.log2(NUM_CHANNEL))                 
    PCHANNEL_BITS   = int(math.log2(NUM_PSEUDOCHANNEL))           
    BANKGROUP_BITS  = int(math.log2(NUM_BANKGROUP))               
    BANK_BITS       = int(math.log2(NUM_BANK))                    
    ROW_BITS        = int(math.log2(NUM_ROW))                     
    COLUMN_BITS     = int(math.log2(NUM_COL))             

    # Host-side NDP Controller Address Space (BG,BK)
    global HSNU_CTR_REG_BK
    global HSNU_CTR_REG_BG
    # Host-side NDP Access Information Buffer Address Space (BG,BK)
    global HSNU_CTR_BUF_BK
    global HSNU_CTR_BUF_BG
    # DIMM-side NDP Contrller Address Space (BG,BK)
    global NDP_CTRL_BK
    global NDP_CTRL_BG
    # Address Space of NDP Instruction Memory on NDP Unit  (BG,BK)
    global NDP_INS_MEM_BK
    global NDP_INS_MEM_BG
    # Address Space of NDP Data Memory on NDP Unit  (BG,BK)
    global NDP_DAT0_MEM_BK
    global NDP_DAT0_MEM_BG
    global NDP_DAT1_MEM_BK
    global NDP_DAT1_MEM_BG
    global NDP_DAT2_MEM_BK
    global NDP_DAT2_MEM_BG
    global NDP_DAT3_MEM_BK
    global NDP_DAT3_MEM_BG
    if scaling_factor == 1:
        # Host-side NDP Controller Address Space (BG,BK)
        HSNU_CTR_REG_BK = 3
        HSNU_CTR_REG_BG = 7
        # Host-side NDP Access Information Buffer Address Space (BG,BK)
        HSNU_CTR_BUF_BK = 3
        HSNU_CTR_BUF_BG = 6
        # DIMM-side NDP Contrller Address Space (BG,BK)
        NDP_CTRL_BK     = 3
        NDP_CTRL_BG     = 5
        # Address Space of NDP Instruction Memory on NDP Unit  (BG,BK)
        NDP_INS_MEM_BK  = 3 
        NDP_INS_MEM_BG  = 4 
        # Address Space of NDP Data Memory on NDP Unit  (BG,BK)
        NDP_DAT0_MEM_BK = 3
        NDP_DAT0_MEM_BG = 3 # BG0-BG3
        NDP_DAT1_MEM_BK = -1 # Not Used when DRAM x4 
        NDP_DAT1_MEM_BG = -1 # Not Used when DRAM x4 
        NDP_DAT2_MEM_BK = -1 # Not Used when DRAM x4 
        NDP_DAT2_MEM_BG = -1 # Not Used when DRAM x4 
        NDP_DAT3_MEM_BK = -1 # Not Used when DRAM x4 
        NDP_DAT3_MEM_BG = -1 # Not Used when DRAM x4 
    elif scaling_factor == 2:
        # Host-side NDP Controller Address Space (BG,BK)
        HSNU_CTR_REG_BK = 3
        HSNU_CTR_REG_BG = 7
        # Host-side NDP Access Information Buffer Address Space (BG,BK)
        HSNU_CTR_BUF_BK = 3
        HSNU_CTR_BUF_BG = 6
        # DIMM-side NDP Contrller Address Space (BG,BK)
        NDP_CTRL_BK     = 3
        NDP_CTRL_BG     = 5
        # Address Space of NDP Instruction Memory on NDP Unit  (BG,BK)
        NDP_INS_MEM_BK  = 3 
        NDP_INS_MEM_BG  = 4 
        # Address Space of NDP Data Memory on NDP Unit  (BG,BK)
        NDP_DAT0_MEM_BK = 3
        NDP_DAT0_MEM_BG = 3 # BG0-BG3
        NDP_DAT1_MEM_BK = 2 
        NDP_DAT1_MEM_BG = 3 # BG0-BG3
        NDP_DAT2_MEM_BK = -1 
        NDP_DAT2_MEM_BG = -1 
        NDP_DAT3_MEM_BK = -1 
        NDP_DAT3_MEM_BG = -1 
    elif scaling_factor == 4:
        # Host-side NDP Controller Address Space (BG,BK)
        HSNU_CTR_REG_BK = 3
        HSNU_CTR_REG_BG = 3
        # Host-side NDP Access Information Buffer Address Space (BG,BK)
        HSNU_CTR_BUF_BK = 3
        HSNU_CTR_BUF_BG = 2
        # DIMM-side NDP Contrller Address Space (BG,BK)
        NDP_CTRL_BK     = 3
        NDP_CTRL_BG     = 1
        # Address Space of NDP Instruction Memory on NDP Unit  (BG,BK)
        NDP_INS_MEM_BK  = 3 
        NDP_INS_MEM_BG  = 0 
        # Address Space of NDP Data Memory on NDP Unit  (BG,BK)
        NDP_DAT0_MEM_BK = 2
        NDP_DAT0_MEM_BG = 3 # BG0-BG3
        NDP_DAT1_MEM_BK = 2 
        NDP_DAT1_MEM_BG = 3 # BG0-BG3
        NDP_DAT2_MEM_BK = 1 
        NDP_DAT2_MEM_BG = 3 # BG0-BG3
        NDP_DAT3_MEM_BK = 1 
        NDP_DAT3_MEM_BG = 3 # BG0-BG3
    else:
        print("Error: Wrong DRAM Type")
        exit(1)


def calc_per_pch_bytes(input_size_label):
    """Calculate per-PCH byte size for fixed total data.

    input_size labels (8K, 32K, ...) represent per-PCH data for the 1-DIMM baseline.
    Total data is fixed: input_size_bytes × BASE_NUM_PCH_PER_DIMM (=8 for 1 DIMM).
    When NUM_DIMM > 1, more PCHs share the same total → per-PCH shrinks.

    Returns per_pch_bytes (int).

    Examples (input_size="8K", per-PCH base = 8192):
      1 DIMM (8 PCH):  8192 × 8 /  8 = 8192  (unchanged)
      2 DIMM (16 PCH): 8192 × 8 / 16 = 4096
      4 DIMM (32 PCH): 8192 × 8 / 32 = 2048
    """
    base_bytes = input_size_byte_list[input_size_label]
    total_data = base_bytes * BASE_NUM_PCH_PER_DIMM
    actual_num_pch = int(NUM_CHANNEL) * int(NUM_PSEUDOCHANNEL)
    per_pch_bytes = total_data // actual_num_pch
    if per_pch_bytes == 0:
        print(f"Error: per_pch_bytes=0 (total={total_data}, pch={actual_num_pch}). "
              f"Input size '{input_size_label}' too small for {NUM_DIMM} DIMMs.")
        exit(1)
    return per_pch_bytes


def encode_address(channel, pseudo_channel, rank, bg, bank, row, col):
    """
        ROCoBaBgRaPcCH
        RoBaBgRaCoPcCH
        BaRoCoBgRaPcCH
        RoBaBgCoRaPcCH
        BhRoBlBgCoRaPcCH
    """
    address = 0
    global PCH_ADDRESS_SCHEME
    if PCH_ADDRESS_SCHEME == "RoCoBaBgRaPcCH":
        address |= (channel          & ((1 << CHANNEL_BITS) - 1))
        address |= (pseudo_channel   & ((1 << PCHANNEL_BITS) - 1))      << (CHANNEL_BITS)
        address |= (rank             & ((1 << RANK_BITS) - 1))          << (CHANNEL_BITS+PCHANNEL_BITS)
        address |= (bg               & ((1 << BANKGROUP_BITS) - 1))     << (CHANNEL_BITS+PCHANNEL_BITS+RANK_BITS)
        address |= (bank             & ((1 << BANK_BITS) - 1))          << (CHANNEL_BITS+PCHANNEL_BITS+RANK_BITS+BANKGROUP_BITS)
        address |= (col              & ((1 << COLUMN_BITS) - 1))        << (CHANNEL_BITS+PCHANNEL_BITS+RANK_BITS+BANKGROUP_BITS+BANK_BITS)
        address |= (row              & ((1 << ROW_BITS) - 1))           << (CHANNEL_BITS+PCHANNEL_BITS+RANK_BITS+BANKGROUP_BITS+BANK_BITS+COLUMN_BITS)
        # Shift by DRAM Acccess Granularity (64B)
        address = address << GRANULARITY
    elif PCH_ADDRESS_SCHEME == "RoBaBgRaCoPcCH":
        address |= (channel          & ((1 << CHANNEL_BITS) - 1))
        address |= (pseudo_channel   & ((1 << PCHANNEL_BITS) - 1))      << (CHANNEL_BITS)
        address |= (col              & ((1 << COLUMN_BITS) - 1))        << (CHANNEL_BITS+PCHANNEL_BITS)
        address |= (rank             & ((1 << RANK_BITS) - 1))          << (CHANNEL_BITS+PCHANNEL_BITS+COLUMN_BITS)
        address |= (bg               & ((1 << BANKGROUP_BITS) - 1))     << (CHANNEL_BITS+PCHANNEL_BITS+COLUMN_BITS+RANK_BITS)
        address |= (bank             & ((1 << BANK_BITS) - 1))          << (CHANNEL_BITS+PCHANNEL_BITS+COLUMN_BITS+RANK_BITS+BANKGROUP_BITS)
        address |= (row              & ((1 << ROW_BITS) - 1))           << (CHANNEL_BITS+PCHANNEL_BITS+COLUMN_BITS+RANK_BITS+BANKGROUP_BITS+BANK_BITS)
        # Shift by DRAM Acccess Granularity (64B)
        address = address << GRANULARITY
    elif PCH_ADDRESS_SCHEME == "BaRoCoBgRaPcCH":
        address |= (channel          & ((1 << CHANNEL_BITS) - 1))
        address |= (pseudo_channel   & ((1 << PCHANNEL_BITS) - 1))      << (CHANNEL_BITS)
        address |= (rank             & ((1 << RANK_BITS) - 1))          << (CHANNEL_BITS+PCHANNEL_BITS)
        address |= (bg               & ((1 << BANKGROUP_BITS) - 1))     << (CHANNEL_BITS+PCHANNEL_BITS+RANK_BITS)
        address |= (col              & ((1 << COLUMN_BITS) - 1))        << (CHANNEL_BITS+PCHANNEL_BITS+RANK_BITS+BANKGROUP_BITS)
        address |= (row              & ((1 << ROW_BITS) - 1))           << (CHANNEL_BITS+PCHANNEL_BITS+RANK_BITS+BANKGROUP_BITS+COLUMN_BITS)
        address |= (bank             & ((1 << BANK_BITS) - 1))          << (CHANNEL_BITS+PCHANNEL_BITS+RANK_BITS+BANKGROUP_BITS+COLUMN_BITS+ROW_BITS)
        # Shift by DRAM Acccess Granularity (64B)
        address = address << GRANULARITY        
    elif PCH_ADDRESS_SCHEME == "RoBaBgCoRaPcCH":
        address |= (channel          & ((1 << CHANNEL_BITS) - 1))
        address |= (pseudo_channel   & ((1 << PCHANNEL_BITS) - 1))      << (CHANNEL_BITS)
        address |= (rank             & ((1 << RANK_BITS) - 1))          << (CHANNEL_BITS+PCHANNEL_BITS)
        address |= (col              & ((1 << COLUMN_BITS) - 1))        << (CHANNEL_BITS+PCHANNEL_BITS+RANK_BITS)
        address |= (bg               & ((1 << BANKGROUP_BITS) - 1))     << (CHANNEL_BITS+PCHANNEL_BITS+RANK_BITS+COLUMN_BITS)     
        address |= (bank             & ((1 << BANK_BITS) - 1))          << (CHANNEL_BITS+PCHANNEL_BITS+RANK_BITS+COLUMN_BITS+BANKGROUP_BITS)        
        address |= (row              & ((1 << ROW_BITS) - 1))           << (CHANNEL_BITS+PCHANNEL_BITS+RANK_BITS+COLUMN_BITS+BANKGROUP_BITS+BANK_BITS)
        # Shift by DRAM Acccess Granularity (64B)
        address = address << GRANULARITY          
    # print(f" Address Translate CH{channel},PCH{pseudo_channel},RK{rank},BG{bg},BK{bank},RO{row},CO{col} --> 0x{address:016X}\n")        
    elif PCH_ADDRESS_SCHEME == "BhRoBlBgCoRaPcCH":
        if BANK_BITS != 2:
            print("Bank Bits of BhRoBlBgCoRaPcCH must be 2!")
            exit(1)
        bank_low = bank & 1
        bank_high = (bank >> 1) & 1
        address |= (channel          & ((1 << CHANNEL_BITS) - 1))
        address |= (pseudo_channel   & ((1 << PCHANNEL_BITS) - 1))      << (CHANNEL_BITS)
        address |= (rank             & ((1 << RANK_BITS) - 1))          << (CHANNEL_BITS+PCHANNEL_BITS)
        address |= (col              & ((1 << COLUMN_BITS) - 1))        << (CHANNEL_BITS+PCHANNEL_BITS+RANK_BITS)
        address |= (bg               & ((1 << BANKGROUP_BITS) - 1))     << (CHANNEL_BITS+PCHANNEL_BITS+RANK_BITS+COLUMN_BITS)     
        address |= (bank_low         & (1))                             << (CHANNEL_BITS+PCHANNEL_BITS+RANK_BITS+COLUMN_BITS+BANKGROUP_BITS)        
        address |= (row              & ((1 << ROW_BITS) - 1))           << (CHANNEL_BITS+PCHANNEL_BITS+RANK_BITS+COLUMN_BITS+BANKGROUP_BITS+BANK_BITS + 1)
        address |= (bank_high        & (1))                             << (CHANNEL_BITS+PCHANNEL_BITS+RANK_BITS+COLUMN_BITS+BANKGROUP_BITS+BANK_BITS + 1 + ROW_BITS)
        # Shift by DRAM Acccess Granularity (64B)
        address = address << GRANULARITY          

    return address

def encode_normal_address(channel, rank, bg, bank, row, col):
    """
        RoCoBaRaCh
        RoBaRaCoCh
        RoRaCoBaCh        
        BhRoBlBgCoRaCh
    """
    address = 0
    global NORMAL_ADDRESS_SCHEME
    if NORMAL_ADDRESS_SCHEME == "RoCoBaRaCh":
        address |= (channel          & ((1 << NORMAL_CHANNEL_BITS) - 1))
        address |= (rank             & ((1 << NORMAL_RANK_BITS) - 1))      << (NORMAL_CHANNEL_BITS)
        address |= (bg               & ((1 << NORMAL_BANKGROUP_BITS) - 1)) << (NORMAL_CHANNEL_BITS+NORMAL_RANK_BITS)
        address |= (bank             & ((1 << NORMAL_BANK_BITS) - 1))      << (NORMAL_CHANNEL_BITS+NORMAL_RANK_BITS+NORMAL_BANKGROUP_BITS)
        address |= (col              & ((1 << NORMAL_COLUMN_BITS) - 1))    << (NORMAL_CHANNEL_BITS+NORMAL_RANK_BITS+NORMAL_BANKGROUP_BITS+NORMAL_BANK_BITS)
        address |= (row              & ((1 << NORMAL_ROW_BITS) - 1))       << (NORMAL_CHANNEL_BITS+NORMAL_RANK_BITS+NORMAL_BANKGROUP_BITS+NORMAL_BANK_BITS+NORMAL_COLUMN_BITS)
        # Shift by DRAM Acccess Granularity (64B)
        address = address << GRANULARITY
    elif NORMAL_ADDRESS_SCHEME == "RoBaRaCoCh":
        address |= (channel          & ((1 << NORMAL_CHANNEL_BITS) - 1))
        address |= (col              & ((1 << NORMAL_COLUMN_BITS) - 1))    << (NORMAL_CHANNEL_BITS)
        address |= (rank             & ((1 << NORMAL_RANK_BITS) - 1))      << (NORMAL_CHANNEL_BITS+NORMAL_COLUMN_BITS)
        address |= (bg               & ((1 << NORMAL_BANKGROUP_BITS) - 1)) << (NORMAL_CHANNEL_BITS+NORMAL_COLUMN_BITS+NORMAL_RANK_BITS)
        address |= (bank             & ((1 << NORMAL_BANK_BITS) - 1))      << (NORMAL_CHANNEL_BITS+NORMAL_COLUMN_BITS+NORMAL_RANK_BITS+NORMAL_BANKGROUP_BITS)
        address |= (row              & ((1 << NORMAL_ROW_BITS) - 1))       << (NORMAL_CHANNEL_BITS+NORMAL_COLUMN_BITS+NORMAL_RANK_BITS+NORMAL_BANKGROUP_BITS+NORMAL_BANK_BITS)
        # Shift by DRAM Acccess Granularity (64B)
        address = address << GRANULARITY
    elif NORMAL_ADDRESS_SCHEME == "RoRaCoBaCh":
        address |= (channel          & ((1 << NORMAL_CHANNEL_BITS) - 1))
        address |= (bg               & ((1 << NORMAL_BANKGROUP_BITS) - 1)) << (NORMAL_CHANNEL_BITS)
        address |= (bank             & ((1 << NORMAL_BANK_BITS) - 1))      << (NORMAL_CHANNEL_BITS+NORMAL_BANKGROUP_BITS)
        address |= (col              & ((1 << NORMAL_COLUMN_BITS) - 1))    << (NORMAL_CHANNEL_BITS+NORMAL_BANKGROUP_BITS+NORMAL_BANK_BITS)
        address |= (rank             & ((1 << NORMAL_RANK_BITS) - 1))      << (NORMAL_CHANNEL_BITS+NORMAL_BANKGROUP_BITS+NORMAL_BANK_BITS+NORMAL_COLUMN_BITS)
        address |= (row              & ((1 << NORMAL_ROW_BITS) - 1))       << (NORMAL_CHANNEL_BITS+NORMAL_BANKGROUP_BITS+NORMAL_BANK_BITS+NORMAL_COLUMN_BITS+NORMAL_RANK_BITS)
        # Shift by DRAM Acccess Granularity (64B)
        address = address << GRANULARITY        
    elif NORMAL_ADDRESS_SCHEME == "RoBaCoRaCh":
        address |= (channel          & ((1 << NORMAL_CHANNEL_BITS) - 1))
        address |= (rank             & ((1 << NORMAL_RANK_BITS) - 1))      << (NORMAL_CHANNEL_BITS)
        address |= (col              & ((1 << NORMAL_COLUMN_BITS) - 1))    << (NORMAL_CHANNEL_BITS+NORMAL_RANK_BITS)
        address |= (bg               & ((1 << NORMAL_BANKGROUP_BITS) - 1)) << (NORMAL_CHANNEL_BITS+NORMAL_RANK_BITS+NORMAL_COLUMN_BITS)
        address |= (bank             & ((1 << NORMAL_BANK_BITS) - 1))      << (NORMAL_CHANNEL_BITS+NORMAL_RANK_BITS+NORMAL_COLUMN_BITS+NORMAL_BANKGROUP_BITS)
        address |= (row              & ((1 << NORMAL_ROW_BITS) - 1))       << (NORMAL_CHANNEL_BITS+NORMAL_RANK_BITS+NORMAL_COLUMN_BITS+NORMAL_BANKGROUP_BITS+NORMAL_BANK_BITS)
        # Shift by DRAM Acccess Granularity (64B)
        address = address << GRANULARITY     
    elif NORMAL_ADDRESS_SCHEME == "BhRoBlBgCoRaCh":
        if BANK_BITS != 2:
            print("Bank Bits of BhRoBlBgCoRaCh must be 2!")
            exit(1)        
        bank_low = bank & 1
        bank_high = (bank >> 1) & 1                    
        address |= (channel          & ((1 << NORMAL_CHANNEL_BITS) - 1))
        address |= (rank             & ((1 << NORMAL_RANK_BITS) - 1))      << (NORMAL_CHANNEL_BITS)
        address |= (col              & ((1 << NORMAL_COLUMN_BITS) - 1))    << (NORMAL_CHANNEL_BITS+NORMAL_RANK_BITS)
        address |= (bg               & ((1 << NORMAL_BANKGROUP_BITS) - 1)) << (NORMAL_CHANNEL_BITS+NORMAL_RANK_BITS+NORMAL_COLUMN_BITS)
        address |= (bank_low         & (1))                                << (NORMAL_CHANNEL_BITS+NORMAL_RANK_BITS+NORMAL_COLUMN_BITS+NORMAL_BANKGROUP_BITS)
        address |= (row              & ((1 << NORMAL_ROW_BITS) - 1))       << (NORMAL_CHANNEL_BITS+NORMAL_RANK_BITS+NORMAL_COLUMN_BITS+NORMAL_BANKGROUP_BITS+1)
        address |= (bank_high        & (1))                                << (NORMAL_CHANNEL_BITS+NORMAL_RANK_BITS+NORMAL_COLUMN_BITS+NORMAL_BANKGROUP_BITS+1+NORMAL_ROW_BITS)

        # Shift by DRAM Acccess Granularity (64B)
        address = address << GRANULARITY     

    return address

def write_trace(f, instr_type, address, data_array):
    if instr_type == 'ST':
        f.write(f"{instr_type} 0x{address:016X} 0x{data_array[0]:016X} 0x{data_array[1]:016X} 0x{data_array[2]:016X} 0x{data_array[3]:016X} 0x{data_array[4]:016X} 0x{data_array[5]:016X} 0x{data_array[6]:016X} 0x{data_array[7]:016X}\n")
    else:
        f.write(f"{instr_type} 0x{address:016X}\n")

def write_normal_trace(f, instr_type, address):
    if instr_type == 'ST':
        f.write(f"{instr_type} 0x{address:016X}\n")
    else:
        f.write(f"{instr_type} 0x{address:016X}\n")

def write_wait_ndp(f):
    """Emit WAIT_NDP synchronization barrier.
    Blocks trace issuance until NDP execution completes and all outstanding reads drain."""
    f.write("WAIT_NDP\n")

# =========================================================================
#  SLS Helper Functions
# =========================================================================

def sls_table_to_bank(table_id):
    """Map table_id (0~127) -> (ch, rk, bg, bk).
    CH->RK->BG->BK placement for maximum parallelism.
    128 tables use BK=0,1 only (BK=2,3 reserved for output)."""
    ch = table_id % 2
    rk = (table_id // 2) % 4
    bg = (table_id // 8) % 8
    bk = (table_id // 64) % 4
    return ch, rk, bg, bk

def sls_entry_to_row_col(entry_idx):
    """Convert embedding entry index to (row, col_start) within a table's bank.
    16 vectors per row (8KB row / 512B vector), 8 columns per vector."""
    row = entry_idx // SLS_VECTORS_PER_ROW
    col_start = (entry_idx % SLS_VECTORS_PER_ROW) * SLS_COLS_PER_VECTOR
    return row, col_start

def encode_sls_address(channel, rank, bg, bank, row, col):
    """Encode address that exactly matches the RoBaCoRaCh mapper decoding.

    Mapper extraction order (after >> tx_offset=6):
      Ch(1b) | Ra(2b) | Col(7b) | BG(3b) | BK(2b) | Row(16b)

    Col = 7 bits (log2(2048) - log2(prefetch=16) = 11 - 4 = 7).
    """
    address = 0
    address |= (channel & ((1 << SLS_CH_BITS) - 1))
    address |= (rank    & ((1 << SLS_RA_BITS) - 1)) << SLS_CH_BITS
    address |= (col     & ((1 << SLS_CO_BITS) - 1)) << (SLS_CH_BITS + SLS_RA_BITS)
    address |= (bg      & ((1 << SLS_BG_BITS) - 1)) << (SLS_CH_BITS + SLS_RA_BITS + SLS_CO_BITS)
    address |= (bank    & ((1 << SLS_BK_BITS) - 1)) << (SLS_CH_BITS + SLS_RA_BITS + SLS_CO_BITS + SLS_BG_BITS)
    address |= (row     & ((1 << SLS_RO_BITS) - 1)) << (SLS_CH_BITS + SLS_RA_BITS + SLS_CO_BITS + SLS_BG_BITS + SLS_BK_BITS)
    address = address << SLS_TX_OFFSET
    return address

def sls_generate_zipf_indices(B, L, T, N, alpha, seed):
    """Generate Zipf-distributed embedding indices.
    Returns list of shape [T][B][L] with values in [0, N-1].
    Uses inverse CDF method (no numpy dependency)."""
    rng = random.Random(seed)

    # Pre-compute CDF for Zipf(alpha) over [1, N]
    # For large N, compute harmonic numbers incrementally
    H = 0.0
    pmf = [0.0] * (N + 1)  # pmf[k] = 1/k^alpha, k=1..N
    for k in range(1, N + 1):
        pmf[k] = 1.0 / (k ** alpha)
        H += pmf[k]

    # Build CDF table (sampled at checkpoints for memory efficiency)
    # For N=1M with alpha=1.05, most probability mass is in k<10000.
    # Use rejection sampling: generate uniform, binary search CDF.
    # Optimization: pre-compute cumulative sums
    cdf = [0.0] * (N + 1)
    for k in range(1, N + 1):
        cdf[k] = cdf[k - 1] + pmf[k] / H

    def sample_one():
        u = rng.random()
        # Binary search for smallest k where cdf[k] >= u
        lo, hi = 1, N
        while lo < hi:
            mid = (lo + hi) // 2
            if cdf[mid] < u:
                lo = mid + 1
            else:
                hi = mid
        return lo - 1  # 0-indexed

    # Generate all indices
    indices = [[[sample_one() for _ in range(L)] for _ in range(B)] for _ in range(T)]
    return indices

# NDP Instruction: Opcode, Opsize, ID, BG, BK, OP1, OP2, OP3, wildcard
# bit[42] = wildcard flag: 1 = match by id only (ignore bg/bk), 0 = exact 3-field match
def inst(opcode,opsize,id,bg,bk,op1,op2,op3,wildcard=0):
    inst_64bit = 0
    inst_64bit |= (opcode & 0x3f) << 58
    inst_64bit |= (opsize & 0x7f) << 51
    inst_64bit |= (id & 0x7) << 48
    inst_64bit |= (bg & 0x7) << 45
    inst_64bit |= (bk & 0x3) << 43
    inst_64bit |= (wildcard & 0x1) << 42       # Wildcard flag
    if opcode == ndp_inst_opcode["LOOP"]:
        inst_64bit |= (op1 & 0x3FF) << 16      # Iteration
        inst_64bit |= (op2 & 0x3FF)            # PC
    return inst_64bit

# Redefined AccInst_Slot (64-bit) with Direct/Undirect Mode
# [63:60] opcode(4b) [59] mode(1b) [58:52] opsize(7b) [51:46] ch(6b) [45:44] pch(2b)
# [43:41] bg(3b) [40:39] bk(2b) [38:21] row(18b) [20:14] col(7b) [13:11] id(3b) [10:0] etc(11b)
#
# mode=0: Direct (row = absolute row address)
# mode=1: Undirect (row[17:15]=base_reg_idx, row[14:0]=offset)
#
# SET_BASE: [63:60]=opcode [59]=dc [58:56]=reg_idx(3b) [55:38]=base_value(18b) [37:0]=dc
# INC_BASE: [63:60]=opcode [59]=dc [58:56]=reg_idx(3b) [55:38]=inc_value(18b) [37:0]=dc
# SET_LOOP: [63:60]=opcode [59]=dc [58:56]=cnt_reg_idx(3b) [55:40]=loop_count(16b) [39:0]=dc
# LOOP:     [63:60]=opcode [59]=dc [58:56]=cnt_reg_idx(3b) [55:40]=jump_pc(16b) [39:0]=dc
def acc_inst(opcode, opsize, ch, pch, bg, bk, row, col, id, etc, mode=0):
    inst_64bit = 0
    inst_64bit |= (opcode & 0xf)      << 60    # [63:60]
    inst_64bit |= (mode   & 0x1)      << 59    # [59]
    inst_64bit |= (opsize & 0x7f)     << 52    # [58:52]
    inst_64bit |= (ch     & 0x3f)     << 46    # [51:46]
    inst_64bit |= (pch    & 0x3)      << 44    # [45:44]
    inst_64bit |= (bg     & 0x7)      << 41    # [43:41]
    inst_64bit |= (bk     & 0x3)      << 39    # [40:39]
    inst_64bit |= (row    & 0x3FFFF)  << 21    # [38:21]
    inst_64bit |= (col    & 0x7F)     << 14    # [20:14]
    inst_64bit |= (id     & 0x7)      << 11    # [13:11]
    inst_64bit |= (etc    & 0x7FF)             # [10:0] 11b
    return inst_64bit

def acc_inst_set_base(reg_idx, base_value):
    """SET_BASE: base_reg[reg_idx] = base_value (18b row value)
    [63:60]=SET_BASE [58:56]=reg_idx(3b) [55:38]=base_value(18b)"""
    inst_64bit = 0
    inst_64bit |= (ndp_acc_inst_opcode["SET_BASE"] & 0xf) << 60  # [63:60]
    inst_64bit |= (reg_idx    & 0x7)    << 56    # [58:56]
    inst_64bit |= (base_value & 0x3FFFF) << 38   # [55:38]
    return inst_64bit

def acc_inst_inc_base(reg_idx, inc_value):
    """INC_BASE: base_reg[reg_idx] += inc_value (18b stride)
    [63:60]=INC_BASE [58:56]=reg_idx(3b) [55:38]=inc_value(18b)"""
    inst_64bit = 0
    inst_64bit |= (ndp_acc_inst_opcode["INC_BASE"] & 0xf) << 60  # [63:60]
    inst_64bit |= (reg_idx   & 0x7)    << 56     # [58:56]
    inst_64bit |= (inc_value & 0x3FFFF) << 38    # [55:38]
    return inst_64bit

def acc_inst_set_loop(cnt_reg_idx, loop_count):
    """SET_LOOP: loop_cnt_reg[cnt_reg_idx] = loop_count
    [63:60]=SET_LOOP [58:56]=cnt_reg_idx(3b) [55:40]=loop_count(16b)"""
    inst_64bit = 0
    inst_64bit |= (ndp_acc_inst_opcode["SET_LOOP"] & 0xf) << 60  # [63:60]
    inst_64bit |= (cnt_reg_idx & 0x7)    << 56    # [58:56]
    inst_64bit |= (loop_count  & 0xFFFF) << 40    # [55:40]
    return inst_64bit

def acc_inst_loop(cnt_reg_idx, jump_pc):
    """LOOP: if loop_cnt_reg[cnt_reg_idx] > 0: cnt--, PC=jump_pc; else: continue
    [63:60]=LOOP [58:56]=cnt_reg_idx(3b) [55:40]=jump_pc(16b)"""
    inst_64bit = 0
    inst_64bit |= (ndp_acc_inst_opcode["LOOP"] & 0xf) << 60  # [63:60]
    inst_64bit |= (cnt_reg_idx & 0x7)  << 56     # [58:56]
    inst_64bit |= (jump_pc    & 0xFFFF) << 40    # [55:40]
    return inst_64bit

def undirect_row(reg_idx, offset=0):
    """Encode Undirect mode row field: row[17:15]=base_reg_idx(3b), row[14:0]=offset(15b)
    Used with mode=1 in acc_inst() for Undirect addressing."""
    return ((reg_idx & 0x7) << 15) | (offset & 0x7FFF)


def dump_ndp_acc_inst(f, inst_list):
    # [UNUSED] Legacy 1D dump — replaced by dump_ndp_acc_inst_per_pch()
    # Generation Write Request for NDP Access Instruction
    num_inst = len(inst_list)
    it = int(num_inst/8)
    remain = int(num_inst)%8
    data_array = [0] * 8
    for i in range(it):
        for j in range(8):
            data_array[j] = inst_list[i*8+j]
        write_trace(f,'ST',encode_address(0, 0, 0, HSNU_CTR_BUF_BG, HSNU_CTR_BUF_BK, NDP_ROW, i),data_array)

    if remain != 0:
        data_array = [0] * 8
        for j in range(remain):
            data_array[j] = inst_list[it*8+j]       
        write_trace(f,'ST',encode_address(0, 0, 0, HSNU_CTR_BUF_BG, HSNU_CTR_BUF_BK, NDP_ROW, it),data_array)

def dump_ndp_acc_inst_2d(f, inst_list, start_col):
    # [UNUSED] Legacy 2D interleaved dump — replaced by dump_ndp_acc_inst_per_pch()
    # Generation Write Request for NDP Access Instruction
    all_acc_inst_empty = False
    col_addr = start_col
    while not all_acc_inst_empty:
        none_acc_inst = True
        for ch in range(int(NUM_CHANNEL)):
            for pch in range(int(NUM_PSEUDOCHANNEL)):  
                idx = ch * 4 + pch          
                # Check Each Inst List is Empty or not
                num_remain_inst = len(inst_list[idx])
                if num_remain_inst != 0:
                    none_acc_inst = False
                    data_array = [0] * 8
                    # Make 64B payload
                    if num_remain_inst >= 8:
                        for i in range(8):
                            data_array[i] = inst_list[idx].pop(0)
                    else :
                        for i in range(num_remain_inst):
                            data_array[i] = inst_list[idx].pop(0)  

                    # Write trace
                    write_trace(f,'ST',encode_address(ch, pch, 0, HSNU_CTR_BUF_BG, HSNU_CTR_BUF_BK, NDP_ROW, col_addr),data_array)

                    # Increase Column Address
                    col_addr+=1
                    if col_addr == 128 :
                        col_addr = 0

        if none_acc_inst:
            all_acc_inst_empty = True


def dump_ndp_acc_inst_to_dram(f, acc_inst_list):
    # [UNUSED] DRAM Spill mode dump — replaced by dump_ndp_acc_inst_per_pch() with desc_store
    """Write AccInst descriptors to DRAM for HSNC prefetch (DRAM Spill mode).

    When descriptor count exceeds PCH_DESC_BUFFER_SIZE (64), descriptors are
    written to reserved DRAM rows instead of the on-chip magic-path buffer.
    HSNC prefetches from DRAM into pch_desc_buffer during execution.

    DRAM layout per (ch, pch):
      row = NDP_DESC_SPILL_ROW_BASE, bg = NDP_DESC_SPILL_BG, bk = NDP_TARGET_BK
      col 0..N: 8 descriptors per 64B cache line (col)

    Returns desc_count_per_pch: list of descriptor counts per (ch*NUM_PCH+pch).
    """
    desc_count_per_pch = []
    for ch in range(int(NUM_CHANNEL)):
        for pch in range(int(NUM_PSEUDOCHANNEL)):
            idx = ch * int(NUM_PSEUDOCHANNEL) + pch
            inst_list = acc_inst_list[idx]
            num_inst = len(inst_list)
            desc_count_per_pch.append(num_inst)
            if num_inst == 0:
                continue

            it = int(num_inst / 8)
            remain = num_inst % 8

            col_addr = 0
            row_addr = NDP_DESC_SPILL_ROW_BASE

            for i in range(it):
                data_array = [0] * 8
                for j in range(8):
                    data_array[j] = inst_list[i * 8 + j]
                write_trace(f, 'ST', encode_address(ch, pch, 0, NDP_DESC_SPILL_BG, NDP_TARGET_BK, row_addr, col_addr), data_array)
                col_addr += 1
                if col_addr >= 128:
                    col_addr = 0
                    row_addr += 1

            if remain > 0:
                data_array = [0] * 8
                for j in range(remain):
                    data_array[j] = inst_list[it * 8 + j]
                write_trace(f, 'ST', encode_address(ch, pch, 0, NDP_DESC_SPILL_BG, NDP_TARGET_BK, row_addr, col_addr), data_array)

            print(f"  DRAM Spill: ch={ch} pch={pch} desc_count={num_inst} rows={row_addr - NDP_DESC_SPILL_ROW_BASE + 1}")

    return desc_count_per_pch


def dump_ndp_acc_inst_per_pch(f, acc_inst_list):
    """Write AccInst descriptors as per-PCH sequential streams.

    Each PCH's descriptors written contiguously to DRAM magic-path address:
      (ch, pch, HSNU_CTR_BUF_BG, HSNU_CTR_BUF_BK, NDP_ROW, col=0..127)
    Max 128 cols × 8 entries/col = 1024 descriptors per PCH.
    HSNC intercepts all writes → desc_store[pch][col][idx] 저장.
    Col 0~7은 추가로 pch_desc_buffer (Descriptor Cache)에 적재.

    Returns: list of descriptor counts per (ch*NUM_PCH+pch).
    """
    desc_counts = []
    for ch in range(int(NUM_CHANNEL)):
        for pch in range(int(NUM_PSEUDOCHANNEL)):
            idx = ch * int(NUM_PSEUDOCHANNEL) + pch
            inst_list = acc_inst_list[idx]
            num_inst = len(inst_list)
            desc_counts.append(num_inst)
            if num_inst == 0:
                continue

            if num_inst > 1024:
                print(f"Error: ch={ch} pch={pch} desc_count={num_inst} exceeds max 1024")
                exit(1)

            it = int(num_inst / 8)
            remain = num_inst % 8

            for i in range(it):
                data_array = [0] * 8
                for j in range(8):
                    data_array[j] = inst_list[i * 8 + j]
                write_trace(f, 'ST', encode_address(ch, pch, 0, HSNU_CTR_BUF_BG, HSNU_CTR_BUF_BK, NDP_ROW, i), data_array)

            if remain > 0:
                data_array = [0] * 8
                for j in range(remain):
                    data_array[j] = inst_list[it * 8 + j]
                write_trace(f, 'ST', encode_address(ch, pch, 0, HSNU_CTR_BUF_BG, HSNU_CTR_BUF_BK, NDP_ROW, it), data_array)

    return desc_counts


def write_ndp_start(f, desc_counts):
    """Write NDP Start control register with per-PCH start flag + descriptor count.

    Payload per PCH (64-bit): bit[16]=start_flag(1b), bits[15:0]=desc_count.
    Must be called AFTER all AccInst writes.

    Multi-DIMM: issues one NDP Start write per DIMM.
    C++ side: dimm_id = addr_vec[channel] / 2, so CH 0→DIMM 0, CH 2→DIMM 1, etc.
    Each DIMM has NUM_CH_PER_DIMM(2) × NUM_PSEUDOCHANNEL(4) = 8 PCHs.
    desc_counts is ordered as: [CH0_PCH0..3, CH1_PCH0..3, CH2_PCH0..3, ...].
    """
    num_pch_per_dimm = int(NUM_CH_PER_DIMM) * int(NUM_PSEUDOCHANNEL)  # 8
    for dimm_id in range(int(NUM_DIMM)):
        data_array = [0] * 8
        base_idx = dimm_id * num_pch_per_dimm
        for i in range(num_pch_per_dimm):
            if base_idx + i < len(desc_counts):
                count = desc_counts[base_idx + i]
                if count > 0:
                    data_array[i] = (1 << 16) | count   # bit[16]=start, [15:0]=count
        # Address with CH belonging to this DIMM (first CH of DIMM)
        ch_for_dimm = dimm_id * int(NUM_CH_PER_DIMM)
        write_trace(f, 'ST', encode_address(ch_for_dimm, 0, 0, HSNU_CTR_REG_BG, HSNU_CTR_REG_BK, NDP_ROW, 0), data_array)


def dump_ndp_inst(f, inst_list, ch, pch):
    # Generation Write Request for NDP Instruction (DIMM-side)
    num_inst = len(inst_list)
    it = int(num_inst/8)
    remain = int(num_inst)%8
    data_array = [0] * 8
    for i in range(it):
        for j in range(8):
            data_array[j] = inst_list[i*8+j]
        write_trace(f,'ST',encode_address(ch, pch, 0, NDP_INS_MEM_BG, NDP_INS_MEM_BK, NDP_ROW, i),data_array)

    if remain != 0:
        data_array = [0] * 8
        for j in range(remain):
            data_array[j] = inst_list[it*8+j]       
        write_trace(f,'ST',encode_address(ch, pch, 0, NDP_INS_MEM_BG, NDP_INS_MEM_BK, NDP_ROW, it),data_array)

def gen_normal_req_from_row(f,start_row,num_req,req_type):
    cnt_req = 0
    done = False
    if SPLIT_HIGH_BANK:
        if LINEAR_ACCESS and NORMAL_ADDRESS_SCHEME == "BhRoBlBgCoRaCh":
            for ro in range(NUM_ROW):
                for ba in range(int(NUM_BANK/2),int(NUM_BANK-1)):
                    for bg in range(NUM_BANKGROUP):
                        for co in range(NUM_COL): 
                            for rk in range(NUM_RANK):        
                                for ch in range(NUM_CHANNEL):
                                    write_normal_trace(f,req_type,encode_normal_address(ch, rk, bg, ba, start_row+ro, co))
                                    cnt_req+=1
                                    if cnt_req == num_req:
                                        done = True
                                    
                                    if done == True:
                                        break
                                if done == True:
                                    break    
                            if done == True:
                                break 
                        if done == True:
                            break
                    if done == True:
                        break
                if done == True:
                    break       
        elif FIX_BANK:
            print("not supported FIX BANK with SPLIT_HIGH_BANK")
            exit(1)                            
        else:
            for ro in range(NUM_ROW):
                for ba in range(int(NUM_BANK/2),int(NUM_BANK-1)):
                    for co in range(NUM_COL): 
                        for bg in range(NUM_BANKGROUP):
                            for rk in range(NUM_RANK):        
                                for ch in range(NUM_CHANNEL):
                                    write_normal_trace(f,req_type,encode_normal_address(ch, rk, bg, ba, start_row+ro, co))
                                    cnt_req+=1
                                    if cnt_req == num_req:
                                        done = True
                                    
                                    if done == True:
                                        break
                                if done == True:
                                    break    
                            if done == True:
                                break 
                        if done == True:
                            break
                    if done == True:
                        break
                if done == True:
                    break        
    elif GEN_BGCH:
        if FIX_BANK:
            print("not supported FIX BANK with SPLIT_HIGH_BANK")
            exit(1)
        else:
            for ro in range(NUM_ROW):
                for ba in range(NUM_BANK):
                    for co in range(NUM_COL): 
                        for rk in range(NUM_RANK):        
                            for ch in range(NUM_CHANNEL):
                                for bg in range(NUM_BANKGROUP):
                                    write_normal_trace(f,req_type,encode_normal_address(ch, rk, bg, ba, start_row+ro, co))
                                    cnt_req+=1
                                    if cnt_req == num_req:
                                        done = True
                                    
                                    if done == True:
                                        break
                                if done == True:
                                    break    
                            if done == True:
                                break 
                        if done == True:
                            break
                    if done == True:
                        break
                if done == True:
                    break             
    else:
        if LINEAR_ACCESS and NORMAL_ADDRESS_SCHEME == "RoBaCoRaCh":
            for ro in range(NUM_ROW):
                for ba in range(NUM_BANK):
                    for bg in range(NUM_BANKGROUP):
                        for co in range(NUM_COL): 
                            for rk in range(NUM_RANK):        
                                for ch in range(NUM_CHANNEL):
                                    write_normal_trace(f,req_type,encode_normal_address(ch, rk, bg, ba, start_row+ro, co))
                                    cnt_req+=1
                                    if cnt_req == num_req:
                                        done = True
                                    
                                    if done == True:
                                        break
                                if done == True:
                                    break    
                            if done == True:
                                break 
                        if done == True:
                            break
                    if done == True:
                        break
                if done == True:
                    break        
        else:
            if FIX_BANK:
                for ro in range(NUM_ROW):
                    for co in range(NUM_COL): 
                        for bg in range(NUM_BANKGROUP):
                            for rk in range(NUM_RANK):        
                                for ch in range(NUM_CHANNEL):
                                    write_normal_trace(f,req_type,encode_normal_address(ch, rk, bg, NDP_TARGET_BK, start_row+ro, co))
                                    cnt_req+=1
                                    if cnt_req == num_req:
                                        done = True
                                    
                                    if done == True:
                                        break
                                if done == True:
                                    break    
                            if done == True:
                                break 
                        if done == True:
                            break
                    if done == True:
                        break                 
            else:
                for ro in range(NUM_ROW):
                    for ba in range(NUM_BANK):
                        for co in range(NUM_COL): 
                            for bg in range(NUM_BANKGROUP):
                                for rk in range(NUM_RANK):        
                                    for ch in range(NUM_CHANNEL):
                                        write_normal_trace(f,req_type,encode_normal_address(ch, rk, bg, ba, start_row+ro, co))
                                        cnt_req+=1
                                        if cnt_req == num_req:
                                            done = True
                                        
                                        if done == True:
                                            break
                                    if done == True:
                                        break    
                                if done == True:
                                    break 
                            if done == True:
                                break
                        if done == True:
                            break
                    if done == True:
                        break        

def gen_normal_req_from_row_pch(f,start_row,num_req,req_type):
    cnt_req = 0
    done = False
    if SPLIT_HIGH_BANK:    
        if LINEAR_ACCESS and PCH_ADDRESS_SCHEME == "BhRoBlBgCoRaPcCH":
            for ro in range(NUM_ROW):
                for ba in range(int(NUM_BANK/2),int(NUM_BANK-1)):
                    for bg in range(NUM_BANKGROUP):
                        for co in range(NUM_COL): 
                            for pch in range(int(NUM_PSEUDOCHANNEL)):        
                                for ch in range(NUM_CHANNEL):
                                    write_normal_trace(f,req_type,encode_address(ch, pch, 0, bg, ba, start_row+ro, co))                                                            
                                    cnt_req+=1
                                    if cnt_req == num_req:
                                        done = True
                                    
                                    if done == True:
                                        break
                                if done == True:
                                    break    
                            if done == True:
                                break 
                        if done == True:
                            break
                    if done == True:
                        break
                if done == True:
                    break    
        elif FIX_BANK:
            print("not supported FIX BANK with SPLIT_HIGH_BANK")
            exit(1)                                 
        else:
            for ro in range(NUM_ROW):
                for ba in range(int(NUM_BANK/2),int(NUM_BANK-1)):
                    for co in range(NUM_COL): 
                        for bg in range(NUM_BANKGROUP):
                            for pch in range(int(NUM_PSEUDOCHANNEL)):        
                                for ch in range(NUM_CHANNEL):
                                    write_normal_trace(f,req_type,encode_address(ch, pch, 0, bg, ba, start_row+ro, co))                                                            
                                    cnt_req+=1
                                    if cnt_req == num_req:
                                        done = True
                                    
                                    if done == True:
                                        break
                                if done == True:
                                    break    
                            if done == True:
                                break 
                        if done == True:
                            break
                    if done == True:
                        break
                if done == True:
                    break     
    elif GEN_BGCH:
        if FIX_BANK:
            print("not supported FIX BANK with SPLIT_HIGH_BANK")
            exit(1)
        else:
            for ro in range(NUM_ROW):
                for ba in range(NUM_BANK):
                    for co in range(NUM_COL): 
                        for pch in range(int(NUM_PSEUDOCHANNEL)):        
                            for ch in range(NUM_CHANNEL):
                                for bg in range(NUM_BANKGROUP):
                                    write_normal_trace(f,req_type,encode_address(ch, pch, 0, bg, ba, start_row+ro, co))                                                                
                                    cnt_req+=1
                                    if cnt_req == num_req:
                                        done = True
                                    
                                    if done == True:
                                        break
                                if done == True:
                                    break    
                            if done == True:
                                break 
                        if done == True:
                            break
                    if done == True:
                        break
                if done == True:
                    break   
    else:
        if LINEAR_ACCESS and PCH_ADDRESS_SCHEME == "RoBaBgCoRaPcCH":
            for ro in range(NUM_ROW):
                for ba in range(NUM_BANK):
                    for bg in range(NUM_BANKGROUP):
                        for co in range(NUM_COL): 
                            for pch in range(int(NUM_PSEUDOCHANNEL)):        
                                for ch in range(NUM_CHANNEL):
                                    write_normal_trace(f,req_type,encode_address(ch, pch, 0, bg, ba, start_row+ro, co))
                                                                
                                    cnt_req+=1
                                    if cnt_req == num_req:
                                        done = True
                                    
                                    if done == True:
                                        break
                                if done == True:
                                    break    
                            if done == True:
                                break 
                        if done == True:
                            break
                    if done == True:
                        break
                if done == True:
                    break       
        else:
            if FIX_BANK:
                for ro in range(NUM_ROW):
                    for co in range(NUM_COL): 
                        for bg in range(NUM_BANKGROUP):
                            for pch in range(int(NUM_PSEUDOCHANNEL)):        
                                for ch in range(NUM_CHANNEL):
                                    write_normal_trace(f,req_type,encode_address(ch, pch, 0, bg, NDP_TARGET_BK, start_row+ro, co))                                                                
                                    cnt_req+=1
                                    if cnt_req == num_req:
                                        done = True
                                    
                                    if done == True:
                                        break
                                if done == True:
                                    break    
                            if done == True:
                                break 
                        if done == True:
                            break
                    if done == True:
                        break                   
            else:
                for ro in range(NUM_ROW):
                    for ba in range(NUM_BANK):
                        for co in range(NUM_COL): 
                            for bg in range(NUM_BANKGROUP):
                                for pch in range(int(NUM_PSEUDOCHANNEL)):        
                                    for ch in range(NUM_CHANNEL):
                                        write_normal_trace(f,req_type,encode_address(ch, pch, 0, bg, ba, start_row+ro, co))                                                                
                                        cnt_req+=1
                                        if cnt_req == num_req:
                                            done = True
                                        
                                        if done == True:
                                            break
                                    if done == True:
                                        break    
                                if done == True:
                                    break 
                            if done == True:
                                break
                        if done == True:
                            break
                    if done == True:
                        break       
                          
def cal_it(input_size, scaling):
    # [UNUSED] Legacy iteration calculator — replaced by cal_it_v2()
    if scaling == 4:
        row_size = 8192 * 2
    else:
        row_size = 8192

    # Max opsize varies depending on the scaling factor
    if scaling == 1:
        opsize = 127
    else:
        opsize = 63

    # Input Size에 따라, 필요한 Row size 및 Working Bank Group이 다름. 
    num_row = int(input_size_byte_list[input_size]/row_size)
    if scaling == 1:
        # Data Memory 용량에 의해 최대 동작 가능한 Row Size 4
        if num_row > 4:
            num_working_bg = 4
            iteration = int(num_row / 4)
        else:
            num_working_bg = num_row
            iteration      = 1
    elif scaling == 2:
        # 최대 동작 가능한 Row size 8 (Data memory 증가로 인해 증가)
        if num_row > 4:
            num_working_bg = 4
            iteration = int(num_row / 4)
        else:
            num_working_bg = num_row
            iteration      = 1
    elif scaling == 4:
        # 최대 동작 가능한 Row size 4 (Bank Group 크기 4)
        if num_row > 4:
            num_working_bg = 4
            iteration = int(num_row / 4)
        elif num_row == 0:
            num_working_bg = 1
            iteration      = 1
            opsize         = 31           
        else:
            num_working_bg = num_row
            iteration      = 1   
    return iteration, num_working_bg, opsize

def cal_it_v2(input_size, scaling):
    if scaling == 4:
        row_size = 8192 * 2
    else:
        row_size = 8192

    # Max opsize varies depending on the scaling factor
    if scaling == 1:
        opsize = 128
    else:
        opsize = 64

    # Input Size에 따라, 필요한 Row size 및 Working Bank Group이 다름. 
    num_row = int(input_size_byte_list[input_size]/row_size)
    vec_size = int(input_size_byte_list[input_size])

    if scaling == 1:
        num_working_bg = 8
    elif scaling == 2:
        num_working_bg = 8
    elif scaling == 4:
        num_working_bg = 4

    if scaling == 1:
        # Data Memory 32KB
        # Max n_BG x ROW / 2
        max_size = num_working_bg * row_size / 2
        if vec_size > max_size:
            ratio     = int(vec_size / max_size)
            iteration = ratio
            opsize    = int(opsize / 2)
        else:
            ratio     = int(max_size / vec_size)
            iteration = 1
            opsize    = int(opsize / 2 / ratio)
    elif scaling == 2:
        # Data Memory 64KB
        # Max n_BG x ROW  
        max_size = num_working_bg * row_size 
        if vec_size > max_size:
            ratio     = int(vec_size / max_size)
            iteration = ratio
            opsize    = int(opsize)
        else:
            ratio     = int(max_size / vec_size)
            iteration = 1
            opsize    = int(opsize / ratio)
    elif scaling == 4:
        # Data Memory 128KB
        # Max n_BG x ROW x 2 (but, use 4 Row)
        max_size = num_working_bg * row_size 
        if vec_size > max_size:
            ratio     = int(vec_size / max_size)
            iteration = ratio
            opsize    = int(opsize)
        else:
            ratio     = int(max_size / vec_size)
            iteration = 1
            opsize    = int(opsize / ratio)        

    return iteration, num_working_bg, (opsize - 1)


def cal_it_v3(per_pch_bytes, scaling):
    """Compute iteration, num_working_bg, opsize from per-PCH byte size.

    Same logic as cal_it_v2 but accepts byte size directly instead of a label.
    This enables fixed total data across different DIMM counts.

    Args:
        per_pch_bytes: data size in bytes per PCH (from calc_per_pch_bytes)
        scaling: DRAM scaling factor (1=x4, 2=x8, 4=x16)

    Returns:
        (iteration, num_working_bg, opsize)
        opsize is 0-indexed: opsize=N means N+1 column accesses.
    """
    if scaling == 4:
        row_size = 8192 * 2   # x16: 16KB/row
    else:
        row_size = 8192        # x4/x8: 8KB/row

    # Max opsize (columns per row)
    if scaling == 1:
        opsize = 128           # x4: 128 cols × 64B = 8KB
    else:
        opsize = 64            # x8: 64 cols × 128B = 8KB, x16: 64 cols × 256B = 16KB

    vec_size = per_pch_bytes

    # num_working_bg: max BG count for NDP data memory
    if scaling == 1:
        num_working_bg = 8
    elif scaling == 2:
        num_working_bg = 8
    elif scaling == 4:
        num_working_bg = 4

    # Data memory capacity determines iteration vs opsize tradeoff
    if scaling == 1:
        # Data Memory 32KB = n_BG × ROW / 2
        max_size = num_working_bg * row_size // 2
        if vec_size > max_size:
            ratio     = vec_size // max_size
            iteration = ratio
            opsize    = opsize // 2
        else:
            ratio     = max_size // vec_size
            iteration = 1
            opsize    = opsize // 2 // ratio
    elif scaling == 2:
        # Data Memory 64KB = n_BG × ROW
        max_size = num_working_bg * row_size
        if vec_size > max_size:
            ratio     = vec_size // max_size
            iteration = ratio
            opsize    = opsize
        else:
            ratio     = max_size // vec_size
            iteration = 1
            opsize    = opsize // ratio
    elif scaling == 4:
        # Data Memory 64KB = n_BG × ROW (4 BG × 16KB)
        max_size = num_working_bg * row_size
        if vec_size > max_size:
            ratio     = vec_size // max_size
            iteration = ratio
            opsize    = opsize
        else:
            ratio     = max_size // vec_size
            iteration = 1
            opsize    = opsize // ratio

    # Clamp opsize minimum to 1 column access
    if opsize < 1:
        opsize = 1

    return iteration, num_working_bg, (opsize - 1)


'''
    AXPBY     : Z = aX + bY
    AXPBYPCZ  : W = aX + bB + zZ
    AXPY      : Y = aY + X
    COPY      : Y = X
    XMY       : Y = X ⨀ Y
    DOT       : c = X·Y
    SCAL      : X = aX
    GEMV      : y = AX

    size: 8KB/32KB/128KB/512KB/8MB per Pseudo-Channel 
    
'''

# Scaling Factor: 1 (x4), 2 (x8), 4 (x16)
def axpby_pch(f, per_pch_bytes, scaling):
    '''DBX-DIMM AXPBY: Z = aX + bY (BK interleaving)'''
    iteration, num_working_bg, opsize = cal_it_v3(per_pch_bytes, scaling)
    num_banks = int(NUM_BANK)
    if iteration >= num_banks:
        iterations_per_bk = iteration // num_banks
    else:
        iterations_per_bk = 1
        num_banks = iteration  # Use fewer banks when iteration < num_banks
    print(f"Working Bank Group: {num_working_bg}")
    print(f"Iteration: {iteration}, iter_per_bk: {iterations_per_bk}")

    # Inst (NDP Unit) — BK-unrolled
    for ch in range(int(NUM_CHANNEL)):
        for pch in range(int(NUM_PSEUDOCHANNEL)):
            ndp_inst_list = []
            for bk in range(num_banks):
                iters_this_bk = iterations_per_bk + (1 if bk < (iteration % num_banks) else 0)
                if iters_this_bk == 0: continue
                jump_pc = len(ndp_inst_list)
                for bg in range(num_working_bg):
                    ndp_inst_list.append(inst(ndp_inst_opcode["LOAD_MUL"],opsize,0,bg,bk,0,0,0))
                for bg in range(num_working_bg):
                    ndp_inst_list.append(inst(ndp_inst_opcode["SCALE_ADD"],opsize,1,bg,bk,0,0,0))
                for bg in range(num_working_bg):
                    ndp_inst_list.append(inst(ndp_inst_opcode["WBD"],opsize,2,bg,bk,0,0,0))
                if iters_this_bk > 1:
                    ndp_inst_list.append(inst(ndp_inst_opcode["LOOP"],0,0,0,0,(iters_this_bk-1),jump_pc,0))
            ndp_inst_list.append(inst(ndp_inst_opcode["EXIT"],0,0,0,0,0,0,0))
            if len(ndp_inst_list) >= 1024:
                print("Error: Over NDP Instruction Memory")
                exit(1)
            dump_ndp_inst(f,ndp_inst_list,ch,pch)

    # AccInst (Host DRAM access) — BK-unrolled
    acc_inst_list = [[]]
    acc_inst_list_num = [[]]
    for ch in range(int(NUM_CHANNEL)):
        for pch in range(int(NUM_PSEUDOCHANNEL)):
            acc_inst_list.append([])
            acc_inst_list_num.append(0)
    for ch in range(int(NUM_CHANNEL)):
        for pch in range(int(NUM_PSEUDOCHANNEL)):
            idx = ch * int(NUM_PSEUDOCHANNEL) + pch
            for bk in range(num_banks):
                iters_this_bk = iterations_per_bk + (1 if bk < (iteration % num_banks) else 0)
                if iters_this_bk == 0: continue
                acc_inst_list[idx].append(acc_inst_set_base(0, 5000))
                acc_inst_list[idx].append(acc_inst_set_base(1, 6000))
                acc_inst_list[idx].append(acc_inst_set_base(2, 7000))
                acc_inst_list[idx].append(acc_inst_set_loop(0, max(0, iters_this_bk - 1)))
                jump_pc = len(acc_inst_list[idx])
                for bg in range(num_working_bg):
                    acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["RD"],opsize,ch,pch,bg,bk,undirect_row(0,0),0,0,0,mode=1))
                acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["BAR"],0,0,0,0,0,0,0,0,0))
                for bg in range(num_working_bg):
                    acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["RD"],opsize,ch,pch,bg,bk,undirect_row(1,0),0,1,0,mode=1))
                acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["BAR"],0,0,0,0,0,0,0,0,0))
                for bg in range(num_working_bg):
                    acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["WR"],opsize,ch,pch,bg,bk,undirect_row(2,0),0,2,0,mode=1))
                acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["BAR"],0,0,0,0,0,0,0,0,0))
                if iters_this_bk > 1:
                    acc_inst_list[idx].append(acc_inst_inc_base(0, 1))
                    acc_inst_list[idx].append(acc_inst_inc_base(1, 1))
                    acc_inst_list[idx].append(acc_inst_inc_base(2, 1))
                    acc_inst_list[idx].append(acc_inst_loop(0, jump_pc))
            acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["DONE"],0,0,0,0,0,0,0,0,0))
    desc_counts = dump_ndp_acc_inst_per_pch(f, acc_inst_list)
    write_ndp_start(f, desc_counts)
    return

# AXPBYPCZ  : W = aX + bB + zZ
def axpbypcz_pch(f, per_pch_bytes, scaling):
    '''DBX-DIMM AXPBYPCZ: W = aX + bB + cZ (BK interleaving)'''
    iteration, num_working_bg, opsize = cal_it_v3(per_pch_bytes, scaling)
    num_banks = int(NUM_BANK)
    if iteration >= num_banks:
        iterations_per_bk = iteration // num_banks
    else:
        iterations_per_bk = 1
        num_banks = iteration  # Use fewer banks when iteration < num_banks
    print(f"Working Bank Group: {num_working_bg}")
    print(f"Iteration: {iteration}, iter_per_bk: {iterations_per_bk}")

    for ch in range(int(NUM_CHANNEL)):
        for pch in range(int(NUM_PSEUDOCHANNEL)):
            ndp_inst_list = []
            for bk in range(num_banks):
                iters_this_bk = iterations_per_bk + (1 if bk < (iteration % num_banks) else 0)
                if iters_this_bk == 0: continue
                jump_pc = len(ndp_inst_list)
                for bg in range(num_working_bg):
                    ndp_inst_list.append(inst(ndp_inst_opcode["LOAD_MUL"],opsize,0,bg,bk,0,0,0))
                for bg in range(num_working_bg):
                    ndp_inst_list.append(inst(ndp_inst_opcode["SCALE_ADD"],opsize,1,bg,bk,0,0,0))
                for bg in range(num_working_bg):
                    ndp_inst_list.append(inst(ndp_inst_opcode["SCALE_ADD"],opsize,2,bg,bk,0,0,0))
                for bg in range(num_working_bg):
                    ndp_inst_list.append(inst(ndp_inst_opcode["WBD"],opsize,3,bg,bk,0,0,0))
                if iters_this_bk > 1:
                    ndp_inst_list.append(inst(ndp_inst_opcode["LOOP"],0,0,0,0,(iters_this_bk-1),jump_pc,0))
            ndp_inst_list.append(inst(ndp_inst_opcode["EXIT"],0,0,0,0,0,0,0))
            if len(ndp_inst_list) >= 1024:
                print("Error: Over NDP Instruction Memory")
                exit(1)
            dump_ndp_inst(f,ndp_inst_list,ch,pch)

    acc_inst_list = [[]]
    acc_inst_list_num = [[]]
    for ch in range(int(NUM_CHANNEL)):
        for pch in range(int(NUM_PSEUDOCHANNEL)):
            acc_inst_list.append([])
            acc_inst_list_num.append(0)
    for ch in range(int(NUM_CHANNEL)):
        for pch in range(int(NUM_PSEUDOCHANNEL)):
            idx = ch * int(NUM_PSEUDOCHANNEL) + pch
            for bk in range(num_banks):
                iters_this_bk = iterations_per_bk + (1 if bk < (iteration % num_banks) else 0)
                if iters_this_bk == 0: continue
                acc_inst_list[idx].append(acc_inst_set_base(0, 5000))
                acc_inst_list[idx].append(acc_inst_set_base(1, 6000))
                acc_inst_list[idx].append(acc_inst_set_base(2, 7000))
                acc_inst_list[idx].append(acc_inst_set_base(3, 8000))
                acc_inst_list[idx].append(acc_inst_set_loop(0, max(0, iters_this_bk - 1)))
                jump_pc = len(acc_inst_list[idx])
                for bg in range(num_working_bg):
                    acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["RD"],opsize,ch,pch,bg,bk,undirect_row(0,0),0,0,0,mode=1))
                acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["BAR"],0,0,0,0,0,0,0,0,0))
                for bg in range(num_working_bg):
                    acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["RD"],opsize,ch,pch,bg,bk,undirect_row(1,0),0,1,0,mode=1))
                acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["BAR"],0,0,0,0,0,0,0,0,0))
                for bg in range(num_working_bg):
                    acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["RD"],opsize,ch,pch,bg,bk,undirect_row(2,0),0,2,0,mode=1))
                acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["BAR"],0,0,0,0,0,0,0,0,0))
                for bg in range(num_working_bg):
                    acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["WR"],opsize,ch,pch,bg,bk,undirect_row(3,0),0,3,0,mode=1))
                acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["BAR"],0,0,0,0,0,0,0,0,0))
                if iters_this_bk > 1:
                    acc_inst_list[idx].append(acc_inst_inc_base(0, 1))
                    acc_inst_list[idx].append(acc_inst_inc_base(1, 1))
                    acc_inst_list[idx].append(acc_inst_inc_base(2, 1))
                    acc_inst_list[idx].append(acc_inst_inc_base(3, 1))
                    acc_inst_list[idx].append(acc_inst_loop(0, jump_pc))
            acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["DONE"],0,0,0,0,0,0,0,0,0))
    desc_counts = dump_ndp_acc_inst_per_pch(f, acc_inst_list)
    write_ndp_start(f, desc_counts)
    return

# AXPY      : Y = aY + X
def axpy_pch(f, per_pch_bytes, scaling):
    '''DBX-DIMM AXPY: Z = aX + Y (BK interleaving)'''
    iteration, num_working_bg, opsize = cal_it_v3(per_pch_bytes, scaling)
    num_banks = int(NUM_BANK)
    if iteration >= num_banks:
        iterations_per_bk = iteration // num_banks
    else:
        iterations_per_bk = 1
        num_banks = iteration  # Use fewer banks when iteration < num_banks
    print(f"Working Bank Group: {num_working_bg}")
    print(f"Iteration: {iteration}, iter_per_bk: {iterations_per_bk}")

    for ch in range(int(NUM_CHANNEL)):
        for pch in range(int(NUM_PSEUDOCHANNEL)):
            ndp_inst_list = []
            for bk in range(num_banks):
                iters_this_bk = iterations_per_bk + (1 if bk < (iteration % num_banks) else 0)
                if iters_this_bk == 0: continue
                jump_pc = len(ndp_inst_list)
                for bg in range(num_working_bg):
                    ndp_inst_list.append(inst(ndp_inst_opcode["LOAD_MUL"],opsize,0,bg,bk,0,0,0))
                for bg in range(num_working_bg):
                    ndp_inst_list.append(inst(ndp_inst_opcode["ADD"],opsize,1,bg,bk,0,0,0))
                for bg in range(num_working_bg):
                    ndp_inst_list.append(inst(ndp_inst_opcode["WBD"],opsize,2,bg,bk,0,0,0))
                if iters_this_bk > 1:
                    ndp_inst_list.append(inst(ndp_inst_opcode["LOOP"],0,0,0,0,(iters_this_bk-1),jump_pc,0))
            ndp_inst_list.append(inst(ndp_inst_opcode["EXIT"],0,0,0,0,0,0,0))
            if len(ndp_inst_list) >= 1024:
                print("Error: Over NDP Instruction Memory")
                exit(1)
            dump_ndp_inst(f,ndp_inst_list,ch,pch)

    acc_inst_list = [[]]
    acc_inst_list_num = [[]]
    for ch in range(int(NUM_CHANNEL)):
        for pch in range(int(NUM_PSEUDOCHANNEL)):
            acc_inst_list.append([])
            acc_inst_list_num.append(0)
    for ch in range(int(NUM_CHANNEL)):
        for pch in range(int(NUM_PSEUDOCHANNEL)):
            idx = ch * int(NUM_PSEUDOCHANNEL) + pch
            for bk in range(num_banks):
                iters_this_bk = iterations_per_bk + (1 if bk < (iteration % num_banks) else 0)
                if iters_this_bk == 0: continue
                acc_inst_list[idx].append(acc_inst_set_base(0, 5000))
                acc_inst_list[idx].append(acc_inst_set_base(1, 6000))
                acc_inst_list[idx].append(acc_inst_set_base(2, 7000))
                acc_inst_list[idx].append(acc_inst_set_loop(0, max(0, iters_this_bk - 1)))
                jump_pc = len(acc_inst_list[idx])
                for bg in range(num_working_bg):
                    acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["RD"],opsize,ch,pch,bg,bk,undirect_row(0,0),0,0,0,mode=1))
                acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["BAR"],0,0,0,0,0,0,0,0,0))
                for bg in range(num_working_bg):
                    acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["RD"],opsize,ch,pch,bg,bk,undirect_row(1,0),0,1,0,mode=1))
                acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["BAR"],0,0,0,0,0,0,0,0,0))
                for bg in range(num_working_bg):
                    acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["WR"],opsize,ch,pch,bg,bk,undirect_row(2,0),0,2,0,mode=1))
                acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["BAR"],0,0,0,0,0,0,0,0,0))
                if iters_this_bk > 1:
                    acc_inst_list[idx].append(acc_inst_inc_base(0, 1))
                    acc_inst_list[idx].append(acc_inst_inc_base(1, 1))
                    acc_inst_list[idx].append(acc_inst_inc_base(2, 1))
                    acc_inst_list[idx].append(acc_inst_loop(0, jump_pc))
            acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["DONE"],0,0,0,0,0,0,0,0,0))
    desc_counts = dump_ndp_acc_inst_per_pch(f, acc_inst_list)
    write_ndp_start(f, desc_counts)
    return

# COPY      : Y = X
def copy_pch(f, per_pch_bytes, scaling):
    '''DBX-DIMM COPY: Z = X (BK interleaving)'''
    iteration, num_working_bg, opsize = cal_it_v3(per_pch_bytes, scaling)
    num_banks = int(NUM_BANK)
    if iteration >= num_banks:
        iterations_per_bk = iteration // num_banks
    else:
        iterations_per_bk = 1
        num_banks = iteration  # Use fewer banks when iteration < num_banks
    print(f"Working Bank Group: {num_working_bg}")
    print(f"Iteration: {iteration}, iter_per_bk: {iterations_per_bk}")

    for ch in range(int(NUM_CHANNEL)):
        for pch in range(int(NUM_PSEUDOCHANNEL)):
            ndp_inst_list = []
            for bk in range(num_banks):
                iters_this_bk = iterations_per_bk + (1 if bk < (iteration % num_banks) else 0)
                if iters_this_bk == 0: continue
                jump_pc = len(ndp_inst_list)
                for bg in range(num_working_bg):
                    ndp_inst_list.append(inst(ndp_inst_opcode["LOAD"],opsize,0,bg,bk,0,0,0))
                for bg in range(num_working_bg):
                    ndp_inst_list.append(inst(ndp_inst_opcode["WBD"],opsize,2,bg,bk,0,0,0))
                if iters_this_bk > 1:
                    ndp_inst_list.append(inst(ndp_inst_opcode["LOOP"],0,0,0,0,(iters_this_bk-1),jump_pc,0))
            ndp_inst_list.append(inst(ndp_inst_opcode["EXIT"],0,0,0,0,0,0,0))
            if len(ndp_inst_list) >= 1024:
                print("Error: Over NDP Instruction Memory")
                exit(1)
            dump_ndp_inst(f,ndp_inst_list,ch,pch)

    acc_inst_list = [[]]
    acc_inst_list_num = [[]]
    for ch in range(int(NUM_CHANNEL)):
        for pch in range(int(NUM_PSEUDOCHANNEL)):
            acc_inst_list.append([])
            acc_inst_list_num.append(0)
    for ch in range(int(NUM_CHANNEL)):
        for pch in range(int(NUM_PSEUDOCHANNEL)):
            idx = ch * int(NUM_PSEUDOCHANNEL) + pch
            for bk in range(num_banks):
                iters_this_bk = iterations_per_bk + (1 if bk < (iteration % num_banks) else 0)
                if iters_this_bk == 0: continue
                acc_inst_list[idx].append(acc_inst_set_base(0, 5000))
                acc_inst_list[idx].append(acc_inst_set_base(1, 7000))
                acc_inst_list[idx].append(acc_inst_set_loop(0, max(0, iters_this_bk - 1)))
                jump_pc = len(acc_inst_list[idx])
                for bg in range(num_working_bg):
                    acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["RD"],opsize,ch,pch,bg,bk,undirect_row(0,0),0,0,0,mode=1))
                acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["BAR"],0,0,0,0,0,0,0,0,0))
                for bg in range(num_working_bg):
                    acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["WR"],opsize,ch,pch,bg,bk,undirect_row(1,0),0,2,0,mode=1))
                acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["BAR"],0,0,0,0,0,0,0,0,0))
                if iters_this_bk > 1:
                    acc_inst_list[idx].append(acc_inst_inc_base(0, 1))
                    acc_inst_list[idx].append(acc_inst_inc_base(1, 1))
                    acc_inst_list[idx].append(acc_inst_loop(0, jump_pc))
            acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["DONE"],0,0,0,0,0,0,0,0,0))
    desc_counts = dump_ndp_acc_inst_per_pch(f, acc_inst_list)
    write_ndp_start(f, desc_counts)
    return


# XMY       : Y = X ⨀ Y
def xmy_pch(f, per_pch_bytes, scaling):
    '''DBX-DIMM XMY: Z = X ⨀ Y (BK interleaving)'''
    iteration, num_working_bg, opsize = cal_it_v3(per_pch_bytes, scaling)
    num_banks = int(NUM_BANK)
    if iteration >= num_banks:
        iterations_per_bk = iteration // num_banks
    else:
        iterations_per_bk = 1
        num_banks = iteration  # Use fewer banks when iteration < num_banks
    print(f"Working Bank Group: {num_working_bg}")
    print(f"Iteration: {iteration}, iter_per_bk: {iterations_per_bk}")

    for ch in range(int(NUM_CHANNEL)):
        for pch in range(int(NUM_PSEUDOCHANNEL)):
            ndp_inst_list = []
            for bk in range(num_banks):
                iters_this_bk = iterations_per_bk + (1 if bk < (iteration % num_banks) else 0)
                if iters_this_bk == 0: continue
                jump_pc = len(ndp_inst_list)
                for bg in range(num_working_bg):
                    ndp_inst_list.append(inst(ndp_inst_opcode["LOAD"],opsize,0,bg,bk,0,0,0))
                for bg in range(num_working_bg):
                    ndp_inst_list.append(inst(ndp_inst_opcode["MUL"],opsize,1,bg,bk,0,0,0))
                for bg in range(num_working_bg):
                    ndp_inst_list.append(inst(ndp_inst_opcode["WBD"],opsize,2,bg,bk,0,0,0))
                if iters_this_bk > 1:
                    ndp_inst_list.append(inst(ndp_inst_opcode["LOOP"],0,0,0,0,(iters_this_bk-1),jump_pc,0))
            ndp_inst_list.append(inst(ndp_inst_opcode["EXIT"],0,0,0,0,0,0,0))
            if len(ndp_inst_list) >= 1024:
                print("Error: Over NDP Instruction Memory")
                exit(1)
            dump_ndp_inst(f,ndp_inst_list,ch,pch)

    acc_inst_list = [[]]
    acc_inst_list_num = [[]]
    for ch in range(int(NUM_CHANNEL)):
        for pch in range(int(NUM_PSEUDOCHANNEL)):
            acc_inst_list.append([])
            acc_inst_list_num.append(0)
    for ch in range(int(NUM_CHANNEL)):
        for pch in range(int(NUM_PSEUDOCHANNEL)):
            idx = ch * int(NUM_PSEUDOCHANNEL) + pch
            for bk in range(num_banks):
                iters_this_bk = iterations_per_bk + (1 if bk < (iteration % num_banks) else 0)
                if iters_this_bk == 0: continue
                acc_inst_list[idx].append(acc_inst_set_base(0, 5000))
                acc_inst_list[idx].append(acc_inst_set_base(1, 6000))
                acc_inst_list[idx].append(acc_inst_set_base(2, 7000))
                acc_inst_list[idx].append(acc_inst_set_loop(0, max(0, iters_this_bk - 1)))
                jump_pc = len(acc_inst_list[idx])
                for bg in range(num_working_bg):
                    acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["RD"],opsize,ch,pch,bg,bk,undirect_row(0,0),0,0,0,mode=1))
                acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["BAR"],0,0,0,0,0,0,0,0,0))
                for bg in range(num_working_bg):
                    acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["RD"],opsize,ch,pch,bg,bk,undirect_row(1,0),0,1,0,mode=1))
                acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["BAR"],0,0,0,0,0,0,0,0,0))
                for bg in range(num_working_bg):
                    acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["WR"],opsize,ch,pch,bg,bk,undirect_row(2,0),0,2,0,mode=1))
                acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["BAR"],0,0,0,0,0,0,0,0,0))
                if iters_this_bk > 1:
                    acc_inst_list[idx].append(acc_inst_inc_base(0, 1))
                    acc_inst_list[idx].append(acc_inst_inc_base(1, 1))
                    acc_inst_list[idx].append(acc_inst_inc_base(2, 1))
                    acc_inst_list[idx].append(acc_inst_loop(0, jump_pc))
            acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["DONE"],0,0,0,0,0,0,0,0,0))
    desc_counts = dump_ndp_acc_inst_per_pch(f, acc_inst_list)
    write_ndp_start(f, desc_counts)
    return

# DOT       : c = X·Y
def dot_pch(f, per_pch_bytes, scaling):
    '''DBX-DIMM DOT: c = X·Y (BK interleaving)'''
    iteration, num_working_bg, opsize = cal_it_v3(per_pch_bytes, scaling)
    num_banks = int(NUM_BANK)
    if iteration >= num_banks:
        iterations_per_bk = iteration // num_banks
    else:
        iterations_per_bk = 1
        num_banks = iteration  # Use fewer banks when iteration < num_banks
    print(f"Working Bank Group: {num_working_bg}")
    print(f"Iteration: {iteration}, iter_per_bk: {iterations_per_bk}")
    print(f"opsize: {opsize}")

    for ch in range(int(NUM_CHANNEL)):
        for pch in range(int(NUM_PSEUDOCHANNEL)):
            ndp_inst_list = []
            for bk in range(num_banks):
                iters_this_bk = iterations_per_bk + (1 if bk < (iteration % num_banks) else 0)
                if iters_this_bk == 0: continue
                jump_pc = len(ndp_inst_list)
                for bg in range(num_working_bg):
                    ndp_inst_list.append(inst(ndp_inst_opcode["LOAD"],opsize,0,bg,bk,0,0,0))
                for bg in range(num_working_bg):
                    ndp_inst_list.append(inst(ndp_inst_opcode["MAC"],opsize,1,bg,bk,0,0,0))
                ndp_inst_list.append(inst(ndp_inst_opcode["SELF_EXEC_ON"],0,0,0,0,0,0,0))
                if num_working_bg > 1:
                    ndp_inst_list.append(inst(ndp_inst_opcode["T_V_RED"],(2*num_working_bg),0,0,0,0,0,0))
                    ndp_inst_list.append(inst(ndp_inst_opcode["BARRIER"],0,0,0,0,0,0,0))
                ndp_inst_list.append(inst(ndp_inst_opcode["T_ADD"],0,0,0,0,0,0,0))
                ndp_inst_list.append(inst(ndp_inst_opcode["BARRIER"],0,0,0,0,0,0,0))
                ndp_inst_list.append(inst(ndp_inst_opcode["T_S_RED"],0,0,0,0,0,0,0))
                ndp_inst_list.append(inst(ndp_inst_opcode["BARRIER"],0,0,0,0,0,0,0))
                ndp_inst_list.append(inst(ndp_inst_opcode["SELF_EXEC_OFF"],0,0,0,0,0,0,0))
                ndp_inst_list.append(inst(ndp_inst_opcode["WBD"],0,2,0,bk,0,0,0))
                if iters_this_bk > 1:
                    ndp_inst_list.append(inst(ndp_inst_opcode["LOOP"],0,0,0,0,(iters_this_bk-1),jump_pc,0))
            ndp_inst_list.append(inst(ndp_inst_opcode["EXIT"],0,0,0,0,0,0,0))
            if len(ndp_inst_list) >= 1024:
                print("Error: Over NDP Instruction Memory")
                exit(1)
            dump_ndp_inst(f,ndp_inst_list,ch,pch)

    acc_inst_list = [[]]
    acc_inst_list_num = [[]]
    for ch in range(int(NUM_CHANNEL)):
        for pch in range(int(NUM_PSEUDOCHANNEL)):
            acc_inst_list.append([])
            acc_inst_list_num.append(0)
    for ch in range(int(NUM_CHANNEL)):
        for pch in range(int(NUM_PSEUDOCHANNEL)):
            idx = ch * int(NUM_PSEUDOCHANNEL) + pch
            for bk in range(num_banks):
                iters_this_bk = iterations_per_bk + (1 if bk < (iteration % num_banks) else 0)
                if iters_this_bk == 0: continue
                acc_inst_list[idx].append(acc_inst_set_base(0, 5000))
                acc_inst_list[idx].append(acc_inst_set_base(1, 6000))
                acc_inst_list[idx].append(acc_inst_set_base(2, 7000))
                acc_inst_list[idx].append(acc_inst_set_loop(0, max(0, iters_this_bk - 1)))
                jump_pc = len(acc_inst_list[idx])
                for bg in range(num_working_bg):
                    acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["RD"],opsize,ch,pch,bg,bk,undirect_row(0,0),0,0,0,mode=1))
                acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["BAR"],0,0,0,0,0,0,0,0,0))
                for bg in range(num_working_bg):
                    acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["RD"],opsize,ch,pch,bg,bk,undirect_row(1,0),0,1,0,mode=1))
                acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["BAR"],0,0,0,0,0,0,0,0,0))
                acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["WAIT"],0,0,0,0,0,0,0,0,100))
                acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["WR"],0,ch,pch,0,bk,undirect_row(2,0),0,2,0,mode=1))
                acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["BAR"],0,0,0,0,0,0,0,0,0))
                if iters_this_bk > 1:
                    acc_inst_list[idx].append(acc_inst_inc_base(0, 1))
                    acc_inst_list[idx].append(acc_inst_inc_base(1, 1))
                    acc_inst_list[idx].append(acc_inst_inc_base(2, 1))
                    acc_inst_list[idx].append(acc_inst_loop(0, jump_pc))
            acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["DONE"],0,0,0,0,0,0,0,0,0))
    desc_counts = dump_ndp_acc_inst_per_pch(f, acc_inst_list)
    write_ndp_start(f, desc_counts)
    return

# GEMV       : Z = GEMV

def gemv_pch(f, input_size, scaling):
    '''
        input_size (vector): 8KB(4K), 16KB(8K), 32KB(16K), 64KB(32K), 128KB(64K)
        Matrix Size(elements); 4K x 4K, 8K x 8K, 16K x 16K, ..
        Tile Size = ch x pch x BG?       
        Z = GEMV
        Scaling Factor 1, 2, 4
        row size: 8K or 16K
        Need Tiling 
        data memory: 32KB-> Max 4 Row
        --- Iteration (Big Tile)-------
        LOAD (Partial Vector)-> BARRIER -> 
        MAC (Partial Sum; 2 x num_working_bg) x # of Tiles (Vertical)

        WBD 1 
        ---
        Input: 8K
        BG0/ROW5000, BG0/ROW6000 -> BG0/ROW7000
        COL:0-127 or 0-63
        BG0-3/ROW5000, BG0-3/ROW6000 -> BG0/ROW7000
        COL:0-127 or 0-63                 
    '''
    print(f"Generating Memory Trace of GEMV Operation for NDP Ops")

    ndp_bk_idx = NDP_TARGET_BK

    # Based on the number of elements 
    # Each Element is FP16 (2B)
    if scaling == 4:
        row_size = 8192
        n_bg = 4
    else:
        row_size = 4096
        n_bg = 8

    print(f" - row_size: {row_size}") 
    # Data Memory Size: 32KB, 64KB, 128KB
    # elements per access 
    # x4:32, x8;64, x16; 128    
    if scaling == 1:
        max_p_vec_size  =  8192
        n_per_acc = 32
    elif scaling == 2:
        max_p_vec_size  =  8192 * 2
        n_per_acc = 64 
    else:
        max_p_vec_size  =  8192 * 4
        n_per_acc = 128 

    print(f" - max_p_vec_size: {max_p_vec_size}") 

    vec_size = int(mat_input_size_byte_list[input_size]/2)       
    print(f" - vec_size: {vec_size}") 

    if scaling == 1:
        opsize = 127
        max_opsize = 127
    else:
        opsize = 63
        max_opsize = 63


    # Partial Vector Size
    if max_p_vec_size > vec_size:
        p_vec_size = vec_size
        col_tile = 1
        if row_size > vec_size:
            row_vec_ratio = int(row_size/vec_size)
            opsize = int((opsize + 1)/row_vec_ratio) - 1
            print(f" - row_size {row_size} is larger then vec_size {vec_size}, so reduce opsize {max_opsize} -->  {opsize}")
    else:
        p_vec_size = max_p_vec_size
        col_tile = int(vec_size/p_vec_size)
    
    print(f" - opsize: {opsize}")

    print(f" - p_vec_size: {p_vec_size}")
    print(f" - col_tile: {col_tile}")
    # Require n_row for partial vector
    if row_size > p_vec_size:
            n_row_p_vec = 1
    else:
        n_row_p_vec = int(p_vec_size / row_size)

    print(f" - n_row_p_vec: {n_row_p_vec}")

    # each tile size 
    if scaling == 4:
        # BG x CH x PCH 
        n_tile_row = n_bg * int(NUM_CHANNEL) * int(NUM_PSEUDOCHANNEL)
    else: 
        # BG x CH x PCH
        n_tile_row = n_bg * int(NUM_CHANNEL) * int(NUM_PSEUDOCHANNEL)
    n_tile_col = p_vec_size

    print(f" - n_tile_row: {n_tile_row}")

    # Tile Block Row (nTile)
    tmp0 = n_row_p_vec * n_bg
    if col_tile > 1:
        tmp0 = tmp0 * 2
    tmp1 = int(1024/tmp0)
    if tmp1 >= 256:
        n_tile_block_row = 256
    elif tmp1 >= 128:
        n_tile_block_row = 128
    elif tmp1 >= 64:
        n_tile_block_row = 64
    elif tmp1 >= 32:
        n_tile_block_row = 32
    elif tmp1 >= 16:
        n_tile_block_row = 16
    elif tmp1 >= 8:
        n_tile_block_row = 8        
    elif tmp1 >= 4:
        n_tile_block_row = 4
    elif tmp1 >= 2:
        n_tile_block_row = 2             
    elif tmp1 >= 1:
        n_tile_block_row = 1        
    else:
        print("Error: too small n_tile_block_row")
        exit(1)      


    if col_tile > 1:
        tmp2 = n_tile_block_row * n_row_p_vec * n_bg * 2
    else:
        tmp2 = n_tile_block_row * n_row_p_vec * n_bg
    
    if tmp2 > 1000:
        n_tile_block_row = int(n_tile_block_row / 2)
        
    print(f" - n_tile_block_row: {n_tile_block_row}")
    # n_tile_block_row * n_row_p_vec * n_bg * 1 or 2 < 1024 
    # Iteration Tile Block 
    iteration_tile_block = int(vec_size/(n_tile_block_row*n_tile_row))
    if iteration_tile_block == 0:
        n_tile_block_row =  int(vec_size/n_tile_row)
        print(f" - resize n_tile_block_row to {n_tile_block_row}")    
        iteration_tile_block = int(vec_size/(n_tile_block_row*n_tile_row))
    print(f" - iteration_tile_block: {iteration_tile_block}")
    # results vector colum size (block)
    # n_tile_block_row*n_tile_row/(n_ch * n_pch)
    n_partial_y =  n_tile_block_row*n_bg
    # Vector Reduction --> 2 Partial Sum
    post_n_it = int(int(n_partial_y) / (max_opsize + 1))
    opsize_v_rec = max_opsize
    # if opsize_v_rec > 127:
    #     print(f"Error: {opsize_v_rec} Over Opsize (128)")
    #     exit(1)        
    # Scalare Reduction --> 32 or 64 or 128 --> 1
    opsize_s_rec = max_opsize
    if scaling == 1:
        opsize_follow_s_rec = -1
    else:
        opsize_follow_s_rec = opsize_s_rec
    
    opsize_wbd = int(n_partial_y / n_per_acc)

    # Tile Size 
    # iteration, num_working_bg, opsize = cal_it(input_size, scaling)
    extra_delay = 20
    self_exec_delay = post_n_it * (2 * opsize_v_rec)  + extra_delay + \
                      post_n_it * (2 * opsize_v_rec)  + extra_delay + \
                      post_n_it * (2 * opsize_s_rec) + extra_delay 
    if opsize_follow_s_rec != -1:
        self_exec_delay = post_n_it * (2 * opsize_follow_s_rec)  + extra_delay

    #  NDP clocks run four times slower than regular clocks.
    self_exec_delay = self_exec_delay * 4
    
    # Input_size x 8 pch 
    # print(f"vec_size:                {vec_size}")
    # print(f"max_p_vec_size:          {max_p_vec_size}")
    # print(f"p_vec_size:              {p_vec_size}")
    # print(f"col_tile:                {col_tile}")
    # print(f"n_row_p_vec:             {n_row_p_vec}")
    # print(f"n_tile_row:              {n_tile_row}")
    # print(f"n_tile_col:              {n_tile_col}")
    # print(f"iteration_tile_block:    {iteration_tile_block}")
    print(f"self_exec_delay:         {self_exec_delay}")
    print(f"post_n_it:               {post_n_it}")
    print(f"opsize_v_rec:            {opsize_v_rec}")    
    print(f"opsize_s_rec:            {opsize_s_rec}")    
    print(f"opsize_follow_s_rec:     {opsize_follow_s_rec}")    
    print(f"opsize_wbd:              {opsize_wbd}")
    
    # Write DRAM (Vector Data) — BK=0 (vector uses BK=0 in LOAD)
    n_wr_dram_row = int(vec_size/row_size)
    for ro in range(n_wr_dram_row):
        for co in range(int(NUM_COL)):
            for ch in range(int(NUM_CHANNEL)):
               for pch in range(int(NUM_PSEUDOCHANNEL)):
                   for bg in range(n_row_p_vec):
                       write_normal_trace(f,'ST',encode_address(ch, pch, 0, bg, 0, 1000 + ro, co))
    
    jump_pc0 = 0
    jump_pc1 = 0
    self_before_pc = 0
    self_after_pc = 0
    fianl_pc = 0
    # NDP Instruction: Opcode, Opsize, ID, BG, BK, OP1, OP2, OP3
    # BK interleaving: MAC uses bk = t_tile % NUM_BANK; vector LOAD/WBD use BK=0
    num_banks = int(NUM_BANK)
    for ch in range(int(NUM_CHANNEL)):
        for pch in range(int(NUM_PSEUDOCHANNEL)):
            ndp_inst_list = []
            jump_pc0 = len(ndp_inst_list)
            # Block-level GEMV
            # Load Partial Vector to Data Memory (BK=0, vector data)
            for bg in range(n_row_p_vec):
                ndp_inst_list.append(inst(ndp_inst_opcode["LOAD"],opsize,0,bg,0,0,0,0))
            ndp_inst_list.append(inst(ndp_inst_opcode["BARRIER"],0,0,0,0,0,0,0))
            # MAC Operation — BK rotates per t_tile
            for t_tile in range(n_tile_block_row):
                mac_bk = t_tile % num_banks
                for _ in range(n_row_p_vec):
                    for bg in range(n_bg):
                        ndp_inst_list.append(inst(ndp_inst_opcode["MAC"],opsize,0,bg,mac_bk,0,0,0))
                    ndp_inst_list.append(inst(ndp_inst_opcode["BARRIER"],0,0,0,0,0,0,0))
            # Column-wise gemv ops
            if col_tile > 1:
                jump_pc1 = len(ndp_inst_list)
                for bg in range(n_row_p_vec):
                    ndp_inst_list.append(inst(ndp_inst_opcode["LOAD"],opsize,0,bg,0,0,0,0))
                ndp_inst_list.append(inst(ndp_inst_opcode["BARRIER"],0,0,0,0,0,0,0))
                for t_tile in range(n_tile_block_row):
                    mac_bk = t_tile % num_banks
                    for _ in range(n_row_p_vec):
                        for bg in range(n_bg):
                            ndp_inst_list.append(inst(ndp_inst_opcode["MAC"],opsize,0,bg,mac_bk,0,0,0))
                        ndp_inst_list.append(inst(ndp_inst_opcode["BARRIER"],0,0,0,0,0,0,0))
                # ndp_inst_list.append(inst(ndp_inst_opcode["BARRIER"],0,0,0,0,0,0,0))
                ndp_inst_list.append(inst(ndp_inst_opcode["LOOP"],0,0,0,0,(col_tile - 2),jump_pc1,0))
            ndp_inst_list.append(inst(ndp_inst_opcode["BARRIER"],0,0,0,0,0,0,0))
            # Partial-Sum Vector 
            self_before_pc = len(ndp_inst_list)
            ndp_inst_list.append(inst(ndp_inst_opcode["SELF_EXEC_ON"],0,0,0,0,0,0,0))
            if int(post_n_it/n_bg) > 0:
                for _ in range(int(post_n_it/n_bg)):
                    for bg in range(n_bg):
                        ndp_inst_list.append(inst(ndp_inst_opcode["T_ADD"],opsize_v_rec,0,bg,ndp_bk_idx,0,0,0))
                    ndp_inst_list.append(inst(ndp_inst_opcode["BARRIER"],0,0,0,0,0,0,0))
            if int(post_n_it%n_bg) != 0:                     
                for bg in range(int(post_n_it%n_bg)):
                    ndp_inst_list.append(inst(ndp_inst_opcode["T_ADD"],opsize_v_rec,0,bg,ndp_bk_idx,0,0,0))
                ndp_inst_list.append(inst(ndp_inst_opcode["BARRIER"],0,0,0,0,0,0,0))                
            # Scalar Reduction
            if int(post_n_it/n_bg) > 0:
                for _ in range(int(post_n_it/n_bg)):
                    for bg in range(n_bg):            
                        ndp_inst_list.append(inst(ndp_inst_opcode["T_S_RED"],opsize_s_rec,0,bg,ndp_bk_idx,0,0,0))
                    ndp_inst_list.append(inst(ndp_inst_opcode["BARRIER"],0,0,0,0,0,0,0))
            if int(post_n_it%n_bg) != 0:                     
                for bg in range(int(post_n_it%n_bg)):
                    ndp_inst_list.append(inst(ndp_inst_opcode["T_S_RED"],opsize_s_rec,0,bg,ndp_bk_idx,0,0,0))
                ndp_inst_list.append(inst(ndp_inst_opcode["BARRIER"],0,0,0,0,0,0,0))    
            # Extra Reduction for x8 or x16 NDP Ops
            if opsize_follow_s_rec != -1:
                if int(post_n_it/n_bg) > 0:
                    for _ in range(int(post_n_it/n_bg)):
                        for bg in range(n_bg):            
                            ndp_inst_list.append(inst(ndp_inst_opcode["T_S_RED"],opsize_follow_s_rec,0,bg,ndp_bk_idx,0,0,0))
                        ndp_inst_list.append(inst(ndp_inst_opcode["BARRIER"],0,0,0,0,0,0,0))
                if int(post_n_it%n_bg) != 0:                     
                    for bg in range(int(post_n_it%n_bg)):
                        ndp_inst_list.append(inst(ndp_inst_opcode["T_S_RED"],opsize_follow_s_rec,0,bg,ndp_bk_idx,0,0,0))
                    ndp_inst_list.append(inst(ndp_inst_opcode["BARRIER"],0,0,0,0,0,0,0)) 
            ndp_inst_list.append(inst(ndp_inst_opcode["SELF_EXEC_OFF"],0,0,0,0,0,0,0)) 
            self_after_pc = len(ndp_inst_list)
            # WBD (BK=0, output result)
            ndp_inst_list.append(inst(ndp_inst_opcode["WBD"],opsize_wbd,0,0,0,0,0,0))

            if iteration_tile_block > 1:
                ndp_inst_list.append(inst(ndp_inst_opcode["LOOP"],0,1,0,0,(iteration_tile_block - 1),jump_pc0,0))
            ndp_inst_list.append(inst(ndp_inst_opcode["EXIT"],0,0,0,0,0,0,0))
            fianl_pc = len(ndp_inst_list)

            if(len(ndp_inst_list) >= 1024):
                print(f"Error: Over NDP Instruction Memory {len(ndp_inst_list)}")
                exit(1)
            dump_ndp_inst(f,ndp_inst_list,ch,pch)
    print(f"jump0 PC:              {jump_pc0}")
    print(f"jump1 PC:              {jump_pc1}")
    print(f"self_before_pc PC:     {self_before_pc}")
    print(f"self_after_pc PC:      {self_after_pc}")
    print(f"fianl_pc PC:           {fianl_pc}")



    # Make 2-D NDL-Launch Request Inst
    # BK interleaving: vector RD uses BK=0, matrix RD uses bk=t_tile%num_banks, WR uses BK=0
    acc_inst_list = [[ ]]
    acc_inst_list_num = [[ ]]
    for ch in range(int(NUM_CHANNEL)):
        for pch in range(int(NUM_PSEUDOCHANNEL)):
            acc_inst_list.append([])
            acc_inst_list_num.append(0)
    for ch in range(int(NUM_CHANNEL)):
        for pch in range(int(NUM_PSEUDOCHANNEL)):
            idx = ch * int(NUM_PSEUDOCHANNEL) + pch
            acc_inst_list[idx].append(acc_inst_set_base(0, 1000))   # vector base
            acc_inst_list[idx].append(acc_inst_set_base(1, 3000))   # matrix base
            acc_inst_list[idx].append(acc_inst_set_base(2, 7000))   # output base
            acc_inst_list[idx].append(acc_inst_set_loop(0, max(0, iteration_tile_block - 1)))
            jump_pc = len(acc_inst_list[idx])
            # ① Load Partial Vector (BK=0)
            for bg in range(n_row_p_vec):
                acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["RD"],opsize,ch,pch,bg,0,undirect_row(0,0),0,0,0,mode=1))
            acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["BAR"],0,0,0,0,0,0,0,0,0))
            # ② Matrix MAC — BK rotates per t_tile
            for t_tile in range(n_tile_block_row):
                mac_bk = t_tile % num_banks
                for n_row in range(n_row_p_vec):
                    for bg in range(n_bg):
                        offset = t_tile * n_row_p_vec + n_row
                        acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["RD"],opsize,ch,pch,bg,mac_bk,undirect_row(1,offset),0,0,0,mode=1))
                    acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["BAR"],0,0,0,0,0,0,0,0,0))
            # ③ col_tile > 1
            if col_tile > 1:
                if col_tile > 2:
                    acc_inst_list[idx].append(acc_inst_set_loop(1, col_tile - 2))
                col_jump_pc = len(acc_inst_list[idx])
                for bg in range(n_row_p_vec):
                    acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["RD"],opsize,ch,pch,bg,0,undirect_row(0,0),0,0,0,mode=1))
                acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["BAR"],0,0,0,0,0,0,0,0,0))
                for t_tile in range(n_tile_block_row):
                    mac_bk = t_tile % num_banks
                    for n_row in range(n_row_p_vec):
                        for bg in range(n_bg):
                            offset = t_tile * n_row_p_vec + n_row
                            acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["RD"],opsize,ch,pch,bg,mac_bk,undirect_row(1,offset),0,0,0,mode=1))
                        acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["BAR"],0,0,0,0,0,0,0,0,0))
                if col_tile > 2:
                    acc_inst_list[idx].append(acc_inst_loop(1, col_jump_pc))
            # ④ Wait + WR (BK=0, output)
            acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["WAIT"],0,0,0,0,0,0,0,0,self_exec_delay))
            acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["WR"],opsize_wbd,ch,pch,0,0,undirect_row(2,0),0,0,0,mode=1))
            acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["BAR"],0,0,0,0,0,0,0,0,0))
            # ⑤ INC_BASE + LOOP
            if iteration_tile_block > 1:
                acc_inst_list[idx].append(acc_inst_inc_base(0, col_tile))
                acc_inst_list[idx].append(acc_inst_inc_base(1, n_tile_block_row * n_row_p_vec))
                acc_inst_list[idx].append(acc_inst_inc_base(2, 1))
                acc_inst_list[idx].append(acc_inst_loop(0, jump_pc))
            acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["DONE"],0,0,0,0,0,0,0,0,0))


    # Write AccInst per-PCH sequential streams → HSNC intercepts all writes
    desc_counts = dump_ndp_acc_inst_per_pch(f, acc_inst_list)

    # NDP Start (must be LAST — after all AccInst writes)
    write_ndp_start(f, desc_counts)
       
    return         

def axpby_normal(f, input_size):
    '''
        input_size: (8K, 32K, 64K, 128K, 512K, 8M)*NORMAL_SCALE_FACTOR*NUM_CHANNEL
        Z = aX + bY
        --- Iteration InputSize/8K -------
        Read X
        Read Y

        Vector X: Row 5000
        Vector Y: Row 6000
        Vector Z: Row 7000
        ROW5000        
    '''
    num_rd = int(input_size_byte_list[input_size]/8192) * NORMAL_SCALE_FACTOR * NUM_CHANNEL * NUM_NORMAL_COL
    # src_X_addr = encode_normal_address(0, 0, 0, 0, 5000, 0)
    # src_Y_addr = encode_normal_address(0, 0, 0, 0, 6000, 0)
    # des_Z_addr = encode_normal_address(0, 0, 0, 0, 7000, 0)
    # NUM_CHANNEL
    # NUM_RANK
    # NUM_BANKGROUP
    # NUM_BANK
    # NUM_COL
    # encode_normal_address(channel, rank, bg, bank, row, col)

    if GEN_PCH_NORMAL_MODE:
        gen_normal_req_from_row_pch(f,5000,num_rd,'LD')
        gen_normal_req_from_row_pch(f,6000,num_rd,'LD')
        gen_normal_req_from_row_pch(f,7000,num_rd,'ST')
    else:
        gen_normal_req_from_row(f,5000,num_rd,'LD')
        gen_normal_req_from_row(f,6000,num_rd,'LD')
        gen_normal_req_from_row(f,7000,num_rd,'ST')
 

# AXPBYPCZ  : W = aX + bB + zZ
def axpbypcz_normal(f, input_size):
    '''
        input_size: (8K, 32K, 64K, 128K, 512K, 8M)*NORMAL_SCALE_FACTOR*NUM_CHANNEL
        W = aX + bB + zZ
        --- Iteration InputSize/8K -------
        Read X
        Read Y
        Read Z

        Write W      
    ''' 
    num_rd = int(input_size_byte_list[input_size]/8192) * NORMAL_SCALE_FACTOR * NUM_CHANNEL * NUM_NORMAL_COL
    # src_X_addr = encode_normal_address(0, 0, 0, 0, 5000, 0)
    # src_B_addr = encode_normal_address(0, 0, 0, 0, 6000, 0)
    # src_Z_addr = encode_normal_address(0, 0, 0, 0, 7000, 0)
    # des_W_addr = encode_normal_address(0, 0, 0, 0, 8000, 0)

    if GEN_PCH_NORMAL_MODE:
        gen_normal_req_from_row_pch(f,5000,num_rd,'LD')
        gen_normal_req_from_row_pch(f,6000,num_rd,'LD')
        gen_normal_req_from_row_pch(f,7000,num_rd,'LD')
        gen_normal_req_from_row_pch(f,8000,num_rd,'ST')
    else:
        gen_normal_req_from_row(f,5000,num_rd,'LD')
        gen_normal_req_from_row(f,6000,num_rd,'LD')
        gen_normal_req_from_row(f,7000,num_rd,'LD')
        gen_normal_req_from_row(f,8000,num_rd,'ST')         

def axpy_normal(f, input_size):
    '''
        input_size: (8K, 32K, 64K, 128K, 512K, 8M)*NORMAL_SCALE_FACTOR*NUM_CHANNEL
        Z = aX + Y
        --- Iteration InputSize/8K -------
        Read X
        Read Y

        Vector X: Row 5000
        Vector Y: Row 6000
        Vector Z: Row 7000
        ROW5000        
    '''
    num_rd = int(input_size_byte_list[input_size]/8192) * NORMAL_SCALE_FACTOR * NUM_CHANNEL * NUM_NORMAL_COL

    if GEN_PCH_NORMAL_MODE:
        gen_normal_req_from_row_pch(f,5000,num_rd,'LD')
        gen_normal_req_from_row_pch(f,6000,num_rd,'LD')
        gen_normal_req_from_row_pch(f,7000,num_rd,'ST')        
    else:
        gen_normal_req_from_row(f,5000,num_rd,'LD')
        gen_normal_req_from_row(f,6000,num_rd,'LD')
        gen_normal_req_from_row(f,7000,num_rd,'ST')

def copy_normal(f, input_size):
    '''
        input_size: (8K, 32K, 64K, 128K, 512K, 8M)*NORMAL_SCALE_FACTOR*NUM_CHANNEL
        Y = X
        --- Iteration InputSize/8K -------
        Read X
        Write Y

        Vector X: Row 5000
        Vector Y: Row 6000
        ROW5000        
    '''
    num_rd = int(input_size_byte_list[input_size]/8192) * NORMAL_SCALE_FACTOR * NUM_CHANNEL * NUM_NORMAL_COL

    if GEN_PCH_NORMAL_MODE:
        gen_normal_req_from_row_pch(f,5000,num_rd,'LD')
        gen_normal_req_from_row_pch(f,6000,num_rd,'ST')    
    else:
        gen_normal_req_from_row(f,5000,num_rd,'LD')
        gen_normal_req_from_row(f,6000,num_rd,'ST')    

def xmy_normal(f, input_size):
    '''
        input_size: (8K, 32K, 64K, 128K, 512K, 8M)*NORMAL_SCALE_FACTOR*NUM_CHANNEL
        Z = X ⨀ Y
        --- Iteration InputSize/8K -------
        Read X
        Read Y
        Write Z

        Vector X: Row 5000
        Vector Y: Row 6000
        Vector Z: Row 7000
    '''
    num_rd = int(input_size_byte_list[input_size]/8192) * NORMAL_SCALE_FACTOR * NUM_CHANNEL * NUM_NORMAL_COL

    if GEN_PCH_NORMAL_MODE:
        gen_normal_req_from_row_pch(f,5000,num_rd,'LD')
        gen_normal_req_from_row_pch(f,6000,num_rd,'LD')
        gen_normal_req_from_row_pch(f,7000,num_rd,'ST')
    else:
        gen_normal_req_from_row(f,5000,num_rd,'LD')
        gen_normal_req_from_row(f,6000,num_rd,'LD')
        gen_normal_req_from_row(f,7000,num_rd,'ST')        

def dot_normal(f, input_size):
    '''
        input_size: (8K, 32K, 64K, 128K, 512K, 8M)*NORMAL_SCALE_FACTOR*NUM_CHANNEL
        c = X·Y
        --- Iteration InputSize/8K -------
        Read X
        Read Y
        Write c

        Vector X: Row 5000
        Vector Y: Row 6000
        Scalar C: Row 7000
    '''
    num_rd = int(input_size_byte_list[input_size]/8192) * NORMAL_SCALE_FACTOR * NUM_CHANNEL * NUM_NORMAL_COL

    if GEN_PCH_NORMAL_MODE:        
        gen_normal_req_from_row_pch(f,5000,num_rd,'LD')
        gen_normal_req_from_row_pch(f,6000,num_rd,'LD')
        gen_normal_req_from_row_pch(f,7000,1,'ST')     
    else:
        gen_normal_req_from_row(f,5000,num_rd,'LD')
        gen_normal_req_from_row(f,6000,num_rd,'LD')
        gen_normal_req_from_row(f,7000,1,'ST')             

def gemv_normal(f, input_size):
    '''
        input_size (vector): 8KB(4K), 16KB(8K), 32KB(16K), 64KB(32K), 128KB(64K)
        Matrix Size(elements); 4K x 4K, 8K x 8K, 16K x 16K, ..
        
        Read Vector 
        Read Matrix .. each Row.. 

        vec_size = int(mat_input_size_byte_list[input_size]/2)        

    '''
    # Require RD per Row
    num_rd = int(mat_input_size_byte_list[input_size]/2) / 32
    num_row = int(mat_input_size_byte_list[input_size]/2)
    # elements_per_row = 32 * NUM_COL * NUM_BANK * NUM_BANKGROUP * NUM_RANK * NUM_CHANNEL (-1M)

    if GEN_PCH_NORMAL_MODE:        
        gen_normal_req_from_row_pch(f,1000,num_rd,'LD')
        gen_normal_req_from_row_pch(f,2000,num_rd * num_row,'LD')
        gen_normal_req_from_row_pch(f,1001,num_rd,'ST')         
    else:
        gen_normal_req_from_row(f,1000,num_rd,'LD')
        gen_normal_req_from_row(f,2000,num_rd * num_row,'LD')
        gen_normal_req_from_row(f,1001,num_rd,'ST')         
    print(f"=======================================")
    print(f" GEMV Normal Mode - Size: {mat_input_size_byte_list[input_size]}")    
    print(f"  - RD Vecotr: {num_rd}")    
    print(f"  - RD Matrix: {num_rd * num_row}")    
    print(f"  - WR Vecotr: {num_rd}")    

###############################################################################
# SLS (SparseLengthsSum) Baseline Trace Generation
#
# Random access to T=128 embedding tables with Zipf(alpha=1.05) distribution.
# Each table occupies a dedicated bank (CH->RK->BG->BK placement).
# Access order: for b -> for l -> for t (lookup-interleaved for max BLP).
###############################################################################

def sls_normal(f, sls_config_name):
    """Generate baseline SLS trace for conventional DDR5.

    Read-only: B × L × T vectors fetched from DRAM (each 8 column accesses = 512B).
    CPU performs element-wise sum reduction in cache and consumes the result.
    No DRAM write-back — reduced vectors stay in CPU cache/registers.

    Access order: for b -> for l -> for t (table-interleaved per lookup)
    maximizes bank-level parallelism since consecutive tables map to
    different channels/ranks/bank groups.
    """
    config = sls_config_list[sls_config_name]
    B, L = config["B"], config["L"]
    T = SLS_NUM_TABLES
    N = SLS_ENTRIES_PER_TABLE

    # Pre-generate all Zipf indices: shape (T, B, L)
    indices = sls_generate_zipf_indices(B, L, T, N, SLS_ZIPF_ALPHA, SLS_ZIPF_SEED)

    # --- Read phase: fetch embedding vectors ---
    # Order: for b -> for l -> for t  (BLP maximized)
    # CPU accumulates in cache: Out_t += E_t[idx] for each lookup
    for b in range(B):
        for l in range(L):
            for t in range(T):
                entry_idx = indices[t][b][l]
                row, col_start = sls_entry_to_row_col(entry_idx)
                ch, rk, bg, bk = sls_table_to_bank(t)
                for c in range(SLS_COLS_PER_VECTOR):
                    addr = encode_sls_address(ch, rk, bg, bk, row, col_start + c)
                    write_normal_trace(f, 'LD', addr)

    # No write phase: reduction result consumed by host in cache

    total_rd = B * L * T * SLS_COLS_PER_VECTOR
    print(f"  SLS {sls_config_name}: B={B}, L={L}, T={T}, N={N}")
    print(f"    Reads:  {total_rd:>10,} lines ({total_rd * 64 / (1024*1024):.1f} MB)")
    print(f"    Total:  {total_rd:>10,} lines (read-only, no write-back)")


# =========================================================================
#  SLS DBX-DIMM NDP Trace — Wildcard + Intra-Table Bank Interleaving
# =========================================================================

def sls_table_to_pch_bank(table_id):
    """Map table_id (0~127) -> (ch, pch, bg, bk, local_t) for DBX-DIMM.
    16 tables per PCH. Within PCH: local_t 0~7 → BK=0, 8~15 → BK=1.
    bg = local_t % 8 for each pass."""
    pch_idx = table_id // 16                   # 0~7
    ch  = pch_idx // int(NUM_PSEUDOCHANNEL)
    pch = pch_idx %  int(NUM_PSEUDOCHANNEL)
    local_t = table_id % 16                    # 0~15 within this PCH
    bg = local_t % 8                           # BG for table-per-bank
    bk = local_t // 8                          # 0 or 1 (2 passes)
    return ch, pch, bg, bk, local_t


def sls_entry_to_row_col_interleaved(entry_idx):
    """Convert entry index to (row_in_bg, col_start, bg) with intra-table
    bank interleaving: entry-level round-robin across 8 BGs.

    Entry-level interleaving distributes consecutive entries across BGs,
    so Zipf hot entries (low indices) are spread evenly across all 8 BGs.

    Layout: entry_idx → bg = entry_idx % 8
            Within each BG, entries are packed sequentially:
              logical_pos_in_bg = entry_idx // 8
              row_in_bg = logical_pos_in_bg // vectors_per_row
              col_start = (logical_pos_in_bg % vectors_per_row) * cols_per_vector
    """
    bg = entry_idx % 8
    pos_in_bg = entry_idx // 8                          # entry's position within its BG
    row_in_bg = pos_in_bg // SLS_VECTORS_PER_ROW        # row within this BG
    col_start = (pos_in_bg % SLS_VECTORS_PER_ROW) * SLS_COLS_PER_VECTOR
    return row_in_bg, col_start, bg


def sls_pch_ndp(f, sls_config_name):
    """Generate DBX-DIMM NDP SLS trace with V_RED Partial Vector + Wildcard.

    V_RED partial vector approach:
      - 8 ids = 2 tables × 4 partial vectors (cols)
      - First LOAD initializes DM[id] with lookup 0's partial vector
      - V_RED accumulates remaining (L-1) lookups per partial vector
      - 8 V_REDs with distinct ids coexist → no BARRIER between them
      - DRAM pipeline stays full: (L-1) × 2 tables × 4 cols = reads per pass

    Pass structure: 2 tables per pass, 8 passes for 16 tables/PCH.
    LOOP for batch repetition within a tile.
    Batch tiling when descriptors exceed desc_store limit (1024).
    """
    config = sls_config_list[sls_config_name]
    B, L = config["B"], config["L"]
    T = SLS_NUM_TABLES
    N = SLS_ENTRIES_PER_TABLE
    cols_per_vec = SLS_COLS_PER_VECTOR        # 4

    num_pch = int(NUM_CHANNEL) * int(NUM_PSEUDOCHANNEL)  # 8
    tables_per_pch = T // num_pch                         # 16
    tables_per_pass = 2                                   # 2 tables × 4 cols = 8 ids
    passes = (tables_per_pch + tables_per_pass - 1) // tables_per_pass  # 8

    # Tiling: descriptors per batch per PCH
    # Per pass: 8 LOAD(opsize=0) + BAR + 8*(L-1) V_RED reads(opsize=0) + BAR
    #         = 8 + 1 + 8*(L-1) + 1 = 8*L + 2
    descs_per_pass = 8 * L + 2
    # Max passes that fit in desc_store (1024) per tile, leaving room for DONE(1)
    max_passes_per_tile = (1024 - 1) // descs_per_pass   # -1 for DONE
    if max_passes_per_tile < 1:
        print(f"  ERROR: SLS {sls_config_name} single pass descs={descs_per_pass} > 1023")
        return
    # If all passes fit in one tile per batch, use batch tiling
    # Otherwise, use pass tiling (split passes across tiles within same batch)
    if passes <= max_passes_per_tile:
        # All passes fit → batch tiling
        descs_per_batch = passes * descs_per_pass
        tile_size = (1024 - 1) // descs_per_batch   # batches per tile
        tile_size = max(tile_size, 1)
        tile_size = min(tile_size, B)
        num_tiles = (B + tile_size - 1) // tile_size
        pass_tiles = False  # batch-level tiling
    else:
        # Need pass tiling: split passes within each batch
        tile_size = 1  # 1 batch per tile group
        num_tiles = B
        pass_tiles = True  # pass-level tiling
    passes_per_launch = min(passes, max_passes_per_tile)

    num_pass_groups = (passes + passes_per_launch - 1) // passes_per_launch
    total_launches = num_tiles * num_pass_groups

    print(f"  SLS-DBX {sls_config_name}: B={B}, L={L}, T={T}, tables/PCH={tables_per_pch}")
    print(f"    V_RED partial vector: {tables_per_pass} tables/pass, {passes} passes")
    print(f"    descs/pass={descs_per_pass}, passes/launch={passes_per_launch}, pass_groups={num_pass_groups}")
    print(f"    tile={tile_size} batches, num_tiles={num_tiles}, total_launches={total_launches}")

    # Pre-generate Zipf indices: [T][B][L]
    indices = sls_generate_zipf_indices(B, L, T, N, SLS_ZIPF_ALPHA, SLS_ZIPF_SEED)

    # Build per-PCH table list
    pch_tables = [[] for _ in range(num_pch)]
    for t in range(T):
        pch_idx = t // tables_per_pch
        pch_tables[pch_idx].append(t)

    # === Helper: build Inst_Slot for given pass range and loop_count ===
    def build_inst_slot(pass_start, pass_end, loop_count):
        ndp_inst_list = []
        jump_pc = len(ndp_inst_list)
        for p in range(pass_start, pass_end):
            t_count = min(tables_per_pass, tables_per_pch - p * tables_per_pass)
            # LOAD: first lookup, opsize=0 (1 col each)
            for t in range(t_count):
                for c in range(cols_per_vec):
                    id_val = t * cols_per_vec + c
                    ndp_inst_list.append(inst(ndp_inst_opcode["LOAD"], 0, id_val, 0, 0, 0, 0, 0, wildcard=1))
            ndp_inst_list.append(inst(ndp_inst_opcode["BARRIER"], 0, 0, 0, 0, 0, 0, 0))
            # V_RED: remaining (L-1) lookups, opsize=(L-2) means (L-1) reads
            for t in range(t_count):
                for c in range(cols_per_vec):
                    id_val = t * cols_per_vec + c
                    ndp_inst_list.append(inst(ndp_inst_opcode["V_RED"], L - 2, id_val, 0, 0, 0, 0, 0, wildcard=1))
            ndp_inst_list.append(inst(ndp_inst_opcode["BARRIER"], 0, 0, 0, 0, 0, 0, 0))
        if loop_count > 0:
            ndp_inst_list.append(inst(ndp_inst_opcode["LOOP"], 0, 0, 0, 0, loop_count, jump_pc, 0))
        ndp_inst_list.append(inst(ndp_inst_opcode["EXIT"], 0, 0, 0, 0, 0, 0, 0))
        return ndp_inst_list

    # === Helper: build AccInst for given batch range and pass range ===
    def build_acc_inst(b_start, b_end, pass_start, pass_end):
        acc = [[] for _ in range(num_pch)]
        for b in range(b_start, b_end):
            for p in range(pass_start, pass_end):
                t_start_local = p * tables_per_pass
                t_count = min(tables_per_pass, tables_per_pch - t_start_local)

                # LOAD phase: first lookup (l=0), opsize=0 per column
                for t in range(t_count):
                    for c in range(cols_per_vec):
                        id_val = t * cols_per_vec + c
                        for pch_idx in range(num_pch):
                            global_t = pch_tables[pch_idx][t_start_local + t]
                            entry_idx = indices[global_t][b][0]
                            row_in_bg, col_start, bg = sls_entry_to_row_col_interleaved(entry_idx)
                            ch_t, pch_t, _, _, _ = sls_table_to_pch_bank(global_t)
                            bk = (t_start_local + t) // 8
                            acc[pch_idx].append(
                                acc_inst(ndp_acc_inst_opcode["RD"], 0,
                                         ch_t, pch_t, bg, bk, row_in_bg, col_start + c,
                                         id_val, 0, mode=0))
                for pch_idx in range(num_pch):
                    acc[pch_idx].append(
                        acc_inst(ndp_acc_inst_opcode["BAR"], 0, 0, 0, 0, 0, 0, 0, 0, 0))

                # V_RED phase: lookups 1~(L-1), opsize=0 per column
                for t in range(t_count):
                    for c in range(cols_per_vec):
                        id_val = t * cols_per_vec + c
                        for l in range(1, L):
                            for pch_idx in range(num_pch):
                                global_t = pch_tables[pch_idx][t_start_local + t]
                                entry_idx = indices[global_t][b][l]
                                row_in_bg, col_start, bg = sls_entry_to_row_col_interleaved(entry_idx)
                                ch_t, pch_t, _, _, _ = sls_table_to_pch_bank(global_t)
                                bk = (t_start_local + t) // 8
                                acc[pch_idx].append(
                                    acc_inst(ndp_acc_inst_opcode["RD"], 0,
                                             ch_t, pch_t, bg, bk, row_in_bg, col_start + c,
                                             id_val, 0, mode=0))
                for pch_idx in range(num_pch):
                    acc[pch_idx].append(
                        acc_inst(ndp_acc_inst_opcode["BAR"], 0, 0, 0, 0, 0, 0, 0, 0, 0))

        for pch_idx in range(num_pch):
            acc[pch_idx].append(
                acc_inst(ndp_acc_inst_opcode["DONE"], 0, 0, 0, 0, 0, 0, 0, 0, 0))
        return acc

    # === Emit: iterate over batch tiles × pass groups ===
    total_acc = 0
    first_launch = True
    for tile_idx in range(num_tiles):
        b_start = tile_idx * tile_size
        b_end = min(b_start + tile_size, B)
        actual_tile = b_end - b_start

        for pg in range(num_pass_groups):
            p_start = pg * passes_per_launch
            p_end = min(p_start + passes_per_launch, passes)
            actual_passes = p_end - p_start

            loop_count = actual_tile - 1

            # Inst_Slot: emit for first launch, or when loop_count/pass_range differs
            if first_launch or (pg == 0 and actual_tile != tile_size):
                inst_list = build_inst_slot(p_start, p_end, loop_count)
                if len(inst_list) > 1024:
                    print(f"  ERROR: Inst_Slot overflow: {len(inst_list)} > 1024")
                    return
                if first_launch:
                    print(f"    Inst_Slot count: {len(inst_list)}")
                for ch in range(int(NUM_CHANNEL)):
                    for pch in range(int(NUM_PSEUDOCHANNEL)):
                        dump_ndp_inst(f, inst_list, ch, pch)
                first_launch = False
            elif pg > 0:
                # Different pass range → re-emit Inst_Slot
                inst_list = build_inst_slot(p_start, p_end, loop_count)
                for ch in range(int(NUM_CHANNEL)):
                    for pch in range(int(NUM_PSEUDOCHANNEL)):
                        dump_ndp_inst(f, inst_list, ch, pch)

            # AccInst
            acc_inst_list = build_acc_inst(b_start, b_end, p_start, p_end)
            for pch_idx in range(num_pch):
                total_acc += len(acc_inst_list[pch_idx])

            desc_counts = dump_ndp_acc_inst_per_pch(f, acc_inst_list)
            write_ndp_start(f, desc_counts)

            # Phase sync: wait for NDP to complete before next tile/pass
            write_wait_ndp(f)

    print(f"    Total AccInst (all PCH, all launches): {total_acc:,}")
    print(f"    BARRIER per batch: {passes * 2} (LOAD + V_RED per pass)")


def generate_sls_pch_trace(sls_config_name, output_path=''):
    """Generate DBX-DIMM NDP SLS trace."""
    if sls_config_name not in sls_config_list:
        print(f"Error: Invalid SLS config '{sls_config_name}'. Available: {list(sls_config_list.keys())}")
        return

    config_scale_factor(1)  # x4

    file_name = f"pch_ndp_x4_{sls_config_name}.txt"
    file_name = output_path + "/" + file_name

    with open(file_name, 'w') as f:
        sls_pch_ndp(f, sls_config_name)

    print(f"-> Generated SLS DBX-DIMM trace in '{file_name}'.")


###############################################################################
# AsyncDIMM Trace Generation
#
# AsyncDIMM uses base DDR5 (no pseudo-channel), rank-level NMA MC.
# Address mapping: RoBaCoRaCh (same as NORMAL_ADDRESS_SCHEME = "RoBaCoRaCh")
# NMAInst: unified 64-bit encoding with embedded row/col (no separate AccInst)
#
# Magic-path addresses (per system init):
#   inst-buffer: row=MAX, bg=MAX-1, bk=MAX
#   ctrl-register: row=MAX, bg=MAX, bk=MAX
###############################################################################

# AsyncDIMM DRAM parameters
ASYNCDIMM_NUM_CHANNEL    = 2
ASYNCDIMM_NUM_RANK       = 4
ASYNCDIMM_NUM_BANKGROUP  = 8
ASYNCDIMM_NUM_BANK       = 4
ASYNCDIMM_NUM_ROW        = 65536
ASYNCDIMM_NUM_COL        = 128   # 1KB row / 8B
ASYNCDIMM_GRANULARITY    = 6     # 64B

# AsyncDIMM NMA control region
ASYNCDIMM_NMA_ROW     = ASYNCDIMM_NUM_ROW - 1      # 65535
ASYNCDIMM_NMA_CTRL_BG = ASYNCDIMM_NUM_BANKGROUP - 1  # 7
ASYNCDIMM_NMA_CTRL_BK = ASYNCDIMM_NUM_BANK - 1       # 3
ASYNCDIMM_NMA_BUF_BG  = ASYNCDIMM_NUM_BANKGROUP - 2  # 6
ASYNCDIMM_NMA_BUF_BK  = ASYNCDIMM_NUM_BANK - 1       # 3
ASYNCDIMM_NDP_TARGET_BK = 3

# NMAInst_Slot 64-bit encoding (matches request.h NMAInst_Slot)
# [63:58] comp_opcode(6b) [57:51] opsize(7b) [50:48] bg(3b) [47:46] bk(2b)
# [45:28] row(18b) [27:21] col(7b) [20:18] id(3b) [17:12] reserved(6b) [11:0] etc(12b)
def nma_inst(comp_opcode, opsize, bg, bk, row, col, id, etc=0):
    inst_64bit = 0
    inst_64bit |= (comp_opcode & 0x3F)  << 58
    inst_64bit |= (opsize      & 0x7F)  << 51
    inst_64bit |= (bg          & 0x7)   << 48
    inst_64bit |= (bk          & 0x3)   << 46
    inst_64bit |= (row         & 0x3FFFF) << 28
    inst_64bit |= (col         & 0x7F)  << 21
    inst_64bit |= (id          & 0x7)   << 18
    inst_64bit |= (etc         & 0xFFF)
    return inst_64bit

def nma_inst_loop(loop_cnt, jump_pc):
    """Encode LOOP NMAInst: row=loop_cnt(18b), etc=jump_pc(12b)"""
    return nma_inst(ndp_inst_opcode["LOOP"], 0, 0, 0, loop_cnt, 0, 0, jump_pc)

def nma_inst_set_base(reg_id, row_value):
    """SET_BASE: base_reg[reg_id] = row_value"""
    return nma_inst(ndp_inst_opcode["SET_BASE"], 0, 0, 0, row_value, 0, reg_id)

def nma_inst_inc_base(reg_id, stride):
    """INC_BASE: base_reg[reg_id] += stride (row field, 18-bit, max 262143)"""
    return nma_inst(ndp_inst_opcode["INC_BASE"], 0, 0, 0, stride, 0, reg_id)

def encode_asyncdimm_address(channel, rank, bg, bank, row, col):
    """Encode address for AsyncDIMM RoBaCoRaCh mapping (no pseudo-channel)."""
    ch_bits  = int(math.log2(ASYNCDIMM_NUM_CHANNEL))    # 1
    ra_bits  = int(math.log2(ASYNCDIMM_NUM_RANK))       # 2
    co_bits  = int(math.log2(ASYNCDIMM_NUM_COL))        # 7
    bg_bits  = int(math.log2(ASYNCDIMM_NUM_BANKGROUP))  # 3
    ba_bits  = int(math.log2(ASYNCDIMM_NUM_BANK))       # 2
    ro_bits  = int(math.log2(ASYNCDIMM_NUM_ROW))        # 16

    address = 0
    address |= (channel & ((1 << ch_bits) - 1))
    address |= (rank    & ((1 << ra_bits) - 1))  << (ch_bits)
    address |= (col     & ((1 << co_bits) - 1))  << (ch_bits + ra_bits)
    address |= (bg      & ((1 << bg_bits) - 1))  << (ch_bits + ra_bits + co_bits)
    address |= (bank    & ((1 << ba_bits) - 1))  << (ch_bits + ra_bits + co_bits + bg_bits)
    address |= (row     & ((1 << ro_bits) - 1))  << (ch_bits + ra_bits + co_bits + bg_bits + ba_bits)
    address = address << ASYNCDIMM_GRANULARITY
    return address

def dump_asyncdimm_nma_inst(f, inst_list, ch, rank):
    """Write NMAInst list to inst-buffer address for a specific channel/rank.
    Each WR carries 8 x 64-bit NMAInst as payload."""
    num_inst = len(inst_list)
    it = num_inst // 8
    remain = num_inst % 8
    data_array = [0] * 8
    for i in range(it):
        for j in range(8):
            data_array[j] = inst_list[i * 8 + j]
        addr = encode_asyncdimm_address(ch, rank, ASYNCDIMM_NMA_BUF_BG,
                                        ASYNCDIMM_NMA_BUF_BK, ASYNCDIMM_NMA_ROW, i)
        write_trace(f, 'ST', addr, data_array)
    if remain != 0:
        data_array = [0] * 8
        for j in range(remain):
            data_array[j] = inst_list[it * 8 + j]
        addr = encode_asyncdimm_address(ch, rank, ASYNCDIMM_NMA_BUF_BG,
                                        ASYNCDIMM_NMA_BUF_BK, ASYNCDIMM_NMA_ROW, it)
        write_trace(f, 'ST', addr, data_array)

def asyncdimm_start_nma(f, ch, rank):
    """Write ctrl-reg to trigger NMA start for a specific rank.
    payload[rank] = 1 signals start for that rank."""
    data_array = [0] * 8
    data_array[rank] = 1
    addr = encode_asyncdimm_address(ch, rank, ASYNCDIMM_NMA_CTRL_BG,
                                    ASYNCDIMM_NMA_CTRL_BK, ASYNCDIMM_NMA_ROW, 0)
    write_trace(f, 'ST', addr, data_array)

def asyncdimm_cal_it(input_size):
    """Calculate iteration count and working parameters for AsyncDIMM.
    AsyncDIMM: base DDR5, 8 BG, 4 BK, 128 col/row, no pCH.
    Same data memory constraint as DBX-DIMM cal_it_v2 (scaling=1, x4):
      Data memory = 32KB → max num_working_bg * row_size / 2 per iteration.
    opsize is 0-indexed (opsize=N means N+1 column accesses)."""
    row_size = 8192   # 128 cols × 64B = 8KB per row
    accesses = 128    # Initial column accesses per instruction (= cal_it_v2 opsize=128)
    num_working_bg = 8

    vec_size = input_size_byte_list[input_size]
    # Data memory: 32KB (same as DBX-DIMM x4: num_working_bg * row_size / 2)
    max_size = num_working_bg * row_size // 2

    if vec_size > max_size:
        ratio     = vec_size // max_size
        iteration = ratio
        accesses  = accesses // 2
    else:
        ratio     = max_size // vec_size
        iteration = 1
        accesses  = accesses // 2 // ratio

    opsize = accesses - 1  # Convert to 0-indexed
    return iteration, num_working_bg, opsize

def copy_asyncdimm(f, input_size):
    '''
    AsyncDIMM COPY: Y = X (BK interleaving)
    Distributes iterations across all banks (BK0~3) for reduced bank contention.
    Each BK phase: SET_BASE → LOAD×BG → BARRIER → WBD×BG → INC_BASE → LOOP
    '''
    iteration, num_working_bg, opsize = asyncdimm_cal_it(input_size)
    num_banks = ASYNCDIMM_NUM_BANK
    if iteration >= num_banks:
        iterations_per_bk = iteration // num_banks
    else:
        iterations_per_bk = 1
        num_banks = iteration  # Use fewer banks when iteration < num_banks

    print(f"  AsyncDIMM COPY: size={input_size}, iteration={iteration}, "
          f"working_bg={num_working_bg}, opsize={opsize}, iter_per_bk={iterations_per_bk}")

    for ch in range(ASYNCDIMM_NUM_CHANNEL):
        for rk in range(ASYNCDIMM_NUM_RANK):
            nma_inst_list = []

            for bk in range(num_banks):
                iters_this_bk = iterations_per_bk + (1 if bk < (iteration % num_banks) else 0)
                if iters_this_bk == 0:
                    continue

                # Set base address registers (same base row for all BKs — different physical banks)
                nma_inst_list.append(nma_inst_set_base(0, 5000))  # src base
                nma_inst_list.append(nma_inst_set_base(1, 7000))  # dst base

                jump_pc = len(nma_inst_list)

                for bg in range(num_working_bg):
                    nma_inst_list.append(
                        nma_inst(ndp_inst_opcode["LOAD"], opsize, bg, bk, 0, 0, 0))

                nma_inst_list.append(nma_inst(ndp_inst_opcode["BARRIER"], 0, 0, 0, 0, 0, 0))

                for bg in range(num_working_bg):
                    nma_inst_list.append(
                        nma_inst(ndp_inst_opcode["WBD"], opsize, bg, bk, 0, 0, 1))

                if iters_this_bk > 1:
                    nma_inst_list.append(nma_inst_inc_base(0, 1))
                    nma_inst_list.append(nma_inst_inc_base(1, 1))
                    nma_inst_list.append(nma_inst_loop(iters_this_bk - 1, jump_pc))

            nma_inst_list.append(
                nma_inst(ndp_inst_opcode["EXIT"], 0, 0, 0, 0, 0, 0))

            if len(nma_inst_list) >= MAX_INST:
                print(f"Error: NMAInst count ({len(nma_inst_list)}) exceeds MAX_INST ({MAX_INST})")
                exit(1)

            dump_asyncdimm_nma_inst(f, nma_inst_list, ch, rk)
            asyncdimm_start_nma(f, ch, rk)


def axpy_asyncdimm(f, input_size):
    '''AsyncDIMM AXPY: Z = aX + Y (BK interleaving)'''
    iteration, num_working_bg, opsize = asyncdimm_cal_it(input_size)
    num_banks = ASYNCDIMM_NUM_BANK
    if iteration >= num_banks:
        iterations_per_bk = iteration // num_banks
    else:
        iterations_per_bk = 1
        num_banks = iteration  # Use fewer banks when iteration < num_banks
    print(f"  AsyncDIMM AXPY: size={input_size}, iteration={iteration}, "
          f"working_bg={num_working_bg}, opsize={opsize}, iter_per_bk={iterations_per_bk}")

    for ch in range(ASYNCDIMM_NUM_CHANNEL):
        for rk in range(ASYNCDIMM_NUM_RANK):
            nma_inst_list = []
            for bk in range(num_banks):
                iters_this_bk = iterations_per_bk + (1 if bk < (iteration % num_banks) else 0)
                if iters_this_bk == 0: continue
                nma_inst_list.append(nma_inst_set_base(0, 5000))
                nma_inst_list.append(nma_inst_set_base(1, 6000))
                nma_inst_list.append(nma_inst_set_base(2, 7000))
                jump_pc = len(nma_inst_list)
                for bg in range(num_working_bg):
                    nma_inst_list.append(nma_inst(ndp_inst_opcode["LOAD_MUL"], opsize, bg, bk, 0, 0, 0))
                nma_inst_list.append(nma_inst(ndp_inst_opcode["BARRIER"], 0, 0, 0, 0, 0, 0))
                for bg in range(num_working_bg):
                    nma_inst_list.append(nma_inst(ndp_inst_opcode["ADD"], opsize, bg, bk, 0, 0, 1))
                nma_inst_list.append(nma_inst(ndp_inst_opcode["BARRIER"], 0, 0, 0, 0, 0, 0))
                for bg in range(num_working_bg):
                    nma_inst_list.append(nma_inst(ndp_inst_opcode["WBD"], opsize, bg, bk, 0, 0, 2))
                if iters_this_bk > 1:
                    nma_inst_list.append(nma_inst_inc_base(0, 1))
                    nma_inst_list.append(nma_inst_inc_base(1, 1))
                    nma_inst_list.append(nma_inst_inc_base(2, 1))
                    nma_inst_list.append(nma_inst_loop(iters_this_bk - 1, jump_pc))
            nma_inst_list.append(nma_inst(ndp_inst_opcode["EXIT"], 0, 0, 0, 0, 0, 0))
            if len(nma_inst_list) >= MAX_INST:
                print(f"Error: NMAInst count ({len(nma_inst_list)}) exceeds MAX_INST ({MAX_INST})")
                exit(1)
            dump_asyncdimm_nma_inst(f, nma_inst_list, ch, rk)
            asyncdimm_start_nma(f, ch, rk)


def axpby_asyncdimm(f, input_size):
    '''AsyncDIMM AXPBY: Z = aX + bY (BK interleaving)'''
    iteration, num_working_bg, opsize = asyncdimm_cal_it(input_size)
    num_banks = ASYNCDIMM_NUM_BANK
    if iteration >= num_banks:
        iterations_per_bk = iteration // num_banks
    else:
        iterations_per_bk = 1
        num_banks = iteration  # Use fewer banks when iteration < num_banks
    print(f"  AsyncDIMM AXPBY: size={input_size}, iteration={iteration}, "
          f"working_bg={num_working_bg}, opsize={opsize}, iter_per_bk={iterations_per_bk}")

    for ch in range(ASYNCDIMM_NUM_CHANNEL):
        for rk in range(ASYNCDIMM_NUM_RANK):
            nma_inst_list = []
            for bk in range(num_banks):
                iters_this_bk = iterations_per_bk + (1 if bk < (iteration % num_banks) else 0)
                if iters_this_bk == 0: continue
                nma_inst_list.append(nma_inst_set_base(0, 5000))
                nma_inst_list.append(nma_inst_set_base(1, 6000))
                nma_inst_list.append(nma_inst_set_base(2, 7000))
                jump_pc = len(nma_inst_list)
                for bg in range(num_working_bg):
                    nma_inst_list.append(nma_inst(ndp_inst_opcode["LOAD_MUL"], opsize, bg, bk, 0, 0, 0))
                nma_inst_list.append(nma_inst(ndp_inst_opcode["BARRIER"], 0, 0, 0, 0, 0, 0))
                for bg in range(num_working_bg):
                    nma_inst_list.append(nma_inst(ndp_inst_opcode["SCALE_ADD"], opsize, bg, bk, 0, 0, 1))
                nma_inst_list.append(nma_inst(ndp_inst_opcode["BARRIER"], 0, 0, 0, 0, 0, 0))
                for bg in range(num_working_bg):
                    nma_inst_list.append(nma_inst(ndp_inst_opcode["WBD"], opsize, bg, bk, 0, 0, 2))
                if iters_this_bk > 1:
                    nma_inst_list.append(nma_inst_inc_base(0, 1))
                    nma_inst_list.append(nma_inst_inc_base(1, 1))
                    nma_inst_list.append(nma_inst_inc_base(2, 1))
                    nma_inst_list.append(nma_inst_loop(iters_this_bk - 1, jump_pc))
            nma_inst_list.append(nma_inst(ndp_inst_opcode["EXIT"], 0, 0, 0, 0, 0, 0))
            if len(nma_inst_list) >= MAX_INST:
                print(f"Error: NMAInst count ({len(nma_inst_list)}) exceeds MAX_INST ({MAX_INST})")
                exit(1)
            dump_asyncdimm_nma_inst(f, nma_inst_list, ch, rk)
            asyncdimm_start_nma(f, ch, rk)


def axpbypcz_asyncdimm(f, input_size):
    '''AsyncDIMM AXPBYPCZ: W = aX + bB + cZ (BK interleaving)'''
    iteration, num_working_bg, opsize = asyncdimm_cal_it(input_size)
    num_banks = ASYNCDIMM_NUM_BANK
    if iteration >= num_banks:
        iterations_per_bk = iteration // num_banks
    else:
        iterations_per_bk = 1
        num_banks = iteration  # Use fewer banks when iteration < num_banks
    print(f"  AsyncDIMM AXPBYPCZ: size={input_size}, iteration={iteration}, "
          f"working_bg={num_working_bg}, opsize={opsize}, iter_per_bk={iterations_per_bk}")

    for ch in range(ASYNCDIMM_NUM_CHANNEL):
        for rk in range(ASYNCDIMM_NUM_RANK):
            nma_inst_list = []
            for bk in range(num_banks):
                iters_this_bk = iterations_per_bk + (1 if bk < (iteration % num_banks) else 0)
                if iters_this_bk == 0: continue
                nma_inst_list.append(nma_inst_set_base(0, 5000))
                nma_inst_list.append(nma_inst_set_base(1, 6000))
                nma_inst_list.append(nma_inst_set_base(2, 7000))
                nma_inst_list.append(nma_inst_set_base(3, 8000))
                jump_pc = len(nma_inst_list)
                for bg in range(num_working_bg):
                    nma_inst_list.append(nma_inst(ndp_inst_opcode["LOAD_MUL"], opsize, bg, bk, 0, 0, 0))
                nma_inst_list.append(nma_inst(ndp_inst_opcode["BARRIER"], 0, 0, 0, 0, 0, 0))
                for bg in range(num_working_bg):
                    nma_inst_list.append(nma_inst(ndp_inst_opcode["SCALE_ADD"], opsize, bg, bk, 0, 0, 1))
                nma_inst_list.append(nma_inst(ndp_inst_opcode["BARRIER"], 0, 0, 0, 0, 0, 0))
                for bg in range(num_working_bg):
                    nma_inst_list.append(nma_inst(ndp_inst_opcode["SCALE_ADD"], opsize, bg, bk, 0, 0, 2))
                nma_inst_list.append(nma_inst(ndp_inst_opcode["BARRIER"], 0, 0, 0, 0, 0, 0))
                for bg in range(num_working_bg):
                    nma_inst_list.append(nma_inst(ndp_inst_opcode["WBD"], opsize, bg, bk, 0, 0, 3))
                if iters_this_bk > 1:
                    nma_inst_list.append(nma_inst_inc_base(0, 1))
                    nma_inst_list.append(nma_inst_inc_base(1, 1))
                    nma_inst_list.append(nma_inst_inc_base(2, 1))
                    nma_inst_list.append(nma_inst_inc_base(3, 1))
                    nma_inst_list.append(nma_inst_loop(iters_this_bk - 1, jump_pc))
            nma_inst_list.append(nma_inst(ndp_inst_opcode["EXIT"], 0, 0, 0, 0, 0, 0))
            if len(nma_inst_list) >= MAX_INST:
                print(f"Error: NMAInst count ({len(nma_inst_list)}) exceeds MAX_INST ({MAX_INST})")
                exit(1)
            dump_asyncdimm_nma_inst(f, nma_inst_list, ch, rk)
            asyncdimm_start_nma(f, ch, rk)


def xmy_asyncdimm(f, input_size):
    '''AsyncDIMM XMY: Z = X ⊙ Y (BK interleaving)'''
    iteration, num_working_bg, opsize = asyncdimm_cal_it(input_size)
    num_banks = ASYNCDIMM_NUM_BANK
    if iteration >= num_banks:
        iterations_per_bk = iteration // num_banks
    else:
        iterations_per_bk = 1
        num_banks = iteration  # Use fewer banks when iteration < num_banks
    print(f"  AsyncDIMM XMY: size={input_size}, iteration={iteration}, "
          f"working_bg={num_working_bg}, opsize={opsize}, iter_per_bk={iterations_per_bk}")

    for ch in range(ASYNCDIMM_NUM_CHANNEL):
        for rk in range(ASYNCDIMM_NUM_RANK):
            nma_inst_list = []
            for bk in range(num_banks):
                iters_this_bk = iterations_per_bk + (1 if bk < (iteration % num_banks) else 0)
                if iters_this_bk == 0: continue
                nma_inst_list.append(nma_inst_set_base(0, 5000))
                nma_inst_list.append(nma_inst_set_base(1, 6000))
                nma_inst_list.append(nma_inst_set_base(2, 7000))
                jump_pc = len(nma_inst_list)
                for bg in range(num_working_bg):
                    nma_inst_list.append(nma_inst(ndp_inst_opcode["LOAD"], opsize, bg, bk, 0, 0, 0))
                nma_inst_list.append(nma_inst(ndp_inst_opcode["BARRIER"], 0, 0, 0, 0, 0, 0))
                for bg in range(num_working_bg):
                    nma_inst_list.append(nma_inst(ndp_inst_opcode["MUL"], opsize, bg, bk, 0, 0, 1))
                nma_inst_list.append(nma_inst(ndp_inst_opcode["BARRIER"], 0, 0, 0, 0, 0, 0))
                for bg in range(num_working_bg):
                    nma_inst_list.append(nma_inst(ndp_inst_opcode["WBD"], opsize, bg, bk, 0, 0, 2))
                if iters_this_bk > 1:
                    nma_inst_list.append(nma_inst_inc_base(0, 1))
                    nma_inst_list.append(nma_inst_inc_base(1, 1))
                    nma_inst_list.append(nma_inst_inc_base(2, 1))
                    nma_inst_list.append(nma_inst_loop(iters_this_bk - 1, jump_pc))
            nma_inst_list.append(nma_inst(ndp_inst_opcode["EXIT"], 0, 0, 0, 0, 0, 0))
            if len(nma_inst_list) >= MAX_INST:
                print(f"Error: NMAInst count ({len(nma_inst_list)}) exceeds MAX_INST ({MAX_INST})")
                exit(1)
            dump_asyncdimm_nma_inst(f, nma_inst_list, ch, rk)
            asyncdimm_start_nma(f, ch, rk)


def dot_asyncdimm(f, input_size):
    '''AsyncDIMM DOT: c = X · Y (BK interleaving)'''
    iteration, num_working_bg, opsize = asyncdimm_cal_it(input_size)
    num_banks = ASYNCDIMM_NUM_BANK
    if iteration >= num_banks:
        iterations_per_bk = iteration // num_banks
    else:
        iterations_per_bk = 1
        num_banks = iteration  # Use fewer banks when iteration < num_banks
    print(f"  AsyncDIMM DOT: size={input_size}, iteration={iteration}, "
          f"working_bg={num_working_bg}, opsize={opsize}, iter_per_bk={iterations_per_bk}")

    for ch in range(ASYNCDIMM_NUM_CHANNEL):
        for rk in range(ASYNCDIMM_NUM_RANK):
            nma_inst_list = []
            for bk in range(num_banks):
                iters_this_bk = iterations_per_bk + (1 if bk < (iteration % num_banks) else 0)
                if iters_this_bk == 0: continue
                nma_inst_list.append(nma_inst_set_base(0, 5000))
                nma_inst_list.append(nma_inst_set_base(1, 6000))
                nma_inst_list.append(nma_inst_set_base(2, 7000))
                jump_pc = len(nma_inst_list)
                for bg in range(num_working_bg):
                    nma_inst_list.append(nma_inst(ndp_inst_opcode["LOAD"], opsize, bg, bk, 0, 0, 0))
                nma_inst_list.append(nma_inst(ndp_inst_opcode["BARRIER"], 0, 0, 0, 0, 0, 0))
                for bg in range(num_working_bg):
                    nma_inst_list.append(nma_inst(ndp_inst_opcode["MAC"], opsize, bg, bk, 0, 0, 1))
                nma_inst_list.append(nma_inst(ndp_inst_opcode["BARRIER"], 0, 0, 0, 0, 0, 0))
                if num_working_bg > 1:
                    nma_inst_list.append(
                        nma_inst(ndp_inst_opcode["T_V_RED"], (2 * num_working_bg) - 1, 0, 0, 0, 0, 0))
                nma_inst_list.append(nma_inst(ndp_inst_opcode["T_ADD"], 0, 0, 0, 0, 0, 0))
                nma_inst_list.append(nma_inst(ndp_inst_opcode["T_S_RED"], 0, 0, 0, 0, 0, 0))
                nma_inst_list.append(nma_inst(ndp_inst_opcode["WBD"], 0, 0, bk, 0, 0, 2))
                if iters_this_bk > 1:
                    nma_inst_list.append(nma_inst_inc_base(0, 1))
                    nma_inst_list.append(nma_inst_inc_base(1, 1))
                    nma_inst_list.append(nma_inst_loop(iters_this_bk - 1, jump_pc))
            nma_inst_list.append(nma_inst(ndp_inst_opcode["EXIT"], 0, 0, 0, 0, 0, 0))
            if len(nma_inst_list) >= MAX_INST:
                print(f"Error: NMAInst count ({len(nma_inst_list)}) exceeds MAX_INST ({MAX_INST})")
                exit(1)
            dump_asyncdimm_nma_inst(f, nma_inst_list, ch, rk)
            asyncdimm_start_nma(f, ch, rk)


def gemv_asyncdimm(f, input_size):
    '''
    AsyncDIMM GEMV: y = Ax (matrix-vector multiply)
    Adapted from gemv_pch (scaling=1, x4).

    Tiling algorithm:
      - Partial vector loaded to VMA via LOAD
      - MAC across bank groups for each matrix tile row
      - T_ADD + T_S_RED reduction for partial sums
      - WBD writes result

    Base registers:
      id=0: vector base (row 1000)
      id=1: matrix base (row 3000)
      id=2: result base (row 7000)
    '''
    row_size = 4096       # elements per row (FP16, 8KB physical row = 128 cols × 64B)
    n_bg = 8              # bank groups per rank
    max_p_vec_size = 8192 # max partial vector size (elements)
    n_per_acc = 32        # elements per column access (x4)
    max_opsize = 127      # 128 column accesses (0-indexed)
    opsize = max_opsize
    ndp_bk = ASYNCDIMM_NDP_TARGET_BK

    vec_size = int(mat_input_size_byte_list[input_size] / 2)  # bytes→elements (FP16)

    # Partial vector sizing
    if max_p_vec_size > vec_size:
        p_vec_size = vec_size
        col_tile = 1
        if row_size > vec_size:
            row_vec_ratio = int(row_size / vec_size)
            opsize = int((opsize + 1) / row_vec_ratio) - 1
    else:
        p_vec_size = max_p_vec_size
        col_tile = int(vec_size / p_vec_size)

    # Rows needed for partial vector
    if row_size > p_vec_size:
        n_row_p_vec = 1
    else:
        n_row_p_vec = int(p_vec_size / row_size)

    # Total tile rows across all units (ch × rank × bg)
    n_tile_row = n_bg * ASYNCDIMM_NUM_CHANNEL * ASYNCDIMM_NUM_RANK

    # Tile block row sizing (same algorithm as gemv_pch)
    tmp0 = n_row_p_vec * n_bg
    if col_tile > 1:
        tmp0 = tmp0 * 2
    tmp1 = int(1024 / tmp0)
    if tmp1 >= 256:
        n_tile_block_row = 256
    elif tmp1 >= 128:
        n_tile_block_row = 128
    elif tmp1 >= 64:
        n_tile_block_row = 64
    elif tmp1 >= 32:
        n_tile_block_row = 32
    elif tmp1 >= 16:
        n_tile_block_row = 16
    elif tmp1 >= 8:
        n_tile_block_row = 8
    elif tmp1 >= 4:
        n_tile_block_row = 4
    elif tmp1 >= 2:
        n_tile_block_row = 2
    elif tmp1 >= 1:
        n_tile_block_row = 1
    else:
        print("Error: too small n_tile_block_row")
        exit(1)

    if col_tile > 1:
        tmp2 = n_tile_block_row * n_row_p_vec * n_bg * 2
    else:
        tmp2 = n_tile_block_row * n_row_p_vec * n_bg
    if tmp2 > 1000:
        n_tile_block_row = int(n_tile_block_row / 2)

    # Iteration count
    iteration_tile_block = int(vec_size / (n_tile_block_row * n_tile_row))
    if iteration_tile_block == 0:
        n_tile_block_row = int(vec_size / n_tile_row)
        iteration_tile_block = int(vec_size / (n_tile_block_row * n_tile_row))

    # Result vector parameters
    n_partial_y = n_tile_block_row * n_bg
    post_n_it = int(n_partial_y / (max_opsize + 1))
    opsize_v_rec = max_opsize
    opsize_s_rec = max_opsize
    opsize_wbd = int(n_partial_y / n_per_acc)

    print(f"  AsyncDIMM GEMV: size={input_size}, vec_size={vec_size}")
    print(f"    opsize={opsize}, p_vec_size={p_vec_size}, col_tile={col_tile}")
    print(f"    n_row_p_vec={n_row_p_vec}, n_tile_block_row={n_tile_block_row}")
    print(f"    iteration_tile_block={iteration_tile_block}, post_n_it={post_n_it}")
    print(f"    opsize_wbd={opsize_wbd}")

    n_mac_iterations = n_tile_block_row * n_row_p_vec

    # Write DRAM (Vector Data) — BK=0 (vector uses BK=0 in LOAD)
    num_banks = ASYNCDIMM_NUM_BANK
    n_wr_dram_row = int(vec_size / row_size)
    if n_wr_dram_row == 0:
        n_wr_dram_row = 1
    print(f"    Writing vector data to DRAM: {n_wr_dram_row} rows × {ASYNCDIMM_NUM_COL} cols × {ASYNCDIMM_NUM_CHANNEL} ch × {ASYNCDIMM_NUM_RANK} rk × {n_row_p_vec} bg")
    for ro in range(n_wr_dram_row):
        for co in range(ASYNCDIMM_NUM_COL):
            for ch in range(ASYNCDIMM_NUM_CHANNEL):
                for rk in range(ASYNCDIMM_NUM_RANK):
                    for bg in range(n_row_p_vec):
                        addr = encode_asyncdimm_address(ch, rk, bg, 0, 1000 + ro, co)
                        write_normal_trace(f, 'ST', addr)

    # BK interleaving: split MAC iterations into BK phases
    if n_mac_iterations >= num_banks:
        mac_iters_per_bk = n_mac_iterations // num_banks
    else:
        mac_iters_per_bk = 1
        num_banks = n_mac_iterations  # Use fewer banks for MAC phase

    for ch in range(ASYNCDIMM_NUM_CHANNEL):
        for rk in range(ASYNCDIMM_NUM_RANK):
            nma_inst_list = []

            nma_inst_list.append(nma_inst_set_base(1, 3000))  # matrix base
            nma_inst_list.append(nma_inst_set_base(2, 7000))  # result base

            if col_tile > 1:
                jump_outer = len(nma_inst_list)
                nma_inst_list.append(nma_inst_set_base(0, 1000))
            else:
                nma_inst_list.append(nma_inst_set_base(0, 1000))
                jump_outer = len(nma_inst_list)

            # Load partial vector (BK=0)
            for bg in range(n_row_p_vec):
                nma_inst_list.append(
                    nma_inst(ndp_inst_opcode["LOAD"], opsize, bg, 0, 0, 0, 0))
            nma_inst_list.append(
                nma_inst(ndp_inst_opcode["BARRIER"], 0, 0, 0, 0, 0, 0))

            # MAC — split into BK phases
            for bk_phase in range(num_banks):
                iters_this_bk = mac_iters_per_bk + (1 if bk_phase < (n_mac_iterations % num_banks) else 0)
                if iters_this_bk == 0: continue
                jump_mac = len(nma_inst_list)
                for bg in range(n_bg):
                    nma_inst_list.append(
                        nma_inst(ndp_inst_opcode["MAC"], opsize, bg, bk_phase, 0, 0, 1))
                nma_inst_list.append(
                    nma_inst(ndp_inst_opcode["BARRIER"], 0, 0, 0, 0, 0, 0))
                nma_inst_list.append(nma_inst_inc_base(1, 1))
                if iters_this_bk > 1:
                    nma_inst_list.append(
                        nma_inst_loop(iters_this_bk - 1, jump_mac))

            # Column tile inner loop (col_tile > 1)
            if col_tile > 1:
                jump_col = len(nma_inst_list)
                nma_inst_list.append(nma_inst_inc_base(0, n_row_p_vec))

                for bg in range(n_row_p_vec):
                    nma_inst_list.append(
                        nma_inst(ndp_inst_opcode["LOAD"], opsize, bg, 0, 0, 0, 0))
                nma_inst_list.append(
                    nma_inst(ndp_inst_opcode["BARRIER"], 0, 0, 0, 0, 0, 0))

                # MAC for col tile — same BK phase split
                for bk_phase in range(num_banks):
                    iters_this_bk = mac_iters_per_bk + (1 if bk_phase < (n_mac_iterations % num_banks) else 0)
                    if iters_this_bk == 0: continue
                    jump_mac2 = len(nma_inst_list)
                    for bg in range(n_bg):
                        nma_inst_list.append(
                            nma_inst(ndp_inst_opcode["MAC"], opsize, bg, bk_phase, 0, 0, 1))
                    nma_inst_list.append(
                        nma_inst(ndp_inst_opcode["BARRIER"], 0, 0, 0, 0, 0, 0))
                    nma_inst_list.append(nma_inst_inc_base(1, 1))
                    if iters_this_bk > 1:
                        nma_inst_list.append(
                            nma_inst_loop(iters_this_bk - 1, jump_mac2))

                if col_tile > 2:
                    nma_inst_list.append(
                        nma_inst_loop(col_tile - 2, jump_col))

            # Final barrier before reduction
            nma_inst_list.append(
                nma_inst(ndp_inst_opcode["BARRIER"], 0, 0, 0, 0, 0, 0))

            # Reduction: T_ADD (partial sum vector reduction)
            if post_n_it // n_bg > 0:
                for _ in range(post_n_it // n_bg):
                    for bg in range(n_bg):
                        nma_inst_list.append(
                            nma_inst(ndp_inst_opcode["T_ADD"], opsize_v_rec, bg, 0, 0, 0, 0))
                    nma_inst_list.append(
                        nma_inst(ndp_inst_opcode["BARRIER"], 0, 0, 0, 0, 0, 0))
            if post_n_it % n_bg != 0:
                for bg in range(post_n_it % n_bg):
                    nma_inst_list.append(
                        nma_inst(ndp_inst_opcode["T_ADD"], opsize_v_rec, bg, 0, 0, 0, 0))
                nma_inst_list.append(
                    nma_inst(ndp_inst_opcode["BARRIER"], 0, 0, 0, 0, 0, 0))

            # Reduction: T_S_RED (scalar reduction)
            if post_n_it // n_bg > 0:
                for _ in range(post_n_it // n_bg):
                    for bg in range(n_bg):
                        nma_inst_list.append(
                            nma_inst(ndp_inst_opcode["T_S_RED"], opsize_s_rec, bg, 0, 0, 0, 0))
                    nma_inst_list.append(
                        nma_inst(ndp_inst_opcode["BARRIER"], 0, 0, 0, 0, 0, 0))
            if post_n_it % n_bg != 0:
                for bg in range(post_n_it % n_bg):
                    nma_inst_list.append(
                        nma_inst(ndp_inst_opcode["T_S_RED"], opsize_s_rec, bg, 0, 0, 0, 0))
                nma_inst_list.append(
                    nma_inst(ndp_inst_opcode["BARRIER"], 0, 0, 0, 0, 0, 0))

            # WBD result (bg=0, BK=0, write to base[2])
            nma_inst_list.append(
                nma_inst(ndp_inst_opcode["WBD"], opsize_wbd, 0, 0, 0, 0, 2))

            # Outer loop: advance result base + repeat
            if iteration_tile_block > 1:
                nma_inst_list.append(nma_inst_inc_base(2, 1))
                nma_inst_list.append(
                    nma_inst_loop(iteration_tile_block - 1, jump_outer))

            nma_inst_list.append(
                nma_inst(ndp_inst_opcode["EXIT"], 0, 0, 0, 0, 0, 0))

            if len(nma_inst_list) >= MAX_INST:
                print(f"Error: NMAInst count ({len(nma_inst_list)}) exceeds MAX_INST ({MAX_INST})")
                exit(1)

            dump_asyncdimm_nma_inst(f, nma_inst_list, ch, rk)
            asyncdimm_start_nma(f, ch, rk)


def generate_trace(workload, size, output_path='', pch=True, is_ndp_ops=True, scaling_factor=-1, num_dimm=1):
    global GEN_PCH_NORMAL_MODE
    global NUM_DIMM
    global NUM_CHANNEL
    NUM_DIMM = num_dimm
    NUM_CHANNEL = int(NUM_CH_PER_DIMM * NUM_DIMM)
    if pch:
        GEN_PCH_NORMAL_MODE = True
    else:
        GEN_PCH_NORMAL_MODE = False

    file_name = "none.txt"
    if workload in workload_list:
        file_name = workload + ".txt"
    else:
        print("Error: Not Valid Workload")
        exit(1)

    if workload == "GEMV":
        if size in mat_input_size_list:
            file_name = size + "_" + file_name        
        else:
            print("Error: Wrong Input Size")
            exit(1)
    else:
        if size in input_size_list:
            file_name = size + "_" + file_name
        else:
            print("Error: Wrong Input Size")
            exit(1)

    # DIMM tag: only added when num_dimm > 1 (backward compatible)
    dimm_tag = f"_d{num_dimm}" if num_dimm > 1 else ""

    if is_ndp_ops:
        if scaling_factor == 1:
            file_name = f"pch_ndp_x4{dimm_tag}_" + file_name
        elif scaling_factor == 2:
            file_name = f"pch_ndp_x8{dimm_tag}_" + file_name
        elif scaling_factor == 4:
            file_name = f"pch_ndp_x16{dimm_tag}_" + file_name
        else:
            print("Error: Wrong Input Size")
            exit(1)
    else:
        if pch:
            if scaling_factor == 1:
                file_name = f"pch_non_ndp_x4{dimm_tag}_" + file_name
            elif scaling_factor == 2:
                file_name = f"pch_non_ndp_x8{dimm_tag}_" + file_name
            elif scaling_factor == 4:
                file_name = f"pch_non_ndp_x16{dimm_tag}_" + file_name
        else:
            file_name = f"baseline{dimm_tag}_" + file_name
    
    # Set Address Mapping Scheme 
    config_scale_factor(scaling_factor)

    file_name = output_path + "/" + file_name
    # Size: "8K","32K","128K","512K","2M","8M" (GEMV: 8K, 16K, 32K, 64K, 128K)
    # workload: "AXPBY", "AXPBYPCZ", "AXPY", "COPY", "XMY", "DOT", "SCAL", "GEMV"
    # Compute per-PCH byte size for BLAS 1-level (fixed total data)
    if workload != "GEMV":
        per_pch = calc_per_pch_bytes(size)
    with open(file_name, 'w') as f:
        if is_ndp_ops:
            if workload == "AXPBY":
                axpby_pch(f,per_pch,scaling_factor)
            elif workload == "AXPBYPCZ":
                axpbypcz_pch(f,per_pch,scaling_factor)
            elif workload == "AXPY":
                axpy_pch(f,per_pch,scaling_factor)
            elif workload == "COPY":
                copy_pch(f,per_pch,scaling_factor)
            elif workload == "XMY":
                xmy_pch(f,per_pch,scaling_factor)
            elif workload == "DOT":
                dot_pch(f,per_pch,scaling_factor)
            elif workload == "GEMV":
                gemv_pch(f,size,scaling_factor)                                            
            else:
                print("Error: Not Implemented Workload")
                exit(1)
        else:
            if workload == "AXPBY":
                axpby_normal(f,size)
            elif workload == "AXPBYPCZ":
                axpbypcz_normal(f,size)
            elif workload == "AXPY":
                axpy_normal(f,size)          
            elif workload == "COPY":
                copy_normal(f,size)          
            elif workload == "XMY":
                xmy_normal(f,size)                                    
            elif workload == "DOT":
                dot_normal(f,size)  
            elif workload == "GEMV":
                gemv_normal(f,size)                    
            else:
                print("Error: Not Implemented Workload")
                exit(1)

    print(f"-> Generated memory traces in '{file_name}'.")

def generate_asyncdimm_trace(workload, size, output_path=''):
    """Generate trace for AsyncDIMM NMA workload (base DDR5, rank-level NMA MC)."""
    asyncdimm_workloads = {"COPY", "AXPY", "AXPBY", "AXPBYPCZ", "XMY", "DOT", "GEMV"}

    if workload not in asyncdimm_workloads:
        print(f"Error: AsyncDIMM workload '{workload}' not implemented. Available: {asyncdimm_workloads}")
        return

    # GEMV uses mat_input_size_list; others use input_size_list
    if workload == "GEMV":
        if size not in mat_input_size_list:
            print(f"Error: Invalid GEMV size '{size}'")
            return
    else:
        if size not in input_size_list:
            print(f"Error: Invalid size '{size}'")
            return

    file_name = f"asyncdimm_nma_{size}_{workload}.txt"
    file_name = output_path + "/" + file_name

    workload_funcs = {
        "COPY":     copy_asyncdimm,
        "AXPY":     axpy_asyncdimm,
        "AXPBY":    axpby_asyncdimm,
        "AXPBYPCZ": axpbypcz_asyncdimm,
        "XMY":      xmy_asyncdimm,
        "DOT":      dot_asyncdimm,
        "GEMV":     gemv_asyncdimm,
    }

    with open(file_name, 'w') as f:
        workload_funcs[workload](f, size)

    print(f"-> Generated AsyncDIMM trace in '{file_name}'.")


def generate_sls_trace(sls_config_name, output_path=''):
    """Generate baseline SLS trace for conventional DDR5."""
    if sls_config_name not in sls_config_list:
        print(f"Error: Invalid SLS config '{sls_config_name}'. Available: {list(sls_config_list.keys())}")
        return

    file_name = f"baseline_{sls_config_name}.txt"
    file_name = output_path + "/" + file_name

    with open(file_name, 'w') as f:
        sls_normal(f, sls_config_name)

    print(f"-> Generated SLS trace in '{file_name}'.")


def crete_folder(folder_path):
    if not os.path.exists(folder_path):
        os.mkdir(folder_path)

if __name__ == '__main__':
    print("=" * 72)
    print(" NDP Workload Trace Generator")
    print("=" * 72)

    BLAS_WORKLOADS = ["COPY", "AXPY", "AXPBY", "AXPBYPCZ", "XMY", "DOT"]

    # =====================================================================
    #  Experiment selector — set True/False to enable/disable each
    # =====================================================================
    GEN_EXP1_BASE_STANDALONE   = True   # 1 DIMM, x4: Base, AsyncDIMM-N, DBX-N
    GEN_EXP2_SCALING_DRAM      = True   # 1 DIMM: Base, DBX_x4, DBX_x8, DBX_x16
    GEN_EXP3_SCALING_DIMM      = True   # x4: 1, 2, 4, 8, 16 DIMM DBX
    GEN_SLS_BASELINE           = False   # SLS baseline (conventional DDR5)
    GEN_SLS_DBX                = False   # SLS DBX-DIMM NDP (Wildcard + Interleaving)

    root_path = "generated_traces"

    def setup_dir(path):
        """Create directory tree, removing existing if present."""
        p = Path(path)
        if p.exists() and p.is_dir():
            shutil.rmtree(path)
            print(f"  Removed existing: {path}")
        os.makedirs(path, exist_ok=True)

    # =================================================================
    #  Experiment 1: Base Standalone
    #    1 DIMM, x4 DRAM
    #    - baseline (conventional DDR5)
    #    - pch_non_ndp (DBX host-only, no NDP)
    #    - pch_ndp (DBX with NDP = DBX-N)
    #    - asyncdimm_nma (AsyncDIMM-N)
    # =================================================================
    if GEN_EXP1_BASE_STANDALONE:
        exp1_root = root_path + "/exp1_base_standalone"
        exp1_baseline     = exp1_root + "/baseline"
        exp1_pch_non_ndp  = exp1_root + "/pch_non_ndp"
        exp1_pch_ndp      = exp1_root + "/pch_ndp"
        exp1_asyncdimm    = exp1_root + "/asyncdimm_nma"

        setup_dir(exp1_root)
        for d in [exp1_baseline, exp1_pch_non_ndp, exp1_pch_ndp, exp1_asyncdimm]:
            os.makedirs(d, exist_ok=True)

        print()
        print("[Exp1] Base Standalone: 1 DIMM, x4")
        print(f"  Output: {exp1_root}")
        print()

        # BLAS 1-level workloads
        for bench in BLAS_WORKLOADS:
            for size in input_size_list:
                # Baseline (conventional DDR5, no pCH)
                generate_trace(bench, size, exp1_baseline, pch=False, is_ndp_ops=False, scaling_factor=1, num_dimm=1)
                # DBX host-only (pCH, no NDP)
                generate_trace(bench, size, exp1_pch_non_ndp, pch=True, is_ndp_ops=False, scaling_factor=1, num_dimm=1)
                # DBX-N (pCH + NDP)
                generate_trace(bench, size, exp1_pch_ndp, pch=True, is_ndp_ops=True, scaling_factor=1, num_dimm=1)
                # AsyncDIMM-N
                generate_asyncdimm_trace(bench, size, exp1_asyncdimm)

        # GEMV
        for size in mat_input_size_list:
            generate_trace("GEMV", size, exp1_baseline, pch=False, is_ndp_ops=False, scaling_factor=1, num_dimm=1)
            generate_trace("GEMV", size, exp1_pch_non_ndp, pch=True, is_ndp_ops=False, scaling_factor=1, num_dimm=1)
            generate_trace("GEMV", size, exp1_pch_ndp, pch=True, is_ndp_ops=True, scaling_factor=1, num_dimm=1)
            generate_asyncdimm_trace("GEMV", size, exp1_asyncdimm)

        print(f"[Exp1] Done.")

    # =================================================================
    #  SLS Baseline: Recommendation System workload
    #    Conventional DDR5, Zipf random access to 128 embedding tables
    # =================================================================
    if GEN_SLS_BASELINE:
        sls_root = root_path + "/sls_baseline"
        setup_dir(sls_root)

        print()
        print("[SLS] Baseline: conventional DDR5, Zipf(1.05)")
        print(f"  Output: {sls_root}")
        print()

        for sls_cfg in sls_config_names:
            generate_sls_trace(sls_cfg, sls_root)

        print(f"[SLS] Done.")

    # =================================================================
    #  SLS DBX-DIMM NDP: Wildcard + Intra-Table Bank Interleaving
    #    x4 DRAM, 1 DIMM, 8 PCH (2CH × 4PCH), 16 tables/PCH
    # =================================================================
    if GEN_SLS_DBX:
        sls_dbx_root = root_path + "/sls_dbx"
        setup_dir(sls_dbx_root)

        print()
        print("[SLS-DBX] DBX-DIMM NDP: Wildcard + Interleaving")
        print(f"  Output: {sls_dbx_root}")
        print()

        for sls_cfg in sls_config_names:
            generate_sls_pch_trace(sls_cfg, sls_dbx_root)

        print(f"[SLS-DBX] Done.")

    # =================================================================
    #  Experiment 2: Scaling DRAM Device
    #    1 DIMM, x4 / x8 / x16
    #    - baseline (conventional DDR5, x4 only)
    #    - pch_non_ndp per scaling
    #    - pch_ndp per scaling (DBX_x4, DBX_x8, DBX_x16)
    # =================================================================
    if GEN_EXP2_SCALING_DRAM:
        exp2_root = root_path + "/exp2_scaling_dram"
        exp2_baseline    = exp2_root + "/baseline"
        exp2_pch_non_ndp = exp2_root + "/pch_non_ndp"
        exp2_pch_ndp     = exp2_root + "/pch_ndp"

        setup_dir(exp2_root)
        for d in [exp2_baseline, exp2_pch_non_ndp, exp2_pch_ndp]:
            os.makedirs(d, exist_ok=True)

        print()
        print("[Exp2] Scaling DRAM Device: 1 DIMM, x4/x8/x16")
        print(f"  Output: {exp2_root}")
        print()

        # BLAS 1-level
        for bench in BLAS_WORKLOADS:
            for size in input_size_list:
                # Baseline (x4 only, conventional)
                generate_trace(bench, size, exp2_baseline, pch=False, is_ndp_ops=False, scaling_factor=1, num_dimm=1)
                for scaling in [1, 2, 4]:
                    generate_trace(bench, size, exp2_pch_non_ndp, pch=True, is_ndp_ops=False, scaling_factor=scaling, num_dimm=1)
                    generate_trace(bench, size, exp2_pch_ndp, pch=True, is_ndp_ops=True, scaling_factor=scaling, num_dimm=1)

        # GEMV
        for size in mat_input_size_list:
            generate_trace("GEMV", size, exp2_baseline, pch=False, is_ndp_ops=False, scaling_factor=1, num_dimm=1)
            for scaling in [1, 2, 4]:
                generate_trace("GEMV", size, exp2_pch_non_ndp, pch=True, is_ndp_ops=False, scaling_factor=scaling, num_dimm=1)
                generate_trace("GEMV", size, exp2_pch_ndp, pch=True, is_ndp_ops=True, scaling_factor=scaling, num_dimm=1)

        print(f"[Exp2] Done.")

    # =================================================================
    #  Experiment 3: Scaling DIMMs
    #    x4 DRAM, 1 / 2 / 4 / 8 / 16 DIMM
    #    - pch_ndp per DIMM count (DBX-N)
    #    Fixed total data: input_size × 8 PCH (1 DIMM baseline)
    # =================================================================
    if GEN_EXP3_SCALING_DIMM:
        exp3_root    = root_path + "/exp3_scaling_dimm"
        exp3_pch_ndp = exp3_root + "/pch_ndp"

        setup_dir(exp3_root)
        os.makedirs(exp3_pch_ndp, exist_ok=True)

        print()
        print("[Exp3] Scaling DIMMs: x4, 1/2/4/8/16 DIMM")
        print(f"  Output: {exp3_root}")
        print()

        dimm_counts = [1, 2, 4, 8, 16]

        # BLAS 1-level
        for bench in BLAS_WORKLOADS:
            for size in input_size_list:
                for nd in dimm_counts:
                    generate_trace(bench, size, exp3_pch_ndp, pch=True, is_ndp_ops=True, scaling_factor=1, num_dimm=nd)

        # GEMV
        for size in mat_input_size_list:
            for nd in dimm_counts:
                generate_trace("GEMV", size, exp3_pch_ndp, pch=True, is_ndp_ops=True, scaling_factor=1, num_dimm=nd)

        print(f"[Exp3] Done.")

    print()
    print("=" * 72)
    print(" All trace generation complete.")
    print(f" Output root: {root_path}/")
    print("=" * 72)
