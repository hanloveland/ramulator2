
import math
import os
import shutil

# 1 (x4), 2 (x8), 4 (x16)
DRAM_SCALING=1

# Default :  DRAM_SCALING=1
NUM_CHANNEL = 2
NUM_PSEUDOCHANNEL = 4
NUM_RANK = 4 # Basline DRAM
NUM_BANKGROUP = 8
NUM_BANK = 4
NUM_ROW = 65536
NUM_COL = 128 # 1KB/8B    
NDP_ACC_GRA = 1 # Same with Normal Access

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
NORMAL_COLUMN_BITS     = int(math.log2(BURST_SIZE))           # 2K Column (1KB/8B=128)
NORMAL_BURST_SIZE      = 64                                   # Prefetch Size 64
NORMAL_GRANULARITY     = int(math.log2(NORMAL_BURST_SIZE))    # 64B 

NORMAL_SCALE_FACTOR = 4 #  PCH per CH 

MAX_INST        = 1024 # 8K/8B
NDP_ROW         = 65535

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

PCH_ADDRESS_SCHEME = "RoCoBaBgRaPcCH"
# PCH_ADDRESS_SCHEME = "BaRoCoBgRaPcCH"
# PCH_ADDRESS_SCHEME = "RoBaBgRaCoPcCH"
# NORMAL_ADDRESS_SCHEME = "RoCoBaRaCh"
# NORMAL_ADDRESS_SCHEME = "RoBaRaCoCh"
NORMAL_ADDRESS_SCHEME = "RoRaCoBaCh"

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
    "LOOP"           :52
}

ndp_acc_inst_opcode = {
    # "RD", "WR", "BAR", "WAIT_RES", "LOOP_START", "LOOP_END", "WAIT", "DONE" 
    "RD"         :0,
    "WR"         :1,
    "BAR"        :2,
    "WAIT_RES"   :3,
    "LOOP_START" :4,
    "LOOP_END"   :5,
    "WAIT"       :6,
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

def config_scale_factor(scaling_factor):
    global NUM_CHANNEL
    global NUM_PSEUDOCHANNEL
    global NUM_BANKGROUP
    global NUM_BANK
    global NUM_ROW
    global NUM_COL
    global NDP_ACC_GRA    
    if scaling_factor == 1:
        NUM_CHANNEL = 2
        NUM_PSEUDOCHANNEL = 4
        NUM_BANKGROUP = 8
        NUM_BANK = 4
        NUM_ROW = 65536
        NUM_COL = 128 # 1KB/8B    
        NDP_ACC_GRA = 1 # Same with Normal Access
    elif scaling_factor == 2:
        NUM_CHANNEL = 2
        NUM_PSEUDOCHANNEL = 4
        NUM_BANKGROUP = 8
        NUM_BANK = 4
        NUM_ROW = 65536
        NUM_COL = 128 # 1KB/128B
        NDP_ACC_GRA = 2 # 2x then normal access
    elif scaling_factor == 4:
        NUM_CHANNEL = 2
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

def encode_address(channel, pseudo_channel, rank, bg, bank, row, col):
    """
        ROCoBaBgRaPcCH
        RoBaBgRaCoPcCH
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

    # print(f" Address Translate CH{channel},PCH{pseudo_channel},RK{rank},BG{bg},BK{bank},RO{row},CO{col} --> 0x{address:016X}\n")        
    return address

def encode_normal_address(channel, rank, bg, bank, row, col):
    """
        RoCoBaRaCh
        RoBaRaCoCh
        RoRaCoBaCh        
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

# NDP Instruction: Opcode, Opsize, ID, BG, BK, OP1, OP2, OP3
def inst(opcode,opsize,id,bg,bk,op1,op2,op3):
    inst_64bit = 0
    inst_64bit |= (opcode & 0x3f) << 58
    inst_64bit |= (opsize & 0x7f) << 51
    inst_64bit |= (id & 0x7) << 48
    inst_64bit |= (bg & 0x7) << 45
    inst_64bit |= (bk & 0x3) << 43
    if opcode == ndp_inst_opcode["LOOP"]:
        inst_64bit |= (op1 & 0x3FF) << 16      # Iteration
        inst_64bit |= (op2 & 0x3FF)            # PC
    return inst_64bit

# Access Type (4) / Opsize (7)/ Channel(3) / Pseudo-Channel(2) / BG (3) / BK (3) / ROW (18) / COL (7) / ID (3) / Reserved
def acc_inst(opcode,opsize,ch,pch,bg,bk,row,col,id,etc):
    inst_64bit = 0
    inst_64bit |= (opcode & 0xf)      << 60
    inst_64bit |= (opsize & 0x7f)     << 53
    inst_64bit |= (ch     & 0x7)      << 50
    inst_64bit |= (pch    & 0x3)      << 48
    inst_64bit |= (bg     & 0x7)      << 45
    inst_64bit |= (bk     & 0x3)      << 43
    inst_64bit |= (row    & 0x3FFFF)  << 25
    inst_64bit |= (col    & 0x7F)     << 18
    inst_64bit |= (id     & 0x7)      << 15
    inst_64bit |= (etc    & 0x7FFF)  
    return inst_64bit

def dump_ndp_acc_inst(f, inst_list):
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
    for ro in range(NUM_ROW):
        for rk in range(NUM_RANK):        
            for ba in range(NUM_BANK):
                for co in range(NUM_COL): 
                    for bg in range(NUM_BANKGROUP):
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

def cal_it(input_size, scaling):
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
def axpby_pch(f, input_size, scaling):
    '''
        input_size: 8K, 32K, 128K, 512K, 2M, 8M        
        Z = aX + bY
        Scaling Factor 1 (x4)
        row size: 8K
        data memory: 32KB-> Max 4 Row
        --- Iteration InputSize/8K -------
        LOAD_MUL:  8KB
        BARRIER 
        SCALE_ADD: 8KB
        BARRIER 
        WBD: 8KB
        ---
        Input: 8K
        BG0/ROW5000, BG0/ROW6000 -> BG0/ROW7000
        COL:0-127 or 0-63
        BG0-3/ROW5000, BG0-3/ROW6000 -> BG0/ROW7000
        COL:0-127 or 0-63                 
    '''

    iteration, num_working_bg, opsize = cal_it_v2(input_size, scaling)

    ndp_bk_idx = NDP_TARGET_BK

    # Input_size x 8 pch 
    print(f"Working Bank Group: {num_working_bg}")
    print(f"Iteration: {iteration}")
    # Max NDP Instruction 8KB/8B=1K
    for ch in range(int(NUM_CHANNEL)):
        for pch in range(int(NUM_PSEUDOCHANNEL)):
            ndp_inst_list = []
            jump_pc = len(ndp_inst_list)
            for bg in range(num_working_bg):
                ndp_inst_list.append(inst(ndp_inst_opcode["LOAD_MUL"],opsize,0,bg,ndp_bk_idx,0,0,0))
            for bg in range(num_working_bg):                
                ndp_inst_list.append(inst(ndp_inst_opcode["SCALE_ADD"],opsize,1,bg,ndp_bk_idx,0,0,0))       
            for bg in range(num_working_bg):                
                ndp_inst_list.append(inst(ndp_inst_opcode["WBD"],opsize,2,bg,ndp_bk_idx,0,0,0))                      
            if iteration > 1:
                ndp_inst_list.append(inst(ndp_inst_opcode["LOOP"],0,0,0,0,(iteration-1),jump_pc,0))
            ndp_inst_list.append(inst(ndp_inst_opcode["EXIT"],0,0,0,0,0,0,0))
            if(len(ndp_inst_list) >= 1024):
                print("Error: Over NDP Instruction Memory")
                exit(1)
            dump_ndp_inst(f,ndp_inst_list,ch,pch)

    # Generate NDP Start Request 
    data_array = [0] * 8
    for ch in range(int(NUM_CHANNEL)):
        for pch in range(int(NUM_PSEUDOCHANNEL)):  
            idx = ch * 4 + pch     
            data_array[idx] = 1   
    
    write_trace(f,'ST',encode_address(0, 0, 0, HSNU_CTR_REG_BG, HSNU_CTR_REG_BK, NDP_ROW, 0),data_array)

    # Make 2-D NDL-Launch Request Inst
    acc_inst_list = [[ ]]
    acc_inst_list_num = [[ ]]
    for ch in range(int(NUM_CHANNEL)):
        for pch in range(int(NUM_PSEUDOCHANNEL)):    
            acc_inst_list.append([])
            acc_inst_list_num.append(0)
    # Start NDP ops (HSNU-Ctrl Access Info Buffer)
    # Type / Opsize/ Channel/ Pseudo-Channel/ BG/ BK / ROW/ COL/ ID / Reserved
    for ch in range(int(NUM_CHANNEL)):
        for pch in range(int(NUM_PSEUDOCHANNEL)):    
            idx = ch * int(NUM_PSEUDOCHANNEL) + pch
            for i in range(iteration):
                for bg in range(num_working_bg):
                    acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["RD"],opsize,ch,pch,bg,ndp_bk_idx,(5000+i),0,0,0))            
                acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["BAR"],0,0,0,0,0,0,0,0,0))
                for bg in range(num_working_bg):
                    acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["RD"],opsize,ch,pch,bg,ndp_bk_idx,(6000+i),0,1,0))               
                acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["BAR"],0,0,0,0,0,0,0,0,0))
                for bg in range(num_working_bg):
                    acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["WR"],opsize,ch,pch,bg,ndp_bk_idx,(7000+i),0,2,0))             
                acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["BAR"],0,0,0,0,0,0,0,0,0))
            acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["DONE"],0,0,0,0,0,0,0,0,0))

    # Reshape 2-D NDL-Lauch Request Lists to 1-D List
    if(len(acc_inst_list) >= 1024):
        print("Error: Over NDP Instruction Memory")
        exit(1)    
    dump_ndp_acc_inst_2d(f,acc_inst_list,0)
       
    return 

# AXPBYPCZ  : W = aX + bB + zZ
def axpbypcz_pch(f, input_size, scaling):
    '''
        input_size: 8K, 32K, 128K, 512K, 2M, 8M        
        Z = W = aX + bB + zZ
        x4  DRAM: data memory: 32KB  -> Max 4 Row
        x8  DRAM: data memory: 64KB  -> Max 8 Row
        x16 DRAM: data memory: 128KB -> Max 4 Row (max bank group)
        --- Iteration InputSize/8K -------
        LOAD_MUL: 8KB
        BARRIER 
        SCALE_ADD: 8KB
        BARRIER 
        SCALE_ADD: 8KB
        BARRIER 
        WBD: 8KB
        ---
        Input X: 8K
        BG0-7/ROW5000
        Input B: 8K
        BG0-7/ROW6000
        Input Z: 8K
        BG0-7/ROW7000 
        Output W
        BG0-7/ROW8000       
        
    '''

    iteration, num_working_bg, opsize = cal_it_v2(input_size, scaling)

    ndp_bk_idx = NDP_TARGET_BK

    # Input_size x 8 pch 
    print(f"Working Bank Group: {num_working_bg}")
    print(f"Iteration: {iteration}")
    # Max NDP Instruction 8KB/8B=1K
    # NDP Instruction: Opcode, Opsize, ID, BG, BK, OP1, OP2, OP3
    for ch in range(int(NUM_CHANNEL)):
        for pch in range(int(NUM_PSEUDOCHANNEL)):
            ndp_inst_list = []
            jump_pc = len(ndp_inst_list) 
            for bg in range(num_working_bg):
                ndp_inst_list.append(inst(ndp_inst_opcode["LOAD_MUL"],opsize,0,bg,ndp_bk_idx,0,0,0))       
            for bg in range(num_working_bg):                
                ndp_inst_list.append(inst(ndp_inst_opcode["SCALE_ADD"],opsize,1,bg,ndp_bk_idx,0,0,0))               
            for bg in range(num_working_bg):                
                ndp_inst_list.append(inst(ndp_inst_opcode["SCALE_ADD"],opsize,2,bg,ndp_bk_idx,0,0,0))   
            for bg in range(num_working_bg):
                ndp_inst_list.append(inst(ndp_inst_opcode["WBD"],opsize,3,bg,ndp_bk_idx,0,0,0))   
            if iteration > 1:
                ndp_inst_list.append(inst(ndp_inst_opcode["LOOP"],0,0,0,0,(iteration-1),jump_pc,0))
            ndp_inst_list.append(inst(ndp_inst_opcode["EXIT"],0,0,0,0,0,0,0))
            if(len(ndp_inst_list) >= 1024):
                print("Error: Over NDP Instruction Memory")
                exit(1)
            dump_ndp_inst(f,ndp_inst_list,ch,pch)

    # Generate NDP Start Request 
    data_array = [0] * 8
    for ch in range(int(NUM_CHANNEL)):
        for pch in range(int(NUM_PSEUDOCHANNEL)):  
            idx = ch * int(NUM_PSEUDOCHANNEL) + pch     
            data_array[idx] = 1   
    
    write_trace(f,'ST',encode_address(0, 0, 0, HSNU_CTR_REG_BG, HSNU_CTR_REG_BK, NDP_ROW, 0),data_array)

    # Make 2-D NDL-Launch Request Inst
    acc_inst_list = [[ ]]
    acc_inst_list_num = [[ ]]
    for ch in range(int(NUM_CHANNEL)):
        for pch in range(int(NUM_PSEUDOCHANNEL)):    
            acc_inst_list.append([])
            acc_inst_list_num.append(0)
    # Start NDP ops (HSNU-Ctrl Access Info Buffer)
    # Type / Opsize/ Channel/ Pseudo-Channel/ BG/ BK / ROW/ COL/ ID / Reserved
    for ch in range(int(NUM_CHANNEL)):
        for pch in range(int(NUM_PSEUDOCHANNEL)):    
            idx = ch * int(NUM_PSEUDOCHANNEL) + pch
            for i in range(iteration):
                for bg in range(num_working_bg):
                    acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["RD"],opsize,ch,pch,bg,ndp_bk_idx,(5000+i),0,0,0))            
                acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["BAR"],0,0,0,0,0,0,0,0,0))
                for bg in range(num_working_bg):
                    acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["RD"],opsize,ch,pch,bg,ndp_bk_idx,(6000+i),0,1,0))               
                acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["BAR"],0,0,0,0,0,0,0,0,0))
                for bg in range(num_working_bg):
                    acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["RD"],opsize,ch,pch,bg,ndp_bk_idx,(7000+i),0,2,0))               
                acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["BAR"],0,0,0,0,0,0,0,0,0))                    
                for bg in range(num_working_bg):
                    acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["WR"],opsize,ch,pch,bg,ndp_bk_idx,(8000+i),0,3,0))             
                acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["BAR"],0,0,0,0,0,0,0,0,0))                    
            acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["DONE"],0,0,0,0,0,0,0,0,0))

    # Reshape 2-D NDL-Lauch Request Lists to 1-D List
    if(len(acc_inst_list) >= 1024):
        print("Error: Over NDP Instruction Memory")
        exit(1)    
    dump_ndp_acc_inst_2d(f,acc_inst_list,0)
       
    return 

# AXPY      : Y = aY + X
def axpy_pch(f, input_size, scaling):
    '''
        input_size: 8K, 32K, 128K, 512K, 2M, 8M        
        Z = aX + Y
        Scaling Factor 1 (x4)
        row size: 8K
        data memory: 32KB-> Max 4 Row
        --- Iteration InputSize/8K -------
        LOAD_MUL:  8KB
        BARRIER 
        SCALE_ADD: 8KB
        BARRIER 
        WBD: 8KB
        ---
        Input: 8K
        BG0/ROW5000, BG0/ROW6000 -> BG0/ROW7000
        COL:0-127 or 0-63
        BG0-3/ROW5000, BG0-3/ROW6000 -> BG0/ROW7000
        COL:0-127 or 0-63                 
    '''

    iteration, num_working_bg, opsize = cal_it_v2(input_size, scaling)

    ndp_bk_idx = NDP_TARGET_BK

    # Input_size x 8 pch 
    print(f"Working Bank Group: {num_working_bg}")
    print(f"Iteration: {iteration}")
    # Max NDP Instruction 8KB/8B=1K
    for ch in range(int(NUM_CHANNEL)):
        for pch in range(int(NUM_PSEUDOCHANNEL)):
            ndp_inst_list = []
            jump_pc = len(ndp_inst_list)
            for bg in range(num_working_bg):
                ndp_inst_list.append(inst(ndp_inst_opcode["LOAD_MUL"],opsize,0,bg,ndp_bk_idx,0,0,0))
            for bg in range(num_working_bg):                
                ndp_inst_list.append(inst(ndp_inst_opcode["ADD"],opsize,1,bg,ndp_bk_idx,0,0,0))       
            for bg in range(num_working_bg):                
                ndp_inst_list.append(inst(ndp_inst_opcode["WBD"],opsize,2,bg,ndp_bk_idx,0,0,0))                      
            if iteration > 1:
                ndp_inst_list.append(inst(ndp_inst_opcode["LOOP"],0,0,0,0,(iteration-1),jump_pc,0))
            ndp_inst_list.append(inst(ndp_inst_opcode["EXIT"],0,0,0,0,0,0,0))
            if(len(ndp_inst_list) >= 1024):
                print("Error: Over NDP Instruction Memory")
                exit(1)
            dump_ndp_inst(f,ndp_inst_list,ch,pch)

    # Generate NDP Start Request 
    data_array = [0] * 8
    for ch in range(int(NUM_CHANNEL)):
        for pch in range(int(NUM_PSEUDOCHANNEL)):  
            idx = ch * 4 + pch     
            data_array[idx] = 1   
    
    write_trace(f,'ST',encode_address(0, 0, 0, HSNU_CTR_REG_BG, HSNU_CTR_REG_BK, NDP_ROW, 0),data_array)

    # Make 2-D NDL-Launch Request Inst
    acc_inst_list = [[ ]]
    acc_inst_list_num = [[ ]]
    for ch in range(int(NUM_CHANNEL)):
        for pch in range(int(NUM_PSEUDOCHANNEL)):    
            acc_inst_list.append([])
            acc_inst_list_num.append(0)
    # Start NDP ops (HSNU-Ctrl Access Info Buffer)
    # Type / Opsize/ Channel/ Pseudo-Channel/ BG/ BK / ROW/ COL/ ID / Reserved
    for ch in range(int(NUM_CHANNEL)):
        for pch in range(int(NUM_PSEUDOCHANNEL)):    
            idx = ch * int(NUM_PSEUDOCHANNEL) + pch
            for i in range(iteration):
                for bg in range(num_working_bg):
                    acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["RD"],opsize,ch,pch,bg,ndp_bk_idx,(5000+i),0,0,0))            
                acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["BAR"],0,0,0,0,0,0,0,0,0))
                for bg in range(num_working_bg):
                    acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["RD"],opsize,ch,pch,bg,ndp_bk_idx,(6000+i),0,1,0))               
                acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["BAR"],0,0,0,0,0,0,0,0,0))
                for bg in range(num_working_bg):
                    acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["WR"],opsize,ch,pch,bg,ndp_bk_idx,(7000+i),0,2,0))             
                acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["BAR"],0,0,0,0,0,0,0,0,0))
            acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["DONE"],0,0,0,0,0,0,0,0,0))

    # Reshape 2-D NDL-Lauch Request Lists to 1-D List
    if(len(acc_inst_list) >= 1024):
        print("Error: Over NDP Instruction Memory")
        exit(1)    
    dump_ndp_acc_inst_2d(f,acc_inst_list,0)
       
    return     

# COPY      : Y = X
def copy_pch(f, input_size, scaling):
    '''
        input_size: 8K, 32K, 128K, 512K, 2M, 8M        
        Z = X
        Scaling Factor 1 (x4)
        row size: 8K
        data memory: 32KB-> Max 4 Row
        --- Iteration InputSize/8K -------
        LOAD_MUL:  8KB
        BARRIER 
        WBD: 8KB
        ---
        Input: 8K
        BG0/ROW5000, BG0/ROW6000 -> BG0/ROW7000
        COL:0-127 or 0-63
        BG0-3/ROW5000, BG0-3/ROW6000 -> BG0/ROW7000
        COL:0-127 or 0-63                 
    '''

    iteration, num_working_bg, opsize = cal_it_v2(input_size, scaling)
    ndp_bk_idx = NDP_TARGET_BK

    # Input_size x 8 pch 
    print(f"Working Bank Group: {num_working_bg}")
    print(f"Iteration: {iteration}")
    # Max NDP Instruction 8KB/8B=1K
    for ch in range(int(NUM_CHANNEL)):
        for pch in range(int(NUM_PSEUDOCHANNEL)):
            ndp_inst_list = []
            jump_pc = len(ndp_inst_list)
            for bg in range(num_working_bg):
                ndp_inst_list.append(inst(ndp_inst_opcode["LOAD"],opsize,0,bg,ndp_bk_idx,0,0,0))
            for bg in range(num_working_bg):                
                ndp_inst_list.append(inst(ndp_inst_opcode["WBD"],opsize,2,bg,ndp_bk_idx,0,0,0))                      
            if iteration > 1:
                ndp_inst_list.append(inst(ndp_inst_opcode["LOOP"],0,0,0,0,(iteration-1),jump_pc,0))
            ndp_inst_list.append(inst(ndp_inst_opcode["EXIT"],0,0,0,0,0,0,0))
            if(len(ndp_inst_list) >= 1024):
                print("Error: Over NDP Instruction Memory")
                exit(1)
            dump_ndp_inst(f,ndp_inst_list,ch,pch)

    # Generate NDP Start Request 
    data_array = [0] * 8
    for ch in range(int(NUM_CHANNEL)):
        for pch in range(int(NUM_PSEUDOCHANNEL)):  
            idx = ch * 4 + pch     
            data_array[idx] = 1   
    
    write_trace(f,'ST',encode_address(0, 0, 0, HSNU_CTR_REG_BG, HSNU_CTR_REG_BK, NDP_ROW, 0),data_array)

    # Make 2-D NDL-Launch Request Inst
    acc_inst_list = [[ ]]
    acc_inst_list_num = [[ ]]
    for ch in range(int(NUM_CHANNEL)):
        for pch in range(int(NUM_PSEUDOCHANNEL)):    
            acc_inst_list.append([])
            acc_inst_list_num.append(0)
    # Start NDP ops (HSNU-Ctrl Access Info Buffer)
    # Type / Opsize/ Channel/ Pseudo-Channel/ BG/ BK / ROW/ COL/ ID / Reserved
    for ch in range(int(NUM_CHANNEL)):
        for pch in range(int(NUM_PSEUDOCHANNEL)):    
            idx = ch * int(NUM_PSEUDOCHANNEL) + pch
            for i in range(iteration):
                for bg in range(num_working_bg):
                    acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["RD"],opsize,ch,pch,bg,ndp_bk_idx,(5000+i),0,0,0))            
                acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["BAR"],0,0,0,0,0,0,0,0,0))
                for bg in range(num_working_bg):
                    acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["WR"],opsize,ch,pch,bg,ndp_bk_idx,(7000+i),0,2,0))             
                acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["BAR"],0,0,0,0,0,0,0,0,0))
            acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["DONE"],0,0,0,0,0,0,0,0,0))

    # Reshape 2-D NDL-Lauch Request Lists to 1-D List
    if(len(acc_inst_list) >= 1024):
        print("Error: Over NDP Instruction Memory")
        exit(1)    
    dump_ndp_acc_inst_2d(f,acc_inst_list,0)
       
    return 


# XMY       : Y = X ⨀ Y
def xmy_pch(f, input_size, scaling):
    '''
        input_size: 8K, 32K, 128K, 512K, 2M, 8M        
        Z = X ⨀ Y
        Scaling Factor 1 (x4)
        row size: 8K
        data memory: 32KB-> Max 4 Row
        --- Iteration InputSize/8K -------
        LOAD:  8KB
        BARRIER 
        MAC: 8KB
        BARRIER
        V_REC
        BARRIER 
        WBD: 8KB
        ---
        Input: 8K
        BG0/ROW5000, BG0/ROW6000 -> BG0/ROW7000
        COL:0-127 or 0-63
        BG0-3/ROW5000, BG0-3/ROW6000 -> BG0/ROW7000
        COL:0-127 or 0-63                 
    '''

    iteration, num_working_bg, opsize = cal_it_v2(input_size, scaling)
    ndp_bk_idx = NDP_TARGET_BK

    # Input_size x 8 pch 
    print(f"Working Bank Group: {num_working_bg}")
    print(f"Iteration: {iteration}")
    # Max NDP Instruction 8KB/8B=1K
    for ch in range(int(NUM_CHANNEL)):
        for pch in range(int(NUM_PSEUDOCHANNEL)):
            ndp_inst_list = []
            jump_pc = len(ndp_inst_list)
            for bg in range(num_working_bg):
                ndp_inst_list.append(inst(ndp_inst_opcode["LOAD"],opsize,0,bg,ndp_bk_idx,0,0,0))
            for bg in range(num_working_bg):                
                ndp_inst_list.append(inst(ndp_inst_opcode["MUL"],opsize,1,bg,ndp_bk_idx,0,0,0))       
            for bg in range(num_working_bg):                
                ndp_inst_list.append(inst(ndp_inst_opcode["WBD"],opsize,2,bg,ndp_bk_idx,0,0,0))                      
            if iteration > 1:
                ndp_inst_list.append(inst(ndp_inst_opcode["LOOP"],0,0,0,0,(iteration-1),jump_pc,0))
            ndp_inst_list.append(inst(ndp_inst_opcode["EXIT"],0,0,0,0,0,0,0))
            if(len(ndp_inst_list) >= 1024):
                print("Error: Over NDP Instruction Memory")
                exit(1)
            dump_ndp_inst(f,ndp_inst_list,ch,pch)

    # Generate NDP Start Request 
    data_array = [0] * 8
    for ch in range(int(NUM_CHANNEL)):
        for pch in range(int(NUM_PSEUDOCHANNEL)):  
            idx = ch * 4 + pch     
            data_array[idx] = 1   
    
    write_trace(f,'ST',encode_address(0, 0, 0, HSNU_CTR_REG_BG, HSNU_CTR_REG_BK, NDP_ROW, 0),data_array)

    # Make 2-D NDL-Launch Request Inst
    acc_inst_list = [[ ]]
    acc_inst_list_num = [[ ]]
    for ch in range(int(NUM_CHANNEL)):
        for pch in range(int(NUM_PSEUDOCHANNEL)):    
            acc_inst_list.append([])
            acc_inst_list_num.append(0)
    # Start NDP ops (HSNU-Ctrl Access Info Buffer)
    # Type / Opsize/ Channel/ Pseudo-Channel/ BG/ BK / ROW/ COL/ ID / Reserved
    for ch in range(int(NUM_CHANNEL)):
        for pch in range(int(NUM_PSEUDOCHANNEL)):    
            idx = ch * int(NUM_PSEUDOCHANNEL) + pch
            for i in range(iteration):
                for bg in range(num_working_bg):
                    acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["RD"],opsize,ch,pch,bg,ndp_bk_idx,(5000+i),0,0,0))            
                acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["BAR"],0,0,0,0,0,0,0,0,0))
                for bg in range(num_working_bg):
                    acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["RD"],opsize,ch,pch,bg,ndp_bk_idx,(6000+i),0,1,0))               
                acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["BAR"],0,0,0,0,0,0,0,0,0))
                for bg in range(num_working_bg):
                    acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["WR"],opsize,ch,pch,bg,ndp_bk_idx,(7000+i),0,2,0))             
                acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["BAR"],0,0,0,0,0,0,0,0,0))
            acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["DONE"],0,0,0,0,0,0,0,0,0))

    # Reshape 2-D NDL-Lauch Request Lists to 1-D List
    if(len(acc_inst_list) >= 1024):
        print("Error: Over NDP Instruction Memory")
        exit(1)    
    dump_ndp_acc_inst_2d(f,acc_inst_list,0)
       
    return     

# DOT       : c = X·Y
def dot_pch(f, input_size, scaling):
    '''
        input_size: 8K, 32K, 128K, 512K, 2M, 8M        
        Z = X·Y
        Scaling Factor 1 (x4)
        row size: 8K
        data memory: 32KB-> Max 4 Row
        --- Iteration -------
        LOAD -> BARRIER -> MAC (Partial Sum; 2 x num_working_bg) -> BARRIER -> 
        SELF_EXEC_ON -> 
        (if num_working_bg > 1) T_V_RED x (2xnum_working_bg) -> T_ADD -> T_S_RED
        SELF_EXEC_OFF ->
        WBD 1 
        ---
        Input: 8K
        BG0/ROW5000, BG0/ROW6000 -> BG0/ROW7000
        COL:0-127 or 0-63
        BG0-3/ROW5000, BG0-3/ROW6000 -> BG0/ROW7000
        COL:0-127 or 0-63                 
    '''

    iteration, num_working_bg, opsize = cal_it_v2(input_size, scaling)
    ndp_bk_idx = NDP_TARGET_BK

    # Input_size x 8 pch 
    print(f"Working Bank Group: {num_working_bg}")
    print(f"Iteration: {iteration}")
    print(f"opsize: {opsize}")
    # Max NDP Instruction 8KB/8B=1K
    for ch in range(int(NUM_CHANNEL)):
        for pch in range(int(NUM_PSEUDOCHANNEL)):
            ndp_inst_list = []
            jump_pc = len(ndp_inst_list)
            for bg in range(num_working_bg):
                ndp_inst_list.append(inst(ndp_inst_opcode["LOAD"],opsize,0,bg,ndp_bk_idx,0,0,0))
            for bg in range(num_working_bg):                
                ndp_inst_list.append(inst(ndp_inst_opcode["MAC"],opsize,1,bg,ndp_bk_idx,0,0,0))    
            ndp_inst_list.append(inst(ndp_inst_opcode["SELF_EXEC_ON"],0,0,0,0,0,0,0))  
            if num_working_bg > 1:
                ndp_inst_list.append(inst(ndp_inst_opcode["T_V_RED"],(2*num_working_bg),0,0,0,0,0,0))
                ndp_inst_list.append(inst(ndp_inst_opcode["BARRIER"],0,0,0,0,0,0,0))  
            ndp_inst_list.append(inst(ndp_inst_opcode["T_ADD"],0,0,0,0,0,0,0))
            ndp_inst_list.append(inst(ndp_inst_opcode["BARRIER"],0,0,0,0,0,0,0))  
            ndp_inst_list.append(inst(ndp_inst_opcode["T_S_RED"],0,0,0,0,0,0,0))  
            ndp_inst_list.append(inst(ndp_inst_opcode["BARRIER"],0,0,0,0,0,0,0))   
            ndp_inst_list.append(inst(ndp_inst_opcode["SELF_EXEC_OFF"],0,0,0,0,0,0,0)) 
            ndp_inst_list.append(inst(ndp_inst_opcode["WBD"],0,2,0,ndp_bk_idx,0,0,0))                      
            if iteration > 1:
                ndp_inst_list.append(inst(ndp_inst_opcode["LOOP"],0,0,0,0,(iteration-1),jump_pc,0))
            ndp_inst_list.append(inst(ndp_inst_opcode["EXIT"],0,0,0,0,0,0,0))
            if(len(ndp_inst_list) >= 1024):
                print("Error: Over NDP Instruction Memory")
                exit(1)
            dump_ndp_inst(f,ndp_inst_list,ch,pch)

    # Generate NDP Start Request 
    data_array = [0] * 8
    for ch in range(int(NUM_CHANNEL)):
        for pch in range(int(NUM_PSEUDOCHANNEL)):  
            idx = ch * 4 + pch     
            data_array[idx] = 1   
    
    write_trace(f,'ST',encode_address(0, 0, 0, HSNU_CTR_REG_BG, HSNU_CTR_REG_BK, NDP_ROW, 0),data_array)

    # Make 2-D NDL-Launch Request Inst
    acc_inst_list = [[ ]]
    acc_inst_list_num = [[ ]]
    for ch in range(int(NUM_CHANNEL)):
        for pch in range(int(NUM_PSEUDOCHANNEL)):    
            acc_inst_list.append([])
            acc_inst_list_num.append(0)
    # Start NDP ops (HSNU-Ctrl Access Info Buffer)
    # Type / Opsize/ Channel/ Pseudo-Channel/ BG/ BK / ROW/ COL/ ID / Reserved
    for ch in range(int(NUM_CHANNEL)):
        for pch in range(int(NUM_PSEUDOCHANNEL)):    
            idx = ch * int(NUM_PSEUDOCHANNEL) + pch
            for i in range(iteration):
                # LOAD
                for bg in range(num_working_bg):
                    acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["RD"],opsize,ch,pch,bg,ndp_bk_idx,(5000+i),0,0,0))            
                acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["BAR"],0,0,0,0,0,0,0,0,0))
                # MAC
                for bg in range(num_working_bg):
                    acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["RD"],opsize,ch,pch,bg,ndp_bk_idx,(6000+i),0,1,0))               
                acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["BAR"],0,0,0,0,0,0,0,0,0))
                # for bg in range(num_working_bg):
                acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["WAIT"],0,0,0,0,0,0,0,0,100))
                acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["WR"],0,ch,pch,0,ndp_bk_idx,(7000+i),0,2,0))             
                acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["BAR"],0,0,0,0,0,0,0,0,0))
            acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["DONE"],0,0,0,0,0,0,0,0,0))

    # Reshape 2-D NDL-Lauch Request Lists to 1-D List
    if(len(acc_inst_list) >= 1024):
        print("Error: Over NDP Instruction Memory")
        exit(1)    
    dump_ndp_acc_inst_2d(f,acc_inst_list,0)
       
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
    
    # Write DRAM (Vector Data)
    n_wr_dram_row = int(vec_size/row_size)
    for ro in range(n_wr_dram_row):
        for co in range(int(NUM_COL)):
            for ch in range(int(NUM_CHANNEL)):
               for pch in range(int(NUM_PSEUDOCHANNEL)):   
                   for bg in range(n_row_p_vec):
                       write_normal_trace(f,'ST',encode_address(ch, pch, 0, bg, ndp_bk_idx, 1000 + ro, co))           
    
    jump_pc0 = 0
    jump_pc1 = 0
    self_before_pc = 0
    self_after_pc = 0
    fianl_pc = 0
    # NDP Instruction: Opcode, Opsize, ID, BG, BK, OP1, OP2, OP3
    for ch in range(int(NUM_CHANNEL)):
        for pch in range(int(NUM_PSEUDOCHANNEL)):
            ndp_inst_list = []
            jump_pc0 = len(ndp_inst_list)
            # Block-level GEMV
            # Load Partial Vector to Data Memory
            for bg in range(n_row_p_vec):
                ndp_inst_list.append(inst(ndp_inst_opcode["LOAD"],opsize,0,bg,ndp_bk_idx,0,0,0))
            # MAC Operation 
            for _ in range(n_tile_block_row):
                # Each Tile GEMV
                for _ in range(n_row_p_vec): 
                    for bg in range(n_bg):                
                        ndp_inst_list.append(inst(ndp_inst_opcode["MAC"],opsize,0,bg,ndp_bk_idx,0,0,0))
            # Column-wise gemv ops
            if col_tile > 1:
                jump_pc1 = len(ndp_inst_list)
                for bg in range(n_row_p_vec):
                    ndp_inst_list.append(inst(ndp_inst_opcode["LOAD"],opsize,0,bg,ndp_bk_idx,0,0,0))
                ndp_inst_list.append(inst(ndp_inst_opcode["BARRIER"],0,0,0,0,0,0,0))
                for _ in range(n_tile_block_row):
                    for _ in range(n_row_p_vec):
                        for bg in range(n_bg):                
                            ndp_inst_list.append(inst(ndp_inst_opcode["MAC"],opsize,0,bg,ndp_bk_idx,0,0,0))                
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
            # WBD
            ndp_inst_list.append(inst(ndp_inst_opcode["WBD"],opsize_wbd,0,0,ndp_bk_idx,0,0,0)) 

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
    # Generate NDP Start Request 
    data_array = [0] * 8
    for ch in range(int(NUM_CHANNEL)):
        for pch in range(int(NUM_PSEUDOCHANNEL)):  
            idx = ch * 4 + pch     
            data_array[idx] = 1   
    
    write_trace(f,'ST',encode_address(0, 0, 0, HSNU_CTR_REG_BG, HSNU_CTR_REG_BK, NDP_ROW, 0),data_array)

    # Make 2-D NDL-Launch Request Inst
    acc_inst_list = [[ ]]
    acc_inst_list_num = [[ ]]
    for ch in range(int(NUM_CHANNEL)):
        for pch in range(int(NUM_PSEUDOCHANNEL)):    
            acc_inst_list.append([])
            acc_inst_list_num.append(0)
    # Start NDP ops (HSNU-Ctrl Access Info Buffer)
    # Type / Opsize/ Channel/ Pseudo-Channel/ BG/ BK / ROW/ COL/ ID / Reserved
    for ch in range(int(NUM_CHANNEL)):
        for pch in range(int(NUM_PSEUDOCHANNEL)):    
            idx = ch * int(NUM_PSEUDOCHANNEL) + pch
            for i in range(iteration_tile_block):
                # Load Partial Vector
                for bg in range(n_row_p_vec):
                    acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["RD"],opsize,ch,pch,bg,ndp_bk_idx,(1000+col_tile*i),0,0,0))            
                acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["BAR"],0,0,0,0,0,0,0,0,0))
                # MAC Operation 
                for t_tile in range(n_tile_block_row):
                    # Each Tile GEMV
                    for n_row in range(n_row_p_vec):                 
                        for bg in range(n_bg):
                            acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["RD"],opsize,ch,pch,bg,ndp_bk_idx,(3000+i*n_tile_block_row*n_row_p_vec+t_tile*n_row_p_vec+n_row),0,0,0))               
                        acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["BAR"],0,0,0,0,0,0,0,0,0))
                if col_tile > 1:
                    for col_it in range(col_tile - 1):
                        # Load Partial Vector
                        for bg in range(n_row_p_vec):
                            acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["RD"],opsize,ch,pch,bg,ndp_bk_idx,(1000+col_tile*i),0,0,0))            
                        acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["BAR"],0,0,0,0,0,0,0,0,0))
                        # MAC Operation 
                        for t_tile in range(n_tile_block_row):
                            # Each Tile GEMV
                            for n_row in range(n_row_p_vec):                 
                                for bg in range(n_bg):
                                    acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["RD"],opsize,ch,pch,bg,ndp_bk_idx,(3000+i*n_tile_block_row*n_row_p_vec+t_tile*n_row_p_vec+n_row),0,0,0))               
                                acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["BAR"],0,0,0,0,0,0,0,0,0))       
                        # acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["BAR"],0,0,0,0,0,0,0,0,0))       
                             
                # Wait Self Execution (N-Cycle)
                # acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["BAR"],0,0,0,0,0,0,0,0,0))
                acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["WAIT"],0,0,0,0,0,0,0,0,self_exec_delay)) #self_exec_delay
                acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["WR"],opsize_wbd,ch,pch,0,ndp_bk_idx,(7000+i),0,0,0))             
                acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["BAR"],0,0,0,0,0,0,0,0,0))
            acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["DONE"],0,0,0,0,0,0,0,0,0))

    # Reshape 2-D NDL-Lauch Request Lists to 1-D List
    if(len(acc_inst_list) >= 1024):
        print("Warning: Over NDP Access Request Memory")
    dump_ndp_acc_inst_2d(f,acc_inst_list,0)
       
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
    num_rd = int(input_size_byte_list[input_size]/8192) * NORMAL_SCALE_FACTOR * NUM_CHANNEL * NUM_COL
    # src_X_addr = encode_normal_address(0, 0, 0, 0, 5000, 0)
    # src_Y_addr = encode_normal_address(0, 0, 0, 0, 6000, 0)
    # des_Z_addr = encode_normal_address(0, 0, 0, 0, 7000, 0)
    # NUM_CHANNEL
    # NUM_RANK
    # NUM_BANKGROUP
    # NUM_BANK
    # NUM_COL
    # encode_normal_address(channel, rank, bg, bank, row, col)

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
    num_rd = int(input_size_byte_list[input_size]/8192) * NORMAL_SCALE_FACTOR * NUM_CHANNEL * NUM_COL
    # src_X_addr = encode_normal_address(0, 0, 0, 0, 5000, 0)
    # src_B_addr = encode_normal_address(0, 0, 0, 0, 6000, 0)
    # src_Z_addr = encode_normal_address(0, 0, 0, 0, 7000, 0)
    # des_W_addr = encode_normal_address(0, 0, 0, 0, 8000, 0)

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
    num_rd = int(input_size_byte_list[input_size]/8192) * NORMAL_SCALE_FACTOR * NUM_CHANNEL * NUM_COL

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
    num_rd = int(input_size_byte_list[input_size]/8192) * NORMAL_SCALE_FACTOR * NUM_CHANNEL * NUM_COL

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
    num_rd = int(input_size_byte_list[input_size]/8192) * NORMAL_SCALE_FACTOR * NUM_CHANNEL * NUM_COL

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
    num_rd = int(input_size_byte_list[input_size]/8192) * NORMAL_SCALE_FACTOR * NUM_CHANNEL * NUM_COL

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

    gen_normal_req_from_row(f,1000,num_rd,'LD')
    gen_normal_req_from_row(f,2000,num_rd * num_row,'LD')
    gen_normal_req_from_row(f,1001,num_rd,'ST')         
    print(f"=======================================")
    print(f" GEMV Normal Mode - Size: {mat_input_size_byte_list[input_size]}")    
    print(f"  - RD Vecotr: {num_rd}")    
    print(f"  - RD Matrix: {num_rd * num_row}")    
    print(f"  - WR Vecotr: {num_rd}")    

def generate_trace(workload, size,output_path='',is_ndp_ops=True, scaling_factor=-1):
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

    if is_ndp_ops:
        if scaling_factor == 1:
            file_name = "ndp_4x_" +  file_name 
        elif scaling_factor == 2: 
            file_name = "ndp_8x_" +  file_name 
        elif scaling_factor == 4: 
            file_name = "ndp_16x_" +  file_name 
        else:
            print("Error: Wrong Input Size")
            exit(1)
    else: 
        file_name = "baseline_" + file_name
    
    # Set Address Mapping Scheme 
    config_scale_factor(scaling_factor)

    file_name = output_path + "/" + file_name
    # Size: "8K","32K","128K","512K","2M","8M" (GEMV: 8K, 16K, 32K, 64K, 128K)
    # workload: "AXPBY", "AXPBYPCZ", "AXPY", "COPY", "XMY", "DOT", "SCAL", "GEMV"
    with open(file_name, 'w') as f:      
        if is_ndp_ops:
            if workload == "AXPBY":
                axpby_pch(f,size,scaling_factor)
            elif workload == "AXPBYPCZ":
                axpbypcz_pch(f,size,scaling_factor)
            elif workload == "AXPY":
                axpy_pch(f,size,scaling_factor)          
            elif workload == "COPY":
                copy_pch(f,size,scaling_factor)          
            elif workload == "XMY":
                xmy_pch(f,size,scaling_factor)                                    
            elif workload == "DOT":
                dot_pch(f,size,scaling_factor)  
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

def crete_folder(folder_path):
    if not os.path.exists(folder_path):
        os.mkdir(folder_path)

if __name__ == '__main__':
    print(" ========================================================================  ")
    print(" Generate NDP Workload  ")    
    print(" ========================================================================  ")

    # make generated trace output path 
    ndp_trace_path = "trace/ndp"
    non_ndp_trace_path = "trace/non_ndp"

    shutil.rmtree("trace")
    crete_folder("trace")
    crete_folder(ndp_trace_path)
    crete_folder(non_ndp_trace_path)

    print(" - NDP Workload Path: ",ndp_trace_path)
    print(" - Non-NDP Workload Path: ",non_ndp_trace_path)

    # generate_trace("GEMV", mat_input_size_list[0],non_ndp_trace_path,is_ndp_ops=False, scaling_factor=1)

    for size in input_size_list:
        for bench in ["AXPBY", "AXPBYPCZ", "AXPY", "COPY", "XMY", "DOT"]:
            generate_trace(bench, size,non_ndp_trace_path, is_ndp_ops=False, scaling_factor=1)
            for scaling in [1, 2, 4]:
                generate_trace(bench, size,ndp_trace_path, is_ndp_ops=True, scaling_factor=scaling)

    for size in mat_input_size_list:
        generate_trace("GEMV", size,non_ndp_trace_path,is_ndp_ops=False, scaling_factor=1)
        for scaling in [1, 2, 4]:
            generate_trace("GEMV", size,ndp_trace_path,is_ndp_ops=True, scaling_factor=scaling)
        
    # for size in input_size_list:
    #     generate_trace("AXPBY", size,non_ndp_trace_path,is_ndp_ops=False)    
    #     generate_trace("AXPBYPCZ", size,non_ndp_trace_path,is_ndp_ops=False)        
