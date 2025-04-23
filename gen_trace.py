
import math
import os
import shutil

NUM_CHANNEL = 2
NUM_PSEUDOCHANNEL = 4
NUM_RANK = 4
NUM_BANKGROUP = 8
NUM_BANK = 4
NUM_ROW = 65536
NUM_COL = 128 # 1KB/8B

CHANNEL_BITS    = int(math.log2(NUM_CHANNEL))                 # 2 Channel
PCHANNEL_BITS   = int(math.log2(NUM_PSEUDOCHANNEL))           # 4 Pseudo Channel  
RANK_BITS       = 0                                           # 1 Rank
BANKGROUP_BITS  = int(math.log2(NUM_BANKGROUP))               # 8 Bank Group  
BANK_BITS       = int(math.log2(NUM_BANK))                    # 4 Bank   
ROW_BITS        = int(math.log2(NUM_ROW))                     # 64K Row
COLUMN_BITS     = int(math.log2(NUM_COL))                     # 11 - 4 # 2K Column (1KB/8B=128)
BURST_SIZE      = 64                                          # Prefetch Size 64
GRANULARITY     = int(math.log2(BURST_SIZE))                  # 64B 

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
NDP_CTRL_BK     = 3
NDP_CTRL_BG     = 5
NDP_INS_MEM_BK  = 3
NDP_INS_MEM_BG  = 4
NDP_DAT_MEM_BK  = 3
NDP_DAT_MEM_BG  = 3
HSNU_CTR_REG_BK = 3
HSNU_CTR_REG_BG = 7
HSNU_CTR_BUF_BK = 3
HSNU_CTR_BUF_BG = 6
PCH_ADDRESS_SCHEME = "RoCoBaBgRaPcCH"
# PCH_ADDRESS_SCHEME = "RoBaBgRaCoPcCH"
NORMAL_ADDRESS_SCHEME = "RoCoBaRaCh"
# NORMAL_ADDRESS_SCHEME = "RoBaRaCoCh"
# NORMAL_ADDRESS_SCHEME = "RoRaCoBaCh"

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
    "64K":  65536,
    "128K": 131072,
    "512K": 524288,
    "8M": 8388608
}

input_size_list = [
    "8K",
    "32K",
    "64K",
    "128K",
    "512K",
    "8M"
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
    if PCH_ADDRESS_SCHEME == "RoBaBgRaCoPcCH":
        address |= (channel          & ((1 << CHANNEL_BITS) - 1))
        address |= (pseudo_channel   & ((1 << PCHANNEL_BITS) - 1))      << (CHANNEL_BITS)
        address |= (col              & ((1 << COLUMN_BITS) - 1))        << (CHANNEL_BITS+PCHANNEL_BITS)
        address |= (rank             & ((1 << RANK_BITS) - 1))          << (CHANNEL_BITS+PCHANNEL_BITS+COLUMN_BITS)
        address |= (bg               & ((1 << BANKGROUP_BITS) - 1))     << (CHANNEL_BITS+PCHANNEL_BITS+COLUMN_BITS+RANK_BITS)
        address |= (bank             & ((1 << BANK_BITS) - 1))          << (CHANNEL_BITS+PCHANNEL_BITS+COLUMN_BITS+RANK_BITS+BANKGROUP_BITS)
        address |= (row              & ((1 << ROW_BITS) - 1))           << (CHANNEL_BITS+PCHANNEL_BITS+COLUMN_BITS+RANK_BITS+BANKGROUP_BITS+BANK_BITS)
        # Shift by DRAM Acccess Granularity (64B)
        address = address << GRANULARITY
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
    if NORMAL_ADDRESS_SCHEME == "RoBaRaCoCh":
        address |= (channel          & ((1 << NORMAL_CHANNEL_BITS) - 1))
        address |= (col              & ((1 << NORMAL_COLUMN_BITS) - 1))    << (NORMAL_CHANNEL_BITS)
        address |= (rank             & ((1 << NORMAL_RANK_BITS) - 1))      << (NORMAL_CHANNEL_BITS+NORMAL_COLUMN_BITS)
        address |= (bg               & ((1 << NORMAL_BANKGROUP_BITS) - 1)) << (NORMAL_CHANNEL_BITS+NORMAL_COLUMN_BITS+NORMAL_RANK_BITS)
        address |= (bank             & ((1 << NORMAL_BANK_BITS) - 1))      << (NORMAL_CHANNEL_BITS+NORMAL_COLUMN_BITS+NORMAL_RANK_BITS+NORMAL_BANKGROUP_BITS)
        address |= (row              & ((1 << NORMAL_ROW_BITS) - 1))       << (NORMAL_CHANNEL_BITS+NORMAL_COLUMN_BITS+NORMAL_RANK_BITS+NORMAL_BANKGROUP_BITS+NORMAL_BANK_BITS)
        # Shift by DRAM Acccess Granularity (64B)
        address = address << GRANULARITY
    if NORMAL_ADDRESS_SCHEME == "RoRaCoBaCh":
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

def inst(opcode,opsize,id,bg,bk,op1,op2,op3):
    inst_64bit = 0
    inst_64bit |= (opcode & 0x3f) << 58
    inst_64bit |= (opsize & 0x7f) << 51
    inst_64bit |= (id & 0x7) << 48
    inst_64bit |= (bg & 0x7) << 45
    inst_64bit |= (bk & 0x3) << 43
    if opcode == ndp_inst_opcode["LOOP"]:
        inst_64bit |= (op1 & 0x3FF) << 16
        inst_64bit |= (op2 & 0x1FF)     
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
        for ch in range(2):
            for pch in range(4):  
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
                        print("Warning - Over nl-req buf cap.")


        if none_acc_inst:
            all_acc_inst_empty = True


def dump_ndp_inst(f, inst_list, ch, pch):
    # Generation Write Request for NDP Access Instruction
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

def axpby_pch(f, input_size):
    '''
        input_size: 8K, 32K, 64K, 128K, 512K, 8M
        data memory: 32KB-> Max 4 Row
        Z = aX + bY
        --- Iteration InputSize/8K -------
        LOAD_MUL: 8KB
        LOAD_MUL: 8KB
        BARRIER (SELF_EXEC_ON)
        T_ADD: 8KB
        BARRIER (SELF_EXEC_OFF)
        WBD: 8KB
        ---
        Input: 8K
        BG0/ROW5000, BG0/ROW6000 -> BG0/ROW7000
        COL:0-127 
        BG0-3/ROW5000, BG0-3/ROW6000 -> BG0/ROW7000
        COL:0-127         
        
    '''
    num_row = int(input_size_byte_list[input_size]/8192)
    if num_row > 4:
        num_working_bg = 4
        iteration = int(num_row / 4)
    else:
        num_working_bg = num_row
        iteration      = 1

    # Input_size x 8 pch 
    print(f"Working Bank Group: {num_working_bg}")
    print(f"Iteration: {iteration}")
    # Max NDP Instruction 8KB/8B=1K
    for ch in range(2):
        for pch in range(4):
            ndp_inst_list = []
            jump_pc = len(ndp_inst_list)
            for bg in range(num_working_bg):
                ndp_inst_list.append(inst(ndp_inst_opcode["LOAD_MUL"],127,0,bg,0,0,0,0))
            for bg in range(num_working_bg):                
                ndp_inst_list.append(inst(ndp_inst_opcode["SCALE_ADD"],127,1,bg,0,0,0,0))       
            for bg in range(num_working_bg):                
                ndp_inst_list.append(inst(ndp_inst_opcode["WBD"],127,2,bg,0,0,0,0))                      
            if iteration > 1:
                ndp_inst_list.append(inst(ndp_inst_opcode["LOOP"],0,0,0,0,iteration,jump_pc,0))
            ndp_inst_list.append(inst(ndp_inst_opcode["EXIT"],0,0,0,0,0,0,0))
            if(len(ndp_inst_list) >= 1024):
                print("Error: Over NDP Instruction Memory")
                exit(1)
            dump_ndp_inst(f,ndp_inst_list,ch,pch)
            # data_array[2] = inst(ndp_inst_opcode["SELF_EXEC_ON"],0,0,0,0)
            # data_array[3] = inst(ndp_inst_opcode["T_ADD"],127,0,0,0)
            # data_array[4] = inst(ndp_inst_opcode["SELF_EXEC_OFF"],0,0,0,0)
            # write_trace(f,'ST',encode_address(ch, pch, 0, NDP_INS_MEM_BG, NDP_INS_MEM_BK, NDP_ROW, 0),data_array)

    # Generate NDP Start Request 
    data_array = [0] * 8
    for ch in range(2):
        for pch in range(4):  
            idx = ch * 4 + pch     
            data_array[idx] = 1   
    
    write_trace(f,'ST',encode_address(0, 0, 0, HSNU_CTR_REG_BG, HSNU_CTR_REG_BK, NDP_ROW, 0),data_array)

    # Make 2-D NDL-Launch Request Inst
    acc_inst_list = [[ ]]
    acc_inst_list_num = [[ ]]
    for ch in range(2):
        for pch in range(4):    
            acc_inst_list.append([])
            acc_inst_list_num.append(0)
    # Start NDP ops (HSNU-Ctrl Access Info Buffer)
    # Type / Opsize/ Channel/ Pseudo-Channel/ BG/ BK / ROW/ COL/ ID / Reserved
    for ch in range(2):
        for pch in range(4):    
            idx = ch * 4 + pch
            for i in range(iteration):
                for bg in range(num_working_bg):
                    acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["RD"],127,ch,pch,bg,0,(5000+i),0,0,0))            
                acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["BAR"],0,0,0,0,0,0,0,0,0))
                for bg in range(num_working_bg):
                    acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["RD"],127,ch,pch,bg,0,(6000+i),0,1,0))               
                acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["BAR"],0,0,0,0,0,0,0,0,0))
                for bg in range(num_working_bg):
                    acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["WR"],127,ch,pch,bg,0,(7000+i),0,2,0))             
                acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["BAR"],0,0,0,0,0,0,0,0,0))
            acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["DONE"],0,0,0,0,0,0,0,0,0))

    # Reshape 2-D NDL-Lauch Request Lists to 1-D List
    if(len(acc_inst_list) >= 1024):
        print("Error: Over NDP Instruction Memory")
        exit(1)    
    dump_ndp_acc_inst_2d(f,acc_inst_list,0)
       
    return 

# AXPBYPCZ  : W = aX + bB + zZ
def axpbypcz_pch(f, input_size):
    '''
        input_size: 8K, 32K, 64K, 128K, 512K, 8M
        data memory: 32KB-> Max 4 Row
        Z = W = aX + bB + zZ
        --- Iteration InputSize/8K -------
        LOAD_MUL: 8KB
        SCALE_ADD: 8KB
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
    num_row = int(input_size_byte_list[input_size]/8192)
    if num_row > 8:
        num_working_bg = 8
        iteration = int(num_row / 8)
    else:
        num_working_bg = num_row
        iteration      = 1

    # Input_size x 8 pch 
    print(f"Working Bank Group: {num_working_bg}")
    print(f"Iteration: {iteration}")
    # Max NDP Instruction 8KB/8B=1K
    for ch in range(2):
        for pch in range(4):
            ndp_inst_list = []
            jump_pc = len(ndp_inst_list)
            if num_working_bg > 4:
                for bg in range(4):
                    ndp_inst_list.append(inst(ndp_inst_opcode["LOAD_MUL"],127,0,bg,0,0,0,0))
                for bg in range(4):                
                    ndp_inst_list.append(inst(ndp_inst_opcode["SCALE_ADD"],127,1,bg,0,0,0,0))       
                for bg in range(4):                
                    ndp_inst_list.append(inst(ndp_inst_opcode["SCALE_ADD"],127,2,bg,0,0,0,0))       
                for bg in range(4):          
                    ndp_inst_list.append(inst(ndp_inst_opcode["WBD"],127,3,bg,0,0,0,0))        
                for bg in range(4):
                    ndp_inst_list.append(inst(ndp_inst_opcode["LOAD_MUL"],127,0,(4+bg),0,0,0,0))
                for bg in range(4):                
                    ndp_inst_list.append(inst(ndp_inst_opcode["SCALE_ADD"],127,1,(4+bg),0,0,0,0))       
                for bg in range(4):                
                    ndp_inst_list.append(inst(ndp_inst_opcode["SCALE_ADD"],127,2,(4+bg),0,0,0,0))       
                for bg in range(4):          
                    ndp_inst_list.append(inst(ndp_inst_opcode["WBD"],127,3,(4+bg),0,0,0,0))                                   
            else: 
                for bg in range(num_working_bg):
                    ndp_inst_list.append(inst(ndp_inst_opcode["LOAD_MUL"],127,0,bg,0,0,0,0))       
                for bg in range(num_working_bg):                
                    ndp_inst_list.append(inst(ndp_inst_opcode["SCALE_ADD"],127,1,bg,0,0,0,0))               
                for bg in range(num_working_bg):                
                    ndp_inst_list.append(inst(ndp_inst_opcode["SCALE_ADD"],127,2,bg,0,0,0,0))   
                for bg in range(num_working_bg):
                    ndp_inst_list.append(inst(ndp_inst_opcode["WBD"],127,3,bg,0,0,0,0))   
            if iteration > 1:
                ndp_inst_list.append(inst(ndp_inst_opcode["LOOP"],0,0,0,0,iteration,jump_pc,0))
            ndp_inst_list.append(inst(ndp_inst_opcode["EXIT"],0,0,0,0,0,0,0))
            if(len(ndp_inst_list) >= 1024):
                print("Error: Over NDP Instruction Memory")
                exit(1)
            dump_ndp_inst(f,ndp_inst_list,ch,pch)
            # data_array[2] = inst(ndp_inst_opcode["SELF_EXEC_ON"],0,0,0,0)
            # data_array[3] = inst(ndp_inst_opcode["T_ADD"],127,0,0,0)
            # data_array[4] = inst(ndp_inst_opcode["SELF_EXEC_OFF"],0,0,0,0)
            # write_trace(f,'ST',encode_address(ch, pch, 0, NDP_INS_MEM_BG, NDP_INS_MEM_BK, NDP_ROW, 0),data_array)

    # Generate NDP Start Request 
    data_array = [0] * 8
    for ch in range(2):
        for pch in range(4):  
            idx = ch * 4 + pch     
            data_array[idx] = 1   
    
    write_trace(f,'ST',encode_address(0, 0, 0, HSNU_CTR_REG_BG, HSNU_CTR_REG_BK, NDP_ROW, 0),data_array)

    # Make 2-D NDL-Launch Request Inst
    acc_inst_list = [[ ]]
    acc_inst_list_num = [[ ]]
    for ch in range(2):
        for pch in range(4):    
            acc_inst_list.append([])
            acc_inst_list_num.append(0)
    # Start NDP ops (HSNU-Ctrl Access Info Buffer)
    # Type / Opsize/ Channel/ Pseudo-Channel/ BG/ BK / ROW/ COL/ ID / Reserved
    for ch in range(2):
        for pch in range(4):    
            idx = ch * 4 + pch
            for i in range(iteration):
                if(num_working_bg>4):
                    for bg in range(4):
                        acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["RD"],127,ch,pch,bg,0,(5000+i),0,0,0))            
                    acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["BAR"],0,0,0,0,0,0,0,0,0))
                    for bg in range(4):
                        acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["RD"],127,ch,pch,bg,0,(6000+i),0,1,0))               
                    acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["BAR"],0,0,0,0,0,0,0,0,0))
                    for bg in range(4):
                        acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["RD"],127,ch,pch,bg,0,(7000+i),0,2,0))               
                    acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["BAR"],0,0,0,0,0,0,0,0,0))                    
                    for bg in range(4):
                        acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["WR"],127,ch,pch,bg,0,(8000+i),0,3,0))             
                    acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["BAR"],0,0,0,0,0,0,0,0,0))
                    for bg in range(4):
                        acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["RD"],127,ch,pch,(4+bg),0,(5000+i),0,0,0))            
                    acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["BAR"],0,0,0,0,0,0,0,0,0))
                    for bg in range(4):
                        acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["RD"],127,ch,pch,(4+bg),0,(6000+i),0,1,0))               
                    acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["BAR"],0,0,0,0,0,0,0,0,0))
                    for bg in range(4):
                        acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["RD"],127,ch,pch,(4+bg),0,(7000+i),0,2,0))               
                    acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["BAR"],0,0,0,0,0,0,0,0,0))                    
                    for bg in range(4):
                        acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["WR"],127,ch,pch,(4+bg),0,(8000+i),0,3,0))             
                    acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["BAR"],0,0,0,0,0,0,0,0,0))                    
                else: 
                    for bg in range(num_working_bg):
                        acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["RD"],127,ch,pch,bg,0,(5000+i),0,0,0))            
                    acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["BAR"],0,0,0,0,0,0,0,0,0))
                    for bg in range(num_working_bg):
                        acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["RD"],127,ch,pch,bg,0,(6000+i),0,1,0))               
                    acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["BAR"],0,0,0,0,0,0,0,0,0))
                    for bg in range(num_working_bg):
                        acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["RD"],127,ch,pch,bg,0,(7000+i),0,2,0))               
                    acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["BAR"],0,0,0,0,0,0,0,0,0))                    
                    for bg in range(num_working_bg):
                        acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["WR"],127,ch,pch,bg,0,(8000+i),0,3,0))             
                    acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["BAR"],0,0,0,0,0,0,0,0,0))                    
            acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["DONE"],0,0,0,0,0,0,0,0,0))

    # Reshape 2-D NDL-Lauch Request Lists to 1-D List
    if(len(acc_inst_list) >= 1024):
        print("Error: Over NDP Instruction Memory")
        exit(1)    
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
    # cnt_req = 0
    # done = 0
    # for ro in range(NUM_ROW):
    #     for rk in range(NUM_RANK):        
    #         for ba in range(NUM_BANK):
    #             for co in range(NUM_COL): 
    #                 for bg in range(NUM_BANKGROUP):
    #                     for ch in range(NUM_CHANNEL):
    #                         write_normal_trace(f,'LD',encode_normal_address(ch, rk, bg, ba, 5000+ro, co))
    #                         cnt_req+=1
    #                         if cnt_req == num_rd:
    #                             done = 1
                            
    #                         if done == 1:
    #                             break
    #                     if done == 1:
    #                         break    
    #                 if done == 1:
    #                     break 
    #             if done == 1:
    #                 break
    #         if done == 1:
    #             break
    #     if done == 1:
    #         break    
            
    # for _ in range(num_rd):
    #     write_normal_trace(f,'LD',src_X_addr)
    #     write_normal_trace(f,'LD',src_Y_addr)
    #     src_X_addr+=BURST_SIZE
    #     src_Y_addr+=BURST_SIZE
    # for _ in range(num_rd):
    #     write_normal_trace(f,'ST',des_Z_addr)        
    #     des_Z_addr+=BURST_SIZE

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

    # for _ in range(num_rd):
    #     write_normal_trace(f,'LD',src_X_addr)
    #     write_normal_trace(f,'LD',src_B_addr)
    #     write_normal_trace(f,'LD',src_Z_addr)
    #     src_X_addr+=BURST_SIZE
    #     src_B_addr+=BURST_SIZE
    #     src_Z_addr+=BURST_SIZE
    # for _ in range(num_rd):
    #     write_normal_trace(f,'ST',des_W_addr)        
    #     des_W_addr+=BURST_SIZE    

def generate_trace(workload, size,output_path='',is_ndp_ops=True):
    # if seed is not None:
    #     #random.seed(seed)

    file_name = "none.txt"
    if workload in workload_list:
        file_name = workload + ".txt"
    else:
        print("Error: Not Valid Workload")
        exit(1)

    if size in input_size_list:
        file_name = size + "_" + file_name
    else:
        print("Error: Wrong Input Size")
        exit(1)

    if is_ndp_ops:
        file_name = "ndp_" + file_name
    else: 
        file_name = "non_ndp_" + file_name
    
    file_name = output_path + "/" + file_name
    # Size: 8K, 32K, 64K, 128K, 512K, 8M
    # workload: "AXPBY", "AXPBYPCZ", "AXPY", "COPY", "XMY", "DOT", "SCAL", "GEMV"
    with open(file_name, 'w') as f:      
        if is_ndp_ops:
            if workload == "AXPBY":
                axpby_pch(f,size)
            elif workload == "AXPBYPCZ":
                axpbypcz_pch(f,size)
            else:
                print("Error: Not Implemented Workload")
                exit(1)
        else:
            if workload == "AXPBY":
                axpby_normal(f,size)
            elif workload == "AXPBYPCZ":
                axpbypcz_normal(f,size)
            else:
                print("Error: Not Implemented Workload")
                exit(1)

    print(f"Generated memory traces in '{file_name}'.")

def crete_folder(folder_path):
    if not os.path.exists(folder_path):
        os.mkdir(folder_path)

if __name__ == '__main__':
    print(" ========================================================================  ")
    print(" Generate NDP Workload  ")    
    print(" ========================================================================  ")


    # generate_trace("AXPBYPCZ", "512K","trace/ndp",is_ndp_ops=True)
    # exit(1)
    # make generated trace output path 
    ndp_trace_path = "trace/ndp"
    non_ndp_trace_path = "trace/non_ndp"

    shutil.rmtree("trace")
    crete_folder("trace")
    crete_folder(ndp_trace_path)
    crete_folder(non_ndp_trace_path)

    print(" - NDP Workload Path: ",ndp_trace_path)
    print(" - Non-NDP Workload Path: ",non_ndp_trace_path)

    for size in input_size_list:
        generate_trace("AXPBY", size,ndp_trace_path,is_ndp_ops=True)
        generate_trace("AXPBYPCZ", size,ndp_trace_path,is_ndp_ops=True)

    for size in input_size_list:
        generate_trace("AXPBY", size,non_ndp_trace_path,is_ndp_ops=False)    
        generate_trace("AXPBYPCZ", size,non_ndp_trace_path,is_ndp_ops=False)        
