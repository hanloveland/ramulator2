
CHANNEL_BITS    = 1    # 2 Channel
PCHANNEL_BITS   = 2    # 4 Pseudo Channel  
RANK_BITS       = 0    # 1 Rank
BANKGROUP_BITS  = 3    # 8 Bank Group  
BANK_BITS       = 2    # 4 Bank   
ROW_BITS        = 16   # 64K Row
COLUMN_BITS     = 11 - 4   # 2K Column (1KB/8B=128)
BURST_SIZE      = 64   # Prefetch Size 64
GRANULARITY     = 6    # 64B 
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
    "SELF_EXEC_OFF"  :51
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

input_size_list = {
    "8K" :  8192,
    "32K":  32768,
    "64K":  65536,
    "128K": 131072,
    "512K": 524288,
    "8M": 8388608
}

def encode_address(channel, pseudo_channel, rank, bg, bank, row, col):
    """
        ROCoBaBgRaPcCH
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

def write_trace(f, instr_type, address, data_array):
    if instr_type == 'ST':
        f.write(f"{instr_type} 0x{address:016X} 0x{data_array[0]:016X} 0x{data_array[1]:016X} 0x{data_array[2]:016X} 0x{data_array[3]:016X} 0x{data_array[4]:016X} 0x{data_array[5]:016X} 0x{data_array[6]:016X} 0x{data_array[7]:016X}\n")
    else:
        f.write(f"{instr_type} 0x{address:016X}\n")

def inst(opcode,opsize,id,bg,bk):
    inst_64bit = 0
    inst_64bit |= (opcode & 0x3f) << 58
    inst_64bit |= (opsize & 0x7f) << 51
    inst_64bit |= (id & 0x7) << 48
    inst_64bit |= (bg & 0x7) << 45
    inst_64bit |= (bk & 0x3) << 43
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
        data memory: 32KB
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
    iteration = int(input_size_list[input_size]/8192)
    if iteration > 8:
        num_working_bg = 8
    else:
        num_working_bg = iteration
    print(num_working_bg)
    ndp_inst_list = []
    for ch in range(2):
        for pch in range(4):
            for bg in range(num_working_bg):
                ndp_inst_list.append(inst(ndp_inst_opcode["LOAD_MUL"],127,0,bg,0))
            for bg in range(num_working_bg):                
                ndp_inst_list.append(inst(ndp_inst_opcode["SCALE_ADD"],127,1,bg,0))       
            # data_array[2] = inst(ndp_inst_opcode["SELF_EXEC_ON"],0,0,0,0)
            # data_array[3] = inst(ndp_inst_opcode["T_ADD"],127,0,0,0)
            # data_array[4] = inst(ndp_inst_opcode["SELF_EXEC_OFF"],0,0,0,0)
            for bg in range(num_working_bg):                
                ndp_inst_list.append(inst(ndp_inst_opcode["WBD"],127,2,bg,0))                      
            ndp_inst_list.append(inst(ndp_inst_opcode["EXIT"],0,0,0,0))
            dump_ndp_inst(f,ndp_inst_list,ch,pch)
            # write_trace(f,'ST',encode_address(ch, pch, 0, NDP_INS_MEM_BG, NDP_INS_MEM_BK, NDP_ROW, 0),data_array)

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
            for bg in range(num_working_bg):
                acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["RD"],127,ch,pch,bg,0,5000,0,0,0))            
            acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["BAR"],0,0,0,0,0,0,0,0,0))
            for bg in range(num_working_bg):
                acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["RD"],127,ch,pch,bg,0,6000,0,1,0))               
            acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["BAR"],0,0,0,0,0,0,0,0,0))
            for bg in range(num_working_bg):
                acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["WR"],127,ch,pch,bg,0,7000,0,2,0))             
            acc_inst_list[idx].append(acc_inst(ndp_acc_inst_opcode["DONE"],0,0,0,0,0,0,0,0,0))

    # Reshape 2-D NDL-Lauch Request Lists to 1-D List
    dump_ndp_acc_inst_2d(f,acc_inst_list,0)
       
    # deprecated..
    # dump_ndp_acc_inst(f, acc_inst_list)
    return 
# acc_inst(ndp_acc_inst_opcode["WAIT"],0,0,0,0,0,0,0,0,1152)


def generate_trace(filename, num_instructions=1000, seed=None):
    # if seed is not None:
    #     #random.seed(seed)

    with open(filename, 'w') as f:
        data_array = [0] * 8

        axpby_pch(f,"64K")

        # NDP Start (only Ch0PCH0)
        for ch in range(2):
            for pch in range(4):  
                idx = ch * 4 + pch     
                data_array[idx] = 1   
        
        write_trace(f,'ST',encode_address(0, 0, 0, HSNU_CTR_REG_BG, HSNU_CTR_REG_BK, NDP_ROW, 0),data_array)

    print(f"Generated memory traces in '{filename}'.")

generate_trace("memory_trace.txt", num_instructions=64, seed=123)
