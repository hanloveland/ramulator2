
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
        input_size: 8K, 32K, 128K, 512K, 8M
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
        BG0/BG1 -> BG2
        ROW:5000
        COL:0-127 

        input: 32K
        BG0, BG1, BG2, BG2
        
    '''
    iteration = input_size_list[input_size]/8192
    data_array = [0] * 8
    for ch in range(2):
        for pch in range(4):
            data_array[0] = inst(ndp_inst_opcode["LOAD_MUL"],127,0,0,0)
            data_array[1] = inst(ndp_inst_opcode["SCALE_ADD"],127,0,1,0)
            # data_array[2] = inst(ndp_inst_opcode["SELF_EXEC_ON"],0,0,0,0)
            # data_array[3] = inst(ndp_inst_opcode["T_ADD"],127,0,0,0)
            # data_array[4] = inst(ndp_inst_opcode["SELF_EXEC_OFF"],0,0,0,0)
            data_array[2] = inst(ndp_inst_opcode["WBD"],127,0,2,0)
            data_array[4] = inst(ndp_inst_opcode["EXIT"],0,0,0,0)
            write_trace(f,'ST',encode_address(ch, pch, 0, NDP_INS_MEM_BG, NDP_INS_MEM_BK, NDP_ROW, 0),data_array)

    data_array = [0] * 8
    acc_inst_list = []
    # Start NDP ops (HSNU-Ctrl Access Info Buffer)
    # Type / Opsize/ Channel/ Pseudo-Channel/ BG/ BK / ROW/ COL/ ID / Reserved
    for ch in range(2):
        for pch in range(4):    
            acc_inst_list.append(acc_inst(ndp_acc_inst_opcode["RD"],127,ch,pch,0,0,5000,0,0,0))
    acc_inst_list.append(acc_inst(ndp_acc_inst_opcode["BAR"],0,0,0,0,0,0,0,0,0))
    for ch in range(2):
        for pch in range(4):    
            acc_inst_list.append(acc_inst(ndp_acc_inst_opcode["RD"],127,ch,pch,1,0,5000,0,0,0))    
    acc_inst_list.append(acc_inst(ndp_acc_inst_opcode["BAR"],0,0,0,0,0,0,0,0,0))
    for ch in range(2):
        for pch in range(4):    
            acc_inst_list.append(acc_inst(ndp_acc_inst_opcode["WR"],127,ch,pch,2,0,5000,0,0,0)) 
    acc_inst_list.append(acc_inst(ndp_acc_inst_opcode["DONE"],0,0,0,0,0,0,0,0,0))
    dump_ndp_acc_inst(f, acc_inst_list)


    # write_trace(f,'ST',encode_address(0, 0, 0, HSNU_CTR_BUF_BG, HSNU_CTR_BUF_BK, NDP_ROW, 0),data_array)
    # data_array[0] = acc_inst(ndp_acc_inst_opcode["BAR"],0,0,0,0,0,0,0,0,0)
    # data_array[1] = acc_inst(ndp_acc_inst_opcode["RD"],127,0,0,1,0,5000,0,0,0)
    # data_array[2] = acc_inst(ndp_acc_inst_opcode["RD"],127,0,1,1,0,5000,0,0,0)
    # data_array[3] = acc_inst(ndp_acc_inst_opcode["RD"],127,0,2,1,0,5000,0,0,0)
    # data_array[4] = acc_inst(ndp_acc_inst_opcode["RD"],127,0,3,1,0,5000,0,0,0)
    # data_array[5] = acc_inst(ndp_acc_inst_opcode["RD"],127,1,0,1,0,5000,0,0,0)
    # data_array[6] = acc_inst(ndp_acc_inst_opcode["RD"],127,1,1,1,0,5000,0,0,0)
    # data_array[7] = acc_inst(ndp_acc_inst_opcode["RD"],127,1,2,1,0,5000,0,0,0)
    # write_trace(f,'ST',encode_address(0, 0, 0, HSNU_CTR_BUF_BG, HSNU_CTR_BUF_BK, NDP_ROW, 1),data_array)
    # data_array[7] = acc_inst(ndp_acc_inst_opcode["RD"],127,1,3,1,0,5000,0,0,0)

    # data_array[2] = acc_inst(ndp_acc_inst_opcode["RD"],127,0,0,1,0,5000,0,0,0)
    # data_array[3] = acc_inst(ndp_acc_inst_opcode["BAR"],0,0,0,0,0,0,0,0,0)
    # data_array[4] = acc_inst(ndp_acc_inst_opcode["WR"],127,0,0,2,0,5000,0,0,0)
    # data_array[5] = acc_inst(ndp_acc_inst_opcode["DONE"],0,0,0,0,0,0,0,0,0)
    # data_array[6] = 0
    # data_array[7] = 0
    return 
# acc_inst(ndp_acc_inst_opcode["WAIT"],0,0,0,0,0,0,0,0,1152)


def generate_trace(filename, num_instructions=1000, seed=None):
    # if seed is not None:
    #     #random.seed(seed)

    with open(filename, 'w') as f:
        data_array = [0] * 8
        # Write NDP Instruction
        # for i in range(128):
        # col = i
        # random.choice(['LD', 'ST'])

        # 무작위 DRAM 주소 구성 요소 생성
        # instr_type = 'ST'
        # # Write Instruction Memory
        # channel        = 0
        # pseudo_channel = 0
        # rank           = 0
        # bg             = 4
        # bank           = 3
        # row            = 65535 
        # data_array[0] = inst(0,127,0,0,0)
        # data_array[1] = inst(48,0,0,0,0)
        # data_array[2] = inst(1,127,0,0,0) 
        # data_array[3] = inst(49,0,0,0,0) 
        # # encoding phyisical address and write memory access trace
        # write_trace(f,instr_type,encode_address(0, 0, rank, bg, bank, row, 0),data_array)
        # write_trace(f,instr_type,encode_address(0, 1, rank, bg, bank, row, 0),data_array)
        # write_trace(f,instr_type,encode_address(0, 2, rank, bg, bank, row, 0),data_array)
        # write_trace(f,instr_type,encode_address(0, 3, rank, bg, bank, row, 0),data_array)
        # write_trace(f,instr_type,encode_address(1, 0, rank, bg, bank, row, 0),data_array)
        # write_trace(f,instr_type,encode_address(1, 1, rank, bg, bank, row, 0),data_array)
        # write_trace(f,instr_type,encode_address(1, 2, rank, bg, bank, row, 0),data_array)
        # write_trace(f,instr_type,encode_address(1, 3, rank, bg, bank, row, 0),data_array)
        axpby_pch(f,"8K")
        # data_array = [0] * 8
        # Start NDP ops (HSNU-Ctrl Access Info Buffer)
        # data_array[0] = acc_inst(0,127,0,0,0,0,1000,0,0)
        # data_array[1] = acc_inst(0,127,0,1,0,0,1000,0,0)
        # data_array[2] = acc_inst(0,127,0,2,0,0,1000,0,0)
        # data_array[3] = acc_inst(0,127,0,3,0,0,1000,0,0)
        # data_array[4] = acc_inst(0,127,1,0,0,0,1000,0,0)
        # data_array[5] = acc_inst(0,127,1,1,0,0,1000,0,0)
        # data_array[6] = acc_inst(0,127,1,2,0,0,1000,0,0)
        # data_array[7] = acc_inst(0,127,1,3,0,0,1000,0,0)
        # Start NDP ops (HSNU-Ctrl Reg)
        # write_trace(f,'ST',encode_address(0, 0, 0, 6, 3, 65535, 0),data_array)
        # data_array = [0] * 8
        # Start NDP ops (HSNU-Ctrl Access Info Buffer)
        # data_array[0] = acc_inst(3,0,0,0,0,0,0,0,0)
        # data_array[1] = acc_inst(0,127,0,0,0,0,1001,0,0)
        # data_array[2] = acc_inst(0,127,0,1,0,0,1001,0,0)
        # data_array[3] = acc_inst(0,127,0,2,0,0,1001,0,0)
        # data_array[4] = acc_inst(0,127,0,3,0,0,1001,0,0)
        # data_array[5] = acc_inst(0,127,1,0,0,0,1001,0,0)
        # data_array[6] = acc_inst(0,127,1,1,0,0,1001,0,0)
        # data_array[7] = acc_inst(0,127,1,2,0,0,1001,0,0)
        # Start NDP ops (HSNU-Ctrl Reg)
        # write_trace(f,'ST',encode_address(0, 0, 0, 6, 3, 65535, 1),data_array)      

        # data_array = [0] * 8
        # Start NDP ops (HSNU-Ctrl Access Info Buffer)
        # data_array[0] = acc_inst(0,127,1,3,0,0,1001,0,0)
        # data_array[1] = acc_inst(15,0,0,0,0,0,0,0,0)
        # Start NDP ops (HSNU-Ctrl Reg)
        # write_trace(f,'ST',encode_address(0, 0, 0, 6, 3, 65535, 2),data_array) 

        # NDP Start 
        data_array[0] = 1
        # NDP pch mask (2channelx4pseduo_channel --> MAX 8 channel x 8 pseduo_channe)
        data_array[1] = 0xFF
        write_trace(f,'ST',encode_address(0, 0, 0, HSNU_CTR_REG_BG, HSNU_CTR_REG_BK, NDP_ROW, 0),data_array)

    print(f"Generated memory traces in '{filename}'.")

generate_trace("memory_trace.txt", num_instructions=64, seed=123)
