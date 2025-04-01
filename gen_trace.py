
CHANNEL_BITS    = 1    # 2 Channel
PCHANNEL_BITS   = 2    # 4 Pseudo Channel  
RANK_BITS       = 0    # 1 Rank
BANKGROUP_BITS  = 3    # 8 Bank Group  
BANK_BITS       = 2    # 4 Bank   
ROW_BITS        = 16   # 64K Row
COLUMN_BITS     = 11 - 4   # 2K Column (1KB/8B=128)
BURST_SIZE      = 64   # Prefetch Size 64
GRANULARITY     = 6    # 64B 

def encode_address(channel, pseudo_channel, rank, bg, bank, row, col):
    """
        ROCoBaBgRaPcCH
    """
    address = 0
    address |= (channel          & ((1 << CHANNEL_BITS) - 1))
    address |= (pseudo_channel   & ((1 << PCHANNEL_BITS) - 1))      << (CHANNEL_BITS)
    address |= (rank             & ((1 << RANK_BITS) - 1))          << (CHANNEL_BITS+PCHANNEL_BITS)
    address |= (bg               & ((1 << BANKGROUP_BITS) - 1))     << (CHANNEL_BITS+PCHANNEL_BITS+RANK_BITS)
    address |= (bank             & ((1 << BANK_BITS) - 1))          << (CHANNEL_BITS+PCHANNEL_BITS+RANK_BITS+BANKGROUP_BITS)
    address |= (col              & ((1 << COLUMN_BITS) - 1))        << (CHANNEL_BITS+PCHANNEL_BITS+RANK_BITS+BANKGROUP_BITS+BANK_BITS)
    address |= (row              & ((1 << ROW_BITS) - 1))           << (CHANNEL_BITS+PCHANNEL_BITS+RANK_BITS+BANKGROUP_BITS+BANK_BITS+COLUMN_BITS)
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
def acc_inst(opcode,opsize,ch,pch,bg,bk,row,col,id):
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
    return inst_64bit

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
        instr_type = 'ST'
        # Write Instruction Memory
        channel        = 0
        pseudo_channel = 0
        rank           = 0
        bg             = 4
        bank           = 3
        row            = 65535 
        data_array[0] = inst(0,127,0,0,0)
        data_array[1] = inst(48,0,0,0,0)
        data_array[2] = inst(1,127,0,0,0) 
        data_array[3] = inst(49,0,0,0,0) 
        # encoding phyisical address and write memory access trace
        write_trace(f,instr_type,encode_address(0, 0, rank, bg, bank, row, 0),data_array)
        write_trace(f,instr_type,encode_address(0, 1, rank, bg, bank, row, 0),data_array)
        write_trace(f,instr_type,encode_address(0, 2, rank, bg, bank, row, 0),data_array)
        write_trace(f,instr_type,encode_address(0, 3, rank, bg, bank, row, 0),data_array)
        write_trace(f,instr_type,encode_address(1, 0, rank, bg, bank, row, 0),data_array)
        write_trace(f,instr_type,encode_address(1, 1, rank, bg, bank, row, 0),data_array)
        write_trace(f,instr_type,encode_address(1, 2, rank, bg, bank, row, 0),data_array)
        write_trace(f,instr_type,encode_address(1, 3, rank, bg, bank, row, 0),data_array)
        data_array = [0] * 8
        # Start NDP ops (HSNU-Ctrl Access Info Buffer)
        data_array[0] = acc_inst(0,127,0,0,0,0,1000,0,0)
        data_array[1] = acc_inst(0,127,0,1,0,0,1000,0,0)
        data_array[2] = acc_inst(0,127,0,2,0,0,1000,0,0)
        data_array[3] = acc_inst(0,127,0,3,0,0,1000,0,0)
        data_array[4] = acc_inst(0,127,1,0,0,0,1000,0,0)
        data_array[5] = acc_inst(0,127,1,1,0,0,1000,0,0)
        data_array[6] = acc_inst(0,127,1,2,0,0,1000,0,0)
        data_array[7] = acc_inst(0,127,1,3,0,0,1000,0,0)
        # Start NDP ops (HSNU-Ctrl Reg)
        write_trace(f,'ST',encode_address(0, 0, 0, 6, 3, 65535, 0),data_array)
        data_array = [0] * 8
        # Start NDP ops (HSNU-Ctrl Access Info Buffer)
        data_array[0] = acc_inst(3,0,0,0,0,0,0,0,0)
        data_array[1] = acc_inst(0,127,0,0,0,0,1001,0,0)
        data_array[2] = acc_inst(0,127,0,1,0,0,1001,0,0)
        data_array[3] = acc_inst(0,127,0,2,0,0,1001,0,0)
        data_array[4] = acc_inst(0,127,0,3,0,0,1001,0,0)
        data_array[5] = acc_inst(0,127,1,0,0,0,1001,0,0)
        data_array[6] = acc_inst(0,127,1,1,0,0,1001,0,0)
        data_array[7] = acc_inst(0,127,1,2,0,0,1001,0,0)
        # Start NDP ops (HSNU-Ctrl Reg)
        write_trace(f,'ST',encode_address(0, 0, 0, 6, 3, 65535, 1),data_array)      

        data_array = [0] * 8
        # Start NDP ops (HSNU-Ctrl Access Info Buffer)
        data_array[0] = acc_inst(0,127,1,3,0,0,1001,0,0)
        data_array[1] = acc_inst(15,0,0,0,0,0,0,0,0)
        # Start NDP ops (HSNU-Ctrl Reg)
        write_trace(f,'ST',encode_address(0, 0, 0, 6, 3, 65535, 2),data_array) 

        # NDP Start 
        data_array[0] = 1
        # NDP pch mask (2channelx4pseduo_channel --> MAX 8 channel x 8 pseduo_channe)
        data_array[1] = 0xFF
        write_trace(f,'ST',encode_address(0, 0, 0, 7, 3, 65535, 0),data_array)


    print(f"Generated memory traces in '{filename}'.")

generate_trace("memory_trace.txt", num_instructions=64, seed=123)
