
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

def generate_trace(filename, num_instructions=1000, seed=None):
    # if seed is not None:
    #     #random.seed(seed)

    with open(filename, 'w') as f:
        col = 0
        for _ in range(num_instructions):
            instr_type = 'ST'
            # random.choice(['LD', 'ST'])

            # 무작위 DRAM 주소 구성 요소 생성
            channel        = 0#random.randint(0, (1 << CHANNEL_BITS) - 1)
            pseudo_channel = 0
            rank           = 0#random.randint(0, (1 << RANK_BITS) - 1)
            bg             = 6#random.randint(0, (1 << BANKGROUP_BITS) - 1)
            bank           = 2#random.randint(0, (1 << BANK_BITS) - 1)
            row            = 65535 #random.randint(0, (1 << ROW_BITS) - 1)

            # encoding phyisical address 
            address = encode_address(channel, pseudo_channel, rank, bg, bank, row, col)
            col            = col + 1
            if instr_type == 'ST':
                f.write(f"{instr_type} 0x{address:08X} 0x{0:08X} 0x{1:08X} 0x{2:08X} 0x{3:08X} 0x{4:08X} 0x{5:08X} 0x{6:08X} 0x{7:08X}\n")
            else:
                f.write(f"{instr_type} 0x{address:08X}\n")

    print(f"Generated {num_instructions} memory traces in '{filename}'.")

generate_trace("memory_trace.txt", num_instructions=64, seed=123)