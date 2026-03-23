# AsyncDIMM NMAInst Reference

**Date:** 2026-03-20

---

## 1. NMAInst_Slot: 64-bit Unified Instruction Format

AsyncDIMM merges DBX-DIMM's separate AccInst_Slot (memory access) and Inst_Slot (computation) into a single 64-bit instruction. NMA MC handles both computation and memory access.

### 1.1 Bit-field Encoding

```
 63    58 57   51 50 48 47 46 45          28 27   21 20 18 17  12 11        0
┌────────┬───────┬─────┬────┬──────────────┬───────┬─────┬──────┬───────────┐
│comp_op │opsize │ bg  │ bk │     row      │  col  │ id  │rsrvd │    etc    │
│  (6b)  │ (7b)  │(3b) │(2b)│    (18b)     │ (7b)  │(3b) │ (6b) │   (12b)   │
└────────┴───────┴─────┴────┴──────────────┴───────┴─────┴──────┴───────────┘
```

| Field | Bits | Range | Description |
|-------|------|-------|-------------|
| `comp_opcode` | [63:58] | 0-63 | Computation opcode (see Section 2) |
| `opsize` | [57:51] | 0-127 | Number of column accesses (READ/WRITE) or compute elements (NONE) |
| `bg` | [50:48] | 0-7 | Target bankgroup |
| `bk` | [47:46] | 0-3 | Target bank |
| `row` | [45:28] | 0-262143 | Row address (18 bits) |
| `col` | [27:21] | 0-127 | Start column address (7 bits) |
| `id` | [20:18] | 0-7 | Instruction group ID |
| `reserved` | [17:12] | — | Reserved for future use |
| `etc` | [11:0] | — | LOOP: [11:6]=loop_cnt(6b), [5:0]=jump_pc(6b) |

### 1.2 Differences from DBX-DIMM

| Item | DBX-DIMM | AsyncDIMM NMAInst |
|------|----------|-------------------|
| Structure | AccInst(64b) + Inst(64b) = 128b per access | NMAInst(64b) unified |
| Row/Col | AccInst에만 존재 | NMAInst에 통합 |
| ch/pch fields | AccInst에 포함 | 불필요 (NMA MC가 rank-fixed) |
| SELF_EXEC_ON/OFF | Inst opcode 50/51 | 제거 (NMA MC always self-executes) |
| Max program size | 1024 entries (8KB/8B) | 1024 entries (8KB/8B) — `NMA_INST_MAX` |
| Execution model | deque-based | **PC-based** (vector + m_nma_pc) |
| LOOP | body copy + push_front | **in-place counter** (DBX-DIMM style) |

---

## 2. Opcode Table

### 2.1 READ Type — DRAM → NMA Buffer (RD_L × opsize)

X operand is fetched from DRAM. NMA MC generates `opsize` DRAM read requests (RD_L) to the specified bg/bk/row/col.

| Opcode | Name | Operation | Operands | Description |
|--------|------|-----------|----------|-------------|
| 0 | LOAD | Z = X | X: DRAM | Load vector from DRAM to VMA buffer |
| 1 | LOAD_ADD | Z = X + a | X: DRAM, a: scalar const | Load + add scalar constant |
| 2 | LOAD_MUL | Z = X * a | X: DRAM, a: scalar const | Load + multiply scalar constant |
| 3 | ADD | Z = X + Y | X: DRAM, Y: VMA | Element-wise addition |
| 4 | MUL | Z = X * Y | X: DRAM, Y: VMA | Element-wise multiplication |
| 5 | V_RED | Z += X | X: DRAM | Vector reduction (accumulate) |
| 6 | S_RED | z = SUM(X) | X: DRAM | Scalar reduction (sum all elements) |
| 7 | MAC | Z += X * Y | X: DRAM, Y: VMA | Multiply-accumulate |
| 8 | SCALE_ADD | Z = aX + Y | X: DRAM, Y: VMA, a: scalar | Scale and add |

**NMA MC behavior:**
```
AddrGenSlot created → generate_nma_requests():
  opsize requests → REQ FIFO[bg*num_banks+bk]
  Each request: final_command = RD_L
  Sequential column access: col, col+1, col+2, ..., col+opsize-1
```

### 2.2 WRITE Type — NMA Buffer → DRAM (WR_L × opsize)

| Opcode | Name | Operation | Operands | Description |
|--------|------|-----------|----------|-------------|
| 32 | WBD | DRAM = Y | Y: VMA → DRAM | Write-back VMA buffer to DRAM |

**NMA MC behavior:**
```
AddrGenSlot created → generate_nma_requests():
  opsize requests → REQ FIFO[bg*num_banks+bk]
  Each request: final_command = WR_L
```

### 2.3 NONE Type — VMA-Internal Compute (No DRAM Access)

Both operands come from VMA buffer. No DRAM read/write generated.
Latency = `base_latency × opsize` NMA ticks.

| Opcode | Name | Operation | Latency/element | Description |
|--------|------|-----------|-----------------|-------------|
| 16 | T_ADD | Z = Y + Y2 | 4 ticks | Transposed addition |
| 17 | T_MUL | Z = Y * Y2 | 4 ticks | Transposed multiplication |
| 18 | T_V_RED | Z += Y | 4 ticks | Transposed vector reduction |
| 19 | T_S_RED | z = SUM(Y) | 4 ticks | Transposed scalar reduction |
| 20 | T_MAC | Z += Y * Y2 | 4 ticks | Transposed multiply-accumulate |

**NMA MC behavior:**
```
fetch_nma_instructions():
  m_nma_wait_target = COMPUTE_LATENCY_T_* × opsize
  m_nma_state = NMA_WAIT
  (wait ticks complete) → NMA_RUN, pc++
```

**Example:** T_ADD(opsize=127) → 4 × 127 = 508 NMA ticks wait.

### 2.4 CONTROL Type — Flow Control

| Opcode | Name | Fields Used | Description |
|--------|------|-------------|-------------|
| 48 | BARRIER | — | Wait for all outstanding requests + addr_gen_slots to drain |
| 49 | EXIT | — | NMA execution complete → NMA_DONE → PREA_L → NMA_IDLE |
| 52 | LOOP | etc[11:6]=loop_cnt, etc[5:0]=jump_pc | In-place loop control |
| 53 | SET_BASE | id=reg index, row=value | Set base address register: `base_reg[id] = row` |
| 54 | INC_BASE | id=reg index, row=stride | Increment base address register: `base_reg[id] += row` |

**Base Address Registers (8 registers, indexed by `id` field):**
```
READ/WRITE effective_row = base_reg[inst.id] + inst.row

SET_BASE: base_reg[id] = row     (18-bit row field → max 262143)
INC_BASE: base_reg[id] += row    (18-bit stride → max 262143)

Default: base_reg[*] = 0 → effective_row = inst.row (backward compatible)
Reset on start_nma_execution()
```

**LOOP behavior (PC-based, DBX-DIMM style):**
```
inst.cnt = iteration counter (initialized to 0, in-place in m_nma_program)

if (inst.cnt < inst.loop_cnt):
    inst.cnt++          → increment counter
    m_nma_pc = jump_pc  → jump back to loop body start
else:
    inst.cnt = 0        → reset for potential re-execution
    m_nma_pc++          → fall through (loop done)

Total iterations = loop_cnt + 1 (1 initial + loop_cnt repeats)
```

**LOOP + Base Register 조합 예시 (COPY 512K):**
```
pc= 0: SET_BASE(id=0, row=5000)     // source base
pc= 1: SET_BASE(id=1, row=7000)     // destination base
pc= 2: LOAD(opsize=127, bg=0..7, row=0, id=0)  // effective_row = base[0]+0
       ...  (×8 bankgroups)
pc=10: WBD(opsize=127, bg=0..7, row=0, id=1)   // effective_row = base[1]+0
       ...  (×8 bankgroups)
pc=18: INC_BASE(id=0, row=1)        // base[0]++ (5000→5001→...)
pc=19: INC_BASE(id=1, row=1)        // base[1]++ (7000→7001→...)
pc=20: LOOP(loop_cnt=7, jump_pc=2)  // 7 repeats → 8 total iterations
pc=21: EXIT

Execution: rows 5000-5007 (src), 7000-7007 (dst)
Total: 22 instructions (any iteration count)
```

---

## 3. Operand Key

| Symbol | Source | DRAM Access |
|--------|--------|-------------|
| X | Vector from DRAM | RD_L required |
| Y | Vector from NMA VMA buffer | No DRAM access (already loaded) |
| Y2 | Second vector from VMA buffer | No DRAM access |
| Z | Vector result → written to VMA buffer | No DRAM write (stays in VMA) |
| z | Scalar result → written to VMA buffer | No DRAM write |
| a | Scalar constant embedded in instruction | No DRAM access |

---

## 4. Memory Access Type Classification

```cpp
MemAccessType get_mem_access_type():
  READ:    LOAD, LOAD_ADD, LOAD_MUL, ADD, MUL, V_RED, S_RED, MAC, SCALE_ADD
  WRITE:   WBD
  NONE:    T_ADD, T_MUL, T_V_RED, T_S_RED, T_MAC
  CONTROL: BARRIER, EXIT, LOOP

int get_total_accesses():
  READ/WRITE → opsize
  NONE/CONTROL → 0
```

---

## 5. NMA MC Execution Model

### 5.1 Program Loading (via Magic Path)

```
Host trace → ST inst-buf-addr payload[8 × NMAInst]
  → Host MC send() → write_buffer (nma_wr_send_cnt++)
  → Host MC tick() → WR issued → bypass → NMA MC bypass_command()
    → decode_nma_inst(payload) → m_nma_program.push_back()
    → nma_wr_issue_cnt++

Host trace → ST ctrl-reg-addr
  → Host MC: wait nma_wr_send_cnt == nma_wr_issue_cnt
  → WR issued → mode = NMA/CONCURRENT
  → System: start_nma_execution() → m_nma_pc = 0
```

### 5.2 State Machine

```
NMA_IDLE ──(start_nma_execution)──→ NMA_ISSUE_START
NMA_ISSUE_START ──(no refresh pending)──→ NMA_RUN

NMA_RUN: (every NMA tick = every DRAM cycle, NMA_CLOCK_RATIO=1)
  1. check_nma_refresh()      → REFO sets m_nma_refresh_pending
  2. try_issue_*_command()    → ACT_L/RD_L/WR_L/PRE_L (rank-local bus)
  3. generate_nma_requests()  → AddrGenSlot → REQ FIFO (1 per tick)
  4. fetch_nma_instructions() → m_nma_program[m_nma_pc] dispatch

NMA_RUN ──(BARRIER)──→ NMA_BAR ──(FIFOs empty)──→ NMA_RUN
NMA_RUN ──(T_* NONE)──→ NMA_WAIT ──(latency×opsize done)──→ NMA_RUN
NMA_RUN ──(EXIT)──→ NMA_DONE ──(drain + PREA_L)──→ NMA_IDLE
```

### 5.3 DRAM Command Pipeline

```
NMAInst (READ/WRITE)
  ↓ fetch_nma_instructions()
AddrGenSlot {inst, total_accesses=opsize, issued_count, current_col}
  ↓ generate_nma_requests() — 1 per tick, sequential col++
NMARequest {is_read, addr_vec, final_command=RD_L/WR_L}
  → REQ FIFO[bg * num_banks + bk]  (max 8 per bank)
  ↓ try_issue_nma_command() / try_issue_concurrent_command()
    get_nma_preq_command():
      bank CLOSED → ACT_L
      row conflict → PRE_L
      row hit → RD_L / WR_L
    check_ready(cmd) → issue_command(cmd)  ← rank-local bus (no channel)
```

### 5.4 Bus Architecture

```
NMA MC DRAM commands: ACT_L, RD_L, WR_L, PRE_L, PREA_L, REFab_L
  → Rank-local CA bus + Rank-local data bus
  → No channel CA bus occupancy
  → No channel DQ bus occupancy
  → Can issue simultaneously with Host MC's offload commands (ACTO/RDO/WRO/PREO)
```

---

## 6. Workload Examples

### 6.1 COPY (Y = X)

```
Program (base register + LOOP):
  SET_BASE(id=0, row=5000)         // source base
  SET_BASE(id=1, row=7000)         // destination base
  LOAD(opsize=127, bg=0..7, bk=3, row=0, col=0, id=0)  × 8bg
  WBD(opsize=127, bg=0..7, bk=3, row=0, col=0, id=1)   × 8bg
  INC_BASE(id=0, stride=1)        // base[0]++
  INC_BASE(id=1, stride=1)        // base[1]++
  LOOP(loop_cnt=iteration-1, jump_pc=2)
  EXIT

Total: 22 instructions (constant, regardless of iteration count)
DRAM accesses per iteration: 8 BG × 128 RD_L + 8 BG × 128 WR_L = 2048
```

### 6.2 AXPY (Y = aX + Y)

```
Program:
  LOAD_MUL(opsize, bg0..7, bk=3, row=5000)  // Z = X * a
  SCALE_ADD(opsize, bg0..7, bk=3, row=6000)  // Z = aX + Y (Y from DRAM)
  WBD(opsize, bg0..7, bk=3, row=7000)        // DRAM = Z
  LOOP(loop_cnt, jump_pc=0)
  EXIT
```

### 6.3 DOT (c = X·Y)

```
Program:
  MAC(opsize, bg0..7, bk=3, row=5000)        // Z += X * Y (X from DRAM, Y from VMA)
  BARRIER                                      // Wait for all MACs to complete
  T_S_RED(opsize=8)                           // z = SUM(Z) (scalar reduction, VMA-internal)
  EXIT
```

---

## 7. gen_trace.py Encoding Functions

### 7.1 nma_inst() — NMAInst 64-bit encoding

```python
def nma_inst(comp_opcode, opsize, bg, bk, row, col, id, etc=0):
    inst = 0
    inst |= (comp_opcode & 0x3F) << 58
    inst |= (opsize      & 0x7F) << 51
    inst |= (bg          & 0x7)  << 48
    inst |= (bk          & 0x3)  << 46
    inst |= (row       & 0x3FFFF) << 28
    inst |= (col         & 0x7F) << 21
    inst |= (id          & 0x7)  << 18
    inst |= (etc         & 0xFFF)
    return inst
```

### 7.2 nma_inst_loop() — LOOP encoding

```python
def nma_inst_loop(loop_cnt, jump_pc):
    etc = ((loop_cnt & 0x3F) << 6) | (jump_pc & 0x3F)
    return nma_inst(LOOP, 0, 0, 0, 0, 0, 0, etc)
```

### 7.3 nma_inst_set_base() / nma_inst_inc_base() — Base register control

```python
def nma_inst_set_base(reg_id, row_value):
    """SET_BASE: base_reg[reg_id] = row_value"""
    return nma_inst(SET_BASE, 0, 0, 0, row_value, 0, reg_id)

def nma_inst_inc_base(reg_id, stride):
    """INC_BASE: base_reg[reg_id] += stride (row field, 18-bit)"""
    return nma_inst(INC_BASE, 0, 0, 0, stride, 0, reg_id)
```

### 7.3 Address encoding for magic-path writes

```python
# Inst-buffer: row=MAX, bg=MAX-1, bk=MAX
addr = encode_asyncdimm_address(ch, rank, ASYNCDIMM_NMA_BUF_BG, ASYNCDIMM_NMA_BUF_BK,
                                 ASYNCDIMM_NMA_ROW, col_chunk_idx)
# 8 NMAInst per WR (64B payload = 8 × 8B)

# Ctrl-register (start trigger): row=MAX, bg=MAX, bk=MAX
addr = encode_asyncdimm_address(ch, rank, ASYNCDIMM_NMA_CTRL_BG, ASYNCDIMM_NMA_CTRL_BK,
                                 ASYNCDIMM_NMA_ROW, 0)
# payload[rank] = 1
```
