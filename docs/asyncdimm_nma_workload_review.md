# AsyncDIMM NMA Workload Execution Review

**Date:** 2026-03-20
**Scope:** NMA workload execution in NMA Mode and Concurrent Mode

---

## 1. Changes Applied

### 1.1 T_* NONE ops: latency × opsize
```
Before: m_nma_wait_target = COMPUTE_LATENCY_T_ADD (= 1 tick, fixed)
After:  m_nma_wait_target = base_latency * opsize
  T_ADD(opsize=127) → 1 × 127 = 127 NMA ticks
```

### 1.2 LOOP: PC-based execution (DBX-DIMM style)
```
Before: deque-based body copy + push_front (fragile, cnt field reuse)
After:  m_nma_program[m_nma_pc] direct access, in-place counter
  LOOP.cnt tracks current iteration
  cnt < opsize → cnt++, pc = jump_pc (loop back)
  cnt >= opsize → cnt = 0, pc++ (fall through)
```

### 1.3 Program buffer: deque → vector + PC
```
Before: m_nma_inst_queue (deque) + m_nma_program (vector, for LOOP copy)
After:  m_nma_program (vector only) + m_nma_pc (int)
  No instruction queue — direct program counter execution
  m_nma_inst_queue removed entirely
```

### 1.4 NMA_CLOCK_RATIO = 1
NMA MC state machine runs every DRAM cycle.

### 1.5 NMA-Local Commands (ACT_L/RD_L/WR_L/PRE_L/PREA_L/REFab_L)
NMA MC DRAM commands use rank-local CA + data bus (no channel bus occupancy).

---

## 2. Scenario A: NMA Mode — COPY Workload

**Program:** `[LOAD(bg0), LOAD(bg1), ..., LOAD(bg7), WBD(bg0), ..., WBD(bg7), EXIT]`

### Step-by-step trace:
```
=== Instruction Loading ===
Host trace: ST inst-buf payload[LOAD×8, WBD×8, EXIT]
  → Host MC send() → write_buffer[rank 0], is_ndp_req=true, nma_wr_send_cnt++
  → Host MC tick() → ACT(inst-buf bank) → WR(inst-buf bank)
    → bypass → NMA MC bypass_command():
      inst-buf WR → decode 8 NMAInst → m_nma_program = [LOAD×8]
    → nma_wr_issue_cnt++

Host trace: ST inst-buf payload[WBD×8, EXIT, 0, 0, 0, 0, 0]
  → same flow → m_nma_program = [LOAD×8, WBD×8, EXIT]

Host trace: ST ctrl-reg
  → Host MC: nma_wr_send_cnt == nma_wr_issue_cnt? → YES → WR issued
  → mode = NMA → System: start_nma_execution() → m_nma_pc = 0

=== NMA Execution ===
NMA_ISSUE_START: no banks refreshing → NMA_RUN

tick: fetch_nma_instructions()
  pc=0: LOAD(bg=0, bk=3, row=5000, opsize=127) → READ → AddrGenSlot, pc=1
  pc=1: LOAD(bg=1, ...) → AddrGenSlot, pc=2
  ... (one per tick, ADDR_GEN_SLOT_MAX=8 → all 8 fetched in 8 ticks)

  generate_nma_requests(): each tick, 1 request → REQ FIFO[bank]
    NMARequest(is_read=true, final_command=RD_L, row=5000, col=0..126)

  try_issue_nma_command(): round-robin over banks
    ACT_L(bg0,bk3,row=5000) → check_ready → issue → bank OPENED
    (tRCD later) RD_L(bg0,bk3,col=0) → issue → pop from REQ FIFO
    RD_L(bg0,bk3,col=1) → ...
    ... (127 × 8 bg = 1016 RD_L total)

  All LOAD done → addr_gen_slots empty → fetch pc=8: WBD(bg=0,...) → WRITE
  ... (127 × 8 bg = 1016 WR_L total)

  pc=16: EXIT → NMA_DONE
    → drain FIFOs → PREA_L → all banks CLOSED → m_nma_program.clear() → NMA_IDLE

System: is_nma_complete() → TRANSITIONING_N2H → set_mode(HOST) → HOST_MODE ✓
```

**검증 결과: 정상 동작** ✓

---

## 3. Scenario B: Concurrent Mode (Host + NMA)

**설정:** Rank 0 = CONCURRENT, Host LD/ST traffic + NMA COPY workload

### Host offload path:
```
Host LD addr → read_buffer[0]
  → schedule_offload_for_rank(0):
    m_open_row[bank] == -1 → ACTO
  → tick() issue: ACTO → channel CA 2-cycle → bypass → NMA MC CMD FIFO[ACT_L]
  → next tick: row hit → RDO → channel CA 2-cycle → bypass → CMD FIFO[RD_L]
    → RU entry created (cmd_count=2)
    → NMA MC processes CMD FIFO: ACT_L → RD_L (rank-local bus)
    → return_buffer done → TDM interrupt → Host MC on_nma_interrupt
    → RU pop → OT -= 2, rt_read_pending++
    → RT3 issued → DQ bus 3×nBL → pending → callback
```

### NMA workload path (동시):
```
NMA MC tick(): try_issue_concurrent_command()
  H/N Arbiter: CMD FIFO vs REQ FIFO per bank
  REQ FIFO: ACT_L(bg0,row=5000) → rank-local bus (channel 무관)
    → channel CA 비점유! Host MC의 ACTO/RDO와 동시 발행 가능 ✓

  generate_nma_requests(): REQ FIFO에 NMA 요청 추가
  fetch_nma_instructions(): pc 기반으로 계속 진행
```

### Bus 분리 검증:
```
tick 100: Host MC issue ACTO → channel CA [100,102)
          NMA MC issue ACT_L → rank-local CA (channel 무관)
          → 동시 발행 OK ✓

tick 108: Host MC issue RT3 → channel DQ [108,132)
          NMA MC issue RD_L → rank-local data (channel DQ 무관)
          → 동시 발행 OK ✓
```

**검증 결과: 정상 동작 — channel/rank-local bus 분리 ✓**

---

## 4. Scenario C: LOOP 동작

**Program:** `[LOAD(bg0), LOAD(bg1), WBD(bg0), WBD(bg1), LOOP(opsize=3,jump_pc=0), EXIT]`

```
pc=0: LOAD(bg0) → AddrGenSlot, pc=1
pc=1: LOAD(bg1) → AddrGenSlot, pc=2
pc=2: WBD(bg0) → AddrGenSlot, pc=3
pc=3: WBD(bg1) → AddrGenSlot, pc=4
pc=4: LOOP → cnt=0 < opsize=3 → cnt=1, pc=0  (1st repeat)
pc=0: LOAD → ..., pc=4
pc=4: LOOP → cnt=1 < 3 → cnt=2, pc=0  (2nd repeat)
pc=0: LOAD → ..., pc=4
pc=4: LOOP → cnt=2 < 3 → cnt=3, pc=0  (3rd repeat)
pc=0: LOAD → ..., pc=4
pc=4: LOOP → cnt=3 >= 3 → cnt=0, pc=5  (done)
pc=5: EXIT → NMA_DONE

Total body executions: 1 (initial) + 3 (loops) = 4 ✓
```

**Note:** gen_trace.py의 LOOP 인코딩에서 `opsize` field가 아닌 `etc` field에 `(iteration << 6) | jump_pc`를 넣음. NMAInst_Slot 생성자가 LOOP일 때 `loop_cnt = (etc >> 6) & 0x3F`, `jump_pc = etc & 0x3F`로 decode.
fetch_nma_instructions()에서는 `inst.opsize`가 아닌 `inst.loop_cnt`와 `inst.jump_pc`를 사용해야 함.

**버그 발견: LOOP에서 opsize 대신 loop_cnt/jump_pc를 사용해야 함!**

---

## 5. Scenario D: T_* NONE with opsize

**Program:** `[LOAD(opsize=127), T_ADD(opsize=127), WBD(opsize=127), EXIT]`

```
pc=0: LOAD(opsize=127) → 127 RD_L requests → REQ FIFO
pc=1: T_ADD(opsize=127) → NMA_WAIT, target = 1 × 127 = 127 ticks
  (tick_nma_wait: 127 ticks, continue issuing from REQ FIFO)
  → wait complete → NMA_RUN
pc=2: WBD(opsize=127) → 127 WR_L requests → REQ FIFO
pc=3: EXIT → NMA_DONE
```

**검증: latency × opsize 스케일링 정상 ✓**

---

## 6. Scenario E: Refresh + CMD FIFO

```
CMD FIFO[bank 5]: [ACT_L(row=3000)] issued → [RD_L(col=10)] pending
REFO 수신 → m_nma_refresh_pending = true
  try_issue_nma_refresh():
    banks not all closed → PREA_L → all banks CLOSED, SR reset
    all_banks_closed → REFab_L → refreshing → completion

  try_issue_from_cmd_fifo(bank 5):
    front = RD_L, bank = CLOSED → bank-state recovery!
    → ACT_L(row=3000) issued → bank OPENED
    → next tick: RD_L(col=10) → OK ✓
```

**검증: post-refresh bank-state recovery 정상 ✓**

---

## 7. Scenario F: RU Full (64 entries)

```
RU has 64 entries (all waiting for interrupt)
  → schedule_offload_for_rank: m_return_unit.size() >= 64 → return false
  → active_buffer RDO: RU full → skip
  → NMA MC interrupt → RU pop (63) → 다음 tick offload 재개 ✓

  NMA workload (REQ FIFO): RU와 무관, ACT_L/RD_L 계속 발행
  → rank-local bus → channel 무관 → NMA 작업 중단 없음 ✓
```

---

## 8. Bugs Found

### Bug 1 (Critical): LOOP uses opsize but should use loop_cnt/jump_pc

현재 fetch_nma_instructions() LOOP 코드:
```cpp
int loop_limit = inst.opsize;   // ← WRONG!
int jump_pc    = inst.jump_pc;  // ← OK (decoded in constructor)
```

NMAInst_Slot 생성자에서 LOOP일 때:
```cpp
if (_comp_opcode == LOOP) {
    loop_cnt = (_etc >> 6) & 0x3F;  // iteration count
    jump_pc  = _etc & 0x3F;         // jump target
}
```

**opsize는 LOOP에서 무의미**. `loop_cnt`를 사용해야 함:
```cpp
int loop_limit = inst.loop_cnt;  // ← CORRECT
```

### Bug 2 (Minor): gen_trace.py LOOP 인코딩 확인 필요

gen_trace.py의 `nma_inst_loop()`:
```python
etc = ((loop_cnt & 0x3F) << 6) | (jump_pc & 0x3F)
return nma_inst(LOOP, 0, 0, 0, 0, 0, 0, etc)
```

opsize=0으로 전달. NMAInst_Slot의 `opsize` field = 0.
`loop_cnt`는 etc에서 decode → `inst.loop_cnt`에 저장.
→ `inst.opsize`를 쓰면 항상 0 → 루프 1회만 실행 → **버그!**

---

## 9. Fix Required

```cpp
// fetch_nma_instructions() LOOP section:
case NMAInst_Slot::LOOP: {
    int loop_limit = inst.loop_cnt;  // NOT inst.opsize
    int jump_pc    = inst.jump_pc;
    ...
}
```
