# Ramulator2 - Forked DRAM Simulator

Forked from Ramulator 2.0 to customize DRAM structure (DDR5-DIMM with pseudochannel, NDP capability).

## Build

```bash
mkdir -p build && cd build
cmake ..                                    # Release
cmake -DCMAKE_BUILD_TYPE:STRING=Debug ..    # Debug (enables RAMULATOR_DEBUG)
make -j
```

Output binary: `ramulator2` (in project root)
Shared library: `libramulator.so` (in project root)

## Project Structure

- `src/` — Core source code (C++20)
  - `dram/impl/` — DRAM standard implementations (DDR3/4/5, HBM, LPDDR5, GDDR6, DDR5-pCH)
  - `dram_controller/impl/` — Controller implementations (generic, NDP, BH, PRAC)
  - `frontend/` — Frontend (trace-driven, etc.)
  - `addr_mapper/` — Address mapping
  - `memory_system/` — Memory system abstraction
  - `translation/` — Address translation
- `ext/` — External dependencies (yaml-cpp, spdlog, argparse) fetched via CMake
- `gen_trace.py` — Trace generation script
- `*.yaml` — Configuration files (ddr5_config, ddr5_pch_config, etc.)
- `docs/` — Design documents

## Key Concepts

- **Pseudochannel (pCH)**: DDR5 subchannel support
- **NDP (Near-Data Processing)**: Custom NDP controller (`ndp_dram_controller.cpp`)
- **YAML config**: All simulation parameters configured via YAML files

## Dependencies

- C++20 required
- yaml-cpp 0.7.0
- spdlog 1.11.0
- argparse 2.9

---

## AsyncDIMM Implementation Task (DBX-DIMM Comparison)

Implement AsyncDIMM (HPCA 2025) paper in ramulator2 for performance comparison with DBX-DIMM.
- Paper: `docs/AsyncDIMM_Achieving_Asynchronous_Execution_in_DIMM-Based_Near-Memory_Processing.pdf`
- Detailed analysis document: `docs/asyncdimm_implementation_analysis.md`

### Design Decisions
- **DRAM Base**: `DDR5.cpp` (base DDR5, no pseudo-channel)
- **Address Mapping**: `RoBaCoRaCh` (baseline과 동일)
- **NMA Location**: Buffer chip (rank-level)
- **NMA Offloading**: Same 2-step approach as DBX-DIMM (instruction write → control register write)
- **NMA Clock**: 1/4 of DRAM clock (same as DBX-DIMM NDP Unit). On-DIMM digital logic cannot run at full DRAM clock speed (e.g., 2.4GHz for DDR5-4800, 4.4GHz for DDR5-8800). NMA MC operates once every 4 DRAM cycles.

### NMAInst Format (Unified NMA Instruction)

DBX-DIMM separates AccInst_Slot (memory access) and Inst_Slot (computation).
AsyncDIMM unifies them into a single NMAInst since NMA MC handles both computation and memory access.

**64-bit Encoding:**
```
[63:58] comp_opcode  (6b)  — Same computation opcodes as DBX-DIMM Inst_Slot
[57:51] opsize       (7b)  — Number of column accesses
[50:48] bg           (3b)  — Bankgroup
[47:46] bk           (2b)  — Bank
[45:28] row          (18b) — Row address (added; not present in Inst_Slot)
[27:21] col          (7b)  — Start column (added; not present in Inst_Slot)
[20:18] id           (3b)  — Instruction group ID
[17:12] reserved     (6b)
[11:0]  etc          (12b) — LOOP: [11:6]=loop_cnt, [5:0]=jump_pc
```

**comp_opcode → Memory Access Mapping:**

Operand key: `X` = vector from DRAM (RD needed), `Y`/`Y2` = vector from VMA buffer (no DRAM), `Z` = vector result → VMA, `z` = scalar result → VMA, `a` = scalar const in instruction

| opcode | Name | Operation | Memory Access | Description |
|--------|------|-----------|---------------|-------------|
| 0 | LOAD | Z = X | RD × opsize | DRAM → NMA buffer |
| 1 | LOAD_ADD | Z = X + a | RD × opsize | DRAM → buffer, add scalar const |
| 2 | LOAD_MUL | Z = X * a | RD × opsize | DRAM → buffer, mul scalar const |
| 3 | ADD | Z = X + Y | RD × opsize | X from DRAM, Y from VMA |
| 4 | MUL | Z = X * Y | RD × opsize | X from DRAM, Y from VMA |
| 5 | V_RED | Z += X | RD × opsize | Vector reduction (X from DRAM) |
| 6 | S_RED | z = SUM(X) | RD × opsize | Scalar reduction (X from DRAM) |
| 7 | MAC | Z += X * Y | RD × opsize | X from DRAM, Y from VMA |
| 8 | SCALE_ADD | Z = aX + Y | RD × opsize | X from DRAM, Y from VMA |
| 16 | T_ADD | Z = Y + Y2 | (none) | Both operands from VMA; VMA-internal |
| 17 | T_MUL | Z = Y * Y2 | (none) | Both operands from VMA; VMA-internal |
| 18 | T_V_RED | Z += Y | (none) | Y from VMA; VMA-internal |
| 19 | T_S_RED | z = SUM(Y) | (none) | Y from VMA; VMA-internal |
| 20 | T_MAC | Z += Y * Y2 | (none) | Both operands from VMA; VMA-internal |
| 32 | WBD | X = Y | WR × opsize | Y (VMA buffer) → DRAM |
| 48 | BARRIER | — | (none) | Wait for all outstanding requests |
| 49 | EXIT | — | (none) | NMA execution complete |
| 52 | LOOP | — | (none) | etc[11:6]=loop_cnt, etc[5:0]=jump_pc |

**Differences from DBX-DIMM Inst_Slot:**
- Added row/col fields (NMA MC directly addresses DRAM)
- Removed SELF_EXEC_ON/OFF (50,51) — NMA MC always self-executes
- No ch/pch needed (NMA MC is fixed to its rank/channel)

### Development Strategy
Incremental development: Host Mode first → NMA Mode → Concurrent Mode.
Each phase is independently testable before proceeding to the next.

---

### Phase 1: Host Mode (Foundation)
Goal: AsyncDIMM system running in Host Mode with NMA MC tracking host bank states via explicit sync.

#### 1-A: DDR5-AsyncDIMM DRAM Model
- [x] Create `src/dram/impl/DDR5-AsyncDIMM.cpp` (fork DDR5.cpp)
  - Add new commands: ACTO, PREO, RDO, WRO, RT
  - ACTO/PREO: no bank state change action lambda
  - RDO: no DQ bus usage, omit tRCD/tBL/tRTRs/tCCD (relaxed timing)
  - WRO: DQ bus used, omit tWL
  - RT: variable burst length (batch_size × tBL)
- [ ] Modify `src/dram/lambdas/action.h` — offload command actions (no state change)
- [ ] Modify `src/dram/lambdas/preq.h` — offload command prerequisites

#### 1-B: Host MC (Host Mode Only)
- [x] Create `src/dram_controller/impl/asyncdimm_host_controller.cpp` (fork generic)
  - Host Mode only: existing FRFCFS + explicit sync (bypass all issued commands to NMA MC)
  - RefreshManager: standard DDR5 refresh (REFab/REFsb), bypassed to NMA MC
  - Per-rank mode flag (HOST / NMA / CONCURRENT)
  - Bypass callback (BypassCallback) with payload for DQ magic path

#### 1-C: NMA MC Skeleton (Bypass Path Only)
- [x] Create `src/dram_controller/impl/asyncdimm_nma_controller.cpp` (new)
  - Host Mode bypass path only: receive host commands → update bank FSM (no DRAM issue)
  - Per-bank bank state tracking (shadow copy of Host MC's FSM)
  - Phase 2 NMA Mode extensions added (see Phase 2)

#### 1-D: AsyncDIMM Memory System + Config
- [x] Create `src/memory_system/impl/asyncdimm_system.cpp` (reference ndp_DRAM_system)
  - Manage 1 Host MC + N NMA MCs (one per rank) within a channel
  - Host Mode: route all requests to Host MC, bypass to NMA MC
  - Per-rank SystemNMAState machine
  - Reuse existing LoadStoreNCoreTrace frontend (no frontend changes)
- [x] Create `asyncdimm_config.yaml` configuration file
  - DRAM: DDR5-AsyncDIMM, Address Mapping: RoBaBgCoRaCh
  - Controller: AsyncDIMM Host MC + NMA MC

#### 1-E: Host Mode Validation
- [x] Verify Host MC functions identically to GenericDRAMController (same trace, same results)
  - Code review: scheduling, buffer management, row policy, finalize all identical
  - Fixed write-forwarding to match Generic behavior (both search empty singular buffer)
  - Added PREsb row tracking (missing from both Generic and AsyncDIMM, added for correctness)
  - Known difference: host_access_cnt condition (Generic has bug: always true; AsyncDIMM correct)
- [x] Verify NMA MC bank FSM is in sync with Host MC at every cycle
  - Added FSM sync verification: `debug_fsm_sync: true` in YAML
  - Compares Host MC open_row vs NMA MC shadow FSM every tick
  - Handles REFRESHING state (skipped in comparison — Host MC doesn't track it)
- [x] Verify REF commands are properly bypassed to NMA MC
  - AllBankRefresh sends REFab via priority_send → scheduler picks up → issues → bypass callback
  - PREA prerequisite also bypassed through same path
  - NMA MC handles REFab → REFRESHING state + future action for completion

---

### Phase 2: NMA Mode
Goal: NMA MC can independently schedule and execute NMA memory requests.
NMA programming model: Memory-mapped host-initiated, unified NMAInst_Slot format.

#### 2-A: NMA MC REQ FIFO + DRAM Command Scheduling
- [x] NMA state machine: NMA_IDLE → NMA_ISSUE_START → NMA_RUN → NMA_BAR/WAIT → NMA_DONE → NMA_IDLE
- [x] NMAInst_Slot-based address generation (unified instruction format)
- [x] comp_opcode → memory access type mapping (READ/WRITE/NONE/CONTROL)
  - READ: LOAD/LOAD_ADD/LOAD_MUL/ADD/MUL/V_RED/S_RED/MAC/SCALE_ADD (X from DRAM)
  - WRITE: WBD (Y from VMA → DRAM)
  - NONE: T_ADD/T_MUL/T_V_RED/T_S_RED/T_MAC (Y/Y2 from VMA, no DRAM access)
  - CONTROL: BARRIER/EXIT/LOOP
- [x] Shadow FSM-based prerequisite resolution (CLOSED→ACT, row conflict→PRE, row hit→RD/WR)
- [x] DRAM command issuing via `check_ready()` + `issue_command()`
- [x] Per-bank REQ FIFO: round-robin front scheduling (max 8 per bank)
- [x] T_* (NONE type): no DRAM access, modeled as 1 NMA tick fixed latency

#### 2-B: NMA MC Refresh During NMA Mode
- [x] NMA MC tracks nREFI interval and issues REFab when due
- [x] Requires all banks closed before refresh (same as AllBankRefresh)
- [x] FSM update on refresh issue + future action for completion

#### 2-C: Mode Transition (Host→NMA→Host) — DBX-DIMM Style Lifecycle
- [x] System-level state: HOST_MODE → TRANSITIONING_H2N → NMA_MODE → TRANSITIONING_N2H → HOST_MODE
- [x] **Host MC driven mode transition** (not NMA MC polling):
  1. `send()`: detect NMAInst writes (inst-buf address) → `nma_wr_send_cnt++`, flag `is_ndp_req`
  2. `send()`: detect ctrl-reg write → `nma_start_requested = true`
  3. `tick()`: hold ctrl-reg WR issuance until `nma_wr_send_cnt == nma_wr_issue_cnt`
  4. `tick()`: on NMAInst WR issue → `nma_wr_issue_cnt++`
  5. `tick()`: on ctrl-reg WR issue → `set_mode(NMA/CONCURRENT)` (mode change point)
  6. System polls `host_mc->get_mode()` → detects NMA/CONCURRENT → calls `start_nma_execution()`
- [x] N2H (Implicit Sync): NMA MC PREA (close all banks) → IDLE, Host MC restored to HOST mode
- [x] Per-rank mode management
- [x] Race condition prevention: ctrl-reg WR always issued AFTER all NMAInst WRs
- [x] active_buffer requests safe: post-ACT requests offloaded after mode change

#### 2-D: NMA Workload Driver (Trace-Based)
- [x] Memory-mapped control region (same pattern as DBX-DIMM HSNC):
  - Control register: row=MAX, bg=MAX, bk=MAX → Host MC NMA control unit
  - Instruction buffer: row=MAX, bg=MAX-1, bk=MAX → NMAInst_Slot write (magic path)
- [x] 64-bit NMAInst_Slot encoding (unified format, see NMAInst Format section above)
- [x] `decode_nma_inst()`: comp_opcode(6b), opsize(7b), bg(3b), bk(2b), row(18b), col(7b), id(3b), etc(12b)
- [x] Trace file loading (same format: `[TIMESTAMP] LD/ST ADDR [PAYLOAD...]`)
- [x] trace_core functionality in AsyncDIMM System (`try_issue_trace_requests()`)

#### 2-E: Host MC Refresh Suspension
- [x] Host MC `priority_send()`: skip refresh for NMA-mode ranks (return true to avoid error)
- [x] Host MC `send()`: reject host requests for NMA-mode ranks (return false)
- [x] Host MC bypass: disabled for NMA-mode ranks (NMA MC manages own FSM)

#### 2-F: NMAInst Execution Unit
- [x] NMAInst_Slot struct in `request.h` with CompOpcode enum and helper methods
- [x] `get_mem_access_type()`: READ/WRITE/NONE/CONTROL classification
  - ADD/MUL → READ (X from DRAM); T_* → NONE (Y/Y2 from VMA only)
- [x] `get_total_accesses()`: opsize for READ/WRITE, 0 for NONE/CONTROL
- [x] VMA-internal T_* ops modeled as 1 NMA tick fixed latency (NMA_WAIT state)
- [x] LOOP instruction: loop_cnt decrement + jump_pc re-enqueue from program buffer
  - inst copied before pop_front() to avoid dangling reference
  - program end index cached in cnt field for multi-iteration correctness
- [x] NMA clock ratio (1/4 DRAM clock, NMA_CLOCK_RATIO=4) applied to all NMA operations

---

### Phase 3: Concurrent Mode (Full AsyncDIMM)
Goal: Full OSR mechanism — host offloads commands to NMA MC, concurrent execution.

#### Clock Domain & Offload Rate Analysis
- NMA MC runs at 1/4 DRAM clock (NMA_CLOCK_RATIO = 4)
- Offload commands (ACTO/RDO/PREO) have **no DRAM timing constraints** (no tRCD, tCCD, etc.)
  - Only CA bus 2-cycle channel-level constraint → Host MC can issue 1 offload cmd every 2 DRAM cycles
  - WRO additionally uses DQ bus (access=true) → nBL constraint with other WR/WRO
- Per NMA tick (4 DRAM cycles): Host MC can send up to **2 offload commands**
- NMA MC decodes offload commands into real DRAM commands (ACTO→ACT, RDO→RD, etc.)
  - Real DRAM commands ARE subject to timing constraints (tRCD, tRAS, tCCD, etc.)
  - NMA MC DRAM command issue rate is timing-bound, not clock-bound
- **Offload send rate (2 cmds/4 cycles) > DRAM execution rate (timing-bound)**
  → CMD FIFO accumulates commands
  → OT (Offload Table) threshold required for backpressure

```
Host MC (DRAM clock, 2-cycle CA)     NMA MC (1/4 clock)
 ACTO ──┐                            ┌─ decode ─→ ACT ──tRCD──→ RD
 RDO  ──┤ CMD FIFO (accumulates)  ───┤   (1 NMA tick = 4 DRAM cycles)
 ACTO ──┤ write: 2 cmds/NMA tick     └─ DRAM timing is the bottleneck
 PREO ──┘
```

#### 3-A: Host MC Concurrent Extensions
- [x] Offload scheduling: `try_schedule_offload()` — separate path for CONCURRENT ranks
  - **Bypasses normal DRAM scheduler** (`get_best_request()` skipped for CONCURRENT ranks)
  - Uses Host MC's own `m_open_row[]` for prerequisite resolution:
    - Bank closed → ACTO; row conflict → PREO; row hit → RDO/WRO
  - Issues ACTO/RDO/WRO/PREO directly to DRAM model (CA 2-cycle + WRO nBL only)
  - Two-pass scheduling: row hits (RDO/WRO) first, then misses (ACTO/PREO) — FRFCFS-like
  - `m_open_row[]` updated manually (ACTO opens, PREO closes; DRAM model state unchanged)
  - Requests stay in read/write buffer until RDO/WRO (no active_buffer for offload)
  - WRO: DQ bus used (access=true, nBL constraint); RDO: no DQ bus (access=false)
- [x] Offload Table (OT): per-bank counter, threshold = CMD FIFO size (8)
  - Enforced inside `try_schedule_offload()`: skip bank if OT >= threshold
  - OT decremented on NMA MC interrupt via `on_nma_interrupt()`
- [ ] Return Unit (RU) — Host MC side:
  - Per-rank FIFO of offloaded read requests in issue order (addr, R/W, cmd_count)
  - cmd_count: 2-bit per entry (01=RD/WR, 10=ACT+RD/WR, 11=PRE+ACT+RD/WR) for OT update
  - `m_pending_interrupt_batch[rank]`: accumulates received interrupt counts
  - On interrupt received: issue RT with batch_size = interrupt_cnt (highest priority)
  - RT latency = NMA indexing time (not tRL); DQ burst = tBL × batch_size
  - **Status**: interrupt callback wired; RT priority issuance not yet implemented
- [x] Suspend Host RefreshManager during Concurrent Mode (NMA MC handles REF directly)

#### 3-B: NMA MC Concurrent Extensions
- [x] Configurable Mixed FIFO (no new hardware — rebuild existing REQ FIFO):
  - NMA mode: REQ FIFO size = 8 entries per bank (full capacity for NMA)
  - Concurrent mode: CMD FIFO = 8 entries + REQ FIFO = 8 entries per bank (paper Table IV)
  - Note: paper §V.B.2 describes HW sharing ("half capacity reused"), but simulation uses 8 each
  - `m_concurrent_mode`: flag that controls FIFO split behavior
- [x] CMD FIFO (per-bank, size 8 in Concurrent mode):
  - Populated by `bypass_command()` when ACTO/RDO/WRO/PREO received from Host MC
  - CMD Decoder: ACTO→ACT, RDO→RD, WRO→WR, PREO→PRE (before inserting)
  - Row locality inherently preserved (commands pre-scheduled by Host MC FRFCFS)
- [x] H/N Arbiter (per-bank):
  - Switch CMD FIFO → REQ FIFO when: CMD FIFO empty OR PRE issued from CMD FIFO
  - Switch REQ FIFO → CMD FIFO when: REQ FIFO empty OR PRE issued from REQ FIFO
  - Goal: maximize row locality, avoid starvation
- [x] SR Unit (per-bank, Switch-Recovery):
  - Records last BSC (Bank State Change) command from both CMD FIFO and REQ FIFO
  - Paper Fig 5 recovery table (current bank state × destination FIFO's last BSC):
    ```
    Actual\Expected  | ACT (open)  | PRE (closed)
    ACT (open)       | PRE+ACT     | PRE
    PRE (closed)     | ACT         | None
    ```
  - **Current impl**: only handles case (bank OPENED, CMD front=ACT → PRE recovery).
    Missing: PRE+ACT for row conflict, ACT for closed bank with open-expecting dest.
  - Paper reports: <2 banks need recovery on average per mode switch
- [x] Return Unit — NMA MC side:
  - Per offloaded read: assign global seq_num on RDO arrival
  - On DRAM read completion (out-of-order across banks): mark entry done in `m_return_buffer`
  - `check_and_send_interrupt()`: fire interrupt only when head entry is done (head-of-line blocking)
  - TDM interrupt signal: 1-bit alert_n shared by ranks, TDM latency = N_rank/2 cycles
  - On RT received from Host MC: `receive_rt(batch_size)` (stat tracking)

#### 3-C: System Integration
- [x] Mode transition: Host ↔ Concurrent (distinct from Host ↔ NMA transitions)
  - H2C: Suspend Host refresh (NMA MC handles REF), start NMA execution + concurrent mode
    Host MC continues scheduling host requests normally (offloads via bypass callback)
  - C2H (Implicit Sync N2H): NMA MC PREA → IDLE; exit concurrent mode → HOST
    Paper: check per-bank BSC + selective recovery; impl: simplified PREA (conservative)
  - New states: TRANSITIONING_H2C, CONCURRENT, TRANSITIONING_C2H
  - YAML config: `concurrent_mode_enable: true` selects Phase 3 vs Phase 2
- [x] TDM interrupt latency modeling: `TDM_INTERRUPT_LATENCY = 2` (N_rank/2 for 4 ranks)
- [x] Interrupt callback wiring: NMA MC → Host MC `on_nma_interrupt(rank_id, batch_size)`
- [ ] RT command scheduling: highest priority over all DDR commands in Host MC (not yet impl)
- [ ] Memory-mapped Concurrent Mode control region (currently reuses NMA mode trigger)

---

### Phase 4: Validation and DBX-DIMM Comparison
- [ ] Generate Host + NMA traces (BK: COPY/ADD/SCALE/AXPY/TRIAD/GEMV, CK: SpMV/BFS)
- [ ] Collect statistics: Speedup, Bandwidth Utilization, Access Latency, Energy
- [ ] DBX-DIMM vs AsyncDIMM performance comparison analysis

---

## AsyncDIMM Implementation Reference

### asyncdimm_host_controller.cpp
`class AsyncDIMMHostController` — implements `IDRAMController`

#### Data Structures
| Member | Type | Description |
|--------|------|-------------|
| `m_read_buffers[rk]` | `ReqBuffer[]` | Per-rank read request buffers (depth 32) |
| `m_write_buffers[rk]` | `ReqBuffer[]` | Per-rank write request buffers (depth 32) |
| `m_priority_buffers[rk]` | `ReqBuffer[]` | Per-rank refresh/priority buffers |
| `m_active_buffer` | `ReqBuffer` | Requests with open row (post-ACT) |
| `m_mode_per_rank[rk]` | `AsyncDIMMMode[]` | Per-rank mode: HOST / NMA / CONCURRENT |
| `m_open_row[flat_bank_id]` | `int[]` | Tracks currently open row per bank |
| `m_bypass_to_nma` | `BypassCallback` | Callback → NMA MC for explicit sync + magic path |
| `m_ot_counter[flat_bank]` | `int[]` | Phase 3: per-bank Offload Table counter |
| `m_pending_interrupt_batch[rk]` | `int[]` | Phase 3: pending RT batch per rank |

#### Constants
| Constant | Value | Description |
|----------|-------|-------------|
| `HOST_OT_THRESHOLD` | 8 | Phase 3: OT threshold = CMD FIFO size |

#### Public Functions
| Function | Description |
|----------|-------------|
| `init()` | Register YAML params, create scheduler/refresh/rowpolicy children |
| `setup(frontend, memory_system)` | Initialize DRAM, per-rank buffers, row tracking, OT counters, statistics |
| `send(req)` | Enqueue read/write to per-rank buffer; reject if rank in NMA mode |
| `priority_send(req)` | Enqueue refresh; skip if rank in NMA or CONCURRENT mode |
| `tick()` | Main loop: serve_reads → refresh → schedule → OT check → issue → bypass (offload substitution) |
| `set_bypass_callback(cb)` | Wire NMA MC bypass callback (called by AsyncDIMMSystem) |
| `on_nma_interrupt(rank_id, batch_size)` | Phase 3: receive NMA MC interrupt; decrement OT counters |
| `set_mode(rank_id, mode)` | Set per-rank mode (called by system during transitions) |
| `get_mode(rank_id)` | Query current per-rank mode |
| `get_open_row(rank, bg, bk)` | Return currently open row (−1 if closed) |
| `get_counters()` | Return `[host_access_cnt, tcore_host_access_cnt]` |
| `get_req_latency()` | Pop and return next completed read latency |
| `is_finished()` / `is_abs_finished()` | True when all host reads/accesses have departed |
| `finalize()` | Compute avg latency, queue averages, bandwidth stats |

#### Private Functions
| Function | Description |
|----------|-------------|
| `serve_completed_reads()` | Fire callbacks for reads whose `depart <= m_clk` |
| `schedule_request(req_it, buffer)` | FRFCFS: active → priority → read/write (round-robin per rank) |
| `set_write_mode_per_rank(rk)` | Watermark-based RD/WR mode switching per rank |
| `update_rr_rank_idx()` | Rotate round-robin rank index |
| `update_request_stats(req)` | Count row hits/misses/conflicts per request |
| `is_row_hit(req)` / `is_row_open(req)` | Row buffer hit / conflict detection via DRAM model |

#### Phase 3 Concurrent Mode Behavior in `tick()`
1. OT backpressure: if `m_ot_counter[bank] >= HOST_OT_THRESHOLD`, stall issuance for that bank
2. Bypass substitution: ACT→ACTO, PRE→PREO, RD/RDA→RDO, WR/WRA→WRO for CONCURRENT ranks
3. OT increment: `m_ot_counter[bank]++` on each offloaded command
4. OT decrement: `on_nma_interrupt()` decrements OT round-robin across rank's banks

---

### asyncdimm_nma_controller.cpp
`class AsyncDIMMNMAController` — standalone (not IDRAMController)

#### Data Structures (Phase 1-2)
| Member | Type | Description |
|--------|------|-------------|
| `m_bank_fsm[flat_bank_id]` | `BankFSM[]` | Shadow bank FSM: CLOSED / OPENED / REFRESHING + open_row |
| `m_nma_req_buffer[flat_bank_id]` | `deque<NMARequest>[]` | Per-bank REQ FIFO (max 8 per bank) |
| `m_addr_gen_slots` | `vector<AddrGenSlot>` | Active address generation slots (max 8) |
| `m_nma_inst_queue` | `deque<NMAInst_Slot>` | Decoded instruction queue (max 256) |
| `m_nma_program` | `vector<NMAInst_Slot>` | Full program snapshot for LOOP |
| `m_nma_state` | `NMAState` | State machine: IDLE/ISSUE_START/RUN/BAR/WAIT/DONE |
| `m_future_actions` | `vector<FutureAction>` | Scheduled refresh completion events |
| `m_nma_start_pending` | `bool` | Set on ctrl-reg WR; polled by system |

#### Data Structures (Phase 3)
| Member | Type | Description |
|--------|------|-------------|
| `m_cmd_fifo[flat_bank_id]` | `deque<CmdFifoEntry>[]` | Per-bank CMD FIFO (max 8) from offloaded commands |
| `m_arbiter_use_cmd[flat_bank_id]` | `vector<bool>` | H/N Arbiter: true=CMD FIFO, false=REQ FIFO |
| `m_sr_unit[flat_bank_id]` | `SRState[]` | SR Unit: per-bank BSC tracking |
| `m_sr_recovery[flat_bank_id]` | `SRRecovery[]` | SR Unit: pending recovery command per bank |
| `m_return_buffer` | `deque<ReturnEntry>` | Return Unit: offloaded reads ordered by seq_num |
| `m_pending_reads` | `deque<PendingRead>` | Reads issued from CMD FIFO awaiting DRAM completion |
| `m_pending_interrupts` | `deque<PendingInterrupt>` | TDM interrupts deferred by TDM_INTERRUPT_LATENCY |
| `m_interrupt_cb` | `InterruptCallback` | Callback → Host MC `on_nma_interrupt()` |

#### Constants
| Constant | Value | Description |
|----------|-------|-------------|
| `NMA_CLOCK_RATIO` | 4 | NMA tick = every 4 DRAM cycles |
| `NMA_REQ_BUFFER_PER_BANK` | 8 | Max requests per bank REQ FIFO |
| `NMA_INST_QUEUE_MAX` | 1024 | Max instructions in decode queue (8KB SRAM / 8B per NMAInst) |
| `ADDR_GEN_SLOT_MAX` | 8 | Max concurrent address generation slots |
| `COMPUTE_LATENCY_T_*` | 1 | Fixed latency (NMA ticks) for all T_* ops |
| `NMA_CMD_FIFO_SIZE` | 8 | Phase 3: CMD FIFO depth per bank |
| `TDM_INTERRUPT_LATENCY` | 2 | Phase 3: TDM interrupt delay (N_rank/2 for 4 ranks) |
| `NMA_RT_LATENCY` | 20 | Phase 3: RT latency approximation (DRAM cycles) |

#### Public Functions (Phase 1-2)
| Function | Description |
|----------|-------------|
| `init(rank_id, dram, ctrl_row, ctrl_bg, ctrl_bk, buf_bg, buf_bk)` | Initialize shadow FSM, timing, magic-path addresses |
| `set_channel_id(ch_id)` | Set channel ID for address construction |
| `tick()` | Main loop: future actions → pending reads/interrupts → NMA SM (every 4 cycles) |
| `bypass_command(command, addr_vec, payload)` | Phase 1: update shadow FSM + magic-path interception; Phase 3: decode ACTO/RDO/WRO/PREO → CMD FIFO |
| `start_nma_execution()` | Transition IDLE → ISSUE_START (called by system on H2N/H2C) |
| `is_nma_start_pending()` / `consume_nma_start_pending()` | Polling interface for ctrl-reg WR detection |
| `is_nma_complete()` / `is_nma_idle()` | NMA work completion check |
| `get_nma_state()` / `get_nma_outstanding()` | Status queries |
| `get_bank_state(bg, bk)` / `get_open_row(bg, bk)` | Shadow FSM queries |
| `all_banks_closed()` / `any_bank_refreshing()` / `all_req_buffers_empty()` | Bank state checks |
| `print_stats()` / `print_bank_states()` | Statistics / debug output |

#### Public Functions (Phase 3)
| Function | Description |
|----------|-------------|
| `enter_concurrent_mode()` | Clear CMD FIFOs/recovery/return buffer; set arbiter to REQ |
| `exit_concurrent_mode()` | Clear concurrent flag |
| `set_interrupt_callback(cb)` | Wire interrupt → Host MC `on_nma_interrupt()` |
| `receive_rt(batch_size)` | RT received from Host MC (stat tracking) |
| `is_concurrent_complete()` | True when NMA idle + all CMD FIFOs + return buffer + pending interrupts empty |
| `get_cmd_fifo_outstanding()` | Sum of all per-bank CMD FIFO sizes |

#### Private Functions (Phase 2: NMA Mode)
| Function | Description |
|----------|-------------|
| `get_flat_bank_id(addr_vec)` | Compute `bg * num_banks + bk` |
| `handle_future_action(action)` | Process REFAB_END/REFSB_END |
| `decode_nma_inst(raw)` | Decode 64-bit NMAInst from magic-path payload |
| `tick_nma_state_machine()` | Dispatch to current state handler |
| `tick_nma_issue_start()` | Wait for refresh; transition to NMA_RUN |
| `tick_nma_run()` | refresh → issue command → generate requests → fetch instructions |
| `tick_nma_bar()` | BARRIER: drain all outstanding; resume NMA_RUN |
| `tick_nma_wait()` | Fixed-latency compute wait; resume NMA_RUN |
| `tick_nma_done()` | Drain FIFOs; PREA; transition to NMA_IDLE |
| `try_issue_nma_command()` | Round-robin per-bank front scheduling (one cmd per tick) |
| `get_nma_preq_command(req)` | Shadow FSM → prerequisite (ACT/PRE/RD/WR) |
| `update_fsm_on_nma_command(cmd, addr_vec)` | Update shadow FSM on issue |
| `generate_nma_requests()` | AddrGenSlot → per-bank REQ FIFO (one per tick) |
| `fetch_nma_instructions()` | Decode inst: READ/WRITE→AddrGenSlot; NONE→WAIT; CONTROL→BAR/EXIT/LOOP |
| `check_nma_refresh()` / `try_issue_nma_refresh()` | NMA Mode refresh management |
| `issue_nma_prea()` | Issue PREA for N2H implicit sync |

#### Private Functions (Phase 3: Concurrent Mode)
| Function | Description |
|----------|-------------|
| `try_issue_concurrent_command()` | H/N Arbiter: round-robin banks, select CMD or REQ FIFO, switch on empty/PRE |
| `switch_to_cmd_fifo(bank_id)` | Switch arbiter to CMD FIFO with SR recovery (full Fig 5 table: PRE, PRE+ACT, None) |
| `try_issue_from_cmd_fifo(bank_id)` | Issue from CMD FIFO (SR recovery first if pending); schedule read completion |
| `try_issue_from_req_fifo(bank_id)` | Issue from REQ FIFO with prerequisite resolution; switch on PRE |
| `process_pending_reads()` | Mark reads done at `m_clk + nCL + nBL` in return buffer |
| `check_and_send_interrupt()` | Queue TDM interrupt when head of return buffer is done (head-of-line) |
| `deliver_pending_interrupts()` | Fire queued interrupts at `deliver_clk` (TDM_INTERRUPT_LATENCY deferred) |

#### NMA State Machine
```
NMA_IDLE ──(start_nma_execution)──→ NMA_ISSUE_START
NMA_ISSUE_START ──(all banks ready)──→ NMA_RUN
NMA_RUN ──(BARRIER inst)──→ NMA_BAR ──(FIFOs empty)──→ NMA_RUN
NMA_RUN ──(T_* NONE inst)──→ NMA_WAIT ──(latency done)──→ NMA_RUN
NMA_RUN ──(EXIT inst)──→ NMA_DONE ──(FIFOs empty, PREA)──→ NMA_IDLE

In Concurrent Mode: tick_nma_run/bar/wait/done call try_issue_concurrent_command()
instead of try_issue_nma_command() (H/N Arbiter selects CMD or REQ FIFO per bank).
```

---

### asyncdimm_system.cpp
`class AsyncDIMMSystem` — implements `IMemorySystem`

#### Data Structures
| Member | Type | Description |
|--------|------|-------------|
| `m_host_controllers[ch]` | `IDRAMController*[]` | One Host MC per channel |
| `m_nma_controllers[ch][rk]` | `AsyncDIMMNMAController*[][]` | One NMA MC per rank per channel |
| `m_rank_state[ch][rk]` | `SystemNMAState[][]` | Per-rank mode state (7 states) |
| `m_concurrent_mode_enable` | `bool` | Phase 3: YAML `concurrent_mode_enable` flag |
| `m_trace` | `vector<Trace>` | Loaded NMA trace entries |
| `m_outstanding_reads` | `unordered_map<int, OutstandingRequest>` | MSHR for outstanding trace reads |

#### SystemNMAState Enum (7 states)
```
HOST_MODE → TRANSITIONING_H2N → NMA_MODE → TRANSITIONING_N2H → HOST_MODE
HOST_MODE → TRANSITIONING_H2C → CONCURRENT → TRANSITIONING_C2H → HOST_MODE
```

#### Public Functions
| Function | Description |
|----------|-------------|
| `init()` | Create Host/NMA MCs; wire bypass + interrupt callbacks; read YAML config; load trace |
| `send(req)` | Apply address mapping; route to Host MC |
| `tick()` | tick DRAM → Host MCs → NMA MCs → mode transitions → trace requests → FSM sync |
| `is_finished()` / `is_ndp_finished()` | Host MC / NMA MC completion check |
| `mem_sys_finalize()` | Print stats, latency report, FSM sync summary |

#### Private Functions
| Function | Description |
|----------|-------------|
| `tick_mode_transitions()` | Per-rank 7-state machine (handles all transitions) |
| `tick_transition_h2n(ch, rk)` | HOST→NMA: set Host MC mode=NMA, start NMA execution |
| `tick_nma_mode(ch, rk)` | NMA_MODE: poll is_nma_complete() |
| `tick_transition_n2h(ch, rk)` | NMA→HOST: wait NMA idle, restore Host MC mode=HOST |
| `tick_transition_h2c(ch, rk)` | Phase 3: HOST→CONCURRENT: set CONCURRENT, start NMA, enter concurrent |
| `tick_concurrent_mode(ch, rk)` | Phase 3: CONCURRENT: poll is_concurrent_complete() |
| `tick_transition_c2h(ch, rk)` | Phase 3: CONCURRENT→HOST: exit concurrent, restore Host MC mode=HOST |
| `try_issue_trace_requests()` | Issue trace entries via send() (MSHR-bounded) |
| `on_read_complete(req_id, req)` | Remove completed read from MSHR |
| `load_trace(path, trace_vec)` | Parse trace file: `[TIMESTAMP] LD/ST ADDR [8×PAYLOAD]` |
| `verify_fsm_sync()` | Compare Host MC vs NMA MC bank states (debug mode) |
| `record_latency(lat)` / `report()` | Latency histogram and percentile report |

#### Interrupt Callback Wiring (Phase 3)
```
init(): if (concurrent_mode_enable)
  nma_mc->set_interrupt_callback([host_mc](rank_id, batch_size) {
    host_mc->on_nma_interrupt(rank_id, batch_size);
  });
```
