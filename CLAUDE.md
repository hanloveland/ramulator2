# Ramulator2 - Forked DRAM Simulator

Forked from Ramulator 2.0 to customize DRAM structure (DDR5-DIMM with pseudochannel, NDP capability).

## Runtime Environment

- Docker container: `my_linx:gem5`
- Ramulator2 path (inside container): `/home/mklee/sim_env/ramulator2`

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

---

### Future Tasks

- [ ] **Concurrent Mode host access 성능 저하 분석**: Host + NMA 동시 실행 시 host read latency 및 throughput 저하 원인 분석. OT backpressure, CMD/REQ FIFO 경합, row-hit low cap, arbiter switching 빈도, refresh 간섭 등 병목 요소 식별 및 최적화.

---

## DBX-DIMM NDP Launch Descriptor Optimization

### Problem Statement

현재 DBX-DIMM의 Host-Side NDP Controller (HSNC)는 NDP Launch Descriptor에 **full address (38-bit)를 매 descriptor마다** 인코딩하여 두 가지 근본적 문제가 발생한다:

1. **Descriptor 수 폭발**: 순차 접근 워크로드(COPY 1MB = 128 rows)에서 256+ descriptors 필요
2. **HOL Blocking**: 공유 DIMM-level buffer(`dimm_lvl_req_buffer`)에서 head의 target PCH slot이 full이면 모든 PCH가 stall

#### 현재 HSNC Pipeline (문제 구조)

```
dimm_lvl_req_buffer[dimm] (1024, 공유, FIFO 순차)
    → pch_lvl_hsnc_nl_req_slot[dimm][pch] (16, head-of-queue만 pop 가능)
        → pch_lvl_hsnc_nl_addr_gen_slot[dimm][pch] (8, decoded AccInst_Slot)
            → send_ndp_req_to_mc() → MC read/write buffer
```

- `dimm_lvl_req_buffer`: PCH 구분 없이 순차 저장, head의 target PCH만 서비스 가능
- PCH A의 NL-REQ slot이 full → DIMM buffer 전체 stall → PCH B/C/D도 blocked

#### 재정의된 AccInst_Slot (64-bit) — Direct/Undirect Mode 통합

**RD/WR Descriptor (데이터 접근 명령)**:
```
[63:60] opcode  4b    ← RD, WR, BARRIER, WAIT, SET_BASE, INC_BASE, SET_LOOP, LOOP, DONE
[59]    mode    1b    ← 0=Direct, 1=Undirect (opcode 바로 다음)
[58:52] opsize  7b    ← column 접근 횟수
[51:46] ch      6b    ┐
[45:44] pch     2b    │ 주소 필드 (38b)
[43:41] bg      3b    │
[40:39] bk      2b    │
[38:21] row    18b    │ ← Direct: 절대 row
                      │    Undirect: [17:15]=base_reg_idx(3b), [14:0]=offset(15b)
[20:14] col     7b    ┘
[13:11] id      3b    ← DBX 내부 Thread ID (base_reg_idx와 별개)
[10:0]  etc    11b    ← 제어
```

**기존 대비 변경점**:
- `mode` 1b를 bit[59] (opcode 바로 다음)로 이동 (기존 opsize [59:53] → [58:52]로 1b 하향)
- `id`(3b)는 DBX 내부 Thread ID로 유지 (base_reg_idx 용도가 아님)
- Undirect mode의 base_reg_idx는 row 필드 상위 3b [17:15]에 위치
- `etc` 12b → 11b (mode 1b 추가분)

**Mode에 따른 row 해석**:
- **Direct (mode=0)**: `row[17:0]` = 절대 row address (기존과 동일)
- **Undirect (mode=1)**: `row[17:15]` = base_reg index (0~7), `row[14:0]` = offset (15b)
  - `effective_row = base_reg[row[17:15]] + row[14:0]`
- ch, pch, bg, bk, col은 mode와 무관하게 항상 descriptor에서 직접 사용

**SET_BASE Descriptor** (제어 명령, 주소 필드 불필요):
```
[63:60] opcode = SET_BASE   4b
[59]    mode   (don't care) 1b
[58:56] reg_idx             3b  ← target base_reg index (0~7)
[55:38] base_value         18b  ← base_reg에 저장할 row 값
[37:0]  don't care         38b
```
- `base_reg[reg_idx] = base_value`

**INC_BASE Descriptor** (제어 명령, 주소 필드 불필요):
```
[63:60] opcode = INC_BASE   4b
[59]    mode   (don't care) 1b
[58:56] reg_idx             3b  ← target base_reg index (0~7)
[55:38] inc_value          18b  ← stride increment 값
[37:0]  don't care         38b
```
- `base_reg[reg_idx] += inc_value`

**SET_LOOP Descriptor** (제어 명령, Loop Counter Register 초기화):
```
[63:60] opcode = SET_LOOP   4b
[59]    mode   (don't care) 1b
[58:56] cnt_reg_idx         3b  ← target Loop Counter Register (0~7)
[55:40] loop_count         16b  ← 초기 iteration count (max 65535)
[39:0]  don't care         24b
```
- `loop_cnt_reg[cnt_reg_idx] = loop_count`

**LOOP Descriptor** (제어 명령, Loop Counter Register 참조):
```
[63:60] opcode = LOOP       4b
[59]    mode   (don't care) 1b
[58:56] cnt_reg_idx         3b  ← Loop Counter Register index (0~7)
[55:40] jump_pc            16b  ← target PC (max 65535)
[39:0]  don't care         24b
```
- `if loop_cnt_reg[cnt_reg_idx] > 0: loop_cnt_reg[cnt_reg_idx]--, PC = jump_pc`
- `else: PC++ (fall through)`

**WAIT Descriptor** (대기 명령):
```
[63:60] opcode = WAIT       4b
[59:11] don't care          49b
[10:0]  wait_cycle          11b  ← 대기 cycle 수 (max 2047)
```
- NDP_WAIT 상태 전환, wait_cycle 만큼 대기 후 NDP_RUN 복귀

**Loop Counter Register 설계 배경**:
- 기존 FIFO 방식에서는 소비된 descriptor로 "jump back" 불가 → PC 기반 접근으로 해결
- 제한된 SRAM 크기 — loop_cnt를 descriptor 내 inline 저장 불가능
- 해결: HSNC에 **Loop Counter Register [8] × 16b** (3b 인덱스) 하드웨어 추가
- HSNC에 PC (program counter) 추가, desc_store + pch_lvl_inst_buf (Descriptor Cache) 구조
- SRAM 비용: 8 × 16b = 16B per PCH (무시 가능)

**HSNC Register File 요약**:
| Register | 개수 | Width | 인덱스 | 용도 |
|----------|------|-------|--------|------|
| `base_reg[]` | 8 | 18b | 3b (row[17:15] 또는 SET_BASE/INC_BASE [58:56]) | Row base address |
| `loop_cnt_reg[]` | 8 | 16b | 3b (SET_LOOP/LOOP [58:56]) | Loop iteration counter |
| `PC` | 1 | 10b | — | Program counter (desc_store 내 위치, [9:3]=col, [2:0]=idx) |

**Opcode 요약**:

| Opcode | Value | Name | mode 적용 | 필드 해석 | 동작 |
|--------|-------|------|-----------|-----------|------|
| 기존 | 0 | RD | O | 표준 레이아웃 | Direct: 절대 주소 RD, Undirect: base_reg[row[17:15]]+row[14:0] |
| 기존 | 1 | WR | O | 표준 레이아웃 | Direct: 절대 주소 WR, Undirect: base_reg[row[17:15]]+row[14:0] |
| 기존 | 2 | BARRIER | X | (don't care) | 모든 outstanding request 완료 대기 |
| 기존 | 6 | WAIT | X | [10:0]=wait_cycle (11b) | etc 필드 cycle 수 만큼 대기 |
| 기존 | 15 | DONE | X | (don't care) | NDP 실행 종료 |
| NEW | 8 | SET_BASE | X | [58:56]=reg_idx, [55:38]=value | `base_reg[reg_idx] = base_value` |
| NEW | 9 | INC_BASE | X | [58:56]=reg_idx, [55:38]=value | `base_reg[reg_idx] += inc_value` |
| NEW | 10 | SET_LOOP | X | [58:56]=cnt_reg, [55:40]=count | `loop_cnt_reg[cnt_reg] = count` |
| NEW | 11 | LOOP | X | [58:56]=cnt_reg, [55:40]=jump_pc | cnt > 0 → cnt--, PC=jump_pc |

주요 코드: `ndp_DRAM_system.cpp` — `decode_acc_inst()` (L1175-1191), `send_ndp_req_to_mc()` (L1198-1261), `send_ndp_ctrl()` (L967-1043), DIMM-level buffer dispatch (L652-691)

### Optimization Design

#### Phase 1: Per-PCH Descriptor Buffer + desc_store (HOL Blocking 해소)

**현재 → 제안 구조**:
```
현재: dimm_lvl_req_buffer[dimm] (1024, 공유 FIFO)
       → pch_lvl_hsnc_nl_req_slot[dimm][pch] (16)
          → decode_acc_inst() → pch_lvl_hsnc_nl_addr_gen_slot[dimm][pch] (8)
             → send_ndp_req_to_mc()

제안: desc_store[dimm][pch][128][8]         (HSNC 내부 backing array, 전체 프로그램 저장)
       → pch_lvl_inst_buf[dimm][pch][8][8]  (Descriptor Cache: 8 entry groups × 8 = 64, PC 기반 접근)
          → decode_acc_inst() → pch_lvl_hsnc_nl_addr_gen_slot[dimm][pch] (8)
             → send_ndp_req_to_mc()
```

- **desc_store[dimm][pch][128][8]**: HSNC 내부 배열, 최대 1024 descriptors (128 cols × 8 entries/col) 전량 보관
- **pch_lvl_inst_buf[8][8]**: Descriptor Cache로 운용 (8 entry groups × 8 entries = 64 entries)
  - 각 Entry Group Tag = Column Address (7b), fully-associative
  - PC가 가리키는 descriptor를 pch_lvl_inst_buf에서 fetch → decode_acc_inst()로 직접 전달
- **nl_req_slot 삭제**: pch_lvl_inst_buf가 nl_req_slot 역할 흡수, decode에 직접 공급
- **Write Intercept**: Host AccInst write → DRAM timing check만 수행, 실제 데이터는 desc_store에 저장
  - Col 0~7 write 시 pch_lvl_inst_buf에도 동시 적재 (초기 64 entries)
  - DRAM에는 data 저장 안 함 (timing 모델로만 활용)
- **NDP Start**: per-PCH start flag(1b) + desc_count(16b), AccInst 전량 write 후 마지막에 발행
  - Payload: `bit[16]=start_flag, bits[15:0]=desc_count`
- Per-PCH 순차 AccInst stream (interleaved dispatch 제거)
- HOL blocking 완전 해소: 한 PCH가 stall되어도 다른 PCH 독립 동작
- SRAM 비용: desc_store (1024 × 64b × num_PCH) + buffer (64 × 64b × num_PCH)
  = 4 PCH 기준: 32KB (desc_store) + 2KB (buffer) = 34KB

변경 대상:
- `ndp_DRAM_system.cpp`: `dimm_lvl_req_buffer` + `nl_req_slot` → `desc_store[dimm][pch]` + `pch_lvl_inst_buf[dimm][pch][8][8]`
- `send_ndp_ctrl()`: Write Intercept (모든 AccInst → desc_store, Col 0~7 → pch_lvl_inst_buf 동시 적재)
- `tick()`: PC 기반 fetch (pch_lvl_inst_buf → decode_acc_inst() → addr_gen_slot), nl_req_slot 경로 제거
- `request.h`: NDP Start payload에 desc_count 필드 추가

#### Phase 2: Direct / Undirect Access Mode

AccInst_Slot bit[59] (opcode 바로 다음)에 mode flag를 추가하여 두 가지 row 주소 지정 방식을 지원한다.
제어 명령 (SET_BASE, INC_BASE, LOOP)은 주소 필드가 불필요하므로 **opcode별 독립 필드 해석**을 사용한다.

**Direct Access Mode** (mode=0, 기존과 동일):
- 모든 주소 필드 (ch, pch, bg, bk, row, col)가 절대 주소
- 임의 접근 패턴, cross-bank/cross-PCH 접근에 사용

**Undirect Access Mode** (mode=1, 신규):
- HSNC에 `base_reg[8]` (per PCH) 유지, SET_BASE 명령으로 설정
- row[17:15] = base_reg index (3b), row[14:0] = offset (15b)
- `effective_row = base_reg[row[17:15]] + row[14:0]`
- ch, pch, bg, bk, col은 descriptor에서 직접 사용 (Direct와 동일)
- `id`(3b)는 Thread ID로 유지 (base_reg_idx와 별개)
- 순차/stride 접근 패턴에 최적: INC_BASE + LOOP 조합

**PC 구조 (10-bit, desc_store 연동)**:
- PC[9:3] = Column Address (7b, 0~127), PC[2:0] = Entry Index (3b, 0~7)
- 총 PC 범위: 0 ~ 1023, desc_store 공간 전체를 주소 지정
- PC가 가리키는 descriptor를 pch_lvl_inst_buf에서 fetch → decode_acc_inst()로 직접 전달
- decode → addr_gen_slot → send_ndp_req_to_mc() (nl_req_slot 삭제)
- **순차 실행**: PC 증가 시 다음 Entry Group Column이 buffer에 없으면 Cache Miss → DRAM RD (timing) + desc_store에서 buffer 적재
- **LOOP jump**: jump_pc의 Column Address가 buffer Tag에 없으면 Cache Miss → DRAM RD + oldest group evict
- **Cache Hit**: buffer에서 직접 fetch (DRAM 접근 없음)

**`decode_acc_inst()` 확장** (L1175-1191):
```cpp
AccInst_Slot decode_acc_inst(uint64_t inst) {
  uint64_t opcode = (inst >> 60) & 0xf;
  uint64_t mode   = (inst >> 59) & 0x1;     // bit[59], opcode 바로 다음

  if (opcode == SET_BASE) {
    uint64_t reg_idx    = (inst >> 56) & 0x7;   // [58:56]
    uint64_t base_value = (inst >> 38) & 0x3FFFF; // [55:38] 18b
    base_reg[reg_idx] = base_value;
    return;  // 제어 명령
  }
  if (opcode == INC_BASE) {
    uint64_t reg_idx   = (inst >> 56) & 0x7;   // [58:56]
    uint64_t inc_value = (inst >> 38) & 0x3FFFF; // [55:38] 18b
    base_reg[reg_idx] += inc_value;
    return;  // 제어 명령
  }
  if (opcode == SET_LOOP) {
    uint64_t cnt_reg_idx = (inst >> 56) & 0x7;    // [58:56]
    uint64_t loop_count  = (inst >> 40) & 0xFFFF;  // [55:40] 16b
    loop_cnt_reg[cnt_reg_idx] = loop_count;
    return;  // 제어 명령
  }
  if (opcode == LOOP) {
    uint64_t cnt_reg_idx = (inst >> 56) & 0x7;    // [58:56]
    uint64_t jump_pc     = (inst >> 40) & 0xFFFF;  // [55:40] 16b
    if (loop_cnt_reg[cnt_reg_idx] > 0) {
      loop_cnt_reg[cnt_reg_idx]--;
      PC = jump_pc;  // jump back to loop body start
    } else {
      PC++;  // fall through to next descriptor
    }
    return;
  }

  // RD/WR — 표준 레이아웃
  uint64_t opsize = (inst >> 52) & 0x7f;   // [58:52]
  uint64_t ch     = (inst >> 46) & 0x3f;   // [51:46]
  uint64_t pch    = (inst >> 44) & 0x3;    // [45:44]
  uint64_t bg     = (inst >> 41) & 0x7;    // [43:41]
  uint64_t bk     = (inst >> 39) & 0x3;    // [40:39]
  uint64_t row    = (inst >> 21) & 0x3FFFF; // [38:21]
  uint64_t col    = (inst >> 14) & 0x7F;   // [20:14]
  uint64_t id     = (inst >> 11) & 0x7;    // [13:11] Thread ID
  uint64_t etc    = (inst      ) & 0x7FF;  // [10:0] 11b

  if (mode == 1) {  // Undirect
    uint64_t reg_idx = (row >> 15) & 0x7;   // row[17:15]
    uint64_t offset  = row & 0x7FFF;        // row[14:0]
    row = base_reg[reg_idx] + offset;        // effective_row
  }
  return AccInst_Slot(true, opcode, opsize, ch, pch, bg, bk, row, col, id, mode, etc);
}
```

참조 구현: AsyncDIMM `m_base_reg[8]`, `SET_BASE`/`INC_BASE` in `asyncdimm_nma_controller.cpp` (L124-127, L1862-1876)

#### Phase 3: DRAM Spill (Descriptor Cache)

GEMV 등 복잡한 워크로드는 Undirect + LOOP 최적화 후에도 descriptor ~600개 (내부 Matrix MAC 루프 unroll). 64-entry on-chip buffer에 수용 불가 → DRAM을 backing store로 활용.

**개요**: pch_lvl_inst_buf를 8-entry fully-associative **Descriptor Cache**로 운용. 각 cache line = 1 DRAM column = 8 descriptors (64B). DRAM에 전체 프로그램 저장, on-chip buffer는 working set만 캐싱.

**DRAM Descriptor 저장 영역**:
```
Address: (ch, pch, ndp_ctrl_buf_bg, ndp_ctrl_buf_bk, ndp_ctrl_row, col)
Col 0..127 → PC 0..1023 (최대 1024 descriptors)
각 Column = 8 descriptors (64B cache line)
```

**PC 구조 (10-bit)**:
```
PC[9:3] = Column Address (7b, 0~127)
PC[2:0] = Entry Index within Column (3b, 0~7)
총 PC 범위: 0 ~ 1023
```

**pch_lvl_inst_buf 구조 (Descriptor Cache)**:
```
8 Entry Groups × 8 entries/group = 64 entries
각 Entry Group = 1 DRAM Column 크기 (64B)
각 Entry Group에 Tag 저장 = 해당 Column Address (7b)
```

| 구성요소 | 크기 | 설명 |
|----------|------|------|
| Entry Group | 8개 | 각 64B (8 descriptors) |
| Tag per Group | 7b | Column Address (PC[9:3]) |
| 총 용량 | 64 entries (512B) | 8 groups × 8 entries |

**시뮬레이터 구현 모델 (DRAM Timing 활용)**:

AccInst는 실제 DRAM에 저장되지 않고, HSNC 내부 별도 array(`desc_store[]`)에 전량 보관.
DRAM은 **timing 모델로만** 활용 — write/read 타이밍 제약을 정확히 반영하되, 실제 데이터는 `desc_store`에서 관리.

```
Host ST (AccInst Write)
  │
  ├─→ DRAM: timing check (tRCD, tWR, tCCD 등) — 실제 data 저장 안 함
  │
  └─→ HSNC: 모든 AccInst를 intercept → desc_store[pch][col][idx]에 저장
       └─ Col 0~7은 추가로 pch_lvl_inst_buf (Descriptor Cache)에도 적재

HSNC Prefetch (Cache Miss 시)
  │
  ├─→ DRAM: RD 명령 발행 (MC read buffer 경유, timing 체크)
  │
  └─→ RD 응답 도착 시점에 desc_store[pch][target_col]에서 데이터 읽어 buffer에 적재
       (DRAM 응답을 timing trigger로 사용, 실제 data는 desc_store에서 공급)
```

| 동작 | DRAM 역할 | 실제 데이터 소스 |
|------|-----------|------------------|
| AccInst Write | timing check (WR latency) | HSNC `desc_store[]` 에 저장 |
| AccInst Prefetch | timing check (RD latency) | RD 응답 시점에 `desc_store[]` 에서 읽기 |
| pch_lvl_inst_buf Hit | (없음) | buffer에서 직접 실행 |

**초기 적재 (Write Intercept)**:
1. Host가 AccInst를 DRAM magic-path 주소로 ST write (기존과 동일 경로)
2. HSNC는 **모든 AccInst write**를 intercept → `desc_store[pch][col][idx]`에 저장
3. 추가로 **Col 0~7** (PC 0~63)은 pch_lvl_inst_buf에도 직접 적재
4. DRAM에는 timing check만 수행 (실제 data write 없음)
5. 결과: 실행 시작 시 buffer에 처음 64개 descriptor 적재 + 전체 프로그램이 `desc_store`에 보관

**순차 실행 (PC 증가)**:
1. PC 0부터 순차 실행, 현재 PC의 Column Address와 buffer 내 Entry Group Tag 비교
2. **Cache Hit**: Tag 일치 → buffer에서 직접 실행 (DRAM 접근 없음)
3. **Cache Miss** (column address mismatch): PC가 buffer에 없는 Column으로 진입
   - HSNC가 해당 Column에 대해 DRAM RD 발행 (MC 경유, timing 체크)
   - RD 응답 도착 시 `desc_store[pch][target_col]`에서 8 entries 읽어 buffer에 적재
   - 가장 오래된 Entry Group을 evict → 새 Column 데이터로 교체
4. 순차 실행에서는 마지막 Entry Group 실행 완료 시 **다음 Column 자동 prefetch**

**LOOP/Jump 실행**:
1. LOOP 명령 → jump_pc의 Column Address와 buffer Tag 비교
2. **Hit**: jump target이 buffer 내 → 즉시 PC 이동 (DRAM 접근 없음)
3. **Miss**: jump target이 buffer 밖 → DRAM RD 발행 → 응답 시 `desc_store`에서 적재 후 PC 이동

**LOOP 최적화 시나리오**:
- Loop body ≤ 64 entries (8 columns): 첫 iteration 이후 buffer에 완전 캐싱 → DRAM 접근 0회
- Loop body > 64 entries (예: GEMV ~600): 매 iteration마다 (body_cols - 8)회 DRAM RD 필요
  - 그러나 Host가 매 iteration descriptor를 재전송하는 것 대비 대폭 효율적
  - DRAM RD는 64B 단위, MC read buffer 경유 → host request와 경합 가능

**GEMV 예시** (x4, 128K, body ~596 entries):
```
Total columns used: ⌈596/8⌉ = 75 columns
Buffer: 8 entry groups
Per iteration DRAM reads: 75 - 8 = 67 reads (64B each)
iteration_tile_block = 64 iterations
Total DRAM reads: 67 × 64 = 4,288 reads (각 64B)
vs Direct mode: 149,185 descriptors를 Host에서 전송
```

### AccInst → NDP DRAM Request 생성 Flow

Phase 1-3의 설계를 통합한 전체 NDP 실행 파이프라인.

#### 전체 파이프라인 구조

```
[Host AccInst Write]
  │ ST to (ch, pch, ndp_ctrl_buf_bg/bk, ndp_ctrl_row, col)
  │
  ▼
[HSNC Write Intercept]  ── send_ndp_ctrl()
  │
  ├─→ desc_store[dimm][pch][col][idx]   (전량 저장, 최대 128 col × 8 entry = 1024)
  │
  └─→ pch_lvl_inst_buf[dimm][pch][8][8] (Col 0~7만 동시 적재, 초기 64 entries)
      DRAM: timing check만 (data 저장 안 함)

[NDP Start Write]  ── send_ndp_ctrl() (AccInst 전량 write 후 마지막)
  │ Payload: bit[16]=start_flag, bits[15:0]=desc_count per PCH
  │
  ▼
[NDP Execution]  ── tick() per PCH
  │
  ▼
[PC Fetch]  ── PC(10b) → pch_lvl_inst_buf cache lookup
  │
  ├─ Cache Hit:  buffer에서 descriptor fetch (DRAM 접근 없음)
  │
  └─ Cache Miss: DRAM RD 발행 (timing) → 응답 시 desc_store[col]에서 8 entries 적재
                 oldest entry group evict, fetch stall
  │
  ▼
[decode_acc_inst()]  ── opcode별 분기
  │
  ├─ RD/WR ──→ [pch_lvl_hsnc_nl_addr_gen_slot] (max 8, decoded AccInst_Slot)
  │              │
  │              ▼
  │            [send_ndp_req_to_mc()]  ── round-robin, 1 DRAM req/cycle
  │              │
  │              ├─ cnt < opsize: col++, cnt++ (다음 cycle 계속)
  │              └─ cnt == opsize: slot에서 제거
  │
  ├─ BAR ────→ NDP_BAR state (모든 in-flight request 완료 대기 → NDP_RUN)
  │
  ├─ WAIT ───→ NDP_WAIT state (etc 필드 cycle 수 대기 → NDP_RUN)
  │
  ├─ SET_BASE → base_reg[reg_idx] = value, PC++ (제어 명령, no DRAM req)
  ├─ INC_BASE → base_reg[reg_idx] += value, PC++ (제어 명령, no DRAM req)
  ├─ SET_LOOP → loop_cnt_reg[cnt_reg] = count, PC++ (제어 명령, no DRAM req)
  ├─ LOOP ───→ cnt > 0: cnt--, PC = jump_pc (buffer miss 가능)
  │             cnt == 0: PC++ (fall through)
  │
  └─ DONE ───→ NDP_DONE state (in-flight drain → NDP_IDLE)
```

#### Stage 1: Host AccInst Write → desc_store + pch_lvl_inst_buf

```cpp
// send_ndp_ctrl() 내 Write Intercept
// req.addr_vec에서 dimm_id, pch_id, col 추출
int col = req.addr_vec[m_dram->m_levels("column")];

// 1. desc_store에 전량 저장 (8 entries per write = 1 DRAM column)
for (int i = 0; i < 8; i++)
    desc_store[dimm_id][pch_id][col][i] = req.m_payload[i];

// 2. Col 0~7이면 pch_lvl_inst_buf에도 동시 적재
if (col < 8)
    pch_lvl_inst_buf[dimm_id][pch_id][col] = desc_store[dimm_id][pch_id][col];
    // entry_group_tag[col] = col  (tag 설정)

// 3. DRAM: timing check만 수행 (기존 MC 경유 WR timing)
```

- Host trace에서 PCH별 순차 stream으로 write (interleave 없음)
- AccInst 전량 write 완료 후 NDP Start 발행

#### Stage 2: NDP Start → NDP_RUN 전환

```cpp
// send_ndp_ctrl() 내 NDP Start 처리
// NDP Control Register write (ndp_ctrl_bg/bk)
for (int i = 0; i < num_pch; i++) {
    if (req.m_payload[i] != 0) {
        int desc_count = req.m_payload[i] & 0xFFFF;      // bits[15:0]
        bool start_flag = (req.m_payload[i] >> 16) & 0x1; // bit[16]
        pch_desc_count[dimm_id][i] = desc_count;
        pch_lvl_hsnc_status[dimm_id][i] = NDP_ISSUE_START;
        PC[dimm_id][i] = 0;  // PC 초기화
    }
}
```

| 현재 상태 | 조건 | 다음 상태 | 동작 |
|-----------|------|-----------|------|
| NDP_IDLE | NDP Start 수신 | NDP_ISSUE_START | desc_count 기록, PC=0 |
| NDP_ISSUE_START | AccInst write DRAM timing 완료 | NDP_RUN | 실행 시작 |

- 현재 구현: NDP_ISSUE_START에서 DRAM write가 MC queue에서 완료 대기 후 NDP_BEFORE_RUN → NDP_RUN
- 제안: desc_store에 이미 데이터 있으므로, DRAM write timing 완료 확인 후 바로 NDP_RUN 전환 가능

#### Stage 3: PC Fetch → decode_acc_inst()

```cpp
// tick() 내 NDP_RUN 처리
// 조건: addr_gen_slot에 빈 자리 있을 때
if (pch_lvl_hsnc_nl_addr_gen_slot[dimm_id][pch_id].size() < addr_gen_slot_max) {
    int col_addr = PC[dimm_id][pch_id] >> 3;   // PC[9:3]
    int idx      = PC[dimm_id][pch_id] & 0x7;  // PC[2:0]

    // Cache Hit 체크: pch_lvl_inst_buf entry group tag 비교
    int group_idx = find_entry_group(dimm_id, pch_id, col_addr);

    if (group_idx < 0) {
        // Cache Miss → DRAM RD 발행, stall until response
        issue_desc_fetch(dimm_id, pch_id, col_addr);  // DRAM RD (timing)
        // RD 응답 시: desc_store[pch_id][col_addr] → buffer, oldest evict
        return;  // 이번 cycle은 fetch 불가
    }

    // Cache Hit → descriptor fetch
    uint64_t inst = pch_lvl_inst_buf[dimm_id][pch_id][group_idx][idx];
    AccInst_Slot decoded = decode_acc_inst(inst);

    // Opcode별 분기
    switch (decoded.opcode) {
        case RD: case WR:
            pch_lvl_hsnc_nl_addr_gen_slot[dimm_id][pch_id].push_back(decoded);
            PC[dimm_id][pch_id]++;
            break;
        case BAR:
            pch_lvl_hsnc_status[dimm_id][pch_id] = NDP_BAR;
            PC[dimm_id][pch_id]++;
            break;
        case WAIT:
            pch_lvl_hsnc_status[dimm_id][pch_id] = NDP_WAIT;
            wait_cycle = decoded.etc;
            PC[dimm_id][pch_id]++;
            break;
        case SET_BASE: case INC_BASE: case SET_LOOP:
            // 제어 명령: register 업데이트, PC++ (decode 내부에서 처리)
            PC[dimm_id][pch_id]++;
            break;
        case LOOP:
            // decode_acc_inst() 내부에서 처리 완료:
            //   cnt > 0: cnt--, PC = jump_pc (다음 cycle에 target col cache hit/miss 체크)
            //   cnt == 0: PC++ (fall through)
            break;
        case DONE:
            pch_lvl_hsnc_status[dimm_id][pch_id] = NDP_DONE;
            break;
    }
}
```

**PC 증가 규칙**:

| Opcode | PC 동작 | DRAM 요청 |
|--------|---------|-----------|
| RD/WR | PC++ | addr_gen_slot에 enqueue |
| BAR | PC++ | (없음, drain 대기) |
| WAIT | PC++ | (없음, cycle 대기) |
| SET_BASE | PC++ | (없음) |
| INC_BASE | PC++ | (없음) |
| SET_LOOP | PC++ | (없음) |
| LOOP (cnt>0) | PC = jump_pc | (없음, buffer miss 가능) |
| LOOP (cnt==0) | PC++ | (없음) |
| DONE | (정지) | (없음, drain 대기) |

**Fetch Rate**: 최대 1 descriptor/cycle

**Fetch 조건**: addr_gen_slot에 빈 자리가 있거나, 제어 명령(SET_BASE, INC_BASE, SET_LOOP, LOOP)인 경우.
제어 명령은 addr_gen_slot을 사용하지 않으므로 addr_gen_slot full 상태에서도 fetch 가능.
단, opcode를 알려면 먼저 fetch해야 하므로 구현 시 1-entry latch 또는 peek 필요.

#### Stage 4: addr_gen_slot → send_ndp_req_to_mc()

기존 `send_ndp_req_to_mc()` 로직 유지 (변경 없음).

```cpp
// Round-robin across addr_gen_slot entries (max 8)
for (int i = 0; i < addr_gen_slot.size(); i++) {
    int slot_idx = (i + rr_idx) % addr_gen_slot.size();
    AccInst_Slot& slot = addr_gen_slot[slot_idx];

    Request req(0, slot.opcode == RD ? Request::Type::Read : Request::Type::Write);
    // addr_vec에 ch, pch, bg, bk, row, col 매핑 (Undirect mode 시 row는 이미 resolve됨)
    req.addr_vec = {slot.ch, slot.pch, slot.bg, slot.bk, slot.row, slot.col};
    req.is_ndp_req = true;

    if (m_controllers[ch_id]->send(req)) {
        if (slot.cnt == slot.opsize) {
            addr_gen_slot.erase(slot_idx);  // 완료 → 제거
        } else {
            slot.cnt++;
            slot.col++;   // 다음 column
            rr_idx++;
        }
        break;  // 1 DRAM request per cycle
    }
}
```

| 속성 | 값 |
|------|-----|
| Issue Rate | 1 DRAM request / cycle |
| Scheduling | Round-robin across addr_gen_slot entries |
| Column Iteration | cnt++, col++ per cycle until cnt == opsize |
| Slot 제거 | cnt == opsize 시 addr_gen_slot에서 erase |

#### NDP State Machine (제안)

```
┌──────────┐
│ NDP_IDLE │◄──────────────────────────────────────┐
└────┬─────┘                                       │
     │ NDP Start (desc_count, PC=0)                │
     ▼                                             │
┌──────────────────┐                               │
│ NDP_ISSUE_START  │  AccInst write DRAM timing 완료 대기
└────┬─────────────┘                               │
     │ DRAM WR timing settled                      │
     ▼                                             │
╔═══════════╗  PC fetch → decode → addr_gen_slot   │
║  NDP_RUN  ║◄──────────────────────┐              │
╚═══╤═══╤═══╝                       │              │
    │   │                           │              │
    │   ├─ RD/WR: stay (enqueue)    │              │
    │   ├─ SET_BASE/INC_BASE: stay  │              │
    │   ├─ SET_LOOP: stay           │              │
    │   ├─ LOOP: stay (PC jump/++)  │              │
    │   │                           │              │
    │   ├─ BAR ──→ NDP_BAR ─────────┘ (drain)      │
    │   ├─ WAIT ─→ NDP_WAIT ────────┘ (cycle wait) │
    │   └─ DONE ─→ NDP_DONE ──────────────────────┘ (drain → IDLE)
    │
    └─ Cache Miss → NDP_FETCH_STALL ─→ NDP_RUN (DRAM RD 응답 후)
```

| 상태 | 진입 조건 | 동작 | 전환 조건 | 다음 상태 |
|------|-----------|------|-----------|-----------|
| NDP_IDLE | 초기 / DONE 완료 | 대기 | NDP Start 수신 | NDP_ISSUE_START |
| NDP_ISSUE_START | NDP Start | desc_count, PC=0 설정 | DRAM WR timing 완료 | NDP_RUN |
| NDP_RUN | 실행 중 | PC fetch + decode + issue | opcode별 분기 | BAR/WAIT/DONE/FETCH_STALL |
| NDP_BAR | BAR 명령 | addr_gen_slot drain + in-flight 완료 | all drained | NDP_RUN |
| NDP_WAIT | WAIT 명령 | etc cycle 대기 | wait_cnt == wait_cycle | NDP_RUN |
| NDP_FETCH_STALL | Cache Miss | DRAM RD 발행, 응답 대기 | RD 응답 도착 + buffer 적재 | NDP_RUN |
| NDP_DONE | DONE 명령 | addr_gen_slot drain + in-flight 완료 | all drained | NDP_IDLE |

**기존 대비 변경점**:
- NDP_BEFORE_RUN 제거 → NDP_ISSUE_START에서 DRAM WR timing 후 직접 NDP_RUN
- **NDP_FETCH_STALL 추가**: Descriptor Cache miss 시 DRAM RD 응답 대기
- NDP_RUN에서 제어 명령 (SET_BASE, INC_BASE, SET_LOOP, LOOP) 처리 추가
- NDP_WAIT_RES 제거 (미구현 상태, 불필요)

### Descriptor 절약 효과 예측

COPY 1MB 예시 (DDR5 x4, row=8KB, opsize=128 cols):

| 방식 | Descriptor 수 | 비고 |
|------|---------------|------|
| 현재 (Direct Only) | ~258 | 128 RD + 128 WR + BAR + DONE |
| Undirect + LOOP | ~8 | SET_BASE×2 + RD(mode=1) + INC_BASE + WR(mode=1) + INC_BASE + LOOP + DONE |
| 절감률 | **96.5%** | 64-entry buffer에 완전 수용 가능, DRAM spill 불필요 |

### Implementation Plan

```
Phase 1: Per-PCH Buffer + desc_store
  ├── dimm_lvl_req_buffer + nl_req_slot → desc_store[dimm][pch][128][8] + pch_lvl_inst_buf[dimm][pch][8][8]
  ├── send_ndp_ctrl(): Write Intercept (모든 AccInst → desc_store, Col 0~7 → pch_lvl_inst_buf)
  ├── NDP Start: per-PCH start flag(1b) + desc_count(16b)
  ├── tick(): PC 기반 fetch (pch_lvl_inst_buf → decode → addr_gen_slot), nl_req_slot 삭제
  ├── DRAM timing-only 모델 (AccInst data는 desc_store에만 저장)
  └── 검증: 기존 Direct trace로 기능 동등성 확인

Phase 2: Direct/Undirect Access Mode + PC 실행
  ├── AccInst_Slot bit[59] mode 추가 (opcode 바로 다음), 전체 필드 1b 하향 시프트
  ├── AccInst_Slot struct에 mode 필드 추가 (request.h)
  ├── base_reg[8], loop_cnt_reg[8] per PCH in HSNC (ndp_DRAM_system.cpp)
  ├── SET_BASE/INC_BASE: [58:56]=reg_idx, [55:38]=value
  ├── SET_LOOP: [58:56]=cnt_reg(3b), [55:40]=loop_count(16b)
  ├── LOOP: [58:56]=cnt_reg(3b), [55:40]=jump_pc(16b)
  ├── decode_acc_inst(): opcode별 필드 해석 분기 + Undirect row[17:15]=reg, row[14:0]=offset
  ├── PC (10-bit): PC[9:3]=col_addr, PC[2:0]=idx → desc_store/buffer 연동
  ├── 순차 Miss / LOOP Miss → DRAM RD (timing) + desc_store에서 buffer 적재
  ├── Trace generator (ndp_workload_trace_generator.py) 수정: Undirect trace 생성
  └── 검증: 동일 워크로드 Direct vs Undirect 결과 비교

Phase 3: DRAM Spill (Descriptor Cache)
  ├── pch_lvl_inst_buf를 8-entry-group fully-associative Descriptor Cache로 운용
  ├── DRAM 저장: (ch, pch, ndp_ctrl_buf_bg/bk, ndp_ctrl_row, col0~127) → PC 0~1023
  ├── Write Intercept: Col 0~7 write → buffer 직접 적재 (초기 64 entries)
  ├── PC = col_addr[9:3] + idx[2:0], Entry Group Tag = col_addr (7b)
  ├── Sequential Miss: 다음 column DRAM RD (64B) + oldest group evict
  ├── LOOP Miss: jump target column이 buffer에 없으면 DRAM RD + evict
  └── 검증: GEMV (>64 desc) 워크로드 descriptor cache hit/miss 통계
```

### Key Files

| File | Role |
|------|------|
| `src/base/request.h` | `AccInst_Slot` 구조체, 새 opcode 정의 |
| `src/memory_system/impl/ndp_DRAM_system.cpp` | HSNC 전체: buffer, decode, addr gen, state machine |
| `src/dram_controller/impl/ndp_dram_controller.cpp` | DBX MC (변경 최소: request 형태 동일) |
| Config YAML | `pch_lvl_inst_buf_size`, `undirect_mode` 등 |
| `ndp_workload_trace_generator.py` | 재정의된 AccInst 기반 trace 생성 (Undirect + SET_LOOP + LOOP) |
| `gen_trace.py` | 기존 Direct-only trace 생성 (원본 유지) |
