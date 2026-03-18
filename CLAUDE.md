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
Detailed analysis document: `docs/asyncdimm_implementation_analysis.md`

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

| opcode | Name | Memory Access | Description |
|--------|------|---------------|-------------|
| 0 | LOAD | RD × opsize | DRAM → NMA buffer |
| 1 | LOAD_ADD | RD × opsize | DRAM → buffer, ADD |
| 2 | LOAD_MUL | RD × opsize | DRAM → buffer, MUL |
| 3 | ADD | (none) | Buffer-internal, latency only |
| 4 | MUL | (none) | Buffer-internal, latency only |
| 5 | V_RED | RD × opsize | Vector reduction |
| 6 | S_RED | RD × opsize | Scalar reduction |
| 7 | MAC | RD × opsize | Multiply-accumulate |
| 8 | SCALE_ADD | RD × opsize | Scale and add |
| 16 | T_ADD | RD × opsize×2 | Transposed ADD (double access) |
| 17 | T_MUL | RD × opsize×2 | Transposed MUL |
| 18 | T_V_RED | RD × opsize×2 | Transposed V_RED |
| 19 | T_S_RED | RD × opsize×2 | Transposed S_RED |
| 20 | T_MAC | RD × opsize×2 | Transposed MAC |
| 32 | WBD | WR × opsize | NMA buffer → DRAM |
| 48 | BARRIER | (none) | Wait for all outstanding requests |
| 49 | EXIT | (none) | NMA execution complete |
| 52 | LOOP | (none) | etc[11:6]=loop_cnt, etc[5:0]=jump_pc |

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
- [ ] Create `src/dram/impl/DDR5-AsyncDIMM.cpp` (fork DDR5.cpp)
  - Add new commands: ACTO, PREO, RDO, WRO, RT
  - ACTO/PREO: no bank state change action lambda
  - RDO: no DQ bus usage, omit tRCD/tBL/tRTRs/tCCD (relaxed timing)
  - WRO: DQ bus used, omit tWL
  - RT: variable burst length (batch_size × tBL)
- [ ] Modify `src/dram/lambdas/action.h` — offload command actions (no state change)
- [ ] Modify `src/dram/lambdas/preq.h` — offload command prerequisites

#### 1-B: Host MC (Host Mode Only)
- [ ] Create `src/dram_controller/impl/asyncdimm_host_controller.cpp` (fork generic)
  - Host Mode only: existing FRFCFS + explicit sync (bypass all issued commands to NMA MC)
  - RefreshManager: standard DDR5 refresh (REFab/REFsb), bypassed to NMA MC
  - Con Mode register: per-rank mode flag (Host Mode only for now)
  - Placeholder for Concurrent/NMA Mode logic (to be added in Phase 2/3)

#### 1-C: NMA MC Skeleton (Bypass Path Only)
- [ ] Create `src/dram_controller/impl/asyncdimm_nma_controller.cpp` (new)
  - Host Mode bypass path only: receive host commands → update bank FSM (no DRAM issue)
  - Per-bank bank state tracking (shadow copy of Host MC's FSM)
  - Placeholder for CMD/REQ FIFO, SR Unit, Return Unit (to be added in Phase 2/3)

#### 1-D: AsyncDIMM Memory System + Config
- [ ] Create `src/memory_system/impl/asyncdimm_system.cpp` (reference ndp_DRAM_system)
  - Manage 1 Host MC + N NMA MCs (one per rank) within a channel
  - Host Mode: route all requests to Host MC, bypass to NMA MC
  - Per-rank mode register (Host Mode only for now)
  - Reuse existing LoadStoreNCoreTrace frontend (no frontend changes)
- [ ] Create `asyncdimm_config.yaml` configuration file
  - DRAM: DDR5-AsyncDIMM, Address Mapping: RoBaBgCoRaCh
  - Controller: AsyncDIMM Host MC + NMA MC (bypass only)

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
- [x] comp_opcode → memory access type mapping (RD/WR/none/control)
- [x] Shadow FSM-based prerequisite resolution (CLOSED→ACT, row conflict→PRE, row hit→RD/WR)
- [x] DRAM command issuing via `check_ready()` + `issue_command()`
- [x] Round-robin across addr_gen_slots, one request per NMA tick
- [x] Transposed instructions (T_*) generate opsize×2 accesses

#### 2-B: NMA MC Refresh During NMA Mode
- [x] NMA MC tracks nREFI interval and issues REFab when due
- [x] Requires all banks closed before refresh (same as AllBankRefresh)
- [x] FSM update on refresh issue + future action for completion

#### 2-C: Mode Transition (Host→NMA→Host)
- [x] System-level state: HOST_MODE → TRANSITIONING_H2N → NMA_MODE → TRANSITIONING_N2H → HOST_MODE
- [x] H2N: Set Host MC mode to NMA (blocks host requests + refresh), start NMA MC execution
- [x] N2H (Implicit Sync): NMA MC PREA (close all banks) → IDLE, Host MC restored to HOST mode
- [x] Per-rank mode management

#### 2-D: NMA Workload Driver (Trace-Based)
- [x] Memory-mapped control region (same pattern as DBX-DIMM HSNC):
  - Control register: row=MAX, bg=MAX, bk=MAX → mode switch trigger
  - Instruction buffer: row=MAX, bg=MAX-1, bk=MAX → NMAInst_Slot write
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
- [x] `get_total_accesses()`: opsize (normal) or opsize×2 (transposed)
- [x] Compute-only instructions (ADD/MUL): modeled as fixed-latency NMA_WAIT
- [x] LOOP instruction: loop_cnt decrement + jump_pc re-enqueue from program buffer
- [x] NMA clock ratio (1/4 DRAM clock) applied to all NMA operations

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
- [ ] Offload command generation: convert ACT/RD/WR/PRE → ACTO/RDO/WRO/PREO
- [ ] Offload Table (OT): per-bank counter with threshold backpressure
  - Limits in-flight offload commands to prevent CMD FIFO overflow
- [ ] Relaxed timing path for offload commands (no DRAM timing check, CA bus 2-cycle only)
- [ ] RDO/WRO interleaving optimization
- [ ] Return Unit (RU): offload FIFO + per-rank interrupt counter
  - Strict global order → no ID matching needed (counter = completed count)
  - RT issue: highest priority, batch_size = interrupt counter
  - RT latency = NMA indexing time (not tRL), burst = tBL × batch_size
- [ ] Suspend Host RefreshManager during Concurrent Mode

#### 3-B: NMA MC Concurrent Extensions
- [ ] CMD FIFO (per-bank): receive offloaded commands from Host MC
  - Write at DRAM clock rate (up to 2 cmds per NMA tick)
  - Read at NMA clock rate (1/4 DRAM clock)
- [ ] CMD Decoder: ACTO/RDO/WRO/PREO → ACT/RD/WR/PRE
- [ ] H/N Arbiter: switch CMD FIFO vs REQ FIFO (conditions: FIFO empty or PRE issued)
- [ ] Configurable Mixed FIFO: NMA mode=full REQ, Concurrent=half CMD + half REQ
- [ ] SR Unit: per-bank BSC tracking + switch-recovery table for FIFO switch correction
- [ ] Return Unit: Global Completion Buffer (reorder buffer) for strict offload-order returns
  - Assign global seq number per RDO on arrival
  - On DRAM read complete (out-of-order between banks): mark entry done
  - Interrupt: issue only when head entry is done (head-of-line blocking)
  - RT response: send data in offload order (tBL × batch_size burst)

#### 3-C: System Integration
- [ ] Mode transition: Host↔Concurrent with explicit/implicit sync
- [ ] TDM interrupt latency modeling (Nrank/2 cycles)
- [ ] Memory-mapped NMA control region (control register + launch buffer) for host-initiated offloading

---

### Phase 4: Validation and DBX-DIMM Comparison
- [ ] Generate Host + NMA traces (BK: COPY/ADD/SCALE/AXPY/TRIAD/GEMV, CK: SpMV/BFS)
- [ ] Collect statistics: Speedup, Bandwidth Utilization, Access Latency, Energy
- [ ] DBX-DIMM vs AsyncDIMM performance comparison analysis
