# AsyncDIMM Implementation Analysis for Ramulator2

## 1. AsyncDIMM Paper - Core Concepts Summary

### 1.1 Problem Definition
- In DIMM-NMP (Near-Memory Processing) architectures, Host MC and NMA (Near-Memory Accelerator) MC issue memory accesses independently
- Synchronous execution leaves either the host or NMA idle, wasting compute capability
- Existing approaches: Host-only mode or NMA-only mode → mutual exclusion wastes bandwidth
- AsyncDIMM enables asynchronous execution where Host and NMA operate concurrently

### 1.2 Three Memory Modes
| Mode | Description | Memory Access Owner | Host MC Behavior | NMA MC Behavior |
|------|-------------|---------------------|------------------|-----------------|
| Host Mode | Host MC directly accesses DRAM | Host only | Normal DDR5 commands | Bypass path: receive host commands → FSM update only |
| NMA Mode | NMA MC directly accesses DRAM, host access blocked | NMA only | Blocked (no DRAM access) | Full REQ FIFO scheduling |
| Concurrent Mode | Host offloads commands to NMA MC, scheduled together | Host + NMA | Offload commands (ACTO/RDO/WRO/PREO) | CMD FIFO + REQ FIFO mixed scheduling |

### 1.3 Core Mechanisms

#### A. Explicit Synchronization (H2N, Host→NMA Direction)
- In Host Mode, DDR commands issued by Host MC are **simultaneously bypassed to NMA MC**
- NMA MC FSMs always track up-to-date bank states (via bypass path)
- Enables seamless transition from Host Mode to NMA/Concurrent Mode without bank state recovery
- This is a continuous background process during Host Mode, not a transition event

#### B. OSR (Offload-Schedule-Return) Mechanism

**B-1. Offload (Host MC → NMA MC)**
- In Concurrent Mode, Host MC converts normal commands to offload variants and sends to NMA MC
- 4 new offload commands: ACTO, RDO, WRO, PREO
- Offload commands reuse the existing DDR5 C/A bus (no extra pins)
- Timing constraint optimizations for Host MC side:
  - ACTO: no actual bank activation → tRCD omitted
  - RDO: no actual DRAM read → tRCD, tBL, tRTRs, tCCD all omitted
  - WRO: DQ bus still needed for write data → partial optimization (insert RDO before WRO to skip tWTR)
  - PREO: no actual bank precharge → timing relaxed

**B-2. Schedule (Inside NMA MC)**
- Per-bank dual FIFO structure: CMD FIFO (host offloaded) + REQ FIFO (NMA local)
- H/N Arbiter determines which FIFO to serve
- FIFO switch conditions: (1) current FIFO becomes empty, (2) PRE command issued (bank boundary)
- **Switch-Recovery (SR) Method**:
  - On FIFO switch, bank state may mismatch between source's expectation and actual state
  - SR Unit records Bank State Change (BSC) commands (last ACT/PRE per bank)
  - Recovery table determines correction commands:
    - NMA→Host switch: {ACT→PRE+ACT, PRE→ACT, Same→None}
    - Host→NMA switch: {ACT→PRE+ACT, PRE→ACT, Same→None}
  - Correction commands issued before serving new FIFO's commands
  - On average, fewer than 2 banks need recovery per switch

**B-3. Return (NMA MC → Host MC)**

The Return mechanism solves a fundamental problem: the DDR5 alert_n pin is a **1-bit signal**
shared across all ranks, so it cannot convey **which** offloaded read completed — especially
since completions are **out-of-order between banks** within a rank.

**Solution: Strict Global Offload Order for Interrupts**
- Interrupts are restricted to be issued **in the order that memory accesses were offloaded**
- If an older offloaded access has not completed, interrupts for newer (already completed)
  accesses **cannot be issued**, even though they are ready — this is intentional head-of-line blocking
- This guarantees that the Host MC can identify which access completed simply by counting
  interrupts (always the next one in its offload FIFO)

**NMA MC Global Completion Buffer (Reorder Buffer):**
- Each offloaded RDO receives a global sequence number upon arrival at NMA MC
- Offloaded reads enter per-bank CMD FIFOs for scheduling (out-of-order completion between banks)
- On DRAM read completion: mark the corresponding entry as complete in the global completion buffer
- Interrupt issuance: only when the **head** (oldest) entry is marked complete
  - If head is incomplete → all newer completions wait (head-of-line blocking)
  - If head is complete → issue interrupt, advance head; check next head immediately
  - Multiple consecutive completed entries → rapid burst of interrupts (one per TDM slot)

```
Example: Offload order = #1(Bank0), #2(Bank5), #3(Bank0), #4(Bank12)
DRAM completion order = #2, #4, #1, #3

Global Completion Buffer:
  seq=1 Bank0  [pending]  ← head    → #2,#4 완료해도 interrupt 불가
  seq=2 Bank5  [COMPLETE]            (head-of-line blocking)
  seq=3 Bank0  [pending]
  seq=4 Bank12 [COMPLETE]

After #1 completes:
  seq=1 Bank0  [COMPLETE] ← head → interrupt!
  seq=2 Bank5  [COMPLETE] ← new head → interrupt! (즉시)
  seq=3 Bank0  [pending]  ← new head → 대기
  seq=4 Bank12 [COMPLETE]

→ Burst: 2 interrupts in 2 TDM slots → batch RT 가능
```

**TDM (Time-Division Multiplexing) interrupt**: uses DDR5 alert_n pin
- Each rank gets a time slot: rank source identified by interrupt timing
- 1 TDM period = Nrank cycles → 1 interrupt per rank per period
- Average interrupt latency = Nrank/2 cycles

**Host MC Return Unit (RU):**
- Maintains a FIFO of offloaded access records in offload order: {addr, R/W, cmd_count}
- On interrupt received: increment per-rank interrupt counter (no bank ID needed)
- Counter value = number of oldest offloaded accesses confirmed complete
- No request ID or bank tag matching required — strict ordering makes it implicit

**RT (Return) command**: Host MC retrieves completed read data from NMA MC
- Issued with highest priority when interrupts have accumulated
- Adaptive batch size = accumulated interrupt count from offloaded reads
- NMA MC sends data on DQ bus in offload order: tBL × batch_size burst
- RT latency = NMA buffer indexing time (replaces tRL, since data comes from NMA buffer, not DRAM)
- RT is treated as one or multiple special RD commands without bank access
  → coexists with existing DDR commands on memory buses without protocol violations
- Host MC RU pops batch_size entries from its offload FIFO and issues callbacks to frontend

**Head-of-line blocking trade-off:**
- Slow bank (row miss) blocks interrupt for all subsequent completions → added return latency
- But this causes completed reads to accumulate → larger RT batch → amortized RT overhead
- Net effect: the paper's evaluation shows this is acceptable for overall performance

#### C. Implicit Synchronization (N2H, NMA→Host Direction)
- Performed when switching from Concurrent/NMA Mode back to Host Mode
- NMA MC records the last Host BSC command for each bank during concurrent execution
- Before handing DRAM back to Host MC, NMA MC issues recovery commands if bank state mismatches Host MC's expectation
- On average, fewer than 2 banks need recovery → low overhead transition

#### D. Refresh Handling Across Modes

The AsyncDIMM paper does not explicitly detail refresh management in each mode.
However, the refresh responsibility follows logically from the DRAM control ownership in each mode.

**Refresh Responsibility by Mode:**

| Mode | DRAM Control Owner | Refresh Issuer | Rationale |
|------|-------------------|----------------|-----------|
| Host Mode | Host MC | **Host MC** | Host MC directly controls DRAM, issues REF normally. REF commands are bypassed to NMA MC via explicit sync (H2N) to keep NMA's bank FSM updated. |
| NMA Mode | NMA MC | **NMA MC** | NMA MC has exclusive DRAM control. Must independently track tREFI and issue REF. Host MC is blocked, no involvement. |
| Concurrent Mode | NMA MC (actual DRAM issuer) | **NMA MC** | NMA MC is the only entity issuing actual DRAM commands. Refresh is timing-critical (tREFI) — routing REF through the offload path would add latency and risk tREFI violation. |

**Concurrent Mode Refresh — Detailed Flow:**
```
NMA MC RefreshManager:
  tREFI countdown → REF due
    1. Pause CMD FIFO + REQ FIFO scheduling
    2. For REFsb (DDR5 per-bank refresh):
       - Target bank only: issue PRE (if bank Open) → issue REFsb
       - Other banks continue normal FIFO service (unaffected)
       - SR Unit: reset BSC record for refreshed bank (bank state = Closed)
    3. For REFab (all-bank refresh):
       - All banks: issue PRE (if Open) → issue REFab
       - All FIFO service paused during REFab window
       - SR Unit: reset all BSC records (all banks = Closed)
    4. Resume FIFO service after REF completion
```

**Refresh and Host MC State Synchronization:**

When NMA MC issues REF during Concurrent Mode, Host MC's bank state view becomes stale
(Host MC thinks bank may be Open, but REF has closed it). This mismatch is resolved by
**implicit synchronization (N2H)** when transitioning back to Host Mode — the same mechanism
that handles all other bank state discrepancies. No additional synchronization mechanism
is needed specifically for refresh.

**DDR5 REFsb Advantage for AsyncDIMM:**

DDR5's per-bank refresh (REFsb) is particularly beneficial for AsyncDIMM Concurrent Mode:
- REFab: all banks paused → CMD + REQ FIFOs for all banks stall → significant performance loss
- REFsb: only target bank paused → other banks' FIFOs continue uninterrupted → minimal impact
- AsyncDIMM's per-bank FIFO structure naturally accommodates per-bank refresh isolation

#### E. Concurrent Mode Host Blocking Risk (Row-Level NMA Operations)

The AsyncDIMM paper does not explicitly address the scenario where NMA performs long
row-level streaming operations (e.g., GEMV reading an entire row) during Concurrent Mode.

**Problem:** FIFO switch conditions are (1) current FIFO empty, or (2) PRE issued.
When NMA streams many column reads from the same row, neither condition is met for
extended periods, causing the CMD FIFO (host offloaded commands) for that bank to be blocked.

```
Example: NMA GEMV on Bank 0 (256 column reads, BL16)

REQ FIFO:  ACT row100 → RD col0 → RD col1 → ... → RD col255 → PRE
CMD FIFO:  ACT row50 → RD col3 → PRE   ← blocked for ~256 × tCCD cycles
           (waiting for REQ FIFO to hit PRE or empty)
```

This causes per-bank CMD FIFO stall → OT threshold reached → Host MC backpressure → host stall.
Note: this is a **per-bank** issue — other banks' CMD FIFOs are unaffected.

**Comparison with DBX-DIMM:**
DBX-DIMM addresses this with `m_ndp_row_hit_low_cap` (default 16) — NDP can only hit the same
row a limited number of times before yielding to host. AsyncDIMM's SR method has no equivalent
cap mechanism.

**Possible Mitigation (Implementation Option):**
Extend FIFO switch conditions with a service cap:
```
Switch Condition (extended):
  (1) Current FIFO empty
  (2) PRE issued
  (3) [Optional] Current FIFO consecutive service count > max_service_cap
      AND other FIFO non-empty → force switch (SR recovery will handle state mismatch)
```

For implementation, we first implement the paper's original design (conditions 1 and 2 only),
then evaluate whether condition 3 is needed based on performance results with row-streaming workloads.

### 1.4 Hardware Architecture

**Host MC Modifications (from standard DDR5 MC):**
| Component | Description |
|-----------|-------------|
| Offload Table (OT) | Per-bank counter tracking number of offloaded commands. Threshold = NMA CMD FIFO depth for backpressure |
| Return Unit (RU) | FIFO of offloaded access records (addr, R/W). Per-rank interrupt counter. No ID matching needed — strict offload order guarantees implicit identification |
| Con Mode Register | 1-bit per rank: 0=Host Mode, 1=Concurrent Mode |
| Command Encoder | Encodes ACTO/RDO/WRO/PREO/RT using existing DDR5 C/A signals (RFU fields) |
| Scheduler Bypass | Offload commands skip DRAM timing constraints (relaxed timing path) |

**NMA MC Components (on buffer chip, rank-level):**
| Component | Description |
|-----------|-------------|
| Bypass Path | Host Mode: forward host commands directly to bank FSMs (explicit sync) |
| CMD FIFO (per-bank) | Stores decoded offloaded commands from Host MC |
| REQ FIFO (per-bank) | Stores NMA Compute Unit's local memory requests |
| Configurable Mixed FIFO | NMA Mode: full REQ FIFO; Concurrent Mode: half CMD + half REQ |
| CMD Decoder | Decodes ACTO/RDO/WRO/PREO from C/A bus into normal ACT/RD/WR/PRE |
| SR Unit | Records per-bank BSC + switch-recovery table for FIFO switch correction |
| H/N Arbiter | Arbitrates between CMD FIFO and REQ FIFO with SR-based switching |
| Return Unit | Global completion buffer (reorder buffer) for strict offload-order interrupt issuance + RT response data management |
| NMA Compute Unit (CU) | PE array for NDP kernel execution (COPY, ADD, GEMV, SpMV, etc.) |

---

## 2. Ramulator2 Architecture Analysis

### 2.1 Simulation Hierarchy
```
IFrontEnd (trace / SimpleO3 processor model)
    ↓ send(Request)
IMemorySystem (GenericDRAM / ndpDRAM)
    ├── IAddrMapper (physical address → addr_vec decomposition)
    └── IDRAMController[] (per-channel)
        ├── IScheduler (FRFCFS, etc.)
        ├── IRefreshManager (REFab, REFsb)
        ├── IRowPolicy (Open/Closed)
        ├── IControllerPlugin[]
        └── IDRAM (DDR5 / DDR5-pCH)
            └── DRAMNodeBase (channel → rank → bankgroup → bank → row → column)
```

### 2.2 Key Classes and Source Files

| Component | File | Role |
|-----------|------|------|
| Request | `src/base/request.h` | addr, addr_vec[], command, type_id, callback, payload |
| IDRAM | `src/dram/dram.h` | Command/timing/state definitions, issue_command(), check_ready() |
| DRAMNodeBase | `src/dram/node.h` | Hierarchical FSM, m_state, m_cmd_ready_clk, timing checks |
| SpecDef | `src/dram/spec.h` | TimingConsEntry, DRAMCommandMeta definitions |
| DDR5 | `src/dram/impl/DDR5.cpp` | Base DDR5: channel/rank/bg/bank/row/col, standard commands+timing |
| DDR5-pCH | `src/dram/impl/DDR5-pCH.cpp` | DDR5 with pseudo-channel + NDP commands (NDP_DRAM_RD/WR, NDP_DB_RD/WR) |
| GenericController | `src/dram_controller/impl/generic_dram_controller.cpp` | Standard MC: read/write queues, scheduler, pending list |
| NDPController | `src/dram_controller/impl/ndp_dram_controller.cpp` | NDP MC: per-pCH buffers, D2PA prefetch, NDP throttling |
| GenericDRAMSystem | `src/memory_system/impl/generic_DRAM_system.cpp` | Per-channel controller management |
| NDPDRAMSystem | `src/memory_system/impl/ndp_DRAM_system.cpp` | NDP system: HSNC FSM, NL-REQ buffering, address generation |
| LoadStoreNCoreTrace | `src/frontend/impl/loadstore_ncore_trace.cpp` | Multi-core trace frontend |

### 2.3 Request Processing Flow
```
Frontend::tick()
  → MemorySystem::send(req)
    → AddrMapper::apply(req)          // addr → addr_vec [ch, rank, bg, bk, row, col]
    → Controller[ch]::send(req)       // enqueue in read/write buffer
      → Scheduler::get_best_request() // FRFCFS: row-hit first, then FCFS
      → DRAM::get_preq_command()      // check prerequisites (need ACT? need PRE?)
      → DRAM::check_ready()           // timing constraints satisfied?
      → DRAM::issue_command()          // update state + timing via action lambdas
      → add to pending queue
      → callback after read_latency   // notify frontend of completion
```

### 2.4 Bank State Management in Ramulator2
- `DRAMNodeBase::m_state`: current bank state (Opened/Closed/PowerUp/etc.)
- `DRAMNodeBase::m_f_state`: fake state for Data Buffer (used in pCH implementation)
- State transitions via Action Lambdas: `Closed→(ACT)→Opened→(PRE)→Closed`
- `m_cmd_ready_clk[cmd]`: earliest cycle each command can execute (timing enforcement)
- `m_cmd_history[cmd]`: command issue history (for window-based timing constraints like tFAW)
- `update_states()`: propagates state change from child to parent nodes
- `update_timing()`: applies timing constraints to related nodes after command issue

### 2.5 Existing NDP Implementation: DBX-DIMM (NDPDRAMSystem)

DBX-DIMM's NDP implementation is the reference for understanding how NDP workload offloading works in this ramulator2 fork.

**Architecture**: pCH-level NDP with HSNC (Host-Side NDP Controller)
- DDR5-pCH DRAM model with pseudo-channel + NDP-specific commands
- Single `ndpDRAMCtrl` controller handles both host and NDP requests
- `NDPDRAMSystem` manages NDP execution FSM and address generation

**NDP Workload Offloading Flow (DBX-DIMM)**:
```
Step 1: Host Frontend sends NDP instructions via memory-mapped writes
  └── Frontend trace contains writes to magic addresses (ndp_ctrl_row/bk/bg)
  └── NDPDRAMSystem::send() intercepts these addresses → send_ndp_ctrl()

Step 2: Host writes NL-REQ instructions to DIMM-level buffer
  └── NL-REQ = {opcode, ch, pch, bg, bk, row, col, opsize, stride, ...}
  └── Buffered in dimm_lvl_req_buffer[] and distributed to per-pCH slots

Step 3: Host writes control register → triggers NDP_ISSUE_START
  └── pch_lvl_hsnc_status[dimm][pch] = NDP_ISSUE_START
  └── Sends NDP Start Request to NDP unit via memory controller

Step 4: HSNC FSM drives NDP execution per-pCH
  └── NDP_IDLE → NDP_ISSUE_START → NDP_BEFORE_RUN → NDP_RUN → NDP_BAR/WAIT → NDP_DONE
  └── NDP_RUN: decode NL-REQ → address generator → send_ndp_req_to_mc()
  └── Address generator: round-robin across active slots, stride-based addr progression

Step 5: NDP requests go through same memory controller as host requests
  └── req.is_ndp_req = true, req.is_trace_core_req = true
  └── Controller schedules NDP requests alongside host requests
```

**Key Design Points**:
- Host frontend does NOT know about NDP internals — it just writes to magic addresses
- NDP workload description (NL-REQs) is transferred as data payloads in write requests
- Address generation happens inside the memory system (HSNC), not in the frontend
- Offloading overhead is modeled: instruction transfer latency + control register write + FSM transitions

---

## 3. AsyncDIMM → Ramulator2 Mapping Analysis

### 3.1 Design Decisions

| Decision | Choice | Rationale |
|----------|--------|-----------|
| DRAM Base | `DDR5.cpp` (base DDR5, no pseudo-channel) | AsyncDIMM is rank-level NMP, not pCH-level |
| Address Mapping | `RoBaBgCoRaCh` | Standard DDR5 mapping for fair comparison |
| NMA Location | Buffer chip (rank-level) | Same as original paper |
| NDP Offloading | Host-initiated, memory-mapped (DBX-DIMM consistent) | Fair comparison: both model offloading overhead |
| NMA Address Generation | Inside memory system (rank-level HSNC FSM) | Consistent with DBX-DIMM's pCH-level HSNC |

### 3.2 DRAM Model Mapping

AsyncDIMM is implemented on **DDR5** basis for comparison with DBX-DIMM.
The original paper uses DDR4, but we port to base DDR5 (`DDR5.cpp`):
- **DDR5-pCH (pseudo-channel) NOT used** — base DDR5 hierarchy only
- DDR5 timing parameters applied (nCL, nRCD, nRP, etc.)
- DDR5 per-bank REFsb, 16n prefetch reflected
- NMA location: buffer chip (rank-level)

| AsyncDIMM Concept | Ramulator2 Mapping | Implementation |
|-------------------|-------------------|----------------|
| DDR5 DRAM structure | `src/dram/impl/DDR5.cpp` | Fork **DDR5.cpp** → DDR5-AsyncDIMM.cpp |
| Channel/Rank/BG/Bank hierarchy | `m_levels` | Use base DDR5 structure (no pseudo-channel) |
| ACT, PRE, RD, WR commands | `m_commands` | Retain existing DDR5 commands |
| **ACTO, PREO, RDO, WRO** (new) | Add to `m_commands` | Offload command variants |
| **RT** (Return) command (new) | Add to `m_commands` | Batch return command |
| Relaxed timing | `m_timing_cons` | Offload commands have reduced/zero timing constraints |
| Bank States (Open/Close) | `m_states`, Action Lambda | Offload commands do NOT change bank state |

#### DDR4→DDR5 Porting Changes

| Item | DDR4 (Original Paper) | DDR5 (Our Implementation) |
|------|----------------------|--------------------------|
| Prefetch | 8n | 16n |
| Bank Group | 4 BG × 4 BK | 8 BG × 4 BK (x4/x8) or 4 BG × 4 BK (x16) |
| Pseudo-channel | None | None (base DDR5, pCH not used) |
| Per-bank Refresh | REFab only | REFab + REFsb |
| Burst Length | 8 | 16 (BL16) |
| C/A Encoding | DDR4 C2-C0=HHH | DDR5 RFU field utilized |
| NMA location | Buffer chip (rank-level) | Buffer chip (rank-level) |

#### New Command Definitions

```
Existing DDR5 commands: ACT, PRE, PREA, PREsb, RD, WR, RDA, WRA, REFab, REFsb, ...
AsyncDIMM new commands: ACTO, PREO, RDO, WRO, RT

ACTO: offloaded ACT → no bank state change on Host MC side, relaxed timing
PREO: offloaded PRE → no bank state change on Host MC side
RDO:  offloaded RD  → no DQ bus usage, tRCD/tBL/tCCD omitted on Host MC side
WRO:  offloaded WR  → DQ bus used for write data transfer
RT:   Return        → Host MC retrieves data from NMA buffer, variable burst length
```

#### Timing Constraint Mapping

| Constraint | Normal Command (DDR5) | Offload Command (Host MC Side) |
|------------|----------------------|-------------------------------|
| tRCD (ACT→RD/WR) | ~26 cycles | 0 (ACTO→RDO) — no actual bank activation |
| tBL (burst length) | 8 cycles (BL16/2) | 0 (between RDOs) — no DQ bus, retained (between WROs) |
| tRTRs (rank switch) | 4 cycles | 0 (between RDOs) |
| tCCD_S (CAS-to-CAS same BG) | 8 cycles | 0 (between RDOs) |
| tCCD_L (CAS-to-CAS diff BG) | 8 cycles | 0 (between RDOs) |
| tWTR (write-to-read) | varies | 0 (WRO→RDO insertion optimization) |
| tWL (write latency) | ~22 cycles | 0 (WRO) |
| RT latency | N/A | tBL × batch_size + NMA buffer indexing |

### 3.3 Memory Controller Mapping

AsyncDIMM requires **two types of MC**: Host MC (channel-level) and NMA MC (rank-level).

#### A. Host MC (AsyncDIMM)

| AsyncDIMM Host MC | Ramulator2 Implementation | Details |
|-------------------|--------------------------|---------|
| Bank FSM tracking | `DRAMNodeBase::m_state` | Retain existing DDR5 FSM |
| Scheduler (FRFCFS) | `IScheduler` (FRFCFS) | Retain existing, open-row policy |
| Offload Table (OT) | **New** per-bank counter | Track offloaded commands; threshold = NMA CMD FIFO depth |
| Return Unit (RU) | **New** FIFO + counter | Offload record FIFO (addr, R/W) + per-rank interrupt counter. Strict offload order → no ID matching needed |
| Con Mode Register | **New** per-rank bool | Concurrent mode activation flag |
| Timing bypass | Scheduler bypass logic | Offload commands use relaxed timing in check_ready() |
| RT issue logic | **New** | Highest priority RT issue on interrupt received |

**Implementation**: `src/dram_controller/impl/asyncdimm_host_controller.cpp` (fork from `generic_dram_controller.cpp`)

**tick() Logic by Mode:**
```
Host Mode:
  1. RefreshManager: track tREFI, issue REFab/REFsb directly to DRAM
  2. Normal FRFCFS scheduling → DRAM::issue_command()
  3. Explicit sync: every issued command (including REF) also sent to NMA MC via bypass

Concurrent Mode:
  1. RefreshManager: suspended — NMA MC takes over refresh responsibility
  2. Host requests → convert to offload commands (ACTO/RDO/WRO/PREO)
  3. Relaxed timing check → offload commands bypass normal DRAM timing
  4. Send offload commands to NMA MC
  5. OT: increment per-bank counter; if threshold reached → backpressure (stall)
  6. On interrupt received → increment per-rank interrupt counter
     (strict offload order guaranteed by NMA MC → counter = number of oldest accesses completed)
  7. Issue RT command with highest priority → batch_size = interrupt counter
     → retrieve data from NMA MC in offload order (tBL × batch_size burst)
     → RT latency = NMA indexing time (not tRL, data from NMA buffer)
  8. RU: pop batch_size entries from offload FIFO, callback to frontend per entry

NMA Mode:
  1. RefreshManager: suspended — NMA MC takes over refresh responsibility
  2. Block all host DRAM access
  3. Wait for NMA completion signal → trigger implicit sync → return to Host Mode
```

#### B. NMA MC (AsyncDIMM)

| AsyncDIMM NMA MC | Ramulator2 Implementation | Details |
|------------------|--------------------------|---------|
| Bank/BG FSMs | `DRAMNodeBase` | Shared DRAM instance with Host MC (single physical DRAM) |
| CMD FIFO (per-bank) | **New** per-bank queue | Stores decoded host offloaded commands |
| REQ FIFO (per-bank) | **New** per-bank queue | Stores NMA CU's local memory requests |
| Configurable Mixed FIFO | **New** | NMA Mode: full REQ FIFO; Concurrent: half CMD + half REQ |
| CMD Decoder | **New** | Decodes ACTO/RDO/WRO/PREO → ACT/RD/WR/PRE |
| Bypass Path | **New** | Host Mode: receive host commands → update bank FSM only (no DRAM issue) |
| SR Unit | **New** | Per-bank BSC tracking + switch-recovery table |
| H/N Arbiter | **New** | Arbitrates CMD FIFO vs REQ FIFO with SR-based switching |
| Return Unit | **New** Global Completion Buffer | Reorder buffer for strict offload-order interrupt issuance. Seq tracking + head-of-line blocking + RT response |
| RefreshManager | **New** | Manage REFab/REFsb during Concurrent/NMA Mode (see Section 1.3.D) |

**Implementation**: `src/dram_controller/impl/asyncdimm_nma_controller.cpp` (new, reference `ndp_dram_controller.cpp`)

**tick() Logic by Mode:**
```
Host Mode (Bypass Path):
  1. Receive bypassed commands from Host MC (including REF — Host MC manages refresh)
  2. Update bank FSMs (DRAMNodeBase state) — but do NOT issue to physical DRAM
  3. This keeps NMA MC's bank state view in sync with Host MC

Concurrent Mode:
  0. [Refresh Check] RefreshManager checks tREFI countdown
     - If REF due:
       REFsb: pause target bank's FIFOs → PRE (if Open) → REFsb → reset SR BSC → resume
       REFab: pause all FIFOs → PRE all open banks → REFab → reset all SR BSC → resume
  1. CMD Decoder: receive offloaded commands → decode → per-bank CMD FIFO
     + assign global sequence number → Global Completion Buffer entry {seq, bank, done=false}
  2. NMA CU requests → per-bank REQ FIFO (from internal address generator)
  3. H/N Arbiter: select active FIFO
     - Switch conditions: (a) current FIFO empty, (b) PRE command boundary
  4. SR Unit: on FIFO switch, check bank state mismatch
     - If mismatch → issue correction commands (PRE+ACT, ACT, or None)
  5. Selected command → DRAM::check_ready() → DRAM::issue_command()
  6. On offloaded read complete (from any bank, out-of-order):
     → mark Global Completion Buffer entry as done=true
  7. Interrupt issuance (strict offload order):
     → while (buffer.head().done == true): issue TDM interrupt, move to return queue
     → if head is not done: wait (head-of-line blocking, even if newer entries are done)
  8. On RT received (batch_size from Host MC):
     → send return_queue entries in order on DQ bus (tBL × batch_size burst)

NMA Mode:
  0. [Refresh Check] Same as Concurrent Mode — NMA MC manages refresh
  1. REQ FIFO only (full capacity) — CMD FIFO disabled
  2. NMA CU requests scheduled directly
  3. FIFO-based in-order scheduling
```

### 3.4 Memory System Mapping

| AsyncDIMM Concept | Ramulator2 Implementation | Details |
|-------------------|--------------------------|---------|
| Channel (Host MC + DIMMs) | `IMemorySystem` instance | Per-channel Host MC management |
| Rank (NMA on buffer chip) | NMA MC instance | Per-rank NMA MC instance |
| Host→NMA command transfer | Inter-controller method calls | Simulates C/A bus transfer |
| Interrupt (alert_n) | NMA MC → Host MC callback | Simulates TDM interrupt |
| TDM interrupt latency | Cycle-based modeling | Nrank/2 cycles average |
| Mode switch FSM | **New** state machine | Manages host↔concurrent↔nma transitions |
| NDP workload offloading | Memory-mapped control region | Consistent with DBX-DIMM approach |
| NMA address generation | Per-rank HSNC FSM in system | Consistent with DBX-DIMM's per-pCH HSNC |

**Implementation**: `src/memory_system/impl/asyncdimm_system.cpp` (new, reference `ndp_DRAM_system.cpp`)

**Architecture:**
```
AsyncDIMMSystem (per-channel)
├── IAddrMapper (RoBaBgCoRaCh)
├── Host MC (asyncdimm_host_controller) — 1 per channel
├── NMA MC[] (asyncdimm_nma_controller) — 1 per rank
├── Shared IDRAM instance (DDR5-AsyncDIMM)
├── NMA Control Region (memory-mapped addresses)
│   ├── NMA Control Register (trigger offload start)
│   └── NMA Launch Buffer (NDP kernel descriptors / NL-REQs)
├── Per-Rank NMA Execution FSM
│   └── NMA_IDLE → NMA_ISSUE_START → NMA_BEFORE_RUN → NMA_RUN → NMA_BAR/WAIT → NMA_DONE
└── Per-Rank NMA Address Generator
    └── Decode NL-REQ → generate memory requests → feed to NMA MC's REQ FIFO
```

**Mode Transition Logic:**
```
Host → Concurrent:
  1. Host frontend writes NDP kernel descriptor to NMA launch buffer (memory-mapped write)
  2. Host frontend writes NMA control register → trigger NMA_ISSUE_START
  3. NMA execution FSM activates → NMA starts generating requests
  4. Host MC switches to concurrent mode: begin offloading commands
  5. Explicit sync already maintained (bypass path was active during Host Mode)

Concurrent → Host:
  1. NMA execution FSM reaches NMA_DONE (all NMA requests completed)
  2. Implicit sync: NMA MC checks per-bank state vs Host MC expectation
  3. Issue recovery commands for mismatched banks (avg < 2 banks)
  4. Host MC deactivates concurrent mode → resume normal Host Mode

Host → NMA:
  1. Same as Host→Concurrent, but Host MC blocks all host DRAM access
  2. NMA MC has exclusive DRAM access (full REQ FIFO)

NMA → Host:
  1. Same implicit sync as Concurrent→Host
  2. Host MC resumes normal access after bank state recovery
```

### 3.5 NDP Workload Offloading Design (Host → NMA)

**Design Principle**: AsyncDIMM follows the **same offloading model as DBX-DIMM** for fair comparison. The host initiates NDP execution via memory-mapped writes, and address generation happens inside the memory system — not in the frontend.

#### Comparison: DBX-DIMM vs AsyncDIMM Offloading Flow

| Step | DBX-DIMM (Existing) | AsyncDIMM (Planned) |
|------|---------------------|---------------------|
| 1. Host writes NDP instructions | Write NL-REQs to `ndp_ctrl_buf_bk/bg` address | Write NL-REQs to `nma_launch_buf` address |
| 2. Host triggers NDP start | Write to `ndp_ctrl_bk/bg` → `NDP_ISSUE_START` | Write to `nma_ctrl` address → `NMA_ISSUE_START` |
| 3. Start request to NDP unit | HSNC generates NDP Start Write to controller | System generates mode transition signal to NMA MC |
| 4. NDP execution FSM | Per-pCH: `IDLE→ISSUE_START→BEFORE_RUN→RUN→BAR→DONE` | Per-rank: `IDLE→ISSUE_START→BEFORE_RUN→RUN→BAR→DONE` |
| 5. Address generation | Per-pCH addr gen slot, round-robin, stride-based | Per-rank addr gen slot, round-robin, stride-based |
| 6. Memory request issue | `send_ndp_req_to_mc()` → controller | `send_nma_req_to_nma_mc()` → NMA MC REQ FIFO |
| 7. Completion | `NDP_DONE` status per-pCH | `NMA_DONE` per-rank → implicit sync → Host Mode |
| 8. NDP granularity | Per pseudo-channel (pCH-level) | Per rank (rank-level) |

#### NL-REQ Instruction Format (AsyncDIMM)

Reuses DBX-DIMM's NL-REQ concept adapted for rank-level NMA:

```
NL-REQ = {
  opcode:  RD(0) / WR(1) / BAR(2) / WAIT_RES(3) / WAIT(6) / DONE(15)
  rank:    target rank ID
  bg:      bank group
  bk:      bank
  row:     starting row address
  col:     starting column address
  opsize:  number of accesses to generate
  stride:  column stride between consecutive accesses
  id:      NDP request identifier
}
```

#### Host Trace Format

The host trace includes NDP offload markers as memory-mapped writes, identical to how DBX-DIMM traces work:

```
# Normal host access
0     R  0x00001000
1     R  0x00002000
...
# NDP offload phase: write NL-REQ instructions to NMA launch buffer address
1000  W  0xNMA_BUF_ADDR    [payload: NL-REQ descriptor 1]
1001  W  0xNMA_BUF_ADDR    [payload: NL-REQ descriptor 2]
...
# NDP trigger: write to NMA control register
1010  W  0xNMA_CTRL_ADDR   [payload: rank_id, start signal]
# Host continues (concurrent mode) or waits (NMA mode)
1011  R  0x00003000         # host access during concurrent mode
...
# After NMA_DONE: back to host-only access
5000  R  0x00004000
```

#### send() Request Routing in AsyncDIMMSystem

```
AsyncDIMMSystem::send(Request req):
  addr_mapper->apply(req)

  if (is_nma_control_addr(req)):
    // NMA control register write → trigger mode transition
    rank_id = extract_rank(req)
    rank_nma_status[rank_id] = NMA_ISSUE_START
    return true

  elif (is_nma_launch_buffer_addr(req)):
    // NL-REQ instruction write → buffer in per-rank launch buffer
    rank_id = extract_rank(req)
    rank_nma_launch_buffer[rank_id].push_back(decode_nl_req(req.payload))
    return true

  else:
    // Normal host memory access
    if (rank_mode[target_rank] == CONCURRENT):
      // Host MC handles as offload (ACTO/RDO/WRO/PREO)
      return host_controller->send_offload(req)
    elif (rank_mode[target_rank] == NMA):
      // Host access blocked during NMA mode
      return false  // stall
    else:
      // Host mode: normal access
      return host_controller->send(req)
```

#### NMA Execution FSM in AsyncDIMMSystem::tick()

```
for each rank_id:
  switch (rank_nma_status[rank_id]):

    case NMA_IDLE:
      // Do nothing — Host Mode active
      break

    case NMA_ISSUE_START:
      // Distribute NL-REQs from launch buffer to NMA address generator slots
      if (nma_mc[rank_id]->is_empty_req()):
        distribute_nl_reqs_to_addr_gen(rank_id)
        rank_nma_status[rank_id] = NMA_BEFORE_RUN
      break

    case NMA_BEFORE_RUN:
      // Wait for NMA MC to be ready
      if (nma_mc[rank_id]->is_empty_req()):
        rank_nma_status[rank_id] = NMA_RUN
        rank_mode[rank_id] = CONCURRENT  // or NMA, depending on workload config
        host_controller->set_concurrent_mode(rank_id, true)
      break

    case NMA_RUN:
      // Address generator produces memory requests → NMA MC REQ FIFO
      send_nma_req_to_nma_mc(rank_id)
      // Decode next NL-REQ from slot if room available
      fetch_and_decode_nl_req(rank_id)
      // Check for BAR/WAIT/DONE transitions
      break

    case NMA_BAR:
      // Barrier: wait for all outstanding NMA requests to complete
      send_nma_req_to_nma_mc(rank_id)  // drain remaining
      if (addr_gen_empty(rank_id) && nma_mc[rank_id]->is_empty_req()):
        rank_nma_status[rank_id] = NMA_RUN
      break

    case NMA_DONE:
      // NMA kernel completed → trigger implicit sync → return to Host Mode
      nma_mc[rank_id]->perform_implicit_sync()
      rank_mode[rank_id] = HOST
      host_controller->set_concurrent_mode(rank_id, false)
      rank_nma_status[rank_id] = NMA_IDLE
      break
```

### 3.6 Address Mapper Mapping

| AsyncDIMM Concept | Ramulator2 Implementation |
|-------------------|--------------------------|
| Row-Bank-BankGroup-Column-Rank-Channel | `RoBaBgCoRaCh` |

Use existing `RoBaBgCoRaCh` mapping, same as standard DDR5.

### 3.7 Scheduler Mapping

| AsyncDIMM Scheduler | Ramulator2 Implementation | Location |
|---------------------|--------------------------|----------|
| Host MC: FRFCFS, Open-row policy | Existing `FRFCFS` | Reuse as-is |
| Host MC: Offload timing bypass | Modified `check_ready()` | Relaxed timing for ACTO/RDO/WRO/PREO |
| NMA MC: FIFO-based | **New** `FIFOScheduler` | CMD/REQ FIFO in-order |
| NMA MC: H/N Arbiter | **New** | SR-based FIFO switch logic |

### 3.8 Frontend Mapping

**No separate AsyncDIMM frontend needed.** Reuse existing `LoadStoreNCoreTrace` frontend.

| AsyncDIMM Concept | Ramulator2 Implementation | Details |
|-------------------|--------------------------|---------|
| Host CPU (8-core OoO) | `LoadStoreNCoreTrace` or `SimpleO3` | Existing frontend, unchanged |
| NMA CU (PE array) | Address generator in `asyncdimm_system.cpp` | Internal to memory system (not frontend) |
| NDP kernel offload | Memory-mapped writes in host trace | Host trace contains NMA control writes |
| NDP kernel completion | NMA_DONE FSM state | Memory system manages completion |

This is consistent with DBX-DIMM, where the frontend (`LoadStoreNCoreTrace`) has no NDP-specific logic — all NDP management is inside `NDPDRAMSystem`.

---

## 4. Implementation Plan

### 4.1 File List and Phases

#### Phase 1: DRAM Model (Foundation, Low Complexity)
| File | Description | Base |
|------|-------------|------|
| `src/dram/impl/DDR5-AsyncDIMM.cpp` | AsyncDIMM DDR5 DRAM definition | Fork DDR5.cpp |
| `src/dram/lambdas/action.h` (modify) | ACTO/PREO/RDO/WRO action lambdas (no state change) | Add to existing |
| `src/dram/lambdas/preq.h` (modify) | Offload command prerequisites | Add to existing |

**Key Implementation Details:**
- Add ACTO, PREO, RDO, WRO, RT to `m_commands`
- ACTO/PREO action: do NOT change `DRAMNodeBase::m_state` (host-side shadow only)
- RDO action: no DQ bus reservation, no bank state change
- WRO action: DQ bus reservation (write data transfer), no bank state change
- RT action: variable-length DQ bus reservation (batch_size × tBL)
- Relaxed timing entries in `m_timing_cons`: zero or reduced values for offload commands

#### Phase 2: NMA MC (Core, High Complexity)
| File | Description | Base |
|------|-------------|------|
| `src/dram_controller/impl/asyncdimm_nma_controller.cpp` | Full NMA MC implementation | New (reference ndp_dram_controller) |
| `src/dram_controller/impl/scheduler/asyncdimm_scheduler.cpp` | FIFO scheduler + SR switch logic | New |

**Key Implementation Details:**
- Per-bank CMD FIFO + REQ FIFO with configurable depth
- SR Unit: per-bank BSC record + recovery table lookup
- H/N Arbiter: FIFO switch on empty or PRE boundary
- Bypass path: Host Mode FSM update without DRAM issue
- Return Unit: Global Completion Buffer (reorder buffer) for strict offload-order returns
  - Assign global seq number per offloaded RDO on arrival
  - On DRAM read complete (out-of-order): mark entry done in buffer
  - Interrupt: issue only when head entry is done (head-of-line blocking for ordering)
  - RT response: send data in offload order from return queue (tBL × batch_size)
- RefreshManager: manage REFab/REFsb during Concurrent/NMA Mode
  - REFsb: pause target bank FIFOs only, reset SR BSC for that bank
  - REFab: pause all FIFOs, reset all SR BSC records
  - Host Mode: refresh managed by Host MC (NMA MC receives via bypass)

#### Phase 3: Host MC (Extension, High Complexity)
| File | Description | Base |
|------|-------------|------|
| `src/dram_controller/impl/asyncdimm_host_controller.cpp` | AsyncDIMM Host MC | Fork generic_dram_controller |

**Key Implementation Details:**
- Con Mode Register: per-rank mode flag
- Offload Table (OT): per-bank counter with threshold backpressure
- Offload command generation: convert ACT/RD/WR/PRE → ACTO/RDO/WRO/PREO
- Relaxed timing path for offload commands
- Return Unit (RU): offload FIFO (records in offload order) + per-rank interrupt counter
  - No ID matching needed — strict global order guarantees implicit identification
  - RT: batch_size = interrupt counter, pop entries from FIFO, callback per entry
- Explicit sync: bypass issued commands to NMA MC during Host Mode

#### Phase 4: System Integration (Medium Complexity)
| File | Description | Base |
|------|-------------|------|
| `src/memory_system/impl/asyncdimm_system.cpp` | System integration + NMA offloading | New (reference ndp_DRAM_system) |
| `asyncdimm_config.yaml` | Configuration file | New |

**Key Implementation Details:**
- Manage 1 Host MC + N NMA MCs (one per rank) within a channel
- Memory-mapped NMA control region (control register + launch buffer)
- Per-rank NMA execution FSM (IDLE→ISSUE_START→BEFORE_RUN→RUN→BAR→DONE)
- Per-rank NMA address generator (decode NL-REQ → memory request generation)
- Mode transition logic: Host↔Concurrent↔NMA with explicit/implicit sync
- TDM interrupt latency modeling
- No separate frontend needed — reuse existing `LoadStoreNCoreTrace`

#### Phase 5: Validation and Comparison
| Task | Description |
|------|-------------|
| Trace generation | Host (SPEC CPU) + NMA offload markers (BK: COPY/ADD/SCALE/AXPY/TRIAD/GEMV, CK: SpMV/BFS) |
| Statistics collection | Speedup, Bandwidth Utilization, Access Latency, Energy |
| DBX-DIMM comparison | Side-by-side performance comparison analysis |

### 4.2 Configuration (YAML)

```yaml
Frontend:
  impl: LoadStoreNCoreTrace   # Reuse existing frontend (same as DBX-DIMM)
  clock_ratio: 1
  num_cores: 4
  core0_trace: "../spec_trace/403.gcc"
  core0_mshr_size: 32
  core0_is_ndp_trace: false

MemorySystem:
  impl: asyncDIMM
  clock_ratio: 1

  # NMA Control Region Addresses
  nma_ctrl_row: <magic_row>
  nma_ctrl_bk: <magic_bk>
  nma_ctrl_bg: <magic_bg>
  nma_launch_buf_bk: <magic_buf_bk>
  nma_launch_buf_bg: <magic_buf_bg>

  DRAM:
    impl: DDR5-AsyncDIMM
    org:
      preset: DDR5_16Gb_x8
      channel: 1
      rank: 2
      dq: 8
    timing:
      preset: DDR5_4800B

  HostController:
    impl: asyncDIMMHostCtrl
    Scheduler:
      impl: FRFCFS
    RefreshManager:
      impl: DR5CHAllBank
    RowPolicy:
      impl: OpenRowPolicy
    ot_threshold: 16            # Offload Table threshold (= NMA CMD FIFO depth)
    rt_batch_priority: true     # RT has highest scheduling priority

  NMAController:
    impl: asyncDIMMNMACtrl
    cmd_fifo_depth: 16          # Per-bank CMD FIFO depth
    req_fifo_depth: 16          # Per-bank REQ FIFO depth
    sr_enable: true             # Switch-Recovery Unit enable

  AddrMapper:
    impl: RoBaBgCoRaCh
```

---

## 5. Implementation Complexity Analysis

### High Complexity
1. **SR Unit** (NMA MC): FIFO switch + bank state recovery logic
   - Per-bank BSC command tracking (last ACT/PRE per bank per source)
   - Switch-recovery table based correction command generation
   - Must guarantee timing constraint correctness for correction commands
   - Edge cases: multiple banks needing recovery, recovery during ongoing offload

2. **Host MC Offload Logic**: Insert offload bypass into existing FRFCFS scheduler flow
   - OT management with threshold-based backpressure
   - Relaxed timing constraint application (must correctly bypass `check_ready()`)
   - RDO/WRO interleaving optimization (insert RDO before WRO to reduce WRO overhead)

3. **Mode Transition**: Accurate explicit/implicit synchronization modeling
   - H2N explicit sync: continuous bypass of all host commands to NMA MC during Host Mode
   - N2H implicit sync: per-bank state comparison + recovery command generation
   - Transition timing: must account for recovery command latency

4. **Return Mechanism**: Global Completion Buffer + strict order interrupt + RT batch
   - NMA MC: Global Completion Buffer (reorder buffer) across all banks per rank
   - Strict offload-order interrupt issuance with head-of-line blocking
   - Per-bank DRAM completion → mark entry in global buffer → interrupt only when head is done
   - TDM interrupt latency modeling (Nrank/2 cycles, 1 interrupt per TDM slot)
   - Host MC: per-rank interrupt counter, offload FIFO pop on RT
   - RT: variable burst (tBL × batch_size), NMA indexing latency (replaces tRL)

### Medium Complexity

5. **NMA Execution FSM + Address Generator**: Per-rank NDP management
   - FSM state machine with proper transition conditions
   - Address generator with stride-based access pattern
   - NL-REQ decode and slot management (reference DBX-DIMM's implementation)

6. **Configurable Mixed FIFO**: FIFO depth reconfiguration on mode change
   - NMA Mode: full capacity for REQ FIFO
   - Concurrent Mode: split capacity between CMD and REQ FIFOs

### Low Complexity
7. **DRAM Command Addition**: ACTO, RDO, WRO, PREO, RT definitions in DDR5-AsyncDIMM.cpp
8. **Config YAML**: parameter settings and memory-mapped address configuration
9. **Statistics Collection**: leverage existing ramulator2 stats framework

---

## 6. DBX-DIMM vs AsyncDIMM Comparison Design

### 6.1 Architectural Comparison

| Comparison Item | AsyncDIMM (DDR5 Port) | DBX-DIMM (Existing) |
|----------------|----------------------|---------------------|
| DRAM Standard | DDR5 (base, no pCH) | DDR5 (pseudo-channel) |
| NMP Granularity | Rank-level | pCH-level |
| NMA Location | Buffer chip (rank-level) | Buffer chip (pCH-level) |
| MC Architecture | Host MC + NMA MC (distributed, 2 MCs) | Single NDP Controller |
| Execution Mode | Asynchronous (Host+NMA concurrent via OSR) | Sync/Async (NDP mode transition) |
| Host Access in NDP | Offloaded to NMA MC (concurrent access) | Direct DRAM (with NDP throttling) |
| NMA Access Method | Local MC FIFO scheduling | pCH-based NDP commands (NDP_DRAM_RD/WR) |
| Synchronization | Explicit (H2N bypass) + Implicit (N2H recovery) | NDP mode transition (HSNC FSM) |
| Scheduling | Host: FRFCFS + offload; NMA: FIFO + SR | FRFCFS + NDP scheduler (NDPFRFCFS) |
| Command Extension | ACTO/RDO/WRO/PREO/RT | NDP_DRAM_RD/WR, NDP_DB_RD/WR |
| Bandwidth Optimization | Relaxed offload timing (up to 2.25x) | D2PA prefetch |
| Latency Optimization | Concurrent execution (up to 47% reduction) | DB-side access optimization |
| NDP Offloading | Memory-mapped NL-REQ + control register | Memory-mapped NL-REQ + control register |
| NDP Address Generation | Per-rank HSNC FSM in memory system | Per-pCH HSNC FSM in memory system |

### 6.2 Key Architectural Differences

1. **AsyncDIMM**: Rank-level NMP with **offload-based** concurrent access
   - Host MC delegates commands to NMA MC during concurrent mode
   - Two independent MCs coordinate via OSR mechanism
   - Bank state consistency maintained by explicit/implicit sync

2. **DBX-DIMM**: pCH-level NMP with **direct NDP command** via Data Buffer
   - Single controller manages both host and NDP requests
   - NDP commands (NDP_DRAM_RD/WR) are distinct command types
   - Data Buffer serves as NDP processing element

### 6.3 Fair Comparison Conditions

| Condition | Configuration |
|-----------|---------------|
| DRAM Device | DDR5-4800B, same organization (capacity, BG, bank) |
| System | 4~8 core host, same channel/rank count |
| Host Workloads | SPEC CPU2006/2017 traces |
| NDP Workloads | BK: COPY, ADD, SCALE, AXPY, TRIAD, GEMV; CK: SpMV, BFS |
| NDP Offloading | Both use memory-mapped writes + internal address generation |
| Metrics | Speedup, Bandwidth Utilization, Access Latency, Energy |

### 6.4 What This Comparison Reveals

With the same DRAM standard (DDR5) and same offloading overhead model:
- **MC architecture comparison**: Distributed (Host+NMA) vs Single NDP controller
- **Concurrent execution model**: OSR-based offloading vs Direct NDP commands
- **NMP granularity**: Rank-level vs pCH-level processing
- **Scheduling efficiency**: FIFO+SR vs FRFCFS+NDP
- **Bandwidth utilization**: Relaxed timing optimization vs D2PA prefetch
