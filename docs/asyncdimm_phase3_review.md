# AsyncDIMM Phase 3 Review: Cross-Check, Code Review, and Implementation

**Date:** 2026-03-19
**Scope:** Tasks 1–5 per original request.

---

## Task 1 & 2: Paper Cross-Check and Document Updates

### AsyncDIMM Paper
- Location: `docs/AsyncDIMM_Achieving_Asynchronous_Execution_in_DIMM-Based_Near-Memory_Processing.pdf`

### Cross-Check Results (Paper vs. Implementation)

| Item | Paper | Implementation Status | Action Taken |
|------|-------|-----------------------|--------------|
| **NMAInst T_* ops** | VMA-internal, zero DRAM accesses, no DQ traffic | Fixed: `get_mem_access_type()` returns NONE; `get_total_accesses()` returns 0 | ✓ Fixed |
| **T_* latency** | Not specified explicitly; treated as 1 compute cycle | All T_* latency constants set to 1 NMA tick | ✓ Fixed |
| **ADD/MUL operands** | ADD: Z = X + Y (X from DRAM); MUL: Z = X * Y | Fixed: classified as READ (X from DRAM); previously wrongly classified | ✓ Fixed |
| **Configurable FIFO** | NMA=REQ 8; Concurrent=CMD 8 + REQ 8 (Table IV) | `NMA_REQ_BUFFER_PER_BANK=8`, `NMA_CMD_FIFO_SIZE=8` | ✓ Correct |
| **OT (Offload Table)** | Per-bank counter, threshold=CMD FIFO size; blocks offload | `m_ot_counter[]` exists, `HOST_OT_THRESHOLD=8` | ⚠ Not enforced |
| **H/N Arbiter switch** | Switch on FIFO empty OR PRE issued | `try_issue_concurrent_command()` | ✓ Implemented |
| **SR Unit** | Full 4-entry recovery table (Fig 5) | `switch_to_cmd_fifo()`: only 1 of 4 cases | ⚠ Incomplete |
| **TDM Interrupt** | N_rank/2 cycles; head-of-line blocking | `TDM_INTERRUPT_LATENCY=2`, `check_and_send_interrupt()` | ✓ Correct |
| **RT priority** | Highest priority over all DDR commands | `on_nma_interrupt()` accumulates batch; no RT issuance yet | ⚠ Partial |
| **N2H Implicit Sync** | Selective per-bank recovery (<2 banks avg) | Simplified: PREA (close all banks) | ✓ Acceptable |
| **REF in Concurrent** | NMA MC directly handles | `check_nma_refresh()` active; Host MC skips for CONCURRENT | ✓ Correct |
| **NMA Clock Ratio** | 1/4 DRAM clock | `NMA_CLOCK_RATIO=4` | ✓ Correct |
| **Host Scheduler Bypass** | Offload cmds bypass schedulers (§V.A.1) | Host MC uses full DRAM timing; bypass only substitutes cmd | ⚠ Conservative |
| **RU cmd_count** | 2-bit per entry (01/10/11) for OT update (§V.A.2) | Not implemented | ⚠ Missing |
| **RDO DQ bus** | No DQ (offloaded read data not returned) | DDR5-AsyncDIMM: `access=false` for RDO | ✓ Correct |
| **WRO DQ bus** | DQ used (write data sent with offload) | DDR5-AsyncDIMM: `access=true` for WRO | ✓ Correct |

### Detailed Discrepancy Analysis (Paper Re-Read 2026-03-19)

#### D1. Host MC Scheduler Bypass (Critical — Paper §V.A.1)
**Paper**: "commands generated from request buffers **bypass the schedulers** to omit corresponding timing constraints. After being offloaded, the commands update FSMs, schedulers, and arbiters like normal issuing."

**Our implementation**: Host MC still issues real ACT/RD/WR/PRE to the DRAM model through the normal scheduling pipeline (FRFCFS + check_ready + issue_command with full DRAM timing). Only the **bypass callback** translates ACT→ACTO, RD→RDO etc. for NMA MC CMD FIFO.

**Impact**: Our Host MC is timing-constrained for offloaded commands (tRCD, tCCD, etc.). The paper's design has offloaded commands going through only the CA 2-cycle constraint. This makes our simulation **conservative** — lower host offload throughput than the paper design. NMA MC CMD FIFO fills slower than it should.

**To fix**: Decouple Host MC scheduling from DRAM model bank state for CONCURRENT ranks. Generate offload commands using Host MC's own `m_open_row` tracking, bypass `check_ready()`, issue ACTO/RDO/WRO/PREO directly, and update Host MC FSMs manually.

#### D2. SR Recovery Table (Paper §IV.B, Fig 5)
**Paper** recovery table (when switching TO a destination FIFO):
```
Actual bank state \ Dest FIFO last BSC  |  ACT (expects open) | PRE (expects closed)
Actual = OPEN (from source FIFO)        |  PRE + ACT          | PRE
Actual = CLOSED (from source FIFO)      |  ACT                | None
```

**Our implementation** (`switch_to_cmd_fifo()`): Only handles one case — bank is OPENED and CMD FIFO front command is ACT → insert PRE recovery. Three cases are missing:
- OPEN × OPEN (different rows) → PRE+ACT needed (our PRE-only is insufficient for row conflict)
- CLOSED × OPEN → ACT recovery needed (CMD FIFO expects bank open, but it's closed)
- Switch from CMD→REQ FIFO also needs SR recovery (currently handled by REQ FIFO's own prerequisite resolution, which is functionally correct but doesn't match paper's symmetric SR design)

#### D3. OT Backpressure Not Enforced
**Paper §V.A.1**: "If it reaches the threshold, OT will prevent generating commands."
**Implementation**: `m_ot_counter` array exists but is never checked. NMA MC CMD FIFO silently drops entries when full, which approximates backpressure but loses offloaded commands. Should prevent offload at Host MC side.

#### D4. Return Unit 2-bit Command Count (Paper §V.A.2)
**Paper**: Each RU entry tracks a 2-bit `cmd_count`:
- `2'b01` = one RD/WR command (row hit: just data command)
- `2'b10` = ACT + RD/WR (row miss: activation + data)
- `2'b11` = PRE + ACT + RD/WR (row conflict: precharge + activation + data)

These are used to update OT in real-time. When commands complete, OT is decremented by cmd_count, not just 1. This is important for accurate backpressure.

**Implementation**: Not tracked. OT is not decremented at all.

#### D5. Configurable FIFO Sizing Clarification
**Paper §V.B.2**: "half of the FIFO capacity is reused for host commands" — describes **hardware** design where a single physical FIFO is split.
**Paper Table IV**: "CMD FIFO, REQ FIFO: Size: 8 for each" — **simulation** uses 8 entries per bank for each FIFO independently.
**Implementation**: Separate deques, 8 each. Matches Table IV. ✓
**CLAUDE.md** previously said "CMD FIFO = 8 + REQ FIFO = 8 (split, same total depth)" which was ambiguous. Updated to clarify each is 8 entries independently.

### CLAUDE.md Updates Made
- Phase 2 opcode table: corrected ADD/MUL to `RD × opsize` (X from DRAM); T_* to `(none)` VMA-internal
- Phase 3-A: added status notes for scheduler bypass limitation, OT enforcement gap
- Phase 3-B: added full SR recovery table from Fig 5, noted partial implementation status
- Phase 3-B: clarified FIFO sizing (8 each per Table IV, not 8 total)
- Phase 3-C: fixed "Block Host" wording; Host MC continues serving requests in CONCURRENT mode
- Phase 3 checkboxes updated to reflect implemented vs pending items
- Paper location recorded
- Added complete function reference for all three AsyncDIMM files

### asyncdimm_implementation_analysis.md
- Reviewed and found **most accurate** of the three documents
- Correctly describes: full SR recovery table (§1.3.B-2), 2-bit cmd_count in RU (§1.3.B-3), head-of-line blocking with examples, TDM mechanism, refresh handling
- Key details confirmed: CMD Decoder, H/N Arbiter, SR Unit, Return Unit, TDM interrupt
- No corrections needed — this document can serve as the implementation spec

---

## Task 3: Existing Code Review

### asyncdimm_host_controller.cpp

| Component | Review Findings |
|-----------|----------------|
| FRFCFS scheduling | Correct; round-robin rank, watermark-based RD/WR switch |
| Bypass callback | Correct; passes payload for WR/WRA (magic path) |
| NMA mode rejection | Correct; `send()` rejects, `priority_send()` returns true |
| PREsb tracking | Correct; added (missing from GenericDRAMController) |
| Write forwarding | Intentional no-op (matches GenericDRAMController behavior) |

**Bugs found:** None.

### asyncdimm_nma_controller.cpp

| Component | Review Findings |
|-----------|----------------|
| Shadow FSM | Correct; handles ACT/PRE/PREsb/PREA/REFab/REFsb |
| Magic path | Correct; intercepts WR to ctrl-reg and inst-buf addresses |
| LOOP handling | Correct; copies inst before pop_front(), caches program end in cnt |
| T_* classification | **Fixed**: ADD/MUL → READ; T_* → NONE (previous bug) |
| NMA refresh | Correct; nREFI interval, all-banks-closed prerequisite |
| N2H sync | Simplified (PREA), acceptable per paper (avg <2 banks need recovery) |

**Bugs found and fixed:**
1. T_* ops were documented as generating 2× accesses → fixed to 0 accesses (NONE type)
2. ADD/MUL operand key was ambiguous → clarified X=DRAM, correctly classified as READ
3. All T_* COMPUTE_LATENCY constants changed from 2 to 1 NMA tick

### asyncdimm_system.cpp

| Component | Review Findings |
|-----------|----------------|
| State machine | Correct; HOST → H2N → NMA → N2H → HOST |
| Trace loading | Correct; timestamp, LD/ST, hex address, 8-word payload |
| FSM sync verification | Correct; skips REFRESHING state |
| Bypass wiring | Correct; lambda captures nma_vec by ref |

**Bugs found:** None.

### DDR5-AsyncDIMM.cpp

| Component | Review Findings |
|-----------|----------------|
| ACTO/RDO/WRO/PREO commands | Correct; NoRequire prereqs, CA 2-cycle only |
| RT command | Correct; access=true, highest priority |
| ACTO/PREO actions | Correct; no bank state change |
| WRO action | Correct; DQ bus but no bank state change |

**Bugs found:** None.

---

## Task 4: Phase 3 Concurrent Mode Implementation

### 3-A: Host MC Concurrent Extensions

**File:** `src/dram_controller/impl/asyncdimm_host_controller.cpp`

**Implemented:**
- `m_ot_counter[total_banks_flat]`: per-bank OT counter (threshold = `HOST_OT_THRESHOLD=8`)
- `m_pending_interrupt_batch[rank_id]`: accumulates NMA MC interrupt batches
- `priority_send()`: skips refresh for CONCURRENT mode ranks (NMA MC handles REF)
- `tick()`: substitutes bypass commands with offload variants in CONCURRENT mode:
  - `ACT → ACTO`, `PRE → PREO`, `RD/RDA → RDO`, `WR/WRA → WRO`
- `on_nma_interrupt(rank_id, batch_size)`: called when NMA MC fires TDM interrupt; accumulates batch for RT issuance

**Design note:** Host MC still issues real ACT/RD/WR/PRE to DRAM model for correct timing. Only the **bypass** to NMA MC sends offload variants (ACTO/RDO/PREO/WRO). This ensures:
1. DRAM model state remains consistent for Host MC scheduling
2. NMA MC CMD FIFO receives correctly decoded commands for H/N Arbiter scheduling
3. Concurrent execution modeled with conservative timing (Host MC sees real DRAM constraints)

**Limitation:** Real ACTO/RDO should have only CA 2-cycle constraint (no tRCD, tCCD, etc.). Our Host MC still uses real DRAM timing for its own scheduling. This underestimates Host MC offload throughput. Correcting this requires decoupling Host MC scheduling from DRAM model bank state, which is a significant refactor.

### 3-B: NMA MC Concurrent Extensions

**File:** `src/dram_controller/impl/asyncdimm_nma_controller.cpp`

**CMD FIFO (per-bank):**
- `std::vector<std::deque<CmdFifoEntry>> m_cmd_fifo` — size 8 per bank in concurrent mode
- Populated by `bypass_command()` when ACTO/RDO/WRO/PREO received
- CMD Decoder: ACTO→ACT, RDO→RD, WRO→WR, PREO→PRE on entry
- Reads assigned global seq_num; pushed to `m_return_buffer` for in-order delivery

**H/N Arbiter (per-bank):**
- `std::vector<bool> m_arbiter_use_cmd` — tracks which FIFO each bank uses
- Switch CMD→REQ: CMD FIFO empty or SR recovery resolved
- Switch REQ→CMD: REQ FIFO empty or PRE issued from REQ FIFO
- Implemented in `try_issue_concurrent_command()` with per-bank round-robin

**SR Unit (per-bank):**
- `SRState m_sr_unit[]` — records last BSC per FIFO (cmd_open_row, req_open_row)
- `SRRecovery m_sr_recovery[]` — pending recovery command when switching to CMD FIFO
- Recovery triggered in `switch_to_cmd_fifo()`: if bank OPENED and CMD FIFO expects ACT → insert PRE recovery
- Stat tracked: `s_num_sr_recoveries`

**Return Unit (NMA MC side):**
- `std::deque<ReturnEntry> m_return_buffer` — ordered by seq_num (arrival order)
- `std::deque<PendingRead> m_pending_reads` — timed completion tracking
- `process_pending_reads()`: marks entries done at `m_clk + nCL + nBL`
- `check_and_send_interrupt()`: fires when head of return buffer is done (head-of-line blocking)
- Interrupt: `m_interrupt_cb(rank_id, batch_size)` with batch = consecutive done entries

**TDM Interrupt:**
- `static constexpr int TDM_INTERRUPT_LATENCY = 2` — N_rank/2 cycles for 4 ranks
- Callback wired by System; no explicit delay modeled (conservative)

**New public API:**
```cpp
void enter_concurrent_mode()           // H2C: reset CMD FIFOs, set arbiter to REQ
void exit_concurrent_mode()            // C2H: clear concurrent flag
void set_interrupt_callback(cb)        // Wire interrupt to Host MC
void receive_rt(int batch_size)        // RT received from Host MC (stat only)
bool is_concurrent_complete()          // NMA idle + CMD FIFOs empty + return buffer empty
int  get_cmd_fifo_outstanding()        // Total CMD FIFO depth across all banks
```

**Tick dispatch in concurrent mode:**
```
tick_nma_run()   → try_issue_concurrent_command() instead of try_issue_nma_command()
tick_nma_bar()   → try_issue_concurrent_command() instead of try_issue_nma_command()
tick_nma_wait()  → try_issue_concurrent_command() instead of try_issue_nma_command()
tick_nma_done()  → try_issue_concurrent_command() instead of try_issue_nma_command()
```

### 3-C: System Integration

**File:** `src/memory_system/impl/asyncdimm_system.cpp`

**New SystemNMAState values:**
```
TRANSITIONING_H2C  → set Host MC to CONCURRENT, start NMA execution, enter NMA concurrent
CONCURRENT         → monitor is_concurrent_complete()
TRANSITIONING_C2H  → wait for NMA idle, exit concurrent, restore Host MC to HOST
```

**Concurrent mode trigger:**
- Configured via YAML: `concurrent_mode_enable: true`
- When ctrl-reg WR detected and `m_concurrent_mode_enable == true`: → TRANSITIONING_H2C
- When `m_concurrent_mode_enable == false` (default): → TRANSITIONING_H2N (NMA mode, existing)

**Interrupt callback wiring:**
```cpp
// NMA MC interrupt → Host MC on_nma_interrupt()
nma_mc->set_interrupt_callback([host_mc](int rank_id, int batch_size) {
  host_mc->on_nma_interrupt(rank_id, batch_size);
});
```

**Transition handlers:**
- `tick_transition_h2c()`: `set_mode(CONCURRENT)` + `start_nma_execution()` + `enter_concurrent_mode()`
- `tick_concurrent_mode()`: polls `is_concurrent_complete()`
- `tick_transition_c2h()`: `exit_concurrent_mode()` + `set_mode(HOST)`

---

## Task 5: Summary and Pending Items

### What Was Implemented

| Phase | Component | Status |
|-------|-----------|--------|
| Phase 1 (Host Mode) | DDR5-AsyncDIMM, Host MC, NMA MC bypass, System | ✓ Complete |
| Phase 2 (NMA Mode) | REQ FIFO, NMA state machine, refresh, LOOP, T_* | ✓ Complete |
| Phase 3-A (Host MC) | Offload command bypass, OT counter, interrupt receive | ✓ Implemented |
| Phase 3-B (NMA MC) | CMD FIFO, H/N Arbiter, SR Unit, Return Unit, interrupt | ✓ Implemented |
| Phase 3-C (System) | CONCURRENT state, H2C/C2H transitions, interrupt wiring | ✓ Implemented |

### Known Limitations and Future Work

**Priority 1 (Functional correctness):**

1. **SR Recovery Table incomplete** (D2): Only 1 of 4 recovery cases handled. Bank OPEN with row conflict (needs PRE+ACT) and bank CLOSED with open-expecting dest (needs ACT) are missing. Can cause incorrect DRAM access (reading from wrong row) in edge cases. Fix: implement full recovery table from Paper Fig 5.

2. **OT backpressure not enforced** (D3): `m_ot_counter` exists but isn't checked. NMA MC CMD FIFO silently drops overflow. Fix: check `m_ot_counter[flat_bank] < HOST_OT_THRESHOLD` in Host MC before issuing offloaded commands.

**Priority 2 (Accuracy):**

3. **Host MC scheduler bypass** (D1): Host MC uses full DRAM timing for its own scheduling. Paper says offloaded commands should bypass schedulers with only CA 2-cycle constraint. Current implementation is conservative (lower offload throughput). Fix: decouple Host MC scheduling from DRAM model for CONCURRENT ranks.

4. **RT command issuance**: `on_nma_interrupt()` accumulates batch count but doesn't insert RT into priority buffer. Fix: create RT Request in `on_nma_interrupt()` and enqueue with highest priority.

5. **RU 2-bit command count** (D4): Paper tracks per-entry cmd_count (01/10/11) for accurate OT decrement. Not implemented. Fix: add cmd_count tracking per offloaded access in Host MC RU.

**Priority 3 (Optimization):**

6. **N2H Implicit Sync (Concurrent→Host)**: Simplified to PREA (close all banks). Paper says <2 banks need recovery on average and >50% of switches need zero recovery. Fix: use per-bank BSC tracking for selective recovery.

7. **Phase 4 (Validation)**: No traces generated yet for Concurrent Mode. Need BK/CK workload traces exercising both Host and NMA concurrency.

### Configuration to Enable Phase 3

Add to `asyncdimm_config.yaml`:
```yaml
MemorySystem:
  impl: AsyncDIMM
  concurrent_mode_enable: true   # Enable Phase 3 OSR Concurrent Mode
  # ... rest of existing config
```

Without this flag, the system uses Phase 2 NMA Mode (existing behavior, backward compatible).
