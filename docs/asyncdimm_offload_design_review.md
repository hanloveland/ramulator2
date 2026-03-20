# AsyncDIMM Offload & Return Mechanism Design Review

**Date:** 2026-03-19
**Scope:** Host access offload (ACTO/RDO/WRO/PREO/REFO), Return Unit, TDM Interrupt, RT command

---

## 1. Architecture Summary

```
Host MC                              NMA MC (per rank, on buffer chip)
┌──────────────┐                     ┌──────────────────┐
│ Request      │   ACTO/RDO/WRO/    │ CMD Decoder      │
│ Buffers      │──→ PREO/REFO ──────→│ CMD FIFO (per-bank)│
│              │   (CA bus 2-cycle)  │ H/N Arbiter      │
│ OT (per-bank)│                     │ SR Unit          │
│ RU (global)  │←── interrupt ───────│ Return Buffer    │
│ RT pending   │   (TDM 1-bit)      │                  │
│              │──→ RT ─────────────→│ Data Buffer      │
│              │←── data (DQ bus) ───│                  │
└──────────────┘                     └──────────────────┘
```

## 2. Step-by-Step Trace: Host Read Offload

### Precondition
- Rank 0 in CONCURRENT mode
- Bank (bg=2, bk=1) is CLOSED in Host MC's m_open_row[]
- RU has space (< 64 entries)

### Step 1: Request arrival
```
Frontend → send(LD addr) → m_read_buffers[rank=0]
  addr maps to: ch=0, rank=0, bg=2, bk=1, row=5000, col=10
  req.scratchpad[0] = 0 (offload cmd count)
```

### Step 2: schedule_request() — first tick
```
schedule_request():
  active_buffer: empty
  priority_buffer: empty (or HOST ranks only)
  per-rank round-robin → rank 0 = CONCURRENT
    → schedule_offload_for_rank(0):
      RU full? No (size < 64)
      m_open_row[bg2_bk1] == -1 → bank CLOSED
      pass 1: offload_cmd = ACTO
      check_ready(ACTO, addr) → CA bus free? YES
      req.command = ACTO
      return true, req_it, buffer = &m_read_buffers[0]
```

### Step 3: tick() unified issue — ACTO
```
cmd = ACTO, is_offload = true
issue_command(ACTO, addr) → DRAM model: CA bus occupied 2 cycles, NO bank state change
bypass(rank=0, ACTO, addr, {}) → NMA MC bypass_command():
  CONCURRENT mode → decoded ACT → CMD FIFO[bg2_bk1].push(ACT)
Row tracking: m_open_row[bg2_bk1] = 5000
OT: m_ot_counter[bg2_bk1]++, req.scratchpad[0]++ (=1)
Lifecycle: ACTO is not final, is_opening=false → request STAYS in buffer
```

### Step 4: schedule_request() — second tick (2 cycles later)
```
schedule_offload_for_rank(0):
  m_open_row[bg2_bk1] == 5000 == target_row → ROW HIT
  pass 0: offload_cmd = RDO
  check_ready(RDO, addr) → CA bus free? YES
  req.command = RDO
```

### Step 5: tick() unified issue — RDO
```
cmd = RDO, is_offload = true
issue_command(RDO, addr) → DRAM model: CA bus 2 cycles, no DQ, no bank state change
bypass(rank=0, RDO, addr, {}) → NMA MC bypass_command():
  CONCURRENT mode → decoded RD → CMD FIFO[bg2_bk1].push(RD)
  is_read = true → seq_num assigned → return_buffer.push({seq_num, false})
OT: m_ot_counter[bg2_bk1]++, req.scratchpad[0]++ (=2)
Lifecycle: RDO is final_offload
  → RU entry created: {req(with callback), cmd_count=2, bank_flat_id, is_read=true}
  → buffer.remove(req)
  → req is NOT in pending (awaits interrupt + RT)
```

### Step 6: NMA MC processes CMD FIFO
```
NMA MC tick (every 4 DRAM cycles):
  try_issue_concurrent_command():
    H/N Arbiter → CMD FIFO[bg2_bk1] has [ACT, RD]
    Issue ACT(row=5000) → DRAM model bank opens → shadow FSM updated
    (wait tRCD cycles)
    Issue RD(col=10) → DRAM model read → shadow FSM RD tracked

  process_pending_reads():
    pending_read complete_clk = issue_clk + nCL + nBL
    When m_clk >= complete_clk: return_buffer entry marked is_done = true
```

### Step 7: NMA MC fires TDM interrupt
```
check_and_send_interrupt():
  return_buffer.front().is_done == true
  m_clk - m_last_interrupt_clk >= TDM_INTERRUPT_PERIOD (4)? YES
  → m_pending_interrupts.push({batch_size=1, deliver_clk=m_clk+2})
  → return_buffer.pop_front()
  → m_last_interrupt_clk = m_clk

deliver_pending_interrupts():
  When deliver_clk <= m_clk:
    m_interrupt_cb(rank_id=0, batch_size=1)  → Host MC on_nma_interrupt(0, 1)
```

### Step 8: Host MC on_nma_interrupt(0, 1)
```
RU front: {req, cmd_count=2, bank=bg2_bk1, is_read=true}
OT: m_ot_counter[bg2_bk1] -= 2  (was +2 from ACTO+RDO)
is_read → m_rt_read_pending[0].push(req)
        → m_rt_pending_count[0]++ (=1)
RU.pop_front()
```

### Step 9: Host MC tick() — RT issuance
```
tick() step 1.5: RT issuance
  m_rt_pending_count[0] > 0 → YES
  rt_addr = {ch=0, rank=0}
  check_ready(RT, rt_addr) → DQ bus free? YES
  issue_command(RT, rt_addr) → DRAM model: DQ bus occupied tBL cycles
  batch = 1:
    req = m_rt_read_pending[0].pop()
    req.depart = m_clk + m_dram->m_read_latency
    pending.push_back(req)
  m_rt_pending_count[0] = 0
```

### Step 10: serve_completed_reads()
```
Next tick(s): pending.front().depart <= m_clk
  → req.callback(req)  → Frontend receives read response
  → pending.pop_front()

✅ Host read offload complete.
```

---

## 3. Step-by-Step Trace: Host Write Offload

### Step 1-3: Same as read, but ST addr, write_buffer, WRO
```
schedule_offload_for_rank: row hit → WRO
tick() issue: WRO → DRAM model (CA 2-cycle + nBL DQ for write data)
  bypass(WRO, addr, write_payload) → NMA MC CMD FIFO[WR]
  OT++, scratchpad[0]=1
  RU entry: {req, cmd_count=1, bank, is_read=false}
  buffer.remove()
```

### Step 4-5: NMA MC processes, fires interrupt
```
NMA MC: issue WR from CMD FIFO → return_buffer done → interrupt(0, 1)
```

### Step 6: Host MC on_nma_interrupt(0, 1)
```
RU front: is_read=false (write)
OT -= 1
RU.pop_front()
(NO rt_pending, NO RT needed, NO callback)

✅ Host write offload complete.
```

---

## 4. Step-by-Step Trace: REFO (Offloaded Refresh)

```
RefreshManager → priority_send(REFab, rank=0) → priority_buffer[0]
schedule_request(): rank 0 = CONCURRENT
  → REFO 변환, check_ready(REFO) → ready
tick() issue: REFO → DRAM model (CA 2-cycle)
  bypass(REFO) → NMA MC: m_nma_refresh_pending = true
  Lifecycle: REFO is final_offload → complete (no RU entry for REFO)
  priority_buffer.remove()

NMA MC: try_issue_nma_refresh() → wait banks closed → REFab → all banks refreshed

✅ REFO complete.
```

---

## 5. Step-by-Step Trace: Active Buffer Transition

```
tick N: HOST mode, LD addr → ACT(real) → active_buffer
tick N+1: ctrl-reg WR → mode = CONCURRENT

tick N+2: schedule_request()
  active_buffer check: rank = CONCURRENT
  RU full? NO
  RDO 변환, check_ready(RDO) → ready
  req.command = RDO

tick() issue:
  cmd = RDO, is_offload = true
  issue_command(RDO) → DRAM model: CA bus (no bank change, bank already open from real ACT)
  bypass(RDO) → NMA MC CMD FIFO[RD] (shadow FSM: bank open from ACT bypass)
  OT++, scratchpad[0]++ (=1, ACT was real → not counted)
  RU entry: cmd_count=1
  active_buffer.remove()

→ interrupt → on_nma_interrupt → OT -= 1 → rt_pending → RT → pending → callback

✅ Active buffer transition complete. OT 정합성 OK.
```

---

## 6. Bug Checklist

| # | 항목 | 상태 | 설명 |
|---|------|------|------|
| B1 | Active buffer scratchpad: ACT(real) 미카운트 | ✅ OK | OT는 RDO만 +1, cmd_count=1, interrupt 시 -1. 정합 |
| B2 | RU full + ACTO 차단 | ✅ Fixed | `schedule_offload_for_rank()` 시작부에서 RU full → return false |
| B3 | REFO + offload CA bus 충돌 | ✅ OK | DRAM model check_ready가 CA 2-cycle 제한 적용 |
| B4 | RU empty인데 interrupt 수신 | ✅ OK | on_nma_interrupt(): `if (m_return_unit.empty()) return;` |
| B5 | RT + WRO DQ bus 충돌 | ✅ OK | DRAM model: RT→WR, WR→RT에 nBL timing 정의됨 |
| B6 | C2H 시 RU 잔류 entry | ✅ Fixed | tick_concurrent_mode: `has_pending_offloads_for_rank()` 체크 추가 |
| B7 | NMA return_buffer vs Host RU 순서 | ✅ OK | 둘 다 offload order 기준 (seq_num 순서). TDM은 head-of-line |
| B8 | m_rt_read_pending callback 유효성 | ✅ OK | Request copy by value 시 std::function callback 보존됨 |
| B9 | TDM period: rank 0만 4cycle마다 → 다른 rank과 충돌? | ✅ OK | 각 NMA MC는 독립 m_last_interrupt_clk 보유 |
| B10 | OT가 음수 될 수 있는가? | ⚠ 확인 필요 | ACT(real)+RDO: OT=+1, interrupt: OT-=1 → 0. OK. 하지만 double interrupt 시 음수 가능 |

### B10 상세: OT 음수 방지

```
방어 코드 (on_nma_interrupt):
  if (entry.bank_flat_id >= 0 && entry.bank_flat_id < m_total_banks_flat)
    m_ot_counter[entry.bank_flat_id] -= entry.cmd_count;

만약 cmd_count > 현재 OT 값이면 음수 발생.
원인: active_buffer에서 ACT(real) 후 RDO로 전환 시, ACT는 OT 미증가.
  → cmd_count=1 (RDO만), OT도 +1 → interrupt 시 -1 → 0. OK.

정상 offload: PREO+ACTO+RDO → OT=+3, cmd_count=3 → -3 → 0. OK.
→ 음수 불가능 (정상 흐름에서). 방어 코드 추가 권장.
```

---

## 7. Timing Summary

| Event | Latency |
|-------|---------|
| ACTO/PREO/RDO | CA 2-cycle (no bank timing) |
| WRO | CA 2-cycle + nBL (DQ bus for write data) |
| REFO | CA 2-cycle |
| NMA CMD FIFO → DRAM execution | tRCD + tBL (real timing by NMA MC) |
| TDM interrupt period | N_rank cycles (4) |
| TDM interrupt delivery delay | N_rank/2 cycles (2) |
| RT | DQ bus tBL × batch_size |
| Read total latency | offload(~4) + NMA processing(~30+) + TDM(~2+) + RT(~8) = variable |

---

## 8. Data Flow Diagram

```
Host Frontend
  │ LD/ST
  ▼
Host MC send() ──→ read/write_buffer[rank]
  │
  ▼ schedule_offload_for_rank()
  ├─ ACTO ──→ DRAM model (CA) ──→ bypass ──→ NMA MC CMD FIFO
  │           m_open_row update     OT++, scratchpad++
  │           req stays in buffer
  │
  ├─ RDO/WRO ──→ DRAM model (CA+DQ) ──→ bypass ──→ NMA MC CMD FIFO
  │               OT++, scratchpad++
  │               RU entry created (cmd_count = scratchpad[0])
  │               req removed from buffer
  │                                        │
  │                              NMA MC issues real ACT+RD/WR
  │                                        │
  │                              return_buffer: mark done
  │                                        │
  │                              TDM interrupt (1-per-period)
  │                                        │
  ▼ on_nma_interrupt()                     │
  ├─ Write: OT -= cmd_count, RU pop, done  │
  └─ Read:  OT -= cmd_count, RU pop        │
            rt_read_pending[rank].push(req) │
            rt_pending_count[rank]++        │
  │                                        │
  ▼ tick() RT issuance (highest priority)  │
  RT ──→ DRAM model (DQ bus) ──→ NMA MC returns data
  │
  ▼ req.depart = clk + latency → pending
  │
  ▼ serve_completed_reads() → callback → Frontend response
```
