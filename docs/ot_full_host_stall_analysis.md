# AsyncDIMM Concurrent Mode: OT Full → Host Access Stall 원인 분석

**Date:** 2026-03-22
**Log:** `async_co_4.log`
**Symptom:** Concurrent mode에서 Host MC OT가 full(per-bank counter >= HOST_OT_THRESHOLD=12)이 되어 host access가 진행되지 않음

---

## 1. 분석 결과 요약

| 우선순위 | 원인 | 심각도 | 근본 원인 여부 |
|----------|------|--------|---------------|
| **#1** | Return Buffer Head-of-Line Blocking → Interrupt 발행 지연 | **Critical** | **핵심 병목** |
| **#2** | TDM Interrupt Rate vs OT Increment Rate 구조적 불균형 | **High** | 근본 원인 |
| **#3** | H/N Arbiter의 CMD FIFO 서비스 지연 | **Medium** | 악화 요소 |
| **#4** | Single-Rank CONCURRENT에서 host request 탈출구 부재 | **High** | 결과적 blocking |
| **#5** | RU Capacity → OT 추가 blocking | **Low** | 2차 효과 |
| - | OT Increment/Decrement 비대칭 (scratchpad) | **None** | 정상 작동 확인 |
| - | RT 미구현 | **None** | RT는 구현되어 있음 (lines 623-648) |

---

## 2. 상세 분석

### 2.1 [확인완료] OT Increment/Decrement 대칭성 — 정상

OT의 increment/decrement 메커니즘 자체는 정확하게 구현되어 있다.

**OT Increment** (`asyncdimm_host_controller.cpp:792-797`):
- 모든 offload command(ACTO/PREO/RDO/WRO) 발행 시 `m_ot_counter[bank]++`
- `req_it->scratchpad[0]++`로 per-request 누적

**OT Decrement** (`asyncdimm_host_controller.cpp:425-448`):
- `on_nma_interrupt()` → RU의 oldest entry에서 `cmd_count`만큼 한번에 감소
- `cmd_count`는 `scratchpad[0]`에서 복사 (line 818)

**Row-miss 시나리오 (PREO→ACTO→RDO) 검증:**
1. PREO 발행: `scratchpad[0] = 1`, request는 buffer에 잔류 (PREO는 `is_closing`이므로 active_buffer 이동 안함)
2. ACTO 발행: `scratchpad[0] = 2`, request는 buffer에 잔류 (ACTO의 `is_opening = false` — DDR5-AsyncDIMM.cpp line 138)
3. RDO 발행: `scratchpad[0] = 3`, RU에 `cmd_count=3`으로 등록 후 buffer에서 제거

**결론:** OT increment 3 → interrupt 시 decrement 3. 완벽하게 대칭.

---

### 2.2 [핵심 병목 #1] Return Buffer Head-of-Line Blocking

NMA MC의 return_buffer는 offload 도착순으로 정렬된 FIFO이며, interrupt는 **head entry가 is_done=true일 때만** 발행된다.

**코드 근거** (`asyncdimm_nma_controller.cpp:1338-1351`):
```cpp
void check_and_send_interrupt() {
    if (m_return_buffer.empty()) return;
    if (!m_return_buffer.front().is_done) return;  // ← HEAD-OF-LINE CHECK
    if ((m_clk - m_last_interrupt_clk) < TDM_INTERRUPT_PERIOD) return;
    m_return_buffer.pop_front();
    m_pending_interrupts.push_back({1, m_clk + TDM_INTERRUPT_LATENCY});
}
```

**문제 시나리오:**

Return buffer 상태: `[{seq=5, bank0, RD_L, done=false}, {seq=6, bank1, WR_L, done=true}, {seq=7, bank2, RD_L, done=true}]`

- seq=6 (WR_L)는 즉시 `is_done=true` (line 1237-1241: write는 발행 즉시 완료 처리)
- seq=7 (RD_L)는 DRAM read 완료 후 `is_done=true` (line 1232-1234: `m_clk + nCL + nBL`)
- **하지만 seq=5의 RD_L이 아직 발행 안됐으면 → seq=6, seq=7 모두 interrupt 불가**

**Head entry 지연 원인:**
1. CMD FIFO에서 해당 bank의 RD_L이 아직 실행 안됨 (H/N Arbiter가 REQ FIFO 우선 서비스)
2. Bank state 불일치 → SR recovery 필요 (PRE+ACT 추가 latency)
3. DRAM timing constraint (tRCD, tCCD 등)

**최악의 경우 지연:**
- Row conflict recovery: +tRP(~18) + tRCD(~18) = ~36 DRAM cycles
- H/N Arbiter에 의한 CMD FIFO 대기: REQ cap (8 NMA ticks) × 4 = ~32 DRAM cycles
- **Head entry 최대 지연: ~70+ DRAM cycles**

이 기간 동안 **모든 후속 interrupt가 완전히 차단**되어 OT decrement가 정지한다.

---

### 2.3 [핵심 병목 #2] TDM Interrupt Rate vs OT Increment Rate 구조적 불균형

**OT 증가 속도 (Host MC):**
- CA bus 2-cycle constraint → 최대 1 offload command / 2 DRAM cycles
- **OT increment rate: ~0.5 / DRAM cycle**

**OT 감소 속도 (NMA MC → Host MC):**
- TDM_INTERRUPT_PERIOD = 4 cycles (per rank NMA MC)
- 각 interrupt → 1 RU entry 처리 → `cmd_count` (1~3) OT 감소
- TDM_INTERRUPT_LATENCY = 2 cycles (delivery delay)
- **이론적 최대 OT decrement rate: cmd_count / 4 cycles ≈ 0.25~0.75 / cycle**

**하지만 실제로는 Head-of-line blocking으로 인해:**
- Head entry가 done되기 전까지 interrupt 발행 = 0
- 연속적으로 쌓인 entry들도 4 cycle 간격으로만 drain → burst drain 불가
- **실효 OT decrement rate: << 0.25 / cycle (HOL blocking 기간)**

**OT 포화 시나리오:**
```
Host MC:  cycle 0~24에 12개 offload command 발행 (OT=12 도달)
NMA MC:   HEAD entry가 cycle 30에 완료 → cycle 34에 interrupt → cycle 36 delivery
          → 1번째 OT decrement (cmd_count=2 가정 → OT=10)
Host MC:  OT < 12 → 즉시 2개 추가 offload (OT=12 다시 도달)
NMA MC:   다음 interrupt는 cycle 38 이후...

→ OT가 threshold 근처에서 진동하며 host 처리량이 interrupt rate에 종속됨
```

---

### 2.4 [악화 요소 #3] H/N Arbiter CMD FIFO 서비스 지연

NMA MC의 H/N Arbiter가 REQ FIFO를 우선 서비스하면 CMD FIFO의 RD_L/WR_L 실행이 지연되어 interrupt chain 전체가 늦어진다.

**Arbiter 동작** (`asyncdimm_nma_controller.cpp:978-1090`):

| Fairness 메커니즘 | Default 값 | 동작 |
|-------------------|-----------|------|
| (A) CMD Service Interval | 16 NMA ticks (=64 DRAM cycles) | REQ가 16 tick 연속 서비스 시 CMD로 강제 전환 |
| (A) CMD Service Quota | 4 NMA ticks (=16 DRAM cycles) | 강제 전환 시 최소 4 tick CMD 보장 |
| (C) REQ Continuous Cap | 8 NMA ticks (=32 DRAM cycles) | REQ 8회 연속 후 CMD로 전환 |
| Row-Hit Low Cap | 1 | 같은 row hit 1회 후 CMD로 전환 |

**CMD FIFO 기아 가능성:**
- REQ Continuous Cap=8 → CMD FIFO가 최대 32 DRAM cycles 대기
- 이 기간 동안 return_buffer의 head entry가 RD_L이면 → interrupt 완전 차단
- **32 cycles × 0.5 OT increment rate = 16 OT increment (threshold 초과)**

**설정값에 의한 영향** (`asyncdimm_system.cpp:249-257`):
```cpp
fairness_interval = param<int>("hn_cmd_service_interval").default_val(16);
fairness_quota    = param<int>("hn_cmd_service_quota").default_val(4);
fairness_cap      = param<int>("hn_req_continuous_cap").default_val(8);
```

---

### 2.5 [결과적 blocking #4] Single-Rank CONCURRENT에서 Host Request 완전 차단

**Host request scheduling 경로와 OT 체크 위치:**

| schedule_request() 단계 | 대상 | OT 체크 | 코드 위치 |
|--------------------------|------|---------|-----------|
| Step 1a (active_buffer, CONCURRENT) | RDO/WRO 변환 | **Yes** (line 1044) | `m_ot_counter[ab_flat] >= HOST_OT_THRESHOLD` |
| Step 1b (active_buffer, HOST) | 일반 스케줄링 | No | HOST only |
| Step 1c (FRFCFS fallback) | CONCURRENT fallback | **Yes** (line 1090) | 동일 |
| Step 2 (priority, REFO) | Refresh offload | **No** | REFO는 OT 증가 안함 |
| Step 3 (read/write buffer, CONCURRENT) | schedule_offload_for_rank | **Yes** (line 893) | 동일 |
| Step 3 (read/write buffer, HOST) | 일반 FRFCFS | No | HOST only |

**send() 동작** (line 467-477):
- NMA mode: `return false` (거부)
- **CONCURRENT mode: 정상 수락** → read/write buffer에 enqueue

**Single-rank CONCURRENT의 문제:**
1. 유일한 rank가 CONCURRENT → 모든 host request가 offload 경로
2. 모든 bank의 OT >= 12 → `schedule_offload_for_rank()` return false
3. send()는 계속 수락 → buffer에 쌓이지만 schedule 불가
4. **Refresh(REFO)만 발행 가능 (Step 2)** — 유일한 탈출구

**RU Full 추가 blocking** (line 868):
- `m_return_unit.size() >= RU_MAX_ENTRIES(64)` → 모든 offload scheduling 즉시 차단
- HOST mode request는 영향 없으나, single-rank CONCURRENT에선 HOST mode rank 자체가 없음

---

### 2.6 [수정] RT 구현 상태 — 구현되어 있음

초기 가정과 달리 RT command는 **이미 구현**되어 있다.

**구현 위치** (`asyncdimm_host_controller.cpp:623-648`):
```cpp
// 1.5. RT issuance — highest priority (paper §V.A.1)
for (int rk = 0; rk < m_num_rank; rk++) {
    if (m_rt_pending_count[rk] <= 0) continue;
    int batch = std::min(m_rt_pending_count[rk], 8);
    int rt_cmd = m_rt_cmds[batch];
    if (m_dram->check_ready(rt_cmd, rt_addr)) {
        m_dram->issue_command(rt_cmd, rt_addr);
        // → pending으로 이동 → serve_completed_reads()에서 callback 호출
    }
}
```

**데이터 흐름 확인:**
1. NMA MC read 완료 → return_buffer `is_done=true`
2. `check_and_send_interrupt()` → TDM interrupt 발행
3. `deliver_pending_interrupts()` → Host MC `on_nma_interrupt()` 호출
4. `on_nma_interrupt()` → `m_rt_read_pending[rk]`에 추가, `m_rt_pending_count[rk]++`
5. Host MC `tick()` → RT issuance (line 634-635) → `pending`으로 이동
6. `serve_completed_reads()` → callback 호출 → frontend MSHR 해제

**RT 자체는 정상 작동하지만**, RT가 발행되려면 interrupt가 도착해야 하므로 **interrupt 지연이 RT 지연으로 직결**된다.

---

## 3. 인과관계 (Causal Chain)

```
H/N Arbiter REQ FIFO 우선 서비스 (max 32 DRAM cycles)
    ↓
CMD FIFO의 RD_L 실행 지연
    ↓
Return Buffer HEAD entry is_done 지연
    ↓
Head-of-Line Blocking → 모든 interrupt 발행 차단
    ↓
Host MC on_nma_interrupt() 호출 안됨
    ↓
OT decrement 정지 + RU drain 정지
    ↓
OT >= 12 (threshold) 유지
    ↓
schedule_offload_for_rank() → continue (모든 bank skip)
    ↓
Host request scheduling 실패 (CONCURRENT mode에서 대안 경로 없음)
    ↓
Host access stall
```

**추가 악순환:**
```
OT 정체 → Host MC가 새 offload 못 보냄 → CMD FIFO 비어감
→ H/N Arbiter가 CMD→REQ 전환 → NMA REQ만 서비스
→ CMD FIFO 새 entry 도착 시 REQ에 의해 blocking → CMD 대기 연장
→ interrupt 추가 지연 → OT 정체 지속
```

---

## 4. 근본 원인 (Root Cause)

**Concurrent mode에서 OT backpressure의 drain 경로가 단일 FIFO (return_buffer)의 head-of-line ordering에 의존하며, 그 head entry의 실행이 H/N Arbiter의 CMD/REQ FIFO 경합에 의해 지연되는 구조적 문제.**

구체적으로:
1. **Return buffer가 global FIFO** → 한 bank의 지연이 모든 bank의 interrupt를 차단
2. **TDM interrupt가 1-at-a-time** → 대량의 OT를 빠르게 drain할 수 없음
3. **HOST_OT_THRESHOLD=12** 이상이면 host 스케줄링이 **완전히** 중단되는 binary 특성
4. **CONCURRENT mode에서 host request의 비-offload 경로가 없음** → OT threshold 도달 시 100% stall

---

## 5. 개선 방안 (제안)

### 5.1 단기 (Parameter Tuning)

| Parameter | 현재값 | 제안값 | 효과 |
|-----------|--------|--------|------|
| `hn_req_continuous_cap` | 8 | **4** | CMD FIFO 최대 대기 시간 절반 (32→16 cycles) |
| `hn_cmd_service_quota` | 4 | **8** | CMD 강제 서비스 시간 2배 → interrupt 생성 가속 |
| `HOST_OT_THRESHOLD` | 12 | **16~20** | Headroom 확보 → stall 빈도 감소 |

### 5.2 중기 (구조 개선)

1. **Per-bank return buffer**: Global FIFO → per-bank FIFO로 변경. Bank 간 head-of-line blocking 제거.
2. **OT threshold를 soft limit으로**: threshold 이상에서도 일정 비율(예: 1/4)의 offload 허용하여 complete stall 방지.
3. **Interrupt batching**: 한 TDM slot에서 여러 `is_done` entry를 한번에 interrupt 발행.

### 5.3 장기 (Architecture)

1. **비동기 OT decrement**: Return buffer 완료를 polling이 아닌 per-bank flag로 처리하여 HOL 제거.
2. **Host request direct path**: CONCURRENT mode에서도 host request의 일부를 direct DRAM access로 처리하는 hybrid 경로.

---

## 6. 검증을 위한 로그 확인 포인트

`async_co_4.log`에서 다음 패턴을 확인:

1. **`[NMA-ISSUE]` 로그의 `ret_buf` 값**: Return buffer size가 계속 증가하면 HOL blocking 확인
2. **`[NMA-ISSUE]` 로그의 `CMD(host)` vs `REQ(nma)`**: CMD 발행이 0인 구간이 있으면 CMD starvation 확인
3. **`ot=` 값**: Host debug state에서 ot_sum이 threshold(12 × bank수)에 도달하는 시점
4. **`ru=` 값**: RU size가 64에 도달하면 RU capacity blocking 확인
5. **`rt=` 값**: m_rt_pending_count가 0이 아닌데 증가만 하면 RT 발행 지연

**확인 명령어 예시:**
```bash
grep "\[NMA-ISSUE\]" async_co_4.log | awk '{print $0}' | tail -50
grep "\[OT_DEBUG\]" async_co_4.log | tail -20
```

---

## 7. 코드 참조

| 항목 | 파일 | 라인 |
|------|------|------|
| OT threshold 정의 | asyncdimm_host_controller.cpp | 148 |
| OT increment | asyncdimm_host_controller.cpp | 792-797 |
| OT decrement (on_nma_interrupt) | asyncdimm_host_controller.cpp | 425-448 |
| RU entry 생성 (scratchpad→cmd_count) | asyncdimm_host_controller.cpp | 818 |
| schedule_offload_for_rank OT 체크 | asyncdimm_host_controller.cpp | 893-894 |
| active_buffer OT 체크 | asyncdimm_host_controller.cpp | 1044-1045 |
| FRFCFS fallback OT 체크 | asyncdimm_host_controller.cpp | 1090-1091 |
| RU capacity 체크 | asyncdimm_host_controller.cpp | 868 |
| RT issuance (구현됨) | asyncdimm_host_controller.cpp | 623-648 |
| Return buffer HOL check | asyncdimm_nma_controller.cpp | 1341 |
| TDM interrupt rate limit | asyncdimm_nma_controller.cpp | 1344 |
| TDM period/latency 상수 | asyncdimm_nma_controller.cpp | 259-260 |
| CMD FIFO RD_L completion | asyncdimm_nma_controller.cpp | 1232-1234 |
| H/N Arbiter 로직 | asyncdimm_nma_controller.cpp | 978-1090 |
| Fairness parameters 설정 | asyncdimm_system.cpp | 249-257 |
| send() CONCURRENT 허용 | asyncdimm_host_controller.cpp | 467-477 |
