# DBX-DIMM vs AsyncDIMM: Concurrent Mode Host Access 처리 분석

## Context

Host + NDP(NMA) 동시 실행 시, host access의 우선순위를 높이기 위한 사전 분석.
두 아키텍처에서 concurrent mode에서 host request가 어떻게 처리되는지, 어디서 host가 throttle/deprioritize 되는지를 상세히 파악한다.

---

## 1. 아키텍처 개요: Concurrent Mode 구조 비교

### DBX-DIMM
```
Host CPU ─→ [MC (ndpDRAMCtrl)] ─→ DRAM (pCH)
                ↑                      ↑
NDP Unit ─→ [HSNC] → send_ndp_req_to_mc() → MC의 read/write buffer에 합류

Host와 NDP가 *같은 MC의 같은 buffer*를 공유
```

- **단일 MC** 구조: Host와 NDP request가 동일한 `m_read_buffers[pch]` / `m_write_buffers[pch]`에 enqueue
- NDP request는 `is_ndp_req = true` 플래그로 구분
- 스케줄러는 **FRFCFS** — host/NDP 구분 없이 ready + oldest 기준 선택
- **Admission control** (`can_accept_ndp_request()`)로 NDP buffer 점유 제한

### AsyncDIMM
```
Host CPU ─→ [Host MC] ──offload──→ [NMA MC] ─→ DRAM
                │                      ↑
                │                  NMA Program
                └─ host req도 ACTO/RDO/WRO로 offload됨 (Concurrent Mode)
```

- **이중 MC** 구조: Host MC + NMA MC 분리
- Concurrent Mode에서 host request는 Host MC에서 **offload command** (ACTO/PREO/RDO/WRO)로 변환 → NMA MC에 전달
- NMA MC 내부의 **H/N Arbiter**가 per-bank으로 CMD FIFO (host offload) vs REQ FIFO (NMA) 중재
- Host MC는 직접 DRAM 명령을 발행하지 않음 — NMA MC가 실제 DRAM 접근 수행

---

## 2. DBX-DIMM: Host Access 처리 상세

### 2.1 Request Flow (Concurrent Mode)

```
Host Request Flow:
  CPU → send(req) → is_ndp_req=false, is_host_req=true
       → m_read_buffers[pch] 또는 m_write_buffers[pch]에 enqueue (항상 성공)
       → schedule_request() → FRFCFS 선택
       → DRAM command issue

NDP Request Flow:
  HSNC → send_ndp_req_to_mc() (1 req/cycle 제한)
       → send(req) → is_ndp_req=true
       → can_accept_ndp_request() 검사 → 통과 시 같은 buffer에 enqueue
       → schedule_request() → FRFCFS 선택 (host와 경쟁)
       → DRAM command issue
```

**핵심: Host와 NDP request가 같은 buffer에 섞여서 FRFCFS로 경쟁**

### 2.2 NDP Admission Control (`can_accept_ndp_request()`)
**파일**: [ndp_dram_controller.cpp:2892-2909](src/dram_controller/impl/ndp_dram_controller.cpp#L2892-L2909)

```
NDP request 허용 조건:
  Channel-level: ndp_rd_req < m_max_ndp_read_reqs[pch]
  OR Bank-level:  (ndp_rd_req < abs_max) AND (host_access_cnt_per_bank == 0) AND (ndp_per_bank < cap)
```

- Host request는 **항상 accept** (`return true`)
- NDP request만 channel-level cap 적용
- Bank-level 예외: host가 없는 bank에서는 NDP가 더 들어갈 수 있음

### 2.3 Adaptive NDP Throttling (Window-Based)
**파일**: [ndp_dram_controller.cpp:971-1032](src/dram_controller/impl/ndp_dram_controller.cpp#L971-L1032)

#### Long Window (매 `long_win_sz` cycle)
- NDP/Host throughput ratio 계산: `ratio = ndp_acc_out / normal_acc_out`
- `ratio > target_ndp_ratio` → `m_max_ndp_read/write_reqs--` (NDP 줄임)
- `ratio < target_ndp_ratio` → `m_max_ndp_read/write_reqs++` (NDP 늘림)

#### Short Window (매 `win_sz` cycle)
| Host 상태 | NDP 제한 |
|-----------|----------|
| Host 입력 없음 (`normal_acc_in == 0`) | NDP = `high_max / 2` (NDP 자유롭게) |
| Host 입력 있으나 출력 0 (`stall`) | NDP = `low_max` (NDP 대폭 축소) |
| 정상 동작 | Long window ratio 사용 |

**Host stall 감지 → NDP throttle이 핵심 보호 메커니즘**

### 2.4 Row Buffer Cap Management
**파일**: [ndp_dram_controller.cpp:1082-1131](src/dram_controller/impl/ndp_dram_controller.cpp#L1082-L1131)

- **같은 bank에 host + NDP 모두 존재**: row cap = `m_ndp_row_hit_low_cap` (제한)
- **한 쪽만 존재**: row cap = 128 (무제한)

→ Host/NDP가 같은 bank을 공유할 때 row buffer hit 횟수 제한으로 fairness 확보

### 2.5 스케줄링 우선순위
**파일**: [ndp_dram_controller.cpp:2135-2850](src/dram_controller/impl/ndp_dram_controller.cpp#L2135-L2850)

```
1. m_active_buffer  (post-ACT, 최고 우선순위)
2. m_priority_buffer (REF)
3. m_read_buffers[pch] 또는 m_write_buffers[pch]  (watermark에 따라)
   ↳ Host + NDP 모두 여기에 섞여 있음 → FRFCFS
```

- Mode 기반 buffer 검색 순서: `m_mc_db_rw_modes` × `m_db_dram_rw_modes`
- 동일 buffer 내에서는 **is_ndp_req 구분 없이 FRFCFS** (row hit > oldest)

### 2.6 Host 성능 저하 원인 (DBX-DIMM)

| 병목 | 설명 |
|------|------|
| **Buffer 경합** | NDP request가 read/write buffer 공간 점유 → host enqueue 실패 가능 |
| **FRFCFS 경쟁** | NDP가 row hit일 경우 host보다 먼저 선택됨 (age 무관) |
| **Row buffer 오염** | NDP가 열어놓은 row에 host miss 발생 → ACT+PRE 오버헤드 |
| **Admission control이 반응적** | Host stall 감지 후에야 NDP 축소 → 이미 지연 발생 |

---

## 3. AsyncDIMM: Host Access 처리 상세

### 3.1 Request Flow (Concurrent Mode)

```
Host Request Flow:
  CPU → send(req) → Host MC의 m_read/write_buffers[rank]에 enqueue
       → schedule_offload_for_rank() → ACTO/PREO/RDO/WRO 결정
       → DRAM model에 offload cmd 발행 (CA bus 2-cycle, WRO는 DQ bus도)
       → bypass_to_nma() → NMA MC의 CMD FIFO[bank]에 push
       → NMA MC: H/N Arbiter → CMD FIFO에서 pop → 실제 DRAM ACT/RD/WR/PRE 발행
       → Read 완료 → Return Unit → TDM Interrupt → Host MC → RT command → Host에 데이터 반환

NMA Request Flow:
  NMA Program → fetch_nma_instructions() → generate_nma_requests()
       → NMA MC의 REQ FIFO[bank]에 push
       → H/N Arbiter → REQ FIFO에서 pop → 실제 DRAM ACT/RD/WR/PRE 발행
```

**핵심: Host request가 offload command로 변환되어 NMA MC의 CMD FIFO를 통해 실행**

### 3.2 Host MC: Offload Scheduling
**파일**: [asyncdimm_host_controller.cpp:1163-1290](src/dram_controller/impl/asyncdimm_host_controller.cpp#L1163-L1290)

`schedule_offload_for_rank()` — Host MC가 host request를 offload command로 변환:

```
1. RU 용량 체크: m_return_unit.size() >= RU_MAX_ENTRIES (64) → 전체 offload 중단
2. Active buffer 충돌 방지: 이미 ACTO 발행된 bank은 PREO 금지
3. Two-pass FRFCFS:
   Pass 1: 각 bank의 row hit 여부 pre-scan
   Pass 2: 모든 buffer 순회
     - Bank closed → ACTO
     - Row conflict → PREO (단, 해당 bank에 row hit request 있으면 skip)
     - Row hit → RDO/WRO
     - OT backpressure: m_ot_counter[bank] >= HOST_OT_THRESHOLD (8) → skip
   Selection: hit > miss, 동일 priority면 FCFS (oldest)
```

### 3.3 Host MC tick() 우선순위
**파일**: [asyncdimm_host_controller.cpp:762-1040](src/dram_controller/impl/asyncdimm_host_controller.cpp#L762-L1040)

```
tick() 실행 순서:
  1. serve_completed_reads()           — 완료된 read 콜백
  2. RT issuance (최고 우선순위)        — NMA MC로부터 interrupt 받은 read 데이터 수신
     → RT 발행 시 이번 tick의 나머지 스케줄링 전부 SKIP
  3. NMA idle polling + C2H drain      — 모드 전환 감시
  4. Refresh tick                       — CONCURRENT rank은 skip
  5. schedule_request()                 — offload 스케줄링
  6. Unified issue                      — ACTO/PREO/RDO/WRO 발행 + bypass to NMA MC
```

### 3.4 OT (Offload Table) Backpressure
**파일**: [asyncdimm_host_controller.cpp:1003-1012](src/dram_controller/impl/asyncdimm_host_controller.cpp#L1003-L1012)

- 매 offload command 발행 시: `m_ot_counter[bank]++`
- `m_ot_counter[bank] >= HOST_OT_THRESHOLD (8)` → 해당 bank offload 중단
- OT 감소: NMA MC interrupt 수신 시 `on_nma_interrupt()` → `m_ot_counter[bank] -= cmd_count`

**OT는 per-bank → 한 bank이 막혀도 다른 bank은 계속 offload 가능**

### 3.5 Return Unit (RU) & Interrupt 경로
**파일**: [asyncdimm_host_controller.cpp:593-625](src/dram_controller/impl/asyncdimm_host_controller.cpp#L593-L625)

```
Host MC offload RDO → NMA MC CMD FIFO → DRAM RD 완료
  → NMA MC Return Buffer (head-of-line blocking)
  → TDM Interrupt (latency = 2 cycles)
  → Host MC on_nma_interrupt():
    - OT 감소
    - Read → m_rt_read_pending[rank]에 push
  → 다음 tick: RT command 발행 (최고 우선순위)
  → DQ bus로 data 수신 → pending에 추가 → 콜백
```

### 3.6 NMA MC: H/N Arbiter (CMD vs REQ FIFO)
**파일**: [asyncdimm_nma_controller.cpp:1202-1314](src/dram_controller/impl/asyncdimm_nma_controller.cpp#L1202-L1314)

Per-bank round-robin으로 CMD FIFO (host offload) vs REQ FIFO (NMA) 중재:

```
try_issue_concurrent_command():
  1. Refresh → 최고 우선순위 (모든 FIFO blocking)
  2. Per-bank round-robin:
     (A) CMD Service Quota: 강제 CMD 서비스 중이면 CMD 계속
     (B) REQ Capping 검사:
         - Row-hit low cap: 같은 row 연속 hit >= NMA_ROW_HIT_LOW_CAP → REQ "effectively empty"
         - Continuous service cap: REQ 연속 서비스 >= m_req_continuous_cap → REQ "effectively empty"
     (C) Arbiter switch:
         - CMD empty → REQ로 전환
         - REQ empty/capped → CMD로 전환 (SR recovery 수행)
         - Forced CMD interval: REQ가 m_cmd_service_interval 이상 연속 서비스 → quota 강제 부여
     (D) 선택된 FIFO에서 issue (1 cmd per NMA tick)
```

### 3.7 SR Unit (Switch-Recovery)
**파일**: [asyncdimm_nma_controller.cpp:1330-1385](src/dram_controller/impl/asyncdimm_nma_controller.cpp#L1330-L1385)

CMD FIFO ↔ REQ FIFO 전환 시 bank state 불일치 복구:

```
Actual State  |  CMD Front Cmd  |  Recovery
OPENED        |  ACT            |  PRE 발행 (현재 row 닫기)
OPENED        |  RD/WR (다른 row)|  PRE + ACT 삽입
OPENED        |  RD/WR (같은 row)|  없음
OPENED        |  PRE            |  없음
CLOSED        |  *              |  없음
```

### 3.8 Host 성능 저하 원인 (AsyncDIMM)

| 병목 | 설명 |
|------|------|
| **OT Backpressure** | CMD FIFO가 꽉 차면 host offload 중단 → host latency 증가 |
| **H/N Arbiter의 REQ 우선** | Arbiter 초기 상태는 REQ(NMA) → CMD(host)로의 전환 조건 필요 |
| **NMA Row-hit monopoly** | NMA가 같은 row 연속 hit → REQ FIFO가 bank 독점 (row-hit low cap 필요) |
| **SR Recovery 오버헤드** | CMD↔REQ 전환마다 PRE/ACT 복구 → 추가 latency |
| **TDM Interrupt 지연** | NMA MC → Host MC interrupt: TDM latency + head-of-line blocking |
| **RT가 CA bus 점유** | RT 발행 tick에 다른 offload 불가 → throughput 손실 |
| **RU 용량 제한** | `RU_MAX_ENTRIES (64)` 초과 시 전체 offload 중단 |

---

## 4. 핵심 차이점 비교

| 측면 | DBX-DIMM | AsyncDIMM |
|------|----------|-----------|
| **MC 구조** | 단일 MC (공유 buffer) | Host MC + NMA MC (분리) |
| **Host/NDP 경합 지점** | MC의 read/write buffer 내부 (FRFCFS) | NMA MC의 H/N Arbiter (per-bank CMD vs REQ) |
| **Host 보호 메커니즘** | Adaptive NDP throttle (window-based) + Row cap | OT backpressure + cmd_service_interval/quota |
| **Host stall 감지** | Short window: `normal_acc_out == 0` → NDP throttle | 없음 (OT/RU 용량 기반 implicit) |
| **Row buffer 정책** | 공유 bank에서 row_hit_low_cap 강제 | H/N Arbiter: NMA row-hit low cap |
| **NDP 제한 단위** | Per-channel + per-bank | Per-bank (OT) |
| **Host 우선순위** | Admission 시 항상 accept; 스케줄링에서는 동등 | RT 최고 우선순위; offload은 NMA와 arbiter 경쟁 |
| **반응 속도** | Window 기반 (수백~수천 cycle 단위) | Per-command (OT는 즉각적) |

---

## 5. Host 우선순위를 높이기 위한 잠재적 수정 지점

### DBX-DIMM
1. **`can_accept_ndp_request()`**: `m_max_ndp_read_reqs` 초기값/상한 축소
2. **`schedule_request()` FRFCFS**: host request에 ready 우선순위 부여 (is_host_req 기반)
3. **Row cap**: `m_ndp_row_hit_low_cap` 값 축소 → NDP row hit 횟수 제한 강화
4. **Window 파라미터**: `target_ndp_ratio` 축소 → NDP 비율 목표 낮춤
5. **Short window**: Host stall 감지 임계값을 더 민감하게

### AsyncDIMM
1. **H/N Arbiter**: CMD FIFO (host) 우선순위 상향
   - `m_cmd_service_interval` 축소: 더 자주 CMD 서비스
   - `m_cmd_service_quota` 증가: CMD 서비스 시 더 많은 명령 처리
   - `m_req_continuous_cap` 축소: REQ(NMA) 연속 서비스 횟수 제한
2. **NMA_ROW_HIT_LOW_CAP** 축소: NMA의 row hit monopoly 제한
3. **OT threshold**: HOST_OT_THRESHOLD 증가 → host offload 더 적극적으로
4. **RU_MAX_ENTRIES** 증가: host read offload 더 많이 허용
5. **TDM Interrupt 빈도**: period 축소 → 더 빠른 OT 회수

---

## 6. 관련 파일 목록

| 파일 | 역할 |
|------|------|
| [ndp_dram_controller.cpp](src/dram_controller/impl/ndp_dram_controller.cpp) | DBX-DIMM MC: admission control, adaptive throttle, FRFCFS |
| [ndp_DRAM_system.cpp](src/memory_system/impl/ndp_DRAM_system.cpp) | DBX-DIMM System: HSNC, send_ndp_req_to_mc(), trace_core |
| [asyncdimm_host_controller.cpp](src/dram_controller/impl/asyncdimm_host_controller.cpp) | AsyncDIMM Host MC: offload scheduling, OT, RT, RU |
| [asyncdimm_nma_controller.cpp](src/dram_controller/impl/asyncdimm_nma_controller.cpp) | AsyncDIMM NMA MC: H/N Arbiter, CMD/REQ FIFO, SR Unit |
| [asyncdimm_system.cpp](src/memory_system/impl/asyncdimm_system.cpp) | AsyncDIMM System: mode transitions, trace_core |
| [request.h](src/base/request.h) | Request struct: is_ndp_req, is_host_req flags |
| [generic_scheduler.cpp](src/dram_controller/impl/scheduler/generic_scheduler.cpp) | FRFCFS scheduler |
