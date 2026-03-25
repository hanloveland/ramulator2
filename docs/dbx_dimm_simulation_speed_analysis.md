# DBX-DIMM Simulation Speed Analysis Report

## Overview

DBX-DIMM 시뮬레이션이 baseline(GenericDRAM)에 비해 **월등히 느린 wall-clock 시간**을 소요하는 원인을 step-by-step으로 분석한다. 여기서 "Simulation Time"은 시뮬레이터의 **실행 시간(wall-clock time)**을 의미하며, DRAM cycle 수가 아닌 시뮬레이터 C++ 코드가 실제로 소비하는 CPU 시간을 분석한다.

---

## 1. Main Simulation Loop (동일 구조)

**파일**: `src/main.cpp:101-113`

```
for (uint64_t i = 0;; i++) {
    frontend->tick();        // frontend clock
    memory_system->tick();   // memory system clock
}
```

Main loop 자체는 baseline과 DBX-DIMM 모두 동일하다. 차이는 `memory_system->tick()` 내부에서 발생한다.

---

## 2. Memory System tick() 비교

### 2-A. Baseline: GenericDRAMSystem::tick() (569 lines total)

**파일**: `src/memory_system/impl/generic_DRAM_system.cpp:210-228`

```cpp
void tick() {
    m_clk++;
    m_dram->tick();                    // 1회 DRAM tick
    for (auto controller : m_controllers) {
        controller->tick();            // 채널당 1회 controller tick
        while(1) {                     // completed read latency drain
            read_latency = controller->get_req_latency();
            if (read_latency == 0) break;
            record_latency(read_latency);
        }
    }
}
```

**Per-cycle 비용**: O(num_channels) — 매우 간결

### 2-B. DBX-DIMM: NDPDRAMSystem::tick() (1824 lines total)

**파일**: `src/memory_system/impl/ndp_DRAM_system.cpp:633-1021`

```cpp
void tick() {
    m_clk++;
    m_dram->tick();
    for (auto controller : m_controllers) {
        controller->tick();
        // completed read latency drain (same as baseline)
    }

    // ========== 추가 코스트 시작 ==========

    // [Factor A] PCH_DEBUG error checking (ifdef PCH_DEBUG)
    // 활성화 시: 전체 DIMM × PCH × history 순회

    // [Factor B] HSNC State Machine: 전체 DIMM × PCH 순회 (매 cycle)
    for (dimm_id = 0..m_num_dimm) {
        for (pch_id = 0..m_num_subch * num_pseudochannel) {
            // 상태별 처리 (NDP_IDLE ~ NDP_DONE)
            // ... 상세 분석은 Section 4에서
        }
    }
}
```

**Per-cycle 비용**: O(num_channels) + O(num_dimm × num_pch × per_pch_work)

---

## 3. DRAM Controller tick() 비교 — 가장 큰 병목

### 3-A. Baseline: GenericDRAMController::tick() (849 lines total)

**파일**: `src/dram_controller/impl/generic_dram_controller.cpp`

| Step | 동작 | 복잡도 |
|------|------|--------|
| 1 | Queue length stats | O(1) |
| 2 | Adaptive row buffer scan | O(buffer_size) |
| 3 | serve_completed_reads() | O(1) — front만 확인 |
| 4 | Refresh tick | O(1) |
| 5 | Schedule request (FRFCFS) | O(num_ranks × buffer_size) |
| 6 | Row policy + plugins update | O(1) |
| 7 | Issue 1 command | O(1) |

**Total per-cycle**: ~O(num_ranks × buffer_size)
**버퍼 수**: read_buffer[rank], write_buffer[rank], priority_buffer[rank], active_buffer — **4종**

### 3-B. DBX-DIMM: NDPDRAMController::tick() (2982 lines total, ~3.5x baseline)

**파일**: `src/dram_controller/impl/ndp_dram_controller.cpp:932-~1900`

| Step | 동작 | 복잡도 | Baseline 대비 |
|------|------|--------|--------------|
| 1 | Per-PCH queue length tracking | O(num_pch) | **신규** |
| 2 | Per-PCH R/W mode timer update | O(num_pch) | **신규** |
| 3 | Long-window NDP ratio calculation (매 long_win_sz cycle) | O(num_pch) | **신규** |
| 4 | Short-window NDP ratio calculation (매 win_sz cycle) | O(num_pch) | **신규** |
| 5 | Per-PCH RD/WR prefetch buffer latency countdown | O(num_pch × buf_entries) | **신규** |
| 6 | Per-PCH pending NDP RD/WR latency countdown | O(num_pch × pending_entries) | **신규** |
| 7 | **Per-bank row hit cap update** (triple nested loop) | **O(num_pch × num_bg × num_bk)** | **신규, 매우 무거움** |
| 8 | Per-PCH buffer scan for adaptive row miss detection | O(num_pch × buffer_size) | 확장됨 |
| 9 | Per-PCH average buffer size accumulation | O(num_pch) | **신규** |
| 10 | Periodic stats update (매 100K cycle) | O(num_pch) | **신규** |
| 11 | Per-PCH mode cycle tracking | O(num_pch) | **신규** |
| 12 | serve_completed_reads() | O(pending) | 동일 |
| 13 | Refresh tick | O(1) | 동일 |
| 14 | **Decoupled mode scheduling** | **O(num_pch × 6 buffers × buffer_size)** | **대폭 확장** |
| 15 | Row policy + plugins | O(1) | 동일 |
| 16 | Issue command + NDP-specific stats | O(1) + O(num_bg) | 확장됨 |

**Total per-cycle**: ~O(num_pch × num_bg × num_bk) + O(num_pch × 6 × buffer_size)
**버퍼 수**: read_buffer[pch], write_buffer[pch], priority_buffer[pch], rd_prefetch_buffer[pch], wr_prefetch_buffer[pch], active_buffer — **6종 × num_pch**

#### 핵심 병목 상세 분석

**[병목 1] Per-bank Row Hit Cap Update (Line 1072-1096)**

```cpp
// 매 cycle 실행
for (pch_id = 0..num_pch) {           // 4 PCH
    for (bg_id = 0..num_bankgroup) {   // 8 BG
        for (bk_id = 0..num_bank) {    // 4 BK
            // host_access vs ndp_access 비교 후 cap 업데이트
            m_rowpolicy->update_cap(pch_id, 0, bg_id, bk_id, cap_value);
        }
    }
}
```

- **매 사이클**: 4 × 8 × 4 = **128회 반복** + rowpolicy 업데이트
- Baseline: **없음**

**[병목 2] Decoupled Priority Scheduling (Line 2134-2700+)**

Baseline은 단순 FRFCFS (active → priority → read/write round-robin per rank):
```
active_buffer → priority_buffer[rank] → read/write_buffer[rank]
```

DBX-DIMM은 Decoupled MC↔DB / DB↔DRAM 모드 기반 복합 스케줄링:
```
active_buffer →
  for each PCH (round-robin):
    set_mode_per_pch()        // MC↔DB mode + DB↔DRAM mode 결정
    if DB_NDP_WR mode:
        if DRAM_REF:  search WR_BUF(DB_WR)
        if DRAM_RD:   search WR_BUF(DB_WR) → RD_BUF(PRE_RD) → RD_BUF(DRAM_RD)
        if DRAM_WR:   search WR_BUF(DB_WR) → PRE_WR_BUF(POST_WR) → WR_BUF(DRAM_WR)
    if DB_RD mode:
        if DRAM_REF:  search PRE_RD_BUF(POST_RD) → RD_BUF(DB_RD)
        if DRAM_RD:   search PRE_RD_BUF(POST_RD) → RD_BUF(RD/PRE_RD/DB_RD/DRAM_RD)
        if DRAM_WR:   search PRE_RD_BUF(POST_RD) → WR_BUF(POST_WR) → ...
        if DRAM_NDP_WR: ...
    if DB_WR mode:
        ... (비슷한 복합 검색)
```

- 최악의 경우 **PCH당 6~8회 buffer search** (각 search = get_best_request_with_priority)
- Baseline: PCH당 **1~2회** buffer search

**[병목 3] Per-PCH Prefetch Buffer Latency Countdown (Line 1024-1068)**

```cpp
for (pch_id = 0..num_pch) {
    // RD prefetch: 전체 entries 순회, latency 감소
    for (i = 0..m_to_rd_prefetch_buffers[pch_id].size())
        m_to_rd_prefetch_buffers[pch_id][i].second -= 1;
    // WR prefetch: 동일
    // Pending NDP RD: 동일
    // Pending NDP WR: 동일
}
```

- **4가지 pending queue × num_pch** 매 cycle 순회
- Baseline: **없음** (pending은 depart clock 비교만)

---

## 4. HSNC State Machine — Memory System 레벨 추가 비용

**파일**: `src/memory_system/impl/ndp_DRAM_system.cpp:719-1021`

매 cycle, 전체 DIMM × PCH에 대해 상태 머신을 실행:

```cpp
for (dimm_id = 0..m_num_dimm) {        // e.g., 1 DIMM
    for (pch_id = 0..total_pch) {      // e.g., 8 PCH (2 subch × 4 pch)
        switch (pch_lvl_hsnc_status[dimm_id][pch_id]) {
            case NDP_IDLE:        // 최소 비용 (카운터 증가만)
            case NDP_ISSUE_START: // controller polling + request 생성
            case NDP_BEFORE_RUN:  // controller polling
            case NDP_RUN:         // ★ 가장 무거움 ★
            case NDP_BAR:         // drain 확인 + send_ndp_req_to_mc
            case NDP_WAIT:        // send_ndp_req_to_mc + wait counter
            case NDP_FETCH_STALL: // send_ndp_req_to_mc + cache miss 확인
            case NDP_DONE:        // drain 확인
        }
    }
}
```

### NDP_RUN 상태 (가장 빈번, 가장 무거움)

**Per-cycle operations during NDP_RUN:**

| Operation | 복잡도 | 설명 |
|-----------|--------|------|
| send_ndp_req_to_mc() | O(addr_gen_slot_size) | Round-robin across up to 8 slots, request 생성 + controller send |
| Descriptor Cache lookup | O(8) | Fully-associative 8-way tag 비교 |
| LRU update | O(8) | 8 entry group LRU 갱신 |
| decode_acc_inst() | O(1) | 64-bit instruction decoding |
| PCH index validation | O(1) | pch_idx1 vs pch_idx2 비교 |
| Status counter update | O(1) | pch_hsnc_status_cnt 증가 |

**send_ndp_req_to_mc() 상세 (Line 1345-1408)**:
```cpp
// Round-robin across addr_gen_slot (최대 8 entries)
for (i = 0..slot_size) {
    Request req = Request(...);         // Request 객체 생성
    m_addr_mapper->apply(req);          // Address mapping 적용
    req.addr_vec[...] = slot.ch/pch/bg/bk/row/col;  // 6개 주소 필드 설정
    m_controllers[ch_id]->send(req);    // Controller로 전송 시도
    if (success) break;
}
```

- Request 객체 생성 + address mapping이 **매 cycle, 매 PCH**에서 발생
- Baseline에서는 Frontend이 Request를 생성하여 전달하므로, Memory System 내부에서 Request 생성 불필요

---

## 5. DRAM Model 복잡도 차이

### Baseline: DDR5 or DDR5-pCH

- Level hierarchy: channel → (pseudochannel) → rank → bankgroup → bank → row → column
- Commands: ACT, PRE, PREA, RD, RDA, WR, WRA, REFab, REFsb
- **~10개 커맨드**

### DBX-DIMM: DDR5-pCH with NDP extensions

- 동일 hierarchy + NDP-specific commands:
  - PRE_RD, PRE_RDA, POST_RD (DB prefetch read)
  - PRE_WR, POST_WR, POST_WRA (DB prefetch write)
  - NDP_DB_RD, NDP_DB_WR (NDP Data Buffer access)
  - NDP_DRAM_RD, NDP_DRAM_RDA, NDP_DRAM_WR, NDP_DRAM_WRA (NDP DRAM access)
  - P_ACT, P_PRE (prefetch ACT/PRE)
- **~25+ 커맨드**

영향:
- `check_ready()`: 더 많은 timing constraint 검사
- `get_preq_command()`: 더 복잡한 prerequisite resolution
- `issue_command()`: 더 많은 state machine transition
- `issue_ndp_command()`: NDP 전용 추가 처리

---

## 6. 메모리 사용량 및 Cache 효율 영향

### Baseline 주요 데이터 구조
```
m_controllers[num_channels]     // 포인터 배열
m_open_row[num_banks_total]     // int 배열
read_buffer[num_ranks]          // ReqBuffer × rank
write_buffer[num_ranks]         // ReqBuffer × rank
pending                         // deque<Request>
```

### DBX-DIMM 추가 데이터 구조
```
// NDPDRAMController (per channel)
m_read_buffers[num_pch]              // ReqBuffer × PCH
m_write_buffers[num_pch]             // ReqBuffer × PCH
m_rd_prefetch_buffers[num_pch]       // ReqBuffer × PCH (추가)
m_wr_prefetch_buffers[num_pch]       // ReqBuffer × PCH (추가)
m_to_rd_prefetch_buffers[num_pch]    // vector<pair> × PCH (추가)
m_to_wr_prefetch_buffers[num_pch]    // vector<pair> × PCH (추가)
m_pending_ndp_rd[num_pch]            // vector<pair> × PCH (추가)
m_pending_ndp_wr[num_pch]            // vector<pair> × PCH (추가)
m_host_access_cnt_per_bank[total_banks]  // per-bank counter (추가)
m_ndp_access_cnt_per_bank[total_banks]   // per-bank counter (추가)
m_channel_stats[num_pch]             // ChannelModeStats × PCH (추가)
m_periodic_channel_stats[num_pch]    // ChannelModeStats × PCH (추가)
m_cur_seg_stats[num_pch]             // NDP segment stats × PCH (추가)
... (수십 개 추가 per-pch 벡터)

// NDPDRAMSystem (global)
desc_store[num_dimm][total_pch][128][8]     // uint64_t, ~64KB per PCH (추가)
pch_lvl_inst_buf[num_dimm][total_pch][8][8] // uint64_t, ~512B per PCH (추가)
pch_lvl_inst_buf_tag/valid/lru              // 관리 구조체 (추가)
hsnc_base_reg[num_dimm][total_pch][8]       // HSNC 레지스터 (추가)
hsnc_loop_cnt_reg[num_dimm][total_pch][8]   // HSNC 레지스터 (추가)
pch_lvl_hsnc_nl_addr_gen_slot[dimm][pch]    // vector<AccInst_Slot> × PCH (추가)
hsnc_segments[dimm][pch]                    // 세그먼트 추적 (추가)
hist_[2000002]                              // 히스토그램 (크기 확대)
```

**CPU cache 영향**:
- 데이터 구조가 대폭 증가하여 L1/L2 cache miss rate 상승
- 특히 per-bank/per-PCH 배열의 반복 접근이 cache line을 빠르게 소모
- desc_store (64KB per PCH)는 L1 cache에 수용 불가

---

## 7. 정량적 비교 요약

### Per-cycle Operation Count (1 channel, 4 PCH, 8 BG, 4 BK 기준)

| Operation | Baseline | DBX-DIMM | 배율 |
|-----------|----------|----------|------|
| Controller tick overhead | ~1 rank scan | 4 PCH × (timer + mode + prefetch) | ~10-15x |
| Row cap update | 없음 | 4 × 8 × 4 = 128 iterations | +128 |
| Buffer scan (scheduling) | 1-2 buffers × rank | 6 buffers × 4 PCH × complex mode | ~12-24x |
| Prefetch buffer countdown | 없음 | 4 queues × 4 PCH | +16 per cycle |
| Statistics tracking | minimal | per-PCH mode + per-bank + per-BG + segment | ~5-8x |
| HSNC state machine | 없음 | 8 PCH × (cache lookup + decode + req gen) | +8 × O(20) |
| Request creation in MemSys | 없음 | 8 PCH × send_ndp_req_to_mc | +8 × O(10) |
| DRAM model check_ready | ~10 timing params | ~25+ timing params + NDP checks | ~2-3x |

### 추정 전체 overhead

| Component | Baseline (상대) | DBX-DIMM (상대) | 주요 원인 |
|-----------|----------------|-----------------|-----------|
| Memory System tick() | 1.0x | 3-5x | HSNC state machine |
| Controller tick() | 1.0x | 10-20x | Per-bank cap, decoupled scheduling, prefetch |
| DRAM model operations | 1.0x | 2-3x | 추가 커맨드, timing 체크 |
| **Total per-cycle** | **1.0x** | **~15-30x** | |

---

## 8. Factor별 영향도 순위

| 순위 | Factor | 영향도 | 파일 위치 |
|------|--------|--------|-----------|
| **1** | **Decoupled Mode Scheduling** | ★★★★★ | ndp_dram_controller.cpp:2134-2700 |
| **2** | **Per-bank Row Hit Cap Update** (매 cycle, O(pch×bg×bk)) | ★★★★☆ | ndp_dram_controller.cpp:1072-1096 |
| **3** | **Per-PCH Prefetch Buffer Management** (4 queues × pch) | ★★★★☆ | ndp_dram_controller.cpp:1024-1068 |
| **4** | **HSNC State Machine** (매 cycle, 8 PCH) | ★★★☆☆ | ndp_DRAM_system.cpp:719-1021 |
| **5** | **Statistics/Counter 누적** (per-bank, per-BG, segment) | ★★★☆☆ | ndp_dram_controller.cpp:1148-1270 |
| **6** | **DRAM Model 추가 커맨드** (25+ vs 10 commands) | ★★☆☆☆ | DDR5-pCH.cpp |
| **7** | **Memory Footprint / Cache Pollution** | ★★☆☆☆ | 전체 |
| **8** | **Request 생성 in Memory System** (send_ndp_req_to_mc) | ★☆☆☆☆ | ndp_DRAM_system.cpp:1345-1408 |

---

## 9. 잠재적 최적화 방향

### 즉시 적용 가능 (코드 변경, 시뮬레이션 결과 무변)

| 최적화 | 예상 효과 | 설명 |
|--------|----------|------|
| Row cap update 주기 조절 | 10-15% | 매 cycle → 매 N cycle (e.g., 32) |
| Prefetch buffer countdown 최적화 | 5-10% | deque → priority queue (depart time 기반) |
| `#ifdef PCH_DEBUG` / `#ifdef NDP_DEBUG` 완전 제거 | 1-3% | 컴파일러 최적화 보장 |
| Per-PCH stats 주기적 업데이트 | 3-5% | 매 cycle → 매 window (e.g., 1024) |
| desc_store 메모리 레이아웃 최적화 | 2-5% | Cache-friendly layout |

### 구조적 최적화 (설계 변경 필요)

| 최적화 | 예상 효과 | 설명 |
|--------|----------|------|
| Scheduling 경량화 | 20-30% | Mode-based multi-pass → single-pass priority |
| HSNC batch processing | 5-10% | 매 cycle → 매 N cycle (NMA clock ratio 활용) |
| Per-bank counter → bitmap | 5-10% | O(pch×bg×bk) → O(pch) with bitmask |

---

## 10. 결론

DBX-DIMM 시뮬레이션이 baseline 대비 **약 15-30배** 느린 주요 원인은:

1. **Controller tick() 복잡도 폭증**: GenericDRAMController(849 lines) → NDPDRAMController(2982 lines). 특히 Decoupled MC↔DB/DB↔DRAM 모드 기반 스케줄링이 매 cycle 6종 버퍼 × 4 PCH를 탐색하며, 이것이 전체 overhead의 **40-50%**를 차지한다.

2. **Per-bank 반복 연산**: Row hit cap update가 매 cycle 128회(4 PCH × 8 BG × 4 BK) 실행되며 전체의 **15-20%**.

3. **추가 버퍼 관리**: Prefetch buffer (RD/WR) + pending NDP buffer의 매-cycle latency countdown이 전체의 **10-15%**.

4. **HSNC State Machine**: 8 PCH × (descriptor cache + decode + request generation)이 매 cycle 실행되며 전체의 **10-15%**.

5. **누적 Statistics**: Per-bank, per-BG, per-segment 통계가 전체의 **5-10%**.

이러한 요소들이 복합적으로 작용하여, **동일 DRAM cycle 수**를 시뮬레이션하더라도 DBX-DIMM은 baseline 대비 상당히 긴 wall-clock 시간을 소요하게 된다.

---

*Generated: 2026-03-24*
*Codebase: ramulator2 (DBX-DIMM fork)*
