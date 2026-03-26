#ifndef RAMULATOR_ASYNCDIMM_NMA_CONTROLLER_H
#define RAMULATOR_ASYNCDIMM_NMA_CONTROLLER_H

#include <vector>
#include <deque>
#include <iostream>
#include <spdlog/spdlog.h>

#include "base/base.h"
#include "base/request.h"
#include "dram/dram.h"

namespace Ramulator {

/**
 * AsyncDIMM NMA (Near-Memory Accelerator) Memory Controller
 *
 * Resides on the buffer chip, one per rank.
 *
 * Phase 1 (Host Mode): Bypass path only
 *   - Receives all host commands via bypass_command()
 *   - Maintains shadow bank FSM (mirrors Host MC's DRAM state)
 *   - No DRAM command issuing
 *   - Refresh completion tracked internally via future actions
 *
 * Phase 2 (NMA Mode): Independent DRAM access
 *   - NMAInst arrives via DQ magic path (bypass_command intercepts WR to
 *     special addresses; no separate Host→NMA software write path)
 *   - ctrl-reg WR → m_nma_start_pending; system polls and triggers H2N
 *   - inst-buf WR → decode_nma_inst() → m_nma_program
 *   - NMA state machine (IDLE → ISSUE_START → RUN → BAR → WAIT → DONE)
 *   - Per-bank REQ FIFOs: m_nma_req_buffer[flat_bank_id] (max 8 per bank)
 *   - Scheduler issues front of each bank's FIFO (round-robin, first ready)
 *   - comp_opcode → memory access type mapping (RD/WR/none/control)
 *   - Uses shadow FSM to determine ACT/PRE prerequisites
 *   - Handles refresh internally during NMA Mode
 *   - VMA-internal instructions (T_* ops: Y/Y2 from VMA) modeled with fixed latency
 *   - LOOP instruction for iteration support
 *
 * Phase 3 (Concurrent Mode): [Placeholder]
 *   - CMD FIFO per-bank (receive decoded host offloaded commands)
 *   - H/N Arbiter (switch CMD FIFO vs REQ FIFO)
 *   - SR Unit (Switch-Recovery)
 *   - Global Completion Buffer (Return Unit)
 */
class AsyncDIMMNMAController {
  public:
    // Per-bank state in the shadow FSM
    enum class BankState {
      CLOSED = 0,
      OPENED,
      REFRESHING,
    };

    struct BankFSM {
      BankState state = BankState::CLOSED;
      int open_row = -1;       // Currently open row (-1 if closed)
    };

    // NMA execution state machine (analogous to DBX-DIMM's NDP_CTRL_STATUS)
    enum class NMAState {
      NMA_IDLE = 0,        // No NMA workload active
      NMA_ISSUE_START,     // Waiting for all banks to be ready before starting
      NMA_RUN,             // Executing: decode NMAInst → generate DRAM commands
      NMA_BAR,             // Barrier: wait for all outstanding NMA requests to drain
      NMA_WAIT,            // Fixed-latency wait (compute-only instructions)
      NMA_DONE,            // All instructions executed, wait for outstanding requests to drain
    };

  private:
    int m_rank_id = -1;
    int m_channel_id = -1;
    IDRAM* m_dram = nullptr;

    // Shadow bank FSM: indexed by [bankgroup * num_banks + bank]
    int m_num_bankgroups = -1;
    int m_num_banks = -1;
    int m_total_banks = -1;
    std::vector<BankFSM> m_bank_fsm;

    // Level indices for parsing addr_vec
    int m_channel_level = -1;
    int m_rank_level = -1;
    int m_bankgroup_level = -1;
    int m_bank_level = -1;
    int m_row_level = -1;
    int m_column_level = -1;

    // Future actions: NMA MC tracks refresh completion internally
    struct FutureAction {
      enum class Type { REFAB_END, REFSB_END };
      Type type;
      AddrVec_t addr_vec;
      Clk_t complete_clk;
    };
    std::vector<FutureAction> m_future_actions;

    // Timing values cached from DRAM model
    int m_nRFC1 = -1;
    int m_nRFCsb = -1;
    int m_nREFI = -1;     // Refresh interval (for NMA Mode refresh)

    // ===== Magic Path: DQ-based NMAInst and ctrl-reg write interception =====
    // NMA MC intercepts WR commands to these addresses via bypass_command().
    // No separate Host→NMA software write path exists.
    int m_nma_ctrl_row = -1;  // Row address shared by both regions
    int m_nma_ctrl_bg  = -1;  // BG of control register (start trigger)
    int m_nma_ctrl_bk  = -1;  // BK of control register
    int m_nma_buf_bg   = -1;  // BG of NMAInst instruction buffer
    int m_nma_buf_bk   = -1;  // BK of NMAInst instruction buffer

    // Set when ctrl-reg WR is detected via magic path; polled by asyncdimm_system
    bool m_nma_start_pending = false;

    // ===== Phase 2: NMA Mode State Machine =====
    NMAState m_nma_state = NMAState::NMA_IDLE;

    // NMA program: loaded from DQ magic-path WR intercept.
    // PC-based execution (DBX-DIMM style): m_nma_pc indexes into m_nma_program.
    std::vector<NMAInst_Slot> m_nma_program;          // Program buffer (vector, max 1024)
    static constexpr int NMA_INST_MAX = 1024;         // 8KB SRAM / 8B per NMAInst = 1024
    int m_nma_pc = 0;                                 // Program counter

    // Base address registers: effective_row = base_reg[id] + inst.row
    // Set via SET_BASE instruction, incremented via INC_BASE instruction.
    // Default 0 → backward compatible (effective_row = inst.row).
    int m_base_reg[8] = {0};

    // Address generation slots
    // Active instructions being expanded into memory requests
    struct AddrGenSlot {
      NMAInst_Slot inst;
      int total_accesses;  // Total DRAM accesses needed
      int issued_count;    // Accesses issued so far
      int current_col;     // Current column for sequential access
    };
    std::vector<AddrGenSlot> m_addr_gen_slots;
    static constexpr int ADDR_GEN_SLOT_MAX = 8;

    // Per-bank REQ FIFOs: m_nma_req_buffer[flat_bank_id]
    // Scheduler checks front of each bank FIFO and issues the first ready command.
    // flat_bank_id = bankgroup * num_banks + bank
    struct NMARequest {
      bool is_read = true;
      AddrVec_t addr_vec;
      int command = -1;       // Current command to issue (ACT/RD/WR/PRE)
      int final_command = -1; // Final command needed (RD or WR)
      Clk_t arrive = -1;
      bool data_pending = false;      // Waiting for DRAM data response
      Clk_t data_complete_clk = -1;   // Clock when data response arrives
    };
    std::vector<std::deque<NMARequest>> m_nma_req_buffer;  // [total_banks]
    static constexpr int NMA_REQ_BUFFER_PER_BANK = 8;

    // Round-robin bank index for per-bank front scheduling
    int m_req_rr_bank_idx = 0;

    // NMA Wait counter (for compute-only instructions and fixed-latency waits)
    int m_nma_wait_cnt = 0;
    int m_nma_wait_target = 0;

    // Compute latency constants (in NMA clock ticks) for VMA-internal (NONE type) ops.
    // All T_* ops modeled as 1 NMA tick latency.
    static constexpr int COMPUTE_LATENCY_T_ADD  = 4;  // Z = Y + Y2
    static constexpr int COMPUTE_LATENCY_T_MUL  = 4;  // Z = Y * Y2
    static constexpr int COMPUTE_LATENCY_T_VRED = 4;  // Z += Y
    static constexpr int COMPUTE_LATENCY_T_SRED = 4;  // z = SUM(Y)
    static constexpr int COMPUTE_LATENCY_T_MAC  = 4;  // Z += Y * Y2

    // NMA Mode refresh management
    Clk_t m_last_refresh_clk = 0;
    bool m_nma_refresh_pending = false;

    // ===== Statistics =====
    // Phase 1: bypass stats
    size_t s_num_bypass_received = 0;
    size_t s_num_act_bypass = 0;
    size_t s_num_pre_bypass = 0;
    size_t s_num_rd_bypass = 0;
    size_t s_num_wr_bypass = 0;
    size_t s_num_ref_bypass = 0;
    size_t s_num_magic_inst_wr = 0;  // NMAInst writes intercepted via magic path
    size_t s_num_magic_ctrl_wr = 0;  // Ctrl-reg writes intercepted via magic path

    // Phase 2: NMA execution stats
    size_t s_num_nma_act = 0;
    size_t s_num_nma_pre = 0;
    size_t s_num_nma_rd = 0;
    size_t s_num_nma_wr = 0;
    size_t s_num_nma_ref = 0;
    size_t s_num_nma_instructions = 0;
    size_t s_num_nma_executions = 0;
    size_t s_num_nma_compute_only = 0;
    size_t s_num_nma_loops = 0;

    // NMA state cycle counters
    size_t s_nma_idle_cycles = 0;
    size_t s_nma_issue_start_cycles = 0;
    size_t s_nma_run_cycles = 0;
    size_t s_nma_bar_cycles = 0;
    size_t s_nma_wait_cycles = 0;
    size_t s_nma_done_cycles = 0;

    Clk_t m_clk = 0;

    // NMA clock ratio: NMA logic runs at 1/4 of DRAM clock
    // (on-DIMM digital logic cannot run at full DRAM clock speed)
    static constexpr int NMA_CLOCK_RATIO = 1;

    // ===== Phase 3: Concurrent Mode =====

    bool m_concurrent_mode = false;
    bool m_accept_offload = true;   // false after NMA_IDLE in concurrent mode (reject new offloads)

    // CMD FIFO (per-bank): receives decoded host offload commands (ACTO→ACT, RDO→RD, etc.)
    struct CmdFifoEntry {
      int command;         // Decoded DRAM command (ACT/PRE/RD/WR)
      AddrVec_t addr_vec;
      Clk_t arrive;
      bool is_read;
      uint64_t seq_num;    // Global seq number (valid only for reads)
    };
    std::vector<std::deque<CmdFifoEntry>> m_cmd_fifo;  // [total_banks]
    static constexpr int NMA_CMD_FIFO_SIZE = 80;       // Per-bank CMD FIFO depth (10x for testing)
    // Effective FIFO capacities (halved in concurrent mode per paper §V.B.2)
    int m_eff_cmd_fifo_size = NMA_CMD_FIFO_SIZE;
    int m_eff_req_fifo_size = NMA_REQ_BUFFER_PER_BANK;

    // H/N Arbiter: per-bank FIFO selection (true = CMD FIFO, false = REQ FIFO)
    std::vector<bool> m_arbiter_use_cmd;  // [total_banks]

    // SR Unit: per-bank Bank State Change (BSC) tracking
    // Records last ACT/PRE issued from CMD FIFO and REQ FIFO respectively.
    // On FIFO switch, compares expected bank state vs actual to determine recovery.
    struct SRState {
      // CMD FIFO's view of the bank
      int last_cmd_bsc   = -1;   // Last BSC cmd id from CMD FIFO (-1=none)
      int cmd_open_row   = -1;   // Expected open row per CMD FIFO perspective
      // REQ FIFO's view of the bank
      int last_req_bsc   = -1;   // Last BSC cmd id from REQ FIFO (-1=none)
      int req_open_row   = -1;   // Expected open row per REQ FIFO perspective
    };
    std::vector<SRState> m_sr_unit;  // [total_banks]

    // Return Unit (NMA MC side): tracks offloaded reads for interrupt generation
    struct ReturnEntry {
      uint64_t seq_num;
      bool is_done = false;
      int bg = -1;
      int bk = -1;
      bool is_read = false;
    };
    std::deque<ReturnEntry> m_return_buffer;  // Ordered by offload arrival
    uint64_t m_next_seq_num = 0;

    // Debug: log of interrupt-generating entries (NMA side)
    struct InterruptLogEntry {
      uint64_t seq_num;
      int bg, bk;
      bool is_read;
      Clk_t clk;
    };
    std::vector<InterruptLogEntry> s_interrupt_log;

    // Per-command CMD FIFO issue log (NMA side, tracks what NMA actually issues to DRAM)
    struct CmdFifoIssueLogEntry {
      int bg, bk;
      int cmd_type;  // 0=ACT, 1=PRE, 2=RD, 3=WR
      Clk_t clk;
    };
    std::vector<CmdFifoIssueLogEntry> s_cmd_fifo_issue_log;

    // Debug: NMA MC side offload command group tracking for target bank (Ch0 Rk0 BG7 BK1)
    int m_debug_target_bg = -1;
    int m_debug_target_bk = -1;
    // Current in-progress command group (accumulates PRE→ACT→RD/WR)
    struct NMACmdGroup {
      long first_clk = 0;     // clk of first cmd in group
      long last_clk = 0;      // clk of final RD/WR
      int row = -1;
      int cmd_count = 0;      // number of commands in this group
      bool has_pre = false;
      bool has_act = false;
      bool has_rdwr = false;   // RD_L or WR_L
      bool is_read = false;
    };
    NMACmdGroup m_debug_nma_cur_group;
    struct NMACmdGroupLog {
      long first_clk;
      long last_clk;
      int row;
      int cmd_count;         // 1=RD, 2=ACT+RD, 3=PRE+ACT+RD
      bool is_read;
      std::string seq;       // e.g. "PRE→ACT→RD"
    };
    std::vector<NMACmdGroupLog> m_debug_nma_cmd_groups;

    // Interrupt signal to Host MC
    using InterruptCallback = std::function<void(int rank_id, int batch_size)>;
    InterruptCallback m_interrupt_cb = nullptr;

    // TDM interrupt: 1-bit alert_n shared across ranks, time-division multiplexed
    // Period = N_rank cycles (each rank gets 1 slot per period)
    // Latency = N_rank/2 cycles (average delivery delay)
    static constexpr int TDM_INTERRUPT_PERIOD  = 4;  // N_rank = 4
    static constexpr int TDM_INTERRUPT_LATENCY = 2;  // N_rank/2 = 2
    Clk_t m_last_interrupt_clk = -100;  // Initialize far past to allow first interrupt

    // NMA indexing time for RT reads (approximation; paper says not tRL)
    static constexpr int NMA_RT_LATENCY = 20;  // DRAM cycles

    // Pending TDM interrupt: deferred by TDM_INTERRUPT_LATENCY cycles
    struct PendingInterrupt {
      int batch_size;
      Clk_t deliver_clk;
    };
    std::deque<PendingInterrupt> m_pending_interrupts;

    // SR Unit per-bank recovery: when H/N Arbiter switches TO CMD FIFO,
    // a recovery PRE may be needed before CMD FIFO commands can proceed.
    struct SRRecovery {
      bool pending = false;
      int  cmd     = -1;
      AddrVec_t addr_vec;
    };
    std::vector<SRRecovery> m_sr_recovery;  // [total_banks]

    // Pending read completions (Return Unit NMA-side).
    // Populated on RD issue from CMD FIFO; fires return_buffer done-mark.
    struct PendingRead {
      uint64_t seq_num;
      Clk_t    complete_clk;
    };
    std::deque<PendingRead> m_pending_reads;

    // Row-hit low cap for REQ FIFO in concurrent mode (like DBX-DIMM m_ndp_row_hit_low_cap).
    // When CMD FIFO has entries, limit REQ FIFO same-row requests to this cap
    // so CMD FIFO gets more execution opportunities.
    static constexpr int NMA_ROW_HIT_LOW_CAP = 1;
    std::vector<int> m_req_row_hit_count;   // [total_banks] per-bank row-hit counter
    std::vector<int> m_req_row_hit_row;     // [total_banks] row being tracked (-1 = none)

    // ===== Fairness mechanisms for H/N Arbiter =====
    //
    // (A) CMD Service Quota: after REQ FIFO serves CMD_SERVICE_INTERVAL consecutive
    //     NMA ticks, force-switch to CMD FIFO for CMD_SERVICE_QUOTA ticks.
    //     Prevents CMD starvation when NDP workload has sustained bank activity.
    //
    // (C) REQ Continuous Service Cap: regardless of row-hit/miss pattern,
    //     if REQ FIFO has been served REQ_CONTINUOUS_CAP times consecutively
    //     while CMD FIFO has pending entries, treat REQ as "effectively empty"
    //     to force CMD switch. Fixes the gap in row-hit-low-cap which only
    //     triggers on row hits (GEMV has mostly row misses).
    int m_cmd_service_interval = 16;   // (A) REQ ticks before forced CMD switch
    int m_cmd_service_quota    = 4;    // (A) Minimum CMD ticks per forced switch
    int m_req_continuous_cap   = 8;    // (C) REQ consecutive service limit

    std::vector<int> m_req_continuous_count;  // [total_banks] consecutive REQ service counter
    std::vector<int> m_cmd_quota_remaining;   // [total_banks] forced CMD service remaining

    // Cached DRAM timing (nCL = read column latency, nCWL = write column latency, nBL = burst length)
    int m_nCL = -1;
    int m_nCWL = -1;
    int m_nBL = -1;

    // Phase 3 Statistics
    size_t s_num_cmd_received    = 0;  // Offload commands received into CMD FIFO
    size_t s_num_cmd_issued      = 0;  // DRAM commands issued from CMD FIFO
    size_t s_num_cmd_fifo_rd     = 0;  // RD issued from CMD FIFO (DB<->DRAM offload)
    size_t s_num_cmd_fifo_wr     = 0;  // WR issued from CMD FIFO (DB<->DRAM offload)
    size_t s_num_req_fifo_rd     = 0;  // RD issued from REQ FIFO (DB<->DRAM NMA)
    size_t s_num_req_fifo_wr     = 0;  // WR issued from REQ FIFO (DB<->DRAM NMA)
    size_t s_num_hn_switches     = 0;  // H/N Arbiter FIFO switches
    size_t s_num_sr_recoveries   = 0;  // SR Unit recovery commands issued
    size_t s_num_interrupts_sent = 0;  // Interrupts sent to Host MC
    size_t s_num_rt_received     = 0;  // RT commands received from Host MC
    size_t s_num_forced_cmd_switches  = 0;  // (A) Forced CMD switches via service interval
    size_t s_num_req_cap_switches     = 0;  // (C) CMD switches via REQ continuous cap
    // Per-bank per-command receive counters (from Host MC bypass_command)
    // Indexed by flat_bank_id = bg * num_banks + bk
    struct PerBankCmdCount {
      size_t acto = 0, preo = 0, rdo = 0, wro = 0;
      size_t issue_act = 0, issue_pre = 0, issue_rd = 0, issue_wr = 0;
    };
    std::vector<PerBankCmdCount> s_per_bank_cmd;  // [total_banks]

    // Issue tracking (per 100K cycle window, CMD FIFO = host offload, REQ FIFO = NMA)
    static constexpr int ISSUE_TRACK_WINDOW = 100000;
    int m_issue_track_cmd = 0;       // CMD FIFO issues in current window
    int m_issue_track_req = 0;       // REQ FIFO issues in current window
    long m_issue_track_start = 0;    // Start cycle of current window

  public:
    // Host MC debug state query callback (wired by system)
    struct HostDebugState {
      int rd_buf, wr_buf, pri_buf, active_buf;
      int rt_pending, ru_size, ot_sum;
      int pending_depart;
    };
    using HostDebugCallback = std::function<HostDebugState(int rank_id)>;
    HostDebugCallback m_host_debug_cb = nullptr;

    AsyncDIMMNMAController() = default;

    void init(int rank_id, IDRAM* dram,
              int nma_ctrl_row, int nma_ctrl_bg, int nma_ctrl_bk,
              int nma_buf_bg,  int nma_buf_bk) {
      m_rank_id = rank_id;
      m_dram = dram;

      m_channel_level = m_dram->m_levels("channel");
      m_rank_level = m_dram->m_levels("rank");
      m_bankgroup_level = m_dram->m_levels("bankgroup");
      m_bank_level = m_dram->m_levels("bank");
      m_row_level = m_dram->m_levels("row");
      m_column_level = m_dram->m_levels("column");

      m_num_bankgroups = m_dram->get_level_size("bankgroup");
      m_num_banks = m_dram->get_level_size("bank");
      m_total_banks = m_num_bankgroups * m_num_banks;

      // Cache timing values
      m_nRFC1 = m_dram->m_timing_vals("nRFC1");
      m_nRFCsb = m_dram->m_timing_vals("nRFCsb");
      m_nREFI = m_dram->m_timing_vals("nREFI");

      // Initialize all banks to CLOSED
      m_bank_fsm.resize(m_total_banks);
      for (auto& bank : m_bank_fsm) {
        bank.state = BankState::CLOSED;
        bank.open_row = -1;
      }

      // Initialize per-bank REQ FIFOs
      m_nma_req_buffer.resize(m_total_banks);

      // Initialize Phase 3: per-bank CMD FIFOs, H/N Arbiter, SR Unit, recovery
      m_cmd_fifo.resize(m_total_banks);
      m_arbiter_use_cmd.resize(m_total_banks, false);  // Default: REQ FIFO
      m_sr_unit.resize(m_total_banks);
      m_sr_recovery.resize(m_total_banks);
      m_req_row_hit_count.resize(m_total_banks, 0);
      m_req_row_hit_row.resize(m_total_banks, -1);
      m_req_continuous_count.resize(m_total_banks, 0);
      m_cmd_quota_remaining.resize(m_total_banks, 0);
      s_per_bank_cmd.resize(m_total_banks);

      // Debug: target bank for offload command group tracking (BG7 BK1, Rk0 only)
      if (m_rank_id == 0 && m_num_bankgroups > 7 && m_num_banks > 1) {
        m_debug_target_bg = 7;
        m_debug_target_bk = 1;
        m_debug_nma_cmd_groups.reserve(2048);
      }

      // Cache DRAM read timing for read completion modeling
      m_nCL = m_dram->m_timing_vals("nCL");
      m_nCWL = m_dram->m_timing_vals("nCWL");
      m_nBL = m_dram->m_timing_vals("nBL");

      // Store magic path addresses
      m_nma_ctrl_row = nma_ctrl_row;
      m_nma_ctrl_bg  = nma_ctrl_bg;
      m_nma_ctrl_bk  = nma_ctrl_bk;
      m_nma_buf_bg   = nma_buf_bg;
      m_nma_buf_bk   = nma_buf_bk;
    }

    void set_channel_id(int ch_id) { m_channel_id = ch_id; }

    /**
     * Configure H/N Arbiter fairness parameters (called by AsyncDIMMSystem from YAML).
     * @param interval  (A) REQ consecutive ticks before forced CMD switch (0=disabled)
     * @param quota     (A) Minimum CMD ticks per forced switch
     * @param cap       (C) REQ consecutive service cap before CMD switch (0=disabled)
     */
    void set_fairness_params(int interval, int quota, int cap) {
      m_cmd_service_interval = interval;
      m_cmd_service_quota    = quota;
      m_req_continuous_cap   = cap;
    }

    // ===== NMA Mode API =====

    NMAState get_nma_state() const { return m_nma_state; }
    bool is_nma_idle() const { return m_nma_state == NMAState::NMA_IDLE; }

    /**
     * Polling interface: system checks whether ctrl-reg WR arrived via magic path.
     */
    bool is_nma_start_pending() const { return m_nma_start_pending; }
    void consume_nma_start_pending()  { m_nma_start_pending = false; }

    /**
     * Compute total outstanding requests across all per-bank FIFOs.
     * Used for rank-level NMA done checking.
     */
    int get_nma_outstanding() const {
      int total = 0;
      for (const auto& fifo : m_nma_req_buffer)
        total += static_cast<int>(fifo.size());
      return total;
    }

    /**
     * Start NMA execution (transition from IDLE to ISSUE_START).
     * Called by AsyncDIMM System after detecting is_nma_start_pending().
     */
    void start_nma_execution() {
      if (m_nma_state != NMAState::NMA_IDLE) {
        std::cerr << "[NMA MC Rank " << m_rank_id
                  << "] ERROR: start_nma_execution() called in non-IDLE state!" << std::endl;
        return;
      }
      m_nma_pc = 0;
      for (int i = 0; i < 8; i++) m_base_reg[i] = 0;
      m_nma_state = NMAState::NMA_ISSUE_START;
      m_last_refresh_clk = m_clk;
      s_num_nma_executions++;
    }

    /**
     * Clear program buffer for re-loading on trace repeat.
     */
    void clear_program() {
      m_nma_program.clear();
      m_nma_start_pending = false;
    }

    /**
     * Check if NMA MC has finished all work (for mode transition back to HOST).
     * m_nma_outstanding == sum(m_nma_req_buffer[i].size()), so checking all
     * per-bank FIFOs empty is sufficient.
     */
    bool is_nma_complete() const {
      return m_nma_state == NMAState::NMA_IDLE &&
             m_nma_pc >= (int)m_nma_program.size() &&
             m_addr_gen_slots.empty() &&
             all_req_buffers_empty();
    }

    // ===== Main tick =====

    /**
     * Called every DRAM cycle.
     * - Future actions (refresh completion) checked every DRAM cycle.
     * - NMA state machine executes only every NMA_CLOCK_RATIO (4) DRAM cycles.
     */
    void tick() {
      m_clk++;

      // Process future actions every DRAM cycle (DRAM-timed)
      for (int i = (int)m_future_actions.size() - 1; i >= 0; i--) {
        if (m_future_actions[i].complete_clk == m_clk) {
          handle_future_action(m_future_actions[i]);
          m_future_actions.erase(m_future_actions.begin() + i);
        }
      }

      // Process NMA REQ FIFO data completions every DRAM cycle
      // (RD_L/WR_L data responses arrive at DRAM clock granularity)
      process_nma_data_completions();

      // Phase 3: check pending read completions, queue/deliver interrupts every DRAM cycle
      if (m_concurrent_mode) {
        process_pending_reads();
        check_and_send_interrupt();
        deliver_pending_interrupts();
      }

      // [DEBUG] Issue tracking: print CMD(host) vs REQ(NMA) issue counts per 100K cycle window
      // if (m_concurrent_mode && (m_clk - m_issue_track_start >= ISSUE_TRACK_WINDOW)) {
      //   // CMD/REQ FIFO total sizes across all banks
      //   int cmd_fifo_total = 0, req_fifo_total = 0;
      //   for (int b = 0; b < m_total_banks; b++) {
      //     cmd_fifo_total += (int)m_cmd_fifo[b].size();
      //     req_fifo_total += (int)m_nma_req_buffer[b].size();
      //   }
      //   int ret_buf = (int)m_return_buffer.size();
      //   int pend_rd = (int)m_pending_reads.size();
      //   int pend_int = (int)m_pending_interrupts.size();
      //
      //   printf("[NMA-ISSUE] ch=%d rk=%d cycle=%ld-%ld | CMD(host)=%d REQ(nma)=%d | "
      //          "CMD_Q=%d REQ_Q=%d ret_buf=%d pend_rd=%d pend_int=%d",
      //     m_channel_id, m_rank_id, m_issue_track_start, m_clk,
      //     m_issue_track_cmd, m_issue_track_req,
      //     cmd_fifo_total, req_fifo_total, ret_buf, pend_rd, pend_int);
      //
      //   // Host MC state for this rank
      //   if (m_host_debug_cb) {
      //     auto hs = m_host_debug_cb(m_rank_id);
      //     printf(" | H:rd=%d wr=%d pri=%d act=%d rt=%d ru=%d ot=%d pend=%d",
      //       hs.rd_buf, hs.wr_buf, hs.pri_buf, hs.active_buf,
      //       hs.rt_pending, hs.ru_size, hs.ot_sum, hs.pending_depart);
      //   }
      //   printf("\n");
      //
      //   m_issue_track_cmd = 0;
      //   m_issue_track_req = 0;
      //   m_issue_track_start = m_clk;
      // }

      // NMA state machine runs at 1/4 DRAM clock
      if (m_clk % NMA_CLOCK_RATIO == 0) {
        tick_nma_state_machine();
      }
    }

    // ===== Bypass command from Host MC (Phase 1: Explicit Sync H2N + Magic Path) =====

    /**
     * Called for every DRAM command Host MC issues.
     *
     * Two roles:
     *   1. Shadow FSM update (Phase 1 explicit sync H2N)
     *   2. Magic path interception (Phase 2):
     *      - WR to inst-buffer address  → decode payload as NMAInst_Slot
     *      - WR to ctrl-reg address     → set m_nma_start_pending for this rank
     *
     * payload is non-empty only for WR/WRA commands; empty for all others.
     */
    void bypass_command(int command, const AddrVec_t& addr_vec,
                        const std::vector<uint64_t>& payload) {

      // ===== Phase 3: CMD FIFO population (Concurrent Mode) =====
      // Intercept offload commands (ACTO/RDO/WRO/PREO/REFO) from Host MC.
      // Decode → real DRAM command and push into per-bank CMD FIFO.
      // Do NOT update shadow FSM for offload commands (NMA MC manages own state).
      if (m_concurrent_mode) {
        // REFO: Host MC's RefreshManager says refresh is due.
        // Set pending flag — NMA MC will issue real REFab when all banks are ready.
        if (command == m_dram->m_commands("REFO")) {
          m_nma_refresh_pending = true;
          s_num_ref_bypass++;
          return;
        }

        int decoded_cmd = -1;
        // Decode offload → NMA-local commands (rank-local CA + data bus)
        if      (command == m_dram->m_commands("ACTO")) decoded_cmd = m_dram->m_commands("ACT_L");
        else if (command == m_dram->m_commands("PREO")) decoded_cmd = m_dram->m_commands("PRE_L");
        else if (command == m_dram->m_commands("RDO"))  decoded_cmd = m_dram->m_commands("RD_L");
        else if (command == m_dram->m_commands("WRO"))  decoded_cmd = m_dram->m_commands("WR_L");

        if (decoded_cmd >= 0) {
          int flat_id = get_flat_bank_id(addr_vec);
          if ((int)m_cmd_fifo[flat_id].size() < m_eff_cmd_fifo_size) {
            CmdFifoEntry entry;
            entry.command  = decoded_cmd;
            entry.addr_vec = addr_vec;
            entry.arrive   = m_clk;
            entry.is_read  = (decoded_cmd == m_dram->m_commands("RD_L"));
            // Only RD_L/WR_L need return_buffer entries for interrupt-based completion.
            // ACT_L/PRE_L are prerequisites — Host MC RU only tracks RDO/WRO.
            if (decoded_cmd == m_dram->m_commands("RD_L") ||
                decoded_cmd == m_dram->m_commands("WR_L")) {
              entry.seq_num = m_next_seq_num;
              m_return_buffer.push_back({m_next_seq_num, false,
                addr_vec[m_bankgroup_level], addr_vec[m_bank_level], entry.is_read});
              m_next_seq_num++;
            } else {
              entry.seq_num = -1;  // No return tracking for ACT_L/PRE_L
            }
            m_cmd_fifo[flat_id].push_back(entry);
            s_num_cmd_received++;
            // Per-bank per-command receive counters
            if      (command == m_dram->m_commands("ACTO")) s_per_bank_cmd[flat_id].acto++;
            else if (command == m_dram->m_commands("PREO")) s_per_bank_cmd[flat_id].preo++;
            else if (command == m_dram->m_commands("RDO"))  s_per_bank_cmd[flat_id].rdo++;
            else if (command == m_dram->m_commands("WRO"))  s_per_bank_cmd[flat_id].wro++;
            // Debug: target bank command group tracking
            if (addr_vec[m_bankgroup_level] == m_debug_target_bg &&
                addr_vec[m_bank_level] == m_debug_target_bk) {
              int row = addr_vec[m_row_level];
              bool is_pre = (command == m_dram->m_commands("PREO"));
              bool is_act = (command == m_dram->m_commands("ACTO"));
              bool is_rd  = (command == m_dram->m_commands("RDO"));
              bool is_wr  = (command == m_dram->m_commands("WRO"));
              if (is_pre) {
                // PRE starts a new group (if current group has no RD/WR, flush as incomplete)
                if (m_debug_nma_cur_group.cmd_count > 0 && !m_debug_nma_cur_group.has_rdwr) {
                  m_debug_nma_cur_group.last_clk = m_clk;
                  std::string seq;
                  if (m_debug_nma_cur_group.has_pre) seq += "PRE";
                  if (m_debug_nma_cur_group.has_act) { if (!seq.empty()) seq += "→"; seq += "ACT"; }
                  seq += "(INCOMPLETE)";
                  m_debug_nma_cmd_groups.push_back({m_debug_nma_cur_group.first_clk, (long)m_clk,
                    m_debug_nma_cur_group.row, m_debug_nma_cur_group.cmd_count,
                    false, seq});
                }
                m_debug_nma_cur_group = {};
                m_debug_nma_cur_group.first_clk = m_clk;
                m_debug_nma_cur_group.row = row;
                m_debug_nma_cur_group.has_pre = true;
                m_debug_nma_cur_group.cmd_count = 1;
              } else if (is_act) {
                if (m_debug_nma_cur_group.cmd_count == 0) {
                  m_debug_nma_cur_group.first_clk = m_clk;
                  m_debug_nma_cur_group.row = row;
                }
                m_debug_nma_cur_group.has_act = true;
                m_debug_nma_cur_group.cmd_count++;
              } else if (is_rd || is_wr) {
                if (m_debug_nma_cur_group.cmd_count == 0) {
                  m_debug_nma_cur_group.first_clk = m_clk;
                  m_debug_nma_cur_group.row = row;
                }
                m_debug_nma_cur_group.has_rdwr = true;
                m_debug_nma_cur_group.is_read = is_rd;
                m_debug_nma_cur_group.cmd_count++;
                m_debug_nma_cur_group.last_clk = m_clk;
                // Group complete: PRE?→ACT?→RD/WR
                std::string seq;
                if (m_debug_nma_cur_group.has_pre) seq += "PRE→";
                if (m_debug_nma_cur_group.has_act) seq += "ACT→";
                seq += is_rd ? "RD" : "WR";
                m_debug_nma_cmd_groups.push_back({m_debug_nma_cur_group.first_clk, (long)m_clk,
                  m_debug_nma_cur_group.row, m_debug_nma_cur_group.cmd_count,
                  m_debug_nma_cur_group.is_read, seq});
                m_debug_nma_cur_group = {};  // reset for next group
              }
            }
          } else {
            // [DEBUG] CMD_DROP logging
            // const char* cn = "?";
            // if (decoded_cmd == m_dram->m_commands("ACT_L")) cn = "ACT_L";
            // else if (decoded_cmd == m_dram->m_commands("PRE_L")) cn = "PRE_L";
            // else if (decoded_cmd == m_dram->m_commands("RD_L")) cn = "RD_L";
            // else if (decoded_cmd == m_dram->m_commands("WR_L")) cn = "WR_L";
            // printf("[CMD_DROP] ch=%d rk=%d cmd=%s bg=%d bk=%d row=%d col=%d "
            //        "cmd_q=%d/%d cycle=%ld\n",
            //   m_channel_id, m_rank_id, cn,
            //   addr_vec[m_bankgroup_level], addr_vec[m_bank_level],
            //   addr_vec[m_row_level], addr_vec[m_column_level],
            //   (int)m_cmd_fifo[flat_id].size(), m_eff_cmd_fifo_size, m_clk);
          }
          // Offload commands: no shadow FSM update, return immediately
          return;
        }
      }

      // REFO in NMA mode: Host MC's RefreshManager requests refresh
      if (command == m_dram->m_commands("REFO")) {
        m_nma_refresh_pending = true;
        s_num_ref_bypass++;
        return;
      }

      s_num_bypass_received++;

      if (command == m_dram->m_commands("ACT")) {
        int flat_id = get_flat_bank_id(addr_vec);
        m_bank_fsm[flat_id].state = BankState::OPENED;
        m_bank_fsm[flat_id].open_row = addr_vec[m_row_level];
        s_num_act_bypass++;
      }
      else if (command == m_dram->m_commands("PRE")) {
        int flat_id = get_flat_bank_id(addr_vec);
        m_bank_fsm[flat_id].state = BankState::CLOSED;
        m_bank_fsm[flat_id].open_row = -1;
        s_num_pre_bypass++;
      }
      else if (command == m_dram->m_commands("PREsb")) {
        int target_bank = addr_vec[m_bank_level];
        for (int bg = 0; bg < m_num_bankgroups; bg++) {
          int flat_id = bg * m_num_banks + target_bank;
          m_bank_fsm[flat_id].state = BankState::CLOSED;
          m_bank_fsm[flat_id].open_row = -1;
        }
        s_num_pre_bypass++;
      }
      else if (command == m_dram->m_commands("PREA")) {
        for (auto& bank : m_bank_fsm) {
          bank.state = BankState::CLOSED;
          bank.open_row = -1;
        }
        s_num_pre_bypass++;
      }
      else if (command == m_dram->m_commands("RD")) {
        s_num_rd_bypass++;
      }
      else if (command == m_dram->m_commands("RDA")) {
        int flat_id = get_flat_bank_id(addr_vec);
        m_bank_fsm[flat_id].state = BankState::CLOSED;
        m_bank_fsm[flat_id].open_row = -1;
        s_num_rd_bypass++;
      }
      else if (command == m_dram->m_commands("WR") ||
               command == m_dram->m_commands("WRA")) {
        // ===== Magic Path Interception =====
        // If this WR targets the NMA control region, intercept the payload.
        // The NMA MC reads the DQ-bus data (payload) instead of forwarding to DRAM cells.
        if (!payload.empty() && addr_vec[m_row_level] == m_nma_ctrl_row) {
          int bg = addr_vec[m_bankgroup_level];
          int bk = addr_vec[m_bank_level];

          if (bg == m_nma_buf_bg && bk == m_nma_buf_bk) {
            // NMAInst buffer write → decode each payload word into program buffer
            for (const auto& pval : payload) {
              if (pval == 0) continue;
              NMAInst_Slot inst = decode_nma_inst(pval);
              if ((int)m_nma_program.size() < NMA_INST_MAX) {
                m_nma_program.push_back(inst);
              }
            }
            s_num_magic_inst_wr++;
          }
          else if (bg == m_nma_ctrl_bg && bk == m_nma_ctrl_bk) {
            // Ctrl-reg write: NMA start is now managed by Host MC (DBX-DIMM style).
            // Host MC tracks NMAInst write count and triggers start_nma_execution()
            // only after all instructions are delivered. No action needed here.
            s_num_magic_ctrl_wr++;
          }
        }

        if (command == m_dram->m_commands("WRA")) {
          int flat_id = get_flat_bank_id(addr_vec);
          m_bank_fsm[flat_id].state = BankState::CLOSED;
          m_bank_fsm[flat_id].open_row = -1;
        }
        s_num_wr_bypass++;
      }
      else if (command == m_dram->m_commands("REFab")) {
        for (auto& bank : m_bank_fsm) {
          bank.state = BankState::REFRESHING;
          bank.open_row = -1;
        }
        m_future_actions.push_back({
          FutureAction::Type::REFAB_END,
          addr_vec,
          m_clk + m_nRFC1 - 1
        });
        s_num_ref_bypass++;
      }
      else if (command == m_dram->m_commands("REFsb")) {
        int target_bank = addr_vec[m_bank_level];
        for (int bg = 0; bg < m_num_bankgroups; bg++) {
          int flat_id = bg * m_num_banks + target_bank;
          m_bank_fsm[flat_id].state = BankState::REFRESHING;
          m_bank_fsm[flat_id].open_row = -1;
        }
        m_future_actions.push_back({
          FutureAction::Type::REFSB_END,
          addr_vec,
          m_clk + m_nRFCsb - 1
        });
        s_num_ref_bypass++;
      }
    }

    // ===== Query API =====

    BankState get_bank_state(int bankgroup, int bank) const {
      return m_bank_fsm[bankgroup * m_num_banks + bank].state;
    }

    int get_open_row(int bankgroup, int bank) const {
      return m_bank_fsm[bankgroup * m_num_banks + bank].open_row;
    }

    int get_rank_id() const { return m_rank_id; }

    bool all_banks_closed() const {
      for (const auto& bank : m_bank_fsm) {
        if (bank.state != BankState::CLOSED) return false;
      }
      return true;
    }

    bool any_bank_refreshing() const {
      for (const auto& bank : m_bank_fsm) {
        if (bank.state == BankState::REFRESHING) return true;
      }
      return false;
    }

    bool all_req_buffers_empty() const {
      for (const auto& fifo : m_nma_req_buffer) {
        if (!fifo.empty()) return false;
      }
      return true;
    }

    // Debug: return comprehensive state string for periodic logging
    std::string get_debug_state_str() const {
      static const char* state_names[] = {"IDLE", "ISSUE_START", "RUN", "BAR", "WAIT", "DONE"};
      int state_idx = static_cast<int>(m_nma_state);
      const char* state_name = (state_idx >= 0 && state_idx <= 5) ? state_names[state_idx] : "???";

      // Per-bank FIFO sizes
      int req_total = 0, cmd_total = 0;
      int req_max = 0, cmd_max = 0;
      int req_max_bank = -1, cmd_max_bank = -1;
      int cmd_using_count = 0;
      int open_count = 0, closed_count = 0, refresh_count = 0;
      for (int b = 0; b < m_total_banks; b++) {
        int rs = static_cast<int>(m_nma_req_buffer[b].size());
        int cs = m_concurrent_mode ? static_cast<int>(m_cmd_fifo[b].size()) : 0;
        req_total += rs;
        cmd_total += cs;
        if (rs > req_max) { req_max = rs; req_max_bank = b; }
        if (cs > cmd_max) { cmd_max = cs; cmd_max_bank = b; }
        if (m_concurrent_mode && m_arbiter_use_cmd[b]) cmd_using_count++;
        switch (m_bank_fsm[b].state) {
          case BankState::OPENED:     open_count++; break;
          case BankState::CLOSED:     closed_count++; break;
          case BankState::REFRESHING: refresh_count++; break;
        }
      }

      char buf[512];
      snprintf(buf, sizeof(buf),
        "state=%s prog=%zu pc=%d remain=%d addr_gen=%zu | "
        "REQ_FIFO=%d(max=%d@bk%d) CMD_FIFO=%d(max=%d@bk%d) | "
        "ret_buf=%zu pend_rd=%zu pend_int=%zu | "
        "arb_cmd=%d banks[O=%d,C=%d,R=%d]",
        state_name,
        m_nma_program.size(),
        m_nma_pc,
        (int)m_nma_program.size() - m_nma_pc,
        m_addr_gen_slots.size(),
        req_total, req_max, req_max_bank,
        cmd_total, cmd_max, cmd_max_bank,
        m_return_buffer.size(),
        m_pending_reads.size(),
        m_pending_interrupts.size(),
        cmd_using_count,
        open_count, closed_count, refresh_count);
      return std::string(buf);
    }

    void print_stats() const {
      std::cout << "[NMA MC Rank " << m_rank_id << "] Stats:" << std::endl;
      std::cout << "  === Bypass (Host Mode) ===" << std::endl;
      std::cout << "  Total bypass commands: " << s_num_bypass_received << std::endl;
      std::cout << "  ACT:  " << s_num_act_bypass << std::endl;
      std::cout << "  PRE:  " << s_num_pre_bypass << std::endl;
      std::cout << "  RD:   " << s_num_rd_bypass << std::endl;
      std::cout << "  WR:   " << s_num_wr_bypass << std::endl;
      std::cout << "  REF:  " << s_num_ref_bypass << std::endl;
      std::cout << "  Magic path NMAInst WR: " << s_num_magic_inst_wr << std::endl;
      std::cout << "  Magic path ctrl-reg WR:" << s_num_magic_ctrl_wr << std::endl;
      std::cout << "  === NMA Execution ===" << std::endl;
      std::cout << "  NMA executions:   " << s_num_nma_executions << std::endl;
      std::cout << "  NMA instructions: " << s_num_nma_instructions << std::endl;
      std::cout << "  NMA compute-only: " << s_num_nma_compute_only << std::endl;
      std::cout << "  NMA loops:        " << s_num_nma_loops << std::endl;
      std::cout << "  NMA ACT:  " << s_num_nma_act << std::endl;
      std::cout << "  NMA PRE:  " << s_num_nma_pre << std::endl;
      std::cout << "  NMA RD:   " << s_num_nma_rd << std::endl;
      std::cout << "  NMA WR:   " << s_num_nma_wr << std::endl;
      std::cout << "  NMA REF:  " << s_num_nma_ref << std::endl;
      std::cout << "  === NMA State Cycles ===" << std::endl;
      std::cout << "  IDLE:        " << s_nma_idle_cycles << std::endl;
      std::cout << "  ISSUE_START: " << s_nma_issue_start_cycles << std::endl;
      std::cout << "  RUN:         " << s_nma_run_cycles << std::endl;
      std::cout << "  BAR:         " << s_nma_bar_cycles << std::endl;
      std::cout << "  WAIT:        " << s_nma_wait_cycles << std::endl;
      std::cout << "  DONE:        " << s_nma_done_cycles << std::endl;
      std::cout << "  Per-bank REQ FIFO outstanding: " << get_nma_outstanding() << std::endl;
      if (m_concurrent_mode || s_num_cmd_received > 0) {
        std::cout << "  === Phase 3 Concurrent Mode ===" << std::endl;
        std::cout << "  CMD received:      " << s_num_cmd_received    << std::endl;
        std::cout << "  CMD issued:        " << s_num_cmd_issued      << std::endl;
        std::cout << "  H/N switches:      " << s_num_hn_switches     << std::endl;
        std::cout << "  SR recoveries:     " << s_num_sr_recoveries   << std::endl;
        std::cout << "  Interrupts sent:   " << s_num_interrupts_sent << std::endl;
        std::cout << "  RT received:       " << s_num_rt_received     << std::endl;
        std::cout << "  Forced CMD switches: " << s_num_forced_cmd_switches << std::endl;
        std::cout << "  REQ cap switches:    " << s_num_req_cap_switches    << std::endl;
        std::cout << "  CMD FIFO pending:  " << get_cmd_fifo_outstanding() << std::endl;
      }

      // Flush final issue tracking window
      {
        // [DEBUG] NMA-ISSUE FINAL print block
        // int cmd_fifo_total = 0, req_fifo_total = 0;
        // for (int b = 0; b < m_total_banks; b++) {
        //   cmd_fifo_total += (int)m_cmd_fifo[b].size();
        //   req_fifo_total += (int)m_nma_req_buffer[b].size();
        // }
        // int ret_buf = (int)m_return_buffer.size();
        // int pend_rd = (int)m_pending_reads.size();
        // int pend_int = (int)m_pending_interrupts.size();
        //
        // printf("[NMA-ISSUE] ch=%d rk=%d cycle=%ld-%ld (FINAL) | CMD(host)=%d REQ(nma)=%d | "
        //        "CMD_Q=%d REQ_Q=%d ret_buf=%d pend_rd=%d pend_int=%d",
        //   m_channel_id, m_rank_id, m_issue_track_start, m_clk,
        //   m_issue_track_cmd, m_issue_track_req,
        //   cmd_fifo_total, req_fifo_total, ret_buf, pend_rd, pend_int);
        //
        // if (m_host_debug_cb) {
        //   auto hs = m_host_debug_cb(m_rank_id);
        //   printf(" | H:rd=%d wr=%d pri=%d act=%d rt=%d ru=%d ot=%d pend=%d",
        //     hs.rd_buf, hs.wr_buf, hs.pri_buf, hs.active_buf,
        //     hs.rt_pending, hs.ru_size, hs.ot_sum, hs.pending_depart);
        // }
        // printf("\n");
      }
    }

    void print_bank_states() const {
      std::cout << "[NMA MC Rank " << m_rank_id << "] Bank States:" << std::endl;
      for (int bg = 0; bg < m_num_bankgroups; bg++) {
        for (int bk = 0; bk < m_num_banks; bk++) {
          int flat_id = bg * m_num_banks + bk;
          const char* state_str = "???";
          switch (m_bank_fsm[flat_id].state) {
            case BankState::CLOSED:     state_str = "CLOSED"; break;
            case BankState::OPENED:     state_str = "OPENED"; break;
            case BankState::REFRESHING: state_str = "REFRESHING"; break;
          }
          std::cout << "  BG" << bg << " BK" << bk << ": " << state_str;
          if (m_bank_fsm[flat_id].state == BankState::OPENED)
            std::cout << " (row=" << m_bank_fsm[flat_id].open_row << ")";
          std::cout << "  REQ_FIFO=" << m_nma_req_buffer[flat_id].size()
                    << "  CMD_FIFO=" << m_cmd_fifo[flat_id].size() << std::endl;
        }
      }
    }

    // ===== Phase 3: Concurrent Mode Public API =====

    /**
     * Enter concurrent mode: Host MC offloads commands via CMD FIFO.
     * Called by AsyncDIMMSystem during H2C transition.
     */
    void enter_concurrent_mode() {
      m_concurrent_mode = true;
      // Concurrent mode FIFO rebalancing (paper §V.B.2):
      // REQ FIFO gives half its capacity (4) to CMD FIFO → CMD=12, REQ=4
      m_eff_cmd_fifo_size = NMA_CMD_FIFO_SIZE + NMA_REQ_BUFFER_PER_BANK / 2 + 1;  // 80 + 4 + 1(SR) = 85
      m_eff_req_fifo_size = NMA_REQ_BUFFER_PER_BANK / 2;                       // 8 / 2 = 4
      // Arbiter starts at REQ FIFO (NMA workload may already be running)
      for (size_t i = 0; i < m_arbiter_use_cmd.size(); i++) m_arbiter_use_cmd[i] = false;
      // Clear stale CMD FIFO state and SR recovery
      for (auto& fifo : m_cmd_fifo) fifo.clear();
      for (auto& sr   : m_sr_recovery) sr = SRRecovery{};
      m_return_buffer.clear();
      m_pending_reads.clear();
      m_pending_interrupts.clear();

      m_next_seq_num = 0;
      // Reset row-hit tracking and fairness counters
      std::fill(m_req_row_hit_count.begin(), m_req_row_hit_count.end(), 0);
      std::fill(m_req_row_hit_row.begin(), m_req_row_hit_row.end(), -1);
      std::fill(m_req_continuous_count.begin(), m_req_continuous_count.end(), 0);
      std::fill(m_cmd_quota_remaining.begin(), m_cmd_quota_remaining.end(), 0);
    }

    /**
     * Exit concurrent mode: restore pure NMA scheduling.
     * Called by AsyncDIMMSystem during C2H transition.
     */
    void exit_concurrent_mode() {
      m_concurrent_mode = false;
      // Restore full capacity for REQ FIFO in NMA-only mode
      m_eff_cmd_fifo_size = NMA_CMD_FIFO_SIZE;
      m_eff_req_fifo_size = NMA_REQ_BUFFER_PER_BANK;
      // Clear any remaining concurrent state
      for (auto& fifo : m_cmd_fifo) fifo.clear();
      m_return_buffer.clear();
      m_pending_reads.clear();
      m_pending_interrupts.clear();

      m_nma_refresh_pending = false;
      for (auto& sr : m_sr_recovery) sr.pending = false;
      // Reset row-hit tracking and fairness counters
      std::fill(m_req_row_hit_count.begin(), m_req_row_hit_count.end(), 0);
      std::fill(m_req_row_hit_row.begin(), m_req_row_hit_row.end(), -1);
      std::fill(m_req_continuous_count.begin(), m_req_continuous_count.end(), 0);
      std::fill(m_cmd_quota_remaining.begin(), m_cmd_quota_remaining.end(), 0);
    }

    void flush_pending_interrupts() {
      // Deliver all queued interrupts immediately (C2H cleanup)
      while (!m_pending_interrupts.empty()) {
        auto& pi = m_pending_interrupts.front();
        if (m_interrupt_cb)
          m_interrupt_cb(m_rank_id, pi.batch_size);
        m_pending_interrupts.pop_front();
      }
      // Mark all remaining return_buffer entries as done and send interrupts
      while (!m_return_buffer.empty()) {
        if (m_interrupt_cb)
          m_interrupt_cb(m_rank_id, 1);
        m_return_buffer.pop_front();
      }
    }

    /**
     * Register interrupt callback (wired by AsyncDIMMSystem to Host MC).
     * Fired when head-of-line offloaded read is complete.
     */
    void set_interrupt_callback(InterruptCallback cb) {
      m_interrupt_cb = cb;
    }

    void set_host_debug_callback(HostDebugCallback cb) {
      m_host_debug_cb = cb;
    }

    /**
     * Host MC issued RT command: NMA MC "sends" data batch via DQ.
     * In simulation, we just track the stat; real DQ timing handled by DRAM model.
     */
    void receive_rt(int batch_size) {
      s_num_rt_received++;
      (void)batch_size;
    }

    const std::vector<NMACmdGroupLog>& get_debug_nma_cmd_groups() const { return m_debug_nma_cmd_groups; }
    const NMACmdGroup& get_debug_nma_cur_group() const { return m_debug_nma_cur_group; }

    /**
     * True when NMA state is IDLE and all CMD FIFOs are drained.
     * Used by System to detect end of concurrent execution.
     */
    bool is_concurrent_complete() const {
      if (!m_concurrent_mode) return false;
      return is_nma_idle()
          && all_cmd_fifos_empty()
          && m_return_buffer.empty()
          && m_pending_reads.empty()
          && m_pending_interrupts.empty();
    }

    bool all_cmd_fifos_empty() const {
      for (const auto& fifo : m_cmd_fifo)
        if (!fifo.empty()) return false;
      return true;
    }

    /**
     * Number of entries pending in all CMD FIFOs.
     */
    int get_cmd_fifo_outstanding() const {
      int total = 0;
      for (const auto& fifo : m_cmd_fifo)
        total += (int)fifo.size();
      return total;
    }
    int get_return_buffer_size() const { return (int)m_return_buffer.size(); }
    int get_pending_reads_size() const { return (int)m_pending_reads.size(); }
    int get_pending_interrupts_size() const { return (int)m_pending_interrupts.size(); }

    int get_total_banks() const { return m_total_banks; }
    int get_num_bankgroups() const { return m_num_bankgroups; }
    int get_num_banks() const { return m_num_banks; }

    // Per-bank debug state for external logging
    struct NMABankDebugState {
      int cmd_fifo_sz;
      int req_fifo_sz;
      int bank_state;    // 0=CLOSED, 1=OPENED, 2=REFRESHING
      int open_row;
      bool sr_pending;
      bool arbiter_use_cmd;
    };
    NMABankDebugState get_bank_debug_state(int flat_bank_id) const {
      NMABankDebugState s{};
      if (flat_bank_id < 0 || flat_bank_id >= m_total_banks) return s;
      s.cmd_fifo_sz = (int)m_cmd_fifo[flat_bank_id].size();
      s.req_fifo_sz = (int)m_nma_req_buffer[flat_bank_id].size();
      s.bank_state  = (int)m_bank_fsm[flat_bank_id].state;
      s.open_row    = m_bank_fsm[flat_bank_id].open_row;
      s.sr_pending  = m_sr_recovery[flat_bank_id].pending;
      s.arbiter_use_cmd = m_arbiter_use_cmd[flat_bank_id];
      return s;
    }
    bool is_refresh_pending() const { return m_nma_refresh_pending; }
    const PerBankCmdCount& get_per_bank_cmd(int flat_bank_id) const {
      return s_per_bank_cmd[flat_bank_id];
    }
    int get_per_bank_cmd_fifo_size(int flat_bank_id) const {
      return (int)m_cmd_fifo[flat_bank_id].size();
    }
    size_t get_num_interrupts_sent() const { return s_num_interrupts_sent; }
    const std::vector<InterruptLogEntry>& get_interrupt_log() const { return s_interrupt_log; }
    const std::vector<CmdFifoIssueLogEntry>& get_cmd_fifo_issue_log() const { return s_cmd_fifo_issue_log; }

    // DB<->DRAM bandwidth counters
    size_t get_cmd_fifo_rd() const { return s_num_cmd_fifo_rd; }
    size_t get_cmd_fifo_wr() const { return s_num_cmd_fifo_wr; }
    size_t get_req_fifo_rd() const { return s_num_req_fifo_rd; }
    size_t get_req_fifo_wr() const { return s_num_req_fifo_wr; }

  private:
    int get_flat_bank_id(const AddrVec_t& addr_vec) const {
      return addr_vec[m_bankgroup_level] * m_num_banks + addr_vec[m_bank_level];
    }

    // ===== Phase 3: Concurrent Mode Private Functions =====

    /**
     * Try to issue one DRAM command using H/N Arbiter (CMD FIFO or REQ FIFO).
     * Called once per NMA tick in Concurrent Mode, replacing try_issue_nma_command().
     *
     * H/N Arbiter switch conditions (per bank):
     *   CMD → REQ: CMD FIFO empty OR PRE was just issued from CMD FIFO
     *   REQ → CMD: REQ FIFO empty OR PRE was just issued from REQ FIFO
     */
    void try_issue_concurrent_command() {
      // Refresh has highest priority — block ALL other commands until refresh completes
      if (m_nma_refresh_pending) {
        try_issue_nma_refresh();
        return;  // No CMD/REQ FIFO commands while refresh pending
      }

      for (int i = 0; i < m_total_banks; i++) {
        int bank_id = (m_req_rr_bank_idx + i) % m_total_banks;
        bool use_cmd = m_arbiter_use_cmd[bank_id];

        auto& cmd_fifo = m_cmd_fifo[bank_id];
        auto& req_fifo = m_nma_req_buffer[bank_id];
        bool cmd_empty = cmd_fifo.empty() && !m_sr_recovery[bank_id].pending;
        bool req_empty = req_fifo.empty();

        // ---- (A) CMD Service Quota: forced CMD service in progress ----
        // If quota remaining > 0, stay on CMD FIFO until quota exhausted or CMD empty.
        if (m_cmd_quota_remaining[bank_id] > 0) {
          if (!cmd_empty) {
            // Continue forced CMD service
            bool issued = try_issue_from_cmd_fifo(bank_id);
            if (issued) {
              // [DEBUG] m_issue_track_cmd++;
              m_cmd_quota_remaining[bank_id]--;
              return;
            }
            // CMD not ready this tick — try other banks (don't block)
            continue;
          }
          // CMD became empty during quota — end forced service, fall through
          m_cmd_quota_remaining[bank_id] = 0;
          if (!req_empty) {
            m_arbiter_use_cmd[bank_id] = false;
            use_cmd = false;
            s_num_hn_switches++;
          } else {
            continue;  // Both empty
          }
        }

        // ---- Determine if REQ FIFO should be treated as "effectively empty" ----
        bool req_effectively_empty = false;

        if (!req_empty && !cmd_empty) {
          // (Original) Row-hit low cap: consecutive same-row hits exceed threshold
          int req_front_row = req_fifo.front().addr_vec[m_row_level];
          if (m_req_row_hit_row[bank_id] == req_front_row &&
              m_req_row_hit_count[bank_id] >= NMA_ROW_HIT_LOW_CAP) {
            req_effectively_empty = true;
          }

          // (C) REQ Continuous Service Cap: regardless of row-hit/miss pattern,
          //     if REQ has been served too many times consecutively, force CMD switch.
          //     Disabled when m_req_continuous_cap <= 0.
          if (m_req_continuous_cap > 0 &&
              m_req_continuous_count[bank_id] >= m_req_continuous_cap) {
            req_effectively_empty = true;
            s_num_req_cap_switches++;
          }
        }

        // ---- Arbiter switch logic ----
        if (use_cmd && cmd_empty) {
          // CMD empty → switch to REQ if available
          if (!req_empty && !req_effectively_empty) {
            m_arbiter_use_cmd[bank_id] = false;
            use_cmd = false;
            s_num_hn_switches++;
            m_req_continuous_count[bank_id] = 0;
          } else {
            continue;  // Both sides empty/capped
          }
        } else if (!use_cmd && (req_empty || req_effectively_empty)) {
          if (!cmd_empty) {
            // (A) Check if forced CMD service interval reached (disabled when interval <= 0)
            bool force_quota = (m_cmd_service_interval > 0 &&
                                m_req_continuous_count[bank_id] >= m_cmd_service_interval);
            switch_to_cmd_fifo(bank_id);
            use_cmd = true;
            m_req_continuous_count[bank_id] = 0;
            if (force_quota) {
              m_cmd_quota_remaining[bank_id] = m_cmd_service_quota;
              s_num_forced_cmd_switches++;
            }
          } else if (req_empty) {
            continue;  // Both sides truly empty
          } else {
            // req_effectively_empty but cmd_empty → fall through to try REQ anyway
            req_effectively_empty = false;
          }
        }

        // ---- Issue from selected FIFO ----
        bool issued;
        if (use_cmd) {
          issued = try_issue_from_cmd_fifo(bank_id);
          if (issued) {
            // [DEBUG] m_issue_track_cmd++;
            m_req_continuous_count[bank_id] = 0;  // Reset REQ counter on CMD service
            if (m_cmd_quota_remaining[bank_id] > 0)
              m_cmd_quota_remaining[bank_id]--;
          }
        } else {
          issued = try_issue_from_req_fifo(bank_id);
          if (issued) {
            // [DEBUG] m_issue_track_req++;
            m_req_continuous_count[bank_id]++;
          }
        }
        if (issued) return;  // One command per NMA tick
      }
    }

    /**
     * Switch H/N Arbiter to CMD FIFO with SR Unit recovery check.
     *
     * Paper Fig 5 recovery table (actual bank state × CMD FIFO's expected state):
     *   Actual=OPEN,   Expected=OPEN  (different row) → PRE + ACT  (close, reopen correct row)
     *   Actual=OPEN,   Expected=CLOSED                → PRE        (close bank)
     *   Actual=CLOSED, Expected=OPEN                  → (none)     (CMD FIFO starts with ACT)
     *   Actual=CLOSED, Expected=CLOSED                → (none)     (already matches)
     *
     * CMD FIFO's "expected state" is inferred from its front command:
     *   front=ACT → expects bank CLOSED (needs to activate)
     *   front=RD/WR → expects bank OPEN with the correct row
     *   front=PRE → expects bank OPEN (needs to close)
     */
    void switch_to_cmd_fifo(int bank_id) {
      m_arbiter_use_cmd[bank_id] = true;
      s_num_hn_switches++;

      auto& fifo = m_cmd_fifo[bank_id];
      if (fifo.empty()) return;

      const auto& bank = m_bank_fsm[bank_id];
      int front_cmd = fifo.front().command;
      int act_cmd   = m_dram->m_commands("ACT_L");
      int pre_cmd   = m_dram->m_commands("PRE_L");
      int rd_cmd    = m_dram->m_commands("RD_L");
      int wr_cmd    = m_dram->m_commands("WR_L");

      // Build recovery address vector for this bank
      auto make_bank_addr = [&]() -> AddrVec_t {
        AddrVec_t a(m_dram->m_levels.size(), -1);
        a[m_channel_level]   = m_channel_id;
        a[m_rank_level]      = m_rank_id;
        a[m_bankgroup_level] = bank_id / m_num_banks;
        a[m_bank_level]      = bank_id % m_num_banks;
        return a;
      };

      if (bank.state == BankState::OPENED) {
        if (front_cmd == act_cmd) {
          // Actual=OPEN, Expected=CLOSED → PRE recovery
          // CMD FIFO will issue its own ACT next tick after PRE completes
          m_sr_recovery[bank_id] = {true, pre_cmd, make_bank_addr()};
          s_num_sr_recoveries++;
        }
        else if (front_cmd == rd_cmd || front_cmd == wr_cmd) {
          // Actual=OPEN, CMD expects row-hit RD/WR.
          // If the open row differs from CMD FIFO's target row → row conflict → PRE+ACT
          int cmd_target_row = fifo.front().addr_vec[m_row_level];
          if (bank.open_row != cmd_target_row) {
            // PRE first; after PRE completes, we'll need ACT before the RD/WR.
            // Insert ACT to the CMD FIFO front so it executes after PRE recovery.
            CmdFifoEntry act_entry;
            act_entry.command  = act_cmd;
            act_entry.addr_vec = fifo.front().addr_vec;  // same target address
            act_entry.arrive   = m_clk;
            act_entry.is_read  = false;
            act_entry.seq_num  = 0;
            fifo.push_front(act_entry);
            // Issue PRE as recovery
            m_sr_recovery[bank_id] = {true, pre_cmd, make_bank_addr()};
            s_num_sr_recoveries++;
          }
          // else: row matches → no recovery needed
        }
        // front=PRE while bank is OPEN: no recovery (PRE can proceed normally)
      }
      // bank CLOSED: CMD FIFO starts with ACT or PRE → no recovery needed
      // (ACT issues normally on closed bank; PRE on closed bank is a no-op)
    }

    /**
     * Try to issue one command from CMD FIFO for bank_id.
     * Issues SR recovery command first if pending.
     * Returns true if a command was issued.
     */
    bool try_issue_from_cmd_fifo(int bank_id) {
      // Helper: log CMD FIFO issue
      auto log_cmd_fifo_issue = [&](int cmd, int bid) {
        int bg = bid / m_num_banks;
        int bk = bid % m_num_banks;
        int cmd_type = -1;
        if      (cmd == m_dram->m_commands("ACT_L")) cmd_type = 0;
        else if (cmd == m_dram->m_commands("PRE_L")) cmd_type = 1;
        else if (cmd == m_dram->m_commands("RD_L"))  cmd_type = 2;
        else if (cmd == m_dram->m_commands("WR_L"))  cmd_type = 3;
        // [DEBUG] if (cmd_type >= 0)
        //   s_cmd_fifo_issue_log.push_back({bg, bk, cmd_type, m_clk});
      };

      // SR recovery has priority within CMD FIFO issuing
      if (m_sr_recovery[bank_id].pending) {
        int rec_cmd      = m_sr_recovery[bank_id].cmd;
        auto& rec_addr   = m_sr_recovery[bank_id].addr_vec;
        if (m_dram->check_ready(rec_cmd, rec_addr)) {
          m_dram->issue_command(rec_cmd, rec_addr);
          update_fsm_on_nma_command(rec_cmd, rec_addr);
          log_cmd_fifo_issue(rec_cmd, bank_id);
          m_sr_recovery[bank_id].pending = false;
          m_req_rr_bank_idx = (bank_id + 1) % m_total_banks;
          return true;
        }
        return false;  // Stall until recovery is ready
      }

      auto& fifo = m_cmd_fifo[bank_id];
      if (fifo.empty()) return false;

      auto& entry = fifo.front();

      // Bank-state recovery for CMD FIFO command/bank state mismatch:
      const auto& bank = m_bank_fsm[bank_id];

      // Case 1: Bank CLOSED but front is RD/WR → need ACT first
      if (bank.state == BankState::CLOSED &&
          (entry.command == m_dram->m_commands("RD_L") ||
           entry.command == m_dram->m_commands("WR_L"))) {
        int act_cmd = m_dram->m_commands("ACT_L");
        if (m_dram->check_ready(act_cmd, entry.addr_vec)) {
          m_dram->issue_command(act_cmd, entry.addr_vec);
          update_fsm_on_nma_command(act_cmd, entry.addr_vec);
          log_cmd_fifo_issue(act_cmd, bank_id);
          s_num_nma_act++;
          m_req_rr_bank_idx = (bank_id + 1) % m_total_banks;
          return true;
        }
        return false;
      }

      // Case 2: Bank OPENED but front is ACT → need PRE first (close current row)
      if (bank.state == BankState::OPENED &&
          entry.command == m_dram->m_commands("ACT_L")) {
        int pre_cmd = m_dram->m_commands("PRE_L");
        AddrVec_t pre_addr = entry.addr_vec;  // same bank address
        if (m_dram->check_ready(pre_cmd, pre_addr)) {
          m_dram->issue_command(pre_cmd, pre_addr);
          update_fsm_on_nma_command(pre_cmd, pre_addr);
          log_cmd_fifo_issue(pre_cmd, bank_id);
          s_num_nma_pre++;
          m_req_rr_bank_idx = (bank_id + 1) % m_total_banks;
          return true;  // PRE issued; next tick ACT proceeds
        }
        return false;
      }

      if (!m_dram->check_ready(entry.command, entry.addr_vec)) return false;

      // Issue command
      m_dram->issue_command(entry.command, entry.addr_vec);
      update_fsm_on_nma_command(entry.command, entry.addr_vec);
      s_num_cmd_issued++;

      // Per-bank per-command CMD FIFO issue counters
      if      (entry.command == m_dram->m_commands("ACT_L")) { s_num_nma_act++; s_per_bank_cmd[bank_id].issue_act++; }
      else if (entry.command == m_dram->m_commands("PRE_L")) { s_num_nma_pre++; s_per_bank_cmd[bank_id].issue_pre++; }
      else if (entry.command == m_dram->m_commands("RD_L"))  { s_num_nma_rd++; s_num_cmd_fifo_rd++; s_per_bank_cmd[bank_id].issue_rd++; }
      else if (entry.command == m_dram->m_commands("WR_L"))  { s_num_nma_wr++; s_num_cmd_fifo_wr++; s_per_bank_cmd[bank_id].issue_wr++; }
      log_cmd_fifo_issue(entry.command, bank_id);

      bool is_pre = (entry.command == m_dram->m_commands("PRE_L"));
      bool is_rd  = (entry.command == m_dram->m_commands("RD_L"));

      // Schedule completion for Return Unit interrupt generation
      if (is_rd) {
        // Read: completion after nCL + nBL (data available in buffer)
        m_pending_reads.push_back({entry.seq_num, m_clk + m_nCL + m_nBL});
      } else if (entry.command == m_dram->m_commands("WR_L")) {
        // Write: mark done immediately in return_buffer (no data return needed)
        for (auto& rb_entry : m_return_buffer) {
          if (rb_entry.seq_num == entry.seq_num) {
            rb_entry.is_done = true;
            break;
          }
        }
      }

      fifo.pop_front();  // Each CMD FIFO entry is one discrete DRAM command

      // H/N Arbiter: PRE issued → switch to REQ FIFO if it has work
      if (is_pre && !m_nma_req_buffer[bank_id].empty()) {
        m_arbiter_use_cmd[bank_id] = false;
        s_num_hn_switches++;
        m_req_continuous_count[bank_id] = 0;
        m_cmd_quota_remaining[bank_id] = 0;
      }

      m_req_rr_bank_idx = (bank_id + 1) % m_total_banks;
      return true;
    }

    /**
     * Try to issue one command from REQ FIFO for bank_id.
     * Identical prerequisite resolution as try_issue_nma_command(), but per-bank.
     * Returns true if a command was issued.
     */
    bool try_issue_from_req_fifo(int bank_id) {
      auto& fifo = m_nma_req_buffer[bank_id];
      if (fifo.empty()) return false;

      auto& req = fifo.front();

      // Skip if front request is waiting for data response
      if (req.data_pending) return false;

      int cmd = get_nma_preq_command(req);
      if (cmd < 0) return false;  // Bank refreshing

      req.command = cmd;
      if (!m_dram->check_ready(cmd, req.addr_vec)) return false;

      m_dram->issue_command(cmd, req.addr_vec);
      update_fsm_on_nma_command(cmd, req.addr_vec);

      bool is_pre = (cmd == m_dram->m_commands("PRE_L"));

      if      (cmd == m_dram->m_commands("ACT_L")) { s_num_nma_act++; }
      else if (cmd == m_dram->m_commands("PRE_L")) { s_num_nma_pre++; }
      else if (cmd == m_dram->m_commands("RD_L"))  {
        s_num_nma_rd++; s_num_req_fifo_rd++;
        req.data_pending = true;
        req.data_complete_clk = m_clk + m_nCL + m_nBL;
      }
      else if (cmd == m_dram->m_commands("WR_L"))  {
        s_num_nma_wr++; s_num_req_fifo_wr++;
        req.data_pending = true;
        req.data_complete_clk = m_clk + m_nCWL + m_nBL;
      }

      // PRE issued from REQ FIFO → reset row-hit counter (row closed)
      if (is_pre) {
        m_req_row_hit_count[bank_id] = 0;
        m_req_row_hit_row[bank_id] = -1;
      }

      // H/N Arbiter: PRE issued → switch to CMD FIFO if it has work
      if (is_pre && !m_cmd_fifo[bank_id].empty()) {
        switch_to_cmd_fifo(bank_id);
      }

      m_req_rr_bank_idx = (bank_id + 1) % m_total_banks;
      return true;
    }

    /**
     * Mark pending reads as done when their completion time arrives.
     * Called every DRAM cycle in concurrent mode.
     */
    void process_pending_reads() {
      for (auto it = m_pending_reads.begin(); it != m_pending_reads.end(); ) {
        if (it->complete_clk <= m_clk) {
          // Mark the corresponding return_buffer entry as done
          for (auto& entry : m_return_buffer) {
            if (entry.seq_num == it->seq_num) {
              entry.is_done = true;
              break;
            }
          }
          it = m_pending_reads.erase(it);
        } else {
          ++it;
        }
      }
    }

    /**
     * TDM interrupt: fire at most 1 interrupt per TDM period (N_rank cycles).
     * Each interrupt signals completion of the OLDEST offloaded access (head-of-line).
     * Paper §IV.C: "one-bit alert_n signal", "restrict interrupts at offload order".
     */
    void check_and_send_interrupt() {
      if (m_interrupt_cb == nullptr) return;
      if (m_return_buffer.empty()) return;
      if (!m_return_buffer.front().is_done) return;

      // TDM rate limit: 1 interrupt per N_rank cycles
      if ((m_clk - m_last_interrupt_clk) < (Clk_t)TDM_INTERRUPT_PERIOD) return;

      // Fire ONE interrupt (1 access completion, always batch_size=1)
      auto& front = m_return_buffer.front();
      // [DEBUG] s_interrupt_log.push_back({front.seq_num, front.bg, front.bk, front.is_read, m_clk});
      m_return_buffer.pop_front();
      m_pending_interrupts.push_back({1, m_clk + TDM_INTERRUPT_LATENCY});
      m_last_interrupt_clk = m_clk;
      s_num_interrupts_sent++;
    }

    /**
     * Deliver pending TDM interrupts that have reached their delivery clock.
     * Called every DRAM cycle in concurrent mode (from tick()).
     */
    void deliver_pending_interrupts() {
      while (!m_pending_interrupts.empty() &&
             m_pending_interrupts.front().deliver_clk <= m_clk) {
        auto& pi = m_pending_interrupts.front();
        m_interrupt_cb(m_rank_id, pi.batch_size);
        m_pending_interrupts.pop_front();
      }
    }

    void handle_future_action(const FutureAction& action) {
      switch (action.type) {
        case FutureAction::Type::REFAB_END:
          for (auto& bank : m_bank_fsm)
            bank.state = BankState::CLOSED;
          break;

        case FutureAction::Type::REFSB_END: {
          int target_bank = action.addr_vec[m_bank_level];
          for (int bg = 0; bg < m_num_bankgroups; bg++) {
            int flat_id = bg * m_num_banks + target_bank;
            m_bank_fsm[flat_id].state = BankState::CLOSED;
          }
          break;
        }
      }
    }

    // ===== NMAInst Decode (magic path) =====

    /**
     * Decode 64-bit encoded NMA instruction (unified format).
     * Bit layout:
     *   [63:58] comp_opcode (6b)
     *   [57:51] opsize      (7b)
     *   [50:48] bg          (3b)
     *   [47:46] bk          (2b)
     *   [45:28] row         (18b)
     *   [27:21] col         (7b)
     *   [20:18] id          (3b)
     *   [17:12] reserved    (6b)
     *   [11:0]  etc         (12b)
     */
    static NMAInst_Slot decode_nma_inst(uint64_t raw) {
      int comp_opcode = static_cast<int>((raw >> 58) & 0x3F);
      int opsize      = static_cast<int>((raw >> 51) & 0x7F);
      int bg          = static_cast<int>((raw >> 48) & 0x7);
      int bk          = static_cast<int>((raw >> 46) & 0x3);
      int row         = static_cast<int>((raw >> 28) & 0x3FFFF);
      int col         = static_cast<int>((raw >> 21) & 0x7F);
      int id          = static_cast<int>((raw >> 18) & 0x7);
      int etc         = static_cast<int>( raw        & 0xFFF);
      return NMAInst_Slot(true, comp_opcode, opsize, bg, bk, row, col, id, etc);
    }

    // ===== Phase 2: NMA State Machine =====

    void tick_nma_state_machine() {
      switch (m_nma_state) {
        case NMAState::NMA_IDLE:
          // In concurrent mode, host offloaded commands may still arrive in CMD FIFO
          // after NMA execution completes. Must continue draining CMD FIFO and refresh.
          if (m_concurrent_mode) {
            check_nma_refresh();
            try_issue_concurrent_command();
          }
          s_nma_idle_cycles++;
          break;

        case NMAState::NMA_ISSUE_START:
          tick_nma_issue_start();
          s_nma_issue_start_cycles++;
          break;

        case NMAState::NMA_RUN:
          tick_nma_run();
          s_nma_run_cycles++;
          break;

        case NMAState::NMA_BAR:
          tick_nma_bar();
          s_nma_bar_cycles++;
          break;

        case NMAState::NMA_WAIT:
          tick_nma_wait();
          s_nma_wait_cycles++;
          break;

        case NMAState::NMA_DONE:
          tick_nma_done();
          s_nma_done_cycles++;
          break;
      }
    }

    /**
     * NMA_ISSUE_START: Wait for any ongoing refresh to complete,
     * then transition to NMA_RUN.
     */
    void tick_nma_issue_start() {
      if (!any_bank_refreshing())
        m_nma_state = NMAState::NMA_RUN;
    }

    /**
     * NMA_RUN: Main execution loop
     * 1. Check refresh needs
     * 2. Try to issue DRAM command (per-bank front scheduling)
     * 3. Generate new NMA requests from address generation slots
     * 4. Fetch new instructions from instruction queue
     */
    void tick_nma_run() {
      check_nma_refresh();
      if (m_concurrent_mode) try_issue_concurrent_command();
      else                   try_issue_nma_command();
      generate_nma_requests();
      fetch_nma_instructions();
    }

    /**
     * NMA_BAR: Barrier — wait for all per-bank FIFOs and addr_gen_slots to drain.
     */
    void tick_nma_bar() {
      if (m_concurrent_mode) try_issue_concurrent_command();
      else                   try_issue_nma_command();
      generate_nma_requests();

      if (m_addr_gen_slots.empty() && all_req_buffers_empty()) {
        m_nma_state = NMAState::NMA_RUN;
      }
    }

    /**
     * NMA_WAIT: Fixed-latency wait (used for compute-only instructions).
     */
    void tick_nma_wait() {
      if (m_concurrent_mode) try_issue_concurrent_command();
      else                   try_issue_nma_command();
      generate_nma_requests();

      m_nma_wait_cnt++;
      if (m_nma_wait_cnt >= m_nma_wait_target) {
        if (m_addr_gen_slots.empty() && all_req_buffers_empty()) {
          m_nma_state = NMAState::NMA_RUN;
        }
      }
    }

    /**
     * NMA_DONE: Drain all outstanding requests, then close all banks (N2H implicit sync).
     */
    void tick_nma_done() {
      if (m_concurrent_mode) try_issue_concurrent_command();
      else                   try_issue_nma_command();
      generate_nma_requests();

      if (m_addr_gen_slots.empty() && all_req_buffers_empty()) {
        if (m_concurrent_mode) {
          // Concurrent mode: NMA work is done, transition to IDLE immediately.
          // Do NOT issue PREA — host offloaded commands (CMD FIFO) may still use open banks.
          // Bank cleanup happens during C2H transition.
          m_nma_program.clear();
          m_nma_state = NMAState::NMA_IDLE;
        } else {
          // NMA-only mode: close all banks (N2H implicit sync) before going IDLE.
          bool all_closed = true;
          for (const auto& bank : m_bank_fsm) {
            if (bank.state == BankState::OPENED) { all_closed = false; break; }
          }
          if (all_closed) {
            m_nma_program.clear();
            m_nma_state = NMAState::NMA_IDLE;
          } else {
            issue_nma_prea();
          }
        }
      }
    }

    // ===== NMA DRAM Command Scheduling (per-bank front) =====

    /**
     * Try to issue one DRAM command.
     *
     * Strategy:
     *   - Refresh first if pending and all banks closed.
     *   - Round-robin over all banks.
     *   - For each bank: check the front of its REQ FIFO.
     *     Determine prerequisite command (ACT / PRE / RD / WR).
     *     If DRAM timing allows, issue and advance round-robin.
     *   - One command per NMA tick.
     */
    void try_issue_nma_command() {
      // Refresh has highest priority — block ALL other commands until refresh completes
      if (m_nma_refresh_pending) {
        try_issue_nma_refresh();
        return;
      }

      // Round-robin over banks: check front of each FIFO
      for (int i = 0; i < m_total_banks; i++) {
        int bank_id = (m_req_rr_bank_idx + i) % m_total_banks;
        auto& fifo = m_nma_req_buffer[bank_id];
        if (fifo.empty()) continue;

        auto& req = fifo.front();

        // Skip banks whose front request is waiting for data response
        if (req.data_pending) continue;

        int cmd = get_nma_preq_command(req);
        if (cmd < 0) continue;  // Bank is refreshing, skip

        req.command = cmd;
        if (!m_dram->check_ready(req.command, req.addr_vec)) continue;

        // Issue the command
        m_dram->issue_command(req.command, req.addr_vec);
        update_fsm_on_nma_command(req.command, req.addr_vec);

        if (req.command == m_dram->m_commands("ACT_L")) {
          s_num_nma_act++;
        } else if (req.command == m_dram->m_commands("PRE_L")) {
          s_num_nma_pre++;
        } else if (req.command == m_dram->m_commands("RD_L")) {
          s_num_nma_rd++; s_num_req_fifo_rd++;
          // Don't pop — wait for data response (nCL + nBL)
          req.data_pending = true;
          req.data_complete_clk = m_clk + m_nCL + m_nBL;
          m_req_rr_bank_idx = (bank_id + 1) % m_total_banks;
        } else if (req.command == m_dram->m_commands("WR_L")) {
          s_num_nma_wr++; s_num_req_fifo_wr++;
          // Don't pop — wait for write data transfer (nCWL + nBL)
          req.data_pending = true;
          req.data_complete_clk = m_clk + m_nCWL + m_nBL;
          m_req_rr_bank_idx = (bank_id + 1) % m_total_banks;
        }

        return;  // One command per NMA tick
      }
    }

    /**
     * Process NMA REQ FIFO data completions.
     * Called every DRAM cycle (data responses arrive at DRAM clock granularity).
     * When data_complete_clk is reached, pop the request from the FIFO.
     */
    void process_nma_data_completions() {
      for (int bank_id = 0; bank_id < m_total_banks; bank_id++) {
        auto& fifo = m_nma_req_buffer[bank_id];
        if (fifo.empty()) continue;
        auto& req = fifo.front();
        if (req.data_pending && m_clk >= req.data_complete_clk) {
          fifo.pop_front();
        }
      }
    }

    /**
     * Determine the prerequisite command for the front request of a bank's FIFO.
     * Returns: command id to issue, or -1 if bank is refreshing (skip).
     */
    int get_nma_preq_command(const NMARequest& req) {
      int flat_id = get_flat_bank_id(req.addr_vec);
      const auto& bank = m_bank_fsm[flat_id];

      if (bank.state == BankState::REFRESHING) return -1;

      if (bank.state == BankState::CLOSED)
        return m_dram->m_commands("ACT_L");

      if (bank.state == BankState::OPENED) {
        if (bank.open_row == req.addr_vec[m_row_level])
          return req.final_command;   // Row hit → RD_L or WR_L
        else
          return m_dram->m_commands("PRE_L");  // Row conflict
      }

      return -1;
    }

    /**
     * Update shadow FSM after NMA MC issues a DRAM command.
     */
    void update_fsm_on_nma_command(int command, const AddrVec_t& addr_vec) {
      if (command == m_dram->m_commands("ACT") || command == m_dram->m_commands("ACT_L")) {
        int flat_id = get_flat_bank_id(addr_vec);
        m_bank_fsm[flat_id].state = BankState::OPENED;
        m_bank_fsm[flat_id].open_row = addr_vec[m_row_level];
      }
      else if (command == m_dram->m_commands("PRE") || command == m_dram->m_commands("PRE_L")) {
        int flat_id = get_flat_bank_id(addr_vec);
        m_bank_fsm[flat_id].state = BankState::CLOSED;
        m_bank_fsm[flat_id].open_row = -1;
      }
      else if (command == m_dram->m_commands("PREA") || command == m_dram->m_commands("PREA_L")) {
        for (auto& bank : m_bank_fsm) {
          bank.state = BankState::CLOSED;
          bank.open_row = -1;
        }
      }
      else if (command == m_dram->m_commands("REFab") || command == m_dram->m_commands("REFab_L")) {
        for (auto& bank : m_bank_fsm) {
          bank.state = BankState::REFRESHING;
          bank.open_row = -1;
        }
        m_future_actions.push_back({
          FutureAction::Type::REFAB_END,
          addr_vec,
          m_clk + m_nRFC1 - 1
        });
      }
    }

    // ===== NMA Address Generation =====

    /**
     * Generate DRAM requests from address generation slots into per-bank FIFOs.
     * Each NMAInst_Slot with memory access is expanded into sequential column accesses.
     *
     * Memory access classification:
     *   READ  ops (LOAD, LOAD_ADD, LOAD_MUL, ADD, MUL, V_RED, S_RED, MAC, SCALE_ADD):
     *     X operand fetched from DRAM → RD × opsize
     *   WRITE ops (WBD):
     *     Y (VMA buffer) written back to DRAM → WR × opsize
     *   NONE  ops (T_ADD, T_MUL, T_V_RED, T_S_RED, T_MAC):
     *     Both operands Y/Y2 from VMA; no DRAM access (handled in fetch_nma_instructions)
     *
     * Round-robin across active addr_gen_slots. One request per NMA tick.
     */
    void generate_nma_requests() {
      if (m_addr_gen_slots.empty()) return;

      int start_idx = m_addr_gen_slots.size() > 0 ? 0 : -1;
      for (int i = 0; i < (int)m_addr_gen_slots.size(); i++) {
        auto& slot = m_addr_gen_slots[i];

        // Determine target bank FIFO
        int flat_bank_id = slot.inst.bg * m_num_banks + slot.inst.bk;
        auto& fifo = m_nma_req_buffer[flat_bank_id];

        if ((int)fifo.size() >= m_eff_req_fifo_size) continue;  // Bank FIFO full

        // Row-hit low cap: in concurrent mode, when CMD FIFO has work,
        // limit same-row REQ generation so CMD FIFO gets execution opportunity.
        int reg_id = slot.inst.id & 0x7;
        int target_row = m_base_reg[reg_id] + slot.inst.row;
        if (m_concurrent_mode && !m_cmd_fifo[flat_bank_id].empty()) {
          if (m_req_row_hit_row[flat_bank_id] == target_row &&
              m_req_row_hit_count[flat_bank_id] >= NMA_ROW_HIT_LOW_CAP) {
            continue;  // Cap reached — let CMD FIFO run
          }
        }

        // Build request
        NMARequest req;
        auto mem_type = slot.inst.get_mem_access_type();
        req.is_read = (mem_type == NMAInst_Slot::MemAccessType::READ);
        req.addr_vec.resize(m_dram->m_levels.size(), -1);
        req.addr_vec[m_channel_level] = m_channel_id;
        req.addr_vec[m_rank_level]    = m_rank_id;
        req.addr_vec[m_bankgroup_level] = slot.inst.bg;
        req.addr_vec[m_bank_level]    = slot.inst.bk;
        req.addr_vec[m_row_level]     = target_row;
        req.addr_vec[m_column_level]  = slot.current_col;
        req.final_command = req.is_read ? m_dram->m_commands("RD_L")
                                        : m_dram->m_commands("WR_L");
        req.arrive = m_clk;

        fifo.push_back(req);

        // Update row-hit tracking
        if (m_req_row_hit_row[flat_bank_id] != target_row) {
          m_req_row_hit_row[flat_bank_id] = target_row;
          m_req_row_hit_count[flat_bank_id] = 1;
        } else {
          m_req_row_hit_count[flat_bank_id]++;
        }

        // Advance column; remove slot when complete
        slot.issued_count++;
        slot.current_col++;

        if (slot.issued_count >= slot.total_accesses) {
          m_addr_gen_slots.erase(m_addr_gen_slots.begin() + i);
        }

        return;  // One request per NMA tick
      }
    }

    /**
     * Fetch and execute the instruction at m_nma_pc from m_nma_program.
     * PC-based execution (DBX-DIMM style): no inst_queue deque, direct vector access.
     *
     * LOOP: in-place counter. inst.cnt tracks current iteration.
     *   cnt < opsize → cnt++, pc = jump_pc (loop back)
     *   cnt >= opsize → cnt = 0, pc++ (fall through, loop done)
     *   Total iterations = opsize + 1 (initial body + opsize repeats)
     */
    void fetch_nma_instructions() {
      if (m_nma_pc >= (int)m_nma_program.size()) return;
      if ((int)m_addr_gen_slots.size() >= ADDR_GEN_SLOT_MAX) return;

      NMAInst_Slot& inst = m_nma_program[m_nma_pc];
      s_num_nma_instructions++;

      auto mem_type = inst.get_mem_access_type();

      if (mem_type == NMAInst_Slot::MemAccessType::CONTROL) {
        switch (inst.comp_opcode) {
          case NMAInst_Slot::BARRIER:
            m_nma_pc++;
            m_nma_state = NMAState::NMA_BAR;
            return;

          case NMAInst_Slot::EXIT:
            m_nma_pc++;
            m_nma_state = NMAState::NMA_DONE;
            return;

          case NMAInst_Slot::SET_BASE: {
            // Set base address register: base_reg[id] = row
            int reg_id = inst.id & 0x7;
            m_base_reg[reg_id] = inst.row;
            m_nma_pc++;
            return;
          }

          case NMAInst_Slot::INC_BASE: {
            // Increment base address register: base_reg[id] += row (stride, 18-bit)
            int reg_id = inst.id & 0x7;
            m_base_reg[reg_id] += inst.row;
            m_nma_pc++;
            return;
          }

          case NMAInst_Slot::LOOP: {
            // DBX-DIMM style: in-place counter using inst.cnt
            // loop_cnt = number of additional iterations (total = loop_cnt + 1)
            // jump_pc = target PC to jump back to
            // Note: loop_cnt/jump_pc decoded from etc field in NMAInst_Slot constructor
            int loop_limit = inst.loop_cnt; // From etc[11:6], NOT opsize
            int jump_pc    = inst.jump_pc;  // From etc[5:0]
            s_num_nma_loops++;

            if (inst.cnt < loop_limit) {
              inst.cnt++;          // Increment iteration counter
              m_nma_pc = jump_pc;  // Jump back to loop body start
            } else {
              inst.cnt = 0;        // Reset for potential future re-execution
              m_nma_pc++;          // Fall through (loop done)
            }
            return;
          }

          default:
            m_nma_pc++;
            return;
        }
      }
      else if (mem_type == NMAInst_Slot::MemAccessType::NONE) {
        // VMA-internal compute (T_* ops). Latency scales with opsize.
        // Each element takes base_latency NMA ticks; total = base_latency × opsize.
        int base_latency = 1;
        switch (inst.comp_opcode) {
          case NMAInst_Slot::T_ADD:   base_latency = COMPUTE_LATENCY_T_ADD;  break;
          case NMAInst_Slot::T_MUL:   base_latency = COMPUTE_LATENCY_T_MUL;  break;
          case NMAInst_Slot::T_V_RED: base_latency = COMPUTE_LATENCY_T_VRED; break;
          case NMAInst_Slot::T_S_RED: base_latency = COMPUTE_LATENCY_T_SRED; break;
          case NMAInst_Slot::T_MAC:   base_latency = COMPUTE_LATENCY_T_MAC;  break;
          default:                    base_latency = 1; break;
        }

        m_nma_wait_cnt = 0;
        m_nma_wait_target = base_latency * std::max(1, (int)inst.opsize + 1);  // 0-indexed
        m_nma_pc++;
        m_nma_state = NMAState::NMA_WAIT;
        s_num_nma_compute_only++;
        return;
      }
      else {
        // Memory access instruction (READ/WRITE) → create addr_gen_slot
        AddrGenSlot slot;
        slot.inst = inst;
        slot.total_accesses = inst.get_total_accesses();
        slot.issued_count = 0;
        slot.current_col = inst.col;
        m_addr_gen_slots.push_back(slot);
        m_nma_pc++;
      }
    }

    // ===== NMA Refresh Management =====

    /**
     * Check if refresh is needed.
     * REFO-based: m_nma_refresh_pending is set by bypass_command() when REFO received.
     * No independent nREFI tracking — Host MC's RefreshManager is the authority.
     */
    void check_nma_refresh() {
      // m_nma_refresh_pending is set externally via REFO from Host MC.
      // Nothing to do here — just a compatibility shim for tick_nma_run().
    }

    bool try_issue_nma_refresh() {
      if (!all_banks_closed()) {
        // Banks still open → issue PREA to force-close all banks
        // Next tick: all_banks_closed() → REFab
        issue_nma_prea();
        return true;  // PREA issued this tick
      }

      AddrVec_t ref_addr_vec(m_dram->m_levels.size(), -1);
      ref_addr_vec[m_channel_level] = m_channel_id;
      ref_addr_vec[m_rank_level]    = m_rank_id;

      int ref_cmd = m_dram->m_commands("REFab_L");
      if (m_dram->check_ready(ref_cmd, ref_addr_vec)) {
        m_dram->issue_command(ref_cmd, ref_addr_vec);
        update_fsm_on_nma_command(ref_cmd, ref_addr_vec);
        m_last_refresh_clk = m_clk;
        m_nma_refresh_pending = false;
        s_num_nma_ref++;

        // SR Unit reset: after refresh all banks = CLOSED (ground truth).
        // Both CMD FIFO and REQ FIFO perspectives are invalidated.
        // Clear SR state and pending recoveries to avoid stale mismatches.
        for (int b = 0; b < m_total_banks; b++) {
          m_sr_unit[b] = SRState{};
          m_sr_recovery[b].pending = false;
        }
        return true;
      }
      return false;
    }

    void issue_nma_prea() {
      AddrVec_t prea_addr_vec(m_dram->m_levels.size(), -1);
      prea_addr_vec[m_channel_level] = m_channel_id;
      prea_addr_vec[m_rank_level]    = m_rank_id;

      int prea_cmd = m_dram->m_commands("PREA_L");
      if (m_dram->check_ready(prea_cmd, prea_addr_vec)) {
        m_dram->issue_command(prea_cmd, prea_addr_vec);
        update_fsm_on_nma_command(prea_cmd, prea_addr_vec);
        s_num_nma_pre++;
      }
    }
};

}  // namespace Ramulator

#endif  // RAMULATOR_ASYNCDIMM_NMA_CONTROLLER_H
