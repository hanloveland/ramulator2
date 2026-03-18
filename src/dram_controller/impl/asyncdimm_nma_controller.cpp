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
 *   - inst-buf WR → decode_nma_inst() → m_nma_inst_queue
 *   - NMA state machine (IDLE → ISSUE_START → RUN → BAR → WAIT → DONE)
 *   - Per-bank REQ FIFOs: m_nma_req_buffer[flat_bank_id] (max 8 per bank)
 *   - Scheduler issues front of each bank's FIFO (round-robin, first ready)
 *   - comp_opcode → memory access type mapping (RD/WR/none/control)
 *   - Uses shadow FSM to determine ACT/PRE prerequisites
 *   - Handles refresh internally during NMA Mode
 *   - Compute-only instructions (ADD/MUL) modeled with fixed latency
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

    // NMAInst_Slot-based workload queue
    // Loaded from DQ magic-path WR intercept
    std::deque<NMAInst_Slot> m_nma_inst_queue;       // Instruction queue
    static constexpr int NMA_INST_QUEUE_MAX = 256;   // Large enough to hold full program

    // Program Counter for LOOP support
    int m_nma_pc = 0;
    std::vector<NMAInst_Slot> m_nma_program;  // Full program (for LOOP jump-back)

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
    };
    std::vector<std::deque<NMARequest>> m_nma_req_buffer;  // [total_banks]
    static constexpr int NMA_REQ_BUFFER_PER_BANK = 8;

    // Round-robin bank index for per-bank front scheduling
    int m_req_rr_bank_idx = 0;

    // NMA Wait counter (for compute-only instructions and fixed-latency waits)
    int m_nma_wait_cnt = 0;
    int m_nma_wait_target = 0;

    // Compute latency constants (in NMA clock ticks)
    static constexpr int COMPUTE_LATENCY_ADD = 1;
    static constexpr int COMPUTE_LATENCY_MUL = 2;

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
    static constexpr int NMA_CLOCK_RATIO = 4;

  public:
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

      // Store magic path addresses
      m_nma_ctrl_row = nma_ctrl_row;
      m_nma_ctrl_bg  = nma_ctrl_bg;
      m_nma_ctrl_bk  = nma_ctrl_bk;
      m_nma_buf_bg   = nma_buf_bg;
      m_nma_buf_bk   = nma_buf_bk;
    }

    void set_channel_id(int ch_id) { m_channel_id = ch_id; }

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
      m_nma_state = NMAState::NMA_ISSUE_START;
      m_last_refresh_clk = m_clk;
      s_num_nma_executions++;
    }

    /**
     * Check if NMA MC has finished all work (for mode transition back to HOST).
     * m_nma_outstanding == sum(m_nma_req_buffer[i].size()), so checking all
     * per-bank FIFOs empty is sufficient.
     */
    bool is_nma_complete() const {
      return m_nma_state == NMAState::NMA_IDLE &&
             m_nma_inst_queue.empty() &&
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
            // NMAInst buffer write → decode each payload word as an NMAInst_Slot
            for (const auto& pval : payload) {
              if (pval == 0) continue;
              NMAInst_Slot inst = decode_nma_inst(pval);
              if (m_nma_inst_queue.size() < NMA_INST_QUEUE_MAX) {
                m_nma_inst_queue.push_back(inst);
                m_nma_program.push_back(inst);
              }
            }
            s_num_magic_inst_wr++;
          }
          else if (bg == m_nma_ctrl_bg && bk == m_nma_ctrl_bk) {
            // Ctrl-reg write → check if this rank should start
            // payload[rank_id] != 0 means start NMA for that rank
            if (m_rank_id < (int)payload.size() && payload[m_rank_id] != 0) {
              m_nma_start_pending = true;
            }
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
          std::cout << "  REQ_FIFO=" << m_nma_req_buffer[flat_id].size() << std::endl;
        }
      }
    }

  private:
    int get_flat_bank_id(const AddrVec_t& addr_vec) const {
      return addr_vec[m_bankgroup_level] * m_num_banks + addr_vec[m_bank_level];
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
      try_issue_nma_command();
      generate_nma_requests();
      fetch_nma_instructions();
    }

    /**
     * NMA_BAR: Barrier — wait for all per-bank FIFOs and addr_gen_slots to drain.
     */
    void tick_nma_bar() {
      try_issue_nma_command();
      generate_nma_requests();

      if (m_addr_gen_slots.empty() && all_req_buffers_empty()) {
        m_nma_state = NMAState::NMA_RUN;
      }
    }

    /**
     * NMA_WAIT: Fixed-latency wait (used for compute-only instructions).
     */
    void tick_nma_wait() {
      try_issue_nma_command();
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
      try_issue_nma_command();
      generate_nma_requests();

      if (m_addr_gen_slots.empty() && all_req_buffers_empty()) {
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
      // Refresh has highest priority
      if (m_nma_refresh_pending) {
        if (try_issue_nma_refresh()) return;
      }

      // Round-robin over banks: check front of each FIFO
      for (int i = 0; i < m_total_banks; i++) {
        int bank_id = (m_req_rr_bank_idx + i) % m_total_banks;
        auto& fifo = m_nma_req_buffer[bank_id];
        if (fifo.empty()) continue;

        auto& req = fifo.front();
        int cmd = get_nma_preq_command(req);
        if (cmd < 0) continue;  // Bank is refreshing, skip

        req.command = cmd;
        if (!m_dram->check_ready(req.command, req.addr_vec)) continue;

        // Issue the command
        m_dram->issue_command(req.command, req.addr_vec);
        update_fsm_on_nma_command(req.command, req.addr_vec);

        if (req.command == m_dram->m_commands("ACT")) {
          s_num_nma_act++;
          // Request stays at front of FIFO; next tick will issue RD/WR
        } else if (req.command == m_dram->m_commands("PRE")) {
          s_num_nma_pre++;
          // Request stays at front; next tick ACT→RD/WR
        } else if (req.command == m_dram->m_commands("RD")) {
          s_num_nma_rd++;
          fifo.pop_front();
          m_req_rr_bank_idx = (bank_id + 1) % m_total_banks;
        } else if (req.command == m_dram->m_commands("WR")) {
          s_num_nma_wr++;
          fifo.pop_front();
          m_req_rr_bank_idx = (bank_id + 1) % m_total_banks;
        }

        return;  // One command per NMA tick
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
        return m_dram->m_commands("ACT");

      if (bank.state == BankState::OPENED) {
        if (bank.open_row == req.addr_vec[m_row_level])
          return req.final_command;   // Row hit
        else
          return m_dram->m_commands("PRE");  // Row conflict
      }

      return -1;
    }

    /**
     * Update shadow FSM after NMA MC issues a DRAM command.
     */
    void update_fsm_on_nma_command(int command, const AddrVec_t& addr_vec) {
      if (command == m_dram->m_commands("ACT")) {
        int flat_id = get_flat_bank_id(addr_vec);
        m_bank_fsm[flat_id].state = BankState::OPENED;
        m_bank_fsm[flat_id].open_row = addr_vec[m_row_level];
      }
      else if (command == m_dram->m_commands("PRE")) {
        int flat_id = get_flat_bank_id(addr_vec);
        m_bank_fsm[flat_id].state = BankState::CLOSED;
        m_bank_fsm[flat_id].open_row = -1;
      }
      else if (command == m_dram->m_commands("PREA")) {
        for (auto& bank : m_bank_fsm) {
          bank.state = BankState::CLOSED;
          bank.open_row = -1;
        }
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
      }
    }

    // ===== NMA Address Generation =====

    /**
     * Generate DRAM requests from address generation slots into per-bank FIFOs.
     * Each NMAInst_Slot with memory access is expanded into sequential column accesses.
     * Transposed ops (T_*) generate 2× the accesses (opsize×2) in the same bank/row.
     *
     * NOTE: True transposed operand from a different row/bank is expected to be
     * encoded as a separate NMAInst_Slot by the trace generator.
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

        if ((int)fifo.size() >= NMA_REQ_BUFFER_PER_BANK) continue;  // Bank FIFO full

        // Build request
        NMARequest req;
        auto mem_type = slot.inst.get_mem_access_type();
        req.is_read = (mem_type == NMAInst_Slot::MemAccessType::READ);
        req.addr_vec.resize(m_dram->m_levels.size(), -1);
        req.addr_vec[m_channel_level] = m_channel_id;
        req.addr_vec[m_rank_level]    = m_rank_id;
        req.addr_vec[m_bankgroup_level] = slot.inst.bg;
        req.addr_vec[m_bank_level]    = slot.inst.bk;
        req.addr_vec[m_row_level]     = slot.inst.row;
        req.addr_vec[m_column_level]  = slot.current_col;
        req.final_command = req.is_read ? m_dram->m_commands("RD")
                                        : m_dram->m_commands("WR");
        req.arrive = m_clk;

        fifo.push_back(req);

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
     * Fetch instructions from instruction queue into address generation slots.
     */
    void fetch_nma_instructions() {
      if (m_nma_inst_queue.empty()) return;
      if ((int)m_addr_gen_slots.size() >= ADDR_GEN_SLOT_MAX) return;

      NMAInst_Slot& inst = m_nma_inst_queue.front();
      s_num_nma_instructions++;
      m_nma_pc++;

      auto mem_type = inst.get_mem_access_type();

      if (mem_type == NMAInst_Slot::MemAccessType::CONTROL) {
        switch (inst.comp_opcode) {
          case NMAInst_Slot::BARRIER:
            m_nma_inst_queue.pop_front();
            m_nma_state = NMAState::NMA_BAR;
            return;

          case NMAInst_Slot::EXIT:
            m_nma_inst_queue.pop_front();
            m_nma_state = NMAState::NMA_DONE;
            return;

          case NMAInst_Slot::LOOP: {
            // CRITICAL: Copy inst BEFORE pop_front() to avoid dangling reference.
            NMAInst_Slot loop_inst = inst;
            int loop_cnt = (loop_inst.etc >> 6) & 0x3F;
            int jump_pc  = loop_inst.etc & 0x3F;
            s_num_nma_loops++;

            if (loop_cnt > 0) {
              loop_inst.etc = ((loop_cnt - 1) << 6) | jump_pc;

              // Determine body range [jump_pc, loop_prog_end) in m_nma_program.
              // m_nma_pc already incremented → LOOP is at index [m_nma_pc - 1].
              // Cache in cnt for subsequent iterations (m_nma_pc drifts after re-fetch).
              int loop_prog_end;
              if (loop_inst.cnt == 0) {
                loop_prog_end = m_nma_pc - 1;
                loop_inst.cnt = loop_prog_end;
              } else {
                loop_prog_end = loop_inst.cnt;
              }

              m_nma_inst_queue.pop_front();  // inst now invalid

              if (jump_pc < loop_prog_end &&
                  loop_prog_end <= (int)m_nma_program.size()) {
                std::deque<NMAInst_Slot> loop_body;
                for (int pc = jump_pc; pc < loop_prog_end; pc++)
                  loop_body.push_back(m_nma_program[pc]);
                loop_body.push_back(loop_inst);  // Decremented LOOP

                for (int j = (int)loop_body.size() - 1; j >= 0; j--)
                  m_nma_inst_queue.push_front(loop_body[j]);
              }
            } else {
              m_nma_inst_queue.pop_front();
            }
            return;
          }

          default:
            m_nma_inst_queue.pop_front();
            return;
        }
      }
      else if (mem_type == NMAInst_Slot::MemAccessType::NONE) {
        // Compute-only (ADD/MUL): model as fixed-latency wait
        int latency = 1;
        if (inst.comp_opcode == NMAInst_Slot::ADD) latency = COMPUTE_LATENCY_ADD;
        if (inst.comp_opcode == NMAInst_Slot::MUL) latency = COMPUTE_LATENCY_MUL;

        m_nma_wait_cnt = 0;
        m_nma_wait_target = latency;
        m_nma_inst_queue.pop_front();
        m_nma_state = NMAState::NMA_WAIT;
        s_num_nma_compute_only++;
        return;
      }
      else {
        // Memory access instruction → create addr_gen_slot
        AddrGenSlot slot;
        slot.inst = inst;
        slot.total_accesses = inst.get_total_accesses();
        slot.issued_count = 0;
        slot.current_col = inst.col;
        m_addr_gen_slots.push_back(slot);
        m_nma_inst_queue.pop_front();
      }
    }

    // ===== NMA Refresh Management =====

    void check_nma_refresh() {
      if (m_nma_state == NMAState::NMA_IDLE) return;
      if ((m_clk - m_last_refresh_clk) >= (Clk_t)m_nREFI)
        m_nma_refresh_pending = true;
    }

    bool try_issue_nma_refresh() {
      if (!all_banks_closed()) return false;

      AddrVec_t ref_addr_vec(m_dram->m_levels.size(), -1);
      ref_addr_vec[m_channel_level] = m_channel_id;
      ref_addr_vec[m_rank_level]    = m_rank_id;

      int ref_cmd = m_dram->m_commands("REFab");
      if (m_dram->check_ready(ref_cmd, ref_addr_vec)) {
        m_dram->issue_command(ref_cmd, ref_addr_vec);
        update_fsm_on_nma_command(ref_cmd, ref_addr_vec);
        m_last_refresh_clk = m_clk;
        m_nma_refresh_pending = false;
        s_num_nma_ref++;
        return true;
      }
      return false;
    }

    void issue_nma_prea() {
      AddrVec_t prea_addr_vec(m_dram->m_levels.size(), -1);
      prea_addr_vec[m_channel_level] = m_channel_id;
      prea_addr_vec[m_rank_level]    = m_rank_id;

      int prea_cmd = m_dram->m_commands("PREA");
      if (m_dram->check_ready(prea_cmd, prea_addr_vec)) {
        m_dram->issue_command(prea_cmd, prea_addr_vec);
        update_fsm_on_nma_command(prea_cmd, prea_addr_vec);
        s_num_nma_pre++;
      }
    }
};

}  // namespace Ramulator

#endif  // RAMULATOR_ASYNCDIMM_NMA_CONTROLLER_H
