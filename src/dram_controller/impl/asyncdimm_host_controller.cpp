#include "dram_controller/controller.h"
#include "memory_system/memory_system.h"

namespace Ramulator {

// AsyncDIMM Concurrent Mode state (per-rank)
enum class AsyncDIMMMode {
  HOST = 0,       // Host MC has exclusive DRAM access
  NMA,            // NMA MC has exclusive DRAM access (Phase 2)
  CONCURRENT,     // Both Host MC and NMA MC access DRAM (Phase 3)
};

class AsyncDIMMHostController final : public IDRAMController, public Implementation {
  RAMULATOR_REGISTER_IMPLEMENTATION(IDRAMController, AsyncDIMMHostController, "AsyncDIMMHost", "AsyncDIMM Host Memory Controller (HPCA 2025).")
  private:
    std::deque<Request> pending;          // Requests waiting for read latency completion

    ReqBuffer m_active_buffer;            // Buffer for requests being served (already activated)
    ReqBuffer m_priority_buffer;          // Buffer for high-priority requests (refresh)
    ReqBuffer m_read_buffer;
    ReqBuffer m_write_buffer;

    std::vector<ReqBuffer> m_read_buffers;        // Per-rank read buffers
    std::vector<ReqBuffer> m_write_buffers;       // Per-rank write buffers
    std::vector<ReqBuffer> m_priority_buffers;    // Per-rank priority buffers

    size_t buf_size = 32;

    int m_bank_addr_idx = -1;
    int m_rank_addr_idx = -1;
    int m_num_rank      = -1;

    float m_wr_low_watermark;
    float m_wr_high_watermark;
    std::vector<bool> m_is_write_mode_per_rank;
    std::vector<bool> is_empty_priority_per_rank;
    std::vector<int> rr_rk_idx;   // Round-Robin rank index

    // Per-rank AsyncDIMM mode register
    std::vector<AsyncDIMMMode> m_mode_per_rank;

    // ===== Explicit Sync (H2N) Bypass + Magic Path =====
    // Callback to NMA MC: called for every command issued to DRAM in Host Mode.
    // WR/WRA commands include m_payload so NMA MC can intercept NMAInst/ctrl-reg
    // writes via DQ magic path (no separate Host→NMA software write path).
    using BypassCallback = std::function<void(int rank_id, int command,
                                              const AddrVec_t& addr_vec,
                                              const std::vector<uint64_t>& payload)>;
    BypassCallback m_bypass_to_nma = nullptr;

    // Statistics
    size_t s_row_hits = 0;
    size_t s_row_misses = 0;
    size_t s_row_conflicts = 0;
    size_t s_read_row_hits = 0;
    size_t s_read_row_misses = 0;
    size_t s_read_row_conflicts = 0;
    size_t s_write_row_hits = 0;
    size_t s_write_row_misses = 0;
    size_t s_write_row_conflicts = 0;

    size_t m_num_cores = 0;
    std::vector<size_t> s_read_row_hits_per_core;
    std::vector<size_t> s_read_row_misses_per_core;
    std::vector<size_t> s_read_row_conflicts_per_core;

    size_t s_num_read_reqs = 0;
    size_t s_num_write_reqs = 0;
    size_t s_num_other_reqs = 0;
    size_t s_queue_len = 0;
    size_t s_read_queue_len = 0;
    size_t s_write_queue_len = 0;
    size_t s_priority_queue_len = 0;
    float s_queue_len_avg = 0;
    float s_read_queue_len_avg = 0;
    float s_write_queue_len_avg = 0;
    float s_priority_queue_len_avg = 0;

    size_t s_read_latency = 0;
    float s_avg_read_latency = 0;

    float s_bandwidth = 0;
    float s_dq_bandwidth = 0;
    float s_max_bandwidth = 0;
    size_t s_num_issue_reads = 0;
    size_t s_num_issue_writes = 0;

    size_t s_num_rw_switch = 0;

    std::vector<size_t> s_num_refresh_cc_per_rank;
    size_t s_num_bypass_cmds = 0;   // Count of commands bypassed to NMA MC

    // Memory request counters
    uint64_t m_host_access_cnt = 0;
    uint64_t m_tcore_host_access_cnt = 0;
    uint64_t m_host_acceess_rec_counter = 0;
    uint64_t m_host_acceess_iss_counter = 0;
    uint64_t m_host_rd_acceess_rec_counter = 0;
    uint64_t m_host_rd_acceess_iss_counter = 0;

    // Row tracking for adaptive open-page policy
    std::vector<int> m_open_row;
    std::vector<int> m_pre_open_row;
    std::vector<bool> m_open_row_miss;
    int num_ranks = -1;
    int num_bankgroups = -1;
    int num_banks = -1;
    int rank_idx = 0;
    int bankgroup_idx = 0;
    int bank_idx = 0;
    int row_idx = 0;
    uint32_t m_adaptive_row_cap;

    // Read latency recording
    std::vector<uint64_t> m_lat_vec;

    // ===== NMA Write Tracking & Start Flag (DBX-DIMM style lifecycle) =====
    // Host MC manages NMA instruction delivery ordering:
    //   1. send() detects NMAInst writes (inst-buf address) → nma_wr_send_cnt++
    //   2. tick() detects WR issue to inst-buf address → nma_wr_issue_cnt++
    //   3. send() detects ctrl-reg write → nma_start_requested = true
    //   4. tick() waits for nma_wr_send_cnt == nma_wr_issue_cnt, then issues ctrl-reg WR
    //   5. When ctrl-reg WR is ISSUED → mode transition + bypass start to NMA MC
    int m_nma_ctrl_row = -1;  // Magic-path addresses (set by system via set_nma_addresses())
    int m_nma_ctrl_bg  = -1;
    int m_nma_ctrl_bk  = -1;
    int m_nma_buf_bg   = -1;
    int m_nma_buf_bk   = -1;
    int m_nma_row_idx  = -1;  // Address level indices
    int m_nma_bg_idx   = -1;
    int m_nma_bk_idx   = -1;

    std::vector<int> m_nma_wr_send_cnt;   // [rank] NMAInst writes enqueued via send()
    std::vector<int> m_nma_wr_issue_cnt;  // [rank] NMAInst writes issued to DRAM
    std::vector<bool> m_nma_start_requested; // [rank] ctrl-reg write received, awaiting drain

    // ===== Phase 3: Concurrent Mode =====
    // Per-bank Offload Table (OT): counts outstanding offloaded commands.
    // Prevents CMD FIFO overflow; threshold = NMA_CMD_FIFO_SIZE (8).
    static constexpr int HOST_OT_THRESHOLD = 8;
    std::vector<int> m_ot_counter;  // [total_banks flat index]
    int m_total_banks_flat = -1;

    // ===== Return Unit (RU) — Host MC side =====
    // Records offloaded memory accesses in order per rank (paper §V.A.2).
    // Read: waits for interrupt + RT data → callback to frontend
    // Write: waits for interrupt → complete (OT decrement)
    struct ReturnUnitEntry {
      Request req;           // Original request (read: has callback)
      int cmd_count;         // Total offloaded commands (1/2/3) for OT decrement
      int bank_flat_id;      // Target bank for OT decrement
      bool is_read;
    };
    std::deque<ReturnUnitEntry> m_return_unit;    // Global RU (all ranks, in offload order)
    static constexpr int RU_MAX_ENTRIES = 64;     // Paper: 64 total for 16 ranks
    std::vector<int> m_rt_pending_count;          // [rank] Accumulated read interrupts → RT batch size
    std::vector<std::deque<Request>> m_rt_read_pending; // [rank] Reads awaiting RT data return

    // Phase 3 concurrent mode flag (set by system via set_concurrent_mode_enable())
    bool m_concurrent_mode_enable = false;

    // RT command IDs: RT_N occupies N × nBL on DQ bus (DRAM model enforced)
    int m_rt_cmds[9] = {};  // m_rt_cmds[1]=RT1, ..., m_rt_cmds[8]=RT8


  public:
    void init() override {
      m_wr_low_watermark =  param<float>("wr_low_watermark").desc("Threshold for switching back to read mode.").default_val(0.2f);
      m_wr_high_watermark = param<float>("wr_high_watermark").desc("Threshold for switching to write mode.").default_val(0.8f);
      m_adaptive_row_cap = param<size_t>("adaptive_row_cap").desc("Row Buffer Hit Cap for Adaptive Open-Page Policy").default_val(16);
      m_scheduler = create_child_ifce<IScheduler>();
      m_refresh = create_child_ifce<IRefreshManager>();
      m_rowpolicy = create_child_ifce<IRowPolicy>();

      if (m_config["plugins"]) {
        YAML::Node plugin_configs = m_config["plugins"];
        for (YAML::iterator it = plugin_configs.begin(); it != plugin_configs.end(); ++it) {
          m_plugins.push_back(create_child_ifce<IControllerPlugin>(*it));
        }
      }
    };

    void setup(IFrontEnd* frontend, IMemorySystem* memory_system) override {
      m_dram = memory_system->get_ifce<IDRAM>();
      m_bank_addr_idx = m_dram->m_levels("bank");
      m_rank_addr_idx = m_dram->m_levels("rank");
      m_priority_buffer.max_size = 512 * 3 + 32;
      m_active_buffer.max_size = 32 * 4;
      m_write_buffer.max_size = 128;
      m_read_buffer.max_size = 128;

      num_ranks = m_dram->get_level_size("rank");
      num_bankgroups = m_dram->get_level_size("bankgroup");
      num_banks = m_dram->get_level_size("bank");
      rank_idx = m_dram->m_levels("rank");
      bankgroup_idx = m_dram->m_levels("bankgroup");
      bank_idx = m_dram->m_levels("bank");
      row_idx = m_dram->m_levels("row");

      m_num_cores = frontend->get_num_cores();

      s_read_row_hits_per_core.resize(m_num_cores, 0);
      s_read_row_misses_per_core.resize(m_num_cores, 0);
      s_read_row_conflicts_per_core.resize(m_num_cores, 0);

      register_stat(s_row_hits).name("row_hits_{}", m_channel_id);
      register_stat(s_row_misses).name("row_misses_{}", m_channel_id);
      register_stat(s_row_conflicts).name("row_conflicts_{}", m_channel_id);
      register_stat(s_read_row_hits).name("read_row_hits_{}", m_channel_id);
      register_stat(s_read_row_misses).name("read_row_misses_{}", m_channel_id);
      register_stat(s_read_row_conflicts).name("read_row_conflicts_{}", m_channel_id);
      register_stat(s_write_row_hits).name("write_row_hits_{}", m_channel_id);
      register_stat(s_write_row_misses).name("write_row_misses_{}", m_channel_id);
      register_stat(s_write_row_conflicts).name("write_row_conflicts_{}", m_channel_id);

      for (size_t core_id = 0; core_id < m_num_cores; core_id++) {
        register_stat(s_read_row_hits_per_core[core_id]).name("read_row_hits_core_{}", core_id);
        register_stat(s_read_row_misses_per_core[core_id]).name("read_row_misses_core_{}", core_id);
        register_stat(s_read_row_conflicts_per_core[core_id]).name("read_row_conflicts_core_{}", core_id);
      }

      register_stat(s_num_read_reqs).name("num_read_reqs_{}", m_channel_id);
      register_stat(s_num_write_reqs).name("num_write_reqs_{}", m_channel_id);
      register_stat(s_num_other_reqs).name("num_other_reqs_{}", m_channel_id);
      register_stat(s_queue_len).name("queue_len_{}", m_channel_id);
      register_stat(s_read_queue_len).name("read_queue_len_{}", m_channel_id);
      register_stat(s_write_queue_len).name("write_queue_len_{}", m_channel_id);
      register_stat(s_priority_queue_len).name("priority_queue_len_{}", m_channel_id);
      register_stat(s_queue_len_avg).name("queue_len_avg_{}", m_channel_id);
      register_stat(s_read_queue_len_avg).name("read_queue_len_avg_{}", m_channel_id);
      register_stat(s_write_queue_len_avg).name("write_queue_len_avg_{}", m_channel_id);
      register_stat(s_priority_queue_len_avg).name("priority_queue_len_avg_{}", m_channel_id);

      register_stat(s_read_latency).name("read_latency_{}", m_channel_id);
      register_stat(s_avg_read_latency).name("avg_read_latency_{}", m_channel_id);

      register_stat(s_bandwidth).name("rw_bandwidth_{}", m_channel_id);
      register_stat(s_num_issue_reads).name("num_issue_reads_{}", m_channel_id);
      register_stat(s_num_issue_writes).name("num_issue_writes_{}", m_channel_id);
      register_stat(s_dq_bandwidth).name("dq_bandwidth_{}", m_channel_id);
      register_stat(s_max_bandwidth).name("max_bandwidth_{}", m_channel_id);

      register_stat(s_num_rw_switch).name("s_num_rw_switch_{}", m_channel_id);
      register_stat(s_num_bypass_cmds).name("s_num_bypass_cmds_{}", m_channel_id);

      m_num_rank = m_dram->get_level_size("rank");

      // Initialize per-rank request buffers
      m_read_buffers.resize(m_num_rank, ReqBuffer());
      m_write_buffers.resize(m_num_rank, ReqBuffer());
      m_priority_buffers.resize(m_num_rank, ReqBuffer());

      m_is_write_mode_per_rank.resize(m_num_rank, false);
      is_empty_priority_per_rank.resize(m_num_rank, false);
      s_num_refresh_cc_per_rank.resize(m_num_rank, 0);
      for (int i = 0; i < m_num_rank; i++) {
        m_read_buffers[i].max_size = buf_size;
        m_write_buffers[i].max_size = buf_size;
        m_priority_buffers[i].max_size = (512 * 3 + 32) / 4;
        rr_rk_idx.push_back(i);
        register_stat(s_num_refresh_cc_per_rank[i]).name("s_num_refresh_cc_per_rank_{}_{}", m_channel_id, i);
      }

      // Initialize per-rank mode register (Host Mode only in Phase 1)
      m_mode_per_rank.resize(m_num_rank, AsyncDIMMMode::HOST);

      // NMA write tracking
      m_nma_wr_send_cnt.resize(num_ranks, 0);
      m_nma_wr_issue_cnt.resize(num_ranks, 0);
      m_nma_start_requested.resize(num_ranks, false);
      m_nma_row_idx = m_dram->m_levels("row");
      m_nma_bg_idx  = m_dram->m_levels("bankgroup");
      m_nma_bk_idx  = m_dram->m_levels("bank");

      // Phase 3: OT counter and Return Unit
      m_total_banks_flat = num_ranks * num_bankgroups * num_banks;
      m_ot_counter.resize(m_total_banks_flat, 0);
      m_rt_pending_count.resize(num_ranks, 0);
      m_rt_read_pending.resize(num_ranks);
      // Cache RT1~RT8 command IDs for batch return
      for (int i = 1; i <= 8; i++)
        m_rt_cmds[i] = m_dram->m_commands("RT" + std::to_string(i));

      // Row tracking
      m_open_row.resize(num_ranks * num_bankgroups * num_banks, -1);
      m_pre_open_row.resize(num_ranks * num_bankgroups * num_banks, -1);
      m_open_row_miss.resize(num_ranks * num_bankgroups * num_banks, false);
    };

    // ===== Public API for AsyncDIMM Memory System =====

    // Set bypass callback (called by asyncdimm_system to wire NMA MC)
    void set_bypass_callback(BypassCallback cb) {
      m_bypass_to_nma = cb;
    }

    void set_concurrent_mode_enable(bool enable) { m_concurrent_mode_enable = enable; }

    /**
     * Check if any RU entries or RT-pending reads exist for a rank.
     * Used by System for C2H transition: must drain all offloaded accesses first.
     */
    bool has_pending_offloads_for_rank(int rank_id) const {
      // Check RU for entries targeting this rank
      for (const auto& entry : m_return_unit) {
        if (entry.req.addr_vec[m_rank_addr_idx] == rank_id) return true;
      }
      // Check RT-pending reads for this rank
      if (rank_id >= 0 && rank_id < m_num_rank) {
        if (!m_rt_read_pending[rank_id].empty()) return true;
        if (m_rt_pending_count[rank_id] > 0) return true;
      }
      return false;
    }

    // Set magic-path addresses (called by AsyncDIMMSystem during init)
    void set_nma_addresses(int ctrl_row, int ctrl_bg, int ctrl_bk, int buf_bg, int buf_bk) {
      m_nma_ctrl_row = ctrl_row;
      m_nma_ctrl_bg  = ctrl_bg;
      m_nma_ctrl_bk  = ctrl_bk;
      m_nma_buf_bg   = buf_bg;
      m_nma_buf_bk   = buf_bk;
    }

    // ===== Phase 3: Concurrent Mode API =====

    /**
     * Called by AsyncDIMMSystem when NMA MC fires TDM interrupt.
     * batch_size = number of offloaded reads that completed.
     * Accumulates interrupt batch; RT issuance handled in tick().
     */
    /**
     * Called by AsyncDIMMSystem when NMA MC fires TDM interrupt.
     * Always batch_size=1 (TDM: 1 interrupt per slot = 1 access completion).
     * Paper §V.A.2:
     *   Write: deleted from RU on interrupt → complete
     *   Read: deleted from RU on interrupt → awaits RT for data return
     */
    void on_nma_interrupt(int rank_id, int batch_size) {
      if (rank_id < 0 || rank_id >= m_num_rank) return;
      if (m_return_unit.empty()) return;

      auto& entry = m_return_unit.front();

      // OT decrement: cmd_count commands completed for this bank
      if (entry.bank_flat_id >= 0 && entry.bank_flat_id < m_total_banks_flat)
        m_ot_counter[entry.bank_flat_id] -= entry.cmd_count;

      if (entry.is_read) {
        // Read: interrupt received. Data not yet returned.
        // Store req in rt_read_pending → Host MC issues RT → data arrives → callback
        m_rt_read_pending[rank_id].push_back(entry.req);
        m_rt_pending_count[rank_id]++;
      }
      // Write: complete on interrupt (no data return, no frontend callback)

      m_return_unit.pop_front();
    }

    // Get open row for a specific bank (for FSM sync verification)
    // Returns -1 if closed, else the open row id
    int get_open_row(int rank, int bankgroup, int bank) const {
      int flat_bank_id = bank + bankgroup * num_banks + rank * num_bankgroups * num_banks;
      return m_open_row[flat_bank_id];
    }

    // Get current mode for a rank
    AsyncDIMMMode get_mode(int rank_id) const {
      return m_mode_per_rank[rank_id];
    }

    // Set mode for a rank (used by memory system for mode transitions)
    void set_mode(int rank_id, AsyncDIMMMode mode) {
      m_mode_per_rank[rank_id] = mode;
    }

    bool send(Request& req) override {
      bool is_success = false;
      bool is_success_forwarding = false;
      req.final_command = m_dram->m_request_translations(req.type_id);

      // During NMA Mode, reject host requests for that rank
      int req_rank_id = req.addr_vec[m_rank_addr_idx];
      if (req_rank_id >= 0 && req_rank_id < m_num_rank &&
          m_mode_per_rank[req_rank_id] == AsyncDIMMMode::NMA) {
        return false;  // Host access blocked during NMA Mode
      }

      // ===== NMA Write Tracking (DBX-DIMM style) =====
      // Detect NMAInst writes and ctrl-reg writes at enqueue time.
      if (req.type_id == Request::Type::Write && m_nma_ctrl_row >= 0 &&
          req_rank_id >= 0 && req_rank_id < m_num_rank) {
        int req_row = req.addr_vec[m_nma_row_idx];
        int req_bg  = req.addr_vec[m_nma_bg_idx];
        int req_bk  = req.addr_vec[m_nma_bk_idx];
        if (req_row == m_nma_ctrl_row) {
          if (req_bg == m_nma_buf_bg && req_bk == m_nma_buf_bk) {
            // NMAInst write → track send count
            m_nma_wr_send_cnt[req_rank_id]++;
            req.is_ndp_req = true;  // Flag for issue-time tracking
          }
          else if (req_bg == m_nma_ctrl_bg && req_bk == m_nma_ctrl_bk) {
            // Ctrl-reg (start) write → register start request
            // Host MC will issue this AFTER all NMAInst writes are drained
            m_nma_start_requested[req_rank_id] = true;
            req.is_ndp_req = true;  // Flag for issue-time tracking
          }
        }
      }

      // Forward existing write requests to incoming read requests
      // NOTE: GenericDRAMController has a bug here — it searches m_write_buffer (singular,
      // always empty) instead of per-rank m_write_buffers[rank]. We intentionally match
      // the Generic behavior (no forwarding) for behavioral equivalence in Host Mode.
      // TODO: Fix this in Phase 2+ if write forwarding is desired.
      if (req.type_id == Request::Type::Read) {
        auto compare_addr = [req](const Request& wreq) {
          return wreq.addr == req.addr;
        };
        ReqBuffer& wr_buffer = m_write_buffers[req.addr_vec[m_rank_addr_idx]];
        if (std::find_if(m_write_buffer.begin(), m_write_buffer.end(), compare_addr) != m_write_buffer.end()) {
          req.arrive = m_clk;
          req.depart = m_clk + 1;
          pending.push_back(req);
          is_success_forwarding = true;
        }
      }

      // Enqueue to corresponding buffer
      if (!is_success_forwarding) {
        req.arrive = m_clk;
        if (req.type_id == Request::Type::Read) {
          is_success = m_read_buffers[req.addr_vec[m_rank_addr_idx]].enqueue(req);
        } else if (req.type_id == Request::Type::Write) {
          is_success = m_write_buffers[req.addr_vec[m_rank_addr_idx]].enqueue(req);
        } else {
          throw std::runtime_error("Invalid request type!");
        }
      }

      if (is_success || is_success_forwarding) {
        switch (req.type_id) {
          case Request::Type::Read:  s_num_read_reqs++;  break;
          case Request::Type::Write: s_num_write_reqs++; break;
          default:                   s_num_other_reqs++; break;
        }
      }

      if (is_success && req.is_host_req) {
        m_host_acceess_rec_counter++;
        if (req.type_id == Request::Type::Read) {
          m_host_rd_acceess_rec_counter++;
        }
      }

      return is_success || is_success_forwarding;
    };

    bool priority_send(Request& req) override {
      req.final_command = m_dram->m_request_translations(req.type_id);

      // All modes: enqueue refresh normally. For CONCURRENT/NMA ranks,
      // schedule_request() converts REFab → REFO (offloaded to NMA MC).
      int rank_id = req.addr_vec[m_rank_addr_idx];
      bool is_success = m_priority_buffers[rank_id].enqueue(req);
      return is_success;
    }

    void tick() override {
      m_clk++;

      // Update queue length statistics
      s_queue_len += m_read_buffer.size() + m_write_buffer.size() + m_priority_buffer.size() + pending.size();
      s_read_queue_len += m_read_buffer.size() + pending.size();
      s_write_queue_len += m_write_buffer.size();
      s_priority_queue_len += m_priority_buffer.size();

      // Update row cap for adaptive open-page policy
      for (int rk_id = 0; rk_id < num_ranks; rk_id++) {
        auto rd_buffer = m_read_buffers[rk_id];
        for (auto next = std::next(rd_buffer.begin(), 1); next != rd_buffer.end(); next++) {
          int flat_bank_id = next->addr_vec[bank_idx] + next->addr_vec[bankgroup_idx] * num_banks + next->addr_vec[rank_idx] * num_bankgroups * num_banks;
          if ((m_open_row[flat_bank_id] != -1) && (m_open_row[flat_bank_id] != next->addr_vec[row_idx])) {
            m_open_row_miss[flat_bank_id] = true;
            m_scheduler->update_open_row_miss(flat_bank_id, true);
            m_rowpolicy->update_cap(0, next->addr_vec[rank_idx], next->addr_vec[bankgroup_idx], next->addr_vec[bank_idx], m_adaptive_row_cap);
          }
        }
        auto wr_buffer = m_write_buffers[rk_id];
        for (auto next = std::next(wr_buffer.begin(), 1); next != wr_buffer.end(); next++) {
          int flat_bank_id = next->addr_vec[bank_idx] + next->addr_vec[bankgroup_idx] * num_banks + next->addr_vec[rank_idx] * num_bankgroups * num_banks;
          if ((m_open_row[flat_bank_id] != -1) && (m_open_row[flat_bank_id] != next->addr_vec[row_idx])) {
            m_open_row_miss[flat_bank_id] = true;
            m_scheduler->update_open_row_miss(flat_bank_id, true);
            m_rowpolicy->update_cap(0, next->addr_vec[rank_idx], next->addr_vec[bankgroup_idx], next->addr_vec[bank_idx], m_adaptive_row_cap);
          }
        }
      }

      // 1. Serve completed reads (including RT-returned offloaded reads)
      serve_completed_reads();

      // 1.5. RT issuance — highest priority (paper §V.A.1)
      //   RT_N = 1 command, batch=N. DQ bus occupied N × nBL (DRAM model enforced).
      //   If RT issued this tick, skip all subsequent scheduling (CA bus occupied).
      bool rt_issued = false;
      for (int rk = 0; rk < m_num_rank; rk++) {
        if (m_rt_pending_count[rk] <= 0) continue;
        int batch = std::min(m_rt_pending_count[rk], 8);  // Max batch = 8
        int rt_cmd = m_rt_cmds[batch];
        AddrVec_t rt_addr(m_dram->m_levels.size(), -1);
        rt_addr[m_dram->m_levels("channel")] = m_channel_id;
        rt_addr[m_dram->m_levels("rank")]    = rk;
        if (m_dram->check_ready(rt_cmd, rt_addr)) {
          m_dram->issue_command(rt_cmd, rt_addr);  // 1 command, DQ = batch × nBL
          int nBL = m_dram->m_timing_vals("nBL");
          int nCL = m_dram->m_timing_vals("nCL");
          for (int i = 0; i < batch && !m_rt_read_pending[rk].empty(); i++) {
            auto& req = m_rt_read_pending[rk].front();
            req.depart = m_clk + nCL + i * nBL;
            pending.push_back(req);
            m_rt_read_pending[rk].pop_front();
          }
          m_rt_pending_count[rk] -= batch;
          rt_issued = true;
          break;  // 1 RT per tick
        }
      }

      // 2. Tick refresh manager (sends REF via priority_send)
      m_refresh->tick();

      // 3. Skip scheduling if RT was issued (CA bus occupied by RT this tick)
      if (rt_issued) return;
      ReqBuffer::iterator req_it;
      ReqBuffer* buffer = nullptr;
      bool request_found = schedule_request(req_it, buffer);

      // 4. Take row policy action
      m_rowpolicy->update(request_found, req_it);

      // 5. Update all plugins
      for (auto plugin : m_plugins) {
        plugin->update(request_found, req_it);
      }

      // 6. Pre-issue checks
      if (request_found) {
        int req_rank = req_it->addr_vec[m_rank_addr_idx];

        // NMA start gate: hold ctrl-reg WR until all NMAInst writes are issued
        if (req_it->is_ndp_req && req_rank >= 0 && req_rank < m_num_rank) {
          int rr = req_it->addr_vec[m_nma_row_idx];
          int rb = req_it->addr_vec[m_nma_bg_idx];
          int rk = req_it->addr_vec[m_nma_bk_idx];
          if (rr == m_nma_ctrl_row && rb == m_nma_ctrl_bg && rk == m_nma_ctrl_bk) {
            if (m_nma_wr_send_cnt[req_rank] != m_nma_wr_issue_cnt[req_rank])
              request_found = false;
          }
        }

        // NMA mode: only allow offload commands (REFO), block real commands
        if (request_found && req_rank >= 0 && req_rank < m_num_rank &&
            m_mode_per_rank[req_rank] == AsyncDIMMMode::NMA) {
          int cmd = req_it->command;
          if (cmd != m_dram->m_commands("REFO"))
            request_found = false;
        }
      }

      // 7. Unified issue: handles both real commands and offload commands
      if (request_found) {
        if (!req_it->is_stat_updated) update_request_stats(req_it);

        int cmd = req_it->command;
        int cmd_rank_id = req_it->addr_vec[m_rank_addr_idx];
        int flat_bank_id = req_it->addr_vec[bank_idx]
                         + req_it->addr_vec[bankgroup_idx] * num_banks
                         + req_it->addr_vec[rank_idx] * num_bankgroups * num_banks;
        bool is_offload = (cmd == m_dram->m_commands("ACTO") || cmd == m_dram->m_commands("PREO") ||
                           cmd == m_dram->m_commands("RDO")  || cmd == m_dram->m_commands("WRO")  ||
                           cmd == m_dram->m_commands("REFO"));

        // Issue command to DRAM model
        m_dram->issue_command(cmd, req_it->addr_vec);

        // NMA write tracking: inst-buf WR → issue count; ctrl-reg WR → mode transition
        if (req_it->is_ndp_req && (cmd == req_it->final_command) &&
            cmd_rank_id >= 0 && cmd_rank_id < m_num_rank) {
          int rr = req_it->addr_vec[m_nma_row_idx];
          int rb = req_it->addr_vec[m_nma_bg_idx];
          int rk = req_it->addr_vec[m_nma_bk_idx];
          if (rr == m_nma_ctrl_row && rb == m_nma_buf_bg && rk == m_nma_buf_bk)
            m_nma_wr_issue_cnt[cmd_rank_id]++;
          else if (rr == m_nma_ctrl_row && rb == m_nma_ctrl_bg && rk == m_nma_ctrl_bk) {
            m_mode_per_rank[cmd_rank_id] = m_concurrent_mode_enable
              ? AsyncDIMMMode::CONCURRENT : AsyncDIMMMode::NMA;
            m_nma_start_requested[cmd_rank_id] = false;
          }
        }

        // Bypass to NMA MC (unified: real cmds for FSM sync, offload for CMD FIFO/REFO)
        if (m_bypass_to_nma && (is_offload || m_mode_per_rank[cmd_rank_id] != AsyncDIMMMode::NMA)) {
          static const std::vector<uint64_t> s_empty_payload{};
          const auto& payload =
            (cmd == m_dram->m_commands("WR") || cmd == m_dram->m_commands("WRA") ||
             cmd == m_dram->m_commands("WRO"))
            ? req_it->m_payload : s_empty_payload;
          m_bypass_to_nma(cmd_rank_id, cmd, req_it->addr_vec, payload);
          s_num_bypass_cmds++;
        }

        // Row tracking (unified: ACT/ACTO open, PRE/PREO close)
        if (cmd == m_dram->m_commands("ACT") || cmd == m_dram->m_commands("ACTO")) {
          m_open_row_miss[flat_bank_id] = false;
          m_open_row[flat_bank_id] = req_it->addr_vec[row_idx];
          m_pre_open_row[flat_bank_id] = -1;
          if (!is_offload) {
            m_scheduler->update_open_row_miss(flat_bank_id, false);
            m_scheduler->update_open_row(flat_bank_id, req_it->addr_vec[row_idx]);
            m_scheduler->update_pre_open_row(flat_bank_id, -1);
            m_scheduler->update_bk_status(flat_bank_id, false);
            m_rowpolicy->update_cap(0, req_it->addr_vec[rank_idx], req_it->addr_vec[bankgroup_idx], req_it->addr_vec[bank_idx], 128);
          }
        } else if (cmd == m_dram->m_commands("PRE") || cmd == m_dram->m_commands("PREO")) {
          m_pre_open_row[flat_bank_id] = m_open_row[flat_bank_id];
          m_open_row[flat_bank_id] = -1;
          if (!is_offload) {
            m_scheduler->update_open_row(flat_bank_id, -1);
            m_scheduler->update_pre_open_row(flat_bank_id, m_pre_open_row[flat_bank_id]);
            m_scheduler->update_bk_status(flat_bank_id, true);
          }
        } else if (cmd == m_dram->m_commands("PREsb")) {
          int rk_id = req_it->addr_vec[rank_idx]; int t_bk = req_it->addr_vec[bank_idx];
          for (int bg = 0; bg < num_bankgroups; bg++) {
            int fid = t_bk + bg * num_banks + rk_id * num_bankgroups * num_banks;
            m_pre_open_row[fid] = m_open_row[fid]; m_open_row[fid] = -1;
            m_scheduler->update_open_row(fid, -1);
            m_scheduler->update_pre_open_row(fid, m_pre_open_row[fid]);
            m_scheduler->update_bk_status(fid, true);
          }
        } else if (cmd == m_dram->m_commands("PREA")) {
          int rk_id = req_it->addr_vec[rank_idx];
          for (int bg = 0; bg < num_bankgroups; bg++)
            for (int bk = 0; bk < num_banks; bk++) {
              int fid = bk + bg * num_banks + rk_id * num_bankgroups * num_banks;
              m_pre_open_row[fid] = m_open_row[fid]; m_open_row[fid] = -1;
              m_scheduler->update_open_row(fid, -1);
              m_scheduler->update_pre_open_row(fid, m_pre_open_row[fid]);
              m_scheduler->update_bk_status(fid, true);
            }
        }

        // OT tracking: every offload command increments OT (except REFO)
        if (is_offload && cmd != m_dram->m_commands("REFO")) {
          if (flat_bank_id >= 0 && flat_bank_id < m_total_banks_flat)
            m_ot_counter[flat_bank_id]++;
          // Per-request cmd_count tracking via scratchpad[0]
          req_it->scratchpad[0]++;
        }

        // Stats
        if (cmd == m_dram->m_commands("RD") || cmd == m_dram->m_commands("RDA") ||
            cmd == m_dram->m_commands("RDO")) s_num_issue_reads++;
        if (cmd == m_dram->m_commands("WR") || cmd == m_dram->m_commands("WRA") ||
            cmd == m_dram->m_commands("WRO")) s_num_issue_writes++;
        if (cmd == m_dram->m_commands("REFab"))
          s_num_refresh_cc_per_rank[req_it->addr_vec[m_rank_addr_idx]] += m_dram->m_timing_vals("nRFC1");

        // Request lifecycle
        bool is_final_real    = (cmd == req_it->final_command);
        bool is_final_offload = (cmd == m_dram->m_commands("RDO") || cmd == m_dram->m_commands("WRO") ||
                                 cmd == m_dram->m_commands("REFO"));

        if (is_final_offload) {
          // Offload final (RDO/WRO/REFO): register in Return Unit, NOT in pending.
          // Completion happens when NMA MC fires interrupt (+ RT for reads).
          if (cmd != m_dram->m_commands("REFO")) {
            bool is_read = (req_it->type_id == Request::Type::Read);
            m_return_unit.push_back({*req_it, req_it->scratchpad[0], flat_bank_id, is_read});
          }
          // REFO: no RU entry needed (refresh has no frontend callback)
          m_host_access_cnt++;
          if (req_it->is_trace_core_req) m_tcore_host_access_cnt++;
          if (req_it->is_host_req) {
            m_host_acceess_iss_counter++;
            if (req_it->type_id == Request::Type::Read) m_host_rd_acceess_iss_counter++;
          }
          buffer->remove(req_it);
        } else if (is_final_real) {
          // Real final (RD/WR for HOST mode): complete immediately with fixed latency
          if (req_it->type_id == Request::Type::Read) {
            req_it->depart = m_clk + m_dram->m_read_latency;
            pending.push_back(*req_it);
          } else {
            // WR completes immediately — count as received
            if (req_it->is_host_req) m_host_acceess_rec_counter++;
          }
          m_host_access_cnt++;
          if (req_it->is_trace_core_req) m_tcore_host_access_cnt++;
          if (req_it->is_host_req) {
            m_host_acceess_iss_counter++;
            if (req_it->type_id == Request::Type::Read) m_host_rd_acceess_iss_counter++;
          }
          buffer->remove(req_it);
        } else if (!is_offload && m_dram->m_command_meta(cmd).is_opening) {
          // Real ACT → active_buffer (ACTO has is_opening=false → stays in buffer)
          req_it->is_actived = true;
          if (!m_active_buffer.enqueue(*req_it))
            throw std::runtime_error("Fail to enque to m_active_buffer");
          buffer->remove(req_it);
        }
        // ACTO/PREO: request stays in buffer → next tick m_open_row hit → RDO/WRO
      }
    };


  private:
    // ===== Phase 3: Concurrent Mode Offload Scheduling =====

    /**
     * Select best offload request for a CONCURRENT-mode rank.
     * Uses m_open_row[] for prerequisite, OT for backpressure, RU capacity check.
     * Sets req_it->command to ACTO/PREO/RDO/WRO. Does NOT issue.
     * Two-pass: row hits first (FRFCFS-like).
     */
    bool schedule_offload_for_rank(int rk_idx, ReqBuffer::iterator& req_it, ReqBuffer*& req_buffer) {
      // RU capacity check: if global RU is full, block ALL offload commands
      // (ACTO/PREO would waste OT slots since RDO/WRO can't follow until RU drains)
      if ((int)m_return_unit.size() >= RU_MAX_ENTRIES) return false;
      bool ru_full = false;  // Already checked above
      set_write_mode_per_rank(rk_idx);

      ReqBuffer* buffers[2];
      if (m_is_write_mode_per_rank[rk_idx]) {
        buffers[0] = &m_write_buffers[rk_idx];
        buffers[1] = &m_read_buffers[rk_idx];
      } else {
        buffers[0] = &m_read_buffers[rk_idx];
        buffers[1] = &m_write_buffers[rk_idx];
      }

      for (int bi = 0; bi < 2; bi++) {
        auto& buffer = *buffers[bi];
        if (buffer.size() == 0) continue;

        // Two-pass: row hits (RDO/WRO) first, then misses (ACTO/PREO)
        for (int pass = 0; pass < 2; pass++) {
          for (auto it = buffer.begin(); it != buffer.end(); it++) {
            int flat_bank_id = it->addr_vec[bank_idx]
                             + it->addr_vec[bankgroup_idx] * num_banks
                             + it->addr_vec[rank_idx] * num_bankgroups * num_banks;

            // OT backpressure
            if (flat_bank_id >= 0 && flat_bank_id < m_total_banks_flat &&
                m_ot_counter[flat_bank_id] >= HOST_OT_THRESHOLD) continue;

            int target_row = it->addr_vec[row_idx];
            bool is_read   = (it->type_id == Request::Type::Read);
            int offload_cmd = -1;

            if (m_open_row[flat_bank_id] == -1) {
              if (pass == 1) offload_cmd = m_dram->m_commands("ACTO");
            } else if (m_open_row[flat_bank_id] != target_row) {
              if (pass == 1) offload_cmd = m_dram->m_commands("PREO");
            } else {
              // Row hit → RDO/WRO (final command → needs RU entry)
              if (pass == 0) {
                if (ru_full) continue;  // RU full: can't issue final command
                offload_cmd = is_read ? m_dram->m_commands("RDO")
                                      : m_dram->m_commands("WRO");
              }
            }

            if (offload_cmd < 0) continue;
            if (!m_dram->check_ready(offload_cmd, it->addr_vec)) continue;

            // Found: set command and return (tick() will issue)
            it->command = offload_cmd;
            req_it = it;
            req_buffer = &buffer;
            return true;
          }
        }
      }
      return false;
    }

    bool is_row_hit(ReqBuffer::iterator& req) {
      return m_dram->check_rowbuffer_hit(req->final_command, req->addr_vec);
    }

    bool is_row_open(ReqBuffer::iterator& req) {
      return m_dram->check_node_open(req->final_command, req->addr_vec);
    }

    void update_request_stats(ReqBuffer::iterator& req) {
      req->is_stat_updated = true;

      if (req->type_id == Request::Type::Read) {
        if (is_row_hit(req)) {
          s_read_row_hits++;
          s_row_hits++;
          if (req->source_id != -1)
            s_read_row_hits_per_core[req->source_id]++;
        } else if (is_row_open(req)) {
          s_read_row_conflicts++;
          s_row_conflicts++;
          if (req->source_id != -1)
            s_read_row_conflicts_per_core[req->source_id]++;
        } else {
          s_read_row_misses++;
          s_row_misses++;
          if (req->source_id != -1)
            s_read_row_misses_per_core[req->source_id]++;
        }
      } else if (req->type_id == Request::Type::Write) {
        if (is_row_hit(req)) {
          s_write_row_hits++;
          s_row_hits++;
        } else if (is_row_open(req)) {
          s_write_row_conflicts++;
          s_row_conflicts++;
        } else {
          s_write_row_misses++;
          s_row_misses++;
        }
      }
    }

    void serve_completed_reads() {
      if (pending.size()) {
        auto& req = pending[0];
        if (req.depart <= m_clk) {
          uint64_t a_read_latency = req.depart - req.arrive;
          if (a_read_latency > 1) {
            s_read_latency += a_read_latency;
            if (req.is_host_req) {
              m_lat_vec.push_back(a_read_latency);
            }
          }

          if (req.callback) {
            req.callback(req);
          }
          pending.pop_front();
        }
      };
    };

    void set_write_mode_per_rank(int rk_idx) {
      if (!m_is_write_mode_per_rank[rk_idx]) {
        if ((m_write_buffers[rk_idx].size() > m_wr_high_watermark * m_write_buffers[rk_idx].max_size) || m_read_buffers[rk_idx].size() == 0) {
          m_is_write_mode_per_rank[rk_idx] = true;
          s_num_rw_switch++;
        }
      } else {
        if ((m_write_buffers[rk_idx].size() < m_wr_low_watermark * m_write_buffers[rk_idx].max_size) && m_read_buffers[rk_idx].size() != 0) {
          m_is_write_mode_per_rank[rk_idx] = false;
          s_num_rw_switch++;
        }
      }
    };

    void update_rr_rank_idx() {
      rr_rk_idx.push_back(rr_rk_idx[0]);
      rr_rk_idx.erase(rr_rk_idx.begin());
    }

    /**
     * schedule_request(): Select the best request to serve.
     * Sets req_it->command to the appropriate command (real or offload).
     * Does NOT issue or bypass — tick() handles that uniformly.
     *
     * For CONCURRENT/NMA ranks:
     *   active_buffer: RD→RDO, WR→WRO (bank already open from pre-transition ACT)
     *   priority_buffer: REFab→REFO
     *   read/write_buffer: m_open_row[] based → ACTO/PREO/RDO/WRO (OT checked)
     */
    bool schedule_request(ReqBuffer::iterator& req_it, ReqBuffer*& req_buffer) {
      bool request_found = false;

      // Priority queue empty flag array
      for (int rk_idx = 0; rk_idx < m_num_rank; rk_idx++) {
        is_empty_priority_per_rank[rk_idx] = (m_priority_buffers[rk_idx].size() == 0);
      }

      // 1. Active buffer (highest priority — avoid useless ACTs)
      if (req_it = m_scheduler->get_best_request(m_active_buffer); req_it != m_active_buffer.end()) {
        int ab_rk = req_it->addr_vec[m_rank_addr_idx];
        if (ab_rk >= 0 && ab_rk < m_num_rank &&
            m_mode_per_rank[ab_rk] != AsyncDIMMMode::HOST) {
          // CONCURRENT/NMA: convert RD/WR → RDO/WRO (needs RU entry)
          if ((int)m_return_unit.size() >= RU_MAX_ENTRIES) {
            // RU full: can't create RU entry, skip
          } else {  // RU has space
          bool is_read = (req_it->type_id == Request::Type::Read);
          int offload_cmd = is_read ? m_dram->m_commands("RDO") : m_dram->m_commands("WRO");
          if (m_dram->check_ready(offload_cmd, req_it->addr_vec)) {
            req_it->command = offload_cmd;
            request_found = true;
            req_buffer = &m_active_buffer;
          }
          } // end RU capacity check
        } else {
          // HOST: normal scheduling
          if (m_dram->check_ready(req_it->command, req_it->addr_vec)) {
            request_found = true;
            req_buffer = &m_active_buffer;
          }
        }
      }

      // 2. Priority buffer (refresh) — round-robin across ranks
      if (!request_found) {
        for (auto rk_idx : rr_rk_idx) {
          if (m_priority_buffers[rk_idx].size() == 0) continue;

          if (m_mode_per_rank[rk_idx] != AsyncDIMMMode::HOST) {
            // CONCURRENT/NMA: convert REFab → REFO
            req_it = m_priority_buffers[rk_idx].begin();
            int refo_cmd = m_dram->m_commands("REFO");
            if (m_dram->check_ready(refo_cmd, req_it->addr_vec)) {
              req_it->command = refo_cmd;
              request_found = true;
              req_buffer = &m_priority_buffers[rk_idx];
              break;
            }
          } else {
            // HOST: normal priority scheduling
            req_buffer = &m_priority_buffers[rk_idx];
            req_it = m_priority_buffers[rk_idx].begin();
            req_it->command = m_dram->get_preq_command(req_it->final_command, req_it->addr_vec);
            request_found = m_dram->check_ready(req_it->command, req_it->addr_vec);
            if (request_found) break;
          }
        }
      }

      // 3. Read/write buffers — round-robin across ranks (HOST + CONCURRENT interleaved)
      if (!request_found) {
        for (auto rk_idx : rr_rk_idx) {
          if (m_mode_per_rank[rk_idx] == AsyncDIMMMode::NMA) continue;

          if (m_mode_per_rank[rk_idx] == AsyncDIMMMode::CONCURRENT) {
            // Offload: use m_open_row[] for prerequisite, check OT
            if (schedule_offload_for_rank(rk_idx, req_it, req_buffer)) {
              request_found = true;
              break;
            }
          } else {
            // HOST: normal FRFCFS scheduling
            set_write_mode_per_rank(rk_idx);
            if (is_empty_priority_per_rank[rk_idx]) {
              auto& buffer = m_is_write_mode_per_rank[rk_idx] ? m_write_buffers[rk_idx] : m_read_buffers[rk_idx];
              if (req_it = m_scheduler->get_best_request(buffer); req_it != buffer.end()) {
                request_found = m_dram->check_ready(req_it->command, req_it->addr_vec);
                req_buffer = &buffer;
              }
              if (request_found) break;
            }
          }
        }
      }

      // 3. Check if the scheduled command would close an open row in the active buffer
      if (request_found) {
        if (m_dram->m_command_meta(req_it->command).is_closing) {
          auto& rowgroup = req_it->addr_vec;
          for (auto _it = m_active_buffer.begin(); _it != m_active_buffer.end(); _it++) {
            auto& _it_rowgroup = _it->addr_vec;
            bool is_matching = true;
            for (int i = 0; i < m_bank_addr_idx + 1; i++) {
              if (_it_rowgroup[i] != rowgroup[i] && _it_rowgroup[i] != -1 && rowgroup[i] != -1) {
                is_matching = false;
                break;
              }
            }
            if (is_matching) {
              request_found = false;
              break;
            }
          }
        }
      }

      if (request_found) update_rr_rank_idx();
      return request_found;
    }

    void finalize() override {
      s_avg_read_latency = (float)s_read_latency / (float)s_num_read_reqs;

      s_queue_len_avg = (float)s_queue_len / (float)m_clk;
      s_read_queue_len_avg = (float)s_read_queue_len / (float)m_clk;
      s_write_queue_len_avg = (float)s_write_queue_len / (float)m_clk;
      s_priority_queue_len_avg = (float)s_priority_queue_len / (float)m_clk;

      int tx_bytes = m_dram->m_internal_prefetch_size * m_dram->m_channel_width / 8;
      s_bandwidth = ((float)((s_num_read_reqs + s_num_write_reqs) * tx_bytes) / (float)(m_clk * m_dram->m_timing_vals("tCK_ps"))) * 1E12 / (1024 * 1024 * 1012);
      s_dq_bandwidth = ((float)((s_num_issue_reads + s_num_issue_writes) * tx_bytes) / (float)(m_clk * m_dram->m_timing_vals("tCK_ps"))) * 1E12 / (1024 * 1024 * 1012);
      s_max_bandwidth = (float)(m_dram->m_channel_width / 8) * (float)m_dram->m_timing_vals("rate") * 1E6 / (1024 * 1024 * 1012);
      return;
    }

    bool is_finished() override {
      return (m_host_rd_acceess_rec_counter == m_host_rd_acceess_iss_counter);
    }

    bool is_abs_finished() override {
      return (m_host_acceess_rec_counter == m_host_acceess_iss_counter);
    }

    uint64_t get_iss_counter() { return m_host_acceess_iss_counter; }
    uint64_t get_rec_counter() { return m_host_acceess_rec_counter; }

    bool is_empty_ndp_req() override {
      return true;
    }

    bool is_empty_ndp_req(int pch_idx) override {
      return true;
    }

    int get_config_reg_resp(int pch_idx) override {
      return -1;
    }

    size_t get_host_acces_latency() {
      return s_read_latency;
    }

    std::vector<uint64_t> get_counters() override {
      std::vector<uint64_t> v;
      v.reserve(2);
      v.push_back(m_host_access_cnt);
      v.push_back(m_tcore_host_access_cnt);
      return v;
    }

    uint64_t get_req_latency() override {
      if (m_lat_vec.size() != 0) {
        auto& req_latency = m_lat_vec[0];
        m_lat_vec.erase(m_lat_vec.begin());
        return req_latency;
      } else {
        return 0;
      }
    }
};

}   // namespace Ramulator
