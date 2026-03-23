#include <optional>
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
    // Per-bank per-command offload counters (Host MC → NMA MC)
    struct PerBankOffload {
      size_t acto = 0, preo = 0, rdo = 0, wro = 0;
      size_t ot_inc = 0, ot_dec = 0;  // Per-bank cumulative OT inc/dec
    };
    std::vector<PerBankOffload> s_offload_per_bank;  // [total_banks_flat]
    // Cumulative OT increment/decrement counters
    size_t s_ot_total_inc = 0;
    size_t s_ot_total_dec = 0;
    // RU entry cmd_count histogram (index 0=unused, 1/2/3 = cmd_count)
    size_t s_ru_cmd_count_hist[4] = {};
    // Per-rank interrupt count
    std::vector<size_t> s_interrupt_count;  // [num_rank]
    // Per-bank OT decrement cmd_count histogram
    struct PerBankOTDecHist {
      size_t hist[4] = {};  // [0]=unused, [1]/[2]/[3] = cmd_count
    };
    std::vector<PerBankOTDecHist> s_ot_dec_hist;  // [total_banks_flat]

    // Debug: log of RU pop events (Host MC side)
    struct RUPopLogEntry {
      int rank, bg, bk;
      int cmd_count;
      bool is_read;
      long clk;
    };
    std::vector<RUPopLogEntry> s_ru_pop_log;  // per-rank RU pop log

    // Debug: log of offload commands issued (per-bank timeline)
    struct OffloadCmdLogEntry {
      int rank, bg, bk;
      int cmd_type;  // 0=ACTO, 1=PREO, 2=RDO, 3=WRO
      int row;       // target row address
      int buf_type;  // 0=active_buf, 1=rd_buf, 2=wr_buf
      long clk;
    };
    std::vector<OffloadCmdLogEntry> s_offload_cmd_log;

    // RT issue frequency tracking (per 256-cycle window)
    static constexpr int RT_TRACK_WINDOW = 256;
    int m_rt_track_window_issues = 0;      // RT issues in current window
    int m_rt_track_window_pending = 0;     // Cycles with RT pending in current window
    int m_rt_track_window_start = 0;       // Start cycle of current window

    // Memory request counters
    uint64_t m_host_access_cnt = 0;
    uint64_t m_tcore_host_access_cnt = 0;
    uint64_t m_host_acceess_rec_counter = 0;
    uint64_t m_host_acceess_iss_counter = 0;
    uint64_t m_host_rd_acceess_rec_counter = 0;
    uint64_t m_host_rd_acceess_iss_counter = 0;

    // Bandwidth counters (AsyncDIMM-specific: Host<->NMA path)
    // Bypass = real DRAM commands (ACT/RD/WR/PRE) bypassed to NMA for bank state sync
    // Offload = offload commands (ACTO/RDO/WRO/PREO) sent to NMA for CMD FIFO execution
    uint64_t m_bw_host_nma_bypass = 0;         // Main: real RD/WR bypassed
    uint64_t m_bw_host_nma_offload = 0;        // Main: offloaded RDO/WRO
    uint64_t m_bw_tcore_host_nma_bypass = 0;   // tcore: real RD/WR bypassed
    uint64_t m_bw_tcore_host_nma_offload = 0;  // tcore: offloaded RDO/WRO

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
    //   3. send() detects ctrl-reg write → held in m_nma_start_hold[rank] (NOT in write buffer)
    //   4. tick() checks m_nma_wr_send_cnt == m_nma_wr_issue_cnt → enqueues to write buffer
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
    std::vector<std::optional<Request>> m_nma_start_hold; // [rank] ctrl-reg write held until NMAInst drain

    // ===== Phase 3: Concurrent Mode =====
    // Per-bank Offload Table (OT): counts outstanding offloaded commands.
    // Prevents CMD FIFO overflow; threshold = CMD FIFO size in concurrent mode (paper §V.B.2).
    static constexpr int HOST_OT_THRESHOLD = 12;  // CMD(8) + half REQ(4) = 12
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
    // Debug: return buffer/queue sizes for a given rank
    struct HostDebugState {
      int rd_buf, wr_buf, pri_buf, active_buf;
      int rt_pending, ru_size, ot_sum;
      int pending_depart;
    };
    HostDebugState get_debug_state(int rank_id) {
      HostDebugState s{};
      if (rank_id >= 0 && rank_id < m_num_rank) {
        s.rd_buf  = m_read_buffers[rank_id].size();
        s.wr_buf  = m_write_buffers[rank_id].size();
        s.pri_buf = m_priority_buffers[rank_id].size();
        s.rt_pending = m_rt_pending_count[rank_id];
        s.ot_sum = 0;
        for (int bg = 0; bg < num_bankgroups; bg++)
          for (int bk = 0; bk < num_banks; bk++)
            s.ot_sum += m_ot_counter[bk + bg * num_banks + rank_id * num_bankgroups * num_banks];
      }
      s.active_buf = m_active_buffer.size();
      s.ru_size = (int)m_return_unit.size();
      s.pending_depart = (int)pending.size();
      return s;
    }

    int get_total_banks_flat() const { return m_total_banks_flat; }
    int get_num_ranks() const { return num_ranks; }
    int get_num_bankgroups() const { return num_bankgroups; }
    int get_num_banks() const { return num_banks; }
    const PerBankOffload& get_offload_per_bank(int flat_bank_id) const {
      return s_offload_per_bank[flat_bank_id];
    }
    int get_ot_counter(int flat_bank_id) const {
      return m_ot_counter[flat_bank_id];
    }
    size_t get_ot_total_inc() const { return s_ot_total_inc; }
    size_t get_ot_total_dec() const { return s_ot_total_dec; }
    const size_t* get_ru_cmd_count_hist() const { return s_ru_cmd_count_hist; }
    size_t get_interrupt_count(int rank_id) const { return s_interrupt_count[rank_id]; }
    const PerBankOTDecHist& get_ot_dec_hist(int flat_bank_id) const { return s_ot_dec_hist[flat_bank_id]; }
    const std::vector<RUPopLogEntry>& get_ru_pop_log() const { return s_ru_pop_log; }
    const std::vector<OffloadCmdLogEntry>& get_offload_cmd_log() const { return s_offload_cmd_log; }

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
      m_nma_start_hold.resize(num_ranks, std::nullopt);
      m_nma_row_idx = m_dram->m_levels("row");
      m_nma_bg_idx  = m_dram->m_levels("bankgroup");
      m_nma_bk_idx  = m_dram->m_levels("bank");

      // Phase 3: OT counter and Return Unit
      m_total_banks_flat = num_ranks * num_bankgroups * num_banks;
      m_ot_counter.resize(m_total_banks_flat, 0);
      s_offload_per_bank.resize(m_total_banks_flat);
      s_interrupt_count.resize(num_ranks, 0);
      s_ot_dec_hist.resize(m_total_banks_flat);
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
     * Reset NMA write tracking counters for all ranks.
     * Called by system on trace repeat to prevent counter accumulation mismatch.
     */
    void reset_nma_counters() {
      for (int rk = 0; rk < m_num_rank; rk++) {
        m_nma_wr_send_cnt[rk] = 0;
        m_nma_wr_issue_cnt[rk] = 0;
        m_nma_start_requested[rk] = false;
        m_nma_start_hold[rk] = std::nullopt;
      }
    }

    /**
     * Check if any RU entries or RT-pending reads exist for a rank.
     * Used by System for C2H transition: must drain all offloaded accesses first.
     */
    bool has_pending_offloads_for_rank(int rank_id) const {
      for (const auto& entry : m_return_unit) {
        if (entry.req.addr_vec[m_rank_addr_idx] == rank_id) return true;
      }
      if (rank_id >= 0 && rank_id < m_num_rank) {
        if (!m_rt_read_pending[rank_id].empty()) return true;
        if (m_rt_pending_count[rank_id] > 0) return true;
      }
      return false;
    }

    /**
     * Drain all offloaded entries for a rank during C2H transition.
     * Completes reads (into pending for callback) and writes immediately.
     */
    void drain_offloads_for_rank(int rank_id) {
      // Clear stale refresh requests in priority buffer for this rank
      // (REFO requests that weren't issued before C2H transition)
      if (rank_id >= 0 && rank_id < m_num_rank) {
        while (m_priority_buffers[rank_id].size() > 1) {
          // Keep at most 1 pending refresh (the current interval's)
          m_priority_buffers[rank_id].remove(m_priority_buffers[rank_id].begin());
        }
      }

      // Drain RU entries for this rank
      for (auto it = m_return_unit.begin(); it != m_return_unit.end(); ) {
        if (it->req.addr_vec[m_rank_addr_idx] == rank_id) {
          if (it->bank_flat_id >= 0 && it->bank_flat_id < m_total_banks_flat) {
            m_ot_counter[it->bank_flat_id] = std::max(0, m_ot_counter[it->bank_flat_id] - it->cmd_count);
            // [DEBUG]
            // s_ot_total_dec += it->cmd_count;
            s_offload_per_bank[it->bank_flat_id].ot_dec += it->cmd_count;
          }
          if (it->is_read) {
            it->req.depart = m_clk + m_dram->m_read_latency;
            pending.push_back(it->req);
          } else {
            if (it->req.is_host_req) m_host_acceess_rec_counter++;
          }
          it = m_return_unit.erase(it);
        } else {
          ++it;
        }
      }
      // Drain RT-pending reads
      if (rank_id >= 0 && rank_id < m_num_rank) {
        while (!m_rt_read_pending[rank_id].empty()) {
          auto& req = m_rt_read_pending[rank_id].front();
          req.depart = m_clk + m_dram->m_read_latency;
          pending.push_back(req);
          m_rt_read_pending[rank_id].pop_front();
        }
        m_rt_pending_count[rank_id] = 0;
      }
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
      // [DEBUG]
      // s_interrupt_count[rank_id]++;

      // Find the OLDEST entry for this rank (head-of-line per rank)
      for (auto it = m_return_unit.begin(); it != m_return_unit.end(); ++it) {
        if (it->req.addr_vec[m_rank_addr_idx] != rank_id) continue;

        // OT decrement: cmd_count commands completed for this bank
        if (it->bank_flat_id >= 0 && it->bank_flat_id < m_total_banks_flat) {
          int cc = it->cmd_count;
          m_ot_counter[it->bank_flat_id] -= cc;
          // [DEBUG]
          // s_ot_total_dec += cc;
          s_offload_per_bank[it->bank_flat_id].ot_dec += cc;
          // [DEBUG]
          // if (cc >= 1 && cc <= 3) s_ot_dec_hist[it->bank_flat_id].hist[cc]++;
        }

        // Log RU pop event
        // [DEBUG]
        // s_ru_pop_log.push_back({rank_id,
        //   it->req.addr_vec[bankgroup_idx], it->req.addr_vec[bank_idx],
        //   it->cmd_count, it->is_read, (long)m_clk});

        if (it->is_read) {
          m_rt_read_pending[rank_id].push_back(it->req);
          m_rt_pending_count[rank_id]++;
        } else {
          if (it->req.is_host_req)
            m_host_acceess_rec_counter++;
        }

        m_return_unit.erase(it);
        return;
      }
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
            // Ctrl-reg (start) write → hold in separate unit, NOT in write buffer
            // Will be released to write buffer when all NMAInst writes are issued
            m_nma_start_requested[req_rank_id] = true;
            req.is_ndp_req = true;
            req.arrive = m_clk;
            m_nma_start_hold[req_rank_id] = req;
            s_num_write_reqs++;
            if (req.is_host_req) {
              m_host_acceess_rec_counter++;
            }
            return true;  // Accepted but held — skip normal enqueue
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



      // NMA start hold: release ctrl-reg WR to write buffer when all NMAInst WRs are issued
      for (int rk_id = 0; rk_id < m_num_rank; rk_id++) {
        if (m_nma_start_hold[rk_id].has_value() &&
            m_nma_wr_send_cnt[rk_id] == m_nma_wr_issue_cnt[rk_id]) {
          auto& held_req = m_nma_start_hold[rk_id].value();
          if (m_write_buffers[rk_id].enqueue(held_req)) {
            m_nma_start_hold[rk_id] = std::nullopt;
          }
        }
      }

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

      // // RT issue frequency tracking (per 256-cycle window)
      // {
      //   bool any_rt_pending = false;
      //   for (int rk = 0; rk < m_num_rank; rk++) {
      //     if (m_rt_pending_count[rk] > 0) { any_rt_pending = true; break; }
      //   }
      //   if (any_rt_pending) m_rt_track_window_pending++;
      //   if (rt_issued) m_rt_track_window_issues++;
      //
      //   if ((int)m_clk - m_rt_track_window_start >= RT_TRACK_WINDOW) {
      //     if (m_rt_track_window_pending > 0) {
      //       printf("[RT-TRACK] ch=%d cycle=%d-%ld | RT_issues=%d RT_pending_cycles=%d/%d\n",
      //         m_channel_id, m_rt_track_window_start, m_clk,
      //         m_rt_track_window_issues, m_rt_track_window_pending, RT_TRACK_WINDOW);
      //     }
      //     m_rt_track_window_issues = 0;
      //     m_rt_track_window_pending = 0;
      //     m_rt_track_window_start = m_clk;
      //   }
      // }

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

        // NMA start gate: no longer needed here — ctrl-reg WR is held in
        // m_nma_start_hold[] and only released to write buffer when
        // m_nma_wr_send_cnt == m_nma_wr_issue_cnt (see tick() above)

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

        // NMA write tracking: inst-buf WR/WRO → issue count; ctrl-reg WR → mode transition
        // Note: In CONCURRENT mode, NMAInst writes are offloaded as WRO (not WR).
        // Must check both final_command (WR in HOST) and WRO (offload in CONCURRENT).
        bool is_nma_final = (cmd == req_it->final_command) ||
                            (cmd == m_dram->m_commands("WRO"));
        if (req_it->is_ndp_req && is_nma_final &&
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
          if (flat_bank_id >= 0 && flat_bank_id < m_total_banks_flat) {
            m_ot_counter[flat_bank_id]++;
            // [DEBUG]
            // s_ot_total_inc++;
            s_offload_per_bank[flat_bank_id].ot_inc++;
          }
          // Per-request cmd_count tracking via scratchpad[0]
          req_it->scratchpad[0]++;
          // Per-bank per-command offload counters
          if (flat_bank_id >= 0 && flat_bank_id < m_total_banks_flat) {
            int cmd_type = -1;
            if      (cmd == m_dram->m_commands("ACTO")) { s_offload_per_bank[flat_bank_id].acto++; cmd_type = 0; }
            else if (cmd == m_dram->m_commands("PREO")) { s_offload_per_bank[flat_bank_id].preo++; cmd_type = 1; }
            else if (cmd == m_dram->m_commands("RDO"))  { s_offload_per_bank[flat_bank_id].rdo++;  cmd_type = 2; }
            else if (cmd == m_dram->m_commands("WRO"))  { s_offload_per_bank[flat_bank_id].wro++;  cmd_type = 3; }
            if (cmd_type >= 0) {
              int buf_type = 0;  // default: active_buf
              if (buffer == &m_active_buffer) buf_type = 0;
              else {
                int rk = req_it->addr_vec[rank_idx];
                if (rk >= 0 && rk < m_num_rank) {
                  if (buffer == &m_read_buffers[rk]) buf_type = 1;
                  else if (buffer == &m_write_buffers[rk]) buf_type = 2;
                }
              }
              // [DEBUG]
              // s_offload_cmd_log.push_back({req_it->addr_vec[rank_idx],
              //   req_it->addr_vec[bankgroup_idx], req_it->addr_vec[bank_idx], cmd_type,
              //   req_it->addr_vec[row_idx], buf_type, (long)m_clk});
            }
          }
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
            int cc = req_it->scratchpad[0];
            m_return_unit.push_back({*req_it, cc, flat_bank_id, is_read});
            // [DEBUG]
            // if (cc >= 1 && cc <= 3) s_ru_cmd_count_hist[cc]++;
          }
          // REFO: no RU entry needed (refresh has no frontend callback)
          m_host_access_cnt++;
          if (req_it->is_trace_core_req) m_tcore_host_access_cnt++;
          if (req_it->is_host_req) {
            m_host_acceess_iss_counter++;
            if (req_it->type_id == Request::Type::Read) m_host_rd_acceess_iss_counter++;
          }
          // BW: Host<->NMA Offload (concurrent mode RDO/WRO, not REFO)
          if (cmd != m_dram->m_commands("REFO")) {
            m_bw_host_nma_offload++;
            if (req_it->is_trace_core_req) m_bw_tcore_host_nma_offload++;
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
          // BW: Host<->NMA Bypass (real RD/WR bypassed to NMA for sync)
          m_bw_host_nma_bypass++;
          if (req_it->is_trace_core_req) m_bw_tcore_host_nma_bypass++;
          buffer->remove(req_it);
        } else if (!is_offload && m_dram->m_command_meta(cmd).is_opening) {
          // Real ACT → active_buffer
          req_it->is_actived = true;
          if (!m_active_buffer.enqueue(*req_it))
            throw std::runtime_error("Fail to enque to m_active_buffer");
          buffer->remove(req_it);
        } else if (is_offload && cmd == m_dram->m_commands("ACTO")) {
          // ACTO → active_buffer (ensures RDO/WRO gets Step 1a priority)
          req_it->is_actived = true;
          if (!m_active_buffer.enqueue(*req_it))
            throw std::runtime_error("Fail to enque to m_active_buffer");
          buffer->remove(req_it);
        }
        // PREO: request stays in buffer → next tick m_open_row closed → ACTO
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

      // Build set of banks that already have requests in active_buffer.
      // These banks are waiting for RDO/WRO in Step 1a — issuing PREO/ACTO
      // for the same bank from another request would close/reopen the row,
      // invalidating the active_buffer entry and wasting OT slots.
      std::vector<bool> bank_in_active(m_total_banks_flat, false);
      for (auto it = m_active_buffer.begin(); it != m_active_buffer.end(); it++) {
        int ab_rk = it->addr_vec[m_rank_addr_idx];
        if (ab_rk != rk_idx) continue;
        int ab_flat = it->addr_vec[bank_idx]
                    + it->addr_vec[bankgroup_idx] * num_banks
                    + it->addr_vec[rank_idx] * num_bankgroups * num_banks;
        if (ab_flat >= 0 && ab_flat < m_total_banks_flat)
          bank_in_active[ab_flat] = true;
      }

      // FR-FCFS offload scheduling:
      //   1st priority: Row hit (RDO/WRO) — "First Ready"
      //   2nd priority: Row miss/conflict (ACTO/PREO) — oldest request (FCFS)
      // Within same priority, prefer the primary buffer's mode (write/read),
      // then pick oldest by arrive time.

      ReqBuffer* buffers[2];
      if (m_is_write_mode_per_rank[rk_idx]) {
        buffers[0] = &m_write_buffers[rk_idx];
        buffers[1] = &m_read_buffers[rk_idx];
      } else {
        buffers[0] = &m_read_buffers[rk_idx];
        buffers[1] = &m_write_buffers[rk_idx];
      }

      // Best candidate tracking
      ReqBuffer::iterator best_it;
      ReqBuffer* best_buffer = nullptr;
      int best_cmd = -1;
      bool best_is_hit = false;
      Clk_t best_arrive = std::numeric_limits<Clk_t>::max();

      for (int bi = 0; bi < 2; bi++) {
        auto& buffer = *buffers[bi];
        if (buffer.size() == 0) continue;

        for (auto it = buffer.begin(); it != buffer.end(); it++) {
          int flat_bank_id = it->addr_vec[bank_idx]
                           + it->addr_vec[bankgroup_idx] * num_banks
                           + it->addr_vec[rank_idx] * num_bankgroups * num_banks;

          // OT backpressure
          if (flat_bank_id >= 0 && flat_bank_id < m_total_banks_flat &&
              m_ot_counter[flat_bank_id] >= HOST_OT_THRESHOLD) continue;

          // Skip banks with pending RDO/WRO in active_buffer
          if (flat_bank_id >= 0 && flat_bank_id < m_total_banks_flat &&
              bank_in_active[flat_bank_id]) continue;

          int target_row = it->addr_vec[row_idx];
          bool is_read   = (it->type_id == Request::Type::Read);
          bool is_hit = false;
          int offload_cmd = -1;

          if (m_open_row[flat_bank_id] == -1) {
            // Bank closed → ACTO
            offload_cmd = m_dram->m_commands("ACTO");
          } else if (m_open_row[flat_bank_id] != target_row) {
            // Row conflict → PREO
            offload_cmd = m_dram->m_commands("PREO");
          } else {
            // Row hit → RDO/WRO
            is_hit = true;
            offload_cmd = is_read ? m_dram->m_commands("RDO")
                                  : m_dram->m_commands("WRO");
          }

          if (!m_dram->check_ready(offload_cmd, it->addr_vec)) continue;

          // Compare with current best: hit > miss, then FCFS (oldest arrive)
          bool dominated = false;
          if (best_cmd >= 0) {
            if (best_is_hit && !is_hit) dominated = true;          // existing hit beats miss
            else if (best_is_hit == is_hit && best_arrive <= it->arrive) dominated = true;  // same priority, existing is older
          }
          if (dominated) continue;

          best_it = it;
          best_buffer = &buffer;
          best_cmd = offload_cmd;
          best_is_hit = is_hit;
          best_arrive = it->arrive;
        }
      }

      if (best_cmd >= 0) {
        best_it->command = best_cmd;
        req_it = best_it;
        req_buffer = best_buffer;
        return true;
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

      // 1. Active buffer — drain entries for refresh-pending ranks first,
      //    then serve remaining entries via FRFCFS.

      // 1a. CONCURRENT/NMA entries: serve directly with RDO/WRO, bypassing FRFCFS.
      //     get_best_request() calls get_preq_command() which checks real DRAM bank state.
      //     NMA MC modifies that state during concurrent execution, so CONCURRENT entries
      //     appear "not ready" and are never picked by FRFCFS — causing permanent stall.
      //     RDO/WRO have NoRequire prerequisites, so we issue them directly.
      if ((int)m_return_unit.size() < RU_MAX_ENTRIES) {
        for (auto it = m_active_buffer.begin(); it != m_active_buffer.end(); it++) {
          int ab_rk = it->addr_vec[m_rank_addr_idx];
          if (ab_rk < 0 || ab_rk >= m_num_rank) continue;
          if (m_mode_per_rank[ab_rk] == AsyncDIMMMode::HOST) continue;
          // OT backpressure: prevent CMD FIFO overflow on NMA side
          int ab_flat = it->addr_vec[bank_idx]
                      + it->addr_vec[bankgroup_idx] * num_banks
                      + it->addr_vec[rank_idx] * num_bankgroups * num_banks;
          if (ab_flat >= 0 && ab_flat < m_total_banks_flat &&
              m_ot_counter[ab_flat] >= HOST_OT_THRESHOLD) continue;
          bool is_read = (it->type_id == Request::Type::Read);
          int offload_cmd = is_read ? m_dram->m_commands("RDO") : m_dram->m_commands("WRO");
          if (m_dram->check_ready(offload_cmd, it->addr_vec)) {
            it->command = offload_cmd;
            req_it = it;
            request_found = true;
            req_buffer = &m_active_buffer;
            break;
          }
        }
      }

      // 1b. Prioritize active_buffer entries for refresh-pending HOST ranks.
      //     Without this, FRFCFS picks entries for OTHER ranks, starving refresh.
      if (!request_found) {
        for (auto it = m_active_buffer.begin(); it != m_active_buffer.end(); it++) {
          int ab_rk = it->addr_vec[m_rank_addr_idx];
          if (ab_rk < 0 || ab_rk >= m_num_rank) continue;
          if (m_mode_per_rank[ab_rk] != AsyncDIMMMode::HOST) continue;
          if (is_empty_priority_per_rank[ab_rk]) continue;  // No pending refresh
          it->command = m_dram->get_preq_command(it->final_command, it->addr_vec);
          if (m_dram->check_ready(it->command, it->addr_vec)) {
            req_it = it;
            request_found = true;
            req_buffer = &m_active_buffer;

            break;
          }
        }
      }

      // 1c. HOST entries: use FRFCFS scheduler (normal path)
      if (!request_found) {
        if (req_it = m_scheduler->get_best_request(m_active_buffer); req_it != m_active_buffer.end()) {
          int ab_rk = req_it->addr_vec[m_rank_addr_idx];
          if (ab_rk >= 0 && ab_rk < m_num_rank &&
              m_mode_per_rank[ab_rk] != AsyncDIMMMode::HOST) {
            // CONCURRENT/NMA entry picked by FRFCFS (1a didn't serve it — RU full, CA timing, or OT)
            if ((int)m_return_unit.size() < RU_MAX_ENTRIES) {
              // OT backpressure
              int fc_flat = req_it->addr_vec[bank_idx]
                          + req_it->addr_vec[bankgroup_idx] * num_banks
                          + req_it->addr_vec[rank_idx] * num_bankgroups * num_banks;
              bool ot_ok = !(fc_flat >= 0 && fc_flat < m_total_banks_flat &&
                             m_ot_counter[fc_flat] >= HOST_OT_THRESHOLD);
              if (ot_ok) {
                bool is_read = (req_it->type_id == Request::Type::Read);
                int offload_cmd = is_read ? m_dram->m_commands("RDO") : m_dram->m_commands("WRO");
                if (m_dram->check_ready(offload_cmd, req_it->addr_vec)) {
                  req_it->command = offload_cmd;
                  request_found = true;
                  req_buffer = &m_active_buffer;

                }
              }
            }
          } else {
            // HOST: normal scheduling
            if (m_dram->check_ready(req_it->command, req_it->addr_vec)) {
              request_found = true;
              req_buffer = &m_active_buffer;

            }
          }
        }
      }

      // 2. Priority buffer (refresh) — per-rank, runs when rank has no active_buffer conflict.
      //    Unlike before, this runs even if Step 1 found a request for a DIFFERENT rank.
      //    This prevents refresh starvation when other ranks' active_buffer entries
      //    keep Step 1 busy.
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

      // 4. Check if the scheduled command would close an open row in the active buffer
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
      v.reserve(10);
      // [Main window] Host <-> NMA
      v.push_back(m_bw_host_nma_bypass);     // [0] Bypass (real RD/WR)
      v.push_back(m_bw_host_nma_offload);    // [1] Offload (RDO/WRO)
      // [Main window] NMA <-> DRAM: placeholder (filled by system from NMA MC)
      v.push_back(0);                        // [2] Bypass (= [0])
      v.push_back(0);                        // [3] Offload (CMD FIFO)
      v.push_back(0);                        // [4] NMA (REQ FIFO)
      // [tcore window] Host <-> NMA
      v.push_back(m_bw_tcore_host_nma_bypass);   // [5] Bypass
      v.push_back(m_bw_tcore_host_nma_offload);  // [6] Offload
      // [tcore window] NMA <-> DRAM: placeholder
      v.push_back(0);                        // [7] Bypass (= [5])
      v.push_back(0);                        // [8] Offload (CMD FIFO)
      v.push_back(0);                        // [9] NMA (REQ FIFO)
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
