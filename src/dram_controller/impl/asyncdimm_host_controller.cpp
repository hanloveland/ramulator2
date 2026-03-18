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

      // During NMA Mode, skip refresh for that rank (NMA MC handles refresh)
      int rank_id = req.addr_vec[m_rank_addr_idx];
      if (m_mode_per_rank[rank_id] == AsyncDIMMMode::NMA) {
        return true;  // Pretend success to avoid AllBankRefresh throwing error
      }

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

      // 1. Serve completed reads
      serve_completed_reads();

      // 2. Tick refresh manager (sends REF via priority_send)
      m_refresh->tick();

      // 3. Try to find a request to serve
      ReqBuffer::iterator req_it;
      ReqBuffer* buffer = nullptr;
      bool request_found = schedule_request(req_it, buffer);

      // 4. Take row policy action
      m_rowpolicy->update(request_found, req_it);

      // 5. Update all plugins
      for (auto plugin : m_plugins) {
        plugin->update(request_found, req_it);
      }

      // 6. Issue the command to DRAM and bypass to NMA MC
      if (request_found) {
        if (req_it->is_stat_updated == false) {
          update_request_stats(req_it);
        }

        // Issue command to DRAM
        m_dram->issue_command(req_it->command, req_it->addr_vec);

        // ===== Explicit Sync (H2N): Bypass command to NMA MC =====
        // Every command issued to DRAM is forwarded to NMA MC:
        //   - Shadow FSM update (bank state tracking)
        //   - WR/WRA to magic-path addresses: NMA MC intercepts payload via DQ bus
        // Bypass is disabled during NMA Mode (NMA MC manages its own FSM).
        int cmd_rank_id = req_it->addr_vec[m_rank_addr_idx];
        if (m_mode_per_rank[cmd_rank_id] != AsyncDIMMMode::NMA && m_bypass_to_nma) {
          // Pass payload for WR/WRA so NMA MC can detect magic-path writes
          static const std::vector<uint64_t> s_empty_payload{};
          const std::vector<uint64_t>& payload =
            (req_it->command == m_dram->m_commands("WR") ||
             req_it->command == m_dram->m_commands("WRA"))
            ? req_it->m_payload
            : s_empty_payload;
          m_bypass_to_nma(cmd_rank_id, req_it->command, req_it->addr_vec, payload);
          s_num_bypass_cmds++;
        }

        // Track issued command types
        if (req_it->command == m_dram->m_commands("RD") || req_it->command == m_dram->m_commands("RDA"))
          s_num_issue_reads++;
        if (req_it->command == m_dram->m_commands("WR") || req_it->command == m_dram->m_commands("WRA"))
          s_num_issue_writes++;

        if (req_it->command == m_dram->m_commands("REFab")) {
          int nRFC_latency = m_dram->m_timing_vals("nRFC1");
          int rank_addr = req_it->addr_vec[m_rank_addr_idx];
          s_num_refresh_cc_per_rank[rank_addr] += nRFC_latency;
        }

        // Update row tracking
        if (req_it->command == m_dram->m_commands("ACT")) {
          int flat_bank_id = req_it->addr_vec[bank_idx] + req_it->addr_vec[bankgroup_idx] * num_banks + req_it->addr_vec[rank_idx] * num_bankgroups * num_banks;
          m_open_row_miss[flat_bank_id] = false;
          m_open_row[flat_bank_id] = req_it->addr_vec[row_idx];
          m_pre_open_row[flat_bank_id] = -1;
          m_scheduler->update_open_row_miss(flat_bank_id, false);
          m_scheduler->update_open_row(flat_bank_id, req_it->addr_vec[row_idx]);
          m_scheduler->update_pre_open_row(flat_bank_id, -1);
          m_scheduler->update_bk_status(flat_bank_id, false);
          m_rowpolicy->update_cap(0, req_it->addr_vec[rank_idx], req_it->addr_vec[bankgroup_idx], req_it->addr_vec[bank_idx], 128);
        } else if (req_it->command == m_dram->m_commands("PRE")) {
          int flat_bank_id = req_it->addr_vec[bank_idx] + req_it->addr_vec[bankgroup_idx] * num_banks + req_it->addr_vec[rank_idx] * num_bankgroups * num_banks;
          m_pre_open_row[flat_bank_id] = m_open_row[flat_bank_id];
          m_open_row[flat_bank_id] = -1;
          m_scheduler->update_open_row(flat_bank_id, -1);
          m_scheduler->update_pre_open_row(flat_bank_id, m_pre_open_row[flat_bank_id]);
          m_scheduler->update_bk_status(flat_bank_id, true);
        } else if (req_it->command == m_dram->m_commands("PREsb")) {
          // Same-bank precharge: close same bank_id across all bankgroups in the rank
          // NOTE: Missing from GenericDRAMController — added here for correctness
          int rank_id = req_it->addr_vec[rank_idx];
          int target_bank = req_it->addr_vec[bank_idx];
          for (int bg_idx = 0; bg_idx < num_bankgroups; bg_idx++) {
            int flat_bank_id = target_bank + bg_idx * num_banks + rank_id * num_bankgroups * num_banks;
            m_pre_open_row[flat_bank_id] = m_open_row[flat_bank_id];
            m_open_row[flat_bank_id] = -1;
            m_scheduler->update_open_row(flat_bank_id, -1);
            m_scheduler->update_pre_open_row(flat_bank_id, m_pre_open_row[flat_bank_id]);
            m_scheduler->update_bk_status(flat_bank_id, true);
          }
        } else if (req_it->command == m_dram->m_commands("PREA")) {
          int rank_id = req_it->addr_vec[rank_idx];
          for (int bg_idx = 0; bg_idx < num_bankgroups; bg_idx++) {
            for (int bk_idx = 0; bk_idx < num_banks; bk_idx++) {
              int flat_bank_id = bk_idx + bg_idx * num_banks + rank_id * num_bankgroups * num_banks;
              m_pre_open_row[flat_bank_id] = m_open_row[flat_bank_id];
              m_open_row[flat_bank_id] = -1;
              m_scheduler->update_open_row(flat_bank_id, -1);
              m_scheduler->update_pre_open_row(flat_bank_id, m_pre_open_row[flat_bank_id]);
              m_scheduler->update_bk_status(flat_bank_id, true);
            }
          }
        }

        // Handle request completion
        if (req_it->command == req_it->final_command) {
          if (req_it->type_id == Request::Type::Read) {
            req_it->depart = m_clk + m_dram->m_read_latency;
            pending.push_back(*req_it);
          }

          if (req_it->command == m_dram->m_commands("RD") || req_it->command == m_dram->m_commands("RDA") ||
              req_it->command == m_dram->m_commands("WR") || req_it->command == m_dram->m_commands("WRA")) {
            m_host_access_cnt++;
            if (req_it->is_trace_core_req) {
              m_tcore_host_access_cnt++;
            }
            if (req_it->is_host_req) {
              m_host_acceess_iss_counter++;
              if (req_it->type_id == Request::Type::Read) {
                m_host_rd_acceess_iss_counter++;
              }
            }
          }
          buffer->remove(req_it);
        } else {
          if (m_dram->m_command_meta(req_it->command).is_opening) {
            bool is_success = false;
            req_it->is_actived = true;
            is_success = m_active_buffer.enqueue(*req_it);
            if (!is_success) {
              throw std::runtime_error("Fail to enque to m_active_buffer");
            }
            buffer->remove(req_it);
          }
        }
      }
    };


  private:
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

    bool schedule_request(ReqBuffer::iterator& req_it, ReqBuffer*& req_buffer) {
      bool request_found = false;

      // Priority queue empty flag array
      for (int rk_idx = 0; rk_idx < m_num_rank; rk_idx++) {
        is_empty_priority_per_rank[rk_idx] = (m_priority_buffers[rk_idx].size() == 0);
      }

      // 1. First, check the active buffer (avoid useless ACTs)
      if (req_it = m_scheduler->get_best_request(m_active_buffer); req_it != m_active_buffer.end()) {
        if (m_dram->check_ready(req_it->command, req_it->addr_vec)) {
          request_found = true;
          req_buffer = &m_active_buffer;
        }
      }

      // 2. Check priority buffer (refresh), then read/write buffers
      if (!request_found) {
        for (auto rk_idx : rr_rk_idx) {
          if (m_priority_buffers[rk_idx].size() != 0) {
            req_buffer = &m_priority_buffers[rk_idx];
            req_it = m_priority_buffers[rk_idx].begin();
            req_it->command = m_dram->get_preq_command(req_it->final_command, req_it->addr_vec);

            request_found = m_dram->check_ready(req_it->command, req_it->addr_vec);
            if (request_found) break;
          }
        }

        if (!request_found) {
          for (auto rk_idx : rr_rk_idx) {
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
