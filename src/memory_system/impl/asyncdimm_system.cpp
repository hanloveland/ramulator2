#include "memory_system/memory_system.h"
#include "translation/translation.h"
#include "dram_controller/controller.h"
#include "dram_controller/impl/asyncdimm_host_controller.cpp"
#include "dram_controller/impl/asyncdimm_nma_controller.cpp"
#include "addr_mapper/addr_mapper.h"
#include "dram/dram.h"
#include <sstream>
#include <filesystem>
#include <iostream>
#include <fstream>
#include <iomanip>

namespace Ramulator {

namespace fs = std::filesystem;

/**
 * AsyncDIMM Memory System (HPCA 2025)
 *
 * Manages 1 Host MC per channel + N NMA MCs (one per rank) within a channel.
 *
 * Phase 1 (Host Mode):
 *   - All requests routed to Host MC (standard DDR5 scheduling)
 *   - Every command issued by Host MC is bypassed to NMA MC (explicit sync H2N)
 *   - NMA MC maintains shadow bank FSM
 *
 * Phase 2 (NMA Mode) — Magic Path (DQ-based):
 *   - NMAInst writes: host writes to inst-buffer address → Host MC issues ACT+WR
 *     → NMA MC intercepts WR payload via bypass_command() (DQ magic path)
 *     → NMAInst decoded directly into NMA MC instruction queue
 *   - Ctrl-reg writes: same DQ path → NMA MC sets m_nma_start_pending
 *   - System polls is_nma_start_pending() → triggers H2N transition
 *   - No separate Host→NMA software write path; no m_nma_inst_buffers staging area
 *   - NMA MC issues DRAM commands independently; implicit sync (N2H) via PREA
 *
 * Phase 3 (Concurrent Mode): [Placeholder]
 */
class AsyncDIMMSystem final : public IMemorySystem, public Implementation {
  RAMULATOR_REGISTER_IMPLEMENTATION(IMemorySystem, AsyncDIMMSystem, "AsyncDIMM", "AsyncDIMM memory system with Host MC + NMA MC (HPCA 2025).");

  protected:
    Clk_t m_clk = 0;
    IDRAM* m_dram;
    IAddrMapper* m_addr_mapper;

    // One Host MC per channel (registered via Factory as IDRAMController)
    std::vector<IDRAMController*> m_host_controllers;

    // One NMA MC per rank per channel: m_nma_controllers[channel][rank]
    std::vector<std::vector<AsyncDIMMNMAController*>> m_nma_controllers;

    int m_num_channels = -1;
    int m_num_ranks = -1;
    int m_num_bankgroups = -1;
    int m_num_banks = -1;

    // FSM sync verification (enabled via YAML: debug_fsm_sync: true)
    bool m_debug_fsm_sync = false;
    size_t m_fsm_sync_errors = 0;

    // ===== NMA Control Region (memory-mapped, DQ magic path) =====
    // Writes to these addresses go through Host MC (ACT+WR) and are
    // intercepted by NMA MC via bypass_command() on the DQ bus.
    int m_nma_ctrl_row = -1;
    int m_nma_ctrl_bg  = -1;   // BG of control register (start trigger)
    int m_nma_ctrl_bk  = -1;
    int m_nma_buf_bg   = -1;   // BG of NMAInst instruction buffer
    int m_nma_buf_bk   = -1;

    // Level indices for address parsing
    int m_row_addr_idx = -1;
    int m_bg_addr_idx  = -1;
    int m_bk_addr_idx  = -1;
    int m_ch_addr_idx  = -1;
    int m_rk_addr_idx  = -1;

    // ===== NMA Workload Trace (Phase 2) =====
    bool m_trace_core_enable = false;
    bool m_nma_trace = false;

    // Host stall detection (tcore mode only)
    static constexpr uint64_t HOST_STALL_THRESHOLD = 100000;
    uint64_t m_last_host_send_clk = 0;
    bool m_host_send_ever = false;
    bool m_host_stall_terminated = false;

    struct Trace {
      uint64_t timestamp;
      bool is_write;
      Addr_t addr;
      std::vector<uint64_t> payload;
    };

    size_t m_trace_core_mshr_size = 16;
    size_t m_curr_trace_idx = 0;
    std::vector<Trace> m_trace;
    int m_next_request_id = 0;
    bool m_debug_mode = false;
    int m_wait_trace_done = 0;
    bool m_trace_done = false;
    bool m_nma_ever_started = false;  // Track if NMA was ever triggered
    int m_trace_repeat = 1;           // Number of times to repeat the trace
    int m_trace_repeat_done = 0;      // Number of completed repeats

    // Outstanding reads tracking
    struct OutstandingRequest {
      uint64_t issue_time;
      Addr_t addr;
    };
    size_t m_max_outstanding = 16;
    std::unordered_map<int, OutstandingRequest> m_outstanding_reads;

    // Per-rank, per-channel mode state (system-level view)
    enum class SystemNMAState {
      HOST_MODE,            // Host MC active, NMA MC in bypass
      TRANSITIONING_H2N,   // About to switch to NMA (set Host MC mode, start NMA MC)
      NMA_MODE,             // NMA MC active, Host MC idle
      TRANSITIONING_N2H,   // NMA MC done, syncing back to Host Mode
      TRANSITIONING_H2C,   // About to switch to Concurrent (Phase 3)
      CONCURRENT,           // Both Host MC and NMA MC active (Phase 3 OSR)
      TRANSITIONING_C2H,   // Concurrent done, syncing back to Host Mode
    };
    std::vector<std::vector<SystemNMAState>> m_rank_state;  // [ch][rk]

    // Phase 3: concurrent mode configuration
    bool m_concurrent_mode_enable = false;  // Set via YAML: concurrent_mode_enable: true

  public:
    Logger_t m_logger;

    size_t s_num_read_requests = 0;
    size_t s_num_write_requests = 0;
    size_t s_num_other_requests = 0;
    float s_avg_read_latency = 0;

    // Latency Tracking
    static constexpr uint32_t BIN_WIDTH = 1;
    static constexpr uint32_t MAX_LAT   = 2000000;
    static constexpr uint32_t NUM_BINS  = (MAX_LAT / BIN_WIDTH) + 2;
    std::array<uint64_t, NUM_BINS> hist_{};
    uint64_t total_reads_ = 0;
    uint64_t sum_lat_     = 0;
    uint64_t max_lat_     = 0;
    uint64_t overflow_    = 0;
    uint64_t read_latency;

  public:
    void init() override {
      m_logger = Logging::create_logger("AsyncDIMMSystem");
      m_logger->info("AsyncDIMM Memory System init()");

      m_dram = create_child_ifce<IDRAM>();
      m_addr_mapper = create_child_ifce<IAddrMapper>();

      m_num_channels  = m_dram->get_level_size("channel");
      m_num_ranks     = m_dram->get_level_size("rank");
      m_num_bankgroups = m_dram->get_level_size("bankgroup");
      m_num_banks     = m_dram->get_level_size("bank");

      m_ch_addr_idx  = m_dram->m_levels("channel");
      m_rk_addr_idx  = m_dram->m_levels("rank");
      m_bg_addr_idx  = m_dram->m_levels("bankgroup");
      m_bk_addr_idx  = m_dram->m_levels("bank");
      m_row_addr_idx = m_dram->m_levels("row");

      m_debug_fsm_sync = param<bool>("debug_fsm_sync").default_val(false);
      m_debug_mode     = param<bool>("debug_mode").default_val(false);

      // NMA control region: highest row, specific BG/BK
      m_nma_ctrl_row = m_dram->get_level_size("row") - 1;
      m_nma_ctrl_bg  = m_num_bankgroups - 1;   // Last BG → control register
      m_nma_ctrl_bk  = m_num_banks - 1;        // Last BK → control register
      m_nma_buf_bg   = m_num_bankgroups - 2;   // Second-to-last BG → inst buffer
      m_nma_buf_bk   = m_num_banks - 1;        // Last BK → inst buffer

      m_logger->info("  NMA Control Region: row={}, ctrl_bg={}, ctrl_bk={}, buf_bg={}, buf_bk={}",
                     m_nma_ctrl_row, m_nma_ctrl_bg, m_nma_ctrl_bk, m_nma_buf_bg, m_nma_buf_bk);

      // Calculate total memory capacity
      int num_dram_die = m_num_channels * m_num_ranks * (m_dram->m_channel_width / m_dram->m_organization.dq);
      total_memory_capacity = (num_dram_die * m_dram->m_organization.density / 1024) / 8;
      m_logger->info(" AsyncDIMM System Configuration");
      m_logger->info("   - # of Channels          : {}", m_num_channels);
      m_logger->info("   - # of Ranks             : {}", m_num_ranks);
      m_logger->info("   - DQs per DRAM Die       : {}", m_dram->m_organization.dq);
      m_logger->info("   - DQs per Channel        : {}", m_dram->m_channel_width);
      m_logger->info("   - DRAM die density (Mb)  : {}", m_dram->m_organization.density);
      m_logger->info("   - Total DRAM Dies        : {}", num_dram_die);
      m_logger->info("   - Total DRAM Capacity(GB): {}", total_memory_capacity);

      // Create Host MCs (one per channel)
      for (int ch = 0; ch < m_num_channels; ch++) {
        IDRAMController* controller = create_child_ifce<IDRAMController>();
        controller->m_impl->set_id(fmt::format("Channel {}", ch));
        controller->m_channel_id = ch;
        m_host_controllers.push_back(controller);
      }

      // Create NMA MCs (one per rank per channel)
      // Pass magic-path addresses so NMA MC can intercept DQ WR commands.
      m_nma_controllers.resize(m_num_channels);
      for (int ch = 0; ch < m_num_channels; ch++) {
        for (int rk = 0; rk < m_num_ranks; rk++) {
          auto* nma = new AsyncDIMMNMAController();
          nma->init(rk, m_dram,
                    m_nma_ctrl_row, m_nma_ctrl_bg, m_nma_ctrl_bk,
                    m_nma_buf_bg,   m_nma_buf_bk);
          nma->set_channel_id(ch);
          m_nma_controllers[ch].push_back(nma);
          m_logger->info("   - Created NMA MC for Channel {} Rank {}", ch, rk);
        }
      }

      // Wire bypass callback + NMA addresses: Host MC → NMA MC
      for (int ch = 0; ch < m_num_channels; ch++) {
        auto* host_mc = dynamic_cast<AsyncDIMMHostController*>(m_host_controllers[ch]->m_impl);
        if (host_mc) {
          auto& nma_vec = m_nma_controllers[ch];
          host_mc->set_bypass_callback(
            [&nma_vec](int rank_id, int command, const AddrVec_t& addr_vec,
                       const std::vector<uint64_t>& payload) {
              nma_vec[rank_id]->bypass_command(command, addr_vec, payload);
            }
          );
          // Pass magic-path addresses to Host MC for NMA write tracking
          host_mc->set_nma_addresses(m_nma_ctrl_row, m_nma_ctrl_bg, m_nma_ctrl_bk,
                                     m_nma_buf_bg, m_nma_buf_bk);
          m_logger->info("   - Wired bypass + NMA addresses: Host MC Ch {} -> NMA MCs", ch);

          // Wire NMA idle query callback (Host MC polls NMA MC state for C2H transition)
          host_mc->set_nma_query_callback([this, ch](int rk) {
            return m_nma_controllers[ch][rk]->is_nma_idle();
          });
          m_logger->info("   - Wired NMA idle query: Host MC Ch {} -> NMA MCs", ch);
        } else {
          m_logger->warn("   - Host MC Ch {} is not AsyncDIMMHostController, bypass not wired!", ch);
        }
      }

      // Initialize per-rank mode state
      m_rank_state.resize(m_num_channels);
      for (int ch = 0; ch < m_num_channels; ch++)
        m_rank_state[ch].resize(m_num_ranks, SystemNMAState::HOST_MODE);

      m_clock_ratio = param<uint>("clock_ratio").required();
      m_concurrent_mode_enable = param<bool>("concurrent_mode_enable").default_val(false);
      if (m_concurrent_mode_enable)
        m_logger->info("  -- Concurrent Mode (Phase 3 OSR) ENABLED");

      // Pass concurrent mode flag to Host MCs
      for (int ch = 0; ch < m_num_channels; ch++) {
        auto* host_mc = dynamic_cast<AsyncDIMMHostController*>(m_host_controllers[ch]->m_impl);
        if (host_mc) host_mc->set_concurrent_mode_enable(m_concurrent_mode_enable);
      }

      // Phase 3: Wire NMA MC interrupt callback → Host MC on_nma_interrupt()
      // and configure H/N Arbiter fairness parameters
      if (m_concurrent_mode_enable) {
        // H/N Arbiter fairness parameters (YAML configurable)
        int fairness_interval = param<int>("hn_cmd_service_interval")
            .desc("(A) REQ consecutive NMA ticks before forced CMD switch (0=disabled)")
            .default_val(16);
        int fairness_quota = param<int>("hn_cmd_service_quota")
            .desc("(A) Minimum CMD ticks per forced CMD switch")
            .default_val(4);
        int fairness_cap = param<int>("hn_req_continuous_cap")
            .desc("(C) REQ consecutive service cap before CMD switch (0=disabled)")
            .default_val(8);
        m_logger->info("  -- H/N Arbiter fairness: interval={}, quota={}, cap={}",
                       fairness_interval, fairness_quota, fairness_cap);

        for (int ch = 0; ch < m_num_channels; ch++) {
          auto* host_mc = dynamic_cast<AsyncDIMMHostController*>(m_host_controllers[ch]->m_impl);
          if (!host_mc) continue;
          for (int rk = 0; rk < m_num_ranks; rk++) {
            m_nma_controllers[ch][rk]->set_interrupt_callback(
              [host_mc](int rank_id, int batch_size) {
                host_mc->on_nma_interrupt(rank_id, batch_size);
              }
            );
            // [DEBUG]
            // m_nma_controllers[ch][rk]->set_host_debug_callback(
            //   [host_mc](int rank_id) -> AsyncDIMMNMAController::HostDebugState {
            //     auto s = host_mc->get_debug_state(rank_id);
            //     return {s.rd_buf, s.wr_buf, s.pri_buf, s.active_buf,
            //             s.rt_pending, s.ru_size, s.ot_sum, s.pending_depart};
            //   }
            // );
            m_nma_controllers[ch][rk]->set_fairness_params(
              fairness_interval, fairness_quota, fairness_cap);
          }
          m_logger->info("   - Wired NMA interrupt callback: NMA MCs Ch {} -> Host MC", ch);
        }
      }

      // NMA trace configuration
      m_trace_core_enable = param<bool>("trace_core_enable").default_val(false);
      if (m_trace_core_enable) {
        m_nma_trace = param<bool>("trace_nma_type").default_val(false);
        m_trace_core_mshr_size = param<size_t>("trace_core_mshr_size").default_val(16);
        m_max_outstanding = m_trace_core_mshr_size;

        std::string trace_path = param<std::string>("trace_path").desc("NMA trace file path").required();
        m_trace_repeat = param<int>("trace_repeat").desc("Number of times to repeat trace (0=infinite)").default_val(0);
        m_logger->info("  -- NMA Trace Path: {}", trace_path);
        m_logger->info("  -- Trace Repeat: {}", m_trace_repeat);
        load_trace(trace_path, m_trace);
        m_logger->info("  -- Loaded {} trace lines", m_trace.size());
      }

      register_stat(m_clk).name("memory_system_cycles");
      register_stat(s_num_read_requests).name("total_num_read_requests");
      register_stat(s_num_write_requests).name("total_num_write_requests");
      register_stat(s_num_other_requests).name("total_num_other_requests");
      register_stat(s_avg_read_latency).name("avg_host_read_latency");
    };

    void setup(IFrontEnd* frontend, IMemorySystem* memory_system) override { }

    /**
     * send(): Route request to Host MC.
     *
     * Magic-path writes (NMAInst / ctrl-reg) are routed to the Host MC as
     * normal WR requests. The Host MC schedules ACT+WR; when WR is issued,
     * bypass_command() carries the payload to the NMA MC via the DQ path.
     * No separate system-level interception is performed.
     */
    bool send(Request req) override {
      m_addr_mapper->apply(req);
      int channel_id = req.addr_vec[m_ch_addr_idx];

      req.is_host_req = true;
      bool is_success = m_host_controllers[channel_id]->send(req);

      if (is_success) {
        switch (req.type_id) {
          case Request::Type::Read:  s_num_read_requests++;  break;
          case Request::Type::Write: s_num_write_requests++; break;
          default:                   s_num_other_requests++; break;
        }
      }

      if (is_success && !req.is_trace_core_req) {
        m_last_host_send_clk = m_clk;
        m_host_send_ever = true;
      }
      return is_success;
    };

    void tick() override {
      m_clk++;

      // Host stall detection: tcore backpressure → host can't send
      if (m_trace_core_enable && m_host_send_ever && !m_host_stall_terminated &&
          (m_clk - m_last_host_send_clk > HOST_STALL_THRESHOLD)) {
        m_logger->warn("Host stalled for {} cycles (tcore backpressure) — terminating simulation.",
                       m_clk - m_last_host_send_clk);
        m_host_stall_terminated = true;
      }

      m_dram->tick();

      for (auto controller : m_host_controllers) {
        controller->tick();
        while (true) {
          read_latency = controller->get_req_latency();
          if (read_latency == 0) break;
          record_latency(read_latency);
        }
      }

      for (int ch = 0; ch < m_num_channels; ch++)
        for (int rk = 0; rk < m_num_ranks; rk++)
          m_nma_controllers[ch][rk]->tick();

      tick_mode_transitions();

      if (m_trace_core_enable)
        try_issue_trace_requests();

      if (m_debug_fsm_sync)
        verify_fsm_sync();

      // [DEBUG]
      // // Debug dump at cycle 2100000 and exit (disabled)
      // if (false && m_clk == 2100000) {
      //   printf("\n========== DEBUG DUMP @ cycle %ld ==========\n", m_clk);
      //   for (int ch = 0; ch < m_num_channels; ch++) {
      //     auto* host_mc = dynamic_cast<AsyncDIMMHostController*>(m_host_controllers[ch]);
      //     if (!host_mc) continue;
      //     int n_bg = host_mc->get_num_bankgroups();
      //     int n_bk = host_mc->get_num_banks();
      //     int n_rk = host_mc->get_num_ranks();
      //
      //     printf("[Host MC ch=%d] OT total_inc=%zu  total_dec=%zu  delta=%zd\n",
      //       ch, host_mc->get_ot_total_inc(), host_mc->get_ot_total_dec(),
      //       (ssize_t)host_mc->get_ot_total_inc() - (ssize_t)host_mc->get_ot_total_dec());
      //     {
      //       auto* h = host_mc->get_ru_cmd_count_hist();
      //       printf("[Host MC ch=%d] RU cmd_count histogram: cc=1:%zu  cc=2:%zu  cc=3:%zu  (total=%zu)\n",
      //         ch, h[1], h[2], h[3], h[1]+h[2]+h[3]);
      //     }
      //     for (int rk = 0; rk < n_rk; rk++) {
      //       auto* nma_mc = m_nma_controllers[ch][rk];
      //       printf("[ch=%d rk=%d] Interrupts sent(NMA): %zu  received(Host): %zu  delta: %zd\n",
      //         ch, rk, nma_mc->get_num_interrupts_sent(), host_mc->get_interrupt_count(rk),
      //         (ssize_t)nma_mc->get_num_interrupts_sent() - (ssize_t)host_mc->get_interrupt_count(rk));
      //     }
      //
      //     for (int rk = 0; rk < n_rk; rk++) {
      //       auto* nma_mc = m_nma_controllers[ch][rk];
      //
      //       // Per-rank summary
      //       size_t h_acto=0,h_preo=0,h_rdo=0,h_wro=0;
      //       size_t n_acto=0,n_preo=0,n_rdo=0,n_wro=0;
      //       size_t i_act=0,i_pre=0,i_rd=0,i_wr=0;
      //       for (int bg = 0; bg < n_bg; bg++)
      //         for (int bk = 0; bk < n_bk; bk++) {
      //           int hf = bk + bg * n_bk + rk * n_bg * n_bk;
      //           int nf = bg * n_bk + bk;
      //           auto& ho = host_mc->get_offload_per_bank(hf);
      //           h_acto+=ho.acto; h_preo+=ho.preo; h_rdo+=ho.rdo; h_wro+=ho.wro;
      //           auto& nc = nma_mc->get_per_bank_cmd(nf);
      //           n_acto+=nc.acto; n_preo+=nc.preo; n_rdo+=nc.rdo; n_wro+=nc.wro;
      //           i_act+=nc.issue_act; i_pre+=nc.issue_pre; i_rd+=nc.issue_rd; i_wr+=nc.issue_wr;
      //         }
      //       printf("\n--- ch=%d rk=%d ---\n", ch, rk);
      //       printf("[Host Offload Total] ACTO=%zu  PREO=%zu  RDO=%zu  WRO=%zu  (sum=%zu)\n",
      //         h_acto, h_preo, h_rdo, h_wro, h_acto+h_preo+h_rdo+h_wro);
      //       printf("[NMA  Recv    Total] ACTO=%zu  PREO=%zu  RDO=%zu  WRO=%zu  (sum=%zu)\n",
      //         n_acto, n_preo, n_rdo, n_wro, n_acto+n_preo+n_rdo+n_wro);
      //       printf("[NMA  Issue   Total] ACT_L=%zu PRE_L=%zu RD_L=%zu WR_L=%zu (sum=%zu)\n",
      //         i_act, i_pre, i_rd, i_wr, i_act+i_pre+i_rd+i_wr);
      //       printf("[NMA  Queue] CMD_FIFO=%d  ret_buf=%d  pend_rd=%d  pend_int=%d\n",
      //         nma_mc->get_cmd_fifo_outstanding(), nma_mc->get_return_buffer_size(),
      //         nma_mc->get_pending_reads_size(), nma_mc->get_pending_interrupts_size());
      //
      //       // Per-bank detail
      //       printf("  BG BK | Host:ACTO PREO  RDO  WRO | NMA_Recv:ACTO PREO  RDO  WRO | CMD_Issue:ACT  PRE   RD   WR | OT CMDQ | OT_inc OT_dec delta | dec_cc1 dec_cc2 dec_cc3\n");
      //       printf("  ------+---------------------------+------------------------------+-------------------------------+---------+---------------------+-----------------------\n");
      //       for (int bg = 0; bg < n_bg; bg++) {
      //         for (int bk = 0; bk < n_bk; bk++) {
      //           int hf = bk + bg * n_bk + rk * n_bg * n_bk;
      //           int nf = bg * n_bk + bk;
      //           auto& ho = host_mc->get_offload_per_bank(hf);
      //           auto& nc = nma_mc->get_per_bank_cmd(nf);
      //           auto& dh = host_mc->get_ot_dec_hist(hf);
      //           int ot = host_mc->get_ot_counter(hf);
      //           int cmdq = (int)nma_mc->get_per_bank_cmd_fifo_size(nf);
      //           printf("  %2d %2d | %5zu %4zu %4zu %4zu | %5zu %5zu %4zu %4zu | %5zu %4zu %4zu %4zu | %2d  %2d  | %6zu %6zu %5zd | %7zu %7zu %7zu\n",
      //             bg, bk,
      //             ho.acto, ho.preo, ho.rdo, ho.wro,
      //             nc.acto, nc.preo, nc.rdo, nc.wro,
      //             nc.issue_act, nc.issue_pre, nc.issue_rd, nc.issue_wr,
      //             ot, cmdq,
      //             ho.ot_inc, ho.ot_dec, (ssize_t)ho.ot_inc - (ssize_t)ho.ot_dec,
      //             dh.hist[1], dh.hist[2], dh.hist[3]);
      //         }
      //       }
      //     }
      //   }
      //   // === Filtered logs: Ch0 Rk2 BG3 BK0 (ALL entries) ===
      //   {
      //     const char* cmd_names[] = {"ACTO", "PREO", "RDO", "WRO"};
      //     auto* host_mc = dynamic_cast<AsyncDIMMHostController*>(m_host_controllers[0]);
      //     if (host_mc) {
      //       // Offload command timeline
      //       auto& cmd_log = host_mc->get_offload_cmd_log();
      //       size_t cnt = 0;
      //       for (auto& e : cmd_log)
      //         if (e.rank == 2 && e.bg == 3 && e.bk == 0) cnt++;
      //       printf("\n[Host MC ch=0 rk=2 BG=3 BK=0] Offload Cmd Log (ALL %zu entries):\n", cnt);
      //       printf("  %10s  %4s\n", "cycle", "cmd");
      //       for (auto& e : cmd_log) {
      //         if (e.rank == 2 && e.bg == 3 && e.bk == 0)
      //           printf("  %10ld  %4s\n", e.clk, cmd_names[e.cmd_type]);
      //       }
      //
      //       // RU pop log
      //       auto& ru_log = host_mc->get_ru_pop_log();
      //       cnt = 0;
      //       for (auto& e : ru_log)
      //         if (e.rank == 2 && e.bg == 3 && e.bk == 0) cnt++;
      //       printf("\n[Host MC ch=0 rk=2 BG=3 BK=0] RU Pop Log (ALL %zu entries):\n", cnt);
      //       printf("  %10s  %3s  %4s\n", "cycle", "cc", "R/W");
      //       for (auto& e : ru_log) {
      //         if (e.rank == 2 && e.bg == 3 && e.bk == 0)
      //           printf("  %10ld  %3d  %4s\n", e.clk, e.cmd_count, e.is_read ? "RD" : "WR");
      //       }
      //     }
      //     // Matching NMA interrupt log for same bank
      //     auto* nma_mc = m_nma_controllers[0][2];
      //     if (nma_mc) {
      //       auto& int_log = nma_mc->get_interrupt_log();
      //       size_t cnt = 0;
      //       for (auto& e : int_log)
      //         if (e.bg == 3 && e.bk == 0) cnt++;
      //       printf("\n[NMA MC ch=0 rk=2 BG=3 BK=0] Interrupt Log (ALL %zu entries):\n", cnt);
      //       printf("  %10s  %6s  %4s\n", "cycle", "seq", "R/W");
      //       for (auto& e : int_log) {
      //         if (e.bg == 3 && e.bk == 0)
      //           printf("  %10ld  %6lu  %4s\n", (long)e.clk, e.seq_num, e.is_read ? "RD" : "WR");
      //       }
      //     }
      //   }
      //
      //   printf("\n============================================\n");
      //   std::exit(0);
      // }

      // [DEBUG]
      // // Debug dump at cycle 900000: Host MC + NMA MC full state per rank, then exit
      // if (false && m_clk == 2900000) {
      //   printf("\n========== BUFFER DEBUG DUMP @ cycle %ld ==========\n", m_clk);
      //   for (int ch = 0; ch < m_num_channels; ch++) {
      //     auto* host_mc = dynamic_cast<AsyncDIMMHostController*>(m_host_controllers[ch]);
      //     if (!host_mc) continue;
      //     int n_rk = host_mc->get_num_ranks();
      //     int n_bg = host_mc->get_num_bankgroups();
      //     int n_bk = host_mc->get_num_banks();
      //
      //     // Global RU cmd_count histogram
      //     {
      //       auto* h = host_mc->get_ru_cmd_count_hist();
      //       printf("[Host MC ch=%d] RU cmd_count histogram: cc=1:%zu  cc=2:%zu  cc=3:%zu  (total=%zu)\n",
      //         ch, h[1], h[2], h[3], h[1]+h[2]+h[3]);
      //     }
      //
      //     for (int rk = 0; rk < n_rk; rk++) {
      //       auto ds = host_mc->get_debug_state(rk);
      //       auto* nma_mc = m_nma_controllers[ch][rk];
      //
      //       printf("\n===== ch=%d rk=%d =====\n", ch, rk);
      //
      //       // --- Host MC summary ---
      //       printf("[Host MC] active_buf=%d  rd_buf=%d  wr_buf=%d  pri_buf=%d\n",
      //         ds.active_buf, ds.rd_buf, ds.wr_buf, ds.pri_buf);
      //       printf("[Host MC] RU_size=%d  RT_pending=%d  OT_sum=%d  pending_depart=%d\n",
      //         ds.ru_size, ds.rt_pending, ds.ot_sum, ds.pending_depart);
      //       printf("[Host MC] Interrupts_received=%zu\n", host_mc->get_interrupt_count(rk));
      //
      //       // --- NMA MC summary ---
      //       printf("[NMA  MC] CMD_FIFO_total=%d  REQ_FIFO_outstanding=%d\n",
      //         nma_mc->get_cmd_fifo_outstanding(), nma_mc->get_nma_outstanding());
      //       printf("[NMA  MC] return_buf=%d  pending_reads=%d  pending_interrupts=%d\n",
      //         nma_mc->get_return_buffer_size(),
      //         nma_mc->get_pending_reads_size(),
      //         nma_mc->get_pending_interrupts_size());
      //       printf("[NMA  MC] Interrupts_sent=%zu\n", nma_mc->get_num_interrupts_sent());
      //
      //       // --- Per-bank detail table ---
      //       printf("  BG BK | Host_Offload:ACTO PREO  RDO  WRO | RU_dec:cc1  cc2  cc3 | OT  | NMA_CMD_Issue:ACT  PRE   RD   WR | CMDQ\n");
      //       printf("  ------+------------------------------------+---------------------+-----+-----------------------------------+-----\n");
      //       for (int bg = 0; bg < n_bg; bg++) {
      //         for (int bk = 0; bk < n_bk; bk++) {
      //           int hf = bk + bg * n_bk + rk * n_bg * n_bk;
      //           int nf = bg * n_bk + bk;
      //           auto& ho = host_mc->get_offload_per_bank(hf);
      //           auto& dh = host_mc->get_ot_dec_hist(hf);
      //           int ot = host_mc->get_ot_counter(hf);
      //           auto& nc = nma_mc->get_per_bank_cmd(nf);
      //           int cmdq = (int)nma_mc->get_per_bank_cmd_fifo_size(nf);
      //           printf("  %2d %2d | %5zu %4zu %4zu %4zu | %4zu %4zu %4zu | %3d | %5zu %4zu %4zu %4zu | %3d\n",
      //             bg, bk,
      //             ho.acto, ho.preo, ho.rdo, ho.wro,
      //             dh.hist[1], dh.hist[2], dh.hist[3],
      //             ot,
      //             nc.issue_act, nc.issue_pre, nc.issue_rd, nc.issue_wr,
      //             cmdq);
      //         }
      //       }
      //     }
      //   }
      //   // === Filtered timeline logs: Ch0 Rk0 BG0 BK0 ===
      //   {
      //     auto* host_mc = dynamic_cast<AsyncDIMMHostController*>(m_host_controllers[0]);
      //     auto* nma_mc = m_nma_controllers[0][0];
      //     if (host_mc && nma_mc) {
      //       const char* offload_names[] = {"ACTO", "PREO", "RDO", "WRO"};
      //       const char* issue_names[] = {"ACT", "PRE", "RD", "WR"};
      //
      //       // Host offload command timeline
      //       auto& cmd_log = host_mc->get_offload_cmd_log();
      //       size_t cnt = 0;
      //       for (auto& e : cmd_log)
      //         if (e.rank == 0 && e.bg == 6 && e.bk == 1) cnt++;
      //       const char* buf_names[] = {"ACT_BUF", "RD_BUF", "WR_BUF"};
      //       printf("\n[Host MC ch=0 rk=0 BG=6 BK=1] Offload Cmd Log (%zu entries):\n", cnt);
      //       printf("  %10s  %4s  %8s  %7s\n", "cycle", "cmd", "row", "buffer");
      //       for (auto& e : cmd_log) {
      //         if (e.rank == 0 && e.bg == 6 && e.bk == 1)
      //           printf("  %10ld  %4s  %8d  %7s\n", e.clk, offload_names[e.cmd_type],
      //             e.row, buf_names[e.buf_type]);
      //       }
      //
      //       // NMA CMD FIFO issue timeline
      //       auto& issue_log = nma_mc->get_cmd_fifo_issue_log();
      //       cnt = 0;
      //       for (auto& e : issue_log)
      //         if (e.bg == 6 && e.bk == 1) cnt++;
      //       printf("\n[NMA MC ch=0 rk=0 BG=6 BK=1] CMD FIFO Issue Log (%zu entries):\n", cnt);
      //       printf("  %10s  %4s\n", "cycle", "cmd");
      //       for (auto& e : issue_log) {
      //         if (e.bg == 6 && e.bk == 1)
      //           printf("  %10ld  %4s\n", (long)e.clk, issue_names[e.cmd_type]);
      //       }
      //
      //       // NMA interrupt log for same bank
      //       auto& int_log = nma_mc->get_interrupt_log();
      //       cnt = 0;
      //       for (auto& e : int_log)
      //         if (e.bg == 6 && e.bk == 1) cnt++;
      //       printf("\n[NMA MC ch=0 rk=0 BG=6 BK=1] Interrupt Log (%zu entries):\n", cnt);
      //       printf("  %10s  %6s  %4s\n", "cycle", "seq", "R/W");
      //       for (auto& e : int_log) {
      //         if (e.bg == 6 && e.bk == 1)
      //           printf("  %10ld  %6lu  %4s\n", (long)e.clk, e.seq_num, e.is_read ? "RD" : "WR");
      //       }
      //     }
      //   }
      //
      //   printf("\n============================================\n");
      //   std::exit(0);
      // }
    };

    float get_tCK() override {
      return m_dram->m_timing_vals("tCK_ps") / 1000.0f;
    }

    bool is_finished() override {
      if (m_host_stall_terminated) return true;

      // Trace-core NMA mode
      if (m_trace_core_enable && m_nma_trace) {
        // Infinite repeat (trace_repeat <= 0): never block termination
        // Frontend cores control simulation lifetime
        if (m_trace_repeat <= 0) return true;
        return m_trace_done && is_ndp_finished();
      }

      // Frontend mode: Host MC completion check
      // But NMA inst-buf/ctrl-reg WRs have rec/iss counter mismatch (bypass path),
      // so also accept NDP-finished as sufficient for termination.
      bool host_done = true;
      for (int i = 0; i < m_num_channels; i++)
        if (!m_host_controllers[i]->is_abs_finished()) { host_done = false; break; }

      return host_done || (m_nma_ever_started && is_ndp_finished());
    }

    bool is_ndp_finished() override {
      if (m_trace_core_enable && m_nma_trace && !m_trace_done) return false;
      for (int ch = 0; ch < m_num_channels; ch++)
        for (int rk = 0; rk < m_num_ranks; rk++)
          if (!m_nma_controllers[ch][rk]->is_nma_idle()) return false;
      return true;
    }

    bool is_host_stall_terminated() override {
      return m_host_stall_terminated;
    }

    virtual void mem_sys_finalize() override {
      size_t total_latency = 0;
      for (int i = 0; i < m_num_channels; i++)
        total_latency += m_host_controllers[i]->get_host_acces_latency();

      s_avg_read_latency = (s_num_read_requests == 0)
        ? -1.0f
        : (float)total_latency / (float)s_num_read_requests;

      // Bandwidth report (AsyncDIMM: Host<->NMA, NMA<->DRAM)
      std::cout << "\n=== Memory System Bandwidth (GB/s) ===\n";
      std::cout << "Assume: 512 bits/access, BW = bytes/ns\n\n";

      int tCK_ps = m_dram->m_timing_vals("tCK_ps");

      // Collect Host MC counters (Host<->NMA side)
      // Layout: [0] main bypass, [1] main offload, [2-4] NMA<->DRAM placeholders,
      //         [5] tcore bypass, [6] tcore offload, [7-9] NMA<->DRAM placeholders
      std::vector<uint64_t> counters(10, 0);
      for (int i = 0; i < m_num_channels; i++) {
        auto c = m_host_controllers[i]->get_counters();
        if (c.size() == 10) {
          for (int ci = 0; ci < 10; ci++) counters[ci] += c[ci];
        }
      }

      // Fill NMA<->DRAM counters from NMA MCs
      // NMA<->DRAM Bypass = Host<->NMA Bypass (same data path)
      counters[2] = counters[0];  // main
      counters[7] = counters[5];  // tcore
      // NMA<->DRAM Offload = CMD FIFO RD+WR (from host frontend → main window)
      // NMA<->DRAM NMA = REQ FIFO RD+WR (from NMA computation → tcore window)
      uint64_t total_cmd_fifo_rw = 0, total_req_fifo_rw = 0;
      for (int ch = 0; ch < m_num_channels; ch++)
        for (int rk = 0; rk < m_num_ranks; rk++) {
          total_cmd_fifo_rw += m_nma_controllers[ch][rk]->get_cmd_fifo_rd()
                             + m_nma_controllers[ch][rk]->get_cmd_fifo_wr();
          total_req_fifo_rw += m_nma_controllers[ch][rk]->get_req_fifo_rd()
                             + m_nma_controllers[ch][rk]->get_req_fifo_wr();
        }
      counters[3] = total_cmd_fifo_rw;  // main NMA<->DRAM Offload
      counters[4] = 0;                  // main NMA<->DRAM NMA (NMA requests are tcore, not main)
      counters[8] = 0;                  // tcore NMA<->DRAM Offload (offloads are from host frontend, not tcore)
      counters[9] = total_req_fifo_rw;  // tcore NMA<->DRAM NMA

      // Separate main window = total - tcore
      // Host<->NMA: counters[0,1] are total, [5,6] are tcore
      // NMA<->DRAM: [2,3,4] and [7,8,9] — but [3](CMD FIFO) and [9](REQ FIFO)
      //   are already exclusive, so just subtract for [0,1] and [2]
      uint64_t m0 = counters[0] - counters[5];  // main bypass
      uint64_t m1 = counters[1] - counters[6];  // main offload
      uint64_t m2 = m0;                          // main NMA<->DRAM bypass = main Host<->NMA bypass
      uint64_t m3 = counters[3];                 // main NMA<->DRAM offload (CMD FIFO, already non-tcore)
      uint64_t m4 = counters[4];                 // main NMA<->DRAM NMA (always 0)

      // ---- Main window (non-tcore only) ----
      std::cout << "[Main window]\n";
      print_bw("Host <-> NMA",
          calc_bw_gbs(m0+m1, 1.0, m_clk, tCK_ps));
      print_bw("Host <-> NMA: Bypass",
          calc_bw_gbs(m0, 1.0, m_clk, tCK_ps));
      print_bw("Host <-> NMA: Offload",
          calc_bw_gbs(m1, 1.0, m_clk, tCK_ps));
      print_bw("NMA <-> DRAM",
          calc_bw_gbs(m2+m3+m4, 1.0, m_clk, tCK_ps));
      print_bw("NMA <-> DRAM: Bypass",
          calc_bw_gbs(m2, 1.0, m_clk, tCK_ps));
      print_bw("NMA <-> DRAM: Offload",
          calc_bw_gbs(m3, 1.0, m_clk, tCK_ps));
      print_bw("NMA <-> DRAM: NMA",
          calc_bw_gbs(m4, 1.0, m_clk, tCK_ps));

      // ---- tcore window ----
      std::cout << "\n[tcore window]\n";
      print_bw("tcore Host <-> NMA",
          calc_bw_gbs(counters[5]+counters[6], 1.0, m_clk, tCK_ps));
      print_bw("tcore Host <-> NMA: Bypass",
          calc_bw_gbs(counters[5], 1.0, m_clk, tCK_ps));
      print_bw("tcore Host <-> NMA: Offload",
          calc_bw_gbs(counters[6], 1.0, m_clk, tCK_ps));
      print_bw("tcore NMA <-> DRAM",
          calc_bw_gbs(counters[7]+counters[8]+counters[9], 1.0, m_clk, tCK_ps));
      print_bw("tcore NMA <-> DRAM: Bypass",
          calc_bw_gbs(counters[7], 1.0, m_clk, tCK_ps));
      print_bw("tcore NMA <-> DRAM: Offload",
          calc_bw_gbs(counters[8], 1.0, m_clk, tCK_ps));
      print_bw("tcore NMA <-> DRAM: NMA",
          calc_bw_gbs(counters[9], 1.0, m_clk, tCK_ps));

      std::cout << std::endl;

      // NMA MC stats
      for (int ch = 0; ch < m_num_channels; ch++)
        for (int rk = 0; rk < m_num_ranks; rk++)
          m_nma_controllers[ch][rk]->print_stats();

      report();

      // FSM sync summary
      if (m_debug_fsm_sync) {
        std::cout << "\n=== AsyncDIMM FSM Sync Verification ===" << std::endl;
        if (m_fsm_sync_errors == 0)
          std::cout << "PASS: FSMs in sync for " << m_clk << " cycles" << std::endl;
        else
          std::cout << "FAIL: " << m_fsm_sync_errors << " FSM sync errors!" << std::endl;
      }

      // Cleanup
      for (int ch = 0; ch < m_num_channels; ch++)
        for (int rk = 0; rk < m_num_ranks; rk++)
          delete m_nma_controllers[ch][rk];
    }

  private:
    // ===== Mode Transition State Machine =====

    void tick_mode_transitions() {
      for (int ch = 0; ch < m_num_channels; ch++) {
        for (int rk = 0; rk < m_num_ranks; rk++) {
          switch (m_rank_state[ch][rk]) {

            case SystemNMAState::HOST_MODE: {
              // Host MC driven mode transition (DBX-DIMM style):
              // Host MC tracks NMAInst write count → issues ctrl-reg WR only after
              // all NMAInst writes are issued → sets mode to NMA/CONCURRENT at issue time.
              // System detects mode change by polling Host MC get_mode().
              auto* host_mc = dynamic_cast<AsyncDIMMHostController*>(m_host_controllers[ch]->m_impl);
              if (host_mc) {
                AsyncDIMMMode hm = host_mc->get_mode(rk);
                if (hm == AsyncDIMMMode::NMA) {
                  m_rank_state[ch][rk] = SystemNMAState::TRANSITIONING_H2N;
                  m_logger->info("[{}] Ch{} Rk{}: Host MC set NMA mode, H2N transition",
                                 m_clk, ch, rk);
                } else if (hm == AsyncDIMMMode::CONCURRENT) {
                  m_rank_state[ch][rk] = SystemNMAState::TRANSITIONING_H2C;
                  m_logger->info("[{}] Ch{} Rk{}: Host MC set CONCURRENT mode, H2C transition",
                                 m_clk, ch, rk);
                }
              }
              break;
            }

            case SystemNMAState::TRANSITIONING_H2N:
              tick_transition_h2n(ch, rk);
              break;

            case SystemNMAState::NMA_MODE:
              tick_nma_mode(ch, rk);
              break;

            case SystemNMAState::TRANSITIONING_N2H:
              tick_transition_n2h(ch, rk);
              break;

            case SystemNMAState::TRANSITIONING_H2C:
              tick_transition_h2c(ch, rk);
              break;

            case SystemNMAState::CONCURRENT:
              tick_concurrent_mode(ch, rk);
              break;

            case SystemNMAState::TRANSITIONING_C2H:
              // C2H is now Host MC driven — this state should not be reached.
              // Fallback: treat as HOST_MODE.
              m_rank_state[ch][rk] = SystemNMAState::HOST_MODE;
              break;
          }
        }
      }
    }

    /**
     * HOST → NMA: Host MC has already set mode to NMA (at ctrl-reg WR issue time).
     * All NMAInst writes are guaranteed delivered. Start NMA MC execution.
     */
    void tick_transition_h2n(int ch, int rk) {
      m_nma_controllers[ch][rk]->start_nma_execution();
      m_nma_ever_started = true;

      m_rank_state[ch][rk] = SystemNMAState::NMA_MODE;
      m_logger->info("[{}] Ch{} Rk{}: H2N complete, entering NMA_MODE", m_clk, ch, rk);
    }

    /**
     * NMA Mode: Monitor NMA MC for completion.
     * NMA outstanding = sum of per-bank REQ FIFO sizes (computed by NMA MC).
     */
    void tick_nma_mode(int ch, int rk) {
      auto* nma_mc = m_nma_controllers[ch][rk];
      if (nma_mc->is_nma_complete()) {
        m_rank_state[ch][rk] = SystemNMAState::TRANSITIONING_N2H;
        m_logger->info("[{}] Ch{} Rk{}: NMA complete (outstanding={}), N2H transition",
                       m_clk, ch, rk, nma_mc->get_nma_outstanding());
      }
    }

    /**
     * NMA → HOST (Implicit Sync N2H):
     * NMA MC has already closed all banks (PREA in NMA_DONE state).
     * Restore Host MC mode to HOST.
     */
    void tick_transition_n2h(int ch, int rk) {
      auto* nma_mc = m_nma_controllers[ch][rk];
      if (!nma_mc->is_nma_idle()) return;

      auto* host_mc = dynamic_cast<AsyncDIMMHostController*>(m_host_controllers[ch]->m_impl);
      if (host_mc)
        host_mc->set_mode(rk, AsyncDIMMMode::HOST);

      m_rank_state[ch][rk] = SystemNMAState::HOST_MODE;
      m_logger->info("[{}] Ch{} Rk{}: N2H complete, returning to HOST_MODE", m_clk, ch, rk);
    }

    // Note: NMA MC's m_nma_start_pending is no longer used for mode transitions.
    // Host MC manages the lifecycle: NMAInst write tracking → start flag → mode change.
    // NMA MC's start_nma_execution() is called directly by the system.

    // ===== Phase 3: Concurrent Mode Transitions =====

    /**
     * HOST → CONCURRENT (H2C):
     * Host MC has already set mode to CONCURRENT (at ctrl-reg WR issue time).
     * Start NMA MC execution and enter concurrent mode.
     */
    void tick_transition_h2c(int ch, int rk) {
      m_nma_controllers[ch][rk]->start_nma_execution();
      m_nma_ever_started = true;
      m_nma_controllers[ch][rk]->enter_concurrent_mode();

      m_rank_state[ch][rk] = SystemNMAState::CONCURRENT;
      m_logger->info("[{}] Ch{} Rk{}: H2C complete, entering CONCURRENT mode", m_clk, ch, rk);
    }

    /**
     * CONCURRENT: Monitor for completion.
     * Done when NMA MC is idle AND all CMD FIFOs drained AND return buffer empty
     * AND Host MC has no pending offloads (RU empty + RT pending empty) for this rank.
     */
    void tick_concurrent_mode(int ch, int rk) {
      auto* host_mc = dynamic_cast<AsyncDIMMHostController*>(m_host_controllers[ch]->m_impl);
      if (!host_mc) return;

      // Host MC drives C2H: polls NMA idle → stops offload → drains RU/RT → sets mode=HOST
      if (host_mc->get_mode(rk) == AsyncDIMMMode::HOST) {
        // Host MC has set mode=HOST. Wait for NMA MC to finish draining CMD FIFO.
        auto* nma_mc = m_nma_controllers[ch][rk];
        if (!nma_mc->is_concurrent_complete()) return;  // CMD FIFO still draining
        nma_mc->exit_concurrent_mode();

        m_rank_state[ch][rk] = SystemNMAState::HOST_MODE;
        m_logger->info("[{}] Ch{} Rk{}: C2H complete (Host MC driven), returning to HOST_MODE", m_clk, ch, rk);

        // Check if ALL ranks across ALL channels are now HOST_MODE
        bool all_host = true;
        for (int c = 0; c < m_num_channels && all_host; c++)
          for (int r = 0; r < m_num_ranks && all_host; r++)
            if (m_rank_state[c][r] != SystemNMAState::HOST_MODE) all_host = false;

        if (all_host) {
          m_logger->info("[{}] === All ranks returned to HOST_MODE ===", m_clk);
          for (int c = 0; c < m_num_channels; c++) {
            auto* hmc = dynamic_cast<AsyncDIMMHostController*>(m_host_controllers[c]->m_impl);
            if (!hmc) continue;
            for (int r = 0; r < m_num_ranks; r++) {
              auto s = hmc->get_debug_state(r);
              m_logger->info("[{}]   Ch{} Rk{}: RU={}, RT_pending={}, OT_sum={}, rd_buf={}, wr_buf={}, active={}, pending={}",
                m_clk, c, r, s.ru_size, s.rt_pending, s.ot_sum,
                s.rd_buf, s.wr_buf, s.active_buf, s.pending_depart);
            }
          }
        }
      }
    }

    // ===== NMA Trace Workload Driver =====

    /**
     * Issue requests from the loaded NMA trace.
     * Magic-path writes (NMAInst / ctrl-reg) are sent as normal WR requests
     * through send() → Host MC → DQ bypass → NMA MC.
     */
    void try_issue_trace_requests() {
      if (m_curr_trace_idx >= m_trace.size() && m_outstanding_reads.empty()) {
        if (m_nma_trace) {
          // NMA trace: wait for all NMAs to return to idle before repeating or finishing.
          if (m_nma_ever_started) {
            bool all_idle = true;
            for (int ch = 0; ch < m_num_channels && all_idle; ch++)
              for (int rk = 0; rk < m_num_ranks && all_idle; rk++)
                if (!m_nma_controllers[ch][rk]->is_nma_idle()) all_idle = false;
            if (all_idle) {
              m_trace_repeat_done++;
              // trace_repeat <= 0 means infinite repeat (until Frontend finishes)
              if (m_trace_repeat > 0 && m_trace_repeat_done >= m_trace_repeat) {
                m_trace_done = true;
              } else {
                // Reset trace and NMA for next iteration
                // N2H already completed, so rank_state=HOST_MODE and Host MC mode=HOST
                m_logger->info("[{}] Trace repeat {}/{} complete, restarting trace",
                               m_clk, m_trace_repeat_done,
                               m_trace_repeat > 0 ? m_trace_repeat : -1);
                m_curr_trace_idx = 0;
                m_next_request_id = 0;
                m_nma_ever_started = false;
                // Clear NMA programs for fresh instruction loading
                for (int ch = 0; ch < m_num_channels; ch++)
                  for (int rk = 0; rk < m_num_ranks; rk++)
                    m_nma_controllers[ch][rk]->clear_program();
                // Reset Host MC NMA write tracking counters
                for (int ch = 0; ch < m_num_channels; ch++) {
                  auto* host_mc = dynamic_cast<AsyncDIMMHostController*>(m_host_controllers[ch]->m_impl);
                  if (host_mc) host_mc->reset_nma_counters();
                }
              }
            }
          }
          return;
        } else {
          m_wait_trace_done++;
        }
        if (m_wait_trace_done > 1000) {
          m_curr_trace_idx = 0;
          m_next_request_id = 0;
          m_wait_trace_done = 0;
        }
      }

      while (m_curr_trace_idx < m_trace.size() &&
             m_outstanding_reads.size() < m_max_outstanding) {

        const Trace& t = m_trace[m_curr_trace_idx];
        if (t.timestamp > m_clk) break;

        Request req = Request(t.addr, t.is_write ? Request::Type::Write : Request::Type::Read);

        if (t.is_write && !t.payload.empty())
          for (auto v : t.payload) req.m_payload.push_back(v);

        int req_id = -1;
        if (!t.is_write) {
          req_id = m_next_request_id++;
          req.callback = [this, req_id](Request& completed_req) {
            this->on_read_complete(req_id, completed_req);
          };
        }

        req.is_trace_core_req = true;
        bool sent = send(req);

        if (sent) {
          if (!t.is_write && req_id >= 0) {
            m_outstanding_reads[req_id] = { (uint64_t)m_clk, t.addr };
          }
          m_curr_trace_idx++;
        } else {
          break;
        }
      }
    }

    void on_read_complete(int request_id, Request& req) {
      m_outstanding_reads.erase(request_id);
    }

    // ===== Trace Loading =====

    void load_trace(const std::string& file_path_str, std::vector<Trace>& trace_vec) {
      fs::path trace_path(file_path_str);
      if (!fs::exists(trace_path))
        throw ConfigurationError("Trace {} does not exist!", file_path_str);

      std::ifstream trace_file(trace_path);
      if (!trace_file.is_open())
        throw ConfigurationError("Trace {} cannot be opened!", file_path_str);

      std::string line;
      size_t line_number = 0;
      uint64_t default_timestamp = 0;

      while (std::getline(trace_file, line)) {
        line_number++;
        if (line.empty() || line[0] == '#') continue;

        Trace t;
        std::vector<std::string> tokens;
        tokenize(tokens, line, " ");
        if (tokens.empty()) continue;

        size_t token_offset = 0;

        if (tokens.size() >= 3 && std::isdigit(tokens[0][0])) {
          t.timestamp = std::stoull(tokens[0]);
          token_offset = 1;
        } else if (tokens.size() >= 2) {
          t.timestamp = default_timestamp++;
          token_offset = 0;
        } else {
          throw ConfigurationError("Trace {} format invalid at line {}!", file_path_str, line_number);
        }

        if (tokens[token_offset] == "LD")
          t.is_write = false;
        else if (tokens[token_offset] == "ST")
          t.is_write = true;
        else
          throw ConfigurationError("Trace {} invalid operation '{}' at line {}!",
                                   file_path_str, tokens[token_offset], line_number);

        std::string addr_str = tokens[token_offset + 1];
        if (addr_str.compare(0, 2, "0x") == 0 || addr_str.compare(0, 2, "0X") == 0)
          t.addr = std::stoll(addr_str.substr(2), nullptr, 16);
        else
          t.addr = std::stoll(addr_str);

        t.is_write = (tokens[token_offset] == "ST");

        if (t.is_write && tokens.size() > token_offset + 2) {
          size_t payload_count = tokens.size() - (token_offset + 2);
          if (payload_count == 8) {
            for (size_t i = 0; i < 8; i++) {
              std::string ps = tokens[token_offset + 2 + i];
              if (ps.compare(0, 2, "0x") == 0 || ps.compare(0, 2, "0X") == 0)
                t.payload.push_back(std::stoull(ps.substr(2), nullptr, 16));
              else
                t.payload.push_back(std::stoull(ps));
            }
          }
        }

        trace_vec.push_back(t);
      }
      trace_file.close();
    }

    static void tokenize(std::vector<std::string>& tokens, const std::string& str,
                         const std::string& delim) {
      size_t start = 0;
      size_t end = str.find(delim, start);
      while (end != std::string::npos) {
        if (end > start) tokens.push_back(str.substr(start, end - start));
        start = end + delim.length();
        end = str.find(delim, start);
      }
      if (start < str.length()) tokens.push_back(str.substr(start));
    }

    // ===== FSM Sync Verification =====

    void verify_fsm_sync() {
      for (int ch = 0; ch < m_num_channels; ch++) {
        auto* host_mc = dynamic_cast<AsyncDIMMHostController*>(m_host_controllers[ch]->m_impl);
        if (!host_mc) continue;

        for (int rk = 0; rk < m_num_ranks; rk++) {
          if (m_rank_state[ch][rk] != SystemNMAState::HOST_MODE) continue;

          auto* nma_mc = m_nma_controllers[ch][rk];
          for (int bg = 0; bg < m_num_bankgroups; bg++) {
            for (int bk = 0; bk < m_num_banks; bk++) {
              int host_open_row = host_mc->get_open_row(rk, bg, bk);
              int nma_open_row  = nma_mc->get_open_row(bg, bk);
              auto nma_state    = nma_mc->get_bank_state(bg, bk);

              if (nma_state == AsyncDIMMNMAController::BankState::REFRESHING) continue;

              bool host_closed = (host_open_row == -1);
              bool nma_closed  = (nma_state == AsyncDIMMNMAController::BankState::CLOSED);

              if (host_closed != nma_closed) {
                m_fsm_sync_errors++;
                m_logger->error("FSM SYNC ERROR @ clk {}: Ch{} Rk{} BG{} BK{} "
                  "Host={} NMA={}", m_clk, ch, rk, bg, bk,
                  host_closed ? "CLOSED" : "OPENED",
                  nma_closed  ? "CLOSED" : "OPENED");
              } else if (!host_closed && host_open_row != nma_open_row) {
                m_fsm_sync_errors++;
                m_logger->error("FSM SYNC ERROR @ clk {}: Ch{} Rk{} BG{} BK{} "
                  "Both OPENED but row mismatch: Host={} NMA={}",
                  m_clk, ch, rk, bg, bk, host_open_row, nma_open_row);
              }
            }
          }
        }
      }
    }

    // ===== Utility =====

    static inline double calc_bw_gbs(uint64_t acc_cnt, double dq_scaling,
                                     uint64_t clk_cycles, uint64_t tCK_ps) {
      if (clk_cycles == 0 || tCK_ps == 0) return 0.0;
      const double bytes = (static_cast<double>(acc_cnt) * 512.0 * dq_scaling) / 8.0;
      const double time_ns = static_cast<double>(clk_cycles) * (static_cast<double>(tCK_ps) / 1000.0);
      return bytes / time_ns;
    }

    static inline void print_bw(const std::string& name, double bw) {
      std::cout << std::left << std::setw(32) << name
                << " : " << std::right << std::setw(10)
                << std::fixed << std::setprecision(3) << bw << " GB/s\n";
    }

    inline void record_latency(uint64_t lat) {
      ++total_reads_;
      sum_lat_ += lat;
      max_lat_ = std::max(max_lat_, lat);
      const uint64_t idx = lat / BIN_WIDTH;
      if (idx < NUM_BINS - 1) hist_[idx] += 1;
      else { hist_[NUM_BINS - 1] += 1; ++overflow_; }
    }

    uint64_t percentile_from_hist(double p) const {
      const uint64_t total = total_reads_;
      const uint64_t target = (uint64_t)std::ceil(p * (double)total);
      uint64_t cdf = 0;
      for (uint32_t i = 0; i < NUM_BINS; ++i) {
        cdf += hist_[i];
        if (cdf >= target) {
          if (i == NUM_BINS - 1) return MAX_LAT;
          return (uint64_t)i * BIN_WIDTH;
        }
      }
      return MAX_LAT;
    }

    void report() const {
      const uint64_t total = total_reads_;
      std::cout << "\n=== AsyncDIMM Read Latency Report ===\n";
      std::cout << "READ latency samples: " << total << "\n";
      if (total == 0) return;
      std::cout << "  avg: "  << (double)sum_lat_ / (double)total << " cycles\n";
      std::cout << "  p50: "  << percentile_from_hist(0.50)  << " cycles\n";
      std::cout << "  p95: "  << percentile_from_hist(0.95)  << " cycles\n";
      std::cout << "  p99: "  << percentile_from_hist(0.99)  << " cycles\n";
      std::cout << "  p999: " << percentile_from_hist(0.999) << " cycles\n";
      std::cout << "  max: "  << max_lat_ << " cycles\n";
    }
};

}   // namespace Ramulator
