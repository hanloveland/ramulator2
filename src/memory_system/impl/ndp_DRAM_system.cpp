#include "memory_system/memory_system.h"
#include "translation/translation.h"
#include "dram_controller/controller.h"
#include "addr_mapper/addr_mapper.h"
#include "dram/dram.h"
#include <filesystem>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <array>
#include <climits>

// #define NDP_DEBUG
#define PCH_DEBUG

#ifdef NDP_DEBUG
#define DEBUG_PRINT(clk, unit_str, dimm_id, pch, msg) do { std::cout <<"["<<clk<<"]["<<unit_str<<"] DIMM["<<dimm_id<<"] PCH["<<pch<<"] "<<msg<<std::endl; } while(0)
#else
#define DEBUG_PRINT(clk, unit_str, ch, pch, msg) do {} while(0)
#endif


namespace Ramulator {

namespace fs = std::filesystem;

class NDPDRAMSystem final : public IMemorySystem, public Implementation {
  RAMULATOR_REGISTER_IMPLEMENTATION(IMemorySystem, NDPDRAMSystem, "ndpDRAM", "A NDP-Supported DRAM-based memory system.");

  protected:
    Clk_t m_clk = 0;
    IDRAM*  m_dram;
    IAddrMapper*  m_addr_mapper;
    std::vector<IDRAMController*> m_controllers;    

  public:
    size_t s_num_read_requests = 0;
    size_t s_num_write_requests = 0;
    size_t s_num_other_requests = 0;
    size_t s_num_ndp_read_requests = 0;
    size_t s_num_ndp_write_requests = 0;
    size_t s_num_ndp_other_requests = 0;
    float s_avg_read_latency = 0; // Only Access for Normal 

    // NDP Controller
    enum NDP_CTRL_STATUS {
      NDP_IDLE,
      NDP_ISSUE_START,
      NDP_BEFORE_RUN,
      NDP_RUN,
      NDP_BAR,
      NDP_WAIT,
      NDP_FETCH_STALL,
      NDP_DONE
    };

    // std::unordered_map<int, std::string> ndp_ctrl_status_to_str;

    // NDP Access Instruction Opcode
    std::map<std::string, int> m_ndp_access_inst_op = {
      {"RD",         0},
      {"WR",         1},
      {"BAR",        2},
      {"WAIT_RES",   3},
      {"LOOP_START", 4},
      {"LOOP_END",   5},
      {"WAIT",       6},
      {"SET_BASE",   8},
      {"INC_BASE",   9},
      {"SET_LOOP",  10},
      {"LOOP",      11},
      {"DONE",       15}
    };

    enum NDP_Op {
        NDP_ACC_OP_RD = 0,
        NDP_ACC_OP_WR = 1,
        NDP_ACC_OP_BAR = 2,
        NDP_ACC_OP_WAIT_RES = 3,
        NDP_ACC_OP_LOOP_START = 4,
        NDP_ACC_OP_LOOP_END = 5,
        NDP_ACC_OP_WAIT = 6,
        NDP_ACC_OP_SET_BASE = 8,
        NDP_ACC_OP_INC_BASE = 9,
        NDP_ACC_OP_SET_LOOP = 10,
        NDP_ACC_OP_LOOP = 11,
        NDP_ACC_OP_DONE = 15
    };

    std::unordered_map<int, std::string> ndp_ctrl_status_to_str = {
      {NDP_IDLE, "NDP_IDLE"},
      {NDP_ISSUE_START, "NDP_ISSUE_START"},
      {NDP_BEFORE_RUN, "NDP_BEFORE_RUN"},
      {NDP_RUN, "NDP_RUN"},
      {NDP_BAR, "NDP_BAR"},
      {NDP_WAIT, "NDP_WAIT"},
      {NDP_FETCH_STALL, "NDP_FETCH_STALL"},
      {NDP_DONE, "NDP_DONE"}
    };

    std::unordered_map<int, std::string> op_to_str = {
      {NDP_ACC_OP_RD, "RD"},
      {NDP_ACC_OP_WR, "WR"},
      {NDP_ACC_OP_BAR, "BAR"},
      {NDP_ACC_OP_WAIT_RES, "WAIT_RES"},
      {NDP_ACC_OP_LOOP_START, "LOOP_START"},
      {NDP_ACC_OP_LOOP_END, "LOOP_END"},
      {NDP_ACC_OP_WAIT, "WAIT"},
      {NDP_ACC_OP_SET_BASE, "SET_BASE"},
      {NDP_ACC_OP_INC_BASE, "INC_BASE"},
      {NDP_ACC_OP_SET_LOOP, "SET_LOOP"},
      {NDP_ACC_OP_LOOP, "LOOP"},
      {NDP_ACC_OP_DONE, "DONE"}
    };    


    // static const std::unordered_map<int, std::string> op_to_str;

    // desc_store: HSNC 내부 backing array [dimm][pch][col(0~127)][entry(0~7)]
    std::vector<std::vector<std::array<std::array<uint64_t, 8>, 128>>>  desc_store;

    // Descriptor Cache: pch_lvl_inst_buf[dimm][pch][group(0~7)][entry(0~7)]
    std::vector<std::vector<std::array<std::array<uint64_t, 8>, 8>>>    pch_lvl_inst_buf;
    // Cache Tag per entry group = column address (7b), -1 if invalid
    std::vector<std::vector<std::array<int, 8>>>                         pch_lvl_inst_buf_tag;
    // Cache valid per entry group
    std::vector<std::vector<std::array<bool, 8>>>                        pch_lvl_inst_buf_valid;
    // LRU counter per entry group (higher = more recently used)
    std::vector<std::vector<std::array<int, 8>>>                         pch_lvl_inst_buf_lru;

    // Base Address Register: 8 × 18b per PCH (for Undirect mode)
    // effective_row = base_reg[row[17:15]] + row[14:0]
    std::vector<std::vector<std::array<int, 8>>>                         hsnc_base_reg;
    // Loop Counter Register: 8 × 16b per PCH
    // SET_LOOP: loop_cnt_reg[cnt_reg] = count
    // LOOP: if loop_cnt_reg[cnt_reg] > 0: cnt--, PC = jump_pc
    std::vector<std::vector<std::array<int, 8>>>                         hsnc_loop_cnt_reg;

    // PC (10-bit): PC[9:3]=col_addr, PC[2:0]=entry_idx
    std::vector<std::vector<int>>                                        pch_lvl_pc;
    // desc_count per PCH (from NDP Start payload)
    std::vector<std::vector<int>>                                        pch_desc_count;
    // Fetch stall flag (Descriptor Cache miss pending)
    std::vector<std::vector<bool>>                                       pch_fetch_stall;
    // Fetch stall target column (for DRAM RD response matching)
    std::vector<std::vector<int>>                                        pch_fetch_stall_col;
    // Fetch complete flag (set by DRAM RD callback)
    std::vector<std::vector<bool>>                                       pch_fetch_complete;

    // Descriptor Cache statistics
    std::vector<std::vector<uint64_t>>  desc_cache_hit_cnt;      // Cache hit count per PCH
    std::vector<std::vector<uint64_t>>  desc_cache_miss_cnt;     // Cache miss count per PCH
    std::vector<std::vector<uint64_t>>  desc_cache_evict_cnt;    // Cache eviction count per PCH

    // PCH-level Status Reg pch_lvl_hsnc_status[DIMM_ID][PCH_ID]
    std::vector<std::vector<NDP_CTRL_STATUS>>             pch_lvl_hsnc_status;
    std::vector<std::vector<bool>>                        pch_lvl_polling;
    // PCH-level HSNC Memory Request Generator Slot[DIMM_ID][PCH_ID][SLOT_IDX]
    std::vector<std::vector<std::vector<AccInst_Slot>>>   pch_lvl_hsnc_nl_addr_gen_slot;
    int                                                   pch_lvl_hsnc_nl_addr_gen_slot_max;

    // PCH-level HSNC Memory Request Generator Round Robin Index [DIMM_ID][PCH_ID]
    std::vector<std::vector<int>>                         pch_lvl_hsnc_nl_addr_gen_slot_rr_idx;

    // PCH-level HSNC Fixed Latency Wait [DIMM_ID][PCH_ID]
    std::vector<std::vector<int>>                         pch_lvl_hsnc_nl_addr_gen_wait_cnt;
    std::vector<std::vector<int>>                         pch_lvl_hsnc_nl_addr_gen_wait_cycle;
    std::vector<std::vector<int>>                         pch_lvl_hsnc_nl_addr_empty_cnt;
    std::vector<std::vector<int>>                         pch_lvl_hsnc_nl_addr_wait_cnt;

    static constexpr int NUM_NDP_STATES = 8;  // NDP_IDLE ~ NDP_DONE (includes NDP_BEFORE_RUN)
    std::vector<std::vector<std::vector<size_t>>>         pch_hsnc_status_cnt;

    bool                                                  all_ndp_idle;


    // HSNC Segment Tracking (per DIMM × pCH)
    // Tracks cycle counts for each RUN→BAR segment from the host-side NDP controller
    enum class HsncSegType { RD, WR, WAIT, UNKNOWN };
    struct HsncSegment {
      int         seg_id;
      HsncSegType type;
      Clk_t       run_start;     // cycle when NDP_RUN entered (fetch phase begins)
      Clk_t       drain_start;   // cycle when NDP_BAR/NDP_WAIT/NDP_DONE entered (drain/wait phase)
      Clk_t       end;           // cycle when drain complete (BAR→RUN or DONE→IDLE)
    };
    // Completed segments: hsnc_segments[dimm_id][pch_id]
    std::vector<std::vector<std::vector<HsncSegment>>>    hsnc_segments;
    // Current segment being tracked: hsnc_cur_seg[dimm_id][pch_id]
    std::vector<std::vector<HsncSegment>>                 hsnc_cur_seg;
    // Whether tracking is active: hsnc_seg_tracking[dimm_id][pch_id]
    std::vector<std::vector<bool>>                        hsnc_seg_tracking;
    // Segment counter: hsnc_seg_cnt[dimm_id][pch_id]
    std::vector<std::vector<int>>                         hsnc_seg_cnt;

    static const char* hsnc_seg_type_str(HsncSegType t) {
      switch(t) {
        case HsncSegType::RD:      return "RD";
        case HsncSegType::WR:      return "WR";
        case HsncSegType::WAIT:    return "WAIT";
        default:                   return "UNKNOWN";
      }
    }

    #ifdef PCH_DEBUG
    std::vector<std::vector<std::vector<AccInst_Slot>>>   pch_lvl_history;
    int                                                   pch_lvl_history_max;
    #endif
    /* Deprecated variable
    std::vector<AccInst_Slot>          ndp_access_infos;
    NDP_CTRL_STATUS                    ndp_ctrl_status;
    int                                ndp_ctrl_pc;
    std::vector<AccInst_Slot>          ndp_access_inst_slots;
    int                                ndp_access_slot_idx;
    std::vector<bool>                  issue_ndp_start;
    */

    // Address Variable
    int ndp_ctrl_row = 0;
    int ndp_ctrl_bk = 0;
    int ndp_ctrl_bg = 0;
    int ndp_ctrl_buf_bk = 0;
    int ndp_ctrl_buf_bg = 0;
    int db_ndp_ctrl_access_bk = 0;
    int db_ndp_ctrl_access_bg = 0;
    int db_ndp_ins_mem_access_bk = 0;
    int db_ndp_ins_mem_access_bg = 0;
    int db_ndp_dat_mem_access_bk = 0;
    int db_ndp_dat_mem_access_bg = 0;
    int row_addr_idx = 0;
    int bk_addr_idx = 0;
    int bg_addr_idx = 0;
    int col_addr_idx = 0;
    int ch_addr_idx = 0;
    /*
    bool ndp_on = false;
    bool wait_ndp_on = false;
    bool ndp_issued_start = false;
    */
    int m_num_dimm = -1;
    int m_num_subch = 2;
    int num_channels = -1;
    int num_pseudochannel = -1;

    /*
      Gem5 Simulation with Trace Core (Virtual Core)
    */
    bool m_host_access = false;
    bool m_trace_core_enable = false;    
    bool m_ndp_trace = true;
    // Copy Structure within loadstore_ncore_trace.cpp
    struct Trace {
      uint64_t timestamp;  // When this request should be issued (in cycles)
      bool is_write;
      Addr_t addr;
      std::vector<uint64_t> payload;
    };
    size_t m_trace_core_mshr_size;
    size_t m_curr_trace_idx;
    std::vector<Trace> m_trace;
    
    int m_next_request_id = 0;
    bool m_debug_mode = false;
    int m_wait_trace_done = 0;
    
    // Outstanding requests tracking
    struct OutstandingRequest {
      uint64_t issue_time;
      Addr_t addr;
    };

    size_t m_max_outstanding;
    std::unordered_map<int, OutstandingRequest> m_outstanding_reads;

    // Latency Tracking
    static constexpr uint32_t BIN_WIDTH = 1;           // 1 cycle per bin
    static constexpr uint32_t MAX_LAT   = 2000000;     // adjust for your sim
    static constexpr uint32_t NUM_BINS  = (MAX_LAT / BIN_WIDTH) + 2;    

    // Histograms
    std::array<uint64_t, NUM_BINS> hist_{};      // latency
    std::array<uint64_t, NUM_BINS> q_hist_{};    // queueing (optional)
    std::array<uint64_t, NUM_BINS> s_hist_{};    // service  (optional)

    // Aggregates
    uint64_t total_reads_ = 0;
    uint64_t sum_lat_     = 0;
    uint64_t max_lat_     = 0;
    uint64_t overflow_    = 0;

    uint64_t read_latency;    

  public:
    void init() override { 
      Logger_t m_logger;

      m_logger = Logging::create_logger("NDPDRAMSystem");
      m_logger->info("DRAM_System init()");
    
      // Create device (a top-level node wrapping all channel nodes)
      m_dram = create_child_ifce<IDRAM>();
      m_addr_mapper = create_child_ifce<IAddrMapper>();

      num_channels = m_dram->get_level_size("channel");   
      int num_ranks = m_dram->get_level_size("rank");   
      num_pseudochannel = m_dram->get_level_size("pseudochannel");   
      m_num_dimm   = num_channels / 2;

      if(num_pseudochannel == -1 ) num_pseudochannel = 1;
      
      // Calcuate Total Memory Capacity
      int num_dram_die = num_channels * num_pseudochannel * num_ranks * (m_dram->m_channel_width / m_dram->m_organization.dq);
      total_memory_capacity = (num_dram_die * m_dram->m_organization.density / 1024)/ 8;
      m_logger->info(" DRAM System Configuration");
      m_logger->info("   - # of Channels          : {}",num_channels);
      m_logger->info("   - # of DIMMs             : {}",m_num_dimm);
      m_logger->info("   - # of Pseudo Channels   : {}",num_pseudochannel);
      m_logger->info("   - # of Ranks             : {}",num_ranks);
      m_logger->info("   - DQs per DRAM Die       : {}",m_dram->m_organization.dq);
      m_logger->info("   - DQs per Channel        : {}",m_dram->m_channel_width);
      m_logger->info("   - DRAM die density (Mb)  : {}",m_dram->m_organization.density);
      m_logger->info("   - Total DRAM Dies        : {}",num_dram_die);
      m_logger->info("   - Total DRAM Capacity(GB): {}",total_memory_capacity);

      // NDP Controller Address
      ndp_ctrl_row = (m_dram->get_level_size("row") - 1);
      if(m_dram->m_ndp_scaling == 1) {
        // x4 DRAM
        ndp_ctrl_bk     = 3;
        ndp_ctrl_bg     = 7;
        ndp_ctrl_buf_bk = 3;
        ndp_ctrl_buf_bg = 6;
        // x4 DRAM with 8BG and 4 BK per BG
        db_ndp_ctrl_access_bk      = 3;
        db_ndp_ctrl_access_bg      = 5;
        db_ndp_ins_mem_access_bk   = 3;
        db_ndp_ins_mem_access_bg   = 4;
        db_ndp_dat_mem_access_bk   = 3;
        db_ndp_dat_mem_access_bg   = 3; // BG0-BG3        
      } else if(m_dram->m_ndp_scaling == 2) {
        // x8 DRAM
        ndp_ctrl_bk     = 3;
        ndp_ctrl_bg     = 7;
        ndp_ctrl_buf_bk = 3;
        ndp_ctrl_buf_bg = 6;
        // x8 DRAM with 8BG and 4 BK per BG
        db_ndp_ctrl_access_bk      = 3;
        db_ndp_ctrl_access_bg      = 5;
        db_ndp_ins_mem_access_bk   = 3;
        db_ndp_ins_mem_access_bg   = 4;
        db_ndp_dat_mem_access_bk   = 3; // BK2-BK3
        db_ndp_dat_mem_access_bg   = 3; // BG0-BG3        
      } else if(m_dram->m_ndp_scaling == 4) {
        // x16 DRAM
        ndp_ctrl_bk     = 3;
        ndp_ctrl_bg     = 3;
        ndp_ctrl_buf_bk = 3;
        ndp_ctrl_buf_bg = 2;
        // x16 DRAM with 4BG and 4 BK per BG
        db_ndp_ctrl_access_bk      = 3;
        db_ndp_ctrl_access_bg      = 1;
        db_ndp_ins_mem_access_bk   = 3;
        db_ndp_ins_mem_access_bg   = 0;
        db_ndp_dat_mem_access_bk   = 2; // BK1-BK2
        db_ndp_dat_mem_access_bg   = 3; // BG0-BG3
      } else {
        throw std::runtime_error("Wrong NDP Scaling Factor!!!");
      }
      if(m_dram->m_ndp_scaling == 4) {
        ndp_ctrl_bk     = 3;
        ndp_ctrl_bg     = 3;
        ndp_ctrl_buf_bk = 3;
        ndp_ctrl_buf_bg = 2;
        //x16 DRAM with 4BG and 4BK per BG
        db_ndp_ctrl_access_bk      = 3;
        db_ndp_ctrl_access_bg      = 1;
        db_ndp_ins_mem_access_bk   = 3;
        db_ndp_ins_mem_access_bg   = 0;
        db_ndp_dat_mem_access_bk   = 2;
        db_ndp_dat_mem_access_bg   = 3; // BG0-BG3
      } else {
      }

      m_logger->info(" NDP Address Configuration");
      m_logger->info("  - Address Space (Row) of NDP Unit: {}",ndp_ctrl_row);
      m_logger->info("  - Address Space (BK) of Control Reg of Host-Side NPU Ctrl: {}",ndp_ctrl_bk);
      m_logger->info("  - Address Space (BG) of Control Reg of Host-Side NPU Ctrl: {}",ndp_ctrl_bg);
      m_logger->info("  - Address Space (BK) of Access Info Buf of Host-Side NPU Ctrl: {}",ndp_ctrl_buf_bk);
      m_logger->info("  - Address Space (BG) of Access Info Buf of Host-Side NPU Ctrl: {}",ndp_ctrl_buf_bg);    
      row_addr_idx  = m_dram->m_levels("row");
      bg_addr_idx   = m_dram->m_levels("bankgroup");
      bk_addr_idx   = m_dram->m_levels("bank");
      col_addr_idx  = m_dram->m_levels("column");
      ch_addr_idx   = m_dram->m_levels("channel");
      // deprecated code
      /*
      issue_ndp_start.resize(num_channels*num_pseudochannel,false);
      ndp_access_infos.resize(8*128,AccInst_Slot());
      ndp_ctrl_status = NDP_IDLE;
      ndp_ctrl_pc     = -1;
      */
      // Create memory controllers
      for (int i = 0; i < num_channels; i++) {
        IDRAMController* controller = create_child_ifce<IDRAMController>();
        controller->m_impl->set_id(fmt::format("Channel {}", i));
        controller->m_channel_id = i;
        m_controllers.push_back(controller);
      }

      if(num_channels % 2 != 0) {
        throw std::runtime_error("The number of channels must be even number!!!");
      }
      m_clock_ratio = param<uint>("clock_ratio").required();

      register_stat(m_clk).name("memory_system_cycles");
      register_stat(s_num_read_requests).name("total_num_read_requests");
      register_stat(s_num_write_requests).name("total_num_write_requests");
      register_stat(s_num_other_requests).name("total_num_other_requests");
      register_stat(s_num_ndp_read_requests).name("total_num_ndp_read_requests");
      register_stat(s_num_ndp_write_requests).name("total_num_ndp_write_requests");
      register_stat(s_num_ndp_other_requests).name("total_num_ndp_other_requests");      
      register_stat(s_avg_read_latency).name("avg_host_read_latency");
      /*
        Each DIMM has 2 Channel (Ramulator 2 Channel --> One DIMM and Two Sub-Channel)
        The number of channels are must 2x 
        Each Host-Side NDP Controller (HSNC) has 8KB Buffer (128 x 64B) (DIMM-level HSNC)
        Channel 0-1: DIMM0
        Channel 2-3: DIMM1
        Chnnael 4-5: DIMM2
        ...
        DIMM (channel/2) : NDP Launch Request (NL-REQ) Buffer (8KB)
          - Subchannel0 (channel%2==0)
            |- pseudo channel 0 : HSNC-PCH0 (PCH-level HSNC)
            |- pseudo channel 1 : HSNC-PCH1 (PCH-level HSNC)
            |- pseudo channel 2 : HSNC-PCH2 (PCH-level HSNC)
            |- pseudo channel 3 : HSNC-PCH3 (PCH-level HSNC)
          - Subchannel1 (channel%2==1)
            |- pseudo channel 0 : HSNC-PCH0 (PCH-level HSNC)
            |- pseudo channel 1 : HSNC-PCH1 (PCH-level HSNC)
            |- pseudo channel 2 : HSNC-PCH2 (PCH-level HSNC)
            |- pseudo channel 3 : HSNC-PCH3 (PCH-level HSNC)

        Host writes AccInst to desc_store via Write Intercept (send_ndp_ctrl)
        Host writes NDP Start (Control Register) to trigger execution

        tick()
        loop all pch ()
          - NDP_IDLE: Do Nothing
          - NDP_ISSUE_START: Wait DRAM WR timing → NDP_RUN
          - NDP_RUN: PC-based fetch from Descriptor Cache → decode → addr_gen_slot
          - NDP_BAR: drain all outstanding requests
          - NDP_WAIT: wait specific latency (polling NDP unit)
          - NDP_FETCH_STALL: Descriptor Cache miss, wait DRAM RD response
          - NDP_DONE: drain remaining → NDP_IDLE

        Each pch-lvl HSNC
          - desc_store[128][8] (backing array, all descriptors)
          - pch_lvl_inst_buf[8][8] (Descriptor Cache, 64 entries)
          - PC (10-bit) → fetch from cache → decode → addr_gen_slot (x8)
          - base_reg[8] (18b, Undirect mode row base address)
          - loop_cnt_reg[8] (16b, LOOP iteration counter)
          - wait counter/cycle (wait self-execution mode on NDP unit)
    */       
      all_ndp_idle = true;

      int total_pch = m_num_subch * num_pseudochannel;

      // desc_store: HSNC 내부 backing array (128 cols × 8 entries = 1024 descriptors per PCH)
      desc_store.resize(m_num_dimm);
      for (int d = 0; d < m_num_dimm; d++) {
        desc_store[d].resize(total_pch);
        for (int p = 0; p < total_pch; p++) {
          for (auto& col : desc_store[d][p]) col.fill(0);
        }
      }

      // Descriptor Cache (8 entry groups × 8 entries = 64 entries per PCH)
      pch_lvl_inst_buf.resize(m_num_dimm);
      pch_lvl_inst_buf_tag.resize(m_num_dimm);
      pch_lvl_inst_buf_valid.resize(m_num_dimm);
      pch_lvl_inst_buf_lru.resize(m_num_dimm);
      for (int d = 0; d < m_num_dimm; d++) {
        pch_lvl_inst_buf[d].resize(total_pch);
        pch_lvl_inst_buf_tag[d].resize(total_pch);
        pch_lvl_inst_buf_valid[d].resize(total_pch);
        pch_lvl_inst_buf_lru[d].resize(total_pch);
        for (int p = 0; p < total_pch; p++) {
          for (auto& grp : pch_lvl_inst_buf[d][p]) grp.fill(0);
          pch_lvl_inst_buf_tag[d][p].fill(-1);
          pch_lvl_inst_buf_valid[d][p].fill(false);
          pch_lvl_inst_buf_lru[d][p].fill(0);
        }
      }

      // HSNC Register File: base_reg[8] + loop_cnt_reg[8] per PCH
      hsnc_base_reg.resize(m_num_dimm);
      hsnc_loop_cnt_reg.resize(m_num_dimm);
      for (int d = 0; d < m_num_dimm; d++) {
        hsnc_base_reg[d].resize(total_pch);
        hsnc_loop_cnt_reg[d].resize(total_pch);
        for (int p = 0; p < total_pch; p++) {
          hsnc_base_reg[d][p].fill(0);
          hsnc_loop_cnt_reg[d][p].fill(0);
        }
      }

      // PC, desc_count, fetch_stall
      pch_lvl_pc.resize(m_num_dimm, std::vector<int>(total_pch, 0));
      pch_desc_count.resize(m_num_dimm, std::vector<int>(total_pch, 0));
      pch_fetch_stall.resize(m_num_dimm, std::vector<bool>(total_pch, false));
      pch_fetch_stall_col.resize(m_num_dimm, std::vector<int>(total_pch, -1));
      pch_fetch_complete.resize(m_num_dimm, std::vector<bool>(total_pch, false));
      desc_cache_hit_cnt.resize(m_num_dimm, std::vector<uint64_t>(total_pch, 0));
      desc_cache_miss_cnt.resize(m_num_dimm, std::vector<uint64_t>(total_pch, 0));
      desc_cache_evict_cnt.resize(m_num_dimm, std::vector<uint64_t>(total_pch, 0));

      // PCH_level Host-Side NDP Controller
      pch_lvl_hsnc_status.resize(m_num_dimm, std::vector<NDP_CTRL_STATUS>(total_pch, NDP_IDLE));
      pch_lvl_polling.resize(m_num_dimm, std::vector<bool>(total_pch, false));

      // nl-req address generator at each pch
      pch_lvl_hsnc_nl_addr_gen_slot_max = 8;
      pch_lvl_hsnc_nl_addr_gen_slot.resize(m_num_dimm, std::vector<std::vector<AccInst_Slot>>(total_pch, std::vector<AccInst_Slot>(0, AccInst_Slot())));

      // Round-Robin Index
      pch_lvl_hsnc_nl_addr_gen_slot_rr_idx.resize(m_num_dimm, std::vector<int>(total_pch, 0));

      // Address Generator Wait Counter and Wait Cycle
      pch_lvl_hsnc_nl_addr_gen_wait_cnt.resize(m_num_dimm, std::vector<int>(total_pch, -1));
      pch_lvl_hsnc_nl_addr_gen_wait_cycle.resize(m_num_dimm, std::vector<int>(total_pch, -1));
      pch_lvl_hsnc_nl_addr_empty_cnt.resize(m_num_dimm, std::vector<int>(total_pch, 0));
      pch_lvl_hsnc_nl_addr_wait_cnt.resize(m_num_dimm, std::vector<int>(total_pch, 0));
      pch_hsnc_status_cnt.resize(m_num_dimm, std::vector<std::vector<size_t>>(total_pch, std::vector<size_t>(NUM_NDP_STATES, 0)));

      // HSNC Segment Tracking init
      hsnc_segments.resize(m_num_dimm, std::vector<std::vector<HsncSegment>>(total_pch));
      hsnc_cur_seg.resize(m_num_dimm, std::vector<HsncSegment>(total_pch, {0, HsncSegType::UNKNOWN, 0, 0, 0}));
      hsnc_seg_tracking.resize(m_num_dimm, std::vector<bool>(total_pch, false));
      hsnc_seg_cnt.resize(m_num_dimm, std::vector<int>(total_pch, 0));

      #ifdef PCH_DEBUG
      pch_lvl_history.resize(m_num_dimm, std::vector<std::vector<AccInst_Slot>>(total_pch, std::vector<AccInst_Slot>(0, AccInst_Slot())));
      pch_lvl_history_max = 32;
      #endif

      // NDP Trace Initialization
      m_trace_core_enable = param<bool>("trace_core_enable").desc("Enable Trace Simulation Core with Gem5").default_val(false);
      
      if(m_trace_core_enable) {
        m_ndp_trace = param<bool>("trace_ndp_type").desc("Trace Type for Trace Simulation Core").default_val(false);
        m_logger->info(" Enable Trace Core Simulation with Gem5 frontend");                            
        if(m_ndp_trace) m_logger->info("  -- Trace Type: NDP Workload");
        else            m_logger->info("  -- Trace Type: Host Access Workload");

        // Set Trace Simulation Core MSHR SIze
        m_trace_core_mshr_size = param<size_t>("trace_core_mshr_size").desc("MSHR size for Trace_Core").default_val(16);
        m_logger->info("  -- Trace Core MSHR Size: {}",m_trace_core_mshr_size);  
        m_max_outstanding = m_trace_core_mshr_size;
        
        // Load Trace File
        std::string trace_path = param<std::string>("trace_path").desc("Trace file path for Trace Core").required();
        m_logger->info("  -- Trace Path: {}",trace_path);  
        load_trace(trace_path,m_trace);
        m_logger->info("  -- Loaded {} trace lines",m_trace.size());        

      }                                      
    };

    void setup(IFrontEnd* frontend, IMemorySystem* memory_system) override { }

    bool send(Request req) override {
      m_addr_mapper->apply(req);
      
      bool is_success;      
      if(req.addr_vec[row_addr_idx] == ndp_ctrl_row && 
        ((req.addr_vec[bk_addr_idx] == ndp_ctrl_bk     && req.addr_vec[bg_addr_idx] == ndp_ctrl_bg) || 
         (req.addr_vec[bk_addr_idx] == ndp_ctrl_buf_bk && req.addr_vec[bg_addr_idx] == ndp_ctrl_buf_bg))) {          
          // Access NDP-specific Region (Host-Side NDP Controller)
          is_success = send_ndp_ctrl(req);
          if (is_success) {
            switch (req.type_id) {
              case Request::Type::Read: {
                s_num_ndp_read_requests++;
                break;
              }
              case Request::Type::Write: {
                s_num_ndp_write_requests++;
                break;
              }
              default: {
                s_num_ndp_other_requests++;
                break;
              }
            }
          }
      } else {
          int channel_id = req.addr_vec[0];
          // Check is NDP Request and set Request is_ndp_req as true (DIMM-Side NDP Controller)
          if(req.addr_vec[row_addr_idx] == ndp_ctrl_row) {
            req.is_ndp_req = true;
            req.is_trace_core_req = true;
            int dimm_id = req.addr_vec[0]/2;
            int pch_id = ((req.addr_vec[0]%2 == 1) ? num_pseudochannel : 0) + req.addr_vec[m_dram->m_levels("pseudochannel")];
            DEBUG_PRINT(m_clk,"HSNC", dimm_id, pch_id, "Host send NDP Request to NDP Unit");            
          } 

          if(!(req.is_trace_core_req)) {
            req.is_host_req = true;
          } else {
            req.is_host_req = false;
          }

          is_success = m_controllers[channel_id]->send(req);
          if (is_success) {
            switch (req.type_id) {
              case Request::Type::Read: {
                s_num_read_requests++;
                break;
              }
              case Request::Type::Write: {
                s_num_write_requests++;
                break;
              }
              default: {
                s_num_other_requests++;
                break;
              }
            }
          }
      }
      
      m_host_access = true;
      return is_success;
    };
    
    void tick() override {
      m_clk++;
      // Trace Mode :  Send Trace-based Request to DRAM System
      if(m_trace_core_enable && !m_host_access) try_issue_requests(); 
      m_host_access = false;
      m_dram->tick();
      for (auto controller : m_controllers) {
        controller->tick();
        while(1) {
          read_latency = controller->get_req_latency();
          if(read_latency == 0) {
            break;
          } else {
            record_latency(read_latency);
          }
        }        
      }

      #ifdef PCH_DEBUG
        int error_pch;
        error_pch = m_dram->get_pch_error();

        if(error_pch != -1) {
          std::cout<<" ============= PCH ERROR "<<error_pch<<"==========="<< std::endl;
          std::cout<<" Print All PCH HSNC Status "<<std::endl;
          for(int dimm_id=0;dimm_id<m_num_dimm;dimm_id++) { 
            for(int pch_id=0;pch_id<(m_num_subch*num_pseudochannel);pch_id++) {
              int dram_lvl_pch_id = dimm_id*m_num_subch*num_pseudochannel + pch_id;
              std::cout<<"["<<dimm_id<<"]["<<pch_id<<"] --> "<<dram_lvl_pch_id;
              std::cout<<" : "<<ndp_ctrl_status_to_str.at(static_cast<NDP_CTRL_STATUS>(pch_lvl_hsnc_status[dimm_id][pch_id]))<<std::endl;                                        
            }
          }      
          std::cout<<" Print All PCH Req History "<<std::endl;
          for(int dimm_id=0;dimm_id<m_num_dimm;dimm_id++) { 
            for(int pch_id=0;pch_id<(m_num_subch*num_pseudochannel);pch_id++) {
              int dram_lvl_pch_id = dimm_id*m_num_subch*num_pseudochannel + pch_id;
              std::cout<<"["<<dimm_id<<"]["<<pch_id<<"] --> "<<dram_lvl_pch_id<<std::endl;
              for(int his_idx=0;his_idx<pch_lvl_history[dimm_id][pch_id].size();his_idx++) {                                              
                std::cout<<" - ["<<his_idx<<"] OPCODE: "<<op_to_str.at(static_cast<NDP_Op>(pch_lvl_history[dimm_id][pch_id][his_idx].opcode));
                std::cout<<", OPSIZE: "<<pch_lvl_history[dimm_id][pch_id][his_idx].opsize;
                std::cout<<", CH: "<<pch_lvl_history[dimm_id][pch_id][his_idx].ch;
                std::cout<<", PCH: "<<pch_lvl_history[dimm_id][pch_id][his_idx].pch;
                std::cout<<", BG: "<<pch_lvl_history[dimm_id][pch_id][his_idx].bg;
                std::cout<<", BK: "<<pch_lvl_history[dimm_id][pch_id][his_idx].bk;
                std::cout<<", ROW: "<<pch_lvl_history[dimm_id][pch_id][his_idx].row<<std::endl;
              }
            }
          }
          std::cout<<" Print All PCH Remained Req in Slot "<<std::endl;
          for(int dimm_id=0;dimm_id<m_num_dimm;dimm_id++) { 
            for(int pch_id=0;pch_id<(m_num_subch*num_pseudochannel);pch_id++) {
              int dram_lvl_pch_id = dimm_id*m_num_subch*num_pseudochannel + pch_id;
              std::cout<<"["<<dimm_id<<"]["<<pch_id<<"] --> "<<dram_lvl_pch_id<<std::endl;
              for(int his_idx=0;his_idx<pch_lvl_hsnc_nl_addr_gen_slot[dimm_id][pch_id].size();his_idx++) {       
                auto nl_req_it = pch_lvl_hsnc_nl_addr_gen_slot[dimm_id][pch_id][his_idx];                                        
                std::cout<<" - ["<<his_idx<<"] OPCODE: "<<op_to_str.at(static_cast<NDP_Op>(nl_req_it.opcode));
                std::cout<<", OPSIZE: "<<nl_req_it.opsize;
                std::cout<<", CNT: "<<nl_req_it.cnt;
                std::cout<<", CH: "<<nl_req_it.ch;
                std::cout<<", PCH: "<<nl_req_it.pch;
                std::cout<<", BG: "<<nl_req_it.bg;
                std::cout<<", BK: "<<nl_req_it.bk;
                std::cout<<", ROW: "<<nl_req_it.row<<std::endl;
              }
            }
          }            

          throw std::runtime_error("Detecting PCH Error");   
        }
      #endif 
      #ifdef NDP_DEBUG
        if(m_clk%10000 == 0) {
          std::cout<<"Status"<<std::endl;
          for(int dimm_id=0;dimm_id<m_num_dimm;dimm_id++) {
            std::cout<<"  - DIMM["<<dimm_id<<"]"<<std::endl;
            for(int pch_id=0;pch_id<(m_num_subch*num_pseudochannel);pch_id++) {
              std::cout<<"  - PCH["<<pch_id<<"] : "<<ndp_ctrl_status_to_str.at(pch_lvl_hsnc_status[dimm_id][pch_id])
                       <<" PC="<<pch_lvl_pc[dimm_id][pch_id]<<std::endl;
            }
          }
        }
      #endif

      // Loop All pseudo channel along all DIMMs
      bool pch_lvl_ndp_idle = true;

      for(int dimm_id=0;dimm_id<m_num_dimm;dimm_id++) {
        for(int pch_id=0;pch_id<(m_num_subch*num_pseudochannel);pch_id++) {

          if(!(pch_lvl_hsnc_status[dimm_id][pch_id] == NDP_IDLE)) {
            pch_lvl_ndp_idle = false;
          }

          if(pch_lvl_hsnc_status[dimm_id][pch_id] == NDP_IDLE) {
            // NDP Status: NDP IDLE 

            // Do Nothing!!
            pch_hsnc_status_cnt[dimm_id][pch_id][NDP_IDLE]++;
          } // NDP Status: NDP IDLE End
          else if(pch_lvl_hsnc_status[dimm_id][pch_id] == NDP_ISSUE_START) {
            // NDP Status: NDP_ISSUE_START
            // NDP Inst writes 완료 확인 후 DSNC로 NDP Start Write 전송
            int ch_id = (pch_id >= num_pseudochannel ? 1 : 0) + dimm_id * m_num_subch;
            int pch_id_per_subch = pch_id%num_pseudochannel;
            if(m_controllers[ch_id]->is_empty_ndp_req(pch_id_per_subch)) {
              // Generate NDP Start Write-Request to DSNC (db_ndp_ctrl_access_bg/bk)
              Request req = Request(0,Request::Type::Write);
              m_addr_mapper->apply(req);
              req.addr_vec[m_dram->m_levels("channel")]       = ch_id;
              req.addr_vec[m_dram->m_levels("pseudochannel")] = pch_id_per_subch;
              req.addr_vec[m_dram->m_levels("bankgroup")]     = db_ndp_ctrl_access_bg;
              req.addr_vec[m_dram->m_levels("bank")]          = db_ndp_ctrl_access_bk;
              req.addr_vec[m_dram->m_levels("row")]           = ndp_ctrl_row;
              req.is_ndp_req = true;
              req.is_trace_core_req = true;
              for(int i=0;i<8;i++) {
                req.m_payload.push_back(1);
              }
              if(m_controllers[ch_id]->send(req)) {
                // NDP Status transit from NDP_ISSUE_START to NDP_BEFORE_RUN
                pch_lvl_hsnc_status[dimm_id][pch_id] = NDP_BEFORE_RUN;
                DEBUG_PRINT(m_clk,"HSNC", dimm_id, pch_id, "transit from NDP_ISSUE_START to NDP_BEFORE_RUN");
              }
            }
            pch_hsnc_status_cnt[dimm_id][pch_id][NDP_ISSUE_START]++;
          } // NDP Status: NDP_ISSUE_START END
          else if(pch_lvl_hsnc_status[dimm_id][pch_id] == NDP_BEFORE_RUN) {
            // NDP Status: NDP_BEFORE_RUN
            // DSNC로 보낸 NDP Start Write 완료 대기 → NDP_RUN 전환
            int ch_id = (pch_id >= num_pseudochannel ? 1 : 0) + dimm_id * m_num_subch;
            int pch_id_per_subch = pch_id%num_pseudochannel;
            if(m_controllers[ch_id]->is_empty_ndp_req(pch_id_per_subch)) {
              pch_lvl_hsnc_status[dimm_id][pch_id] = NDP_RUN;
              // Reset HSNC registers for new NDP execution
              hsnc_base_reg[dimm_id][pch_id].fill(0);
              hsnc_loop_cnt_reg[dimm_id][pch_id].fill(0);
              desc_cache_hit_cnt[dimm_id][pch_id] = 0;
              desc_cache_miss_cnt[dimm_id][pch_id] = 0;
              desc_cache_evict_cnt[dimm_id][pch_id] = 0;
              DEBUG_PRINT(m_clk,"HSNC", dimm_id, pch_id, "transit from NDP_BEFORE_RUN to NDP_RUN");
              // Segment tracking: start first segment
              hsnc_cur_seg[dimm_id][pch_id] = {hsnc_seg_cnt[dimm_id][pch_id], HsncSegType::UNKNOWN, m_clk, 0, 0};
              hsnc_seg_tracking[dimm_id][pch_id] = true;
            }
            pch_hsnc_status_cnt[dimm_id][pch_id][NDP_BEFORE_RUN]++;
          } // NDP Status: NDP_BEFORE_RUN END
          else if(pch_lvl_hsnc_status[dimm_id][pch_id] == NDP_RUN) {
            // NDP Status: NDP_RUN — PC 기반 fetch + decode + addr_gen_slot

            // Stage 4: addr_gen_slot → send_ndp_req_to_mc()
            if(pch_lvl_hsnc_nl_addr_gen_slot[dimm_id][pch_id].size() != 0) send_ndp_req_to_mc(dimm_id,pch_id);

            // Stage 3: PC 기반 fetch → decode → dispatch
            // 제어 명령은 addr_gen_slot 불필요 → 가드 없이 항상 fetch 시도
            {
              int pc = pch_lvl_pc[dimm_id][pch_id];
              int col_addr = pc >> 3;    // PC[9:3]
              int idx = pc & 0x7;        // PC[2:0]

              int group_idx = find_cache_group(dimm_id, pch_id, col_addr);

              if(group_idx < 0) {
                // Cache Miss → NDP_FETCH_STALL, DRAM RD 발행
                desc_cache_miss_cnt[dimm_id][pch_id]++;
                issue_desc_fetch(dimm_id, pch_id, col_addr);
                pch_lvl_hsnc_status[dimm_id][pch_id] = NDP_FETCH_STALL;
                pch_fetch_stall[dimm_id][pch_id] = true;
                pch_fetch_stall_col[dimm_id][pch_id] = col_addr;
                DEBUG_PRINT(m_clk,"HSNC", dimm_id, pch_id, "Cache Miss col=" + std::to_string(col_addr) + " → NDP_FETCH_STALL");
              } else {
                // Cache Hit → fetch & decode
                desc_cache_hit_cnt[dimm_id][pch_id]++;
                update_lru(dimm_id, pch_id, group_idx);
                uint64_t inst = pch_lvl_inst_buf[dimm_id][pch_id][group_idx][idx];
                AccInst_Slot nl_req = decode_acc_inst(inst, dimm_id, pch_id);

                DEBUG_PRINT(m_clk,"HSNC", dimm_id, pch_id, "PC=" + std::to_string(pc) + " fetch opcode=" + std::to_string(nl_req.opcode));

                #ifdef PCH_DEBUG
                if((int)pch_lvl_history[dimm_id][pch_id].size() >= pch_lvl_history_max) {
                  pch_lvl_history[dimm_id][pch_id].erase(pch_lvl_history[dimm_id][pch_id].begin());
                }
                pch_lvl_history[dimm_id][pch_id].push_back(nl_req);
                #endif

                // Opcode dispatch
                if(nl_req.opcode == NDP_ACC_OP_RD || nl_req.opcode == NDP_ACC_OP_WR) {
                  // RD/WR: addr_gen_slot 용량 체크 필요
                  if(pch_lvl_hsnc_nl_addr_gen_slot[dimm_id][pch_id].size() < (size_t)pch_lvl_hsnc_nl_addr_gen_slot_max) {
                    // PCH index 검증
                    int pch_idx1 = nl_req.ch * num_pseudochannel + nl_req.pch;
                    int pch_idx2 = dimm_id * m_num_subch * num_pseudochannel + pch_id;
                    if(pch_idx1 != pch_idx2) {
                      std::cout<<"["<<m_clk<<"] Miss pch_idx PC="<<pc<<std::endl;
                      std::cout<<" dimm_id:"<<dimm_id<<" pch_id:"<<pch_id<<std::endl;
                      std::cout<<" nl_req.ch:"<<nl_req.ch<<" nl_req.pch:"<<nl_req.pch<<std::endl;
                      throw std::runtime_error("Miss pch_idx");
                    }
                    pch_lvl_hsnc_nl_addr_gen_slot[dimm_id][pch_id].push_back(nl_req);
                    pch_lvl_pc[dimm_id][pch_id]++;
                    // Segment tracking: determine type from AccInst opcode
                    if (hsnc_seg_tracking[dimm_id][pch_id] && hsnc_cur_seg[dimm_id][pch_id].type == HsncSegType::UNKNOWN) {
                      hsnc_cur_seg[dimm_id][pch_id].type =
                        (nl_req.opcode == NDP_ACC_OP_WR) ? HsncSegType::WR : HsncSegType::RD;
                    }
                  } else {
                    // addr_gen_slot full → stall (Undirect row resolution is idempotent, safe to re-decode)
                    pch_lvl_hsnc_nl_addr_empty_cnt[dimm_id][pch_id]++;
                  }
                }
                else if(nl_req.opcode == NDP_ACC_OP_SET_BASE || nl_req.opcode == NDP_ACC_OP_INC_BASE) {
                  // Control: register update already done in decode_acc_inst, just advance PC
                  DEBUG_PRINT(m_clk,"HSNC", dimm_id, pch_id,
                    op_to_str.at(nl_req.opcode) + " reg=" + std::to_string(nl_req.bg) + " val=" + std::to_string(nl_req.row));
                  pch_lvl_pc[dimm_id][pch_id]++;
                }
                else if(nl_req.opcode == NDP_ACC_OP_SET_LOOP) {
                  // Control: loop_cnt_reg update already done in decode_acc_inst
                  DEBUG_PRINT(m_clk,"HSNC", dimm_id, pch_id,
                    "SET_LOOP cnt_reg=" + std::to_string(nl_req.bg) + " count=" + std::to_string(nl_req.row));
                  pch_lvl_pc[dimm_id][pch_id]++;
                }
                else if(nl_req.opcode == NDP_ACC_OP_LOOP) {
                  // LOOP: check loop_cnt_reg, jump or fall through
                  int cnt_reg = nl_req.bg;     // decode repurposed bg = cnt_reg_idx
                  int target_pc = nl_req.row;  // decode repurposed row = jump_pc
                  if(hsnc_loop_cnt_reg[dimm_id][pch_id][cnt_reg] > 0) {
                    hsnc_loop_cnt_reg[dimm_id][pch_id][cnt_reg]--;
                    pch_lvl_pc[dimm_id][pch_id] = target_pc;
                    DEBUG_PRINT(m_clk,"HSNC", dimm_id, pch_id,
                      "LOOP jump: cnt_reg=" + std::to_string(cnt_reg) + " remain=" +
                      std::to_string(hsnc_loop_cnt_reg[dimm_id][pch_id][cnt_reg]) + " → PC=" + std::to_string(target_pc));
                  } else {
                    pch_lvl_pc[dimm_id][pch_id]++;
                    DEBUG_PRINT(m_clk,"HSNC", dimm_id, pch_id, "LOOP fall-through");
                  }
                  // LOOP jump target cache miss handled naturally in next cycle's fetch
                }
                else if(nl_req.opcode == NDP_ACC_OP_BAR) {
                  pch_lvl_hsnc_status[dimm_id][pch_id] = NDP_BAR;
                  pch_lvl_pc[dimm_id][pch_id]++;
                  DEBUG_PRINT(m_clk,"HSNC", dimm_id, pch_id, "transit from NDP_RUN to NDP_BAR");
                  if (hsnc_seg_tracking[dimm_id][pch_id]) {
                    hsnc_cur_seg[dimm_id][pch_id].drain_start = m_clk;
                  }
                }
                else if(nl_req.opcode == NDP_ACC_OP_WAIT) {
                  pch_lvl_hsnc_status[dimm_id][pch_id] = NDP_WAIT;
                  pch_lvl_hsnc_nl_addr_gen_wait_cnt[dimm_id][pch_id] = 0;
                  pch_lvl_hsnc_nl_addr_gen_wait_cycle[dimm_id][pch_id] = nl_req.etc;
                  pch_lvl_pc[dimm_id][pch_id]++;
                  DEBUG_PRINT(m_clk,"HSNC", dimm_id, pch_id, "transit from NDP_RUN to NDP_WAIT");
                  if (hsnc_seg_tracking[dimm_id][pch_id]) {
                    hsnc_cur_seg[dimm_id][pch_id].drain_start = m_clk;
                  }
                }
                else if(nl_req.opcode == NDP_ACC_OP_DONE) {
                  pch_lvl_hsnc_status[dimm_id][pch_id] = NDP_DONE;
                  DEBUG_PRINT(m_clk,"HSNC", dimm_id, pch_id, "transit from NDP_RUN to NDP_DONE");
                  if (hsnc_seg_tracking[dimm_id][pch_id]) {
                    hsnc_cur_seg[dimm_id][pch_id].drain_start = m_clk;
                  }
                }
                else {
                  throw std::runtime_error("Invalid NDP opcode=" + std::to_string(nl_req.opcode) + " at PC=" + std::to_string(pc));
                }
              }
            }
            pch_hsnc_status_cnt[dimm_id][pch_id][NDP_RUN]++;
          } // NDP Status: NDP_RUN END
          else if(pch_lvl_hsnc_status[dimm_id][pch_id] == NDP_BAR) {
            // NDP Status: NDP_BAR — drain all outstanding requests

            int ch_id = (pch_id >= num_pseudochannel ? 1 : 0) + dimm_id * m_num_subch;
            int pch_id_per_subch = pch_id%num_pseudochannel;
            if(pch_lvl_hsnc_nl_addr_gen_slot[dimm_id][pch_id].size() == 0 && m_controllers[ch_id]->is_empty_ndp_req(pch_id_per_subch)) {
              pch_lvl_hsnc_status[dimm_id][pch_id] = NDP_RUN;
              DEBUG_PRINT(m_clk,"HSNC", dimm_id, pch_id, "transit from NDP_BAR to NDP_RUN");
              if (hsnc_seg_tracking[dimm_id][pch_id]) {
                hsnc_cur_seg[dimm_id][pch_id].end = m_clk;
                hsnc_segments[dimm_id][pch_id].push_back(hsnc_cur_seg[dimm_id][pch_id]);
                hsnc_seg_cnt[dimm_id][pch_id]++;
                m_controllers[ch_id]->notify_segment_boundary(pch_id_per_subch, hsnc_seg_cnt[dimm_id][pch_id]);
                hsnc_cur_seg[dimm_id][pch_id] = {hsnc_seg_cnt[dimm_id][pch_id], HsncSegType::UNKNOWN, m_clk, 0, 0};
              }
            }
            if(pch_lvl_hsnc_nl_addr_gen_slot[dimm_id][pch_id].size() != 0) send_ndp_req_to_mc(dimm_id,pch_id);
            pch_hsnc_status_cnt[dimm_id][pch_id][NDP_BAR]++;
          } // NDP Status: NDP_BAR END
          else if(pch_lvl_hsnc_status[dimm_id][pch_id] == NDP_WAIT) {
            // NDP Status: NDP_WAIT — cycle 기반 대기
            if(pch_lvl_hsnc_nl_addr_gen_slot[dimm_id][pch_id].size() != 0) {
              send_ndp_req_to_mc(dimm_id,pch_id);
            } else {
              pch_lvl_hsnc_nl_addr_wait_cnt[dimm_id][pch_id]++;
              if(pch_lvl_hsnc_nl_addr_gen_wait_cnt[dimm_id][pch_id] == pch_lvl_hsnc_nl_addr_gen_wait_cycle[dimm_id][pch_id]) {
                int ch_id = (pch_id >= num_pseudochannel ? 1 : 0) + dimm_id * m_num_subch;
                int pch_id_per_subch = pch_id%num_pseudochannel;
                if(pch_lvl_polling[dimm_id][pch_id]) {
                  int ndp_status = m_controllers[ch_id]->get_config_reg_resp(pch_id_per_subch);
                  if(ndp_status == -1) {
                    // Wait more
                  } else if(m_dram->is_ndp_issuable(ndp_status)) {
                    pch_lvl_hsnc_status[dimm_id][pch_id] = NDP_RUN;
                    DEBUG_PRINT(m_clk,"HSNC", dimm_id, pch_id, "transit from NDP_WAIT to NDP_RUN");
                    pch_lvl_hsnc_nl_addr_gen_wait_cnt[dimm_id][pch_id] = -1;
                    pch_lvl_hsnc_nl_addr_gen_wait_cycle[dimm_id][pch_id] = -1;
                    pch_lvl_polling[dimm_id][pch_id] = false;
                    if (hsnc_seg_tracking[dimm_id][pch_id]) {
                      hsnc_cur_seg[dimm_id][pch_id].type = HsncSegType::WAIT;
                      hsnc_cur_seg[dimm_id][pch_id].end = m_clk;
                      hsnc_segments[dimm_id][pch_id].push_back(hsnc_cur_seg[dimm_id][pch_id]);
                      hsnc_seg_cnt[dimm_id][pch_id]++;
                      m_controllers[ch_id]->notify_segment_boundary(pch_id_per_subch, hsnc_seg_cnt[dimm_id][pch_id]);
                      hsnc_cur_seg[dimm_id][pch_id] = {hsnc_seg_cnt[dimm_id][pch_id], HsncSegType::UNKNOWN, m_clk, 0, 0};
                    }
                  } else {
                    pch_lvl_polling[dimm_id][pch_id] = false;
                    pch_lvl_hsnc_nl_addr_gen_wait_cnt[dimm_id][pch_id]   = 0;
                    pch_lvl_hsnc_nl_addr_gen_wait_cycle[dimm_id][pch_id] = 64*10;
                  }
                } else {
                  Request req = Request(0,Request::Type::Read);
                  m_addr_mapper->apply(req);
                  req.addr_vec[m_dram->m_levels("channel")]       = ch_id;
                  req.addr_vec[m_dram->m_levels("pseudochannel")] = pch_id_per_subch;
                  req.addr_vec[m_dram->m_levels("bankgroup")]     = db_ndp_ctrl_access_bg;
                  req.addr_vec[m_dram->m_levels("bank")]          = db_ndp_ctrl_access_bk;
                  req.addr_vec[m_dram->m_levels("row")]           = ndp_ctrl_row;
                  req.is_ndp_req = true;
                  if(m_controllers[ch_id]->send(req)) {
                    pch_lvl_polling[dimm_id][pch_id] = true;
                  }
                }
              } else {
                pch_lvl_hsnc_nl_addr_gen_wait_cnt[dimm_id][pch_id]++;
              }
            }
            pch_hsnc_status_cnt[dimm_id][pch_id][NDP_WAIT]++;
          } // NDP Status: NDP_WAIT END
          else if(pch_lvl_hsnc_status[dimm_id][pch_id] == NDP_FETCH_STALL) {
            // NDP Status: NDP_FETCH_STALL — Descriptor Cache miss 대기

            // Stage 4는 계속 동작
            if(pch_lvl_hsnc_nl_addr_gen_slot[dimm_id][pch_id].size() != 0) send_ndp_req_to_mc(dimm_id,pch_id);

            // DRAM RD 완료 확인 (callback에 의해 pch_fetch_complete 설정됨)
            if(pch_fetch_complete[dimm_id][pch_id]) {
              // RD 응답 도착 → desc_store에서 buffer로 적재
              int target_col = pch_fetch_stall_col[dimm_id][pch_id];
              int evict_group = find_lru_group(dimm_id, pch_id);
              if(pch_lvl_inst_buf_valid[dimm_id][pch_id][evict_group]) {
                desc_cache_evict_cnt[dimm_id][pch_id]++;
              }
              load_to_cache(dimm_id, pch_id, evict_group, target_col);
              pch_fetch_stall[dimm_id][pch_id] = false;
              pch_fetch_complete[dimm_id][pch_id] = false;
              pch_lvl_hsnc_status[dimm_id][pch_id] = NDP_RUN;
              DEBUG_PRINT(m_clk,"HSNC", dimm_id, pch_id, "FETCH_STALL resolved col=" + std::to_string(target_col) + " → NDP_RUN");
            }
            pch_hsnc_status_cnt[dimm_id][pch_id][NDP_FETCH_STALL]++;
          } // NDP Status: NDP_FETCH_STALL END
          else if(pch_lvl_hsnc_status[dimm_id][pch_id] == NDP_DONE) {
            // NDP Status: NDP_DONE — drain all then IDLE

            int ch_id = (pch_id >= num_pseudochannel ? 1 : 0) + dimm_id * m_num_subch;
            int pch_id_per_subch = pch_id%num_pseudochannel;
            if(pch_lvl_hsnc_nl_addr_gen_slot[dimm_id][pch_id].size() == 0 && m_controllers[ch_id]->is_empty_ndp_req(pch_id_per_subch)) {
              pch_lvl_hsnc_status[dimm_id][pch_id] = NDP_IDLE;
              DEBUG_PRINT(m_clk,"HSNC", dimm_id, pch_id, "transit from NDP_DONE to NDP_IDLE");
              if (hsnc_seg_tracking[dimm_id][pch_id]) {
                hsnc_cur_seg[dimm_id][pch_id].end = m_clk;
                hsnc_segments[dimm_id][pch_id].push_back(hsnc_cur_seg[dimm_id][pch_id]);
                hsnc_seg_cnt[dimm_id][pch_id]++;
                m_controllers[ch_id]->notify_segment_boundary(pch_id_per_subch, hsnc_seg_cnt[dimm_id][pch_id]);
                hsnc_seg_tracking[dimm_id][pch_id] = false;
              }
            }
            if(pch_lvl_hsnc_nl_addr_gen_slot[dimm_id][pch_id].size() != 0) send_ndp_req_to_mc(dimm_id,pch_id);
            pch_hsnc_status_cnt[dimm_id][pch_id][NDP_DONE]++;
          } // NDP Status: NDP_DONE END
        } // LOOP-PCH END
      } // LOOP-DIMM END
      if(!all_ndp_idle && pch_lvl_ndp_idle) {
        DEBUG_PRINT(m_clk,"HSNC", -1, -1, "=================== All Pseudo Channel NDP DONE ============== ");
      }        
      all_ndp_idle = pch_lvl_ndp_idle;     
    };

    float get_tCK() override {
      return m_dram->m_timing_vals("tCK_ps") / 1000.0f;
    }

    // const SpecDef& get_supported_requests() override {
    //   return m_dram->m_requests;
    // };
    bool send_ndp_ctrl(Request req) {

      if(req.addr_vec[bk_addr_idx] == ndp_ctrl_bk && req.addr_vec[bg_addr_idx] == ndp_ctrl_bg) {
        // NDP Control Register Write (NDP Start)
        int dimm_id = req.addr_vec[0]/2;
        DEBUG_PRINT(m_clk,"HSNC", dimm_id, 0, "NDP Ctrl Get NDP Control Request");
        if(req.type_id == Request::Type::Write) {
          #ifdef NDP_DEBUG
          for (int i = 0; i < 8; i++)
            std::cout<<"[HSNU] DIMM["<<dimm_id<<"][payload"<<i<<"]:"<<req.m_payload[i]<<std::endl;
          #endif

          // NDP Start: per-PCH start flag + desc_count
          for(size_t i=0;i<pch_lvl_hsnc_status[dimm_id].size();i++) {
            if(pch_lvl_hsnc_status[dimm_id][i] != NDP_IDLE && req.m_payload[i] != 0) {
              throw std::runtime_error("Overlap Write to NDP Control Register");
            }
            if(req.m_payload[i] != 0) {
              pch_desc_count[dimm_id][i] = req.m_payload[i] & 0xFFFF;  // bits[15:0] = desc_count
              pch_lvl_pc[dimm_id][i] = 0;  // PC 초기화
              pch_lvl_hsnc_status[dimm_id][i] = NDP_ISSUE_START;
              DEBUG_PRINT(m_clk,"HSNC", dimm_id, i, "NDP Start: desc_count=" + std::to_string(pch_desc_count[dimm_id][i]));
            }
          }
        } else {
          throw std::runtime_error("Not Allow NDP Controller Read (Not Implemented)");
        }
      } else {
        // NDP Launch Request Buffer — Write Intercept
        if(req.type_id == Request::Type::Write) {
          int dimm_id = req.addr_vec[ch_addr_idx]/2;
          int pch_id = ((req.addr_vec[ch_addr_idx]%2 == 1) ? num_pseudochannel : 0) + req.addr_vec[m_dram->m_levels("pseudochannel")];
          int col = req.addr_vec[col_addr_idx];
          int ch_id = req.addr_vec[ch_addr_idx];

          // DRAM WR timing check (실제 data 저장 안 함, timing만 반영)
          // AccInst Write는 Host→DRAM 경로 (일반 WR 커맨드 사용)
          // is_ndp_req=false → final_command=WR (NDP_DB_WR가 아닌 정상 DRAM Write)
          req.is_ndp_req = false;
          req.is_trace_core_req = true;
          if(!m_controllers[ch_id]->send(req)) {
            return false;  // DRAM WR queue full → frontend 재시도
          }

          DEBUG_PRINT(m_clk,"HSNC", dimm_id, pch_id, "Write Intercept: col=" + std::to_string(col));

          // 1. desc_store에 전량 저장 (8 entries per write = 1 DRAM column)
          for(int i = 0; i < 8; i++)
            desc_store[dimm_id][pch_id][col][i] = req.m_payload[i];

          // 2. Col 0~7이면 pch_lvl_inst_buf(Descriptor Cache)에도 동시 적재
          if(col < 8) {
            for(int i = 0; i < 8; i++)
              pch_lvl_inst_buf[dimm_id][pch_id][col][i] = desc_store[dimm_id][pch_id][col][i];
            pch_lvl_inst_buf_tag[dimm_id][pch_id][col] = col;
            pch_lvl_inst_buf_valid[dimm_id][pch_id][col] = true;
            pch_lvl_inst_buf_lru[dimm_id][pch_id][col] = col;  // 순차 적재이므로 col 값으로 LRU 초기화
          }

          return true;
        } else {
          throw std::runtime_error("Not Allow NL-Req Buffer Read (Not Implemented)");
        }
      }
      return true;
    }

    bool is_finished() override {      
      bool is_dram_ctrl_finished = true;
      int num_channels = m_dram->get_level_size("channel"); 
      for (int i = 0; i < num_channels; i++) {

        if(m_trace_core_enable) {
          if(!m_controllers[i]->is_finished())
            is_dram_ctrl_finished = false;
        } else {
          if(!m_controllers[i]->is_abs_finished())
            is_dram_ctrl_finished = false;
        }
      }        
      if(all_ndp_idle && is_dram_ctrl_finished) {
        int m_num_dimm   = num_channels / 2;
        int m_num_subch  = 2;
        int num_pseudochannel = m_dram->get_level_size("pseudochannel");
        std::cout<<"["<<m_clk<<"] [NDP_DRAM_System] All Request Done!!! (+NDP Ops)"<<std::endl;

        std::cout<<"NDP Addr Gen Slot Full Count ([DIMM][PCH])"<<std::endl;
        for(int dimm_id=0;dimm_id<m_num_dimm;dimm_id++) {
          for(int pch_id=0;pch_id<(m_num_subch*num_pseudochannel);pch_id++) {
              std::cout<<" - ["<<dimm_id<<"]["<<pch_id<<"] : "<<pch_lvl_hsnc_nl_addr_empty_cnt[dimm_id][pch_id]<<std::endl;
          }
        }

        std::cout<<"NDP Request Address Generation Block Count"<<std::endl;
        for(int dimm_id=0;dimm_id<m_num_dimm;dimm_id++) {
          for(int pch_id=0;pch_id<(m_num_subch*num_pseudochannel);pch_id++) {
              std::cout<<" - ["<<dimm_id<<"]["<<pch_id<<"] : "<<pch_lvl_hsnc_nl_addr_wait_cnt[dimm_id][pch_id]<<std::endl;
          }
        }

        // Status name table for printing
        const char* ndp_stat_names[] = {
          "[NDP_IDLE]            | ",
          "[NDP_ISSUE_START]     | ",
          "[NDP_BEFORE_RUN]      | ",
          "[NDP_RUN]             | ",
          "[NDP_BAR]             | ",
          "[NDP_WAIT]            | ",
          "[NDP_FETCH_STALL]     | ",
          "[NDP_DONE]            | "
        };

        std::cout<<"Host-NDP Ctrl Status Stats (total cycle: "<<m_clk<<")"<<std::endl;
        std::cout<<"   - Each PCH per DIMM"<<std::endl;
        std::cout<<"--------------------------------------------------------------------------------------------"<<std::endl;
        for(int ndp_stat=0;ndp_stat<NUM_NDP_STATES;ndp_stat++) {
          std::cout<<ndp_stat_names[ndp_stat];
          for(int dimm_id=0;dimm_id<m_num_dimm;dimm_id++) {
            for(int pch_id=0;pch_id<(m_num_subch*num_pseudochannel);pch_id++) {
                std::cout<<pch_hsnc_status_cnt[dimm_id][pch_id][ndp_stat];
                if(pch_id != (m_num_subch*num_pseudochannel - 1))
                  std::cout<<" | ";
            }
          }
          std::cout<<std::endl;
        }
        std::cout<<"--------------------------------------------------------------------------------------------"<<std::endl;
        for(int ndp_stat=0;ndp_stat<NUM_NDP_STATES;ndp_stat++) {
          std::cout<<ndp_stat_names[ndp_stat];
          for(int dimm_id=0;dimm_id<m_num_dimm;dimm_id++) {
            for(int pch_id=0;pch_id<(m_num_subch*num_pseudochannel);pch_id++) {
                float val = (float)pch_hsnc_status_cnt[dimm_id][pch_id][ndp_stat] / (float)m_clk;
                std::cout<<val;
                if(pch_id != (m_num_subch*num_pseudochannel - 1))
                  std::cout<<" | ";
            }
          }
          std::cout<<std::endl;
        }
        std::cout<<"--------------------------------------------------------------------------------------------"<<std::endl;

        // Descriptor Cache Statistics
        std::cout<<"\nDescriptor Cache Statistics ([DIMM][PCH])"<<std::endl;
        std::cout<<"------+-----------+-----------+-----------+-----------"<<std::endl;
        std::cout<<"  PCH |       Hit |      Miss |     Evict |  Hit Rate"<<std::endl;
        std::cout<<"------+-----------+-----------+-----------+-----------"<<std::endl;
        for(int dimm_id=0;dimm_id<m_num_dimm;dimm_id++) {
          for(int pch_id=0;pch_id<(m_num_subch*num_pseudochannel);pch_id++) {
            uint64_t hit   = desc_cache_hit_cnt[dimm_id][pch_id];
            uint64_t miss  = desc_cache_miss_cnt[dimm_id][pch_id];
            uint64_t evict = desc_cache_evict_cnt[dimm_id][pch_id];
            float hit_rate = (hit + miss > 0) ? (float)hit / (float)(hit + miss) * 100.0f : 0.0f;
            std::cout<<" ["<<dimm_id<<"]["<<pch_id<<"] | "
                     <<std::setw(9)<<hit<<" | "
                     <<std::setw(9)<<miss<<" | "
                     <<std::setw(9)<<evict<<" | "
                     <<std::fixed<<std::setprecision(2)<<std::setw(8)<<hit_rate<<"%"
                     <<std::endl;
          }
        }
        std::cout<<"------+-----------+-----------+-----------+-----------"<<std::endl;

      }
      if(m_trace_core_enable) return true;
      else                    return (all_ndp_idle && is_dram_ctrl_finished);
    }

    bool is_ndp_finished() override {
      if(m_trace_core_enable) 
        return true;
      else return all_ndp_idle;
    }     
    
    
    // ========== Descriptor Cache Helper Functions ==========

    // Descriptor Cache에서 col_addr에 해당하는 entry group 찾기 (fully-associative)
    // 반환: group_idx (0~7), 없으면 -1
    int find_cache_group(int dimm_id, int pch_id, int col_addr) {
      for (int g = 0; g < 8; g++) {
        if (pch_lvl_inst_buf_valid[dimm_id][pch_id][g] &&
            pch_lvl_inst_buf_tag[dimm_id][pch_id][g] == col_addr)
          return g;
      }
      return -1;
    }

    // LRU 업데이트 (accessed group을 most recent로)
    void update_lru(int dimm_id, int pch_id, int group_idx) {
      int cur = pch_lvl_inst_buf_lru[dimm_id][pch_id][group_idx];
      for (int g = 0; g < 8; g++) {
        if (pch_lvl_inst_buf_lru[dimm_id][pch_id][g] > cur)
          pch_lvl_inst_buf_lru[dimm_id][pch_id][g]--;
      }
      pch_lvl_inst_buf_lru[dimm_id][pch_id][group_idx] = 7;  // most recent
    }

    // LRU eviction 대상 찾기 (가장 작은 LRU 값, invalid 우선)
    int find_lru_group(int dimm_id, int pch_id) {
      int min_lru = INT_MAX, min_g = 0;
      for (int g = 0; g < 8; g++) {
        if (!pch_lvl_inst_buf_valid[dimm_id][pch_id][g]) return g;  // invalid group 우선
        if (pch_lvl_inst_buf_lru[dimm_id][pch_id][g] < min_lru) {
          min_lru = pch_lvl_inst_buf_lru[dimm_id][pch_id][g];
          min_g = g;
        }
      }
      return min_g;
    }

    // desc_store에서 Descriptor Cache로 적재
    void load_to_cache(int dimm_id, int pch_id, int group_idx, int col_addr) {
      for (int i = 0; i < 8; i++)
        pch_lvl_inst_buf[dimm_id][pch_id][group_idx][i] = desc_store[dimm_id][pch_id][col_addr][i];
      pch_lvl_inst_buf_tag[dimm_id][pch_id][group_idx] = col_addr;
      pch_lvl_inst_buf_valid[dimm_id][pch_id][group_idx] = true;
      update_lru(dimm_id, pch_id, group_idx);
    }

    // Descriptor Cache miss 시 DRAM RD 발행 (timing check)
    // is_ndp_req=false → 정상 DRAM RD (NDP_DB_RD가 아닌 실제 DRAM Read)
    // callback으로 RD 완료 시 HSNC에 알림 (is_empty_ndp_req 대신)
    void issue_desc_fetch(int dimm_id, int pch_id, int col_addr) {
      int ch_id = (pch_id >= num_pseudochannel ? 1 : 0) + dimm_id * m_num_subch;
      int pch_id_per_subch = pch_id % num_pseudochannel;

      Request req = Request(0, Request::Type::Read);
      m_addr_mapper->apply(req);
      req.addr_vec[m_dram->m_levels("channel")]       = ch_id;
      req.addr_vec[m_dram->m_levels("pseudochannel")] = pch_id_per_subch;
      req.addr_vec[m_dram->m_levels("bankgroup")]     = ndp_ctrl_buf_bg;
      req.addr_vec[m_dram->m_levels("bank")]          = ndp_ctrl_buf_bk;
      req.addr_vec[m_dram->m_levels("row")]           = ndp_ctrl_row;
      req.addr_vec[m_dram->m_levels("column")]        = col_addr;
      req.is_ndp_req = false;
      req.is_trace_core_req = true;

      // DRAM RD 완료 시 callback → pch_fetch_complete 플래그 설정
      req.callback = [this, dimm_id, pch_id, col_addr](Request& completed_req) {
        this->on_desc_fetch_complete(dimm_id, pch_id, col_addr);
      };

      m_controllers[ch_id]->send(req);
    }

    // Descriptor fetch DRAM RD 완료 callback
    void on_desc_fetch_complete(int dimm_id, int pch_id, int col_addr) {
      pch_fetch_complete[dimm_id][pch_id] = true;
      DEBUG_PRINT(m_clk, "HSNC", dimm_id, pch_id, "desc_fetch RD complete col=" + std::to_string(col_addr));
    }

    // ========== Decode & Issue Functions ==========

    // Phase 2: Redefined AccInst_Slot (64-bit) with Direct/Undirect Mode
    // Bit layout (matches ndp_workload_trace_generator.py acc_inst()):
    //   [63:60] opcode(4b) [59] mode(1b) [58:52] opsize(7b) [51:46] ch(6b) [45:44] pch(2b)
    //   [43:41] bg(3b) [40:39] bk(2b) [38:21] row(18b) [20:14] col(7b) [13:11] id(3b) [10:0] etc(11b)
    // Control instructions use opcode-specific field interpretation:
    //   SET_BASE/INC_BASE: [58:56]=reg_idx(3b) [55:38]=value(18b)
    //   SET_LOOP:          [58:56]=cnt_reg_idx(3b) [55:40]=loop_count(16b)
    //   LOOP:              [58:56]=cnt_reg_idx(3b) [55:40]=jump_pc(16b)
    AccInst_Slot decode_acc_inst(uint64_t inst, int dimm_id, int pch_id) {
      uint64_t opcode = (inst >> 60) & 0xf;
      uint64_t mode   = (inst >> 59) & 0x1;

      // === Control instructions: opcode-specific field interpretation ===

      // SET_BASE: base_reg[reg_idx] = base_value
      if (opcode == NDP_ACC_OP_SET_BASE) {
        int reg_idx    = (inst >> 56) & 0x7;
        int base_value = (inst >> 38) & 0x3FFFF;
        hsnc_base_reg[dimm_id][pch_id][reg_idx] = base_value;
        return AccInst_Slot(true, opcode, 0, 0, 0, reg_idx, 0, base_value, 0, 0, 0, 0);
      }

      // INC_BASE: base_reg[reg_idx] += inc_value
      if (opcode == NDP_ACC_OP_INC_BASE) {
        int reg_idx   = (inst >> 56) & 0x7;
        int inc_value = (inst >> 38) & 0x3FFFF;
        hsnc_base_reg[dimm_id][pch_id][reg_idx] += inc_value;
        return AccInst_Slot(true, opcode, 0, 0, 0, reg_idx, 0, inc_value, 0, 0, 0, 0);
      }

      // SET_LOOP: loop_cnt_reg[cnt_reg_idx] = loop_count
      if (opcode == NDP_ACC_OP_SET_LOOP) {
        int cnt_reg_idx = (inst >> 56) & 0x7;
        int loop_count  = (inst >> 40) & 0xFFFF;
        hsnc_loop_cnt_reg[dimm_id][pch_id][cnt_reg_idx] = loop_count;
        return AccInst_Slot(true, opcode, 0, 0, 0, cnt_reg_idx, 0, loop_count, 0, 0, 0, 0);
      }

      // LOOP: decode values only — PC change handled in NDP_RUN dispatch
      // bg=cnt_reg_idx, row=jump_pc (repurposed fields for dispatch)
      if (opcode == NDP_ACC_OP_LOOP) {
        int cnt_reg_idx = (inst >> 56) & 0x7;
        int jump_pc     = (inst >> 40) & 0xFFFF;
        return AccInst_Slot(true, opcode, 0, 0, 0, cnt_reg_idx, 0, jump_pc, 0, 0, 0, 0);
      }

      // === RD/WR/BAR/WAIT/DONE: standard layout ===
      uint64_t opsize = (inst >> 52) & 0x7f;   // [58:52]
      uint64_t ch     = (inst >> 46) & 0x3f;   // [51:46]
      uint64_t pch    = (inst >> 44) & 0x3;    // [45:44]
      uint64_t bg     = (inst >> 41) & 0x7;    // [43:41]
      uint64_t bk     = (inst >> 39) & 0x3;    // [40:39]
      uint64_t row    = (inst >> 21) & 0x3FFFF; // [38:21]
      uint64_t col    = (inst >> 14) & 0x7F;   // [20:14]
      uint64_t id     = (inst >> 11) & 0x7;    // [13:11]
      uint64_t etc    = inst & 0x7FF;           // [10:0] 11b

      // Undirect mode (mode=1): row[17:15]=base_reg_idx, row[14:0]=offset
      if (mode == 1 && (opcode == NDP_ACC_OP_RD || opcode == NDP_ACC_OP_WR)) {
        int reg_idx = (row >> 15) & 0x7;
        int offset  = row & 0x7FFF;
        row = hsnc_base_reg[dimm_id][pch_id][reg_idx] + offset;
      }

      #ifdef NDP_DEBUG
        std::cout<<"acc inst decoding opcode "<<opcode<<" mode "<<mode<<" opsize "<<opsize<<" ch "<<ch<<" pch "<<pch<<" bg "<<bg;
        std::cout<<" bk "<<bk<<" row "<<row<<" col "<<col<<" id "<<id<<" etc "<<etc<<std::endl;
      #endif
      return AccInst_Slot(true, opcode, opsize, ch, pch, bg, bk, row, col, id, mode, etc);
    }

    void print_acc_inst(const AccInst_Slot& slot) {
      std::cout<<"acc inst opcode "<<slot.opcode<<" mode "<<slot.mode<<" opsize "<<slot.opsize<<" ch "<<slot.ch<<" pch "<<slot.pch<<" bg "<<slot.bg;
      std::cout<<" bk "<<slot.bk<<" row "<<slot.row<<" col "<<slot.col<<" id "<<slot.id<<" etc "<<slot.etc<<std::endl;
    }

    void send_ndp_req_to_mc(int dimm_id, int pch_id) {
      int ch_id = (pch_id >= num_pseudochannel ? 1 : 0) + dimm_id * m_num_subch;
      int pch_id_per_subch = pch_id%num_pseudochannel;

      // if Round-Robin Index over slot size, set zero
      if(pch_lvl_hsnc_nl_addr_gen_slot_rr_idx[dimm_id][pch_id] >= pch_lvl_hsnc_nl_addr_gen_slot[dimm_id][pch_id].size())
        pch_lvl_hsnc_nl_addr_gen_slot_rr_idx[dimm_id][pch_id] = 0;

      int start_slot_idx = pch_lvl_hsnc_nl_addr_gen_slot_rr_idx[dimm_id][pch_id];        
      for(int i = 0; i < pch_lvl_hsnc_nl_addr_gen_slot[dimm_id][pch_id].size(); i++) {
        int slot_idx = (i + start_slot_idx) % pch_lvl_hsnc_nl_addr_gen_slot[dimm_id][pch_id].size();
        bool is_read;
        if (pch_lvl_hsnc_nl_addr_gen_slot[dimm_id][pch_id][slot_idx].opcode == m_ndp_access_inst_op.at("RD")) 
          is_read = true;
        else if (pch_lvl_hsnc_nl_addr_gen_slot[dimm_id][pch_id][slot_idx].opcode == m_ndp_access_inst_op.at("WR")) 
          is_read = false;
        else 
          throw std::runtime_error("Invalid NDP-launch Request Opcode!");      

        // Generate Request
        Request req = Request(0,is_read ? Request::Type::Read : Request::Type::Write);
        m_addr_mapper->apply(req);
        req.addr_vec[m_dram->m_levels("channel")]       = pch_lvl_hsnc_nl_addr_gen_slot[dimm_id][pch_id][slot_idx].ch;
        req.addr_vec[m_dram->m_levels("pseudochannel")] = pch_lvl_hsnc_nl_addr_gen_slot[dimm_id][pch_id][slot_idx].pch;
        req.addr_vec[m_dram->m_levels("bankgroup")]     = pch_lvl_hsnc_nl_addr_gen_slot[dimm_id][pch_id][slot_idx].bg;
        req.addr_vec[m_dram->m_levels("bank")]          = pch_lvl_hsnc_nl_addr_gen_slot[dimm_id][pch_id][slot_idx].bk;
        req.addr_vec[m_dram->m_levels("row")]           = pch_lvl_hsnc_nl_addr_gen_slot[dimm_id][pch_id][slot_idx].row;
        req.addr_vec[m_dram->m_levels("column")]        = pch_lvl_hsnc_nl_addr_gen_slot[dimm_id][pch_id][slot_idx].col;
        req.ndp_id                                      = pch_lvl_hsnc_nl_addr_gen_slot[dimm_id][pch_id][slot_idx].id;
        req.is_ndp_req                                  = true;
        req.arrive                                      = m_clk;
        req.is_trace_core_req                           = true;
        // Send NDP-Launch Request to Memory Controller 
        bool issue_req;
        req.is_trace_core_req = true;
        issue_req = m_controllers[ch_id]->send(req);            
        if(issue_req) {
          #ifdef NDP_DEBUG
          std::cout<<"["<<m_clk<<"] [HSNC] Send : ";
          m_dram->print_req(req);
          #endif         

          if(pch_lvl_hsnc_nl_addr_gen_slot[dimm_id][pch_id][slot_idx].cnt == pch_lvl_hsnc_nl_addr_gen_slot[dimm_id][pch_id][slot_idx].opsize) {
            // Remove Done Request
            pch_lvl_hsnc_nl_addr_gen_slot[dimm_id][pch_id].erase(pch_lvl_hsnc_nl_addr_gen_slot[dimm_id][pch_id].begin() + slot_idx);
            std::string msg = std::string(" One NL-Req Done! Remove from addresss Generator Slot (Remained ") + std::to_string(pch_lvl_hsnc_nl_addr_gen_slot[dimm_id][pch_id].size()) + std::string(" inst)");
            DEBUG_PRINT(m_clk,"HSNC", dimm_id, pch_id, msg);
          } else {
            pch_lvl_hsnc_nl_addr_gen_slot[dimm_id][pch_id][slot_idx].cnt++;
            pch_lvl_hsnc_nl_addr_gen_slot[dimm_id][pch_id][slot_idx].col++;
            // Round-Robin
            pch_lvl_hsnc_nl_addr_gen_slot_rr_idx[dimm_id][pch_id]++;
          }

          // If success, stop for-loop
          break;
        } else {
          // #ifdef NDP_DEBUG
          // std::cout<<"["<<m_clk<<"] [HSNC] Fail Send : ";
          // m_dram->print_req(req);
          // #endif                
        }
      }
    }

    virtual void mem_sys_finalize() override {
      size_t total_latency = 0;
      for (int i = 0; i < num_channels; i++) {
        total_latency+=m_controllers[i]->get_host_acces_latency();
      }      

      // There is no normal read request, read latency is minus 1
      if (s_num_read_requests == 0) 
        s_avg_read_latency = -1.0;
      else                          
        s_avg_read_latency = (float)total_latency/(float)s_num_read_requests;

      std::cout << "\n=== Memory System Bandwidth (GB/s) ===\n";
      std::cout << "Assume: 512 bits/access, BW = bytes/ns\n\n";      

      int dq_scaling = m_dram->get_dq_scaling();
      // 12 Counters per Memory Controller
      std::vector<uint64_t> counters;
      counters.resize(12, 0);
      for (int i = 0; i < num_channels; i++) {
        auto counters_per_mc = m_controllers[i]->get_counters();
        if(counters_per_mc.size() == 12) {
          for (int c_idx = 0; c_idx < 12; c_idx++) {
            counters[c_idx]+= counters_per_mc[c_idx];
          }
        } else {
          throw std::runtime_error("Invalid Counter Number");
        }
      }          
      int tCK_ps = m_dram->m_timing_vals("tCK_ps");
      // Separate main window = total - tcore (counters[0..5] include tcore)
      // When tcore is disabled, main = total, tcore = 0
      std::vector<uint64_t> main_counters(6, 0);
      if (m_trace_core_enable) {
        for (int i = 0; i < 6; i++)
          main_counters[i] = counters[i] - counters[i + 6];
      } else {
        for (int i = 0; i < 6; i++) {
          main_counters[i] = counters[i];
          counters[i + 6] = 0;
        }
      }

      // ---- Main window (non-tcore only) ----
      std::cout << "[Main window]\n";
      print_bw("Host<->DB",
          calc_bw_gbs(main_counters[0]+main_counters[1]+main_counters[2], 1.0, m_clk, tCK_ps));
      print_bw("Host<->DB HOST",
          calc_bw_gbs(main_counters[0], 1.0, m_clk, tCK_ps));
      print_bw("Host<->DB D2PA",
          calc_bw_gbs(main_counters[1], 1.0, m_clk, tCK_ps));
      print_bw("Host<->DB NDP",
          calc_bw_gbs(main_counters[2], 1.0, m_clk, tCK_ps));

      print_bw("DB<->DRAM",
          calc_bw_gbs(main_counters[3]+main_counters[4]+main_counters[5], dq_scaling, m_clk, tCK_ps));
      print_bw("DB<->DRAM HOST",
          calc_bw_gbs(main_counters[3], dq_scaling, m_clk, tCK_ps));
      print_bw("DB<->DRAM D2PA",
          calc_bw_gbs(main_counters[4], dq_scaling, m_clk, tCK_ps));
      print_bw("DB<->DRAM NDP",
          calc_bw_gbs(main_counters[5], dq_scaling, m_clk, tCK_ps));

      // ---- tcore window ----
      std::cout << "\n[tcore window]\n";
      print_bw("tcore Host<->DB",
          calc_bw_gbs(counters[6]+counters[7]+counters[8], 1.0, m_clk, tCK_ps));      
      print_bw("tcore Host<->DB HOST",
          calc_bw_gbs(counters[6], 1.0, m_clk, tCK_ps));
      print_bw("tcore Host<->DB D2PA",
          calc_bw_gbs(counters[7], 1.0, m_clk, tCK_ps));
      print_bw("tcore Host<->DB NDP",
          calc_bw_gbs(counters[8], 1.0, m_clk, tCK_ps));

      print_bw("tcore DB<->DRAM",
          calc_bw_gbs(counters[9]+counters[10]+counters[11], dq_scaling, m_clk, tCK_ps));             
      print_bw("tcore DB<->DRAM HOST",
          calc_bw_gbs(counters[9], dq_scaling, m_clk, tCK_ps));
      print_bw("tcore DB<->DRAM D2PA",
          calc_bw_gbs(counters[10], dq_scaling, m_clk, tCK_ps));
      print_bw("tcore DB<->DRAM NDP",
          calc_bw_gbs(counters[11], dq_scaling, m_clk, tCK_ps));

      std::cout << std::endl;

      // HSNC Segment Tracking Report
      double tCK_ns = (double)tCK_ps / 1000.0;
      std::cout<<"\n=== HSNC Segment Tracking Report (Host-side NDP Controller) ==="<<std::endl;
      for(int dimm_id=0;dimm_id<m_num_dimm;dimm_id++) {
        for(int pch_id=0;pch_id<(m_num_subch*num_pseudochannel);pch_id++) {
          auto& segs = hsnc_segments[dimm_id][pch_id];
          if (segs.empty()) continue;
          std::cout<<"\n--- DIMM["<<dimm_id<<"] PCH["<<pch_id<<"] ("<<segs.size()<<" segments) ---"<<std::endl;
          std::cout<<"  Seg | Type    |  RunStart  | DrainStart |    SegEnd  |  Total | Fetch | Drain"<<std::endl;
          std::cout<<"------+---------+------------+------------+-----------+--------+-------+------"<<std::endl;

          Clk_t total_rd_cycles = 0, total_wr_cycles = 0, total_wait_cycles = 0;
          int rd_cnt = 0, wr_cnt = 0, wait_cnt = 0;

          for (auto& s : segs) {
            Clk_t total  = s.end - s.run_start;
            Clk_t fetch  = (s.drain_start > 0) ? (s.drain_start - s.run_start) : total;
            Clk_t drain  = (s.drain_start > 0) ? (s.end - s.drain_start) : 0;
            std::cout<<"  "<<std::setw(3)<<s.seg_id<<" | "
                     <<std::setw(7)<<std::left<<hsnc_seg_type_str(s.type)<<std::right<<" | "
                     <<std::setw(10)<<s.run_start<<" | "
                     <<std::setw(10)<<((s.drain_start > 0) ? std::to_string(s.drain_start) : "-")<<" | "
                     <<std::setw(9)<<s.end<<" | "
                     <<std::setw(6)<<total<<" | "
                     <<std::setw(5)<<fetch<<" | "
                     <<std::setw(5)<<drain
                     <<std::endl;
            switch(s.type) {
              case HsncSegType::RD:   total_rd_cycles += total; rd_cnt++; break;
              case HsncSegType::WR:   total_wr_cycles += total; wr_cnt++; break;
              case HsncSegType::WAIT: total_wait_cycles += total; wait_cnt++; break;
              default: break;
            }
          }
          std::cout<<"------+---------+------------+------------+-----------+--------+-------+------"<<std::endl;
          Clk_t first_start = segs.front().run_start;
          Clk_t last_end = segs.back().end;
          Clk_t total_span = last_end - first_start;
          std::cout<<"  Total NDP span: "<<total_span<<" cycles ("
                   <<(double)total_span * tCK_ns<<" ns)"<<std::endl;
          if (rd_cnt > 0)
            std::cout<<"  RD segments:   "<<rd_cnt<<" segs, "<<total_rd_cycles<<" cycles (avg "<<(total_rd_cycles/rd_cnt)<<")"<<std::endl;
          if (wr_cnt > 0)
            std::cout<<"  WR segments:   "<<wr_cnt<<" segs, "<<total_wr_cycles<<" cycles (avg "<<(total_wr_cycles/wr_cnt)<<")"<<std::endl;
          if (wait_cnt > 0)
            std::cout<<"  WAIT segments: "<<wait_cnt<<" segs, "<<total_wait_cycles<<" cycles (avg "<<(total_wait_cycles/wait_cnt)<<")"<<std::endl;
        }
      }
      std::cout<<"=== End HSNC Segment Tracking Report ===\n"<<std::endl;

      // Descriptor Cache Summary
      std::cout<<"\n=== Descriptor Cache Summary ==="<<std::endl;
      for(int dimm_id=0;dimm_id<m_num_dimm;dimm_id++) {
        for(int pch_id=0;pch_id<(m_num_subch*num_pseudochannel);pch_id++) {
          uint64_t hit   = desc_cache_hit_cnt[dimm_id][pch_id];
          uint64_t miss  = desc_cache_miss_cnt[dimm_id][pch_id];
          if(hit + miss == 0) continue;
          float hit_rate = (float)hit / (float)(hit + miss) * 100.0f;
          std::cout<<"  DIMM["<<dimm_id<<"] PCH["<<pch_id<<"]: "
                   <<"hit="<<hit<<" miss="<<miss
                   <<" evict="<<desc_cache_evict_cnt[dimm_id][pch_id]
                   <<" hit_rate="<<std::fixed<<std::setprecision(2)<<hit_rate<<"%"
                   <<std::endl;
        }
      }
      std::cout<<"=== End Descriptor Cache Summary ===\n"<<std::endl;

      report();

    }

    // Copy Structure within loadstore_ncore_trace.cpp
    void load_trace(const std::string& file_path_str, std::vector<Trace>& trace_vec) {
      fs::path trace_path(file_path_str);
      if (!fs::exists(trace_path)) {
        throw ConfigurationError("Trace {} does not exist!", file_path_str);
      }

      std::ifstream trace_file(trace_path);
      if (!trace_file.is_open()) {
        throw ConfigurationError("Trace {} cannot be opened!", file_path_str);
      }

      std::string line;
      size_t line_number = 0;
      uint64_t default_timestamp = 0;  // For traces without timing info
      
      while (std::getline(trace_file, line)) {
        line_number++;
        
        // Skip empty lines and comments
        if (line.empty() || line[0] == '#') {
          continue;
        }
        
        Trace t;
        std::vector<std::string> tokens;
        tokenize(tokens, line, " ");

        if (tokens.empty()) {
          continue;
        }

        // Format: [TIMESTAMP] LD/ST ADDR [PAYLOAD...]
        // Or legacy: LD/ST ADDR [PAYLOAD...]
        
        size_t token_offset = 0;
        
        // Check if first token is a timestamp (number)
        if (tokens.size() >= 3 && std::isdigit(tokens[0][0])) {
          t.timestamp = std::stoull(tokens[0]);
          token_offset = 1;
        } else if (tokens.size() >= 2) {
          // Legacy format without timestamp - use sequential numbering
          t.timestamp = default_timestamp++;
          token_offset = 0;
        } else {
          throw ConfigurationError("Trace {} format invalid at line {}!", file_path_str, line_number);
        }

        // Parse operation
        bool is_write = false;
        if (tokens[token_offset] == "LD") {
          is_write = false;
        } else if (tokens[token_offset] == "ST") {
          is_write = true;
        } else {
          throw ConfigurationError("Trace {} format invalid at line {}! Unknown operation '{}'.", 
                                  file_path_str, line_number, tokens[token_offset]);
        }

        // Parse address
        Addr_t addr = -1;
        std::string addr_str = tokens[token_offset + 1];
        if (addr_str.compare(0, 2, "0x") == 0 || addr_str.compare(0, 2, "0X") == 0) {
          addr = std::stoll(addr_str.substr(2), nullptr, 16);
        } else {
          addr = std::stoll(addr_str);
        }

        t.is_write = is_write;
        t.addr = addr;

        // Parse payload if present (for writes)
        if (is_write && tokens.size() > token_offset + 2) {
          size_t payload_count = tokens.size() - (token_offset + 2);
          if (payload_count == 8) {  // Expecting 8 x 64-bit values
            for (size_t i = 0; i < 8; i++) {
              std::string payload_str = tokens[token_offset + 2 + i];
              if (payload_str.compare(0, 2, "0x") == 0 || payload_str.compare(0, 2, "0X") == 0) {
                t.payload.push_back(std::stoull(payload_str.substr(2), nullptr, 16));
              } else {
                t.payload.push_back(std::stoull(payload_str));
              }
            }
          }
        }

        trace_vec.push_back(t);
      }

      trace_file.close();
    }

    void try_issue_requests() {
      if (m_curr_trace_idx >= m_trace.size() && m_outstanding_reads.empty()) {
        if(m_ndp_trace) {
          bool all_ndp_req_done = false;
          if(all_ndp_idle) {
            all_ndp_req_done = true;
            // Check All Sub-Channel has no more NDP Request
            for(int dimm_id=0;dimm_id<m_num_dimm;dimm_id++) {
              for(int pch_id=0;pch_id<(m_num_subch*num_pseudochannel);pch_id++) {
                  int ch_id = (pch_id >= num_pseudochannel ? 1 : 0) + dimm_id * m_num_subch;
                  int pch_id_per_subch = pch_id%num_pseudochannel;
    
                  if(!(m_controllers[ch_id]->is_empty_ndp_req(pch_id_per_subch)))
                    all_ndp_req_done = false;
               }
            }
          }
          if(all_ndp_req_done) m_wait_trace_done++;
        } else {
          m_wait_trace_done++;
        }

        if(m_wait_trace_done > 1000) {
          m_curr_trace_idx = 0; 
          m_next_request_id = 0;
          m_wait_trace_done = 0;
        }   
      }

      while (m_curr_trace_idx < m_trace.size() && 
             m_outstanding_reads.size() < m_max_outstanding) {
        
        const Trace& t = m_trace[m_curr_trace_idx];
        
        // Check if it's time to issue this request
        if (t.timestamp > m_clk) {
          break;  // Too early for this request
        }

        // Create request
        Request req = Request(t.addr, t.is_write ? Request::Type::Write : Request::Type::Read);
        
        if (t.is_write && !t.payload.empty()) {
          for (uint32_t i = 0; i < t.payload.size(); i++) {
            req.m_payload.push_back(t.payload[i]);
          }
        }

        // Only set callback for READ requests
        int req_id = -1;
        if (!t.is_write) {
          req_id = m_next_request_id++;
          
          // Set callback for read completion
          req.callback = [this, req_id](Request& completed_req) {
            this->on_read_complete(req_id, completed_req);
          };
        }
        
        // Try to send the request
        req.is_trace_core_req = true;
        bool request_sent = send(req);
        
        if (request_sent) {
          if (t.is_write) {
            // Write is fire-and-forget
            if (m_debug_mode) {
              m_logger->debug("Trace issued WRITE (addr={:#x})",t.addr);
            }
          } else {
            // Track outstanding read
            OutstandingRequest out_req;
            out_req.issue_time = m_clk;
            out_req.addr = t.addr;
            
            m_outstanding_reads[req_id] = out_req;

            if (m_debug_mode) {
              m_logger->debug("Trace issued READ {} (addr={:#x}, outstanding={})", 
                             req_id, t.addr, m_outstanding_reads.size());
            }                           
          }
          
          m_curr_trace_idx++;
        } else {        
          break;  // Memory system busy, try next cycle
        }
      }
    }

    void on_read_complete(int request_id, Request& req) {
      auto it = m_outstanding_reads.find(request_id);
      
      if (it != m_outstanding_reads.end()) {
        uint64_t latency = m_clk - it->second.issue_time;        
        if (m_debug_mode) {
          m_logger->debug("Trace READ {} completed (addr={:#x}, latency={} cycles, outstanding={})", 
                         request_id, it->second.addr, latency, 
                         m_outstanding_reads.size() - 1);
        }
        
        m_outstanding_reads.erase(it);
      } else {
        m_logger->error("Trace attempted to complete unknown READ request {}", request_id);
      }
    }      

    static inline double calc_bw_gbs(uint64_t acc_cnt,
                                    double dq_scaling,
                                    uint64_t clk_cycles,
                                    uint64_t tCK_ps)
    {
        if (clk_cycles == 0 || tCK_ps == 0) return 0.0;

        // 512 bits/access -> bytes, time = clk * tCK (ps -> ns)
        const double bytes = (static_cast<double>(acc_cnt) * 512.0 * dq_scaling) / 8.0;
        const double time_ns = static_cast<double>(clk_cycles) * (static_cast<double>(tCK_ps) / 1000.0);
        return bytes / time_ns; // GB/s
    }

    static inline void print_bw(const std::string& name, double bw)
    {
        std::cout << std::left << std::setw(32) << name
                  << " : " << std::right << std::setw(10)
                  << std::fixed << std::setprecision(3)
                  << bw << " GB/s\n";
    }
    
    inline void record_latency(uint64_t lat) {
      ++total_reads_;
      sum_lat_ += lat;
      max_lat_ = std::max(max_lat_, lat);

      const uint64_t idx = lat / BIN_WIDTH;
      if (idx < NUM_BINS - 1) {
        hist_[idx] += 1;
      } else {
        hist_[NUM_BINS - 1] += 1;
        ++overflow_;
      }
    }

    uint64_t percentile_from_hist(double p) const {
      // Returns smallest latency L such that CDF(L) >= p
      const uint64_t total = total_reads_;
      const uint64_t target = (uint64_t)std::ceil(p * (double)total);
      uint64_t cdf = 0;
      for (uint32_t i = 0; i < NUM_BINS; ++i) {
        cdf += hist_[i];
        if (cdf >= target) {
          // convert bin index to representative latency
          if (i == NUM_BINS - 1) return MAX_LAT; // overflow bucket
          return (uint64_t)i * BIN_WIDTH;
        }
      }
      return MAX_LAT;
    }

    void report() const {
      const uint64_t total = total_reads_;
      std::cout << "READ latency samples: " << total << "\n";
      if (total == 0) return;

      std::cout << "  avg: "  << (double)sum_lat_ / (double)total << " cycles\n";
      std::cout << "  p50: "  << percentile_from_hist(0.50) << " cycles\n";
      std::cout << "  p95: "  << percentile_from_hist(0.95) << " cycles\n";
      std::cout << "  p99: "  << percentile_from_hist(0.99) << " cycles\n";
      std::cout << "  p999: " << percentile_from_hist(0.999) << " cycles\n";
      std::cout << "  max: "  << max_lat_ << " cycles\n";
      std::cout << "  overflow: " << overflow_ << "\n";
    }

};
  
}   // namespace 