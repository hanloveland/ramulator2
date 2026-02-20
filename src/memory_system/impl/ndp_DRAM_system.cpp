#include "memory_system/memory_system.h"
#include "translation/translation.h"
#include "dram_controller/controller.h"
#include "addr_mapper/addr_mapper.h"
#include "dram/dram.h"
#include <filesystem>
#include <iostream>
#include <fstream>

// #define NDP_DEBUG
#define DIMM_LVL_BUF

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
      NDP_WAIT_RES,
      NDP_WAIT,
      NDP_DONE
    };
    // NDP Access Instruction Opcode
    std::map<std::string, int> m_ndp_access_inst_op = {
      {"RD",         0},
      {"WR",         1},
      {"BAR",        2},
      {"WAIT_RES",   3},
      {"LOOP_START", 4},
      {"LOOP_END",   5},
      {"WAIT",       6},
      {"DONE",       15}      
    };
    // DIMM-level request buffer 
    std::vector<std::vector<uint64_t>>                    dimm_lvl_req_buffer;
    std::vector<std::vector<std::vector<uint64_t>>>       pch_lvl_req_buffer;
    std::vector<std::vector<uint64_t>>                    dimm_lvl_req_pch_addr;
    int                                                   m_max_req_buffer_cap_per_dimm;
    int                                                   m_max_req_buffer_cap_per_pch;
    // PCH-level Status Reg pch_lvl_hsnc_status[DIMM_ID][PCH_ID]
    std::vector<std::vector<NDP_CTRL_STATUS>>             pch_lvl_hsnc_status;
    std::vector<std::vector<bool>>                        pch_lvl_polling;
    // PCH-level REQ slot pch_lvl_hsnc_nl_req_slot[DIMM_ID][PCH_ID][SLOT_IDX]
    std::vector<std::vector<std::vector<uint64_t>>>       pch_lvl_hsnc_nl_req_slot;
    int                                                   pch_lvl_hsnc_nl_req_slot_max;
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

    std::vector<std::vector<std::vector<size_t>>>         pch_hsnc_status_cnt;

    bool                                                  all_ndp_idle;
    bool                                                  all_nl_req_buffer_empty;


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

        Host issue NL-REQ (8 NL-REQs) to Buffer (128x64B) in DIMM-level HSNC 
        -> send(req) to #DIMM x BUF (Nx8x64)
        Host issue Control Request to DIMM-level HSNC
        -> update each PCH (all pch belong to same DIMM)

        tick() 
        loop all DIMMs 
          - each DIMM-level nl-req buf check whether the target pch has empty slot
        loop all pch ()
          - NDP_IDLE: Do Nothing 
          - NDP_ISSUE_START: Issue Start Request to NDP Unit
          - NDP_BEFORE_RUN: Wait NDP Start Request to be issued to NDP unit from Memory Controller
          - NDP_RUN: Issue NL-Req to NDP unit and fetch from nl-req slot
          - NDP_BAR: only issue NL-Req to NDP unit
          - NDP_WAIT_RES: wait read request response
          - NDP_WAIT: wait specific latency
          - NDP_DONE: All NL-Reqs are done (Do Nothing)

        Each pch-lvl HSNC
          - nl-req slot (x8) 
          - nl-req address genertor slot (x4)
          - RR(nl-req slot) -> decoding -> nl-req address genertor slot
          - wait counter/cycle (wait self-execution mode on NDP unit )
    */       
      all_ndp_idle = true;
      all_nl_req_buffer_empty = true;

      // Each DIMM has 1K (128x8) Element (MAX)
      dimm_lvl_req_buffer.resize(m_num_dimm,std::vector<uint64_t>(0,0));
      dimm_lvl_req_pch_addr.resize(m_num_dimm,std::vector<uint64_t>(0,0));
      pch_lvl_req_buffer.resize(m_num_dimm,std::vector<std::vector<uint64_t>>(m_num_subch*num_pseudochannel,std::vector<uint64_t>(0,0)));
      m_max_req_buffer_cap_per_dimm = 1024;
      m_max_req_buffer_cap_per_pch = m_max_req_buffer_cap_per_dimm / (m_num_subch*num_pseudochannel);

      // PCH_level Host-Side NDP Controller
      pch_lvl_hsnc_status.resize(m_num_dimm,std::vector<NDP_CTRL_STATUS>(m_num_subch*num_pseudochannel,NDP_IDLE));      
      pch_lvl_polling.resize(m_num_dimm,std::vector<bool>(m_num_subch*num_pseudochannel,false));      

      // nl-req slot at each pch Fixed to 8 (64B/64)
      pch_lvl_hsnc_nl_req_slot_max = 16;      
      pch_lvl_hsnc_nl_req_slot.resize(m_num_dimm,std::vector<std::vector<uint64_t>>(m_num_subch*num_pseudochannel,std::vector<uint64_t>(0,0)));

      // nl-req address generator at each pch 
      pch_lvl_hsnc_nl_addr_gen_slot_max = 8;      
      pch_lvl_hsnc_nl_addr_gen_slot.resize(m_num_dimm,std::vector<std::vector<AccInst_Slot>>(m_num_subch*num_pseudochannel,std::vector<AccInst_Slot>(0,AccInst_Slot())));

      // Round-Robin Index
      pch_lvl_hsnc_nl_addr_gen_slot_rr_idx.resize(m_num_dimm,std::vector<int>(m_num_subch*num_pseudochannel,0));

      // Address Generator Wait Counter and Wait Cycle
      pch_lvl_hsnc_nl_addr_gen_wait_cnt.resize(m_num_dimm,std::vector<int>(m_num_subch*num_pseudochannel,-1));
      pch_lvl_hsnc_nl_addr_gen_wait_cycle.resize(m_num_dimm,std::vector<int>(m_num_subch*num_pseudochannel,-1));    
      pch_lvl_hsnc_nl_addr_empty_cnt.resize(m_num_dimm,std::vector<int>(m_num_subch*num_pseudochannel,0));     
      pch_lvl_hsnc_nl_addr_wait_cnt.resize(m_num_dimm,std::vector<int>(m_num_subch*num_pseudochannel,0));     
      pch_hsnc_status_cnt.resize(m_num_dimm,std::vector<std::vector<size_t>>(m_num_subch*num_pseudochannel,std::vector<size_t>(8,0)));     
    

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
      }

      // Loop All DIMM 
      #ifdef NDP_DEBUG      
        if(m_clk%10000 == 0) {
          std::cout<<"dimm/pch_lvl_req_buffer"<<std::endl;
          for(int dimm_id=0;dimm_id<m_num_dimm;dimm_id++) {
            std::cout<<"  - DIMM["<<dimm_id<<"] : "<<dimm_lvl_req_buffer[dimm_id].size()<<std::endl;
            for(int pch_id=0;pch_id<(m_num_subch*num_pseudochannel);pch_id++) {
              std::cout<<"  - PCH["<<pch_id<<"] : "<<pch_lvl_hsnc_nl_req_slot[dimm_id][pch_id].size()<<std::endl;;            
            }
          }

          std::cout<<"Status"<<std::endl;
          for(int dimm_id=0;dimm_id<m_num_dimm;dimm_id++) {
            std::cout<<"  - DIMM["<<dimm_id<<"]"<<std::endl;
            for(int pch_id=0;pch_id<(m_num_subch*num_pseudochannel);pch_id++) {
              std::cout<<"  - PCH["<<pch_id<<"] : "<<pch_lvl_hsnc_status[dimm_id][pch_id]<<std::endl;;            
            }
          }                
        }
      #endif 
      bool dimm_lvl_buf_empty = true;
      #ifdef DIMM_LVL_BUF
        // DIMM-level Address Buffer
        for(int dimm_id=0;dimm_id<m_num_dimm;dimm_id++) {
          // Check to send nl-req to target pch 
          if(!dimm_lvl_req_pch_addr[dimm_id].empty()) {
            dimm_lvl_buf_empty = false;
            int pch_id = dimm_lvl_req_pch_addr[dimm_id][0];
            // if target pch is empty, fetch nl-req to target pch hsnc
            if((pch_lvl_hsnc_nl_req_slot_max - pch_lvl_hsnc_nl_req_slot[dimm_id][pch_id].size()) >= 8) {
              for(int i=0;i<8;i++) {
                if(dimm_lvl_req_buffer[dimm_id][0] != 0) 
                  pch_lvl_hsnc_nl_req_slot[dimm_id][pch_id].push_back(dimm_lvl_req_buffer[dimm_id][0]);
                // std::cout<<" nl_buf -> nl_req_slot :"<<std::hex<<dimm_lvl_req_buffer[dimm_id][0]<<std::endl;
                // remove 8 nl-req from dimm_lvl_req_buffer 
                dimm_lvl_req_buffer[dimm_id].erase(dimm_lvl_req_buffer[dimm_id].begin() + 0);
              }
              // remove target pch idx from dimm_lvl_req_pch_addr
              dimm_lvl_req_pch_addr[dimm_id].erase(dimm_lvl_req_pch_addr[dimm_id].begin() + 0);
            }
          } 
        }
      #else
        // PCH-level Address Buffer
        for(int dimm_id=0;dimm_id<m_num_dimm;dimm_id++) {
          // Check to send nl-req to target pch 
          for(int pch_id=0;pch_id<(m_num_subch*num_pseudochannel);pch_id++) {
            if(!pch_lvl_req_buffer[dimm_id][pch_id].empty()) {
              dimm_lvl_buf_empty = false;
              // if((pch_lvl_hsnc_nl_req_slot_max - pch_lvl_hsnc_nl_req_slot[dimm_id][pch_id].size()) >= 8) {
              if((pch_lvl_hsnc_nl_req_slot_max > pch_lvl_hsnc_nl_req_slot[dimm_id][pch_id].size())) {
                // for(int i=0;i<8;i++) {
                  pch_lvl_hsnc_nl_req_slot[dimm_id][pch_id].push_back(pch_lvl_req_buffer[dimm_id][pch_id][0]);
                  // remove 8 nl-req from pch_lvl_req_buffer 
                  pch_lvl_req_buffer[dimm_id][pch_id].erase(pch_lvl_req_buffer[dimm_id][pch_id].begin() + 0);
                // }              
              }
            }
          }
        }            
      #endif       

      for(int dimm_id=0;dimm_id<m_num_dimm;dimm_id++) {
        for(int pch_id=0;pch_id<(m_num_subch*num_pseudochannel);pch_id++) {
          if(pch_lvl_hsnc_nl_req_slot[dimm_id][pch_id].empty()) {
            pch_lvl_hsnc_nl_addr_empty_cnt[dimm_id][pch_id]++;
          }
        }
      }

      all_nl_req_buffer_empty = dimm_lvl_buf_empty;
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

            int ch_id = (pch_id >= num_pseudochannel ? 1 : 0) + dimm_id * m_num_subch;
            int pch_id_per_subch = pch_id%num_pseudochannel;
            // Check whether all NDP request are issued (in Read/Write Queue)
            if(m_controllers[ch_id]->is_empty_ndp_req(pch_id_per_subch)) {
              // Generatr NDP Start Write-Request 
              Request req = Request(0,Request::Type::Write);
              m_addr_mapper->apply(req);
              req.addr_vec[m_dram->m_levels("channel")]       = ch_id;
              req.addr_vec[m_dram->m_levels("pseudochannel")] = pch_id_per_subch;
              req.addr_vec[m_dram->m_levels("bankgroup")]     = db_ndp_ctrl_access_bg;
              req.addr_vec[m_dram->m_levels("bank")]          = db_ndp_ctrl_access_bk;
              req.addr_vec[m_dram->m_levels("row")]           = ndp_ctrl_row;
              req.is_ndp_req = true;
              for(int i=0;i<8;i++) {
                req.m_payload.push_back(1);
              }
              req.is_trace_core_req = true;
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

            int ch_id = (pch_id >= num_pseudochannel ? 1 : 0) + dimm_id * m_num_subch;
            int pch_id_per_subch = pch_id%num_pseudochannel;
            if(m_controllers[ch_id]->is_empty_ndp_req(pch_id_per_subch)) {
              pch_lvl_hsnc_status[dimm_id][pch_id] = NDP_RUN;
              DEBUG_PRINT(m_clk,"HSNC", dimm_id, pch_id, "transit from NDP_BEFORE_RUN to NDP_RUN");
            }
            pch_hsnc_status_cnt[dimm_id][pch_id][NDP_BEFORE_RUN]++;
          } // NDP Status: NDP_BEFORE_RUN END
          else if(pch_lvl_hsnc_status[dimm_id][pch_id] == NDP_RUN) {
            // NDP Status: NDP_RUN 

            // send NDP Request from NDP-lanuch Request and send to read/write queue of memory controller
            if(pch_lvl_hsnc_nl_addr_gen_slot[dimm_id][pch_id].size() != 0) send_ndp_req_to_mc(dimm_id,pch_id);
            
            // fetch nl_req from pch_lvl_hsnc_nl_req_slot and enque nl_req into pch_lvl_hsnc_nl_addr_gen_slot
            // if pch_lvl_hsnc_nl_addr_gen_slot has room and pch_lvl_hsnc_nl_req_slot has nl_req 
            if(pch_lvl_hsnc_nl_addr_gen_slot[dimm_id][pch_id].size() < pch_lvl_hsnc_nl_addr_gen_slot_max && pch_lvl_hsnc_nl_req_slot[dimm_id][pch_id].size() != 0) {
              DEBUG_PRINT(m_clk,"HSNC", dimm_id, pch_id, "Decoding NDP-NL Request");
              AccInst_Slot nl_req = decode_acc_inst(pch_lvl_hsnc_nl_req_slot[dimm_id][pch_id][0]);
              pch_lvl_hsnc_nl_req_slot[dimm_id][pch_id].erase(pch_lvl_hsnc_nl_req_slot[dimm_id][pch_id].begin() + 0);
              
              // NDP Status Transition 
              if(nl_req.opcode == m_ndp_access_inst_op.at("BAR")) {
                pch_lvl_hsnc_status[dimm_id][pch_id] = NDP_BAR;
                DEBUG_PRINT(m_clk,"HSNC", dimm_id, pch_id, "transit from NDP_RUN to NDP_BAR");
              } else if(nl_req.opcode == m_ndp_access_inst_op.at("WAIT_RES")) {
                pch_lvl_hsnc_status[dimm_id][pch_id] = NDP_WAIT_RES;
                DEBUG_PRINT(m_clk,"HSNC", dimm_id, pch_id, "transit from NDP_RUN to NDP_WAIT_RES");                
              } else if(nl_req.opcode == m_ndp_access_inst_op.at("WAIT")) {
                pch_lvl_hsnc_status[dimm_id][pch_id] = NDP_WAIT;
                pch_lvl_hsnc_nl_addr_gen_wait_cnt[dimm_id][pch_id] = 0;
                pch_lvl_hsnc_nl_addr_gen_wait_cycle[dimm_id][pch_id] = nl_req.etc;              
                DEBUG_PRINT(m_clk,"HSNC", dimm_id, pch_id, "transit from NDP_RUN to NDP_WAIT");                
              } else if(nl_req.opcode == m_ndp_access_inst_op.at("DONE")) {
                pch_lvl_hsnc_status[dimm_id][pch_id] = NDP_DONE;
                DEBUG_PRINT(m_clk,"HSNC", dimm_id, pch_id, "transit from NDP_RUN to NDP_DONE");
              } else if(nl_req.opcode == m_ndp_access_inst_op.at("LOOP_START")) {
                throw std::runtime_error("LOOP_START Not Implemented!!");
              } else if(nl_req.opcode == m_ndp_access_inst_op.at("LOOP_END")) {
                throw std::runtime_error("LOOP_END Not Implemented!!");
              } else if(nl_req.opcode == m_ndp_access_inst_op.at("RD") || nl_req.opcode == m_ndp_access_inst_op.at("WR")) {
                pch_lvl_hsnc_nl_addr_gen_slot[dimm_id][pch_id].push_back(nl_req);                
              } else {
                throw std::runtime_error("Invalid NDP-Lanuch Request Opcode!!");                
              }           
            }           
            pch_hsnc_status_cnt[dimm_id][pch_id][NDP_RUN]++;
          } // NDP Status: NDP_RUN END
          else if(pch_lvl_hsnc_status[dimm_id][pch_id] == NDP_BAR) {
            // NDP Status: NDP_BAR 

            int ch_id = (pch_id >= num_pseudochannel ? 1 : 0) + dimm_id * m_num_subch;
            int pch_id_per_subch = pch_id%num_pseudochannel;
            // Check whether all NDP request are issued (in Read/Write Queue)
            if(pch_lvl_hsnc_nl_addr_gen_slot[dimm_id][pch_id].size() == 0 && m_controllers[ch_id]->is_empty_ndp_req(pch_id_per_subch)) {
              pch_lvl_hsnc_status[dimm_id][pch_id] = NDP_RUN;
              DEBUG_PRINT(m_clk,"HSNC", dimm_id, pch_id, "transit from NDP_BAR to NDP_RUN");
            }
            
            // send NDP Request from NDP-lanuch Request and send to read/write queue of memory controller
            if(pch_lvl_hsnc_nl_addr_gen_slot[dimm_id][pch_id].size() != 0) send_ndp_req_to_mc(dimm_id,pch_id);
            pch_hsnc_status_cnt[dimm_id][pch_id][NDP_BAR]++;
          } // NDP Status: NDP_BAR END          
          else if(pch_lvl_hsnc_status[dimm_id][pch_id] == NDP_WAIT_RES) {
            // NDP Status: NDP_WAIT_RES 

            throw std::runtime_error("NDP_WAIT_RES Not Implemented!!");
            pch_hsnc_status_cnt[dimm_id][pch_id][NDP_WAIT_RES]++;
          } // NDP Status: NDP_WAIT_RES END             
          else if(pch_lvl_hsnc_status[dimm_id][pch_id] == NDP_WAIT) {
            pch_lvl_hsnc_nl_addr_wait_cnt[dimm_id][pch_id]++;
            // NDP Status: NDP_WAIT 
            if(pch_lvl_hsnc_nl_addr_gen_wait_cnt[dimm_id][pch_id] == pch_lvl_hsnc_nl_addr_gen_wait_cycle[dimm_id][pch_id]) {
              // Issue NDP RD for CONF_REG 
              int ch_id = (pch_id >= num_pseudochannel ? 1 : 0) + dimm_id * m_num_subch;
              int pch_id_per_subch = pch_id%num_pseudochannel;                
              if(pch_lvl_polling[dimm_id][pch_id]) {
                // issued NDP Polling Request and Wait
                int ndp_status = m_controllers[ch_id]->get_config_reg_resp(pch_id_per_subch);
                
                // If the ndp_status is capable of issuing a new reques
                if(ndp_status == -1) {
                  // Wait More Time to get response
                } else if(m_dram->is_ndp_issuable(ndp_status)) {
                  // std::cout<<"["<<m_clk<<"] HSNC NDP_WAIT --> to NDP_RUN"<<dimm_id<<"/ "<<pch_id<<std::endl;
                  pch_lvl_hsnc_status[dimm_id][pch_id] = NDP_RUN;
                  DEBUG_PRINT(m_clk,"HSNC", dimm_id, pch_id, "transit from NDP_WAIT to NDP_RUN");
                  pch_lvl_hsnc_nl_addr_gen_wait_cnt[dimm_id][pch_id] = -1;
                  pch_lvl_hsnc_nl_addr_gen_wait_cycle[dimm_id][pch_id] = -1;
                  pch_lvl_polling[dimm_id][pch_id] = false;
                } else {
                  // Need more time for NDP unit to issue new request and reset pch_lvl_polling to false
                  // std::cout<<"["<<m_clk<<"] HSNC NDP is not Ready (reissue NDP RD CONF REG)"<<dimm_id<<"/ "<<pch_id<<std::endl;
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
                  // std::cout<<"["<<m_clk<<"] HSNC Issue NDP RD CONFG REQ "<<dimm_id<<" / "<<pch_id<<std::endl;
                }                
              }                            
            } else {
              pch_lvl_hsnc_nl_addr_gen_wait_cnt[dimm_id][pch_id]++;
            }          
            pch_hsnc_status_cnt[dimm_id][pch_id][NDP_WAIT]++;
          } // NDP Status: NDP_WAIT END               
          else if(pch_lvl_hsnc_status[dimm_id][pch_id] == NDP_DONE) {
            // NDP Status: NDP_DONE 

            int ch_id = (pch_id >= num_pseudochannel ? 1 : 0) + dimm_id * m_num_subch;
            int pch_id_per_subch = pch_id%num_pseudochannel;
            // Check whether all NDP request are issued (in Read/Write Queue)
            if(pch_lvl_hsnc_nl_addr_gen_slot[dimm_id][pch_id].size() == 0 && m_controllers[ch_id]->is_empty_ndp_req(pch_id_per_subch)) {
              pch_lvl_hsnc_status[dimm_id][pch_id] = NDP_IDLE;
              DEBUG_PRINT(m_clk,"HSNC", dimm_id, pch_id, "transit from NDP_DONE to NDP_IDLE");
            }
            
            // send NDP Request from NDP-lanuch Request and send to read/write queue of memory controller
            if(pch_lvl_hsnc_nl_addr_gen_slot[dimm_id][pch_id].size() != 0) send_ndp_req_to_mc(dimm_id,pch_id);
            pch_hsnc_status_cnt[dimm_id][pch_id][NDP_DONE]++;
          } // NDP Status: NDP_WAIT END
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
        // NDP Control 
        int dimm_id = req.addr_vec[0]/2;
        DEBUG_PRINT(m_clk,"HSNC", dimm_id, 0, "NDP Ctrl Get NDP Control Request");
        if(req.type_id == Request::Type::Write) {
          #ifdef NDP_DEBUG
          std::cout<<"[HSNU] DIMM["<<dimm_id<<"][SCH0][PCH0]:"<<req.m_payload[0]<<std::endl;
          std::cout<<"[HSNU] DIMM["<<dimm_id<<"][SCH0][PCH1]:"<<req.m_payload[1]<<std::endl;
          std::cout<<"[HSNU] DIMM["<<dimm_id<<"][SCH0][PCH2]:"<<req.m_payload[2]<<std::endl;
          std::cout<<"[HSNU] DIMM["<<dimm_id<<"][SCH0][PCH3]:"<<req.m_payload[3]<<std::endl;
          std::cout<<"[HSNU] DIMM["<<dimm_id<<"][SCH1][PCH0]:"<<req.m_payload[4]<<std::endl;
          std::cout<<"[HSNU] DIMM["<<dimm_id<<"][SCH1][PCH1]:"<<req.m_payload[5]<<std::endl;
          std::cout<<"[HSNU] DIMM["<<dimm_id<<"][SCH1][PCH2]:"<<req.m_payload[6]<<std::endl;
          std::cout<<"[HSNU] DIMM["<<dimm_id<<"][SCH1][PCH3]:"<<req.m_payload[7]<<std::endl;
          #endif 

          // Update pch-lvl status reg in the dimm
          for(int i=0;i<pch_lvl_hsnc_status[dimm_id].size();i++) {
            if(pch_lvl_hsnc_status[dimm_id][i] != NDP_IDLE && req.m_payload[i] != 0) {
              throw std::runtime_error("Ovelap Write to NDP Control Register");   
            }
            if(req.m_payload[i] != 0) pch_lvl_hsnc_status[dimm_id][i] = NDP_ISSUE_START;
          }
        } else {
          // Read Access Req? 
          throw std::runtime_error("Not Allow NDP Controller Read (Not Implemented)");          
        }
      } else {
        // NDP Launch Request Buffer
        if(req.type_id == Request::Type::Write) {
          // Each DIMM has two Channel (i.e., sub-channel)
          int dimm_id = req.addr_vec[0]/2;
          int pch_id = ((req.addr_vec[0]%2 == 1) ? num_pseudochannel : 0) + req.addr_vec[m_dram->m_levels("pseudochannel")];                    
          #ifdef DIMM_LVL_BUF
            // DIMM-level Request Buffer          
            if(dimm_lvl_req_buffer[dimm_id].size() < m_max_req_buffer_cap_per_dimm) {
              // Each Channel has four pseudo-channels
              DEBUG_PRINT(m_clk,"HSNC", dimm_id, pch_id, "NDP Ctrl nl-req from host");
              dimm_lvl_req_pch_addr[dimm_id].push_back(pch_id);
              for(uint32_t i=0;i<req.m_payload.size();i++) {
                // std::cout<<"host -> nl_req_buf : 0x"<<std::hex<<req.m_payload[i]<<std::endl;
                dimm_lvl_req_buffer[dimm_id].push_back(req.m_payload[i]);
              }

              all_nl_req_buffer_empty = false;
              return true;
            } else {
              // DEBUG_PRINT(m_clk,"HSNC", dimm_id, pch_id, "DIMM-level NDP Launch Request Buffer FULL!");
              // dimm_level req_buffer is full
              return false;
            }
          #else 
            // PCH-level Request Buffer 
            if((m_max_req_buffer_cap_per_pch - pch_lvl_req_buffer[dimm_id][pch_id].size()) >= 8) {
              // Each Channel has four pseudo-channels
              DEBUG_PRINT(m_clk,"HSNC", dimm_id, pch_id, "NDP Ctrl nl-req from host");            
              for(uint32_t i=0;i<req.m_payload.size();i++) {
                pch_lvl_req_buffer[dimm_id][pch_id].push_back(req.m_payload[i]);
                // std::cout<<"host -> nl_req_buf : 0x"<<std::hex<<req.m_payload[i]<<std::endl;
              }

              all_nl_req_buffer_empty = false;
              return true;
            } else {
              // dimm_level req_buffer is full
              return false;
            }        
          #endif       
        } else {
          // Read Access Req? 
          throw std::runtime_error("Not Allow NL-Req Buffer Read (Not Implemented)");
        }
      }
      return true;
    }

    bool is_finished() override {      
      bool is_dram_ctrl_finished = true;
      int num_channels = m_dram->get_level_size("channel"); 
      for (int i = 0; i < num_channels; i++) {
        if(!m_controllers[i]->is_finished())
        is_dram_ctrl_finished = false;
      }        
      if(all_ndp_idle && is_dram_ctrl_finished && all_nl_req_buffer_empty) {
        int m_num_dimm   = num_channels / 2;
        int m_num_subch  = 2;
        int num_pseudochannel = m_dram->get_level_size("pseudochannel");         
        std::cout<<"["<<m_clk<<"] [NDP_DRAM_System] All Request Done!!! (+NDP Ops)"<<std::endl;

        std::cout<<"NDP Request Launch Empty Count ([DIMM][PCH])"<<std::endl;
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
        
        std::cout<<"Host-NDP Ctrl Status Stats (total cycle: "<<m_clk<<")"<<std::endl;
        std::cout<<"   - Each PCH per DIMM"<<std::endl;        
        std::cout<<"--------------------------------------------------------------------------------------------"<<std::endl;
        for(int ndp_stat=0;ndp_stat<8;ndp_stat++) {
          switch (ndp_stat)
          {
          case 0:
            std::cout<<"[NDP_IDLE]            | ";          
            break;          
          case 1:            
            std::cout<<"[NDP_ISSUE_START]     | ";
            break;
          case 2:            
            std::cout<<"[NDP_BEFORE_RUN]      | ";
            break;
          case 3:                        
            std::cout<<"[NDP_RUN]             | ";
            break;
            case 4:                                    
            std::cout<<"[NDP_BAR]             | ";
            break;
          case 5:
            std::cout<<"[NDP_WAIT_RES]        | ";
            break;
          case 6:
            std::cout<<"[NDP_WAIT]            | ";
            break;
          case 7:
            std::cout<<"[NDP_DONE]            | ";
            break;
          default:
            break;
          }
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
        for(int ndp_stat=0;ndp_stat<8;ndp_stat++) {
          switch (ndp_stat)
          {
          case 0:
            std::cout<<"[NDP_IDLE]            | ";          
            break;          
          case 1:            
            std::cout<<"[NDP_ISSUE_START]     | ";
            break;
          case 2:            
            std::cout<<"[NDP_BEFORE_RUN]      | ";
            break;
          case 3:                        
            std::cout<<"[NDP_RUN]             | ";
            break;
          case 4:                                    
            std::cout<<"[NDP_BAR]             | ";
            break;
          case 5:
            std::cout<<"[NDP_WAIT_RES]        | ";
            break;
          case 6:
            std::cout<<"[NDP_WAIT]            | ";
            break;
          case 7:
            std::cout<<"[NDP_DONE]            | ";
            break;
          default:
            break;
          }
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
         
      }
      if(m_trace_core_enable) return true;
      else                    return (all_ndp_idle && is_dram_ctrl_finished && all_nl_req_buffer_empty);
    }

    bool is_ndp_finished() override {
      if(m_trace_core_enable) 
        return true;
      else return all_ndp_idle;
    }     
    
    
    AccInst_Slot decode_acc_inst(uint64_t inst) {
      uint64_t opcode = (inst >> 60) & 0xf;
      uint64_t opsize = (inst >> 53) & 0x7f;
      uint64_t ch     = (inst >> 50) & 0x7;
      uint64_t pch    = (inst >> 48) & 0x3;
      uint64_t bg     = (inst >> 45) & 0x7;
      uint64_t bk     = (inst >> 43) & 0x3;
      uint64_t row    = (inst >> 25) & 0x3FFFF;
      uint64_t col    = (inst >> 18) & 0x7F;
      uint64_t id     = (inst >> 15) & 0x7;
      uint64_t etc    = (inst      ) & 0x7FFF;
      #ifdef NDP_DEBUG
        std::cout<<"acc inst decoding opcode "<<opcode<<" opsize "<<opsize<<" ch "<<ch<<" pch "<<pch<<" bg "<<bg;
        std::cout<<" bk "<<bk<<" row "<<row<<" col "<<col<<" id "<<id<<" etc "<<etc<<std::endl;
      #endif 
      return AccInst_Slot(true,opcode,opsize,ch,pch,bg,bk,row,col,id,etc);      
    }

    void print_acc_inst(const AccInst_Slot& slot) {
      std::cout<<"acc inst opcode "<<slot.opcode<<" opsize "<<slot.opsize<<" ch "<<slot.ch<<" pch "<<slot.pch<<" bg "<<slot.bg;
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
      // ---- Main window ----
      std::cout << "[Main window]\n";
      print_bw("Host<->DB",
          calc_bw_gbs(counters[0]+counters[1]+counters[2], 1.0, m_clk, tCK_ps));
      print_bw("Host<->DB HOST",
          calc_bw_gbs(counters[0], 1.0, m_clk, tCK_ps));
      print_bw("Host<->DB D2PA",
          calc_bw_gbs(counters[1], 1.0, m_clk, tCK_ps));
      print_bw("Host<->DB NDP",
          calc_bw_gbs(counters[2], 1.0, m_clk, tCK_ps));

      print_bw("DB<->DRAM",
          calc_bw_gbs(counters[3]+counters[4]+counters[5], dq_scaling, m_clk, tCK_ps));          
      print_bw("DB<->DRAM HOST",
          calc_bw_gbs(counters[3], dq_scaling, m_clk, tCK_ps));
      print_bw("DB<->DRAM D2PA",
          calc_bw_gbs(counters[4], dq_scaling, m_clk, tCK_ps));
      print_bw("DB<->DRAM NDP",
          calc_bw_gbs(counters[5], dq_scaling, m_clk, tCK_ps));

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
          if(all_ndp_idle && all_nl_req_buffer_empty) {
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
    

};
  
}   // namespace 