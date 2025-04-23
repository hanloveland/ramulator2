#include "memory_system/memory_system.h"
#include "translation/translation.h"
#include "dram_controller/controller.h"
#include "addr_mapper/addr_mapper.h"
#include "dram/dram.h"

// #define NDP_DEBUG

#ifdef NDP_DEBUG
#define DEBUG_PRINT(clk, unit_str, dimm_id, pch, msg) do { std::cout <<"["<<clk<<"]["<<unit_str<<"] DIMM["<<dimm_id<<"] PCH["<<pch<<"] "<<msg<<std::endl; } while(0)
#else
#define DEBUG_PRINT(clk, unit_str, ch, pch, msg) do {} while(0)
#endif


namespace Ramulator {

class NDPDRAMSystem final : public IMemorySystem, public Implementation {
  RAMULATOR_REGISTER_IMPLEMENTATION(IMemorySystem, NDPDRAMSystem, "ndpDRAM", "A NDP-Supported DRAM-based memory system.");

  protected:
    Clk_t m_clk = 0;
    IDRAM*  m_dram;
    IAddrMapper*  m_addr_mapper;
    std::vector<IDRAMController*> m_controllers;    

  public:
    int s_num_read_requests = 0;
    int s_num_write_requests = 0;
    int s_num_other_requests = 0;

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
    std::vector<std::vector<uint64_t>>                    dimm_lvl_req_pch_addr;
    int                                                   m_max_req_buffer_cap;
    // PCH-level Status Reg pch_lvl_hsnc_status[DIMM_ID][PCH_ID]
    std::vector<std::vector<NDP_CTRL_STATUS>>             pch_lvl_hsnc_status;
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
    int ndp_wait_cnt = -1;
    int ndp_wait_cycle = -1;
    */
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
      if(m_dram->m_organization.dq == 16) {
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
        ndp_ctrl_bk     = 3;
        ndp_ctrl_bg     = 7;
        ndp_ctrl_buf_bk = 3;
        ndp_ctrl_buf_bg = 6;
        // x4/x8 DRAM with 8BG and 4 BK per BG
        db_ndp_ctrl_access_bk      = 3;
        db_ndp_ctrl_access_bg      = 5;
        db_ndp_ins_mem_access_bk   = 3;
        db_ndp_ins_mem_access_bg   = 4;
        db_ndp_dat_mem_access_bk   = 2;
        db_ndp_dat_mem_access_bg   = 3; // BG0-BG3        
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
      col_addr_idx   = m_dram->m_levels("column");

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
      m_max_req_buffer_cap = 1024;

      // PCH_level Host-Side NDP Controller
      pch_lvl_hsnc_status.resize(m_num_dimm,std::vector<NDP_CTRL_STATUS>(m_num_subch*num_pseudochannel,NDP_IDLE));      

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
      } else {
          int channel_id = req.addr_vec[0];
          // Check is NDP Request and set Request is_ndp_req as true (DIMM-Side NDP Controller)
          if(req.addr_vec[row_addr_idx] == ndp_ctrl_row) {
            req.is_ndp_req = true;
            int dimm_id = req.addr_vec[0]/2;
            int pch_id = ((req.addr_vec[0]%2 == 1) ? num_pseudochannel : 0) + req.addr_vec[m_dram->m_levels("pseudochannel")];
            DEBUG_PRINT(m_clk,"HSNC", dimm_id, pch_id, "Host send NDP Request to NDP Unit");            
          } else {
            DEBUG_PRINT(m_clk,"HSNC", -1, -1, "Host send Normal Request to NDP Unit");
          }
          
          is_success = m_controllers[channel_id]->send(req);
      }
      
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

      return is_success;
    };
    
    void tick() override {
      m_clk++;
      m_dram->tick();
      for (auto controller : m_controllers) {
        controller->tick();
      }

      // Loop All DIMM 
      bool dimm_lvl_buf_empty = true;
      for(int dimm_id=0;dimm_id<m_num_dimm;dimm_id++) {
        // Check to send nl-req to target pch 
        if(!dimm_lvl_req_pch_addr[dimm_id].empty()) {
          dimm_lvl_buf_empty = false;
          int pch_id = dimm_lvl_req_pch_addr[dimm_id][0];
          // if target pch is empty, fetch nl-req to target pch hsnc
          if((pch_lvl_hsnc_nl_req_slot_max - pch_lvl_hsnc_nl_req_slot[dimm_id][pch_id].size()) >= 8) {
            for(int i=0;i<8;i++) {
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
              if(m_controllers[ch_id]->send(req)) {
                // NDP Status transit from NDP_ISSUE_START to NDP_BEFORE_RUN
                pch_lvl_hsnc_status[dimm_id][pch_id] = NDP_BEFORE_RUN;
                DEBUG_PRINT(m_clk,"HSNC", dimm_id, pch_id, "transit from NDP_ISSUE_START to NDP_BEFORE_RUN");
              }
            }

          } // NDP Status: NDP_ISSUE_START END
          else if(pch_lvl_hsnc_status[dimm_id][pch_id] == NDP_BEFORE_RUN) {
            // NDP Status: NDP_BEFORE_RUN 

            int ch_id = (pch_id >= num_pseudochannel ? 1 : 0) + dimm_id * m_num_subch;
            int pch_id_per_subch = pch_id%num_pseudochannel;
            if(m_controllers[ch_id]->is_empty_ndp_req(pch_id_per_subch)) {
              pch_lvl_hsnc_status[dimm_id][pch_id] = NDP_RUN;
              DEBUG_PRINT(m_clk,"HSNC", dimm_id, pch_id, "transit from NDP_BEFORE_RUN to NDP_RUN");
            }
          } // NDP Status: NDP_BEFORE_RUN END
          else if(pch_lvl_hsnc_status[dimm_id][pch_id] == NDP_RUN) {
            // NDP Status: NDP_RUN 

            // send NDP Request from NDP-lanuch Request and send to read/write queue of memory controller
            if(pch_lvl_hsnc_nl_addr_gen_slot[dimm_id][pch_id].size() != 0) send_ndp_req_to_mc(dimm_id,pch_id);
            
            // fetch nl_req from pch_lvl_hsnc_nl_req_slot and enque nl_req into pch_lvl_hsnc_nl_addr_gen_slot
            // if pch_lvl_hsnc_nl_addr_gen_slot has room and pch_lvl_hsnc_nl_req_slot has nl_req 
            if(pch_lvl_hsnc_nl_addr_gen_slot[dimm_id][pch_id].size() < pch_lvl_hsnc_nl_addr_gen_slot_max && pch_lvl_hsnc_nl_req_slot[dimm_id][pch_id].size() != 0) {
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
                pch_lvl_hsnc_status[dimm_id][pch_id] = NDP_WAIT_RES;
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

          } // NDP Status: NDP_BAR END          
          else if(pch_lvl_hsnc_status[dimm_id][pch_id] == NDP_WAIT_RES) {
            // NDP Status: NDP_WAIT_RES 

            throw std::runtime_error("NDP_WAIT_RES Not Implemented!!");

          } // NDP Status: NDP_WAIT_RES END             
          else if(pch_lvl_hsnc_status[dimm_id][pch_id] == NDP_WAIT) {
            // NDP Status: NDP_WAIT 
            if(pch_lvl_hsnc_nl_addr_gen_wait_cnt[dimm_id][pch_id] == pch_lvl_hsnc_nl_addr_gen_wait_cycle[dimm_id][pch_id]) {
              pch_lvl_hsnc_status[dimm_id][pch_id] = NDP_RUN;
              DEBUG_PRINT(m_clk,"HSNC", dimm_id, pch_id, "transit from NDP_WAIT to NDP_RUN");
              pch_lvl_hsnc_nl_addr_gen_wait_cnt[dimm_id][pch_id] = -1;
              pch_lvl_hsnc_nl_addr_gen_wait_cycle[dimm_id][pch_id] = -1;
            } else {
              pch_lvl_hsnc_nl_addr_gen_wait_cnt[dimm_id][pch_id]++;
            }          
            
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
            
          } // NDP Status: NDP_WAIT END
        } // LOOP-PCH END
      } // LOOP-DIMM END
      all_ndp_idle = pch_lvl_ndp_idle;
      /*
      deprecated code
      if(ndp_on) {
        if(!ndp_issued_start) {
          // Send Start WR Command to each NDP Unit
          bool issued_all_start_req = true;
          bool issu_try_req = false;
          for(int ch=0;ch<num_channels;ch++) {
            for(int pch=0;pch<num_pseudochannel;pch++) {
              int pch_idx = ch*num_pseudochannel + pch;
              if(!issue_ndp_start[pch_idx]) {
                issued_all_start_req = false;
                issu_try_req         = true;
                Request req = Request(0,Request::Type::Write);
                m_addr_mapper->apply(req);
                req.addr_vec[m_dram->m_levels("channel")]       = ch;
                req.addr_vec[m_dram->m_levels("pseudochannel")] = pch;
                req.addr_vec[m_dram->m_levels("bankgroup")]     = db_ndp_ctrl_access_bg;
                req.addr_vec[m_dram->m_levels("bank")]          = db_ndp_ctrl_access_bk;
                req.addr_vec[m_dram->m_levels("row")]           = ndp_ctrl_row;
                req.is_ndp_req = true;
                for(int i=0;i<8;i++) {
                  req.m_payload.push_back(1);
                }
                bool issue_req;
                issue_req = m_controllers[ch]->send(req);
                // Issue
                if(issue_req) issue_ndp_start[pch_idx] = true;
              }                      
              if(issu_try_req) break;
            }
            if(issu_try_req) break;
          }                
          if(issued_all_start_req) {
            ndp_issued_start    = true;
            ndp_ctrl_status     = NDP_RUN;
            ndp_ctrl_pc         = 0;
            ndp_access_slot_idx = 0;
            #ifdef NDP_DEBUG
            std::cout<<"["<<m_clk<<"]";
            std::cout<<"[HSNU] Send start request to all target pseudo-channels"<<std::endl;           
            #endif 
          }
        } else {
          if(ndp_ctrl_status != NDP_IDLE && ndp_access_inst_slots.size() != 0) {
            // Generate NDP EXEC REQ and Send to MCs
            if(ndp_access_slot_idx >= ndp_access_inst_slots.size()) ndp_access_slot_idx = 0;

            bool is_read;
            if (ndp_access_inst_slots[ndp_access_slot_idx].opcode == m_ndp_access_inst_op.at("RD")) 
              is_read = true;
            else if (ndp_access_inst_slots[ndp_access_slot_idx].opcode == m_ndp_access_inst_op.at("WR")) 
              is_read = false;
            else 
              throw std::runtime_error("Invalid NDP Access Instruction during.. NDP Access");
            
            Request req = Request(0,is_read ? Request::Type::Read : Request::Type::Write);
            m_addr_mapper->apply(req);
            req.addr_vec[m_dram->m_levels("channel")]       = ndp_access_inst_slots[ndp_access_slot_idx].ch;
            req.addr_vec[m_dram->m_levels("pseudochannel")] = ndp_access_inst_slots[ndp_access_slot_idx].pch;
            req.addr_vec[m_dram->m_levels("bankgroup")]     = ndp_access_inst_slots[ndp_access_slot_idx].bg;
            req.addr_vec[m_dram->m_levels("bank")]          = ndp_access_inst_slots[ndp_access_slot_idx].bk;
            req.addr_vec[m_dram->m_levels("row")]           = ndp_access_inst_slots[ndp_access_slot_idx].row;
            req.addr_vec[m_dram->m_levels("column")]        = ndp_access_inst_slots[ndp_access_slot_idx].col;
            req.ndp_id                                      = ndp_access_inst_slots[ndp_access_slot_idx].id;
            req.is_ndp_req                                  = true;

            bool issue_req;
            issue_req = m_controllers[ndp_access_inst_slots[ndp_access_slot_idx].ch]->send(req);      
            if(issue_req) {
              #ifdef NDP_DEBUG
              std::cout<<"[NDP_MEM_SYS] Send MC :";
              m_dram->print_req(req);
              #endif 
              if(ndp_access_inst_slots[ndp_access_slot_idx].cnt == ndp_access_inst_slots[ndp_access_slot_idx].opsize) {
                // Remove Done Request
                ndp_access_inst_slots.erase(ndp_access_inst_slots.begin() + ndp_access_slot_idx);
                #ifdef NDP_DEBUG
                std::cout<<"[NDP_MEM_SYS] One Access Instruction Done! Remove from inst_slots"<<std::endl;
                #endif
              } else {
                ndp_access_inst_slots[ndp_access_slot_idx].cnt++;
                ndp_access_inst_slots[ndp_access_slot_idx].col++;
                // Round-Robin
                ndp_access_slot_idx++;
              }
            }
          }

          if(ndp_ctrl_status == NDP_RUN) {
            if(ndp_access_inst_slots.size()<16) {
              #ifdef NDP_DEBUG
              std::cout<<"[NDP_MEM_SYS] NDP_CTRL_PC: "<<ndp_ctrl_pc<<std::endl;
              #endif 
              AccInst_Slot access_inst = ndp_access_infos[ndp_ctrl_pc];
              if(access_inst.opcode == m_ndp_access_inst_op.at("BAR")) {
                ndp_ctrl_status = NDP_BAR;
                #ifdef NDP_DEBUG
                std::cout<<"[NDP_MEM_SYS] NDP_CTRL Status NDP_RUN --> NDP_BAR"<<std::endl;
                #endif
              } else if(access_inst.opcode == m_ndp_access_inst_op.at("DONE")) {
                ndp_ctrl_status = NDP_DONE;
                #ifdef NDP_DEBUG
                std::cout<<"[NDP_MEM_SYS] NDP_CTRL Status NDP_RUN --> NDP_DONE"<<std::endl;
                #endif
              } else if(access_inst.opcode == m_ndp_access_inst_op.at("WAIT_RES")) {
                ndp_ctrl_status = NDP_WAIT_RES;
                #ifdef NDP_DEBUG
                std::cout<<"[NDP_MEM_SYS] NDP_CTRL Status NDP_RUN --> WAIT_RES"<<std::endl;
                #endif        
              } else if(access_inst.opcode == m_ndp_access_inst_op.at("WAIT")) {
                ndp_ctrl_status = NDP_WAIT;
                ndp_wait_cnt   = 0;
                ndp_wait_cycle = access_inst.etc;                
                #ifdef NDP_DEBUG
                std::cout<<"[NDP_MEM_SYS] NDP_CTRL Status NDP_RUN --> NDP_WAIT ("<<ndp_wait_cycle<<")"<<std::endl;
                #endif                         
              } else {
                ndp_access_inst_slots.push_back(access_inst);
              }
              ndp_ctrl_pc++;
            }            
          } else if(ndp_ctrl_status == NDP_BAR) {
            bool is_empty_ndp_req = true;
            for (auto controller : m_controllers) {
              if(!(controller->is_empty_ndp_req())) is_empty_ndp_req = false;
            }
            if(is_empty_ndp_req && ndp_access_inst_slots.size() == 0) {
              ndp_ctrl_status = NDP_RUN;
              #ifdef NDP_DEUBG
              std::cout<<"[NDP_MEM_SYS] NDP_CTRL Status NDP_BAR --> NDP_RUN"<<std::endl;
              #endif
            }
          } else if(ndp_ctrl_status == NDP_DONE) {
            bool is_empty_ndp_req = true;
            for (auto controller : m_controllers) {
              if(!(controller->is_empty_ndp_req())) is_empty_ndp_req = false;
            }
            if(is_empty_ndp_req && ndp_access_inst_slots.size() == 0) {
              ndp_ctrl_status = NDP_IDLE;
              ndp_on = false;
              #ifdef NDP_DEUBG
              std::cout<<"[NDP_MEM_SYS] NDP_CTRL Status NDP_DONE --> NDP_IDLE"<<std::endl;
              #endif
            }
          } else if(ndp_ctrl_status == NDP_WAIT_RES) {
            // Not Imple..
          } else if(ndp_ctrl_status == NDP_WAIT) {
            // Wait Fixed Latency
            if(ndp_wait_cnt == ndp_wait_cycle) {
              ndp_ctrl_status = NDP_RUN;
              #ifdef NDP_DEUBG
              std::cout<<"[NDP_MEM_SYS] NDP_CTRL Status NDP_WAIT --> NDP_RUN"<<std::endl;
              #endif
            } else {
              ndp_wait_cnt++;
            }           
          }


        }
      }
      //
      if(wait_ndp_on) {
        // Try Check NDP Request in R/W Request Buffer
        bool is_empty_ndp_req = true;
        for (auto controller : m_controllers) {
          if(!(controller->is_empty_ndp_req())) is_empty_ndp_req = false;
        }
        if(is_empty_ndp_req) {
          wait_ndp_on = false;
          ndp_on      = true;          
          #ifdef NDP_DEBUG
          std::cout<<"["<<m_clk<<"]";
          std::cout<<"[HSNU] Start NDP_ON ["<<ndp_on<<"] WAIT_NDP_ON ["<<wait_ndp_on<<"]"<<std::endl;          
          #endif 
        }
      }
      */
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

          /*
          // Deprecated Code
          if(req.m_payload[0] == 1) {
            bool is_empty_ndp_req = true;
            for (auto controller : m_controllers) {
              if(!(controller->is_empty_ndp_req())) is_empty_ndp_req = false;
            }
            if(!(wait_ndp_on || ndp_on)) {        
              if(is_empty_ndp_req) {
                wait_ndp_on      = true;
                ndp_on           = false;
                ndp_issued_start = false;
              } else {
                wait_ndp_on      = true;
                ndp_on           = false;
                ndp_issued_start = false;
              }
              #ifdef NDP_DEBUG
              std::cout<<"["<<m_clk<<"]";
              std::cout<<"[HSNU] Get NDP Start Request - NDP_ON ["<<ndp_on<<"] WAIT_NDP_ON ["<<wait_ndp_on<<"]"<<std::endl;
              #endif

              for(int i=0;i<issue_ndp_start.size();i++) {
                if(((req.m_payload[1]>>i) & 0x1) == 0x1) issue_ndp_start[i] = false;
                else                                     issue_ndp_start[i] = true;
              }              
              
              #ifdef NDP_DEBUG
              std::cout<<"[HSNU] Print Acc Inst. "<<std::endl;
              for(int i=0;i<18;i++) {
                std::cout<<"["<<i<<"]";
                print_acc_inst(ndp_access_infos[i]);
              }
              #endif 
            }
          }
          */
        } else {
          // Read Access Req? 
          throw std::runtime_error("Not Allow NDP Controller Read (Not Implemented)");          
        }
      } else {
        // NDP Launch Request Buffer
        if(req.type_id == Request::Type::Write) {
          // Each DIMM has two Channel (i.e., sub-channel)
          int dimm_id = req.addr_vec[0]/2;
          if(dimm_lvl_req_buffer[dimm_id].size() < m_max_req_buffer_cap) {
            // Each Channel has four pseudo-channels
            int pch_id = ((req.addr_vec[0]%2 == 1) ? num_pseudochannel : 0) + req.addr_vec[m_dram->m_levels("pseudochannel")];
            DEBUG_PRINT(m_clk,"HSNC", dimm_id, pch_id, "NDP Ctrl nl-req from host");
            dimm_lvl_req_pch_addr[dimm_id].push_back(pch_id);
            for(uint32_t i=0;i<req.m_payload.size();i++) {
              dimm_lvl_req_buffer[dimm_id].push_back(req.m_payload[i]);
              // std::cout<<"host -> nl_req_buf : 0x"<<std::hex<<req.m_payload[i]<<std::endl;
            }

            all_nl_req_buffer_empty = false;
            return true;
          } else {
            // dimm_level req_buffer is full
            return false;
          }

          // Deprecated Code
          /*
          if(req.m_payload.size() > 0) {
            for(uint32_t i=0;i<req.m_payload.size();i++) {
              ndp_access_infos[req.addr_vec[col_addr_idx]*8 + i] = decode_acc_inst(req.m_payload[i]);
            }
          }
          */

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
        std::cout<<"["<<m_clk<<"] [NDP_DRAM_System] All Request Done!!! (+NDP Ops)"<<std::endl;
      }
      return (all_ndp_idle && is_dram_ctrl_finished && all_nl_req_buffer_empty);
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

      int slot_idx = pch_lvl_hsnc_nl_addr_gen_slot_rr_idx[dimm_id][pch_id];        
      
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

      // Send NDP-Launch Request to Memory Controller 
      bool issue_req;
      issue_req = m_controllers[ch_id]->send(req);            
      if(issue_req) {
        #ifdef NDP_DEBUG
        std::cout<<"["<<m_clk<<"] [HSNC] Send : ";
        m_dram->print_req(req);
        #endif         

        if(pch_lvl_hsnc_nl_addr_gen_slot[dimm_id][pch_id][slot_idx].cnt == pch_lvl_hsnc_nl_addr_gen_slot[dimm_id][pch_id][slot_idx].opsize) {
          // Remove Done Request
          pch_lvl_hsnc_nl_addr_gen_slot[dimm_id][pch_id].erase(pch_lvl_hsnc_nl_addr_gen_slot[dimm_id][pch_id].begin() + slot_idx);
          DEBUG_PRINT(m_clk,"HSNC", dimm_id, pch_id, "One NL-Req Done! Remove from addresss Generator Slot");
        } else {
          pch_lvl_hsnc_nl_addr_gen_slot[dimm_id][pch_id][slot_idx].cnt++;
          pch_lvl_hsnc_nl_addr_gen_slot[dimm_id][pch_id][slot_idx].col++;
          // Round-Robin
          pch_lvl_hsnc_nl_addr_gen_slot_rr_idx[dimm_id][pch_id]++;
        }

      }
    }
};
  
}   // namespace 