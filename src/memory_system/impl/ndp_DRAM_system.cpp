#include "memory_system/memory_system.h"
#include "translation/translation.h"
#include "dram_controller/controller.h"
#include "addr_mapper/addr_mapper.h"
#include "dram/dram.h"

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
      NDP_RUN,
      NDP_BAR,
      NDP_DONE
    };
    NDP_CTRL_STATUS                    ndp_ctrl_status;
    int                                ndp_ctrl_pc;
    std::vector<AccInst_Slot>          ndp_access_infos;
    std::vector<AccInst_Slot>          ndp_access_inst_slots;
    int                                ndp_access_slot_idx;
    std::vector<bool>                  issue_ndp_start;
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
    bool ndp_on = false;
    bool wait_ndp_on = false;
    bool ndp_issued_start = false;

    int num_channels = -1;
    int num_pseudochannel = -1;
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

      if(num_pseudochannel == -1 ) num_pseudochannel = 1;
      
      // Calcuate Total Memory Capacity
      int num_dram_die = num_channels * num_pseudochannel * num_ranks * (m_dram->m_channel_width / m_dram->m_organization.dq);
      total_memory_capacity = (num_dram_die * m_dram->m_organization.density / 1024)/ 8;
      m_logger->info(" DRAM System Configuration");
      m_logger->info("   - # of Channels          : {}",num_channels);
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

      issue_ndp_start.resize(num_channels*num_pseudochannel,false);
      ndp_access_infos.resize(8*128,AccInst_Slot());
      ndp_ctrl_status = NDP_IDLE;
      ndp_ctrl_pc     = -1;
      // Create memory controllers
      for (int i = 0; i < num_channels; i++) {
        IDRAMController* controller = create_child_ifce<IDRAMController>();
        controller->m_impl->set_id(fmt::format("Channel {}", i));
        controller->m_channel_id = i;
        m_controllers.push_back(controller);
      }

      m_clock_ratio = param<uint>("clock_ratio").required();

      register_stat(m_clk).name("memory_system_cycles");
      register_stat(s_num_read_requests).name("total_num_read_requests");
      register_stat(s_num_write_requests).name("total_num_write_requests");
      register_stat(s_num_other_requests).name("total_num_other_requests");
    };

    void setup(IFrontEnd* frontend, IMemorySystem* memory_system) override { }

    bool send(Request req) override {
      m_addr_mapper->apply(req);
      
      bool is_success;
      if(req.addr_vec[row_addr_idx] == ndp_ctrl_row && 
        ((req.addr_vec[bk_addr_idx] == ndp_ctrl_bk     && req.addr_vec[bg_addr_idx] == ndp_ctrl_bg) || 
         (req.addr_vec[bk_addr_idx] == ndp_ctrl_buf_bk && req.addr_vec[bg_addr_idx] == ndp_ctrl_buf_bg))) {          
          is_success = send_ndp_ctrl(req);
      } else {
          int channel_id = req.addr_vec[0];
          // Check is NDP Request and set Request is_ndp_req as true
          if(req.addr_vec[row_addr_idx] == ndp_ctrl_row) req.is_ndp_req = true;
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

      // for(int i=0;i<req.addr_vec.size();i++) {
      //   std::cout<<m_dram->m_levels(i)<<" : "<<req.addr_vec[i]<<std::endl;
      // }

      return is_success;
    };
    
    void tick() override {
      m_clk++;
      m_dram->tick();
      for (auto controller : m_controllers) {
        controller->tick();
      }
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

            Request req = Request(0,Request::Type::Read);
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
            #ifdef NDP_DEBUG
            std::cout<<"[NDP_MEM_SYS] Send MC :";
            m_dram->print_req(req);
            #endif 
            issue_req = m_controllers[ndp_access_inst_slots[ndp_access_slot_idx].ch]->send(req);      
            if(issue_req) {
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
              if(access_inst.opcode == 3) {
                ndp_ctrl_status = NDP_BAR;
                #ifdef NDP_DEBUG
                std::cout<<"[NDP_MEM_SYS] NDP_CTRL Status NDP_RUN --> NDP_BAR"<<std::endl;
                #endif
              } else if(access_inst.opcode == 15) {
                ndp_ctrl_status = NDP_DONE;
                #ifdef NDP_DEBUG
                std::cout<<"[NDP_MEM_SYS] NDP_CTRL Status NDP_RUN --> NDP_DONE"<<std::endl;
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
    };

    float get_tCK() override {
      return m_dram->m_timing_vals("tCK_ps") / 1000.0f;
    }

    // const SpecDef& get_supported_requests() override {
    //   return m_dram->m_requests;
    // };
    bool send_ndp_ctrl(Request req) {
      
      #ifdef NDP_DEBUG
      std::cout<<"NDP Ctrl Get .. NDP Control Request"<<std::endl;
      #endif
      if(req.addr_vec[bk_addr_idx] == ndp_ctrl_bk && req.addr_vec[bg_addr_idx] == ndp_ctrl_bg) {
        // NDP Control 
        if(req.type_id == Request::Type::Write) {
          #ifdef NDP_DEBUG
          std::cout<<"[HSNU] payload [0]:"<<req.m_payload[0]<<std::endl;
          std::cout<<"[HSNU] payload [1]:"<<req.m_payload[1]<<std::endl;
          #endif 

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
        }
      } else {
        // NDP Access Info Buffer 
        if(req.type_id == Request::Type::Write) {
          if(req.m_payload.size() > 0) {
            for(uint32_t i=0;i<req.m_payload.size();i++) {
              ndp_access_infos[req.addr_vec[col_addr_idx]*8 + i] = decode_acc_inst(req.m_payload[i]);
            }
          }
        }
      }
      return true;
    }

    bool is_finished() override {
      bool is_ndp_finished = true;
      bool is_dram_ctrl_finished = true;
      int num_channels = m_dram->get_level_size("channel"); 
      for (int i = 0; i < num_channels; i++) {
        if(!m_controllers[i]->is_finished())
          is_dram_ctrl_finished = false;
      }
      if(ndp_on || wait_ndp_on) {
        is_ndp_finished = false;
      }

      if(is_ndp_finished && is_dram_ctrl_finished) {
        std::cout<<"All Request Done!!! (+NDP Ops)"<<std::endl;
      }
      return (is_ndp_finished && is_dram_ctrl_finished);
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
      std::cout<<"acc inst decoding opcode "<<opcode<<" opsize "<<opsize<<" ch "<<ch<<" pch "<<pch<<" bg "<<bg;
      std::cout<<" bk "<<bk<<" row "<<row<<" col "<<col<<" id "<<id<<std::endl;
      return AccInst_Slot(true,opcode,opsize,ch,pch,bg,bk,row,col,id);      
    }

    void print_acc_inst(const AccInst_Slot& slot) {
      std::cout<<"acc inst opcode "<<slot.opcode<<" opsize "<<slot.opsize<<" ch "<<slot.ch<<" pch "<<slot.pch<<" bg "<<slot.bg;
      std::cout<<" bk "<<slot.bk<<" row "<<slot.row<<" col "<<slot.col<<" id "<<slot.id<<std::endl;      
    }
};
  
}   // namespace 

