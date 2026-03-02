#include <vector>

#include "base/base.h"
#include "dram_controller/controller.h"
#include "dram_controller/scheduler.h"

namespace Ramulator {

class NDPFRFCFS : public IScheduler, public Implementation {
  RAMULATOR_REGISTER_IMPLEMENTATION(IScheduler, NDPFRFCFS, "NDPFRFCFS", "NDPFRFCFS DRAM Scheduler.")
  private:
    IDRAM* m_dram;
    std::vector<int> m_cmd_priority_list;             // Command priority list
    std::unordered_map<int, int> m_cmd_priority_map;  // Command -> priority mapping
    std::array<std::vector<int>, 7> m_cmd_prio_luts;

    std::vector<int> m_open_row;
    std::vector<int> m_pre_open_row;
    std::vector<bool> m_open_row_miss;
    std::vector<bool> m_open_idle;
    
    int num_pseudochannel = -1;    
    int num_bankgroup = -1;
    int num_bank = -1;
    int psuedo_ch_idx = 0;
    int bankgroup_idx = 0;
    int bank_idx = 0;
    int row_idx = 0;

    struct CmdIds {
      int RD = -1;
      int RDA = -1;
      int WR = -1;
      int WRA = -1;
      int NDP_DB_RD = -1;
      int NDP_DB_WR = -1;
      int NDP_DRAM_RD = -1;
      int NDP_DRAM_RDA = -1;
      int NDP_DRAM_WR = -1;
      int NDP_DRAM_WRA = -1;
    };    
    CmdIds m_cmd;
    uint32_t m_prio_idx = -1;    
  public:
    void init() override { };

    void setup(IFrontEnd* frontend, IMemorySystem* memory_system) override {
      m_dram = cast_parent<IDRAMController>()->m_dram;
      
      m_cmd.RD = m_dram->m_commands("RD");
      m_cmd.RDA = m_dram->m_commands("RDA");
      m_cmd.WR = m_dram->m_commands("WR");
      m_cmd.WRA = m_dram->m_commands("WRA");
      m_cmd.NDP_DB_RD = m_dram->m_commands("NDP_DB_RD");
      m_cmd.NDP_DB_WR = m_dram->m_commands("NDP_DB_WR");
      m_cmd.NDP_DRAM_RD = m_dram->m_commands("NDP_DRAM_RD");
      m_cmd.NDP_DRAM_RDA = m_dram->m_commands("NDP_DRAM_RDA");
      m_cmd.NDP_DRAM_WR = m_dram->m_commands("NDP_DRAM_WR");
      m_cmd.NDP_DRAM_WRA = m_dram->m_commands("NDP_DRAM_WRA");

      // Init m_cmd_prio_luts 
      // m_cmd_prio_luts[0]
      m_cmd_prio_luts[0].assign(m_dram->m_commands.size(), 0);
      m_cmd_prio_luts[0][m_cmd.RD] = 1;
      // m_cmd_prio_luts[1] 
      m_cmd_prio_luts[1].assign(m_dram->m_commands.size(), 0);
      m_cmd_prio_luts[1][m_cmd.NDP_DB_RD] = 1;
      // m_cmd_prio_luts[2] 
      m_cmd_prio_luts[2].assign(m_dram->m_commands.size(), 0);
      m_cmd_prio_luts[2][m_cmd.NDP_DB_WR] = 1;
      // m_cmd_prio_luts[3] 
      m_cmd_prio_luts[3].assign(m_dram->m_commands.size(), 0);
      m_cmd_prio_luts[3][m_cmd.NDP_DRAM_RD] = 2;      
      m_cmd_prio_luts[3][m_cmd.NDP_DRAM_RDA] = 1;      
      // m_cmd_prio_luts[4] 
      m_cmd_prio_luts[4].assign(m_dram->m_commands.size(), 0);
      m_cmd_prio_luts[4][m_cmd.NDP_DRAM_WR] = 2;      
      m_cmd_prio_luts[4][m_cmd.NDP_DRAM_WRA] = 1;            
      // m_cmd_prio_luts[5] 
      m_cmd_prio_luts[5].assign(m_dram->m_commands.size(), 0);
      m_cmd_prio_luts[5][m_cmd.NDP_DB_RD] = 3;      
      m_cmd_prio_luts[5][m_cmd.NDP_DRAM_RD] = 2;      
      m_cmd_prio_luts[5][m_cmd.NDP_DRAM_RDA] = 1;             
      // m_cmd_prio_luts[6] 
      m_cmd_prio_luts[6].assign(m_dram->m_commands.size(), 0);   
      m_cmd_prio_luts[6][m_cmd.WR] = 1;     
    
      num_pseudochannel = m_dram->get_level_size("pseudochannel");  
      num_bankgroup = m_dram->get_level_size("bankgroup");  
      num_bank = m_dram->get_level_size("bank");  
      psuedo_ch_idx = m_dram->m_levels("pseudochannel");
      bankgroup_idx = m_dram->m_levels("bankgroup");
      bank_idx = m_dram->m_levels("bank"); 
      row_idx = m_dram->m_levels("row"); 

      // Record Row address per bank for Adaptive Open-Page Policy
      m_open_row.resize(num_pseudochannel*num_bankgroup*num_bank, -1);
      m_pre_open_row.resize(num_pseudochannel*num_bankgroup*num_bank, -1);
      m_open_row_miss.resize(num_pseudochannel*num_bankgroup*num_bank, false);      
      m_open_idle.resize(num_pseudochannel*num_bankgroup*num_bank, true);  
            
    };

    void update_open_row(int flat_bank_id, int row) override {
      m_open_row[flat_bank_id] = row;
    };

    void update_pre_open_row(int flat_bank_id, int row) override {
      m_pre_open_row[flat_bank_id] = row;
    };

    void update_open_row_miss(int flat_bank_id, bool miss) override {
      m_open_row_miss[flat_bank_id] = miss;
    };    

    void update_bk_status(int flat_bank_id, bool idle) override {
      m_open_idle[flat_bank_id] = idle;
    };   

    void set_command_priority(const std::vector<int>& cmd_list) {
        // m_cmd_priority_list = cmd_list;
        // m_cmd_priority_map.clear();
        
        // // Higher index = higher priority
        // for (size_t i = 0; i < cmd_list.size(); i++) {
        //     m_cmd_priority_map[cmd_list[i]] = cmd_list.size() - i;
        // }
    };
    
    int get_command_priority(const int cmd) const {
        return m_cmd_prio_luts[m_prio_idx][cmd];        
    };

    ReqBuffer::iterator compare(ReqBuffer::iterator req1, ReqBuffer::iterator req2, bool req1_ready, bool req2_ready) override {
      bool ready1;
      bool ready2;
  
      bool req1_not_low_pri = true;
      int req1_flat_bank_id = req1->addr_vec[bank_idx] + req1->addr_vec[bankgroup_idx] * num_bank + req1->addr_vec[psuedo_ch_idx] * num_bankgroup*num_bank;  
      if(req1->command == m_dram->m_commands("ACT") && m_open_idle[req1_flat_bank_id]) {
        
        if(m_open_row_miss[req1_flat_bank_id] && m_pre_open_row[req1_flat_bank_id] == req1->addr_vec[row_idx]) {
          req1_not_low_pri = false;
        }
      }

      bool req2_not_low_pri = true;
      int req2_flat_bank_id = req2->addr_vec[bank_idx] + req2->addr_vec[bankgroup_idx] * num_bank + req2->addr_vec[psuedo_ch_idx] * num_bankgroup*num_bank;  
      if(req2->command == m_dram->m_commands("ACT") && m_open_idle[req2_flat_bank_id]) {
        
        if(m_open_row_miss[req2_flat_bank_id] && m_pre_open_row[req2_flat_bank_id] == req2->addr_vec[row_idx]) {
          req2_not_low_pri = false;
        }
      }

      ready1 = req1_not_low_pri && m_dram->check_ready(req1->command, req1->addr_vec);
      ready2 = req2_not_low_pri && m_dram->check_ready(req2->command, req2->addr_vec);

      ready1 = ready1 && req1_ready;
      ready2 = ready2 && req2_ready;

      if (ready1 ^ ready2) {
        if (ready1) {
          return req1;
        } else {
          return req2;
        }
      }

      // Fallback to FCFS
      if (req1->arrive <= req2->arrive) {
        return req1;
      } else {
        return req2;
      } 
    }
  
    ReqBuffer::iterator compare_priority(ReqBuffer::iterator req1, ReqBuffer::iterator req2, bool req1_ready, bool req2_ready) override {
      bool ready1, ready2;
      int  priority1, priority2;

      bool req1_not_low_pri = true;
      int req1_flat_bank_id = req1->addr_vec[bank_idx] + req1->addr_vec[bankgroup_idx] * num_bank + req1->addr_vec[psuedo_ch_idx] * num_bankgroup*num_bank;  
      if(req1->command == m_dram->m_commands("ACT") && m_open_idle[req1_flat_bank_id]) {
        
        if(m_open_row_miss[req1_flat_bank_id] && m_pre_open_row[req1_flat_bank_id] == req1->addr_vec[row_idx]) {
          req1_not_low_pri = false;
        }
      }

      bool req2_not_low_pri = true;
      int req2_flat_bank_id = req2->addr_vec[bank_idx] + req2->addr_vec[bankgroup_idx] * num_bank + req2->addr_vec[psuedo_ch_idx] * num_bankgroup*num_bank;  
      if(req2->command == m_dram->m_commands("ACT") && m_open_idle[req2_flat_bank_id]) {
        
        if(m_open_row_miss[req2_flat_bank_id] && m_pre_open_row[req2_flat_bank_id] == req2->addr_vec[row_idx]) {
          req2_not_low_pri = false;
        }
      }

      ready1 = req1_not_low_pri && m_dram->check_ready(req1->command, req1->addr_vec);
      ready2 = req2_not_low_pri && m_dram->check_ready(req2->command, req2->addr_vec);
      
      priority1 = get_command_priority(req1->command);
      priority2 = get_command_priority(req2->command);

      ready1 = ready1 && req1_ready && (priority1 > 0);
      ready2 = ready2 && req2_ready && (priority2 > 0);
      
      // Both ready - compare by command priority first
      if (ready1 && ready2) {          
          // Higher priority wins
          if (priority1 > priority2) {
              return req1;
          } else if (priority2 > priority1) {
              return req2;
          }
          
          // Same priority - fall back to FCFS (First-Come-First-Serve)
          if (req1->arrive <= req2->arrive) {
              return req1;
          } else {
              return req2;
          }
      }
      
      // Only one ready
      if (ready1) {
          return req1;
      } else if (ready2) {
          return req2;
      }
      
      // Neither ready - fallback to FCFS by arrival time
      if (req1->arrive <= req2->arrive) {
          return req1;
      } else {
          return req2;
      }
    };

    bool check_db_buf_over_th(ReqBuffer::iterator req) {
      bool over_threshold = false;
      // Check It's not in m_activity_buffer and it's request is PRE_WR
      
      if((req->final_command == m_cmd.WR || req->final_command == m_cmd.WRA)) {
        // Check Write Fetch Buffer 
        if((m_dram->get_db_wr_fetch_per_pch(req->addr_vec)) >= 8) over_threshold = true;
      } else if((req->final_command == m_cmd.RD || req->final_command == m_cmd.RDA)) {
        if((m_dram->get_db_rd_fetch_per_pch(req->addr_vec)) >= 8) over_threshold = true;
      }
      return over_threshold;
    }

    bool check_pre_req(ReqBuffer::iterator req) {
      bool is_pre_enabled_req = false;
      if(req->type_id == Request::Type::Read) {
        if(req->final_command == m_cmd.RD || req->final_command == m_cmd.RDA) is_pre_enabled_req = true;
      } else {
        if(req->final_command == m_cmd.WR || req->final_command == m_cmd.WRA) is_pre_enabled_req = true;
      }
      return is_pre_enabled_req;
    }

    
    // Call by Active Buffer, Priority Buffer, Prefetch Buffer
    ReqBuffer::iterator get_best_request(ReqBuffer& buffer) override {
      if (buffer.size() == 0) {
        return buffer.end();
      }
      
      for (auto& req : buffer) {
        // Specifies a command (preq_cmd) for all requests in the buffer to be fulfilled 
        req.command = m_dram->get_preq_command(req.final_command, req.addr_vec);
      }
      
      // Pick up the oldest of commands that are ready
      auto candidate = buffer.begin();
      for (auto next = std::next(buffer.begin(), 1); next != buffer.end(); next++) {
        candidate = compare(candidate, next, true, true);
      }
      
      return candidate;
    }

    ReqBuffer::iterator get_best_request_with_priority(ReqBuffer& buffer, uint32_t priority_list_index) override  {
      if (buffer.size() == 0) {
          return buffer.end();
      }
      
      // Update priority mapping if provided
      // if (!cmd_priority_list.empty()) {
      //     set_command_priority(cmd_priority_list);
      // }
      if(priority_list_index > 6) {
        throw std::runtime_error("Invalid Priority List Index!");
      }
      m_prio_idx = priority_list_index;

      // Specify command for all requests in the buffer
      for (auto& req : buffer) {
          req.command = m_dram->get_preq_command(req.final_command, req.addr_vec);
      }
      
      // Pick up the oldest/highest priority command that is ready
      auto candidate = buffer.begin();
      for (auto next = std::next(buffer.begin(), 1); next != buffer.end(); next++) {
          candidate = compare_priority(candidate, next, true, true);
      }
      
      return candidate;
    }    

    
    // Get Best 
    //   - WRITE (PRE_WR) Reuqest
    //   - READ  (PRE_RD) Reuqest
    ReqBuffer::iterator get_best_pre_request(ReqBuffer& buffer) override {
      if (buffer.size() == 0) {
        return buffer.end();
      }
      
      for (auto& req : buffer) {
        // Specifies a command (preq_cmd) for all requests in the buffer to be fulfilled 
        req.command = m_dram->get_preq_pre_command(req.final_command, req.addr_vec);
      }
      
      // Pick up the oldest of commands that are ready
      auto candidate = buffer.begin();
      for (auto next = std::next(buffer.begin(), 1); next != buffer.end(); next++) {
        bool req1_ready = !check_db_buf_over_th(candidate) && check_pre_req(candidate);
        bool req2_ready = !check_db_buf_over_th(next) && check_pre_req(next);
        candidate = compare(candidate, next, req1_ready, req2_ready);
      }
      
      if(!check_db_buf_over_th(candidate) && check_pre_req(candidate)) return candidate;
      else                                                             return buffer.end();
    }  
    
    
    // Deprecated Source 
    bool check_post_rw(ReqBuffer::iterator req, bool write_mode) {
      return false;
    }    
    
    // Get Best Reuqest within masked pseudo channel
    ReqBuffer::iterator get_best_request_with_mask(ReqBuffer& buffer, std::vector<bool>& mask_array) override {
      return buffer.end();
    }   

    // Get Best Reuqest within masked pseudo channel for prefetched buffer
    ReqBuffer::iterator get_best_request_prefetch_with_mask(ReqBuffer& buffer, std::vector<bool>& mask_array,bool is_refreshing,bool is_wr_mode) override { 
      return buffer.end();
    }   

    // Get Best Reuqest during Refresh to issue PRE_WR
    ReqBuffer::iterator get_best_request_refresh_ch(ReqBuffer& buffer) override {
      return buffer.end();
    }    

    // Get Best Reuqest during Refresh to issue PRE_WR within masked pseudo channel
    ReqBuffer::iterator get_best_request_refresh_ch_with_mask(ReqBuffer& buffer, std::vector<bool>& mask_array) override {
      return buffer.end();
    }  
};

}       // namespace Ramulator
