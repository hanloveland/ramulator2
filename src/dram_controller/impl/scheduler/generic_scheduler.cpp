#include <vector>

#include "base/base.h"
#include "dram_controller/controller.h"
#include "dram_controller/scheduler.h"

namespace Ramulator {

class FRFCFS : public IScheduler, public Implementation {
  RAMULATOR_REGISTER_IMPLEMENTATION(IScheduler, FRFCFS, "FRFCFS", "FRFCFS DRAM Scheduler.")
  private:
    IDRAM* m_dram;
    bool m_use_wr_prefetch;
    bool m_use_rd_prefetch;
    std::vector<int> m_open_row;
    std::vector<int> m_pre_open_row;
    std::vector<bool> m_open_row_miss;
    std::vector<bool> m_open_idle;
    
    int num_ranks = -1;    
    int num_bankgroups = -1;
    int num_banks = -1;
    int rank_idx = 0;
    int bankgroup_idx = 0;
    int bank_idx = 0;
    int row_idx = 0;

  public:
    void init() override { };

    void setup(IFrontEnd* frontend, IMemorySystem* memory_system) override {
      m_dram = cast_parent<IDRAMController>()->m_dram;

      num_ranks = m_dram->get_level_size("rank");  
      num_bankgroups = m_dram->get_level_size("bankgroup");  
      num_banks = m_dram->get_level_size("bank");  
      rank_idx = m_dram->m_levels("rank");
      bankgroup_idx = m_dram->m_levels("bankgroup");
      bank_idx = m_dram->m_levels("bank"); 
      row_idx = m_dram->m_levels("row"); 

      // Record Row address per bank for Adaptive Open-Page Policy
      m_open_row.resize(num_ranks*num_bankgroups*num_banks, -1);
      m_pre_open_row.resize(num_ranks*num_bankgroups*num_banks, -1);
      m_open_row_miss.resize(num_ranks*num_bankgroups*num_banks, false);      
      m_open_idle.resize(num_ranks*num_bankgroups*num_banks, true);      
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

    ReqBuffer::iterator compare(ReqBuffer::iterator req1, ReqBuffer::iterator req2, bool req1_ready, bool req2_ready) override {
      bool ready1;
      bool ready2;      

      // BK Status Check 
      bool req1_not_low_pri = true;
      int req1_flat_bank_id = req1->addr_vec[bank_idx] + req1->addr_vec[bankgroup_idx] * num_banks + req1->addr_vec[rank_idx] * num_bankgroups*num_banks;  
      if(req1->command == m_dram->m_commands("ACT") && m_open_idle[req1_flat_bank_id]) {
        
        if(m_open_row_miss[req1_flat_bank_id] && m_pre_open_row[req1_flat_bank_id] == req1->addr_vec[row_idx]) {
          req1_not_low_pri = false;
        }
      }

      bool req2_not_low_pri = true;
      int req2_flat_bank_id = req2->addr_vec[bank_idx] + req2->addr_vec[bankgroup_idx] * num_banks + req2->addr_vec[rank_idx] * num_bankgroups*num_banks;  
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

    bool check_wr_fetch_over_th(ReqBuffer::iterator req) {
      return false;
    }

    bool check_post_rw(ReqBuffer::iterator req, bool write_mode) {
      return true;
    }    

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
        bool req1_ready = true;
        bool req2_ready = true;
        candidate = compare(candidate, next, req1_ready, req2_ready);
      }

      return candidate;
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

    // Get Best (PRE_RD/PRE_WR) Reuqest
    ReqBuffer::iterator get_best_pre_request(ReqBuffer& buffer) override {
      return buffer.end();
    } 

    ReqBuffer::iterator get_best_request_with_priority(ReqBuffer& buffer, uint32_t priority_list_index) override  {
      return buffer.end();
    }    

    ReqBuffer::iterator compare_priority(ReqBuffer::iterator req1, ReqBuffer::iterator req2, bool req1_ready, bool req2_ready) override {
      return req1;      
    };    

};

}       // namespace Ramulator
