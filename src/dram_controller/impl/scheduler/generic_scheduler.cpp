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

  public:
    void init() override { };

    void setup(IFrontEnd* frontend, IMemorySystem* memory_system) override {
      m_dram = cast_parent<IDRAMController>()->m_dram;

    };

    ReqBuffer::iterator compare(ReqBuffer::iterator req1, ReqBuffer::iterator req2, bool req1_ready, bool req2_ready) override {
      bool ready1;
      bool ready2;      
      if(m_dram->get_use_pch()) {
        ready1 = false;
        ready2 = false;
      } else {
        ready1 = m_dram->check_ready(req1->command, req1->addr_vec);
        ready2 = m_dram->check_ready(req2->command, req2->addr_vec);
      }

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
      bool over_threshold = false;
      // Check It's not in m_activity_buffer and it's request is PRE_WR
      
      if(m_dram->get_use_pch() && !req->is_actived && (req->command == m_dram->m_commands("P_ACT")  || 
                                                       req->command == m_dram->m_commands("PRE_WR") ||
                                                       req->command == m_dram->m_commands("P_PRE"))) {
        if((m_dram->get_db_fetch_per_pch(req->addr_vec)) >= 16 ) over_threshold = true;
      }
      return over_threshold;
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

    ReqBuffer::iterator get_best_request_with_priority(ReqBuffer& buffer, const std::vector<int>& cmd_priority_list) override  {
      return buffer.end();
    }    

    ReqBuffer::iterator compare_priority(ReqBuffer::iterator req1, ReqBuffer::iterator req2, bool req1_ready, bool req2_ready) override {
      return req1;      
    };    

};

}       // namespace Ramulator
