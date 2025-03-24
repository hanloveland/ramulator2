#include <vector>

#include "base/base.h"
#include "dram_controller/controller.h"
#include "dram_controller/scheduler.h"

namespace Ramulator {

class FRFCFS : public IScheduler, public Implementation {
  RAMULATOR_REGISTER_IMPLEMENTATION(IScheduler, FRFCFS, "FRFCFS", "FRFCFS DRAM Scheduler.")
  private:
    IDRAM* m_dram;

  public:
    void init() override { };

    void setup(IFrontEnd* frontend, IMemorySystem* memory_system) override {
      m_dram = cast_parent<IDRAMController>()->m_dram;
    };

    ReqBuffer::iterator compare(ReqBuffer::iterator req1, ReqBuffer::iterator req2) override {
      bool ready1 = m_dram->check_ready(req1->command, req1->addr_vec);
      bool ready2 = m_dram->check_ready(req2->command, req2->addr_vec);

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
        if(!check_wr_fetch_over_th(next)) candidate = compare(candidate, next);
      }

      if(!check_wr_fetch_over_th(candidate)) return candidate;
      else                                   return buffer.end();
    }

    // Get Best Reuqest within masked pseudo channel
    ReqBuffer::iterator get_best_request_with_mask(ReqBuffer& buffer, std::vector<bool>& mask_array) override {
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
        if(!check_wr_fetch_over_th(next) && mask_array[next->addr_vec[1]]) candidate = compare(candidate, next);
      }

      if(!check_wr_fetch_over_th(candidate) && mask_array[candidate->addr_vec[1]]) return candidate;
      else                                                                         return buffer.end();
    }   

    // Get Best Reuqest during Refresh to issue PRE_WR
    ReqBuffer::iterator get_best_request_refresh_ch(ReqBuffer& buffer) override {
      if (buffer.size() == 0) {
        return buffer.end();
      }

      for (auto& req : buffer) {
        // Specifies a command (preq_cmd) for all requests in the buffer to be fulfilled 
        req.command = m_dram->get_preq_command_refresh_ch(req.final_command, req.addr_vec);
      }

      // Pick up the oldest of commands that are ready
      auto candidate = buffer.begin();
      for (auto next = std::next(buffer.begin(), 1); next != buffer.end(); next++) {
        if(!check_wr_fetch_over_th(next) && m_dram->check_ch_refrsehing(next->addr_vec)) candidate = compare(candidate, next);
      }

      if(!check_wr_fetch_over_th(candidate)) return candidate;
      else                                   return buffer.end();
    }    

    // Get Best Reuqest during Refresh to issue PRE_WR within masked pseudo channel
    ReqBuffer::iterator get_best_request_refresh_ch_with_mask(ReqBuffer& buffer, std::vector<bool>& mask_array) override {
      if (buffer.size() == 0) {
        return buffer.end();
      }

      for (auto& req : buffer) {
        // Specifies a command (preq_cmd) for all requests in the buffer to be fulfilled 
        req.command = m_dram->get_preq_command_refresh_ch(req.final_command, req.addr_vec);
      }

      // Pick up the oldest of commands that are ready
      auto candidate = buffer.begin();
      for (auto next = std::next(buffer.begin(), 1); next != buffer.end(); next++) {
        if(!check_wr_fetch_over_th(next) && m_dram->check_ch_refrsehing(next->addr_vec) && mask_array[next->addr_vec[1]]) candidate = compare(candidate, next);
      }
      
      if(!check_wr_fetch_over_th(candidate) && mask_array[candidate->addr_vec[1]]) return candidate;
      else                                                                         return buffer.end();
    }  

};

}       // namespace Ramulator
