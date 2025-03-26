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

      m_use_rd_prefetch = m_dram->get_use_rd_prefetch();
      m_use_wr_prefetch = m_dram->get_use_wr_prefetch();
    };

    ReqBuffer::iterator compare(ReqBuffer::iterator req1, ReqBuffer::iterator req2, bool req1_ready, bool req2_ready) override {
      bool ready1;
      bool ready2;      
      if(m_dram->get_use_pch()) {
        // try first ready1 check 
        ready1 = m_dram->check_ready(req1->command, req1->addr_vec);
        // try second ready check 
        bool is_req1_enable_rd_prefetch = m_dram->get_db_fetch_mode(req1->addr_vec[0],req1->addr_vec[1]) == 0/*MODE_PRE_RD:0*/;
        if(m_use_rd_prefetch && !ready1 && m_dram->get_enable_rd_prefetch(req1->addr_vec[0],req1->addr_vec[1]) && 
           is_req1_enable_rd_prefetch && ((m_dram->get_db_fetch_per_pch(req1->addr_vec)) < 16) &&
          ((req1->command == m_dram->m_commands("RD")) || (req1->command == m_dram->m_commands("RDA")))) {
            int new_cmd = -1;
            if(req1->command == m_dram->m_commands("RD")) new_cmd = m_dram->m_commands("PRE_RD");
            else                                          new_cmd = m_dram->m_commands("PRE_RDA"); 

            ready1 = m_dram->check_ready(new_cmd, req1->addr_vec);
            if(ready1) req1->command = new_cmd;
        }
        // try first ready2 check 
        ready2 = m_dram->check_ready(req2->command, req2->addr_vec);
        // try second ready check 
        bool is_req2_enable_rd_prefetch = m_dram->get_db_fetch_mode(req2->addr_vec[0],req2->addr_vec[1]) == 0/*MODE_PRE_RD:0*/;
        if(m_use_rd_prefetch && !ready2 && m_dram->get_enable_rd_prefetch(req2->addr_vec[0],req2->addr_vec[1]) && 
           is_req2_enable_rd_prefetch && ((m_dram->get_db_fetch_per_pch(req2->addr_vec)) < 16) &&
           ((req2->command == m_dram->m_commands("RD")) || (req2->command == m_dram->m_commands("RDA")))) {
          int new_cmd = -1;
          if(req2->command == m_dram->m_commands("RD")) new_cmd = m_dram->m_commands("PRE_RD");
          else                                          new_cmd = m_dram->m_commands("PRE_RDA"); 

          ready2 = m_dram->check_ready(new_cmd, req2->addr_vec);
          if(ready2) req2->command = new_cmd;
        }
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
      bool is_enable = false;
      bool is_refresh_pch = m_dram->check_ch_refrsehing(req->addr_vec);
      if(req->final_command == m_dram->m_commands("POST_RD")) {
        if(m_dram->get_db_fetch_mode(req->addr_vec[0],req->addr_vec[1]) == 1/*MODE_POST_RD:1*/) {
          if(is_refresh_pch) is_enable = true;
          else if(!write_mode) is_enable = true;
          if(is_enable) {
            // bool check = m_dram->check_ready(req->command, req->addr_vec);
            // std::cout<<"Enable POST_RD | check : "<<check<<" | ";
            // m_dram->print_req(*req);
          }
        }
      }
      else if(req->final_command == m_dram->m_commands("POST_WR")) {
        if(m_dram->get_db_fetch_mode(req->addr_vec[0],req->addr_vec[1]) == 3/*MODE_POST_WR:3*/) {
          if(!is_refresh_pch && write_mode) is_enable = true;
          if(is_enable) {
            // bool check = m_dram->check_ready(req->command, req->addr_vec);
            // std::cout<<"Enable POST_WR | check : "<<check<<" | ";
            // m_dram->print_req(*req);
          }            
        }
      }
      return is_enable;
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
        bool req1_ready = !check_wr_fetch_over_th(candidate);
        bool req2_ready = !check_wr_fetch_over_th(next); 
        candidate = compare(candidate, next, req1_ready, req2_ready);
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
        bool req1_ready = !check_wr_fetch_over_th(candidate) && mask_array[candidate->addr_vec[1]]; 
        bool req2_ready = !check_wr_fetch_over_th(next) && mask_array[next->addr_vec[1]];
        candidate = compare(candidate, next, req1_ready, req2_ready);
      }

      if(!check_wr_fetch_over_th(candidate) && mask_array[candidate->addr_vec[1]]) return candidate;
      else                                                                         return buffer.end();
    }   

    // Get Best Reuqest within masked pseudo channel for prefetched buffer
    ReqBuffer::iterator get_best_request_prefetch_with_mask(ReqBuffer& buffer, std::vector<bool>& mask_array,bool is_refreshing,bool is_wr_mode) override {
      if (buffer.size() == 0) {
        return buffer.end();
      }

      for (auto& req : buffer) {
        // Specifies a command (preq_cmd) for all requests in the buffer to be fulfilled 
        req.command = m_dram->get_preq_command(req.final_command, req.addr_vec);
      }

      // m_dram->get_db_fetch_mode(req1->addr_vec[0],req1->addr_vec[1]) == 1
      // m_dram->get_db_fetch_mode(req1->addr_vec[0],req1->addr_vec[1]) == 3/*MODE_POST_WR:3*/
      // Pick up the oldest of commands that are ready      
      auto candidate = buffer.begin();
      for (auto next = std::next(buffer.begin(), 1); next != buffer.end(); next++) {
        bool req_ready1 = mask_array[candidate->addr_vec[1]] && check_post_rw(candidate,is_wr_mode);
        bool req_ready2 = mask_array[next->addr_vec[1]] && check_post_rw(next,is_wr_mode);
        candidate = compare(candidate, next, req_ready1, req_ready2);
      }

      if(mask_array[candidate->addr_vec[1]] && check_post_rw(candidate,is_wr_mode)) return candidate;
      else                                                                          return buffer.end();
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
        bool req1_ready = !check_wr_fetch_over_th(candidate) && m_dram->check_ch_refrsehing(candidate->addr_vec);
        bool req2_ready = !check_wr_fetch_over_th(next) && m_dram->check_ch_refrsehing(next->addr_vec);
        candidate = compare(candidate, next, req1_ready, req2_ready);
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
        bool req1_ready = !check_wr_fetch_over_th(candidate) && m_dram->check_ch_refrsehing(candidate->addr_vec) && mask_array[candidate->addr_vec[1]];
        bool req2_ready = !check_wr_fetch_over_th(next) && m_dram->check_ch_refrsehing(next->addr_vec) && mask_array[next->addr_vec[1]];
        candidate = compare(candidate, next, req1_ready, req2_ready);
      }

      if(!check_wr_fetch_over_th(candidate) && mask_array[candidate->addr_vec[1]]) return candidate;
      else                                                                         return buffer.end();
    }  

};

}       // namespace Ramulator
