#include <vector>

#include "base/base.h"
#include "dram_controller/controller.h"
#include "dram_controller/refresh.h"

namespace Ramulator {

class DR5PCHAllBankRefresh : public IRefreshManager, public Implementation {
  RAMULATOR_REGISTER_IMPLEMENTATION(IRefreshManager, DR5PCHAllBankRefresh, "DR5CHAllBank", "DDR5-pCh All-Bank Refresh scheme.")
  private:
    Clk_t m_clk = 0;
    IDRAM* m_dram;
    IDRAMController* m_ctrl;

    int m_dram_org_levels = -1;
    int m_num_pseudochannels = -1;
    int m_num_ranks = -1;

    int m_nrefi = -1;
    int m_nrfc  = -1;
    int m_prefetch = -1;
    int m_ref_req_id = -1;
    Clk_t m_next_refresh_cycle = -1;
    Clk_t m_next_enable_rd_prefetch_cycle = -1;

  public:
    void init() override { 
      m_ctrl = cast_parent<IDRAMController>();
    };

    void setup(IFrontEnd* frontend, IMemorySystem* memory_system) override {
      m_dram = m_ctrl->m_dram;

      m_dram_org_levels = m_dram->m_levels.size();
      m_num_pseudochannels = m_dram->get_level_size("pseudochannel");
      m_num_ranks = m_dram->get_level_size("rank");

      m_nrefi = m_dram->m_timing_vals("nREFI");
      m_nrfc  = m_dram->m_timing_vals("nRFC1");
      m_ref_req_id = m_dram->m_requests("all-bank-refresh");

      // Prefetch Cycle (Max 8*16*1.5)
      m_prefetch = 8*24;

      m_next_refresh_cycle = m_nrefi;
      m_next_enable_rd_prefetch_cycle = (m_next_refresh_cycle - m_prefetch);
    };

    void tick() {
      m_clk++;
      
      // 
      if (m_clk == m_next_enable_rd_prefetch_cycle) {
        for (int p = 0; p < m_num_pseudochannels; p++) {
          m_dram->set_high_pri_prefetch(m_ctrl->m_channel_id,p);
        }
      }
      if (m_clk == m_next_refresh_cycle) {
        // std::cout<<"["<<m_clk<<"] Enque All-Back Refresh"<<std::endl;
        m_next_refresh_cycle += m_nrefi;
        m_next_enable_rd_prefetch_cycle = (m_next_refresh_cycle - m_prefetch);
        for (int p = 0; p < m_num_pseudochannels; p++) {
          m_dram->reset_high_pri_prefetch(m_ctrl->m_channel_id,p);
          for (int r = 0; r < m_num_ranks; r++) {
            std::vector<int> addr_vec(m_dram_org_levels, -1);
            addr_vec[0] = m_ctrl->m_channel_id;
            addr_vec[1] = p; // pseudo channel 
            addr_vec[2] = 0; // narrow-I/O
            addr_vec[3] = 0; // Wide-I/O 
            addr_vec[4] = r; // Rank
            Request req(addr_vec, m_ref_req_id);

            bool is_success = m_ctrl->priority_send(req);
            if (!is_success) {
              throw std::runtime_error("Failed to send refresh!");
            }
          }
        }
      }
    };

};

}       // namespace Ramulator
