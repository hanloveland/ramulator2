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
    std::vector<std::vector<uint64_t>> ndp_access_infos;
    int ndp_ctrl_row = 0;
    int ndp_ctrl_bk = 0;
    int ndp_ctrl_bg = 0;
    int row_addr_idx = 0;
    int bk_addr_idx = 0;
    int bg_addr_idx = 0;

  public:
    void init() override { 
      Logger_t m_logger;

      m_logger = Logging::create_logger("NDPDRAMSystem");
      m_logger->info("DRAM_System init()");
    
      // Create device (a top-level node wrapping all channel nodes)
      m_dram = create_child_ifce<IDRAM>();
      m_addr_mapper = create_child_ifce<IAddrMapper>();

      int num_channels = m_dram->get_level_size("channel");   
      int num_ranks = m_dram->get_level_size("rank");   
      int num_pseudochannel = m_dram->get_level_size("pseudochannel");   

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
      ndp_ctrl_bg  = (m_dram->get_level_size("bankgroup") - 1);
      ndp_ctrl_bk  = (m_dram->get_level_size("bank") - 1);
      row_addr_idx  = m_dram->m_levels("row");
      bg_addr_idx   = m_dram->m_levels("bankgroup");
      bk_addr_idx   = m_dram->m_levels("bank");

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
      std::cout<<"Get Request"<<std::endl;
      if(req.addr_vec[row_addr_idx] == ndp_ctrl_row && req.addr_vec[bg_addr_idx] == ndp_ctrl_bg) {          
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
    };

    float get_tCK() override {
      return m_dram->m_timing_vals("tCK_ps") / 1000.0f;
    }

    // const SpecDef& get_supported_requests() override {
    //   return m_dram->m_requests;
    // };
    bool send_ndp_ctrl(Request req) {
      // 
      std::cout<<"NDP Ctrl Get .. NDP Control Request"<<std::endl;
      if(req.addr_vec[bk_addr_idx] == ndp_ctrl_bk) {
        // NDP Control 
      } else {
        // NDP Access Info Buffer 
        if(req.type_id == Request::Type::Write) {
          if(req.m_payload.size() > 0) {
            std::vector<uint64_t> access_info;
            for(uint32_t i=0;i<req.m_payload.size();i++) {
              access_info.push_back(req.m_payload[i]);
              std::cout<<"["<<i<<"]"<<req.m_payload[i]<<std::endl;
            }
            ndp_access_infos.push_back(access_info);
          }
        }
      }
      // Request send to Host-Side NDP Ctrl (currently, only WR)
      // CH:x PCH:x BG:7 BK:3 ROW:FFFF COL:x
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

      return (is_ndp_finished && is_dram_ctrl_finished);
    }
};
  
}   // namespace 

