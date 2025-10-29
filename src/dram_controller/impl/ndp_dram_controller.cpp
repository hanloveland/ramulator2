#include "dram_controller/controller.h"
#include "memory_system/memory_system.h"

// #define PRINT_DB_CNT

namespace Ramulator {

class NDPDRAMController final : public IDRAMController, public Implementation {
  RAMULATOR_REGISTER_IMPLEMENTATION(IDRAMController, NDPDRAMController, "ndpDRAMCtrl", "A NDP DRAM controller.");
  private:
    std::deque<Request> pending;           // A queue for read requests that are about to finish (callback after RL)
    // Hardcoding Pseudo-Channel is fixed to 4
    ReqBuffer m_active_buffer;             // Buffer for requests being served. This has the highest priority 
    ReqBuffer m_priority_buffer;           // Buffer for high-priority requests (e.g., maintenance like refresh).
    ReqBuffer m_read_buffer;               // Read request buffer
    ReqBuffer m_write_buffer;              // Write request buffer
    ReqBuffer m_prefetched_buffer;          // Prefetched buffer
    
    std::vector<ReqBuffer> m_read_buffers;        // Read Buffers Per Pseudo Channel
    std::vector<ReqBuffer> m_write_buffers;       // Write Buffers Per Pseudo Channel    
    std::vector<ReqBuffer> m_rd_prefetch_buffers; // Read Prefetch Buffers Per Pseudo Channel    
    std::vector<ReqBuffer> m_wr_prefetch_buffers; // Write Prefetch Buffers Per Pseudo Channel    

    int m_bank_addr_idx = -1;

    float m_wr_low_watermark;
    float m_wr_high_watermark;
    bool  m_is_write_mode = false;
    std::vector<bool> m_is_write_mode_per_pch;
    std::vector<bool> prefetch_mode_before_ref_per_ch;

    // Estimation .. Each Status: 0: Read, 1: Write, 2: Prefetch, 3: Refresh
    std::vector<int>   current_sch_mode_per_pch;
    std::vector<int>   sch_mode_per_pch;
    std::vector<int>   read_mode_cycle_per_pch;
    std::vector<int>   write_mode_cycle_per_pch;
    std::vector<int>   prefetch_mode_cycle_per_pch;
    std::vector<int>   refresh_mode_cycle_per_pch;
    std::vector<int>   busy_read_mode_cycle_per_pch;
    std::vector<int>   busy_write_mode_cycle_per_pch;
    std::vector<int>   busy_prefetch_mode_cycle_per_pch;
    std::vector<int>   busy_refresh_mode_cycle_per_pch;

    std::vector<int>   cmd_cycle_per_pch;

    size_t s_row_hits = 0;
    size_t s_row_misses = 0;
    size_t s_row_conflicts = 0;
    size_t s_read_row_hits = 0;
    size_t s_read_row_misses = 0;
    size_t s_read_row_conflicts = 0;
    size_t s_write_row_hits = 0;
    size_t s_write_row_misses = 0;
    size_t s_write_row_conflicts = 0;

    size_t m_num_cores = 0;
    std::vector<size_t> s_read_row_hits_per_core;
    std::vector<size_t> s_read_row_misses_per_core;
    std::vector<size_t> s_read_row_conflicts_per_core;

    size_t s_num_read_reqs = 0;
    size_t s_num_write_reqs = 0;
    size_t s_num_other_reqs = 0;
    size_t s_queue_len = 0;
    size_t s_read_queue_len = 0;
    size_t s_write_queue_len = 0;
    size_t s_priority_queue_len = 0;
    size_t s_read_prefetch_queue_len = 0;
    size_t s_write_prefetch_queue_len = 0;

    float s_queue_len_avg = 0;
    float s_read_queue_len_avg = 0;
    float s_write_queue_len_avg = 0;
    float s_priority_queue_len_avg = 0;
    float s_read_prefetch_queue_len_avg = 0;
    float s_write_prefetch_queue_len_avg = 0;

    size_t s_read_latency = 0;
    float s_avg_read_latency = 0;

    float s_bandwidth = 0;
    float s_dq_bandwidth = 0;
    float s_max_bandwidth = 0;
    size_t s_num_issue_reads = 0;
    size_t s_num_issue_writes = 0;
    float s_effective_bandwidth = 0; 
    float s_max_effective_bandwidth = 0; 

    std::vector<size_t> s_num_trans_per_pch;
    std::vector<size_t> s_num_refresh_cc_per_pch;
    std::vector<size_t> s_num_busy_refresh_cc_per_pch;
    std::vector<size_t> s_num_max_prefetch_per_pch;
    std::vector<size_t> s_num_write_mode_per_pch;
    std::vector<size_t> s_narrow_io_busy_clk_per_pch;
    std::vector<size_t> s_wide_io_busy_clk_per_pch;
    size_t s_num_rw_switch = 0;

    bool use_pseudo_ch = false;
    int num_pseudochannel = -1;
    int psuedo_ch_idx = 0;
    std::vector<int> db_prefetch_cnt_per_pch;
    std::vector<int> db_prefetch_rd_cnt_per_pch;
    std::vector<int> db_prefetch_wr_cnt_per_pch;

    uint32_t s_num_act      = 0;
    uint32_t s_num_rd       = 0;
    uint32_t s_num_wr       = 0;
    uint32_t s_num_pre      = 0;
    
    uint32_t s_num_p_act    = 0;
    uint32_t s_num_pre_wr   = 0;
    uint32_t s_num_post_wr  = 0;
    uint32_t s_num_pre_rd   = 0;
    uint32_t s_num_post_rd  = 0;    
    uint32_t s_num_p_pre    = 0;    

    uint32_t s_num_ndp_dram_rd = 0; 
    uint32_t s_num_ndp_dram_wr = 0; 
    uint32_t s_num_ndp_db_rd   = 0; 
    uint32_t s_num_ndp_db_wr   = 0; 

    float s_cmd_io_util = 0.0;
    uint32_t cmd_io_cc = 0; 

    uint32_t pre_clk = 0;
    std::vector<int> io_busy_clk_per_pch;

    std::vector<bool> is_empty_priority_per_pch;

    // Enable Write/Read Prefetcher (DRAM->DB)/(MC->DB)
    bool m_use_prefetch;

    // tracking ndp_related rquest
    int num_ndp_rd_req;
    int num_ndp_wr_req;
    std::vector<int> num_ndp_wr_req_per_pch;
    std::vector<int> num_ndp_rd_req_per_pch;

    // NDP CONF REG RD Response from DRAM
    std::vector<std::vector<int>> ndp_config_reg_resp_per_pch;

    // Round-Robin Vector
    std::vector<int> rr_pch_idx;
  public:
    void init() override {
      m_wr_low_watermark =  param<float>("wr_low_watermark").desc("Threshold for switching back to read mode.").default_val(0.2f);
      m_wr_high_watermark = param<float>("wr_high_watermark").desc("Threshold for switching to write mode.").default_val(0.8f);

      m_scheduler = create_child_ifce<IScheduler>();
      m_refresh = create_child_ifce<IRefreshManager>();    
      m_rowpolicy = create_child_ifce<IRowPolicy>();    

      if (m_config["plugins"]) {
        YAML::Node plugin_configs = m_config["plugins"];
        for (YAML::iterator it = plugin_configs.begin(); it != plugin_configs.end(); ++it) {
          m_plugins.push_back(create_child_ifce<IControllerPlugin>(*it));
        }
      }      
    };

    void setup(IFrontEnd* frontend, IMemorySystem* memory_system) override {
      m_dram = memory_system->get_ifce<IDRAM>();
      m_bank_addr_idx = m_dram->m_levels("bank");
      m_priority_buffer.max_size = 512*3 + 32;
      m_active_buffer.max_size = 32*4;
      m_prefetched_buffer.max_size = 16*4;
      m_write_buffer.max_size = 64;
      m_read_buffer.max_size = 64;
      // std::cout<<m_active_buffer.max_size<<std::endl;
      // exit(1);

      if(m_dram->get_level_size("pseudochannel") == -1) use_pseudo_ch = false;
      else                                              use_pseudo_ch = true;

      if(use_pseudo_ch) psuedo_ch_idx = m_dram->m_levels("pseudochannel");

      m_use_prefetch = m_dram->get_use_prefetch();

      m_num_cores = frontend->get_num_cores();

      s_read_row_hits_per_core.resize(m_num_cores, 0);
      s_read_row_misses_per_core.resize(m_num_cores, 0);
      s_read_row_conflicts_per_core.resize(m_num_cores, 0);

      register_stat(s_row_hits).name("row_hits_{}", m_channel_id);
      register_stat(s_row_misses).name("row_misses_{}", m_channel_id);
      register_stat(s_row_conflicts).name("row_conflicts_{}", m_channel_id);
      register_stat(s_read_row_hits).name("read_row_hits_{}", m_channel_id);
      register_stat(s_read_row_misses).name("read_row_misses_{}", m_channel_id);
      register_stat(s_read_row_conflicts).name("read_row_conflicts_{}", m_channel_id);
      register_stat(s_write_row_hits).name("write_row_hits_{}", m_channel_id);
      register_stat(s_write_row_misses).name("write_row_misses_{}", m_channel_id);
      register_stat(s_write_row_conflicts).name("write_row_conflicts_{}", m_channel_id);

      for (size_t core_id = 0; core_id < m_num_cores; core_id++) {
        register_stat(s_read_row_hits_per_core[core_id]).name("read_row_hits_core_{}", core_id);
        register_stat(s_read_row_misses_per_core[core_id]).name("read_row_misses_core_{}", core_id);
        register_stat(s_read_row_conflicts_per_core[core_id]).name("read_row_conflicts_core_{}", core_id);
      }

      register_stat(s_num_read_reqs).name("num_read_reqs_{}", m_channel_id);
      register_stat(s_num_write_reqs).name("num_write_reqs_{}", m_channel_id);
      register_stat(s_num_other_reqs).name("num_other_reqs_{}", m_channel_id);
      register_stat(s_queue_len).name("queue_len_{}", m_channel_id);
      register_stat(s_read_queue_len).name("read_queue_len_{}", m_channel_id);
      register_stat(s_write_queue_len).name("write_queue_len_{}", m_channel_id);
      register_stat(s_priority_queue_len).name("priority_queue_len_{}", m_channel_id);
      register_stat(s_queue_len_avg).name("queue_len_avg_{}", m_channel_id);
      register_stat(s_read_queue_len_avg).name("read_queue_len_avg_{}", m_channel_id);
      register_stat(s_write_queue_len_avg).name("write_queue_len_avg_{}", m_channel_id);
      register_stat(s_priority_queue_len_avg).name("priority_queue_len_avg_{}", m_channel_id);

      register_stat(s_read_latency).name("read_latency_{}", m_channel_id);
      register_stat(s_avg_read_latency).name("avg_read_latency_{}", m_channel_id);

      register_stat(s_bandwidth).name("rw_bandwidth_{}", m_channel_id);
      register_stat(s_num_issue_reads).name("num_issue_reads_{}", m_channel_id);
      register_stat(s_num_issue_writes).name("num_issue_writes_{}", m_channel_id);
      register_stat(s_dq_bandwidth).name("dq_bandwidth_{}", m_channel_id);
      register_stat(s_max_bandwidth).name("max_bandwidth_{}", m_channel_id);
      register_stat(s_effective_bandwidth).name("s_effective_bandwidth_{}", m_channel_id);
      register_stat(s_max_effective_bandwidth).name("s_max_effective_bandwidth_{}", m_channel_id);

      num_pseudochannel = m_dram->get_level_size("pseudochannel");  

      // Initialize Per Pseudo-Channel Request Buffer and set Max Request Capacity     
      m_read_buffers.resize(num_pseudochannel,ReqBuffer());
      m_write_buffers.resize(num_pseudochannel,ReqBuffer());
      m_rd_prefetch_buffers.resize(num_pseudochannel,ReqBuffer());
      m_wr_prefetch_buffers.resize(num_pseudochannel,ReqBuffer());
      m_is_write_mode_per_pch.resize(num_pseudochannel,false);
      num_ndp_wr_req_per_pch.resize(num_pseudochannel,0);
      num_ndp_rd_req_per_pch.resize(num_pseudochannel,0);
      prefetch_mode_before_ref_per_ch.resize(num_pseudochannel,false);

      for(int i=0; i<num_pseudochannel; i++) {
        m_read_buffers[i].max_size = 32;
        m_write_buffers[i].max_size = 32;
        m_rd_prefetch_buffers[i].max_size = 16;
        m_wr_prefetch_buffers[i].max_size = 16;
        rr_pch_idx.push_back(i);
      }

      if(num_pseudochannel != -1) {
        s_num_trans_per_pch.resize(num_pseudochannel, 0);
        s_num_refresh_cc_per_pch.resize(num_pseudochannel, 0);
        s_num_busy_refresh_cc_per_pch.resize(num_pseudochannel, 0);
        db_prefetch_cnt_per_pch.resize(num_pseudochannel, 0);
        db_prefetch_rd_cnt_per_pch.resize(num_pseudochannel, 0);
        db_prefetch_wr_cnt_per_pch.resize(num_pseudochannel, 0);
        s_num_max_prefetch_per_pch.resize(num_pseudochannel, 0);
        s_num_write_mode_per_pch.resize(num_pseudochannel, 0);
        s_narrow_io_busy_clk_per_pch.resize(num_pseudochannel, 0);
        s_wide_io_busy_clk_per_pch.resize(num_pseudochannel, 0);

        io_busy_clk_per_pch.resize(num_pseudochannel, 0);
        is_empty_priority_per_pch.resize(num_pseudochannel, 0);
        for (size_t pch_id = 0; pch_id < num_pseudochannel; pch_id++) {
          register_stat(s_num_trans_per_pch[pch_id]).name("s_num_trans_per_pch_{}_{}", m_channel_id,pch_id);        
          register_stat(s_num_refresh_cc_per_pch[pch_id]).name("s_num_refresh_cc_per_pch_{}_{}", m_channel_id,pch_id);      
          register_stat(s_num_busy_refresh_cc_per_pch[pch_id]).name("s_num_busy_refresh_cc_per_pch_{}_{}", m_channel_id,pch_id);             
          register_stat(s_num_max_prefetch_per_pch[pch_id]).name("s_num_max_prefetch_per_pch_{}_{}", m_channel_id,pch_id);   
          register_stat(s_num_write_mode_per_pch[pch_id]).name("s_num_write_mode_per_pch_{}_{}", m_channel_id,pch_id);    
          register_stat(s_narrow_io_busy_clk_per_pch[pch_id]).name("s_narrow_io_busy_clk_per_pch_{}_{}", m_channel_id,pch_id);    
          register_stat(s_wide_io_busy_clk_per_pch[pch_id]).name("s_wide_io_busy_clk_per_pch_{}_{}", m_channel_id,pch_id);                     
        }
      }

      register_stat(s_num_rw_switch).name("s_num_rw_switch_{}", m_channel_id);
      register_stat(s_num_pre_wr).name("s_num_pre_wr_{}", m_channel_id);
      register_stat(s_num_post_wr).name("s_num_post_wr_{}", m_channel_id);
      register_stat(s_num_pre_rd).name("s_num_pre_rd_{}", m_channel_id);
      register_stat(s_num_post_rd).name("s_num_post_rd_{}", m_channel_id);
      register_stat(s_num_act).name("s_num_act_{}", m_channel_id);
      register_stat(s_num_pre).name("s_num_pre_{}", m_channel_id);
      register_stat(s_num_p_act).name("s_num_p_act_{}", m_channel_id);
      register_stat(s_num_p_pre).name("s_num_p_pre_{}", m_channel_id);
      register_stat(s_num_rd).name("s_num_rd_{}", m_channel_id);
      register_stat(s_num_wr).name("s_num_wr_{}", m_channel_id);      
      register_stat(s_num_ndp_dram_rd).name("s_num_ndp_dram_rd_{}", m_channel_id);      
      register_stat(s_num_ndp_dram_wr).name("s_num_ndp_dram_wr_{}", m_channel_id);      
      register_stat(s_num_ndp_db_rd).name("s_num_ndp_db_rd_{}", m_channel_id);      
      register_stat(s_num_ndp_db_wr).name("s_num_ndp_db_wr_{}", m_channel_id);                                  

      register_stat(s_read_prefetch_queue_len).name("s_read_prefetch_queue_len_{}", m_channel_id);    
      register_stat(s_write_prefetch_queue_len).name("s_write_prefetch_queue_len_{}", m_channel_id);    
      register_stat(s_read_prefetch_queue_len_avg).name("s_read_prefetch_queue_len_avg_{}", m_channel_id);    
      register_stat(s_write_prefetch_queue_len_avg).name("s_write_prefetch_queue_len_avg_{}", m_channel_id);    
      register_stat(s_cmd_io_util).name("s_cmd_io_util_{}", m_channel_id);                         

      // Inintialization NDP-related variables
      num_ndp_rd_req = 0;
      num_ndp_wr_req = 0;      

      current_sch_mode_per_pch.resize(num_pseudochannel,0);
      sch_mode_per_pch.resize(num_pseudochannel,0);
      read_mode_cycle_per_pch.resize(num_pseudochannel,0);
      write_mode_cycle_per_pch.resize(num_pseudochannel,0);
      prefetch_mode_cycle_per_pch.resize(num_pseudochannel,0);
      refresh_mode_cycle_per_pch.resize(num_pseudochannel,0);
      busy_read_mode_cycle_per_pch.resize(num_pseudochannel,0);
      busy_write_mode_cycle_per_pch.resize(num_pseudochannel,0);
      busy_prefetch_mode_cycle_per_pch.resize(num_pseudochannel,0);
      busy_refresh_mode_cycle_per_pch.resize(num_pseudochannel,0);

      ndp_config_reg_resp_per_pch.resize(num_pseudochannel,std::vector<int>(0,0));     

      cmd_cycle_per_pch.resize(num_pseudochannel,0);
    };

    bool send(Request& req) override {
      bool is_success = false;

      if(req.is_ndp_req) {
        if(m_dram->is_ndp_access(req.addr_vec)) {
          // NDP Access (NDP_CONF, ISNT/DAT MMEM)
          if(req.type_id == Request::Type::Read) req.final_command = m_dram->m_request_translations(m_dram->m_requests("ndp-db-read"));
          else                                   req.final_command = m_dram->m_request_translations(m_dram->m_requests("ndp-db-write")); 
        } else {
          // NDP Execution
          if(req.type_id == Request::Type::Read) req.final_command = m_dram->m_request_translations(m_dram->m_requests("ndp-dram-read"));
          else                                   req.final_command = m_dram->m_request_translations(m_dram->m_requests("ndp-dram-write")); 
        }                   
      } else {
        req.final_command = m_dram->m_request_translations(req.type_id);
      }

      // Forward existing write requests to incoming read requests
      if (req.type_id == Request::Type::Read) {
        // lamda function 
        auto compare_addr = [req](const Request& wreq) {
          return wreq.addr == req.addr;
        };
        // if existing write request which is same address with read, send to pending request queue
        ReqBuffer& wr_buffer = m_write_buffers[req.addr_vec[psuedo_ch_idx]];        
        if (std::find_if(wr_buffer.begin(), wr_buffer.end(), compare_addr) != wr_buffer.end()) {
          // The request will depart at the next cycle
          req.depart = m_clk + 1;
          pending.push_back(req);
          is_success = true;
        }
      }

      // Else, enqueue them to corresponding buffer based on request type id
      if(!is_success) {
        req.arrive = m_clk;
        if (req.type_id == Request::Type::Read) {
          // RD, RDA, NDP_DB_RD, NDP_DRAM_RD
          is_success = m_read_buffers[req.addr_vec[psuedo_ch_idx]].enqueue(req);
        } else if (req.type_id == Request::Type::Write) {
          // WR, WRA, NDP_DB_WR, NDP_DRAM_WR
          is_success = m_write_buffers[req.addr_vec[psuedo_ch_idx]].enqueue(req);       
        } else {
          throw std::runtime_error("Invalid request type!");
        }
      }

      if(is_success) {
        switch (req.type_id) {
          case Request::Type::Read: {
            s_num_read_reqs++;
            break;
          }
          case Request::Type::Write: {
            s_num_write_reqs++;
            break;
          }
          default: {
            s_num_other_reqs++;
            break;
          }
        }      

        if(req.is_ndp_req) {
          if(req.type_id == Request::Type::Read) {
            num_ndp_rd_req++;
            num_ndp_rd_req_per_pch[req.addr_vec[psuedo_ch_idx]]++;
          }
          else {
            num_ndp_wr_req++;
            num_ndp_wr_req_per_pch[req.addr_vec[psuedo_ch_idx]]++;
          }          
        }
      }
      // if(is_success) {
        // std::cout<<"[NDP_DRAM_CTRL] Get Request ";
        // m_dram->print_req(req);
      // }
      return is_success;
    };

    bool priority_send(Request& req) override {
      req.final_command = m_dram->m_request_translations(req.type_id);

      bool is_success = false;
      is_success = m_priority_buffer.enqueue(req);
      return is_success;
    }

    void tick() override {
      m_clk++;

      // Update statistics
      s_queue_len += m_priority_buffer.size() + pending.size();
      s_read_queue_len += pending.size();
      for(int i=0;i<num_pseudochannel;i++) {
        s_read_queue_len += m_read_buffers[i].size();
        s_write_queue_len += m_write_buffers[i].size();
        s_read_prefetch_queue_len += m_rd_prefetch_buffers[i].size();
        s_write_prefetch_queue_len += m_wr_prefetch_buffers[i].size();
        if(m_is_write_mode_per_pch[i]) s_num_write_mode_per_pch[i]++;
      }
      s_priority_queue_len += m_priority_buffer.size();

      for(int pch_idx=0;pch_idx<num_pseudochannel;pch_idx++) {
        bool prefetch_mode_before_ref = m_use_prefetch ? m_dram->get_pri_prefetch(m_channel_id,pch_idx) : false;
        if((m_read_buffers[pch_idx].size() == 0  || m_rd_prefetch_buffers[pch_idx].size() == m_rd_prefetch_buffers[pch_idx].max_size))
          prefetch_mode_before_ref = false;
        
        prefetch_mode_before_ref_per_ch[pch_idx] = prefetch_mode_before_ref;
      }

      for(int pch_idx=0;pch_idx<num_pseudochannel;pch_idx++) {
        if(current_sch_mode_per_pch[pch_idx] == 0)        read_mode_cycle_per_pch[pch_idx]++;
        else if(current_sch_mode_per_pch[pch_idx] == 1)   write_mode_cycle_per_pch[pch_idx]++;
        else if(current_sch_mode_per_pch[pch_idx] == 2)   prefetch_mode_cycle_per_pch[pch_idx]++;
        else if(current_sch_mode_per_pch[pch_idx] == 3)   refresh_mode_cycle_per_pch[pch_idx]++;
      }
              
      // 1. Serve completed reads
      serve_completed_reads();

      // Check tREF and Send REF by priority_send()
      m_refresh->tick();

      // 2. Try to find a request to serve.
      ReqBuffer::iterator req_it;
      ReqBuffer* buffer = nullptr;
      bool request_found = schedule_request(req_it, buffer);

      // 2.1 Take row policy action
      m_rowpolicy->update(request_found, req_it);

      // 3. Update all plugins
      for (auto plugin : m_plugins) {
        plugin->update(request_found, req_it);
      }

      // 4. Finally, issue the commands to serve the request
      if (request_found) {
        /*
          ======================= Update PRE_WR/POST_WR Counter and Update Request Information =====================
        */
        if(use_pseudo_ch) {
          
          int req_pch_idx = req_it->addr_vec[psuedo_ch_idx];
          if(prefetch_mode_before_ref_per_ch[req_pch_idx] && (req_it->command == m_dram->m_commands("RD") || req_it->command == m_dram->m_commands("RDA")) && 
             db_prefetch_rd_cnt_per_pch[req_pch_idx] < m_rd_prefetch_buffers[req_pch_idx].max_size) {
            // Convert RD or RDA to PRE_RD when the pseudo channel is high priority mode of prefetch
            if(req_it->command == m_dram->m_commands("RD")) req_it->command = m_dram->m_commands("PRE_RD");
            else                                            req_it->command = m_dram->m_commands("PRE_RDA");\
            req_it->is_db_cmd = true;
          }

          // Update PRE_WR from Normal WR/WRA
          if((req_it->command == m_dram->m_commands("P_ACT") || req_it->command == m_dram->m_commands("PRE_WR")) &&
             (req_it->final_command == m_dram->m_commands("WR") || req_it->final_command == m_dram->m_commands("WRA"))) {
              // PRE_WR from WRITE_BUFFER --> Update PRE_WR Counter
              s_num_pre_wr++;
              db_prefetch_cnt_per_pch[req_it->addr_vec[psuedo_ch_idx]]++;
              db_prefetch_wr_cnt_per_pch[req_it->addr_vec[psuedo_ch_idx]]++;       
              // Change Final Command from WR/WRA to PRE_WR 
              req_it->final_command = m_dram->m_commands("PRE_WR");      
              #ifdef PRINT_DB_CNT
              std::cout<<"PRE_WR ["<<req_it->addr_vec[0]<<"]["<<req_it->addr_vec[1]<<"] |";
              std::cout<<" db cnt "<<db_prefetch_cnt_per_pch[req_it->addr_vec[psuedo_ch_idx]];
              std::cout<<" db rd cnt "<<db_prefetch_rd_cnt_per_pch[req_it->addr_vec[psuedo_ch_idx]];
              std::cout<<" db wr cnt "<<db_prefetch_wr_cnt_per_pch[req_it->addr_vec[psuedo_ch_idx]]<<std::endl;                       
              #endif                   
          }

          // Update POST_WR 
          if(req_it->command == m_dram->m_commands("POST_WR")) {
            // Issued DDR commands related with PRE_WR
            s_num_post_wr++;
            db_prefetch_cnt_per_pch[req_it->addr_vec[psuedo_ch_idx]]--;
            db_prefetch_wr_cnt_per_pch[req_it->addr_vec[psuedo_ch_idx]]--;
            #ifdef PRINT_DB_CNT
            std::cout<<"POST_WR ["<<req_it->addr_vec[0]<<"]["<<req_it->addr_vec[1]<<"] |";
            std::cout<<" db cnt "<<db_prefetch_cnt_per_pch[req_it->addr_vec[psuedo_ch_idx]];
            std::cout<<" db rd cnt "<<db_prefetch_rd_cnt_per_pch[req_it->addr_vec[psuedo_ch_idx]];
            std::cout<<" db wr cnt "<<db_prefetch_wr_cnt_per_pch[req_it->addr_vec[psuedo_ch_idx]]<<std::endl;                
            #endif
          }

          // Update PRE_RD
          if((req_it->command == m_dram->m_commands("ACT")  || req_it->command == m_dram->m_commands("RD") ||
              req_it->command  == m_dram->m_commands("RDA") || req_it->command  == m_dram->m_commands("PRE_RD") ||
              req_it->command  == m_dram->m_commands("PRE_RDA")) && (req_it->final_command == m_dram->m_commands("RD") || 
              req_it->final_command == m_dram->m_commands("RDA")) && req_it->is_db_cmd) {                
            // PRE_RD from READ_BUFFER --> update PRD_RD Counter
            s_num_pre_rd++;
            db_prefetch_cnt_per_pch[req_it->addr_vec[psuedo_ch_idx]]++;
            db_prefetch_rd_cnt_per_pch[req_it->addr_vec[psuedo_ch_idx]]++;
            // Change Final Command from RD/RDA to PRE_RD/PRE_RDA 
            if(req_it->final_command == m_dram->m_commands("RD")) req_it->final_command = m_dram->m_commands("PRE_RD");
            else                                                  req_it->final_command = m_dram->m_commands("PRE_RDA");
            #ifdef PRINT_DB_CNT
            std::cout<<"PRE_RD ["<<req_it->addr_vec[0]<<"]["<<req_it->addr_vec[1]<<"] |";
            std::cout<<" db cnt "<<db_prefetch_cnt_per_pch[req_it->addr_vec[psuedo_ch_idx]];
            std::cout<<" db rd cnt "<<db_prefetch_rd_cnt_per_pch[req_it->addr_vec[psuedo_ch_idx]];
            std::cout<<" db wr cnt "<<db_prefetch_wr_cnt_per_pch[req_it->addr_vec[psuedo_ch_idx]]<<std::endl;            
            #endif
          }
          
          // Update POST_RD
          if(req_it->command  == m_dram->m_commands("POST_RD")) {
            s_num_post_rd++;
            db_prefetch_cnt_per_pch[req_it->addr_vec[psuedo_ch_idx]]--;
            db_prefetch_rd_cnt_per_pch[req_it->addr_vec[psuedo_ch_idx]]--;
            #ifdef PRINT_DB_CNT
            std::cout<<"POST_RD ["<<req_it->addr_vec[0]<<"]["<<req_it->addr_vec[1]<<"] |";
            std::cout<<" db cnt "<<db_prefetch_cnt_per_pch[req_it->addr_vec[psuedo_ch_idx]];
            std::cout<<" db rd cnt "<<db_prefetch_rd_cnt_per_pch[req_it->addr_vec[psuedo_ch_idx]];
            std::cout<<" db wr cnt "<<db_prefetch_wr_cnt_per_pch[req_it->addr_vec[psuedo_ch_idx]]<<std::endl;
            #endif
          }            
            

          // Update DRAM Prefetch Counter Value
          m_dram->set_db_fetch_per_pch(req_it->addr_vec,db_prefetch_cnt_per_pch[req_it->addr_vec[psuedo_ch_idx]],
                                                        db_prefetch_rd_cnt_per_pch[req_it->addr_vec[psuedo_ch_idx]],
                                                        db_prefetch_wr_cnt_per_pch[req_it->addr_vec[psuedo_ch_idx]]);

          // Update Max Prefetch Counter 
          if(db_prefetch_cnt_per_pch[req_it->addr_vec[psuedo_ch_idx]] > s_num_max_prefetch_per_pch[req_it->addr_vec[psuedo_ch_idx]])
            s_num_max_prefetch_per_pch[req_it->addr_vec[psuedo_ch_idx]] = db_prefetch_cnt_per_pch[req_it->addr_vec[psuedo_ch_idx]];       
            
              
          if(false) {
            if(req_it->command == m_dram->m_commands("PRE_WR"))  std::cout<<"["<<m_clk<<"] ISSUE PRE_WR CH["<<req_it->addr_vec[0]<<"]PCH["<<req_it->addr_vec[1]<<"] / "<<s_num_pre_wr  << "/" <<s_num_post_wr
            <<" db cnt : "<<db_prefetch_cnt_per_pch[req_it->addr_vec[psuedo_ch_idx]]<<" / "<<m_dram->get_db_fetch_per_pch(req_it->addr_vec)<<std::endl;
            if(req_it->command == m_dram->m_commands("POST_WR")) std::cout<<"["<<m_clk<<"] ISSUE POST_WR CH["<<req_it->addr_vec[0]<<"]PCH["<<req_it->addr_vec[1]<<"] / "<<s_num_pre_wr  << "/" <<s_num_post_wr
            <<" db cnt : "<<db_prefetch_cnt_per_pch[req_it->addr_vec[psuedo_ch_idx]]<<" / "<<m_dram->get_db_fetch_per_pch(req_it->addr_vec)<<std::endl;
            if(req_it->command == m_dram->m_commands("PRE_RD")) std::cout<<"["<<m_clk<<"] ISSUE PRE_RD CH["<<req_it->addr_vec[0]<<"]PCH["<<req_it->addr_vec[1]<<"] / "<<s_num_pre_rd  << "/" <<s_num_post_rd
            <<" db cnt : "<<db_prefetch_cnt_per_pch[req_it->addr_vec[psuedo_ch_idx]]<<" / "<<m_dram->get_db_fetch_per_pch(req_it->addr_vec)<<std::endl;              
            if(req_it->command == m_dram->m_commands("PRE_RDA")) std::cout<<"["<<m_clk<<"] ISSUE PRE_RDA CH["<<req_it->addr_vec[0]<<"]PCH["<<req_it->addr_vec[1]<<"] / "<<s_num_pre_rd  << "/" <<s_num_post_rd
            <<" db cnt : "<<db_prefetch_cnt_per_pch[req_it->addr_vec[psuedo_ch_idx]]<<" / "<<m_dram->get_db_fetch_per_pch(req_it->addr_vec)<<std::endl;                            
            if(req_it->command == m_dram->m_commands("POST_RD")) std::cout<<"["<<m_clk<<"] ISSUE POST_RD CH["<<req_it->addr_vec[0]<<"]PCH["<<req_it->addr_vec[1]<<"] / "<<s_num_pre_rd  << "/" <<s_num_post_rd
            <<" db cnt : "<<db_prefetch_cnt_per_pch[req_it->addr_vec[psuedo_ch_idx]]<<" / "<<m_dram->get_db_fetch_per_pch(req_it->addr_vec)<<std::endl;                                          
          }
          // Exception 
          if(db_prefetch_cnt_per_pch[req_it->addr_vec[psuedo_ch_idx]] > 32 || db_prefetch_cnt_per_pch[req_it->addr_vec[psuedo_ch_idx]] < 0) {
            std::string write_mode;
            if(m_is_write_mode_per_pch[req_it->addr_vec[psuedo_ch_idx]]) write_mode = "[WRITE_MODE]";
            else                                                         write_mode = "[READ_MODE]";
            std::cout<<"[ndpDRAMCtrl]";
            std::cout<<write_mode<<" - ";            
            m_dram->print_req(*req_it);
            std::cout<<"[ndpDRAMCtrl] - Over Prefresh Threshold!"<<std::endl;
            for(int i=0;i<num_pseudochannel;i++) {
              std::cout<<"["<<m_clk<<"] CH["<<m_channel_id<<"]PCH["<<i<<"] "<<std::endl;
              std::cout<<" - Total: "<<db_prefetch_cnt_per_pch[i]<<std::endl;
              std::cout<<" - Write: "<<db_prefetch_wr_cnt_per_pch[i]<<std::endl;
              std::cout<<" - Read: "<<db_prefetch_rd_cnt_per_pch[i]<<std::endl;       
            }
            exit(1);
          }
        }
        
        /*
          ======================= Update Stats =====================
        */

        if(req_it->command == m_dram->m_commands("P_ACT")     ||
           req_it->command == m_dram->m_commands("PRE")       ||
           req_it->command == m_dram->m_commands("PREA")      ||
           req_it->command == m_dram->m_commands("P_PRE")     ||
           req_it->command == m_dram->m_commands("REFab")     ||
           req_it->command == m_dram->m_commands("REFsb")     ||
           req_it->command == m_dram->m_commands("REFab_end") ||
           req_it->command == m_dram->m_commands("REFsb_end")) {
          // Single Cycle Command
          cmd_io_cc+=1;
          cmd_cycle_per_pch[req_it->addr_vec[psuedo_ch_idx]]+=1;
        } else {
          // Two Cycle Command
          cmd_io_cc+=2;
          cmd_cycle_per_pch[req_it->addr_vec[psuedo_ch_idx]]+=2;
        }

        if(req_it->command == m_dram->m_commands("RD") || req_it->command == m_dram->m_commands("RDA") || 
        req_it->command == m_dram->m_commands("WR") || req_it->command == m_dram->m_commands("WRA")) {
          s_narrow_io_busy_clk_per_pch[req_it->addr_vec[1]]+=(32);
          s_wide_io_busy_clk_per_pch[req_it->addr_vec[1]]+=(8);
          if(current_sch_mode_per_pch[req_it->addr_vec[1]] == 0)      busy_read_mode_cycle_per_pch[req_it->addr_vec[1]]+=32;
          else if(current_sch_mode_per_pch[req_it->addr_vec[1]] == 1) busy_write_mode_cycle_per_pch[req_it->addr_vec[1]]+=32;
          else if(current_sch_mode_per_pch[req_it->addr_vec[1]] == 2) busy_prefetch_mode_cycle_per_pch[req_it->addr_vec[1]]+=32;
          else if(current_sch_mode_per_pch[req_it->addr_vec[1]] == 3) busy_refresh_mode_cycle_per_pch[req_it->addr_vec[1]]+=32;                              
        }

        if(req_it->command == m_dram->m_commands("POST_RD") || req_it->command == m_dram->m_commands("PRE_WR") || 
           req_it->command == m_dram->m_commands("NDP_DB_RD") || req_it->command == m_dram->m_commands("NDP_DB_WR")) {
          s_narrow_io_busy_clk_per_pch[req_it->addr_vec[1]]+=(32);
          if(current_sch_mode_per_pch[req_it->addr_vec[1]] == 0)      busy_read_mode_cycle_per_pch[req_it->addr_vec[1]]+=32;
          else if(current_sch_mode_per_pch[req_it->addr_vec[1]] == 1) busy_write_mode_cycle_per_pch[req_it->addr_vec[1]]+=32;
          else if(current_sch_mode_per_pch[req_it->addr_vec[1]] == 2) busy_prefetch_mode_cycle_per_pch[req_it->addr_vec[1]]+=32;
          else if(current_sch_mode_per_pch[req_it->addr_vec[1]] == 3) busy_refresh_mode_cycle_per_pch[req_it->addr_vec[1]]+=32;                    
        }

        if(req_it->command == m_dram->m_commands("PRE_RD") || req_it->command == m_dram->m_commands("PRE_RDA")          || 
           req_it->command == m_dram->m_commands("POST_WR") || req_it->command == m_dram->m_commands("POST_WRA")        ||
           req_it->command == m_dram->m_commands("NDP_DRAM_RD") || req_it->command == m_dram->m_commands("NDP_DRAM_WR") ||
           req_it->command == m_dram->m_commands("NDP_DRAM_RDA") || req_it->command == m_dram->m_commands("NDP_DRAM_WRA")) {
            s_wide_io_busy_clk_per_pch[req_it->addr_vec[1]]+=(8);
        }

        if(req_it->command == m_dram->m_commands("ACT"))                                                                  s_num_act++;
        if(req_it->command == m_dram->m_commands("PRE") || req_it->command == m_dram->m_commands("PREA"))                 s_num_pre++;
        if(req_it->command == m_dram->m_commands("P_ACT"))                                                                s_num_p_act++;
        if(req_it->command == m_dram->m_commands("P_PRE"))                                                                s_num_p_pre++;
        if(req_it->command == m_dram->m_commands("RD") || req_it->command == m_dram->m_commands("RDA"))                   s_num_rd++;
        if(req_it->command == m_dram->m_commands("WR") || req_it->command == m_dram->m_commands("WRA"))                   s_num_wr++;
        if(req_it->command == m_dram->m_commands("NDP_DRAM_RD") || req_it->command == m_dram->m_commands("NDP_DRAM_RDA")) s_num_ndp_dram_rd++;
        if(req_it->command == m_dram->m_commands("NDP_DRAM_WR") || req_it->command == m_dram->m_commands("NDP_DRAM_WRA")) s_num_ndp_dram_wr++;
        if(req_it->command == m_dram->m_commands("NDP_DB_RD"))                                                            s_num_ndp_db_rd++;
        if(req_it->command == m_dram->m_commands("NDP_DB_WR"))                                                            s_num_ndp_db_wr++;              

        if(req_it->command == m_dram->m_commands("RD") || req_it->command == m_dram->m_commands("RDA") || req_it->command == m_dram->m_commands("POST_RD")) s_num_issue_reads++;
        if(req_it->command == m_dram->m_commands("WR") || req_it->command == m_dram->m_commands("WRA") || req_it->command == m_dram->m_commands("PRE_WR"))  s_num_issue_writes++;
        if((req_it->command == m_dram->m_commands("RD") || req_it->command == m_dram->m_commands("RDA") || req_it->command == m_dram->m_commands("POST_RD")) || 
            (req_it->command == m_dram->m_commands("WR") || req_it->command == m_dram->m_commands("WRA") || req_it->command == m_dram->m_commands("PRE_WR"))) {
            s_num_trans_per_pch[req_it->addr_vec[psuedo_ch_idx]]++;
        }

        if(req_it->command == m_dram->m_commands("REFab")) {
          if(use_pseudo_ch) {        
            int nRFC_latency = m_dram->m_timing_vals("nRFC1");
            int pch_addr = req_it->addr_vec[psuedo_ch_idx];
            s_num_refresh_cc_per_pch[pch_addr]+=nRFC_latency;
          }    
        }

        /*
          ======================= Issue Scheduled Request to DRAM with NDP  =====================
        */

        // If we find a real request to serve
        if (req_it->is_stat_updated == false) {
          update_request_stats(req_it);
        }
        m_dram->issue_command(req_it->command, req_it->addr_vec);

        // Issue NDP Command to DRAM
        if ((req_it->is_ndp_req && req_it->final_command == req_it->command))
        {
          if(req_it->m_payload.size()!=0) {
            m_dram->issue_ndp_command(req_it->command, req_it->addr_vec, req_it->ndp_id, req_it->m_payload);
          } else {
            m_dram->issue_ndp_command(req_it->command, req_it->addr_vec, req_it->ndp_id);
          }

        }
        
        if(false/* && req_it->addr_vec[0] == 0 && req_it->addr_vec[1] == 1*/) {

          std::cout<<"["<<(m_clk)<<"] ";
          std::cout<<" Ramined Req (active_buf/read_buf/write_buf/rd_pre_buf/wr_pre_buf/pending) "<<(m_active_buffer.size())<<" / "
                                                                                         <<m_read_buffers[req_it->addr_vec[psuedo_ch_idx]].size()<<" / "
                                                                                         <<m_write_buffers[req_it->addr_vec[psuedo_ch_idx]].size()<<" / "
                                                                                         <<m_rd_prefetch_buffers[req_it->addr_vec[psuedo_ch_idx]].size()<<" / "
                                                                                         <<m_wr_prefetch_buffers[req_it->addr_vec[psuedo_ch_idx]].size()<<" / "
                                                                                         <<pending.size()<<" - ";
          m_dram->print_req(*req_it);
          pre_clk = m_clk;
        }
        
        if(false 
           /*
           true && req_it->addr_vec[0]==0 && req_it->addr_vec[1]==0 && (
           req_it->command == m_dram->m_commands("RD") ||
           req_it->command == m_dram->m_commands("RDA") ||
           req_it->command == m_dram->m_commands("WR") ||
           req_it->command == m_dram->m_commands("WRA") ||
           req_it->command == m_dram->m_commands("POST_RD") ||
           req_it->command == m_dram->m_commands("POST_WR") ||
           req_it->command == m_dram->m_commands("PRE_RD") ||
           req_it->command == m_dram->m_commands("PRE_WR"))*/ 
          ) {
            std::cout<<"["<<m_clk<<"]["<<(m_clk-pre_clk)<<"] ";
            pre_clk = m_clk;          
            std::cout<<"[NDP_DRAM_CTRL] read ["<<s_num_issue_reads<<"] write ["<<s_num_issue_writes<<"] ";
            std::cout<<" / act_buf : "<<m_active_buffer.size();
            std::cout<<" / pri_buf : "<<m_priority_buffer.size();
            std::cout<<" / rd_buf : "<<m_read_buffers[0].size()<<"/"<<m_read_buffers[1].size()<<"/"<<m_read_buffers[2].size()<<"/"<<m_read_buffers[3].size();
            std::cout<<" / wr_buf : "<<m_write_buffers[0].size()<<"/"<<m_write_buffers[1].size()<<"/"<<m_write_buffers[2].size()<<"/"<<m_write_buffers[3].size();
            std::cout<<" / rd_pre_buf : "<<m_rd_prefetch_buffers[0].size()<<"/"<<m_rd_prefetch_buffers[1].size()<<"/"<<m_rd_prefetch_buffers[2].size()<<"/"<<m_rd_prefetch_buffers[3].size();
            std::cout<<" / wr_pre_buf : "<<m_wr_prefetch_buffers[0].size()<<"/"<<m_wr_prefetch_buffers[1].size()<<"/"<<m_wr_prefetch_buffers[2].size()<<"/"<<m_wr_prefetch_buffers[3].size()<<" - ";
            std::cout<<" / s_num_pre_rd "<<s_num_pre_rd<<"/";
            std::string write_mode;
            if(m_is_write_mode_per_pch[req_it->addr_vec[psuedo_ch_idx]]) write_mode = "[WRITE_MODE]";
            else                                                         write_mode = "[READ_MODE]";
            std::cout<<write_mode<<" ";
            m_dram->print_req(*req_it);
            // std::cout<<std::endl;
        }
        // #endif
        // If we are issuing the last command, set depart clock cycle and move the request to the pending queue
        if (req_it->command == req_it->final_command) {
          if(req_it->is_ndp_req) {
            if(req_it->type_id == Request::Type::Read) {
              num_ndp_rd_req--;
              num_ndp_rd_req_per_pch[req_it->addr_vec[psuedo_ch_idx]]--;
            }
            else {
              num_ndp_wr_req--;
              num_ndp_wr_req_per_pch[req_it->addr_vec[psuedo_ch_idx]]--;
            }                                       
            // std::cout<<"["<<m_clk<<"]"<<"[NDP-MC] num_ndp_rd_req ["<<num_ndp_rd_req<<"] num_ndp_wr_req ["<<num_ndp_wr_req<<"]"<<std::endl;
          }

          if(req_it->command == m_dram->m_commands("PRE_WR") || req_it->command == m_dram->m_commands("POST_RD")) {
            if(m_dram->check_ch_refrsehing(req_it->addr_vec)) {
              s_num_busy_refresh_cc_per_pch[req_it->addr_vec[psuedo_ch_idx]]+=(8*4);
            }
          }

          // Move issued Read Request to pending buffer
          if (req_it->type_id == Request::Type::Read) {
            if(!(req_it->command == m_dram->m_commands("PRE_RD") || req_it->command == m_dram->m_commands("PRE_RDA") ||   
                 req_it->command == m_dram->m_commands("NDP_DRAM_RD") || req_it->command == m_dram->m_commands("NDP_DRAM_RDA"))) {
              req_it->depart = m_clk + m_dram->m_read_latency;
              pending.push_back(*req_it);  
            }
          } else if (req_it->type_id == Request::Type::Write) {
            // TODO: Add code to update statistics
          }
          
          if(req_it->command == m_dram->m_commands("PRE_WR")) {
            // Generate POST_WR and Enqueue to Prefetched Buffer
            Request new_req = Request(req_it->addr_vec, Request::Type::Write);
            new_req.addr          = req_it->addr;
            new_req.arrive        = req_it->arrive;
            new_req.final_command = m_dram->m_commands("POST_WR");
            bool is_success = false;              

            buffer->remove(req_it);
            // is_success = m_prefetched_buffer.enqueue(new_req);
            is_success = m_wr_prefetch_buffers[new_req.addr_vec[psuedo_ch_idx]].enqueue(new_req);
            // std::cout<<"[NDP_DRAM_CTRL] Generate POST_WR and Enqueue to WR_PREFETCH_BUFFER"<<std::endl;
            if(!is_success) {
              throw std::runtime_error("Fail to enque to m_wr_prefetch_buffers");
            }                    
          } else if(req_it->command == m_dram->m_commands("PRE_RD") || req_it->command == m_dram->m_commands("PRE_RDA")) {
            // Generate POST_RD and Enqueue to Prefetched Buffer
            Request new_req = Request(req_it->addr_vec, Request::Type::Read);
            new_req.addr          = req_it->addr;
            new_req.arrive        = req_it->arrive;
            new_req.source_id     = req_it->source_id;
            new_req.callback      = req_it->callback;
            new_req.final_command = m_dram->m_commands("POST_RD");
            bool is_success = false;              

            buffer->remove(req_it);
            // is_success = m_prefetched_buffer.enqueue(new_req);
            is_success = m_rd_prefetch_buffers[new_req.addr_vec[psuedo_ch_idx]].enqueue(new_req);
            if(!is_success) {
              throw std::runtime_error("Fail to enque to m_rd_prefetch_buffers");
            }                    
          } else {
            buffer->remove(req_it);
          }
        } else {
          if (m_dram->m_command_meta(req_it->command).is_opening) {
            bool is_success = false;
            req_it->is_actived = true;
            is_success = m_active_buffer.enqueue(*req_it);
            if(!is_success) {
              throw std::runtime_error("Fail to enque to m_active_buffer");
            }
            buffer->remove(req_it);
          }
        }

      }

      // get NDP CONF REG Response
      for(int i=0;i<num_pseudochannel;i++) {
        int conf_reg_resp = m_dram->get_ndp_response(m_channel_id,i);
        if(conf_reg_resp != -1) {
          // if the ndp response is valid, then store 
          ndp_config_reg_resp_per_pch[i].push_back(conf_reg_resp);
        }
      }
      
    };


  private:
    /**
     * @brief    Helper function to check if a request is hitting an open row
     * @details
     * 
     */
    bool is_row_hit(ReqBuffer::iterator& req)
    {
        return m_dram->check_rowbuffer_hit(req->final_command, req->addr_vec);
    }
    /**
     * @brief    Helper function to check if a request is opening a row
     * @details
     * 
    */
    bool is_row_open(ReqBuffer::iterator& req)
    {
        return m_dram->check_node_open(req->final_command, req->addr_vec);
    }

    /**
     * @brief    
     * @details
     * 
     */
    void update_request_stats(ReqBuffer::iterator& req)
    {
      req->is_stat_updated = true;

      if (req->type_id == Request::Type::Read) 
      {
        if (is_row_hit(req)) {
          s_read_row_hits++;
          s_row_hits++;
          if (req->source_id != -1)
            s_read_row_hits_per_core[req->source_id]++;
        } else if (is_row_open(req)) {
          s_read_row_conflicts++;
          s_row_conflicts++;
          if (req->source_id != -1)
            s_read_row_conflicts_per_core[req->source_id]++;
        } else {
          s_read_row_misses++;
          s_row_misses++;
          if (req->source_id != -1)
            s_read_row_misses_per_core[req->source_id]++;
        } 
      } 
      else if (req->type_id == Request::Type::Write) 
      {
        if (is_row_hit(req)) {
          s_write_row_hits++;
          s_row_hits++;
        } else if (is_row_open(req)) {
          s_write_row_conflicts++;
          s_row_conflicts++;
        } else {
          s_write_row_misses++;
          s_row_misses++;
        }
      }
    }

    /**
     * @brief    Helper function to serve the completed read requests
     * @details
     * This function is called at the beginning of the tick() function.
     * It checks the pending queue to see if the top request has received data from DRAM.
     * If so, it finishes this request by calling its callback and poping it from the pending queue.
     */
    void serve_completed_reads() {
      if (pending.size()) {
        // Check the first pending request
        auto& req = pending[0];
        if (req.depart <= m_clk) {
          // Request received data from dram
          if (req.depart - req.arrive > 1) {
            // Check if this requests accesses the DRAM or is being forwarded.
            // TODO add the stats back
            s_read_latency += req.depart - req.arrive;
            // std::cout<<" RD latency ["<<(req.depart - req.arrive)<<"] ";
            // m_dram->print_req(req);
          }

          if (req.callback) {
            // If the request comes from outside (e.g., processor), call its callback
            req.callback(req);
          }
          // Finally, remove this request from the pending queue
          pending.pop_front();
        }
      };
    };


    /**
     * @brief    Checks if we need to switch to write mode
     * 
     */
    void set_write_mode() {
      if (!m_is_write_mode) {
        if ((m_write_buffer.size() > m_wr_high_watermark * m_write_buffer.max_size) || m_read_buffer.size() == 0 || num_ndp_wr_req != 0) {
          m_is_write_mode = true;
          s_num_rw_switch++;
        }
      } else {
        if ((m_write_buffer.size() < m_wr_low_watermark * m_write_buffer.max_size) && m_read_buffer.size() != 0) {
          m_is_write_mode = false;
          s_num_rw_switch++;
        }
      }
    };

    void set_write_mode_per_pch(int pch_idx) {
      if (!m_is_write_mode_per_pch[pch_idx]) {
        if ((m_write_buffers[pch_idx].size() > m_wr_high_watermark * m_write_buffers[pch_idx].max_size) || 
            (m_read_buffers[pch_idx].size() == 0 && m_rd_prefetch_buffers[pch_idx].size() == 0) 
            || num_ndp_wr_req_per_pch[pch_idx] != 0) {
          m_is_write_mode_per_pch[pch_idx] = true;
          s_num_rw_switch++;
        }
      } else {
        if ((num_ndp_wr_req_per_pch[pch_idx] == 0) && 
            ((m_write_buffers[pch_idx].size() < m_wr_low_watermark * m_write_buffers[pch_idx].max_size) || 
              m_wr_prefetch_buffers[pch_idx].size() == m_wr_prefetch_buffers[pch_idx].max_size) &&             
            (m_rd_prefetch_buffers[pch_idx].size() != 0 || m_wr_prefetch_buffers[pch_idx].size() != 0 || m_read_buffers[pch_idx].size() != 0)) {
          m_is_write_mode_per_pch[pch_idx] = false;
          s_num_rw_switch++;
        }
      }
    };    

    /**
     * @brief    Helper function to find a request to schedule from the buffers.
     * 
     */
    bool schedule_request(ReqBuffer::iterator& req_it, ReqBuffer*& req_buffer) {

      bool request_found = false;

      // Update Priority Queue Empty Flag Array 
      if(use_pseudo_ch) {
        m_dram->reset_need_be_open_per_bank(m_channel_id);
        for(int pch_idx=0;pch_idx<m_dram->get_level_size("pseudochannel");pch_idx++) {
          is_empty_priority_per_pch[pch_idx] = true;
          for(auto req : m_priority_buffer) {
            if(req.addr_vec[1] == pch_idx) is_empty_priority_per_pch[pch_idx] = false;
          }
        }
        for(auto req : m_active_buffer) {
          m_dram->set_need_be_open_per_bank(req.addr_vec);
        }
      }
      
      // 2.1    First, check the act buffer to serve requests that are already activating (avoid useless ACTs)
      // what is active_buffer? (opened row requesst?)
      if (req_it= m_scheduler->get_best_request(m_active_buffer); req_it != m_active_buffer.end()) {
        if (m_dram->check_ready(req_it->command, req_it->addr_vec)) {
          request_found = true;
          req_buffer = &m_active_buffer;
        }
      }

      // 2.2    If no requests can be scheduled from the act buffer, check the rest of the buffers
      if (!request_found) {
        // 2.2.1    We first check the priority buffer to prioritize e.g., maintenance requests
        if (m_priority_buffer.size() != 0) {
          req_buffer = &m_priority_buffer;
          req_it = m_priority_buffer.begin();
          req_it->command = m_dram->get_preq_command(req_it->final_command, req_it->addr_vec);
          
          request_found = m_dram->check_ready(req_it->command, req_it->addr_vec);
          if(!use_pseudo_ch) {
            if (!request_found & m_priority_buffer.size() != 0) {
              return false;
            }
          }
        }

        /*
          WRITE MODE
            --> PRE_WR - PRE_RD 
          READ MODE
            --> POST_RD - POST_WR - Normal RD

          BEFORE-PREFRESH
           -->PRD_RD/RD
          REFRESHING
           -->POST_RD-->PRE_WR          
        */
        /* ==================================================================================================== */
        if(!request_found) {
          for(auto pch_idx : rr_pch_idx) {
            // If each pch has high priority request, skip search issuable requset 
            if(is_empty_priority_per_pch[pch_idx]) {
              set_write_mode_per_pch(pch_idx);
              if(!request_found && prefetch_mode_before_ref_per_ch[pch_idx]) {                
                /*=============== Prefetch Mode Before Refresh =================*/
                /*
                  1. PRE_RD (READ_BUFFER)
                  2. POST_RD (RD_PREFETCH_BUFFER)

                */            
                // PRE_RD (READ_BUFFER)
                current_sch_mode_per_pch[pch_idx] = 2;
                if(!request_found && m_use_prefetch) { 
                  if (req_it = m_scheduler->get_best_pre_request(m_read_buffers[pch_idx]); req_it != m_read_buffers[pch_idx].end()) {
                    request_found = m_dram->check_ready(req_it->command, req_it->addr_vec);
                    req_buffer = &m_read_buffers[pch_idx];   
                    if(request_found) {
                      if(req_it->final_command == m_dram->m_commands("RD") || req_it->final_command == m_dram->m_commands("RDA")) {
                        if(!(req_it->command == m_dram->m_commands("PRE") || req_it->command == m_dram->m_commands("P_PRE"))) {
                          req_it->is_db_cmd = true;  
                        }
                      }
                    }                                 
                  }      
                }
                // PRE_WR
                if(!request_found) {
                  if (req_it = m_scheduler->get_best_pre_request(m_write_buffers[pch_idx]); req_it != m_write_buffers[pch_idx].end()) {
                    request_found = m_dram->check_ready(req_it->command, req_it->addr_vec);
                    req_buffer = &m_write_buffers[pch_idx];     
                  }                              
                } 
                /*
                // POST_WR (WR_PREFETCH_BUFFER)
                if(!request_found) {
                  if (req_it = m_scheduler->get_best_request(m_wr_prefetch_buffers[pch_idx]); req_it != m_wr_prefetch_buffers[pch_idx].end()) {
                    request_found = m_dram->check_ready(req_it->command, req_it->addr_vec);
                    req_buffer = &m_wr_prefetch_buffers[pch_idx];                                                               
                  }      
                } 
                */                            
                // Search POST_RD
                if(!request_found && m_use_prefetch) {
                  if (req_it = m_scheduler->get_best_request(m_rd_prefetch_buffers[pch_idx]); req_it != m_rd_prefetch_buffers[pch_idx].end()) {
                    request_found = m_dram->check_ready(req_it->command, req_it->addr_vec);
                    req_buffer = &m_rd_prefetch_buffers[pch_idx];                                                                       
                  }      
                }                     
              } else if(!request_found && m_dram->check_pch_refrsehing_by_idx(m_channel_id,pch_idx) && m_use_prefetch) {
                /*=============== Scheduling during Refresh =================*/
                current_sch_mode_per_pch[pch_idx] = 3;
                // Fist Check POST_RD
                if(!request_found) {
                  if (req_it = m_scheduler->get_best_request(m_rd_prefetch_buffers[pch_idx]); req_it != m_rd_prefetch_buffers[pch_idx].end()) {
                    request_found = m_dram->check_ready(req_it->command, req_it->addr_vec);
                    req_buffer = &m_rd_prefetch_buffers[pch_idx];                                                                     
                  }
                }                     
                // Second PRE_WR in WRITE BUFFER
                if(!request_found) {
                  if (req_it = m_scheduler->get_best_pre_request(m_write_buffers[pch_idx]); req_it != m_write_buffers[pch_idx].end()) {
                    request_found = m_dram->check_ready(req_it->command, req_it->addr_vec);
                    req_buffer = &m_write_buffers[pch_idx];     
                  }                              
                } 
              } else if(m_is_write_mode_per_pch[pch_idx]) { // Write Mode 
                /*=============== WRITE MODE  =================*/
                /*
                  Simplify Write Mode 
                  X 1. PRE_WR (WRITE_BUFFER)
                  X 2. PRE_RD (READ_BUFFER)
                  X 3. POST_WR (WR_PREFETCH_BUFFER)
                  X 4. POST_RD (RD_PREFETCH_BUFFER)    
                  5. NORMAL WR (WRITE_BUFFER)             
                */
                // PRE_WR/NDP_DB_WRITE (WRITE_BUFFER)
                current_sch_mode_per_pch[pch_idx] = 1;
                /*
                if(!request_found && m_use_prefetch) {
                  if (req_it = m_scheduler->get_best_pre_request(m_write_buffers[pch_idx]); req_it != m_write_buffers[pch_idx].end()) {
                    request_found = m_dram->check_ready(req_it->command, req_it->addr_vec);
                    req_buffer = &m_write_buffers[pch_idx];     
                  }                              
                } 
                  */
                // PRE_RD (READ_BUFFER)
                /*
                if(!request_found && m_use_prefetch) { 
                  if (req_it = m_scheduler->get_best_pre_request(m_read_buffers[pch_idx]); req_it != m_read_buffers[pch_idx].end()) {
                    request_found = m_dram->check_ready(req_it->command, req_it->addr_vec);
                    req_buffer = &m_read_buffers[pch_idx];   
                    if(request_found) {
                      if(req_it->final_command == m_dram->m_commands("RD") || req_it->final_command == m_dram->m_commands("RDA")) {
                        if(!(req_it->command == m_dram->m_commands("PRE") || req_it->command == m_dram->m_commands("P_PRE"))) {
                          req_it->is_db_cmd = true;  
                        }
                      }
                    }                                 
                  }      
                }
                */
                /*
                // POST_WR (WR_PREFETCH_BUFFER)
                if(!request_found) {
                  if (req_it = m_scheduler->get_best_request(m_wr_prefetch_buffers[pch_idx]); req_it != m_wr_prefetch_buffers[pch_idx].end()) {
                    request_found = m_dram->check_ready(req_it->command, req_it->addr_vec);
                    req_buffer = &m_wr_prefetch_buffers[pch_idx];                                                               
                  }      
                }
                // POST_RD (RD_PREFETCH_BUFFER) 
                if(!request_found) {
                  if (req_it = m_scheduler->get_best_request(m_rd_prefetch_buffers[pch_idx]); req_it != m_rd_prefetch_buffers[pch_idx].end()) {
                    request_found = m_dram->check_ready(req_it->command, req_it->addr_vec);
                    req_buffer = &m_rd_prefetch_buffers[pch_idx];                                                                      
                  }      
                }        
                */      
                // NORMAL WR (WRITE_BUFFER)  
                if(!request_found/*&& !m_use_prefetch*/) {
                  if (req_it = m_scheduler->get_best_request(m_write_buffers[pch_idx]); req_it != m_write_buffers[pch_idx].end()) {
                    request_found = m_dram->check_ready(req_it->command, req_it->addr_vec);
                    req_buffer = &m_write_buffers[pch_idx];                                                                   
                  }      
                }                
              } else { // Read Mode
                // Normal RD - POST RD - POST WR
                // First Check Normal RD
                /*=============== Read MODE  =================*/
                /*
                  1. POST_RD (RD_PREFETCH_BUFFER)    
                  2. NORMAL RD (READ_BUFFER)             
                  3. POST_WR (WR_PREFETCH_BUFFER)
                */                
               current_sch_mode_per_pch[pch_idx] = 0;
                // Search POST_WR
                if(!request_found && m_use_prefetch) {
                  if (req_it = m_scheduler->get_best_request(m_wr_prefetch_buffers[pch_idx]); req_it != m_wr_prefetch_buffers[pch_idx].end()) {
                    request_found = m_dram->check_ready(req_it->command, req_it->addr_vec);
                    req_buffer = &m_wr_prefetch_buffers[pch_idx];                                                                
                  }      
                }
                // Search POST_RD
                if(!request_found && m_use_prefetch) {
                  if (req_it = m_scheduler->get_best_request(m_rd_prefetch_buffers[pch_idx]); req_it != m_rd_prefetch_buffers[pch_idx].end()) {
                    request_found = m_dram->check_ready(req_it->command, req_it->addr_vec);
                    req_buffer = &m_rd_prefetch_buffers[pch_idx];                                                                       
                  }      
                }               
                // Search Normal RD
                if(!request_found) { 
                  if (req_it = m_scheduler->get_best_request(m_read_buffers[pch_idx]); req_it != m_read_buffers[pch_idx].end()) {
                    request_found = m_dram->check_ready(req_it->command, req_it->addr_vec);
                    req_buffer = &m_read_buffers[pch_idx];                                                                                
                  }      
                }               
              }
            }
            if(request_found) break;
          }
        }                        
        
        /* ==================================================================================================== */      
      }

      // 2.3 If we find a request to schedule, we need to check if it will close an opened row in the active buffer.
      if (request_found) {
        if (m_dram->m_command_meta(req_it->command).is_closing) {
          auto& rowgroup = req_it->addr_vec;
          for (auto _it = m_active_buffer.begin(); _it != m_active_buffer.end(); _it++) {
            auto& _it_rowgroup = _it->addr_vec;
            bool is_matching = true;
            for (int i = 0; i < m_bank_addr_idx + 1 ; i++) {
              if (_it_rowgroup[i] != rowgroup[i] && _it_rowgroup[i] != -1 && rowgroup[i] != -1) {
                is_matching = false;
                break;
              }
            }
            if (is_matching) {
              request_found = false;
              if(use_pseudo_ch) {
                if(req_it->command == m_dram->m_commands("P_PRE")) req_it->is_db_cmd = false;
                  // m_dram->print_req(*req_it);
              }
              break;
            }
          }
        }
      }

      if(request_found) update_rr_pch_idx();
      return request_found;
    }

    void finalize() override {
      s_avg_read_latency = (float) s_read_latency / (float) s_num_read_reqs;

      s_queue_len_avg = (float) s_queue_len / (float) m_clk;
      s_read_queue_len_avg = (float) s_read_queue_len / (float) m_clk;
      s_write_queue_len_avg = (float) s_write_queue_len / (float) m_clk;
      s_priority_queue_len_avg = (float) s_priority_queue_len / (float) m_clk;
      s_read_prefetch_queue_len_avg = (float) s_read_prefetch_queue_len / (float) m_clk; 
      s_write_prefetch_queue_len_avg = (float) s_write_prefetch_queue_len / (float) m_clk; 
      // GB/s 
      // Request Bandwidth, DQ Bandwidth, Max DQ Bandwidth
      int tx_bytes = m_dram->m_internal_prefetch_size * m_dram->m_channel_width / 8;
      s_bandwidth = ((float)((s_num_read_reqs + s_num_write_reqs) * tx_bytes) / (float)(m_clk * m_dram->m_timing_vals("tCK_ps"))) * 1E12 / (1024*1024*1012);
      s_dq_bandwidth = ((float)((s_num_issue_reads + s_num_issue_writes) * tx_bytes) / (float)(m_clk * m_dram->m_timing_vals("tCK_ps"))) * 1E12 / (1024*1024*1012);
      // rate unit is MT/s 
      // 10^6 T/s --> /(2^30) GB/s
      s_max_bandwidth = (float)(m_dram->m_channel_width / 8) * (float)m_dram->m_timing_vals("rate") * 1E6 / (1024*1024*1012);      

      // I/O Utilization (%)
      s_cmd_io_util = 100 * (float) cmd_io_cc / (float) m_clk;    

      // Effective Bandwidth (DB <-> DRAMs)
      size_t total_transfer_data = 0;
      for(auto io_busy_clk : s_wide_io_busy_clk_per_pch) {
        // Each Pseudo Channel has one DB --> 2 DRAMs I/O with 4 ranks
        // Double Data Rate 
        total_transfer_data += 2 * m_dram->m_organization.dq * 4 * io_busy_clk * 2;
      }
      s_effective_bandwidth = ((float)(total_transfer_data/8)/(float)(m_clk * m_dram->m_timing_vals("tCK_ps"))) * 1E12 / (1024*1024*1012);

      // Max Effective Bandwidth (except refresh)
      float refresh_ratio = ((float)m_dram->m_timing_vals("nRFC1")/(float)m_dram->m_timing_vals("nREFI"));
      s_max_effective_bandwidth = s_max_bandwidth * (float)num_pseudochannel * (1.0 - refresh_ratio);      
      
      /*
      for(int pch_idx=0;pch_idx<num_pseudochannel;pch_idx++) {
        std::cout<<"Read Mode :"<<read_mode_cycle_per_pch[pch_idx]<<" ("<<(100*read_mode_cycle_per_pch[pch_idx]/m_clk)<<" %)"<<std::endl;
        std::cout<<"Write Mode :"<<write_mode_cycle_per_pch[pch_idx]<<" ("<<(100*write_mode_cycle_per_pch[pch_idx]/m_clk)<<" %)"<<std::endl;
        std::cout<<"Prefetch Mode :"<<prefetch_mode_cycle_per_pch[pch_idx]<<" ("<<(100*prefetch_mode_cycle_per_pch[pch_idx]/m_clk)<<" %)"<<std::endl;
        std::cout<<"Refresh Mode :"<<refresh_mode_cycle_per_pch[pch_idx]<<" ("<<(100*refresh_mode_cycle_per_pch[pch_idx]/m_clk)<<" %)"<<std::endl;

        if(read_mode_cycle_per_pch[pch_idx] !=0) std::cout<<"Read Mode Util     :"<<busy_read_mode_cycle_per_pch[pch_idx]<<" ("<<(100*busy_read_mode_cycle_per_pch[pch_idx]/read_mode_cycle_per_pch[pch_idx])<<" %)"<<std::endl;
        if(write_mode_cycle_per_pch[pch_idx] !=0) std::cout<<"Write Mode Util    :"<<busy_write_mode_cycle_per_pch[pch_idx]<<" ("<<(100*busy_write_mode_cycle_per_pch[pch_idx]/write_mode_cycle_per_pch[pch_idx])<<" %)"<<std::endl;
        if(prefetch_mode_cycle_per_pch[pch_idx] !=0) std::cout<<"Prefetch Mode Util :"<<busy_prefetch_mode_cycle_per_pch[pch_idx]<<" ("<<(100*busy_prefetch_mode_cycle_per_pch[pch_idx]/prefetch_mode_cycle_per_pch[pch_idx])<<" %)"<<std::endl;
        if(refresh_mode_cycle_per_pch[pch_idx] !=0) std::cout<<"Refresh Mode Util  :"<<busy_refresh_mode_cycle_per_pch[pch_idx]<<" ("<<(100*busy_refresh_mode_cycle_per_pch[pch_idx]/refresh_mode_cycle_per_pch[pch_idx])<<" %)"<<std::endl;
      }
      */     

        std::cout<<"------------------------------------------------"<<std::endl;
        std::cout<<"Command IO Utilization (Channel: "<<m_channel_id<<")"<<std::endl;
        for(int i=0;i<num_pseudochannel;i++) {
          float val = (float)cmd_cycle_per_pch[i]/(float)m_clk;
          std::cout<<val;
          if (i != (num_pseudochannel-1))
            std::cout<<" | ";
        }
        std::cout<<std::endl;
        std::cout<<"------------------------------------------------"<<std::endl;
      
      return;
    }

    bool is_finished() override {
      bool is_dram_ctrl_finished = true;

      if((m_active_buffer.size() != 0) || (pending.size() != 0)) is_dram_ctrl_finished = false;

      if(is_dram_ctrl_finished) {
        for(int i=0;i<num_pseudochannel;i++) {
          if(m_read_buffers[i].size() != 0 || m_write_buffers[i].size() != 0  || m_rd_prefetch_buffers[i].size() != 0  || m_wr_prefetch_buffers[i].size() != 0) is_dram_ctrl_finished = false;
          if(!is_dram_ctrl_finished) break;
        }
      }

      return (is_dram_ctrl_finished);
    }

    bool is_empty_ndp_req() override {
      if(num_ndp_rd_req == 0 && num_ndp_wr_req == 0) return true;
      else                                           return false;
    }

    bool is_empty_ndp_req(int pch_idx) override {
      if(num_ndp_rd_req_per_pch[pch_idx] == 0 && num_ndp_wr_req_per_pch[pch_idx] == 0) return true;
      else                                                                             return false;
    }    

    int get_config_reg_resp(int pch_idx) override {

      if(ndp_config_reg_resp_per_pch[pch_idx].size() == 0) {
        return -1;
      } else {
        int resp = ndp_config_reg_resp_per_pch[pch_idx][0];
        ndp_config_reg_resp_per_pch[pch_idx].erase(ndp_config_reg_resp_per_pch[pch_idx].begin());      
        return resp;        
      }
    }        

    void update_rr_pch_idx() {
      // Shift Round-Robin Pseudo Channel Index
      rr_pch_idx.push_back(rr_pch_idx[0]);
      rr_pch_idx.erase(rr_pch_idx.begin());
    }

};
  
}   // namespace Ramulator