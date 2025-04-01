#include "dram_controller/controller.h"
#include "memory_system/memory_system.h"

//#define PRINT_DB_CNT

namespace Ramulator {

class GenericDRAMController final : public IDRAMController, public Implementation {
  RAMULATOR_REGISTER_IMPLEMENTATION(IDRAMController, GenericDRAMController, "Generic", "A generic DRAM controller.");
  private:
    std::deque<Request> pending;          // A queue for read requests that are about to finish (callback after RL)

    ReqBuffer m_active_buffer;            // Buffer for requests being served. This has the highest priority 
    ReqBuffer m_priority_buffer;          // Buffer for high-priority requests (e.g., maintenance like refresh).
    ReqBuffer m_read_buffer;              // Read request buffer
    ReqBuffer m_write_buffer;             // Write request buffer
    ReqBuffer m_prefetched_buffer;        // Prefetched buffer

    int m_bank_addr_idx = -1;

    float m_wr_low_watermark;
    float m_wr_high_watermark;
    bool  m_is_write_mode = false;

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
    float s_queue_len_avg = 0;
    float s_read_queue_len_avg = 0;
    float s_write_queue_len_avg = 0;
    float s_priority_queue_len_avg = 0;

    size_t s_read_latency = 0;
    float s_avg_read_latency = 0;

    float s_bandwidth = 0;
    float s_dq_bandwidth = 0;
    float s_max_bandwidth = 0;
    size_t s_num_issue_reads = 0;
    size_t s_num_issue_writes = 0;

    std::vector<size_t> s_num_trans_per_pch;
    std::vector<size_t> s_num_refresh_cc_per_pch;
    std::vector<size_t> s_num_busy_refresh_cc_per_pch;
    std::vector<size_t> s_num_max_prefetch_per_pch;
    size_t s_num_rw_switch = 0;

    bool use_pseudo_ch = false;
    int psuedo_ch_idx = 0;
    std::vector<int> db_prefetch_cnt_per_pch;
    std::vector<int> db_prefetch_rd_cnt_per_pch;
    std::vector<int> db_prefetch_wr_cnt_per_pch;

    uint32_t s_num_pre_wr;
    uint32_t s_num_post_wr;
    uint32_t s_num_pre_rd;
    uint32_t s_num_post_rd;    

    uint32_t pre_clk = 0;
    std::vector<int> io_busy_clk_per_pch;

    std::vector<bool> is_empty_priority_per_pch;

    // Enable Write/Read Prefetcher from DRAM to DB
    bool m_use_wr_prefetch;
    bool m_use_rd_prefetch;
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

      m_use_rd_prefetch = m_dram->get_use_rd_prefetch();
      m_use_wr_prefetch = m_dram->get_use_wr_prefetch();

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

      int num_pseudochannel = m_dram->get_level_size("pseudochannel");  
      if(num_pseudochannel != -1) {
        s_num_trans_per_pch.resize(num_pseudochannel, 0);
        s_num_refresh_cc_per_pch.resize(num_pseudochannel, 0);
        s_num_busy_refresh_cc_per_pch.resize(num_pseudochannel, 0);
        db_prefetch_cnt_per_pch.resize(num_pseudochannel, 0);
        db_prefetch_rd_cnt_per_pch.resize(num_pseudochannel, 0);
        db_prefetch_wr_cnt_per_pch.resize(num_pseudochannel, 0);
        s_num_max_prefetch_per_pch.resize(num_pseudochannel, 0);


        io_busy_clk_per_pch.resize(num_pseudochannel, 0);
        is_empty_priority_per_pch.resize(num_pseudochannel, 0);
        for (size_t pch_id = 0; pch_id < num_pseudochannel; pch_id++) {
          register_stat(s_num_trans_per_pch[pch_id]).name("s_num_trans_per_pch_{}_{}", m_channel_id,pch_id);        
          register_stat(s_num_refresh_cc_per_pch[pch_id]).name("s_num_refresh_cc_per_pch_{}_{}", m_channel_id,pch_id);      
          register_stat(s_num_busy_refresh_cc_per_pch[pch_id]).name("s_num_busy_refresh_cc_per_pch_{}_{}", m_channel_id,pch_id);             
          register_stat(s_num_max_prefetch_per_pch[pch_id]).name("s_num_max_prefetch_per_pch_{}_{}", m_channel_id,pch_id);             
        }
      }

      register_stat(s_num_rw_switch).name("s_num_rw_switch_{}", m_channel_id);
      register_stat(s_num_pre_wr).name("s_num_pre_wr_{}", m_channel_id);
      register_stat(s_num_post_wr).name("s_num_post_wr_{}", m_channel_id);
      register_stat(s_num_pre_rd).name("s_num_pre_rd_{}", m_channel_id);
      register_stat(s_num_post_rd).name("s_num_post_rd_{}", m_channel_id);      
    };

    bool send(Request& req) override {
      bool is_success = false;
      req.final_command = m_dram->m_request_translations(req.type_id);

      // Forward existing write requests to incoming read requests
      if (req.type_id == Request::Type::Read) {
        // lamda function 
        auto compare_addr = [req](const Request& wreq) {
          return wreq.addr == req.addr;
        };
        // if existing write request which is same address with read, send to pending request queue
        if (std::find_if(m_write_buffer.begin(), m_write_buffer.end(), compare_addr) != m_write_buffer.end()) {
          // The request will depart at the next cycle
          req.depart = m_clk + 1;
          pending.push_back(req);
          is_success = true;
          // return true;
        }
      }

      // Else, enqueue them to corresponding buffer based on request type id
      if(!is_success) {
        req.arrive = m_clk;
        if        (req.type_id == Request::Type::Read) {
          is_success = m_read_buffer.enqueue(req);
        } else if (req.type_id == Request::Type::Write) {
          is_success = m_write_buffer.enqueue(req);
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
      }

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
      s_queue_len += m_read_buffer.size() + m_write_buffer.size() + m_priority_buffer.size() + pending.size();
      s_read_queue_len += m_read_buffer.size() + pending.size();
      s_write_queue_len += m_write_buffer.size();
      s_priority_queue_len += m_priority_buffer.size();

      if(use_pseudo_ch) {
        for(int pch=0;pch<m_dram->m_organization.count[1];pch++) {
          bool is_find = false;
          for(auto req_it : m_read_buffer) {
            if(req_it.addr_vec[1] == pch) is_find = true;
          }
          for(auto req_it : m_read_buffer) {
            if(req_it.addr_vec[1] == pch) is_find = true;
          }
          for(auto req_it : m_active_buffer) {
            if(req_it.addr_vec[1] == pch) is_find = true;
          }

          if(m_clk>io_busy_clk_per_pch[pch]) {
          }
        }
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
        // If we find a real request to serve
        if (req_it->is_stat_updated == false/* && !req_it->is_db_cmd*/) {
          update_request_stats(req_it);
        }
        m_dram->issue_command(req_it->command, req_it->addr_vec);


        // Update PRE_WR/POST_WR Counter 
        if(use_pseudo_ch) {
          if(!req_it->is_actived && req_it->final_command != m_dram->m_commands("POST_RD") && 
              (req_it->command == m_dram->m_commands("P_ACT") || req_it->command == m_dram->m_commands("PRE_WR"))) {
            // Issued DDR commands related with PRE_WR
            s_num_pre_wr++;
            db_prefetch_cnt_per_pch[req_it->addr_vec[psuedo_ch_idx]]++;
            db_prefetch_wr_cnt_per_pch[req_it->addr_vec[psuedo_ch_idx]]++;
            req_it->is_actived = true;
            #ifdef PRINT_DB_CNT
            std::cout<<"PRE_WR ["<<req_it->addr_vec[0]<<"]["<<req_it->addr_vec[1]<<"] |";
            std::cout<<" db cnt "<<db_prefetch_cnt_per_pch[req_it->addr_vec[psuedo_ch_idx]];
            std::cout<<" db rd cnt "<<db_prefetch_rd_cnt_per_pch[req_it->addr_vec[psuedo_ch_idx]];
            std::cout<<" db wr cnt "<<db_prefetch_wr_cnt_per_pch[req_it->addr_vec[psuedo_ch_idx]]<<std::endl;                       
            #endif 
          }
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
          if(req_it->command  == m_dram->m_commands("PRE_RD") || req_it->command  == m_dram->m_commands("PRE_RDA")) {
            s_num_pre_rd++;
            db_prefetch_cnt_per_pch[req_it->addr_vec[psuedo_ch_idx]]++;
            db_prefetch_rd_cnt_per_pch[req_it->addr_vec[psuedo_ch_idx]]++;
            #ifdef PRINT_DB_CNT
            std::cout<<"PRE_RD ["<<req_it->addr_vec[0]<<"]["<<req_it->addr_vec[1]<<"] |";
            std::cout<<" db cnt "<<db_prefetch_cnt_per_pch[req_it->addr_vec[psuedo_ch_idx]];
            std::cout<<" db rd cnt "<<db_prefetch_rd_cnt_per_pch[req_it->addr_vec[psuedo_ch_idx]];
            std::cout<<" db wr cnt "<<db_prefetch_wr_cnt_per_pch[req_it->addr_vec[psuedo_ch_idx]]<<std::endl;            
            #endif
          }
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
          if(db_prefetch_cnt_per_pch[req_it->addr_vec[psuedo_ch_idx]] > 16 || db_prefetch_cnt_per_pch[req_it->addr_vec[psuedo_ch_idx]] < 0) {
            std::cout<<"["<<m_clk<<"] CH["<<req_it->addr_vec[0]<<"]PCH["<<req_it->addr_vec[1]<<"] Over Prefetched cnt"<<std::endl;
            std::cout<<" - "<<db_prefetch_cnt_per_pch[req_it->addr_vec[psuedo_ch_idx]]<<std::endl;
            exit(1);
          }
          }

        if(use_pseudo_ch) {
          // Convert Normal WR(A) to PRE_WR when issue WR Prefetch to DB
          if(req_it->command  == m_dram->m_commands("P_ACT") || req_it->command  == m_dram->m_commands("PRE_WR")) {
            if((req_it->final_command == m_dram->m_commands("WR")) || (req_it->final_command == m_dram->m_commands("WRA"))) {
              req_it->final_command = m_dram->m_commands("PRE_WR");            
            }
          }
          // if(req_it->command  == m_dram->m_commands("P_ACT") || req_it->command  == m_dram->m_commands("POST_RD")) {
          //   if((req_it->final_command == m_dram->m_commands("RD")) || (req_it->final_command == m_dram->m_commands("RDA")))
          //     req_it->final_command = m_dram->m_commands("POST_RD");          
          // }          

          if(req_it->command  == m_dram->m_commands("PRE_RD") || req_it->command  == m_dram->m_commands("PRE_RDA")) {
            req_it->final_command = req_it->command;
          }                    
        }

        if(false && req_it->addr_vec[0] == 0 && req_it->addr_vec[1] == 1) {

            int act_req_cnt=0;
            for (auto& req : m_active_buffer) {
              if(req.addr_vec[1] == 0 && req.addr_vec[0] == 0) act_req_cnt++;
            }
            int rd_req_cnt=0;
            for (auto& req : m_read_buffer) {
              if(req.addr_vec[1] == 0 && req.addr_vec[0] == 0) rd_req_cnt++;
            }            
            int wr_req_cnt=0;
            for (auto& req : m_write_buffer) {
              if(req.addr_vec[1] == 0 && req.addr_vec[0] == 0) wr_req_cnt++;
            }      

          std::cout<<"["<<(m_clk-pre_clk)<<"] ";
          std::cout<<" Ramined Req (active_buf/read_buf/write_buf) "<<(act_req_cnt)<<" / "<<rd_req_cnt<<" / "<<wr_req_cnt<<" - ";
          m_dram->print_req(*req_it);
          pre_clk = m_clk;
        }
        
        if(use_pseudo_ch) {
          if(req_it->command == m_dram->m_commands("RD") || req_it->command == m_dram->m_commands("RDA") || req_it->command == m_dram->m_commands("POST_RD")) s_num_issue_reads++;
          if(req_it->command == m_dram->m_commands("WR") || req_it->command == m_dram->m_commands("WRA") || req_it->command == m_dram->m_commands("PRE_WR"))  s_num_issue_writes++;
          if((req_it->command == m_dram->m_commands("RD") || req_it->command == m_dram->m_commands("RDA") || req_it->command == m_dram->m_commands("POST_RD")) || 
             (req_it->command == m_dram->m_commands("WR") || req_it->command == m_dram->m_commands("WRA") || req_it->command == m_dram->m_commands("PRE_WR"))) {
              s_num_trans_per_pch[req_it->addr_vec[psuedo_ch_idx]]++;
          }
        } else {
          if(req_it->command == m_dram->m_commands("RD") || req_it->command == m_dram->m_commands("RDA")) s_num_issue_reads++;
          if(req_it->command == m_dram->m_commands("WR") || req_it->command == m_dram->m_commands("WRA")) s_num_issue_writes++;
        }

        if(req_it->command == m_dram->m_commands("REFab")) {
          if(use_pseudo_ch) {        
            int nRFC_latency = m_dram->m_timing_vals("nRFC1");
            int pch_addr = req_it->addr_vec[psuedo_ch_idx];
            s_num_refresh_cc_per_pch[pch_addr]+=nRFC_latency;
            if(false && db_prefetch_cnt_per_pch[pch_addr]!=0) {
             std::cout<<"Refresh start But some entry of prefetched buffer are remained CH/PCH : "<<req_it->addr_vec[0]<<"/"<<req_it->addr_vec[1]<<std::endl;
             std::cout<<" Prefetched Counter  :"<<db_prefetch_cnt_per_pch[pch_addr]<<std::endl;
             for (auto& req : m_prefetched_buffer) m_dram->print_req(req);
             
             std::cout<<" Active Buffer       :"<<m_active_buffer.size()<<std::endl;
             for (auto& req : m_active_buffer) m_dram->print_req(req);

             std::cout<<" prefetched buffer :"<<m_prefetched_buffer.size()<<std::endl;
             for (auto& req : m_prefetched_buffer) m_dram->print_req(req);              

             std::cout<<" write buffer size :"<<m_write_buffer.size()<<std::endl;
             for (auto& req : m_write_buffer) m_dram->print_req(req); 
             exit(1);
            }
          }    
        }
        
        if(use_pseudo_ch) {
          if(req_it->command == m_dram->m_commands("PRE_WR") || req_it->command == m_dram->m_commands("POST_RD")) {
            auto& req_addr_vec = req_it->addr_vec;
            int pch_addr = req_addr_vec[psuedo_ch_idx];
            s_num_busy_refresh_cc_per_pch[req_it->addr_vec[psuedo_ch_idx]]+=(8*4);
          } 
        }

        // If we are issuing the last command, set depart clock cycle and move the request to the pending queue
        if (req_it->command == req_it->final_command) {
          if (req_it->type_id == Request::Type::Read) {
            if(use_pseudo_ch) {
              if(!(req_it->command == m_dram->m_commands("PRE_RD") || req_it->command == m_dram->m_commands("PRE_RDA"))) {
                req_it->depart = m_clk + m_dram->m_read_latency;
                pending.push_back(*req_it);  
              }
            } else {
              req_it->depart = m_clk + m_dram->m_read_latency;
              pending.push_back(*req_it);
            }
          } else if (req_it->type_id == Request::Type::Write) {
            // TODO: Add code to update statistics
          }
          
          if(use_pseudo_ch) {
            if(req_it->command == m_dram->m_commands("PRE_WR")) {
              // Generate POST_WR and Enqueue to Prefetched Buffer
              Request new_req = Request(req_it->addr_vec, Request::Type::Write);
              new_req.addr          = req_it->addr;
              new_req.arrive        = req_it->arrive;
              new_req.final_command = m_dram->m_commands("POST_WR");
              bool is_success = false;              

              buffer->remove(req_it);
              is_success = m_prefetched_buffer.enqueue(new_req);
              if(!is_success) {
                throw std::runtime_error("Fail to enque to m_prefetched_buffer");
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
              is_success = m_prefetched_buffer.enqueue(new_req);
              if(!is_success) {
                throw std::runtime_error("Fail to enque to m_prefetched_buffer");
              }                    
            } else {
              buffer->remove(req_it);
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
        if ((m_write_buffer.size() > m_wr_high_watermark * m_write_buffer.max_size) || m_read_buffer.size() == 0) {
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


        // Find request in prefetched buffer
        if(use_pseudo_ch) {
          // Only Called by DDR5-PCH
          set_write_mode();
          bool is_refreshing = m_dram->check_dram_refrsehing();
          if(!request_found && m_prefetched_buffer.size() != 0) {
            // std::cout<<"["<<m_clk<<"] try .. get request .. within "<<m_prefetched_buffer.size()<<std::endl;;
            if (req_it = m_scheduler->get_best_request_prefetch_with_mask(m_prefetched_buffer,is_empty_priority_per_pch,is_refreshing,m_is_write_mode); req_it != m_prefetched_buffer.end()) {
              request_found = m_dram->check_ready(req_it->command, req_it->addr_vec);
              req_buffer = &m_prefetched_buffer;
            }
            // std::cout<<" result .. "<<request_found<<std::endl;
            // if(request_found) std::cout<<"Request from Prefteched BUF"<<std::endl;
          }
        }

        // 2.2.1    If no request to be scheduled in the priority buffer, check the read and write buffers.
        if (!request_found) {
          // Query the write policy to decide which buffer to servef
          set_write_mode();
          auto& buffer = m_is_write_mode ? m_write_buffer : m_read_buffer;
          if(use_pseudo_ch) {
            if (req_it = m_scheduler->get_best_request_with_mask(buffer,is_empty_priority_per_pch); req_it != buffer.end()) {
              request_found = m_dram->check_ready(req_it->command, req_it->addr_vec);
              req_buffer = &buffer;
            }
          } else {
            if (req_it = m_scheduler->get_best_request(buffer); req_it != buffer.end()) {
              request_found = m_dram->check_ready(req_it->command, req_it->addr_vec);
              req_buffer = &buffer;
            }
          }

          // Check WR Command to issue PRE_WR command during refreshing..
          if(use_pseudo_ch) {
            // Only Called by DDR5-PCH
            bool is_refreshing = m_dram->check_dram_refrsehing();
            if(!request_found && is_refreshing && m_use_wr_prefetch) {
              auto& buffer = m_write_buffer;
              if (req_it = m_scheduler->get_best_request_refresh_ch_with_mask(buffer,is_empty_priority_per_pch); req_it != buffer.end()) {
                request_found = m_dram->check_ready(req_it->command, req_it->addr_vec);
                req_buffer = &buffer;
                req_it->is_db_cmd = true;
              }            
            }
          }
        }
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
              }
              break;
            }
          }
        }
      }

      return request_found;
    }

    void finalize() override {
      s_avg_read_latency = (float) s_read_latency / (float) s_num_read_reqs;

      s_queue_len_avg = (float) s_queue_len / (float) m_clk;
      s_read_queue_len_avg = (float) s_read_queue_len / (float) m_clk;
      s_write_queue_len_avg = (float) s_write_queue_len / (float) m_clk;
      s_priority_queue_len_avg = (float) s_priority_queue_len / (float) m_clk;

      // GB/s 
      // Request Bandwidth, DQ Bandwidth, Max DQ Bandwidth
      int tx_bytes = m_dram->m_internal_prefetch_size * m_dram->m_channel_width / 8;
      s_bandwidth = ((float)((s_num_read_reqs + s_num_write_reqs) * tx_bytes) / (float)(m_clk * m_dram->m_timing_vals("tCK_ps"))) * 1E12 / (1024*1024*1012);
      s_dq_bandwidth = ((float)((s_num_issue_reads + s_num_issue_writes) * tx_bytes) / (float)(m_clk * m_dram->m_timing_vals("tCK_ps"))) * 1E12 / (1024*1024*1012);
      // rate unit is MT/s 
      // 10^6 T/s --> /(2^30) GB/s
      s_max_bandwidth = (float)(m_dram->m_channel_width / 8) * (float)m_dram->m_timing_vals("rate") * 1E6 / (1024*1024*1012);      
    
      return;
    }

    bool is_finished() override {
      bool is_dram_ctrl_finished = true;
      if((m_active_buffer.size() != 0) || (m_read_buffer.size() != 0) || (m_write_buffer.size() != 0) || (pending.size() != 0)) 
        is_dram_ctrl_finished = false;

      return (is_dram_ctrl_finished);

    }    


    bool is_empty_ndp_req() override {    
      return false;
     }        

};
  
}   // namespace Ramulator