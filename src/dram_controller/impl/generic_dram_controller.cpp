#include "dram_controller/controller.h"
#include "memory_system/memory_system.h"

//#define PRINT_DB_CNT
//#define STAT_RANK_IDLE_TIME
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

    std::vector<ReqBuffer> m_read_buffers;        // Read requestBuffers Per Rank 
    std::vector<ReqBuffer> m_write_buffers;       // Write request Buffers Per rank
    std::vector<ReqBuffer> m_priority_buffers;    // high-priority requests Buffers Per Rank

    size_t buf_size = 32;

    int m_bank_addr_idx = -1;
    int m_rank_addr_idx = -1;
    int m_num_rank      = -1;

    float m_wr_low_watermark;
    float m_wr_high_watermark;
    bool  m_is_write_mode = false;
    std::vector<bool> m_is_write_mode_per_rank;
    std::vector<bool> is_empty_priority_per_rank;
    // Round-Robin Vector
    std::vector<int> rr_rk_idx;


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

    size_t s_num_rw_switch = 0;

    bool use_pseudo_ch = false;
    int psuedo_ch_idx = 0;
    std::vector<int> db_prefetch_cnt_per_pch;
    std::vector<int> db_prefetch_rd_cnt_per_pch;
    std::vector<int> db_prefetch_wr_cnt_per_pch;

    std::vector<size_t> s_num_refresh_cc_per_rank;

    uint32_t s_num_pre_wr;
    uint32_t s_num_post_wr;
    uint32_t s_num_pre_rd;
    uint32_t s_num_post_rd;    

    uint32_t pre_clk = 0;

    // Memory Request Counter for Memory-System 
    uint64_t m_host_access_cnt;
    uint64_t m_tcore_host_access_cnt;

    #ifdef PRINT_DB_CNT
    // Rank-level Idle Time Counter
    std::vector<uint64_t> s_rdwr_cnt;
    std::vector<uint64_t> s_idle_cnt;
    std::vector<uint64_t> s_busy_cnt;
    std::vector<uint64_t> s_idle_interval_10_cnt; // 1-10
    std::vector<uint64_t> s_idle_interval_100_cnt; // 10-100
    std::vector<uint64_t> s_idle_interval_250_cnt; // 100-250
    std::vector<uint64_t> s_idle_interval_500_cnt; // 250-500
    std::vector<uint64_t> s_idle_interval_1000_cnt; // 500-1000
    std::vector<uint64_t> s_idle_interval_over_1000_cnt; // over 1000  
    #endif 

    // Adaptive OpenPage Policy 
    std::vector<int> m_open_row;
    int num_ranks = -1;    
    int num_bankgroups = -1;
    int num_banks = -1;
    int rank_idx = 0;
    int bankgroup_idx = 0;
    int bank_idx = 0;
    int row_idx = 0;
    uint32_t m_adaptive_row_cap;
  public:
    void init() override {
      m_wr_low_watermark =  param<float>("wr_low_watermark").desc("Threshold for switching back to read mode.").default_val(0.2f);
      m_wr_high_watermark = param<float>("wr_high_watermark").desc("Threshold for switching to write mode.").default_val(0.8f);
      m_adaptive_row_cap = param<size_t>("adaptive_row_cap").desc("Row Buffer Hit Cap for Adaptive Open-Page Policy").default_val(16);      
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
      m_rank_addr_idx = m_dram->m_levels("rank");
      m_priority_buffer.max_size = 512*3 + 32;
      m_active_buffer.max_size = 32*4;
      m_prefetched_buffer.max_size = 16*4;
      m_write_buffer.max_size = 128;
      m_read_buffer.max_size = 128;

      num_ranks = m_dram->get_level_size("rank");  
      num_bankgroups = m_dram->get_level_size("bankgroup");  
      num_banks = m_dram->get_level_size("bank");  
      rank_idx = m_dram->m_levels("rank");
      bankgroup_idx = m_dram->m_levels("bankgroup");
      bank_idx = m_dram->m_levels("bank"); 
      row_idx = m_dram->m_levels("row"); 
      // std::cout<<m_active_buffer.max_size<<std::endl;
      // exit(1);

      // if(m_dram->get_level_size("pseudochannel") == -1) use_pseudo_ch = false;
      // else                                              use_pseudo_ch = true;

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

      register_stat(s_num_rw_switch).name("s_num_rw_switch_{}", m_channel_id);
      register_stat(s_num_pre_wr).name("s_num_pre_wr_{}", m_channel_id);
      register_stat(s_num_post_wr).name("s_num_post_wr_{}", m_channel_id);
      register_stat(s_num_pre_rd).name("s_num_pre_rd_{}", m_channel_id);
      register_stat(s_num_post_rd).name("s_num_post_rd_{}", m_channel_id); 
      
      m_host_access_cnt = 0;
      m_tcore_host_access_cnt = 0;      
      
      m_num_rank = m_dram->get_level_size("rank");  
      
      // Initialize Per Rank Request Buffer and set Max Request Capacity     
      m_read_buffers.resize(m_num_rank,ReqBuffer());
      m_write_buffers.resize(m_num_rank,ReqBuffer());
      m_priority_buffers.resize(m_num_rank,ReqBuffer());

      m_is_write_mode_per_rank.resize(m_num_rank, false);     
      is_empty_priority_per_rank.resize(m_num_rank, false);
      s_num_refresh_cc_per_rank.resize(m_num_rank, 0);
      for(int i=0; i<m_num_rank; i++) {
        m_read_buffers[i].max_size = buf_size;
        m_write_buffers[i].max_size = buf_size;
        m_priority_buffers[i].max_size = (512*3 + 32)/4;
        rr_rk_idx.push_back(i);
        register_stat(s_num_refresh_cc_per_rank[i]).name("s_num_refresh_cc_per_rank_{}_{}", m_channel_id,i);      
      }

      #ifdef PRINT_DB_CNT
      s_rdwr_cnt.resize(m_num_rank,0);
      s_idle_cnt.resize(m_num_rank,0);
      s_busy_cnt.resize(m_num_rank,0);
      s_idle_interval_10_cnt.resize(m_num_rank,0);
      s_idle_interval_100_cnt.resize(m_num_rank,0);
      s_idle_interval_250_cnt.resize(m_num_rank,0);
      s_idle_interval_500_cnt.resize(m_num_rank,0);
      s_idle_interval_1000_cnt.resize(m_num_rank,0);
      s_idle_interval_over_1000_cnt.resize(m_num_rank,0);
      #endif

      // Record Row address per bank for Adaptive Open-Page Policy
      m_open_row.resize(num_ranks*num_bankgroups*num_banks, -1);
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
        ReqBuffer& wr_buffer = m_write_buffers[req.addr_vec[m_rank_addr_idx]];    
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
          is_success = m_read_buffers[req.addr_vec[m_rank_addr_idx]].enqueue(req);
          // is_success = m_read_buffer.enqueue(req);
        } else if (req.type_id == Request::Type::Write) {
          is_success = m_write_buffers[req.addr_vec[m_rank_addr_idx]].enqueue(req);       
          // is_success = m_write_buffer.enqueue(req);
        } else {
          throw std::runtime_error("Invalid request type!");
        }

        #ifdef PRINT_DB_CNT
        if(is_success) {
          s_rdwr_cnt[req.addr_vec[m_rank_addr_idx]]++;
        }
        #endif
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
      // is_success = m_priority_buffer.enqueue(req);
      is_success = m_priority_buffers[req.addr_vec[m_rank_addr_idx]].enqueue(req);
      return is_success;
    }

    void tick() override {
      m_clk++;

      // Update statistics
      s_queue_len += m_read_buffer.size() + m_write_buffer.size() + m_priority_buffer.size() + pending.size();
      s_read_queue_len += m_read_buffer.size() + pending.size();
      s_write_queue_len += m_write_buffer.size();
      s_priority_queue_len += m_priority_buffer.size();

      #ifdef PRINT_DB_CNT
      for(int rk=0;rk<m_num_rank;rk++) {
        if(s_rdwr_cnt[rk] == 0) {
          s_idle_cnt[rk]++;
        } else {
          s_busy_cnt[rk]++;
          if(s_idle_cnt[rk] > 0) {
            if(s_idle_cnt[rk] < 10) s_idle_interval_10_cnt[rk] += s_idle_cnt[rk];
            else if(s_idle_cnt[rk] < 100) s_idle_interval_100_cnt[rk] += s_idle_cnt[rk];
            else if(s_idle_cnt[rk] < 250) s_idle_interval_250_cnt[rk] += s_idle_cnt[rk];
            else if(s_idle_cnt[rk] < 500) s_idle_interval_500_cnt[rk] += s_idle_cnt[rk];
            else if(s_idle_cnt[rk] < 1000) s_idle_interval_1000_cnt[rk] += s_idle_cnt[rk];
            else s_idle_interval_over_1000_cnt[rk] += s_idle_cnt[rk];
          }
          s_idle_cnt[rk] = 0;
        }
      }
      #endif

      // Update Each Row Cap       
      for(int rk_id=0;rk_id<(num_ranks);rk_id++) {
        auto rd_buffer = m_read_buffers[rk_id];
        for (auto next = std::next(rd_buffer.begin(), 1); next != rd_buffer.end(); next++) {          
          int flat_bank_id = next->addr_vec[bank_idx] + next->addr_vec[bankgroup_idx] * num_banks + next->addr_vec[rank_idx] * num_bankgroups*num_banks;          
          if((m_open_row[flat_bank_id] != -1) && (m_open_row[flat_bank_id] != next->addr_vec[row_idx])) {
            // Update Cap 
            m_rowpolicy->update_cap(0,next->addr_vec[rank_idx],next->addr_vec[bankgroup_idx],next->addr_vec[bank_idx],m_adaptive_row_cap);
          } 
        }
        auto wr_buffer = m_write_buffers[rk_id];
        for (auto next = std::next(wr_buffer.begin(), 1); next != wr_buffer.end(); next++) {          
          int flat_bank_id = next->addr_vec[bank_idx] + next->addr_vec[bankgroup_idx] * num_banks + next->addr_vec[rank_idx] * num_bankgroups*num_banks;          
          if((m_open_row[flat_bank_id] != -1) && (m_open_row[flat_bank_id] != next->addr_vec[row_idx])) {
            // Update Cap 
            m_rowpolicy->update_cap(0,next->addr_vec[rank_idx],next->addr_vec[bankgroup_idx],next->addr_vec[bank_idx],m_adaptive_row_cap);
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

        
        if(req_it->command == m_dram->m_commands("RD") || req_it->command == m_dram->m_commands("RDA")) s_num_issue_reads++;
        if(req_it->command == m_dram->m_commands("WR") || req_it->command == m_dram->m_commands("WRA")) s_num_issue_writes++;

        if(req_it->command == m_dram->m_commands("REFab")) {
          int nRFC_latency = m_dram->m_timing_vals("nRFC1");
          int rank_addr = req_it->addr_vec[m_rank_addr_idx];
          s_num_refresh_cc_per_rank[rank_addr]+=nRFC_latency;
        }

        if(req_it->command == m_dram->m_commands("ACT")) {
          int flat_bank_id = req_it->addr_vec[bank_idx] + req_it->addr_vec[bankgroup_idx] * num_banks + req_it->addr_vec[rank_idx] * num_bankgroups*num_banks;          
          m_open_row[flat_bank_id] = req_it->addr_vec[row_idx];
          m_rowpolicy->update_cap(0,req_it->addr_vec[rank_idx],req_it->addr_vec[bankgroup_idx],req_it->addr_vec[bank_idx],128);          
        }
        else if(req_it->command == m_dram->m_commands("PRE")) {
          int flat_bank_id = req_it->addr_vec[bank_idx] + req_it->addr_vec[bankgroup_idx] * num_banks + req_it->addr_vec[rank_idx] * num_bankgroups*num_banks;          
          m_open_row[flat_bank_id] = -1;          
        } else if(req_it->command == m_dram->m_commands("PREA")) {
          // Reset All Bank in the Rank
          int rank_id = req_it->addr_vec[rank_idx];
          for(int bg_idx=0;bg_idx<num_bankgroups; bg_idx++) {
            for(int bk_idx=0;bk_idx<num_banks; bk_idx++) {
              int flat_bank_id = bk_idx + bg_idx * num_banks + rank_id * num_bankgroups*num_banks;          
              m_open_row[flat_bank_id] = -1;
            }          
          }
        }

        // If we are issuing the last command, set depart clock cycle and move the request to the pending queue
        if (req_it->command == req_it->final_command) {
          if (req_it->type_id == Request::Type::Read) {
            req_it->depart = m_clk + m_dram->m_read_latency;
            pending.push_back(*req_it);
          } else if (req_it->type_id == Request::Type::Write) {
            // TODO: Add code to update statistics
          }
          
          if(req_it->command == m_dram->m_commands("RD")     || m_dram->m_commands("RDA") || 
             req_it->command == m_dram->m_commands("WR")     || m_dram->m_commands("WRA")) {
              m_host_access_cnt++;
              if(req_it->is_trace_core_req) {
                m_tcore_host_access_cnt++;
              }          
          }          
          #ifdef PRINT_DB_CNT
          s_rdwr_cnt[req_it->addr_vec[m_rank_addr_idx]]--;
          #endif
          buffer->remove(req_it);
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
     * @brief    Checks if we need to switch to write mode per rank
     * 
     */
    void set_write_mode_per_rank(int rk_idx) {      
      if (!m_is_write_mode_per_rank[rk_idx]) {
        if ((m_write_buffers[rk_idx].size() > m_wr_high_watermark * m_write_buffers[rk_idx].max_size) || m_read_buffers[rk_idx].size() == 0) {
          m_is_write_mode_per_rank[rk_idx] = true;
          s_num_rw_switch++;
        }
      } else {
        if ((m_write_buffers[rk_idx].size() < m_wr_low_watermark * m_write_buffers[rk_idx].max_size) && m_read_buffers[rk_idx].size() != 0) {
          m_is_write_mode_per_rank[rk_idx] = false;
          s_num_rw_switch++;
        }
      }
    };    

    void update_rr_rank_idx() {
      // Shift Round-Robin Pseudo Channel Index
      rr_rk_idx.push_back(rr_rk_idx[0]);
      rr_rk_idx.erase(rr_rk_idx.begin());
    }


    /**
     * @brief    Helper function to find a request to schedule from the buffers.
     * 
     */
    bool schedule_request(ReqBuffer::iterator& req_it, ReqBuffer*& req_buffer) {

      bool request_found = false;
     
      // Priority Queue Empty Flag Array 
      for(int rk_idx=0;rk_idx<m_num_rank;rk_idx++) {
        if(m_priority_buffers[rk_idx].size() != 0) {
          is_empty_priority_per_rank[rk_idx] = false;
        } else {
          is_empty_priority_per_rank[rk_idx] = true;
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
        for(auto rk_idx : rr_rk_idx) {
          if (m_priority_buffers[rk_idx].size() != 0) {
            req_buffer = &m_priority_buffers[rk_idx];
            req_it = m_priority_buffers[rk_idx].begin();
            req_it->command = m_dram->get_preq_command(req_it->final_command, req_it->addr_vec);
            
            request_found = m_dram->check_ready(req_it->command, req_it->addr_vec);

            if(request_found) break;
            // if (!request_found & m_priority_buffer.size() != 0) {
            //   return false;
            // }
          }
        }

        // 2.2.1    If no request to be scheduled in the priority buffer, check the read and write buffers.
        if (!request_found) {
          // Query the write policy to decide which buffer to servef
          for(auto rk_idx : rr_rk_idx) {
            set_write_mode_per_rank(rk_idx);
            if(is_empty_priority_per_rank[rk_idx]) {
              auto& buffer = m_is_write_mode_per_rank[rk_idx] ? m_write_buffers[rk_idx] : m_read_buffers[rk_idx];
              if (req_it = m_scheduler->get_best_request(buffer); req_it != buffer.end()) {
                request_found = m_dram->check_ready(req_it->command, req_it->addr_vec);
                req_buffer = &buffer;
              }
              if(request_found) break;
            }
          }
          /*
          //Single Queue Per Channel 
          set_write_mode();
          auto& buffer = m_is_write_mode ? m_write_buffer : m_read_buffer;
          if (req_it = m_scheduler->get_best_request(buffer); req_it != buffer.end()) {
            request_found = m_dram->check_ready(req_it->command, req_it->addr_vec);
            req_buffer = &buffer;
          }
          */
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
              break;
            }
          }
        }
      }

      if(request_found) update_rr_rank_idx();
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
    
      #ifdef PRINT_DB_CNT
      for(int rk=0;rk<m_num_rank;rk++) {
        std::cout<<" Rank ["<<rk<<"] Idle Time Breakdown (%)"<<std::endl;
        std::cout<<"    - Busy           : "<<(100.0 * float(s_busy_cnt[rk]) / float(m_clk))<<std::endl;
        std::cout<<"    - IDLE(1_10)     : "<<(100.0 * float(s_idle_interval_10_cnt[rk]) / float(m_clk))<<std::endl;
        std::cout<<"    - IDLE(10_100)   : "<<(100.0 * float(s_idle_interval_100_cnt[rk]) / float(m_clk))<<std::endl;
        std::cout<<"    - IDLE(100_250)  : "<<(100.0 * float(s_idle_interval_250_cnt[rk]) / float(m_clk))<<std::endl;
        std::cout<<"    - IDLE(250_500)  : "<<(100.0 * float(s_idle_interval_500_cnt[rk]) / float(m_clk))<<std::endl;
        std::cout<<"    - IDLE(500_1000) : "<<(100.0 * float(s_idle_interval_1000_cnt[rk]) / float(m_clk))<<std::endl;
        std::cout<<"    - IDLE(1000) -   : "<<(100.0 * float(s_idle_interval_over_1000_cnt[rk]) / float(m_clk))<<std::endl;
      }
      #endif
      return;
    }

    bool is_finished() override {
      bool is_dram_ctrl_finished = true;
      if((m_active_buffer.size() != 0) || (m_read_buffer.size() != 0) || (pending.size() != 0)) 
      // || (m_write_buffer.size() != 0) 
        is_dram_ctrl_finished = false;

      return (is_dram_ctrl_finished);

    }    


    bool is_empty_ndp_req() override {    
      return true;
     }        

    bool is_empty_ndp_req(int pch_idx) override {
      return true;
    }        

    int get_config_reg_resp(int pch_idx) override {
      return -1;
    }      

    virtual size_t get_host_acces_latency() {
      return s_read_latency;
    }    

    std::vector<uint64_t> get_counters() override {
      std::vector<uint64_t> v;
      v.reserve(2);
      v.push_back(m_host_access_cnt);
      v.push_back(m_tcore_host_access_cnt);

      return v; 
    }    
};
  
}   // namespace Ramulator