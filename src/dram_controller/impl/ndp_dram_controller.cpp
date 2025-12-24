#include "dram_controller/controller.h"
#include "memory_system/memory_system.h"
#include <iomanip>  

// #define PRINT_DB_CNT
// #define PRINT_DEBUG

#ifdef PRINT_DEBUG
#define DEBUG_PRINT(clk, unit_str, ch, pch, msg) do { std::cout <<"["<<clk<<"]["<<unit_str<<"] CH["<<ch<<"] PCH["<<pch<<"]"<<msg<<std::endl; } while(0)
#else
#define DEBUG_PRINT(clk, unit_str, ch, pch, msg) do {} while(0)
#endif

namespace Ramulator {

class NDPDRAMController final : public IDRAMController, public Implementation {
  RAMULATOR_REGISTER_IMPLEMENTATION(IDRAMController, NDPDRAMController, "ndpDRAMCtrl", "A NDP DRAM controller.");
  private:
    Logger_t m_logger;
    std::deque<Request> pending;           // A queue for read requests that are about to finish (callback after RL)
    // Hardcoding Pseudo-Channel is fixed to 4
    ReqBuffer m_active_buffer;             // Buffer for requests being served. This has the highest priority 
    ReqBuffer m_priority_buffer;           // Buffer for high-priority requests (e.g., maintenance like refresh).
    // ReqBuffer m_read_buffer;               // Read request buffer
    // ReqBuffer m_write_buffer;              // Write request buffer
    // ReqBuffer m_prefetched_buffer;         // Prefetched buffer
    
    std::vector<ReqBuffer> m_read_buffers;        // Read requestBuffers Per Pseudo Channel
    std::vector<ReqBuffer> m_write_buffers;       // Write request Buffers Per Pseudo Channel    
    std::vector<ReqBuffer> m_priority_buffers;    // high-priority requests Buffers Per Pseudo Channel    
    std::vector<ReqBuffer> m_rd_prefetch_buffers; // Read Prefetch Buffers (D2PA Buffer) Per Pseudo Channel    
    std::vector<ReqBuffer> m_wr_prefetch_buffers; // Write Prefetch Buffers (D2PA Buffer) Per Pseudo Channel    

    std::vector<std::vector<std::pair<Request, int>>> m_to_rd_prefetch_buffers;
    std::vector<std::vector<std::pair<Request, int>>> m_to_wr_prefetch_buffers;

    // Decoupled Priority Mode (MC <-> DB / DB <-> DRAM)
    enum MC_DB_RW_MODE {
      DB_NDP_WR, 
      DB_RD, 
      DB_WR
    };

    enum DB_DRAM_RW_MODE {
      DRAM_REF,
      DRAM_RD, 
      DRAM_WR,
      DRAM_NDP_WR
    };

    // Mode name strings for printing
    const std::array<std::string, 3> MC_DB_MODE_NAMES = {
        "DB_NDP_WR", "DB_RD", "DB_WR"
    };
    
    const std::array<std::string, 4> DB_DRAM_MODE_NAMES = {
        "DRAM_REF", "DRAM_RD", "DRAM_WR", "DRAM_NDP_WR"
    };

    // mc_db, db_dram rw mode per PCH 
    std::vector<enum MC_DB_RW_MODE> m_mc_db_rw_modes;
    std::vector<enum DB_DRAM_RW_MODE> m_db_dram_rw_modes;
    
    // Counter for Mode Selection 
    std::vector<size_t> m_num_rd_cnts; 
    std::vector<size_t> m_num_db_rd_cnts;
    std::vector<size_t> m_num_dram_rd_cnts;
    std::vector<size_t> m_num_wr_cnts;
    std::vector<size_t> m_num_db_wr_cnts;
    std::vector<size_t> m_num_dram_wr_cnts;
    std::vector<size_t> m_num_post_rd_cnts;
    std::vector<size_t> m_num_post_wr_cnts;
    std::vector<size_t> m_num_ref_cnts;
    std::vector<Clk_t> m_last_ndp_dram_wr;
    std::vector<Clk_t> m_last_host_rd;

    std::vector<size_t> m_num_read_req;
    std::vector<size_t> m_num_write_req;

    std::vector<size_t> m_ndp_dram_wr_timer;    
    std::vector<size_t> m_dram_rd_timer;
    std::vector<size_t> m_mc_rd_timer;
    std::vector<size_t> m_dram_ndp_rd_token;
    std::vector<bool>   m_enable_pre_rd;

    // Per-Bank RD/WR Counter
    std::vector<size_t> m_host_access_cnt_per_bank;
    std::vector<size_t> m_ndp_access_cnt_per_bank;

    // Per-channel mode statistics
    struct ChannelModeStats {
        // MC <-> DB mode cycles
        std::array<uint64_t, 3> mc_db_mode_cycles;  // [DB_NDP_WR, DB_RD, DB_WR]
        
        // DB <-> DRAM mode cycles
        std::array<uint64_t, 4> db_dram_mode_cycles;  // [DRAM_REF, DRAM_RD, DRAM_WR, DRAM_NDP_WR]
            
        ChannelModeStats() 
            : mc_db_mode_cycles{0, 0, 0},
              db_dram_mode_cycles{0, 0, 0, 0} {}      
    };

    // Statistics per pseudo channel
    std::vector<ChannelModeStats> m_channel_stats;
    std::vector<ChannelModeStats> m_periodic_channel_stats;

    // Enable Write/Read Prefetcher (D2PA Mode)
    bool m_use_prefetch;

    // tracking ndp_related rquest
    int num_ndp_total_rd_req;
    int num_ndp_total_wr_req;
    int num_ndp_rd_req;
    int num_ndp_wr_req;
    std::vector<int> num_ndp_wr_req_per_pch;
    std::vector<int> num_ndp_rd_req_per_pch;

    int m_bank_addr_idx = -1;

    float m_wr_low_watermark;
    float m_wr_high_watermark;
    size_t m_wr_high_threshold;
    size_t m_wr_low_threshold;
    size_t ndp_dram_wr_max_age;
    size_t ndp_wr_mode_min_time;
    size_t dram_rd_mode_min_time;
    // NDP Request threshold (0.0 ~ 1.0)
    float m_ndp_read_max_threshold;
    float m_ndp_write_max_threshold;
    float m_ndp_read_high_threshold;
    float m_ndp_write_high_threshold;    
    float m_ndp_read_low_threshold;
    float m_ndp_write_low_threshold;    
    size_t high_max_ndp_read_reqs;
    size_t low_max_ndp_read_reqs;
    size_t high_max_ndp_write_reqs;
    size_t low_max_ndp_write_reqs;

    // Calculated max NDP request counts
    std::vector<size_t> m_max_ndp_read_reqs;     // max_ndp_read_reqs = read_buffer_size * threshold
    std::vector<size_t> m_max_ndp_write_reqs;    // max_ndp_write_reqs = write_buffer_size * threshold
    
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
    size_t s_normal_read_latency = 0;
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
    // std::vector<size_t> s_num_busy_refresh_cc_per_pch;
    // std::vector<size_t> s_num_max_prefetch_per_pch;
    // std::vector<size_t> s_num_write_mode_per_pch;
    std::vector<size_t> s_narrow_io_busy_clk_per_pch;
    std::vector<size_t> s_wide_io_busy_clk_per_pch;
    size_t s_num_rw_switch = 0;

    bool use_pseudo_ch = false;
    int num_pseudochannel = -1;
    int num_bankgroup = -1;
    int num_bank = -1;
    int ch_idx = 0;
    int psuedo_ch_idx = 0;
    int bankgroup_idx = 0;
    int bank_idx = 0;
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

    uint32_t m_prev_num_cmd = 0;
    uint32_t m_prev_num_rd = 0;
    uint32_t m_prev_num_wr = 0;
    uint32_t m_prev_num_pre_wr = 0;
    uint32_t m_prev_num_post_wr = 0;
    uint32_t m_prev_num_pre_rd = 0;
    uint32_t m_prev_num_post_rd = 0;
    uint32_t m_prev_num_ndp_dram_rd = 0;
    uint32_t m_prev_num_ndp_dram_wr = 0;
    uint32_t m_prev_num_ndp_db_rd = 0;
    uint32_t m_prev_num_ndp_db_wr = 0;
    
    std::vector <uint32_t> m_his_num_cmd;
    std::vector <uint32_t> m_his_num_rd;
    std::vector <uint32_t> m_his_num_wr;
    std::vector <uint32_t> m_his_num_pre_wr;
    std::vector <uint32_t> m_his_num_post_wr;
    std::vector <uint32_t> m_his_num_pre_rd;
    std::vector <uint32_t> m_his_num_post_rd;
    std::vector <uint32_t> m_his_num_ndp_dram_rd;
    std::vector <uint32_t> m_his_num_ndp_dram_wr;
    std::vector <uint32_t> m_his_num_ndp_db_rd;
    std::vector <uint32_t> m_his_num_ndp_db_wr;

    float s_cmd_io_util = 0.0;
    uint32_t cmd_io_cc = 0; 

    uint32_t pre_clk = 0;
    std::vector<int> io_busy_clk_per_pch;

    std::vector<bool> is_empty_priority_per_pch;

    // NDP CONF REG RD Response from DRAM
    std::vector<std::vector<int>> ndp_config_reg_resp_per_pch;

    // Round-Robin Vector
    std::vector<int> rr_pch_idx;

    uint64_t m_avg_active_buffer;
    std::vector<uint64_t> m_avg_read_buffers;
    std::vector<uint64_t> m_avg_write_buffers;;
    std::vector<uint64_t> m_avg_priority_buffers;
    std::vector<uint64_t> m_avg_rd_prefetch_buffers;
    std::vector<uint64_t> m_avg_wr_prefetch_buffers;

    size_t buf_size = 32;

    struct CmdIds {
      int ACT = -1;
      int P_ACT = -1;
      int PRE = -1;
      int PREA = -1;
      int PREsb = -1;
      int P_PRE = -1;
      int RD = -1;
      int WR = -1;
      int RDA = -1;
      int WRA = -1;
      int PRE_RD = -1;
      int PRE_WR = -1;
      int POST_RD = -1;
      int POST_WR = -1;
      int PRE_RDA = -1;
      int POST_WRA = -1;
      int NDP_DRAM_RD = -1;
      int NDP_DRAM_WR = -1;
      int NDP_DRAM_RDA = -1;
      int NDP_DRAM_WRA = -1;
      int NDP_DB_RD = -1;
      int NDP_DB_WR = -1;
      int REFab = -1;
      int REFsb = -1;
      int REFab_end = -1;
      int REFsb_end = -1;
    };    

    CmdIds m_cmds;

  public:
    void init() override {      
      m_wr_low_watermark =  param<float>("wr_low_watermark").desc("Threshold for switching back to read mode.").default_val(0.2f);
      m_wr_high_watermark = param<float>("wr_high_watermark").desc("Threshold for switching to write mode.").default_val(0.8f);
      ndp_dram_wr_max_age = param<size_t>("ndp_wr_max_age").desc("Threshold for Maxium NDP Write Age").default_val(512);
      ndp_wr_mode_min_time = param<size_t>("ndp_wr_mode_min_time").desc("Keep NDP_WR_MODE at least ..").default_val(512);
      dram_rd_mode_min_time = param<size_t>("dram_rd_mode_min_time").desc("Keep DRAM_RD at least ..").default_val(512);
      m_ndp_read_high_threshold = param<float>("ndp_read_high_threshold").desc("Threshold for NDP Read Request").default_val(0.5f);
      m_ndp_write_high_threshold = param<float>("ndp_write_high_threshold").desc("Threshold for NDP Write Request").default_val(0.5f);
      m_ndp_read_low_threshold = param<float>("ndp_read_low_threshold").desc("Threshold for NDP Read Request").default_val(0.5f);
      m_ndp_write_low_threshold = param<float>("ndp_write_low_threshold").desc("Threshold for NDP Write Request").default_val(0.5f);      

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
      // m_prefetched_buffer.max_size = 16*4;
      // m_write_buffer.max_size = 64;
      // m_read_buffer.max_size = 64;

      if(m_dram->get_level_size("pseudochannel") == -1) use_pseudo_ch = false;
      else                                              use_pseudo_ch = true;

      ch_idx = m_dram->m_levels("channel");
      psuedo_ch_idx = m_dram->m_levels("pseudochannel");
      bankgroup_idx = m_dram->m_levels("bankgroup");
      bank_idx = m_dram->m_levels("bank");

      m_use_prefetch = m_dram->get_use_prefetch();

      m_num_cores = frontend->get_num_cores();

      m_logger = Logging::create_logger("NDPDRAMCtrl_"+std::to_string(m_channel_id));

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
      register_stat(s_normal_read_latency).name("normal_read_latency_{}", m_channel_id);
      register_stat(s_avg_read_latency).name("avg_read_latency_{}", m_channel_id);

      register_stat(s_bandwidth).name("rw_bandwidth_{}", m_channel_id);
      register_stat(s_num_issue_reads).name("num_issue_reads_{}", m_channel_id);
      register_stat(s_num_issue_writes).name("num_issue_writes_{}", m_channel_id);
      register_stat(s_dq_bandwidth).name("dq_bandwidth_{}", m_channel_id);
      register_stat(s_max_bandwidth).name("max_bandwidth_{}", m_channel_id);
      register_stat(s_effective_bandwidth).name("s_effective_bandwidth_{}", m_channel_id);
      register_stat(s_max_effective_bandwidth).name("s_max_effective_bandwidth_{}", m_channel_id);

      num_pseudochannel = m_dram->get_level_size("pseudochannel");  
      num_bankgroup = m_dram->get_level_size("bankgroup");  
      num_bank = m_dram->get_level_size("bank");         

      // Initialize Per Pseudo-Channel Request Buffer and set Max Request Capacity     
      m_read_buffers.resize(num_pseudochannel,ReqBuffer());
      m_write_buffers.resize(num_pseudochannel,ReqBuffer());
      m_priority_buffers.resize(num_pseudochannel,ReqBuffer());
      m_rd_prefetch_buffers.resize(num_pseudochannel,ReqBuffer());
      m_wr_prefetch_buffers.resize(num_pseudochannel,ReqBuffer());

      m_mc_db_rw_modes.resize(num_pseudochannel,DB_RD);
      m_db_dram_rw_modes.resize(num_pseudochannel,DRAM_RD);

      m_to_rd_prefetch_buffers.resize(num_pseudochannel);
      m_to_wr_prefetch_buffers.resize(num_pseudochannel);

      m_channel_stats.resize(num_pseudochannel);
      m_periodic_channel_stats.resize(num_pseudochannel);

      // Counter for Mode Selection 
      m_num_rd_cnts.resize(num_pseudochannel,0);
      m_num_db_rd_cnts.resize(num_pseudochannel,0);
      m_num_dram_rd_cnts.resize(num_pseudochannel,0);
      m_num_wr_cnts.resize(num_pseudochannel,0);
      m_num_db_wr_cnts.resize(num_pseudochannel,0);
      m_num_dram_wr_cnts.resize(num_pseudochannel,0);
      m_num_post_rd_cnts.resize(num_pseudochannel,0);
      m_num_post_wr_cnts.resize(num_pseudochannel,0);
      m_num_ref_cnts.resize(num_pseudochannel,0);
      m_last_ndp_dram_wr.resize(num_pseudochannel,0);
      m_last_host_rd.resize(num_pseudochannel,0);
      
      num_ndp_wr_req_per_pch.resize(num_pseudochannel,0);
      num_ndp_rd_req_per_pch.resize(num_pseudochannel,0);

      m_num_read_req.resize(num_pseudochannel,0);
      m_num_write_req.resize(num_pseudochannel,0);      
      
      m_ndp_dram_wr_timer.resize(num_pseudochannel,0);
      m_dram_rd_timer.resize(num_pseudochannel,0);
      m_mc_rd_timer.resize(num_pseudochannel,0);

      m_dram_ndp_rd_token.resize(num_pseudochannel,0);
      m_enable_pre_rd.resize(num_pseudochannel,false);      

      m_host_access_cnt_per_bank.resize(num_pseudochannel*num_bankgroup*num_bank,0);      
      m_ndp_access_cnt_per_bank.resize(num_pseudochannel*num_bankgroup*num_bank,0);            

      // to be Deprecated 
      // m_is_write_mode_per_pch.resize(num_pseudochannel,false);
      // prefetch_mode_before_ref_per_ch.resize(num_pseudochannel,false);

      for(int i=0; i<num_pseudochannel; i++) {
        m_read_buffers[i].max_size = buf_size;
        m_write_buffers[i].max_size = buf_size;
        m_priority_buffers[i].max_size = (512*3 + 32)/4;
        m_rd_prefetch_buffers[i].max_size = 8;
        m_wr_prefetch_buffers[i].max_size = 8;
        rr_pch_idx.push_back(i);
      }
      m_wr_high_threshold   = (size_t)(buf_size * m_wr_high_watermark);
      m_wr_low_threshold    = (size_t)(buf_size * m_wr_low_watermark);      
      m_max_ndp_read_reqs.resize(num_pseudochannel,(size_t)(buf_size * (m_ndp_read_high_threshold+m_ndp_read_low_threshold)/2));
      m_max_ndp_write_reqs.resize(num_pseudochannel,(size_t)(buf_size * (m_ndp_write_high_threshold+m_ndp_write_low_threshold)/2));            

      high_max_ndp_read_reqs = (size_t)(buf_size * m_ndp_read_high_threshold);
      low_max_ndp_read_reqs = (size_t)(buf_size * m_ndp_read_low_threshold);
      high_max_ndp_write_reqs = (size_t)(buf_size * m_ndp_write_high_threshold);
      low_max_ndp_write_reqs = (size_t)(buf_size * m_ndp_write_low_threshold);                    

      m_logger->info("NDPDRAMCtrl init()");
      m_logger->info(" - Active Buffer Size         : {}",m_active_buffer.max_size);
      m_logger->info(" - Priority Buffer Size       : {}",m_priority_buffers[0].max_size);
      m_logger->info(" - Read Buffer Size           : {}",m_read_buffers[0].max_size);
      m_logger->info(" - Write Buffer Size          : {}",m_write_buffers[0].max_size);
      m_logger->info(" - Read Prefetch Buffer Size  : {}",m_rd_prefetch_buffers[0].max_size);
      m_logger->info(" - Write Prefetch Buffer Size : {}",m_rd_prefetch_buffers[0].max_size);
      m_logger->info(" - MAX NDP Read Request       : {}",m_max_ndp_read_reqs[0]);
      m_logger->info(" - MAX NDP Write Request      : {}",m_max_ndp_write_reqs[0]);

      if(num_pseudochannel != -1) {
        s_num_trans_per_pch.resize(num_pseudochannel, 0);
        s_num_refresh_cc_per_pch.resize(num_pseudochannel, 0);
        // s_num_busy_refresh_cc_per_pch.resize(num_pseudochannel, 0);
        db_prefetch_cnt_per_pch.resize(num_pseudochannel, 0);
        db_prefetch_rd_cnt_per_pch.resize(num_pseudochannel, 0);
        db_prefetch_wr_cnt_per_pch.resize(num_pseudochannel, 0);
        // s_num_max_prefetch_per_pch.resize(num_pseudochannel, 0);
        // s_num_write_mode_per_pch.resize(num_pseudochannel, 0);
        s_narrow_io_busy_clk_per_pch.resize(num_pseudochannel, 0);
        s_wide_io_busy_clk_per_pch.resize(num_pseudochannel, 0);

        io_busy_clk_per_pch.resize(num_pseudochannel, 0);
        is_empty_priority_per_pch.resize(num_pseudochannel, 0);
        for (size_t pch_id = 0; pch_id < num_pseudochannel; pch_id++) {
          register_stat(s_num_trans_per_pch[pch_id]).name("s_num_trans_per_pch_{}_{}", m_channel_id,pch_id);        
          register_stat(s_num_refresh_cc_per_pch[pch_id]).name("s_num_refresh_cc_per_pch_{}_{}", m_channel_id,pch_id);      
          // register_stat(s_num_busy_refresh_cc_per_pch[pch_id]).name("s_num_busy_refresh_cc_per_pch_{}_{}", m_channel_id,pch_id);             
          // register_stat(s_num_max_prefetch_per_pch[pch_id]).name("s_num_max_prefetch_per_pch_{}_{}", m_channel_id,pch_id);   
          // register_stat(s_num_write_mode_per_pch[pch_id]).name("s_num_write_mode_per_pch_{}_{}", m_channel_id,pch_id);    
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

      // current_sch_mode_per_pch.resize(num_pseudochannel,0); // To be deprecated
      // sch_mode_per_pch.resize(num_pseudochannel,0);         // To be deprecated
      // read_mode_cycle_per_pch.resize(num_pseudochannel,0);
      // write_mode_cycle_per_pch.resize(num_pseudochannel,0);
      // prefetch_mode_cycle_per_pch.resize(num_pseudochannel,0);
      // refresh_mode_cycle_per_pch.resize(num_pseudochannel,0);
      // busy_read_mode_cycle_per_pch.resize(num_pseudochannel,0);
      // busy_write_mode_cycle_per_pch.resize(num_pseudochannel,0);
      // busy_prefetch_mode_cycle_per_pch.resize(num_pseudochannel,0);
      // busy_refresh_mode_cycle_per_pch.resize(num_pseudochannel,0);

      ndp_config_reg_resp_per_pch.resize(num_pseudochannel,std::vector<int>(0,0));     

      cmd_cycle_per_pch.resize(num_pseudochannel,0);

      m_avg_active_buffer = 0;
      m_avg_read_buffers.resize(num_pseudochannel,0);
      m_avg_write_buffers.resize(num_pseudochannel,0);
      m_avg_priority_buffers.resize(num_pseudochannel,0);
      m_avg_rd_prefetch_buffers.resize(num_pseudochannel,0);
      m_avg_wr_prefetch_buffers.resize(num_pseudochannel,0);

      m_cmds.ACT = m_dram->m_commands("ACT");
      m_cmds.P_ACT = m_dram->m_commands("P_ACT");
      m_cmds.PRE = m_dram->m_commands("PRE");
      m_cmds.PREA = m_dram->m_commands("PREA");
      m_cmds.PREsb = m_dram->m_commands("PREsb");
      m_cmds.P_PRE = m_dram->m_commands("P_PRE");
      m_cmds.RD = m_dram->m_commands("RD");
      m_cmds.WR = m_dram->m_commands("WR");
      m_cmds.RDA = m_dram->m_commands("RDA");
      m_cmds.WRA = m_dram->m_commands("WRA");
      m_cmds.PRE_RD = m_dram->m_commands("PRE_RD");
      m_cmds.PRE_WR = m_dram->m_commands("PRE_WR");
      m_cmds.POST_RD = m_dram->m_commands("POST_RD");
      m_cmds.POST_WR = m_dram->m_commands("POST_WR");
      m_cmds.PRE_RDA = m_dram->m_commands("PRE_RDA");
      m_cmds.POST_WRA = m_dram->m_commands("POST_WRA");
      m_cmds.NDP_DRAM_RD = m_dram->m_commands("NDP_DRAM_RD");
      m_cmds.NDP_DRAM_WR = m_dram->m_commands("NDP_DRAM_WR");
      m_cmds.NDP_DRAM_RDA = m_dram->m_commands("NDP_DRAM_RDA");
      m_cmds.NDP_DRAM_WRA = m_dram->m_commands("NDP_DRAM_WRA");
      m_cmds.NDP_DB_RD = m_dram->m_commands("NDP_DB_RD");
      m_cmds.NDP_DB_WR = m_dram->m_commands("NDP_DB_WR");
      m_cmds.REFab = m_dram->m_commands("REFab");
      m_cmds.REFsb = m_dram->m_commands("REFsb");
      m_cmds.REFab_end = m_dram->m_commands("REFab_end");
      m_cmds.REFsb_end = m_dram->m_commands("REFsb_end");
    };

    bool send(Request& req) override {
      bool is_success = false;
      bool is_success_forwarding = false;

      if(req.is_ndp_req) {
        if(m_dram->is_ndp_access(req.addr_vec)) {
          // NDP Access (NDP_CONF, ISNT/DAT MEM)
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
          is_success_forwarding = true;
        }
      }

      // Else, enqueue them to corresponding buffer based on request type id
      if(!is_success && can_accept_ndp_request(req,req.addr_vec[psuedo_ch_idx])) {
        req.arrive = m_clk;
        if (req.type_id == Request::Type::Read) {
          // RD, RDA, NDP_DB_RD, NDP_DRAM_RD
          is_success = m_read_buffers[req.addr_vec[psuedo_ch_idx]].enqueue(req);
          if(is_success) {
            std::string msg = std::string(" Insert RD Request to read_buffer ( ") + std::to_string(req.addr) + std::string(" )");
            DEBUG_PRINT(m_clk, "Memory Controller", req.addr_vec[ch_idx], req.addr_vec[psuedo_ch_idx], msg);          
          }
        } else if (req.type_id == Request::Type::Write) {
          // WR, WRA, NDP_DB_WR, NDP_DRAM_WR
          is_success = m_write_buffers[req.addr_vec[psuedo_ch_idx]].enqueue(req);       
          if(is_success) {
            std::string msg = std::string(" Insert WR Request to write_buffer ( ") + std::to_string(req.addr) + std::string(" )");
            DEBUG_PRINT(m_clk, "Memory Controller", req.addr_vec[ch_idx], req.addr_vec[psuedo_ch_idx], msg);                    
          }
        } else {
          throw std::runtime_error("Invalid request type!");
        }
      }
      
      if(is_success && !is_success_forwarding) {
        switch (req.type_id) {
          case Request::Type::Read: {
            s_num_read_reqs++;
            m_num_read_req[req.addr_vec[psuedo_ch_idx]]++;
            // Update Request Type Counter (RD, RDA, NDP_DB_RD, NDP_DRAM_RD)
            if (!req.is_ndp_req)                         m_num_rd_cnts[req.addr_vec[psuedo_ch_idx]]++; // RD, RDA
            else if(m_dram->is_ndp_access(req.addr_vec)) m_num_db_rd_cnts[req.addr_vec[psuedo_ch_idx]]++; // NDP_DB_RD
            else                                         m_num_dram_rd_cnts[req.addr_vec[psuedo_ch_idx]]++; // NDP_DRAM_RD
            break;
          }
          case Request::Type::Write: {
            s_num_write_reqs++;
            m_num_write_req[req.addr_vec[psuedo_ch_idx]]++;
            // Update Request Type Counter (WR, WRA, NDP_DB_WR, NDP_DRAM_WR)
            if (!req.is_ndp_req)                         m_num_wr_cnts[req.addr_vec[psuedo_ch_idx]]++;
            else if(m_dram->is_ndp_access(req.addr_vec)) m_num_db_wr_cnts[req.addr_vec[psuedo_ch_idx]]++;
            else                                         m_num_dram_wr_cnts[req.addr_vec[psuedo_ch_idx]]++;          
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
            num_ndp_total_rd_req++;
          }
          else {
            num_ndp_wr_req++;
            num_ndp_wr_req_per_pch[req.addr_vec[psuedo_ch_idx]]++;
            num_ndp_total_wr_req++;
          }          
        }

        int flat_bank_id = req.addr_vec[bank_idx] + req.addr_vec[bankgroup_idx] * num_bank + req.addr_vec[psuedo_ch_idx] * num_bankgroup*num_bank;
        if(req.is_ndp_req) m_ndp_access_cnt_per_bank[flat_bank_id]++;
        else               m_host_access_cnt_per_bank[flat_bank_id]++;

      }
      // if(is_success) {
        // std::cout<<"[NDP_DRAM_CTRL] Get Request ";
        // m_dram->print_req(req);
      // }

      if(is_success && req.is_ndp_req && req.type_id == Request::Type::Write && !(m_dram->is_ndp_access(req.addr_vec))) {
        m_last_ndp_dram_wr[req.addr_vec[psuedo_ch_idx]] = req.arrive;
      }
      if(is_success && !req.is_ndp_req && req.type_id == Request::Type::Read) {
        m_last_host_rd[req.addr_vec[psuedo_ch_idx]] = req.arrive;
      }
      return is_success;
    };

    bool priority_send(Request& req) override {
      req.final_command = m_dram->m_request_translations(req.type_id);

      bool is_success = false;
      // is_success = m_priority_buffer.enqueue(req);
      is_success = m_priority_buffers[req.addr_vec[psuedo_ch_idx]].enqueue(req);
      if(req.final_command == m_cmds.REFab)
        m_num_ref_cnts[req.addr_vec[psuedo_ch_idx]]++;
      return is_success;
    }

    void tick() override {

      m_clk++;

      for(int pch_id=0;pch_id<(num_pseudochannel);pch_id++) {
        if(m_db_dram_rw_modes[pch_id] == DRAM_NDP_WR) {
          m_ndp_dram_wr_timer[pch_id]++;
        } else {
          m_ndp_dram_wr_timer[pch_id] = 0;
        }

        if(m_db_dram_rw_modes[pch_id] == DRAM_RD) {
          m_dram_rd_timer[pch_id]++;
        } else {
          m_dram_rd_timer[pch_id] = 0;
        }

        if(m_mc_db_rw_modes[pch_id] == DB_RD) {
          m_dram_rd_timer[pch_id]++;
        } else {
          m_dram_rd_timer[pch_id] = 0;
        }        
      }

      for(int pch_id=0;pch_id<(num_pseudochannel);pch_id++) {                  
        // Move to m_rd_prefetch_buffers
        if(m_to_rd_prefetch_buffers[pch_id].size() != 0) {
          for(int i=0;i<m_to_rd_prefetch_buffers[pch_id].size();i++) {
            m_to_rd_prefetch_buffers[pch_id][i].second-=1;
          }
          if(m_to_rd_prefetch_buffers[pch_id][0].second == 0) {
            bool is_success = m_rd_prefetch_buffers[pch_id].enqueue(m_to_rd_prefetch_buffers[pch_id][0].first);
            if(!is_success) {
              throw std::runtime_error("Fail to enque to m_rd_prefetch_buffers");
            }    
            m_to_rd_prefetch_buffers[pch_id].erase(m_to_rd_prefetch_buffers[pch_id].begin());
          }
        }
        // Move to m_wr_prefetch_buffers
        if(m_to_wr_prefetch_buffers[pch_id].size() != 0) {
          for(int i=0;i<m_to_wr_prefetch_buffers[pch_id].size();i++) {
            m_to_wr_prefetch_buffers[pch_id][i].second-=1;
          }
          if(m_to_wr_prefetch_buffers[pch_id][0].second == 0) {
            bool is_success = m_wr_prefetch_buffers[pch_id].enqueue(m_to_wr_prefetch_buffers[pch_id][0].first);
            if(!is_success) {
              throw std::runtime_error("Fail to enque to m_rd_prefetch_buffers");
            }    
            m_to_wr_prefetch_buffers[pch_id].erase(m_to_wr_prefetch_buffers[pch_id].begin());
          }
        }        
      }

      // Update Each Row Cap 

      for(int pch_id=0;pch_id<(num_pseudochannel);pch_id++) {
        for(int bg_id=0;bg_id<num_bankgroup;bg_id++) {
          for(int bk_id=0;bk_id<num_bank;bk_id++) {
            int flat_bank_id = bk_id + bg_id * num_bank + pch_id * num_bankgroup*num_bank;
            int host_access = m_host_access_cnt_per_bank[flat_bank_id];
            int ndp_access = m_ndp_access_cnt_per_bank[flat_bank_id];
            // Max; Read Buffer + Write Buffer + Active Buffer + WR_PRE-BUFFER --> 74 
            if(host_access < 0 || host_access > 75) {
              std::string msg = std::string(" Host Access Count Error (") + std::to_string(host_access) + std::string(")");
              throw std::runtime_error(msg);
            }
            // MAX NDP Access 
            if(ndp_access < 0 || ndp_access > 66) {
              std::string msg = std::string(" NDP Access Count Error (") + std::to_string(ndp_access) + std::string(")");
              throw std::runtime_error(msg);
            }
            if(host_access == 0 || ndp_access == 0) {
              // Max Cap 
              m_rowpolicy->update_cap(pch_id,bg_id,bk_id,128);
            } else {
              m_rowpolicy->update_cap(pch_id,bg_id,bk_id,16);
            }
          }
        }             
      }

      //Update Max NDP Read/Write Requests
      if(m_clk%32 == 0) {
        for(int pch_id=0;pch_id<(num_pseudochannel);pch_id++) {
          int rd_buf_headroom = buf_size - m_read_buffers[pch_id].size();          
          int ndp_rd_headroom = m_max_ndp_read_reqs[pch_id] - (m_num_db_rd_cnts[pch_id] + m_num_dram_rd_cnts[pch_id]);

          if(rd_buf_headroom >=4 && ndp_rd_headroom <= 1 && high_max_ndp_read_reqs > m_max_ndp_read_reqs[pch_id]) {
            m_max_ndp_read_reqs[pch_id]+=4;
          } else if(rd_buf_headroom < 4 && ndp_rd_headroom > 1 && low_max_ndp_read_reqs < m_max_ndp_read_reqs[pch_id]) {
            m_max_ndp_read_reqs[pch_id]-=4;
          } 

          int wr_buf_headroom = buf_size - m_write_buffers[pch_id].size();          
          int ndp_wr_headroom = m_max_ndp_write_reqs[pch_id] - (m_num_db_wr_cnts[pch_id] + m_num_dram_wr_cnts[pch_id]);

          if(wr_buf_headroom >=4 && ndp_wr_headroom <= 1 && high_max_ndp_write_reqs > m_max_ndp_write_reqs[pch_id]) {
            m_max_ndp_write_reqs[pch_id]+=4;
          } else if(wr_buf_headroom < 4 && ndp_wr_headroom > 1 && low_max_ndp_write_reqs < m_max_ndp_write_reqs[pch_id]) {
            m_max_ndp_write_reqs[pch_id]-=4;
          }         
        }
      }   
      m_avg_active_buffer+=m_active_buffer.size();
      for(int pch_id=0;pch_id<(num_pseudochannel);pch_id++) {
        m_avg_read_buffers[pch_id]+=m_read_buffers[pch_id].size();
        m_avg_write_buffers[pch_id]+=m_write_buffers[pch_id].size();
        m_avg_priority_buffers[pch_id]+=m_priority_buffers[pch_id].size();
        m_avg_rd_prefetch_buffers[pch_id]+=m_rd_prefetch_buffers[pch_id].size();
        m_avg_wr_prefetch_buffers[pch_id]+=m_wr_prefetch_buffers[pch_id].size();
      }      
      uint64_t print_interval = 100000;
      // uint64_t print_interval = 1024;

      // if(m_clk%print_interval == 0) {
      //   std::cout<<"Active Buf: "<<m_active_buffer.size()<<std::endl;
      //   std::cout<<"RD_BUF"<<std::endl;
      //   for(int pch_id=0;pch_id<(num_pseudochannel);pch_id++) {
      //     std::cout<<"  - PCH["<<pch_id<<"] : "<<m_read_buffers[pch_id].size()<<std::endl;
      //   }        
      //   std::cout<<"WR_BUF"<<std::endl;
      //   for(int pch_id=0;pch_id<(num_pseudochannel);pch_id++) {
      //     std::cout<<"  - PCH["<<pch_id<<"] : "<<m_write_buffers[pch_id].size()<<std::endl;
      //   }        
      //   std::cout<<"PRE_RD_BUF"<<std::endl;
      //   for(int pch_id=0;pch_id<(num_pseudochannel);pch_id++) {
      //     std::cout<<"  - PCH["<<pch_id<<"] : "<<m_rd_prefetch_buffers[pch_id].size()<<std::endl;
      //   }
      //   std::cout<<"PRE_WR_BUF"<<std::endl;
      //   for(int pch_id=0;pch_id<(num_pseudochannel);pch_id++) {
      //     std::cout<<"  - PCH["<<pch_id<<"] : "<<m_wr_prefetch_buffers[pch_id].size()<<std::endl;
      //   }              
      // }

      if(m_clk%print_interval == 0) {
          #ifdef PRINT_DEBUG    
          std::cout<<"Active Buf: "<<m_avg_active_buffer/print_interval<<std::endl;
          std::cout<<"RD_BUF"<<std::endl;
          for(int pch_id=0;pch_id<(num_pseudochannel);pch_id++) {
            std::cout<<"  - PCH["<<pch_id<<"] : "<<m_avg_read_buffers[pch_id]/print_interval<<std::endl;
          }
          std::cout<<"WR_BUF"<<std::endl;
          for(int pch_id=0;pch_id<(num_pseudochannel);pch_id++) {
            std::cout<<"  - PCH["<<pch_id<<"] : "<<m_avg_write_buffers[pch_id]/print_interval<<std::endl;
          }       
          std::cout<<"PRE_RD_BUF"<<std::endl;
          for(int pch_id=0;pch_id<(num_pseudochannel);pch_id++) {
            std::cout<<"  - PCH["<<pch_id<<"] : "<<m_avg_rd_prefetch_buffers[pch_id]/print_interval<<std::endl;
          }
          std::cout<<"PRE_WR_BUF"<<std::endl;
          for(int pch_id=0;pch_id<(num_pseudochannel);pch_id++) {
            std::cout<<"  - PCH["<<pch_id<<"] : "<<m_avg_wr_prefetch_buffers[pch_id]/print_interval<<std::endl;
          }       
          std::cout<<"PRI_BUF"<<std::endl;
          for(int pch_id=0;pch_id<(num_pseudochannel);pch_id++) {
            std::cout<<"  - PCH["<<pch_id<<"] : "<<m_avg_priority_buffers[pch_id]/print_interval<<std::endl;
          }    
          #endif          
          m_avg_active_buffer = 0;
          for(int pch_id=0;pch_id<(num_pseudochannel);pch_id++) {
            m_avg_read_buffers[pch_id]       = 0;
            m_avg_write_buffers[pch_id]      = 0;
            m_avg_priority_buffers[pch_id]   = 0;
            m_avg_rd_prefetch_buffers[pch_id]= 0;
            m_avg_wr_prefetch_buffers[pch_id]= 0;
          }         
          
          #ifdef PRINT_DEBUG    
          for(int i=0;i<num_pseudochannel;i++) {
            print_channel_periodic_stats(i);
          }
          #endif 
          // Reset Mode Info
          for(int i=0;i<num_pseudochannel;i++) {
            for (int mode = 0; mode < 3; mode++) {
              m_periodic_channel_stats[i].mc_db_mode_cycles[mode] = 0;
            }
            for (int mode = 0; mode < 4; mode++) {
              m_periodic_channel_stats[i].db_dram_mode_cycles[mode] = 0;
            }            
          }

          u_int32_t num_cmd_all_pch = 0;
          for(int pch_id=0;pch_id<(num_pseudochannel);pch_id++) {
            num_cmd_all_pch+=cmd_cycle_per_pch[pch_id];
          }          
          m_his_num_cmd.push_back(num_cmd_all_pch - m_prev_num_cmd);
          m_his_num_rd.push_back(s_num_rd - m_prev_num_rd);
          m_his_num_wr.push_back(s_num_wr - m_prev_num_wr);
          m_his_num_pre_wr.push_back(s_num_pre_wr - m_prev_num_pre_wr);
          m_his_num_post_wr.push_back(s_num_post_wr - m_prev_num_post_wr);
          m_his_num_pre_rd.push_back(s_num_pre_rd - m_prev_num_pre_rd);
          m_his_num_post_rd.push_back(s_num_post_rd - m_prev_num_post_rd);
          m_his_num_ndp_dram_rd.push_back(s_num_ndp_dram_rd - m_prev_num_ndp_dram_rd);
          m_his_num_ndp_dram_wr.push_back(s_num_ndp_dram_wr - m_prev_num_ndp_dram_wr);
          m_his_num_ndp_db_rd.push_back(s_num_ndp_db_rd - m_prev_num_ndp_db_rd);
          m_his_num_ndp_db_wr.push_back(s_num_ndp_db_wr - m_prev_num_ndp_db_wr);

          m_prev_num_cmd = num_cmd_all_pch;
          m_prev_num_rd = s_num_rd;
          m_prev_num_wr = s_num_wr;
          m_prev_num_pre_wr = s_num_pre_wr;
          m_prev_num_post_wr = s_num_post_wr;
          m_prev_num_pre_rd = s_num_pre_rd;
          m_prev_num_post_rd = s_num_post_rd;
          m_prev_num_ndp_dram_rd = s_num_ndp_dram_rd;
          m_prev_num_ndp_dram_wr = s_num_ndp_dram_wr;
          m_prev_num_ndp_db_rd = s_num_ndp_db_rd;
          m_prev_num_ndp_db_wr = s_num_ndp_db_wr;
         
        }
      // Update statistics
      s_queue_len += pending.size();
      for(int i=0;i<num_pseudochannel;i++) {
        s_read_queue_len += m_read_buffers[i].size();
        s_write_queue_len += m_write_buffers[i].size();
        s_read_prefetch_queue_len += m_rd_prefetch_buffers[i].size();
        s_write_prefetch_queue_len += m_wr_prefetch_buffers[i].size();
        s_priority_queue_len += m_priority_buffers[i].size();
      }

      for(int i=0;i<num_pseudochannel;i++) {
        m_channel_stats[i].mc_db_mode_cycles[m_mc_db_rw_modes[i]]++;
        m_channel_stats[i].db_dram_mode_cycles[m_db_dram_rw_modes[i]]++;
        m_periodic_channel_stats[i].mc_db_mode_cycles[m_mc_db_rw_modes[i]]++;
        m_periodic_channel_stats[i].db_dram_mode_cycles[m_db_dram_rw_modes[i]]++;
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
          
        int req_pch_idx = req_it->addr_vec[psuedo_ch_idx];

        // Update prefetch rd/wr counter for PRE_RD/WR
        if(req_it->is_db_cmd) {
          if(req_it->final_command == m_cmds.PRE_RD || req_it->final_command == m_cmds.PRE_RDA) {
            s_num_pre_rd++;
            db_prefetch_cnt_per_pch[req_it->addr_vec[psuedo_ch_idx]]++;
            db_prefetch_rd_cnt_per_pch[req_it->addr_vec[psuedo_ch_idx]]++; 
            #ifdef PRINT_DB_CNT
            std::cout<<"PRE_RD ["<<req_it->addr_vec[0]<<"]["<<req_it->addr_vec[1]<<"] |";
            std::cout<<" db cnt "<<db_prefetch_cnt_per_pch[req_it->addr_vec[psuedo_ch_idx]];
            std::cout<<" db rd cnt "<<db_prefetch_rd_cnt_per_pch[req_it->addr_vec[psuedo_ch_idx]];
            std::cout<<" db wr cnt "<<db_prefetch_wr_cnt_per_pch[req_it->addr_vec[psuedo_ch_idx]]<<std::endl;            
            #endif                     
          } else {
            s_num_pre_wr++;
            db_prefetch_cnt_per_pch[req_it->addr_vec[psuedo_ch_idx]]++;
            db_prefetch_wr_cnt_per_pch[req_it->addr_vec[psuedo_ch_idx]]++;                
            #ifdef PRINT_DB_CNT
            std::cout<<"PRE_WR ["<<req_it->addr_vec[0]<<"]["<<req_it->addr_vec[1]<<"] |";
            std::cout<<" db cnt "<<db_prefetch_cnt_per_pch[req_it->addr_vec[psuedo_ch_idx]];
            std::cout<<" db rd cnt "<<db_prefetch_rd_cnt_per_pch[req_it->addr_vec[psuedo_ch_idx]];
            std::cout<<" db wr cnt "<<db_prefetch_wr_cnt_per_pch[req_it->addr_vec[psuedo_ch_idx]]<<std::endl;                       
            #endif             
          }
          req_it->is_db_cmd = false;
        }

        // Update POST_RD
        if(req_it->command  == m_cmds.POST_RD) {
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

        // Update POST_WR 
        if(req_it->command == m_cmds.POST_WR) {
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

        // if((req_it->command == m_cmds.RD || req_it->command == m_cmds.RDA) && 
        //     db_prefetch_rd_cnt_per_pch[req_pch_idx] < m_rd_prefetch_buffers[req_pch_idx].max_size) {
        //   // Convert RD or RDA to PRE_RD when the pseudo channel is high priority mode of prefetch
        //   if(req_it->command == m_cmds.RD) req_it->command = m_cmds.PRE_RD;
        //   else                                            req_it->command = m_cmds.PRE_RDA;\
        //   req_it->is_db_cmd = true;
        // }

        // // Update PRE_WR from Normal WR/WRA
        // if((req_it->command == m_cmds.P_ACT || req_it->command == m_cmds.PRE_WR) &&
        //     (req_it->final_command == m_cmds.WR || req_it->final_command == m_cmds.WRA)) {
        //     // PRE_WR from WRITE_BUFFER --> Update PRE_WR Counter
        //     s_num_pre_wr++;
        //     db_prefetch_cnt_per_pch[req_it->addr_vec[psuedo_ch_idx]]++;
        //     db_prefetch_wr_cnt_per_pch[req_it->addr_vec[psuedo_ch_idx]]++;       
        //     // Change Final Command from WR/WRA to PRE_WR 
        //     req_it->final_command = m_cmds.PRE_WR;      
        //     #ifdef PRINT_DB_CNT
        //     std::cout<<"PRE_WR ["<<req_it->addr_vec[0]<<"]["<<req_it->addr_vec[1]<<"] |";
        //     std::cout<<" db cnt "<<db_prefetch_cnt_per_pch[req_it->addr_vec[psuedo_ch_idx]];
        //     std::cout<<" db rd cnt "<<db_prefetch_rd_cnt_per_pch[req_it->addr_vec[psuedo_ch_idx]];
        //     std::cout<<" db wr cnt "<<db_prefetch_wr_cnt_per_pch[req_it->addr_vec[psuedo_ch_idx]]<<std::endl;                       
        //     #endif                   
        // }

        // // Update POST_WR 
        // if(req_it->command == m_cmds.POST_WR) {
        //   // Issued DDR commands related with PRE_WR
        //   s_num_post_wr++;
        //   db_prefetch_cnt_per_pch[req_it->addr_vec[psuedo_ch_idx]]--;
        //   db_prefetch_wr_cnt_per_pch[req_it->addr_vec[psuedo_ch_idx]]--;
        //   #ifdef PRINT_DB_CNT
        //   std::cout<<"POST_WR ["<<req_it->addr_vec[0]<<"]["<<req_it->addr_vec[1]<<"] |";
        //   std::cout<<" db cnt "<<db_prefetch_cnt_per_pch[req_it->addr_vec[psuedo_ch_idx]];
        //   std::cout<<" db rd cnt "<<db_prefetch_rd_cnt_per_pch[req_it->addr_vec[psuedo_ch_idx]];
        //   std::cout<<" db wr cnt "<<db_prefetch_wr_cnt_per_pch[req_it->addr_vec[psuedo_ch_idx]]<<std::endl;                
        //   #endif
        // }

        // // Update PRE_RD
        // if((req_it->command == m_cmds.ACT  || req_it->command == m_cmds.RD ||
        //     req_it->command  == m_cmds.RDA || req_it->command  == m_cmds.PRE_RD ||
        //     req_it->command  == m_cmds.PRE_RDA) && (req_it->final_command == m_cmds.RD || 
        //     req_it->final_command == m_cmds.RDA) && req_it->is_db_cmd) {                
        //   // PRE_RD from READ_BUFFER --> update PRD_RD Counter
        //   s_num_pre_rd++;
        //   db_prefetch_cnt_per_pch[req_it->addr_vec[psuedo_ch_idx]]++;
        //   db_prefetch_rd_cnt_per_pch[req_it->addr_vec[psuedo_ch_idx]]++;
        //   // Change Final Command from RD/RDA to PRE_RD/PRE_RDA 
        //   if(req_it->final_command == m_cmds.RD) req_it->final_command = m_cmds.PRE_RD;
        //   else                                                  req_it->final_command = m_cmds.PRE_RDA;
        //   #ifdef PRINT_DB_CNT
        //   std::cout<<"PRE_RD ["<<req_it->addr_vec[0]<<"]["<<req_it->addr_vec[1]<<"] |";
        //   std::cout<<" db cnt "<<db_prefetch_cnt_per_pch[req_it->addr_vec[psuedo_ch_idx]];
        //   std::cout<<" db rd cnt "<<db_prefetch_rd_cnt_per_pch[req_it->addr_vec[psuedo_ch_idx]];
        //   std::cout<<" db wr cnt "<<db_prefetch_wr_cnt_per_pch[req_it->addr_vec[psuedo_ch_idx]]<<std::endl;            
        //   #endif
        // }
        
        // // Update POST_RD
        // if(req_it->command  == m_cmds.POST_RD) {
        //   s_num_post_rd++;
        //   db_prefetch_cnt_per_pch[req_it->addr_vec[psuedo_ch_idx]]--;
        //   db_prefetch_rd_cnt_per_pch[req_it->addr_vec[psuedo_ch_idx]]--;
        //   #ifdef PRINT_DB_CNT
        //   std::cout<<"POST_RD ["<<req_it->addr_vec[0]<<"]["<<req_it->addr_vec[1]<<"] |";
        //   std::cout<<" db cnt "<<db_prefetch_cnt_per_pch[req_it->addr_vec[psuedo_ch_idx]];
        //   std::cout<<" db rd cnt "<<db_prefetch_rd_cnt_per_pch[req_it->addr_vec[psuedo_ch_idx]];
        //   std::cout<<" db wr cnt "<<db_prefetch_wr_cnt_per_pch[req_it->addr_vec[psuedo_ch_idx]]<<std::endl;
        //   #endif
        // }            
          

        // Update DRAM Prefetch Counter Value
        m_dram->set_db_fetch_per_pch(req_it->addr_vec,db_prefetch_cnt_per_pch[req_it->addr_vec[psuedo_ch_idx]],
                                                      db_prefetch_rd_cnt_per_pch[req_it->addr_vec[psuedo_ch_idx]],
                                                      db_prefetch_wr_cnt_per_pch[req_it->addr_vec[psuedo_ch_idx]]);
                            
        if(false) {
          if(req_it->command == m_cmds.PRE_WR)  std::cout<<"["<<m_clk<<"] ISSUE PRE_WR CH["<<req_it->addr_vec[0]<<"]PCH["<<req_it->addr_vec[1]<<"] / "<<s_num_pre_wr  << "/" <<s_num_post_wr
          <<" db cnt : "<<db_prefetch_cnt_per_pch[req_it->addr_vec[psuedo_ch_idx]]<<" / "<<m_dram->get_db_fetch_per_pch(req_it->addr_vec)<<std::endl;
          if(req_it->command == m_cmds.POST_WR) std::cout<<"["<<m_clk<<"] ISSUE POST_WR CH["<<req_it->addr_vec[0]<<"]PCH["<<req_it->addr_vec[1]<<"] / "<<s_num_pre_wr  << "/" <<s_num_post_wr
          <<" db cnt : "<<db_prefetch_cnt_per_pch[req_it->addr_vec[psuedo_ch_idx]]<<" / "<<m_dram->get_db_fetch_per_pch(req_it->addr_vec)<<std::endl;
          if(req_it->command == m_cmds.PRE_RD) std::cout<<"["<<m_clk<<"] ISSUE PRE_RD CH["<<req_it->addr_vec[0]<<"]PCH["<<req_it->addr_vec[1]<<"] / "<<s_num_pre_rd  << "/" <<s_num_post_rd
          <<" db cnt : "<<db_prefetch_cnt_per_pch[req_it->addr_vec[psuedo_ch_idx]]<<" / "<<m_dram->get_db_fetch_per_pch(req_it->addr_vec)<<std::endl;              
          if(req_it->command == m_cmds.PRE_RDA) std::cout<<"["<<m_clk<<"] ISSUE PRE_RDA CH["<<req_it->addr_vec[0]<<"]PCH["<<req_it->addr_vec[1]<<"] / "<<s_num_pre_rd  << "/" <<s_num_post_rd
          <<" db cnt : "<<db_prefetch_cnt_per_pch[req_it->addr_vec[psuedo_ch_idx]]<<" / "<<m_dram->get_db_fetch_per_pch(req_it->addr_vec)<<std::endl;                            
          if(req_it->command == m_cmds.POST_RD) std::cout<<"["<<m_clk<<"] ISSUE POST_RD CH["<<req_it->addr_vec[0]<<"]PCH["<<req_it->addr_vec[1]<<"] / "<<s_num_pre_rd  << "/" <<s_num_post_rd
          <<" db cnt : "<<db_prefetch_cnt_per_pch[req_it->addr_vec[psuedo_ch_idx]]<<" / "<<m_dram->get_db_fetch_per_pch(req_it->addr_vec)<<std::endl;                                          
        }
        // Exception 
        if(db_prefetch_cnt_per_pch[req_it->addr_vec[psuedo_ch_idx]] > 32 || db_prefetch_cnt_per_pch[req_it->addr_vec[psuedo_ch_idx]] < 0) {
          std::cout<<"[ndpDRAMCtrl]";
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
        
        /*
          ======================= Update Stats =====================
        */

        if(req_it->command == m_cmds.P_ACT     ||
           req_it->command == m_cmds.PRE       ||
           req_it->command == m_cmds.PREA      ||
           req_it->command == m_cmds.P_PRE     ||
           req_it->command == m_cmds.REFab     ||
           req_it->command == m_cmds.REFsb     ||
           req_it->command == m_cmds.REFab_end ||
           req_it->command == m_cmds.REFsb_end) {
          // Single Cycle Command
          cmd_io_cc+=1;
          cmd_cycle_per_pch[req_it->addr_vec[psuedo_ch_idx]]+=1;
        } else {
          // Two Cycle Command
          cmd_io_cc+=2;
          cmd_cycle_per_pch[req_it->addr_vec[psuedo_ch_idx]]+=2;
        }

        // Normal Request
        if(req_it->command == m_cmds.RD || req_it->command == m_cmds.RDA || 
           req_it->command == m_cmds.WR || req_it->command == m_cmds.WRA) {
          s_narrow_io_busy_clk_per_pch[req_it->addr_vec[1]]+=(32);
          s_wide_io_busy_clk_per_pch[req_it->addr_vec[1]]+=(8);                            
        }

        if(req_it->command == m_cmds.POST_RD || req_it->command == m_cmds.PRE_WR || 
           req_it->command == m_cmds.NDP_DB_RD || req_it->command == m_cmds.NDP_DB_WR) {
          s_narrow_io_busy_clk_per_pch[req_it->addr_vec[1]]+=(32);
        }

        // NDP and D2PA Request
        if(req_it->command == m_cmds.PRE_RD || req_it->command == m_cmds.PRE_RDA          || 
           req_it->command == m_cmds.POST_WR || req_it->command == m_cmds.POST_WRA        ||
           req_it->command == m_cmds.NDP_DRAM_RD || req_it->command == m_cmds.NDP_DRAM_WR ||
           req_it->command == m_cmds.NDP_DRAM_RDA || req_it->command == m_cmds.NDP_DRAM_WRA) {
            s_wide_io_busy_clk_per_pch[req_it->addr_vec[1]]+=(8);
        }

        if(req_it->command == m_cmds.ACT)                                                                  s_num_act++;
        if(req_it->command == m_cmds.PRE || req_it->command == m_cmds.PREA)                 s_num_pre++;
        if(req_it->command == m_cmds.P_ACT)                                                                s_num_p_act++;
        if(req_it->command == m_cmds.P_PRE)                                                                s_num_p_pre++;
        if(req_it->command == m_cmds.RD || req_it->command == m_cmds.RDA)                   s_num_rd++;
        if(req_it->command == m_cmds.WR || req_it->command == m_cmds.WRA)                   s_num_wr++;
        if(req_it->command == m_cmds.NDP_DRAM_RD || req_it->command == m_cmds.NDP_DRAM_RDA) s_num_ndp_dram_rd++;
        if(req_it->command == m_cmds.NDP_DRAM_WR || req_it->command == m_cmds.NDP_DRAM_WRA) s_num_ndp_dram_wr++;
        if(req_it->command == m_cmds.NDP_DB_RD)                                                            s_num_ndp_db_rd++;
        if(req_it->command == m_cmds.NDP_DB_WR)                                                            s_num_ndp_db_wr++;              

        if(req_it->command == m_cmds.RD || req_it->command == m_cmds.RDA || req_it->command == m_cmds.POST_RD) s_num_issue_reads++;
        if(req_it->command == m_cmds.WR || req_it->command == m_cmds.WRA || req_it->command == m_cmds.PRE_WR)  s_num_issue_writes++;
        if((req_it->command == m_cmds.RD || req_it->command == m_cmds.RDA || req_it->command == m_cmds.POST_RD) || 
            (req_it->command == m_cmds.WR || req_it->command == m_cmds.WRA || req_it->command == m_cmds.PRE_WR)) {
            s_num_trans_per_pch[req_it->addr_vec[psuedo_ch_idx]]++;
        }

        if(req_it->command == m_cmds.REFab) {
          int nRFC_latency = m_dram->m_timing_vals("nRFC1");
          int pch_addr = req_it->addr_vec[psuedo_ch_idx];
          s_num_refresh_cc_per_pch[pch_addr]+=nRFC_latency;
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
        if ((req_it->is_ndp_req && req_it->final_command == req_it->command)) {
          if(req_it->m_payload.size()!=0) {
            m_dram->issue_ndp_command(req_it->command, req_it->addr_vec, req_it->ndp_id, req_it->m_payload);
          } else {
            m_dram->issue_ndp_command(req_it->command, req_it->addr_vec, req_it->ndp_id);
          }
        }
        /*
        if(false) {

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
        if(false) {
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
            m_dram->print_req(*req_it);
        }
        */
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
          }
          // Update Counter for Mode Selection
          if(req_it->command == m_cmds.RD     || req_it->command == m_cmds.RDA || 
             req_it->command == m_cmds.PRE_RD || req_it->command == m_cmds.PRE_RDA) {
            if(m_num_rd_cnts[req_it->addr_vec[psuedo_ch_idx]] == 0) throw std::runtime_error("Underflow for RD Counter");
            m_num_rd_cnts[req_it->addr_vec[psuedo_ch_idx]]--;
          } else if(req_it->command == m_cmds.NDP_DB_RD) {
            if(m_num_db_rd_cnts[req_it->addr_vec[psuedo_ch_idx]]  == 0) throw std::runtime_error("Underflow for DB_RD Counter");
            m_num_db_rd_cnts[req_it->addr_vec[psuedo_ch_idx]]--;
          } else if(req_it->command == m_cmds.NDP_DRAM_RD || req_it->command == m_cmds.NDP_DRAM_RDA) {
            if(m_num_dram_rd_cnts[req_it->addr_vec[psuedo_ch_idx]]  == 0) throw std::runtime_error("Underflow for DRAM_RD Counter");
            m_num_dram_rd_cnts[req_it->addr_vec[psuedo_ch_idx]]--;
          } else if(req_it->command == m_cmds.WR     || req_it->command == m_cmds.WRA || req_it->command == m_cmds.PRE_WR) {
            if(m_num_wr_cnts[req_it->addr_vec[psuedo_ch_idx]]  == 0) throw std::runtime_error("Underflow for WR Counter");
            m_num_wr_cnts[req_it->addr_vec[psuedo_ch_idx]]--;
          } else if(req_it->command == m_cmds.NDP_DB_WR) {
            if(m_num_db_wr_cnts[req_it->addr_vec[psuedo_ch_idx]]  == 0) throw std::runtime_error("Underflow for DB_WR Counter");
            m_num_db_wr_cnts[req_it->addr_vec[psuedo_ch_idx]]--;
          } else if(req_it->command == m_cmds.NDP_DRAM_WR || req_it->command == m_cmds.NDP_DRAM_WRA) {
            if(m_num_dram_wr_cnts[req_it->addr_vec[psuedo_ch_idx]]  == 0) throw std::runtime_error("Underflow for DRAM_WR Counter");
            m_num_dram_wr_cnts[req_it->addr_vec[psuedo_ch_idx]]--;
          } else if(req_it->command == m_cmds.POST_RD) {
            if(m_num_post_rd_cnts[req_it->addr_vec[psuedo_ch_idx]]  == 0) throw std::runtime_error("Underflow for POST_RD Counter");
            m_num_post_rd_cnts[req_it->addr_vec[psuedo_ch_idx]]--;
          } else if(req_it->command == m_cmds.POST_WR || req_it->command == m_cmds.POST_WRA) {
            if(m_num_post_wr_cnts[req_it->addr_vec[psuedo_ch_idx]]  == 0) throw std::runtime_error("Underflow for POST_WR Counter");
            m_num_post_wr_cnts[req_it->addr_vec[psuedo_ch_idx]]--;
          } else if(req_it->command == m_cmds.REFab) {
            if(m_num_ref_cnts[req_it->addr_vec[psuedo_ch_idx]]  == 0) throw std::runtime_error("Underflow for REF Counter");            
            m_num_ref_cnts[req_it->addr_vec[psuedo_ch_idx]]--;
          }

          if(req_it->command == m_cmds.NDP_DRAM_RD || req_it->command == m_cmds.NDP_DRAM_RDA) {
            if(m_dram_ndp_rd_token[req_it->addr_vec[psuedo_ch_idx]] < 128)
              m_dram_ndp_rd_token[req_it->addr_vec[psuedo_ch_idx]]++;
          }        
          
          if(req_it->command == m_cmds.PRE_RD || req_it->command == m_cmds.PRE_RDA) {
            if(m_dram_ndp_rd_token[req_it->addr_vec[psuedo_ch_idx]] >= 16)
              m_dram_ndp_rd_token[req_it->addr_vec[psuedo_ch_idx]]-=16;            
          }

          int flat_bank_id = req_it->addr_vec[bank_idx] + req_it->addr_vec[bankgroup_idx] * num_bank + req_it->addr_vec[psuedo_ch_idx] * num_bankgroup*num_bank;
          if(req_it->command == m_cmds.RD     || req_it->command == m_cmds.RDA || 
             req_it->command == m_cmds.PRE_RD || req_it->command == m_cmds.PRE_RDA ||
             req_it->command == m_cmds.WR     || req_it->command == m_cmds.WRA || 
             req_it->command == m_cmds.POST_WR || req_it->command == m_cmds.POST_WRA) {
             m_host_access_cnt_per_bank[flat_bank_id]--;             
            }
            
            if(req_it->command == m_cmds.NDP_DRAM_RD || req_it->command == m_cmds.NDP_DRAM_RDA ||
            req_it->command == m_cmds.NDP_DB_WR   || req_it->command == m_cmds.NDP_DB_RD ||
            req_it->command == m_cmds.NDP_DRAM_WR || req_it->command == m_cmds.NDP_DRAM_WRA) {
              m_ndp_access_cnt_per_bank[flat_bank_id]--;         
          }

          // Move issued Read Request to pending buffer
          if (req_it->type_id == Request::Type::Read) {
            if(req_it->command == m_cmds.RD || req_it->command == m_cmds.RDA || req_it->command == m_cmds.POST_RD) {
              req_it->depart = m_clk + m_dram->m_read_latency;
              pending.push_back(*req_it);  
            }
          } else if (req_it->type_id == Request::Type::Write) {
            // TODO: Add code to update statistics
          }
          
          if(req_it->command == m_cmds.PRE_WR) {
            // Generate POST_WR and Enqueue to Prefetched Buffer
            Request new_req = Request(req_it->addr_vec, Request::Type::Write);
            new_req.addr          = req_it->addr;
            new_req.arrive        = req_it->arrive;
            new_req.final_command = m_cmds.POST_WR;
            m_num_post_wr_cnts[req_it->addr_vec[psuedo_ch_idx]]++;
            std::string msg = std::string(" Remove Request (PRE_WR) from Buffer ") + std::to_string(buffer->size()) + std::string(")");
            DEBUG_PRINT(m_clk, "Memory Controller", new_req.addr_vec[ch_idx], new_req.addr_vec[psuedo_ch_idx], msg);              
            buffer->remove(req_it);
            // std::cout<<"[NDP_DRAM_CTRL] Generate POST_WR and Enqueue to WR_PREFETCH_BUFFER"<<std::endl;
            m_to_wr_prefetch_buffers[new_req.addr_vec[psuedo_ch_idx]].push_back(std::make_pair(new_req, (m_dram->m_timing_vals("nBL")*4)));            
          } else if(req_it->command == m_cmds.PRE_RD || req_it->command == m_cmds.PRE_RDA) {
            // Generate POST_RD and Enqueue to Prefetched Buffer
            Request new_req = Request(req_it->addr_vec, Request::Type::Read);
            new_req.addr          = req_it->addr;
            new_req.arrive        = req_it->arrive;
            new_req.source_id     = req_it->source_id;
            new_req.callback      = req_it->callback;
            new_req.final_command = m_cmds.POST_RD;
            m_num_post_rd_cnts[req_it->addr_vec[psuedo_ch_idx]]++;
            bool is_success = false;              

            std::string msg = std::string(" Remove Request (PRE_WR) from Buffer (remained ") + std::to_string(buffer->size()) + std::string(" )");
            DEBUG_PRINT(m_clk, "Memory Controller", new_req.addr_vec[ch_idx], new_req.addr_vec[psuedo_ch_idx], msg);              
            buffer->remove(req_it);
            m_to_rd_prefetch_buffers[new_req.addr_vec[psuedo_ch_idx]].push_back(std::make_pair(new_req, (m_dram->m_timing_vals("nBL"))));
            // is_success = m_rd_prefetch_buffers[new_req.addr_vec[psuedo_ch_idx]].enqueue(new_req);
            // if(!is_success) {
            //   throw std::runtime_error("Fail to enque to m_rd_prefetch_buffers");
            // }                    
          } else {
            std::string msg = std::string(" Remove Request from Buffer (remained ") + std::to_string(buffer->size()) + std::string(" )");
            DEBUG_PRINT(m_clk, "Memory Controller", req_it->addr_vec[ch_idx], req_it->addr_vec[psuedo_ch_idx], msg);                        
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

      // If skip POST_RD or PRE_WR because there are D2PA Access 
      if(!(req->final_command != m_cmds.POST_RD || req->final_command != m_cmds.PRE_WR)) {
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
            if((req.command == m_cmds.RD) || (req.command == m_cmds.RDA) ||
               (req.command == m_cmds.POST_RD)) {
              s_normal_read_latency += req.depart - req.arrive;
            }
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
    // Deprecated 
    void set_write_mode() {
    };

    void set_mode_per_pch(int pch_idx) {

      // Set MC-DB RW Mode
      size_t num_rd = m_num_rd_cnts[pch_idx] + m_num_post_rd_cnts[pch_idx];
      size_t num_wr = m_num_wr_cnts[pch_idx];
      size_t elaped_time = m_clk - m_last_host_rd[pch_idx];
      switch (m_mc_db_rw_modes[pch_idx]) {
        case DB_NDP_WR: {
          if(m_num_db_wr_cnts[pch_idx] > 0) {
            m_mc_db_rw_modes[pch_idx] = DB_NDP_WR;
          } else if(m_num_wr_cnts[pch_idx] > m_wr_high_threshold) {
            m_mc_db_rw_modes[pch_idx] = DB_WR;
          } else {
            m_mc_db_rw_modes[pch_idx] = DB_RD;
          }            
          break;
        }
        case DB_WR: {
          if((m_num_wr_cnts[pch_idx] > m_wr_low_threshold && !(elaped_time > ndp_dram_wr_max_age && num_rd > 0)) ||
            (num_rd == 0 && num_wr !=0)) {
            m_mc_db_rw_modes[pch_idx] = DB_WR;
          } else if(m_num_db_wr_cnts[pch_idx] > 0) {
            m_mc_db_rw_modes[pch_idx] = DB_NDP_WR;
          } else {
            m_mc_db_rw_modes[pch_idx] = DB_RD;
          }
          break;
        }
        case DB_RD: {
          if(m_num_db_wr_cnts[pch_idx] > 0) {
            m_mc_db_rw_modes[pch_idx] = DB_NDP_WR;
          } else if((m_num_wr_cnts[pch_idx] > m_wr_high_threshold && !(m_mc_rd_timer[pch_idx] < ndp_dram_wr_max_age && num_rd > 0)) ||
                    (num_rd == 0 && num_wr !=0)) {
            m_mc_db_rw_modes[pch_idx] = DB_WR;
          } else {
            m_mc_db_rw_modes[pch_idx] = DB_RD;
          }             
          break;
        }          
        default: {
          throw std::runtime_error("Invalid MC-DB RW Mode!");
          break;
        }
      }   

      // Set DB RW Mode
      size_t num_dram_wr = m_num_wr_cnts[pch_idx] + m_num_dram_wr_cnts[pch_idx] + m_num_post_wr_cnts[pch_idx];
      size_t num_dram_rd = m_num_rd_cnts[pch_idx] + m_num_dram_rd_cnts[pch_idx]; 
      elaped_time = m_clk - m_last_ndp_dram_wr[pch_idx];
      if(m_dram_ndp_rd_token[pch_idx]>=16 || m_num_dram_rd_cnts[pch_idx] == 0)
        m_enable_pre_rd[pch_idx] = true;
      else 
        m_enable_pre_rd[pch_idx] = false;
      
      switch (m_db_dram_rw_modes[pch_idx]) {
        case DRAM_REF: {          
          if(m_num_ref_cnts[pch_idx] > 0) {
            m_db_dram_rw_modes[pch_idx] = DRAM_REF; 
          } else if((m_num_dram_wr_cnts[pch_idx] > 0 && elaped_time > ndp_dram_wr_max_age)) {
            m_db_dram_rw_modes[pch_idx] = DRAM_NDP_WR;
          } else if(num_dram_wr > m_wr_high_threshold || (num_dram_wr > 0 && num_dram_rd == 0)) {
            m_db_dram_rw_modes[pch_idx] = DRAM_WR;
          } else {
            m_db_dram_rw_modes[pch_idx] = DRAM_RD;
          }
          break;
        }
        case DRAM_WR: {
          if(m_num_ref_cnts[pch_idx] > 0) {
            m_db_dram_rw_modes[pch_idx] = DRAM_REF;
          } else if(num_dram_wr > m_wr_low_threshold || (num_dram_wr > 0 && num_dram_rd == 0) ) {
            m_db_dram_rw_modes[pch_idx] = DRAM_WR;
          } else if((m_num_dram_wr_cnts[pch_idx] > 0 && elaped_time > ndp_dram_wr_max_age)) {
            m_db_dram_rw_modes[pch_idx] = DRAM_NDP_WR;
          } else {
            m_db_dram_rw_modes[pch_idx] = DRAM_RD;
          }          
          break;          
        }
        case DRAM_NDP_WR: {         
          if(m_num_ref_cnts[pch_idx] > 0) {
            m_db_dram_rw_modes[pch_idx] = DRAM_REF;
          } else if((m_num_dram_wr_cnts[pch_idx] > 0 && ((m_ndp_dram_wr_timer[pch_idx] < ndp_wr_mode_min_time) || m_num_rd_cnts[pch_idx] == 0))) {
            m_db_dram_rw_modes[pch_idx] = DRAM_NDP_WR;              
          } else if(num_dram_wr > m_wr_high_threshold || (num_dram_wr > 0 && num_dram_rd == 0)) {
            m_db_dram_rw_modes[pch_idx] = DRAM_WR;
          } else {
            m_db_dram_rw_modes[pch_idx] = DRAM_RD;
          }          
          break;
        }          
        case DRAM_RD: {
          if(m_num_ref_cnts[pch_idx] > 0) {
            m_db_dram_rw_modes[pch_idx] = DRAM_REF;
          } else if((m_num_dram_wr_cnts[pch_idx] > 0 && elaped_time > ndp_dram_wr_max_age) && 
                    ((m_dram_rd_timer[pch_idx] >= dram_rd_mode_min_time) || 
                    m_num_post_rd_cnts[pch_idx] == 8)) {
            m_db_dram_rw_modes[pch_idx] = DRAM_NDP_WR;              
          } else if(num_dram_wr > m_wr_high_threshold || (num_dram_wr > 0 && num_dram_rd == 0)) {
            m_db_dram_rw_modes[pch_idx] = DRAM_WR;
          } else {
            m_db_dram_rw_modes[pch_idx] = DRAM_RD;
          }         
          break;
        }
        default: {
          throw std::runtime_error("Invalid DB-DRAM RW Mode!");
          break;
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
      for(int pch_idx=0;pch_idx<num_pseudochannel;pch_idx++) {
        if(m_priority_buffers[pch_idx].size() != 0) {
          is_empty_priority_per_pch[pch_idx] = false;
        } else {
          is_empty_priority_per_pch[pch_idx] = true;
        }
      }

      // To evoid the closing of the request in the activated_buffer 
      m_dram->reset_need_be_open_per_bank(m_channel_id);
      for(auto req : m_active_buffer) {
        m_dram->set_need_be_open_per_bank(req.addr_vec);
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
        for(auto pch_idx : rr_pch_idx) {
          if (m_priority_buffers[pch_idx].size() != 0) {
            req_buffer = &m_priority_buffers[pch_idx];
            req_it = m_priority_buffers[pch_idx].begin();
            req_it->command = m_dram->get_preq_command(req_it->final_command, req_it->addr_vec);
            
            request_found = m_dram->check_ready(req_it->command, req_it->addr_vec);
          }
          if(request_found) break;
        }

        /* ==================================================================================================== */
        if(!request_found) {
          for(auto pch_idx : rr_pch_idx) {
            // If each pch has high priority request, skip search issuable requset 
            // Search allowed command list for RD_BUF/WR_BUF
            //    - Search RD, DB_RD, DRAM_RD
            //    - Search PRE_RD
            //    - Search DB_WR, DRAM_WR
            //    - Search PRE_WR
            // Search FR-FC at PRE_RD/WR_BUF

            set_mode_per_pch(pch_idx);
            if(is_empty_priority_per_pch[pch_idx]) {
              if(m_mc_db_rw_modes[pch_idx] == DB_NDP_WR) {
                // ================================ DB_NDP_WR MODE ================================
                if(m_db_dram_rw_modes[pch_idx] == DRAM_REF) {
                  // NDP_WR && REF MODE: DB_WR
                  // Seach Buffer: WR_BUF (DB_WR)
                  // 2 std::vector<int> cmd_list = {m_cmds.NDP_DB_WR};
                  if (req_it = m_scheduler->get_best_request_with_priority(m_write_buffers[pch_idx],2); req_it != m_write_buffers[pch_idx].end()) {
                    request_found = m_dram->check_ready(req_it->command, req_it->addr_vec);
                    req_buffer = &m_write_buffers[pch_idx];                                  
                  }   

                } else if(m_db_dram_rw_modes[pch_idx] == DRAM_RD) {
                  // NDP_WR && DRAM_RD MODE: DB_WR, PRE_RD, DRAM_RD
                  // Seach Buffer: WR_BUF (DB_WR) -> RD_BUF (PRE_RD) -> RD_BUF (DRAM_RD)
                  // Search Issuable DB_WR
                  if(!request_found) {
                    // 2 std::vector<int> cmd_list = {m_cmds.NDP_DB_WR};
                    if (req_it = m_scheduler->get_best_request_with_priority(m_write_buffers[pch_idx],2); req_it != m_write_buffers[pch_idx].end()) {
                      request_found = m_dram->check_ready(req_it->command, req_it->addr_vec);
                      req_buffer = &m_write_buffers[pch_idx];                                  
                    }   
                  }
                  // Search Issuable PRE_RD
                  if(!request_found && m_enable_pre_rd[pch_idx]) {
                    if (req_it = m_scheduler->get_best_pre_request(m_read_buffers[pch_idx]); req_it != m_read_buffers[pch_idx].end()) {
                      request_found = m_dram->check_ready(req_it->command, req_it->addr_vec);
                      req_buffer = &m_read_buffers[pch_idx];   
                      if(request_found) {
                        if(req_it->final_command == m_cmds.RD || req_it->final_command == m_cmds.RDA) {
                          req_it->final_command = m_cmds.PRE_RD;
                          req_it->is_db_cmd = true;  
                        } else {
                          throw std::runtime_error("DB_WR/DB_NDP_WR Mode - Wrong Pick up DDR Command (Not RD/RDA)");
                        }
                      }                                 
                    }      
                  }
                  // Search Issuable DRAM_RD
                  if(!request_found) {
                    // 3 std::vector<int> cmd_list = {m_cmds.NDP_DRAM_RD,m_cmds.NDP_DRAM_RDA};
                    if (req_it = m_scheduler->get_best_request_with_priority(m_read_buffers[pch_idx],3); req_it != m_read_buffers[pch_idx].end()) {
                      request_found = m_dram->check_ready(req_it->command, req_it->addr_vec);
                      req_buffer = &m_read_buffers[pch_idx];                                  
                    }   
                  }
                } else {
                  // NDP_WR && DRAM_WR MODE: DB_WR, POST_WR, DRAM_WR
                  // Seach Buffer: WR_BUF (DB_WR) -> PRE_WR_BUF (POST_WR) -> WR_BUF (DRAM_WR)
                  // Search Issuable DB_WR
                  if(!request_found) {
                    // 2 std::vector<int> cmd_list = {m_cmds.NDP_DB_WR};
                    if (req_it = m_scheduler->get_best_request_with_priority(m_write_buffers[pch_idx],2); req_it != m_write_buffers[pch_idx].end()) {
                      request_found = m_dram->check_ready(req_it->command, req_it->addr_vec);
                      req_buffer = &m_write_buffers[pch_idx];                                  
                    }   
                  }
                  // Search Issuable POST_WR
                  if(!request_found) {
                    if (req_it = m_scheduler->get_best_request(m_wr_prefetch_buffers[pch_idx]); req_it != m_wr_prefetch_buffers[pch_idx].end()) {
                      request_found = m_dram->check_ready(req_it->command, req_it->addr_vec);
                      req_buffer = &m_wr_prefetch_buffers[pch_idx];                                                                       
                    }  
                  }
                  // Search Issuable DRAM_WR
                  if(!request_found) {
                    // 4 std::vector<int> cmd_list = {m_cmds.NDP_DRAM_WR,m_cmds.NDP_DRAM_WRA};
                    if (req_it = m_scheduler->get_best_request_with_priority(m_write_buffers[pch_idx],4); req_it != m_write_buffers[pch_idx].end()) {
                      request_found = m_dram->check_ready(req_it->command, req_it->addr_vec);
                      req_buffer = &m_write_buffers[pch_idx];                                  
                    }   
                  }
                }
              }
              else if(m_mc_db_rw_modes[pch_idx] == DB_RD) {
                // ================================ DB_RD MODE ================================
                if(m_db_dram_rw_modes[pch_idx] == DRAM_REF) {
                  // -------------------------------- DRAM_REF MODE --------------------------------
                  // DB_RD && REF MODE: POST_RD, DB_RD
                  // Seach Buffer: PRE_RD_BUF (POST_RD) -> RD_BUF (DB_RD)
                  // Search Issuable POST_RD
                  if(!request_found) {
                    if (req_it = m_scheduler->get_best_request(m_rd_prefetch_buffers[pch_idx]); req_it != m_rd_prefetch_buffers[pch_idx].end()) {
                      request_found = m_dram->check_ready(req_it->command, req_it->addr_vec);
                      req_buffer = &m_rd_prefetch_buffers[pch_idx];                                                                       
                    }  
                  }    
                  // Search Issuable DB_RD
                  if(!request_found) {
                    // 1 std::vector<int> cmd_list = {m_cmds.NDP_DB_RD};
                    if (req_it = m_scheduler->get_best_request_with_priority(m_read_buffers[pch_idx],1); req_it != m_read_buffers[pch_idx].end()) {
                      request_found = m_dram->check_ready(req_it->command, req_it->addr_vec);
                      req_buffer = &m_read_buffers[pch_idx];                                  
                    }   
                  }                  
                } else if(m_db_dram_rw_modes[pch_idx] == DRAM_RD) {
                  // -------------------------------- DRAM_RD MODE --------------------------------
                  // DB_RD && DRAM_RD MODE: POST_RD, RD, PRE_RD, DB_RD, DRAM_RD
                  // Seach Buffer: PRE_RD_BUF (POST_RD) -> RD_BUF (RD, PRE_RD, DB_RD, DRAM_RD)
                  // Search Issuable POST_RD
                  if(!request_found) {
                    if (req_it = m_scheduler->get_best_request(m_rd_prefetch_buffers[pch_idx]); req_it != m_rd_prefetch_buffers[pch_idx].end()) {
                      request_found = m_dram->check_ready(req_it->command, req_it->addr_vec);
                      req_buffer = &m_rd_prefetch_buffers[pch_idx];                                                                       
                    }  
                  }    
                  // Search Issuable RD, DB_RD, DRAM_RD
                  if(!request_found) {
                    // 0 std::vector<int> cmd_list = {m_cmds.RD};
                    if (req_it = m_scheduler->get_best_request_with_priority(m_read_buffers[pch_idx],0); req_it != m_read_buffers[pch_idx].end()) {
                      request_found = m_dram->check_ready(req_it->command, req_it->addr_vec);
                      req_buffer = &m_read_buffers[pch_idx];                                  
                    }   
                  } 
                  // Search Issuable PRE_RD
                  if(!request_found && m_enable_pre_rd[pch_idx]) {
                    if (req_it = m_scheduler->get_best_pre_request(m_read_buffers[pch_idx]); req_it != m_read_buffers[pch_idx].end()) {
                      request_found = m_dram->check_ready(req_it->command, req_it->addr_vec);
                      req_buffer = &m_read_buffers[pch_idx];   
                      if(request_found) {
                        if(req_it->final_command == m_cmds.RD || req_it->final_command == m_cmds.RDA) {
                          req_it->final_command = m_cmds.PRE_RD;
                          req_it->is_db_cmd = true;  
                        } else {
                          throw std::runtime_error("DB_WR/DRAM_RD Mode - Wrong Pick up DDR Command (Not RD/RDA)");
                        }
                      }                                 
                    }
                  }   
                  // Search Issuable DB_RD, DRAM_RD
                  if(!request_found) {
                    // 3 std::vector<int> cmd_list = {m_cmds.NDP_DRAM_RD,m_cmds.NDP_DRAM_RDA};
                    if (req_it = m_scheduler->get_best_request_with_priority(m_read_buffers[pch_idx],3); req_it != m_read_buffers[pch_idx].end()) {
                      request_found = m_dram->check_ready(req_it->command, req_it->addr_vec);
                      req_buffer = &m_read_buffers[pch_idx];                                  
                    }   
                  }                  
                } else {
                  // -------------------------------- DRAM_WR MODE --------------------------------
                  // DB_RD && DRAM_WR MODE: POST_RD, DB_RD, POST_WR, DRAM_WR
                  // Seach Buffer: PRE_RD_BUF (POST_RD) -> RD_BUF (DB_RD) -> PRE_WR (POST_WR) -> WR_BUF (DRAM_WR)
                  // Search Issuable POST_RD
                  if(!request_found) {
                    if (req_it = m_scheduler->get_best_request(m_rd_prefetch_buffers[pch_idx]); req_it != m_rd_prefetch_buffers[pch_idx].end()) {
                      request_found = m_dram->check_ready(req_it->command, req_it->addr_vec);
                      req_buffer = &m_rd_prefetch_buffers[pch_idx];                                                                       
                    }  
                  }    
                  // Search Issuable DB_RD
                  if(!request_found) {
                    // 1 std::vector<int> cmd_list = {m_cmds.NDP_DB_RD};
                    if (req_it = m_scheduler->get_best_request_with_priority(m_read_buffers[pch_idx],1); req_it != m_read_buffers[pch_idx].end()) {
                      request_found = m_dram->check_ready(req_it->command, req_it->addr_vec);
                      req_buffer = &m_read_buffers[pch_idx];                                  
                    }   
                  } 
                  // Search Issuable POST_WR
                  if(!request_found) {
                    if (req_it = m_scheduler->get_best_request(m_wr_prefetch_buffers[pch_idx]); req_it != m_wr_prefetch_buffers[pch_idx].end()) {
                      request_found = m_dram->check_ready(req_it->command, req_it->addr_vec);
                      req_buffer = &m_wr_prefetch_buffers[pch_idx];                                                                       
                    }  
                  }
                  // Search Issuable DRAM_WR
                  if(!request_found) {
                    // 4 std::vector<int> cmd_list = {m_cmds.NDP_DRAM_WR,m_cmds.NDP_DRAM_WRA};
                    if (req_it = m_scheduler->get_best_request_with_priority(m_write_buffers[pch_idx],4); req_it != m_write_buffers[pch_idx].end()) {
                      request_found = m_dram->check_ready(req_it->command, req_it->addr_vec);
                      req_buffer = &m_write_buffers[pch_idx];                                  
                    }   
                  }                                    
                }
              } else {
                // ================================ DB_WR MODE ================================
                if(m_db_dram_rw_modes[pch_idx] == DRAM_REF) {
                  // -------------------------------- DRAM_REF MODE --------------------------------
                  // DB_WR && REF MODE: PRE_WR
                  // Seach Buffer: WR_BUF (PRE_WR)
                  // Search Issuable PRE_WR
                  if(!request_found) {
                    if (req_it = m_scheduler->get_best_pre_request(m_write_buffers[pch_idx]); req_it != m_write_buffers[pch_idx].end()) {
                      request_found = m_dram->check_ready(req_it->command, req_it->addr_vec);
                      req_buffer = &m_write_buffers[pch_idx];   
                      if(request_found) {
                        if(req_it->final_command == m_cmds.WR || req_it->final_command == m_cmds.WRA) {
                          req_it->final_command = m_cmds.PRE_WR;
                          req_it->is_db_cmd = true;  
                        } else {
                          m_dram->print_req(*req_it);
                          throw std::runtime_error("DB_WR/DRAM_REF Mode - Wrong Pick up DDR Command (Not WR/WRA)");
                        }
                      }                                 
                    }      
                  }
                } else if(m_db_dram_rw_modes[pch_idx] == DRAM_RD) {
                  // -------------------------------- DRAM_RD MODE --------------------------------
                  // DB_WR && DRAM_RD MODE: PRE_WR, PRE_RD, DRAM_RD
                  // Seach Buffer: WR_BUF (PRE_WR) -> RD_BUF (PRE_RD) -> RD_BUF (DRAM_RD)
                  // Search Issuable PRE_WR
                  if(!request_found) {
                    if (req_it = m_scheduler->get_best_pre_request(m_write_buffers[pch_idx]); req_it != m_write_buffers[pch_idx].end()) {
                      request_found = m_dram->check_ready(req_it->command, req_it->addr_vec);
                      req_buffer = &m_write_buffers[pch_idx];   
                      if(request_found) {
                        if(req_it->final_command == m_cmds.WR || req_it->final_command == m_cmds.WRA) {
                          req_it->final_command = m_cmds.PRE_WR;
                          req_it->is_db_cmd = true;  
                        } else {
                          m_dram->print_req(*req_it);
                          throw std::runtime_error("DB_WR/DRAM_RD Mode - Wrong Pick up DDR Command (Not WR/WRA)");
                        }
                      }                                 
                    }      
                  }
                  // Search Issuable PRE_RD
                  if(!request_found && m_enable_pre_rd[pch_idx]) {
                    if (req_it = m_scheduler->get_best_pre_request(m_read_buffers[pch_idx]); req_it != m_read_buffers[pch_idx].end()) {
                      request_found = m_dram->check_ready(req_it->command, req_it->addr_vec);
                      req_buffer = &m_read_buffers[pch_idx];   
                      if(request_found) {
                        if(req_it->final_command == m_cmds.RD || req_it->final_command == m_cmds.RDA) {
                          req_it->final_command = m_cmds.PRE_RD;
                          req_it->is_db_cmd = true;  
                        } else {
                          throw std::runtime_error("DB_WR/DRAM_RD Mode - Wrong Pick up DDR Command (Not RD/RDA)");
                        }
                      }                                 
                    }
                  }                  
                  // Search Issuable DRAM_RD
                  if(!request_found) {
                    // 3 std::vector<int> cmd_list = {m_cmds.NDP_DRAM_RD,m_cmds.NDP_DRAM_RDA};
                    if (req_it = m_scheduler->get_best_request_with_priority(m_read_buffers[pch_idx],3); req_it != m_read_buffers[pch_idx].end()) {
                      request_found = m_dram->check_ready(req_it->command, req_it->addr_vec);
                      req_buffer = &m_read_buffers[pch_idx];                                  
                    }   
                  }                  
                } else {
                  // -------------------------------- DRAM_WR MODE --------------------------------
                  // DB_WR && DRAM_WR MODE: PRE_WR, POST_WR, DRAM_WR
                  // Seach Buffer: WR_BUF (PRE_WR) -> PRE_WR_BUF (POST_WR) -> WR_BUF (DRAM_WR)
                  // Search Issuable PRE_WR
                  if(!request_found) {
                    if (req_it = m_scheduler->get_best_pre_request(m_write_buffers[pch_idx]); req_it != m_write_buffers[pch_idx].end()) {
                      request_found = m_dram->check_ready(req_it->command, req_it->addr_vec);
                      req_buffer = &m_write_buffers[pch_idx];   
                      if(request_found) {
                        if(req_it->final_command == m_cmds.WR || req_it->final_command == m_cmds.WRA) {
                          req_it->final_command = m_cmds.PRE_WR;
                          req_it->is_db_cmd = true;  
                        } else {
                          m_dram->print_req(*req_it);
                          throw std::runtime_error("DB_WR/DRAM_WR Mode - Wrong Pick up DDR Command (Not WR/WRA)");
                        }
                      }                                 
                    }      
                  }       
                  // Search Issuable POST_WR
                  if(!request_found) {
                    if (req_it = m_scheduler->get_best_request(m_wr_prefetch_buffers[pch_idx]); req_it != m_wr_prefetch_buffers[pch_idx].end()) {
                      request_found = m_dram->check_ready(req_it->command, req_it->addr_vec);
                      req_buffer = &m_wr_prefetch_buffers[pch_idx];                                                                       
                    }  
                  }
                  // Search Issuable DRAM_WR
                  if(!request_found) {
                    // 4 std::vector<int> cmd_list = {m_cmds.NDP_DRAM_WR,m_cmds.NDP_DRAM_WRA};
                    if (req_it = m_scheduler->get_best_request_with_priority(m_write_buffers[pch_idx],4); req_it != m_write_buffers[pch_idx].end()) {
                      request_found = m_dram->check_ready(req_it->command, req_it->addr_vec);
                      req_buffer = &m_write_buffers[pch_idx];                                  
                    }   
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
              if(req_it->is_db_cmd) {
                req_it->is_db_cmd = false;
                if(req_it->final_command == m_cmds.PRE_WR)  
                  req_it->final_command == m_cmds.WR;
                else 
                  req_it->final_command == m_cmds.RD;
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

        std::cout << "\n========== Decoupled Scheduling Mode Statistics ==========" << std::endl;
        std::cout << std::fixed << std::setprecision(2);
        
        for (int ch = 0; ch < num_pseudochannel; ch++) {
            print_channel_stats(ch);
        }        
      

        print_interval_statistics();

      return;
    }

    bool is_finished() override {
      bool is_dram_ctrl_finished = true;

      if((m_active_buffer.size() != 0) || (pending.size() != 0)) is_dram_ctrl_finished = false;

      if(is_dram_ctrl_finished) {
        for(int i=0;i<num_pseudochannel;i++) {
          if(m_read_buffers[i].size() != 0 || m_rd_prefetch_buffers[i].size() != 0) is_dram_ctrl_finished = false;
          // m_write_buffers[i].size() != 0  || 
          // || m_wr_prefetch_buffers[i].size() != 0
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

    virtual size_t get_host_acces_latency() {
      return s_normal_read_latency;
    }

    void print_channel_stats(int ch) const {
        const auto& stats = m_channel_stats[ch];
        
        std::cout << "\n[Pseudo Channel " << ch << "]" << std::endl;

        // Request counts
        std::cout << "  --- Request Counts ---" << std::endl;
        std::cout << "    " << std::setw(12) << std::left << "Read Requests" << ": "
                  << std::setw(12) << std::right << m_num_read_req[ch] << std::endl;
        std::cout << "    " << std::setw(12) << std::left << "Write Requests" << ": "
                  << std::setw(12) << std::right << m_num_write_req[ch] << std::endl;
        std::cout << "    " << std::setw(12) << std::left << "Total Requests" << ": "
                  << std::setw(12) << std::right << (m_num_read_req[ch] + m_num_write_req[ch]) << std::endl;

              
        std::cout << "  --- MC <-> DB Modes ---" << std::endl;
        
        // Print MC <-> DB mode statistics
        for (int mode = 0; mode < 3; mode++) {
            uint64_t cycles = stats.mc_db_mode_cycles[mode];
            double ratio = (m_clk > 0) ? (100.0 * cycles / m_clk) : 0.0;
            
            std::cout << "    " << std::setw(12) << std::left << MC_DB_MODE_NAMES[mode] << ": "
                      << std::setw(12) << std::right << cycles 
                      << " cycles (" << std::setw(6) << ratio << "%)" << std::endl;
        }
        
        
        std::cout << "\n  --- DB <-> DRAM Modes ---" << std::endl;
        
        // Print DB <-> DRAM mode statistics
        for (int mode = 0; mode < 4; mode++) {
            uint64_t cycles = stats.db_dram_mode_cycles[mode];
            double ratio = (m_clk > 0) ? (100.0 * cycles / m_clk) : 0.0;
            
            std::cout << "    " << std::setw(12) << std::left << DB_DRAM_MODE_NAMES[mode] << ": "
                      << std::setw(12) << std::right << cycles 
                      << " cycles (" << std::setw(6) << ratio << "%)" << std::endl;
        }
              
    } 


    void print_channel_periodic_stats(int ch) const {
        const auto& stats = m_periodic_channel_stats[ch];
        
        std::cout << "\n[Pseudo Channel " << ch << "]" << std::endl;

        std::cout << "  --- MC <-> DB Modes ---" << std::endl;
        
        // Print MC <-> DB mode statistics
        for (int mode = 0; mode < 3; mode++) {
            uint64_t cycles = stats.mc_db_mode_cycles[mode];
            double ratio = (100000 > 0) ? (100.0 * cycles / 100000) : 0.0;
            
            std::cout << "    " << std::setw(12) << std::left << MC_DB_MODE_NAMES[mode] << ": "
                      << std::setw(12) << std::right << cycles 
                      << " cycles (" << std::setw(6) << ratio << "%)" << std::endl;
        }
        
        
        std::cout << "\n  --- DB <-> DRAM Modes ---" << std::endl;
        
        // Print DB <-> DRAM mode statistics
        for (int mode = 0; mode < 4; mode++) {
            uint64_t cycles = stats.db_dram_mode_cycles[mode];
            double ratio = (100000 > 0) ? (100.0 * cycles / 100000) : 0.0;
            
            std::cout << "    " << std::setw(12) << std::left << DB_DRAM_MODE_NAMES[mode] << ": "
                      << std::setw(12) << std::right << cycles 
                      << " cycles (" << std::setw(6) << ratio << "%)" << std::endl;
        }
              
    } 

    // Check if NDP request can be added to buffer    
    bool can_accept_ndp_request(const Request& req, int psuedo_ch_idx) {
        if(req.is_ndp_req) {
          if (req.type_id == Request::Type::Read) {
              // Check total NDP read requests (NDP_DB_RD + NDP_DRAM_RD)
              size_t ndp_rd_req = m_num_db_rd_cnts[psuedo_ch_idx] +  m_num_dram_rd_cnts[psuedo_ch_idx];
              return ndp_rd_req < m_max_ndp_read_reqs[psuedo_ch_idx];
          } else if (req.type_id == Request::Type::Write) {
              // Check total NDP write requests (NDP_DB_WR + NDP_DRAM_WR)
              size_t ndp_wr_req = m_num_db_wr_cnts[psuedo_ch_idx] +  m_num_dram_wr_cnts[psuedo_ch_idx];            
              return ndp_wr_req < m_max_ndp_write_reqs[psuedo_ch_idx];
          }
        }
        return true;  // Non-NDP requests always accepted
    }    

    void print_interval_statistics() {
        size_t num_intervals = m_his_num_rd.size();
        
        std::cout << "=== Interval Statistics ===" << std::endl;
        std::cout << std::setw(10) << "Interval"
                  << std::setw(12) << "CMD"
                  << std::setw(12) << "RD"
                  << std::setw(12) << "WR"
                  << std::setw(12) << "PRE_WR"
                  << std::setw(12) << "POST_WR"
                  << std::setw(12) << "PRE_RD"
                  << std::setw(12) << "POST_RD"
                  << std::setw(12) << "NDP_D_RD"
                  << std::setw(12) << "NDP_D_WR"
                  << std::setw(12) << "NDP_DB_RD"
                  << std::setw(12) << "NDP_DB_WR"
                  << std::endl;
        
        for (size_t i = 0; i < num_intervals; i++) {
            std::cout << std::setw(10) << i
                      << std::setw(12) << m_his_num_cmd[i]
                      << std::setw(12) << m_his_num_rd[i]
                      << std::setw(12) << m_his_num_wr[i]
                      << std::setw(12) << m_his_num_pre_wr[i]
                      << std::setw(12) << m_his_num_post_wr[i]
                      << std::setw(12) << m_his_num_pre_rd[i]
                      << std::setw(12) << m_his_num_post_rd[i]
                      << std::setw(12) << m_his_num_ndp_dram_rd[i]
                      << std::setw(12) << m_his_num_ndp_dram_wr[i]
                      << std::setw(12) << m_his_num_ndp_db_rd[i]
                      << std::setw(12) << m_his_num_ndp_db_wr[i]
                      << std::endl;
        }
    }    

};
  
}   // namespace Ramulator