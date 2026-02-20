#include "memory_system/memory_system.h"
#include "translation/translation.h"
#include "dram_controller/controller.h"
#include "addr_mapper/addr_mapper.h"
#include "dram/dram.h"
#include <sstream>
#include <filesystem>
#include <iostream>
#include <fstream>
// #define TRACE_ON 

namespace Ramulator {

namespace fs = std::filesystem;

class GenericDRAMSystem final : public IMemorySystem, public Implementation {
  RAMULATOR_REGISTER_IMPLEMENTATION(IMemorySystem, GenericDRAMSystem, "GenericDRAM", "A generic DRAM-based memory system.");

  protected:
    Clk_t m_clk = 0;
    IDRAM*  m_dram;
    IAddrMapper*  m_addr_mapper;
    std::vector<IDRAMController*> m_controllers;    

  public:
    Logger_t m_logger;
    
    size_t s_num_read_requests = 0;
    size_t s_num_write_requests = 0;
    size_t s_num_other_requests = 0;
    float s_avg_read_latency = 0;
    bool is_open_trace_file = false;

    /*
      Gem5 Simulation with Trace Core (Virtual Core)
    */

    bool m_host_access = false;
    bool m_trace_core_enable = false;    
    bool m_ndp_trace = true;
    // Copy Structure within loadstore_ncore_trace.cpp
    struct Trace {
      uint64_t timestamp;  // When this request should be issued (in cycles)
      bool is_write;
      Addr_t addr;
      std::vector<uint64_t> payload;
    };
    size_t m_trace_core_mshr_size;
    size_t m_curr_trace_idx;
    std::vector<Trace> m_trace;
    
    int m_next_request_id = 0;
    bool m_debug_mode = false;
    int m_wait_trace_done = 0;
    
    // Outstanding requests tracking
    struct OutstandingRequest {
      uint64_t issue_time;
      Addr_t addr;
    };

    size_t m_max_outstanding;
    std::unordered_map<int, OutstandingRequest> m_outstanding_reads;

  public:
    void init() override { 

      m_logger = Logging::create_logger("GenericDRAMSystem");
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
      register_stat(s_avg_read_latency).name("avg_host_read_latency");      

      // NDP Trace Initialization
      m_trace_core_enable = param<bool>("trace_core_enable").desc("Enable Trace Simulation Core with Gem5").default_val(false);
      
      if(m_trace_core_enable) {
        m_ndp_trace = false;
        m_logger->info(" Enable Trace Core Simulation with Gem5 frontend");                            
        if(m_ndp_trace) m_logger->info("  -- Trace Type: NDP Workload");
        else            m_logger->info("  -- Trace Type: Host Access Workload");

        // Set Trace Simulation Core MSHR SIze
        m_trace_core_mshr_size = param<size_t>("trace_core_mshr_size").desc("MSHR size for Trace_Core").default_val(16);
        m_logger->info("  -- Trace Core MSHR Size: {}",m_trace_core_mshr_size);  
        m_max_outstanding = m_trace_core_mshr_size;
        
        // Load Trace File
        std::string trace_path = param<std::string>("trace_path").desc("Trace file path for Trace Core").required();
        m_logger->info("  -- Trace Path: {}",trace_path);  
        load_trace(trace_path,m_trace);
        m_logger->info("  -- Loaded {} trace lines",m_trace.size()); 
      }

    };

    void setup(IFrontEnd* frontend, IMemorySystem* memory_system) override { }

    bool send(Request req) override {
      m_addr_mapper->apply(req);
      int channel_id = req.addr_vec[0];
      bool is_success = m_controllers[channel_id]->send(req);

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

      #ifdef TRACE_ON
        // Open Trace File to track memory pattern 
        if(!is_open_trace_file) {
          if(tout.is_open()) {
            throw std::runtime_error("The file is already open!");
          }
          // Open Trace Path File 
          tout.open(trace_path);
          is_open_trace_file = true;
          if(tout.is_open()) {
            m_logger->info(" Open Trace File to store memroy address pattern {}",trace_path);
          }
        }
        if(is_open_trace_file) {
          if(is_success) {
            std::stringstream ss;
            if(req.type_id == 0) {
              ss << "LD "<<"0x"<<std::hex<<req.addr<<std::endl;
            } else {
              ss << "ST "<<"0x"<<std::hex<<req.addr<<std::endl;
            }
            std::string pattern = ss.str();
            tout.write(pattern.c_str(),pattern.size());
          }
        }
      #endif 
      m_host_access = true;
      return is_success;
    };
    
    void tick() override {
      m_clk++;
      // Trace Mode :  Send Trace-based Request to DRAM System
      if(m_trace_core_enable && !m_host_access) try_issue_requests(); 
      m_host_access = false;

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

    bool is_finished() override {
      bool is_dram_ctrl_finished = true;
      int num_channels = m_dram->get_level_size("channel"); 
      for (int i = 0; i < num_channels; i++) {
        if(!m_controllers[i]->is_finished())
          is_dram_ctrl_finished = false;
      }

      return (is_dram_ctrl_finished);
    }    

    // Always Done
    bool is_ndp_finished() override {
      return true;
    }     
    

    virtual void mem_sys_finalize() override {
      size_t total_latency = 0;
      int num_channels = m_dram->get_level_size("channel");
      for (int i = 0; i < num_channels; i++) {
        total_latency+=m_controllers[i]->get_host_acces_latency();
        std::cout<<"Latency : "<<total_latency<<std::endl;
      } 
      
      // There is no normal read request, read latency is minus 1
      if (s_num_read_requests == 0) 
        s_avg_read_latency = -1.0;
      else                          
        s_avg_read_latency = (float)total_latency/(float)s_num_read_requests;

      #ifdef TRACE_ON
        if(is_open_trace_file) {
          tout.close();
          m_logger->info(" Close Trace File {}",trace_path);
        }
      #endif        

      std::cout << "\n=== Memory System Bandwidth (GB/s) ===\n";
      std::cout << "Assume: 512 bits/access, BW = bytes/ns\n\n";      

      // 12 Counters per Memory Controller
      std::vector<uint64_t> counters;
      counters.resize(2, 0);
      for (int i = 0; i < num_channels; i++) {
        auto counters_per_mc = m_controllers[i]->get_counters();
        if(counters_per_mc.size() == 2) {
          for (int c_idx = 0; c_idx < 2; c_idx++) {
            counters[c_idx]+= counters_per_mc[c_idx];
          }
        } else {
          throw std::runtime_error("Invalid Counter Number");
        }
      }          
      int tCK_ps = m_dram->m_timing_vals("tCK_ps");
      // ---- Main window ----
      std::cout << "[Main window]\n";      
      print_bw("Host<->DB/DRAM",
          calc_bw_gbs(counters[0], 1.0, m_clk, tCK_ps));

      // ---- tcore window ----
      std::cout << "\n[tcore window]\n";
      print_bw("tcore Host<->DB/DRAM",
          calc_bw_gbs(counters[1], 1.0, m_clk, tCK_ps));      
    }


    /*
      Trace Core - Load trace & Send Request to the memory system
    */

    // Copy Structure within loadstore_ncore_trace.cpp
    void load_trace(const std::string& file_path_str, std::vector<Trace>& trace_vec) {
      fs::path trace_path(file_path_str);
      if (!fs::exists(trace_path)) {
        throw ConfigurationError("Trace {} does not exist!", file_path_str);
      }

      std::ifstream trace_file(trace_path);
      if (!trace_file.is_open()) {
        throw ConfigurationError("Trace {} cannot be opened!", file_path_str);
      }

      std::string line;
      size_t line_number = 0;
      uint64_t default_timestamp = 0;  // For traces without timing info
      
      while (std::getline(trace_file, line)) {
        line_number++;
        
        // Skip empty lines and comments
        if (line.empty() || line[0] == '#') {
          continue;
        }
        
        Trace t;
        std::vector<std::string> tokens;
        tokenize(tokens, line, " ");

        if (tokens.empty()) {
          continue;
        }

        // Format: [TIMESTAMP] LD/ST ADDR [PAYLOAD...]
        // Or legacy: LD/ST ADDR [PAYLOAD...]
        
        size_t token_offset = 0;
        
        // Check if first token is a timestamp (number)
        if (tokens.size() >= 3 && std::isdigit(tokens[0][0])) {
          t.timestamp = std::stoull(tokens[0]);
          token_offset = 1;
        } else if (tokens.size() >= 2) {
          // Legacy format without timestamp - use sequential numbering
          t.timestamp = default_timestamp++;
          token_offset = 0;
        } else {
          throw ConfigurationError("Trace {} format invalid at line {}!", file_path_str, line_number);
        }

        // Parse operation
        bool is_write = false;
        if (tokens[token_offset] == "LD") {
          is_write = false;
        } else if (tokens[token_offset] == "ST") {
          is_write = true;
        } else {
          throw ConfigurationError("Trace {} format invalid at line {}! Unknown operation '{}'.", 
                                  file_path_str, line_number, tokens[token_offset]);
        }

        // Parse address
        Addr_t addr = -1;
        std::string addr_str = tokens[token_offset + 1];
        if (addr_str.compare(0, 2, "0x") == 0 || addr_str.compare(0, 2, "0X") == 0) {
          addr = std::stoll(addr_str.substr(2), nullptr, 16);
        } else {
          addr = std::stoll(addr_str);
        }

        t.is_write = is_write;
        t.addr = addr;

        // Parse payload if present (for writes)
        if (is_write && tokens.size() > token_offset + 2) {
          size_t payload_count = tokens.size() - (token_offset + 2);
          if (payload_count == 8) {  // Expecting 8 x 64-bit values
            for (size_t i = 0; i < 8; i++) {
              std::string payload_str = tokens[token_offset + 2 + i];
              if (payload_str.compare(0, 2, "0x") == 0 || payload_str.compare(0, 2, "0X") == 0) {
                t.payload.push_back(std::stoull(payload_str.substr(2), nullptr, 16));
              } else {
                t.payload.push_back(std::stoull(payload_str));
              }
            }
          }
        }

        trace_vec.push_back(t);
      }

      trace_file.close();
    }

    void try_issue_requests() {
      if (m_curr_trace_idx >= m_trace.size() && m_outstanding_reads.empty()) {
          m_wait_trace_done++;

        if(m_wait_trace_done > 1000) {
          m_curr_trace_idx = 0; 
          m_next_request_id = 0;
          m_wait_trace_done = 0;
        }   
      }

      while (m_curr_trace_idx < m_trace.size() && 
             m_outstanding_reads.size() < m_max_outstanding) {
        
        const Trace& t = m_trace[m_curr_trace_idx];
        
        // Check if it's time to issue this request
        if (t.timestamp > m_clk) {
          break;  // Too early for this request
        }

        // Create request
        Request req = Request(t.addr, t.is_write ? Request::Type::Write : Request::Type::Read);
        
        if (t.is_write && !t.payload.empty()) {
          for (uint32_t i = 0; i < t.payload.size(); i++) {
            req.m_payload.push_back(t.payload[i]);
          }
        }

        // Only set callback for READ requests
        int req_id = -1;
        if (!t.is_write) {
          req_id = m_next_request_id++;
          
          // Set callback for read completion
          req.callback = [this, req_id](Request& completed_req) {
            this->on_read_complete(req_id, completed_req);
          };
        }
        
        // Try to send the request
        req.is_trace_core_req = true;
        bool request_sent = send(req);
        
        if (request_sent) {
          if (t.is_write) {
            // Write is fire-and-forget
            if (m_debug_mode) {
              m_logger->debug("Trace issued WRITE (addr={:#x})",t.addr);
            }
          } else {
            // Track outstanding read
            OutstandingRequest out_req;
            out_req.issue_time = m_clk;
            out_req.addr = t.addr;
            
            m_outstanding_reads[req_id] = out_req;

            if (m_debug_mode) {
              m_logger->debug("Trace issued READ {} (addr={:#x}, outstanding={})", 
                             req_id, t.addr, m_outstanding_reads.size());
            }                           
          }
          
          m_curr_trace_idx++;
        } else {        
          break;  // Memory system busy, try next cycle
        }
      }
    }

    void on_read_complete(int request_id, Request& req) {
      auto it = m_outstanding_reads.find(request_id);
      
      if (it != m_outstanding_reads.end()) {
        uint64_t latency = m_clk - it->second.issue_time;        
        if (m_debug_mode) {
          m_logger->debug("Trace READ {} completed (addr={:#x}, latency={} cycles, outstanding={})", 
                         request_id, it->second.addr, latency, 
                         m_outstanding_reads.size() - 1);
        }
        
        m_outstanding_reads.erase(it);
      } else {
        m_logger->error("Trace attempted to complete unknown READ request {}", request_id);
      }
    }
    static inline double calc_bw_gbs(uint64_t acc_cnt,
                                    double dq_scaling,
                                    uint64_t clk_cycles,
                                    uint64_t tCK_ps)
    {
        if (clk_cycles == 0 || tCK_ps == 0) return 0.0;

        // 512 bits/access -> bytes, time = clk * tCK (ps -> ns)
        const double bytes = (static_cast<double>(acc_cnt) * 512.0 * dq_scaling) / 8.0;
        const double time_ns = static_cast<double>(clk_cycles) * (static_cast<double>(tCK_ps) / 1000.0);
        return bytes / time_ns; // GB/s
    }

    static inline void print_bw(const std::string& name, double bw)
    {
        std::cout << std::left << std::setw(32) << name
                  << " : " << std::right << std::setw(10)
                  << std::fixed << std::setprecision(3)
                  << bw << " GB/s\n";
    }              
};
  
}   // namespace 

