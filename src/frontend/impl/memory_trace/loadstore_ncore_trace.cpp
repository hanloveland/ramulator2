#include <filesystem>
#include <iostream>
#include <fstream>
#include <unordered_map>
#include <vector>

#include "frontend/frontend.h"
#include "base/exception.h"

namespace Ramulator {

namespace fs = std::filesystem;

class LoadStoreNCoreTrace : public IFrontEnd, public Implementation {
  RAMULATOR_REGISTER_IMPLEMENTATION(IFrontEnd, LoadStoreNCoreTrace, "LoadStoreNCoreTrace", 
    "Load/Store memory address trace with multi-core support.")

  private:
    uint64_t stat_interval;
    struct Trace {
      uint64_t timestamp;  // When this request should be issued (in cycles)
      bool is_write;
      Addr_t addr;
      std::vector<uint64_t> payload;
    };

    // Per-core data structure
    struct CoreState {
      int core_id;
      std::vector<Trace> trace;
      size_t curr_idx;
      size_t max_outstanding;
      uint64_t m_max_trace_inst;      
      bool is_ndp_trace;
      bool is_ndp_done;
      size_t repeat_trace_count;
      size_t repeat_trace;

      // Outstanding requests tracking
      struct OutstandingRequest {
        uint64_t issue_time;
        Addr_t addr;
      };
      std::unordered_map<int, OutstandingRequest> outstanding_reads;
      
      // Statistics
      uint64_t total_read_requests;
      uint64_t total_write_requests;
      uint64_t completed_reads;
      uint64_t total_read_latency;
      uint64_t avg_outstanding_reads;
      uint64_t reach_max_outstanding_reads;
      uint64_t reach_controller_buffer;
      uint64_t issued_rd;
      uint64_t issued_wr;
      
      
      CoreState(int id, size_t mshr_size) 
        : core_id(id), curr_idx(0), max_outstanding(mshr_size), m_max_trace_inst(0), is_ndp_trace(false), is_ndp_done(false), repeat_trace_count(0), repeat_trace(1),
          total_read_requests(0), total_write_requests(0), 
          completed_reads(0), total_read_latency(0), 
          avg_outstanding_reads(0), reach_max_outstanding_reads(0),
          reach_controller_buffer(0), issued_rd(0), issued_wr(0)  {}
      
      bool is_finished() const {
        return (repeat_trace_count >= repeat_trace);      
      }
      
      double avg_read_latency() const {
        return completed_reads > 0 ? (double)total_read_latency / completed_reads : 0.0;
      }
    };

    std::vector<CoreState*> m_cores;
    int m_num_cores = 0;
    int m_next_request_id = 0;
    uint64_t m_current_cycle = 0;
    
    Logger_t m_logger;
    bool m_debug_mode = false;  // Debug mode flag

  public:
    void init() override {
      m_clock_ratio = param<uint>("clock_ratio").required();
      m_logger = Logging::create_logger("LoadStoreNCoreTrace");

      // Get number of cores
      m_num_cores = param<int>("num_cores").desc("Number of cores (traces)").default_val(1);
      
      if (m_num_cores <= 0) {
        throw ConfigurationError("num_cores must be positive!");
      }
      
      // Debug mode configuration
      m_debug_mode = param<bool>("debug_mode").desc("Enable detailed debug logging").default_val(false);
      
      m_logger->info("Initializing {} cores... (debug_mode={})", m_num_cores, m_debug_mode);

      uint64_t m_max_trace_inst = param<int>("max_inst").desc("Number of Issued Trace Instruction").default_val(1000);

      // Initialize each core
      for (int core_id = 0; core_id < m_num_cores; core_id++) {
        // Get per-core MSHR size
        std::string mshr_param_name = "core" + std::to_string(core_id) + "_mshr_size";
        size_t mshr_size = param<size_t>(mshr_param_name)
                            .desc("MSHR size for core " + std::to_string(core_id))
                            .default_val(16);
        
        CoreState* core = new CoreState(core_id, mshr_size);
        
        std::string is_ndp_trace_param_name = "core" + std::to_string(core_id) + "_is_ndp_trace";
        bool is_ndp_trace = param<bool>(is_ndp_trace_param_name)
                            .desc("Is NDP Trace for core " + std::to_string(core_id))
                            .default_val(false);
                                    
        core->is_ndp_trace = is_ndp_trace;

        std::string repeat_trace_param_name = "core" + std::to_string(core_id) + "_repeat";
        size_t repeat_trace = param<size_t>(repeat_trace_param_name)
                            .desc("Repeat Trace for core " + std::to_string(core_id))
                            .default_val(1);                                    
        core->repeat_trace = repeat_trace;

        core->m_max_trace_inst = m_max_trace_inst;
        // Load trace for this core
        std::string trace_param_name = "core" + std::to_string(core_id) + "_trace";
        try {
          std::string trace_path = param<std::string>(trace_param_name)
                                    .desc("Trace file path for core " + std::to_string(core_id))
                                    .required();
          
          m_logger->info("Loading trace for Core {} from {} ...", core_id, trace_path);
          load_trace(trace_path, core->trace);
          m_logger->info("Core {}: Loaded {} trace lines, MSHR size = {}", 
                        core_id, core->trace.size(), mshr_size);
        } catch (const std::exception& e) {
          delete core;
          throw ConfigurationError("Failed to load trace for core {}: {}", core_id, e.what());
        }
        
        m_cores.push_back(core);

        stat_interval = 100000;
      }

      m_logger->info("All {} cores initialized successfully", m_num_cores);
    };

    ~LoadStoreNCoreTrace() {
      for (auto core : m_cores) {
        delete core;
      }
    }

    void tick() override {
      m_current_cycle++;

      // Try to issue requests from each core
      bool is_ndp_done = m_memory_system->is_ndp_finished();
      for (auto core : m_cores) {
        core->is_ndp_done = is_ndp_done;      
        try_issue_requests(core);
      }

      // Progress reporting
      if (m_current_cycle % stat_interval == 0) {
        log_progress();
      }
    };

  private:
    void try_issue_requests(CoreState* core) {
      // Try to issue as many requests as possible from this core
      core->avg_outstanding_reads+=core->outstanding_reads.size();
      if(core->outstanding_reads.size() >= core->max_outstanding)
        core->reach_max_outstanding_reads += 1;

      if ((core->curr_idx >= core->trace.size() || core->curr_idx >= core->m_max_trace_inst) && core->outstanding_reads.empty() && 
          (!core->is_ndp_trace || (core->is_ndp_trace && core->is_ndp_done)) && core->repeat_trace_count < core->repeat_trace) {
        core->repeat_trace_count+=1;
        if(core->repeat_trace_count < core->repeat_trace) core->curr_idx = 0; 
      }

      while (core->curr_idx < core->trace.size() && 
             core->outstanding_reads.size() < core->max_outstanding && 
             core->m_max_trace_inst > core->curr_idx) {
        
        const Trace& t = core->trace[core->curr_idx];
        
        // Check if it's time to issue this request
        if (t.timestamp > m_current_cycle) {
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
          req.callback = [this, core, req_id](Request& completed_req) {
            this->on_read_complete(core, req_id, completed_req);
          };
        }
        
        // Try to send the request
        bool request_sent = m_memory_system->send(req);
        
        if (request_sent) {
          if (t.is_write) {
            // Write is fire-and-forget
            core->total_write_requests++;
            core->issued_wr++;
            if (m_debug_mode) {
              m_logger->debug("Core {} issued WRITE (addr={:#x})", 
                             core->core_id, t.addr);
            }
          } else {
            // Track outstanding read
            CoreState::OutstandingRequest out_req;
            out_req.issue_time = m_current_cycle;
            out_req.addr = t.addr;
            
            core->outstanding_reads[req_id] = out_req;
            core->total_read_requests++;
            core->issued_rd++;
            if (m_debug_mode) {
              m_logger->debug("Core {} issued READ {} (addr={:#x}, outstanding={})", 
                             core->core_id, req_id, t.addr, core->outstanding_reads.size());
            }                           
          }
          
          core->curr_idx++;
        } else {
          core->reach_controller_buffer+=1;
          break;  // Memory system busy, try next cycle
        }
      }
    }

    void on_read_complete(CoreState* core, int request_id, Request& req) {
      auto it = core->outstanding_reads.find(request_id);
      
      if (it != core->outstanding_reads.end()) {
        uint64_t latency = m_current_cycle - it->second.issue_time;
        core->total_read_latency += latency;
        core->completed_reads++;
        
        if (m_debug_mode) {
          m_logger->debug("Core {} READ {} completed (addr={:#x}, latency={} cycles, outstanding={})", 
                         core->core_id, request_id, it->second.addr, latency, 
                         core->outstanding_reads.size() - 1);
        }
        
        core->outstanding_reads.erase(it);
      } else {
        m_logger->error("Core {} attempted to complete unknown READ request {}", 
                       core->core_id, request_id);
      }
    }

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

      // Sort by timestamp to ensure chronological order
      // std::sort(trace_vec.begin(), trace_vec.end(), 
      //           [](const Trace& a, const Trace& b) { return a.timestamp < b.timestamp; });

      // std::cout<<"Vector Size : "<<trace_vec.size()<<std::endl;
    }

    void log_progress() {
      m_logger->info("=== Cycle {} Progress ===", m_current_cycle);
      for (auto core : m_cores) {
        m_logger->info("  Core {}: {}/{} issued (RD:{}, WR:{}), outstanding_reads:{}, avg_rd_latency={:.1f} cycles, Issued Read:{}, Issued Write:{}", 
                      core->core_id, 
                      core->curr_idx, 
                      core->trace.size(),
                      core->total_read_requests,
                      core->total_write_requests,
                      core->avg_outstanding_reads/stat_interval,
                      core->avg_read_latency(),
                      core->issued_rd,
                      core->issued_wr);
        core->avg_outstanding_reads = 0;
        core->reach_max_outstanding_reads = 0;
        core->reach_controller_buffer = 0;      
        core->issued_rd = 0;
        core->issued_wr = 0;
      }
    }

    bool is_finished() override {
      bool all_cores_done = true;
      for (auto core : m_cores) {
        if (!core->is_finished()) {
          all_cores_done = false;
          break;
        }
      }
      
      // Print final statistics when finished
      bool mem_system_done = false;
      if (all_cores_done) {
        mem_system_done = m_memory_system->is_finished();
        if(mem_system_done)
          print_final_statistics();
      }
      
      return all_cores_done && mem_system_done;
    }

    void print_final_statistics() {
      static bool printed = false;
      if (printed) return;
      printed = true;
      
      m_logger->info("=== Final Statistics ===");
      m_logger->info("Total simulation cycles: {}", m_current_cycle);
      
      uint64_t total_reads = 0;
      uint64_t total_writes = 0;
      uint64_t total_completed_reads = 0;
      
      for (auto core : m_cores) {
        m_logger->info("Core {}: Reads={}, Writes={}, Completed_Reads={}, Avg_Read_Latency={:.2f} cycles, MSHR={}",
                      core->core_id,
                      core->total_read_requests,
                      core->total_write_requests,
                      core->completed_reads,
                      core->avg_read_latency(),
                      core->max_outstanding);
        
        total_reads += core->total_read_requests;
        total_writes += core->total_write_requests;
        total_completed_reads += core->completed_reads;
      }
      
      m_logger->info("Overall: Total_Reads={}, Total_Writes={}, Completed_Reads={}", 
                    total_reads, total_writes, total_completed_reads);
    }
};

}        // namespace Ramulator