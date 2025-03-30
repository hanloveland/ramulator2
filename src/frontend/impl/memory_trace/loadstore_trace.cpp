#include <filesystem>
#include <iostream>
#include <fstream>

#include "frontend/frontend.h"
#include "base/exception.h"

namespace Ramulator {

namespace fs = std::filesystem;

class LoadStoreTrace : public IFrontEnd, public Implementation {
  RAMULATOR_REGISTER_IMPLEMENTATION(IFrontEnd, LoadStoreTrace, "LoadStoreTrace", "Load/Store memory address trace.")

  private:
    struct Trace {
      bool is_write;
      Addr_t addr;
      std::vector<uint64_t> payload;
    };
    std::vector<Trace> m_trace;

    size_t m_trace_length = 0;
    size_t m_curr_trace_idx = 0;

    size_t m_trace_count = 0;

    Logger_t m_logger;

  public:
    void init() override {
      std::string trace_path_str = param<std::string>("path").desc("Path to the load store trace file.").required();
      m_clock_ratio = param<uint>("clock_ratio").required();

      m_logger = Logging::create_logger("LoadStoreTrace");
      m_logger->info("Loading trace file {} ...", trace_path_str);
      init_trace(trace_path_str);
      m_logger->info("Loaded {} lines.", m_trace.size());
    };


    void tick() override {
      if(!(m_trace_count >= m_trace_length)) {
        const Trace& t = m_trace[m_curr_trace_idx];
        Request req = Request(t.addr, t.is_write ? Request::Type::Write : Request::Type::Read);
        
        if(t.is_write && !t.payload.empty()) {
          for(uint32_t i=0;i<t.payload.size();i++) {
            req.m_payload.push_back(t.payload[i]);
          }
        }
        bool request_sent = m_memory_system->send(req);
        if (request_sent) {
          m_curr_trace_idx = (m_curr_trace_idx + 1) % m_trace_length;
          m_trace_count++;
        }
      }
    };


  private:
    void init_trace(const std::string& file_path_str) {
      fs::path trace_path(file_path_str);
      if (!fs::exists(trace_path)) {
        throw ConfigurationError("Trace {} does not exist!", file_path_str);
      }

      std::ifstream trace_file(trace_path);
      if (!trace_file.is_open()) {
        throw ConfigurationError("Trace {} cannot be opened!", file_path_str);
      }

      std::string line;
      while (std::getline(trace_file, line)) {
        Trace t;
        std::vector<std::string> tokens;
        tokenize(tokens, line, " ");

        // TODO: Add line number here for better error messages
        if (!(tokens.size() == 2 || tokens.size() == 10)) {
          throw ConfigurationError("Trace {} format invalid!", file_path_str);
        }

        bool is_write = false; 
        if (tokens[0] == "LD") {
          is_write = false;
          if(tokens.size() != 2) {
            throw ConfigurationError("Trace {} format invalid!", file_path_str);
          }
        } else if (tokens[0] == "ST") {
          is_write = true;
          if(tokens.size() != 10) {
            throw ConfigurationError("Trace {} format invalid!", file_path_str);
          }          
        } else {
          throw ConfigurationError("Trace {} format invalid!", file_path_str);
        }

        Addr_t addr = -1;
        if (tokens[1].compare(0, 2, "0x") == 0 | tokens[1].compare(0, 2, "0X") == 0) {
          addr = std::stoll(tokens[1].substr(2), nullptr, 16);
        } else {
          addr = std::stoll(tokens[1]);
        }

        t.is_write = is_write;
        t.addr     = addr;
        if(is_write) {
          if(tokens.size() == 10) {
            for(uint32_t i=0;i<8;i++) {
              if (tokens[i+2].compare(0, 2, "0x") == 0 | tokens[i+2].compare(0, 2, "0X") == 0) {
                t.payload.push_back(std::stoll(tokens[i+2].substr(2), nullptr, 16));
              } else {
                t.payload.push_back(std::stoll(tokens[i+2]));
              }              
            }
          } 
        }
        m_trace.push_back(t);
      }

      trace_file.close();

      m_trace_length = m_trace.size();
    };

    // TODO: FIXME
    bool is_finished() override {
      // return m_trace_count >= m_trace_length; 
      return m_memory_system->is_finished();
    };
};

}        // namespace Ramulator