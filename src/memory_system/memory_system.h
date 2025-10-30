#ifndef     RAMULATOR_MEMORYSYSTEM_MEMORY_H
#define     RAMULATOR_MEMORYSYSTEM_MEMORY_H

#include <map>
#include <vector>
#include <string>
#include <functional>

#include "base/base.h"
#include "frontend/frontend.h"

#include <fstream>

namespace Ramulator {

class IMemorySystem : public TopLevel<IMemorySystem> {
  RAMULATOR_REGISTER_INTERFACE(IMemorySystem, "MemorySystem", "Memory system interface (e.g., communicates between processor and memory controller).")

  friend class Factory;

  protected:
    IFrontEnd* m_frontend;
    uint m_clock_ratio = 1;
    std::string output_path = "ramulator2_simulation_result.yaml";
    std::string trace_path = "ramulator2.trace";
    bool use_gem5_frontend = false;
    int total_memory_capacity = 0;
    std::ofstream tout;
    
  public:
    virtual void connect_frontend(IFrontEnd* frontend) { 
      m_frontend = frontend; 
      m_impl->setup(frontend, this);
      for (auto component : m_components) {
        component->setup(frontend, this);
      }
    };

    virtual void finalize() { 
      for (auto component : m_components) {
        component->finalize();
      }

      mem_sys_finalize();
      YAML::Emitter emitter;
      emitter << YAML::BeginMap;
      m_impl->print_stats(emitter);
      emitter << YAML::EndMap;
      if(use_gem5_frontend) {
        // Simulation Result Dump to file
        std::ofstream fout(output_path);
        fout << emitter.c_str();
      } else {
        std::cout << emitter.c_str() << std::endl;
      }
    };

    /**
     * @brief         Tries to send the request to the memory system
     * 
     * @param    req      The request
     * @return   true     Request is accepted by the memory system.
     * @return   false    Request is rejected by the memory system, maybe the memory controller is full?
     */
    virtual bool send(Request req) = 0;

    /**
     * @brief         Ticks the memory system
     * 
     */
    virtual void tick() = 0;

    /**
     * @brief    Returns 
     * 
     * @return   int 
     */
    int get_clock_ratio() { return m_clock_ratio; };

    // /**
    //  * @brief    Get the integer id of the request type from the memory spec
    //  * 
    //  */
    // virtual const SpecDef& get_supported_requests() = 0;

    virtual float get_tCK() { return -1.0f; };

    virtual void set_output_path(std::string output_path_) {
      output_path = output_path_;
      trace_path = output_path_ + std::string(".trace");
      tout.open(trace_path);
    };

    virtual void set_use_gem5_frontend() {
      use_gem5_frontend = true;
    };    

    virtual int check_dram_capcity(int dram_capacity) {
      // Check DRAM Capacity is same with configuration 
      if((total_memory_capacity != 0) && (dram_capacity == total_memory_capacity)) return 1;
      else return 0;
    }

    // Check All Buffers are Empty to finish simulation 
    virtual bool is_finished() = 0;

    // finalize memory system itself 
    virtual void mem_sys_finalize() = 0;
};

}        // namespace Ramulator


#endif   // RAMULATOR_MEMORYSYSTEM_MEMORY_H