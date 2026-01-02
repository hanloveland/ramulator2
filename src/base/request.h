#ifndef     RAMULATOR_BASE_REQUEST_H
#define     RAMULATOR_BASE_REQUEST_H

#include <vector>
#include <list>
#include <string>
#include <memory>

#include "base/base.h"

namespace Ramulator {

struct Request { 
  Addr_t    addr = -1;
  AddrVec_t addr_vec {};

  // Basic request id convention
  // 0 = Read, 1 = Write. The device spec defines all others
  struct Type {
    enum : int {
      Read = 0, 
      Write,
    };
  };

  int type_id = -1;    // An identifier for the type of the request
  int source_id = -1;  // An identifier for where the request is coming from (e.g., which core)

  int command = -1;             // The command that need to be issued to progress the request
  int final_command = -1;       // The final command that is needed to finish the request
  bool is_stat_updated = false; // Memory controller stats
  bool is_db_cmd = false;       // PRE-WR or POST-RD (only DDR5-PCH)
  bool is_actived = false;
  bool is_ndp_req = false;      // NDP-related Request
  int ndp_id = -1;              // NDP-related Request ID
  bool is_trace_core_req = false; // Request is from Trace_core 

  Clk_t arrive = -1;   // Clock cycle when the request arrive at the memory controller
  Clk_t depart = -1;   // Clock cycle when the request depart the memory controller

  std::array<int, 4> scratchpad = { 0 };    // A scratchpad for the request

  std::function<void(Request&)> callback;

  // void* m_payload = nullptr;    // Point to a generic payload
  // std::vector based payload variable 
  std::vector<uint64_t> m_payload; 
  
  Request(Addr_t addr, int type);
  Request(AddrVec_t addr_vec, int type);
  Request(Addr_t addr, int type, int source_id, std::function<void(Request&)> callback);
};


struct ReqBuffer {
  std::list<Request> buffer;
  size_t max_size = 32;


  using iterator = std::list<Request>::iterator;
  iterator begin() { return buffer.begin(); };
  iterator end() { return buffer.end(); };


  size_t size() const { return buffer.size(); }

  bool enqueue(const Request& request) {
    if (buffer.size() <= max_size) {
      buffer.push_back(request);
      return true;
    } else {
      return false;
    }
  }

  void remove(iterator it) {
    buffer.erase(it);
  }
};

struct Inst_Slot {
  bool valid = false;
  int opcode = -1;
  int opsize = -1;
  int id = -1;
  int bg = -1;
  int bk = -1;
  int cnt = 0;
  int loop_cnt = -1;
  int jump_pc = -1;

  Inst_Slot() : 
    valid(false), opcode(-1), opsize(-1), id(-1), bg(-1), bk(-1), cnt(0), loop_cnt(-1), jump_pc(-1) {};

  Inst_Slot(bool is_valid, int _opcode, int _opsize, int _id, int _bg, int _bk, int _loop, int _pc) : 
    valid(is_valid), opcode(_opcode), opsize(_opsize), id(_id), bg(_bg), bk(_bk), cnt(0), loop_cnt(_loop), jump_pc(_pc)  {};
};

struct AccInst_Slot {
  bool valid = false;
  int opcode = -1;
  int opsize = -1;
  int ch = -1;
  int pch = -1;
  int bg = -1;
  int bk = -1;
  int row = -1;
  int col = -1;
  int id = -1;
  int etc = -1;
  int cnt = 0;

  AccInst_Slot() : 
    valid(false), cnt(0) {};

    AccInst_Slot(bool is_valid, int _opcode, int _opsize, int _ch, int _pch, int _bg,
                 int _bk, int _row, int _col, int _id, int _etc) : 
    valid(is_valid), opcode(_opcode), opsize(_opsize), ch(_ch), pch(_pch), bg(_bg), 
    bk(_bk), row(_row), col(_col), id(_id), etc(_etc), cnt(0)  {};
};

}        // namespace Ramulator


#endif   // RAMULATOR_BASE_REQUEST_H