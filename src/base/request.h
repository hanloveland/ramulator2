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
  bool is_host_req = false; // Request is from Host Processor 

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

/**
 * NMAInst_Slot: Unified NMA Instruction for AsyncDIMM (HPCA 2025)
 *
 * Merges DBX-DIMM's separate AccInst_Slot (memory access) and Inst_Slot (computation)
 * into a single 64-bit instruction. NMA MC handles both computation and memory access.
 *
 * 64-bit Encoding:
 *   [63:58] comp_opcode  (6b)  — Computation opcode (same as DBX-DIMM Inst_Slot opcodes)
 *   [57:51] opsize       (7b)  — Number of column accesses
 *   [50:48] bg           (3b)  — Bankgroup
 *   [47:46] bk           (2b)  — Bank
 *   [45:28] row          (18b) — Row address
 *   [27:21] col          (7b)  — Start column
 *   [20:18] id           (3b)  — Instruction group ID
 *   [17:12] reserved     (6b)
 *   [11:0]  etc          (12b) — LOOP: [11:6]=loop_cnt, [5:0]=jump_pc
 *
 * comp_opcode → Memory Access Mapping:
 *   0  LOAD       RD × opsize        DRAM → NMA buffer
 *   1  LOAD_ADD   RD × opsize        DRAM → buffer, ADD
 *   2  LOAD_MUL   RD × opsize        DRAM → buffer, MUL
 *   3  ADD        (none)             Buffer-internal, latency only
 *   4  MUL        (none)             Buffer-internal, latency only
 *   5  V_RED      RD × opsize        Vector reduction
 *   6  S_RED      RD × opsize        Scalar reduction
 *   7  MAC        RD × opsize        Multiply-accumulate
 *   8  SCALE_ADD  RD × opsize        Scale and add
 *  16  T_ADD      RD × opsize×2      Transposed ADD (double access)
 *  17  T_MUL      RD × opsize×2      Transposed MUL
 *  18  T_V_RED    RD × opsize×2      Transposed V_RED
 *  19  T_S_RED    RD × opsize×2      Transposed S_RED
 *  20  T_MAC      RD × opsize×2      Transposed MAC
 *  32  WBD        WR × opsize        NMA buffer → DRAM
 *  48  BARRIER    (none)             Wait for all outstanding requests
 *  49  EXIT       (none)             NMA execution complete
 *  52  LOOP       (none)             etc[11:6]=loop_cnt, etc[5:0]=jump_pc
 */
struct NMAInst_Slot {
  // comp_opcode constants
  enum CompOpcode : int {
    LOAD      = 0,
    LOAD_ADD  = 1,
    LOAD_MUL  = 2,
    ADD       = 3,
    MUL       = 4,
    V_RED     = 5,
    S_RED     = 6,
    MAC       = 7,
    SCALE_ADD = 8,
    T_ADD     = 16,
    T_MUL     = 17,
    T_V_RED   = 18,
    T_S_RED   = 19,
    T_MAC     = 20,
    WBD       = 32,
    BARRIER   = 48,
    EXIT      = 49,
    LOOP      = 52,
  };

  // Memory access type derived from comp_opcode
  enum class MemAccessType {
    READ,       // RD × opsize (or opsize×2 for transposed)
    WRITE,      // WR × opsize
    NONE,       // Compute-only or control
    CONTROL,    // BARRIER, EXIT, LOOP
  };

  bool valid = false;
  int comp_opcode = -1;
  int opsize = -1;
  int bg = -1;
  int bk = -1;
  int row = -1;
  int col = -1;
  int id = -1;
  int etc = -1;
  int cnt = 0;         // Dual-use:
                       //   Memory access instructions: current access count (for address gen)
                       //   LOOP instruction: cached program end index (loop body upper bound)
                       //     - Set to (m_nma_pc - 1) on first LOOP execution
                       //     - Reused on subsequent iterations to avoid m_nma_pc drift
  int loop_cnt = -1;   // Decoded from etc[11:6]
  int jump_pc = -1;    // Decoded from etc[5:0]

  NMAInst_Slot() : valid(false), cnt(0) {};

  NMAInst_Slot(bool is_valid, int _comp_opcode, int _opsize, int _bg, int _bk,
               int _row, int _col, int _id, int _etc) :
    valid(is_valid), comp_opcode(_comp_opcode), opsize(_opsize),
    bg(_bg), bk(_bk), row(_row), col(_col), id(_id), etc(_etc), cnt(0) {
    // Decode LOOP fields from etc
    if (_comp_opcode == LOOP) {
      loop_cnt = (_etc >> 6) & 0x3F;
      jump_pc  = _etc & 0x3F;
    }
  };

  /**
   * Get the memory access type for this instruction's comp_opcode.
   */
  MemAccessType get_mem_access_type() const {
    switch (comp_opcode) {
      case LOAD: case LOAD_ADD: case LOAD_MUL:
      case V_RED: case S_RED: case MAC: case SCALE_ADD:
      case T_ADD: case T_MUL: case T_V_RED: case T_S_RED: case T_MAC:
        return MemAccessType::READ;
      case WBD:
        return MemAccessType::WRITE;
      case ADD: case MUL:
        return MemAccessType::NONE;
      case BARRIER: case EXIT: case LOOP:
        return MemAccessType::CONTROL;
      default:
        return MemAccessType::NONE;
    }
  }

  /**
   * Get total number of DRAM accesses for this instruction.
   * Transposed opcodes (16-20) require double the accesses.
   */
  int get_total_accesses() const {
    if (get_mem_access_type() == MemAccessType::NONE ||
        get_mem_access_type() == MemAccessType::CONTROL) {
      return 0;
    }
    bool is_transposed = (comp_opcode >= T_ADD && comp_opcode <= T_MAC);
    return is_transposed ? opsize * 2 : opsize;
  }

  /**
   * Check if this is a transposed operation (double memory access).
   */
  bool is_transposed() const {
    return (comp_opcode >= T_ADD && comp_opcode <= T_MAC);
  }

  /**
   * Check if this is a control instruction (BARRIER, EXIT, LOOP).
   */
  bool is_control() const {
    return get_mem_access_type() == MemAccessType::CONTROL;
  }

  /**
   * Check if this is a compute-only instruction (no memory access).
   */
  bool is_compute_only() const {
    return get_mem_access_type() == MemAccessType::NONE;
  }
};

}        // namespace Ramulator


#endif   // RAMULATOR_BASE_REQUEST_H