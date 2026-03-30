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
  bool wildcard = false;  // bit[42]: 1=match by id only (ignore bg/bk), 0=exact 3-field
  int cnt = 0;
  int loop_cnt = -1;
  int jump_pc = -1;

  Inst_Slot() :
    valid(false), opcode(-1), opsize(-1), id(-1), bg(-1), bk(-1), wildcard(false),
    cnt(0), loop_cnt(-1), jump_pc(-1) {};

  Inst_Slot(bool is_valid, int _opcode, int _opsize, int _id, int _bg, int _bk,
            bool _wildcard, int _loop, int _pc) :
    valid(is_valid), opcode(_opcode), opsize(_opsize), id(_id), bg(_bg), bk(_bk),
    wildcard(_wildcard), cnt(0), loop_cnt(_loop), jump_pc(_pc) {};
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
  int mode = 0;    // 0=Direct, 1=Undirect (Phase 2)
  int etc = -1;
  int cnt = 0;

  AccInst_Slot() :
    valid(false), cnt(0) {};

    AccInst_Slot(bool is_valid, int _opcode, int _opsize, int _ch, int _pch, int _bg,
                 int _bk, int _row, int _col, int _id, int _mode, int _etc) :
    valid(is_valid), opcode(_opcode), opsize(_opsize), ch(_ch), pch(_pch), bg(_bg),
    bk(_bk), row(_row), col(_col), id(_id), mode(_mode), etc(_etc), cnt(0)  {};
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
 *   [11:0]  etc          (12b) — LOOP uses row/col instead: row=loop_cnt(18b), col=jump_pc(7b)
 *
 * comp_opcode → Memory Access / Compute Mapping:
 *
 *   === READ type: X operand fetched from DRAM (RD_L × opsize) ===
 *   0  LOAD       Z = X              DRAM → NMA buffer
 *   1  LOAD_ADD   Z = X + a          DRAM → buffer + scalar const
 *   2  LOAD_MUL   Z = X * a          DRAM → buffer × scalar const
 *   3  ADD        Z = X + Y          X from DRAM, Y from VMA buffer
 *   4  MUL        Z = X * Y          X from DRAM, Y from VMA buffer
 *   5  V_RED      Z += X             Vector reduction (X from DRAM)
 *   6  S_RED      z = SUM(X)         Scalar reduction (X from DRAM)
 *   7  MAC        Z += X * Y         X from DRAM, Y from VMA buffer
 *   8  SCALE_ADD  Z = aX + Y         X from DRAM, Y from VMA buffer
 *
 *   === NONE type: VMA-internal compute, no DRAM access ===
 *   === Latency = base_latency × opsize NMA ticks ===
 *  16  T_ADD      Z = Y + Y2         Both operands from VMA buffer
 *  17  T_MUL      Z = Y * Y2         Both operands from VMA buffer
 *  18  T_V_RED    Z += Y             Y from VMA buffer
 *  19  T_S_RED    z = SUM(Y)         Y from VMA buffer
 *  20  T_MAC      Z += Y * Y2        Both operands from VMA buffer
 *
 *   === WRITE type: VMA buffer → DRAM (WR_L × opsize) ===
 *  32  WBD        DRAM = Y           Write-back from VMA buffer to DRAM
 *
 *   === CONTROL type: no DRAM access, no compute ===
 *  48  BARRIER    —                  Wait for all outstanding requests to drain
 *  49  EXIT       —                  NMA execution complete → NMA_DONE
 *  52  LOOP       —                  etc[11:6]=loop_cnt, etc[5:0]=jump_pc
 *                                    In-place counter: cnt < loop_cnt → jump, else fall through
 *  53  SET_BASE   —                  base_reg[id] = row (set base address register)
 *  54  INC_BASE   —                  base_reg[id] += row (increment by stride, 18-bit)
 *
 * Base Address Register:
 *   8 registers indexed by id field (3 bits, 0-7).
 *   READ/WRITE effective_row = base_reg[id] + inst.row
 *   If SET_BASE never called: base_reg[*] = 0 → effective_row = inst.row (backward compatible)
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
    SET_BASE  = 53,   // Set base address register: base_reg[id] = row
    INC_BASE  = 54,   // Increment base address register: base_reg[id] += opsize (stride)
  };

  // Memory access type derived from comp_opcode
  //
  // Data operand definitions:
  //   X   : Vector from DRAM            → DRAM RD required
  //   Y   : Vector from NMA VMA buffer  → no DRAM access (already loaded)
  //   Y2  : Vector from NMA VMA buffer  → no DRAM access (already loaded)
  //   Z   : Vector result → written to VMA buffer (no DRAM write)
  //   z   : Scalar result → written to VMA buffer (no DRAM write)
  //   a   : Scalar constant embedded in instruction
  //
  // NOTE: The "T_" prefix means both operands come from VMA (Y, Y2).
  //       No DRAM access is needed; these are VMA-internal compute operations.
  enum class MemAccessType {
    READ,       // RD × opsize — X is fetched from DRAM (bg/bk/row/col in instruction)
    WRITE,      // WR × opsize — Y (VMA) written back to DRAM (WBD only)
    NONE,       // VMA-internal or scalar-const compute; no DRAM access
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
  int cnt = 0;         // LOOP: in-place iteration counter (DBX-DIMM style)
                       //   cnt < loop_cnt → cnt++, pc = jump_pc (loop back)
                       //   cnt >= loop_cnt → cnt = 0, pc++ (fall through)
  int loop_cnt = -1;   // LOOP: decoded from row field (18b, max 262143)
  int jump_pc = -1;    // LOOP: decoded from col field (7b, max 127)

  NMAInst_Slot() : valid(false), cnt(0) {};

  NMAInst_Slot(bool is_valid, int _comp_opcode, int _opsize, int _bg, int _bk,
               int _row, int _col, int _id, int _etc) :
    valid(is_valid), comp_opcode(_comp_opcode), opsize(_opsize),
    bg(_bg), bk(_bk), row(_row), col(_col), id(_id), etc(_etc), cnt(0) {
    // Decode LOOP fields from row/col (LOOP has no memory access, reuses these fields)
    if (_comp_opcode == LOOP) {
      loop_cnt = _row;  // row(18b): max 262143 iterations
      jump_pc  = _col;  // col(7b): max 127 jump target
    }
  };

  /**
   * Get the memory access type for this instruction's comp_opcode.
   *
   * Opcodes that read X from DRAM → READ:
   *   LOAD       Z = X
   *   LOAD_ADD   Z = X + a      (a: scalar const, X: DRAM)
   *   LOAD_MUL   Z = X * a
   *   ADD        Z = X + Y      (X: DRAM, Y: VMA)
   *   MUL        Z = X * Y
   *   V_RED      Z += X
   *   S_RED      z = SUM(X)
   *   MAC        Z += X * Y
   *   SCALE_ADD  Z = aX + Y
   *
   * Opcodes that write Y (VMA) back to DRAM → WRITE:
   *   WBD        X = Y          (Y: VMA → DRAM)
   *
   * Opcodes that use only VMA data (Y, Y2); no DRAM access → NONE:
   *   T_ADD      Z = Y + Y2
   *   T_MUL      Z = Y * Y2
   *   T_V_RED    Z += Y
   *   T_S_RED    z = SUM(Y)
   *   T_MAC      Z += Y * Y2
   *
   * Control → CONTROL:
   *   BARRIER, EXIT, LOOP
   */
  MemAccessType get_mem_access_type() const {
    switch (comp_opcode) {
      // All non-T ops that read X from DRAM
      case LOAD: case LOAD_ADD: case LOAD_MUL:
      case ADD:  case MUL:
      case V_RED: case S_RED: case MAC: case SCALE_ADD:
        return MemAccessType::READ;

      // Write-back: Y (VMA) → DRAM
      case WBD:
        return MemAccessType::WRITE;

      // T_* ops: both operands in VMA; no DRAM access
      case T_ADD: case T_MUL: case T_V_RED: case T_S_RED: case T_MAC:
        return MemAccessType::NONE;

      // Control
      case BARRIER: case EXIT: case LOOP: case SET_BASE: case INC_BASE:
        return MemAccessType::CONTROL;

      default:
        return MemAccessType::NONE;
    }
  }

  /**
   * Get total number of DRAM accesses for this instruction.
   * All READ/WRITE ops access DRAM opsize times (no 2× multiplier).
   * T_* ops (NONE) and CONTROL ops require 0 DRAM accesses.
   */
  int get_total_accesses() const {
    auto type = get_mem_access_type();
    if (type == MemAccessType::NONE || type == MemAccessType::CONTROL)
      return 0;
    return opsize + 1;  // 0-indexed: opsize=127 means 128 accesses
  }

  /**
   * Check if this is a T_* (transposed) instruction.
   * T_* ops operate entirely on VMA data (Y, Y2); no DRAM access.
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
   * Check if this is a VMA-internal compute instruction (no DRAM access).
   * Includes T_* ops (VMA-to-VMA) and scalar compute with no X from DRAM.
   */
  bool is_compute_only() const {
    return get_mem_access_type() == MemAccessType::NONE;
  }
};

}        // namespace Ramulator


#endif   // RAMULATOR_BASE_REQUEST_H