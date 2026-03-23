#include "dram/dram.h"
#include "dram/lambdas.h"

#define DEBUG_POWER

namespace Ramulator {

class DDR5AsyncDIMM : public IDRAM, public Implementation {
  RAMULATOR_REGISTER_IMPLEMENTATION(IDRAM, DDR5AsyncDIMM, "DDR5-AsyncDIMM", "DDR5 AsyncDIMM Device Model (HPCA 2025)")
  private:
    int m_RH_radius = -1;


  public:
    inline static const std::map<std::string, Organization> org_presets = {
      //   name         density   DQ   Ch Ra Bg Ba   Ro     Co
      {"DDR5_8Gb_x4",   {8<<10,   4,  {1, 1, 8, 2, 1<<16, 1<<11}}},
      {"DDR5_8Gb_x8",   {8<<10,   8,  {1, 1, 8, 2, 1<<16, 1<<10}}},
      {"DDR5_8Gb_x16",  {8<<10,   16, {1, 1, 4, 2, 1<<16, 1<<10}}},
      {"DDR5_16Gb_x4",  {16<<10,  4,  {1, 1, 8, 4, 1<<16, 1<<11}}},
      {"DDR5_16Gb_x8",  {16<<10,  8,  {1, 1, 8, 4, 1<<16, 1<<10}}},
      {"DDR5_16Gb_x16", {16<<10,  16, {1, 1, 4, 4, 1<<16, 1<<10}}},
      {"DDR5_32Gb_x4",  {32<<10,  4,  {1, 1, 8, 4, 1<<17, 1<<11}}},
      {"DDR5_32Gb_x8",  {32<<10,  8,  {1, 1, 8, 4, 1<<17, 1<<10}}},
      {"DDR5_32Gb_x16", {32<<10,  16, {1, 1, 4, 4, 1<<17, 1<<10}}},
    };

    inline static const std::map<std::string, std::vector<int>> timing_presets = {
      //   name         rate   nBL  nCL nRCD   nRP  nRAS   nRC   nWR  nRTP nCWL nPPD nCCDS nCCDS_WR nCCDS_WTR nCCDL nCCDL_WR nCCDL_WTR nRRDS nRRDL nFAW nRFC1 nRFC2 nRFCsb nREFI nREFSBRD nRFM1 nRFM2 nRFMsb nDRFMab nDRFMsb nCS, tCK_ps
      {"DDR5_3200AN",  {3200,   8,  24,  24,   24,   52,   75,   48,   12,  22,  2,    8,     8,     22+8+4,    8,     32,    22+8+16,   8,   -1,   -1,  -1,   -1,   -1,    -1,     30,    -1,   -1,   -1,     -1,     -1,    2,   625}},
      {"DDR5_3200BN",  {3200,   8,  26,  26,   26,   52,   77,   48,   12,  24,  2,    8,     8,     24+8+4,    8,     32,    24+8+16,   8,   -1,   -1,  -1,   -1,   -1,    -1,     30,    -1,   -1,   -1,     -1,     -1,    2,   625}},
      {"DDR5_3200C",   {3200,   8,  28,  28,   28,   52,   79,   48,   12,  26,  2,    8,     8,     26+8+4,    8,     32,    26+8+16,   8,   -1,   -1,  -1,   -1,   -1,    -1,     30,    -1,   -1,   -1,     -1,     -1,    2,   625}},
      {"DDR5_4800B",   {4800,   8,  40,  39,   39,   77,   116,  73,   19,  38,  2,    8,     8,     38+8+7,    8,     49,    38+8+25,   8,   -1,   -1,  -1,   -1,   -1,    -1,     30,    -1,   -1,   -1,     -1,     -1,    2,   416}},
      {"DDR5_6400AN",  {6400,   8,  46,  46,   46,   103,  149,  97,   25,  44,  2,    8,     8,     44+8+9,    8,     65,    44+8+33,   8,   -1,   -1,  -1,   -1,   -1,    -1,     30,    -1,   -1,   -1,     -1,     -1,    2,   312}},
      {"DDR5_7200AN",  {7200,   8,  52,  52,   52,   116,  168,  109,  28,  50,  2,    8,     8,     50+8+9,    10,    73,    50+8+37,   8,   -1,   -1,  -1,   -1,   -1,    -1,     30,    -1,   -1,   -1,     -1,     -1,    2,   277}},
      {"DDR5_8000AN",  {8000,   8,  56,  56,   56,   128,  184,  120,  30,  54,  4,    8,     8,     54+8+8,    20,    80,    54+8+40,   8,   -1,   -1,  -1,   -1,   -1,    -1,     30,    -1,   -1,   -1,     -1,     -1,    2,   250}},
      {"DDR5_8800AN",  {8800,   8,  62,  62,   62,   141,  204,  133,  34,  60,  4,    8,     8,     60+8+6,    23,    89,    60+8+45,   8,   -1,   -1,  -1,   -1,   -1,    -1,     30,    -1,   -1,   -1,     -1,     -1,    2,   227}},
    };

    inline static const std::map<std::string, std::vector<double>> voltage_presets = {
      //   name          VDD      VPP
      {"Default",       {1.1,     1.8}},
    };

    inline static const std::map<std::string, std::vector<double>> current_presets = {
      // name           IDD0  IDD2N   IDD3N   IDD4R   IDD4W   IDD5B   IPP0  IPP2N  IPP3N  IPP4R  IPP4W  IPP5B
      {"Default",       {60,   50,     55,     145,    145,    362,     3,    3,     3,     3,     3,     48}},
      {"DDR5_4800x4",   {103,  92,     142,    318,    345,    277,     8,    7,     7,     9,     36,    28}},
    };
  /************************************************
   *                Organization
   ***********************************************/
    const int m_internal_prefetch_size = 16;

    inline static constexpr ImplDef m_levels = {
      "channel", "rank", "bankgroup", "bank", "row", "column",
    };


  /************************************************
   *             Requests & Commands
   ***********************************************/
    inline static constexpr ImplDef m_commands = {
      // Standard DDR5 commands
      "ACT",
      "PRE", "PREA", "PREsb",
      "RD",  "WR",  "RDA",  "WRA",
      "REFab",  "REFsb", "REFab_end", "REFsb_end",
      "RFMab",  "RFMsb", "RFMab_end", "RFMsb_end",
      "DRFMab", "DRFMsb", "DRFMab_end", "DRFMsb_end",
      // AsyncDIMM offload commands (Host MC -> NMA MC via CA bus)
      "ACTO",   // Offload ACT: no bank state change on host side
      "PREO",   // Offload PRE: no bank state change on host side
      "RDO",    // Offload RD: no DQ bus usage, relaxed timing
      "WRO",    // Offload WR: uses DQ bus, relaxed timing (omit tWL)
      "REFO",   // Offload REF: no bank state change, NMA MC handles actual REFab
      // Return commands: RT_N returns N reads' data, DQ bus occupied N × tBL
      "RT1", "RT2", "RT3", "RT4", "RT5", "RT6", "RT7", "RT8",
      // NMA-Local commands: rank-local CA + rank-local data bus (no channel bus)
      "ACT_L",    // NMA MC → DRAM: activate (rank-local CA)
      "PRE_L",    // NMA MC → DRAM: precharge (rank-local CA)
      "RD_L",     // NMA MC → DRAM: read (rank-local CA + rank-local data)
      "WR_L",     // NMA MC → DRAM: write (rank-local CA + rank-local data)
      "PREA_L",   // NMA MC → DRAM: precharge all (rank-local CA)
      "REFab_L",  // NMA MC → DRAM: all-bank refresh (rank-local CA)
      "REFab_L_end", // Refresh completion marker
    };

    inline static const ImplLUT m_command_scopes = LUT (
      m_commands, m_levels, {
        // Standard DDR5
        {"ACT",   "row"},
        {"PRE",   "bank"},   {"PREA",   "rank"},   {"PREsb", "bank"},
        {"RD",    "column"}, {"WR",     "column"}, {"RDA",   "column"}, {"WRA",   "column"},
        {"REFab",  "rank"},  {"REFsb",  "bank"}, {"REFab_end",  "rank"},  {"REFsb_end",  "bank"},
        {"RFMab",  "rank"},  {"RFMsb",  "bank"}, {"RFMab_end",  "rank"},  {"RFMsb_end",  "bank"},
        {"DRFMab", "rank"},  {"DRFMsb", "bank"}, {"DRFMab_end", "rank"},  {"DRFMsb_end", "bank"},
        // AsyncDIMM offload commands
        {"ACTO",  "row"},     // Targets a specific row (like ACT)
        {"PREO",  "bank"},    // Targets a specific bank (like PRE)
        {"RDO",   "column"},  // Targets a specific column (like RD)
        {"WRO",   "column"},  // Targets a specific column (like WR)
        {"REFO",  "rank"},    // Targets a rank (like REFab)
        {"RT1",   "rank"}, {"RT2", "rank"}, {"RT3", "rank"}, {"RT4", "rank"},
        {"RT5",   "rank"}, {"RT6", "rank"}, {"RT7", "rank"}, {"RT8", "rank"},
        // NMA-Local commands (same scope as originals)
        {"ACT_L", "row"}, {"PRE_L", "bank"}, {"RD_L", "column"}, {"WR_L", "column"},
        {"PREA_L", "rank"}, {"REFab_L", "rank"}, {"REFab_L_end", "rank"},
      }
    );

    inline static const ImplLUT m_command_meta = LUT<DRAMCommandMeta> (
      m_commands, {
                      // open?   close?   access?  refresh?
        // Standard DDR5 commands
        {"ACT",         {true,   false,   false,   false}},
        {"PRE",         {false,  true,    false,   false}},
        {"PREA",        {false,  true,    false,   false}},
        {"PREsb",       {false,  true,    false,   false}},
        {"RD",          {false,  false,   true,    false}},
        {"WR",          {false,  false,   true,    false}},
        {"RDA",         {false,  true,    true,    false}},
        {"WRA",         {false,  true,    true,    false}},
        {"REFab",       {false,  false,   false,   true }},
        {"REFsb",       {false,  false,   false,   true }},
        {"REFab_end",   {false,  true,    false,   false}},
        {"REFsb_end",   {false,  true,    false,   false}},
        {"RFMab",       {false,  false,   false,   true }},
        {"RFMsb",       {false,  false,   false,   true }},
        {"RFMab_end",   {false,  true,    false,   false}},
        {"RFMsb_end",   {false,  true,    false,   false}},
        {"DRFMab",      {false,  false,   false,   true }},
        {"DRFMsb",      {false,  false,   false,   true }},
        {"DRFMab_end",  {false,  true,    false,   false}},
        {"DRFMsb_end",  {false,  true,    false,   false}},
        // AsyncDIMM offload commands: NOT open/close/access/refresh
        // These are pass-through commands that don't change host-side DRAM state
        {"ACTO",        {false,  false,   false,   false}},
        {"PREO",        {false,  false,   false,   false}},
        {"RDO",         {false,  false,   false,   false}},
        {"WRO",         {false,  false,   true,    false}},
        {"REFO",        {false,  false,   false,   false}},  // No bank state change on host side
        // RT_N: access=true (DQ bus used for data return)
        {"RT1", {false,false,true,false}}, {"RT2", {false,false,true,false}},
        {"RT3", {false,false,true,false}}, {"RT4", {false,false,true,false}},
        {"RT5", {false,false,true,false}}, {"RT6", {false,false,true,false}},
        {"RT7", {false,false,true,false}}, {"RT8", {false,false,true,false}},
        // NMA-Local: same meta as originals (bank state changes on DRAM model)
        //                  open?  close?  access? refresh?
        {"ACT_L",       {true,   false,  false,  false}},
        {"PRE_L",       {false,  true,   false,  false}},
        {"RD_L",        {false,  false,  false,  false}},  // access=false (rank-local data, not channel DQ)
        {"WR_L",        {false,  false,  false,  false}},  // access=false (rank-local data)
        {"PREA_L",      {false,  true,   false,  false}},
        {"REFab_L",     {false,  false,  false,  true }},
        {"REFab_L_end", {false,  true,   false,  false}},
      }
    );

    inline static constexpr ImplDef m_requests = {
      "read", "write",
      "all-bank-refresh", "same-bank-refresh",
      "rfm", "same-bank-rfm",
      "directed-rfm", "same-bank-directed-rfm",
      "open-row", "close-row",
      // AsyncDIMM offload requests
      "offload-read", "offload-write", "offload-refresh",
    };

    inline static const ImplLUT m_request_translations = LUT (
      m_requests, m_commands, {
        // Standard DDR5 request -> command
        {"read", "RD"}, {"write", "WR"},
        {"all-bank-refresh", "REFab"}, {"same-bank-refresh", "REFsb"},
        {"rfm", "RFMab"}, {"same-bank-rfm", "RFMsb"},
        {"directed-rfm", "DRFMab"}, {"same-bank-directed-rfm", "DRFMsb"},
        {"open-row", "ACT"}, {"close-row", "PRE"},
        // AsyncDIMM offload request -> command
        {"offload-read", "RDO"}, {"offload-write", "WRO"}, {"offload-refresh", "REFO"},
      }
    );

  /************************************************
   *                   Timing
   ***********************************************/
    inline static constexpr ImplDef m_timings = {
      "rate",
      "nBL", "nCL", "nRCD", "nRP", "nRAS", "nRC", "nWR", "nRTP", "nCWL",
      "nPPD",
      "nCCDS", "nCCDS_WR", "nCCDS_WTR",
      "nCCDL", "nCCDL_WR", "nCCDL_WTR",
      "nRRDS", "nRRDL",
      "nFAW",
      "nRFC1", "nRFC2", "nRFCsb", "nREFI", "nREFSBRD",
      "nRFM1", "nRFM2", "nRFMsb",
      "nDRFMab", "nDRFMsb",
      "nCS",
      "tCK_ps"
    };

  /************************************************
   *                   Power
   ***********************************************/
    inline static constexpr ImplDef m_voltages = {
      "VDD", "VPP"
    };

    inline static constexpr ImplDef m_currents = {
      "IDD0", "IDD2N", "IDD3N", "IDD4R", "IDD4W", "IDD5B",
      "IPP0", "IPP2N", "IPP3N", "IPP4R", "IPP4W", "IPP5B"
    };

    inline static constexpr ImplDef m_cmds_counted = {
      "ACT", "PRE", "RD", "WR", "REF", "RFM",
      // NMA-Local commands (rank-local bus, same DRAM energy as originals)
      "ACT_L", "PRE_L", "RD_L", "WR_L", "REFab_L"
    };

  /************************************************
   *                 Node States
   ***********************************************/
    inline static constexpr ImplDef m_states = {
       "Opened", "Closed", "PowerUp", "N/A", "Refreshing"
    };

    inline static const ImplLUT m_init_states = LUT (
      m_levels, m_states, {
        {"channel",   "N/A"},
        {"rank",      "PowerUp"},
        {"bankgroup", "N/A"},
        {"bank",      "Closed"},
        {"row",       "Closed"},
        {"column",    "N/A"},
      }
    );

    // Not Used in DDR5-AsyncDIMM (only for compiling..)
    inline static constexpr ImplDef m_f_states = {
      "Opened", "Closed", "N/A"
    };

    // Not Used in DDR5-AsyncDIMM (only for compiling..)
    inline static const ImplLUT m_init_f_states = LUT (
      m_levels, m_states, {
        {"channel",         "N/A"},
        {"rank",            "N/A"},
        {"bankgroup",       "N/A"},
        {"bank",            "Closed"},
        {"row",             "Closed"},
        {"column",          "N/A"},
      }
    );

  public:
    struct Node : public DRAMNodeBase<DDR5AsyncDIMM> {
      Node(DDR5AsyncDIMM* dram, Node* parent, int level, int id) : DRAMNodeBase<DDR5AsyncDIMM>(dram, parent, level, id) {};
    };
    std::vector<Node*> m_channels;

    FuncMatrix<ActionFunc_t<Node>>  m_actions;
    FuncMatrix<PreqFunc_t<Node>>    m_preqs;
    FuncMatrix<RowhitFunc_t<Node>>  m_rowhits;
    FuncMatrix<RowopenFunc_t<Node>> m_rowopens;
    FuncMatrix<PowerFunc_t<Node>>   m_powers;

    double s_total_rfm_energy = 0.0;

    std::vector<size_t> s_total_rfm_cycles;

  /************************************************
   *                 RFM Related
   ***********************************************/
  public:
    int m_BRC = 2;


  public:
    void tick() override {
      m_clk++;

      // Check if there is any future action at this cycle
      for (int i = m_future_actions.size() - 1; i >= 0; i--) {
        auto& future_action = m_future_actions[i];
        if (future_action.clk == m_clk) {
          handle_future_action(future_action.cmd, future_action.addr_vec);
          m_future_actions.erase(m_future_actions.begin() + i);
        }
      }
    };

    void init() override {
      RAMULATOR_DECLARE_SPECS();
      set_organization();
      set_timing_vals();

      set_actions();
      set_preqs();
      set_rowhits();
      set_rowopens();
      set_powers();

      create_nodes();
    };

    void issue_command(int command, const AddrVec_t& addr_vec) override {
      int channel_id = addr_vec[m_levels["channel"]];
      m_channels[channel_id]->update_timing(command, addr_vec, m_clk);
      m_channels[channel_id]->update_powers(command, addr_vec, m_clk);
      m_channels[channel_id]->update_states(command, addr_vec, m_clk);

      // Check if the command requires future action
      check_future_action(command, addr_vec);
    };

    void check_future_action(int command, const AddrVec_t& addr_vec) {
      switch (command) {
        case m_commands("REFab"):
          m_future_actions.push_back({command, addr_vec, m_clk + m_timing_vals("nRFC1") - 1});
          break;
        case m_commands("REFsb"):
          m_future_actions.push_back({command, addr_vec, m_clk + m_timing_vals("nRFCsb") - 1});
          break;
        case m_commands("RFMab"):
          m_future_actions.push_back({command, addr_vec, m_clk + m_timing_vals("nRFM1") - 1});
          break;
        case m_commands("RFMsb"):
          m_future_actions.push_back({command, addr_vec, m_clk + m_timing_vals("nRFMsb") - 1});
          break;
        case m_commands("DRFMab"):
          m_future_actions.push_back({command, addr_vec, m_clk + m_timing_vals("nDRFMab") - 1});
          break;
        case m_commands("DRFMsb"):
          m_future_actions.push_back({command, addr_vec, m_clk + m_timing_vals("nDRFMsb") - 1});
          break;
        case m_commands("REFab_L"):
          m_future_actions.push_back({command, addr_vec, m_clk + m_timing_vals("nRFC1") - 1});
          break;
        default:
          // Other commands (including offload commands) do not require future actions
          break;
      }
    }

    void handle_future_action(int command, const AddrVec_t& addr_vec) {
      int channel_id = addr_vec[m_levels["channel"]];
      switch (command) {
        case m_commands("REFab"):
          m_channels[channel_id]->update_powers(m_commands("REFab_end"), addr_vec, m_clk);
          m_channels[channel_id]->update_states(m_commands("REFab_end"), addr_vec, m_clk);
          break;
        case m_commands("REFsb"):
          m_channels[channel_id]->update_powers(m_commands("REFsb_end"), addr_vec, m_clk);
          m_channels[channel_id]->update_states(m_commands("REFsb_end"), addr_vec, m_clk);
          break;
        case m_commands("RFMab"):
          m_channels[channel_id]->update_powers(m_commands("RFMab_end"), addr_vec, m_clk);
          m_channels[channel_id]->update_states(m_commands("RFMab_end"), addr_vec, m_clk);
          break;
        case m_commands("RFMsb"):
          m_channels[channel_id]->update_powers(m_commands("RFMsb_end"), addr_vec, m_clk);
          m_channels[channel_id]->update_states(m_commands("RFMsb_end"), addr_vec, m_clk);
          break;
        case m_commands("DRFMab"):
          m_channels[channel_id]->update_powers(m_commands("DRFMab_end"), addr_vec, m_clk);
          m_channels[channel_id]->update_states(m_commands("DRFMab_end"), addr_vec, m_clk);
          break;
        case m_commands("DRFMsb"):
          m_channels[channel_id]->update_powers(m_commands("DRFMsb_end"), addr_vec, m_clk);
          m_channels[channel_id]->update_states(m_commands("DRFMsb_end"), addr_vec, m_clk);
          break;
        case m_commands("REFab_L"):
          m_channels[channel_id]->update_powers(m_commands("REFab_L_end"), addr_vec, m_clk);
          m_channels[channel_id]->update_states(m_commands("REFab_L_end"), addr_vec, m_clk);
          break;
        default:
          break;
      }
    };

    int get_preq_command(int command, const AddrVec_t& addr_vec) override {
      int channel_id = addr_vec[m_levels["channel"]];
      return m_channels[channel_id]->get_preq_command(command, addr_vec, m_clk);
    };

    int get_preq_command_refresh_ch(int command, const AddrVec_t& addr_vec) override {
      return -1;
    };

    int get_preq_pre_command(int command, const AddrVec_t& addr_vec) override {
      return -1;
    };

    bool check_ready(int command, const AddrVec_t& addr_vec) override {
      int channel_id = addr_vec[m_levels["channel"]];
      return m_channels[channel_id]->check_ready(command, addr_vec, m_clk);
    };

    bool check_rowbuffer_hit(int command, const AddrVec_t& addr_vec) override {
      int channel_id = addr_vec[m_levels["channel"]];
      return m_channels[channel_id]->check_rowbuffer_hit(command, addr_vec, m_clk);
    };

    bool check_node_open(int command, const AddrVec_t& addr_vec) override {
      int channel_id = addr_vec[m_levels["channel"]];
      return m_channels[channel_id]->check_node_open(command, addr_vec, m_clk);
    };

    bool check_dram_refrsehing() override {
      return false;
    };

    bool check_ch_refrsehing(const AddrVec_t& addr_vec) override {
      return false;
    };

    bool check_pch_refrsehing_by_idx(int ch_idx, int pch_idx) override {
      return false;
    };


    int get_db_fetch_per_pch(const AddrVec_t& addr_vec) override {
      return 0;
    }

    int get_db_rd_fetch_per_pch(const AddrVec_t& addr_vec) override {
      return 0;
    }

    int get_db_wr_fetch_per_pch(const AddrVec_t& addr_vec) override {
      return 0;
    }


    void set_db_fetch_per_pch(const AddrVec_t& addr_vec, int value, int rd_value, int wr_value) override {
    };


    void reset_need_be_open_per_bank(u_int32_t channel_id) override {

    };

    void set_need_be_open_per_bank(const AddrVec_t& addr_vec) override {
    };

    bool get_need_be_open_per_bank(const AddrVec_t& addr_vec) override {
      return false;
    };

    bool get_use_pch() override {
      return false;
    };

    // Print Request
    void print_req(Request& req) {
      std::cout<<"Final["<<m_commands(req.final_command)<<"] Current ["<<m_commands(req.command)
                         <<"CH["<<req.addr_vec[m_levels["channel"]]
                         <<"]RK["<<req.addr_vec[m_levels["rank"]]
                         <<"]BG["<<req.addr_vec[m_levels["bankgroup"]]
                         <<"]BK["<<req.addr_vec[m_levels["bank"]]
                         <<"]RO["<<req.addr_vec[m_levels["row"]]
                         <<"]CO["<<req.addr_vec[m_levels["column"]]<<"]"<<std::endl;
    };

    void set_high_pri_prefetch(u_int32_t channel_id, u_int32_t pseudo_channel_id) {
    };
    void reset_high_pri_prefetch(u_int32_t channel_id, u_int32_t pseudo_channel_id) {
    };

    bool get_pri_prefetch(u_int32_t channel_id, u_int32_t pseudo_channel_id) {
      return false;
    };

    int get_db_fetch_mode(u_int32_t channel_id, u_int32_t pseudo_channel_id) {
      return 0;
    }

    bool get_use_prefetch() override {
      return false;
    };

    bool is_ndp_access(const AddrVec_t& addr_vec) override {
      return false;
    }

    void issue_ndp_command(int command, const AddrVec_t& addr_vec, int thread_id, const std::vector<uint64_t> payload) override {
    }

    int get_ndp_response(int ch_id, int pch_id) override {
      return 0;
    }

    bool is_ndp_issuable(int ndp_status) override {
      return false;
    }

    int get_dq_scaling() override {
        return 1;
    }

    int get_io_boost() override {
      return 1;
    }

    int get_pch_error() override {
      return -1;
    }

  private:
    void set_organization() {
      // Channel width
      m_channel_width = param_group("org").param<int>("channel_width").default_val(32);

      m_parity_width = param_group("org").param<int>("parity_width").default_val(8);

      // Organization
      m_organization.count.resize(m_levels.size(), -1);

      // Load organization preset if provided
      if (auto preset_name = param_group("org").param<std::string>("preset").optional()) {
        if (org_presets.count(*preset_name) > 0) {
          m_organization = org_presets.at(*preset_name);
        } else {
          throw ConfigurationError("Unrecognized organization preset \"{}\" in {}!", *preset_name, get_name());
        }
      }

      // Override the preset with any provided settings
      if (auto dq = param_group("org").param<int>("dq").optional()) {
        m_organization.dq = *dq;
      }

      for (int i = 0; i < m_levels.size(); i++){
        auto level_name = m_levels(i);
        if (auto sz = param_group("org").param<int>(level_name).optional()) {
          m_organization.count[i] = *sz;
        }
      }

      if (auto density = param_group("org").param<int>("density").optional()) {
        m_organization.density = *density;
      }

      // Sanity check: is the calculated chip density the same as the provided one?
      size_t _density = size_t(m_organization.count[m_levels["bankgroup"]]) *
                        size_t(m_organization.count[m_levels["bank"]]) *
                        size_t(m_organization.count[m_levels["row"]]) *
                        size_t(m_organization.count[m_levels["column"]]) *
                        size_t(m_organization.dq);
      _density >>= 20;
      if (m_organization.density != _density) {
        throw ConfigurationError(
            "Calculated {} chip density {} Mb does not equal the provided density {} Mb!",
            get_name(),
            _density,
            m_organization.density
        );
      }

      int num_channels = m_organization.count[m_levels["channel"]];
      int num_ranks = m_organization.count[m_levels["rank"]];
      s_total_rfm_cycles.resize(num_channels * num_ranks, 0);

      for (int r = 0; r < num_channels * num_ranks; r++) {
        register_stat(s_total_rfm_cycles[r]).name("total_rfm_cycles_rank{}", r);
      }
    };

    void set_timing_vals() {
      m_timing_vals.resize(m_timings.size(), -1);

      // Load timing preset if provided
      bool preset_provided = false;
      if (auto preset_name = param_group("timing").param<std::string>("preset").optional()) {
        if (timing_presets.count(*preset_name) > 0) {
          m_timing_vals = timing_presets.at(*preset_name);
          preset_provided = true;
        } else {
          throw ConfigurationError("Unrecognized timing preset \"{}\" in {}!", *preset_name, get_name());
        }
      }

      // Check for rate (in MT/s), and if provided, calculate and set tCK (in picosecond)
      if (auto dq = param_group("timing").param<int>("rate").optional()) {
        if (preset_provided) {
          throw ConfigurationError("Cannot change the transfer rate of {} when using a speed preset !", get_name());
        }
        m_timing_vals("rate") = *dq;
      }
      int tCK_ps = 1E6 / (m_timing_vals("rate") / 2);
      m_timing_vals("tCK_ps") = tCK_ps;

      // Load the organization specific timings
      int dq_id = [](int dq) -> int {
        switch (dq) {
          case 4:  return 0;
          case 8:  return 1;
          case 16: return 2;
          default: return -1;
        }
      }(m_organization.dq);

      int rate_id = [](int rate) -> int {
        switch (rate) {
          case 3200:  return 0;
          case 4800:  return 1;
          case 6400:  return 2;
          case 7200:  return 3;
          case 8000:  return 4;
          case 8800:  return 5;
          default:    return -1;
        }
      }(m_timing_vals("rate"));

      constexpr int nRRDL_TABLE[3][6] = {
      // 3200,4800,6400,7200,8000,8800
        { 5, 13, 17, 17, 16, 18},  // x4
        { 5, 13, 17, 17, 16, 18},  // x8
        { 5, 13, 17, 17, 16, 18},  // x16
      };
      constexpr int nFAW_TABLE[3][6] = {
      // 3200,4800, 6400, 7200, 8000, 8800
        { 40, 33, 33, 33, 32, 33},  // x4
        { 32, 33, 33, 33, 32, 37},  // x8
        { 32, 41, 43, 41, 40, 44},  // x16
      };

      if (dq_id != -1 && rate_id != -1) {
        m_timing_vals("nRRDL") = nRRDL_TABLE[dq_id][rate_id];
        m_timing_vals("nFAW")  = nFAW_TABLE [dq_id][rate_id];
      }

      // tCCD_L_WR2 (without RMW) table
      constexpr int nCCD_L_WR2_TABLE[6] = {
      // 3200 / 4800 / 6400 / 7200 / 8000 / 8800
        16,25,33,37,40,45
      };
      if (dq_id == 0) {
        m_timing_vals("nCCDL_WR") = nCCD_L_WR2_TABLE[rate_id];
      }

      // Refresh timings
      constexpr int tRFC_TABLE[2][3] = {
      //  8Gb   16Gb  32Gb
        { 195,  295,  410 }, // Normal refresh (tRFC1)
        { 130,  160,  220 }, // FGR 2x (tRFC2)
      };

      constexpr int tRFCsb_TABLE[1][3] = {
      //  8Gb   16Gb  32Gb
        { 115,  130,  190 }, // Normal refresh (tRFC1)
      };

      constexpr int tREFI_BASE = 3900;
      int density_id = [](int density_Mb) -> int {
        switch (density_Mb) {
          case 8192:  return 0;
          case 16384: return 1;
          case 32768: return 2;
          default:    return -1;
        }
      }(m_organization.density);

      m_RH_radius = param<int>("RH_radius").desc("The number of rows to refresh on each side").default_val(2);

      m_timing_vals("nRFC1")  = JEDEC_rounding_DDR5(tRFC_TABLE[0][density_id], tCK_ps);
      m_timing_vals("nRFC2")  = JEDEC_rounding_DDR5(tRFC_TABLE[1][density_id], tCK_ps);
      m_timing_vals("nRFCsb") = JEDEC_rounding_DDR5(tRFCsb_TABLE[0][density_id], tCK_ps);
      m_timing_vals("nREFI")  = JEDEC_rounding_DDR5(tREFI_BASE, tCK_ps);

      m_timing_vals("nRFM1")  = m_timing_vals("nRFC1");
      m_timing_vals("nRFM2")  = m_timing_vals("nRFC2");
      m_timing_vals("nRFMsb") = m_timing_vals("nRFCsb") * m_RH_radius;

      // tRRF table (unit is nanosecond!)
      constexpr int tRRFsb_TABLE[2][3] = {
      //  8Gb 16Gb 32Gb
        { 70,  70,  70 }, // tRRFab
        { 60,  60,  60 }, // tRRFsb
      };
      m_BRC = param_group("RFM").param<int>("BRC").default_val(2);
      m_timing_vals("nDRFMab") = 2 * m_BRC * JEDEC_rounding_DDR5(tRRFsb_TABLE[0][density_id], tCK_ps);
      m_timing_vals("nDRFMsb") = 2 * m_BRC * JEDEC_rounding_DDR5(tRRFsb_TABLE[1][density_id], tCK_ps);


      // Overwrite timing parameters with any user-provided value
      for (int i = 1; i < m_timings.size() - 1; i++) {
        auto timing_name = std::string(m_timings(i));

        if (auto provided_timing = param_group("timing").param<int>(timing_name).optional()) {
          m_timing_vals(i) = *provided_timing;
        } else if (auto provided_timing = param_group("timing").param<float>(timing_name.replace(0, 1, "t")).optional()) {
          m_timing_vals(i) = JEDEC_rounding_DDR5(*provided_timing, tCK_ps);
        }
      }

      // Check if there is any uninitialized timings
      for (int i = 0; i < m_timing_vals.size(); i++) {
        if (m_timing_vals(i) == -1) {
          throw ConfigurationError("In \"{}\", timing {} is not specified!", get_name(), m_timings(i));
        }
      }

      // Set read latency
      m_read_latency = m_timing_vals("nCL") + m_timing_vals("nBL");

      // Populate the timing constraints
      #define V(timing) (m_timing_vals(timing))
      auto all_commands = std::vector<std::string_view>(m_commands.begin(), m_commands.end());

      // Channel-bus commands: excludes _L commands (rank-local, no channel bus)
      auto channel_commands = std::vector<std::string_view>();
      for (auto& cmd : all_commands) {
        std::string_view sv(cmd);
        // Exclude NMA-local commands (ACT_L, PRE_L, RD_L, WR_L, PREA_L, REFab_L, REFab_L_end)
        if (sv.size() >= 2 && sv[sv.size()-1] == 'L' && sv[sv.size()-2] == '_') continue;
        if (sv == "REFab_L_end") continue;
        channel_commands.push_back(cmd);
      }
      populate_timingcons(this, {
          /*** Channel ***/
          // Two-Cycle Commands (standard DDR5 commands only) — excludes _L (rank-local)
          {.level = "channel", .preceding = {"ACT", "RD", "RDA", "WR", "WRA"}, .following = channel_commands, .latency = 2},

          // CAS <-> CAS
          /// Data bus occupancy
          {.level = "channel", .preceding = {"RD", "RDA"}, .following = {"RD", "RDA"}, .latency = V("nBL")},
          {.level = "channel", .preceding = {"WR", "WRA"}, .following = {"WR", "WRA"}, .latency = V("nBL")},

          // AsyncDIMM: Offload commands also occupy CA bus (2-cycle)
          {.level = "channel", .preceding = {"ACTO", "PREO", "RDO", "WRO", "REFO", "RT1","RT2","RT3","RT4","RT5","RT6","RT7","RT8"}, .following = channel_commands, .latency = 2},
          // RT_N: DQ bus occupied for N × nBL (batch return)
          // RT_N → any DQ command must wait N × nBL
          {.level = "channel", .preceding = {"RT1"}, .following = {"RD","RDA","WR","WRA","WRO","RT1","RT2","RT3","RT4","RT5","RT6","RT7","RT8"}, .latency = 1 * V("nBL")},
          {.level = "channel", .preceding = {"RT2"}, .following = {"RD","RDA","WR","WRA","WRO","RT1","RT2","RT3","RT4","RT5","RT6","RT7","RT8"}, .latency = 2 * V("nBL")},
          {.level = "channel", .preceding = {"RT3"}, .following = {"RD","RDA","WR","WRA","WRO","RT1","RT2","RT3","RT4","RT5","RT6","RT7","RT8"}, .latency = 3 * V("nBL")},
          {.level = "channel", .preceding = {"RT4"}, .following = {"RD","RDA","WR","WRA","WRO","RT1","RT2","RT3","RT4","RT5","RT6","RT7","RT8"}, .latency = 4 * V("nBL")},
          {.level = "channel", .preceding = {"RT5"}, .following = {"RD","RDA","WR","WRA","WRO","RT1","RT2","RT3","RT4","RT5","RT6","RT7","RT8"}, .latency = 5 * V("nBL")},
          {.level = "channel", .preceding = {"RT6"}, .following = {"RD","RDA","WR","WRA","WRO","RT1","RT2","RT3","RT4","RT5","RT6","RT7","RT8"}, .latency = 6 * V("nBL")},
          {.level = "channel", .preceding = {"RT7"}, .following = {"RD","RDA","WR","WRA","WRO","RT1","RT2","RT3","RT4","RT5","RT6","RT7","RT8"}, .latency = 7 * V("nBL")},
          {.level = "channel", .preceding = {"RT8"}, .following = {"RD","RDA","WR","WRA","WRO","RT1","RT2","RT3","RT4","RT5","RT6","RT7","RT8"}, .latency = 8 * V("nBL")},
          // Any DQ command → RT_N must wait nBL (previous DQ transfer must finish)
          {.level = "channel", .preceding = {"RD","RDA"}, .following = {"RT1","RT2","RT3","RT4","RT5","RT6","RT7","RT8"}, .latency = V("nBL")},
          // WRO uses DQ bus (write data for offloaded write)
          {.level = "channel", .preceding = {"WRO"}, .following = {"WR", "WRA", "WRO"}, .latency = V("nBL")},
          {.level = "channel", .preceding = {"WR", "WRA"}, .following = {"WRO"}, .latency = V("nBL")},
          // RDO does NOT use DQ bus (no data transfer on host DQ)

          /*** Rank (or different BankGroup) ***/
          // CAS <-> CAS (cross-compatible: normal + NMA-local)
          {.level = "rank", .preceding = {"RD","RDA","RD_L"}, .following = {"RD","RDA","RD_L"}, .latency = V("nCCDS")},
          {.level = "rank", .preceding = {"WR","WRA","WR_L"}, .following = {"WR","WRA","WR_L"}, .latency = V("nCCDS_WR")},
          /// RD <-> WR
          {.level = "rank", .preceding = {"RD","RDA","RD_L"}, .following = {"WR","WRA","WR_L"}, .latency = V("nCL") + V("nBL") + 2 - V("nCWL") + 2},
          /// WR <-> RD
          {.level = "rank", .preceding = {"WR","WRA","WR_L"}, .following = {"RD","RDA","RD_L"}, .latency = V("nCCDS_WTR")},
          /// CAS <-> CAS between sibling ranks (only host-visible commands; _L is rank-internal)
          {.level = "rank", .preceding = {"RD", "RDA"}, .following = {"RD", "RDA", "WR", "WRA"}, .latency = V("nBL") + V("nCS"), .is_sibling = true},
          {.level = "rank", .preceding = {"WR", "WRA"}, .following = {"RD", "RDA"}, .latency = V("nCL")  + V("nBL") + V("nCS") - V("nCWL"), .is_sibling = true},
          /// CAS <-> PREab (cross-compatible)
          {.level = "rank", .preceding = {"RD"},   .following = {"PREA","PREA_L"}, .latency = V("nRTP")},
          {.level = "rank", .preceding = {"RD_L"}, .following = {"PREA","PREA_L"}, .latency = V("nRTP")},
          {.level = "rank", .preceding = {"WR"},   .following = {"PREA","PREA_L"}, .latency = V("nCWL") + V("nBL") + V("nWR")},
          {.level = "rank", .preceding = {"WR_L"}, .following = {"PREA","PREA_L"}, .latency = V("nCWL") + V("nBL") + V("nWR")},
          /// RAS <-> RAS (cross-compatible)
          {.level = "rank", .preceding = {"ACT","ACT_L"}, .following = {"ACT","ACT_L"}, .latency = V("nRRDS")},
          {.level = "rank", .preceding = {"ACT","ACT_L"}, .following = {"ACT","ACT_L"}, .latency = V("nFAW"), .window = 4},
          {.level = "rank", .preceding = {"ACT","ACT_L"}, .following = {"PREA","PREA_L"}, .latency = V("nRAS")},
          {.level = "rank", .preceding = {"PREA","PREA_L"}, .following = {"ACT","ACT_L"}, .latency = V("nRP")},
          /// RAS <-> REF (cross-compatible with _L)
          {.level = "rank", .preceding = {"ACT","ACT_L"}, .following = {"REFab","REFab_L","RFMab","DRFMab"}, .latency = V("nRC")},
          {.level = "rank", .preceding = {"PRE","PREsb","PRE_L"}, .following = {"REFab","REFab_L","RFMab","DRFMab"}, .latency = V("nRP")},
          {.level = "rank", .preceding = {"PREA","PREA_L"}, .following = {"REFab","REFab_L","RFMab","DRFMab","REFsb","RFMsb","DRFMsb"}, .latency = V("nRP")},
          {.level = "rank", .preceding = {"RDA"}, .following = {"REFab","REFab_L","RFMab","DRFMab"}, .latency = V("nRP") + V("nRTP")},
          {.level = "rank", .preceding = {"WRA"}, .following = {"REFab","REFab_L","RFMab","DRFMab"}, .latency = V("nCWL") + V("nBL") + V("nWR") + V("nRP")},
          {.level = "rank", .preceding = {"REFab","REFab_L"}, .following = {"ACT","ACT_L","PREA","PREA_L","REFab","REFab_L","RFMab","DRFMab","REFsb","RFMsb","DRFMsb"}, .latency = V("nRFC1")},
          {.level = "rank", .preceding = {"RFMab"}, .following = {"ACT","ACT_L","PREA","PREA_L","REFab","REFab_L","RFMab","DRFMab","REFsb","RFMsb","DRFMsb"}, .latency = V("nRFM1")},
          {.level = "rank", .preceding = {"DRFMab"}, .following = {"ACT","ACT_L","PREA","PREA_L","REFab","REFab_L","RFMab","DRFMab","REFsb","RFMsb","DRFMsb"}, .latency = V("nDRFMab")},
          {.level = "rank", .preceding = {"REFsb"},  .following = {"PREA","PREA_L","REFab","REFab_L","RFMab","DRFMab"}, .latency = V("nRFCsb")},
          {.level = "rank", .preceding = {"RFMsb"},  .following = {"PREA","PREA_L","REFab","REFab_L","RFMab","DRFMab"}, .latency = V("nRFMsb")},
          {.level = "rank", .preceding = {"DRFMsb"}, .following = {"PREA","PREA_L","REFab","REFab_L","RFMab","DRFMab"}, .latency = V("nDRFMsb")},

          // AsyncDIMM: Offload commands have relaxed timing at rank level
          // ACTO/PREO/RDO/WRO: only CA bus constraint (2-cycle at channel level)
          // _L commands: rank-level timing but NO channel-level timing

          /*** Same Bank Group (cross-compatible) ***/
          /// CAS <-> CAS
          {.level = "bankgroup", .preceding = {"RD","RDA","RD_L"}, .following = {"RD","RDA","RD_L"}, .latency = V("nCCDL")},
          {.level = "bankgroup", .preceding = {"WR","WRA","WR_L"}, .following = {"WR","WRA","WR_L"}, .latency = V("nCCDL_WR")},
          {.level = "bankgroup", .preceding = {"WR","WRA","WR_L"}, .following = {"RD","RDA","RD_L"}, .latency = V("nCCDL_WTR")},
          /// RAS <-> RAS
          {.level = "bankgroup", .preceding = {"ACT","ACT_L"}, .following = {"ACT","ACT_L"}, .latency = V("nRRDL")},

          /*** Bank (cross-compatible) ***/
          // ACT → CAS/PRE
          {.level = "bank", .preceding = {"ACT","ACT_L"}, .following = {"ACT","ACT_L","REFsb","RFMsb","DRFMsb"}, .latency = V("nRC")},
          {.level = "bank", .preceding = {"ACT","ACT_L"}, .following = {"RD","RDA","WR","WRA","RD_L","WR_L"}, .latency = V("nRCD")},
          {.level = "bank", .preceding = {"ACT","ACT_L"}, .following = {"PRE","PREsb","PRE_L"}, .latency = V("nRAS")},
          // PRE → ACT
          {.level = "bank", .preceding = {"PRE","PREsb","PRE_L"}, .following = {"ACT","ACT_L","REFsb","RFMsb","DRFMsb"}, .latency = V("nRP")},
          // RD → PRE
          {.level = "bank", .preceding = {"RD","RD_L"},  .following = {"PRE","PREsb","PRE_L"}, .latency = V("nRTP")},
          // WR → PRE
          {.level = "bank", .preceding = {"WR","WR_L"},  .following = {"PRE","PREsb","PRE_L"}, .latency = V("nCWL") + V("nBL") + V("nWR")},
          // RDA/WRA (host-only auto-precharge)
          {.level = "bank", .preceding = {"RDA"}, .following = {"ACT","ACT_L","REFsb","RFMsb","DRFMsb"}, .latency = V("nRTP") + V("nRP")},
          {.level = "bank", .preceding = {"WRA"}, .following = {"ACT","ACT_L","REFsb","RFMsb","DRFMsb"}, .latency = V("nCWL") + V("nBL") + V("nWR") + V("nRP")},
          {.level = "bank", .preceding = {"WR","WR_L"},  .following = {"RDA"}, .latency = V("nCWL") + V("nBL") + V("nWR") - V("nRTP")},

          /// Same-bank refresh/RFM (cross-compatible)
          {.level = "bank", .preceding = {"REFsb"},  .following = {"ACT","ACT_L","REFsb","RFMsb","DRFMsb"}, .latency = V("nRFCsb")},
          {.level = "bank", .preceding = {"RFMsb"},  .following = {"ACT","ACT_L","REFsb","RFMsb","DRFMsb"}, .latency = V("nRFMsb")},
          {.level = "bank", .preceding = {"DRFMsb"}, .following = {"ACT","ACT_L","REFsb","RFMsb","DRFMsb"}, .latency = V("nDRFMsb")},

          // AsyncDIMM: Offload commands have NO bank-level timing constraints
          // _L commands have FULL bank/BG/rank timing but NO channel-level timing
        }
      );
      #undef V

    };

    void set_actions() {
      m_actions.resize(m_levels.size(), std::vector<ActionFunc_t<Node>>(m_commands.size()));

      // Rank Actions
      m_actions[m_levels["rank"]][m_commands["PREA"]] = Lambdas::Action::Rank::PREab<DDR5AsyncDIMM>;
      m_actions[m_levels["rank"]][m_commands["REFab"]] = Lambdas::Action::Rank::REFab<DDR5AsyncDIMM>;
      m_actions[m_levels["rank"]][m_commands["REFab_end"]] = Lambdas::Action::Rank::REFab_end<DDR5AsyncDIMM>;
      m_actions[m_levels["rank"]][m_commands["RFMab"]] = Lambdas::Action::Rank::REFab<DDR5AsyncDIMM>;
      m_actions[m_levels["rank"]][m_commands["RFMab_end"]] = Lambdas::Action::Rank::REFab_end<DDR5AsyncDIMM>;
      m_actions[m_levels["rank"]][m_commands["DRFMab"]] = Lambdas::Action::Rank::REFab<DDR5AsyncDIMM>;
      m_actions[m_levels["rank"]][m_commands["DRFMab_end"]] = Lambdas::Action::Rank::REFab_end<DDR5AsyncDIMM>;

      // Same-Bank Actions
      m_actions[m_levels["bankgroup"]][m_commands["PREsb"]] = Lambdas::Action::BankGroup::PREsb<DDR5AsyncDIMM>;
      m_actions[m_levels["bankgroup"]][m_commands["REFsb"]]  = Lambdas::Action::BankGroup::REFsb<DDR5AsyncDIMM>;
      m_actions[m_levels["bankgroup"]][m_commands["REFsb_end"]]  = Lambdas::Action::BankGroup::REFsb_end<DDR5AsyncDIMM>;
      m_actions[m_levels["bankgroup"]][m_commands["RFMsb"]]  = Lambdas::Action::BankGroup::REFsb<DDR5AsyncDIMM>;
      m_actions[m_levels["bankgroup"]][m_commands["RFMsb_end"]]  = Lambdas::Action::BankGroup::REFsb_end<DDR5AsyncDIMM>;
      m_actions[m_levels["bankgroup"]][m_commands["DRFMsb"]] = Lambdas::Action::BankGroup::REFsb<DDR5AsyncDIMM>;
      m_actions[m_levels["bankgroup"]][m_commands["DRFMsb_end"]] = Lambdas::Action::BankGroup::REFsb_end<DDR5AsyncDIMM>;

      // Bank actions
      m_actions[m_levels["bank"]][m_commands["ACT"]] = Lambdas::Action::Bank::ACT<DDR5AsyncDIMM>;
      m_actions[m_levels["bank"]][m_commands["PRE"]] = Lambdas::Action::Bank::PRE<DDR5AsyncDIMM>;
      m_actions[m_levels["bank"]][m_commands["RDA"]] = Lambdas::Action::Bank::PRE<DDR5AsyncDIMM>;
      m_actions[m_levels["bank"]][m_commands["WRA"]] = Lambdas::Action::Bank::PRE<DDR5AsyncDIMM>;

      // NMA-Local: same bank/rank actions (DRAM array state changes)
      m_actions[m_levels["bank"]][m_commands["ACT_L"]] = Lambdas::Action::Bank::ACT<DDR5AsyncDIMM>;
      m_actions[m_levels["bank"]][m_commands["PRE_L"]] = Lambdas::Action::Bank::PRE<DDR5AsyncDIMM>;
      m_actions[m_levels["rank"]][m_commands["PREA_L"]] = Lambdas::Action::Rank::PREab<DDR5AsyncDIMM>;
      m_actions[m_levels["rank"]][m_commands["REFab_L"]] = Lambdas::Action::Rank::REFab<DDR5AsyncDIMM>;
      m_actions[m_levels["rank"]][m_commands["REFab_L_end"]] = Lambdas::Action::Rank::REFab_end<DDR5AsyncDIMM>;

      // AsyncDIMM: Offload commands do NOT change bank state on the host DRAM model
      // ACTO/PREO/RDO/WRO are no-op actions (state changes happen in NMA MC's bank FSM)
      // No action lambdas needed — nullptr (default) means no state change
      // RT also does not change bank state (it's a data return on DQ bus)
    };

    void set_preqs() {
      m_preqs.resize(m_levels.size(), std::vector<PreqFunc_t<Node>>(m_commands.size()));

      // Rank Preqs
      m_preqs[m_levels["rank"]][m_commands["REFab"]]  = Lambdas::Preq::Rank::RequireAllBanksClosed<DDR5AsyncDIMM>;
      m_preqs[m_levels["rank"]][m_commands["RFMab"]]  = Lambdas::Preq::Rank::RequireAllBanksClosed<DDR5AsyncDIMM>;
      m_preqs[m_levels["rank"]][m_commands["DRFMab"]] = Lambdas::Preq::Rank::RequireAllBanksClosed<DDR5AsyncDIMM>;

      // Same-Bank Preqs
      m_preqs[m_levels["rank"]][m_commands["REFsb"]]  = Lambdas::Preq::Rank::RequireSameBanksClosed<DDR5AsyncDIMM>;
      m_preqs[m_levels["rank"]][m_commands["RFMsb"]]  = Lambdas::Preq::Rank::RequireSameBanksClosed<DDR5AsyncDIMM>;
      m_preqs[m_levels["rank"]][m_commands["DRFMsb"]] = Lambdas::Preq::Rank::RequireSameBanksClosed<DDR5AsyncDIMM>;

      // Bank Preqs (standard DDR5)
      m_preqs[m_levels["bank"]][m_commands["RD"]] = Lambdas::Preq::Bank::RequireRowOpen<DDR5AsyncDIMM>;
      m_preqs[m_levels["bank"]][m_commands["WR"]] = Lambdas::Preq::Bank::RequireRowOpen<DDR5AsyncDIMM>;
      m_preqs[m_levels["bank"]][m_commands["ACT"]] = Lambdas::Preq::Bank::RequireRowOpen<DDR5AsyncDIMM>;
      m_preqs[m_levels["bank"]][m_commands["PRE"]] = Lambdas::Preq::Bank::RequireBankClosed<DDR5AsyncDIMM>;

      // NMA-Local: same bank prerequisites (DRAM array state must match)
      m_preqs[m_levels["bank"]][m_commands["RD_L"]] = Lambdas::Preq::Bank::RequireRowOpen<DDR5AsyncDIMM>;
      m_preqs[m_levels["bank"]][m_commands["WR_L"]] = Lambdas::Preq::Bank::RequireRowOpen<DDR5AsyncDIMM>;
      m_preqs[m_levels["bank"]][m_commands["ACT_L"]] = Lambdas::Preq::Bank::RequireRowOpen<DDR5AsyncDIMM>;
      m_preqs[m_levels["bank"]][m_commands["PRE_L"]] = Lambdas::Preq::Bank::RequireBankClosed<DDR5AsyncDIMM>;
      m_preqs[m_levels["rank"]][m_commands["REFab_L"]] = Lambdas::Preq::Rank::RequireAllBanksClosed<DDR5AsyncDIMM>;

      // AsyncDIMM: Offload commands have no bank state prerequisites on host DRAM
      // ACTO/PREO/RDO/WRO can be issued regardless of host-side bank state
      // (They are forwarded to NMA MC which has its own bank FSM)
      // NoRequire: always returns the command itself (no prerequisite needed)
      m_preqs[m_levels["rank"]][m_commands["ACTO"]] = Lambdas::Preq::Rank::NoRequire<DDR5AsyncDIMM>;
      m_preqs[m_levels["rank"]][m_commands["PREO"]] = Lambdas::Preq::Rank::NoRequire<DDR5AsyncDIMM>;
      m_preqs[m_levels["rank"]][m_commands["RDO"]]  = Lambdas::Preq::Rank::NoRequire<DDR5AsyncDIMM>;
      m_preqs[m_levels["rank"]][m_commands["WRO"]]  = Lambdas::Preq::Rank::NoRequire<DDR5AsyncDIMM>;
      m_preqs[m_levels["rank"]][m_commands["REFO"]] = Lambdas::Preq::Rank::NoRequire<DDR5AsyncDIMM>;
      m_preqs[m_levels["rank"]][m_commands["RT1"]] = Lambdas::Preq::Rank::NoRequire<DDR5AsyncDIMM>;
      m_preqs[m_levels["rank"]][m_commands["RT2"]] = Lambdas::Preq::Rank::NoRequire<DDR5AsyncDIMM>;
      m_preqs[m_levels["rank"]][m_commands["RT3"]] = Lambdas::Preq::Rank::NoRequire<DDR5AsyncDIMM>;
      m_preqs[m_levels["rank"]][m_commands["RT4"]] = Lambdas::Preq::Rank::NoRequire<DDR5AsyncDIMM>;
      m_preqs[m_levels["rank"]][m_commands["RT5"]] = Lambdas::Preq::Rank::NoRequire<DDR5AsyncDIMM>;
      m_preqs[m_levels["rank"]][m_commands["RT6"]] = Lambdas::Preq::Rank::NoRequire<DDR5AsyncDIMM>;
      m_preqs[m_levels["rank"]][m_commands["RT7"]] = Lambdas::Preq::Rank::NoRequire<DDR5AsyncDIMM>;
      m_preqs[m_levels["rank"]][m_commands["RT8"]] = Lambdas::Preq::Rank::NoRequire<DDR5AsyncDIMM>;
    };

    void set_rowhits() {
      m_rowhits.resize(m_levels.size(), std::vector<RowhitFunc_t<Node>>(m_commands.size()));

      m_rowhits[m_levels["bank"]][m_commands["RD"]] = Lambdas::RowHit::Bank::RDWR<DDR5AsyncDIMM>;
      m_rowhits[m_levels["bank"]][m_commands["WR"]] = Lambdas::RowHit::Bank::RDWR<DDR5AsyncDIMM>;
      // Offload commands don't participate in host-side row hit tracking
    }


    void set_rowopens() {
      m_rowopens.resize(m_levels.size(), std::vector<RowhitFunc_t<Node>>(m_commands.size()));

      m_rowopens[m_levels["bank"]][m_commands["RD"]] = Lambdas::RowOpen::Bank::RDWR<DDR5AsyncDIMM>;
      m_rowopens[m_levels["bank"]][m_commands["WR"]] = Lambdas::RowOpen::Bank::RDWR<DDR5AsyncDIMM>;
    }

    void set_powers() {

      m_drampower_enable = param<bool>("drampower_enable").default_val(false);

      if (!m_drampower_enable)
        return;

      m_voltage_vals.resize(m_voltages.size(), -1);

      if (auto preset_name = param_group("voltage").param<std::string>("preset").optional()) {
        if (voltage_presets.count(*preset_name) > 0) {
          m_voltage_vals = voltage_presets.at(*preset_name);
        } else {
          throw ConfigurationError("Unrecognized voltage preset \"{}\" in {}!", *preset_name, get_name());
        }
      }

      m_current_vals.resize(m_currents.size(), -1);

      if (auto preset_name = param_group("current").param<std::string>("preset").optional()) {
        if (current_presets.count(*preset_name) > 0) {
          m_current_vals = current_presets.at(*preset_name);
        } else {
          throw ConfigurationError("Unrecognized current preset \"{}\" in {}!", *preset_name, get_name());
        }
      }

      m_power_debug = param<bool>("power_debug").default_val(false);

      int num_channels = m_organization.count[m_levels["channel"]];
      int num_ranks = m_organization.count[m_levels["rank"]];
      m_power_stats.resize(num_channels * num_ranks);
      for (int i = 0; i < num_channels; i++) {
        for (int j = 0; j < num_ranks; j++) {
          m_power_stats[i * num_ranks + j].rank_id = i * num_ranks + j;
          m_power_stats[i * num_ranks + j].cmd_counters.resize(m_cmds_counted.size(), 0);
        }
      }

      m_powers.resize(m_levels.size(), std::vector<PowerFunc_t<Node>>(m_commands.size()));

      m_powers[m_levels["bank"]][m_commands["ACT"]] = Lambdas::Power::Bank::ACT<DDR5AsyncDIMM>;
      m_powers[m_levels["bank"]][m_commands["PRE"]] = Lambdas::Power::Bank::PRE<DDR5AsyncDIMM>;
      m_powers[m_levels["bank"]][m_commands["RD"]]  = Lambdas::Power::Bank::RD<DDR5AsyncDIMM>;
      m_powers[m_levels["bank"]][m_commands["WR"]]  = Lambdas::Power::Bank::WR<DDR5AsyncDIMM>;

      m_powers[m_levels["rank"]][m_commands["RFMsb"]] = Lambdas::Power::Rank::RFMsb<DDR5AsyncDIMM>;
      m_powers[m_levels["rank"]][m_commands["RFMsb_end"]] = Lambdas::Power::Rank::RFMsb_end<DDR5AsyncDIMM>;

      m_powers[m_levels["rank"]][m_commands["ACT"]] = Lambdas::Power::Rank::ACT<DDR5AsyncDIMM>;
      m_powers[m_levels["rank"]][m_commands["PRE"]] = Lambdas::Power::Rank::PRE<DDR5AsyncDIMM>;
      m_powers[m_levels["rank"]][m_commands["PREA"]] = Lambdas::Power::Rank::PREA<DDR5AsyncDIMM>;
      m_powers[m_levels["rank"]][m_commands["REFab"]] = Lambdas::Power::Rank::REFab<DDR5AsyncDIMM>;
      m_powers[m_levels["rank"]][m_commands["REFab_end"]] = Lambdas::Power::Rank::REFab_end<DDR5AsyncDIMM>;

      m_powers[m_levels["rank"]][m_commands["PREsb"]] = Lambdas::Power::Rank::PREsb<DDR5AsyncDIMM>;

      // NMA-Local commands: same DRAM array energy as originals (rank-local bus)
      m_powers[m_levels["bank"]][m_commands["ACT_L"]] = Lambdas::Power::Bank::ACT_L<DDR5AsyncDIMM>;
      m_powers[m_levels["bank"]][m_commands["PRE_L"]] = Lambdas::Power::Bank::PRE_L<DDR5AsyncDIMM>;
      m_powers[m_levels["bank"]][m_commands["RD_L"]]  = Lambdas::Power::Bank::RD_L<DDR5AsyncDIMM>;
      m_powers[m_levels["bank"]][m_commands["WR_L"]]  = Lambdas::Power::Bank::WR_L<DDR5AsyncDIMM>;

      m_powers[m_levels["rank"]][m_commands["ACT_L"]]       = Lambdas::Power::Rank::ACT_L<DDR5AsyncDIMM>;
      m_powers[m_levels["rank"]][m_commands["PRE_L"]]       = Lambdas::Power::Rank::PRE_L<DDR5AsyncDIMM>;
      m_powers[m_levels["rank"]][m_commands["PREA_L"]]      = Lambdas::Power::Rank::PREA_L<DDR5AsyncDIMM>;
      m_powers[m_levels["rank"]][m_commands["REFab_L"]]     = Lambdas::Power::Rank::REFab_L<DDR5AsyncDIMM>;
      m_powers[m_levels["rank"]][m_commands["REFab_L_end"]] = Lambdas::Power::Rank::REFab_L_end<DDR5AsyncDIMM>;

      // Offload commands: no DRAM array power (only CA/DQ bus, tracked separately)

      // register stats
      register_stat(s_total_background_energy).name("total_background_energy");
      register_stat(s_total_cmd_energy).name("total_cmd_energy");
      register_stat(s_total_energy).name("total_energy");
      register_stat(s_total_energy).name("s_total_dq_energy");
      register_stat(s_total_rfm_energy).name("total_rfm_energy");

      register_stat(s_total_background_power).name("s_total_background_power");
      register_stat(s_total_cmd_power).name("s_total_cmd_power");
      register_stat(s_total_dq_power).name("s_total_dq_power");
      register_stat(s_total_power).name("s_total_power");

      for (auto& power_stat : m_power_stats){
        register_stat(power_stat.total_background_energy).name("total_background_energy_rank{}", power_stat.rank_id);
        register_stat(power_stat.total_cmd_energy).name("total_cmd_energy_rank{}", power_stat.rank_id);
        register_stat(power_stat.total_energy).name("total_energy_rank{}", power_stat.rank_id);
        register_stat(power_stat.act_background_energy).name("act_background_energy_rank{}", power_stat.rank_id);
        register_stat(power_stat.pre_background_energy).name("pre_background_energy_rank{}", power_stat.rank_id);
        register_stat(power_stat.active_cycles).name("active_cycles_rank{}", power_stat.rank_id);
        register_stat(power_stat.idle_cycles).name("idle_cycles_rank{}", power_stat.rank_id);
      }
    }

    void create_nodes() {
      int num_channels = m_organization.count[m_levels["channel"]];
      for (int i = 0; i < num_channels; i++) {
        Node* channel = new Node(this, nullptr, 0, i);
        m_channels.push_back(channel);
      }
    }

    void finalize() override {

      if (!m_drampower_enable)
        return;

      int num_channels = m_organization.count[m_levels["channel"]];
      int num_ranks = m_organization.count[m_levels["rank"]];

      // pJ per bit
      double socket_dq_energy = 18.48;    // Host MC <-> DIMM connector (external channel)
      double on_board_dq_energy = 10.08;  // DIMM internal traces (buffer chip <-> DRAM)
      size_t total_acc = 0;
      for (int i = 0; i < num_channels; i++) {
        size_t num_host_trans = 0;  // Host RD/WR: traverse socket + on-board
        size_t num_nma_trans = 0;   // NMA RD_L/WR_L: on-board only (rank-local)
        for (int j = 0; j < num_ranks; j++) {
          process_rank_energy(m_power_stats[i * num_ranks + j], m_channels[i]->m_child_nodes[j]);
          num_host_trans += m_power_stats[i * num_ranks + j].cmd_counters[m_cmds_counted("RD")] +
                            m_power_stats[i * num_ranks + j].cmd_counters[m_cmds_counted("WR")];
          num_nma_trans  += m_power_stats[i * num_ranks + j].cmd_counters[m_cmds_counted("RD_L")] +
                            m_power_stats[i * num_ranks + j].cmd_counters[m_cmds_counted("WR_L")];
        }
        size_t num_trans = num_host_trans + num_nma_trans;
        // Socket DQ: only host transactions cross the DIMM connector
        double channel_socket_dq_energy = (double)num_host_trans * (double)(16 * (m_channel_width+m_parity_width)) * socket_dq_energy / 1E3;
        // On-board DQ: all transactions traverse DIMM internal traces
        double channel_onboard_dq_energy = (double)num_trans * (double)(16 * (m_channel_width+m_parity_width)) * on_board_dq_energy / 1E3;
        double dq_energy = channel_socket_dq_energy + channel_onboard_dq_energy;
        double dq_power = dq_energy/((double)m_clk * (double)m_timing_vals("tCK_ps") / 1000.0);
        std::cout<<"["<<num_channels<<"] Channel DQ Power Report"<<std::endl;
        std::cout<<" - DQ (Socket) Energy (nJ) : "<<channel_socket_dq_energy<<std::endl;
        std::cout<<" - DQ (OnBoard) Energy (nJ): "<<channel_onboard_dq_energy<<std::endl;
        std::cout<<" - DQ Energy (nJ) : "<<dq_energy<<std::endl;
        std::cout<<" - DQ Power (W)   : "<<dq_power<<std::endl;
        s_total_dq_energy += (dq_energy);
        s_total_energy    += (dq_energy);
        s_total_dq_power  += (dq_power);
        s_total_power     += (dq_power);
        total_acc         += (num_trans);
      }


      std::cout<<" ==== Total Channel Power Report === "<<std::endl;
      std::cout<<" - DRAM Background Energy (nJ)  : "<<s_total_background_energy<<std::endl;
      std::cout<<" - DRAM Command Energy (nJ)     : "<<s_total_cmd_energy<<std::endl;
      std::cout<<" - DRAM DQ Energy (nJ)          : "<<s_total_dq_energy<<std::endl;
      std::cout<<" - Total DRAM Energy (nJ)       : "<<s_total_energy<<std::endl;

      std::cout<<" - DRAM Background Power (W)    : "<<s_total_background_power<<std::endl;
      std::cout<<" - DRAM Command Power (W)       : "<<s_total_cmd_power<<std::endl;
      std::cout<<" - DRAM DQ Power(W)             : "<<s_total_dq_power<<std::endl;
      std::cout<<" - Total DRAM Power (W)         : "<<s_total_power<<std::endl;

      std::cout<<" ==== Total Channel Bandwidth (GB/s) Report === "<<std::endl;
      size_t total_host_acc = 0;
      size_t total_nma_acc = 0;
      for (int i = 0; i < num_channels; i++) {
        for (int j = 0; j < num_ranks; j++) {
          total_host_acc += m_power_stats[i * num_ranks + j].cmd_counters[m_cmds_counted("RD")] +
                            m_power_stats[i * num_ranks + j].cmd_counters[m_cmds_counted("WR")];
          total_nma_acc  += m_power_stats[i * num_ranks + j].cmd_counters[m_cmds_counted("RD_L")] +
                            m_power_stats[i * num_ranks + j].cmd_counters[m_cmds_counted("WR_L")];
        }
      }
      double total_bw = (double)((double)total_acc * 512.0) / ((double)m_clk * (double)m_timing_vals("tCK_ps") / 1000.0) / 8;
      double host_bw  = (double)((double)total_host_acc * 512.0) / ((double)m_clk * (double)m_timing_vals("tCK_ps") / 1000.0) / 8;
      double nma_bw   = (double)((double)total_nma_acc * 512.0) / ((double)m_clk * (double)m_timing_vals("tCK_ps") / 1000.0) / 8;
      std::cout<<" - Total Bandwidth                  : "<<total_bw<<std::endl;
      std::cout<<" - Host Bandwidth                   : "<<host_bw<<std::endl;
      std::cout<<" - NMA Bandwidth                    : "<<nma_bw<<std::endl;
      std::cout<<" - Total Access                     : "<<total_acc<<std::endl;
      std::cout<<" - Host Access                      : "<<total_host_acc<<std::endl;
      std::cout<<" - NMA Access                       : "<<total_nma_acc<<std::endl;

    }

    void process_rank_energy(PowerStats& rank_stats, Node* rank_node) {

      Lambdas::Power::Rank::finalize_rank<DDR5AsyncDIMM>(rank_node, 0, AddrVec_t(), m_clk);

      size_t num_bankgroups = m_organization.count[m_levels["bankgroup"]];
      size_t num_banks = m_organization.count[m_levels["bank"]];
      int num_dev_per_rank = (m_channel_width+m_parity_width)/m_organization.dq;

      auto TS = [&](std::string_view timing) { return m_timing_vals(timing); };
      auto VE = [&](std::string_view voltage) { return m_voltage_vals(voltage); };
      auto CE = [&](std::string_view current) { return m_current_vals(current); };

      double tCK_ns = (double) TS("tCK_ps") / 1000.0;

      double one_bank_idd3N = (CE("IDD3N") - CE("IDD2N"))/(num_bankgroups * num_banks);
      double one_bank_ipp3N = (CE("IPP3N") - CE("IPP2N"))/(num_bankgroups * num_banks);

      rank_stats.act_background_energy = (VE("VDD") * CE("IDD3N") + VE("VPP") * CE("IPP3N"))
                                            * rank_stats.active_cycles * tCK_ns / 1E3;

      rank_stats.pre_background_energy = (VE("VDD") * CE("IDD2N") + VE("VPP") * CE("IPP2N"))
                                            * rank_stats.idle_cycles * tCK_ns / 1E3;


      // Host MC commands (channel bus)
      size_t act_count = rank_stats.cmd_counters[m_cmds_counted("ACT")];
      size_t pre_count = rank_stats.cmd_counters[m_cmds_counted("PRE")];
      size_t rd_count  = rank_stats.cmd_counters[m_cmds_counted("RD")];
      size_t wr_count  = rank_stats.cmd_counters[m_cmds_counted("WR")];
      size_t ref_count = rank_stats.cmd_counters[m_cmds_counted("REF")];

      // NMA-Local commands (rank-local bus, same DRAM array energy)
      act_count += rank_stats.cmd_counters[m_cmds_counted("ACT_L")];
      pre_count += rank_stats.cmd_counters[m_cmds_counted("PRE_L")];
      rd_count  += rank_stats.cmd_counters[m_cmds_counted("RD_L")];
      wr_count  += rank_stats.cmd_counters[m_cmds_counted("WR_L")];
      ref_count += rank_stats.cmd_counters[m_cmds_counted("REFab_L")];

      double act_cmd_energy  = (VE("VDD") * (CE("IDD0") - one_bank_idd3N) + VE("VPP") * (CE("IPP0") - one_bank_ipp3N))
                                      * act_count * TS("nRAS") * tCK_ns / 1E3;

      double pre_cmd_energy  = (VE("VDD") * (CE("IDD0") - CE("IDD2N")) + VE("VPP") * (CE("IPP0") - CE("IPP2N")))
                                      * pre_count * TS("nRP")  * tCK_ns / 1E3;

      double rd_cmd_energy   = (VE("VDD") * (CE("IDD4R") - CE("IDD3N")) + VE("VPP") * (CE("IPP4R") - CE("IPP3N")))
                                      * rd_count * TS("nBL") * tCK_ns / 1E3;

      double wr_cmd_energy   = (VE("VDD") * (CE("IDD4W") - CE("IDD3N")) + VE("VPP") * (CE("IPP4W") - CE("IPP3N")))
                                      * wr_count * TS("nBL") * tCK_ns / 1E3;

      double ref_cmd_energy  = (VE("VDD") * (CE("IDD5B")) + VE("VPP") * (CE("IPP5B")))
                                      * ref_count * TS("nRFC1") * tCK_ns / 1E3;

      double rfm_cmd_energy = (VE("VDD") * (CE("IDD0") - CE("IDD3N")) + VE("VPP") * (CE("IPP0") - CE("IPP3N"))) * num_bankgroups
                                      * rank_stats.cmd_counters[m_cmds_counted("RFM")] * TS("nRFMsb") * tCK_ns / 1E3;

      #ifdef DEBUG_POWER
        std::cout<<"act_cmd_energy  : "<<act_cmd_energy<<std::endl;
        std::cout<<"pre_cmd_energy  : "<<pre_cmd_energy<<std::endl;
        std::cout<<"rd_cmd_energy   : "<<rd_cmd_energy<<std::endl;
        std::cout<<"wr_cmd_energy   : "<<wr_cmd_energy<<std::endl;
        std::cout<<"ref_cmd_energy  : "<<ref_cmd_energy<<std::endl;
        std::cout<<"rank_stats.cmd_counters[ACT]    : "<<rank_stats.cmd_counters[m_cmds_counted("ACT")]<<std::endl;
        std::cout<<"rank_stats.cmd_counters[PRE]    : "<<rank_stats.cmd_counters[m_cmds_counted("PRE")]<<std::endl;
        std::cout<<"rank_stats.cmd_counters[RD]     : "<<rank_stats.cmd_counters[m_cmds_counted("RD")]<<std::endl;
        std::cout<<"rank_stats.cmd_counters[WR]     : "<<rank_stats.cmd_counters[m_cmds_counted("WR")]<<std::endl;
        std::cout<<"rank_stats.cmd_counters[ACT_L]  : "<<rank_stats.cmd_counters[m_cmds_counted("ACT_L")]<<std::endl;
        std::cout<<"rank_stats.cmd_counters[PRE_L]  : "<<rank_stats.cmd_counters[m_cmds_counted("PRE_L")]<<std::endl;
        std::cout<<"rank_stats.cmd_counters[RD_L]   : "<<rank_stats.cmd_counters[m_cmds_counted("RD_L")]<<std::endl;
        std::cout<<"rank_stats.cmd_counters[WR_L]   : "<<rank_stats.cmd_counters[m_cmds_counted("WR_L")]<<std::endl;
        std::cout<<"rank_stats.cmd_counters[REFab_L]: "<<rank_stats.cmd_counters[m_cmds_counted("REFab_L")]<<std::endl;
      #endif

      rank_stats.total_background_energy = num_dev_per_rank * (rank_stats.act_background_energy + rank_stats.pre_background_energy);
      rank_stats.total_cmd_energy = ( act_cmd_energy
                                    + pre_cmd_energy
                                    + rd_cmd_energy
                                    + wr_cmd_energy
                                    + ref_cmd_energy
                                    + rfm_cmd_energy) * num_dev_per_rank;

      rank_stats.total_energy = rank_stats.total_background_energy + rank_stats.total_cmd_energy;

      s_total_background_energy += rank_stats.total_background_energy;
      s_total_cmd_energy += rank_stats.total_cmd_energy;
      s_total_energy += rank_stats.total_energy;
      s_total_rfm_energy += rfm_cmd_energy;

      s_total_rfm_cycles[rank_stats.rank_id] = rank_stats.cmd_counters[m_cmds_counted("RFM")] * TS("nRFMsb");

      double background_power = rank_stats.total_background_energy / ((double)m_clk * tCK_ns);
      double command_power = rank_stats.total_cmd_energy / ((double)m_clk * tCK_ns);
      double total_power = (background_power + command_power);
      s_total_background_power += (background_power);
      s_total_cmd_power        += (command_power);
      s_total_power            += total_power;
      std::cout<<"["<<rank_stats.rank_id<<"] Power Report"<<std::endl;
      std::cout<<" - Background Energy (nJ) : "<<rank_stats.total_background_energy<<std::endl;
      std::cout<<" - Command Energy (nJ)    : "<<rank_stats.total_cmd_energy<<std::endl;
      std::cout<<" - Background Power (W)   : "<<background_power<<std::endl;
      std::cout<<" - Command Power (W)      : "<<command_power<<std::endl;
      std::cout<<" - Rank Power (W)         : "<<total_power<<std::endl;
    }
};


}        // namespace Ramulator
