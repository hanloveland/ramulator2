#include "dram/dram.h"
#include "dram/lambdas.h"

// #define PRINT_DEBUG

#ifdef PRINT_DEBUG
#define DEBUG_PRINT(clk, unit_str, ch, pch, msg) do { std::cout <<"["<<clk<<"]["<<unit_str<<"] CH["<<ch<<"] PCH["<<pch<<"]"<<msg<<std::endl; } while(0)
#else
#define DEBUG_PRINT(clk, unit_str, ch, pch, msg) do {} while(0)
#endif

namespace Ramulator {

class DDR5PCH : public IDRAM, public Implementation {
  RAMULATOR_REGISTER_IMPLEMENTATION(IDRAM, DDR5PCH, "DDR5-pCH", "DDR5 Device Model with Pseudo Channel")
  private:
    int m_RH_radius = -1;


  public:
    // Add pCH-NI (narrow-I/O == Off-PCB I/O) and pCH-WI (Wide-I/O == On-PCB I/O)
    // pCH-nI and PCH-wI must be 1
    inline static const std::map<std::string, Organization> org_presets = {
      //   name         density   DQ   Ch pCH nI wI Ra Bg Ba   Ro     Co
      {"DDR5_8Gb_x4",   {8<<10,   4,  {1, 1,  1, 1, 1, 8, 2,   1<<16, 1<<11}}},
      {"DDR5_8Gb_x8",   {8<<10,   8,  {1, 1,  1, 1, 1, 8, 2,   1<<16, 1<<10}}},
      {"DDR5_8Gb_x16",  {8<<10,   16, {1, 1,  1, 1, 1, 4, 2,   1<<16, 1<<10}}},
      {"DDR5_16Gb_x4",  {16<<10,  4,  {1, 1,  1, 1, 1, 8, 4,   1<<16, 1<<11}}},
      {"DDR5_16Gb_x8",  {16<<10,  8,  {1, 1,  1, 1, 1, 8, 4,   1<<16, 1<<10}}},
      {"DDR5_16Gb_x16", {16<<10,  16, {1, 1,  1, 1, 1, 4, 4,   1<<16, 1<<10}}},
      {"DDR5_32Gb_x4",  {32<<10,  4,  {1, 1,  1, 1, 1, 8, 4,   1<<17, 1<<11}}},
      {"DDR5_32Gb_x8",  {32<<10,  8,  {1, 1,  1, 1, 1, 8, 4,   1<<17, 1<<10}}},
      {"DDR5_32Gb_x16", {32<<10,  16, {1, 1,  1, 1, 1, 4, 4,   1<<17, 1<<10}}},

      // {"DDR5_64Gb_x4",  {64<<10,  4,  {1, 1, 8, 4, 1<<18, 1<<11}}},
      // {"DDR5_64Gb_x8",  {64<<10,  8,  {1, 1, 8, 4, 1<<18, 1<<10}}},
      // {"DDR5_64Gb_x16", {64<<10,  16, {1, 1, 4, 4, 1<<18, 1<<10}}},
    };
    // tCCD_S_WTR = CWL + BL/2 + max(4ntCK,2.5ns)
    inline static const std::map<std::string, std::vector<int>> timing_presets = {
      //   name         rate   nBL  nCL nRCD   nRP  nRAS   nRC   nWR  nRTP nCWL nPPD nCCDS nCCDS_WR nCCDS_WTR      nCCDS_WTR_WI nCCDL nCCDL_WR  nCCDL_WTR      nCCDL_WTR_WI nRRDS nRRDL nFAW nRFC1 nRFC2 nRFCsb nREFI nREFSBRD nRFM1 nRFM2 nRFMsb nDRFMab nDRFMsb nCS, tCK_ps
      {"DDR5_3200AN",  {3200,   8,  24,  24,   24,   52,   75,   48,   12,  22,  2,    8,     8,     22+(4*8)+4,   22+(1*8)+4,  8,    16,       22+(4*8)+16,   22+(1*8)+16,   8,   -1,   -1,  -1,   -1,   -1,    -1,     30,    -1,   -1,   -1,     -1,     -1,    2,   625}},
      {"DDR5_3200BN",  {3200,   8,  26,  26,   26,   52,   77,   48,   12,  24,  2,    8,     8,     24+(4*8)+4,   24+(1*8)+4,  8,    16,       24+(4*8)+16,   24+(1*8)+16,   8,   -1,   -1,  -1,   -1,   -1,    -1,     30,    -1,   -1,   -1,     -1,     -1,    2,   625}},
      {"DDR5_3200C",   {3200,   8,  28,  28,   28,   52,   79,   48,   12,  26,  2,    8,     8,     26+(4*8)+4,   26+(1*8)+4,  8,    16,       26+(4*8)+16,   26+(1*8)+16,   8,   -1,   -1,  -1,   -1,   -1,    -1,     30,    -1,   -1,   -1,     -1,     -1,    2,   625}},
      {"DDR5_4800B",   {4800,   8,  40,  39,   39,   77,   116,  72,   18,  38,  2,    8,     8,     38+(4*8)+6,   38+(1*8)+4,  8,    24,       38+(4*8)+24,   38+(1*8)+24,   8,   -1,   -1,  -1,   -1,   -1,    -1,     30,    -1,   -1,   -1,     -1,     -1,    2,   416}},
      {"DDR5_6400AN",  {6400,   8,  46,  46,   46,   103,  149,  97,   25,  44,  2,    8,     8,     44+(4*8)+9,   44+(1*8)+9,  8,    33,       44+(4*8)+33,   44+(1*8)+33,   8,   -1,   -1,  -1,   -1,   -1,    -1,     30,    -1,   -1,   -1,     -1,     -1,    2,   312}},
      {"DDR5_7200AN",  {7200,   8,  52,  52,   52,   116,  168,  109,  28,  50,  2,    8,     8,     50+(4*8)+9,   50+(1*8)+9,  10,   37,       50+(4*8)+37,   50+(1*8)+37,   8,   -1,   -1,  -1,   -1,   -1,    -1,     30,    -1,   -1,   -1,     -1,     -1,    2,   277}},
      {"DDR5_8000AN",  {8000,   8,  56,  56,   56,   128,  184,  120,  30,  54,  4,    8,     8,     54+(4*8)+8,   54+(1*8)+8,  20,   40,       54+(4*8)+40,   54+(1*8)+40,   8,   -1,   -1,  -1,   -1,   -1,    -1,     30,    -1,   -1,   -1,     -1,     -1,    2,   250}},
      {"DDR5_8800AN",  {8800,   8,  62,  62,   62,   141,  204,  133,  34,  60,  4,    8,     8,     60+(4*8)+6,   60+(1*8)+6,  23,   45,       60+(4*8)+45,   60+(1*8)+45,   8,   -1,   -1,  -1,   -1,   -1,    -1,     30,    -1,   -1,   -1,     -1,     -1,    2,   227}},      
    };

    inline static const std::map<std::string, std::vector<double>> voltage_presets = {
      //   name          VDD      VPP
      {"Default",       {1.1,     1.8}},
    };

    inline static const std::map<std::string, std::vector<double>> current_presets = {
      // name           IDD0  IDD2N   IDD3N   IDD4R   IDD4W   IDD5B   IPP0  IPP2N  IPP3N  IPP4R  IPP4W  IPP5B
      {"Default",       {60,   50,     55,     145,    145,    362,     3,    3,     3,     3,     3,     48}},
      {"DDR5_4800x4",   {103,  92,     142,    318,    345,    277,     8,    7,     7,     9,     36,    28}},
      {"DDR5_4800x8",   {103,  92,     142,    377,    359,    277,     8,    7,     7,     9,     37,    28}},
      {"DDR5_4800x16",  {122,  92,     142,    530,    479,    277,     10,   7,     7,     9,     64,    28}},
    };

    /*
        16Gb DDR5 SDRAM (MT60B4G4, MT60B2G8, MT60B1G16) -48B Speed Grade, 4800B nCL-nRCD-nRP (40-39-39) 
        (x4/x8/x16)
        - IDD0: 103/103/122 
        - IDD2N: 92
        - IDD3N: 142
        - IDD4R: 318/377/530
        - IDD4W: 345/359/479
        - IDD5B: 277
        - IPP0: 8/8/10
        - IPP2N: 6
        - IPP3N: 7
        - IPP4R: 9
        - IPP4W: 36/37/64
        - IPP5B: 28
    */
  /************************************************
   *                Organization
   ***********************************************/   
    const int m_internal_prefetch_size = 16;

    inline static constexpr ImplDef m_levels = {
      "channel", "pseudochannel", "narrowio", "wideio", "rank", "bankgroup", "bank", "row", "column",    
    };


  /************************************************
   *             Requests & Commands
   ***********************************************/
   // P_ACT: Pseudo-ACT 
   // Pair 
   // - P_ACT - PRE_WR or POST_RD
   // - ACT   - WR/WRA or RD/RDA 
   // - ACT   - POST_WR/POST_WRA or PRE_RD/PRE_RDA   
    inline static constexpr ImplDef m_commands = {
      "ACT",         "P_ACT",
      "PRE",         "PREA",        "PREsb",      "P_PRE",
      "RD",          "WR",          "RDA",        "WRA",
      "PRE_RD",      "PRE_WR",      "PRE_RDA",   
      "POST_RD",     "POST_WR",     "POST_WRA",
      "NDP_DRAM_RD", "NDP_DRAM_WR", "NDP_DRAM_RDA", "NDP_DRAM_WRA",
      "NDP_DB_RD",   "NDP_DB_WR",
      "REFab",       "REFsb",       "REFab_end", "REFsb_end",
      // "RFMab",  "RFMsb", "RFMab_end", "RFMsb_end",
      // "DRFMab", "DRFMsb", "DRFMab_end", "DRFMsb_end",
    };
    // NDP_DB_RD   == POST_RD, NDP_DB_WR   == PRE_WR
    // NDP_DRAM_RD == PRE_RD,  NDP_DRAM_WR == POST_WR 

    /*
      DDR Command
      - RD/WR (DRAM->DB->MC)/(MC->DB->DRAM)
      - PRE/POST RD/WR (MC->DB)/(DB->DRAM)/(DRAM->DB)/(DB->MC)
      - NDP_DRAM_RD/WR (DRAM->DB)/(DB->DRAN)
      - NDP_DB_RD/WR (DB->MC)/(MC->DB)

      BCOM Command
      N: NDP BCOM
      PP: is_PRE/POST
      P: is_PRE
      BI: Buffer Index for Pre/Post Request
      R: Reserved
      ID: NDP Exeuction ID
      C: Column Address
      NT: NDP Type(CONF_REG, INST_MEM, DAT_MEM)
      - DRAM_RD     : 2 cycle : BCOM_RD(3'b011)  → {N(1'b0),PP(1'b0),BL}
      - DRAM_WR     : 2 cycle : BCOM_WR(3'b010)  → {N(1'b0),PP(1'b0),BL}
      - PRE_RD      : 4 cycle : BCOM_RD(3'b011)  → {N(1'b0),PP(1'b1),BL} → {P(1'b1), BI2, BI1} → {BI1, BI0, R}
      - POST_RD     : 4 cycle : BCOM_RD(3'b011)  → {N(1'b0),PP(1'b1),BL} → {P(1'b0), BI2, BI1} → {BI1, BI0, R}
      - PRE_WR      : 4 cycle : BCOM_WR(3'b010)  → {N(1'b0),PP(1'b1),BL} → {P(1'b1), BI2, BI1} → {BI1, BI0, R}
      - POST_WR     : 4 cycle : BCOM_WR(3'b010)  → {N(1'b0),PP(1'b1),BL} → {P(1'b0), BI2, BI1} → {BI1, BI0, R}
      | MC - 2 cc - RCD - 4 cc - DB - max 7 cc - BUF in NDP - 4 cc - DB | 
      - NDP_DRAM_RD : 8 cycle : BCOM_RD(3'b011)  → {N(1'b1),PP(1'b0),BL} → {BG2, BG1, BG0}     → {BK1, BK0, R} → {ID2 ,ID1, ID0} → {C10, C9, C8} → {C7, C6, C5} → {C4, R, R}
      - NDP_DRAM_WR : 8 cycle : BCOM_WR(3'b010)  → {N(1'b1),PP(1'b0),BL} → {BG2, BG1, BG0}     → {BK1, BK0, R} → {ID2 ,ID1, ID0} → {C10, C9, C8} → {C7, C6, C5} → {C4, R, R}
      | MC - 2 cc - RCD - 8 cc - DB - max 7 cc - BCOM DEC of NDP - 4 cc - REG/SRAM - 4 cc - BUF - 4 cc - ECC - 4 cc - O.H (bypass) - DB | 31? (Support Only CWL > 32)
      - NDP_DB_RD   : 8 cycle : BCOM_RFU(3’b100) → {RW(1’b1), BK1, BK0}  → {BG2, BG1, BG0}     → {C10, C9, C8} → {C7, C6, C5}    → {C4, R, R}    → {R, R, R}    → {R, R, R}
      | MC - 2 cc - RCD - 8 cc - DB - max 7 cc - BCOM DEC of NDP - 4 cc - REG/SRAM - 4 cc - BUF - 4 cc - O.H (bypass) - DB | total 27 cycle (Support Only CL > 28) 
      - NDP_DB_WR   : 8 cycle : BCOM_RFU(3’b100) → {RW(1’b0), BK1, BK0}  → {BG2, BG1, BG0}     → {C10, C9, C8} → {C7, C6, C5}    → {C4, R, R}    → {R, R, R}    → {R, R, R}
    */
   

    inline static const ImplLUT m_command_scopes = LUT (
      m_commands, m_levels, {
        {"ACT",     "row"},        {"P_ACT",     "row"},
        {"PRE",     "bank"},       {"PREA",    "rank"},       {"PREsb",     "bank"},       {"P_PRE",    "bank"},
        {"RD",      "column"},     {"WR",      "column"},     {"RDA",       "column"},     {"WRA",        "column"},
        {"PRE_RD",  "column"},     {"PRE_WR",  "column"},     {"PRE_RDA",   "column"},     
        {"POST_RD", "column"},     {"POST_WR", "column"},     {"POST_WRA",  "column"},
        {"NDP_DRAM_RD", "column"}, {"NDP_DRAM_WR", "column"}, {"NDP_DRAM_RDA",  "column"}, {"NDP_DRAM_WRA",  "column"},
        {"NDP_DB_RD", "column"},   {"NDP_DB_WR", "column"}, 
        {"REFab",   "rank"},       {"REFsb",   "bank"},       {"REFab_end", "rank"},       {"REFsb_end",  "bank"},
        // {"RFMab",  "rank"},  {"RFMsb",  "bank"}, {"RFMab_end",  "rank"},  {"RFMsb_end",  "bank"},
        // {"DRFMab", "rank"},  {"DRFMsb", "bank"}, {"DRFMab_end", "rank"},  {"DRFMsb_end", "bank"},
      }
    );

    inline static const ImplLUT m_command_meta = LUT<DRAMCommandMeta> (
      m_commands, {
                      // open?   close?   access?  refresh?
        {"ACT",         {true,   false,   false,   false}},
        {"P_ACT",       {true,   false,   false,   false}},
        {"PRE",         {false,  true,    false,   false}},
        {"PREA",        {false,  true,    false,   false}},
        {"PREsb",       {false,  true,    false,   false}},
        {"P_PRE",       {false,  true,    false,   false}},
        {"RD",          {false,  false,   true,    false}},
        {"WR",          {false,  false,   true,    false}},
        {"RDA",         {false,  true,    true,    false}},
        {"WRA",         {false,  true,    true,    false}},
        {"PRE_RD",      {false,  false,   true,    false}},
        {"PRE_WR",      {false,  false,   false,   false}},
        {"POST_RD",     {false,  false,   false,   false}},
        {"POST_WR",     {false,  false,   true,    false}},
        {"PRE_RDA",     {false,  true,    true,    false}},
        {"POST_WRA",    {false,  true,    true,    false}},
        {"NDP_DRAM_RD", {false,  false,   true,    false}},
        {"NDP_DRAM_WR", {false,  false,   true,    false}},
        {"NDP_DRAM_RDA",{false,  true,    true,    false}},
        {"NDP_DRAM_WRA",{false,  true,    true,    false}},
        {"NDP_DB_RD",   {false,  false,   false,   false}},
        {"NDP_DB_WR",   {false,  false,   false,   false}},
        {"REFab",       {false,  false,   false,   true }},
        {"REFsb",       {false,  false,   false,   true }},
        {"REFab_end",   {false,  true,    false,   false}},
        {"REFsb_end",   {false,  true,    false,   false}},
        // {"RFMab",       {false,  false,   false,   true }},
        // {"RFMsb",       {false,  false,   false,   true }},
        // {"RFMab_end",   {false,  true,    false,   false}},
        // {"RFMsb_end",   {false,  true,    false,   false}},
        // {"DRFMab",      {false,  false,   false,   true }},
        // {"DRFMsb",      {false,  false,   false,   true }},
        // {"DRFMab_end",  {false,  true,    false,   false}},
        // {"DRFMsb_end",  {false,  true,    false,   false}},
      }
    );

    inline static constexpr ImplDef m_requests = {
      "read", "write", 
      "all-bank-refresh", "same-bank-refresh", 
      // "rfm", "same-bank-rfm",
      // "directed-rfm", "same-bank-directed-rfm",
      "open-row", "close-row",
      "ndp-db-read",   "ndp-db-write",
      "ndp-dram-read", "ndp-dram-write",
    };

    inline static const ImplLUT m_request_translations = LUT (
      m_requests, m_commands, {
        {"read", "RD"}, {"write", "WR"}, 
        {"all-bank-refresh", "REFab"}, {"same-bank-refresh", "REFsb"}, 
        // {"rfm", "RFMab"}, {"same-bank-rfm", "RFMsb"}, 
        // {"directed-rfm", "DRFMab"}, {"same-bank-directed-rfm", "DRFMsb"}, 
        {"open-row", "ACT"}, {"close-row", "PRE"},
        {"ndp-db-read", "NDP_DB_RD"}, {"ndp-db-write", "NDP_DB_WR"},
        {"ndp-dram-read", "NDP_DRAM_RD"}, {"ndp-dram-write", "NDP_DRAM_WR"},
      }
    );

  /************************************************
   *                   Timing
   ***********************************************/
    inline static constexpr ImplDef m_timings = {
      "rate", 
      "nBL", "nCL", "nRCD", "nRP", "nRAS", "nRC", "nWR", "nRTP", "nCWL",
      "nPPD",
      "nCCDS", "nCCDS_WR", "nCCDS_WTR", "nCCDS_WTR_WI",
      "nCCDL", "nCCDL_WR", "nCCDL_WTR", "nCCDL_WTR_WI",
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
      "ACT", "PRE", "RD", "WR", "REF", "RFM", "DRAM2DB_RD", "DB2DRAM_WR", "DB2MC_RD", "MC2DB_WR"
    };

  /************************************************
   *                 Node States
   ***********************************************/
    inline static constexpr ImplDef m_states = {
       "Opened", "Closed", "PowerUp", "N/A", "Refreshing"
    };

    inline static constexpr ImplDef m_f_states = {
      "Opened", "Closed", "N/A"
    };

    inline static const ImplLUT m_init_states = LUT (
      m_levels, m_states, {
        {"channel",         "N/A"}, 
        {"pseudochannel",   "N/A"}, 
        {"narrowio",        "N/A"}, 
        {"wideio",          "N/A"}, 
        {"rank",            "PowerUp"},
        {"bankgroup",       "N/A"},
        {"bank",            "Closed"},
        {"row",             "Closed"},
        {"column",          "N/A"},
      }
    );

    inline static const ImplLUT m_init_f_states = LUT (
      m_levels, m_states, {
        {"channel",         "N/A"}, 
        {"pseudochannel",   "N/A"}, 
        {"narrowio",        "N/A"}, 
        {"wideio",          "N/A"}, 
        {"rank",            "N/A"},
        {"bankgroup",       "N/A"},
        {"bank",            "Closed"},
        {"row",             "Closed"},
        {"column",          "N/A"},
      }
    );

    inline static constexpr ImplDef m_ndp_status = {
      "idle", "run", "barrier", "wait_done", "done"
   };

  public:
    struct Node : public DRAMNodeBase<DDR5PCH> {
      Node(DDR5PCH* dram, Node* parent, int level, int id) : DRAMNodeBase<DDR5PCH>(dram, parent, level, id) {};
    };
    std::vector<Node*> m_channels;
    
    FuncMatrix<ActionFunc_t<Node>>  m_actions;
    FuncMatrix<PreqFunc_t<Node>>    m_preqs;
    FuncMatrix<RowhitFunc_t<Node>>  m_rowhits;
    FuncMatrix<RowopenFunc_t<Node>> m_rowopens;
    FuncMatrix<PowerFunc_t<Node>>   m_powers;

    double s_total_rfm_energy = 0.0;

    std::vector<size_t> s_total_rfm_cycles;
    std::vector<bool>   each_pch_refreshing;
    int num_pseudo_ch = 0; 
    std::vector<int> db_prefetch_cnt_per_pch;
    std::vector<int> db_prefetch_rd_cnt_per_pch;
    std::vector<int> db_prefetch_wr_cnt_per_pch;
    std::vector<int> pre_wr_cnt_per_ch;
    std::vector<int> post_wr_cnt_per_ch;

    std::vector<std::vector<bool>> need_be_open_per_bank;
    int m_num_channels;          
    int m_num_pseudochannel;     
    int m_num_ranks;             
    int m_num_bankgroups;        
    int m_num_banks;             
    bool m_use_pch;
    bool m_use_prefetch;

    std::vector<bool> m_high_pri_prefetch;
    std::vector<int>  m_db_prefetch_mode;

    const int MODE_PRE_RD  = 0;
    const int MODE_POST_RD = 1;
    const int MODE_PRE_WR  = 2;
    const int MODE_POST_WR = 3;

    int ndp_access_row; 
    int ndp_ctrl_access_bk;
    int ndp_ctrl_access_bg; 
    int ndp_ins_mem_access_bk;
    int ndp_ins_mem_access_bg;
    int ndp_dat_mem_access_bk;
    int ndp_dat_mem_access_bg;    
    // NDP Unit per pch 
    std::vector<std::vector<uint64_t>>  ins_mem_per_pch;
    std::vector<std::vector<uint64_t>>  dat_mem_per_pch;
    std::vector<int>                    ndp_status_per_pch;
    std::vector<int>                    ndp_pc_per_pch;
    std::vector<std::vector<Inst_Slot>> ndp_inst_slot_per_pch; 

    // MC -> RCD -> BD Clock
    std::vector<std::vector<int>>                   pipe_ndp_latency_per_pch;
    std::vector<std::vector<int>>                   pipe_ndp_cmd_per_pch;
    std::vector<std::vector<AddrVec_t>>             pipe_ndp_addr_per_pch;
    std::vector<std::vector<int>>                   pipe_ndp_id_per_pch;
    std::vector<std::vector<bool>>                  pipe_ndp_payload_valid_per_pch;
    std::vector<bool>                               ndp_valid_per_pch;
    std::vector<int>                                ndp_cmd_per_pch;
    std::vector<AddrVec_t>                          ndp_addr_per_pch;    
    std::vector<int>                                ndp_id_per_pch;    
    std::vector<bool>                               ndp_payload_valid_per_pch;    
    std::vector<std::vector<std::vector<uint64_t>>> pipe_ndp_payload_per_pch;

    /*
      x4/x8 DRAM: 8BG,4BK per BG --> 32 BK
      CHx/PCHx/BK3/BG7/ROW(Highest Row): HSNC-Ctrl Reg
      CHx/PCHx/BK3/BG6/ROW(Highest Row)/COL0-127: HSNC-AccessInfo Buf
      CHv/PCHv/BK3/BG5/ROW(Highest Row)/COL0-127: DSNU-Ctrl Reg (8KB) 
      CHv/PCHv/BK3/BG4/ROW(Highest Row)/COL0-127: Inst Memory (8KB/8KB)
      CHv/PCHv/BK3/BG3/ROW(Highest Row)/COL0-127: Data Memory (8KB/32KB)
      CHv/PCHv/BK3/BG2/ROW(Highest Row)/COL0-127: Data Memory (8KB/32KB)
      CHv/PCHv/BK3/BG1/ROW(Highest Row)/COL0-127: Data Memory (8KB/32KB)
      CHv/PCHv/BK3/BG0/ROW(Highest Row)/COL0-127: Data Memory (8KB/32KB)
      CHv/PCHv/BK2/BG0-BG7/ROW(Highest Row)/COL0-127: Reserved
      CHv/PCHv/BK1/BG0-BG7/ROW(Highest Row)/COL0-127: Reserved
      CHv/PCHv/BK0/BG0-BG7/ROW(Highest Row)/COL0-127: Reserved
      x16 DRAM: 4BG,4BK per BG --> 16 BK
      CHx/PCHx/BK3/BG3/ROW(Highest Row): HSNC-Ctrl Reg
      CHx/PCHx/BK3/BG2/ROW(Highest Row)/COL0-127: HSNC-AccessInfo Buf
      CHv/PCHv/BK3/BG1/ROW(Highest Row)/COL0-127: DSNU-Ctrl Reg (8KB) 
      CHv/PCHv/BK3/BG0/ROW(Highest Row)/COL0-127: Inst Memory (8KB/8KB)
      CHv/PCHv/BK2/BG3/ROW(Highest Row)/COL0-127: Data Memory (8KB/32KB)
      CHv/PCHv/BK2/BG2/ROW(Highest Row)/COL0-127: Data Memory (8KB/32KB)
      CHv/PCHv/BK2/BG1/ROW(Highest Row)/COL0-127: Data Memory (8KB/32KB)
      CHv/PCHv/BK2/BG0/ROW(Highest Row)/COL0-127: Data Memory (8KB/32KB)
      CHv/PCHv/BK1/BG0-BG3/ROW(Highest Row)/COL0-127: Reserved
      CHv/PCHv/BK0/BG0-BG3/ROW(Highest Row)/COL0-127: Reserved    
    */
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

      // NDP Unit tick()
      if(m_clk%4 == 0) {
        // NDP Unit Clock is 1/4 of DRAM clock
        // iteration each channel and pseudo channel
        for(int ch=0;ch<m_num_channels;ch++) {
          for(int pch=0;pch<m_num_pseudochannel;pch++) {
            int pch_idx = ch*m_num_pseudochannel + pch;

            // Fetch Instruction from instruction memory
            if(ndp_status_per_pch[pch_idx] == m_ndp_status("run")) {              
              if(ndp_inst_slot_per_pch[pch_idx].size() < 16) {
                std::cout<<"["<<m_clk<<"] CH["<<ch<<"] PCH["<<pch<<"] - ";
                Inst_Slot inst = decoding_inst(ins_mem_per_pch[pch_idx][ndp_pc_per_pch[pch_idx]]);
                if(inst.opcode == 48) {
                  ndp_status_per_pch[pch_idx] = m_ndp_status("barrier");
                  DEBUG_PRINT(m_clk, "NDP Unit", ch, pch," Status run -> barrier"); 
                } else if(inst.opcode == 49) {
                  ndp_status_per_pch[pch_idx] = m_ndp_status("wait_done");
                } else {
                  ndp_inst_slot_per_pch[pch_idx].push_back(inst);
                }
                ndp_pc_per_pch[pch_idx]++;
              }                            
            } else if(ndp_status_per_pch[pch_idx] == m_ndp_status("barrier")) {
              if(ndp_inst_slot_per_pch[pch_idx].size() == 0) {
                ndp_status_per_pch[pch_idx] = m_ndp_status("run");
                DEBUG_PRINT(m_clk, "NDP Unit", ch, pch," Status barrier -> run"); 
              }
            } else if(ndp_status_per_pch[pch_idx] == m_ndp_status("wait_done")) {
              if(ndp_inst_slot_per_pch[pch_idx].size() == 0) {
                ndp_status_per_pch[pch_idx] = m_ndp_status("done");
                DEBUG_PRINT(m_clk, "NDP Unit", ch, pch," Status wait_done -> done"); 
              }
            } else if(ndp_status_per_pch[pch_idx] == m_ndp_status("done")) {
              
            } else if(ndp_status_per_pch[pch_idx] == m_ndp_status("idle")) {

            }

            if(ndp_valid_per_pch[pch_idx]) {
              ndp_valid_per_pch[pch_idx] =  false;
              DEBUG_PRINT(m_clk, "NDP Unit", ch, pch, (std::string(" Receive : ") + std::string(m_commands(ndp_cmd_per_pch[pch_idx]))));                                              
              
              if(ndp_cmd_per_pch[pch_idx] == m_commands["NDP_DB_RD"] || ndp_cmd_per_pch[pch_idx] == m_commands["NDP_DB_WR"]) {
                if(ndp_addr_per_pch[pch_idx][m_levels["row"]] != ndp_access_row) {
                  throw std::runtime_error("Invalid Address to access NDP!");
                } else {
                  if(ndp_addr_per_pch[pch_idx][m_levels["bank"]] == ndp_ctrl_access_bk && ndp_addr_per_pch[pch_idx][m_levels["bankgroup"]] == ndp_ctrl_access_bg) {
                    // Access NDP Configuration Register 
                    if(ndp_cmd_per_pch[pch_idx] == m_commands["NDP_DB_WR"]) {
                      if(ndp_payload_valid_per_pch[pch_idx]) {
                        if(pipe_ndp_payload_per_pch[pch_idx][0].size() != 8) {
                          throw std::runtime_error("Invalid Payload Size!!");
                        } else {
                          if(pipe_ndp_payload_per_pch[pch_idx][0][0] == 1) {
                            if(ndp_status_per_pch[pch_idx] != m_ndp_status("idle")) {
                              DEBUG_PRINT(m_clk, "NDP Unit", ch, pch, (std::string("NDP status : ") + std::string(m_ndp_status(ndp_status_per_pch[pch_idx]))));                                              
                              throw std::runtime_error("NDP Unit start when is not idle");
                            } else {
                              DEBUG_PRINT(m_clk, "NDP Unit", ch, pch,"Start NDP Operation");                                              
                              ndp_status_per_pch[pch_idx] = m_ndp_status("run");
                            }
                          }                        
                        }
                      } 
                    }

                  } else if(ndp_addr_per_pch[pch_idx][m_levels["bank"]] == ndp_ins_mem_access_bk && ndp_addr_per_pch[pch_idx][m_levels["bankgroup"]] == ndp_ins_mem_access_bg) {
                    // Access NDP Instruction Memory
                    if(ndp_cmd_per_pch[pch_idx] == m_commands["NDP_DB_WR"]) {

                      if(ndp_status_per_pch[pch_idx] != m_ndp_status("idle")) {
                        throw std::runtime_error("NDP Write when NDP unit is exec!");
                      }

                      if(ndp_payload_valid_per_pch[pch_idx]) {
                        if(pipe_ndp_payload_per_pch[pch_idx][0].size() != 8) {
                          throw std::runtime_error("Invalid Payload Size!!");
                        } else {
                          for(int i=0;i<8;i++) {
                            ins_mem_per_pch[pch_idx][8*ndp_addr_per_pch[pch_idx][m_levels["column"]] + i] = pipe_ndp_payload_per_pch[pch_idx][0][i];
                            std::cout<<"CH["<<ch<<"] PCH["<<pch;
                            std::cout<<"] - Insert Insruction Memory COL["<<ndp_addr_per_pch[pch_idx][m_levels["column"]]<<"]["<<i<<"]: 0x";
                            std::cout<<std::hex<<ins_mem_per_pch[pch_idx][8*ndp_addr_per_pch[pch_idx][m_levels["column"]] + i]<<std::dec<<std::endl;
                          }
                        }
                      } 
                    }
                  } else if(ndp_addr_per_pch[pch_idx][m_levels["bank"]] <= ndp_dat_mem_access_bk && ndp_addr_per_pch[pch_idx][m_levels["bankgroup"]] <= ndp_dat_mem_access_bg) {
                    // Access NDP Data Memory
                    // dat_mem_per_pch
                    
                  } else {
                    // Access Not Mapped Address
                    throw std::runtime_error("Invalid Not Mapped NDP Address!");
                  }
                }
              } else if(ndp_cmd_per_pch[pch_idx] == m_commands["NDP_DRAM_RD"] || ndp_cmd_per_pch[pch_idx] == m_commands["NDP_DRAM_RDA"]) {
                // NDP Exeuction with RD Data   
                if(ndp_status_per_pch[pch_idx] == m_ndp_status("idle")) {
                  throw std::runtime_error("NDP DRAM RD when NDP is idle!!");
                } else {
                  if(ndp_inst_slot_per_pch[pch_idx].size() == 0) {
                    throw std::runtime_error("NDP DRAM RD when ndp_inst_slot is empty!");
                  } else {
                    // Find Matching Inst
                    bool is_find = false;
                    int match_idx = -1;
                    for(int i=0;i<ndp_inst_slot_per_pch[pch_idx].size();i++) {
                      if(ndp_inst_slot_per_pch[pch_idx][i].id == ndp_id_per_pch[pch_idx] &&
                         ndp_inst_slot_per_pch[pch_idx][i].bg == ndp_addr_per_pch[pch_idx][m_levels["bankgroup"]] &&
                         ndp_inst_slot_per_pch[pch_idx][i].bk == ndp_addr_per_pch[pch_idx][m_levels["bank"]]) {
                        is_find = true;
                        match_idx = i;
                      }
                      if(is_find) break;
                    }
                    if(!is_find) {
                      throw std::runtime_error("Cannot Find Matched Instruction with NDP DRAM RD!!");
                    } else {
                      // If Opsize and Counter is equal, the ndp_inst is done, so remove this ndp_isnt from ndp_inst_slot
                      if(ndp_inst_slot_per_pch[pch_idx][match_idx].opsize == ndp_inst_slot_per_pch[pch_idx][match_idx].cnt) {
                        ndp_inst_slot_per_pch[pch_idx].erase(ndp_inst_slot_per_pch[pch_idx].begin() + match_idx);
                        DEBUG_PRINT(m_clk, "NDP Unit", ch, pch, " Remove Done Request!!");                         
                      } else {
                        ndp_inst_slot_per_pch[pch_idx][match_idx].cnt++;
                      }
                    }
                  }              
                }                
                
              } else if(ndp_cmd_per_pch[pch_idx] == m_commands["NDP_DRAM_WR"] || ndp_cmd_per_pch[pch_idx] == m_commands["NDP_DRAM_WRA"]) {
                // NDP Write Back to DRAM (Data Memory)
                if(ndp_status_per_pch[pch_idx] == m_ndp_status("idle")) {
                  throw std::runtime_error("NDP DRAM WR when NDP is idle!!");
                } else {
                  if(ndp_inst_slot_per_pch[pch_idx].size() == 0) {
                    throw std::runtime_error("NDP DRAM WR when ndp_inst_slot is empty!");
                  } else {
                    // Find Matching Inst
                    bool is_find = false;
                    int match_idx = -1;
                    for(int i=0;i<ndp_inst_slot_per_pch[pch_idx].size();i++) {
                      if(ndp_inst_slot_per_pch[pch_idx][i].id == ndp_id_per_pch[pch_idx] &&
                         ndp_inst_slot_per_pch[pch_idx][i].bg == ndp_addr_per_pch[pch_idx][m_levels["bankgroup"]] &&
                         ndp_inst_slot_per_pch[pch_idx][i].bk == ndp_addr_per_pch[pch_idx][m_levels["bank"]]) {
                        is_find = true;
                        match_idx = i;
                      }
                      if(is_find) break;
                    }
                    if(!is_find) {
                      throw std::runtime_error("Cannot Find Matched Instruction with NDP DRAM RD!!");
                    } else {
                      // If Opsize and Counter is equal, the ndp_inst is done, so remove this ndp_isnt from ndp_inst_slot
                      if(ndp_inst_slot_per_pch[pch_idx][match_idx].opsize == ndp_inst_slot_per_pch[pch_idx][match_idx].cnt) {
                        ndp_inst_slot_per_pch[pch_idx].erase(ndp_inst_slot_per_pch[pch_idx].begin() + match_idx);
                      } else {
                        ndp_inst_slot_per_pch[pch_idx][match_idx].cnt++;
                      }
                    }
                  }              
                }                
                // NDP Exeuction with RD Data
              }

              // Remove Used Data 
              if(ndp_payload_valid_per_pch[pch_idx]) {
                ndp_payload_valid_per_pch[pch_idx] = false;
                // remove first vector 
                if(pipe_ndp_payload_per_pch[pch_idx].size() == 0) {
                  throw std::runtime_error("Remove Empty pipe_ndp_payload_per_pch");
                } else {
                  pipe_ndp_payload_per_pch[pch_idx].erase(pipe_ndp_payload_per_pch[pch_idx].begin());
                }
              }
              
            } 
          } // PCH 
        } // CH
      } // NDP_CLK 

      // DB -> NDP Unit
      for(int pch_idx=0;pch_idx<m_num_channels*m_num_pseudochannel;pch_idx++) {
        if(pipe_ndp_latency_per_pch[pch_idx].size() != 0) {
          for(int i=0;i<pipe_ndp_latency_per_pch[pch_idx].size();i++) {
            pipe_ndp_latency_per_pch[pch_idx][i]--;
            if(pipe_ndp_latency_per_pch[pch_idx][i]<0) {
              throw std::runtime_error("Timing Error from MC to DB!");
            }
          }
          if(pipe_ndp_latency_per_pch[pch_idx][0] == 0) {
            if(ndp_valid_per_pch[pch_idx]) {
              throw std::runtime_error("Collision Bus DB to NDP Unit");
            }
            ndp_valid_per_pch[pch_idx]         = true;
            ndp_cmd_per_pch[pch_idx]           = pipe_ndp_cmd_per_pch[pch_idx][0];
            ndp_addr_per_pch[pch_idx]          = pipe_ndp_addr_per_pch[pch_idx][0];
            ndp_id_per_pch[pch_idx]            = pipe_ndp_id_per_pch[pch_idx][0];
            ndp_payload_valid_per_pch[pch_idx] = pipe_ndp_payload_valid_per_pch[pch_idx][0];
            // Remove First Element of vector
            pipe_ndp_latency_per_pch[pch_idx].erase(pipe_ndp_latency_per_pch[pch_idx].begin());
            pipe_ndp_cmd_per_pch[pch_idx].erase(pipe_ndp_cmd_per_pch[pch_idx].begin());
            pipe_ndp_addr_per_pch[pch_idx].erase(pipe_ndp_addr_per_pch[pch_idx].begin());
            pipe_ndp_id_per_pch[pch_idx].erase(pipe_ndp_id_per_pch[pch_idx].begin());
            pipe_ndp_payload_valid_per_pch[pch_idx].erase(pipe_ndp_payload_valid_per_pch[pch_idx].begin());
            // std::cout<<"["<<m_clk<<"] CH["<<ndp_addr_per_pch[pch_idx][m_levels["channel"]]<<"] PCH["<<ndp_addr_per_pch[pch_idx][m_levels["pseudochannel"]];
            // std::cout<<"] ISSUE COMMAND from DB to NDP : "<<m_commands(ndp_cmd_per_pch[pch_idx])<<std::endl;
          }
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

      Logger_t m_logger;
      m_logger = Logging::create_logger("DDR5-PCH");
      m_logger->info("DRAM init()");      
      m_logger->info(" Address Space (Row) of NDP UNit       : {}",ndp_access_row);
      m_logger->info(" Address Space (Bank) of NDP Unit Ctrl : {}",ndp_ctrl_access_bk);
      m_logger->info(" Address Space (BankGroup) of NDP Unit Ctrl : {}",ndp_ctrl_access_bg);
      m_logger->info(" Address Space (Bank) of NDP Unit Instruction Memory : {}",ndp_ins_mem_access_bk);
      m_logger->info(" Address Space (BankGroup) of NDP Unit Instruction Memory : {}",ndp_ins_mem_access_bg);     
      m_logger->info(" Address Space (0<=Bank) of NDP Unit Instruction Memory : {}",ndp_dat_mem_access_bk);
      m_logger->info(" Address Space (0<=BankGroup) of NDP Unit Instruction Memory : {}",ndp_dat_mem_access_bg);            
    };

    void issue_command(int command, const AddrVec_t& addr_vec) override {
      int channel_id = addr_vec[m_levels["channel"]];

      m_channels[channel_id]->update_timing(command, addr_vec, m_clk);
      m_channels[channel_id]->update_powers(command, addr_vec, m_clk);
      m_channels[channel_id]->update_states(command, addr_vec, m_clk);  

      
      if(command == m_commands("PRE_WR")){
        pre_wr_cnt_per_ch[channel_id]++;
      }
      if(command == m_commands("POST_WR")) {
        post_wr_cnt_per_ch[channel_id]++;
      }
                       
      // Check if the command requires future action
      check_future_action(command, addr_vec);
    };
    
    void issue_ndp_command(int command, const AddrVec_t& addr_vec, int thread_id, const std::vector<uint64_t> payload) override {
      // NDP-related Code
      // "NDP_DRAM_RD", "NDP_DRAM_WR", "NDP_DRAM_RDA", "NDP_DRAM_WRA", "NDP_DB_RD",   "NDP_DB_WR",
      // std::cout<<"["<<m_clk<<"] CH["<<addr_vec[m_levels["channel"]]<<"] PCH["<<addr_vec[m_levels["pseudochannel"]]<<"] ISSUE COMMAND from MC to DB : "<<m_commands(command)<<std::endl;
      int pch_idx = addr_vec[m_levels["channel"]]*num_pseudo_ch + addr_vec[m_levels["pseudochannel"]];
      // Write Payload..
      if(payload.size()!=0) {
        if(payload.size() != 8) {
          throw std::runtime_error("Invalid Payload Size!!");
        } else {
          pipe_ndp_payload_per_pch[pch_idx].push_back(payload);
        }
        pipe_ndp_payload_valid_per_pch[pch_idx].push_back(true);
      } else {
        pipe_ndp_payload_valid_per_pch[pch_idx].push_back(false);
      }
      pipe_ndp_latency_per_pch[pch_idx].push_back(10);
      pipe_ndp_cmd_per_pch[pch_idx].push_back(command);
      pipe_ndp_addr_per_pch[pch_idx].push_back(addr_vec);      
      pipe_ndp_id_per_pch[pch_idx].push_back(thread_id);      
    }

    void check_future_action(int command, const AddrVec_t& addr_vec) {
      switch (command) {
        case m_commands("REFab"):
          // Psuedo Channel State Chagne to Refresing..
          each_pch_refreshing[num_pseudo_ch*addr_vec[0]+addr_vec[1]]=true;
          // db_prefetch_change_mode(addr_vec[0],addr_vec[1]);
          // std::cout<<"["<<m_clk<<"] REFab Start CH["<<addr_vec[0]<<"]PCH["<<addr_vec[1]<<"] MODE"<<m_db_prefetch_mode[num_pseudo_ch*addr_vec[0]+addr_vec[1]]<<" | ";
          // std::cout<<db_prefetch_cnt_per_pch[num_pseudo_ch*addr_vec[0]+addr_vec[1]];
          // std::cout<<" / "<<db_prefetch_rd_cnt_per_pch[num_pseudo_ch*addr_vec[0]+addr_vec[1]];
          // std::cout<<" / "<<db_prefetch_wr_cnt_per_pch[num_pseudo_ch*addr_vec[0]+addr_vec[1]]<<std::endl;
          // if(db_prefetch_cnt_per_pch[num_pseudo_ch*addr_vec[0]+addr_vec[1]] > 0) exit(1);
          m_future_actions.push_back({command, addr_vec, m_clk + m_timing_vals("nRFC1") - 1});
          break;
        case m_commands("REFsb"):
          m_future_actions.push_back({command, addr_vec, m_clk + m_timing_vals("nRFCsb") - 1});
          break;
        // case m_commands("RFMab"):
        //   m_future_actions.push_back({command, addr_vec, m_clk + m_timing_vals("nRFM1") - 1});
        //   break;
        // case m_commands("RFMsb"):
        //   m_future_actions.push_back({command, addr_vec, m_clk + m_timing_vals("nRFMsb") - 1});
        //   break;
        // case m_commands("DRFMab"):
        //   m_future_actions.push_back({command, addr_vec, m_clk + m_timing_vals("nDRFMab") - 1});
        //   break;
        // case m_commands("DRFMsb"):
        //   m_future_actions.push_back({command, addr_vec, m_clk + m_timing_vals("nDRFMsb") - 1});
        //   break;
        default:
          // Other commands do not require future actions
          break;
      }
    }

    void handle_future_action(int command, const AddrVec_t& addr_vec) {
      int channel_id = addr_vec[m_levels["channel"]];
      switch (command) {
        case m_commands("REFab"):
          // Refresh Done!
          each_pch_refreshing[num_pseudo_ch*addr_vec[0]+addr_vec[1]]=false;
          // db_prefetch_change_mode(addr_vec[0],addr_vec[1]);
          // std::cout<<"["<<m_clk<<"] REFab Done CH["<<addr_vec[0]<<"]PCH["<<addr_vec[1]<<"] "<<db_prefetch_cnt_per_pch[num_pseudo_ch*addr_vec[0]+addr_vec[1]]<<std::endl;
          m_channels[channel_id]->update_powers(m_commands("REFab_end"), addr_vec, m_clk);
          m_channels[channel_id]->update_states(m_commands("REFab_end"), addr_vec, m_clk);
          break;
        case m_commands("REFsb"):
          m_channels[channel_id]->update_powers(m_commands("REFsb_end"), addr_vec, m_clk);
          m_channels[channel_id]->update_states(m_commands("REFsb_end"), addr_vec, m_clk);
          break;
        // case m_commands("RFMab"):
        //   m_channels[channel_id]->update_powers(m_commands("RFMab_end"), addr_vec, m_clk);
        //   m_channels[channel_id]->update_states(m_commands("RFMab_end"), addr_vec, m_clk);
        //   break;
        // case m_commands("RFMsb"):
        //   m_channels[channel_id]->update_powers(m_commands("RFMsb_end"), addr_vec, m_clk);
        //   m_channels[channel_id]->update_states(m_commands("RFMsb_end"), addr_vec, m_clk);
        //   break;
        // case m_commands("DRFMab"):
        //   m_channels[channel_id]->update_powers(m_commands("DRFMab_end"), addr_vec, m_clk);
        //   m_channels[channel_id]->update_states(m_commands("DRFMab_end"), addr_vec, m_clk);
        //   break;
        // case m_commands("DRFMsb"):
        //   m_channels[channel_id]->update_powers(m_commands("DRFMsb_end"), addr_vec, m_clk);
        //   m_channels[channel_id]->update_states(m_commands("DRFMsb_end"), addr_vec, m_clk);
        //   break;
        default:
          // Other commands do not require future actions
          break;
      }
    };

    int get_preq_command(int command, const AddrVec_t& addr_vec) override {
      int channel_id = addr_vec[m_levels["channel"]];        
      return m_channels[channel_id]->get_preq_command(command, addr_vec, m_clk);
    };

    int get_preq_command_refresh_ch(int command, const AddrVec_t& addr_vec) override {
      int channel_id = addr_vec[m_levels["channel"]];
      // std::cout<<"DDR5-pCH::Get Preq Command!"<<std::endl;
      // Check Each Pseudo Channel is Refreshing, Issue PRE_WR to DB (Not to DRAM)
      int new_command;
      bool is_enable_wr_prefetch = m_db_prefetch_mode[addr_vec[0]*num_pseudo_ch+addr_vec[1]] == MODE_PRE_WR;
      if(each_pch_refreshing[addr_vec[0]*num_pseudo_ch+addr_vec[1]] && is_enable_wr_prefetch &&
        ((command == m_commands("WR")) || (command == m_commands("WRA")))) {      
          new_command = m_commands("PRE_WR");
      } else {
          new_command = command;
      }               
      return m_channels[channel_id]->get_preq_command(new_command, addr_vec, m_clk);      
    };

    int get_preq_pre_command(int command, const AddrVec_t& addr_vec) override {
      int channel_id = addr_vec[m_levels["channel"]];    
      // get PRE-* Command 
      // RD/RDA --> PRE_RD
      // WR/WRA --> PRE_WR
      // NDP_DB_WR --> NDP_DB_WR
      // NDP_DB_RD --> NDP_DB_RD
      // NDP_DRAM_RD --> NDP_DRAM_RD
      // NDP_DRAM_WR --> NDP_DRAM_WR 
      int new_command;
      if((command == m_commands("WR")) || (command == m_commands("WRA"))) {      
          new_command = m_commands("PRE_WR");
      } else if((command == m_commands("RD")) || (command == m_commands("RDA"))) {
          new_command = m_commands("PRE_RD");
      } else {
          new_command = command;
      }              
      return m_channels[channel_id]->get_preq_command(new_command, addr_vec, m_clk);      
    };    


    bool check_ready(int command, const AddrVec_t& addr_vec) override {
      int channel_id = addr_vec[m_levels["channel"]];
      bool is_ready = true;
      if(m_command_meta[command].is_closing && get_need_be_open_per_bank(addr_vec)) is_ready = false;
      return is_ready && m_channels[channel_id]->check_ready(command, addr_vec, m_clk);
    };

    bool check_rowbuffer_hit(int command, const AddrVec_t& addr_vec) override {
      int channel_id = addr_vec[m_levels["channel"]];
      return m_channels[channel_id]->check_rowbuffer_hit(command, addr_vec, m_clk);
    };
    
    bool check_node_open(int command, const AddrVec_t& addr_vec) override {
      int channel_id = addr_vec[m_levels["channel"]];
      return m_channels[channel_id]->check_node_open(command, addr_vec, m_clk);
    };

    void db_prefetch_change_mode(int _ch_idx, int _pch_idx) {
      /*
      int pch_idx = _ch_idx*num_pseudo_ch+_pch_idx;
      if(m_db_prefetch_mode[pch_idx] == MODE_POST_WR) {
        if((db_prefetch_wr_cnt_per_pch[pch_idx] == 0) && m_enable_rd_prefetch[pch_idx]) {
          // Prefetch RD Mode from DRAM to DB
          m_db_prefetch_mode[pch_idx] = MODE_PRE_RD;        
          // std::cout<<"["<<_ch_idx<<"] ["<<_pch_idx<<"] MODE Change from MODE_POST_WR to MODE_PRE_RD"<<std::endl;
        }
      }
      else if(m_db_prefetch_mode[pch_idx] == MODE_PRE_WR) {
        if(!each_pch_refreshing[pch_idx]) { 
          m_db_prefetch_mode[pch_idx] = MODE_POST_WR;
          // std::cout<<"["<<_ch_idx<<"] ["<<_pch_idx<<"] MODE Change from MODE_PRE_WR to MODE_POST_WR"<<std::endl;
        }
      }
      else if(m_db_prefetch_mode[pch_idx] == MODE_POST_RD) {
        if(db_prefetch_rd_cnt_per_pch[pch_idx]==0) {
          if(each_pch_refreshing[pch_idx]) {
            m_db_prefetch_mode[pch_idx] = MODE_PRE_WR;
            // std::cout<<"["<<_ch_idx<<"] ["<<_pch_idx<<"] MODE Change from MODE_POST_RD to MODE_PRE_WR"<<std::endl;
          } else {
            m_db_prefetch_mode[pch_idx] = MODE_POST_WR;
            // std::cout<<"["<<_ch_idx<<"] ["<<_pch_idx<<"] MODE Change from MODE_POST_RD to MODE_POST_WR"<<std::endl;
          }           
        }
      } else { 
        // MODE_PRE_RD
        if(!m_enable_rd_prefetch[pch_idx]) {
          m_db_prefetch_mode[pch_idx] = MODE_POST_RD;
          // std::cout<<"["<<_ch_idx<<"] ["<<_pch_idx<<"] MODE Change from MODE_PRE_RD to MODE_POST_RD - "<<db_prefetch_rd_cnt_per_pch[pch_idx]<<std::endl;
        }
      }
      */
    };

    bool check_dram_refrsehing() override {
      bool is_refreshing = false;
      for(int i=0;i<each_pch_refreshing.size();i++) {
          if(each_pch_refreshing[i]) is_refreshing = true;
      }      
      return is_refreshing;
    };

    bool check_ch_refrsehing(const AddrVec_t& addr_vec) override {
      bool is_refreshing = each_pch_refreshing[addr_vec[0]*num_pseudo_ch+addr_vec[1]];
      return is_refreshing;
    };

    bool check_pch_refrsehing_by_idx(int ch_idx, int pch_idx) override {
      bool is_refreshing = each_pch_refreshing[ch_idx*num_pseudo_ch+pch_idx];
      return is_refreshing;
    };    

    int get_db_fetch_per_pch(const AddrVec_t& addr_vec) override {
      return db_prefetch_cnt_per_pch[addr_vec[0]*num_pseudo_ch+addr_vec[1]];
    };

    int get_db_rd_fetch_per_pch(const AddrVec_t& addr_vec) override {
      return db_prefetch_rd_cnt_per_pch[addr_vec[0]*num_pseudo_ch+addr_vec[1]];
    };    
    int get_db_wr_fetch_per_pch(const AddrVec_t& addr_vec) override {
      return db_prefetch_wr_cnt_per_pch[addr_vec[0]*num_pseudo_ch+addr_vec[1]];
    };        

    void set_db_fetch_per_pch(const AddrVec_t& addr_vec, int value, int rd_value, int wr_value) override {
      int pch_idx = addr_vec[0]*num_pseudo_ch+addr_vec[1];
      db_prefetch_cnt_per_pch[pch_idx]    = value;
      db_prefetch_rd_cnt_per_pch[pch_idx] = rd_value;
      db_prefetch_wr_cnt_per_pch[pch_idx] = wr_value;
      // db_prefetch_change_mode(addr_vec[0],addr_vec[1]);
    };    

    void reset_need_be_open_per_bank(u_int32_t channel_idx) override {
      need_be_open_per_bank[channel_idx].assign(need_be_open_per_bank[channel_idx].size(), false);
    };

    void set_need_be_open_per_bank(const AddrVec_t& addr_vec) override {
      if(addr_vec[5] != -1 && addr_vec[6] != -1) {
        uint32_t idx = addr_vec[1] * m_num_ranks * m_num_bankgroups * m_num_banks + 
                       addr_vec[4] * m_num_bankgroups * m_num_banks + 
                       addr_vec[5] * m_num_banks +
                       addr_vec[6];
        need_be_open_per_bank[addr_vec[0]][idx] = true;
      }
    };

    bool get_need_be_open_per_bank(const AddrVec_t& addr_vec) override {
      if(addr_vec[5] != -1 && addr_vec[6] != -1) {
        uint32_t idx = addr_vec[1] * m_num_ranks * m_num_bankgroups * m_num_banks + 
                       addr_vec[4] * m_num_bankgroups * m_num_banks + 
                       addr_vec[5] * m_num_banks +
                       addr_vec[6];
        return need_be_open_per_bank[addr_vec[0]][idx];
      } 
      return false;
    };        

    bool get_use_pch() override {
      return m_use_pch;
    };

    bool get_use_prefetch() override {
      return m_use_prefetch;
    };        

    // Print Request 
    void print_req(Request& req) {      
      std::cout<<"Final ["<<m_commands(req.final_command)<<"] Current ["<<m_commands(req.command)
                         <<"] CH["<<req.addr_vec[m_levels["channel"]]
                         <<"]PC["<<req.addr_vec[m_levels["pseudochannel"]]
                         <<"]BG["<<req.addr_vec[m_levels["bankgroup"]]
                         <<"]BK["<<req.addr_vec[m_levels["bank"]]
                         <<"]RO["<<req.addr_vec[m_levels["row"]]
                         <<"]CO["<<req.addr_vec[m_levels["column"]]<<"]"
                         <<"PF["<<req.is_db_cmd<<"]"<<std::endl;
    };

    void set_high_pri_prefetch(u_int32_t channel_id, u_int32_t pseudo_channel_id) {
      m_high_pri_prefetch[channel_id*num_pseudo_ch+pseudo_channel_id] = true;
      // db_prefetch_change_mode(channel_id,pseudo_channel_id);
      // std::cout<<"["<<channel_id<<"] ["<<pseudo_channel_id<<"] Enable RD Prefetch from DRAM to DB"<<std::endl;
    };
    void reset_high_pri_prefetch(u_int32_t channel_id, u_int32_t pseudo_channel_id) {
      m_high_pri_prefetch[channel_id*num_pseudo_ch+pseudo_channel_id] = false;
      // db_prefetch_change_mode(channel_id,pseudo_channel_id);
      // std::cout<<"["<<channel_id<<"] ["<<pseudo_channel_id<<"] Disable RD Prefetch from DRAM to DB"<<std::endl;
    };        

    bool get_pri_prefetch(u_int32_t channel_id, u_int32_t pseudo_channel_id) {
      return m_high_pri_prefetch[channel_id*num_pseudo_ch+pseudo_channel_id];
    };    

    int get_db_fetch_mode(u_int32_t channel_id, u_int32_t pseudo_channel_id) {
      // int pch_idx = channel_id*num_pseudo_ch + pseudo_channel_id;
      return -1;
    }    

    bool is_ndp_access(const AddrVec_t& addr_vec) override {
      // is NDP Unit Access or NDP Execution
      if(addr_vec[m_levels["row"]] == ndp_access_row) return true;
      else                                            return false;
    }

    Inst_Slot decoding_inst(uint64_t inst) {
      uint64_t opcode = (inst >> 58) & 0x3f;
      uint64_t opsize = (inst >> 51) & 0x7f;
      uint64_t id     = (inst >> 48) & 0x7;
      uint64_t bg     = (inst >> 45) & 0x7;
      uint64_t bk     = (inst >> 43) & 0x3;
      std::cout<<"decoding opcode "<<opcode<<" opsize "<<opsize<<" id "<<id<<" bg "<<bg<<" bk "<<bk<<std::endl;
      return Inst_Slot(true,opcode,opsize,id,bg,bk);
    };

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

      /*
      for (int i = 0; i < m_levels.size(); i++){
        std::cout<<" Check level Name :"<<m_levels(i)<<" - "<<m_organization.count[i]<<std::endl;
      } 
        */     
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

      if(_density < 16384) {
        throw ConfigurationError("DDR5-PCH with NDP Not support the DRAM density under ({}) Mb.!", _density);
      }

      int num_channels = m_organization.count[m_levels["channel"]];
      int num_pseudochannel = m_organization.count[m_levels["pseudochannel"]];
      int num_ranks = m_organization.count[m_levels["rank"]];
      int num_bankgroups = m_organization.count[m_levels["bankgroup"]];
      int num_banks = m_organization.count[m_levels["bank"]];
      s_total_rfm_cycles.resize(num_channels * num_pseudochannel * num_ranks, 0);
      m_use_pch = true;
      m_num_channels      = num_channels;          
      m_num_pseudochannel = num_pseudochannel;     
      m_num_ranks         = num_ranks;             
      m_num_bankgroups    = num_bankgroups;        
      m_num_banks         = num_banks;       
 
      need_be_open_per_bank.resize(num_channels,std::vector<bool>(num_pseudochannel * num_bankgroups * num_banks, false));

      if(num_ranks != 1) {
        throw ConfigurationError("The number of pseudo-channel DRAM ranks ({}) cannot be greater than “1”.!", num_ranks);
      }
      
      m_use_prefetch = param<bool>("use_db_fetch").default_val(false);

      pre_wr_cnt_per_ch.resize(num_channels,0);
      post_wr_cnt_per_ch.resize(num_channels,0);
            
      num_pseudo_ch = num_pseudochannel;
      each_pch_refreshing.resize(num_channels * num_pseudochannel * num_ranks, false);
      db_prefetch_cnt_per_pch.resize(num_channels * num_pseudochannel * num_ranks, 0);
      m_db_prefetch_mode.resize(num_channels * num_pseudochannel * num_ranks,MODE_POST_WR);
      db_prefetch_rd_cnt_per_pch.resize(num_channels * num_pseudochannel * num_ranks,0);
      db_prefetch_wr_cnt_per_pch.resize(num_channels * num_pseudochannel * num_ranks,0);
      m_high_pri_prefetch.resize(num_channels * num_pseudochannel, false);
      for (int r = 0; r < num_channels * num_ranks; r++) {
        register_stat(s_total_rfm_cycles[r]).name("total_rfm_cycles_rank{}", r);
      }

      // Set NDP Unit
      // 8KB Instruction memory per pseudo channel 
      ins_mem_per_pch.resize(num_channels* num_pseudochannel,std::vector<u_int64_t>(128*8,0));
      // 32KB Data memory per pseudo channel 
      dat_mem_per_pch.resize(num_channels* num_pseudochannel,std::vector<u_int64_t>(128*4*8,0));

      ndp_status_per_pch.resize(num_channels* num_pseudochannel,m_ndp_status("idle"));      
      ndp_pc_per_pch.resize(num_channels* num_pseudochannel,0);      
      ndp_inst_slot_per_pch.resize(num_channels* num_pseudochannel,std::vector<Inst_Slot>(0,Inst_Slot()));      

      // Command from MC to DB (NDP Unit)
      pipe_ndp_latency_per_pch.resize(num_channels*num_pseudochannel,std::vector<int>(0,0));
      pipe_ndp_cmd_per_pch.resize(num_channels*num_pseudochannel,std::vector<int>(0,0));
      pipe_ndp_addr_per_pch.resize(num_channels*num_pseudochannel,std::vector<AddrVec_t>(0,AddrVec_t(0)));     
      pipe_ndp_id_per_pch.resize(num_channels*num_pseudochannel,std::vector<int>(0,0));     
      pipe_ndp_payload_valid_per_pch.resize(num_channels*num_pseudochannel,std::vector<bool>(0,false));     
      
      ndp_valid_per_pch.resize(num_channels* num_pseudochannel,false);
      ndp_cmd_per_pch.resize(num_channels* num_pseudochannel,0);
      ndp_id_per_pch.resize(num_channels* num_pseudochannel,0);
      ndp_addr_per_pch.resize(num_channels* num_pseudochannel,AddrVec_t(0));
      ndp_payload_valid_per_pch.resize(num_channels* num_pseudochannel,false);
      
      //payload
      pipe_ndp_payload_per_pch.resize(num_channels*num_pseudochannel,std::vector<std::vector<uint64_t>>(0,std::vector<uint64_t>(0,0)));      

      ndp_access_row          = (m_organization.count[m_levels["row"]] - 1);
      if(m_organization.dq == 16) {
        //x16 DRAM with 4BG and 4BK per BG
        ndp_ctrl_access_bk      = 3;
        ndp_ctrl_access_bg      = 1;
        ndp_ins_mem_access_bk   = 3;
        ndp_ins_mem_access_bg   = 0;
        ndp_dat_mem_access_bk   = 2;
        ndp_dat_mem_access_bg   = 3; // BG0-BG3
      } else {
        // x4/x8 DRAM with 8BG and 4 BK per BG
        ndp_ctrl_access_bk      = 3;
        ndp_ctrl_access_bg      = 5;
        ndp_ins_mem_access_bk   = 3;
        ndp_ins_mem_access_bg   = 4;
        ndp_dat_mem_access_bk   = 2;
        ndp_dat_mem_access_bg   = 3; // BG0-BG3        
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

      // tCCD_L_WR2 (with RMW) table
      constexpr int nCCD_L_WR2_TABLE[6] = {
      // 3200 / 4800 / 6400 / 7200 / 8000 / 8800
        32,49,65,73,80,89
      };
      if (dq_id == 0) {
        m_timing_vals("nCCDL_WR") = nCCD_L_WR2_TABLE[rate_id];
      }

      // Refresh timings
      // tRFC table (unit is nanosecond!)
      constexpr int tRFC_TABLE[2][3] = {
      //  8Gb   16Gb  32Gb  
        { 195,  295,  410 }, // Normal refresh (tRFC1)
        { 130,  160,  220 }, // FGR 2x (tRFC2)
      };

      // tRFCsb table (unit is nanosecond!)
      constexpr int tRFCsb_TABLE[1][3] = {
      //  8Gb   16Gb  32Gb  
        { 115,  130,  190 }, // Normal refresh (tRFC1)
      };

      // tREFI(base) table (unit is nanosecond!)
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
      // Rate and tCK should not be overwritten
      for (int i = 1; i < m_timings.size() - 1; i++) {
        auto timing_name = std::string(m_timings(i));

        if (auto provided_timing = param_group("timing").param<int>(timing_name).optional()) {
          // Check if the user specifies in the number of cycles (e.g., nRCD)
          m_timing_vals(i) = *provided_timing;
        } else if (auto provided_timing = param_group("timing").param<float>(timing_name.replace(0, 1, "t")).optional()) {
          // Check if the user specifies in nanoseconds (e.g., tRCD)
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
      // pCH DDR5 has 4 time long nBL (but..narrow DQ)
      m_read_latency = m_timing_vals("nCL") + 4*m_timing_vals("nBL");
      
      // Need Debug
      // Populate the timing constraints
      #define V(timing) (m_timing_vals(timing))
      auto all_commands = std::vector<std::string_view>(m_commands.begin(), m_commands.end());
      populate_timingcons(this, {
          /*** Channel ***/ 
          // Two-Cycle Commands
          {.level = "channel", .preceding = {"ACT", "RD", "RDA", "WR", "WRA", 
                                             "PRE_RD", "PRE_RDA", "POST_WR", "POST_WRA",
                                             "P_ACT",  "PRE_WR",  "POST_RD",
                                             "NDP_DRAM_RD", "NDP_DRAM_WR", "NDP_DRAM_RDA", "NDP_DRAM_WRA","NDP_DB_RD", "NDP_DB_WR"}, .following = all_commands, .latency = 2},
          
          // CAS <-> CAS
          /// Data bus occupancy //
          // {.level = "channel", .preceding = {"RD", "RDA"}, .following = {"RD", "RDA"}, .latency = V("nBL")},
          // {.level = "channel", .preceding = {"WR", "WRA"}, .following = {"WR", "WRA"}, .latency = V("nBL")},          
          // {.level = "pseudochannel", .preceding = {"RD", "RDA"}, .following = {"RD", "RDA"}, .latency = 4*V("nBL")},
          // {.level = "pseudochannel", .preceding = {"WR", "WRA"}, .following = {"WR", "WRA"}, .latency = 4*V("nBL")},
          {.level = "narrowio", .preceding = {"RD", "RDA", "POST_RD", "NDP_DB_RD"}, .following = {"RD", "RDA", "POST_RD", "NDP_DB_RD"},  .latency = 4*V("nBL")},
          {.level = "narrowio", .preceding = {"RD", "RDA", "POST_RD", "NDP_DB_RD"}, .following = {"WR", "WRA", "PRE_WR", "NDP_DB_WR"},   .latency = V("nCL") + 4*V("nBL") + 2 - V("nCWL") + 2},
          {.level = "narrowio", .preceding = {"WR", "WRA", "PRE_WR", "NDP_DB_WR"},  .following = {"WR", "WRA", "PRE_WR", "NDP_DB_WR"},   .latency = 4*V("nBL")},          
          {.level = "narrowio", .preceding = {"WR", "WRA", "PRE_WR", "NDP_DB_WR"},  .following = {"RD", "RDA", "POST_RD", "NDP_DB_RD"},  .latency = V("nCCDS_WTR")},          

          {.level = "wideio", .preceding = {"RD", "RDA", "PRE_RD",  "PRE_RDA", "NDP_DRAM_RD", "NDP_DRAM_RDA"},  .following = {"RD", "RDA", "PRE_RD",  "PRE_RDA", "NDP_DRAM_RD", "NDP_DRAM_RDA"},  .latency = V("nBL")},
          {.level = "wideio", .preceding = {"RD", "RDA", "PRE_RD",  "PRE_RDA", "NDP_DRAM_RD", "NDP_DRAM_RDA"},  .following = {"WR", "WRA", "POST_WR", "POST_WRA","NDP_DRAM_WR", "NDP_DRAM_WRA"}, .latency = V("nCL") + V("nBL") + 2 - V("nCWL") + 2},
          {.level = "wideio", .preceding = {"WR", "WRA"},                                                       .following = {"WR", "WRA", "POST_WR", "POST_WRA", "NDP_DRAM_WR", "NDP_DRAM_WRA"}, .latency = 4*V("nBL")},
          {.level = "wideio", .preceding = {"POST_WR", "POST_WRA", "NDP_DRAM_WR", "NDP_DRAM_WRA"},              .following = {"WR", "WRA", "POST_WR", "POST_WRA", "NDP_DRAM_WR", "NDP_DRAM_WRA"}, .latency = V("nBL")},
          {.level = "wideio", .preceding = {"WR", "WRA"},                                                       .following = {"RD", "RDA", "PRE_RD",  "PRE_RDA", "NDP_DRAM_RD", "NDP_DRAM_RDA"},  .latency = V("nCCDS_WTR")},
          {.level = "wideio", .preceding = {"POST_WR", "POST_WRA", "NDP_DRAM_WR", "NDP_DRAM_WRA"},              .following = {"RD", "RDA", "PRE_RD",  "PRE_RDA", "NDP_DRAM_RD", "NDP_DRAM_RDA"},  .latency = V("nCCDS_WTR_WI")},
          
          
          /*** Rank (or different BankGroup) ***/ 
          // CAS <-> CAS
          /// nCCDS is the minimal latency for column commands 
          {.level = "rank", .preceding = {"RD", "RDA", "PRE_RD", "PRE_RDA", "NDP_DRAM_RD", "NDP_DRAM_RDA"},   .following = {"RD", "RDA",  "PRE_RD", "PRE_RDA", "NDP_DRAM_RD", "NDP_DRAM_RDA"},   .latency = V("nCCDS")},
          {.level = "rank", .preceding = {"WR", "WRA", "POST_WR", "POST_WRA", "NDP_DRAM_WR", "NDP_DRAM_WRA"}, .following = {"WR", "WRA", "POST_WR", "POST_WRA", "NDP_DRAM_WR", "NDP_DRAM_WRA"},  .latency = V("nCCDS_WR")},
          /// RD <-> WR, Minimum Read to Write, Assuming Read DQS Offset = 0, tRPST = 0.5, tWPRE = 2 tCK                          
          {.level = "rank", .preceding = {"RD",     "RDA"},                                      .following = {"WR", "WRA", "POST_WR", "POST_WRA", "NDP_DRAM_WR", "NDP_DRAM_WRA"}, .latency = V("nCL") + 4*V("nBL") + 2 - V("nCWL") + 2},   // nCCDS_RTW
          {.level = "rank", .preceding = {"PRE_RD", "PRE_RDA", "NDP_DRAM_RD", "NDP_DRAM_RDA"},   .following = {"WR", "WRA", "POST_WR", "POST_WRA", "NDP_DRAM_WR", "NDP_DRAM_WRA"}, .latency = V("nCL") + V("nBL") + 2 - V("nCWL") + 2},   // nCCDS_RTW
          /// WR <-> RD, Minimum Read after Write
          {.level = "rank", .preceding = {"WR",      "WRA"},                                     .following = {"RD", "RDA", "PRE_RD",  "PRE_RDA", "NDP_DRAM_RD", "NDP_DRAM_RDA"}, .latency = V("nCCDS_WTR")},
          {.level = "rank", .preceding = {"POST_WR", "POST_WRA", "NDP_DRAM_WR", "NDP_DRAM_WRA"}, .following = {"RD", "RDA", "PRE_RD",  "PRE_RDA", "NDP_DRAM_RD", "NDP_DRAM_RDA"}, .latency = V("nCCDS_WTR_WI")},           

          /// CAS <-> CAS between sibling ranks, nCS (rank switching) is needed for new DQS
          {.level = "rank", .preceding = {"RD", "RDA"}, 
                            .following = {"RD", "RDA", "WR", "WRA", "PRE_RD", "PRE_RDA", "POST_WR", "POST_WRA", "NDP_DRAM_RD", "NDP_DRAM_WR", "NDP_DRAM_RDA", "NDP_DRAM_WRA"}, 
                            .latency = 4*V("nBL") + V("nCS"), .is_sibling = true},
          {.level = "rank", .preceding = {"PRE_RD", "PRE_RDA", "NDP_DRAM_RD", "NDP_DRAM_RDA"}, 
                            .following = {"RD", "RDA", "WR", "WRA", "PRE_RD", "PRE_RDA", "POST_WR", "POST_WRA","NDP_DRAM_RD", "NDP_DRAM_WR", "NDP_DRAM_RDA", "NDP_DRAM_WRA"}, 
                            .latency = V("nBL") + V("nCS"), .is_sibling = true},
          {.level = "rank", .preceding = {"WR", "WRA"}, 
                            .following = {"RD", "RDA","PRE_RD", "PRE_RDA", "NDP_DRAM_RD", "NDP_DRAM_RDA"}, 
                            .latency = V("nCL")  + 4*V("nBL") + V("nCS") - V("nCWL"), .is_sibling = true},
          {.level = "rank", .preceding = {"POST_WR", "POST_WRA", "NDP_DRAM_WR", "NDP_DRAM_WRA"}, 
                            .following = {"RD", "RDA","PRE_RD", "PRE_RDA", "NDP_DRAM_RD", "NDP_DRAM_RDA"}, 
                            .latency = V("nCL")  + V("nBL") + V("nCS") - V("nCWL"), .is_sibling = true},
          /// CAS <-> PREab
          {.level = "rank", .preceding = {"RD", "PRE_RD", "NDP_DRAM_RD"}, .following = {"PREA"}, .latency = V("nRTP")},
          {.level = "rank", .preceding = {"WR"},                          .following = {"PREA"}, .latency = V("nCWL") + 4*V("nBL") + V("nWR")},          
          {.level = "rank", .preceding = {"POST_WR", "NDP_DRAM_WR"},      .following = {"PREA"}, .latency = V("nCWL") + V("nBL") + V("nWR")},
          /// RAS <-> RAS
          {.level = "rank", .preceding = {"ACT"}, .following = {"ACT"}, .latency = V("nRRDS")},          
          {.level = "rank", .preceding = {"ACT"}, .following = {"ACT"}, .latency = V("nFAW"), .window = 4},          
          {.level = "rank", .preceding = {"ACT"}, .following = {"PREA"}, .latency = V("nRAS")},          
          {.level = "rank", .preceding = {"PREA"}, .following = {"ACT"}, .latency = V("nRP")},          
          /// RAS <-> REF
          // {.level = "rank", .preceding = {"ACT"}, .following = {"REFab", "RFMab", "DRFMab"}, .latency = V("nRC")},
          {.level = "rank", .preceding = {"ACT"}, .following = {"REFab"}, .latency = V("nRC")},          
          
          // {.level = "rank", .preceding = {"PRE", "PREsb"}, .following = {"REFab", "RFMab", "DRFMab"}, .latency = V("nRP")},        
          {.level = "rank", .preceding = {"PRE", "PREsb"}, .following = {"REFab"}, .latency = V("nRP")},  
          // {.level = "rank", .preceding = {"PREA"}, .following = {"REFab", "RFMab", "DRFMab", "REFsb", "RFMsb", "DRFMsb"}, .latency = V("nRP")},   
          {.level = "rank", .preceding = {"PREA"}, .following = {"REFab", "REFsb"}, .latency = V("nRP")},          
          // {.level = "rank", .preceding = {"RDA"}, .following = {"REFab", "RFMab", "DRFMab"}, .latency = V("nRP") + V("nRTP")},      
          {.level = "rank", .preceding = {"RDA", "PRE_RDA", "NDP_DRAM_RDA"}, .following = {"REFab"}, .latency = V("nRTP") + V("nRP")},     
          // {.level = "rank", .preceding = {"WRA"}, .following = {"REFab", "RFMab", "DRFMab"}, .latency = V("nCWL") + V("nBL") + V("nWR") + V("nRP")},        
          {.level = "rank", .preceding = {"WRA"}, .following = {"REFab"}, .latency = V("nCWL") + 4*V("nBL") + V("nWR") + V("nRP")},   
          {.level = "rank", .preceding = {"POST_WRA","NDP_DRAM_WRA"}, .following = {"REFab"}, .latency = V("nCWL") + V("nBL") + V("nWR") + V("nRP")},   
          // {.level = "rank", .preceding = {"REFab"}, .following = {"ACT", "PREA", "REFab", "RFMab", "DRFMab", "REFsb", "RFMsb", "DRFMsb"}, .latency = V("nRFC1")},       
          {.level = "rank", .preceding = {"REFab"}, .following = {"ACT", "PREA", "REFab", "REFsb"}, .latency = V("nRFC1")},     
          // {.level = "rank", .preceding = {"RFMab"}, .following = {"ACT", "PREA", "REFab", "RFMab", "DRFMab", "REFsb", "RFMsb", "DRFMsb"}, .latency = V("nRFM1")},          
          // {.level = "rank", .preceding = {"DRFMab"}, .following = {"ACT", "PREA", "REFab", "RFMab", "DRFMab", "REFsb", "RFMsb", "DRFMsb"}, .latency = V("nDRFMab")},          
          // {.level = "rank", .preceding = {"REFsb"},  .following = {"PREA", "REFab", "RFMab", "DRFMab"}, .latency = V("nRFCsb")},  
          {.level = "rank", .preceding = {"REFsb"},  .following = {"PREA", "REFab"}, .latency = V("nRFCsb")},  
          // {.level = "rank", .preceding = {"RFMsb"},  .following = {"PREA", "REFab", "RFMab", "DRFMab"}, .latency = V("nRFMsb")},  
          // {.level = "rank", .preceding = {"DRFMsb"}, .following = {"PREA", "REFab", "RFMab", "DRFMab"}, .latency = V("nDRFMsb")},  
          /*** Same Bank Group ***/ 
          /// CAS <-> CAS
          
          {.level = "bankgroup", .preceding = {"RD", "RDA", "PRE_RD",  "PRE_RDA", "NDP_DRAM_RD", "NDP_DRAM_RDA"},  .following = {"RD", "RDA", "PRE_RD",  "PRE_RDA", "NDP_DRAM_RD", "NDP_DRAM_RDA"}, .latency = V("nCCDL")},          
          {.level = "bankgroup", .preceding = {"WR", "WRA", "POST_WR", "POST_WRA", "NDP_DRAM_WR", "NDP_DRAM_WRA"}, .following = {"WR", "WRA", "POST_WR", "POST_WRA", "NDP_DRAM_WR", "NDP_DRAM_WRA"}, .latency = V("nCCDL_WR")},          
          {.level = "bankgroup", .preceding = {"WR",      "WRA"},                                     .following = {"RD", "RDA", "PRE_RD",  "PRE_RDA", "NDP_DRAM_RD", "NDP_DRAM_RDA"}, .latency = V("nCCDL_WTR")},
          {.level = "bankgroup", .preceding = {"POST_WR", "POST_WRA", "NDP_DRAM_WR", "NDP_DRAM_WRA"}, .following = {"RD", "RDA", "PRE_RD",  "PRE_RDA", "NDP_DRAM_RD", "NDP_DRAM_RDA"}, .latency = V("nCCDL_WTR_WI")},
          /// RAS <-> RAS
          {.level = "bankgroup", .preceding = {"ACT"}, .following = {"ACT"}, .latency = V("nRRDL")},  
          
          /*** Bank ***/ 
          // {.level = "bank", .preceding = {"ACT"}, .following = {"ACT", "REFsb", "RFMsb", "DRFMsb"}, .latency = V("nRC")},  
          {.level = "bank", .preceding = {"ACT"}, .following = {"ACT", "REFsb"}, .latency = V("nRC")},  
          {.level = "bank", .preceding = {"ACT"}, .following = {"RD", "RDA", "WR", "WRA", "PRE_RD", "PRE_RDA", "POST_WR", "POST_WRA", "NDP_DRAM_RD", "NDP_DRAM_RDA", "NDP_DRAM_WR", "NDP_DRAM_WRA"}, .latency = V("nRCD")},  
          {.level = "bank", .preceding = {"ACT"}, .following = {"PRE", "PREsb"}, .latency = V("nRAS")},  
          // {.level = "bank", .preceding = {"PRE", "PREsb"}, .following = {"ACT", "REFsb", "RFMsb", "DRFMsb"}, .latency = V("nRP")}, 
          {.level = "bank", .preceding = {"PRE", "PREsb"},                 .following = {"ACT", "REFsb"}, .latency = V("nRP")},   
          {.level = "bank", .preceding = {"RD", "PRE_RD", "NDP_DRAM_RD"},  .following = {"PRE", "PREsb"}, .latency = V("nRTP")},  
          {.level = "bank", .preceding = {"WR"},                           .following = {"PRE", "PREsb"}, .latency = V("nCWL") + 4*V("nBL") + V("nWR")},  
          {.level = "bank", .preceding = {"POST_WR", "NDP_DRAM_WR"},       .following = {"PRE", "PREsb"}, .latency = V("nCWL") + V("nBL") + V("nWR")},  
          // {.level = "bank", .preceding = {"RDA"}, .following = {"ACT", "REFsb", "RFMsb", "DRFMsb"}, .latency = V("nRTP") + V("nRP")},  
          {.level = "bank", .preceding = {"RDA", "PRE_RDA", "NDP_DRAM_RDA"}, .following = {"ACT", "REFsb"}, .latency = V("nRTP") + V("nRP")},  
          // {.level = "bank", .preceding = {"WRA"}, .following = {"ACT", "REFsb", "RFMsb", "DRFMsb"}, .latency = V("nCWL") + 4*V("nBL") + V("nWR") + V("nRP")},
          {.level = "bank", .preceding = {"WRA"},                       .following = {"ACT", "REFsb"}, .latency = V("nCWL") + 4*V("nBL") + V("nWR") + V("nRP")},  
          {.level = "bank", .preceding = {"POST_WRA", "NDP_DRAM_WRA"},  .following = {"ACT", "REFsb"}, .latency = V("nCWL") + V("nBL") + V("nWR") + V("nRP")},  
          {.level = "bank", .preceding = {"WR"},                        .following = {"RDA"}, .latency = V("nCWL") + 4*V("nBL") + V("nWR") - V("nRTP")},  
          {.level = "bank", .preceding = {"POST_WRA", "NDP_DRAM_WRA"},  .following = {"RDA"}, .latency = V("nCWL") + V("nBL") + V("nWR") - V("nRTP")},  

          /// Same-bank refresh/RFM timings. The timings of the bank in other BGs will be updated by action function
          // {.level = "bank", .preceding = {"REFsb"},  .following = {"ACT", "REFsb", "RFMsb", "DRFMsb"}, .latency = V("nRFCsb")},  
          {.level = "bank", .preceding = {"REFsb"},  .following = {"ACT", "REFsb"}, .latency = V("nRFCsb")},
          // {.level = "bank", .preceding = {"RFMsb"},  .following = {"ACT", "REFsb", "RFMsb", "DRFMsb"}, .latency = V("nRFMsb")},  
          // {.level = "bank", .preceding = {"DRFMsb"}, .following = {"ACT", "REFsb", "RFMsb", "DRFMsb"}, .latency = V("nDRFMsb")},  
        }
      );
      #undef V
      
    };

    void set_actions() {
      m_actions.resize(m_levels.size(), std::vector<ActionFunc_t<Node>>(m_commands.size()));

      // Rank Actions
      m_actions[m_levels["rank"]][m_commands["PREA"]] = Lambdas::Action::Rank::PREab<DDR5PCH>;
      m_actions[m_levels["rank"]][m_commands["REFab"]] = Lambdas::Action::Rank::REFab<DDR5PCH>;
      m_actions[m_levels["rank"]][m_commands["REFab_end"]] = Lambdas::Action::Rank::REFab_end<DDR5PCH>;
      // m_actions[m_levels["rank"]][m_commands["RFMab"]] = Lambdas::Action::Rank::REFab<DDR5PCH>;
      // m_actions[m_levels["rank"]][m_commands["RFMab_end"]] = Lambdas::Action::Rank::REFab_end<DDR5PCH>;
      // m_actions[m_levels["rank"]][m_commands["DRFMab"]] = Lambdas::Action::Rank::REFab<DDR5PCH>;
      // m_actions[m_levels["rank"]][m_commands["DRFMab_end"]] = Lambdas::Action::Rank::REFab_end<DDR5PCH>;
      
      // Same-Bank Actions.
      m_actions[m_levels["bankgroup"]][m_commands["PREsb"]] = Lambdas::Action::BankGroup::PREsb<DDR5PCH>;

      // We call update_timing for the banks in other BGs here
      m_actions[m_levels["bankgroup"]][m_commands["REFsb"]]  = Lambdas::Action::BankGroup::REFsb<DDR5PCH>;
      m_actions[m_levels["bankgroup"]][m_commands["REFsb_end"]]  = Lambdas::Action::BankGroup::REFsb_end<DDR5PCH>;
      // m_actions[m_levels["bankgroup"]][m_commands["RFMsb"]]  = Lambdas::Action::BankGroup::REFsb<DDR5PCH>;
      // m_actions[m_levels["bankgroup"]][m_commands["RFMsb_end"]]  = Lambdas::Action::BankGroup::REFsb_end<DDR5PCH>;
      // m_actions[m_levels["bankgroup"]][m_commands["DRFMsb"]] = Lambdas::Action::BankGroup::REFsb<DDR5PCH>;
      // m_actions[m_levels["bankgroup"]][m_commands["DRFMsb_end"]] = Lambdas::Action::BankGroup::REFsb_end<DDR5PCH>;

      // Bank actions
      m_actions[m_levels["bank"]][m_commands["ACT"]]          = Lambdas::Action::Bank::ACT<DDR5PCH>;
      m_actions[m_levels["bank"]][m_commands["PRE"]]          = Lambdas::Action::Bank::PRE<DDR5PCH>;
      m_actions[m_levels["bank"]][m_commands["RDA"]]          = Lambdas::Action::Bank::PRE<DDR5PCH>;
      m_actions[m_levels["bank"]][m_commands["PRE_RDA"]]      = Lambdas::Action::Bank::PRE<DDR5PCH>;
      m_actions[m_levels["bank"]][m_commands["WRA"]]          = Lambdas::Action::Bank::PRE<DDR5PCH>;
      m_actions[m_levels["bank"]][m_commands["POST_WRA"]]     = Lambdas::Action::Bank::PRE<DDR5PCH>;
      m_actions[m_levels["bank"]][m_commands["NDP_DRAM_RDA"]] = Lambdas::Action::Bank::PRE<DDR5PCH>;
      m_actions[m_levels["bank"]][m_commands["NDP_DRAM_WRA"]] = Lambdas::Action::Bank::PRE<DDR5PCH>;

      m_actions[m_levels["bank"]][m_commands["P_ACT"]]    = Lambdas::Action::Bank::P_ACT<DDR5PCH>;
      m_actions[m_levels["bank"]][m_commands["P_PRE"]]    = Lambdas::Action::Bank::P_PRE<DDR5PCH>;
    };

    void set_preqs() {
      m_preqs.resize(m_levels.size(), std::vector<PreqFunc_t<Node>>(m_commands.size()));

      // Rank Preqs
      m_preqs[m_levels["rank"]][m_commands["REFab"]]  = Lambdas::Preq::Rank::RequireAllBanksClosed<DDR5PCH>;
      // m_preqs[m_levels["rank"]][m_commands["RFMab"]]  = Lambdas::Preq::Rank::RequireAllBanksClosed<DDR5PCH>;
      // m_preqs[m_levels["rank"]][m_commands["DRFMab"]] = Lambdas::Preq::Rank::RequireAllBanksClosed<DDR5PCH>;

      // Same-Bank Preqs.
      m_preqs[m_levels["rank"]][m_commands["REFsb"]]  = Lambdas::Preq::Rank::RequireSameBanksClosed<DDR5PCH>;
      // m_preqs[m_levels["rank"]][m_commands["RFMsb"]]  = Lambdas::Preq::Rank::RequireSameBanksClosed<DDR5PCH>;
      // m_preqs[m_levels["rank"]][m_commands["DRFMsb"]] = Lambdas::Preq::Rank::RequireSameBanksClosed<DDR5PCH>;

      // Bank Preqs
      m_preqs[m_levels["bank"]][m_commands["RD"]]              = Lambdas::Preq::Bank::PCHRequireRowOpen<DDR5PCH>;
      m_preqs[m_levels["bank"]][m_commands["PRE_RD"]]          = Lambdas::Preq::Bank::PCHRequireRowOpen<DDR5PCH>;
      m_preqs[m_levels["bank"]][m_commands["WR"]]              = Lambdas::Preq::Bank::PCHRequireRowOpen<DDR5PCH>;
      m_preqs[m_levels["bank"]][m_commands["POST_WR"]]         = Lambdas::Preq::Bank::PCHRequireRowOpen<DDR5PCH>;
      m_preqs[m_levels["bank"]][m_commands["NDP_DRAM_RD"]]     = Lambdas::Preq::Bank::PCHRequireRowOpen<DDR5PCH>;
      m_preqs[m_levels["bank"]][m_commands["NDP_DRAM_WR"]]     = Lambdas::Preq::Bank::PCHRequireRowOpen<DDR5PCH>;
      m_preqs[m_levels["bank"]][m_commands["NDP_DRAM_RDA"]]    = Lambdas::Preq::Bank::PCHRequireRowOpen<DDR5PCH>;
      m_preqs[m_levels["bank"]][m_commands["NDP_DRAM_WRA"]]    = Lambdas::Preq::Bank::PCHRequireRowOpen<DDR5PCH>;
      m_preqs[m_levels["bank"]][m_commands["ACT"]]             = Lambdas::Preq::Bank::PCHRequireRowOpen<DDR5PCH>;
      m_preqs[m_levels["bank"]][m_commands["PRE"]]             = Lambdas::Preq::Bank::RequireBankClosed<DDR5PCH>;

      m_preqs[m_levels["bank"]][m_commands["PRE_WR"]]          = Lambdas::Preq::Bank::RequireFakeRowOpen<DDR5PCH>;
      m_preqs[m_levels["bank"]][m_commands["POST_RD"]]         = Lambdas::Preq::Bank::RequireFakeRowOpen<DDR5PCH>;
      m_preqs[m_levels["bank"]][m_commands["NDP_DB_RD"]]       = Lambdas::Preq::Bank::RequireFakeRowOpen<DDR5PCH>;
      m_preqs[m_levels["bank"]][m_commands["NDP_DB_WR"]]       = Lambdas::Preq::Bank::RequireFakeRowOpen<DDR5PCH>;
    };

    void set_rowhits() {
      m_rowhits.resize(m_levels.size(), std::vector<RowhitFunc_t<Node>>(m_commands.size()));

      m_rowhits[m_levels["bank"]][m_commands["RD"]]          = Lambdas::RowHit::Bank::RDWR<DDR5PCH>;
      m_rowhits[m_levels["bank"]][m_commands["PRE_RD"]]      = Lambdas::RowHit::Bank::RDWR<DDR5PCH>;
      m_rowhits[m_levels["bank"]][m_commands["WR"]]          = Lambdas::RowHit::Bank::RDWR<DDR5PCH>;
      m_rowhits[m_levels["bank"]][m_commands["POST_WR"]]     = Lambdas::RowHit::Bank::RDWR<DDR5PCH>;
      m_rowhits[m_levels["bank"]][m_commands["NDP_DRAM_RD"]] = Lambdas::RowHit::Bank::RDWR<DDR5PCH>;
      m_rowhits[m_levels["bank"]][m_commands["NDP_DRAM_WR"]] = Lambdas::RowHit::Bank::RDWR<DDR5PCH>;
    }

    void set_rowopens() {
      m_rowopens.resize(m_levels.size(), std::vector<RowhitFunc_t<Node>>(m_commands.size()));

      m_rowopens[m_levels["bank"]][m_commands["RD"]]          = Lambdas::RowOpen::Bank::RDWR<DDR5PCH>;
      m_rowopens[m_levels["bank"]][m_commands["PRE_RD"]]      = Lambdas::RowOpen::Bank::RDWR<DDR5PCH>;
      m_rowopens[m_levels["bank"]][m_commands["WR"]]          = Lambdas::RowOpen::Bank::RDWR<DDR5PCH>;
      m_rowopens[m_levels["bank"]][m_commands["POST_WR"]]     = Lambdas::RowOpen::Bank::RDWR<DDR5PCH>;
      m_rowopens[m_levels["bank"]][m_commands["NDP_DRAM_RD"]] = Lambdas::RowOpen::Bank::RDWR<DDR5PCH>;
      m_rowopens[m_levels["bank"]][m_commands["NDP_DRAM_WR"]] = Lambdas::RowOpen::Bank::RDWR<DDR5PCH>;
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

      // TODO: Check for multichannel configs.
      int num_channels = m_organization.count[m_levels["channel"]];
      int num_pseudochannels = m_organization.count[m_levels["pseudochannel"]];
      int num_ranks = m_organization.count[m_levels["rank"]];
      m_power_stats.resize(num_channels * num_pseudochannels * num_ranks);
      for (int i = 0; i < num_channels; i++) {
        for (int k = 0; k < num_pseudochannels; k++) {
          for (int j = 0; j < num_ranks; j++) {
            m_power_stats[i * num_ranks * num_pseudochannels + k * num_ranks + j].rank_id = i * num_ranks * num_pseudochannels + k * num_ranks + j;
            m_power_stats[i * num_ranks * num_pseudochannels + k * num_ranks + j].cmd_counters.resize(m_cmds_counted.size(), 0);
          }
        }
      }

      m_powers.resize(m_levels.size(), std::vector<PowerFunc_t<Node>>(m_commands.size()));

      m_powers[m_levels["bank"]][m_commands["ACT"]]          = Lambdas::Power::Bank::ACT<DDR5PCH>;
      m_powers[m_levels["bank"]][m_commands["PRE"]]          = Lambdas::Power::Bank::PRE<DDR5PCH>;
      m_powers[m_levels["bank"]][m_commands["RD"]]           = Lambdas::Power::Bank::RD<DDR5PCH>;
      m_powers[m_levels["bank"]][m_commands["WR"]]           = Lambdas::Power::Bank::WR<DDR5PCH>;
      m_powers[m_levels["bank"]][m_commands["WR"]]           = Lambdas::Power::Bank::WR<DDR5PCH>;
      m_powers[m_levels["bank"]][m_commands["PRE_RD"]]       = Lambdas::Power::Bank::DRAM2DB_RD<DDR5PCH>;
      m_powers[m_levels["bank"]][m_commands["PRE_RDA"]]      = Lambdas::Power::Bank::DRAM2DB_RD<DDR5PCH>;
      m_powers[m_levels["bank"]][m_commands["NDP_DRAM_RD"]]  = Lambdas::Power::Bank::DRAM2DB_RD<DDR5PCH>;
      m_powers[m_levels["bank"]][m_commands["NDP_DRAM_RDA"]] = Lambdas::Power::Bank::DRAM2DB_RD<DDR5PCH>;
      m_powers[m_levels["bank"]][m_commands["POST_WR"]]      = Lambdas::Power::Bank::DB2DRAM_WR<DDR5PCH>;
      m_powers[m_levels["bank"]][m_commands["POST_WRA"]]     = Lambdas::Power::Bank::DB2DRAM_WR<DDR5PCH>;
      m_powers[m_levels["bank"]][m_commands["NDP_DRAM_WR"]]  = Lambdas::Power::Bank::DB2DRAM_WR<DDR5PCH>;
      m_powers[m_levels["bank"]][m_commands["NDP_DRAM_WRA"]] = Lambdas::Power::Bank::DB2DRAM_WR<DDR5PCH>;

      m_powers[m_levels["bank"]][m_commands["POST_RD"]]      = Lambdas::Power::Bank::DB2MC_RD<DDR5PCH>;
      m_powers[m_levels["bank"]][m_commands["NDP_DB_RD"]]    = Lambdas::Power::Bank::DB2MC_RD<DDR5PCH>;
      m_powers[m_levels["bank"]][m_commands["PRE_WR"]]       = Lambdas::Power::Bank::MC2DB_WR<DDR5PCH>;
      m_powers[m_levels["bank"]][m_commands["NDP_DB_WR"]]    = Lambdas::Power::Bank::MC2DB_WR<DDR5PCH>;

      // m_powers[m_levels["rank"]][m_commands["REFsb"]] = Lambdas::Power::Rank::REFsb<DDR5>;
      // m_powers[m_levels["rank"]][m_commands["REFsb_end"]] = Lambdas::Power::Rank::REFsb_end<DDR5>;
      // m_powers[m_levels["rank"]][m_commands["RFMsb"]] = Lambdas::Power::Rank::RFMsb<DDR5PCH>;
      // m_powers[m_levels["rank"]][m_commands["RFMsb_end"]] = Lambdas::Power::Rank::RFMsb_end<DDR5PCH>;
      // m_powers[m_levels["rank"]][m_commands["DRFMsb"]] = Lambdas::Power::Rank::REFsb<DDR5>;
      // m_powers[m_levels["rank"]][m_commands["DRFMsb_end"]] = Lambdas::Power::Rank::REFsb_end<DDR5>;

      m_powers[m_levels["rank"]][m_commands["ACT"]] = Lambdas::Power::Rank::ACT<DDR5PCH>;
      m_powers[m_levels["rank"]][m_commands["PRE"]] = Lambdas::Power::Rank::PRE<DDR5PCH>;
      m_powers[m_levels["rank"]][m_commands["PREA"]] = Lambdas::Power::Rank::PREA<DDR5PCH>;
      m_powers[m_levels["rank"]][m_commands["REFab"]] = Lambdas::Power::Rank::REFab<DDR5PCH>;
      m_powers[m_levels["rank"]][m_commands["REFab_end"]] = Lambdas::Power::Rank::REFab_end<DDR5PCH>;
      // m_powers[m_levels["rank"]][m_commands["RFMab"]] = Lambdas::Power::Rank::REFab<DDR5>;
      // m_powers[m_levels["rank"]][m_commands["RFMab_end"]] = Lambdas::Power::Rank::REFab_end<DDR5>;
      // m_powers[m_levels["rank"]][m_commands["DRFMab"]] = Lambdas::Power::Rank::REFab<DDR5>;
      // m_powers[m_levels["rank"]][m_commands["DRFMab_end"]] = Lambdas::Power::Rank::REFab_end<DDR5>;

      m_powers[m_levels["rank"]][m_commands["PREsb"]] = Lambdas::Power::Rank::PREsb<DDR5PCH>;
      
      // "REF", "RFM",
      // "ACT"
      // "PRE"
      // RD: "RD", 
      // WR: "WR",
      // DRAM2DB_RD: "PRE_RD", "PRE_RDA", "NDP_DRAM_RD", "NDP_DRAM_RDA",
      // DB2DRAM_WR: "POST_WR", "POST_WRA", "NDP_DRAM_WR", , "NDP_DRAM_WRA"  
      // DB2MC_RD: "POST_RD", ,"NDP_DB_RD", 
      // MC2DB_WR: "PRE_WR", "NDP_DB_WR"

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

      // pJ
      double socket_dq_energy = 18.48;
      double on_board_dq_energy = 10.08;
      // pJ
      double on_buffer_energy = 9.5;
      int num_channels = m_organization.count[m_levels["channel"]];
      int num_pseudochannels = m_organization.count[m_levels["pseudochannel"]];
      int num_ranks = m_organization.count[m_levels["rank"]];
      for (int i = 0; i < num_channels; i++) {
        int num_socket_trans = 0;
        int num_on_board_trans = 0;
        int num_access_buf_trans = 0;
        for (int k = 0; k < num_pseudochannels; k++) {
            for (int j = 0; j < num_ranks; j++) {
            process_rank_energy(m_power_stats[i * num_ranks * num_pseudochannels + k * num_ranks + j], 
                                // Channel->pseudochannel->narrowIO->wdeIO-rank
                                m_channels[i]->m_child_nodes[k]->m_child_nodes[0]->m_child_nodes[0]->m_child_nodes[j]);
            num_socket_trans+=     m_power_stats[i * num_ranks * num_pseudochannels + k * num_ranks + j].cmd_counters[m_cmds_counted("RD")] + 
                                   m_power_stats[i * num_ranks * num_pseudochannels + k * num_ranks + j].cmd_counters[m_cmds_counted("WR")] +        
                                   m_power_stats[i * num_ranks * num_pseudochannels + k * num_ranks + j].cmd_counters[m_cmds_counted("DB2MC_RD")] +  
                                   m_power_stats[i * num_ranks * num_pseudochannels + k * num_ranks + j].cmd_counters[m_cmds_counted("MC2DB_WR")];     
            num_on_board_trans+=   m_power_stats[i * num_ranks * num_pseudochannels + k * num_ranks + j].cmd_counters[m_cmds_counted("RD")] + 
                                   m_power_stats[i * num_ranks * num_pseudochannels + k * num_ranks + j].cmd_counters[m_cmds_counted("WR")] +        
                                   m_power_stats[i * num_ranks * num_pseudochannels + k * num_ranks + j].cmd_counters[m_cmds_counted("DRAM2DB_RD")] +  
                                   m_power_stats[i * num_ranks * num_pseudochannels + k * num_ranks + j].cmd_counters[m_cmds_counted("DB2DRAM_WR")];
            num_access_buf_trans+= m_power_stats[i * num_ranks * num_pseudochannels + k * num_ranks + j].cmd_counters[m_cmds_counted("DB2MC_RD")] + 
                                   m_power_stats[i * num_ranks * num_pseudochannels + k * num_ranks + j].cmd_counters[m_cmds_counted("MC2DB_WR")] + 
                                   m_power_stats[i * num_ranks * num_pseudochannels + k * num_ranks + j].cmd_counters[m_cmds_counted("DRAM2DB_RD")] + 
                                   m_power_stats[i * num_ranks * num_pseudochannels + k * num_ranks + j].cmd_counters[m_cmds_counted("DB2DRAM_WR")];                                   
          }
        }
        // DQ Power (nJ) (Include ECC)
        double channel_socket_dq_energy = (double) num_socket_trans * (double) (16 * 4) * (double) ((m_channel_width+m_parity_width)/4) * socket_dq_energy/ 1E3;
        double channel_onboard_dq_energy = (double) num_on_board_trans * (double) (16) * (double) (m_channel_width+m_parity_width) * on_board_dq_energy / 1E3;
        // access 64 bit for on buffer
        double channel_access_buf_energy = (double) num_access_buf_trans * (double) ((m_channel_width+m_parity_width) * 16 / 64) * on_board_dq_energy / 1E3;
        double chanenl_socket_dq_power = channel_socket_dq_energy/((double)m_clk * (double)m_timing_vals("tCK_ps") / 1000.0);
        double chanenl_onboard_dq_power = channel_onboard_dq_energy/((double)m_clk * (double)m_timing_vals("tCK_ps") / 1000.0);
        double channel_access_buf_power = channel_access_buf_energy/((double)m_clk * (double)m_timing_vals("tCK_ps") / 1000.0);
        s_total_dq_energy += (channel_socket_dq_energy + channel_onboard_dq_energy + channel_access_buf_energy);
        s_total_energy    += (channel_socket_dq_energy + channel_onboard_dq_energy + channel_access_buf_energy);
        s_total_dq_power  += (chanenl_socket_dq_power + chanenl_onboard_dq_power + channel_access_buf_power);
        s_total_power     += (chanenl_socket_dq_power + chanenl_onboard_dq_power + channel_access_buf_power);
        std::cout<<"["<<i<<"] Channel DQ Power Report"<<std::endl;
        std::cout<<" - DQ (Socket) Energy (nJ)  : "<<channel_socket_dq_energy<<std::endl;
        std::cout<<" - DQ (OnBoard) Energy (nJ) : "<<channel_onboard_dq_energy<<std::endl;
        std::cout<<" - DQ (Buffer) Energy (nJ)  : "<<channel_access_buf_energy<<std::endl;
        std::cout<<" - DQ (Socket) Power (W)    : "<<chanenl_socket_dq_power<<std::endl;   
        std::cout<<" - DQ (OnBoard) Power (W)   : "<<chanenl_onboard_dq_power<<std::endl; 
        std::cout<<" - DQ (Buffer) Power (W)    : "<<channel_access_buf_power<<std::endl;   
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


    }
// DRAM2DB_RD", "DB2DRAM_WR", "DB2MC_RD", "MC2DB_WR"
    void process_rank_energy(PowerStats& rank_stats, Node* rank_node) {
      
      Lambdas::Power::Rank::finalize_rank<DDR5PCH>(rank_node, 0, AddrVec_t(), m_clk);
      size_t num_bankgroups = m_organization.count[m_levels["bankgroup"]];

      auto TS = [&](std::string_view timing) { return m_timing_vals(timing); };
      auto VE = [&](std::string_view voltage) { return m_voltage_vals(voltage); };
      auto CE = [&](std::string_view current) { return m_current_vals(current); };

      double tCK_ns = (double) TS("tCK_ps") / 1000.0;

      rank_stats.act_background_energy = (VE("VDD") * CE("IDD3N") + VE("VPP") * CE("IPP3N")) 
                                            * rank_stats.active_cycles * tCK_ns / 1E3;

      rank_stats.pre_background_energy = (VE("VDD") * CE("IDD2N") + VE("VPP") * CE("IPP2N")) 
                                            * rank_stats.idle_cycles * tCK_ns / 1E3;


      double act_cmd_energy  = (VE("VDD") * (CE("IDD0") - CE("IDD3N")) + VE("VPP") * (CE("IPP0") - CE("IPP3N"))) 
                                      * rank_stats.cmd_counters[m_cmds_counted("ACT")] * TS("nRAS") * tCK_ns / 1E3;

      double pre_cmd_energy  = (VE("VDD") * (CE("IDD0") - CE("IDD2N")) + VE("VPP") * (CE("IPP0") - CE("IPP2N"))) 
                                      * rank_stats.cmd_counters[m_cmds_counted("PRE")] * TS("nRP")  * tCK_ns / 1E3;

      double rd_cmd_energy   = (VE("VDD") * (CE("IDD4R") - CE("IDD3N")) + VE("VPP") * (CE("IPP4R") - CE("IPP3N"))) 
                                      * (rank_stats.cmd_counters[m_cmds_counted("RD")] + rank_stats.cmd_counters[m_cmds_counted("DRAM2DB_RD")]) * TS("nBL") * tCK_ns / 1E3;
                                      
      double wr_cmd_energy   = (VE("VDD") * (CE("IDD4W") - CE("IDD3N")) + VE("VPP") * (CE("IPP4W") - CE("IPP3N"))) 
                                      * (rank_stats.cmd_counters[m_cmds_counted("WR")] + rank_stats.cmd_counters[m_cmds_counted("DB2DRAM_WR")]) * TS("nBL") * tCK_ns / 1E3;
                                      
      double ref_cmd_energy  = (VE("VDD") * (CE("IDD5B")) + VE("VPP") * (CE("IPP5B"))) 
                                      * rank_stats.cmd_counters[m_cmds_counted("REF")] * TS("nRFC1") * tCK_ns / 1E3;

      double rfm_cmd_energy = (VE("VDD") * (CE("IDD0") - CE("IDD3N")) + VE("VPP") * (CE("IPP0") - CE("IPP3N"))) * num_bankgroups
                                      * rank_stats.cmd_counters[m_cmds_counted("RFM")] * TS("nRFMsb") * tCK_ns / 1E3;

      rank_stats.total_background_energy = rank_stats.act_background_energy + rank_stats.pre_background_energy;
      rank_stats.total_cmd_energy = act_cmd_energy 
                                    + pre_cmd_energy 
                                    + rd_cmd_energy
                                    + wr_cmd_energy 
                                    + ref_cmd_energy
                                    + rfm_cmd_energy;

      rank_stats.total_energy = rank_stats.total_background_energy + rank_stats.total_cmd_energy;

      s_total_background_energy += rank_stats.total_background_energy;
      s_total_cmd_energy += rank_stats.total_cmd_energy;
      s_total_energy += rank_stats.total_energy;
      s_total_rfm_energy += rfm_cmd_energy;

      s_total_rfm_cycles[rank_stats.rank_id] = rank_stats.cmd_counters[m_cmds_counted("RFM")] * TS("nRFMsb");

      // nJ / ns 
      int num_dev_per_rank = (m_channel_width+m_parity_width)/m_organization.dq;      
      double background_power = rank_stats.total_background_energy / ((double)m_clk * tCK_ns);
      double command_power = rank_stats.total_cmd_energy / ((double)m_clk * tCK_ns);
      double total_power = (background_power + command_power) * (double) num_dev_per_rank;

      s_total_background_power += (background_power * (double) num_dev_per_rank);
      s_total_cmd_power        += (command_power * (double) num_dev_per_rank);
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
