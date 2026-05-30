// -----------------------------------------------------------------------------
// Module      : lbus_srps
// Date        : 2026-05-28
// Version     : v1.1.0
// Author      : Jongchul Shin
// Function    : SRPS local bus interconnect top (9 fixed SRP instances).
//               msrp[0..8] -> 1:2 router -> merge/bypass -> sextmem (packed).
//               msrp[0..8] target1 -> io 9:1 merge -> sextio.
//               mext -> router/AHB -> ssrp[0..8] (packed).
// Assumptions : msrp target select (mem vs io) is decoded from msrp AW/AR address.
//               mext AHB target select is decoded from mext AW/AR address.
//               Per-msrp axi3_aw_w_order_gate enforces AW-before-W at input.
//               Register slices on sext* (128-bit wrap) and per-channel on mext.
// Caution     : msrp mem/io split is hard-coded to 0xE000_0000..0xFFFF_FFFF -> sextio;
//               all other msrp addresses -> sextmem. If the system map changes,
//               axi3_msrp_io_region_sel.v must be updated (no runtime parameter).
// Notes       : MSRP_MERGE_CFG selects SRP0~7 merge geometry; SRP8 always bypasses
//               to sextmem[SEXTMEM_PORT_NUM-1]. ADDR_WIDTH_M (msrp/sext*) and
//               ADDR_WIDTH_S (mext/ssrp) may differ. Unused mext address bits are
//               zeroed at input (addr_map) and at the router (addr_tgt).
//               ssrp AHB outputs SINGLE transfers only (hburst=000; htrans=IDLE/NONSEQ).
//               mext AW-before-W is enforced by axi3_router_1toN write ordering FIFO
//               (w_fifo_empty gates s_wready); no gate on mext.
// -----------------------------------------------------------------------------
module lbus_srps #(
    parameter integer MSRP_MERGE_CFG   = 2,              // 1=8:1, 2=4:1x2, 3=2:1x4
    parameter integer SEXTMEM_PORT_NUM   = (1 << (MSRP_MERGE_CFG - 1)) + 1,
    parameter integer MSRP_ID_WIDTH      = 5,              // per-SRP msrp slave AXI ID width
    parameter integer IO_MERGE_TAG_W     = 4,              // io 9:1: source index msrp[0..8]
    parameter integer SEXTMEM_ID_WIDTH   = MSRP_ID_WIDTH +
        ((MSRP_MERGE_CFG == 1) ? 3 : (MSRP_MERGE_CFG == 2) ? 2 : 1), // packed sextmem bus
    parameter integer SEXTIO_ID_WIDTH    = MSRP_ID_WIDTH + IO_MERGE_TAG_W, // sextio after io 9:1
    parameter integer MEXT_ID_WIDTH      = 4,              // mext AXI ID width
    parameter integer ADDR_WIDTH_M       = 35,             // msrp/sextmem/sextio address width
    parameter integer ADDR_WIDTH_S       = 32,             // mext/ssrp address width
    parameter integer ROUTER_OUTSTANDING = 32,           // mext router outstanding depth
    parameter integer WR_CMD_DEPTH       = 16,             // AHB bridge write command depth
    parameter integer RD_CMD_DEPTH       = 16,             // AHB bridge read command depth
    parameter integer RESP_DEPTH         = 8,              // AHB bridge response depth
    parameter integer AHB_REGION_SIZE_KB = 4,              // Equal ssrp AHB slot size (kilobytes)
    parameter integer MSRP_WR_PENDING_DEPTH = 16,          // msrp AW-before-W pending depth per port
    parameter         SEXT_AW_SLICE_EN   = 1'b1,           // sextmem/sextio AW register slice
    parameter         SEXT_W_SLICE_EN    = 1'b1,           // sextmem/sextio W register slice
    parameter         SEXT_B_SLICE_EN    = 1'b1,           // sextmem/sextio B register slice
    parameter         SEXT_AR_SLICE_EN   = 1'b1,           // sextmem/sextio AR register slice
    parameter         SEXT_R_SLICE_EN    = 1'b1,           // sextmem/sextio R register slice
    parameter         MEXT_AW_SLICE_EN   = 1'b1,           // mext AW register slice
    parameter         MEXT_W_SLICE_EN    = 1'b1,           // mext W register slice
    parameter         MEXT_B_SLICE_EN    = 1'b1,           // mext B register slice
    parameter         MEXT_AR_SLICE_EN   = 1'b1,           // mext AR register slice
    parameter         MEXT_R_SLICE_EN    = 1'b1            // mext R register slice
) (
    input  wire                         aclk,
    input  wire                         aresetn,

    // msrp[0..8]: AXI3 128-bit slave (one per SRP AXI master)
    input  wire [8:0]                   msrp_awvalid,
    output wire [8:0]                   msrp_awready,
    input  wire [9*MSRP_ID_WIDTH-1:0]   msrp_awid,
    input  wire [9*ADDR_WIDTH_M-1:0]     msrp_awaddr,
    input  wire [9*4-1:0]              msrp_awlen,
    input  wire [9*3-1:0]              msrp_awsize,
    input  wire [9*2-1:0]              msrp_awburst,
    input  wire [9*2-1:0]              msrp_awlock,
    input  wire [9*4-1:0]              msrp_awcache,
    input  wire [9*3-1:0]              msrp_awprot,

    input  wire [8:0]                   msrp_wvalid,
    output wire [8:0]                   msrp_wready,
    input  wire [9*MSRP_ID_WIDTH-1:0]   msrp_wid,
    input  wire [9*128-1:0]            msrp_wdata,
    input  wire [9*16-1:0]             msrp_wstrb,
    input  wire [8:0]                   msrp_wlast,

    output wire [8:0]                   msrp_bvalid,
    input  wire [8:0]                   msrp_bready,
    output wire [9*MSRP_ID_WIDTH-1:0]   msrp_bid,
    output wire [9*2-1:0]              msrp_bresp,

    input  wire [8:0]                   msrp_arvalid,
    output wire [8:0]                   msrp_arready,
    input  wire [9*MSRP_ID_WIDTH-1:0]   msrp_arid,
    input  wire [9*ADDR_WIDTH_M-1:0]     msrp_araddr,
    input  wire [9*4-1:0]              msrp_arlen,
    input  wire [9*3-1:0]              msrp_arsize,
    input  wire [9*2-1:0]              msrp_arburst,
    input  wire [9*2-1:0]              msrp_arlock,
    input  wire [9*4-1:0]              msrp_arcache,
    input  wire [9*3-1:0]              msrp_arprot,

    output wire [8:0]                   msrp_rvalid,
    input  wire [8:0]                   msrp_rready,
    output wire [9*MSRP_ID_WIDTH-1:0]   msrp_rid,
    output wire [9*128-1:0]            msrp_rdata,
    output wire [9*2-1:0]              msrp_rresp,
    output wire [8:0]                   msrp_rlast,

    // sextmem[0..SEXTMEM_PORT_NUM-1]: AXI3 128-bit master (packed)
    output wire [SEXTMEM_PORT_NUM-1:0]              sextmem_awvalid,
    input  wire [SEXTMEM_PORT_NUM-1:0]              sextmem_awready,
    output wire [SEXTMEM_PORT_NUM*SEXTMEM_ID_WIDTH-1:0] sextmem_awid,
    output wire [SEXTMEM_PORT_NUM*ADDR_WIDTH_M-1:0]   sextmem_awaddr,
    output wire [SEXTMEM_PORT_NUM*4-1:0]            sextmem_awlen,
    output wire [SEXTMEM_PORT_NUM*3-1:0]            sextmem_awsize,
    output wire [SEXTMEM_PORT_NUM*2-1:0]            sextmem_awburst,
    output wire [SEXTMEM_PORT_NUM*2-1:0]            sextmem_awlock,
    output wire [SEXTMEM_PORT_NUM*4-1:0]            sextmem_awcache,
    output wire [SEXTMEM_PORT_NUM*3-1:0]            sextmem_awprot,

    output wire [SEXTMEM_PORT_NUM-1:0]              sextmem_wvalid,
    input  wire [SEXTMEM_PORT_NUM-1:0]              sextmem_wready,
    output wire [SEXTMEM_PORT_NUM*SEXTMEM_ID_WIDTH-1:0] sextmem_wid,
    output wire [SEXTMEM_PORT_NUM*128-1:0]          sextmem_wdata,
    output wire [SEXTMEM_PORT_NUM*16-1:0]           sextmem_wstrb,
    output wire [SEXTMEM_PORT_NUM-1:0]              sextmem_wlast,

    input  wire [SEXTMEM_PORT_NUM-1:0]              sextmem_bvalid,
    output wire [SEXTMEM_PORT_NUM-1:0]              sextmem_bready,
    input  wire [SEXTMEM_PORT_NUM*SEXTMEM_ID_WIDTH-1:0] sextmem_bid,
    input  wire [SEXTMEM_PORT_NUM*2-1:0]            sextmem_bresp,

    output wire [SEXTMEM_PORT_NUM-1:0]              sextmem_arvalid,
    input  wire [SEXTMEM_PORT_NUM-1:0]              sextmem_arready,
    output wire [SEXTMEM_PORT_NUM*SEXTMEM_ID_WIDTH-1:0] sextmem_arid,
    output wire [SEXTMEM_PORT_NUM*ADDR_WIDTH_M-1:0]   sextmem_araddr,
    output wire [SEXTMEM_PORT_NUM*4-1:0]            sextmem_arlen,
    output wire [SEXTMEM_PORT_NUM*3-1:0]            sextmem_arsize,
    output wire [SEXTMEM_PORT_NUM*2-1:0]            sextmem_arburst,
    output wire [SEXTMEM_PORT_NUM*2-1:0]            sextmem_arlock,
    output wire [SEXTMEM_PORT_NUM*4-1:0]            sextmem_arcache,
    output wire [SEXTMEM_PORT_NUM*3-1:0]            sextmem_arprot,

    input  wire [SEXTMEM_PORT_NUM-1:0]              sextmem_rvalid,
    output wire [SEXTMEM_PORT_NUM-1:0]              sextmem_rready,
    input  wire [SEXTMEM_PORT_NUM*SEXTMEM_ID_WIDTH-1:0] sextmem_rid,
    input  wire [SEXTMEM_PORT_NUM*128-1:0]          sextmem_rdata,
    input  wire [SEXTMEM_PORT_NUM*2-1:0]            sextmem_rresp,
    input  wire [SEXTMEM_PORT_NUM-1:0]              sextmem_rlast,

    // sextio: AXI3 128-bit master (target1 io 9:1)
    output wire                         sextio_awvalid,
    input  wire                         sextio_awready,
    output wire [SEXTIO_ID_WIDTH-1:0]   sextio_awid,
    output wire [ADDR_WIDTH_M-1:0]        sextio_awaddr,
    output wire [3:0]                   sextio_awlen,
    output wire [2:0]                   sextio_awsize,
    output wire [1:0]                   sextio_awburst,
    output wire [1:0]                   sextio_awlock,
    output wire [3:0]                   sextio_awcache,
    output wire [2:0]                   sextio_awprot,

    output wire                         sextio_wvalid,
    input  wire                         sextio_wready,
    output wire [SEXTIO_ID_WIDTH-1:0]   sextio_wid,
    output wire [127:0]                 sextio_wdata,
    output wire [15:0]                  sextio_wstrb,
    output wire                         sextio_wlast,

    input  wire                         sextio_bvalid,
    output wire                         sextio_bready,
    input  wire [SEXTIO_ID_WIDTH-1:0]   sextio_bid,
    input  wire [1:0]                   sextio_bresp,

    output wire                         sextio_arvalid,
    input  wire                         sextio_arready,
    output wire [SEXTIO_ID_WIDTH-1:0]   sextio_arid,
    output wire [ADDR_WIDTH_M-1:0]        sextio_araddr,
    output wire [3:0]                   sextio_arlen,
    output wire [2:0]                   sextio_arsize,
    output wire [1:0]                   sextio_arburst,
    output wire [1:0]                   sextio_arlock,
    output wire [3:0]                   sextio_arcache,
    output wire [2:0]                   sextio_arprot,

    input  wire                         sextio_rvalid,
    output wire                         sextio_rready,
    input  wire [SEXTIO_ID_WIDTH-1:0]   sextio_rid,
    input  wire [127:0]                 sextio_rdata,
    input  wire [1:0]                   sextio_rresp,
    input  wire                         sextio_rlast,

    // mext: AXI3 32-bit slave
    input  wire                         mext_awvalid,
    output wire                         mext_awready,
    input  wire [MEXT_ID_WIDTH-1:0]     mext_awid,
    input  wire [ADDR_WIDTH_S-1:0]       mext_awaddr,
    input  wire [3:0]                   mext_awlen,
    input  wire [2:0]                   mext_awsize,
    input  wire [1:0]                   mext_awburst,
    input  wire [1:0]                   mext_awlock,
    input  wire [3:0]                   mext_awcache,
    input  wire [2:0]                   mext_awprot,
    input  wire                         mext_wvalid,
    output wire                         mext_wready,
    input  wire [MEXT_ID_WIDTH-1:0]     mext_wid,
    input  wire [31:0]                  mext_wdata,
    input  wire [3:0]                   mext_wstrb,
    input  wire                         mext_wlast,
    output wire                         mext_bvalid,
    input  wire                         mext_bready,
    output wire [MEXT_ID_WIDTH-1:0]     mext_bid,
    output wire [1:0]                   mext_bresp,
    input  wire                         mext_arvalid,
    output wire                         mext_arready,
    input  wire [MEXT_ID_WIDTH-1:0]     mext_arid,
    input  wire [ADDR_WIDTH_S-1:0]       mext_araddr,
    input  wire [3:0]                   mext_arlen,
    input  wire [2:0]                   mext_arsize,
    input  wire [1:0]                   mext_arburst,
    input  wire [1:0]                   mext_arlock,
    input  wire [3:0]                   mext_arcache,
    input  wire [2:0]                   mext_arprot,
    output wire                         mext_rvalid,
    input  wire                         mext_rready,
    output wire [MEXT_ID_WIDTH-1:0]     mext_rid,
    output wire [31:0]                  mext_rdata,
    output wire [1:0]                   mext_rresp,
    output wire                         mext_rlast,

    // ssrp[0..8]: AHB-Lite master (one per SRP AHB slave)
    output wire [9*ADDR_WIDTH_S-1:0]      ssrp_haddr,
    output wire [9*2-1:0]               ssrp_htrans,
    output wire [8:0]                   ssrp_hwrite,
    output wire [9*3-1:0]               ssrp_hsize,
    output wire [9*3-1:0]               ssrp_hburst,
    output wire [9*32-1:0]              ssrp_hwdata,
    input  wire [9*32-1:0]              ssrp_hrdata,
    input  wire [8:0]                   ssrp_hready,
    input  wire [8:0]                   ssrp_hresp
);

    localparam integer NUM_SRP            = 9;
    localparam integer SRP_LAST           = 8;
    localparam integer NUM_MEM_MERGE      = (1 << (MSRP_MERGE_CFG - 1)); // mem merge instance count
    localparam integer MEM_MERGE_N        = 8 / NUM_MEM_MERGE;           // inputs per mem merge
    localparam integer MEM_MERGE_TAG_W    =
        (MSRP_MERGE_CFG == 1) ? 3 : (MSRP_MERGE_CFG == 2) ? 2 : 1;
    localparam integer MEM_MERGE_ID_W     = MSRP_ID_WIDTH + MEM_MERGE_TAG_W;
    localparam integer BYPASS_IDX         = SEXTMEM_PORT_NUM - 1; // msrp[8] bypass -> sextmem[BYPASS_IDX]
    localparam integer SEXTMEM_ID_PAD_W   = SEXTMEM_ID_WIDTH - MEM_MERGE_ID_W; // zero-pad to SEXTMEM_ID_WIDTH
    localparam integer SEXTMEM_BYPASS_PAD_W = SEXTMEM_ID_WIDTH - MSRP_ID_WIDTH; // msrp[8] bypass -> sextmem[BYPASS_IDX]

    // Router master target0/target1 arrays
    wire [8:0]                   rt0_awvalid;
    wire [8:0]                   rt0_awready;
    wire [9*MSRP_ID_WIDTH-1:0]   rt0_awid;
    wire [9*ADDR_WIDTH_M-1:0]      rt0_awaddr;
    wire [9*4-1:0]               rt0_awlen;
    wire [9*3-1:0]               rt0_awsize;
    wire [9*2-1:0]               rt0_awburst;
    wire [9*2-1:0]               rt0_awlock;
    wire [9*4-1:0]               rt0_awcache;
    wire [9*3-1:0]               rt0_awprot;
    wire [8:0]                   rt0_wvalid;
    wire [8:0]                   rt0_wready;
    wire [9*MSRP_ID_WIDTH-1:0]   rt0_wid;
    wire [9*128-1:0]             rt0_wdata;
    wire [9*16-1:0]              rt0_wstrb;
    wire [8:0]                   rt0_wlast;
    wire [8:0]                   rt0_bvalid;
    wire [8:0]                   rt0_bready;
    wire [9*MSRP_ID_WIDTH-1:0]   rt0_bid;
    wire [9*2-1:0]               rt0_bresp;
    wire [8:0]                   rt0_arvalid;
    wire [8:0]                   rt0_arready;
    wire [9*MSRP_ID_WIDTH-1:0]   rt0_arid;
    wire [9*ADDR_WIDTH_M-1:0]      rt0_araddr;
    wire [9*4-1:0]               rt0_arlen;
    wire [9*3-1:0]               rt0_arsize;
    wire [9*2-1:0]               rt0_arburst;
    wire [9*2-1:0]               rt0_arlock;
    wire [9*4-1:0]               rt0_arcache;
    wire [9*3-1:0]               rt0_arprot;
    wire [8:0]                   rt0_rvalid;
    wire [8:0]                   rt0_rready;
    wire [9*MSRP_ID_WIDTH-1:0]   rt0_rid;
    wire [9*128-1:0]             rt0_rdata;
    wire [9*2-1:0]               rt0_rresp;
    wire [8:0]                   rt0_rlast;

    wire [8:0]                   rt1_awvalid;
    wire [8:0]                   rt1_awready;
    wire [9*MSRP_ID_WIDTH-1:0]   rt1_awid;
    wire [9*ADDR_WIDTH_M-1:0]      rt1_awaddr;
    wire [9*4-1:0]               rt1_awlen;
    wire [9*3-1:0]               rt1_awsize;
    wire [9*2-1:0]               rt1_awburst;
    wire [9*2-1:0]               rt1_awlock;
    wire [9*4-1:0]               rt1_awcache;
    wire [9*3-1:0]               rt1_awprot;
    wire [8:0]                   rt1_wvalid;
    wire [8:0]                   rt1_wready;
    wire [9*MSRP_ID_WIDTH-1:0]   rt1_wid;
    wire [9*128-1:0]             rt1_wdata;
    wire [9*16-1:0]              rt1_wstrb;
    wire [8:0]                   rt1_wlast;
    wire [8:0]                   rt1_bvalid;
    wire [8:0]                   rt1_bready;
    wire [9*MSRP_ID_WIDTH-1:0]   rt1_bid;
    wire [9*2-1:0]               rt1_bresp;
    wire [8:0]                   rt1_arvalid;
    wire [8:0]                   rt1_arready;
    wire [9*MSRP_ID_WIDTH-1:0]   rt1_arid;
    wire [9*ADDR_WIDTH_M-1:0]      rt1_araddr;
    wire [9*4-1:0]               rt1_arlen;
    wire [9*3-1:0]               rt1_arsize;
    wire [9*2-1:0]               rt1_arburst;
    wire [9*2-1:0]               rt1_arlock;
    wire [9*4-1:0]               rt1_arcache;
    wire [9*3-1:0]               rt1_arprot;
    wire [8:0]                   rt1_rvalid;
    wire [8:0]                   rt1_rready;
    wire [9*MSRP_ID_WIDTH-1:0]   rt1_rid;
    wire [9*128-1:0]             rt1_rdata;
    wire [9*2-1:0]               rt1_rresp;
    wire [8:0]                   rt1_rlast;

    wire [8:0]                   msrp_g_wvalid;
    wire [8:0]                   msrp_g_wready;
    wire [9*2-1:0]               msrp_aw_sel;
    wire [9*2-1:0]               msrp_ar_sel;
    wire [8:0]                   msrp_awready_rt;

    genvar gi;
    generate
        for (gi = 0; gi < NUM_SRP; gi = gi + 1) begin : g_msrp_target_sel
            axi3_msrp_io_region_sel #(
                .ADDR_WIDTH(ADDR_WIDTH_M)
            ) u_msrp_aw_target_sel (
                .addr(msrp_awaddr[(gi*ADDR_WIDTH_M) +: ADDR_WIDTH_M]),
                .sel(msrp_aw_sel[(gi*2) +: 2])
            );

            axi3_msrp_io_region_sel #(
                .ADDR_WIDTH(ADDR_WIDTH_M)
            ) u_msrp_ar_target_sel (
                .addr(msrp_araddr[(gi*ADDR_WIDTH_M) +: ADDR_WIDTH_M]),
                .sel(msrp_ar_sel[(gi*2) +: 2])
            );
        end
    endgenerate

    generate
        for (gi = 0; gi < NUM_SRP; gi = gi + 1) begin : g_msrp_aw_w_gate
            axi3_aw_w_order_gate #(
                .PENDING_DEPTH(MSRP_WR_PENDING_DEPTH)
            ) u_aw_w_gate (
                .aclk(aclk), .aresetn(aresetn),
                .awvalid(msrp_awvalid[gi]),
                .awready_i(msrp_awready_rt[gi]),
                .awready_o(msrp_awready[gi]),
                .wvalid(msrp_wvalid[gi]), .wready(msrp_wready[gi]),
                .wlast(msrp_wlast[gi]),
                .wvalid_o(msrp_g_wvalid[gi]), .wready_i(msrp_g_wready[gi])
            );
        end
    endgenerate

    generate
        for (gi = 0; gi < NUM_SRP; gi = gi + 1) begin : g_msrp_router
            wire [1:0]                   m_awvalid;
            wire [1:0]                   m_awready;
            wire [2*MSRP_ID_WIDTH-1:0]    m_awid;
            wire [2*ADDR_WIDTH_M-1:0]      m_awaddr;
            wire [2*4-1:0]               m_awlen;
            wire [2*3-1:0]               m_awsize;
            wire [2*2-1:0]               m_awburst;
            wire [2*2-1:0]               m_awlock;
            wire [2*4-1:0]               m_awcache;
            wire [2*3-1:0]               m_awprot;
            wire [1:0]                   m_wvalid;
            wire [1:0]                   m_wready;
            wire [2*MSRP_ID_WIDTH-1:0]    m_wid;
            wire [2*128-1:0]               m_wdata;
            wire [2*16-1:0]                m_wstrb;
            wire [1:0]                   m_wlast;
            wire [1:0]                   m_bvalid;
            wire [1:0]                   m_bready;
            wire [2*MSRP_ID_WIDTH-1:0]    m_bid;
            wire [2*2-1:0]               m_bresp;
            wire [1:0]                   m_arvalid;
            wire [1:0]                   m_arready;
            wire [2*MSRP_ID_WIDTH-1:0]    m_arid;
            wire [2*ADDR_WIDTH_M-1:0]      m_araddr;
            wire [2*4-1:0]               m_arlen;
            wire [2*3-1:0]               m_arsize;
            wire [2*2-1:0]               m_arburst;
            wire [2*2-1:0]               m_arlock;
            wire [2*4-1:0]               m_arcache;
            wire [2*3-1:0]               m_arprot;
            wire [1:0]                   m_rvalid;
            wire [1:0]                   m_rready;
            wire [2*MSRP_ID_WIDTH-1:0]    m_rid;
            wire [2*128-1:0]               m_rdata;
            wire [2*2-1:0]               m_rresp;
            wire [1:0]                   m_rlast;

            axi3_router_1to2_128 #(
                .ADDR_WIDTH(ADDR_WIDTH_M),
                .DATA_WIDTH(128),
                .STRB_WIDTH(16),
                .ID_WIDTH(MSRP_ID_WIDTH)
            ) u_router (
                .aclk(aclk), .aresetn(aresetn),
                .aw_sel(msrp_aw_sel[(gi*2) +: 2]),
                .ar_sel(msrp_ar_sel[(gi*2) +: 2]),
                .s_awvalid(msrp_awvalid[gi]),
                .s_awready(msrp_awready_rt[gi]),
                .s_awid(msrp_awid[(gi*MSRP_ID_WIDTH) +: MSRP_ID_WIDTH]),
                .s_awaddr(msrp_awaddr[(gi*ADDR_WIDTH_M) +: ADDR_WIDTH_M]),
                .s_awlen(msrp_awlen[(gi*4) +: 4]),
                .s_awsize(msrp_awsize[(gi*3) +: 3]),
                .s_awburst(msrp_awburst[(gi*2) +: 2]),
                .s_awlock(msrp_awlock[(gi*2) +: 2]),
                .s_awcache(msrp_awcache[(gi*4) +: 4]),
                .s_awprot(msrp_awprot[(gi*3) +: 3]),
                .s_wvalid(msrp_g_wvalid[gi]),
                .s_wready(msrp_g_wready[gi]),
                .s_wid(msrp_wid[(gi*MSRP_ID_WIDTH) +: MSRP_ID_WIDTH]),
                .s_wdata(msrp_wdata[(gi*128) +: 128]),
                .s_wstrb(msrp_wstrb[(gi*16) +: 16]),
                .s_wlast(msrp_wlast[gi]),
                .s_bvalid(msrp_bvalid[gi]),
                .s_bready(msrp_bready[gi]),
                .s_bid(msrp_bid[(gi*MSRP_ID_WIDTH) +: MSRP_ID_WIDTH]),
                .s_bresp(msrp_bresp[(gi*2) +: 2]),
                .s_arvalid(msrp_arvalid[gi]),
                .s_arready(msrp_arready[gi]),
                .s_arid(msrp_arid[(gi*MSRP_ID_WIDTH) +: MSRP_ID_WIDTH]),
                .s_araddr(msrp_araddr[(gi*ADDR_WIDTH_M) +: ADDR_WIDTH_M]),
                .s_arlen(msrp_arlen[(gi*4) +: 4]),
                .s_arsize(msrp_arsize[(gi*3) +: 3]),
                .s_arburst(msrp_arburst[(gi*2) +: 2]),
                .s_arlock(msrp_arlock[(gi*2) +: 2]),
                .s_arcache(msrp_arcache[(gi*4) +: 4]),
                .s_arprot(msrp_arprot[(gi*3) +: 3]),
                .s_rvalid(msrp_rvalid[gi]),
                .s_rready(msrp_rready[gi]),
                .s_rid(msrp_rid[(gi*MSRP_ID_WIDTH) +: MSRP_ID_WIDTH]),
                .s_rdata(msrp_rdata[(gi*128) +: 128]),
                .s_rresp(msrp_rresp[(gi*2) +: 2]),
                .s_rlast(msrp_rlast[gi]),
                .m_awvalid(m_awvalid), .m_awready(m_awready),
                .m_awid(m_awid), .m_awaddr(m_awaddr), .m_awlen(m_awlen),
                .m_awsize(m_awsize), .m_awburst(m_awburst), .m_awlock(m_awlock),
                .m_awcache(m_awcache), .m_awprot(m_awprot),
                .m_wvalid(m_wvalid), .m_wready(m_wready),
                .m_wid(m_wid), .m_wdata(m_wdata), .m_wstrb(m_wstrb), .m_wlast(m_wlast),
                .m_bvalid(m_bvalid), .m_bready(m_bready),
                .m_bid(m_bid), .m_bresp(m_bresp),
                .m_arvalid(m_arvalid), .m_arready(m_arready),
                .m_arid(m_arid), .m_araddr(m_araddr), .m_arlen(m_arlen),
                .m_arsize(m_arsize), .m_arburst(m_arburst), .m_arlock(m_arlock),
                .m_arcache(m_arcache), .m_arprot(m_arprot),
                .m_rvalid(m_rvalid), .m_rready(m_rready),
                .m_rid(m_rid), .m_rdata(m_rdata), .m_rresp(m_rresp), .m_rlast(m_rlast)
            );

            assign rt0_awvalid[gi]  = m_awvalid[0];
            assign m_awready[0]     = rt0_awready[gi];
            assign rt0_awid[(gi*MSRP_ID_WIDTH) +: MSRP_ID_WIDTH] =
                m_awid[(0*MSRP_ID_WIDTH) +: MSRP_ID_WIDTH];
            assign rt0_awaddr[(gi*ADDR_WIDTH_M) +: ADDR_WIDTH_M] =
                m_awaddr[(0*ADDR_WIDTH_M) +: ADDR_WIDTH_M];
            assign rt0_awlen[(gi*4) +: 4]   = m_awlen[(0*4) +: 4];
            assign rt0_awsize[(gi*3) +: 3]  = m_awsize[(0*3) +: 3];
            assign rt0_awburst[(gi*2) +: 2] = m_awburst[(0*2) +: 2];
            assign rt0_awlock[(gi*2) +: 2]  = m_awlock[(0*2) +: 2];
            assign rt0_awcache[(gi*4) +: 4] = m_awcache[(0*4) +: 4];
            assign rt0_awprot[(gi*3) +: 3]  = m_awprot[(0*3) +: 3];
            assign rt0_wvalid[gi]  = m_wvalid[0];
            assign m_wready[0]      = rt0_wready[gi];
            assign rt0_wid[(gi*MSRP_ID_WIDTH) +: MSRP_ID_WIDTH] =
                m_wid[(0*MSRP_ID_WIDTH) +: MSRP_ID_WIDTH];
            assign rt0_wdata[(gi*128) +: 128] = m_wdata[(0*128) +: 128];
            assign rt0_wstrb[(gi*16) +: 16] = m_wstrb[(0*16) +: 16];
            assign rt0_wlast[gi] = m_wlast[0];
            assign m_bvalid[0]    = rt0_bvalid[gi];
            assign rt0_bready[gi] = m_bready[0];
            assign m_bid[(0*MSRP_ID_WIDTH) +: MSRP_ID_WIDTH] =
                rt0_bid[(gi*MSRP_ID_WIDTH) +: MSRP_ID_WIDTH];
            assign m_bresp[(0*2) +: 2] = rt0_bresp[(gi*2) +: 2];
            assign rt0_arvalid[gi] = m_arvalid[0];
            assign m_arready[0]     = rt0_arready[gi];
            assign rt0_arid[(gi*MSRP_ID_WIDTH) +: MSRP_ID_WIDTH] =
                m_arid[(0*MSRP_ID_WIDTH) +: MSRP_ID_WIDTH];
            assign rt0_araddr[(gi*ADDR_WIDTH_M) +: ADDR_WIDTH_M] =
                m_araddr[(0*ADDR_WIDTH_M) +: ADDR_WIDTH_M];
            assign rt0_arlen[(gi*4) +: 4]   = m_arlen[(0*4) +: 4];
            assign rt0_arsize[(gi*3) +: 3]  = m_arsize[(0*3) +: 3];
            assign rt0_arburst[(gi*2) +: 2] = m_arburst[(0*2) +: 2];
            assign rt0_arlock[(gi*2) +: 2]  = m_arlock[(0*2) +: 2];
            assign rt0_arcache[(gi*4) +: 4] = m_arcache[(0*4) +: 4];
            assign rt0_arprot[(gi*3) +: 3]  = m_arprot[(0*3) +: 3];
            assign m_rvalid[0]     = rt0_rvalid[gi];
            assign rt0_rready[gi]  = m_rready[0];
            assign m_rid[(0*MSRP_ID_WIDTH) +: MSRP_ID_WIDTH] =
                rt0_rid[(gi*MSRP_ID_WIDTH) +: MSRP_ID_WIDTH];
            assign m_rdata[(0*128) +: 128] = rt0_rdata[(gi*128) +: 128];
            assign m_rresp[(0*2) +: 2] = rt0_rresp[(gi*2) +: 2];
            assign m_rlast[0] = rt0_rlast[gi];

            assign rt1_awvalid[gi]  = m_awvalid[1];
            assign m_awready[1]     = rt1_awready[gi];
            assign rt1_awid[(gi*MSRP_ID_WIDTH) +: MSRP_ID_WIDTH] =
                m_awid[(1*MSRP_ID_WIDTH) +: MSRP_ID_WIDTH];
            assign rt1_awaddr[(gi*ADDR_WIDTH_M) +: ADDR_WIDTH_M] =
                m_awaddr[(1*ADDR_WIDTH_M) +: ADDR_WIDTH_M];
            assign rt1_awlen[(gi*4) +: 4]   = m_awlen[(1*4) +: 4];
            assign rt1_awsize[(gi*3) +: 3]  = m_awsize[(1*3) +: 3];
            assign rt1_awburst[(gi*2) +: 2] = m_awburst[(1*2) +: 2];
            assign rt1_awlock[(gi*2) +: 2]  = m_awlock[(1*2) +: 2];
            assign rt1_awcache[(gi*4) +: 4] = m_awcache[(1*4) +: 4];
            assign rt1_awprot[(gi*3) +: 3]  = m_awprot[(1*3) +: 3];
            assign rt1_wvalid[gi]  = m_wvalid[1];
            assign m_wready[1]      = rt1_wready[gi];
            assign rt1_wid[(gi*MSRP_ID_WIDTH) +: MSRP_ID_WIDTH] =
                m_wid[(1*MSRP_ID_WIDTH) +: MSRP_ID_WIDTH];
            assign rt1_wdata[(gi*128) +: 128] = m_wdata[(1*128) +: 128];
            assign rt1_wstrb[(gi*16) +: 16] = m_wstrb[(1*16) +: 16];
            assign rt1_wlast[gi] = m_wlast[1];
            assign m_bvalid[1]    = rt1_bvalid[gi];
            assign rt1_bready[gi] = m_bready[1];
            assign m_bid[(1*MSRP_ID_WIDTH) +: MSRP_ID_WIDTH] =
                rt1_bid[(gi*MSRP_ID_WIDTH) +: MSRP_ID_WIDTH];
            assign m_bresp[(1*2) +: 2] = rt1_bresp[(gi*2) +: 2];
            assign rt1_arvalid[gi] = m_arvalid[1];
            assign m_arready[1]     = rt1_arready[gi];
            assign rt1_arid[(gi*MSRP_ID_WIDTH) +: MSRP_ID_WIDTH] =
                m_arid[(1*MSRP_ID_WIDTH) +: MSRP_ID_WIDTH];
            assign rt1_araddr[(gi*ADDR_WIDTH_M) +: ADDR_WIDTH_M] =
                m_araddr[(1*ADDR_WIDTH_M) +: ADDR_WIDTH_M];
            assign rt1_arlen[(gi*4) +: 4]   = m_arlen[(1*4) +: 4];
            assign rt1_arsize[(gi*3) +: 3]  = m_arsize[(1*3) +: 3];
            assign rt1_arburst[(gi*2) +: 2] = m_arburst[(1*2) +: 2];
            assign rt1_arlock[(gi*2) +: 2]  = m_arlock[(1*2) +: 2];
            assign rt1_arcache[(gi*4) +: 4] = m_arcache[(1*4) +: 4];
            assign rt1_arprot[(gi*3) +: 3]  = m_arprot[(1*3) +: 3];
            assign m_rvalid[1]     = rt1_rvalid[gi];
            assign rt1_rready[gi]  = m_rready[1];
            assign m_rid[(1*MSRP_ID_WIDTH) +: MSRP_ID_WIDTH] =
                rt1_rid[(gi*MSRP_ID_WIDTH) +: MSRP_ID_WIDTH];
            assign m_rdata[(1*128) +: 128] = rt1_rdata[(gi*128) +: 128];
            assign m_rresp[(1*2) +: 2] = rt1_rresp[(gi*2) +: 2];
            assign m_rlast[1] = rt1_rlast[gi];
        end
    endgenerate

    // Target0 (mem): msrp[0..7] -> mem N:1 -> sextmem[0..NUM_MEM_MERGE-1]; msrp[8] bypass
    genvar mi;
    generate
        for (mi = 0; mi < NUM_MEM_MERGE; mi = mi + 1) begin : g_mem_merge
            localparam integer RT_LO = mi * MEM_MERGE_N;

            wire [MEM_MERGE_N-1:0]                   mem_s_awvalid;
            wire [MEM_MERGE_N-1:0]                   mem_s_awready;
            wire [MEM_MERGE_N*MSRP_ID_WIDTH-1:0]     mem_s_awid;
            wire [MEM_MERGE_N*ADDR_WIDTH_M-1:0]        mem_s_awaddr;
            wire [MEM_MERGE_N*4-1:0]                 mem_s_awlen;
            wire [MEM_MERGE_N*3-1:0]                 mem_s_awsize;
            wire [MEM_MERGE_N*2-1:0]                 mem_s_awburst;
            wire [MEM_MERGE_N*2-1:0]                 mem_s_awlock;
            wire [MEM_MERGE_N*4-1:0]                 mem_s_awcache;
            wire [MEM_MERGE_N*3-1:0]                 mem_s_awprot;
            wire [MEM_MERGE_N-1:0]                   mem_s_wvalid;
            wire [MEM_MERGE_N-1:0]                   mem_s_wready;
            wire [MEM_MERGE_N*MSRP_ID_WIDTH-1:0]     mem_s_wid;
            wire [MEM_MERGE_N*128-1:0]               mem_s_wdata;
            wire [MEM_MERGE_N*16-1:0]                mem_s_wstrb;
            wire [MEM_MERGE_N-1:0]                   mem_s_wlast;
            wire [MEM_MERGE_N-1:0]                   mem_s_bvalid;
            wire [MEM_MERGE_N-1:0]                   mem_s_bready;
            wire [MEM_MERGE_N*MSRP_ID_WIDTH-1:0]     mem_s_bid;
            wire [MEM_MERGE_N*2-1:0]                 mem_s_bresp;
            wire [MEM_MERGE_N-1:0]                   mem_s_arvalid;
            wire [MEM_MERGE_N-1:0]                   mem_s_arready;
            wire [MEM_MERGE_N*MSRP_ID_WIDTH-1:0]     mem_s_arid;
            wire [MEM_MERGE_N*ADDR_WIDTH_M-1:0]        mem_s_araddr;
            wire [MEM_MERGE_N*4-1:0]                 mem_s_arlen;
            wire [MEM_MERGE_N*3-1:0]                 mem_s_arsize;
            wire [MEM_MERGE_N*2-1:0]                 mem_s_arburst;
            wire [MEM_MERGE_N*2-1:0]                 mem_s_arlock;
            wire [MEM_MERGE_N*4-1:0]                 mem_s_arcache;
            wire [MEM_MERGE_N*3-1:0]                 mem_s_arprot;
            wire [MEM_MERGE_N-1:0]                   mem_s_rvalid;
            wire [MEM_MERGE_N-1:0]                   mem_s_rready;
            wire [MEM_MERGE_N*MSRP_ID_WIDTH-1:0]     mem_s_rid;
            wire [MEM_MERGE_N*128-1:0]               mem_s_rdata;
            wire [MEM_MERGE_N*2-1:0]                 mem_s_rresp;
            wire [MEM_MERGE_N-1:0]                   mem_s_rlast;

            wire                                   mem_m_awvalid;
            wire                                   mem_m_wvalid;
            wire                                   mem_m_bvalid;
            wire                                   mem_m_arvalid;
            wire                                   mem_m_rvalid;
            wire                                   mem_m_awready;
            wire                                   mem_m_wready;
            wire                                   mem_m_bready;
            wire                                   mem_m_arready;
            wire                                   mem_m_rready;
            wire [MEM_MERGE_ID_W-1:0]              mem_m_awid;
            wire [MEM_MERGE_ID_W-1:0]              mem_m_wid;
            wire [MEM_MERGE_ID_W-1:0]              mem_m_bid;
            wire [MEM_MERGE_ID_W-1:0]              mem_m_arid;
            wire [MEM_MERGE_ID_W-1:0]              mem_m_rid;
            wire [ADDR_WIDTH_M-1:0]                  mem_m_awaddr;
            wire [ADDR_WIDTH_M-1:0]                  mem_m_araddr;
            wire [3:0]                             mem_m_awlen;
            wire [3:0]                             mem_m_arlen;
            wire [2:0]                             mem_m_awsize;
            wire [2:0]                             mem_m_arsize;
            wire [1:0]                             mem_m_awburst;
            wire [1:0]                             mem_m_arburst;
            wire [1:0]                             mem_m_awlock;
            wire [1:0]                             mem_m_arlock;
            wire [3:0]                             mem_m_awcache;
            wire [3:0]                             mem_m_arcache;
            wire [2:0]                             mem_m_awprot;
            wire [2:0]                             mem_m_arprot;
            wire [127:0]                           mem_m_wdata;
            wire [127:0]                           mem_m_rdata;
            wire [15:0]                            mem_m_wstrb;
            wire                                   mem_m_wlast;
            wire                                   mem_m_rlast;
            wire [1:0]                             mem_m_bresp;
            wire [1:0]                             mem_m_rresp;

            wire [SEXTMEM_ID_WIDTH-1:0]            mem_m_awid_z;
            wire [SEXTMEM_ID_WIDTH-1:0]            mem_m_wid_z;
            wire [SEXTMEM_ID_WIDTH-1:0]            mem_m_bid_z;
            wire [SEXTMEM_ID_WIDTH-1:0]            mem_m_arid_z;
            wire [SEXTMEM_ID_WIDTH-1:0]            mem_m_rid_z;

            assign mem_s_awvalid = rt0_awvalid[RT_LO +: MEM_MERGE_N];
            assign rt0_awready[RT_LO +: MEM_MERGE_N] = mem_s_awready;
            assign mem_s_awid    = rt0_awid[(RT_LO*MSRP_ID_WIDTH) +: MEM_MERGE_N*MSRP_ID_WIDTH];
            assign mem_s_awaddr  = rt0_awaddr[(RT_LO*ADDR_WIDTH_M) +: MEM_MERGE_N*ADDR_WIDTH_M];
            assign mem_s_awlen   = rt0_awlen[(RT_LO*4) +: MEM_MERGE_N*4];
            assign mem_s_awsize  = rt0_awsize[(RT_LO*3) +: MEM_MERGE_N*3];
            assign mem_s_awburst = rt0_awburst[(RT_LO*2) +: MEM_MERGE_N*2];
            assign mem_s_awlock  = rt0_awlock[(RT_LO*2) +: MEM_MERGE_N*2];
            assign mem_s_awcache = rt0_awcache[(RT_LO*4) +: MEM_MERGE_N*4];
            assign mem_s_awprot  = rt0_awprot[(RT_LO*3) +: MEM_MERGE_N*3];
            assign mem_s_wvalid  = rt0_wvalid[RT_LO +: MEM_MERGE_N];
            assign rt0_wready[RT_LO +: MEM_MERGE_N] = mem_s_wready;
            assign mem_s_wid     = rt0_wid[(RT_LO*MSRP_ID_WIDTH) +: MEM_MERGE_N*MSRP_ID_WIDTH];
            assign mem_s_wdata   = rt0_wdata[(RT_LO*128) +: MEM_MERGE_N*128];
            assign mem_s_wstrb   = rt0_wstrb[(RT_LO*16) +: MEM_MERGE_N*16];
            assign mem_s_wlast   = rt0_wlast[RT_LO +: MEM_MERGE_N];
            assign rt0_bvalid[RT_LO +: MEM_MERGE_N] = mem_s_bvalid;
            assign mem_s_bready  = rt0_bready[RT_LO +: MEM_MERGE_N];
            assign rt0_bid[(RT_LO*MSRP_ID_WIDTH) +: MEM_MERGE_N*MSRP_ID_WIDTH] = mem_s_bid;
            assign rt0_bresp[(RT_LO*2) +: MEM_MERGE_N*2] = mem_s_bresp;
            assign mem_s_arvalid = rt0_arvalid[RT_LO +: MEM_MERGE_N];
            assign rt0_arready[RT_LO +: MEM_MERGE_N] = mem_s_arready;
            assign mem_s_arid    = rt0_arid[(RT_LO*MSRP_ID_WIDTH) +: MEM_MERGE_N*MSRP_ID_WIDTH];
            assign mem_s_araddr  = rt0_araddr[(RT_LO*ADDR_WIDTH_M) +: MEM_MERGE_N*ADDR_WIDTH_M];
            assign mem_s_arlen   = rt0_arlen[(RT_LO*4) +: MEM_MERGE_N*4];
            assign mem_s_arsize  = rt0_arsize[(RT_LO*3) +: MEM_MERGE_N*3];
            assign mem_s_arburst = rt0_arburst[(RT_LO*2) +: MEM_MERGE_N*2];
            assign mem_s_arlock  = rt0_arlock[(RT_LO*2) +: MEM_MERGE_N*2];
            assign mem_s_arcache = rt0_arcache[(RT_LO*4) +: MEM_MERGE_N*4];
            assign mem_s_arprot  = rt0_arprot[(RT_LO*3) +: MEM_MERGE_N*3];
            assign rt0_rvalid[RT_LO +: MEM_MERGE_N] = mem_s_rvalid;
            assign mem_s_rready  = rt0_rready[RT_LO +: MEM_MERGE_N];
            assign rt0_rid[(RT_LO*MSRP_ID_WIDTH) +: MEM_MERGE_N*MSRP_ID_WIDTH] = mem_s_rid;
            assign rt0_rdata[(RT_LO*128) +: MEM_MERGE_N*128] = mem_s_rdata;
            assign rt0_rresp[(RT_LO*2) +: MEM_MERGE_N*2] = mem_s_rresp;
            assign rt0_rlast[RT_LO +: MEM_MERGE_N] = mem_s_rlast;

            axi3_merge_Nto1_128 #(
                .N(MEM_MERGE_N),
                .ADDR_WIDTH(ADDR_WIDTH_M),
                .IN_ID_WIDTH(MSRP_ID_WIDTH),
                .WR_OUTSTANDING_DEPTH(MSRP_WR_PENDING_DEPTH),
                .SRC_ID_WIDTH(MEM_MERGE_TAG_W)
            ) u_mem_merge (
                .aclk(aclk), .aresetn(aresetn),
                .s_awvalid(mem_s_awvalid), .s_awready(mem_s_awready), .s_awid(mem_s_awid),
                .s_awaddr(mem_s_awaddr), .s_awlen(mem_s_awlen), .s_awsize(mem_s_awsize),
                .s_awburst(mem_s_awburst), .s_awlock(mem_s_awlock), .s_awcache(mem_s_awcache),
                .s_awprot(mem_s_awprot),
                .s_wvalid(mem_s_wvalid), .s_wready(mem_s_wready), .s_wid(mem_s_wid),
                .s_wdata(mem_s_wdata), .s_wstrb(mem_s_wstrb), .s_wlast(mem_s_wlast),
                .s_bvalid(mem_s_bvalid), .s_bready(mem_s_bready), .s_bid(mem_s_bid),
                .s_bresp(mem_s_bresp),
                .s_arvalid(mem_s_arvalid), .s_arready(mem_s_arready), .s_arid(mem_s_arid),
                .s_araddr(mem_s_araddr), .s_arlen(mem_s_arlen), .s_arsize(mem_s_arsize),
                .s_arburst(mem_s_arburst), .s_arlock(mem_s_arlock), .s_arcache(mem_s_arcache),
                .s_arprot(mem_s_arprot),
                .s_rvalid(mem_s_rvalid), .s_rready(mem_s_rready), .s_rid(mem_s_rid),
                .s_rdata(mem_s_rdata), .s_rresp(mem_s_rresp), .s_rlast(mem_s_rlast),
                .m_awvalid(mem_m_awvalid), .m_awready(mem_m_awready), .m_awid(mem_m_awid),
                .m_awaddr(mem_m_awaddr), .m_awlen(mem_m_awlen), .m_awsize(mem_m_awsize),
                .m_awburst(mem_m_awburst), .m_awlock(mem_m_awlock), .m_awcache(mem_m_awcache),
                .m_awprot(mem_m_awprot),
                .m_wvalid(mem_m_wvalid), .m_wready(mem_m_wready), .m_wid(mem_m_wid),
                .m_wdata(mem_m_wdata), .m_wstrb(mem_m_wstrb), .m_wlast(mem_m_wlast),
                .m_bvalid(mem_m_bvalid), .m_bready(mem_m_bready), .m_bid(mem_m_bid),
                .m_bresp(mem_m_bresp),
                .m_arvalid(mem_m_arvalid), .m_arready(mem_m_arready), .m_arid(mem_m_arid),
                .m_araddr(mem_m_araddr), .m_arlen(mem_m_arlen), .m_arsize(mem_m_arsize),
                .m_arburst(mem_m_arburst), .m_arlock(mem_m_arlock), .m_arcache(mem_m_arcache),
                .m_arprot(mem_m_arprot),
                .m_rvalid(mem_m_rvalid), .m_rready(mem_m_rready), .m_rid(mem_m_rid),
                .m_rdata(mem_m_rdata), .m_rresp(mem_m_rresp), .m_rlast(mem_m_rlast)
            );

            generate
                if (SEXTMEM_ID_PAD_W > 0) begin : gen_id_pad
                    assign mem_m_awid_z = {{SEXTMEM_ID_PAD_W{1'b0}}, mem_m_awid};
                    assign mem_m_wid_z  = {{SEXTMEM_ID_PAD_W{1'b0}}, mem_m_wid};
                    assign mem_m_arid_z = {{SEXTMEM_ID_PAD_W{1'b0}}, mem_m_arid};
                end else begin : gen_id_nopad
                    assign mem_m_awid_z = mem_m_awid;
                    assign mem_m_wid_z  = mem_m_wid;
                    assign mem_m_arid_z = mem_m_arid;
                end
            endgenerate
            assign mem_m_bid    = mem_m_bid_z[MEM_MERGE_ID_W-1:0];
            assign mem_m_rid    = mem_m_rid_z[MEM_MERGE_ID_W-1:0];

            lbus_axi128_reg_slice_wrap #(
                .ID_WIDTH(SEXTMEM_ID_WIDTH), .ADDR_WIDTH(ADDR_WIDTH_M), .DATA_WIDTH(128),
                .AW_SLICE_EN(SEXT_AW_SLICE_EN), .W_SLICE_EN(SEXT_W_SLICE_EN),
                .B_SLICE_EN(SEXT_B_SLICE_EN), .AR_SLICE_EN(SEXT_AR_SLICE_EN),
                .R_SLICE_EN(SEXT_R_SLICE_EN)
            ) u_sextmem_rs (
                .aclk(aclk), .aresetn(aresetn),
                .awvalid_s(mem_m_awvalid), .awready_s(mem_m_awready),
                .awid_s(mem_m_awid_z), .awaddr_s(mem_m_awaddr), .awlen_s(mem_m_awlen),
                .awsize_s(mem_m_awsize), .awburst_s(mem_m_awburst), .awlock_s(mem_m_awlock),
                .awcache_s(mem_m_awcache), .awprot_s(mem_m_awprot),
                .awvalid_m(sextmem_awvalid[mi]), .awready_m(sextmem_awready[mi]),
                .awid_m(sextmem_awid[(mi*SEXTMEM_ID_WIDTH) +: SEXTMEM_ID_WIDTH]),
                .awaddr_m(sextmem_awaddr[(mi*ADDR_WIDTH_M) +: ADDR_WIDTH_M]),
                .awlen_m(sextmem_awlen[(mi*4) +: 4]),
                .awsize_m(sextmem_awsize[(mi*3) +: 3]),
                .awburst_m(sextmem_awburst[(mi*2) +: 2]),
                .awlock_m(sextmem_awlock[(mi*2) +: 2]),
                .awcache_m(sextmem_awcache[(mi*4) +: 4]),
                .awprot_m(sextmem_awprot[(mi*3) +: 3]),
                .wvalid_s(mem_m_wvalid), .wready_s(mem_m_wready),
                .wid_s(mem_m_wid_z), .wdata_s(mem_m_wdata), .wstrb_s(mem_m_wstrb), .wlast_s(mem_m_wlast),
                .wvalid_m(sextmem_wvalid[mi]), .wready_m(sextmem_wready[mi]),
                .wid_m(sextmem_wid[(mi*SEXTMEM_ID_WIDTH) +: SEXTMEM_ID_WIDTH]),
                .wdata_m(sextmem_wdata[(mi*128) +: 128]),
                .wstrb_m(sextmem_wstrb[(mi*16) +: 16]),
                .wlast_m(sextmem_wlast[mi]),
                .bvalid_s(sextmem_bvalid[mi]), .bready_s(sextmem_bready[mi]),
                .bid_s(sextmem_bid[(mi*SEXTMEM_ID_WIDTH) +: SEXTMEM_ID_WIDTH]),
                .bresp_s(sextmem_bresp[(mi*2) +: 2]),
                .bvalid_m(mem_m_bvalid), .bready_m(mem_m_bready),
                .bid_m(mem_m_bid_z), .bresp_m(mem_m_bresp),
                .arvalid_s(mem_m_arvalid), .arready_s(mem_m_arready),
                .arid_s(mem_m_arid_z), .araddr_s(mem_m_araddr), .arlen_s(mem_m_arlen),
                .arsize_s(mem_m_arsize), .arburst_s(mem_m_arburst), .arlock_s(mem_m_arlock),
                .arcache_s(mem_m_arcache), .arprot_s(mem_m_arprot),
                .arvalid_m(sextmem_arvalid[mi]), .arready_m(sextmem_arready[mi]),
                .arid_m(sextmem_arid[(mi*SEXTMEM_ID_WIDTH) +: SEXTMEM_ID_WIDTH]),
                .araddr_m(sextmem_araddr[(mi*ADDR_WIDTH_M) +: ADDR_WIDTH_M]),
                .arlen_m(sextmem_arlen[(mi*4) +: 4]),
                .arsize_m(sextmem_arsize[(mi*3) +: 3]),
                .arburst_m(sextmem_arburst[(mi*2) +: 2]),
                .arlock_m(sextmem_arlock[(mi*2) +: 2]),
                .arcache_m(sextmem_arcache[(mi*4) +: 4]),
                .arprot_m(sextmem_arprot[(mi*3) +: 3]),
                .rvalid_s(sextmem_rvalid[mi]), .rready_s(sextmem_rready[mi]),
                .rid_s(sextmem_rid[(mi*SEXTMEM_ID_WIDTH) +: SEXTMEM_ID_WIDTH]),
                .rdata_s(sextmem_rdata[(mi*128) +: 128]),
                .rresp_s(sextmem_rresp[(mi*2) +: 2]),
                .rlast_s(sextmem_rlast[mi]),
                .rvalid_m(mem_m_rvalid), .rready_m(mem_m_rready),
                .rid_m(mem_m_rid_z), .rdata_m(mem_m_rdata), .rresp_m(mem_m_rresp),
                .rlast_m(mem_m_rlast)
            );
        end
    endgenerate

    // SRP8 (msrp[8]) bypass -> sextmem[BYPASS_IDX]
    wire [SEXTMEM_ID_WIDTH-1:0] bp_awid_z;
    wire [SEXTMEM_ID_WIDTH-1:0] bp_wid_z;
    wire [SEXTMEM_ID_WIDTH-1:0] bp_bid_z;
    wire [SEXTMEM_ID_WIDTH-1:0] bp_arid_z;
    wire [SEXTMEM_ID_WIDTH-1:0] bp_rid_z;

    assign bp_awid_z = {{SEXTMEM_BYPASS_PAD_W{1'b0}}, rt0_awid[(SRP_LAST*MSRP_ID_WIDTH) +: MSRP_ID_WIDTH]};
    assign bp_wid_z  = {{SEXTMEM_BYPASS_PAD_W{1'b0}}, rt0_wid[(SRP_LAST*MSRP_ID_WIDTH) +: MSRP_ID_WIDTH]};
    assign bp_arid_z = {{SEXTMEM_BYPASS_PAD_W{1'b0}}, rt0_arid[(SRP_LAST*MSRP_ID_WIDTH) +: MSRP_ID_WIDTH]};
    assign rt0_bid[(SRP_LAST*MSRP_ID_WIDTH) +: MSRP_ID_WIDTH] =
        bp_bid_z[MSRP_ID_WIDTH-1:0];
    assign rt0_rid[(SRP_LAST*MSRP_ID_WIDTH) +: MSRP_ID_WIDTH] =
        bp_rid_z[MSRP_ID_WIDTH-1:0];

    lbus_axi128_reg_slice_wrap #(
        .ID_WIDTH(SEXTMEM_ID_WIDTH), .ADDR_WIDTH(ADDR_WIDTH_M), .DATA_WIDTH(128),
        .AW_SLICE_EN(SEXT_AW_SLICE_EN), .W_SLICE_EN(SEXT_W_SLICE_EN),
        .B_SLICE_EN(SEXT_B_SLICE_EN), .AR_SLICE_EN(SEXT_AR_SLICE_EN),
        .R_SLICE_EN(SEXT_R_SLICE_EN)
    ) u_sextmem_bypass_rs (
        .aclk(aclk), .aresetn(aresetn),
        .awvalid_s(rt0_awvalid[SRP_LAST]), .awready_s(rt0_awready[SRP_LAST]),
        .awid_s(bp_awid_z),
        .awaddr_s(rt0_awaddr[(SRP_LAST*ADDR_WIDTH_M) +: ADDR_WIDTH_M]),
        .awlen_s(rt0_awlen[(SRP_LAST*4) +: 4]),
        .awsize_s(rt0_awsize[(SRP_LAST*3) +: 3]),
        .awburst_s(rt0_awburst[(SRP_LAST*2) +: 2]),
        .awlock_s(rt0_awlock[(SRP_LAST*2) +: 2]),
        .awcache_s(rt0_awcache[(SRP_LAST*4) +: 4]),
        .awprot_s(rt0_awprot[(SRP_LAST*3) +: 3]),
        .awvalid_m(sextmem_awvalid[BYPASS_IDX]), .awready_m(sextmem_awready[BYPASS_IDX]),
        .awid_m(sextmem_awid[(BYPASS_IDX*SEXTMEM_ID_WIDTH) +: SEXTMEM_ID_WIDTH]),
        .awaddr_m(sextmem_awaddr[(BYPASS_IDX*ADDR_WIDTH_M) +: ADDR_WIDTH_M]),
        .awlen_m(sextmem_awlen[(BYPASS_IDX*4) +: 4]),
        .awsize_m(sextmem_awsize[(BYPASS_IDX*3) +: 3]),
        .awburst_m(sextmem_awburst[(BYPASS_IDX*2) +: 2]),
        .awlock_m(sextmem_awlock[(BYPASS_IDX*2) +: 2]),
        .awcache_m(sextmem_awcache[(BYPASS_IDX*4) +: 4]),
        .awprot_m(sextmem_awprot[(BYPASS_IDX*3) +: 3]),
        .wvalid_s(rt0_wvalid[SRP_LAST]), .wready_s(rt0_wready[SRP_LAST]),
        .wid_s(bp_wid_z),
        .wdata_s(rt0_wdata[(SRP_LAST*128) +: 128]),
        .wstrb_s(rt0_wstrb[(SRP_LAST*16) +: 16]),
        .wlast_s(rt0_wlast[SRP_LAST]),
        .wvalid_m(sextmem_wvalid[BYPASS_IDX]), .wready_m(sextmem_wready[BYPASS_IDX]),
        .wid_m(sextmem_wid[(BYPASS_IDX*SEXTMEM_ID_WIDTH) +: SEXTMEM_ID_WIDTH]),
        .wdata_m(sextmem_wdata[(BYPASS_IDX*128) +: 128]),
        .wstrb_m(sextmem_wstrb[(BYPASS_IDX*16) +: 16]),
        .wlast_m(sextmem_wlast[BYPASS_IDX]),
        .bvalid_s(sextmem_bvalid[BYPASS_IDX]), .bready_s(sextmem_bready[BYPASS_IDX]),
        .bid_s(sextmem_bid[(BYPASS_IDX*SEXTMEM_ID_WIDTH) +: SEXTMEM_ID_WIDTH]),
        .bresp_s(sextmem_bresp[(BYPASS_IDX*2) +: 2]),
        .bvalid_m(rt0_bvalid[SRP_LAST]), .bready_m(rt0_bready[SRP_LAST]),
        .bid_m(bp_bid_z), .bresp_m(rt0_bresp[(SRP_LAST*2) +: 2]),
        .arvalid_s(rt0_arvalid[SRP_LAST]), .arready_s(rt0_arready[SRP_LAST]),
        .arid_s(bp_arid_z),
        .araddr_s(rt0_araddr[(SRP_LAST*ADDR_WIDTH_M) +: ADDR_WIDTH_M]),
        .arlen_s(rt0_arlen[(SRP_LAST*4) +: 4]),
        .arsize_s(rt0_arsize[(SRP_LAST*3) +: 3]),
        .arburst_s(rt0_arburst[(SRP_LAST*2) +: 2]),
        .arlock_s(rt0_arlock[(SRP_LAST*2) +: 2]),
        .arcache_s(rt0_arcache[(SRP_LAST*4) +: 4]),
        .arprot_s(rt0_arprot[(SRP_LAST*3) +: 3]),
        .arvalid_m(sextmem_arvalid[BYPASS_IDX]), .arready_m(sextmem_arready[BYPASS_IDX]),
        .arid_m(sextmem_arid[(BYPASS_IDX*SEXTMEM_ID_WIDTH) +: SEXTMEM_ID_WIDTH]),
        .araddr_m(sextmem_araddr[(BYPASS_IDX*ADDR_WIDTH_M) +: ADDR_WIDTH_M]),
        .arlen_m(sextmem_arlen[(BYPASS_IDX*4) +: 4]),
        .arsize_m(sextmem_arsize[(BYPASS_IDX*3) +: 3]),
        .arburst_m(sextmem_arburst[(BYPASS_IDX*2) +: 2]),
        .arlock_m(sextmem_arlock[(BYPASS_IDX*2) +: 2]),
        .arcache_m(sextmem_arcache[(BYPASS_IDX*4) +: 4]),
        .arprot_m(sextmem_arprot[(BYPASS_IDX*3) +: 3]),
        .rvalid_s(sextmem_rvalid[BYPASS_IDX]), .rready_s(sextmem_rready[BYPASS_IDX]),
        .rid_s(sextmem_rid[(BYPASS_IDX*SEXTMEM_ID_WIDTH) +: SEXTMEM_ID_WIDTH]),
        .rdata_s(sextmem_rdata[(BYPASS_IDX*128) +: 128]),
        .rresp_s(sextmem_rresp[(BYPASS_IDX*2) +: 2]),
        .rlast_s(sextmem_rlast[BYPASS_IDX]),
        .rvalid_m(rt0_rvalid[SRP_LAST]), .rready_m(rt0_rready[SRP_LAST]),
        .rid_m(bp_rid_z),
        .rdata_m(rt0_rdata[(SRP_LAST*128) +: 128]),
        .rresp_m(rt0_rresp[(SRP_LAST*2) +: 2]),
        .rlast_m(rt0_rlast[SRP_LAST])
    );

    // Target1 (io): msrp[0..8] -> io 9:1 merge -> sextio
    wire mem_io_m_awvalid, mem_io_m_wvalid, mem_io_m_bvalid, mem_io_m_arvalid, mem_io_m_rvalid;
    wire mem_io_m_awready, mem_io_m_wready, mem_io_m_bready, mem_io_m_arready, mem_io_m_rready;
    wire [SEXTIO_ID_WIDTH-1:0] mem_io_m_awid, mem_io_m_wid, mem_io_m_bid, mem_io_m_arid, mem_io_m_rid;
    wire [ADDR_WIDTH_M-1:0] mem_io_m_awaddr, mem_io_m_araddr;
    wire [3:0] mem_io_m_awlen, mem_io_m_arlen;
    wire [2:0] mem_io_m_awsize, mem_io_m_arsize;
    wire [1:0] mem_io_m_awburst, mem_io_m_arburst, mem_io_m_awlock, mem_io_m_arlock;
    wire [3:0] mem_io_m_awcache, mem_io_m_arcache;
    wire [2:0] mem_io_m_awprot, mem_io_m_arprot;
    wire [127:0] mem_io_m_wdata, mem_io_m_rdata;
    wire [15:0]  mem_io_m_wstrb;
    wire mem_io_m_wlast, mem_io_m_rlast;
    wire [1:0] mem_io_m_bresp, mem_io_m_rresp;

    axi3_merge_Nto1_128 #(
        .N(NUM_SRP),
        .ADDR_WIDTH(ADDR_WIDTH_M),
        .IN_ID_WIDTH(MSRP_ID_WIDTH),
        .WR_OUTSTANDING_DEPTH(MSRP_WR_PENDING_DEPTH),
        .SRC_ID_WIDTH(IO_MERGE_TAG_W)
    ) u_io_merge (
        .aclk(aclk), .aresetn(aresetn),
        .s_awvalid(rt1_awvalid), .s_awready(rt1_awready), .s_awid(rt1_awid),
        .s_awaddr(rt1_awaddr), .s_awlen(rt1_awlen), .s_awsize(rt1_awsize),
        .s_awburst(rt1_awburst), .s_awlock(rt1_awlock), .s_awcache(rt1_awcache),
        .s_awprot(rt1_awprot),
        .s_wvalid(rt1_wvalid), .s_wready(rt1_wready), .s_wid(rt1_wid),
        .s_wdata(rt1_wdata), .s_wstrb(rt1_wstrb), .s_wlast(rt1_wlast),
        .s_bvalid(rt1_bvalid), .s_bready(rt1_bready), .s_bid(rt1_bid),
        .s_bresp(rt1_bresp),
        .s_arvalid(rt1_arvalid), .s_arready(rt1_arready), .s_arid(rt1_arid),
        .s_araddr(rt1_araddr), .s_arlen(rt1_arlen), .s_arsize(rt1_arsize),
        .s_arburst(rt1_arburst), .s_arlock(rt1_arlock), .s_arcache(rt1_arcache),
        .s_arprot(rt1_arprot),
        .s_rvalid(rt1_rvalid), .s_rready(rt1_rready), .s_rid(rt1_rid),
        .s_rdata(rt1_rdata), .s_rresp(rt1_rresp), .s_rlast(rt1_rlast),
        .m_awvalid(mem_io_m_awvalid), .m_awready(mem_io_m_awready), .m_awid(mem_io_m_awid),
        .m_awaddr(mem_io_m_awaddr), .m_awlen(mem_io_m_awlen), .m_awsize(mem_io_m_awsize),
        .m_awburst(mem_io_m_awburst), .m_awlock(mem_io_m_awlock), .m_awcache(mem_io_m_awcache),
        .m_awprot(mem_io_m_awprot),
        .m_wvalid(mem_io_m_wvalid), .m_wready(mem_io_m_wready), .m_wid(mem_io_m_wid),
        .m_wdata(mem_io_m_wdata), .m_wstrb(mem_io_m_wstrb), .m_wlast(mem_io_m_wlast),
        .m_bvalid(mem_io_m_bvalid), .m_bready(mem_io_m_bready), .m_bid(mem_io_m_bid),
        .m_bresp(mem_io_m_bresp),
        .m_arvalid(mem_io_m_arvalid), .m_arready(mem_io_m_arready), .m_arid(mem_io_m_arid),
        .m_araddr(mem_io_m_araddr), .m_arlen(mem_io_m_arlen), .m_arsize(mem_io_m_arsize),
        .m_arburst(mem_io_m_arburst), .m_arlock(mem_io_m_arlock), .m_arcache(mem_io_m_arcache),
        .m_arprot(mem_io_m_arprot),
        .m_rvalid(mem_io_m_rvalid), .m_rready(mem_io_m_rready), .m_rid(mem_io_m_rid),
        .m_rdata(mem_io_m_rdata), .m_rresp(mem_io_m_rresp), .m_rlast(mem_io_m_rlast)
    );

    lbus_axi128_reg_slice_wrap #(
        .ID_WIDTH(SEXTIO_ID_WIDTH), .ADDR_WIDTH(ADDR_WIDTH_M), .DATA_WIDTH(128),
        .AW_SLICE_EN(SEXT_AW_SLICE_EN), .W_SLICE_EN(SEXT_W_SLICE_EN),
        .B_SLICE_EN(SEXT_B_SLICE_EN), .AR_SLICE_EN(SEXT_AR_SLICE_EN),
        .R_SLICE_EN(SEXT_R_SLICE_EN)
    ) u_sextio_rs (
        .aclk(aclk), .aresetn(aresetn),
        .awvalid_s(mem_io_m_awvalid), .awready_s(mem_io_m_awready),
        .awid_s(mem_io_m_awid), .awaddr_s(mem_io_m_awaddr), .awlen_s(mem_io_m_awlen),
        .awsize_s(mem_io_m_awsize), .awburst_s(mem_io_m_awburst), .awlock_s(mem_io_m_awlock),
        .awcache_s(mem_io_m_awcache), .awprot_s(mem_io_m_awprot),
        .awvalid_m(sextio_awvalid), .awready_m(sextio_awready),
        .awid_m(sextio_awid), .awaddr_m(sextio_awaddr), .awlen_m(sextio_awlen),
        .awsize_m(sextio_awsize), .awburst_m(sextio_awburst), .awlock_m(sextio_awlock),
        .awcache_m(sextio_awcache), .awprot_m(sextio_awprot),
        .wvalid_s(mem_io_m_wvalid), .wready_s(mem_io_m_wready),
        .wid_s(mem_io_m_wid), .wdata_s(mem_io_m_wdata), .wstrb_s(mem_io_m_wstrb),
        .wlast_s(mem_io_m_wlast),
        .wvalid_m(sextio_wvalid), .wready_m(sextio_wready),
        .wid_m(sextio_wid), .wdata_m(sextio_wdata), .wstrb_m(sextio_wstrb),
        .wlast_m(sextio_wlast),
        .bvalid_s(sextio_bvalid), .bready_s(sextio_bready),
        .bid_s(sextio_bid), .bresp_s(sextio_bresp),
        .bvalid_m(mem_io_m_bvalid), .bready_m(mem_io_m_bready),
        .bid_m(mem_io_m_bid), .bresp_m(mem_io_m_bresp),
        .arvalid_s(mem_io_m_arvalid), .arready_s(mem_io_m_arready),
        .arid_s(mem_io_m_arid), .araddr_s(mem_io_m_araddr), .arlen_s(mem_io_m_arlen),
        .arsize_s(mem_io_m_arsize), .arburst_s(mem_io_m_arburst), .arlock_s(mem_io_m_arlock),
        .arcache_s(mem_io_m_arcache), .arprot_s(mem_io_m_arprot),
        .arvalid_m(sextio_arvalid), .arready_m(sextio_arready),
        .arid_m(sextio_arid), .araddr_m(sextio_araddr), .arlen_m(sextio_arlen),
        .arsize_m(sextio_arsize), .arburst_m(sextio_arburst), .arlock_m(sextio_arlock),
        .arcache_m(sextio_arcache), .arprot_m(sextio_arprot),
        .rvalid_s(sextio_rvalid), .rready_s(sextio_rready),
        .rid_s(sextio_rid), .rdata_s(sextio_rdata), .rresp_s(sextio_rresp),
        .rlast_s(sextio_rlast),
        .rvalid_m(mem_io_m_rvalid), .rready_m(mem_io_m_rready),
        .rid_m(mem_io_m_rid), .rdata_m(mem_io_m_rdata), .rresp_m(mem_io_m_rresp),
        .rlast_m(mem_io_m_rlast)
    );

    // mext -> reg slice -> router/AHB -> ssrp
    wire mext_rt_awvalid, mext_rt_wvalid, mext_rt_bvalid, mext_rt_arvalid, mext_rt_rvalid;
    wire mext_rt_awready, mext_rt_wready, mext_rt_bready, mext_rt_arready, mext_rt_rready;
    wire [MEXT_ID_WIDTH-1:0] mext_rt_awid, mext_rt_wid, mext_rt_bid, mext_rt_arid, mext_rt_rid;
    wire [ADDR_WIDTH_S-1:0] mext_rt_awaddr, mext_rt_araddr;
    wire [3:0] mext_rt_awlen, mext_rt_arlen;
    wire [2:0] mext_rt_awsize, mext_rt_arsize;
    wire [1:0] mext_rt_awburst, mext_rt_arburst, mext_rt_awlock, mext_rt_arlock;
    wire [3:0] mext_rt_awcache, mext_rt_arcache;
    wire [2:0] mext_rt_awprot, mext_rt_arprot;
    wire [31:0] mext_rt_wdata, mext_rt_rdata;
    wire [3:0]  mext_rt_wstrb;
    wire mext_rt_wlast, mext_rt_rlast;
    wire [1:0] mext_rt_bresp, mext_rt_rresp;
    wire [NUM_SRP-1:0] mext_rt_aw_sel, mext_rt_ar_sel;
    wire [ADDR_WIDTH_S-1:0] mext_awaddr_map, mext_araddr_map;
    wire [ADDR_WIDTH_S-1:0] mext_rt_awaddr_tgt, mext_rt_araddr_tgt;

    localparam integer MEXT_AW_P = MEXT_ID_WIDTH + ADDR_WIDTH_S + 4 + 3 + 2 + 2 + 4 + 3;
    localparam integer MEXT_AR_P = MEXT_ID_WIDTH + ADDR_WIDTH_S + 4 + 3 + 2 + 2 + 4 + 3;
    localparam integer MEXT_W_P  = MEXT_ID_WIDTH + 32 + 4 + 1;
    localparam integer MEXT_B_P  = MEXT_ID_WIDTH + 2;
    localparam integer MEXT_R_P  = MEXT_ID_WIDTH + 32 + 2 + 1;

    wire [MEXT_AW_P-1:0] mext_aw_pld_s, mext_aw_pld_m;
    wire [MEXT_W_P-1:0]  mext_w_pld_s,  mext_w_pld_m;
    wire [MEXT_B_P-1:0]  mext_b_pld_s,  mext_b_pld_m;
    wire [MEXT_AR_P-1:0] mext_ar_pld_s, mext_ar_pld_m;
    wire [MEXT_R_P-1:0]  mext_r_pld_s,  mext_r_pld_m;

    assign mext_aw_pld_s = {mext_awprot, mext_awcache, mext_awlock, mext_awburst, mext_awsize,
                            mext_awlen, mext_awaddr_map, mext_awid};
    assign {mext_rt_awprot, mext_rt_awcache, mext_rt_awlock, mext_rt_awburst, mext_rt_awsize,
            mext_rt_awlen, mext_rt_awaddr, mext_rt_awid} = mext_aw_pld_m;

    assign mext_w_pld_s = {mext_wlast, mext_wstrb, mext_wdata, mext_wid};
    assign {mext_rt_wlast, mext_rt_wstrb, mext_rt_wdata, mext_rt_wid} = mext_w_pld_m;

    assign mext_b_pld_s = {mext_rt_bresp, mext_rt_bid};
    assign {mext_bresp, mext_bid} = mext_b_pld_m;

    assign mext_ar_pld_s = {mext_arprot, mext_arcache, mext_arlock, mext_arburst, mext_arsize,
                            mext_arlen, mext_araddr_map, mext_arid};
    assign {mext_rt_arprot, mext_rt_arcache, mext_rt_arlock, mext_rt_arburst, mext_rt_arsize,
            mext_rt_arlen, mext_rt_araddr, mext_rt_arid} = mext_ar_pld_m;

    assign mext_r_pld_s = {mext_rt_rlast, mext_rt_rresp, mext_rt_rdata, mext_rt_rid};
    assign {mext_rlast, mext_rresp, mext_rdata, mext_rid} = mext_r_pld_m;

    axi3_ahb_region_sel #(
        .N_PORTS(NUM_SRP),
        .ADDR_WIDTH(ADDR_WIDTH_S),
        .REGION_SIZE_KB(AHB_REGION_SIZE_KB)
    ) u_mext_awaddr_map (
        .addr(mext_awaddr),
        .sel(),
        .addr_map(mext_awaddr_map),
        .addr_tgt()
    );

    axi3_ahb_region_sel #(
        .N_PORTS(NUM_SRP),
        .ADDR_WIDTH(ADDR_WIDTH_S),
        .REGION_SIZE_KB(AHB_REGION_SIZE_KB)
    ) u_mext_araddr_map (
        .addr(mext_araddr),
        .sel(),
        .addr_map(mext_araddr_map),
        .addr_tgt()
    );

    axi3_reg_slice_ch #(
        .PAYLOAD_WIDTH(MEXT_AW_P),
        .ENABLE(MEXT_AW_SLICE_EN)
    ) u_mext_aw_slice (
        .aclk(aclk), .aresetn(aresetn),
        .valid_s(mext_awvalid), .ready_s(mext_awready), .payload_s(mext_aw_pld_s),
        .valid_m(mext_rt_awvalid), .ready_m(mext_rt_awready), .payload_m(mext_aw_pld_m)
    );

    axi3_reg_slice_ch #(
        .PAYLOAD_WIDTH(MEXT_W_P),
        .ENABLE(MEXT_W_SLICE_EN)
    ) u_mext_w_slice (
        .aclk(aclk), .aresetn(aresetn),
        .valid_s(mext_wvalid), .ready_s(mext_wready), .payload_s(mext_w_pld_s),
        .valid_m(mext_rt_wvalid), .ready_m(mext_rt_wready), .payload_m(mext_w_pld_m)
    );

    axi3_reg_slice_ch #(
        .PAYLOAD_WIDTH(MEXT_B_P),
        .ENABLE(MEXT_B_SLICE_EN)
    ) u_mext_b_slice (
        .aclk(aclk), .aresetn(aresetn),
        .valid_s(mext_rt_bvalid), .ready_s(mext_rt_bready), .payload_s(mext_b_pld_s),
        .valid_m(mext_bvalid), .ready_m(mext_bready), .payload_m(mext_b_pld_m)
    );

    axi3_reg_slice_ch #(
        .PAYLOAD_WIDTH(MEXT_AR_P),
        .ENABLE(MEXT_AR_SLICE_EN)
    ) u_mext_ar_slice (
        .aclk(aclk), .aresetn(aresetn),
        .valid_s(mext_arvalid), .ready_s(mext_arready), .payload_s(mext_ar_pld_s),
        .valid_m(mext_rt_arvalid), .ready_m(mext_rt_arready), .payload_m(mext_ar_pld_m)
    );

    axi3_reg_slice_ch #(
        .PAYLOAD_WIDTH(MEXT_R_P),
        .ENABLE(MEXT_R_SLICE_EN)
    ) u_mext_r_slice (
        .aclk(aclk), .aresetn(aresetn),
        .valid_s(mext_rt_rvalid), .ready_s(mext_rt_rready), .payload_s(mext_r_pld_s),
        .valid_m(mext_rvalid), .ready_m(mext_rready), .payload_m(mext_r_pld_m)
    );

    axi3_ahb_region_sel #(
        .N_PORTS(NUM_SRP),
        .ADDR_WIDTH(ADDR_WIDTH_S),
        .REGION_SIZE_KB(AHB_REGION_SIZE_KB)
    ) u_mext_aw_region_sel (
        .addr(mext_rt_awaddr),
        .sel(mext_rt_aw_sel),
        .addr_map(),
        .addr_tgt(mext_rt_awaddr_tgt)
    );

    axi3_ahb_region_sel #(
        .N_PORTS(NUM_SRP),
        .ADDR_WIDTH(ADDR_WIDTH_S),
        .REGION_SIZE_KB(AHB_REGION_SIZE_KB)
    ) u_mext_ar_region_sel (
        .addr(mext_rt_araddr),
        .sel(mext_rt_ar_sel),
        .addr_map(),
        .addr_tgt(mext_rt_araddr_tgt)
    );

    axi3_router_1toN_ahblite #(
        .N(NUM_SRP),
        .ADDR_WIDTH(ADDR_WIDTH_S),
        .ID_WIDTH(MEXT_ID_WIDTH),
        .ROUTER_OUTSTANDING(ROUTER_OUTSTANDING),
        .WR_CMD_DEPTH(WR_CMD_DEPTH),
        .RD_CMD_DEPTH(RD_CMD_DEPTH),
        .RESP_DEPTH(RESP_DEPTH)
    ) u_mext_router_ahb (
        .aclk(aclk), .aresetn(aresetn),
        .aw_sel(mext_rt_aw_sel), .ar_sel(mext_rt_ar_sel),
        .s_awvalid(mext_rt_awvalid), .s_awready(mext_rt_awready),
        .s_awid(mext_rt_awid), .s_awaddr(mext_rt_awaddr_tgt), .s_awlen(mext_rt_awlen),
        .s_awsize(mext_rt_awsize), .s_awburst(mext_rt_awburst), .s_awlock(mext_rt_awlock),
        .s_awcache(mext_rt_awcache), .s_awprot(mext_rt_awprot),
        .s_wvalid(mext_rt_wvalid), .s_wready(mext_rt_wready),
        .s_wid(mext_rt_wid), .s_wdata(mext_rt_wdata), .s_wstrb(mext_rt_wstrb),
        .s_wlast(mext_rt_wlast),
        .s_bvalid(mext_rt_bvalid), .s_bready(mext_rt_bready),
        .s_bid(mext_rt_bid), .s_bresp(mext_rt_bresp),
        .s_arvalid(mext_rt_arvalid), .s_arready(mext_rt_arready),
        .s_arid(mext_rt_arid), .s_araddr(mext_rt_araddr_tgt), .s_arlen(mext_rt_arlen),
        .s_arsize(mext_rt_arsize), .s_arburst(mext_rt_arburst), .s_arlock(mext_rt_arlock),
        .s_arcache(mext_rt_arcache), .s_arprot(mext_rt_arprot),
        .s_rvalid(mext_rt_rvalid), .s_rready(mext_rt_rready),
        .s_rid(mext_rt_rid), .s_rdata(mext_rt_rdata), .s_rresp(mext_rt_rresp),
        .s_rlast(mext_rt_rlast),
        .haddr(ssrp_haddr), .htrans(ssrp_htrans), .hwrite(ssrp_hwrite),
        .hsize(ssrp_hsize), .hburst(ssrp_hburst),
        .hwdata(ssrp_hwdata), .hrdata(ssrp_hrdata),
        .hready(ssrp_hready), .hresp(ssrp_hresp)
    );

    // -------------------------------------------------------------------------
    // Elaboration checks
    // -------------------------------------------------------------------------
    `ifdef SYNTHESIS
    `else
    initial begin
        if (MSRP_MERGE_CFG < 1 || MSRP_MERGE_CFG > 3) begin
            $error("%m: lbus_srps: MSRP_MERGE_CFG must be 1, 2, or 3 (got %0d)",
                   MSRP_MERGE_CFG);
        end
        if (SEXTMEM_PORT_NUM != ((1 << (MSRP_MERGE_CFG - 1)) + 1)) begin
            $error("%m: lbus_srps: SEXTMEM_PORT_NUM (%0d) must equal (1<<(MSRP_MERGE_CFG-1))+1 (%0d)",
                   SEXTMEM_PORT_NUM, (1 << (MSRP_MERGE_CFG - 1)) + 1);
        end
        if (SEXTMEM_ID_WIDTH < MSRP_ID_WIDTH + MEM_MERGE_TAG_W) begin
            $error("%m: lbus_srps: SEXTMEM_ID_WIDTH must be >= MSRP_ID_WIDTH+MEM_MERGE_TAG_W (%0d)",
                   MSRP_ID_WIDTH + MEM_MERGE_TAG_W);
        end
        if (SEXTIO_ID_WIDTH < MSRP_ID_WIDTH + IO_MERGE_TAG_W) begin
            $error("%m: lbus_srps: SEXTIO_ID_WIDTH must be >= MSRP_ID_WIDTH+IO_MERGE_TAG_W (%0d)",
                   MSRP_ID_WIDTH + IO_MERGE_TAG_W);
        end
        if (MSRP_WR_PENDING_DEPTH < 1) begin
            $error("%m: lbus_srps: MSRP_WR_PENDING_DEPTH must be >= 1 (got %0d)",
                   MSRP_WR_PENDING_DEPTH);
        end
        if (SEXTMEM_BYPASS_PAD_W < 0) begin
            $error("%m: lbus_srps: SEXTMEM_ID_WIDTH must be >= MSRP_ID_WIDTH for bypass pad (got %0d)",
                   SEXTMEM_ID_WIDTH);
        end
        if (IO_MERGE_TAG_W < $clog2(NUM_SRP)) begin
            $error("%m: lbus_srps: IO_MERGE_TAG_W must be >= $clog2(NUM_SRP) for io 9:1 merge (got %0d, need %0d)",
                   IO_MERGE_TAG_W, $clog2(NUM_SRP));
        end
    end
    `endif

endmodule
