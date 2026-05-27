// -----------------------------------------------------------------------------
// Module      : lbus_srps
// Date        : 2026-05-28
// Version     : v1.0.0
// Author      : Jongchul Shin
// Function    : SRPS local bus interconnect top.
//               9x AXI3 128-bit slave (msrp0~8) -> 1:2 router -> merge -> sext*.
//               1x AXI3 32-bit slave (mext0) -> router/AHB -> 9x AHB-Lite (ssrp0~8).
// Assumptions : aw_sel/ar_sel are provided externally per port (one-hot).
//               Register slices are placed on sext* and mext0 boundaries.
// Notes       : msrp index i corresponds to msrp<i> port.
// -----------------------------------------------------------------------------
module lbus_srps #(
    parameter integer MSR_ID_WIDTH       = 4,              // msrp AXI ID width
    parameter integer MEXT_ID_WIDTH      = 4,              // mext0 AXI ID width
    parameter integer ADDR_WIDTH         = 32,             // Address width
    parameter integer DATA_WIDTH_128     = 128,            // msrp/sext data width
    parameter integer DATA_WIDTH_32      = 32,             // mext data width
    parameter integer HADDR_LOW_BITS     = 32,             // AHB haddr output mask width
    parameter integer ROUTER_OUTSTANDING   = 16,           // mext router outstanding depth
    parameter integer WR_CMD_DEPTH       = 16,             // AHB bridge write command depth
    parameter integer RD_CMD_DEPTH       = 16,             // AHB bridge read command depth
    parameter integer RESP_DEPTH         = 8,              // AHB bridge response depth
    parameter         BUSY_ENABLE        = 1'b1,           // AHB bridge BUSY enable
    parameter         SEXT_SLICE_EN      = 1'b1,           // Register slice on sext* ports
    parameter         MEXT_SLICE_EN      = 1'b1            // Register slice on mext0 port
) (
    input  wire                         aclk,
    input  wire                         aresetn,

    // msrp0~8: AXI3 128-bit slave (index 0..8)
    input  wire [8:0]                   msrp_awvalid,
    output wire [8:0]                   msrp_awready,
    input  wire [8*MSR_ID_WIDTH-1:0]    msrp_awid,
    input  wire [8*ADDR_WIDTH-1:0]      msrp_awaddr,
    input  wire [8*4-1:0]               msrp_awlen,
    input  wire [8*3-1:0]               msrp_awsize,
    input  wire [8*2-1:0]               msrp_awburst,
    input  wire [8*2-1:0]               msrp_awlock,
    input  wire [8*4-1:0]               msrp_awcache,
    input  wire [8*3-1:0]               msrp_awprot,
    input  wire [8*2-1:0]               msrp_aw_sel,

    input  wire [8:0]                   msrp_wvalid,
    output wire [8:0]                   msrp_wready,
    input  wire [8*MSR_ID_WIDTH-1:0]    msrp_wid,
    input  wire [8*DATA_WIDTH_128-1:0]  msrp_wdata,
    input  wire [8*(DATA_WIDTH_128/8)-1:0] msrp_wstrb,
    input  wire [8:0]                   msrp_wlast,

    output wire [8:0]                   msrp_bvalid,
    input  wire [8:0]                   msrp_bready,
    output wire [8*MSR_ID_WIDTH-1:0]    msrp_bid,
    output wire [8*2-1:0]               msrp_bresp,

    input  wire [8:0]                   msrp_arvalid,
    output wire [8:0]                   msrp_arready,
    input  wire [8*MSR_ID_WIDTH-1:0]    msrp_arid,
    input  wire [8*ADDR_WIDTH-1:0]      msrp_araddr,
    input  wire [8*4-1:0]               msrp_arlen,
    input  wire [8*3-1:0]               msrp_arsize,
    input  wire [8*2-1:0]               msrp_arburst,
    input  wire [8*2-1:0]               msrp_arlock,
    input  wire [8*4-1:0]               msrp_arcache,
    input  wire [8*3-1:0]               msrp_arprot,
    input  wire [8*2-1:0]               msrp_ar_sel,

    output wire [8:0]                   msrp_rvalid,
    input  wire [8:0]                   msrp_rready,
    output wire [8*MSR_ID_WIDTH-1:0]    msrp_rid,
    output wire [8*DATA_WIDTH_128-1:0]  msrp_rdata,
    output wire [8*2-1:0]               msrp_rresp,
    output wire [8:0]                   msrp_rlast,

    // sextmem0~2, sextio0: AXI3 128-bit master
    output wire                         sextmem0_awvalid,
    input  wire                         sextmem0_awready,
    output wire [MSR_ID_WIDTH+2-1:0]    sextmem0_awid,
    output wire [ADDR_WIDTH-1:0]        sextmem0_awaddr,
    output wire [3:0]                   sextmem0_awlen,
    output wire [2:0]                   sextmem0_awsize,
    output wire [1:0]                   sextmem0_awburst,
    output wire [1:0]                   sextmem0_awlock,
    output wire [3:0]                   sextmem0_awcache,
    output wire [2:0]                   sextmem0_awprot,

    output wire                         sextmem0_wvalid,
    input  wire                         sextmem0_wready,
    output wire [MSR_ID_WIDTH+2-1:0]    sextmem0_wid,
    output wire [DATA_WIDTH_128-1:0]    sextmem0_wdata,
    output wire [DATA_WIDTH_128/8-1:0]  sextmem0_wstrb,
    output wire                         sextmem0_wlast,

    input  wire                         sextmem0_bvalid,
    output wire                         sextmem0_bready,
    input  wire [MSR_ID_WIDTH+2-1:0]    sextmem0_bid,
    input  wire [1:0]                   sextmem0_bresp,

    output wire                         sextmem0_arvalid,
    input  wire                         sextmem0_arready,
    output wire [MSR_ID_WIDTH+2-1:0]    sextmem0_arid,
    output wire [ADDR_WIDTH-1:0]        sextmem0_araddr,
    output wire [3:0]                   sextmem0_arlen,
    output wire [2:0]                   sextmem0_arsize,
    output wire [1:0]                   sextmem0_arburst,
    output wire [1:0]                   sextmem0_arlock,
    output wire [3:0]                   sextmem0_arcache,
    output wire [2:0]                   sextmem0_arprot,

    input  wire                         sextmem0_rvalid,
    output wire                         sextmem0_rready,
    input  wire [MSR_ID_WIDTH+2-1:0]    sextmem0_rid,
    input  wire [DATA_WIDTH_128-1:0]    sextmem0_rdata,
    input  wire [1:0]                   sextmem0_rresp,
    input  wire                         sextmem0_rlast,

    output wire                         sextmem1_awvalid,
    input  wire                         sextmem1_awready,
    output wire [MSR_ID_WIDTH+2-1:0]    sextmem1_awid,
    output wire [ADDR_WIDTH-1:0]        sextmem1_awaddr,
    output wire [3:0]                   sextmem1_awlen,
    output wire [2:0]                   sextmem1_awsize,
    output wire [1:0]                   sextmem1_awburst,
    output wire [1:0]                   sextmem1_awlock,
    output wire [3:0]                   sextmem1_awcache,
    output wire [2:0]                   sextmem1_awprot,
    output wire                         sextmem1_wvalid,
    input  wire                         sextmem1_wready,
    output wire [MSR_ID_WIDTH+2-1:0]    sextmem1_wid,
    output wire [DATA_WIDTH_128-1:0]    sextmem1_wdata,
    output wire [DATA_WIDTH_128/8-1:0]  sextmem1_wstrb,
    output wire                         sextmem1_wlast,
    input  wire                         sextmem1_bvalid,
    output wire                         sextmem1_bready,
    input  wire [MSR_ID_WIDTH+2-1:0]    sextmem1_bid,
    input  wire [1:0]                   sextmem1_bresp,
    output wire                         sextmem1_arvalid,
    input  wire                         sextmem1_arready,
    output wire [MSR_ID_WIDTH+2-1:0]    sextmem1_arid,
    output wire [ADDR_WIDTH-1:0]        sextmem1_araddr,
    output wire [3:0]                   sextmem1_arlen,
    output wire [2:0]                   sextmem1_arsize,
    output wire [1:0]                   sextmem1_arburst,
    output wire [1:0]                   sextmem1_arlock,
    output wire [3:0]                   sextmem1_arcache,
    output wire [2:0]                   sextmem1_arprot,
    input  wire                         sextmem1_rvalid,
    output wire                         sextmem1_rready,
    input  wire [MSR_ID_WIDTH+2-1:0]    sextmem1_rid,
    input  wire [DATA_WIDTH_128-1:0]    sextmem1_rdata,
    input  wire [1:0]                   sextmem1_rresp,
    input  wire                         sextmem1_rlast,

    output wire                         sextmem2_awvalid,
    input  wire                         sextmem2_awready,
    output wire [MSR_ID_WIDTH-1:0]      sextmem2_awid,
    output wire [ADDR_WIDTH-1:0]        sextmem2_awaddr,
    output wire [3:0]                   sextmem2_awlen,
    output wire [2:0]                   sextmem2_awsize,
    output wire [1:0]                   sextmem2_awburst,
    output wire [1:0]                   sextmem2_awlock,
    output wire [3:0]                   sextmem2_awcache,
    output wire [2:0]                   sextmem2_awprot,
    output wire                         sextmem2_wvalid,
    input  wire                         sextmem2_wready,
    output wire [MSR_ID_WIDTH-1:0]      sextmem2_wid,
    output wire [DATA_WIDTH_128-1:0]    sextmem2_wdata,
    output wire [DATA_WIDTH_128/8-1:0]  sextmem2_wstrb,
    output wire                         sextmem2_wlast,
    input  wire                         sextmem2_bvalid,
    output wire                         sextmem2_bready,
    input  wire [MSR_ID_WIDTH-1:0]      sextmem2_bid,
    input  wire [1:0]                   sextmem2_bresp,
    output wire                         sextmem2_arvalid,
    input  wire                         sextmem2_arready,
    output wire [MSR_ID_WIDTH-1:0]      sextmem2_arid,
    output wire [ADDR_WIDTH-1:0]        sextmem2_araddr,
    output wire [3:0]                   sextmem2_arlen,
    output wire [2:0]                   sextmem2_arsize,
    output wire [1:0]                   sextmem2_arburst,
    output wire [1:0]                   sextmem2_arlock,
    output wire [3:0]                   sextmem2_arcache,
    output wire [2:0]                   sextmem2_arprot,
    input  wire                         sextmem2_rvalid,
    output wire                         sextmem2_rready,
    input  wire [MSR_ID_WIDTH-1:0]      sextmem2_rid,
    input  wire [DATA_WIDTH_128-1:0]    sextmem2_rdata,
    input  wire [1:0]                   sextmem2_rresp,
    input  wire                         sextmem2_rlast,

    output wire                         sextio0_awvalid,
    input  wire                         sextio0_awready,
    output wire [MSR_ID_WIDTH+4-1:0]    sextio0_awid,
    output wire [ADDR_WIDTH-1:0]        sextio0_awaddr,
    output wire [3:0]                   sextio0_awlen,
    output wire [2:0]                   sextio0_awsize,
    output wire [1:0]                   sextio0_awburst,
    output wire [1:0]                   sextio0_awlock,
    output wire [3:0]                   sextio0_awcache,
    output wire [2:0]                   sextio0_awprot,
    output wire                         sextio0_wvalid,
    input  wire                         sextio0_wready,
    output wire [MSR_ID_WIDTH+4-1:0]    sextio0_wid,
    output wire [DATA_WIDTH_128-1:0]    sextio0_wdata,
    output wire [DATA_WIDTH_128/8-1:0]  sextio0_wstrb,
    output wire                         sextio0_wlast,
    input  wire                         sextio0_bvalid,
    output wire                         sextio0_bready,
    input  wire [MSR_ID_WIDTH+4-1:0]    sextio0_bid,
    input  wire [1:0]                   sextio0_bresp,
    output wire                         sextio0_arvalid,
    input  wire                         sextio0_arready,
    output wire [MSR_ID_WIDTH+4-1:0]    sextio0_arid,
    output wire [ADDR_WIDTH-1:0]        sextio0_araddr,
    output wire [3:0]                   sextio0_arlen,
    output wire [2:0]                   sextio0_arsize,
    output wire [1:0]                   sextio0_arburst,
    output wire [1:0]                   sextio0_arlock,
    output wire [3:0]                   sextio0_arcache,
    output wire [2:0]                   sextio0_arprot,
    input  wire                         sextio0_rvalid,
    output wire                         sextio0_rready,
    input  wire [MSR_ID_WIDTH+4-1:0]    sextio0_rid,
    input  wire [DATA_WIDTH_128-1:0]    sextio0_rdata,
    input  wire [1:0]                   sextio0_rresp,
    input  wire                         sextio0_rlast,

    // mext0: AXI3 32-bit slave
    input  wire                         mext0_awvalid,
    output wire                         mext0_awready,
    input  wire [MEXT_ID_WIDTH-1:0]     mext0_awid,
    input  wire [ADDR_WIDTH-1:0]       mext0_awaddr,
    input  wire [3:0]                   mext0_awlen,
    input  wire [2:0]                   mext0_awsize,
    input  wire [1:0]                   mext0_awburst,
    input  wire [1:0]                   mext0_awlock,
    input  wire [3:0]                   mext0_awcache,
    input  wire [2:0]                   mext0_awprot,
    input  wire [8:0]                   mext0_aw_sel,
    input  wire                         mext0_wvalid,
    output wire                         mext0_wready,
    input  wire [MEXT_ID_WIDTH-1:0]     mext0_wid,
    input  wire [DATA_WIDTH_32-1:0]    mext0_wdata,
    input  wire [DATA_WIDTH_32/8-1:0]  mext0_wstrb,
    input  wire                         mext0_wlast,
    output wire                         mext0_bvalid,
    input  wire                         mext0_bready,
    output wire [MEXT_ID_WIDTH-1:0]     mext0_bid,
    output wire [1:0]                   mext0_bresp,
    input  wire                         mext0_arvalid,
    output wire                         mext0_arready,
    input  wire [MEXT_ID_WIDTH-1:0]     mext0_arid,
    input  wire [ADDR_WIDTH-1:0]       mext0_araddr,
    input  wire [3:0]                   mext0_arlen,
    input  wire [2:0]                   mext0_arsize,
    input  wire [1:0]                   mext0_arburst,
    input  wire [1:0]                   mext0_arlock,
    input  wire [3:0]                   mext0_arcache,
    input  wire [2:0]                   mext0_arprot,
    input  wire [8:0]                   mext0_ar_sel,
    output wire                         mext0_rvalid,
    input  wire                         mext0_rready,
    output wire [MEXT_ID_WIDTH-1:0]     mext0_rid,
    output wire [DATA_WIDTH_32-1:0]    mext0_rdata,
    output wire [1:0]                   mext0_rresp,
    output wire                         mext0_rlast,

    // ssrp0~8: AHB-Lite master (index 0..8)
    output wire [9*ADDR_WIDTH-1:0]      ssrp_haddr,
    output wire [9*2-1:0]               ssrp_htrans,
    output wire [8:0]                   ssrp_hwrite,
    output wire [9*3-1:0]               ssrp_hsize,
    output wire [9*3-1:0]               ssrp_hburst,
    output wire [9*4-1:0]               ssrp_hprot,
    output wire [9*DATA_WIDTH_32-1:0]  ssrp_hwdata,
    input  wire [9*DATA_WIDTH_32-1:0]   ssrp_hrdata,
    input  wire [8:0]                   ssrp_hready,
    input  wire [8:0]                   ssrp_hresp
);

    localparam integer STRB_W128 = DATA_WIDTH_128 / 8;
    localparam integer STRB_W32  = DATA_WIDTH_32 / 8;
    localparam integer MEM01_ID_W = MSR_ID_WIDTH + 2;
    localparam integer IO_ID_W    = MSR_ID_WIDTH + 4;

    // Router master target0/target1 arrays
    wire [8:0]                   rt0_awvalid;
    wire [8:0]                   rt0_awready;
    wire [8*MSR_ID_WIDTH-1:0]    rt0_awid;
    wire [8*ADDR_WIDTH-1:0]      rt0_awaddr;
    wire [8*4-1:0]               rt0_awlen;
    wire [8*3-1:0]               rt0_awsize;
    wire [8*2-1:0]               rt0_awburst;
    wire [8*2-1:0]               rt0_awlock;
    wire [8*4-1:0]               rt0_awcache;
    wire [8*3-1:0]               rt0_awprot;
    wire [8:0]                   rt0_wvalid;
    wire [8:0]                   rt0_wready;
    wire [8*MSR_ID_WIDTH-1:0]    rt0_wid;
    wire [8*DATA_WIDTH_128-1:0]  rt0_wdata;
    wire [8*STRB_W128-1:0]       rt0_wstrb;
    wire [8:0]                   rt0_wlast;
    wire [8:0]                   rt0_bvalid;
    wire [8:0]                   rt0_bready;
    wire [8*MSR_ID_WIDTH-1:0]    rt0_bid;
    wire [8*2-1:0]               rt0_bresp;
    wire [8:0]                   rt0_arvalid;
    wire [8:0]                   rt0_arready;
    wire [8*MSR_ID_WIDTH-1:0]    rt0_arid;
    wire [8*ADDR_WIDTH-1:0]      rt0_araddr;
    wire [8*4-1:0]               rt0_arlen;
    wire [8*3-1:0]               rt0_arsize;
    wire [8*2-1:0]               rt0_arburst;
    wire [8*2-1:0]               rt0_arlock;
    wire [8*4-1:0]               rt0_arcache;
    wire [8*3-1:0]               rt0_arprot;
    wire [8:0]                   rt0_rvalid;
    wire [8:0]                   rt0_rready;
    wire [8*MSR_ID_WIDTH-1:0]    rt0_rid;
    wire [8*DATA_WIDTH_128-1:0]  rt0_rdata;
    wire [8*2-1:0]               rt0_rresp;
    wire [8:0]                   rt0_rlast;

    wire [8:0]                   rt1_awvalid;
    wire [8:0]                   rt1_awready;
    wire [8*MSR_ID_WIDTH-1:0]    rt1_awid;
    wire [8*ADDR_WIDTH-1:0]      rt1_awaddr;
    wire [8*4-1:0]               rt1_awlen;
    wire [8*3-1:0]               rt1_awsize;
    wire [8*2-1:0]               rt1_awburst;
    wire [8*2-1:0]               rt1_awlock;
    wire [8*4-1:0]               rt1_awcache;
    wire [8*3-1:0]               rt1_awprot;
    wire [8:0]                   rt1_wvalid;
    wire [8:0]                   rt1_wready;
    wire [8*MSR_ID_WIDTH-1:0]    rt1_wid;
    wire [8*DATA_WIDTH_128-1:0]  rt1_wdata;
    wire [8*STRB_W128-1:0]       rt1_wstrb;
    wire [8:0]                   rt1_wlast;
    wire [8:0]                   rt1_bvalid;
    wire [8:0]                   rt1_bready;
    wire [8*MSR_ID_WIDTH-1:0]    rt1_bid;
    wire [8*2-1:0]               rt1_bresp;
    wire [8:0]                   rt1_arvalid;
    wire [8:0]                   rt1_arready;
    wire [8*MSR_ID_WIDTH-1:0]    rt1_arid;
    wire [8*ADDR_WIDTH-1:0]      rt1_araddr;
    wire [8*4-1:0]               rt1_arlen;
    wire [8*3-1:0]               rt1_arsize;
    wire [8*2-1:0]               rt1_arburst;
    wire [8*2-1:0]               rt1_arlock;
    wire [8*4-1:0]               rt1_arcache;
    wire [8*3-1:0]               rt1_arprot;
    wire [8:0]                   rt1_rvalid;
    wire [8:0]                   rt1_rready;
    wire [8*MSR_ID_WIDTH-1:0]    rt1_rid;
    wire [8*DATA_WIDTH_128-1:0]  rt1_rdata;
    wire [8*2-1:0]               rt1_rresp;
    wire [8:0]                   rt1_rlast;

    genvar gi;
    generate
        for (gi = 0; gi < 9; gi = gi + 1) begin : g_msrp_router
            wire [1:0]                   m_awvalid;
            wire [1:0]                   m_awready;
            wire [2*MSR_ID_WIDTH-1:0]    m_awid;
            wire [2*ADDR_WIDTH-1:0]      m_awaddr;
            wire [2*4-1:0]               m_awlen;
            wire [2*3-1:0]               m_awsize;
            wire [2*2-1:0]               m_awburst;
            wire [2*2-1:0]               m_awlock;
            wire [2*4-1:0]               m_awcache;
            wire [2*3-1:0]               m_awprot;
            wire [1:0]                   m_wvalid;
            wire [1:0]                   m_wready;
            wire [2*MSR_ID_WIDTH-1:0]    m_wid;
            wire [2*DATA_WIDTH_128-1:0]  m_wdata;
            wire [2*STRB_W128-1:0]       m_wstrb;
            wire [1:0]                   m_wlast;
            wire [1:0]                   m_bvalid;
            wire [1:0]                   m_bready;
            wire [2*MSR_ID_WIDTH-1:0]    m_bid;
            wire [2*2-1:0]               m_bresp;
            wire [1:0]                   m_arvalid;
            wire [1:0]                   m_arready;
            wire [2*MSR_ID_WIDTH-1:0]    m_arid;
            wire [2*ADDR_WIDTH-1:0]      m_araddr;
            wire [2*4-1:0]               m_arlen;
            wire [2*3-1:0]               m_arsize;
            wire [2*2-1:0]               m_arburst;
            wire [2*2-1:0]               m_arlock;
            wire [2*4-1:0]               m_arcache;
            wire [2*3-1:0]               m_arprot;
            wire [1:0]                   m_rvalid;
            wire [1:0]                   m_rready;
            wire [2*MSR_ID_WIDTH-1:0]    m_rid;
            wire [2*DATA_WIDTH_128-1:0]  m_rdata;
            wire [2*2-1:0]               m_rresp;
            wire [1:0]                   m_rlast;

            axi3_router_1to2_128 #(
                .ADDR_WIDTH(ADDR_WIDTH),
                .DATA_WIDTH(DATA_WIDTH_128),
                .STRB_WIDTH(STRB_W128),
                .ID_WIDTH(MSR_ID_WIDTH)
            ) u_router (
                .aclk(aclk), .aresetn(aresetn),
                .aw_sel(msrp_aw_sel[(gi*2) +: 2]),
                .ar_sel(msrp_ar_sel[(gi*2) +: 2]),
                .s_awvalid(msrp_awvalid[gi]),
                .s_awready(msrp_awready[gi]),
                .s_awid(msrp_awid[(gi*MSR_ID_WIDTH) +: MSR_ID_WIDTH]),
                .s_awaddr(msrp_awaddr[(gi*ADDR_WIDTH) +: ADDR_WIDTH]),
                .s_awlen(msrp_awlen[(gi*4) +: 4]),
                .s_awsize(msrp_awsize[(gi*3) +: 3]),
                .s_awburst(msrp_awburst[(gi*2) +: 2]),
                .s_awlock(msrp_awlock[(gi*2) +: 2]),
                .s_awcache(msrp_awcache[(gi*4) +: 4]),
                .s_awprot(msrp_awprot[(gi*3) +: 3]),
                .s_wvalid(msrp_wvalid[gi]),
                .s_wready(msrp_wready[gi]),
                .s_wid(msrp_wid[(gi*MSR_ID_WIDTH) +: MSR_ID_WIDTH]),
                .s_wdata(msrp_wdata[(gi*DATA_WIDTH_128) +: DATA_WIDTH_128]),
                .s_wstrb(msrp_wstrb[(gi*STRB_W128) +: STRB_W128]),
                .s_wlast(msrp_wlast[gi]),
                .s_bvalid(msrp_bvalid[gi]),
                .s_bready(msrp_bready[gi]),
                .s_bid(msrp_bid[(gi*MSR_ID_WIDTH) +: MSR_ID_WIDTH]),
                .s_bresp(msrp_bresp[(gi*2) +: 2]),
                .s_arvalid(msrp_arvalid[gi]),
                .s_arready(msrp_arready[gi]),
                .s_arid(msrp_arid[(gi*MSR_ID_WIDTH) +: MSR_ID_WIDTH]),
                .s_araddr(msrp_araddr[(gi*ADDR_WIDTH) +: ADDR_WIDTH]),
                .s_arlen(msrp_arlen[(gi*4) +: 4]),
                .s_arsize(msrp_arsize[(gi*3) +: 3]),
                .s_arburst(msrp_arburst[(gi*2) +: 2]),
                .s_arlock(msrp_arlock[(gi*2) +: 2]),
                .s_arcache(msrp_arcache[(gi*4) +: 4]),
                .s_arprot(msrp_arprot[(gi*3) +: 3]),
                .s_rvalid(msrp_rvalid[gi]),
                .s_rready(msrp_rready[gi]),
                .s_rid(msrp_rid[(gi*MSR_ID_WIDTH) +: MSR_ID_WIDTH]),
                .s_rdata(msrp_rdata[(gi*DATA_WIDTH_128) +: DATA_WIDTH_128]),
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
            assign rt0_awready[gi]  = m_awready[0];
            assign rt0_awid[(gi*MSR_ID_WIDTH) +: MSR_ID_WIDTH] =
                m_awid[(0*MSR_ID_WIDTH) +: MSR_ID_WIDTH];
            assign rt0_awaddr[(gi*ADDR_WIDTH) +: ADDR_WIDTH] =
                m_awaddr[(0*ADDR_WIDTH) +: ADDR_WIDTH];
            assign rt0_awlen[(gi*4) +: 4]   = m_awlen[(0*4) +: 4];
            assign rt0_awsize[(gi*3) +: 3]  = m_awsize[(0*3) +: 3];
            assign rt0_awburst[(gi*2) +: 2] = m_awburst[(0*2) +: 2];
            assign rt0_awlock[(gi*2) +: 2]  = m_awlock[(0*2) +: 2];
            assign rt0_awcache[(gi*4) +: 4] = m_awcache[(0*4) +: 4];
            assign rt0_awprot[(gi*3) +: 3]  = m_awprot[(0*3) +: 3];
            assign rt0_wvalid[gi]  = m_wvalid[0];
            assign rt0_wready[gi]  = m_wready[0];
            assign rt0_wid[(gi*MSR_ID_WIDTH) +: MSR_ID_WIDTH] =
                m_wid[(0*MSR_ID_WIDTH) +: MSR_ID_WIDTH];
            assign rt0_wdata[(gi*DATA_WIDTH_128) +: DATA_WIDTH_128] =
                m_wdata[(0*DATA_WIDTH_128) +: DATA_WIDTH_128];
            assign rt0_wstrb[(gi*STRB_W128) +: STRB_W128] =
                m_wstrb[(0*STRB_W128) +: STRB_W128];
            assign rt0_wlast[gi] = m_wlast[0];
            assign m_bready[0] = rt0_bready[gi];
            assign rt0_bvalid[gi] = m_bvalid[0];
            assign rt0_bid[(gi*MSR_ID_WIDTH) +: MSR_ID_WIDTH] =
                m_bid[(0*MSR_ID_WIDTH) +: MSR_ID_WIDTH];
            assign rt0_bresp[(gi*2) +: 2] = m_bresp[(0*2) +: 2];
            assign rt0_arvalid[gi] = m_arvalid[0];
            assign rt0_arready[gi] = m_arready[0];
            assign rt0_arid[(gi*MSR_ID_WIDTH) +: MSR_ID_WIDTH] =
                m_arid[(0*MSR_ID_WIDTH) +: MSR_ID_WIDTH];
            assign rt0_araddr[(gi*ADDR_WIDTH) +: ADDR_WIDTH] =
                m_araddr[(0*ADDR_WIDTH) +: ADDR_WIDTH];
            assign rt0_arlen[(gi*4) +: 4]   = m_arlen[(0*4) +: 4];
            assign rt0_arsize[(gi*3) +: 3]  = m_arsize[(0*3) +: 3];
            assign rt0_arburst[(gi*2) +: 2] = m_arburst[(0*2) +: 2];
            assign rt0_arlock[(gi*2) +: 2]  = m_arlock[(0*2) +: 2];
            assign rt0_arcache[(gi*4) +: 4] = m_arcache[(0*4) +: 4];
            assign rt0_arprot[(gi*3) +: 3]  = m_arprot[(0*3) +: 3];
            assign rt0_rvalid[gi] = m_rvalid[0];
            assign rt0_rready[gi] = m_rready[0];
            assign rt0_rid[(gi*MSR_ID_WIDTH) +: MSR_ID_WIDTH] =
                m_rid[(0*MSR_ID_WIDTH) +: MSR_ID_WIDTH];
            assign rt0_rdata[(gi*DATA_WIDTH_128) +: DATA_WIDTH_128] =
                m_rdata[(0*DATA_WIDTH_128) +: DATA_WIDTH_128];
            assign rt0_rresp[(gi*2) +: 2] = m_rresp[(0*2) +: 2];
            assign rt0_rlast[gi] = m_rlast[0];

            assign rt1_awvalid[gi]  = m_awvalid[1];
            assign rt1_awready[gi]  = m_awready[1];
            assign rt1_awid[(gi*MSR_ID_WIDTH) +: MSR_ID_WIDTH] =
                m_awid[(1*MSR_ID_WIDTH) +: MSR_ID_WIDTH];
            assign rt1_awaddr[(gi*ADDR_WIDTH) +: ADDR_WIDTH] =
                m_awaddr[(1*ADDR_WIDTH) +: ADDR_WIDTH];
            assign rt1_awlen[(gi*4) +: 4]   = m_awlen[(1*4) +: 4];
            assign rt1_awsize[(gi*3) +: 3]  = m_awsize[(1*3) +: 3];
            assign rt1_awburst[(gi*2) +: 2] = m_awburst[(1*2) +: 2];
            assign rt1_awlock[(gi*2) +: 2]  = m_awlock[(1*2) +: 2];
            assign rt1_awcache[(gi*4) +: 4] = m_awcache[(1*4) +: 4];
            assign rt1_awprot[(gi*3) +: 3]  = m_awprot[(1*3) +: 3];
            assign rt1_wvalid[gi]  = m_wvalid[1];
            assign rt1_wready[gi]  = m_wready[1];
            assign rt1_wid[(gi*MSR_ID_WIDTH) +: MSR_ID_WIDTH] =
                m_wid[(1*MSR_ID_WIDTH) +: MSR_ID_WIDTH];
            assign rt1_wdata[(gi*DATA_WIDTH_128) +: DATA_WIDTH_128] =
                m_wdata[(1*DATA_WIDTH_128) +: DATA_WIDTH_128];
            assign rt1_wstrb[(gi*STRB_W128) +: STRB_W128] =
                m_wstrb[(1*STRB_W128) +: STRB_W128];
            assign rt1_wlast[gi] = m_wlast[1];
            assign m_bready[1] = rt1_bready[gi];
            assign rt1_bvalid[gi] = m_bvalid[1];
            assign rt1_bid[(gi*MSR_ID_WIDTH) +: MSR_ID_WIDTH] =
                m_bid[(1*MSR_ID_WIDTH) +: MSR_ID_WIDTH];
            assign rt1_bresp[(gi*2) +: 2] = m_bresp[(1*2) +: 2];
            assign rt1_arvalid[gi] = m_arvalid[1];
            assign rt1_arready[gi] = m_arready[1];
            assign rt1_arid[(gi*MSR_ID_WIDTH) +: MSR_ID_WIDTH] =
                m_arid[(1*MSR_ID_WIDTH) +: MSR_ID_WIDTH];
            assign rt1_araddr[(gi*ADDR_WIDTH) +: ADDR_WIDTH] =
                m_araddr[(1*ADDR_WIDTH) +: ADDR_WIDTH];
            assign rt1_arlen[(gi*4) +: 4]   = m_arlen[(1*4) +: 4];
            assign rt1_arsize[(gi*3) +: 3]  = m_arsize[(1*3) +: 3];
            assign rt1_arburst[(gi*2) +: 2] = m_arburst[(1*2) +: 2];
            assign rt1_arlock[(gi*2) +: 2]  = m_arlock[(1*2) +: 2];
            assign rt1_arcache[(gi*4) +: 4] = m_arcache[(1*4) +: 4];
            assign rt1_arprot[(gi*3) +: 3]  = m_arprot[(1*3) +: 3];
            assign rt1_rvalid[gi] = m_rvalid[1];
            assign rt1_rready[gi] = m_rready[1];
            assign rt1_rid[(gi*MSR_ID_WIDTH) +: MSR_ID_WIDTH] =
                m_rid[(1*MSR_ID_WIDTH) +: MSR_ID_WIDTH];
            assign rt1_rdata[(gi*DATA_WIDTH_128) +: DATA_WIDTH_128] =
                m_rdata[(1*DATA_WIDTH_128) +: DATA_WIDTH_128];
            assign rt1_rresp[(gi*2) +: 2] = m_rresp[(1*2) +: 2];
            assign rt1_rlast[gi] = m_rlast[1];
        end
    endgenerate

    // Target0: msrp0~3 -> sextmem0, msrp4~7 -> sextmem1, msrp8 bypass -> sextmem2
    wire [3:0] mem0_s_awvalid;
    wire [3:0] mem0_s_awready;
    wire [3*MSR_ID_WIDTH-1:0] mem0_s_awid;
    wire [3*ADDR_WIDTH-1:0]     mem0_s_awaddr;
    wire [3*4-1:0]              mem0_s_awlen;
    wire [3*3-1:0]              mem0_s_awsize;
    wire [3*2-1:0]              mem0_s_awburst;
    wire [3*2-1:0]              mem0_s_awlock;
    wire [3*4-1:0]              mem0_s_awcache;
    wire [3*3-1:0]              mem0_s_awprot;
    wire [3:0] mem0_s_wvalid;
    wire [3:0] mem0_s_wready;
    wire [3*MSR_ID_WIDTH-1:0] mem0_s_wid;
    wire [3*DATA_WIDTH_128-1:0] mem0_s_wdata;
    wire [3*STRB_W128-1:0] mem0_s_wstrb;
    wire [3:0] mem0_s_wlast;
    wire [3:0] mem0_s_bvalid;
    wire [3:0] mem0_s_bready;
    wire [3*MSR_ID_WIDTH-1:0] mem0_s_bid;
    wire [3*2-1:0] mem0_s_bresp;
    wire [3:0] mem0_s_arvalid;
    wire [3:0] mem0_s_arready;
    wire [3*MSR_ID_WIDTH-1:0] mem0_s_arid;
    wire [3*ADDR_WIDTH-1:0] mem0_s_araddr;
    wire [3*4-1:0] mem0_s_arlen;
    wire [3*3-1:0] mem0_s_arsize;
    wire [3*2-1:0] mem0_s_arburst;
    wire [3*2-1:0] mem0_s_arlock;
    wire [3*4-1:0] mem0_s_arcache;
    wire [3*3-1:0] mem0_s_arprot;
    wire [3:0] mem0_s_rvalid;
    wire [3:0] mem0_s_rready;
    wire [3*MSR_ID_WIDTH-1:0] mem0_s_rid;
    wire [3*DATA_WIDTH_128-1:0] mem0_s_rdata;
    wire [3*2-1:0] mem0_s_rresp;
    wire [3:0] mem0_s_rlast;

    assign mem0_s_awvalid = rt0_awvalid[3:0];
    assign rt0_awready[3:0] = mem0_s_awready;
    assign mem0_s_awid    = rt0_awid[3*MSR_ID_WIDTH-1:0];
    assign mem0_s_awaddr  = rt0_awaddr[3*ADDR_WIDTH-1:0];
    assign mem0_s_awlen   = rt0_awlen[3*4-1:0];
    assign mem0_s_awsize  = rt0_awsize[3*3-1:0];
    assign mem0_s_awburst = rt0_awburst[3*2-1:0];
    assign mem0_s_awlock  = rt0_awlock[3*2-1:0];
    assign mem0_s_awcache = rt0_awcache[3*4-1:0];
    assign mem0_s_awprot  = rt0_awprot[3*3-1:0];
    assign mem0_s_wvalid  = rt0_wvalid[3:0];
    assign rt0_wready[3:0] = mem0_s_wready;
    assign mem0_s_wid     = rt0_wid[3*MSR_ID_WIDTH-1:0];
    assign mem0_s_wdata   = rt0_wdata[3*DATA_WIDTH_128-1:0];
    assign mem0_s_wstrb   = rt0_wstrb[3*STRB_W128-1:0];
    assign mem0_s_wlast   = rt0_wlast[3:0];
    assign rt0_bvalid[3:0] = mem0_s_bvalid;
    assign mem0_s_bready  = rt0_bready[3:0];
    assign rt0_bid[3*MSR_ID_WIDTH-1:0]   = mem0_s_bid;
    assign rt0_bresp[3*2-1:0]            = mem0_s_bresp;
    assign mem0_s_arvalid = rt0_arvalid[3:0];
    assign rt0_arready[3:0] = mem0_s_arready;
    assign mem0_s_arid    = rt0_arid[3*MSR_ID_WIDTH-1:0];
    assign mem0_s_araddr  = rt0_araddr[3*ADDR_WIDTH-1:0];
    assign mem0_s_arlen   = rt0_arlen[3*4-1:0];
    assign mem0_s_arsize  = rt0_arsize[3*3-1:0];
    assign mem0_s_arburst = rt0_arburst[3*2-1:0];
    assign mem0_s_arlock  = rt0_arlock[3*2-1:0];
    assign mem0_s_arcache = rt0_arcache[3*4-1:0];
    assign mem0_s_arprot  = rt0_arprot[3*3-1:0];
    assign rt0_rvalid[3:0] = mem0_s_rvalid;
    assign mem0_s_rready  = rt0_rready[3:0];
    assign rt0_rid[3*MSR_ID_WIDTH-1:0]   = mem0_s_rid;
    assign rt0_rdata[3*DATA_WIDTH_128-1:0] = mem0_s_rdata;
    assign rt0_rresp[3*2-1:0]            = mem0_s_rresp;
    assign rt0_rlast[3:0] = mem0_s_rlast;

    wire mem0_m_awvalid, mem0_m_wvalid, mem0_m_bvalid, mem0_m_arvalid, mem0_m_rvalid;
    wire mem0_m_awready, mem0_m_wready, mem0_m_bready, mem0_m_arready, mem0_m_rready;
    wire [MEM01_ID_W-1:0] mem0_m_awid, mem0_m_wid, mem0_m_bid, mem0_m_arid, mem0_m_rid;
    wire [ADDR_WIDTH-1:0] mem0_m_awaddr, mem0_m_araddr;
    wire [3:0] mem0_m_awlen, mem0_m_arlen;
    wire [2:0] mem0_m_awsize, mem0_m_arsize;
    wire [1:0] mem0_m_awburst, mem0_m_arburst, mem0_m_awlock, mem0_m_arlock;
    wire [3:0] mem0_m_awcache, mem0_m_arcache;
    wire [2:0] mem0_m_awprot, mem0_m_arprot;
    wire [DATA_WIDTH_128-1:0] mem0_m_wdata, mem0_m_rdata;
    wire [STRB_W128-1:0] mem0_m_wstrb;
    wire mem0_m_wlast, mem0_m_rlast;
    wire [1:0] mem0_m_bresp, mem0_m_rresp;

    axi3_merge_Nto1_128 #(
        .N(4), .IN_ID_WIDTH(MSR_ID_WIDTH), .SRC_ID_WIDTH(2)
    ) u_merge_mem0 (
        .aclk(aclk), .aresetn(aresetn),
        .s_awvalid(mem0_s_awvalid), .s_awready(mem0_s_awready), .s_awid(mem0_s_awid),
        .s_awaddr(mem0_s_awaddr), .s_awlen(mem0_s_awlen), .s_awsize(mem0_s_awsize),
        .s_awburst(mem0_s_awburst), .s_awlock(mem0_s_awlock), .s_awcache(mem0_s_awcache),
        .s_awprot(mem0_s_awprot),
        .s_wvalid(mem0_s_wvalid), .s_wready(mem0_s_wready), .s_wid(mem0_s_wid),
        .s_wdata(mem0_s_wdata), .s_wstrb(mem0_s_wstrb), .s_wlast(mem0_s_wlast),
        .s_bvalid(mem0_s_bvalid), .s_bready(mem0_s_bready), .s_bid(mem0_s_bid),
        .s_bresp(mem0_s_bresp),
        .s_arvalid(mem0_s_arvalid), .s_arready(mem0_s_arready), .s_arid(mem0_s_arid),
        .s_araddr(mem0_s_araddr), .s_arlen(mem0_s_arlen), .s_arsize(mem0_s_arsize),
        .s_arburst(mem0_s_arburst), .s_arlock(mem0_s_arlock), .s_arcache(mem0_s_arcache),
        .s_arprot(mem0_s_arprot),
        .s_rvalid(mem0_s_rvalid), .s_rready(mem0_s_rready), .s_rid(mem0_s_rid),
        .s_rdata(mem0_s_rdata), .s_rresp(mem0_s_rresp), .s_rlast(mem0_s_rlast),
        .m_awvalid(mem0_m_awvalid), .m_awready(mem0_m_awready), .m_awid(mem0_m_awid),
        .m_awaddr(mem0_m_awaddr), .m_awlen(mem0_m_awlen), .m_awsize(mem0_m_awsize),
        .m_awburst(mem0_m_awburst), .m_awlock(mem0_m_awlock), .m_awcache(mem0_m_awcache),
        .m_awprot(mem0_m_awprot),
        .m_wvalid(mem0_m_wvalid), .m_wready(mem0_m_wready), .m_wid(mem0_m_wid),
        .m_wdata(mem0_m_wdata), .m_wstrb(mem0_m_wstrb), .m_wlast(mem0_m_wlast),
        .m_bvalid(mem0_m_bvalid), .m_bready(mem0_m_bready), .m_bid(mem0_m_bid),
        .m_bresp(mem0_m_bresp),
        .m_arvalid(mem0_m_arvalid), .m_arready(mem0_m_arready), .m_arid(mem0_m_arid),
        .m_araddr(mem0_m_araddr), .m_arlen(mem0_m_arlen), .m_arsize(mem0_m_arsize),
        .m_arburst(mem0_m_arburst), .m_arlock(mem0_m_arlock), .m_arcache(mem0_m_arcache),
        .m_arprot(mem0_m_arprot),
        .m_rvalid(mem0_m_rvalid), .m_rready(mem0_m_rready), .m_rid(mem0_m_rid),
        .m_rdata(mem0_m_rdata), .m_rresp(mem0_m_rresp), .m_rlast(mem0_m_rlast)
    );

    lbus_axi128_reg_slice_wrap #(
        .ID_WIDTH(MEM01_ID_W), .ADDR_WIDTH(ADDR_WIDTH), .DATA_WIDTH(DATA_WIDTH_128),
        .AW_SLICE_EN(SEXT_SLICE_EN), .W_SLICE_EN(SEXT_SLICE_EN), .B_SLICE_EN(SEXT_SLICE_EN),
        .AR_SLICE_EN(SEXT_SLICE_EN), .R_SLICE_EN(SEXT_SLICE_EN)
    ) u_sextmem0_rs (
        .aclk(aclk), .aresetn(aresetn),
        .awvalid_s(mem0_m_awvalid), .awready_s(mem0_m_awready),
        .awid_s(mem0_m_awid), .awaddr_s(mem0_m_awaddr), .awlen_s(mem0_m_awlen),
        .awsize_s(mem0_m_awsize), .awburst_s(mem0_m_awburst), .awlock_s(mem0_m_awlock),
        .awcache_s(mem0_m_awcache), .awprot_s(mem0_m_awprot),
        .awvalid_m(sextmem0_awvalid), .awready_m(sextmem0_awready),
        .awid_m(sextmem0_awid), .awaddr_m(sextmem0_awaddr), .awlen_m(sextmem0_awlen),
        .awsize_m(sextmem0_awsize), .awburst_m(sextmem0_awburst), .awlock_m(sextmem0_awlock),
        .awcache_m(sextmem0_awcache), .awprot_m(sextmem0_awprot),
        .wvalid_s(mem0_m_wvalid), .wready_s(mem0_m_wready),
        .wid_s(mem0_m_wid), .wdata_s(mem0_m_wdata), .wstrb_s(mem0_m_wstrb), .wlast_s(mem0_m_wlast),
        .wvalid_m(sextmem0_wvalid), .wready_m(sextmem0_wready),
        .wid_m(sextmem0_wid), .wdata_m(sextmem0_wdata), .wstrb_m(sextmem0_wstrb),
        .wlast_m(sextmem0_wlast),
        .bvalid_s(sextmem0_bvalid), .bready_s(sextmem0_bready),
        .bid_s(sextmem0_bid), .bresp_s(sextmem0_bresp),
        .bvalid_m(mem0_m_bvalid), .bready_m(mem0_m_bready),
        .bid_m(mem0_m_bid), .bresp_m(mem0_m_bresp),
        .arvalid_s(mem0_m_arvalid), .arready_s(mem0_m_arready),
        .arid_s(mem0_m_arid), .araddr_s(mem0_m_araddr), .arlen_s(mem0_m_arlen),
        .arsize_s(mem0_m_arsize), .arburst_s(mem0_m_arburst), .arlock_s(mem0_m_arlock),
        .arcache_s(mem0_m_arcache), .arprot_s(mem0_m_arprot),
        .arvalid_m(sextmem0_arvalid), .arready_m(sextmem0_arready),
        .arid_m(sextmem0_arid), .araddr_m(sextmem0_araddr), .arlen_m(sextmem0_arlen),
        .arsize_m(sextmem0_arsize), .arburst_m(sextmem0_arburst), .arlock_m(sextmem0_arlock),
        .arcache_m(sextmem0_arcache), .arprot_m(sextmem0_arprot),
        .rvalid_s(sextmem0_rvalid), .rready_s(sextmem0_rready),
        .rid_s(sextmem0_rid), .rdata_s(sextmem0_rdata), .rresp_s(sextmem0_rresp),
        .rlast_s(sextmem0_rlast),
        .rvalid_m(mem0_m_rvalid), .rready_m(mem0_m_rready),
        .rid_m(mem0_m_rid), .rdata_m(mem0_m_rdata), .rresp_m(mem0_m_rresp),
        .rlast_m(mem0_m_rlast)
    );

    // mem1: msrp4~7 -> sextmem1 (same structure as mem0)
    wire [3:0] mem1_s_awvalid;
    wire [3:0] mem1_s_awready;
    wire [3*MSR_ID_WIDTH-1:0] mem1_s_awid;
    wire [3*ADDR_WIDTH-1:0]     mem1_s_awaddr;
    wire [3*4-1:0]              mem1_s_awlen;
    wire [3*3-1:0]              mem1_s_awsize;
    wire [3*2-1:0]              mem1_s_awburst;
    wire [3*2-1:0]              mem1_s_awlock;
    wire [3*4-1:0]              mem1_s_awcache;
    wire [3*3-1:0]              mem1_s_awprot;
    wire [3:0] mem1_s_wvalid, mem1_s_wready, mem1_s_wlast;
    wire [3*MSR_ID_WIDTH-1:0] mem1_s_wid;
    wire [3*DATA_WIDTH_128-1:0] mem1_s_wdata;
    wire [3*STRB_W128-1:0] mem1_s_wstrb;
    wire [3:0] mem1_s_bvalid, mem1_s_bready;
    wire [3*MSR_ID_WIDTH-1:0] mem1_s_bid;
    wire [3*2-1:0] mem1_s_bresp;
    wire [3:0] mem1_s_arvalid, mem1_s_arready, mem1_s_rvalid, mem1_s_rready, mem1_s_rlast;
    wire [3*MSR_ID_WIDTH-1:0] mem1_s_arid, mem1_s_rid;
    wire [3*ADDR_WIDTH-1:0] mem1_s_araddr;
    wire [3*4-1:0] mem1_s_arlen;
    wire [3*3-1:0] mem1_s_arsize;
    wire [3*2-1:0] mem1_s_arburst, mem1_s_arlock;
    wire [3*4-1:0] mem1_s_arcache;
    wire [3*3-1:0] mem1_s_arprot;
    wire [3*DATA_WIDTH_128-1:0] mem1_s_rdata;
    wire [3*2-1:0] mem1_s_rresp;

    assign mem1_s_awvalid = rt0_awvalid[7:4];
    assign rt0_awready[7:4] = mem1_s_awready;
    assign mem1_s_awid    = rt0_awid[7*MSR_ID_WIDTH-1:4*MSR_ID_WIDTH];
    assign mem1_s_awaddr  = rt0_awaddr[7*ADDR_WIDTH-1:4*ADDR_WIDTH];
    assign mem1_s_awlen   = rt0_awlen[7*4-1:4*4];
    assign mem1_s_awsize  = rt0_awsize[7*3-1:4*3];
    assign mem1_s_awburst = rt0_awburst[7*2-1:4*2];
    assign mem1_s_awlock  = rt0_awlock[7*2-1:4*2];
    assign mem1_s_awcache = rt0_awcache[7*4-1:4*4];
    assign mem1_s_awprot  = rt0_awprot[7*3-1:4*3];
    assign mem1_s_wvalid  = rt0_wvalid[7:4];
    assign rt0_wready[7:4] = mem1_s_wready;
    assign mem1_s_wid     = rt0_wid[7*MSR_ID_WIDTH-1:4*MSR_ID_WIDTH];
    assign mem1_s_wdata   = rt0_wdata[7*DATA_WIDTH_128-1:4*DATA_WIDTH_128];
    assign mem1_s_wstrb   = rt0_wstrb[7*STRB_W128-1:4*STRB_W128];
    assign mem1_s_wlast   = rt0_wlast[7:4];
    assign rt0_bvalid[7:4] = mem1_s_bvalid;
    assign mem1_s_bready  = rt0_bready[7:4];
    assign rt0_bid[7*MSR_ID_WIDTH-1:4*MSR_ID_WIDTH]   = mem1_s_bid;
    assign rt0_bresp[7*2-1:4*2]                       = mem1_s_bresp;
    assign mem1_s_arvalid = rt0_arvalid[7:4];
    assign rt0_arready[7:4] = mem1_s_arready;
    assign mem1_s_arid    = rt0_arid[7*MSR_ID_WIDTH-1:4*MSR_ID_WIDTH];
    assign mem1_s_araddr  = rt0_araddr[7*ADDR_WIDTH-1:4*ADDR_WIDTH];
    assign mem1_s_arlen   = rt0_arlen[7*4-1:4*4];
    assign mem1_s_arsize  = rt0_arsize[7*3-1:4*3];
    assign mem1_s_arburst = rt0_arburst[7*2-1:4*2];
    assign mem1_s_arlock  = rt0_arlock[7*2-1:4*2];
    assign mem1_s_arcache = rt0_arcache[7*4-1:4*4];
    assign mem1_s_arprot  = rt0_arprot[7*3-1:4*3];
    assign rt0_rvalid[7:4] = mem1_s_rvalid;
    assign mem1_s_rready  = rt0_rready[7:4];
    assign rt0_rid[7*MSR_ID_WIDTH-1:4*MSR_ID_WIDTH]       = mem1_s_rid;
    assign rt0_rdata[7*DATA_WIDTH_128-1:4*DATA_WIDTH_128] = mem1_s_rdata;
    assign rt0_rresp[7*2-1:4*2]                           = mem1_s_rresp;
    assign rt0_rlast[7:4] = mem1_s_rlast;

    wire mem1_m_awvalid, mem1_m_wvalid, mem1_m_bvalid, mem1_m_arvalid, mem1_m_rvalid;
    wire mem1_m_awready, mem1_m_wready, mem1_m_bready, mem1_m_arready, mem1_m_rready;
    wire [MEM01_ID_W-1:0] mem1_m_awid, mem1_m_wid, mem1_m_bid, mem1_m_arid, mem1_m_rid;
    wire [ADDR_WIDTH-1:0] mem1_m_awaddr, mem1_m_araddr;
    wire [3:0] mem1_m_awlen, mem1_m_arlen;
    wire [2:0] mem1_m_awsize, mem1_m_arsize;
    wire [1:0] mem1_m_awburst, mem1_m_arburst, mem1_m_awlock, mem1_m_arlock;
    wire [3:0] mem1_m_awcache, mem1_m_arcache;
    wire [2:0] mem1_m_awprot, mem1_m_arprot;
    wire [DATA_WIDTH_128-1:0] mem1_m_wdata, mem1_m_rdata;
    wire [STRB_W128-1:0] mem1_m_wstrb;
    wire mem1_m_wlast, mem1_m_rlast;
    wire [1:0] mem1_m_bresp, mem1_m_rresp;

    axi3_merge_Nto1_128 #(
        .N(4), .IN_ID_WIDTH(MSR_ID_WIDTH), .SRC_ID_WIDTH(2)
    ) u_merge_mem1 (
        .aclk(aclk), .aresetn(aresetn),
        .s_awvalid(mem1_s_awvalid), .s_awready(mem1_s_awready), .s_awid(mem1_s_awid),
        .s_awaddr(mem1_s_awaddr), .s_awlen(mem1_s_awlen), .s_awsize(mem1_s_awsize),
        .s_awburst(mem1_s_awburst), .s_awlock(mem1_s_awlock), .s_awcache(mem1_s_awcache),
        .s_awprot(mem1_s_awprot),
        .s_wvalid(mem1_s_wvalid), .s_wready(mem1_s_wready), .s_wid(mem1_s_wid),
        .s_wdata(mem1_s_wdata), .s_wstrb(mem1_s_wstrb), .s_wlast(mem1_s_wlast),
        .s_bvalid(mem1_s_bvalid), .s_bready(mem1_s_bready), .s_bid(mem1_s_bid),
        .s_bresp(mem1_s_bresp),
        .s_arvalid(mem1_s_arvalid), .s_arready(mem1_s_arready), .s_arid(mem1_s_arid),
        .s_araddr(mem1_s_araddr), .s_arlen(mem1_s_arlen), .s_arsize(mem1_s_arsize),
        .s_arburst(mem1_s_arburst), .s_arlock(mem1_s_arlock), .s_arcache(mem1_s_arcache),
        .s_arprot(mem1_s_arprot),
        .s_rvalid(mem1_s_rvalid), .s_rready(mem1_s_rready), .s_rid(mem1_s_rid),
        .s_rdata(mem1_s_rdata), .s_rresp(mem1_s_rresp), .s_rlast(mem1_s_rlast),
        .m_awvalid(mem1_m_awvalid), .m_awready(mem1_m_awready), .m_awid(mem1_m_awid),
        .m_awaddr(mem1_m_awaddr), .m_awlen(mem1_m_awlen), .m_awsize(mem1_m_awsize),
        .m_awburst(mem1_m_awburst), .m_awlock(mem1_m_awlock), .m_awcache(mem1_m_awcache),
        .m_awprot(mem1_m_awprot),
        .m_wvalid(mem1_m_wvalid), .m_wready(mem1_m_wready), .m_wid(mem1_m_wid),
        .m_wdata(mem1_m_wdata), .m_wstrb(mem1_m_wstrb), .m_wlast(mem1_m_wlast),
        .m_bvalid(mem1_m_bvalid), .m_bready(mem1_m_bready), .m_bid(mem1_m_bid),
        .m_bresp(mem1_m_bresp),
        .m_arvalid(mem1_m_arvalid), .m_arready(mem1_m_arready), .m_arid(mem1_m_arid),
        .m_araddr(mem1_m_araddr), .m_arlen(mem1_m_arlen), .m_arsize(mem1_m_arsize),
        .m_arburst(mem1_m_arburst), .m_arlock(mem1_m_arlock), .m_arcache(mem1_m_arcache),
        .m_arprot(mem1_m_arprot),
        .m_rvalid(mem1_m_rvalid), .m_rready(mem1_m_rready), .m_rid(mem1_m_rid),
        .m_rdata(mem1_m_rdata), .m_rresp(mem1_m_rresp), .m_rlast(mem1_m_rlast)
    );

    lbus_axi128_reg_slice_wrap #(
        .ID_WIDTH(MEM01_ID_W), .ADDR_WIDTH(ADDR_WIDTH), .DATA_WIDTH(DATA_WIDTH_128),
        .AW_SLICE_EN(SEXT_SLICE_EN), .W_SLICE_EN(SEXT_SLICE_EN), .B_SLICE_EN(SEXT_SLICE_EN),
        .AR_SLICE_EN(SEXT_SLICE_EN), .R_SLICE_EN(SEXT_SLICE_EN)
    ) u_sextmem1_rs (
        .aclk(aclk), .aresetn(aresetn),
        .awvalid_s(mem1_m_awvalid), .awready_s(mem1_m_awready),
        .awid_s(mem1_m_awid), .awaddr_s(mem1_m_awaddr), .awlen_s(mem1_m_awlen),
        .awsize_s(mem1_m_awsize), .awburst_s(mem1_m_awburst), .awlock_s(mem1_m_awlock),
        .awcache_s(mem1_m_awcache), .awprot_s(mem1_m_awprot),
        .awvalid_m(sextmem1_awvalid), .awready_m(sextmem1_awready),
        .awid_m(sextmem1_awid), .awaddr_m(sextmem1_awaddr), .awlen_m(sextmem1_awlen),
        .awsize_m(sextmem1_awsize), .awburst_m(sextmem1_awburst), .awlock_m(sextmem1_awlock),
        .awcache_m(sextmem1_awcache), .awprot_m(sextmem1_awprot),
        .wvalid_s(mem1_m_wvalid), .wready_s(mem1_m_wready),
        .wid_s(mem1_m_wid), .wdata_s(mem1_m_wdata), .wstrb_s(mem1_m_wstrb), .wlast_s(mem1_m_wlast),
        .wvalid_m(sextmem1_wvalid), .wready_m(sextmem1_wready),
        .wid_m(sextmem1_wid), .wdata_m(sextmem1_wdata), .wstrb_m(sextmem1_wstrb),
        .wlast_m(sextmem1_wlast),
        .bvalid_s(sextmem1_bvalid), .bready_s(sextmem1_bready),
        .bid_s(sextmem1_bid), .bresp_s(sextmem1_bresp),
        .bvalid_m(mem1_m_bvalid), .bready_m(mem1_m_bready),
        .bid_m(mem1_m_bid), .bresp_m(mem1_m_bresp),
        .arvalid_s(mem1_m_arvalid), .arready_s(mem1_m_arready),
        .arid_s(mem1_m_arid), .araddr_s(mem1_m_araddr), .arlen_s(mem1_m_arlen),
        .arsize_s(mem1_m_arsize), .arburst_s(mem1_m_arburst), .arlock_s(mem1_m_arlock),
        .arcache_s(mem1_m_arcache), .arprot_s(mem1_m_arprot),
        .arvalid_m(sextmem1_arvalid), .arready_m(sextmem1_arready),
        .arid_m(sextmem1_arid), .araddr_m(sextmem1_araddr), .arlen_m(sextmem1_arlen),
        .arsize_m(sextmem1_arsize), .arburst_m(sextmem1_arburst), .arlock_m(sextmem1_arlock),
        .arcache_m(sextmem1_arcache), .arprot_m(sextmem1_arprot),
        .rvalid_s(sextmem1_rvalid), .rready_s(sextmem1_rready),
        .rid_s(sextmem1_rid), .rdata_s(sextmem1_rdata), .rresp_s(sextmem1_rresp),
        .rlast_s(sextmem1_rlast),
        .rvalid_m(mem1_m_rvalid), .rready_m(mem1_m_rready),
        .rid_m(mem1_m_rid), .rdata_m(mem1_m_rdata), .rresp_m(mem1_m_rresp),
        .rlast_m(mem1_m_rlast)
    );

    // msrp8 bypass -> sextmem2
    lbus_axi128_reg_slice_wrap #(
        .ID_WIDTH(MSR_ID_WIDTH), .ADDR_WIDTH(ADDR_WIDTH), .DATA_WIDTH(DATA_WIDTH_128),
        .AW_SLICE_EN(SEXT_SLICE_EN), .W_SLICE_EN(SEXT_SLICE_EN), .B_SLICE_EN(SEXT_SLICE_EN),
        .AR_SLICE_EN(SEXT_SLICE_EN), .R_SLICE_EN(SEXT_SLICE_EN)
    ) u_sextmem2_rs (
        .aclk(aclk), .aresetn(aresetn),
        .awvalid_s(rt0_awvalid[8]), .awready_s(rt0_awready[8]),
        .awid_s(rt0_awid[8*MSR_ID_WIDTH-1:7*MSR_ID_WIDTH]),
        .awaddr_s(rt0_awaddr[8*ADDR_WIDTH-1:7*ADDR_WIDTH]),
        .awlen_s(rt0_awlen[8*4-1:7*4]), .awsize_s(rt0_awsize[8*3-1:7*3]),
        .awburst_s(rt0_awburst[8*2-1:7*2]), .awlock_s(rt0_awlock[8*2-1:7*2]),
        .awcache_s(rt0_awcache[8*4-1:7*4]), .awprot_s(rt0_awprot[8*3-1:7*3]),
        .awvalid_m(sextmem2_awvalid), .awready_m(sextmem2_awready),
        .awid_m(sextmem2_awid), .awaddr_m(sextmem2_awaddr), .awlen_m(sextmem2_awlen),
        .awsize_m(sextmem2_awsize), .awburst_m(sextmem2_awburst), .awlock_m(sextmem2_awlock),
        .awcache_m(sextmem2_awcache), .awprot_m(sextmem2_awprot),
        .wvalid_s(rt0_wvalid[8]), .wready_s(rt0_wready[8]),
        .wid_s(rt0_wid[8*MSR_ID_WIDTH-1:7*MSR_ID_WIDTH]),
        .wdata_s(rt0_wdata[8*DATA_WIDTH_128-1:7*DATA_WIDTH_128]),
        .wstrb_s(rt0_wstrb[8*STRB_W128-1:7*STRB_W128]), .wlast_s(rt0_wlast[8]),
        .wvalid_m(sextmem2_wvalid), .wready_m(sextmem2_wready),
        .wid_m(sextmem2_wid), .wdata_m(sextmem2_wdata), .wstrb_m(sextmem2_wstrb),
        .wlast_m(sextmem2_wlast),
        .bvalid_s(sextmem2_bvalid), .bready_s(sextmem2_bready),
        .bid_s(sextmem2_bid), .bresp_s(sextmem2_bresp),
        .bvalid_m(rt0_bvalid[8]), .bready_m(rt0_bready[8]),
        .bid_m(rt0_bid[8*MSR_ID_WIDTH-1:7*MSR_ID_WIDTH]),
        .bresp_m(rt0_bresp[8*2-1:7*2]),
        .arvalid_s(rt0_arvalid[8]), .arready_s(rt0_arready[8]),
        .arid_s(rt0_arid[8*MSR_ID_WIDTH-1:7*MSR_ID_WIDTH]),
        .araddr_s(rt0_araddr[8*ADDR_WIDTH-1:7*ADDR_WIDTH]),
        .arlen_s(rt0_arlen[8*4-1:7*4]), .arsize_s(rt0_arsize[8*3-1:7*3]),
        .arburst_s(rt0_arburst[8*2-1:7*2]), .arlock_s(rt0_arlock[8*2-1:7*2]),
        .arcache_s(rt0_arcache[8*4-1:7*4]), .arprot_s(rt0_arprot[8*3-1:7*3]),
        .arvalid_m(sextmem2_arvalid), .arready_m(sextmem2_arready),
        .arid_m(sextmem2_arid), .araddr_m(sextmem2_araddr), .arlen_m(sextmem2_arlen),
        .arsize_m(sextmem2_arsize), .arburst_m(sextmem2_arburst), .arlock_m(sextmem2_arlock),
        .arcache_m(sextmem2_arcache), .arprot_m(sextmem2_arprot),
        .rvalid_s(sextmem2_rvalid), .rready_s(sextmem2_rready),
        .rid_s(sextmem2_rid), .rdata_s(sextmem2_rdata), .rresp_s(sextmem2_rresp),
        .rlast_s(sextmem2_rlast),
        .rvalid_m(rt0_rvalid[8]), .rready_m(rt0_rready[8]),
        .rid_m(rt0_rid[8*MSR_ID_WIDTH-1:7*MSR_ID_WIDTH]),
        .rdata_m(rt0_rdata[8*DATA_WIDTH_128-1:7*DATA_WIDTH_128]),
        .rresp_m(rt0_rresp[8*2-1:7*2]), .rlast_m(rt0_rlast[8])
    );

    // Target1: msrp0~8 -> sextio0
    wire mem_io_m_awvalid, mem_io_m_wvalid, mem_io_m_bvalid, mem_io_m_arvalid, mem_io_m_rvalid;
    wire mem_io_m_awready, mem_io_m_wready, mem_io_m_bready, mem_io_m_arready, mem_io_m_rready;
    wire [IO_ID_W-1:0] mem_io_m_awid, mem_io_m_wid, mem_io_m_bid, mem_io_m_arid, mem_io_m_rid;
    wire [ADDR_WIDTH-1:0] mem_io_m_awaddr, mem_io_m_araddr;
    wire [3:0] mem_io_m_awlen, mem_io_m_arlen;
    wire [2:0] mem_io_m_awsize, mem_io_m_arsize;
    wire [1:0] mem_io_m_awburst, mem_io_m_arburst, mem_io_m_awlock, mem_io_m_arlock;
    wire [3:0] mem_io_m_awcache, mem_io_m_arcache;
    wire [2:0] mem_io_m_awprot, mem_io_m_arprot;
    wire [DATA_WIDTH_128-1:0] mem_io_m_wdata, mem_io_m_rdata;
    wire [STRB_W128-1:0] mem_io_m_wstrb;
    wire mem_io_m_wlast, mem_io_m_rlast;
    wire [1:0] mem_io_m_bresp, mem_io_m_rresp;

    axi3_merge_Nto1_128 #(
        .N(9), .IN_ID_WIDTH(MSR_ID_WIDTH), .SRC_ID_WIDTH(4)
    ) u_merge_io (
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
        .ID_WIDTH(IO_ID_W), .ADDR_WIDTH(ADDR_WIDTH), .DATA_WIDTH(DATA_WIDTH_128),
        .AW_SLICE_EN(SEXT_SLICE_EN), .W_SLICE_EN(SEXT_SLICE_EN), .B_SLICE_EN(SEXT_SLICE_EN),
        .AR_SLICE_EN(SEXT_SLICE_EN), .R_SLICE_EN(SEXT_SLICE_EN)
    ) u_sextio0_rs (
        .aclk(aclk), .aresetn(aresetn),
        .awvalid_s(mem_io_m_awvalid), .awready_s(mem_io_m_awready),
        .awid_s(mem_io_m_awid), .awaddr_s(mem_io_m_awaddr), .awlen_s(mem_io_m_awlen),
        .awsize_s(mem_io_m_awsize), .awburst_s(mem_io_m_awburst), .awlock_s(mem_io_m_awlock),
        .awcache_s(mem_io_m_awcache), .awprot_s(mem_io_m_awprot),
        .awvalid_m(sextio0_awvalid), .awready_m(sextio0_awready),
        .awid_m(sextio0_awid), .awaddr_m(sextio0_awaddr), .awlen_m(sextio0_awlen),
        .awsize_m(sextio0_awsize), .awburst_m(sextio0_awburst), .awlock_m(sextio0_awlock),
        .awcache_m(sextio0_awcache), .awprot_m(sextio0_awprot),
        .wvalid_s(mem_io_m_wvalid), .wready_s(mem_io_m_wready),
        .wid_s(mem_io_m_wid), .wdata_s(mem_io_m_wdata), .wstrb_s(mem_io_m_wstrb),
        .wlast_s(mem_io_m_wlast),
        .wvalid_m(sextio0_wvalid), .wready_m(sextio0_wready),
        .wid_m(sextio0_wid), .wdata_m(sextio0_wdata), .wstrb_m(sextio0_wstrb),
        .wlast_m(sextio0_wlast),
        .bvalid_s(sextio0_bvalid), .bready_s(sextio0_bready),
        .bid_s(sextio0_bid), .bresp_s(sextio0_bresp),
        .bvalid_m(mem_io_m_bvalid), .bready_m(mem_io_m_bready),
        .bid_m(mem_io_m_bid), .bresp_m(mem_io_m_bresp),
        .arvalid_s(mem_io_m_arvalid), .arready_s(mem_io_m_arready),
        .arid_s(mem_io_m_arid), .araddr_s(mem_io_m_araddr), .arlen_s(mem_io_m_arlen),
        .arsize_s(mem_io_m_arsize), .arburst_s(mem_io_m_arburst), .arlock_s(mem_io_m_arlock),
        .arcache_s(mem_io_m_arcache), .arprot_s(mem_io_m_arprot),
        .arvalid_m(sextio0_arvalid), .arready_m(sextio0_arready),
        .arid_m(sextio0_arid), .araddr_m(sextio0_araddr), .arlen_m(sextio0_arlen),
        .arsize_m(sextio0_arsize), .arburst_m(sextio0_arburst), .arlock_m(sextio0_arlock),
        .arcache_m(sextio0_arcache), .arprot_m(sextio0_arprot),
        .rvalid_s(sextio0_rvalid), .rready_s(sextio0_rready),
        .rid_s(sextio0_rid), .rdata_s(sextio0_rdata), .rresp_s(sextio0_rresp),
        .rlast_s(sextio0_rlast),
        .rvalid_m(mem_io_m_rvalid), .rready_m(mem_io_m_rready),
        .rid_m(mem_io_m_rid), .rdata_m(mem_io_m_rdata), .rresp_m(mem_io_m_rresp),
        .rlast_m(mem_io_m_rlast)
    );

    // mext0 -> reg slice -> router/AHB -> ssrp0~8
    wire mext_rt_awvalid, mext_rt_wvalid, mext_rt_bvalid, mext_rt_arvalid, mext_rt_rvalid;
    wire mext_rt_awready, mext_rt_wready, mext_rt_bready, mext_rt_arready, mext_rt_rready;
    wire [MEXT_ID_WIDTH-1:0] mext_rt_awid, mext_rt_wid, mext_rt_bid, mext_rt_arid, mext_rt_rid;
    wire [ADDR_WIDTH-1:0] mext_rt_awaddr, mext_rt_araddr;
    wire [3:0] mext_rt_awlen, mext_rt_arlen;
    wire [2:0] mext_rt_awsize, mext_rt_arsize;
    wire [1:0] mext_rt_awburst, mext_rt_arburst, mext_rt_awlock, mext_rt_arlock;
    wire [3:0] mext_rt_awcache, mext_rt_arcache;
    wire [2:0] mext_rt_awprot, mext_rt_arprot;
    wire [DATA_WIDTH_32-1:0] mext_rt_wdata, mext_rt_rdata;
    wire [STRB_W32-1:0] mext_rt_wstrb;
    wire mext_rt_wlast, mext_rt_rlast;
    wire [1:0] mext_rt_bresp, mext_rt_rresp;

    lbus_axi32_reg_slice_wrap #(
        .ID_WIDTH(MEXT_ID_WIDTH), .ADDR_WIDTH(ADDR_WIDTH), .DATA_WIDTH(DATA_WIDTH_32),
        .AW_SLICE_EN(MEXT_SLICE_EN), .W_SLICE_EN(MEXT_SLICE_EN), .B_SLICE_EN(MEXT_SLICE_EN),
        .AR_SLICE_EN(MEXT_SLICE_EN), .R_SLICE_EN(MEXT_SLICE_EN)
    ) u_mext0_rs (
        .aclk(aclk), .aresetn(aresetn),
        .awvalid_s(mext0_awvalid), .awready_s(mext0_awready),
        .awid_s(mext0_awid), .awaddr_s(mext0_awaddr), .awlen_s(mext0_awlen),
        .awsize_s(mext0_awsize), .awburst_s(mext0_awburst), .awlock_s(mext0_awlock),
        .awcache_s(mext0_awcache), .awprot_s(mext0_awprot),
        .awvalid_m(mext_rt_awvalid), .awready_m(mext_rt_awready),
        .awid_m(mext_rt_awid), .awaddr_m(mext_rt_awaddr), .awlen_m(mext_rt_awlen),
        .awsize_m(mext_rt_awsize), .awburst_m(mext_rt_awburst), .awlock_m(mext_rt_awlock),
        .awcache_m(mext_rt_awcache), .awprot_m(mext_rt_awprot),
        .wvalid_s(mext0_wvalid), .wready_s(mext0_wready),
        .wid_s(mext0_wid), .wdata_s(mext0_wdata), .wstrb_s(mext0_wstrb), .wlast_s(mext0_wlast),
        .wvalid_m(mext_rt_wvalid), .wready_m(mext_rt_wready),
        .wid_m(mext_rt_wid), .wdata_m(mext_rt_wdata), .wstrb_m(mext_rt_wstrb),
        .wlast_m(mext_rt_wlast),
        .bvalid_s(mext_rt_bvalid), .bready_s(mext_rt_bready),
        .bid_s(mext_rt_bid), .bresp_s(mext_rt_bresp),
        .bvalid_m(mext0_bvalid), .bready_m(mext0_bready),
        .bid_m(mext0_bid), .bresp_m(mext0_bresp),
        .arvalid_s(mext0_arvalid), .arready_s(mext0_arready),
        .arid_s(mext0_arid), .araddr_s(mext0_araddr), .arlen_s(mext0_arlen),
        .arsize_s(mext0_arsize), .arburst_s(mext0_arburst), .arlock_s(mext0_arlock),
        .arcache_s(mext0_arcache), .arprot_s(mext0_arprot),
        .arvalid_m(mext_rt_arvalid), .arready_m(mext_rt_arready),
        .arid_m(mext_rt_arid), .araddr_m(mext_rt_araddr), .arlen_m(mext_rt_arlen),
        .arsize_m(mext_rt_arsize), .arburst_m(mext_rt_arburst), .arlock_m(mext_rt_arlock),
        .arcache_m(mext_rt_arcache), .arprot_m(mext_rt_arprot),
        .rvalid_s(mext_rt_rvalid), .rready_s(mext_rt_rready),
        .rid_s(mext_rt_rid), .rdata_s(mext_rt_rdata), .rresp_s(mext_rt_rresp),
        .rlast_s(mext_rt_rlast),
        .rvalid_m(mext0_rvalid), .rready_m(mext0_rready),
        .rid_m(mext0_rid), .rdata_m(mext0_rdata), .rresp_m(mext0_rresp),
        .rlast_m(mext0_rlast)
    );

    axi3_router_1toN_ahblite #(
        .N(9),
        .ADDR_WIDTH(ADDR_WIDTH),
        .HADDR_LOW_BITS(HADDR_LOW_BITS),
        .DATA_WIDTH(DATA_WIDTH_32),
        .STRB_WIDTH(STRB_W32),
        .ID_WIDTH(MEXT_ID_WIDTH),
        .ROUTER_OUTSTANDING(ROUTER_OUTSTANDING),
        .WR_CMD_DEPTH(WR_CMD_DEPTH),
        .RD_CMD_DEPTH(RD_CMD_DEPTH),
        .RESP_DEPTH(RESP_DEPTH),
        .BUSY_ENABLE(BUSY_ENABLE)
    ) u_mext_router_ahb (
        .aclk(aclk), .aresetn(aresetn),
        .aw_sel(mext0_aw_sel), .ar_sel(mext0_ar_sel),
        .s_awvalid(mext_rt_awvalid), .s_awready(mext_rt_awready),
        .s_awid(mext_rt_awid), .s_awaddr(mext_rt_awaddr), .s_awlen(mext_rt_awlen),
        .s_awsize(mext_rt_awsize), .s_awburst(mext_rt_awburst), .s_awlock(mext_rt_awlock),
        .s_awcache(mext_rt_awcache), .s_awprot(mext_rt_awprot),
        .s_wvalid(mext_rt_wvalid), .s_wready(mext_rt_wready),
        .s_wid(mext_rt_wid), .s_wdata(mext_rt_wdata), .s_wstrb(mext_rt_wstrb),
        .s_wlast(mext_rt_wlast),
        .s_bvalid(mext_rt_bvalid), .s_bready(mext_rt_bready),
        .s_bid(mext_rt_bid), .s_bresp(mext_rt_bresp),
        .s_arvalid(mext_rt_arvalid), .s_arready(mext_rt_arready),
        .s_arid(mext_rt_arid), .s_araddr(mext_rt_araddr), .s_arlen(mext_rt_arlen),
        .s_arsize(mext_rt_arsize), .s_arburst(mext_rt_arburst), .s_arlock(mext_rt_arlock),
        .s_arcache(mext_rt_arcache), .s_arprot(mext_rt_arprot),
        .s_rvalid(mext_rt_rvalid), .s_rready(mext_rt_rready),
        .s_rid(mext_rt_rid), .s_rdata(mext_rt_rdata), .s_rresp(mext_rt_rresp),
        .s_rlast(mext_rt_rlast),
        .haddr(ssrp_haddr), .htrans(ssrp_htrans), .hwrite(ssrp_hwrite),
        .hsize(ssrp_hsize), .hburst(ssrp_hburst), .hprot(ssrp_hprot),
        .hwdata(ssrp_hwdata), .hrdata(ssrp_hrdata),
        .hready(ssrp_hready), .hresp(ssrp_hresp)
    );

endmodule
