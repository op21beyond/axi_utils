// -----------------------------------------------------------------------------
// Module      : lbus_plt
// Date        : 2026-05-28
// Version     : v1.0.0
// Author      : Jongchul Shin
// Function    : Platform local bus top with two independent paths:
//               mext0 (AXI3 slave) -> sreg0..(NUM_SREG-1) AHB-Lite
//               mext1 (AXI3 slave) -> smem0..(NUM_SMEM-1) AHB-Lite
//               Each path: register slice + axi3_router_1toN + N/M bridges.
// Assumptions : aw_sel/ar_sel are one-hot (width NUM_SREG / NUM_SMEM).
// Notes       : NUM_SREG and NUM_SMEM are independent.
// -----------------------------------------------------------------------------
module lbus_plt #(
    parameter integer NUM_SREG           = 8,              // Number of sreg AHB ports
    parameter integer NUM_SMEM           = 4,              // Number of smem AHB ports
    parameter integer MEXT_ID_WIDTH      = 4,              // mext AXI ID width
    parameter integer ADDR_WIDTH         = 32,             // Address width
    parameter integer DATA_WIDTH         = 32,             // mext AXI data width
    parameter integer HADDR_LOW_BITS     = 32,             // AHB haddr output mask width
    parameter integer ROUTER_OUTSTANDING = 16,           // Router ordering FIFO depth
    parameter integer WR_CMD_DEPTH       = 16,             // Bridge write command depth
    parameter integer RD_CMD_DEPTH       = 16,             // Bridge read command depth
    parameter integer RESP_DEPTH         = 8,              // Bridge response depth
    parameter         BUSY_ENABLE        = 1'b1,           // Bridge BUSY enable
    parameter         MEXT0_SLICE_EN     = 1'b1,           // Register slice on mext0
    parameter         MEXT1_SLICE_EN     = 1'b1            // Register slice on mext1
) (
    input  wire                         aclk,
    input  wire                         aresetn,

    input  wire                         mext0_awvalid,
    output wire                         mext0_awready,
    input  wire [MEXT_ID_WIDTH-1:0]     mext0_awid,
    input  wire [ADDR_WIDTH-1:0]        mext0_awaddr,
    input  wire [3:0]                   mext0_awlen,
    input  wire [2:0]                   mext0_awsize,
    input  wire [1:0]                   mext0_awburst,
    input  wire [1:0]                   mext0_awlock,
    input  wire [3:0]                   mext0_awcache,
    input  wire [2:0]                   mext0_awprot,
    input  wire [NUM_SREG-1:0]          mext0_aw_sel,

    input  wire                         mext0_wvalid,
    output wire                         mext0_wready,
    input  wire [MEXT_ID_WIDTH-1:0]     mext0_wid,
    input  wire [DATA_WIDTH-1:0]        mext0_wdata,
    input  wire [DATA_WIDTH/8-1:0]    mext0_wstrb,
    input  wire                         mext0_wlast,

    output wire                         mext0_bvalid,
    input  wire                         mext0_bready,
    output wire [MEXT_ID_WIDTH-1:0]     mext0_bid,
    output wire [1:0]                   mext0_bresp,

    input  wire                         mext0_arvalid,
    output wire                         mext0_arready,
    input  wire [MEXT_ID_WIDTH-1:0]     mext0_arid,
    input  wire [ADDR_WIDTH-1:0]        mext0_araddr,
    input  wire [3:0]                   mext0_arlen,
    input  wire [2:0]                   mext0_arsize,
    input  wire [1:0]                   mext0_arburst,
    input  wire [1:0]                   mext0_arlock,
    input  wire [3:0]                   mext0_arcache,
    input  wire [2:0]                   mext0_arprot,
    input  wire [NUM_SREG-1:0]          mext0_ar_sel,

    output wire                         mext0_rvalid,
    input  wire                         mext0_rready,
    output wire [MEXT_ID_WIDTH-1:0]     mext0_rid,
    output wire [DATA_WIDTH-1:0]        mext0_rdata,
    output wire [1:0]                   mext0_rresp,
    output wire                         mext0_rlast,

    output wire [NUM_SREG*ADDR_WIDTH-1:0] sreg_haddr,
    output wire [NUM_SREG*2-1:0]          sreg_htrans,
    output wire [NUM_SREG-1:0]            sreg_hwrite,
    output wire [NUM_SREG*3-1:0]          sreg_hsize,
    output wire [NUM_SREG*3-1:0]          sreg_hburst,
    output wire [NUM_SREG*4-1:0]          sreg_hprot,
    output wire [NUM_SREG*DATA_WIDTH-1:0] sreg_hwdata,
    input  wire [NUM_SREG*DATA_WIDTH-1:0] sreg_hrdata,
    input  wire [NUM_SREG-1:0]            sreg_hready,
    input  wire [NUM_SREG-1:0]            sreg_hresp,

    input  wire                         mext1_awvalid,
    output wire                         mext1_awready,
    input  wire [MEXT_ID_WIDTH-1:0]     mext1_awid,
    input  wire [ADDR_WIDTH-1:0]        mext1_awaddr,
    input  wire [3:0]                   mext1_awlen,
    input  wire [2:0]                   mext1_awsize,
    input  wire [1:0]                   mext1_awburst,
    input  wire [1:0]                   mext1_awlock,
    input  wire [3:0]                   mext1_awcache,
    input  wire [2:0]                   mext1_awprot,
    input  wire [NUM_SMEM-1:0]          mext1_aw_sel,

    input  wire                         mext1_wvalid,
    output wire                         mext1_wready,
    input  wire [MEXT_ID_WIDTH-1:0]     mext1_wid,
    input  wire [DATA_WIDTH-1:0]        mext1_wdata,
    input  wire [DATA_WIDTH/8-1:0]    mext1_wstrb,
    input  wire                         mext1_wlast,

    output wire                         mext1_bvalid,
    input  wire                         mext1_bready,
    output wire [MEXT_ID_WIDTH-1:0]     mext1_bid,
    output wire [1:0]                   mext1_bresp,

    input  wire                         mext1_arvalid,
    output wire                         mext1_arready,
    input  wire [MEXT_ID_WIDTH-1:0]     mext1_arid,
    input  wire [ADDR_WIDTH-1:0]        mext1_araddr,
    input  wire [3:0]                   mext1_arlen,
    input  wire [2:0]                   mext1_arsize,
    input  wire [1:0]                   mext1_arburst,
    input  wire [1:0]                   mext1_arlock,
    input  wire [3:0]                   mext1_arcache,
    input  wire [2:0]                   mext1_arprot,
    input  wire [NUM_SMEM-1:0]          mext1_ar_sel,

    output wire                         mext1_rvalid,
    input  wire                         mext1_rready,
    output wire [MEXT_ID_WIDTH-1:0]     mext1_rid,
    output wire [DATA_WIDTH-1:0]        mext1_rdata,
    output wire [1:0]                   mext1_rresp,
    output wire                         mext1_rlast,

    output wire [NUM_SMEM*ADDR_WIDTH-1:0] smem_haddr,
    output wire [NUM_SMEM*2-1:0]          smem_htrans,
    output wire [NUM_SMEM-1:0]            smem_hwrite,
    output wire [NUM_SMEM*3-1:0]          smem_hsize,
    output wire [NUM_SMEM*3-1:0]          smem_hburst,
    output wire [NUM_SMEM*4-1:0]          smem_hprot,
    output wire [NUM_SMEM*DATA_WIDTH-1:0] smem_hwdata,
    input  wire [NUM_SMEM*DATA_WIDTH-1:0] smem_hrdata,
    input  wire [NUM_SMEM-1:0]            smem_hready,
    input  wire [NUM_SMEM-1:0]            smem_hresp
);

    localparam integer STRB_W = DATA_WIDTH / 8;

    wire                         rt0_awvalid, rt0_wvalid, rt0_bvalid, rt0_arvalid, rt0_rvalid;
    wire                         rt0_awready, rt0_wready, rt0_bready, rt0_arready, rt0_rready;
    wire [MEXT_ID_WIDTH-1:0]     rt0_awid, rt0_wid, rt0_bid, rt0_arid, rt0_rid;
    wire [ADDR_WIDTH-1:0]        rt0_awaddr, rt0_araddr;
    wire [3:0]                   rt0_awlen, rt0_arlen;
    wire [2:0]                   rt0_awsize, rt0_arsize;
    wire [1:0]                   rt0_awburst, rt0_arburst, rt0_awlock, rt0_arlock;
    wire [3:0]                   rt0_awcache, rt0_arcache;
    wire [2:0]                   rt0_awprot, rt0_arprot;
    wire [DATA_WIDTH-1:0]        rt0_wdata, rt0_rdata;
    wire [STRB_W-1:0]            rt0_wstrb;
    wire                         rt0_wlast, rt0_rlast;
    wire [1:0]                   rt0_bresp, rt0_rresp;

    wire                         rt1_awvalid, rt1_wvalid, rt1_bvalid, rt1_arvalid, rt1_rvalid;
    wire                         rt1_awready, rt1_wready, rt1_bready, rt1_arready, rt1_rready;
    wire [MEXT_ID_WIDTH-1:0]     rt1_awid, rt1_wid, rt1_bid, rt1_arid, rt1_rid;
    wire [ADDR_WIDTH-1:0]        rt1_awaddr, rt1_araddr;
    wire [3:0]                   rt1_awlen, rt1_arlen;
    wire [2:0]                   rt1_awsize, rt1_arsize;
    wire [1:0]                   rt1_awburst, rt1_arburst, rt1_awlock, rt1_arlock;
    wire [3:0]                   rt1_awcache, rt1_arcache;
    wire [2:0]                   rt1_awprot, rt1_arprot;
    wire [DATA_WIDTH-1:0]        rt1_wdata, rt1_rdata;
    wire [STRB_W-1:0]            rt1_wstrb;
    wire                         rt1_wlast, rt1_rlast;
    wire [1:0]                   rt1_bresp, rt1_rresp;

    lbus_axi32_reg_slice_wrap #(
        .ID_WIDTH(MEXT_ID_WIDTH), .ADDR_WIDTH(ADDR_WIDTH), .DATA_WIDTH(DATA_WIDTH),
        .AW_SLICE_EN(MEXT0_SLICE_EN), .W_SLICE_EN(MEXT0_SLICE_EN),
        .B_SLICE_EN(MEXT0_SLICE_EN), .AR_SLICE_EN(MEXT0_SLICE_EN),
        .R_SLICE_EN(MEXT0_SLICE_EN)
    ) u_mext0_rs (
        .aclk(aclk), .aresetn(aresetn),
        .awvalid_s(mext0_awvalid), .awready_s(mext0_awready),
        .awid_s(mext0_awid), .awaddr_s(mext0_awaddr), .awlen_s(mext0_awlen),
        .awsize_s(mext0_awsize), .awburst_s(mext0_awburst), .awlock_s(mext0_awlock),
        .awcache_s(mext0_awcache), .awprot_s(mext0_awprot),
        .awvalid_m(rt0_awvalid), .awready_m(rt0_awready),
        .awid_m(rt0_awid), .awaddr_m(rt0_awaddr), .awlen_m(rt0_awlen),
        .awsize_m(rt0_awsize), .awburst_m(rt0_awburst), .awlock_m(rt0_awlock),
        .awcache_m(rt0_awcache), .awprot_m(rt0_awprot),
        .wvalid_s(mext0_wvalid), .wready_s(mext0_wready),
        .wid_s(mext0_wid), .wdata_s(mext0_wdata), .wstrb_s(mext0_wstrb), .wlast_s(mext0_wlast),
        .wvalid_m(rt0_wvalid), .wready_m(rt0_wready),
        .wid_m(rt0_wid), .wdata_m(rt0_wdata), .wstrb_m(rt0_wstrb), .wlast_m(rt0_wlast),
        .bvalid_s(rt0_bvalid), .bready_s(rt0_bready),
        .bid_s(rt0_bid), .bresp_s(rt0_bresp),
        .bvalid_m(mext0_bvalid), .bready_m(mext0_bready),
        .bid_m(mext0_bid), .bresp_m(mext0_bresp),
        .arvalid_s(mext0_arvalid), .arready_s(mext0_arready),
        .arid_s(mext0_arid), .araddr_s(mext0_araddr), .arlen_s(mext0_arlen),
        .arsize_s(mext0_arsize), .arburst_s(mext0_arburst), .arlock_s(mext0_arlock),
        .arcache_s(mext0_arcache), .arprot_s(mext0_arprot),
        .arvalid_m(rt0_arvalid), .arready_m(rt0_arready),
        .arid_m(rt0_arid), .araddr_m(rt0_araddr), .arlen_m(rt0_arlen),
        .arsize_m(rt0_arsize), .arburst_m(rt0_arburst), .arlock_m(rt0_arlock),
        .arcache_m(rt0_arcache), .arprot_m(rt0_arprot),
        .rvalid_s(rt0_rvalid), .rready_s(rt0_rready),
        .rid_s(rt0_rid), .rdata_s(rt0_rdata), .rresp_s(rt0_rresp), .rlast_s(rt0_rlast),
        .rvalid_m(mext0_rvalid), .rready_m(mext0_rready),
        .rid_m(mext0_rid), .rdata_m(mext0_rdata), .rresp_m(mext0_rresp), .rlast_m(mext0_rlast)
    );

    lbus_axi32_reg_slice_wrap #(
        .ID_WIDTH(MEXT_ID_WIDTH), .ADDR_WIDTH(ADDR_WIDTH), .DATA_WIDTH(DATA_WIDTH),
        .AW_SLICE_EN(MEXT1_SLICE_EN), .W_SLICE_EN(MEXT1_SLICE_EN),
        .B_SLICE_EN(MEXT1_SLICE_EN), .AR_SLICE_EN(MEXT1_SLICE_EN),
        .R_SLICE_EN(MEXT1_SLICE_EN)
    ) u_mext1_rs (
        .aclk(aclk), .aresetn(aresetn),
        .awvalid_s(mext1_awvalid), .awready_s(mext1_awready),
        .awid_s(mext1_awid), .awaddr_s(mext1_awaddr), .awlen_s(mext1_awlen),
        .awsize_s(mext1_awsize), .awburst_s(mext1_awburst), .awlock_s(mext1_awlock),
        .awcache_s(mext1_awcache), .awprot_s(mext1_awprot),
        .awvalid_m(rt1_awvalid), .awready_m(rt1_awready),
        .awid_m(rt1_awid), .awaddr_m(rt1_awaddr), .awlen_m(rt1_awlen),
        .awsize_m(rt1_awsize), .awburst_m(rt1_awburst), .awlock_m(rt1_awlock),
        .awcache_m(rt1_awcache), .awprot_m(rt1_awprot),
        .wvalid_s(mext1_wvalid), .wready_s(mext1_wready),
        .wid_s(mext1_wid), .wdata_s(mext1_wdata), .wstrb_s(mext1_wstrb), .wlast_s(mext1_wlast),
        .wvalid_m(rt1_wvalid), .wready_m(rt1_wready),
        .wid_m(rt1_wid), .wdata_m(rt1_wdata), .wstrb_m(rt1_wstrb), .wlast_m(rt1_wlast),
        .bvalid_s(rt1_bvalid), .bready_s(rt1_bready),
        .bid_s(rt1_bid), .bresp_s(rt1_bresp),
        .bvalid_m(mext1_bvalid), .bready_m(mext1_bready),
        .bid_m(mext1_bid), .bresp_m(mext1_bresp),
        .arvalid_s(mext1_arvalid), .arready_s(mext1_arready),
        .arid_s(mext1_arid), .araddr_s(mext1_araddr), .arlen_s(mext1_arlen),
        .arsize_s(mext1_arsize), .arburst_s(mext1_arburst), .arlock_s(mext1_arlock),
        .arcache_s(mext1_arcache), .arprot_s(mext1_arprot),
        .arvalid_m(rt1_arvalid), .arready_m(rt1_arready),
        .arid_m(rt1_arid), .araddr_m(rt1_araddr), .arlen_m(rt1_arlen),
        .arsize_m(rt1_arsize), .arburst_m(rt1_arburst), .arlock_m(rt1_arlock),
        .arcache_m(rt1_arcache), .arprot_m(rt1_arprot),
        .rvalid_s(rt1_rvalid), .rready_s(rt1_rready),
        .rid_s(rt1_rid), .rdata_s(rt1_rdata), .rresp_s(rt1_rresp), .rlast_s(rt1_rlast),
        .rvalid_m(mext1_rvalid), .rready_m(mext1_rready),
        .rid_m(mext1_rid), .rdata_m(mext1_rdata), .rresp_m(mext1_rresp), .rlast_m(mext1_rlast)
    );

    axi3_router_1toN_ahblite #(
        .N(NUM_SREG),
        .ADDR_WIDTH(ADDR_WIDTH),
        .HADDR_LOW_BITS(HADDR_LOW_BITS),
        .DATA_WIDTH(DATA_WIDTH),
        .STRB_WIDTH(STRB_W),
        .ID_WIDTH(MEXT_ID_WIDTH),
        .ROUTER_OUTSTANDING(ROUTER_OUTSTANDING),
        .WR_CMD_DEPTH(WR_CMD_DEPTH),
        .RD_CMD_DEPTH(RD_CMD_DEPTH),
        .RESP_DEPTH(RESP_DEPTH),
        .BUSY_ENABLE(BUSY_ENABLE)
    ) u_mext0_router_ahb (
        .aclk(aclk), .aresetn(aresetn),
        .aw_sel(mext0_aw_sel), .ar_sel(mext0_ar_sel),
        .s_awvalid(rt0_awvalid), .s_awready(rt0_awready),
        .s_awid(rt0_awid), .s_awaddr(rt0_awaddr), .s_awlen(rt0_awlen),
        .s_awsize(rt0_awsize), .s_awburst(rt0_awburst), .s_awlock(rt0_awlock),
        .s_awcache(rt0_awcache), .s_awprot(rt0_awprot),
        .s_wvalid(rt0_wvalid), .s_wready(rt0_wready),
        .s_wid(rt0_wid), .s_wdata(rt0_wdata), .s_wstrb(rt0_wstrb), .s_wlast(rt0_wlast),
        .s_bvalid(rt0_bvalid), .s_bready(rt0_bready),
        .s_bid(rt0_bid), .s_bresp(rt0_bresp),
        .s_arvalid(rt0_arvalid), .s_arready(rt0_arready),
        .s_arid(rt0_arid), .s_araddr(rt0_araddr), .s_arlen(rt0_arlen),
        .s_arsize(rt0_arsize), .s_arburst(rt0_arburst), .s_arlock(rt0_arlock),
        .s_arcache(rt0_arcache), .s_arprot(rt0_arprot),
        .s_rvalid(rt0_rvalid), .s_rready(rt0_rready),
        .s_rid(rt0_rid), .s_rdata(rt0_rdata), .s_rresp(rt0_rresp), .s_rlast(rt0_rlast),
        .haddr(sreg_haddr), .htrans(sreg_htrans), .hwrite(sreg_hwrite),
        .hsize(sreg_hsize), .hburst(sreg_hburst), .hprot(sreg_hprot),
        .hwdata(sreg_hwdata), .hrdata(sreg_hrdata),
        .hready(sreg_hready), .hresp(sreg_hresp)
    );

    axi3_router_1toN_ahblite #(
        .N(NUM_SMEM),
        .ADDR_WIDTH(ADDR_WIDTH),
        .HADDR_LOW_BITS(HADDR_LOW_BITS),
        .DATA_WIDTH(DATA_WIDTH),
        .STRB_WIDTH(STRB_W),
        .ID_WIDTH(MEXT_ID_WIDTH),
        .ROUTER_OUTSTANDING(ROUTER_OUTSTANDING),
        .WR_CMD_DEPTH(WR_CMD_DEPTH),
        .RD_CMD_DEPTH(RD_CMD_DEPTH),
        .RESP_DEPTH(RESP_DEPTH),
        .BUSY_ENABLE(BUSY_ENABLE)
    ) u_mext1_router_ahb (
        .aclk(aclk), .aresetn(aresetn),
        .aw_sel(mext1_aw_sel), .ar_sel(mext1_ar_sel),
        .s_awvalid(rt1_awvalid), .s_awready(rt1_awready),
        .s_awid(rt1_awid), .s_awaddr(rt1_awaddr), .s_awlen(rt1_awlen),
        .s_awsize(rt1_awsize), .s_awburst(rt1_awburst), .s_awlock(rt1_awlock),
        .s_awcache(rt1_awcache), .s_awprot(rt1_awprot),
        .s_wvalid(rt1_wvalid), .s_wready(rt1_wready),
        .s_wid(rt1_wid), .s_wdata(rt1_wdata), .s_wstrb(rt1_wstrb), .s_wlast(rt1_wlast),
        .s_bvalid(rt1_bvalid), .s_bready(rt1_bready),
        .s_bid(rt1_bid), .s_bresp(rt1_bresp),
        .s_arvalid(rt1_arvalid), .s_arready(rt1_arready),
        .s_arid(rt1_arid), .s_araddr(rt1_araddr), .s_arlen(rt1_arlen),
        .s_arsize(rt1_arsize), .s_arburst(rt1_arburst), .s_arlock(rt1_arlock),
        .s_arcache(rt1_arcache), .s_arprot(rt1_arprot),
        .s_rvalid(rt1_rvalid), .s_rready(rt1_rready),
        .s_rid(rt1_rid), .s_rdata(rt1_rdata), .s_rresp(rt1_rresp), .s_rlast(rt1_rlast),
        .haddr(smem_haddr), .htrans(smem_htrans), .hwrite(smem_hwrite),
        .hsize(smem_hsize), .hburst(smem_hburst), .hprot(smem_hprot),
        .hwdata(smem_hwdata), .hrdata(smem_hrdata),
        .hready(smem_hready), .hresp(smem_hresp)
    );

endmodule
