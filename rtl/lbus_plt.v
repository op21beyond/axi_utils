// -----------------------------------------------------------------------------
// Module      : lbus_plt
// Date        : 2026-05-28
// Version     : v1.1.0
// Author      : Jongchul Shin
// Function    : Platform local bus top:
//               mext (AXI3 slave) -> per-channel axi3_reg_slice_ch ->
//               axi3_router_1toN_ahblite -> NUM_PORTS AHB-Lite outputs.
// Assumptions : mext AHB target select is decoded from mext AW/AR address.
// Notes       : Register slices are instantiated per AXI channel at this level.
//               Unused mext address bits are zeroed at input (addr_map) and again
//               at the router (addr_tgt) so downstream logic synthesizes smaller.
//               AHB outputs SINGLE transfers only (hburst=000; htrans=IDLE/NONSEQ).
// -----------------------------------------------------------------------------
module lbus_plt #(
    parameter integer NUM_PORTS          = 16,             // AHB-Lite fanout count
    parameter integer MEXT_ID_WIDTH      = 4,              // mext AXI ID width
    parameter integer ADDR_WIDTH         = 32,             // Address width
    parameter integer DATA_WIDTH         = 32,             // mext AXI / AHB data width
    parameter integer ROUTER_OUTSTANDING = 16,             // Router ordering FIFO depth
    parameter integer WR_CMD_DEPTH       = 16,             // Bridge write command depth
    parameter integer RD_CMD_DEPTH       = 16,             // Bridge read command depth
    parameter integer RESP_DEPTH         = 8,              // Bridge response depth
    parameter integer AHB_REGION_SIZE_KB = 4,              // Equal AHB slot size (kilobytes)
    parameter         AW_SLICE_EN        = 1'b1,           // AW register slice enable
    parameter         W_SLICE_EN         = 1'b1,           // W register slice enable
    parameter         B_SLICE_EN         = 1'b1,           // B register slice enable
    parameter         AR_SLICE_EN        = 1'b1,           // AR register slice enable
    parameter         R_SLICE_EN         = 1'b1            // R register slice enable
) (
    input  wire                         aclk,
    input  wire                         aresetn,

    input  wire                         mext_awvalid,
    output wire                         mext_awready,
    input  wire [MEXT_ID_WIDTH-1:0]     mext_awid,
    input  wire [ADDR_WIDTH-1:0]        mext_awaddr,
    input  wire [3:0]                   mext_awlen,
    input  wire [2:0]                   mext_awsize,
    input  wire [1:0]                   mext_awburst,
    input  wire [1:0]                   mext_awlock,
    input  wire [3:0]                   mext_awcache,
    input  wire [2:0]                   mext_awprot,

    input  wire                         mext_wvalid,
    output wire                         mext_wready,
    input  wire [MEXT_ID_WIDTH-1:0]     mext_wid,
    input  wire [DATA_WIDTH-1:0]        mext_wdata,
    input  wire [DATA_WIDTH/8-1:0]      mext_wstrb,
    input  wire                         mext_wlast,

    output wire                         mext_bvalid,
    input  wire                         mext_bready,
    output wire [MEXT_ID_WIDTH-1:0]     mext_bid,
    output wire [1:0]                   mext_bresp,

    input  wire                         mext_arvalid,
    output wire                         mext_arready,
    input  wire [MEXT_ID_WIDTH-1:0]     mext_arid,
    input  wire [ADDR_WIDTH-1:0]        mext_araddr,
    input  wire [3:0]                   mext_arlen,
    input  wire [2:0]                   mext_arsize,
    input  wire [1:0]                   mext_arburst,
    input  wire [1:0]                   mext_arlock,
    input  wire [3:0]                   mext_arcache,
    input  wire [2:0]                   mext_arprot,

    output wire                         mext_rvalid,
    input  wire                         mext_rready,
    output wire [MEXT_ID_WIDTH-1:0]     mext_rid,
    output wire [DATA_WIDTH-1:0]        mext_rdata,
    output wire [1:0]                   mext_rresp,
    output wire                         mext_rlast,

    output wire [NUM_PORTS*ADDR_WIDTH-1:0] ahb_haddr,
    output wire [NUM_PORTS*2-1:0]          ahb_htrans,
    output wire [NUM_PORTS-1:0]            ahb_hwrite,
    output wire [NUM_PORTS*3-1:0]          ahb_hsize,
    output wire [NUM_PORTS*3-1:0]          ahb_hburst,
    output wire [NUM_PORTS*DATA_WIDTH-1:0] ahb_hwdata,
    input  wire [NUM_PORTS*DATA_WIDTH-1:0] ahb_hrdata,
    input  wire [NUM_PORTS-1:0]            ahb_hready,
    input  wire [NUM_PORTS-1:0]            ahb_hresp
);

    localparam integer STRB_W = DATA_WIDTH / 8;
    localparam integer AW_P   = MEXT_ID_WIDTH + ADDR_WIDTH + 4 + 3 + 2 + 2 + 4 + 3;
    localparam integer AR_P   = MEXT_ID_WIDTH + ADDR_WIDTH + 4 + 3 + 2 + 2 + 4 + 3;
    localparam integer W_P    = MEXT_ID_WIDTH + DATA_WIDTH + STRB_W + 1;
    localparam integer B_P    = MEXT_ID_WIDTH + 2;
    localparam integer R_P    = MEXT_ID_WIDTH + DATA_WIDTH + 2 + 1;

    wire                         rt_awvalid, rt_wvalid, rt_bvalid, rt_arvalid, rt_rvalid;
    wire                         rt_awready, rt_wready, rt_bready, rt_arready, rt_rready;
    wire [MEXT_ID_WIDTH-1:0]     rt_awid, rt_wid, rt_bid, rt_arid, rt_rid;
    wire [ADDR_WIDTH-1:0]        rt_awaddr, rt_araddr;
    wire [3:0]                   rt_awlen, rt_arlen;
    wire [2:0]                   rt_awsize, rt_arsize;
    wire [1:0]                   rt_awburst, rt_arburst, rt_awlock, rt_arlock;
    wire [3:0]                   rt_awcache, rt_arcache;
    wire [2:0]                   rt_awprot, rt_arprot;
    wire [DATA_WIDTH-1:0]        rt_wdata, rt_rdata;
    wire [STRB_W-1:0]            rt_wstrb;
    wire                         rt_wlast, rt_rlast;
    wire [1:0]                   rt_bresp, rt_rresp;
    wire [NUM_PORTS-1:0]         rt_aw_sel, rt_ar_sel;
    wire [ADDR_WIDTH-1:0]        mext_awaddr_map, mext_araddr_map;
    wire [ADDR_WIDTH-1:0]        rt_awaddr_tgt, rt_araddr_tgt;

    wire [AW_P-1:0] aw_pld_s, aw_pld_m;
    wire [W_P-1:0]  w_pld_s,  w_pld_m;
    wire [B_P-1:0]  b_pld_s,  b_pld_m;
    wire [AR_P-1:0] ar_pld_s, ar_pld_m;
    wire [R_P-1:0]  r_pld_s,  r_pld_m;

    assign aw_pld_s = {mext_awprot, mext_awcache, mext_awlock, mext_awburst, mext_awsize,
                       mext_awlen, mext_awaddr_map, mext_awid};
    assign {rt_awprot, rt_awcache, rt_awlock, rt_awburst, rt_awsize,
            rt_awlen, rt_awaddr, rt_awid} = aw_pld_m;

    assign w_pld_s = {mext_wlast, mext_wstrb, mext_wdata, mext_wid};
    assign {rt_wlast, rt_wstrb, rt_wdata, rt_wid} = w_pld_m;

    assign b_pld_s = {rt_bresp, rt_bid};
    assign {mext_bresp, mext_bid} = b_pld_m;

    assign ar_pld_s = {mext_arprot, mext_arcache, mext_arlock, mext_arburst, mext_arsize,
                       mext_arlen, mext_araddr_map, mext_arid};
    assign {rt_arprot, rt_arcache, rt_arlock, rt_arburst, rt_arsize,
            rt_arlen, rt_araddr, rt_arid} = ar_pld_m;

    assign r_pld_s = {rt_rlast, rt_rresp, rt_rdata, rt_rid};
    assign {mext_rlast, mext_rresp, mext_rdata, mext_rid} = r_pld_m;

    axi3_ahb_region_sel #(
        .N_PORTS(NUM_PORTS),
        .ADDR_WIDTH(ADDR_WIDTH),
        .REGION_SIZE_KB(AHB_REGION_SIZE_KB)
    ) u_mext_awaddr_map (
        .addr(mext_awaddr),
        .sel(),
        .addr_map(mext_awaddr_map),
        .addr_tgt()
    );

    axi3_ahb_region_sel #(
        .N_PORTS(NUM_PORTS),
        .ADDR_WIDTH(ADDR_WIDTH),
        .REGION_SIZE_KB(AHB_REGION_SIZE_KB)
    ) u_mext_araddr_map (
        .addr(mext_araddr),
        .sel(),
        .addr_map(mext_araddr_map),
        .addr_tgt()
    );

    axi3_reg_slice_ch #(
        .PAYLOAD_WIDTH(AW_P),
        .ENABLE(AW_SLICE_EN)
    ) u_aw_slice (
        .aclk(aclk), .aresetn(aresetn),
        .valid_s(mext_awvalid), .ready_s(mext_awready), .payload_s(aw_pld_s),
        .valid_m(rt_awvalid), .ready_m(rt_awready), .payload_m(aw_pld_m)
    );

    axi3_reg_slice_ch #(
        .PAYLOAD_WIDTH(W_P),
        .ENABLE(W_SLICE_EN)
    ) u_w_slice (
        .aclk(aclk), .aresetn(aresetn),
        .valid_s(mext_wvalid), .ready_s(mext_wready), .payload_s(w_pld_s),
        .valid_m(rt_wvalid), .ready_m(rt_wready), .payload_m(w_pld_m)
    );

    axi3_reg_slice_ch #(
        .PAYLOAD_WIDTH(B_P),
        .ENABLE(B_SLICE_EN)
    ) u_b_slice (
        .aclk(aclk), .aresetn(aresetn),
        .valid_s(rt_bvalid), .ready_s(rt_bready), .payload_s(b_pld_s),
        .valid_m(mext_bvalid), .ready_m(mext_bready), .payload_m(b_pld_m)
    );

    axi3_reg_slice_ch #(
        .PAYLOAD_WIDTH(AR_P),
        .ENABLE(AR_SLICE_EN)
    ) u_ar_slice (
        .aclk(aclk), .aresetn(aresetn),
        .valid_s(mext_arvalid), .ready_s(mext_arready), .payload_s(ar_pld_s),
        .valid_m(rt_arvalid), .ready_m(rt_arready), .payload_m(ar_pld_m)
    );

    axi3_reg_slice_ch #(
        .PAYLOAD_WIDTH(R_P),
        .ENABLE(R_SLICE_EN)
    ) u_r_slice (
        .aclk(aclk), .aresetn(aresetn),
        .valid_s(rt_rvalid), .ready_s(rt_rready), .payload_s(r_pld_s),
        .valid_m(mext_rvalid), .ready_m(mext_rready), .payload_m(r_pld_m)
    );

    axi3_ahb_region_sel #(
        .N_PORTS(NUM_PORTS),
        .ADDR_WIDTH(ADDR_WIDTH),
        .REGION_SIZE_KB(AHB_REGION_SIZE_KB)
    ) u_mext_aw_region_sel (
        .addr(rt_awaddr),
        .sel(rt_aw_sel),
        .addr_map(),
        .addr_tgt(rt_awaddr_tgt)
    );

    axi3_ahb_region_sel #(
        .N_PORTS(NUM_PORTS),
        .ADDR_WIDTH(ADDR_WIDTH),
        .REGION_SIZE_KB(AHB_REGION_SIZE_KB)
    ) u_mext_ar_region_sel (
        .addr(rt_araddr),
        .sel(rt_ar_sel),
        .addr_map(),
        .addr_tgt(rt_araddr_tgt)
    );

    axi3_router_1toN_ahblite #(
        .N(NUM_PORTS),
        .ADDR_WIDTH(ADDR_WIDTH),
        .ID_WIDTH(MEXT_ID_WIDTH),
        .ROUTER_OUTSTANDING(ROUTER_OUTSTANDING),
        .WR_CMD_DEPTH(WR_CMD_DEPTH),
        .RD_CMD_DEPTH(RD_CMD_DEPTH),
        .RESP_DEPTH(RESP_DEPTH)
    ) u_mext_router_ahb (
        .aclk(aclk), .aresetn(aresetn),
        .aw_sel(rt_aw_sel), .ar_sel(rt_ar_sel),
        .s_awvalid(rt_awvalid), .s_awready(rt_awready),
        .s_awid(rt_awid), .s_awaddr(rt_awaddr_tgt), .s_awlen(rt_awlen),
        .s_awsize(rt_awsize), .s_awburst(rt_awburst), .s_awlock(rt_awlock),
        .s_awcache(rt_awcache), .s_awprot(rt_awprot),
        .s_wvalid(rt_wvalid), .s_wready(rt_wready),
        .s_wid(rt_wid), .s_wdata(rt_wdata), .s_wstrb(rt_wstrb), .s_wlast(rt_wlast),
        .s_bvalid(rt_bvalid), .s_bready(rt_bready),
        .s_bid(rt_bid), .s_bresp(rt_bresp),
        .s_arvalid(rt_arvalid), .s_arready(rt_arready),
        .s_arid(rt_arid), .s_araddr(rt_araddr_tgt), .s_arlen(rt_arlen),
        .s_arsize(rt_arsize), .s_arburst(rt_arburst), .s_arlock(rt_arlock),
        .s_arcache(rt_arcache), .s_arprot(rt_arprot),
        .s_rvalid(rt_rvalid), .s_rready(rt_rready),
        .s_rid(rt_rid), .s_rdata(rt_rdata), .s_rresp(rt_rresp), .s_rlast(rt_rlast),
        .haddr(ahb_haddr), .htrans(ahb_htrans), .hwrite(ahb_hwrite),
        .hsize(ahb_hsize), .hburst(ahb_hburst),
        .hwdata(ahb_hwdata), .hrdata(ahb_hrdata),
        .hready(ahb_hready), .hresp(ahb_hresp)
    );

    // -------------------------------------------------------------------------
    // Elaboration checks
    // -------------------------------------------------------------------------
    localparam integer PLT_REGION_BYTES = AHB_REGION_SIZE_KB * 1024;
    localparam integer PLT_REGION_LSB   = $clog2(PLT_REGION_BYTES);
    localparam integer PLT_DECODE_W     = (NUM_PORTS <= 1) ? 1 : $clog2(NUM_PORTS);

    `ifdef SYNTHESIS
    `else
    initial begin
        if (NUM_PORTS < 1) begin
            $error("%m: lbus_plt: NUM_PORTS must be >= 1 (got %0d)", NUM_PORTS);
        end
        if (AHB_REGION_SIZE_KB < 1) begin
            $error("%m: lbus_plt: AHB_REGION_SIZE_KB must be >= 1 (got %0d)",
                   AHB_REGION_SIZE_KB);
        end
        if ((AHB_REGION_SIZE_KB & (AHB_REGION_SIZE_KB - 1)) != 0) begin
            $error("%m: lbus_plt: AHB_REGION_SIZE_KB must be a power of two (got %0d)",
                   AHB_REGION_SIZE_KB);
        end
        if (NUM_PORTS >= 1 && (PLT_REGION_LSB + PLT_DECODE_W) > ADDR_WIDTH) begin
            $error("%m: lbus_plt: AHB decode field exceeds ADDR_WIDTH (REGION_LSB=%0d DECODE_W=%0d ADDR_WIDTH=%0d)",
                   PLT_REGION_LSB, PLT_DECODE_W, ADDR_WIDTH);
        end
    end
    `endif

endmodule
