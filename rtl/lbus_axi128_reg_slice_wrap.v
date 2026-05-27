// -----------------------------------------------------------------------------
// Module      : lbus_axi128_reg_slice_wrap
// Date        : 2026-05-28
// Version     : v1.0.0
// Author      : Jongchul Shin
// Function    : AXI3 128-bit channel wrapper around axi3_reg_slice.
// -----------------------------------------------------------------------------
module lbus_axi128_reg_slice_wrap #(
    parameter integer ID_WIDTH      = 4,
    parameter integer ADDR_WIDTH    = 32,
    parameter integer DATA_WIDTH    = 128,
    parameter integer STRB_WIDTH    = DATA_WIDTH / 8,
    parameter         AW_SLICE_EN   = 1,
    parameter         W_SLICE_EN    = 1,
    parameter         B_SLICE_EN    = 1,
    parameter         AR_SLICE_EN   = 1,
    parameter         R_SLICE_EN    = 1
) (
    input  wire                         aclk,
    input  wire                         aresetn,

    input  wire                         awvalid_s,
    output wire                         awready_s,
    input  wire [ID_WIDTH-1:0]          awid_s,
    input  wire [ADDR_WIDTH-1:0]        awaddr_s,
    input  wire [3:0]                   awlen_s,
    input  wire [2:0]                   awsize_s,
    input  wire [1:0]                   awburst_s,
    input  wire [1:0]                   awlock_s,
    input  wire [3:0]                   awcache_s,
    input  wire [2:0]                   awprot_s,
    output wire                         awvalid_m,
    input  wire                         awready_m,
    output wire [ID_WIDTH-1:0]          awid_m,
    output wire [ADDR_WIDTH-1:0]        awaddr_m,
    output wire [3:0]                   awlen_m,
    output wire [2:0]                   awsize_m,
    output wire [1:0]                   awburst_m,
    output wire [1:0]                   awlock_m,
    output wire [3:0]                   awcache_m,
    output wire [2:0]                   awprot_m,

    input  wire                         wvalid_s,
    output wire                         wready_s,
    input  wire [ID_WIDTH-1:0]          wid_s,
    input  wire [DATA_WIDTH-1:0]        wdata_s,
    input  wire [STRB_WIDTH-1:0]        wstrb_s,
    input  wire                         wlast_s,
    output wire                         wvalid_m,
    input  wire                         wready_m,
    output wire [ID_WIDTH-1:0]          wid_m,
    output wire [DATA_WIDTH-1:0]        wdata_m,
    output wire [STRB_WIDTH-1:0]        wstrb_m,
    output wire                         wlast_m,

    input  wire                         bvalid_s,
    output wire                         bready_s,
    input  wire [ID_WIDTH-1:0]          bid_s,
    input  wire [1:0]                   bresp_s,
    output wire                         bvalid_m,
    input  wire                         bready_m,
    output wire [ID_WIDTH-1:0]          bid_m,
    output wire [1:0]                   bresp_m,

    input  wire                         arvalid_s,
    output wire                         arready_s,
    input  wire [ID_WIDTH-1:0]          arid_s,
    input  wire [ADDR_WIDTH-1:0]        araddr_s,
    input  wire [3:0]                   arlen_s,
    input  wire [2:0]                   arsize_s,
    input  wire [1:0]                   arburst_s,
    input  wire [1:0]                   arlock_s,
    input  wire [3:0]                   arcache_s,
    input  wire [2:0]                   arprot_s,
    output wire                         arvalid_m,
    input  wire                         arready_m,
    output wire [ID_WIDTH-1:0]          arid_m,
    output wire [ADDR_WIDTH-1:0]        araddr_m,
    output wire [3:0]                   arlen_m,
    output wire [2:0]                   arsize_m,
    output wire [1:0]                   arburst_m,
    output wire [1:0]                   arlock_m,
    output wire [3:0]                   arcache_m,
    output wire [2:0]                   arprot_m,

    input  wire                         rvalid_s,
    output wire                         rready_s,
    input  wire [ID_WIDTH-1:0]          rid_s,
    input  wire [DATA_WIDTH-1:0]        rdata_s,
    input  wire [1:0]                   rresp_s,
    input  wire                         rlast_s,
    output wire                         rvalid_m,
    input  wire                         rready_m,
    output wire [ID_WIDTH-1:0]          rid_m,
    output wire [DATA_WIDTH-1:0]        rdata_m,
    output wire [1:0]                   rresp_m,
    output wire                         rlast_m
);

    localparam integer AW_P = ID_WIDTH + ADDR_WIDTH + 4 + 3 + 2 + 2 + 4 + 3;
    localparam integer W_P  = ID_WIDTH + DATA_WIDTH + STRB_WIDTH + 1;
    localparam integer B_P  = ID_WIDTH + 2;
    localparam integer R_P  = ID_WIDTH + DATA_WIDTH + 2 + 1;

    wire [AW_P-1:0] aw_pld_s;
    wire [AW_P-1:0] aw_pld_m;
    wire [W_P-1:0]  w_pld_s;
    wire [W_P-1:0]  w_pld_m;
    wire [B_P-1:0]  b_pld_s;
    wire [B_P-1:0]  b_pld_m;
    wire [AW_P-1:0] ar_pld_s;
    wire [AW_P-1:0] ar_pld_m;
    wire [R_P-1:0]  r_pld_s;
    wire [R_P-1:0]  r_pld_m;

    assign aw_pld_s = {awprot_s, awcache_s, awlock_s, awburst_s, awsize_s,
                       awlen_s, awaddr_s, awid_s};
    assign w_pld_s  = {wlast_s, wstrb_s, wdata_s, wid_s};
    assign b_pld_s  = {bresp_s, bid_s};
    assign ar_pld_s = {arprot_s, arcache_s, arlock_s, arburst_s, arsize_s,
                      arlen_s, araddr_s, arid_s};
    assign r_pld_s  = {rlast_s, rresp_s, rdata_s, rid_s};

    axi3_reg_slice #(
        .AW_PAYLOAD_WIDTH(AW_P),
        .W_PAYLOAD_WIDTH(W_P),
        .B_PAYLOAD_WIDTH(B_P),
        .AR_PAYLOAD_WIDTH(AW_P),
        .R_PAYLOAD_WIDTH(R_P),
        .AW_SLICE_EN(AW_SLICE_EN),
        .W_SLICE_EN(W_SLICE_EN),
        .B_SLICE_EN(B_SLICE_EN),
        .AR_SLICE_EN(AR_SLICE_EN),
        .R_SLICE_EN(R_SLICE_EN)
    ) u_reg_slice (
        .aclk(aclk), .aresetn(aresetn),
        .awvalid_s(awvalid_s), .awready_s(awready_s), .awpayload_s(aw_pld_s),
        .awvalid_m(awvalid_m), .awready_m(awready_m), .awpayload_m(aw_pld_m),
        .wvalid_s(wvalid_s), .wready_s(wready_s), .wpayload_s(w_pld_s),
        .wvalid_m(wvalid_m), .wready_m(wready_m), .wpayload_m(w_pld_m),
        .bvalid_s(bvalid_s), .bready_s(bready_s), .bpayload_s(b_pld_s),
        .bvalid_m(bvalid_m), .bready_m(bready_m), .bpayload_m(b_pld_m),
        .arvalid_s(arvalid_s), .arready_s(arready_s), .arpayload_s(ar_pld_s),
        .arvalid_m(arvalid_m), .arready_m(arready_m), .arpayload_m(ar_pld_m),
        .rvalid_s(rvalid_s), .rready_s(rready_s), .rpayload_s(r_pld_s),
        .rvalid_m(rvalid_m), .rready_m(rready_m), .rpayload_m(r_pld_m)
    );

    assign {awprot_m, awcache_m, awlock_m, awburst_m, awsize_m,
            awlen_m, awaddr_m, awid_m} = aw_pld_m;
    assign {wlast_m, wstrb_m, wdata_m, wid_m} = w_pld_m;
    assign {bresp_m, bid_m} = b_pld_m;
    assign {arprot_m, arcache_m, arlock_m, arburst_m, arsize_m,
            arlen_m, araddr_m, arid_m} = ar_pld_m;
    assign {rlast_m, rresp_m, rdata_m, rid_m} = r_pld_m;

endmodule
