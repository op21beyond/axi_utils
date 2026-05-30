// -----------------------------------------------------------------------------
// Module      : axi3_router_1toN_ahblite
// Date        : 2026-05-27
// Version     : v1.0.0
// Author      : Jongchul Shin
// Function    : Integrated top module:
//               1x AXI3 slave -> AXI3 1:N router -> Nx AXI3-to-AHB-Lite bridges.
//               Exposes N independent AHB-Lite master ports.
// Assumptions : Inherits assumptions from axi3_router_1toN and axi3_to_ahblite.
//               Fixed 32-bit AXI/AHB data path (matches axi3_to_ahblite).
//               aw_sel/ar_sel are one-hot when s_awvalid/s_arvalid is high.
//               No AXI3 write data interleaving: W beats follow AW acceptance
//               order; each write completes with WLAST before W for the next AW.
//               WID equals AWID for every W beat of a write transaction.
//               Read data returns in AR acceptance order (no R interleaving by RID).
//               Each downstream bridge obeys axi3_to_ahblite assumptions.
// Notes       : AHB outputs SINGLE transfers only (hburst=000; htrans=IDLE/NONSEQ).
//               WID is not used for W routing (target select only); it is stored
//               on WLAST for BID restore.
// -----------------------------------------------------------------------------
module axi3_router_1toN_ahblite #(
    parameter integer N                  = 8,                 // Number of AHB-Lite output ports
    parameter integer ADDR_WIDTH         = 32,                // Address bus width
    parameter integer ID_WIDTH           = 4,                 // Upstream AXI slave ID width
    parameter integer ROUTER_OUTSTANDING = 16,                // Router ordering FIFO depth
    parameter integer WR_CMD_DEPTH       = 16,                // Per-bridge write command depth
    parameter integer RD_CMD_DEPTH       = 16,                // Per-bridge read command depth
    parameter integer RESP_DEPTH         = 8                  // Per-bridge response FIFO depth
) (
    input  wire                         aclk,
    input  wire                         aresetn,

    input  wire [N-1:0]                 aw_sel,
    input  wire [N-1:0]                 ar_sel,

    input  wire                         s_awvalid,
    output wire                         s_awready,
    input  wire [ID_WIDTH-1:0]          s_awid,
    input  wire [ADDR_WIDTH-1:0]        s_awaddr,
    input  wire [3:0]                   s_awlen,
    input  wire [2:0]                   s_awsize,
    input  wire [1:0]                   s_awburst,
    input  wire [1:0]                   s_awlock,
    input  wire [3:0]                   s_awcache,
    input  wire [2:0]                   s_awprot,

    input  wire                         s_wvalid,
    output wire                         s_wready,
    input  wire [ID_WIDTH-1:0]          s_wid,
    input  wire [31:0]                  s_wdata,
    input  wire [3:0]                   s_wstrb,
    input  wire                         s_wlast,

    output wire                         s_bvalid,
    input  wire                         s_bready,
    output wire [ID_WIDTH-1:0]          s_bid,
    output wire [1:0]                   s_bresp,

    input  wire                         s_arvalid,
    output wire                         s_arready,
    input  wire [ID_WIDTH-1:0]          s_arid,
    input  wire [ADDR_WIDTH-1:0]        s_araddr,
    input  wire [3:0]                   s_arlen,
    input  wire [2:0]                   s_arsize,
    input  wire [1:0]                   s_arburst,
    input  wire [1:0]                   s_arlock,
    input  wire [3:0]                   s_arcache,
    input  wire [2:0]                   s_arprot,

    output wire                         s_rvalid,
    input  wire                         s_rready,
    output wire [ID_WIDTH-1:0]          s_rid,
    output wire [31:0]                  s_rdata,
    output wire [1:0]                   s_rresp,
    output wire                         s_rlast,

    output wire [N*ADDR_WIDTH-1:0]      haddr,
    output wire [N*2-1:0]               htrans,
    output wire [N-1:0]                 hwrite,
    output wire [N*3-1:0]               hsize,
    output wire [N*3-1:0]               hburst,
    output wire [N*32-1:0]              hwdata,
    input  wire [N*32-1:0]              hrdata,
    input  wire [N-1:0]                 hready,
    input  wire [N-1:0]                 hresp
);

    wire [N-1:0]            m_awvalid;
    wire [N-1:0]            m_awready;
    wire [N*ADDR_WIDTH-1:0] m_awaddr;
    wire [N*4-1:0]          m_awlen;
    wire [N*3-1:0]          m_awsize;
    wire [N*2-1:0]          m_awburst;
    wire [N*2-1:0]          m_awlock;
    wire [N*4-1:0]          m_awcache;
    wire [N*3-1:0]          m_awprot;

    wire [N-1:0]            m_wvalid;
    wire [N-1:0]            m_wready;
    wire [N*32-1:0] m_wdata;
    wire [N*4-1:0]  m_wstrb;
    wire [N-1:0]            m_wlast;

    wire [N-1:0]            m_bvalid;
    wire [N-1:0]            m_bready;
    wire [N*2-1:0]          m_bresp;

    wire [N-1:0]            m_arvalid;
    wire [N-1:0]            m_arready;
    wire [N*ADDR_WIDTH-1:0] m_araddr;
    wire [N*4-1:0]          m_arlen;
    wire [N*3-1:0]          m_arsize;
    wire [N*2-1:0]          m_arburst;
    wire [N*2-1:0]          m_arlock;
    wire [N*4-1:0]          m_arcache;
    wire [N*3-1:0]          m_arprot;

    wire [N-1:0]            m_rvalid;
    wire [N-1:0]            m_rready;
    wire [N*32-1:0] m_rdata;
    wire [N*2-1:0]          m_rresp;
    wire [N-1:0]            m_rlast;

    axi3_router_1toN #(
        .N(N),
        .ADDR_WIDTH(ADDR_WIDTH),
        .DATA_WIDTH(32),
        .STRB_WIDTH(4),
        .ID_WIDTH(ID_WIDTH),
        .OUTSTANDING_DEPTH(ROUTER_OUTSTANDING)
    ) u_axi3_router_1toN (
        .aclk(aclk),
        .aresetn(aresetn),
        .aw_sel(aw_sel),
        .ar_sel(ar_sel),
        .s_awvalid(s_awvalid),
        .s_awready(s_awready),
        .s_awid(s_awid),
        .s_awaddr(s_awaddr),
        .s_awlen(s_awlen),
        .s_awsize(s_awsize),
        .s_awburst(s_awburst),
        .s_awlock(s_awlock),
        .s_awcache(s_awcache),
        .s_awprot(s_awprot),
        .s_wvalid(s_wvalid),
        .s_wready(s_wready),
        .s_wid(s_wid),
        .s_wdata(s_wdata),
        .s_wstrb(s_wstrb),
        .s_wlast(s_wlast),
        .s_bvalid(s_bvalid),
        .s_bready(s_bready),
        .s_bid(s_bid),
        .s_bresp(s_bresp),
        .s_arvalid(s_arvalid),
        .s_arready(s_arready),
        .s_arid(s_arid),
        .s_araddr(s_araddr),
        .s_arlen(s_arlen),
        .s_arsize(s_arsize),
        .s_arburst(s_arburst),
        .s_arlock(s_arlock),
        .s_arcache(s_arcache),
        .s_arprot(s_arprot),
        .s_rvalid(s_rvalid),
        .s_rready(s_rready),
        .s_rid(s_rid),
        .s_rdata(s_rdata),
        .s_rresp(s_rresp),
        .s_rlast(s_rlast),
        .m_awvalid(m_awvalid),
        .m_awready(m_awready),
        .m_awaddr(m_awaddr),
        .m_awlen(m_awlen),
        .m_awsize(m_awsize),
        .m_awburst(m_awburst),
        .m_awlock(m_awlock),
        .m_awcache(m_awcache),
        .m_awprot(m_awprot),
        .m_wvalid(m_wvalid),
        .m_wready(m_wready),
        .m_wdata(m_wdata),
        .m_wstrb(m_wstrb),
        .m_wlast(m_wlast),
        .m_bvalid(m_bvalid),
        .m_bready(m_bready),
        .m_bresp(m_bresp),
        .m_arvalid(m_arvalid),
        .m_arready(m_arready),
        .m_araddr(m_araddr),
        .m_arlen(m_arlen),
        .m_arsize(m_arsize),
        .m_arburst(m_arburst),
        .m_arlock(m_arlock),
        .m_arcache(m_arcache),
        .m_arprot(m_arprot),
        .m_rvalid(m_rvalid),
        .m_rready(m_rready),
        .m_rdata(m_rdata),
        .m_rresp(m_rresp),
        .m_rlast(m_rlast)
    );

    genvar i;
    generate
        for (i = 0; i < N; i = i + 1) begin : g_ahb_bridge
            axi3_to_ahblite #(
                .ADDR_WIDTH(ADDR_WIDTH),
                .WR_CMD_DEPTH(WR_CMD_DEPTH),
                .RD_CMD_DEPTH(RD_CMD_DEPTH),
                .RESP_DEPTH(RESP_DEPTH)
            ) u_axi3_to_ahblite (
                .aclk(aclk),
                .aresetn(aresetn),
                .s_awvalid(m_awvalid[i]),
                .s_awready(m_awready[i]),
                .s_awaddr(m_awaddr[(i*ADDR_WIDTH) +: ADDR_WIDTH]),
                .s_awlen(m_awlen[(i*4) +: 4]),
                .s_awsize(m_awsize[(i*3) +: 3]),
                .s_awburst(m_awburst[(i*2) +: 2]),
                .s_awlock(m_awlock[(i*2) +: 2]),
                .s_awcache(m_awcache[(i*4) +: 4]),
                .s_awprot(m_awprot[(i*3) +: 3]),
                .s_wvalid(m_wvalid[i]),
                .s_wready(m_wready[i]),
                .s_wdata(m_wdata[(i*32) +: 32]),
                .s_wstrb(m_wstrb[(i*4) +: 4]),
                .s_wlast(m_wlast[i]),
                .s_bvalid(m_bvalid[i]),
                .s_bready(m_bready[i]),
                .s_bresp(m_bresp[(i*2) +: 2]),
                .s_arvalid(m_arvalid[i]),
                .s_arready(m_arready[i]),
                .s_araddr(m_araddr[(i*ADDR_WIDTH) +: ADDR_WIDTH]),
                .s_arlen(m_arlen[(i*4) +: 4]),
                .s_arsize(m_arsize[(i*3) +: 3]),
                .s_arburst(m_arburst[(i*2) +: 2]),
                .s_arlock(m_arlock[(i*2) +: 2]),
                .s_arcache(m_arcache[(i*4) +: 4]),
                .s_arprot(m_arprot[(i*3) +: 3]),
                .s_rvalid(m_rvalid[i]),
                .s_rready(m_rready[i]),
                .s_rdata(m_rdata[(i*32) +: 32]),
                .s_rresp(m_rresp[(i*2) +: 2]),
                .s_rlast(m_rlast[i]),
                .haddr(haddr[(i*ADDR_WIDTH) +: ADDR_WIDTH]),
                .htrans(htrans[(i*2) +: 2]),
                .hwrite(hwrite[i]),
                .hsize(hsize[(i*3) +: 3]),
                .hburst(hburst[(i*3) +: 3]),
                .hwdata(hwdata[(i*32) +: 32]),
                .hrdata(hrdata[(i*32) +: 32]),
                .hready(hready[i]),
                .hresp(hresp[i])
            );
        end
    endgenerate

    // synopsys translate_off
    // Simulation-only: verify upstream master meets router ID/ordering assumptions.
    localparam integer SIM_PTR_W = (ROUTER_OUTSTANDING <= 2) ? 1 :
                                   (ROUTER_OUTSTANDING <= 4) ? 2 :
                                   (ROUTER_OUTSTANDING <= 8) ? 3 :
                                   (ROUTER_OUTSTANDING <= 16) ? 4 :
                                   (ROUTER_OUTSTANDING <= 32) ? 5 :
                                   (ROUTER_OUTSTANDING <= 64) ? 6 : 7;

    reg [ID_WIDTH-1:0] sim_wr_id_fifo [0:ROUTER_OUTSTANDING-1];
    reg [ID_WIDTH-1:0] sim_rd_id_fifo [0:ROUTER_OUTSTANDING-1];
    reg [SIM_PTR_W-1:0] sim_wr_wr_ptr, sim_wr_rd_ptr;
    reg [SIM_PTR_W-1:0] sim_rd_wr_ptr, sim_rd_rd_ptr;
    reg [SIM_PTR_W:0]   sim_wr_count, sim_rd_count;

    wire sim_wr_empty = (sim_wr_count == 0);
    wire sim_rd_empty = (sim_rd_count == 0);

    wire aw_hs     = s_awvalid && s_awready;
    wire ar_hs     = s_arvalid && s_arready;
    wire w_hs      = s_wvalid && s_wready;
    wire w_last_hs = w_hs && s_wlast;
    wire r_hs      = s_rvalid && s_rready;
    wire r_last_hs = r_hs && s_rlast;

    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            sim_wr_wr_ptr <= {SIM_PTR_W{1'b0}};
            sim_wr_rd_ptr <= {SIM_PTR_W{1'b0}};
            sim_rd_wr_ptr <= {SIM_PTR_W{1'b0}};
            sim_rd_rd_ptr <= {SIM_PTR_W{1'b0}};
            sim_wr_count  <= {(SIM_PTR_W+1){1'b0}};
            sim_rd_count  <= {(SIM_PTR_W+1){1'b0}};
        end else begin
            if (w_hs) begin
                if (sim_wr_empty)
                    $error("%m: axi3_router_1toN_ahblite: W beat without accepted AW");
                else if (s_wid !== sim_wr_id_fifo[sim_wr_rd_ptr])
                    $error("%m: axi3_router_1toN_ahblite: WID must match front AWID (no write interleaving)");
            end

            if (r_hs) begin
                if (sim_rd_empty)
                    $error("%m: axi3_router_1toN_ahblite: R beat without accepted AR");
                else if (s_rid !== sim_rd_id_fifo[sim_rd_rd_ptr])
                    $error("%m: axi3_router_1toN_ahblite: RID must match front ARID (no read interleaving)");
            end

            if (aw_hs) begin
                if (sim_wr_count == ROUTER_OUTSTANDING)
                    $error("%m: axi3_router_1toN_ahblite: AW exceeds router outstanding depth");
                sim_wr_id_fifo[sim_wr_wr_ptr] <= s_awid;
                sim_wr_wr_ptr <= (sim_wr_wr_ptr == ROUTER_OUTSTANDING-1) ? {SIM_PTR_W{1'b0}} :
                                 (sim_wr_wr_ptr + 1'b1);
            end

            if (w_last_hs && !sim_wr_empty) begin
                sim_wr_rd_ptr <= (sim_wr_rd_ptr == ROUTER_OUTSTANDING-1) ? {SIM_PTR_W{1'b0}} :
                                 (sim_wr_rd_ptr + 1'b1);
            end

            if (ar_hs) begin
                if (sim_rd_count == ROUTER_OUTSTANDING)
                    $error("%m: axi3_router_1toN_ahblite: AR exceeds router outstanding depth");
                sim_rd_id_fifo[sim_rd_wr_ptr] <= s_arid;
                sim_rd_wr_ptr <= (sim_rd_wr_ptr == ROUTER_OUTSTANDING-1) ? {SIM_PTR_W{1'b0}} :
                                 (sim_rd_wr_ptr + 1'b1);
            end

            if (r_last_hs && !sim_rd_empty) begin
                sim_rd_rd_ptr <= (sim_rd_rd_ptr == ROUTER_OUTSTANDING-1) ? {SIM_PTR_W{1'b0}} :
                                 (sim_rd_rd_ptr + 1'b1);
            end

            case ({aw_hs, (w_last_hs && !sim_wr_empty)})
                2'b10: sim_wr_count <= sim_wr_count + 1'b1;
                2'b01: sim_wr_count <= sim_wr_count - 1'b1;
                default: sim_wr_count <= sim_wr_count;
            endcase

            case ({ar_hs, (r_last_hs && !sim_rd_empty)})
                2'b10: sim_rd_count <= sim_rd_count + 1'b1;
                2'b01: sim_rd_count <= sim_rd_count - 1'b1;
                default: sim_rd_count <= sim_rd_count;
            endcase
        end
    end
    // synopsys translate_on

endmodule
