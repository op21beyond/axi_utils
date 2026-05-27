// -----------------------------------------------------------------------------
// Module      : axi3_router_1to2_128
// Date        : 2026-05-28
// Version     : v1.0.0
// Author      : Jongchul Shin
// Function    : 1:2 AXI3 router with 128-bit data path and AXI ID support.
//               External logic provides one-hot aw_sel/ar_sel per AX request.
//               Same AxID cannot switch target while outstanding exists.
// Assumptions : aw_sel/ar_sel are one-hot and valid when AXVALID is high.
//               USER/QOS fields are not present.
//               LOCK/CACHE/EXCLUSIVE/PROT are forwarded without special handling.
// Notes       : Write and read ordering domains are independent.
//               B and R response arbitration uses round-robin across targets.
// -----------------------------------------------------------------------------
module axi3_router_1to2_128 #(
    parameter integer ADDR_WIDTH = 32,                 // AXI address width
    parameter integer DATA_WIDTH = 128,                // AXI data width (fixed intent: 128-bit)
    parameter integer STRB_WIDTH = DATA_WIDTH / 8,     // AXI strobe width
    parameter integer ID_WIDTH   = 4,                  // AXI ID width
    parameter integer CNT_WIDTH  = 8                   // Outstanding counter width per ID (R/W each)
) (
    input  wire                         aclk,
    input  wire                         aresetn,

    input  wire [1:0]                   aw_sel,
    input  wire [1:0]                   ar_sel,

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
    input  wire [DATA_WIDTH-1:0]        s_wdata,
    input  wire [STRB_WIDTH-1:0]        s_wstrb,
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
    output wire [DATA_WIDTH-1:0]        s_rdata,
    output wire [1:0]                   s_rresp,
    output wire                         s_rlast,

    output wire [1:0]                   m_awvalid,
    input  wire [1:0]                   m_awready,
    output wire [2*ID_WIDTH-1:0]        m_awid,
    output wire [2*ADDR_WIDTH-1:0]      m_awaddr,
    output wire [2*4-1:0]               m_awlen,
    output wire [2*3-1:0]               m_awsize,
    output wire [2*2-1:0]               m_awburst,
    output wire [2*2-1:0]               m_awlock,
    output wire [2*4-1:0]               m_awcache,
    output wire [2*3-1:0]               m_awprot,

    output wire [1:0]                   m_wvalid,
    input  wire [1:0]                   m_wready,
    output wire [2*ID_WIDTH-1:0]        m_wid,
    output wire [2*DATA_WIDTH-1:0]      m_wdata,
    output wire [2*STRB_WIDTH-1:0]      m_wstrb,
    output wire [1:0]                   m_wlast,

    input  wire [1:0]                   m_bvalid,
    output wire [1:0]                   m_bready,
    input  wire [2*ID_WIDTH-1:0]        m_bid,
    input  wire [2*2-1:0]               m_bresp,

    output wire [1:0]                   m_arvalid,
    input  wire [1:0]                   m_arready,
    output wire [2*ID_WIDTH-1:0]        m_arid,
    output wire [2*ADDR_WIDTH-1:0]      m_araddr,
    output wire [2*4-1:0]               m_arlen,
    output wire [2*3-1:0]               m_arsize,
    output wire [2*2-1:0]               m_arburst,
    output wire [2*2-1:0]               m_arlock,
    output wire [2*4-1:0]               m_arcache,
    output wire [2*3-1:0]               m_arprot,

    input  wire [1:0]                   m_rvalid,
    output wire [1:0]                   m_rready,
    input  wire [2*ID_WIDTH-1:0]        m_rid,
    input  wire [2*DATA_WIDTH-1:0]      m_rdata,
    input  wire [2*2-1:0]               m_rresp,
    input  wire [1:0]                   m_rlast
);

    localparam integer ID_COUNT = (1 << ID_WIDTH);

    reg                wr_tgt [0:ID_COUNT-1];
    reg [CNT_WIDTH-1:0] wr_outs [0:ID_COUNT-1];
    reg                rd_tgt [0:ID_COUNT-1];
    reg [CNT_WIDTH-1:0] rd_outs [0:ID_COUNT-1];

    reg rr_b_sel;
    reg rr_r_sel;

    wire aw_tgt = aw_sel[1];
    wire ar_tgt = ar_sel[1];

    wire wr_has_outstanding = (wr_outs[s_awid] != {CNT_WIDTH{1'b0}});
    wire rd_has_outstanding = (rd_outs[s_arid] != {CNT_WIDTH{1'b0}});

    wire aw_id_target_ok = !wr_has_outstanding || (wr_tgt[s_awid] == aw_tgt);
    wire ar_id_target_ok = !rd_has_outstanding || (rd_tgt[s_arid] == ar_tgt);

    wire aw_ready_int = aw_id_target_ok && (aw_tgt ? m_awready[1] : m_awready[0]);
    wire ar_ready_int = ar_id_target_ok && (ar_tgt ? m_arready[1] : m_arready[0]);

    wire aw_hs = s_awvalid && aw_ready_int;
    wire ar_hs = s_arvalid && ar_ready_int;
    wire w_has_outstanding = (wr_outs[s_wid] != {CNT_WIDTH{1'b0}});
    wire w_tgt = wr_tgt[s_wid];
    wire w_ready_int = w_has_outstanding && (w_tgt ? m_wready[1] : m_wready[0]);
    wire w_hs = s_wvalid && w_ready_int;

    wire b_take0 = m_bvalid[0] && (!m_bvalid[1] || !rr_b_sel);
    wire b_take1 = m_bvalid[1] && (!m_bvalid[0] || rr_b_sel);
    wire s_bvalid_int = m_bvalid[0] || m_bvalid[1];
    wire b_hs = s_bvalid_int && s_bready;

    wire [ID_WIDTH-1:0] b_id_sel = b_take1 ? m_bid[(1*ID_WIDTH) +: ID_WIDTH] : m_bid[(0*ID_WIDTH) +: ID_WIDTH];

    wire r_take0 = m_rvalid[0] && (!m_rvalid[1] || !rr_r_sel);
    wire r_take1 = m_rvalid[1] && (!m_rvalid[0] || rr_r_sel);
    wire s_rvalid_int = m_rvalid[0] || m_rvalid[1];
    wire r_hs = s_rvalid_int && s_rready;

    wire [ID_WIDTH-1:0] r_id_sel = r_take1 ? m_rid[(1*ID_WIDTH) +: ID_WIDTH] : m_rid[(0*ID_WIDTH) +: ID_WIDTH];
    wire                 r_last_sel = r_take1 ? m_rlast[1] : m_rlast[0];

    assign s_awready = aw_ready_int;
    assign s_arready = ar_ready_int;
    assign s_wready  = w_ready_int;

    assign m_awvalid = (s_awvalid && aw_id_target_ok) ? (aw_tgt ? 2'b10 : 2'b01) : 2'b00;
    assign m_awid    = {s_awid, s_awid};
    assign m_awaddr  = {s_awaddr, s_awaddr};
    assign m_awlen   = {s_awlen, s_awlen};
    assign m_awsize  = {s_awsize, s_awsize};
    assign m_awburst = {s_awburst, s_awburst};
    assign m_awlock  = {s_awlock, s_awlock};
    assign m_awcache = {s_awcache, s_awcache};
    assign m_awprot  = {s_awprot, s_awprot};

    assign m_arvalid = (s_arvalid && ar_id_target_ok) ? (ar_tgt ? 2'b10 : 2'b01) : 2'b00;
    assign m_arid    = {s_arid, s_arid};
    assign m_araddr  = {s_araddr, s_araddr};
    assign m_arlen   = {s_arlen, s_arlen};
    assign m_arsize  = {s_arsize, s_arsize};
    assign m_arburst = {s_arburst, s_arburst};
    assign m_arlock  = {s_arlock, s_arlock};
    assign m_arcache = {s_arcache, s_arcache};
    assign m_arprot  = {s_arprot, s_arprot};

    assign m_wvalid = (s_wvalid && w_has_outstanding) ? (w_tgt ? 2'b10 : 2'b01) : 2'b00;
    assign m_wid    = {s_wid, s_wid};
    assign m_wdata  = {s_wdata, s_wdata};
    assign m_wstrb  = {s_wstrb, s_wstrb};
    assign m_wlast  = {s_wlast, s_wlast};

    assign s_bvalid = s_bvalid_int;
    assign s_bid    = b_id_sel;
    assign s_bresp  = b_take1 ? m_bresp[(1*2) +: 2] : m_bresp[(0*2) +: 2];
    assign m_bready = { (b_take1 && s_bready), (b_take0 && s_bready) };

    assign s_rvalid = s_rvalid_int;
    assign s_rid    = r_id_sel;
    assign s_rdata  = r_take1 ? m_rdata[(1*DATA_WIDTH) +: DATA_WIDTH] : m_rdata[(0*DATA_WIDTH) +: DATA_WIDTH];
    assign s_rresp  = r_take1 ? m_rresp[(1*2) +: 2] : m_rresp[(0*2) +: 2];
    assign s_rlast  = r_last_sel;
    assign m_rready = { (r_take1 && s_rready), (r_take0 && s_rready) };

    integer i;
    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            rr_b_sel <= 1'b0;
            rr_r_sel <= 1'b0;
            for (i = 0; i < ID_COUNT; i = i + 1) begin
                wr_tgt[i]  <= 1'b0;
                wr_outs[i] <= {CNT_WIDTH{1'b0}};
                rd_tgt[i]  <= 1'b0;
                rd_outs[i] <= {CNT_WIDTH{1'b0}};
            end
        end else begin
            if (aw_hs) begin
                if (!wr_has_outstanding) begin
                    wr_tgt[s_awid] <= aw_tgt;
                end
                wr_outs[s_awid] <= wr_outs[s_awid] + {{(CNT_WIDTH-1){1'b0}}, 1'b1};
            end

            if (b_hs) begin
                wr_outs[b_id_sel] <= wr_outs[b_id_sel] - {{(CNT_WIDTH-1){1'b0}}, 1'b1};
            end

            if (ar_hs) begin
                if (!rd_has_outstanding) begin
                    rd_tgt[s_arid] <= ar_tgt;
                end
                rd_outs[s_arid] <= rd_outs[s_arid] + {{(CNT_WIDTH-1){1'b0}}, 1'b1};
            end

            if (r_hs && r_last_sel) begin
                rd_outs[r_id_sel] <= rd_outs[r_id_sel] - {{(CNT_WIDTH-1){1'b0}}, 1'b1};
            end

            if (b_hs && m_bvalid[0] && m_bvalid[1]) begin
                rr_b_sel <= ~rr_b_sel;
            end

            if (r_hs && m_rvalid[0] && m_rvalid[1]) begin
                rr_r_sel <= ~rr_r_sel;
            end
        end
    end

endmodule
