// -----------------------------------------------------------------------------
// Module      : axi3_merge_Nto1_128
// Date        : 2026-05-29
// Version     : v1.1.0
// Author      : Jongchul Shin
// Function    : Merge N AXI3 (128-bit) inputs into a single AXI3 output.
//               AW/AR use round-robin; W follows AW acceptance order via FIFO.
//               Output ID is widened with source index bits.
// Assumptions : USER/QOS signals are not present.
//               LOCK/CACHE/PROT are routed without special behavior.
//               Each input port does not interleave W beats (AW-before-W per port).
// Notes       : B/R channels are de-multiplexed using widened BID/RID source bits.
//               Payload fields are don't-care when valid is low (no default mux).
//               AW is backpressured when the W ordering FIFO is full.
// -----------------------------------------------------------------------------
module axi3_merge_Nto1_128 #(
    parameter integer N                    = 2,                 // Number of AXI input ports
    parameter integer ADDR_WIDTH             = 32,                // AXI address width
    parameter integer DATA_WIDTH             = 128,               // AXI data width (target: 128-bit)
    parameter integer STRB_WIDTH             = DATA_WIDTH / 8,    // AXI strobe width
    parameter integer IN_ID_WIDTH            = 4,                 // Input AXI ID width per source
    parameter integer WR_OUTSTANDING_DEPTH   = 16,                // W ordering FIFO depth
    parameter integer SRC_ID_WIDTH           = (N <= 2)  ? 1 :
                                               (N <= 4)  ? 2 :
                                               (N <= 8)  ? 3 :
                                               (N <= 16) ? 4 :
                                               (N <= 32) ? 5 : 6  // Source-index width
) (
    input  wire                             aclk,
    input  wire                             aresetn,

    input  wire [N-1:0]                     s_awvalid,
    output wire [N-1:0]                     s_awready,
    input  wire [N*IN_ID_WIDTH-1:0]         s_awid,
    input  wire [N*ADDR_WIDTH-1:0]          s_awaddr,
    input  wire [N*4-1:0]                   s_awlen,
    input  wire [N*3-1:0]                   s_awsize,
    input  wire [N*2-1:0]                   s_awburst,
    input  wire [N*2-1:0]                   s_awlock,
    input  wire [N*4-1:0]                   s_awcache,
    input  wire [N*3-1:0]                   s_awprot,

    input  wire [N-1:0]                     s_wvalid,
    output wire [N-1:0]                     s_wready,
    input  wire [N*IN_ID_WIDTH-1:0]         s_wid,
    input  wire [N*DATA_WIDTH-1:0]          s_wdata,
    input  wire [N*STRB_WIDTH-1:0]          s_wstrb,
    input  wire [N-1:0]                     s_wlast,

    output wire [N-1:0]                     s_bvalid,
    input  wire [N-1:0]                     s_bready,
    output wire [N*IN_ID_WIDTH-1:0]         s_bid,
    output wire [N*2-1:0]                   s_bresp,

    input  wire [N-1:0]                     s_arvalid,
    output wire [N-1:0]                     s_arready,
    input  wire [N*IN_ID_WIDTH-1:0]         s_arid,
    input  wire [N*ADDR_WIDTH-1:0]          s_araddr,
    input  wire [N*4-1:0]                   s_arlen,
    input  wire [N*3-1:0]                   s_arsize,
    input  wire [N*2-1:0]                   s_arburst,
    input  wire [N*2-1:0]                   s_arlock,
    input  wire [N*4-1:0]                   s_arcache,
    input  wire [N*3-1:0]                   s_arprot,

    output wire [N-1:0]                     s_rvalid,
    input  wire [N-1:0]                     s_rready,
    output wire [N*IN_ID_WIDTH-1:0]         s_rid,
    output wire [N*DATA_WIDTH-1:0]          s_rdata,
    output wire [N*2-1:0]                   s_rresp,
    output wire [N-1:0]                     s_rlast,

    output wire                             m_awvalid,
    input  wire                             m_awready,
    output wire [IN_ID_WIDTH+SRC_ID_WIDTH-1:0] m_awid,
    output wire [ADDR_WIDTH-1:0]            m_awaddr,
    output wire [3:0]                       m_awlen,
    output wire [2:0]                       m_awsize,
    output wire [1:0]                       m_awburst,
    output wire [1:0]                       m_awlock,
    output wire [3:0]                       m_awcache,
    output wire [2:0]                       m_awprot,

    output wire                             m_wvalid,
    input  wire                             m_wready,
    output wire [IN_ID_WIDTH+SRC_ID_WIDTH-1:0] m_wid,
    output wire [DATA_WIDTH-1:0]            m_wdata,
    output wire [STRB_WIDTH-1:0]            m_wstrb,
    output wire                             m_wlast,

    input  wire                             m_bvalid,
    output wire                             m_bready,
    input  wire [IN_ID_WIDTH+SRC_ID_WIDTH-1:0] m_bid,
    input  wire [1:0]                       m_bresp,

    output wire                             m_arvalid,
    input  wire                             m_arready,
    output wire [IN_ID_WIDTH+SRC_ID_WIDTH-1:0] m_arid,
    output wire [ADDR_WIDTH-1:0]            m_araddr,
    output wire [3:0]                       m_arlen,
    output wire [2:0]                       m_arsize,
    output wire [1:0]                       m_arburst,
    output wire [1:0]                       m_arlock,
    output wire [3:0]                       m_arcache,
    output wire [2:0]                       m_arprot,

    input  wire                             m_rvalid,
    output wire                             m_rready,
    input  wire [IN_ID_WIDTH+SRC_ID_WIDTH-1:0] m_rid,
    input  wire [DATA_WIDTH-1:0]            m_rdata,
    input  wire [1:0]                       m_rresp,
    input  wire                             m_rlast
);

    localparam integer SRC_W = SRC_ID_WIDTH;
    localparam integer OUT_ID_WIDTH = IN_ID_WIDTH + SRC_W;

    localparam integer W_PTR_W = (WR_OUTSTANDING_DEPTH <= 2) ? 1 :
                                 (WR_OUTSTANDING_DEPTH <= 4) ? 2 :
                                 (WR_OUTSTANDING_DEPTH <= 8) ? 3 :
                                 (WR_OUTSTANDING_DEPTH <= 16) ? 4 :
                                 (WR_OUTSTANDING_DEPTH <= 32) ? 5 :
                                 (WR_OUTSTANDING_DEPTH <= 64) ? 6 : 7;

    reg [SRC_W-1:0] rr_aw_q, rr_ar_q;

    reg [SRC_W-1:0] w_sel_fifo [0:WR_OUTSTANDING_DEPTH-1];
    reg [W_PTR_W-1:0] w_wr_ptr, w_rd_ptr;
    reg [W_PTR_W:0]   w_count;

    wire w_fifo_empty = (w_count == 0);
    wire w_fifo_full  = (w_count == WR_OUTSTANDING_DEPTH);

    reg              aw_grant_v;
    reg [SRC_W-1:0]  aw_grant_i;
    wire             w_grant_v;
    wire [SRC_W-1:0] w_grant_i;
    reg              ar_grant_v;
    reg [SRC_W-1:0]  ar_grant_i;

    wire aw_hs     = aw_grant_v && m_awready && !w_fifo_full;
    wire w_hs      = w_grant_v && m_wready;
    wire w_last_hs = w_hs && m_wlast;

    wire [SRC_W-1:0] w_cur_sel = w_sel_fifo[w_rd_ptr];

    wire [SRC_W-1:0] b_src_i = m_bid[OUT_ID_WIDTH-1 -: SRC_W];
    wire [SRC_W-1:0] r_src_i = m_rid[OUT_ID_WIDTH-1 -: SRC_W];

    wire [IN_ID_WIDTH-1:0] aw_id_in = s_awid[(aw_grant_i*IN_ID_WIDTH) +: IN_ID_WIDTH];
    wire [IN_ID_WIDTH-1:0] w_id_in  = s_wid[(w_grant_i*IN_ID_WIDTH) +: IN_ID_WIDTH];
    wire [IN_ID_WIDTH-1:0] ar_id_in = s_arid[(ar_grant_i*IN_ID_WIDTH) +: IN_ID_WIDTH];

    integer k;
    integer idx;
    integer fi;

    always @(*) begin
        aw_grant_v = 1'b0;
        aw_grant_i = {SRC_W{1'b0}};
        for (k = 0; k < N; k = k + 1) begin
            idx = rr_aw_q + k;
            if (idx >= N) begin
                idx = idx - N;
            end
            if (!aw_grant_v && s_awvalid[idx]) begin
                aw_grant_v = 1'b1;
                aw_grant_i = idx[SRC_W-1:0];
            end
        end
    end

    assign w_grant_i = w_cur_sel;
    assign w_grant_v = !w_fifo_empty && s_wvalid[w_cur_sel];

    always @(*) begin
        ar_grant_v = 1'b0;
        ar_grant_i = {SRC_W{1'b0}};
        for (k = 0; k < N; k = k + 1) begin
            idx = rr_ar_q + k;
            if (idx >= N) begin
                idx = idx - N;
            end
            if (!ar_grant_v && s_arvalid[idx]) begin
                ar_grant_v = 1'b1;
                ar_grant_i = idx[SRC_W-1:0];
            end
        end
    end

    assign m_awvalid = aw_grant_v && !w_fifo_full;
    assign s_awready = (aw_grant_v && !w_fifo_full) ?
                       ({{(N-1){1'b0}}, 1'b1} << aw_grant_i) & {N{m_awready}} :
                       {N{1'b0}};

    assign m_awid    = {aw_grant_i, aw_id_in};
    assign m_awaddr  = s_awaddr[(aw_grant_i*ADDR_WIDTH) +: ADDR_WIDTH];
    assign m_awlen   = s_awlen[(aw_grant_i*4) +: 4];
    assign m_awsize  = s_awsize[(aw_grant_i*3) +: 3];
    assign m_awburst = s_awburst[(aw_grant_i*2) +: 2];
    assign m_awlock  = s_awlock[(aw_grant_i*2) +: 2];
    assign m_awcache = s_awcache[(aw_grant_i*4) +: 4];
    assign m_awprot  = s_awprot[(aw_grant_i*3) +: 3];

    assign m_wvalid = w_grant_v;
    assign s_wready = w_grant_v ?
                      ({{(N-1){1'b0}}, 1'b1} << w_grant_i) & {N{m_wready}} :
                      {N{1'b0}};

    assign m_wid   = {w_grant_i, w_id_in};
    assign m_wdata = s_wdata[(w_grant_i*DATA_WIDTH) +: DATA_WIDTH];
    assign m_wstrb = s_wstrb[(w_grant_i*STRB_WIDTH) +: STRB_WIDTH];
    assign m_wlast = s_wlast[w_grant_i];

    assign m_arvalid = ar_grant_v;
    assign s_arready = ar_grant_v ?
                       ({{(N-1){1'b0}}, 1'b1} << ar_grant_i) & {N{m_arready}} :
                       {N{1'b0}};

    assign m_arid    = {ar_grant_i, ar_id_in};
    assign m_araddr  = s_araddr[(ar_grant_i*ADDR_WIDTH) +: ADDR_WIDTH];
    assign m_arlen   = s_arlen[(ar_grant_i*4) +: 4];
    assign m_arsize  = s_arsize[(ar_grant_i*3) +: 3];
    assign m_arburst = s_arburst[(ar_grant_i*2) +: 2];
    assign m_arlock  = s_arlock[(ar_grant_i*2) +: 2];
    assign m_arcache = s_arcache[(ar_grant_i*4) +: 4];
    assign m_arprot  = s_arprot[(ar_grant_i*3) +: 3];

    assign s_bvalid = m_bvalid ? ({{(N-1){1'b0}}, 1'b1} << b_src_i) : {N{1'b0}};
    assign m_bready = m_bvalid ? s_bready[b_src_i] : 1'b0;
    assign s_bid    = {N{m_bid[IN_ID_WIDTH-1:0]}};
    assign s_bresp  = {N{m_bresp}};

    assign s_rvalid = m_rvalid ? ({{(N-1){1'b0}}, 1'b1} << r_src_i) : {N{1'b0}};
    assign m_rready = m_rvalid ? s_rready[r_src_i] : 1'b0;
    assign s_rid    = {N{m_rid[IN_ID_WIDTH-1:0]}};
    assign s_rdata  = {N{m_rdata}};
    assign s_rresp  = {N{m_rresp}};
    assign s_rlast  = {N{m_rlast}};

    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            rr_aw_q <= {SRC_W{1'b0}};
            rr_ar_q <= {SRC_W{1'b0}};
            w_wr_ptr <= {W_PTR_W{1'b0}};
            w_rd_ptr <= {W_PTR_W{1'b0}};
            w_count  <= {(W_PTR_W+1){1'b0}};
            for (fi = 0; fi < WR_OUTSTANDING_DEPTH; fi = fi + 1) begin
                w_sel_fifo[fi] <= {SRC_W{1'b0}};
            end
        end else begin
            if (aw_hs) begin
                w_sel_fifo[w_wr_ptr] <= aw_grant_i;
                w_wr_ptr <= (w_wr_ptr == WR_OUTSTANDING_DEPTH-1) ? {W_PTR_W{1'b0}} :
                            (w_wr_ptr + 1'b1);
            end

            if (w_last_hs && !w_fifo_empty) begin
                w_rd_ptr <= (w_rd_ptr == WR_OUTSTANDING_DEPTH-1) ? {W_PTR_W{1'b0}} :
                            (w_rd_ptr + 1'b1);
            end

            case ({aw_hs, (w_last_hs && !w_fifo_empty)})
                2'b10: w_count <= w_count + 1'b1;
                2'b01: w_count <= w_count - 1'b1;
                default: w_count <= w_count;
            endcase

            if (aw_hs) begin
                if (aw_grant_i == N-1) begin
                    rr_aw_q <= {SRC_W{1'b0}};
                end else begin
                    rr_aw_q <= aw_grant_i + {{(SRC_W-1){1'b0}}, 1'b1};
                end
            end
            if (ar_grant_v && m_arready) begin
                if (ar_grant_i == N-1) begin
                    rr_ar_q <= {SRC_W{1'b0}};
                end else begin
                    rr_ar_q <= ar_grant_i + {{(SRC_W-1){1'b0}}, 1'b1};
                end
            end
        end
    end

    // synopsys translate_off
    always @(posedge aclk or negedge aresetn) begin
        if (aresetn) begin
            if (aw_grant_v && m_awready && w_fifo_full)
                $error("%m: axi3_merge_Nto1_128: AW handshake while W ordering FIFO full");
            if (w_hs && w_fifo_empty)
                $error("%m: axi3_merge_Nto1_128: W beat without accepted AW in ordering FIFO");
        end
    end
    // synopsys translate_on

endmodule
