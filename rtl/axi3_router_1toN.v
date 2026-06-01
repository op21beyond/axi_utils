// -----------------------------------------------------------------------------
// Module      : axi3_router_1toN
// Date        : 2026-05-27
// Version     : v1.0.0
// Author      : Jongchul Shin
// Function    : Single AXI3 slave to N AXI3 master router.
//               AW/AR are routed by one-hot selection inputs (aw_sel/ar_sel).
//               W/B/R channels are returned in accepted-order with ID restore.
// Assumptions : aw_sel/ar_sel are one-hot when corresponding valid is high.
//               AXI3 W channel ordering follows AW acceptance order.
//               LOCK/EXCLUSIVE are not handled. ERROR response treated normally.
// Notes       : axprot/axcache are forwarded only (no policy checks).
// -----------------------------------------------------------------------------
module axi3_router_1toN #(
    parameter integer N                 = 8,                 // Number of routed master ports
    parameter integer ADDR_WIDTH        = 32,                // Address bus width
    parameter integer DATA_WIDTH        = 32,                // Data bus width
    parameter integer STRB_WIDTH        = DATA_WIDTH / 8,    // Write strobe width
    parameter integer ID_WIDTH          = 4,                 // AXI slave ID width
    parameter integer OUTSTANDING_DEPTH = 16                 // Internal ordering FIFO depth
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

    output wire [N-1:0]                 m_awvalid,
    input  wire [N-1:0]                 m_awready,
    output wire [N*ADDR_WIDTH-1:0]      m_awaddr,
    output wire [N*4-1:0]               m_awlen,
    output wire [N*3-1:0]               m_awsize,
    output wire [N*2-1:0]               m_awburst,
    output wire [N*2-1:0]               m_awlock,
    output wire [N*4-1:0]               m_awcache,
    output wire [N*3-1:0]               m_awprot,

    output wire [N-1:0]                 m_wvalid,
    input  wire [N-1:0]                 m_wready,
    output wire [N*DATA_WIDTH-1:0]      m_wdata,
    output wire [N*STRB_WIDTH-1:0]      m_wstrb,
    output wire [N-1:0]                 m_wlast,

    input  wire [N-1:0]                 m_bvalid,
    output wire [N-1:0]                 m_bready,
    input  wire [N*2-1:0]               m_bresp,

    output wire [N-1:0]                 m_arvalid,
    input  wire [N-1:0]                 m_arready,
    output wire [N*ADDR_WIDTH-1:0]      m_araddr,
    output wire [N*4-1:0]               m_arlen,
    output wire [N*3-1:0]               m_arsize,
    output wire [N*2-1:0]               m_arburst,
    output wire [N*2-1:0]               m_arlock,
    output wire [N*4-1:0]               m_arcache,
    output wire [N*3-1:0]               m_arprot,

    input  wire [N-1:0]                 m_rvalid,
    output wire [N-1:0]                 m_rready,
    input  wire [N*DATA_WIDTH-1:0]      m_rdata,
    input  wire [N*2-1:0]               m_rresp,
    input  wire [N-1:0]                 m_rlast
);

    localparam integer SEL_W = (N <= 2) ? 1 :
                               (N <= 4) ? 2 :
                               (N <= 8) ? 3 :
                               (N <= 16) ? 4 :
                               (N <= 32) ? 5 : 6;

    localparam integer PTR_W = (OUTSTANDING_DEPTH <= 2) ? 1 :
                               (OUTSTANDING_DEPTH <= 4) ? 2 :
                               (OUTSTANDING_DEPTH <= 8) ? 3 :
                               (OUTSTANDING_DEPTH <= 16) ? 4 :
                               (OUTSTANDING_DEPTH <= 32) ? 5 :
                               (OUTSTANDING_DEPTH <= 64) ? 6 : 7;

    reg [SEL_W-1:0] w_sel_fifo [0:OUTSTANDING_DEPTH-1];
    reg [SEL_W-1:0] b_sel_fifo [0:OUTSTANDING_DEPTH-1];
    reg [ID_WIDTH-1:0] b_id_fifo [0:OUTSTANDING_DEPTH-1];
    reg [SEL_W-1:0] r_sel_fifo [0:OUTSTANDING_DEPTH-1];
    reg [ID_WIDTH-1:0] r_id_fifo [0:OUTSTANDING_DEPTH-1];

    reg [PTR_W-1:0] w_wr_ptr, w_rd_ptr;
    reg [PTR_W-1:0] b_wr_ptr, b_rd_ptr;
    reg [PTR_W-1:0] r_wr_ptr, r_rd_ptr;
    reg [PTR_W:0]   w_count, b_count, r_count;

    wire w_fifo_empty = (w_count == 0);
    wire b_fifo_empty = (b_count == 0);
    wire r_fifo_empty = (r_count == 0);
    wire w_fifo_full  = (w_count == OUTSTANDING_DEPTH);
    wire b_fifo_full  = (b_count == OUTSTANDING_DEPTH);
    wire r_fifo_full  = (r_count == OUTSTANDING_DEPTH);

    function [SEL_W-1:0] onehot_to_idx;
        input [N-1:0] onehot;
        integer k;
        begin
            onehot_to_idx = {SEL_W{1'b0}};
            for (k = 0; k < N; k = k + 1) begin
                if (onehot[k]) begin
                    onehot_to_idx = k[SEL_W-1:0];
                end
            end
        end
    endfunction

    wire [SEL_W-1:0] aw_sel_idx = onehot_to_idx(aw_sel);
    wire [SEL_W-1:0] ar_sel_idx = onehot_to_idx(ar_sel);

    wire aw_hs = s_awvalid && s_awready;
    wire ar_hs = s_arvalid && s_arready;
    wire w_hs  = s_wvalid && s_wready;
    wire b_hs  = s_bvalid && s_bready;
    wire r_hs  = s_rvalid && s_rready;

    wire w_last_hs = w_hs && s_wlast;
    wire b_push    = w_last_hs && !w_fifo_empty && !b_fifo_full;

    integer fi;

    wire [SEL_W-1:0] w_cur_sel = w_sel_fifo[w_rd_ptr];
    wire [SEL_W-1:0] b_cur_sel = b_sel_fifo[b_rd_ptr];
    wire [SEL_W-1:0] r_cur_sel = r_sel_fifo[r_rd_ptr];

    // aw_sel/ar_sel: one-hot when s_awvalid/s_arvalid (assumed by environment)
    assign s_awready = !w_fifo_full && m_awready[aw_sel_idx];
    assign s_arready = !r_fifo_full && m_arready[ar_sel_idx];

    assign s_wready  = !w_fifo_empty && m_wready[w_cur_sel] &&
                       (!s_wlast || !b_fifo_full);

    assign s_bvalid  = !b_fifo_empty && m_bvalid[b_cur_sel];
    assign s_bid     = b_id_fifo[b_rd_ptr];
    assign s_bresp   = m_bresp[(b_cur_sel*2) +: 2];

    assign s_rvalid  = !r_fifo_empty && m_rvalid[r_cur_sel];
    assign s_rid     = r_id_fifo[r_rd_ptr];
    assign s_rdata   = m_rdata[(r_cur_sel*DATA_WIDTH) +: DATA_WIDTH];
    assign s_rresp   = m_rresp[(r_cur_sel*2) +: 2];
    assign s_rlast   = m_rlast[r_cur_sel];

    // Master payload: broadcast slave inputs (unselected ports ignore via valid/ready)
    assign m_awaddr  = {N{s_awaddr}};
    assign m_awlen   = {N{s_awlen}};
    assign m_awsize  = {N{s_awsize}};
    assign m_awburst = {N{s_awburst}};
    assign m_awlock  = {N{s_awlock}};
    assign m_awcache = {N{s_awcache}};
    assign m_awprot  = {N{s_awprot}};
    assign m_awvalid = (s_awvalid && !w_fifo_full) ? ({{(N-1){1'b0}}, 1'b1} << aw_sel_idx) :
                        {N{1'b0}};

    assign m_wdata   = {N{s_wdata}};
    assign m_wstrb   = {N{s_wstrb}};
    assign m_wlast   = {N{s_wlast}};
    assign m_wvalid  = (!w_fifo_empty && s_wvalid) ?
                       ({{(N-1){1'b0}}, 1'b1} << w_cur_sel) : {N{1'b0}};

    assign m_bready  = (!b_fifo_empty && s_bready) ?
                       ({{(N-1){1'b0}}, 1'b1} << b_cur_sel) : {N{1'b0}};

    assign m_araddr  = {N{s_araddr}};
    assign m_arlen   = {N{s_arlen}};
    assign m_arsize  = {N{s_arsize}};
    assign m_arburst = {N{s_arburst}};
    assign m_arlock  = {N{s_arlock}};
    assign m_arcache = {N{s_arcache}};
    assign m_arprot  = {N{s_arprot}};
    assign m_arvalid = (s_arvalid && !r_fifo_full) ?
                       ({{(N-1){1'b0}}, 1'b1} << ar_sel_idx) : {N{1'b0}};

    assign m_rready  = (!r_fifo_empty && s_rready) ?
                       ({{(N-1){1'b0}}, 1'b1} << r_cur_sel) : {N{1'b0}};

    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            w_wr_ptr <= {PTR_W{1'b0}};
            w_rd_ptr <= {PTR_W{1'b0}};
            b_wr_ptr <= {PTR_W{1'b0}};
            b_rd_ptr <= {PTR_W{1'b0}};
            r_wr_ptr <= {PTR_W{1'b0}};
            r_rd_ptr <= {PTR_W{1'b0}};
            w_count  <= {(PTR_W+1){1'b0}};
            b_count  <= {(PTR_W+1){1'b0}};
            r_count  <= {(PTR_W+1){1'b0}};
            for (fi = 0; fi < OUTSTANDING_DEPTH; fi = fi + 1) begin
                w_sel_fifo[fi] <= {SEL_W{1'b0}};
                b_sel_fifo[fi] <= {SEL_W{1'b0}};
                b_id_fifo[fi]  <= {ID_WIDTH{1'b0}};
                r_sel_fifo[fi] <= {SEL_W{1'b0}};
                r_id_fifo[fi]  <= {ID_WIDTH{1'b0}};
            end
        end else begin
            if (aw_hs) begin
                w_sel_fifo[w_wr_ptr] <= aw_sel_idx;
                w_wr_ptr <= (w_wr_ptr == OUTSTANDING_DEPTH-1) ? {PTR_W{1'b0}} :
                            (w_wr_ptr + 1'b1);
            end

            if (w_last_hs && !w_fifo_empty) begin
                w_rd_ptr <= (w_rd_ptr == OUTSTANDING_DEPTH-1) ? {PTR_W{1'b0}} :
                            (w_rd_ptr + 1'b1);
            end

            if (b_push) begin
                b_sel_fifo[b_wr_ptr] <= w_cur_sel;
                b_id_fifo[b_wr_ptr]  <= s_wid;
                b_wr_ptr <= (b_wr_ptr == OUTSTANDING_DEPTH-1) ? {PTR_W{1'b0}} :
                            (b_wr_ptr + 1'b1);
            end

            if (b_hs && !b_fifo_empty) begin
                b_rd_ptr <= (b_rd_ptr == OUTSTANDING_DEPTH-1) ? {PTR_W{1'b0}} :
                            (b_rd_ptr + 1'b1);
            end

            case ({aw_hs, (w_last_hs && !w_fifo_empty)})
                2'b10: w_count <= w_count + 1'b1;
                2'b01: w_count <= w_count - 1'b1;
                default: w_count <= w_count;
            endcase

            case ({b_push, (b_hs && !b_fifo_empty)})
                2'b10: b_count <= b_count + 1'b1;
                2'b01: b_count <= b_count - 1'b1;
                default: b_count <= b_count;
            endcase

            if (ar_hs) begin
                r_sel_fifo[r_wr_ptr] <= ar_sel_idx;
                r_id_fifo[r_wr_ptr]  <= s_arid;
                r_wr_ptr <= (r_wr_ptr == OUTSTANDING_DEPTH-1) ? {PTR_W{1'b0}} :
                            (r_wr_ptr + 1'b1);
            end

            if (r_hs && s_rlast && !r_fifo_empty) begin
                r_rd_ptr <= (r_rd_ptr == OUTSTANDING_DEPTH-1) ? {PTR_W{1'b0}} :
                            (r_rd_ptr + 1'b1);
            end

            case ({ar_hs, (r_hs && s_rlast && !r_fifo_empty)})
                2'b10: r_count <= r_count + 1'b1;
                2'b01: r_count <= r_count - 1'b1;
                default: r_count <= r_count;
            endcase
        end
    end

    // synopsys translate_off
    // Simulation-only: aw_sel/ar_sel must be one-hot when valid is asserted.
    always @(posedge aclk or negedge aresetn) begin
        integer aw_sel_k;
        integer ar_sel_k;
        integer aw_sel_cnt;
        integer ar_sel_cnt;

        if (aresetn) begin
            if (s_awvalid) begin
                aw_sel_cnt = 0;
                for (aw_sel_k = 0; aw_sel_k < N; aw_sel_k = aw_sel_k + 1) begin
                    if (aw_sel[aw_sel_k])
                        aw_sel_cnt = aw_sel_cnt + 1;
                end
                if (aw_sel_cnt != 1)
                    $error("%m: axi3_router_1toN: aw_sel must be one-hot when s_awvalid");
            end

            if (s_arvalid) begin
                ar_sel_cnt = 0;
                for (ar_sel_k = 0; ar_sel_k < N; ar_sel_k = ar_sel_k + 1) begin
                    if (ar_sel[ar_sel_k])
                        ar_sel_cnt = ar_sel_cnt + 1;
                end
                if (ar_sel_cnt != 1)
                    $error("%m: axi3_router_1toN: ar_sel must be one-hot when s_arvalid");
            end
        end
    end
    // synopsys translate_on

endmodule
