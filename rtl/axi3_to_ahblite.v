// -----------------------------------------------------------------------------
// Module      : axi3_to_ahblite
// Date        : 2026-05-27
// Version     : v1.1.0
// Author      : Jongchul Shin
// Function    : AXI3 (no ID) to AHB-Lite bridge.
//               AXI bursts are decomposed into pipelined AHB single transfers.
//               Read/Write issue arbitration uses round-robin policy.
// Assumptions : Fixed 32-bit data bus (WORD-only accesses by system contract).
//               Size/align/fixed/wstrb checks are not enforced in logic.
//               No LOCK/EXCLUSIVE behavior. ERROR response mapped to OKAY.
// Notes       : AHB outputs SINGLE transfers only (hburst=000; htrans=IDLE/NONSEQ).
//               Split-ready slaves: address-phase hready starts the beat; data-phase
//               hready completes it (ahb_data_pend). AXI cache/lock/prot not used.
// -----------------------------------------------------------------------------
module axi3_to_ahblite #(
    parameter integer ADDR_WIDTH = 32,                // Address bus width
    parameter integer WR_CMD_DEPTH = 16,              // Write command FIFO depth (beat-level)
    parameter integer RD_CMD_DEPTH = 16,              // Read command FIFO depth (beat-level)
    parameter integer RESP_DEPTH = 8                  // Read response FIFO depth
) (
    input  wire                     aclk,
    input  wire                     aresetn,

    // AXI3 slave side (no ID/USER/QOS), 32-bit data
    input  wire                     s_awvalid,
    output wire                     s_awready,
    input  wire [ADDR_WIDTH-1:0]    s_awaddr,
    input  wire [3:0]               s_awlen,
    input  wire [2:0]               s_awsize,
    input  wire [1:0]               s_awburst,
    input  wire [1:0]               s_awlock,
    input  wire [3:0]               s_awcache,
    input  wire [2:0]               s_awprot,

    input  wire                     s_wvalid,
    output wire                     s_wready,
    input  wire [31:0]              s_wdata,
    input  wire [3:0]               s_wstrb,
    input  wire                     s_wlast,

    output wire                     s_bvalid,
    input  wire                     s_bready,
    output wire [1:0]               s_bresp,

    input  wire                     s_arvalid,
    output wire                     s_arready,
    input  wire [ADDR_WIDTH-1:0]    s_araddr,
    input  wire [3:0]               s_arlen,
    input  wire [2:0]               s_arsize,
    input  wire [1:0]               s_arburst,
    input  wire [1:0]               s_arlock,
    input  wire [3:0]               s_arcache,
    input  wire [2:0]               s_arprot,

    output wire                     s_rvalid,
    input  wire                     s_rready,
    output wire [31:0]              s_rdata,
    output wire [1:0]               s_rresp,
    output wire                     s_rlast,

    // AHB-Lite master side
    output reg  [ADDR_WIDTH-1:0]    haddr,
    output reg  [1:0]               htrans,
    output reg                      hwrite,
    output reg  [2:0]               hsize,
    output reg  [2:0]               hburst,
    output reg  [31:0]              hwdata,
    input  wire [31:0]              hrdata,
    input  wire                     hready,
    input  wire                     hresp
);

    localparam integer WR_PTR_W = (WR_CMD_DEPTH <= 2) ? 1 :
                                  (WR_CMD_DEPTH <= 4) ? 2 :
                                  (WR_CMD_DEPTH <= 8) ? 3 :
                                  (WR_CMD_DEPTH <= 16) ? 4 : 5;
    localparam integer RD_PTR_W = (RD_CMD_DEPTH <= 2) ? 1 :
                                  (RD_CMD_DEPTH <= 4) ? 2 :
                                  (RD_CMD_DEPTH <= 8) ? 3 :
                                  (RD_CMD_DEPTH <= 16) ? 4 : 5;
    localparam integer RESP_PTR_W = (RESP_DEPTH <= 2) ? 1 :
                                    (RESP_DEPTH <= 4) ? 2 :
                                    (RESP_DEPTH <= 8) ? 3 :
                                    (RESP_DEPTH <= 16) ? 4 : 5;
    localparam [1:0] AHB_IDLE   = 2'b00;
    localparam [1:0] AHB_NONSEQ = 2'b10;
    // Word-aligned burst stride (32-bit data path by design contract)
    localparam [ADDR_WIDTH-1:0] BEAT_ADDR_INCR = 32'd4;

    reg [ADDR_WIDTH-1:0]   wr_addr_fifo  [0:WR_CMD_DEPTH-1];
    reg [31:0]             wr_data_fifo  [0:WR_CMD_DEPTH-1];
    reg [WR_PTR_W-1:0]     wr_wr_ptr, wr_rd_ptr;
    reg [WR_PTR_W:0]       wr_count;
    reg [ADDR_WIDTH-1:0]   rd_addr_fifo  [0:RD_CMD_DEPTH-1];
    reg [RD_PTR_W-1:0]     rd_wr_ptr, rd_rd_ptr;
    reg [RD_PTR_W:0]       rd_count;
    reg [31:0]             rdata_fifo [0:RESP_DEPTH-1];
    reg                    rlast_fifo [0:RESP_DEPTH-1];
    reg [RESP_PTR_W-1:0]   r_wr_ptr, r_rd_ptr;
    reg [RESP_PTR_W:0]     r_count;
    reg                    b_pending;
    integer                fi;
    reg                    ahb_data_pend; // 1: address accepted, waiting for data-phase hready
    reg                    pend_write;
    reg                    pend_rlast;
    reg [31:0]             wr_data_pend;
    reg                    rr_q;
    reg                    wr_active;
    reg [ADDR_WIDTH-1:0]   wr_addr_q;
    reg [4:0]              wr_beats_left_q;
    reg [4:0]              wr_total_beats_q;
    reg [4:0]              wr_done_beats_q;
    reg                    rd_active;
    reg [ADDR_WIDTH-1:0]   rd_addr_q;
    reg [4:0]              rd_beats_left_q;
    reg [4:0]              rd_total_beats_q;
    reg [4:0]              rd_done_beats_q;
    reg [4:0]              rd_pop_beats_q;

    wire wr_full  = (wr_count == WR_CMD_DEPTH);
    wire wr_empty = (wr_count == 0);
    wire rd_full  = (rd_count == RD_CMD_DEPTH);
    wire rd_empty = (rd_count == 0);
    wire r_empty   = (r_count == 0);
    wire r_full    = (r_count == RESP_DEPTH);
    wire b_empty   = !b_pending;
    wire aw_take = s_awvalid && s_awready;
    wire ar_take = s_arvalid && s_arready;
    wire w_take  = s_wvalid && s_wready;
    wire wr_issue_valid = !wr_empty;
    wire rd_issue_valid = !rd_empty && !r_full;
    wire choose_wr = wr_issue_valid && (!rd_issue_valid || !rr_q);
    wire choose_rd = rd_issue_valid && (!wr_issue_valid || rr_q);
    wire ahb_idle    = !ahb_data_pend;
    wire complete_data  = ahb_data_pend && hready;
    wire complete_write = complete_data && pend_write;
    wire complete_read  = complete_data && !pend_write;
    // New beat: address-phase hready while bus idle, or back-to-back on data-phase hready
    wire can_issue   = hready && (ahb_idle || complete_data);
    wire issue       = can_issue && (choose_wr || choose_rd);
    wire both_valid  = wr_issue_valid && rd_issue_valid;
    wire issue_wr    = issue && choose_wr;
    wire issue_rd    = issue && choose_rd;

    wire b_take = s_bvalid && s_bready;
    wire r_take = s_rvalid && s_rready;
    assign s_awready = !wr_active && !b_pending &&
                       (wr_done_beats_q == wr_total_beats_q);
    assign s_wready  = wr_active && !wr_full;
    assign s_arready = !rd_active && (rd_pop_beats_q == rd_total_beats_q) && !aw_take;
    assign s_bvalid = !b_empty;
    assign s_bresp  = 2'b00; // AHB ERROR is treated as AXI OKAY

    assign s_rvalid = !r_empty;
    assign s_rresp  = 2'b00; // AHB ERROR is treated as AXI OKAY
    assign s_rdata  = rdata_fifo[r_rd_ptr];
    assign s_rlast  = rlast_fifo[r_rd_ptr];

    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            wr_wr_ptr <= {WR_PTR_W{1'b0}};
            wr_rd_ptr <= {WR_PTR_W{1'b0}};
            wr_count  <= {(WR_PTR_W+1){1'b0}};
            rd_wr_ptr <= {RD_PTR_W{1'b0}};
            rd_rd_ptr <= {RD_PTR_W{1'b0}};
            rd_count  <= {(RD_PTR_W+1){1'b0}};
            r_wr_ptr <= {RESP_PTR_W{1'b0}};
            r_rd_ptr <= {RESP_PTR_W{1'b0}};
            r_count  <= {(RESP_PTR_W+1){1'b0}};
            b_pending <= 1'b0;
            ahb_data_pend <= 1'b0;
            pend_write <= 1'b0;
            pend_rlast <= 1'b0;
            rr_q <= 1'b0;
            for (fi = 0; fi < WR_CMD_DEPTH; fi = fi + 1) begin
                wr_addr_fifo[fi] <= {ADDR_WIDTH{1'b0}};
                wr_data_fifo[fi] <= 32'd0;
            end
            for (fi = 0; fi < RD_CMD_DEPTH; fi = fi + 1) begin
                rd_addr_fifo[fi] <= {ADDR_WIDTH{1'b0}};
            end
            for (fi = 0; fi < RESP_DEPTH; fi = fi + 1) begin
                rdata_fifo[fi] <= 32'd0;
                rlast_fifo[fi] <= 1'b0;
            end
            wr_active        <= 1'b0;
            wr_addr_q        <= {ADDR_WIDTH{1'b0}};
            wr_beats_left_q  <= 5'd0;
            wr_total_beats_q <= 5'd0;
            wr_done_beats_q  <= 5'd0;
            rd_active        <= 1'b0;
            rd_addr_q        <= {ADDR_WIDTH{1'b0}};
            rd_beats_left_q  <= 5'd0;
            rd_total_beats_q <= 5'd0;
            rd_done_beats_q  <= 5'd0;
            rd_pop_beats_q   <= 5'd0;
            haddr  <= {ADDR_WIDTH{1'b0}};
            htrans <= AHB_IDLE;
            hwrite <= 1'b0;
            hsize  <= 3'b010;
            hburst <= 3'b000;
            hwdata <= 32'b0;
            wr_data_pend <= 32'b0;
        end else begin
            if (both_valid && issue) begin
                rr_q <= ~rr_q;
            end
            // Write burst acceptance (AXI burst supported, FIXED/align/size assumptions by design contract)
            if (aw_take) begin
                wr_active        <= 1'b1;
                wr_addr_q        <= s_awaddr;
                wr_beats_left_q  <= s_awlen + 5'd1;
                wr_total_beats_q <= s_awlen + 5'd1;
                wr_done_beats_q  <= 5'd0;
            end
            if (w_take) begin
                wr_addr_fifo[wr_wr_ptr] <= wr_addr_q;
                wr_data_fifo[wr_wr_ptr] <= s_wdata;
                wr_wr_ptr <= (wr_wr_ptr == WR_CMD_DEPTH-1) ? {WR_PTR_W{1'b0}} : (wr_wr_ptr + 1'b1);
                wr_addr_q <= wr_addr_q + BEAT_ADDR_INCR;
                wr_beats_left_q <= wr_beats_left_q - 5'd1;
                if (wr_beats_left_q == 5'd1) begin
                    wr_active <= 1'b0;
                end
            end
            // Read burst acceptance and beat generation (single AHB transfers)
            if (ar_take) begin
                rd_active        <= 1'b1;
                rd_addr_q        <= s_araddr;
                rd_beats_left_q  <= s_arlen + 5'd1;
                rd_total_beats_q <= s_arlen + 5'd1;
                rd_done_beats_q  <= 5'd0;
                rd_pop_beats_q   <= 5'd0;
            end
            if (rd_active && !rd_full) begin
                rd_addr_fifo[rd_wr_ptr] <= rd_addr_q;
                rd_wr_ptr <= (rd_wr_ptr == RD_CMD_DEPTH-1) ? {RD_PTR_W{1'b0}} : (rd_wr_ptr + 1'b1);
                rd_addr_q <= rd_addr_q + BEAT_ADDR_INCR;
                rd_beats_left_q <= rd_beats_left_q - 5'd1;
                if (rd_beats_left_q == 5'd1) begin
                    rd_active <= 1'b0;
                end
            end
            if (issue_wr) begin
                wr_rd_ptr <= (wr_rd_ptr == WR_CMD_DEPTH-1) ? {WR_PTR_W{1'b0}} : (wr_rd_ptr + 1'b1);
            end
            if (issue_rd) begin
                rd_rd_ptr <= (rd_rd_ptr == RD_CMD_DEPTH-1) ? {RD_PTR_W{1'b0}} : (rd_rd_ptr + 1'b1);
            end
            case ({w_take, issue_wr})
                2'b10: wr_count <= wr_count + 1'b1;
                2'b01: wr_count <= wr_count - 1'b1;
                default: wr_count <= wr_count;
            endcase
            case ({(rd_active && !rd_full), issue_rd})
                2'b10: rd_count <= rd_count + 1'b1;
                2'b01: rd_count <= rd_count - 1'b1;
                default: rd_count <= rd_count;
            endcase
            if (complete_data) begin
                ahb_data_pend <= 1'b0;
            end
            if (complete_write) begin
                wr_done_beats_q <= wr_done_beats_q + 5'd1;
                if ((wr_done_beats_q + 5'd1) == wr_total_beats_q) begin
                    b_pending <= 1'b1;
                end
            end
            if (b_take) begin
                b_pending <= 1'b0;
            end
            if (complete_read) begin
                rdata_fifo[r_wr_ptr] <= hrdata;
                rlast_fifo[r_wr_ptr] <= pend_rlast;
                r_wr_ptr <= (r_wr_ptr == RESP_DEPTH-1) ? {RESP_PTR_W{1'b0}} : (r_wr_ptr + 1'b1);
                rd_done_beats_q <= rd_done_beats_q + 5'd1;
            end
            if (r_take) begin
                r_rd_ptr <= (r_rd_ptr == RESP_DEPTH-1) ? {RESP_PTR_W{1'b0}} : (r_rd_ptr + 1'b1);
                if (rd_pop_beats_q != rd_total_beats_q) begin
                    rd_pop_beats_q <= rd_pop_beats_q + 5'd1;
                end
            end
            case ({complete_read, r_take})
                2'b10: r_count <= r_count + 1'b1;
                2'b01: r_count <= r_count - 1'b1;
                default: r_count <= r_count;
            endcase
            if (issue) begin
                ahb_data_pend <= 1'b1;
                if (issue_wr) begin
                    pend_write <= 1'b1;
                    pend_rlast <= 1'b0;
                end else begin
                    pend_write <= 1'b0;
                    pend_rlast <= ((rd_done_beats_q + (complete_read ? 5'd2 : 5'd1))
                                   == rd_total_beats_q);
                end
            end
            if (hready) begin
                if (issue_wr) begin
                    haddr  <= wr_addr_fifo[wr_rd_ptr];
                    hwrite <= 1'b1;
                    hsize  <= 3'b010; // word only by design assumption
                    hburst <= 3'b000; // AHB single only
                    htrans <= AHB_NONSEQ;
                    wr_data_pend <= wr_data_fifo[wr_rd_ptr];
                    hwdata       <= wr_data_fifo[wr_rd_ptr];
                end else if (issue_rd) begin
                    haddr  <= rd_addr_fifo[rd_rd_ptr];
                    hwrite <= 1'b0;
                    hsize  <= 3'b010;
                    hburst <= 3'b000;
                    htrans <= AHB_NONSEQ;
                end else if (ahb_idle) begin
                    hwrite <= 1'b0;
                    hsize  <= 3'b010;
                    hburst <= 3'b000;
                    htrans <= AHB_IDLE;
                end
            end
        end
    end

    // Explicitly unused by design constraints / requested behavior.
    wire _unused_meta = ^{
        hresp,
        s_awcache, s_awlock, s_awprot,
        s_arcache, s_arlock, s_arprot,
        s_awsize, s_awburst,
        s_wstrb, s_wlast,
        s_arsize, s_arburst
    };
    // synopsys translate_off
    // Simulation-only: verify master/environment meets module assumptions.
    reg [4:0] sim_w_beats_left;

    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            sim_w_beats_left <= 5'd0;
        end else begin
            if (aw_take) begin
                if (s_awsize !== 3'b010)
                    $error("%m: axi3_to_ahblite: AWSIZE must be 32-bit word (3'b010), got %b", s_awsize);
                if (s_awlock !== 2'b00)
                    $error("%m: axi3_to_ahblite: AWLOCK must be normal (2'b00), got %b", s_awlock);
                if (s_awaddr[1:0] !== 2'b00)
                    $error("%m: axi3_to_ahblite: AWADDR must be word-aligned, got 0x%h", s_awaddr);
                if (s_awlen > 4'd15)
                    $error("%m: axi3_to_ahblite: AWLEN exceeds AXI3 maximum (15)");
                if ((s_awlen != 4'd0) && (s_awburst !== 2'b01))
                    $error("%m: axi3_to_ahblite: multi-beat AW burst must be INCR (2'b01), got %b", s_awburst);
                sim_w_beats_left <= s_awlen + 5'd1;
            end
            if (ar_take) begin
                if (s_arsize !== 3'b010)
                    $error("%m: axi3_to_ahblite: ARSIZE must be 32-bit word (3'b010), got %b", s_arsize);
                if (s_arlock !== 2'b00)
                    $error("%m: axi3_to_ahblite: ARLOCK must be normal (2'b00), got %b", s_arlock);
                if (s_araddr[1:0] !== 2'b00)
                    $error("%m: axi3_to_ahblite: ARADDR must be word-aligned, got 0x%h", s_araddr);
                if (s_arlen > 4'd15)
                    $error("%m: axi3_to_ahblite: ARLEN exceeds AXI3 maximum (15)");
                if ((s_arlen != 4'd0) && (s_arburst !== 2'b01))
                    $error("%m: axi3_to_ahblite: multi-beat AR burst must be INCR (2'b01), got %b", s_arburst);
            end
            if (aw_take && ar_take)
                $error("%m: axi3_to_ahblite: AW and AR must not handshake in the same cycle");
            if (s_wvalid && !wr_active)
                $error("%m: axi3_to_ahblite: WVALID asserted outside active write burst");
            if (w_take) begin
                if (s_wstrb !== 4'b1111)
                    $error("%m: axi3_to_ahblite: WSTRB must be full word (4'b1111), got %b", s_wstrb);
                if (wr_addr_q[1:0] !== 2'b00)
                    $error("%m: axi3_to_ahblite: write beat address must stay word-aligned, got 0x%h", wr_addr_q);
                if (sim_w_beats_left == 5'd0)
                    $error("%m: axi3_to_ahblite: unexpected W beat (no beats remaining)");
                if ((sim_w_beats_left == 5'd1) && !s_wlast)
                    $error("%m: axi3_to_ahblite: WLAST required on final W beat");
                if ((sim_w_beats_left != 5'd1) && s_wlast)
                    $error("%m: axi3_to_ahblite: WLAST must only assert on final W beat");
                sim_w_beats_left <= sim_w_beats_left - 5'd1;
            end
            if (s_arvalid && s_arready && rd_active && (rd_pop_beats_q != rd_total_beats_q))
                $error("%m: axi3_to_ahblite: new AR before prior read burst R data fully returned");
        end
    end
    // synopsys translate_on

endmodule
