// -----------------------------------------------------------------------------
// Module      : axi3_aw_w_order_gate
// Date        : 2026-05-29
// Version     : v1.1.0
// Author      : Jongchul Shin
// Function    : Per-port AXI3 write data gating so W beats are accepted only
//               after at least one AW handshake (AW-before-W ordering).
//               awready_o backpressures AW when pending is saturated.
//               Pending count increments on AW handshake and decrements on W
//               handshake with WLAST.
// Assumptions : Downstream router drives awready_i; awready_o is the master-visible
//               ready. Each port does not interleave W beats for a given flow.
// Notes       : W channel only is gated (wvalid_o, wready). Simulation-only checks
//               flag WVALID with no pending AW.
// -----------------------------------------------------------------------------
module axi3_aw_w_order_gate #(
    parameter integer PENDING_DEPTH = 16
) (
    input  wire aclk,
    input  wire aresetn,

    input  wire awvalid,
    input  wire awready_i,
    output wire awready_o,

    input  wire wvalid,
    output wire wready,
    input  wire wlast,

    output wire wvalid_o,
    input  wire wready_i
);

    localparam integer CNT_W = (PENDING_DEPTH <= 1) ? 1 : $clog2(PENDING_DEPTH + 1);

    reg [CNT_W-1:0] pending_cnt;

    wire pending_full = (pending_cnt >= PENDING_DEPTH);
    wire awready_o_int = awready_i && !pending_full;
    wire aw_hs = awvalid && awready_o_int;
    wire w_gate = |pending_cnt;
    wire w_hs   = wvalid_o && wready && wlast;

    wire aw_inc = aw_hs;

    assign awready_o = awready_o_int;
    assign wvalid_o  = wvalid && w_gate;
    assign wready    = w_gate && wready_i;

    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            pending_cnt <= {CNT_W{1'b0}};
        end else begin
            case ({aw_inc, w_hs})
                2'b10: pending_cnt <= pending_cnt + {{(CNT_W-1){1'b0}}, 1'b1};
                2'b01: pending_cnt <= pending_cnt - {{(CNT_W-1){1'b0}}, 1'b1};
                default: ;
            endcase
        end
    end

    // synopsys translate_off
    always @(posedge aclk or negedge aresetn) begin
        if (aresetn) begin
            if (wvalid && !w_gate)
                $error("%m: axi3_aw_w_order_gate: WVALID asserted with no pending AW (gate closed)");
        end
    end
    // synopsys translate_on

endmodule
