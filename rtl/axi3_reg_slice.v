// -----------------------------------------------------------------------------
// Module      : axi3_reg_slice
// Date        : 2026-05-28
// Version     : v1.0.0
// Author      : Jongchul Shin
// Function    : AXI3 channel register slice (depth-2 skid buffer, 1-cycle delay).
//               Splits long timing paths while preserving back-to-back throughput.
//               AW/W/B/AR/R channels are independently enabled or bypassed.
// Assumptions : Each channel uses a simple valid/ready/payload interface.
//               Payload packing (addr, burst, etc.) is done at instantiation.
// Notes       : Disabled channels are combinatorially bypassed.
// -----------------------------------------------------------------------------
module axi3_reg_slice #(
    parameter integer AW_PAYLOAD_WIDTH = 32,             // AW channel packed payload width
    parameter integer W_PAYLOAD_WIDTH  = 32,             // W channel packed payload width
    parameter integer B_PAYLOAD_WIDTH  = 32,             // B channel packed payload width
    parameter integer AR_PAYLOAD_WIDTH = 32,             // AR channel packed payload width
    parameter integer R_PAYLOAD_WIDTH  = 32,             // R channel packed payload width
    parameter         AW_SLICE_EN      = 1,              // 1: register slice, 0: bypass
    parameter         W_SLICE_EN       = 1,              // 1: register slice, 0: bypass
    parameter         B_SLICE_EN       = 1,              // 1: register slice, 0: bypass
    parameter         AR_SLICE_EN      = 1,              // 1: register slice, 0: bypass
    parameter         R_SLICE_EN       = 1               // 1: register slice, 0: bypass
) (
    input  wire                         aclk,
    input  wire                         aresetn,

    input  wire                         awvalid_s,
    output wire                         awready_s,
    input  wire [AW_PAYLOAD_WIDTH-1:0]  awpayload_s,
    output wire                         awvalid_m,
    input  wire                         awready_m,
    output wire [AW_PAYLOAD_WIDTH-1:0]  awpayload_m,

    input  wire                         wvalid_s,
    output wire                         wready_s,
    input  wire [W_PAYLOAD_WIDTH-1:0]   wpayload_s,
    output wire                         wvalid_m,
    input  wire                         wready_m,
    output wire [W_PAYLOAD_WIDTH-1:0]   wpayload_m,

    input  wire                         bvalid_s,
    output wire                         bready_s,
    input  wire [B_PAYLOAD_WIDTH-1:0]   bpayload_s,
    output wire                         bvalid_m,
    input  wire                         bready_m,
    output wire [B_PAYLOAD_WIDTH-1:0]   bpayload_m,

    input  wire                         arvalid_s,
    output wire                         arready_s,
    input  wire [AR_PAYLOAD_WIDTH-1:0]  arpayload_s,
    output wire                         arvalid_m,
    input  wire                         arready_m,
    output wire [AR_PAYLOAD_WIDTH-1:0]  arpayload_m,

    input  wire                         rvalid_s,
    output wire                         rready_s,
    input  wire [R_PAYLOAD_WIDTH-1:0]   rpayload_s,
    output wire                         rvalid_m,
    input  wire                         rready_m,
    output wire [R_PAYLOAD_WIDTH-1:0]   rpayload_m
);

    axi3_reg_slice_ch #(
        .PAYLOAD_WIDTH(AW_PAYLOAD_WIDTH),
        .ENABLE(AW_SLICE_EN)
    ) u_aw_slice (
        .aclk(aclk),
        .aresetn(aresetn),
        .valid_s(awvalid_s),
        .ready_s(awready_s),
        .payload_s(awpayload_s),
        .valid_m(awvalid_m),
        .ready_m(awready_m),
        .payload_m(awpayload_m)
    );

    axi3_reg_slice_ch #(
        .PAYLOAD_WIDTH(W_PAYLOAD_WIDTH),
        .ENABLE(W_SLICE_EN)
    ) u_w_slice (
        .aclk(aclk),
        .aresetn(aresetn),
        .valid_s(wvalid_s),
        .ready_s(wready_s),
        .payload_s(wpayload_s),
        .valid_m(wvalid_m),
        .ready_m(wready_m),
        .payload_m(wpayload_m)
    );

    axi3_reg_slice_ch #(
        .PAYLOAD_WIDTH(B_PAYLOAD_WIDTH),
        .ENABLE(B_SLICE_EN)
    ) u_b_slice (
        .aclk(aclk),
        .aresetn(aresetn),
        .valid_s(bvalid_s),
        .ready_s(bready_s),
        .payload_s(bpayload_s),
        .valid_m(bvalid_m),
        .ready_m(bready_m),
        .payload_m(bpayload_m)
    );

    axi3_reg_slice_ch #(
        .PAYLOAD_WIDTH(AR_PAYLOAD_WIDTH),
        .ENABLE(AR_SLICE_EN)
    ) u_ar_slice (
        .aclk(aclk),
        .aresetn(aresetn),
        .valid_s(arvalid_s),
        .ready_s(arready_s),
        .payload_s(arpayload_s),
        .valid_m(arvalid_m),
        .ready_m(arready_m),
        .payload_m(arpayload_m)
    );

    axi3_reg_slice_ch #(
        .PAYLOAD_WIDTH(R_PAYLOAD_WIDTH),
        .ENABLE(R_SLICE_EN)
    ) u_r_slice (
        .aclk(aclk),
        .aresetn(aresetn),
        .valid_s(rvalid_s),
        .ready_s(rready_s),
        .payload_s(rpayload_s),
        .valid_m(rvalid_m),
        .ready_m(rready_m),
        .payload_m(rpayload_m)
    );

endmodule

// -----------------------------------------------------------------------------
// Module      : axi3_reg_slice_ch
// Function    : Single-channel depth-2 forward register slice with skid buffer.
// -----------------------------------------------------------------------------
module axi3_reg_slice_ch #(
    parameter integer PAYLOAD_WIDTH = 32,                // Packed payload width
    parameter         ENABLE        = 1                  // 1: slice enabled, 0: bypass
) (
    input  wire                         aclk,
    input  wire                         aresetn,

    input  wire                         valid_s,
    output wire                         ready_s,
    input  wire [PAYLOAD_WIDTH-1:0]     payload_s,

    output wire                         valid_m,
    input  wire                         ready_m,
    output wire [PAYLOAD_WIDTH-1:0]     payload_m
);

    generate
        if (ENABLE == 0) begin : gen_bypass
            assign valid_m   = valid_s;
            assign ready_s   = ready_m;
            assign payload_m = payload_s;
        end else begin : gen_slice
            reg                    r_valid;
            reg [PAYLOAD_WIDTH-1:0] r_payload;
            reg                    skid_valid;
            reg [PAYLOAD_WIDTH-1:0] skid_payload;

            wire hs_m = r_valid && ready_m;
            wire hs_s = valid_s && ready_s;

            assign ready_s   = !skid_valid;
            assign valid_m   = r_valid;
            assign payload_m = r_payload;

            always @(posedge aclk or negedge aresetn) begin
                if (!aresetn) begin
                    r_valid    <= 1'b0;
                    skid_valid <= 1'b0;
                end else begin
                    if (hs_m) begin
                        if (skid_valid) begin
                            r_payload  <= skid_payload;
                            skid_valid <= 1'b0;
                            r_valid    <= 1'b1;
                        end else begin
                            r_valid <= hs_s ? 1'b1 : 1'b0;
                        end
                    end

                    if (hs_s) begin
                        if (!r_valid || hs_m) begin
                            r_payload <= payload_s;
                            r_valid   <= 1'b1;
                        end else begin
                            skid_valid   <= 1'b1;
                            skid_payload <= payload_s;
                        end
                    end
                end
            end
        end
    endgenerate

endmodule
