// -----------------------------------------------------------------------------
// Module      : axi3_ahb_region_sel
// Date        : 2026-05-29
// Version     : v1.1.0
// Author      : Jongchul Shin
// Function    : Decode an AXI address into a one-hot AHB router select and
//               zero unused address bits for downstream synthesis.
//               Each AHB port owns a fixed-size region (REGION_SIZE_KB).
//               The mext map is assumed aligned to (region size) x 2^X slots
//               where 2^X >= N_PORTS; unused decode codes map to the last port.
// Assumptions : REGION_SIZE_KB is a power-of-two kilobyte count (e.g. 4 = 4KB).
//               Decode uses addr[REGION_LSB +: DECODE_W]; lower REGION_LSB bits
//               are consumed by the selected AHB target address.
// Notes       : addr_map clears bits above the decode field (apply at input).
//               addr_tgt clears decode and upper bits (apply at router s_*addr).
//               sel is combinational from addr; addr must retain the decode field.
// -----------------------------------------------------------------------------
module axi3_ahb_region_sel #(
    parameter integer N_PORTS         = 8,    // Number of one-hot select bits
    parameter integer ADDR_WIDTH      = 32,   // AXI address width
    parameter integer REGION_SIZE_KB  = 4     // Equal AHB slot size in kilobytes
) (
    input  wire [ADDR_WIDTH-1:0]       addr,
    output wire [N_PORTS-1:0]          sel,
    output wire [ADDR_WIDTH-1:0]       addr_map,
    output wire [ADDR_WIDTH-1:0]       addr_tgt
);

    localparam integer REGION_BYTES = REGION_SIZE_KB * 1024;
    localparam integer REGION_LSB   = $clog2(REGION_BYTES);
    localparam integer DECODE_W     = (N_PORTS <= 1) ? 1 : $clog2(N_PORTS);
    localparam integer ADDR_MAP_W   = REGION_LSB + DECODE_W;

    wire [DECODE_W-1:0] idx_raw = (N_PORTS <= 1) ? {DECODE_W{1'b0}} :
        addr[REGION_LSB + DECODE_W - 1 : REGION_LSB];
    wire [DECODE_W-1:0] idx = (N_PORTS <= 1) ? {DECODE_W{1'b0}} :
        ((idx_raw >= N_PORTS) ? (N_PORTS - 1) : idx_raw);

    genvar gi;
    generate
        for (gi = 0; gi < N_PORTS; gi = gi + 1) begin : g_sel
            assign sel[gi] = (N_PORTS <= 1) ? 1'b1 : (idx == gi);
        end

        if (ADDR_WIDTH > ADDR_MAP_W) begin : g_addr_map_wide
            assign addr_map = {{(ADDR_WIDTH - ADDR_MAP_W){1'b0}},
                               addr[ADDR_MAP_W - 1 : 0]};
        end else begin : g_addr_map_narrow
            assign addr_map = addr[ADDR_WIDTH - 1 : 0];
        end

        if (ADDR_WIDTH > REGION_LSB) begin : g_addr_tgt_wide
            assign addr_tgt = {{(ADDR_WIDTH - REGION_LSB){1'b0}},
                               addr[REGION_LSB - 1 : 0]};
        end else begin : g_addr_tgt_narrow
            assign addr_tgt = addr[ADDR_WIDTH - 1 : 0];
        end
    endgenerate

    // synopsys translate_off
    `ifdef SYNTHESIS
    `else
    initial begin
        if (REGION_SIZE_KB < 1) begin
            $error("%m: axi3_ahb_region_sel: REGION_SIZE_KB must be >= 1 (got %0d)",
                   REGION_SIZE_KB);
        end
        if ((REGION_SIZE_KB & (REGION_SIZE_KB - 1)) != 0) begin
            $error("%m: axi3_ahb_region_sel: REGION_SIZE_KB must be a power of two (got %0d)",
                   REGION_SIZE_KB);
        end
        if (N_PORTS >= 1 && (REGION_LSB + DECODE_W) > ADDR_WIDTH) begin
            $error("%m: axi3_ahb_region_sel: decode field exceeds ADDR_WIDTH (REGION_LSB=%0d DECODE_W=%0d ADDR_WIDTH=%0d)",
                   REGION_LSB, DECODE_W, ADDR_WIDTH);
        end
    end
    `endif
    // synopsys translate_on

endmodule
