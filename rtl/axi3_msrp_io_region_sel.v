// -----------------------------------------------------------------------------
// Module      : axi3_msrp_io_region_sel
// Date        : 2026-05-29
// Version     : v1.0.0
// Author      : Jongchul Shin
// Function    : Decode msrp AXI address into axi3_router_1to2_128 one-hot select.
//               Addresses in the IO window route to target1 (sextio); all others
//               route to target0 (sextmem).
// Assumptions : aw_sel/ar_sel are driven combinationally from addr when valid.
// Notes       : Combinational only; connect to router aw_sel/ar_sel per msrp port.
// Caution     : IO window limits are fixed constants below (not top-level params).
//               If the system address map changes, this module must be edited and
//               RTL re-verified—integration cannot retarget mem/io by parameter alone.
// -----------------------------------------------------------------------------
module axi3_msrp_io_region_sel #(
    parameter integer ADDR_WIDTH = 35    // msrp AXI address width
) (
    input  wire [ADDR_WIDTH-1:0] addr,
    output wire [1:0]            sel     // 2'b01=sextmem (target0), 2'b10=sextio (target1)
);

    // Fixed SRPS system map: 0xE000_0000 .. 0xFFFF_FFFF -> sextio.
    localparam [ADDR_WIDTH-1:0] IO_ADDR_BASE  =
        {{(ADDR_WIDTH - 32){1'b0}}, 32'hE000_0000};
    localparam [ADDR_WIDTH-1:0] IO_ADDR_LIMIT =
        {{(ADDR_WIDTH - 32){1'b0}}, 32'hFFFF_FFFF};

    wire in_io = (addr >= IO_ADDR_BASE) && (addr <= IO_ADDR_LIMIT);

    assign sel = in_io ? 2'b10 : 2'b01;

endmodule
