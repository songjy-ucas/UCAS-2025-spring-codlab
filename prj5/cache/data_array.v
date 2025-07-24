//存储每一路的数据，一个实例化是一路，它里面又有8组数据
`timescale 10 ns / 1 ns
`include "cache_config.vh"

module data_array (
    input                               clk,
    input  [`DARRAY_ADDR_WIDTH - 1 : 0] waddr,
    input  [`DARRAY_ADDR_WIDTH - 1 : 0] raddr,
    input                               wen,
    input  [       `DATA_WIDTH - 1 : 0] wdata,
    input                               wr_hit,
    input  [     `OFFSET_WIDTH - 1 : 0] offset,
    input  [   `DATA_WIDTH / 8 - 1 : 0] strb,
    output [`DARRAY_DATA_WIDTH - 1 : 0] rdata
);

    reg  [`DARRAY_DATA_WIDTH - 1 : 0] array [(1 << `DARRAY_ADDR_WIDTH) - 1 : 0];

    wire [       `DATA_WIDTH - 1 : 0] mask;

    assign mask = {{8{strb[3]}}, {8{strb[2]}}, {8{strb[1]}}, {8{strb[0]}}};

    always @(posedge clk) begin
        if (wen)
            array[waddr] <= (
                {`DARRAY_DATA_WIDTH{wr_hit}} & (
                    (array[waddr] & ~(mask << {offset, 3'b0})) |
                    ((wdata & mask) << {offset, 3'b0})
                ) |
                {`DARRAY_DATA_WIDTH{!wr_hit}} &
                    {wdata, array[waddr][`DARRAY_DATA_WIDTH - 1 : `DATA_WIDTH]}
            );
    end

    assign rdata = array[raddr];

endmodule
