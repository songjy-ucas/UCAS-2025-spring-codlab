//储存每一路的tag信号，用于对比，产生hit信号，一个实例化就是一路，它里面又有8组数据
`timescale 10 ns / 1 ns

`define TARRAY_DATA_WIDTH 24
`define TARRAY_ADDR_WIDTH 3  //8组

module tag_array(
	input                             clk,
	input  [`TARRAY_ADDR_WIDTH - 1:0] waddr,
	input  [`TARRAY_ADDR_WIDTH - 1:0] raddr,
	input                             wen,
	input  [`TARRAY_DATA_WIDTH - 1:0] wdata,
	output [`TARRAY_DATA_WIDTH - 1:0] rdata
);

	reg [`TARRAY_DATA_WIDTH-1:0] array[ (1 << `TARRAY_ADDR_WIDTH) - 1 : 0];
	
	always @(posedge clk)
	begin
		if(wen)
			array[waddr] <= wdata;
	end

assign rdata = array[raddr];

endmodule