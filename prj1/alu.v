`timescale 10 ns / 1 ns

`define DATA_WIDTH 32

module alu(
	input  [`DATA_WIDTH - 1:0]  A,
	input  [`DATA_WIDTH - 1:0]  B,
	input  [              2:0]  ALUop,
	output                      Overflow,
	output                      CarryOut,
	output                      Zero,
	output [`DATA_WIDTH - 1:0]  Result
);
    wire [`DATA_WIDTH - 1:0] and_result;
    wire [`DATA_WIDTH - 1:0] or_result;
    wire [`DATA_WIDTH :0] add_result;
	wire slt_result;

    wire [`DATA_WIDTH:0] a = {1'b0, A};
    wire [`DATA_WIDTH:0] b = (ALUop[2]==1)?~{1'b0,B}:{1'b0,B};

	assign and_result = A&B;
	assign or_result = A|B;
	assign add_result = a+b+ALUop[2];
    assign slt_result = (A[31] == 1 && B[31] == 0)? 1 : 
	(A[31] == 0 && B[31] == 1) ? 0 :
	(add_result[31] == 1) ? 1 : 0;

/*output*/
    assign Overflow =(ALUop == 3'b010||ALUop == 3'b110) ? (A[31]==b[31])&&(add_result[31]!=A[31]):1'b0;

	assign Result = (ALUop == 3'b000) ? and_result :
	(ALUop == 3'b001) ? or_result :
	(ALUop == 3'b010||ALUop == 3'b110) ? add_result[`DATA_WIDTH - 1:0] :
	(ALUop == 3'b111) ?  (slt_result ? 32'b1 : 32'b0) : 32'b0;
    assign CarryOut = add_result[`DATA_WIDTH];

    assign Zero = (Result==32'b0)? 1 : 0;

endmodule

