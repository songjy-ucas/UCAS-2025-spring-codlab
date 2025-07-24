`timescale 10 ns / 1 ns

module alu(
    input  [31:0]  A,
	input  [31:0]  B,
	input  [2: 0]  ALUop,
	output         Zero,
	output [31:0]  Result

);
    wire [31:0]    and_result;
    wire [31:0]    or_result;
    wire [32:0]    add_result;
	wire [31:0]    xor_result;
	wire [31:0]    mul_result; //由于RISC-v不用nor，所以将之前的nor改为mul
	wire           slt_result;
	wire           sltu_result;
	wire           Overflow;
	wire           CarryOut;

    wire [32:0] a = {1'b0, A};
    wire [32:0] b = (ALUop[2]|ALUop[0]) ?~{1'b0,B}:{1'b0,B};

	assign and_result  = A&B;
	assign or_result   = A|B;
	assign xor_result  = A^B;
	assign mul_result  = A*B;//A~^B;
	assign add_result  = a+b+(ALUop[2]|ALUop[0]);
    assign slt_result  = (A[31] == 1'b1 && B[31] == 1'b0) ? 1'b1 : 
	                     (A[31] == 1'b0 && B[31] == 1'b1) ? 1'b0 :
	                     (add_result[31] == 1'b1)         ? 1'b1 : 
						                                    1'b0 ;
	assign sltu_result = add_result[32] == 1'b1           ? 1'b1 :
	                                                        1'b0 ;
    
	assign Result = (ALUop == 3'b000)                  ? and_result                    :
	                (ALUop == 3'b001)                  ? or_result                     :
	                (ALUop == 3'b010||ALUop == 3'b110) ? add_result[31:0]              :
					(ALUop == 3'b100)                  ? xor_result                    :
					(ALUop == 3'b101)                  ? mul_result                    :
	                (ALUop == 3'b111)                  ? (slt_result ? 32'b1 : 32'b0)  : 
					                                     (sltu_result ? 32'b1 : 32'b0) ;
	assign Zero = (Result == 32'b0) ? 1'b1 : 1'b0;				                                                             

endmodule

