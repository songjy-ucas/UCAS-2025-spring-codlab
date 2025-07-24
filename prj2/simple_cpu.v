`timescale 10ns / 1ns

//ALUop译码模块
module ALUop(
    input  [5:0] Opcode,
	input  [5:0]   func,
	output [2:0]  ALUop
);
    assign ALUop = (Opcode == 6'b000000) ? (
	               (func   == 6'b100001) ? 3'b010 :
	               (func   == 6'b100011) ? 3'b110 :
		           (func   == 6'b100100) ? 3'b000 :
		           (func   == 6'b100111) ? 3'b101 :
		           (func   == 6'b100101) ? 3'b001 :
		           (func   == 6'b100110) ? 3'b100 :
		           (func   == 6'b101010) ? 3'b111 :
				   (func   == 6'b101011) ? 3'b011 :
				                           3'b001):
	               (Opcode == 6'b000001) ? 3'b111 :
				   (Opcode[5:3] == 3'b000)? 3'b110:
				   (Opcode[5:3] == 3'b101 || Opcode[5:3] == 3'b100)? 3'b010 :
				   (Opcode == 6'b001001) ? 3'b010 :
				   (Opcode == 6'b001100) ? 3'b000 :
				   (Opcode == 6'b001101) ? 3'b001 :
				   (Opcode == 6'b001110) ? 3'b100 :
				   (Opcode == 6'b001010) ? 3'b111 :
				                           3'b011 ; 

endmodule
					

module simple_cpu(
	input             clk,
	input             rst,

	output [31:0]     PC,
	input  [31:0]     Instruction,

	output [31:0]     Address,    //是lb~swr指令时的output，即访问的内存地址
	output            MemWrite,   //内存写使能,sb~swr指令时为1
	output [31:0]     Write_data, //要写入内存的值
	output [ 3:0]     Write_strb, //写掩码，描述要写几位

	input  [31:0]     Read_data,  //内存读出的值，在lb~lwr指令时写入寄存器堆
	output            MemRead     //内存读使能,lb~lwr指令时为1
);

	// THESE THREE SIGNALS ARE USED IN OUR TESTBENCH
	// PLEASE DO NOT MODIFY SIGNAL NAMES
	// AND PLEASE USE THEM TO CONNECT PORTS
	// OF YOUR INSTANTIATION OF THE REGISTER FILE MODULE
	wire			RF_wen;//寄存器写使能
	wire [4:0]		RF_waddr;//寄存器写地址
	wire [31:0]		RF_wdata;//写入值

    //会用到的instruction的各段
	wire [5:0] Opcode = Instruction[31:26];
	wire [4:0] rs     = Instruction[25:21];
	wire [4:0] rt     = Instruction[20:16];
	wire [4:0] rd     = Instruction[15:11];
	wire [5:0] shamt  = Instruction[10: 6];
    wire [6:0] func   = Instruction[ 5: 0];
	wire [15:0]imm    = Instruction[15: 0];
	wire [25:0]tar    = Instruction[25: 0];

	wire          RegDst;          //选择读写寄存器的地址
	wire          jump;
	wire          branch;
	wire          move;            //判断是不是move指令
	wire [2:0]    ALUSrc;          //选择寄存器还是立即数，读入alu计算,高两位判断alu操作数1的选择，低两位判断alu操作数2的选择 
    wire [31:0]   rsdata;
	wire [31:0]   rtdata;
	wire [31:0]   rddata;          //移位结果
	wire [4:0]    shiftlength;     //移位位数
 	wire          zero;            
	wire [31:0]   Result;          //alu结果
	wire [31:0]   imm_32;          //拓展到32位的立即数 即imm + shifter -----> imm_32

//MemRead
    assign MemRead = Opcode[5:3] == 3'b100 ? 1 : 0 ;
//MemWrite
    assign MemWrite = Opcode[5:3] == 3'b101 ? 1 : 0;

//imm_32的计算
    wire [31:0] imm_32_0E;
	wire [31:0] imm_32_SE;
	wire [31:0] imm_32_SE00;//用于跳转指令的imm(SE)||00
	wire [31:0] imm_32_1600;//用于lui指令
	wire        imm_sign = imm[15];

	assign imm_32_SE00 = {{14{imm_sign}},imm,2'b00};
	assign imm_32_0E   = {{16{1'b0}},imm} ;
	assign imm_32_SE   = {{16{imm_sign}},imm} ;
	assign imm_32_1600 = {imm,{16{1'b0}}};

    assign imm_32 = (Opcode == 6'b001100 || Opcode == 6'b001101 || Opcode == 6'b001110) ? imm_32_0E  :
	                Opcode[5:3] == 3'b000                                               ? imm_32_SE00:
					Opcode[5:0] == 6'b001111                                            ? imm_32_1600:
					                                                                      imm_32_SE  ;

//写地址RF_waddr选择	rt，rd，11111(jal)
	assign RegDst    = Instruction[31]^Instruction[29];
	assign RF_waddr  = RegDst    ? rt       :
	                   Opcode[0] ? 5'b11111 :
							             rd ; //选择写地址是哪一个

//branch计算
    assign branch = Opcode[5:2] == 1'h1 | Opcode == 6'b000001;

//move计算
    assign move = (Opcode == 6'b000000 && (func == 6'b001011 || func == 6'b001010));

//jump计算
    assign jump = Opcode == 6'b000010 || Opcode == 6'b000011 || (Opcode == 6'b000000 && (func == 6'b001000 || func == 6'b001001));

//ALUSrc计算   ALUSrc[2] == 1,选择rsdata，反之为32’b0;ALUSrc[1:0] == 00,选择32'b0,== 01，选择imm_32,==10,选择rtdata
    assign ALUSrc[2]   = move ? 1'b0 : 1'b1 ;
	assign ALUSrc[1:0] = move                                                              ? 2'b10 :
	                     Opcode == 6'b000000 || Opcode == 6'b000100 || Opcode == 6'b000101 ? 2'b10 :
						 Opcode == 6'b000110 || Opcode == 6'b000111 || Opcode == 6'b000001 ? 2'b00 :
						                                                                     2'b01 ;
	                             
//写使能RF_wen判断
    assign RF_wen = Opcode[5:3] == 3'b101                                   ? 1'b0 :
	                Opcode      == 6'b000010                                ? 1'b0 :
	                branch                                                  ? 1'b0 :
					(Opcode     == 6'b000000 && func == 6'b001000)          ? 1'b0 :
					(Opcode     == 6'b000000 && func == 6'b001011 && zero)  ? 1'b0 : 
					(Opcode     == 6'b000000 && func == 6'b001010 && ~zero) ? 1'b0 :
					                                                          1'b1;

//寄存器堆模块
	reg_file instance_reg_file(
        .clk(clk),
		.waddr(RF_waddr),
		.raddr1(rs),
		.raddr2(rt),
		.wen(RF_wen),
		.wdata(RF_wdata),
		.rdata1(rsdata),
		.rdata2(rtdata)
	);

//shifter模块
    assign shiftlength = func[2] ? rsdata[4:0] : shamt ;
    shifter instance_shifter(
		.Shiftop(func[1:0]),
		.A(rtdata),
		.B(shiftlength),
	    .Result(rddata)
	);
//利用ALUSrc选取ALU输入,并计算得Result,Zero
	wire [31:0] rdata1;
	wire [31:0] rdata2;
	wire [2:0]  ALUop;
	assign rdata1 = ALUSrc[2]   == 1'b1  ? rsdata : 32'b0 ;
	assign rdata2 = ALUSrc[1:0] == 2'b00 ? 32'b0  :
	                ALUSrc[1:0] == 2'b01 ? imm_32 : rtdata;
	ALUop instance_ALUop(
		.Opcode(Opcode),
		.func(func),
		.ALUop(ALUop)
	);
    alu instance_alu(
        .A(rdata1),
		.B(rdata2),
		.ALUop(ALUop),
		.Zero(zero),
		.Result(Result)
	);

//mem_data的确定
    wire [31:0] mem_data; //lb~lwr指令，要往寄存器堆写入的东西
	wire [31:0] lwl_data;
	wire [31:0] lwr_data;
	assign lwl_data = Result[1:0] == 2'b00  ? {Read_data[7:0],rtdata[23:0] }    :
	                  Result[1:0] == 2'b01  ? {Read_data[15:0],rtdata[15:0]}    :
					  Result[1:0] == 2'b10  ? {Read_data[23:0],rtdata[7:0] }    :
					                          Read_data                   ;

	assign lwr_data = Result[1:0] == 2'b00  ? Read_data                    :
	                  Result[1:0] == 2'b01  ? {rtdata[31:24],Read_data[31:8]}     :
					  Result[1:0] == 2'b10  ? {rtdata[31:16],Read_data[31:16]}     :
					                          {rtdata[31:8],Read_data[31:24]}       ;
    wire [31:0] lb_mem_data;
	wire [31:0] lbu_mem_data;
	wire [31:0] lh_mem_data;
	wire [31:0] lhu_mem_data;
	assign lb_mem_data = Result[1:0] == 2'b00 ? {{24{Read_data[7]}},Read_data[7:0]}    : 
	                     Result[1:0] == 2'b01 ? {{24{Read_data[15]}},Read_data[15:8]}   :
						 Result[1:0] == 2'b10 ? {{24{Read_data[23]}},Read_data[23:16]}  :
						                        {{24{Read_data[31]}},Read_data[31:24]}  ;
	assign lbu_mem_data = Result[1:0] == 2'b00 ? {{24{1'b0}},Read_data[7:0]}  :
	                      Result[1:0] == 2'b01 ? {{24{1'b0}},Read_data[15:8]} :
						  Result[1:0] == 2'b10 ? {{24{1'b0}},Read_data[23:16]}:
						                         {{24{1'b0}},Read_data[31:24]}; 
	assign lh_mem_data = Result[1:0] == 2'b00 ? {{16{Read_data[15]}},Read_data[15:0]} : 
	                                            {{16{Read_data[31]}},Read_data[31:16]};                     
	assign lhu_mem_data = Result[1:0] == 2'b00 ? {{24{1'b0}},Read_data[15:0]}         :
	                                             {{24{1'b0}},Read_data[31:16]}         ;
	                      
    assign mem_data = Opcode[2:0] == 3'b000 ? lb_mem_data          :
	                  Opcode[2:0] == 3'b100 ? lbu_mem_data         :
					  Opcode[2:0] == 3'b001 ? lh_mem_data          :
					  Opcode[2:0] == 3'b101 ? lhu_mem_data         :
					  Opcode[2:0] == 3'b011 ? Read_data            :
					  Opcode[2:0] == 3'b010 ? lwl_data :   lwr_data;

//RF_wdata的选择 
    assign RF_wdata = Opcode == 6'b0 ? (
		              func   == 6'b001000 || func == 6'b001001 ? PC+8    :
					  func   == 6'b001011 || func == 6'b001010 ? rsdata  :
					  func[5]== 1'b0                           ? rddata  :  
					                                             Result) :
					  Opcode      == 6'b001111 ? imm_32     :
					  Opcode[5:3] == 3'b001    ? Result     :
					  Opcode[5:3] == 3'b100    ? mem_data   :  PC+8       ;
   
//Address的确定
   assign Address = {Result[31:2],2'b00};

//Write_data和Write_strb的确定
   wire [31:0] swr_rtdata;
   wire [31:0] swl_rtdata;
   assign swl_rtdata = Result[1:0] == 2'b00 ? {{24{1'b0}},rtdata[31:24]} :
                       Result[1:0] == 2'b01 ? {{16{1'b0}},rtdata[31:16]} :
					   Result[1:0] == 2'b10 ? {{8{1'b0}},rtdata[31:8]}   :
					                          rtdata                     ;
   assign swr_rtdata = Result[1:0] == 2'b11 ? {rtdata[7:0],{24{1'b0}}}   :
                       Result[1:0] == 2'b10 ? {rtdata[15:0],{16{1'b0}}}  :
					   Result[1:0] == 2'b01 ? {rtdata[23:0],{8{1'b0}}}   :
					                           rtdata                    ;           
   assign Write_data = Opcode == 6'b101000 ? rtdata<<(8*Result[1:0]) :
                       Opcode == 6'b101001 ? rtdata<<(16*Result[1])  :
					   Opcode == 6'b101011 ? rtdata                  :
					   Opcode == 6'b101010 ? swl_rtdata  : swr_rtdata;                                           

	wire [3:0] strb_1;
	assign strb_1[0] = (Result[1:0] == 2'b00);
    assign strb_1[1] = (Result[1:0] == 2'b01);
	assign strb_1[2] = (Result[1:0] == 2'b10);
	assign strb_1[3] = (Result[1:0] == 2'b11);   
	wire [3:0] strb_2;
	assign strb_2[1:0] = {2{(Result[1] == 1'b0)}};
	assign strb_2[3:2] = {2{(Result[1] == 1'b1)}};
	wire [3:0] strb_3 = 4'b1111;
	wire [3:0] strb_4;
	assign strb_4 = 4'b1111 >> (3-Result[1:0]);
	wire [3:0] strb_5;
	assign strb_5 = 4'b1111 << Result[1:0];
	assign Write_strb = Opcode[2:0] == 3'b000 ? strb_1 :
	                    Opcode[2:0] == 3'b001 ? strb_2 :
						Opcode[2:0] == 3'b011 ? strb_3 :
						Opcode[2:0] == 3'b010 ? strb_4 :
						                        strb_5 ;

wire        is_jorb;   //记录是否跳转
wire [31:0] next_pc;   //跳转地址
assign is_jorb = jump   ? 1 :
                 branch ? (
                      Opcode[2:1] == 2'b10 ? Opcode[0]^zero               :
					  Opcode[2:1] == 2'b11 ? Opcode[0]^(Result[31]||zero) :
					                         rt[0]^(Result == 1'b1)       
				 ): 0;
wire [31:0] pc_add4;
assign pc_add4 = PC+4;
assign next_pc = branch    ? pc_add4+imm_32_SE00       :
                 Opcode[1] ? {pc_add4[31:28],tar,2'b0} :
				                               rsdata  ; 
reg [31:0] reg_pc; 
always @(posedge clk) begin
    if(rst) begin
	   reg_pc <= 32'b0;
	end
	else begin
		if(is_jorb) begin
	        reg_pc <= next_pc;	
	    end
		else begin
			reg_pc <= pc_add4;
		end
	end
end
assign PC = reg_pc;

endmodule

