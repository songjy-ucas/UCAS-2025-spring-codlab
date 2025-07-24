`timescale 10ns / 1ns

module custom_cpu (
    input clk,
    input rst,

    //Instruction request channel
    output [31:0] PC,
    output        Inst_Req_Valid,
    input         Inst_Req_Ready,

    //Instruction response channel
    input  [31:0] Instruction,
    input         Inst_Valid,
    output        Inst_Ready,

    //Memory request channel
    output [31:0] Address,
    output        MemWrite,
    output [31:0] Write_data,
    output [ 3:0] Write_strb,
    output        MemRead,
    input         Mem_Req_Ready,

    //Memory data response channel
    input  [31:0] Read_data,
    input         Read_data_Valid,
    output        Read_data_Ready,

    input intr, //中断信号，此处暂时不用

    output [31:0] cpu_perf_cnt_0,
    output [31:0] cpu_perf_cnt_1,
    output [31:0] cpu_perf_cnt_2,
    output [31:0] cpu_perf_cnt_3,
    output [31:0] cpu_perf_cnt_4,
    output [31:0] cpu_perf_cnt_5,
    output [31:0] cpu_perf_cnt_6,
    output [31:0] cpu_perf_cnt_7,
    output [31:0] cpu_perf_cnt_8,
    output [31:0] cpu_perf_cnt_9,
    output [31:0] cpu_perf_cnt_10,
    output [31:0] cpu_perf_cnt_11,
    output [31:0] cpu_perf_cnt_12,
    output [31:0] cpu_perf_cnt_13,
    output [31:0] cpu_perf_cnt_14,
    output [31:0] cpu_perf_cnt_15,

    output [69:0] inst_retire
);


  // TODO: Please add your custom CPU code here

  //----------------------------------------------------------------------------
  // Parameter Definitions
  // (Grouped here for clarity, used across stages)
  //----------------------------------------------------------------------------

  //控制流水线是否前进
    wire next_state_enable;
	wire jorb_type; //流水线中ID阶段的指令是不是（可能）会跳转的指令
	wire is_jorb;   //流水线中ID阶段的指令是不是要跳转的指令
    wire IF_Mod_Ready;
    wire MEM_Mod_Ready;
    wire W_before_R_LD;  //先写后读（有访存）
	wire [31:0] Ahead_value1; //前递值
	wire [31:0] Ahead_value2; //前递值
    assign next_state_enable = IF_Mod_Ready & MEM_Mod_Ready;
    
    wire [6:0] funct_length7 = IF_ID_IR[31:25];
	wire [4:0] rs2           = IF_ID_IR[24:20];
	wire [4:0] rs1           = IF_ID_IR[19:15];
	wire [2:0] funct_length3 = IF_ID_IR[14:12];
	wire [4:0] rd            = IF_ID_IR[11: 7];
    wire [6:0] Opcode        = IF_ID_IR[ 6: 0];
	wire U_type;
	wire J_type;
	wire I_type;
	wire B_type;
	wire S_type;
	wire R_type;
	wire I_LOAD_type;
	assign U_type      = Opcode == 7'b0110111 || Opcode == 7'b0010111;
    assign J_type      = Opcode == 7'b1101111;
	assign I_type      = Opcode == 7'b0010011 || Opcode == 7'b1100111;
	assign B_type      = Opcode == 7'b1100011;
	assign S_type      = Opcode == 7'b0100011;
    assign R_type      = Opcode == 7'b0110011;
	assign I_LOAD_type = Opcode == 7'b0000011;

//前递信号,ALU和shifter结果作为RF_wdata时，可以前递；有访存时，如果该指令以及到MEM和WB级流水，也可以前递
//          注意：零号寄存器不前递！！！！        因为零号寄存器只能读出0 ！！！
    wire       ID_RF_wen;
	wire [4:0] ID_RF_waddr;
    reg        ID_EX_RF_wen;
	reg        EX_MEM_RF_wen;
	reg        MEM_WB_RF_wen;
	reg  [4:0] ID_EX_RF_waddr;
	reg  [4:0] EX_MEM_RF_waddr;
	reg  [4:0] MEM_WB_RF_waddr;
	always @(posedge clk)begin
	    if(rst)begin
		    ID_EX_RF_waddr <= 5'b0;
			ID_EX_RF_wen   <= 1'b0;
			EX_MEM_RF_waddr<= 5'b0;
			EX_MEM_RF_wen  <= 1'b0;
			MEM_WB_RF_waddr<= 5'b0;
			MEM_WB_RF_wen  <= 1'b0;  
		end
		else begin
		    if(next_state_enable && ~W_before_R_LD)begin
			    ID_EX_RF_waddr <= ID_RF_waddr;
			    ID_EX_RF_wen   <= ID_RF_wen;
			    EX_MEM_RF_waddr<= ID_EX_RF_waddr;
			    EX_MEM_RF_wen  <= ID_EX_RF_wen;
			    MEM_WB_RF_waddr<= EX_MEM_RF_waddr;
			    MEM_WB_RF_wen  <= EX_MEM_RF_wen;
			end
			else if(next_state_enable && W_before_R_LD)begin
		        ID_EX_RF_waddr <= 5'b0;
			    ID_EX_RF_wen   <= 1'b0;		
				EX_MEM_RF_waddr<= ID_EX_RF_waddr;
				EX_MEM_RF_wen  <= ID_EX_RF_wen;
				MEM_WB_RF_waddr<= EX_MEM_RF_waddr;
				MEM_WB_RF_wen  <= EX_MEM_RF_wen; 	    
			end
		end
	end

	wire [4:0] ID_reg1   = IF_ID_IR[19:15];
	wire [4:0] ID_reg2   = IF_ID_IR[24:20];
	wire [4:0] EX_waddr  = ID_EX_IR[11:7];
	wire [4:0] MEM_waddr = EX_MEM_IR[11:7];
	wire [4:0] WB_waddr  = MEM_WB_IR[11:7];
	wire [31:0] EX_RF_wdata;
	wire W_before_R_reg1; //rs1的先写后读
	wire W_before_R_reg2; //rs2的先写后读
	assign W_before_R_reg1 = ID_reg1 == 5'b0 ? 1'b0 : (ID_reg1 == EX_waddr && ID_EX_RF_wen ) || (ID_reg1 == MEM_waddr && EX_MEM_RF_wen )|| (ID_reg1 == WB_waddr && MEM_WB_RF_wen );
	assign W_before_R_reg2 = ID_reg2 == 5'b0 ? 1'b0 : (ID_reg2 == EX_waddr && ID_EX_RF_wen ) || (ID_reg2 == MEM_waddr && EX_MEM_RF_wen )|| (ID_reg2 == WB_waddr && MEM_WB_RF_wen );
    assign Ahead_value1 = (ID_reg1 == EX_waddr && ID_EX_RF_wen )   ? EX_RF_wdata :
	                      (ID_reg1 == MEM_waddr && EX_MEM_RF_wen ) ? MEM_RF_wdata:
						                                          MEM_WB_RF_wdata; 
    assign Ahead_value2 = (ID_reg2 == EX_waddr && ID_EX_RF_wen )   ? EX_RF_wdata :
	                      (ID_reg2 == MEM_waddr && EX_MEM_RF_wen ) ? MEM_RF_wdata:
						                         MEM_WB_RF_wdata;
    
//有访存的先写后读，需要NOP等待;EX阶段有一个LOAD指令，但是LOAD值在MEM之后才会得到，所以要等待
//EX阶段的，要等一次流水
    wire EX_I_LOAD_type = ID_EX_IR[6:0] == 7'b0000011;
	assign W_before_R_LD = 
	(ID_reg1 == EX_waddr && EX_I_LOAD_type && ID_EX_RF_wen && ~(ID_reg1 == 5'b0)) || 
	(ID_reg2 == EX_waddr && EX_I_LOAD_type && ID_EX_RF_wen && ~(ID_reg1 == 5'b0)); 
//MEM阶段冲突，看似不冲突，但是因为访存之前就会先行计算is_jorb,所以应该在访存结束后再更新一次PC
    wire MEM_I_LOAD_type = EX_MEM_IR[6:0] == 7'b0000011; 
	assign W_before_R_LD_MEM = 
	(ID_reg1 == MEM_waddr && MEM_I_LOAD_type && EX_MEM_RF_wen && ~(ID_reg1 == 5'b0)) || 
	(ID_reg2 == MEM_waddr && MEM_I_LOAD_type && EX_MEM_RF_wen && ~(ID_reg1 == 5'b0));

  //寄存器模块实例化
	wire [31:0] rdata1;
	wire [31:0] rdata2;
	wire [ 4:0] rs1;
	wire [ 4:0] rs2;

	reg_file instance_reg_file(
        .clk(clk),
		.waddr(inst_retire[68:64]),
		.raddr1(rs1),
		.raddr2(rs2),
		.wen(inst_retire[69]),
		.wdata(inst_retire[63:32]),
		.rdata1(rdata1),
		.rdata2(rdata2)
	);
   
   //--------------------------------------------------------
   //流水各个模块实例化和互联
    
//IF_Mod
    wire [31:0] IF_PC;
	wire [31:0] next_pc;
    reg  [31:0] IF_ID_PC;
    reg  [31:0] ID_EX_PC;
    reg  [31:0] EX_MEM_PC;
    reg  [31:0] MEM_WB_PC;
	assign PC = IF_PC;
    always @(posedge clk)begin
        if(rst)begin
	        IF_ID_PC  <= 32'b0;
		    ID_EX_PC  <= 32'b0;
		    EX_MEM_PC <= 32'b0;
		    MEM_WB_PC <= 32'b0;
	    end
	    else begin
	        if(next_state_enable && ~W_before_R_LD )begin
		        IF_ID_PC  <= IF_PC;
			    ID_EX_PC  <= IF_ID_PC;
			    EX_MEM_PC <= ID_EX_PC;
			    MEM_WB_PC <= EX_MEM_PC;
		    end
			else if(next_state_enable && W_before_R_LD )begin
			    IF_ID_PC <= IF_ID_PC;
				ID_EX_PC <= 32'b0;
				EX_MEM_PC<= ID_EX_PC;
				MEM_WB_PC<= EX_MEM_PC;
			end
			// else if(next_state_enable && ~W_before_R_LD && is_jorb)begin
			//     IF_ID_PC <= 32'b0;
			// 	ID_EX_PC <= IF_ID_PC;
			// 	EX_MEM_PC<= ID_EX_PC;
			// 	MEM_WB_PC<= EX_MEM_PC;			
			// end
			// else if(next_state_enable && W_before_R_LD && is_jorb)begin
			//     IF_ID_PC <= 32'b0;
			// 	ID_EX_PC <= 32'b0;
			// 	EX_MEM_PC<= ID_EX_PC;
			// 	MEM_WB_PC<= EX_MEM_PC;			
			// end
	    end
    end

    wire [31:0] IF_IR;
    reg  [31:0] IF_ID_IR;
    reg  [31:0] ID_EX_IR;
    reg  [31:0] EX_MEM_IR;
    reg  [31:0] MEM_WB_IR;
    always @(posedge clk)begin
        if(rst)begin
		    IF_ID_IR  <= 32'b0;
			ID_EX_IR  <= 32'b0;
			EX_MEM_IR <= 32'b0;
			MEM_WB_IR <= 32'b0;
		end
		else begin
		    if(next_state_enable && ~W_before_R_LD )begin
			    IF_ID_IR  <= IF_IR;
				ID_EX_IR  <= IF_ID_IR;
				EX_MEM_IR <= ID_EX_IR;
				MEM_WB_IR <= EX_MEM_IR;
			end
			else if(next_state_enable && W_before_R_LD )begin
			    IF_ID_IR <= IF_ID_IR;
				ID_EX_IR <= 32'b0;
				EX_MEM_IR<= ID_EX_IR;
				MEM_WB_IR<= EX_MEM_IR;
			end
			// else if(next_state_enable && ~W_before_R_LD && is_jorb)begin
			//     IF_ID_IR <= 32'b0;
			// 	ID_EX_IR <= IF_ID_IR;
			// 	EX_MEM_IR<= ID_EX_IR;
			// 	MEM_WB_IR<= EX_MEM_IR;			
			// end
			// else if(next_state_enable && W_before_R_LD && is_jorb)begin
			//     IF_ID_IR <= 32'b0;
			// 	ID_EX_IR <= 32'b0;
			// 	EX_MEM_IR<= ID_EX_IR;
			// 	MEM_WB_IR<= EX_MEM_IR;			
			// end
		end
    end

    IF_Mod instance_IF_Mod(
        .clk(clk),
		.rst(rst),
		.next_state_enable(next_state_enable),
		.W_before_R_LD(W_before_R_LD),
		.W_before_R_LD_MEM(W_before_R_LD_MEM),
		.Instruction(Instruction),
		.Inst_Valid(Inst_Valid),
		.Inst_Req_Ready(Inst_Req_Ready),
		.PC(IF_PC),
		.next_pc(next_pc),
		.MEM_Mod_Ready(MEM_Mod_Ready),
		.jorb_type(jorb_type),
		.is_jorb(is_jorb),
        .Inst_Ready(Inst_Ready),
		.Inst_Req_Valid(Inst_Req_Valid),
		.IF_Mod_Ready(IF_Mod_Ready),
		.IR(IF_IR)
    );

  //------------------------------------------------------------------------------------  
  //ID_Mod
  //------------------------------------------------------------------------------------

	wire [31:0] shifter_A;
	wire [31:0] shifter_B;
	wire [31:0] ALU_A;
	wire [31:0] ALU_B;
	wire [ 1:0] shifter_op;
	wire [ 2:0] ALU_op;
	wire [ 4:0] rs1;
	wire [ 4:0] rs2;
	reg [31:0] ID_EX_shifter_A;
	reg [31:0] ID_EX_shifter_B;
	reg [31:0] ID_EX_ALU_A;
	reg [31:0] ID_EX_ALU_B;
	reg [ 1:0] ID_EX_shifter_op;
	reg [ 2:0] ID_EX_ALU_op;
	reg [ 4:0] ID_EX_rs1;
	reg [ 4:0] ID_EX_rs2;
    always @(posedge clk)begin
	    if(rst)begin
			ID_EX_ALU_A <= 32'b0;
			ID_EX_ALU_B <= 32'b0;
			ID_EX_ALU_op <= 3'b0;
			ID_EX_rs1 <= 5'b0;
			ID_EX_rs2 <= 5'b0;
			ID_EX_shifter_A <= 32'b0;
			ID_EX_shifter_B <= 32'b0;
			ID_EX_shifter_op <= 2'b0;
	    end
		else begin
		    if(next_state_enable && ~W_before_R_LD)begin
				ID_EX_ALU_A <= ALU_A;
			    ID_EX_ALU_B <= ALU_B;
			    ID_EX_ALU_op <= ALU_op;
			    ID_EX_rs1 <= rs1;
			    ID_EX_rs2 <= rs2;
			    ID_EX_shifter_A <= shifter_A;
			    ID_EX_shifter_B <= shifter_B;
			    ID_EX_shifter_op <= shifter_op;		
			end
			else if(next_state_enable && W_before_R_LD)begin
			    ID_EX_ALU_A <= 32'b0;
			    ID_EX_ALU_B <= 32'b0;
			    ID_EX_ALU_op <= 3'b0;
			    ID_EX_rs1 <= 5'b0;
			    ID_EX_rs2 <= 5'b0;
			    ID_EX_shifter_A <= 32'b0;
			    ID_EX_shifter_B <= 32'b0;
			    ID_EX_shifter_op <= 2'b0;
			end
		end
	end

	wire [31:0] ID_Write_data;
	reg  [31:0] ID_EX_Write_data;
	reg  [31:0] EX_MEM_Write_data;
	always @(posedge clk)begin
	    if(rst)begin
		    ID_EX_Write_data <= 32'b0;
			EX_MEM_Write_data <= 32'b0;
		end
		else begin
			if(next_state_enable && ~W_before_R_LD && ~is_jorb)begin
			    ID_EX_Write_data <= ID_Write_data;
			    EX_MEM_Write_data <= ID_EX_Write_data;			
			end
			else if(next_state_enable && (W_before_R_LD || is_jorb))begin
			    ID_EX_Write_data <= 32'b0;
			    EX_MEM_Write_data <= ID_EX_Write_data;			
			end
		end
	end
    ID_Mod instance_ID_Mod(
        .clk(clk),
		.rst(rst),
		.IR(IF_ID_IR),
		.PC(IF_ID_PC),
		.next_pc(next_pc),
		.jorb_type(jorb_type),
		.is_jorb(is_jorb),
		.shifter_A(shifter_A),
		.shifter_B(shifter_B),
		.ALU_A(ALU_A),
		.ALU_B(ALU_B),
		.shifter_op(shifter_op),
		.ALU_op(ALU_op),
		.rdata1(rdata1),
		.rdata2(rdata2),
		.rs1(rs1),
		.rs2(rs2),
		.RF_wen(ID_RF_wen),
		.RF_waddr(ID_RF_waddr),
		.Write_data(ID_Write_data),
		.W_before_R_reg1(W_before_R_reg1),
		.W_before_R_reg2(W_before_R_reg2),
		.Ahead_value1(Ahead_value1),
		.Ahead_value2(Ahead_value2)
	); 

//----------------------------------------------------------------------
//EX_Mod
//----------------------------------------------------------------------
    wire [31:0] EX_ALU_result;
	wire [31:0] EX_shifter_result;
	reg  [31:0] EX_MEM_ALU_result;
	reg  [31:0] EX_MEM_shifter_result;
	always @(posedge clk)begin
	    if(rst)begin
		    EX_MEM_ALU_result <= 32'b0;
			EX_MEM_shifter_result <= 32'b0;
		end
        else begin
		    if(next_state_enable)begin
			    EX_MEM_ALU_result <= EX_ALU_result;
				EX_MEM_shifter_result <= EX_shifter_result;
			end
		end
	end

	EX_Mod instance_EX_Mod(
        .clk(clk),
		.rst(rst),
		.shifter_A(ID_EX_shifter_A),
		.shifter_B(ID_EX_shifter_B),
		.shifter_op(ID_EX_shifter_op),
		.ALU_A(ID_EX_ALU_A),
		.ALU_B(ID_EX_ALU_B),
		.ALU_op(ID_EX_ALU_op),
		.PC(ID_EX_PC),
		.IR(ID_EX_IR),
		.ALU_result(EX_ALU_result),
		.Shifter_result(EX_shifter_result),
		.EX_RF_wdata(EX_RF_wdata)
	);

//--------------------------------------------------------------------
//MEM_Mod
//--------------------------------------------------------------------
    wire [31:0] MEM_RF_wdata;
	reg  [31:0] MEM_WB_RF_wdata;
	always @(posedge clk)begin
	    if(rst)begin
		    MEM_WB_RF_wdata <= 32'b0;
		end
		else if(next_state_enable)begin
		    MEM_WB_RF_wdata <= MEM_RF_wdata;
		end
	end   

    MEM_Mod instance_MEM_MOD(
        .clk(clk),
		.rst(rst),
		.next_state_enable(next_state_enable),
		.IR(EX_MEM_IR),
		.PC(EX_MEM_PC),
		.ALU_result(EX_MEM_ALU_result),
		.Shifter_result(EX_MEM_shifter_result),
		.read_data(Read_data),
		.Read_data_Valid(Read_data_Valid),
		.Mem_Req_Ready(Mem_Req_Ready),
		.Address(Address),
		.Write_strb(Write_strb),
		.Write_data_in(EX_MEM_Write_data),
		.Write_data(Write_data),
		.MemRead(MemRead),
		.MemWrite(MemWrite),
		.Read_data_Ready(Read_data_Ready),
		.RF_wdata(MEM_RF_wdata),
		.MEM_Mod_Ready(MEM_Mod_Ready)
	);	

//--------------------------------------------------------------------
//WB_Mod
//--------------------------------------------------------------------
    WB_Mod instance_WB_Mod(
        .RF_waddr(MEM_WB_RF_waddr),
		.RF_wen(MEM_WB_RF_wen),
		.RF_wdata(MEM_WB_RF_wdata),
		.PC(MEM_WB_PC),
		.rst(rst),
		.clk(clk),
		.next_state_enable(next_state_enable),
		.inst_retire(inst_retire)
	);

//功能计数器
   reg [31:0] cycle_cnt;
   always @(posedge clk) begin
      if(rst == 1'b1) begin
	     cycle_cnt <= 32'b0;
	  end
	  else begin
	     cycle_cnt <= cycle_cnt + 32'b1;
	  end
   end
   assign cpu_perf_cnt_0 = cycle_cnt;

    reg [31:0] ins_cnt;
	assign cpu_perf_cnt_1  = ins_cnt;
    always @(posedge clk) begin
        if (rst)
            ins_cnt <= 32'b0;
        else if (ID_EX_IR != 32'b0 && next_state_enable)
            ins_cnt <= ins_cnt + 32'b1;
    end
   

endmodule


//------------------------------------------------------------
//IF模块
module IF_Mod(
    input             clk,
    input             rst,
	input             next_state_enable,
	input             W_before_R_LD,
	input             W_before_R_LD_MEM,
    input      [31:0] Instruction,
	input             Inst_Valid,
	input             Inst_Req_Ready,
	input             jorb_type,
	input             is_jorb,       //不是跳转的话，直接取pc+4，如果是，那就重新取
	output reg [31:0] PC,            //当前指令的pc，要传给ID
	input      [31:0] next_pc,       //ID译码计算出下一条指令的pc，用来取指
    input             MEM_Mod_Ready, //用于更新W_before_R_LD_reg,让MEM访存结束后再动IF
	output            Inst_Ready,
	output            Inst_Req_Valid,
	output            IF_Mod_Ready,
	output reg [31:0] IR
);
    always @(posedge clk)begin
        if (rst) begin
            PC <= 32'b0;
        end
        // else if(current_state == RDY)begin
		// 	PC <= {{32{is_jorb}} & next_pc} | {{32{!is_jorb}} & {PC+4}};
        // end
		else if(current_state == INIT )begin
		    PC <= next_pc;
		end
		// else if(current_state == IF && W_before_R_LD_MEM)begin
		//     PC <= next_pc;
		// end
	end

    reg  [4:0] current_state_reg;
	reg  [4:0] next_state_reg;
	wire [4:0] current_state;
	assign current_state = current_state_reg;

	localparam INIT = 5'b00001,//用于W_before_R_LD时，等待;用于更新PC
	           IF   = 5'b00010,
			   IW   = 5'b00100,
			   STA  = 5'b01000,
			   RDY  = 5'b10000; // 之前打算用于更新PC，debug之后

	// reg W_before_R_LD_reg; //用于保存先写后读信号，这样可以等待MEM先访存
	// always @(posedge clk)begin
	// 	if(MEM_Mod_Ready)begin
	//         W_before_R_LD_reg <= W_before_R_LD;		
	// 	end
	// end

	always @(posedge clk)begin
	    if(rst) begin
		    current_state_reg <= INIT;
		end
		else begin
		    current_state_reg <= next_state_reg;
		end
	end

	always @(*)begin
        case (current_state_reg)
		    INIT:begin
                if(rst)begin
				    next_state_reg = current_state_reg;
				end
				else begin
                    if(jorb_type)begin
					    if(W_before_R_LD)begin
						    next_state_reg = current_state_reg;
						end
						else if(~W_before_R_LD && ~MEM_Mod_Ready)begin
						    next_state_reg = current_state_reg;
						end
						else begin
						    next_state_reg = IF;
						end					    
					end
					else begin
					    if(W_before_R_LD)begin
						    next_state_reg = current_state_reg;
						end
						else begin
						    next_state_reg = IF;
						end
					end
				    // if(W_before_R_LD)begin
					//     next_state_reg = current_state_reg;
					// end
					// else begin
					//     next_state_reg = IF;
					// end
				end
			end
			IF:begin
				if(Inst_Req_Ready == 1'b1 ) begin //
					next_state_reg = IW;
				end
				else begin 
					next_state_reg = current_state_reg;
				end			
			end
			IW:begin
				if(Inst_Valid == 1'b1) begin
					next_state_reg = STA;
				end
				else begin
					next_state_reg = current_state_reg;
				end
			end
            STA:begin
			    if(next_state_enable)begin
				    next_state_reg = INIT;
				end
				else begin
				    next_state_reg = current_state_reg;
				end
			end
			// RDY:begin
			// 	// if(W_before_R_LD)begin
			// 	// 	next_state_reg = current_state_reg;
			// 	// end
			// 	// else begin
			// 	    next_state_reg = INIT;
			// 	//end	
			// end
		endcase			
	end

    always @(posedge clk) begin
        if (rst) begin
           IR <= 32'b0;
        end
        else if (current_state == IW && Inst_Valid) begin
            IR <= Instruction;
        end
    end

	assign Inst_Ready = (current_state == INIT) || (current_state == IW);
	assign Inst_Req_Valid = (current_state == IF);
	assign IF_Mod_Ready = current_state == STA || (current_state == INIT && (W_before_R_LD ));
endmodule

//-------------------------------------------------------------
//ID模块
module ID_Mod(
	input         clk,
    input         rst,
    input  [31:0] IR,
	input  [31:0] PC,
	output [31:0] next_pc,
	output        jorb_type,//记录这条指令是不是会跳转
	output        is_jorb,
	output [31:0] shifter_A,
	output [ 4:0] shifter_B,
	output [31:0] ALU_A,
	output [31:0] ALU_B,
	output [ 1:0] shifter_op,
	output [ 2:0] ALU_op,
	input  [31:0] rdata1,
	input  [31:0] rdata2,
	output [ 4:0] rs1,
	output [ 4:0] rs2,
	output        RF_wen,
	output [ 4:0] RF_waddr,
	output [31:0] Write_data,
	input         W_before_R_reg1,
	input         W_before_R_reg2,
    input  [31:0] Ahead_value1,
	input  [31:0] Ahead_value2 
);

    //指令码分解 
    wire [6:0] funct_length7 = IR[31:25];
	assign     rs2           = IR[24:20];
	assign     rs1           = IR[19:15];
	wire [2:0] funct_length3 = IR[14:12];
	wire [4:0] rd            = IR[11: 7];
    wire [6:0] Opcode        = IR[ 6: 0];

    //指令码类型分类
	wire U_type;
	wire J_type;
	wire I_type;
	wire B_type;
	wire S_type;
	wire R_type;
	wire I_LOAD_type;

	assign U_type      = Opcode == 7'b0110111 | Opcode == 7'b0010111;
    assign J_type      = Opcode == 7'b1101111;
	assign I_type      = Opcode == 7'b0010011 | Opcode == 7'b1100111;
	assign B_type      = Opcode == 7'b1100011;
	assign S_type      = Opcode == 7'b0100011;
    assign R_type      = Opcode == 7'b0110011;
	assign I_LOAD_type = Opcode == 7'b0000011;

	wire [31:0] I_imm;
	wire [31:0] S_imm;
	wire [31:0] B_imm;
	wire [31:0] U_imm;
	wire [31:0] J_imm;

    wire [31:0] renew_rdata1;//在前递之后，更新的rdata1
    wire [31:0] renew_rdata2;//在前递之后，更新的rdata2
	assign renew_rdata1 = W_before_R_reg1 ? Ahead_value1 : rdata1;
	assign renew_rdata2 = W_before_R_reg2 ? Ahead_value2 : rdata2; 

	assign shifter_op = funct_length7[5] == 1'b1 ? 2'b11 : funct_length3[2:1];
    assign shifter_B  = R_type ? renew_rdata2[4:0] : I_imm[4:0];
	assign shifter_A  = renew_rdata1;

	assign I_imm = {{20{IR[31]}},IR[31:20]};
	assign S_imm = {{20{IR[31]}},funct_length7,rd};
	assign B_imm = {{20{IR[31]}},IR[7],IR[30:25],IR[11:8],1'b0};
	assign U_imm = {IR[31:12],{12{1'b0}}};
	assign J_imm = {{12{IR[31]}},IR[19:12],IR[20],IR[30:21],1'b0};

	assign ALU_A = Opcode == 7'b0110111 ? 32'b0     :
	               Opcode == 7'b0010111 ? PC    :
				                          renew_rdata1; 
    assign ALU_B = (Opcode == 7'b0110111 || Opcode == 7'b0010111) ? U_imm :
	                Opcode == 7'b0100011                          ? S_imm :
				   (Opcode == 7'b0000011 || Opcode == 7'b0010011) ? I_imm :
				                                                    renew_rdata2;

    assign ALU_op = (Opcode == 7'b0010011 | Opcode == 7'b0110011) && funct_length3 == 3'b111 ? 3'b000 :
	               (Opcode == 7'b0010011 | Opcode == 7'b0110011) && funct_length3 == 3'b110 ? 3'b001 :
				   (Opcode == 7'b0010011 | Opcode == 7'b0110011) && funct_length3 == 3'b100 ? 3'b100 :
				   (Opcode == 7'b0010011 | Opcode == 7'b0110011) && funct_length3 == 3'b011 || (Opcode == 7'b1100011 & funct_length3[2] & funct_length3[1]) ? 3'b011 :
				   ((Opcode == 7'b0010011 | Opcode == 7'b0110011) && funct_length3 == 3'b010) || (Opcode == 7'b1100011 & funct_length3[2] & ~funct_length3[1]) ? 3'b111 :
				   (Opcode == 7'b0110011 && funct_length7[5] == 1'b1) || (Opcode == 7'b1100011 & ~funct_length3[2]) ? 3'b110 :
				   (Opcode == 7'b0110011 && funct_length7[0] == 1'b1)                                               ? 3'b101 :
				                                                                                                      3'b010 ;

 	assign Write_data = funct_length3 == 3'b000 ? {4{renew_rdata2[ 7:0]}} :
	                    funct_length3 == 3'b001 ? {2{renew_rdata2[15:0]}} :
						                          renew_rdata2            ;

    assign RF_wen = IR == 32'b0          ? 0 :
	                Opcode == 7'b1100011 ? 0 :
	                Opcode == 7'b0100011 ? 0 : 
					                       1 ;
    assign RF_waddr = rd;
    
	//提前使用alu判断is_jorb，简化分支预测
	wire Zero;
    wire [31:0] ALU_result;
	alu instance_alu(
        .A(ALU_A),
		.B(ALU_B),
		.ALUop(ALU_op),
		.Zero(Zero),
		.Result(ALU_result)
	);
	
	assign jorb_type = Opcode == 7'b1101111 || Opcode == 7'b1100111 || Opcode == 7'b1100011;
    assign is_jorb = Opcode == 7'b1101111 || Opcode == 7'b1100111    ? 1'b1            :
                     Opcode != 7'b1100011                            ? 1'b0            :
			     	 funct_length3 == 3'b000                         ? Zero            :
			    	 funct_length3 == 3'b001                         ? ~Zero           :
				     funct_length3 == 3'b100|funct_length3 == 3'b110 ? ALU_result [0] :
				     funct_length3 == 3'b101|funct_length3 == 3'b111 ? ~ALU_result[0] :
				                                                      1'b0            ;

    wire [31:0] tem_renew_rdata1;   
    assign tem_renew_rdata1 = renew_rdata1+I_imm; 
	assign next_pc = IR == 32'b0          ? PC       :
	                 is_jorb == 1'b0      ? PC+4     : 
	                 Opcode == 7'b1101111 ? PC+J_imm :
                     Opcode == 7'b1100111 ? {tem_renew_rdata1[31:1],1'b0} :
				     Opcode == 7'b1100011 ? PC+B_imm : 32'b0;

endmodule

//-------------------------------------------------------------
//EX模块
module EX_Mod(
	input         clk,
    input         rst,
	input  [31:0] shifter_A,
    input  [ 4:0] shifter_B,
	input  [ 1:0] shifter_op,
	output [31:0] Shifter_result,
	input  [31:0] ALU_A,
	input  [31:0] ALU_B,
	input  [ 2:0] ALU_op,
	output [31:0] ALU_result,
	input  [31:0] PC,
	input  [31:0] IR,
    output [31:0] EX_RF_wdata
);
//shifter模块
    shifter instance_shifter(
		.Shiftop(shifter_op),
		.A(shifter_A),
		.B(shifter_B),
	    .Result(Shifter_result)
	);

//ALU模块
    alu instance_alu(
        .A(ALU_A),
		.B(ALU_B),
		.ALUop(ALU_op),
		.Zero(Zero),
		.Result(ALU_result)
	);

	wire [6:0] Opcode;
	wire Zero;
	wire [2:0] funct_length3;
	assign funct_length3 = IR[14:12];
	assign Opcode = IR[6:0];
	assign EX_RF_wdata = Opcode == 7'b1101111 || Opcode == 7'b1100111 ? PC+4             :
					     Opcode == 7'b0010011 && (funct_length3 == 3'b001 || funct_length3 == 3'b101) ? Shifter_result :
					     Opcode == 7'b0110011 && (funct_length3 == 3'b001 || funct_length3 == 3'b101) ? Shifter_result :
					                                                                                    ALU_result     ; 
endmodule
    
//-------------------------------------------------------------
//MEM模块
module MEM_Mod(
    input         clk,
    input         rst,
	input         next_state_enable,
	input  [31:0] IR,
	input  [31:0] PC,
	input  [31:0] ALU_result,
	input  [31:0] Shifter_result,
	input  [31:0] read_data,
	input         Read_data_Valid,
	input         Mem_Req_Ready,
	output [31:0] Address,
	output [ 3:0] Write_strb,
	input  [31:0] Write_data_in,
	output [31:0] Write_data,
	output        MemRead,
	output        MemWrite, 
	output        Read_data_Ready,
	output [31:0] RF_wdata,
	output        MEM_Mod_Ready
);
    wire [6:0] funct_length7 = IR[31:25];
	wire [4:0] rs2           = IR[24:20];
	wire [4:0] rs1           = IR[19:15];
	wire [2:0] funct_length3 = IR[14:12];
	wire [4:0] rd            = IR[11: 7];
    wire [6:0] Opcode        = IR[ 6: 0];

    assign Address = {ALU_result[31:2],2'b00};

	wire [3:0] SB_strb;
	wire [3:0] SH_strb;
	wire [3:0] SW_strb;
	assign SB_strb = ALU_result[1:0] == 2'b00 ? 4'b0001 :
	                 ALU_result[1:0] == 2'b01 ? 4'b0010 :
					 ALU_result[1:0] == 2'b10 ? 4'b0100 :
					                            4'b1000 ;
	assign SH_strb = ALU_result[1:0] == 2'b00 ? 4'b0011 :
	                                            4'b1100 ;
	assign SW_strb = 4'b1111;
	assign Write_strb = funct_length3 == 3'b000 ? SB_strb :
	                    funct_length3 == 3'b001 ? SH_strb :
						                          SW_strb ;
	assign Write_data = Write_data_in;
    
	reg [6:0] current_state;
	reg [6:0] next_state;
	localparam INIT   = 7'b0000001,
	           RDY    = 7'b0000010,
	           ST     = 7'b0000100,
	           NMEM   = 7'b0001000,
			   LD     = 7'b0010000,
               RDW    = 7'b0100000,
			   STA    = 7'b1000000;

    wire RDY_ST;
	wire RDY_NMEM;
	wire RDY_LD;
	wire I_type;
	wire B_type;
	wire S_type;
	wire U_type;
	wire J_type;
	wire R_type;
	wire I_LOAD_type;
	assign U_type      = Opcode == 7'b0110111 | Opcode == 7'b0010111;
    assign J_type      = Opcode == 7'b1101111;
	assign I_type      = Opcode == 7'b0010011 | Opcode == 7'b1100111;
	assign B_type      = Opcode == 7'b1100011;
	assign S_type      = Opcode == 7'b0100011;
    assign R_type      = Opcode == 7'b0110011;
	assign I_LOAD_type = Opcode == 7'b0000011;
	assign RDY_NMEM = B_type | R_type | I_type | J_type | U_type;					
	assign RDY_LD = I_LOAD_type;
	assign RDY_ST = S_type;

	always @(posedge clk) begin
	    if(rst) begin
		    current_state <= INIT;
		end
		else begin
		    current_state <= next_state;
		end
	end

	always @(*)begin
	    case(current_state)
		    INIT:begin
			    if(IR != 32'b0)begin
				    next_state = RDY;
				end
				else begin
				    next_state = current_state;
				end
			end
		    RDY:begin
			    if(RDY_LD)begin
				    next_state = LD;
				end
				else if(RDY_NMEM)begin
				    next_state = NMEM;
				end
				else if(RDY_ST) begin
				    next_state = ST;
				end
				else begin 
				    next_state = STA;
				end
			end
			ST:begin
			    if(Mem_Req_Ready)begin
				    next_state = STA;
				end
				else begin
				    next_state = current_state;
				end
			end
			NMEM:begin
				next_state = STA;
			end
			LD:begin
			    if(Mem_Req_Ready)begin
				    next_state = RDW;
				end
				else begin
				    next_state = current_state;
				end
			end
			RDW:begin
			    if(Read_data_Valid)begin
				    next_state = STA;
				end
				else begin
				    next_state = current_state;
				end
			end
			STA:begin
			    if(next_state_enable)begin
				    next_state = RDY;
				end
				else begin
				    next_state = current_state;
				end
			end
		endcase
	end

	assign Read_data_Ready = current_state == RDW;
	assign MemRead         = current_state == LD;
	assign MemWrite        = current_state == ST;
    assign MEM_Mod_Ready   = (current_state == INIT) | (current_state == STA);

	wire [31:0] Read_data_extend;
	wire [31:0] Read_data_extend_LB;
	wire [31:0] Read_data_extend_LBU;
	wire [31:0] Read_data_extend_LH;
	wire [31:0] Read_data_extend_LHU;
	wire [31:0] Read_data_extend_LW;
    reg  [31:0] Read_data;
	always @(posedge clk)begin
	    if(current_state == RDW)begin
		    Read_data <= read_data;
		end
	end

	assign Read_data_extend_LB = ALU_result[1:0] == 2'b00 ? {{24{Read_data[ 7]}},Read_data[ 7: 0]} :
	                             ALU_result[1:0] == 2'b01 ? {{24{Read_data[15]}},Read_data[15: 8]} :
								 ALU_result[1:0] == 2'b10 ? {{24{Read_data[23]}},Read_data[23:16]} :
								                            {{24{Read_data[31]}},Read_data[31:24]} ;
	assign Read_data_extend_LH = ALU_result[1:0] == 2'b00 ? {{16{Read_data[15]}},Read_data[15: 0]} :
	                                                        {{16{Read_data[31]}},Read_data[31:16]} ;
	assign Read_data_extend_LW = Read_data;
	assign Read_data_extend_LBU = ALU_result[1:0] == 2'b00 ? {{24{1'b0}},Read_data[ 7: 0]} :
	                              ALU_result[1:0] == 2'b01 ? {{24{1'b0}},Read_data[15: 8]} :
								  ALU_result[1:0] == 2'b10 ? {{24{1'b0}},Read_data[23:16]} :
								                             {{24{1'b0}},Read_data[31:24]} ;
    assign Read_data_extend_LHU = ALU_result[1:0] == 2'b00 ? {{16{1'b0}},Read_data[15: 0]} :
	                                                         {{16{1'b0}},Read_data[31:16]} ;
	assign Read_data_extend = funct_length3 == 3'b000 ? Read_data_extend_LB :
	                          funct_length3 == 3'b001 ? Read_data_extend_LH :
							  funct_length3 == 3'b010 ? Read_data_extend_LW :
							  funct_length3 == 3'b100 ? Read_data_extend_LBU:
							                            Read_data_extend_LHU;
	assign RF_wdata = Opcode == 7'b1101111 || Opcode == 7'b1100111 ? PC+4             :
	                  Opcode == 7'b0000011                         ? Read_data_extend :
					  Opcode == 7'b0010011 && (funct_length3 == 3'b001 || funct_length3 == 3'b101) ? Shifter_result :
					  Opcode == 7'b0110011 && (funct_length3 == 3'b001 || funct_length3 == 3'b101) ? Shifter_result :
					                                                                                 ALU_result     ;  
endmodule

//--------------------------------------------------------------
//WB模块
module WB_Mod(
    input  [ 4:0] RF_waddr,
	input         RF_wen,
	input         clk,
	input         rst,
	input  [31:0] RF_wdata,
	input  [31:0] PC,
	input         next_state_enable,
	output [69:0] inst_retire
);
	assign inst_retire = {70{next_state_enable}} & {RF_wen, RF_waddr, RF_wdata,PC};
endmodule