`timescale 10ns / 1ns
`include "cache_config.vh"

module icache_top (
	input	      clk,
	input	      rst,
	
	//CPU interface
	/** CPU instruction fetch request to Cache: valid signal */
	input         from_cpu_inst_req_valid,
	/** CPU instruction fetch request to Cache: address (4 byte alignment) */
	input  [31:0] from_cpu_inst_req_addr, 
	/** Acknowledgement from Cache: ready to receive CPU instruction fetch request */
	output        to_cpu_inst_req_ready,
	
	/** Cache responses to CPU: valid signal */
	output        to_cpu_cache_rsp_valid,
	/** Cache responses to CPU: 32-bit Instruction value */
	output [31:0] to_cpu_cache_rsp_data, 
	/** Acknowledgement from CPU: Ready to receive Instruction */
	input	      from_cpu_cache_rsp_ready,

	//Memory interface (32 byte aligned address)
	/** Cache sending memory read request: valid signal */
	output        to_mem_rd_req_valid,
	/** Cache sending memory read request: address (32 byte alignment) */
	output [31:0] to_mem_rd_req_addr, 
	/** Acknowledgement from memory: ready to receive memory read request */
	input         from_mem_rd_req_ready,

	/** Memory return read data: valid signal of one data beat */
	input         from_mem_rd_rsp_valid,
	/** Memory return read data: 32-bit one data beat */
	input  [31:0] from_mem_rd_rsp_data, 
	/** Memory return read data: if current data beat is the last in this burst data transmission */
	input         from_mem_rd_rsp_last,
	/** Acknowledgement from cache: ready to receive current data beat */
	output        to_mem_rd_rsp_ready
);

//TODO: Please add your I-Cache code here

    genvar i; // 用于generate for循环的变量
    wire wen                  [`CACHE_WAY - 1 : 0]; // 写使能信号
    wire hit_all_way          [`CACHE_WAY - 1 : 0]; // 记录哪一路被命中
	wire [2:0]                             hit_way; // 记录哪一路被命中，该信号位宽大于 log2 `CACHE_WAY 就可以
    wire [2:0]                        replaced_way; // 记录替换哪一路，位宽设置同hit_way	
	wire valid_all_way        [`CACHE_WAY - 1 : 0]; // 记录哪一路有效
	wire last_hit_time_all_way[`CACHE_WAY - 1 : 0]; // 记录每一路的时间戳
	wire [`LINE_LEN - 1 : 0] block_all_way [`CACHE_WAY - 1 : 0]; // 记录每一路的目标组（用incex选取）字块内数据
    wire [`TAG_LEN - 1  : 0] tag_all_way   [`CACHE_WAY - 1 : 0]; // 记录每一路的各组字块的tag信息

/* 将CPU地址分解为不同字段 */
    wire [     `TAG_LEN - 1 : 0]  tag;    
    wire [ `INDEX_WIDTH - 1 : 0]  index;  
    wire [`OFFSET_WIDTH - 1 : 0]  offset; 

    assign tag    = from_cpu_inst_req_addr[`DATA_WIDTH - 1 : `OFFSET_WIDTH + `INDEX_WIDTH];
    assign index  = from_cpu_inst_req_addr[`OFFSET_WIDTH + `INDEX_WIDTH - 1 : `OFFSET_WIDTH];
    assign offset = from_cpu_inst_req_addr[`OFFSET_WIDTH - 1 : 0];

 /* 有限状态机 (FSM) 状态定义 */
 //只有读，没有写，也就没有dirty与clear之分
    localparam INIT  = 6'b000001,/* 初始化状态 */
	           WAIT  = 6'b000010,/* 等待CPU请求, 并读取标签 */
	 		   MISS  = 6'b000100,/* 未命中 */
			   REFILL= 6'b001000,/* 填充: 从内存读取新块到Cache */
			   HIT   = 6'b010000,/* 命中 */
			   SEND  = 6'b100000;/* 送出: 将Cache中的数据发送给CPU */ 


// 状态转移模块（三段式表达）
    reg  [5:0] curr_state;
	wire [5:0] next_state;
    always @(posedge clk) begin
        if (rst)
            curr_state <= INIT;
        else
            curr_state <= next_state;
    end

    assign next_state = {{6{curr_state == INIT}} & WAIT}
	                   |{{6{curr_state == WAIT}} & (from_cpu_inst_req_valid ? ( hit ? HIT : MISS) : WAIT)}
					   |{{6{curr_state == MISS}} & (from_mem_rd_req_ready ? REFILL : MISS)}
					   |{{6{curr_state == REFILL}} & ((from_mem_rd_rsp_last && from_mem_rd_rsp_valid) ? HIT : REFILL)}
					   |{{6{curr_state == HIT}} & SEND}
					   |{{6{curr_state == SEND}} & (from_cpu_cache_rsp_ready ? WAIT : SEND)};

    generate
        for (i = 0; i < `CACHE_WAY; i = i + 1) begin
            // 判断每一路是否命中：有效位为1且标签匹配
            assign hit_all_way[i] = valid_all_way[i] && (tag == tag_all_way[i]);

            // 填充时的写使能
            assign wen[i] = (replaced_way == i) && (curr_state == REFILL) && from_mem_rd_rsp_valid;
        end
    endgenerate

// 找出命中的是哪一路 (编码器)
    assign hit_way = {
        {3{hit_all_way[0]}} & 3'h0 |
        {3{hit_all_way[1]}} & 3'h1 |
        {3{hit_all_way[2]}} & 3'h2 |
        {3{hit_all_way[3]}} & 3'h3 //|
    //  {3{hit_all_way[4]}} & 3'h4 |
    //  {3{hit_all_way[5]}} & 3'h5
    };
// 总的命中信号
    assign hit  = hit_all_way[0] || hit_all_way[1] ||
                  hit_all_way[2] || hit_all_way[3] ;//||
    //            hit_all_way[4] || hit_all_way[5];

    assign to_cpu_inst_req_ready  = curr_state == WAIT;
	assign to_mem_rd_rsp_ready    = curr_state == REFILL;
	assign to_cpu_cache_rsp_valid = curr_state == SEND;
	assign to_mem_rd_req_valid    = curr_state == MISS;
    assign to_cpu_cache_rsp_data  = {`DATA_WIDTH{curr_state == SEND}} & block_all_way[hit_way][{offset, 3'b0} +: `DATA_WIDTH];
    assign to_mem_rd_req_addr     = {from_cpu_inst_req_addr[31:5],5'b0}; // 一定要把offset置0，因为替换操作永远是从这个地址开始存后面8个字，当前地址内容
	                                                                     // 一定在缓存字块的第一个字，如果不置0，后面取缓存字块中data会出错

	wire [31:0] to_cache_data;
    assign to_cache_data          = from_mem_rd_rsp_data;


 // 时间戳计数器 (用于LRU)
    reg [`TIME_WIDTH:0] hit_seq_cnt;
    always @(posedge clk) begin
        if (rst)
            hit_seq_cnt <= `TIME_WIDTH'b0;
        else if ((curr_state == WAIT) && hit && hit_seq_cnt != `MAX_32_BIT) // 仅在WAIT状态命中时递增
            hit_seq_cnt <= hit_seq_cnt + 1;
    end


//各模块实例化
generate
        for (i = 0; i < `CACHE_WAY; i = i + 1) begin

            // 实例化有效位存储阵列
            one_bit_array valid_array_inst (
                .clk(clk), .rst(rst), .wen(wen[i]),
                .waddr(index), .wdata(1'b1), // 写入时总是写1
                .raddr(index), .rdata(valid_all_way[i])
            );

            // 实例化标签存储阵列
            tag_array tag_array_inst (
                .clk(clk), .wen(wen[i]),
                .waddr(index), .wdata(tag), // 写入CPU地址的tag部分
                .raddr(index), .rdata(tag_all_way[i])
            );

            // 实例化数据存储阵列
            data_array data_array_inst (
                .clk(clk), .wen(wen[i]),
                .wr_hit(1'b0), // 告知data_array：icache只可能是缓存填充，设为恒0
                .offset(offset),
                .strb(4'b1111), //没有写操作，在缓存替换时掩码也无用，设成恒1111
                .waddr(index), .wdata(to_cache_data), // 写入的数据
                .raddr(index), .rdata(block_all_way[i])
            );

            // 实例化LRU时间戳存储阵列
            last_hit_array last_hit_array_inst (
                .clk(clk), .rst(rst),
                .wen((curr_state == WAIT) && hit_all_way[i]), // 仅在WAIT状态下命中时更新
                .waddr(index), .wdata(hit_seq_cnt ), // 写入当前的时间戳
                .raddr(index), .rdata(last_hit_time_all_way[i])
            );
        end
    endgenerate

    replacement replacement_inst (
        .clk(clk), .rst(rst),
        .data_0(last_hit_time_all_way[0]), .data_1(last_hit_time_all_way[1]),
        .data_2(last_hit_time_all_way[2]), .data_3(last_hit_time_all_way[3]),
    //  .data_4(last_hit_time_all_way[4]), .data_5(last_hit_time_all_way[5]),
        .replaced_way(replaced_way) // 输出要被替换的路
    );

endmodule


