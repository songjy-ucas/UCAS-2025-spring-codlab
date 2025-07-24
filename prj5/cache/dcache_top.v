`timescale 10ns / 1ns
`include "cache_config.vh"

module dcache_top (
    input          clk,
    input          rst,

    //CPU interface
    /** CPU memory/IO access request to Cache: valid signal */
    input          from_cpu_mem_req_valid,
    /** CPU memory/IO access request to Cache: 0 for read; 1 for write (when req_valid is high) */
    input          from_cpu_mem_req,
    /** CPU memory/IO access request to Cache: address (4 byte alignment) */
    input   [31:0] from_cpu_mem_req_addr,
    /** CPU memory/IO access request to Cache: 32-bit write data */
    input   [31:0] from_cpu_mem_req_wdata,
    /** CPU memory/IO access request to Cache: 4-bit write strobe */
    input   [ 3:0] from_cpu_mem_req_wstrb,
    /** Acknowledgement from Cache: ready to receive CPU memory access request */
    output         to_cpu_mem_req_ready,

    /** Cache responses to CPU: valid signal */
    output         to_cpu_cache_rsp_valid,
    /** Cache responses to CPU: 32-bit read data */
    output  [31:0] to_cpu_cache_rsp_data,
    /** Acknowledgement from CPU: Ready to receive read data */
    input          from_cpu_cache_rsp_ready,

    //Memory/IO read interface
    /** Cache sending memory/IO read request: valid signal */
    output         to_mem_rd_req_valid,
    /** Cache sending memory read request: address
      * 4 byte alignment for I/O read
      * 32 byte alignment for cache read miss */
    output  [31:0] to_mem_rd_req_addr,
    /** Cache sending memory read request: burst length
      * 0 for I/O read (read only one data beat)
      * 7 for cache read miss (read eight data beats) */
    output  [ 7:0] to_mem_rd_req_len, //<------- 这个信号是决定内存一次要和cache交互多少个字的数据的关键（icache中默认为8，dcache中根据是否为旁路，有8和1两种情况）
    /** Acknowledgement from memory: ready to receive memory read request */
    input          from_mem_rd_req_ready,

    /** Memory return read data: valid signal of one data beat */
    input          from_mem_rd_rsp_valid,
    /** Memory return read data: 32-bit one data beat */
    input   [31:0] from_mem_rd_rsp_data,
    /** Memory return read data: if current data beat is the last in this burst data transmission */
    input          from_mem_rd_rsp_last,
    /** Acknowledgement from cache: ready to receive current data beat */
    output         to_mem_rd_rsp_ready,

    //Memory/IO write interface
    /** Cache sending memory/IO write request: valid signal */
    output         to_mem_wr_req_valid,
    /** Cache sending memory write request: address
      * 4 byte alignment for I/O write
      * 4 byte alignment for cache write miss
      * 32 byte alignment for cache write-back */
    output  [31:0] to_mem_wr_req_addr,
    /** Cache sending memory write request: burst length
        * 0 for I/O write (write only one data beat)
        * 0 for cache write miss (write only one data beat)
        * 7 for cache write-back (write eight data beats) */
    output  [ 7:0] to_mem_wr_req_len,
    /** Acknowledgement from memory: ready to receive memory write request */
    input          from_mem_wr_req_ready,

    /** Cache sending memory/IO write data: valid signal for current data beat */
    output         to_mem_wr_data_valid,
    /** Cache sending memory/IO write data: current data beat */
    output  [31:0] to_mem_wr_data,
    /** Cache sending memory/IO write data: write strobe
      * 4'b1111 for cache write-back
      * other values for I/O write and cache write miss according to the original CPU request*/
    output  [ 3:0] to_mem_wr_data_strb,
    /** Cache sending memory/IO write data: if current data beat is the last in this burst data transmission */
    output         to_mem_wr_data_last,
    /** Acknowledgement from memory/IO: ready to receive current data beat */
    input          from_mem_wr_data_ready
);

    // TODO: Please add your D-Cache code here

  /*
    ** {=====================================================================
    ** 信号线和寄存器定义
    ** ======================================================================
    */

    genvar i; /* 用于generate for循环的变量 */


    /* 写使能信号 */
    wire  wen_rf   [`CACHE_WAY - 1 : 0]; /* 用于缓存填充(refill)的写使能 */
    wire  wen_hit  [`CACHE_WAY - 1 : 0]; /* 用于写命中(write-hit)的写使能 */
    wire  wen      [`CACHE_WAY - 1 : 0]; /* 总的写使能信号 */


    /* 来自不同路的所有缓存块信息 */
    wire                          valid_all_way          [`CACHE_WAY - 1 : 0]; // 所有路的有效位
    wire                          dirty_all_way          [`CACHE_WAY - 1 : 0]; // 所有路的脏位
    wire [     `TAG_LEN - 1 : 0]  tag_all_way            [`CACHE_WAY - 1 : 0]; // 所有路的标签
    wire [    `LINE_LEN - 1 : 0]  block_all_way          [`CACHE_WAY - 1 : 0]; // 所有路的数据块
    wire [  `TIME_WIDTH - 1 : 0]  last_hit_time_all_way  [`CACHE_WAY - 1 : 0]; // 所有路的上一次命中时间（用于LRU）


    /* 从CPU/内存到Cache的数据总线 */
    wire [  `DATA_WIDTH - 1 : 0]  to_cache_data;

    /* 从Cache到内存的数据通路 */
    reg  [    `LINE_LEN - 1 : 0]  block_tmp;        /* 用于写回的临时数据块寄存器 */
    wire [  `DATA_WIDTH - 1 : 0]  from_cache_wdata; /* 从Cache发往内存的数据 */
    reg  [                3 : 0]  write_cnt;        /* 写操作计数器 (用于突发写) */

    /* LRU替换策略相关 */
    reg  [  `TIME_WIDTH - 1 : 0]  hit_seq_cnt;      /* 命中序列计数器，作为时间戳 */



    /* 将CPU地址分解为不同字段 */
    wire [     `TAG_LEN - 1 : 0]  tag;    // 标签 ---- 确定了哪一个组，使用tag确定是哪一个路（没有路匹配，则说明miss）
    wire [ `INDEX_WIDTH - 1 : 0]  index;  // 索引 ---- 哪一个组
    wire [`OFFSET_WIDTH - 1 : 0]  offset; // 偏移 ---- 块内字地址


    /* 控制信号 */

    wire                    hit_all_way [`CACHE_WAY - 1:0]; /* 标记每一路是否命中 */
    wire [           2 : 0] hit_way;                        /* 命中的是哪一路？ */
    wire                    hit, miss;                      /* 最终的命中或未命中信号 */

    wire [           2 : 0] replaced_way;                   /* 被替换的是哪一路？ */
    wire                    dirty, clean;                   /* 被替换的行是脏的还是干净的？ */
    wire [`TAG_LEN - 1 : 0] replaced_tag;                   /* 被替换行的标签 */

    wire bypass; /* 是否为Bypass请求 (I/O空间或不可缓存区) */


    wire rd_last, wr_last; /* 标记读/写突发传输的最后一个节拍 */

    /* 有限状态机 (FSM) 状态定义 */
    localparam    INIT = 13'h0001; /* 初始化状态 */
    localparam    WAIT = 13'h0002; /* 等待CPU请求, 并读取标签 */
    localparam MISS_DT = 13'h0004; /* 未命中, 且替换块为脏块 */
    localparam    SYNC = 13'h0008; /* 同步: 将旧的脏块写回内存 */
    localparam MISS_CL = 13'h0010; /* 未命中, 且替换块为干净块 */
    localparam  REFILL = 13'h0020; /* 填充: 从内存读取新块到Cache */
    localparam  WR_HIT = 13'h0040; /* 写命中 */
    localparam  RD_HIT = 13'h0080; /* 读命中 */
    localparam  RD_CHE = 13'h0100; /* 检查/送出: 将Cache中的数据发送给CPU */ 
    localparam   BP_WR = 13'h0200; /* 旁路写: CPU -> 内存 (发送请求) */
    localparam   BP_RD = 13'h0400; /* 旁路读: CPU -> 内存 (发送请求) */
    localparam     WRW = 13'h0800; /* 旁路写等待: CPU -> 内存 (等待数据传输完成) */
    localparam     RDW = 13'h1000; /* 旁路读等待: 内存 -> CPU (等待数据传输完成) */


    wire state_INIT, state_WAIT, state_MISS_DT, state_SYNC, state_MISS_CL,
         state_REFILL, state_WR_HIT, state_RD_HIT, state_RD_CHE, state_BP_WR,
         state_BP_RD, state_WRW, state_RDW;

    reg  [12:0] curr_state; // 当前状态 (采用one-hot编码)
    wire [12:0] next_state; // 下一状态


    /*
    ** {=====================================================================
    ** 缓存存储阵列实例化
    ** ======================================================================
    */

    /* 使用generate语句生成6路缓存 */
    generate
        for (i = 0; i < `CACHE_WAY; i = i + 1) begin

            // 实例化有效位存储阵列
            one_bit_array valid_array_inst (
                .clk(clk), .rst(rst), .wen(wen[i]),
                .waddr(index), .wdata(1'b1), // 写入时总是写1
                .raddr(index), .rdata(valid_all_way[i])
            );

            // 实例化脏位存储阵列
            one_bit_array dirty_array_inst (
                .clk(clk), .rst(rst), .wen(wen[i]),
                .waddr(index), .wdata(wen_hit[i]), // 写命中时写1
                .raddr(index), .rdata(dirty_all_way[i])
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
                .wr_hit(state_WR_HIT), // 告知data_array是写命中还是填充
                .offset(offset),
                .strb(from_cpu_mem_req_wstrb),
                .waddr(index), .wdata(to_cache_data), // 写入的数据
                .raddr(index), .rdata(block_all_way[i])
            );

            // 实例化LRU时间戳存储阵列
            last_hit_array last_hit_array_inst (
                .clk(clk), .rst(rst),
                .wen(state_WAIT && hit_all_way[i]), // 仅在WAIT状态下命中时更新
                .waddr(index), .wdata(hit_seq_cnt ), // 写入当前的时间戳
                .raddr(index), .rdata(last_hit_time_all_way[i])
            );
        end
    endgenerate


    /*
    ** {=====================================================================
    ** 控制逻辑
    ** ======================================================================
    */


    /* 时间戳计数器 (用于LRU) */
    always @(posedge clk) begin
        if (rst)
            hit_seq_cnt <= `TIME_WIDTH'b0;
        else if (state_WAIT && hit && hit_seq_cnt != `MAX_32_BIT) // 仅在WAIT状态命中时递增
            hit_seq_cnt <= hit_seq_cnt + 1;
    end


    /* 写操作计数器 */
    always @(posedge clk) begin
        if (rst || state_WAIT) // 在等待状态或复位时清零
            write_cnt <= 4'h0;
        else if ((state_SYNC || state_WRW) && from_mem_wr_data_ready) // 在写回或旁路写时，每次成功发送数据后递增
            write_cnt <= write_cnt + 1;
    end

    // 判断写操作是否为最后一个节拍
    assign wr_last = (state_SYNC && (write_cnt == 4'h8)) || // 写回8个节拍
                     (state_WRW  && (write_cnt == 4'h1));    // 旁路写1个节拍


    /* 用于写回的临时数据块寄存器 */
    always @(posedge clk) begin
        if (rst)
            block_tmp <= `LINE_LEN'b0;
        else if (state_MISS_DT) // 在脏未命中时，锁存被替换的脏数据块
            block_tmp <= block_all_way[replaced_way];
        else if (state_SYNC && from_mem_wr_data_ready) // 在写回时，每发送一个字就移位
            block_tmp <= {`DATA_WIDTH'b0, block_tmp[`LINE_LEN - 1 : `DATA_WIDTH]};
    end

    // 从临时寄存器的低位取出要发送到内存的数据
    assign from_cache_wdata = block_tmp[`DATA_WIDTH - 1 : 0];


    /* 将CPU地址分解为不同字段 */
    assign tag    =
        from_cpu_mem_req_addr[`DATA_WIDTH - 1 : `OFFSET_WIDTH + `INDEX_WIDTH];
    assign index  =
        from_cpu_mem_req_addr[`OFFSET_WIDTH + `INDEX_WIDTH - 1 : `OFFSET_WIDTH];
    assign offset =
        from_cpu_mem_req_addr[`OFFSET_WIDTH - 1 : 0];


    generate
        for (i = 0; i < `CACHE_WAY; i = i + 1) begin
            // 判断每一路是否命中：有效位为1且标签匹配
            assign hit_all_way[i] = valid_all_way[i] && (tag == tag_all_way[i]);

            // 填充时的写使能
            assign wen_rf[i] = (replaced_way == i) && state_REFILL && from_mem_rd_rsp_valid;
            // 写命中时的写使能
            assign wen_hit[i] = (hit_way == i) && state_WR_HIT;
            // 总的写使能
            assign wen[i] = wen_rf[i] || wen_hit[i];

        end
    endgenerate

    // 数据写入Cache的多路选择器
    assign to_cache_data =  (
        {`DATA_WIDTH{state_REFILL}} & from_mem_rd_rsp_data   | // 填充时，数据来自内存
        {`DATA_WIDTH{state_WR_HIT}} & from_cpu_mem_req_wdata   // 写命中时，数据来自CPU
    );

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
    // 总的未命中信号
    assign miss = !hit;

    // 实例化替换策略模块
    replacement replacement_inst (
        .clk(clk), .rst(rst),
        .data_0(last_hit_time_all_way[0]), .data_1(last_hit_time_all_way[1]),
        .data_2(last_hit_time_all_way[2]), .data_3(last_hit_time_all_way[3]),
    //  .data_4(last_hit_time_all_way[4]), .data_5(last_hit_time_all_way[5]),
        .replaced_way(replaced_way) // 输出要被替换的路
    );

    // 获取被替换行的标签
    assign replaced_tag = tag_all_way[replaced_way];

    // 判断被替换行是否为脏
    assign dirty = dirty_all_way[replaced_way];
    assign clean = !dirty;

    // 判断请求是否需要旁路 (Bypass)
    assign bypass = (
        |(from_cpu_mem_req_addr & `IO_SPACE_MASK)    || // 地址在IO空间
        !(|(from_cpu_mem_req_addr & `NO_CACHE_MASK))    // 地址在不可缓存区
    );

    // 判断读操作是否为最后一个节拍
    assign rd_last = from_mem_rd_rsp_valid && from_mem_rd_rsp_last;


    /*
    ** {=====================================================================
    ** 有限状态机 (三段式)
    ** ======================================================================
    */

    // 将one-hot编码的当前状态寄存器拆分为单独的信号线，方便使用
    assign state_INIT    = curr_state[ 0];
    assign state_WAIT    = curr_state[ 1];
    assign state_MISS_DT = curr_state[ 2];
    assign state_SYNC    = curr_state[ 3];
    assign state_MISS_CL = curr_state[ 4];
    assign state_REFILL  = curr_state[ 5];
    assign state_WR_HIT  = curr_state[ 6];
    assign state_RD_HIT  = curr_state[ 7];
    assign state_RD_CHE  = curr_state[ 8];
    assign state_BP_WR   = curr_state[ 9];
    assign state_BP_RD   = curr_state[10];
    assign state_WRW     = curr_state[11];
    assign state_RDW     = curr_state[12];


    /* 第一段: 状态寄存器 */
    always @(posedge clk) begin
        if (rst)
            curr_state <= INIT;
        else
            curr_state <= next_state;
    end

    /* 第二段: 次态逻辑 (组合逻辑) */
    assign next_state = (
        {13{state_INIT}} & WAIT                                                | // 初始化后进入等待
        {13{state_WAIT}} & (
            {13{from_cpu_mem_req_valid}} & ( // 收到CPU请求
                {13{hit}} & (from_cpu_mem_req ? WR_HIT : RD_HIT)    | // 命中 -> 根据读写进入相应状态
                {13{miss && !bypass}} & (dirty ? MISS_DT : MISS_CL) | // 未命中(非旁路) -> 根据脏位决定先写回还是直接填充
                {13{bypass}} & (from_cpu_mem_req ? BP_WR : BP_RD)      // 旁路 -> 根据读写进入旁路状态
            ) |
            {13{!from_cpu_mem_req_valid}} & WAIT // 未收到CPU请求 -> 保持等待
        )                                                                      |
        {13{state_MISS_DT}} & (from_mem_wr_req_ready ? SYNC : MISS_DT)         | // 脏未命中 -> 内存准备好后开始同步(写回)
        {13{state_SYNC}}    & (wr_last ? MISS_CL : SYNC)                          | // 写回 -> 写完后进入干净未命中状态，否则继续写回
        {13{state_MISS_CL}} & (from_mem_rd_req_ready ? REFILL : MISS_CL)       | // 干净未命中 -> 内存准备好后开始填充
        {13{state_REFILL}} & ( // 填充中
            {13{rd_last && from_cpu_mem_req}} & WR_HIT  | // 填充完毕且原请求是写 -> 转为写命中
            {13{rd_last && !from_cpu_mem_req}} & RD_HIT | // 填充完毕且原请求是读 -> 转为读命中
            {13{!rd_last}} & REFILL                        // 未填充完 -> 继续填充
        )                                                                      |
        {13{state_WR_HIT}} & WAIT                                              | // 写命中完成 -> 返回等待
        {13{state_RD_HIT}} & RD_CHE                                            | // 读命中 -> 进入送出数据状态
        {13{state_RD_CHE}} & WAIT                                              | // 数据送出完成 -> 返回等待
        {13{state_BP_WR}}  & (from_mem_wr_req_ready ? WRW : BP_WR)              | // 旁路写 -> 内存准备好后进入等待数据传输
        {13{state_BP_RD}}  & (from_mem_rd_req_ready ? RDW : BP_RD)              | // 旁路读 -> 内存准备好后进入等待数据传输
        {13{state_WRW}}    & (wr_last ? WAIT : WRW)                               | // 旁路写等待 -> 写完后返回等待
        {13{state_RDW}}    & (rd_last ? WAIT : RDW)                               // 旁路读等待 -> 读完后返回等待
    );


    /* 第三段: 输出逻辑 - CPU接口 */
    // Cache何时能接收CPU的新请求
    assign to_cpu_mem_req_ready   = state_WR_HIT         || // 写命中时，一拍完成，立即可接受
                                    state_RD_HIT         || // 读命中时，立即可接受
                                    (state_WRW && wr_last) || // 旁路写完成时
                                    state_BP_RD;            // 发起旁路读请求时

    // Cache何时向CPU返回有效数据
    assign to_cpu_cache_rsp_valid = state_RD_CHE || (state_RDW && rd_last);

    // Cache向CPU返回的数据是什么
    assign to_cpu_cache_rsp_data = (
        {`DATA_WIDTH{state_RD_CHE}} & block_all_way[hit_way][{offset, 3'b0} +: `DATA_WIDTH] | 
        // 命中时，从数据阵列中根据偏移取数（从起始比特位取32位）
		/* assign Address = {ALU_result[31:2],2'b00}; 得益于CPU访存的地址一定是4字节对齐，
        所以offset只可能为0，4，8.....，所以offset*8就是块内某字节对应的比特起始位*/
        {`DATA_WIDTH{state_RDW}} & from_mem_rd_rsp_data  // 旁路读时，直接转发内存来的数据
    );

    /*
    ** {=====================================================================
    ** 第三段: 输出逻辑 - 内存读接口
    ** ======================================================================
    */

    // 何时向内存发起读请求
    assign to_mem_rd_req_valid = state_MISS_CL || state_BP_RD;
    // 何时准备好接收内存返回的读数据
    assign to_mem_rd_rsp_ready = state_INIT || state_REFILL || state_RDW;

    // 向内存读请求的地址
    assign to_mem_rd_req_addr = {
        tag,
        index,
        {5{state_BP_RD}} & offset /* 旁路读时使用完整地址，填充时offset为0 */
    };

    // 向内存读请求的长度
    assign to_mem_rd_req_len = {8{state_MISS_CL}} & 8'h7; /* 仅在填充时为8拍，其他为1拍(默认值0) */

    /* }===================================================================== */



    /*
    ** {=====================================================================
    ** 第三段: 输出逻辑 - 内存写接口
    ** ======================================================================
    */

    // 何时向内存发起写请求
    assign to_mem_wr_req_valid = state_MISS_DT || state_BP_WR;

    // 何时向内存发送有效的写数据
    assign to_mem_wr_data_valid = state_SYNC || state_WRW;

    // 向内存写请求的地址
    assign to_mem_wr_req_addr = {
        (state_MISS_DT ? replaced_tag : tag), // 写回时用被替换行的tag，旁路写用当前请求的tag
        index,
        {5{state_BP_WR}} & offset /* 旁路写时使用完整地址，写回时offset为0 */
    };

    // 向内存写请求的长度
    assign to_mem_wr_req_len = {8{state_MISS_DT}} & 8'h7; /* 仅在写回时为8拍，其他为1拍(默认值0) */

    // 向内存发送的写数据
    assign to_mem_wr_data = (
        {`DATA_WIDTH{state_SYNC}} & from_cache_wdata      | // 写回时，数据来自临时寄存器
        {`DATA_WIDTH{state_WRW}} & from_cpu_mem_req_wdata   // 旁路写时，数据来自CPU
    );

    // 向内存发送的写选通
    assign to_mem_wr_data_strb = (
        {4{state_SYNC}} & 4'hf | // 写回时，总是写全部字节
        {4{state_WRW}} & from_cpu_mem_req_wstrb // 旁路写时，使用CPU的原始选通
    );

    // 判断是否为写操作的最后一个节拍
    assign to_mem_wr_data_last = state_WRW || (state_SYNC && (write_cnt == 4'h7)); // 注意：计数器从0开始，所以第8拍是cnt=7

endmodule



//缓存辅助模块设计

//------------------------------------------------------------------------------
//LAST_HIT_ARRAY MOD 用于存储每个字块的时间戳
//------------------------------------------------------------------------------
//每一个模块对应一路，一路中又有8组，这些组由waddr来区分
module last_hit_array (
    input                            clk,
    input                            rst,
    input                            wen,
    input  [`HIT_ADDR_WIDTH - 1 : 0] waddr,
    input  [`HIT_DATA_WIDTH - 1 : 0] wdata,
    input  [`HIT_ADDR_WIDTH - 1 : 0] raddr,
    output [`HIT_DATA_WIDTH - 1 : 0] rdata
);

    // 存储阵列，用于存放LRU时间戳
    reg [`HIT_DATA_WIDTH - 1:0] array [(1 << `HIT_ADDR_WIDTH) - 1 : 0];

    integer i;
    // 同步写和复位逻辑
    always @(posedge clk) begin
        if (rst) // 复位时，将所有时间戳清零
            for (i = 0; i < (1 << `HIT_ADDR_WIDTH); i = i + 1)
                array[i] <= `HIT_DATA_WIDTH'b0;
        else if (wen) // 写使能时，更新指定地址的时间戳
            array[waddr] <= wdata;
    end

    // 异步读
    assign rdata = array[raddr];

endmodule

//------------------------------------------------------------------
//ONR_BIT_ARRAY MOD
//------------------------------------------------------------------
module one_bit_array (
    input                                clk,
    input                                rst,
    input  [`ONE_BIT_ADDR_WIDTH - 1 : 0] waddr, // 写地址
    input  [`ONE_BIT_ADDR_WIDTH - 1 : 0] raddr, // 读地址
    input                                wen,   // 写使能
    input                                wdata, // 写入数据 (1位)
    output                               rdata  // 读出数据 (1位)
);

    reg array [(1 << `ONE_BIT_ADDR_WIDTH) - 1 : 0]; // 存储阵列

    integer i; // for循环变量
    always @(posedge clk) begin
        if (rst) begin // 复位逻辑
            for (i = 0; i < (1 << `ONE_BIT_ADDR_WIDTH); i = i + 1)
                array[i] <= 1'h0; // 将所有位清零
        end
        else if (wen) begin // 写入逻辑
            array[waddr] <= wdata;
        end
    end

    assign rdata = array[raddr]; // 异步读出

endmodule

//---------------------------------------------------------------------------
//REPLACEMENT MOD
//---------------------------------------------------------------------------
module replacement (
    input                        clk,
    input                        rst,
    input  [`TIME_WIDTH - 1 : 0] data_0, data_1, data_2, data_3,// 各路的时间戳输入
                                // data_4, data_5,
    output [              2 : 0] replaced_way           // 输出被替换的路号
);

    wire         full;       // 标志所有时间戳是否都已饱和
    reg  [2 : 0] random_num; // 用于饱和状态下的随机替换

    // 当所有时间戳都达到最大值时，full为1
    assign full = (data_0 == `MAX_32_BIT) && (data_1 == `MAX_32_BIT) &&
                  (data_2 == `MAX_32_BIT) && (data_3 == `MAX_32_BIT);// &&
                //(data_4 == `MAX_32_BIT) && (data_5 == `MAX_32_BIT);

    // 一个简单的循环计数器，生成0-5的伪随机数
    always @(posedge clk) begin
        if (rst)
            random_num <= 3'b0;
        else if (random_num == 3'h5)
            random_num <= 3'h0;
        else
            random_num <= random_num + 1;
    end

    // 两两比较所有时间戳
    wire le_01, le_02, le_03, le_04, le_05,
         le_12, le_13, le_14, le_15, le_23,
         le_24, le_25, le_34, le_35, le_45;

    // 标志哪一路的时间戳是最小的
    reg  least_0, least_1, least_2,
         least_3, least_4, least_5;

    assign le_01 = (data_0 < data_1);
    assign le_02 = (data_0 < data_2);
    assign le_03 = (data_0 < data_3);
    assign le_12 = (data_1 < data_2);
    assign le_13 = (data_1 < data_3);
    assign le_23 = (data_2 < data_3);
//    assign le_14 = (data_1 < data_4);
//    assign le_15 = (data_1 < data_5);	
//    assign le_04 = (data_0 < data_4);
//    assign le_05 = (data_0 < data_5);	
//    assign le_24 = (data_2 < data_4);
//    assign le_25 = (data_2 < data_5);
//    assign le_34 = (data_3 < data_4);
//    assign le_35 = (data_3 < data_5);
//    assign le_45 = (data_4 < data_5);

    // 找出唯一最小的时间戳对应的路
    always @(posedge clk) begin
        least_0 <=  le_01 &&  le_02 &&  le_03 ;//&& le_04 &&  le_05;
        least_1 <= !le_01 &&  le_12 &&  le_13 ;//&& le_14 &&  le_15;
        least_2 <= !le_02 && !le_12 &&  le_23 ;//&& le_24 &&  le_25;
        least_3 <= !le_03 && !le_13 && !le_23 ;//&& le_34 &&  le_35;
//      least_4 <= !le_04 && !le_14 && !le_24 && !le_34 &&  le_45;
//      least_5 <= !le_05 && !le_15 && !le_25 && !le_35 && !le_45;
    end

    // 输出最终要被替换的路号
    assign replaced_way = {
        ({3{ full           }} & random_num) | // 如果时间戳饱和，随机选择一路
        ({3{!full && least_0}} &       3'h0) | // 否则，选择时间戳最小的那一路
        ({3{!full && least_1}} &       3'h1) |
        ({3{!full && least_2}} &       3'h2) |
        ({3{!full && least_3}} &       3'h3) //|
//      ({3{!full && least_4}} &       3'h4) |
//      ({3{!full && least_5}} &       3'h5)
    };

endmodule