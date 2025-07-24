`timescale 10 ns / 1 ns

`define DATA_WIDTH 32

module shifter (
    input  [`DATA_WIDTH - 1:0] A,      // 操作数
    input  [4:0] B,                     // 要移动的位数
    input  [1:0] Shiftop,               // 移位操作类型：00 左移，11 算术右移，10 逻辑右移
    output [`DATA_WIDTH - 1:0] Result   // 移位结果
);
    wire [`DATA_WIDTH - 1:0] sign = {`DATA_WIDTH{A[`DATA_WIDTH - 1]}};
    wire [`DATA_WIDTH - 1:0] lr;
    assign lr = A >> B;
    wire [`DATA_WIDTH - 1:0] ar;
    wire [`DATA_WIDTH - 1:0] ar_sign;
    assign ar_sign = sign << (`DATA_WIDTH - B);
    assign ar = ar_sign | lr ;
    // 使用 assign 和条件运算符 ?: 实现不同类型的移位
    assign Result = (Shiftop == 2'b00) ? (A << B) :                // 左移
                    (Shiftop == 2'b11) ? ar       :               // 算术右移
                    (Shiftop == 2'b10) ? lr       :               // 逻辑右移
                                    `DATA_WIDTH'b0;              // 默认情况

endmodule
