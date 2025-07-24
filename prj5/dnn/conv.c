#include "printf.h" 
#include "trap.h"   
#include "mul.h"    
#include "div.h"    
#include "perf_cnt.h" // 包含用于性能计数的函数和结构体

#define SHORT_MIN 0x8000 // 定义short类型的最小值，用于在池化层中寻找最大值的初始值

#define FRAC_BIT 10 // 定义定点数的小数位数。右移该位数相当于除以 2^10，用于将乘法累加后的结果转换回定点格式。

// === 输入数据（读地址）的内存布局和尺寸定义 ===
#define RD_ADDR 135106448      // 输入特征图的起始内存地址
#define RD_SIZE_D0 1           // 输入特征图的批量大小 (Batch size)
#define RD_SIZE_D1 1           // 输入特征图的通道数 (Channels)
#define RD_SIZE_D2 28          // 输入特征图的高度 (Height)
#define RD_SIZE_D3 28          // 输入特征图的宽度 (Width)

// === 权重数据（卷积核）的内存布局和尺寸定义 ===
#define WEIGHT_ADDR 134217728  // 权重的起始内存地址
#define WEIGHT_SIZE_D0 20      // 权重（卷积核）的数量，也即输出特征图的通道数
#define WEIGHT_SIZE_D1 1       // 每个卷积核的输入通道数
#define WEIGHT_SIZE_D2 5       // 每个卷积核的高度
#define WEIGHT_SIZE_D3 5       // 每个卷积核的宽度

// === 输出数据（写地址）的内存布局和尺寸定义 ===
#define WR_ADDR 135108240      // 输出特征图的起始内存地址
#define WR_SIZE_D0 1           // 输出特征图的批量大小
#define WR_SIZE_D1 20          // 输出特征图的通道数
#define WR_SIZE_D2 12          // 输出特征图的高度
#define WR_SIZE_D3 12          // 输出特征图的宽度

// === 卷积和池化层的属性定义 ===
#define KERN_ATTR_CONV_PAD 0     // 卷积层的填充 (Padding)
#define KERN_ATTR_CONV_STRIDE 1  // 卷积层的步长 (Stride)
#define KERN_ATTR_POOL_PAD 0     // 池化层的填充 (Padding)
#define KERN_ATTR_POOL_KERN_SIZE 2 // 池化核的大小 (Kernel Size)
#define KERN_ATTR_POOL_STRIDE 2  // 池化层的步长 (Stride)

// === DNN硬件加速器的内存映射I/O (MMIO) 寄存器地址 ===
#define GPIO_START_ADDR    0x60030000 // 启动硬件加速器的控制寄存器地址
#define GPIO_DONE_ADDR     0x60030008 // 查询硬件加速器完成状态的状态寄存器地址

// 结构体：用于存储4维张量（数据）的尺寸
struct size_vec4
{
    unsigned d0; // 维度0 (e.g., 批量)
    unsigned d1; // 维度1 (e.g., 通道)
    unsigned d2; // 维度2 (e.g., 高度)
    unsigned d3; // 维度3 (e.g., 宽度)
};

// 结构体：用于存储不同数据的内存地址
struct mem_addr
{
    unsigned rd_addr;      // 读地址 (输入数据)
    unsigned weight_addr;  // 权重地址
    unsigned wr_addr;      // 写地址 (输出数据)
};

// 乘法函数封装
int mul(short a, short b)
{
// 根据 USE_MUL 宏决定使用自定义乘法函数还是原生乘法操作
#ifndef USE_MUL
    int ans = mul_ll(a, b); // 使用 "mul.h" 中定义的自定义乘法
#else
    int ans = a * b; // 使用C语言内建的乘法操作
#endif
    return ans;
}

// === 全局变量实例化 ===
// 使用宏定义的值初始化内存地址和数据尺寸的全局结构体
struct mem_addr addr = {RD_ADDR, WEIGHT_ADDR, WR_ADDR};
struct size_vec4 rd_size = {RD_SIZE_D0, RD_SIZE_D1, RD_SIZE_D2, RD_SIZE_D3};
struct size_vec4 wr_size = {WR_SIZE_D0, WR_SIZE_D1, WR_SIZE_D2, WR_SIZE_D3};
struct size_vec4 weight_size = {WEIGHT_SIZE_D0, WEIGHT_SIZE_D1, WEIGHT_SIZE_D2, WEIGHT_SIZE_D3};

// 用于存储卷积操作输出尺寸的全局变量
struct size_vec4 conv_size;

// 链接器脚本定义的符号，指向嵌入的二进制文件（黄金参考结果）的起始地址和大小
extern char _binary_data_result_bin_start[];
extern char _binary_data_result_bin_size[];

// 软件实现卷积层
void convolution()
{
    // 将内存地址转换为对应类型的指针
    short *in = (short *)addr.rd_addr;
    short *weight = (short *)addr.weight_addr;
    short *out = (short *)addr.wr_addr;

    // 定义输入和输出的偏移量，此处未使用，但为常见实践
    unsigned output_offset = 0;
    unsigned input_offset = 0;

    // 获取输入特征图的宽度和高度
    unsigned input_fm_w = rd_size.d3;
    unsigned input_fm_h = rd_size.d2;

    // 获取卷积属性
    unsigned pad = KERN_ATTR_CONV_PAD;
    unsigned pad_len = pad << 1; // 填充总长度 (pad * 2)

    // 计算卷积输出特征图的理论宽度和高度
    // 公式: output_size = (input_size - kernel_size + 2 * padding) / stride + 1
    unsigned conv_out_w = rd_size.d3 - weight_size.d3 + pad_len;
    unsigned conv_out_h = rd_size.d2 - weight_size.d2 + pad_len;
    unsigned stride = KERN_ATTR_CONV_STRIDE;

    // 使用自定义除法函数进行计算
    conv_out_w = div(conv_out_w, stride);
    conv_out_h = div(conv_out_h, stride);

    // 完成公式的 "+1" 部分
    conv_out_w++;
    conv_out_h++;

    // 将计算出的卷积输出尺寸保存到全局变量 conv_size 中
    conv_size.d0 = wr_size.d0;
    conv_size.d1 = wr_size.d1;
    conv_size.d2 = conv_out_h;
    conv_size.d3 = conv_out_w;

    // === 卷积计算核心逻辑 ===
    unsigned no, ni; // 循环变量：no - 输出通道索引, ni - 输入通道索引
    unsigned y, x;   // 循环变量：y, x - 输出特征图的坐标
    unsigned ky, kx; // 循环变量：ky, kx - 卷积核内的坐标

    // 遍历每一个输出通道 (或每一个卷积核)
    for (no = 0; no < weight_size.d0; no++) {
        // 遍历每一个输入通道
        for (ni = 0; ni < rd_size.d1; ni++) {
            // 遍历输出特征图的每一个像素 (y, x)
            for (y = 0; y < conv_size.d2; y++) {
                for (x = 0; x < conv_size.d3; x++) {
                    int unfixed_res = 0; // 用于累加乘积的高精度中间结果
                    
                    // 在处理第一个输入通道时，加载偏置项(bias)
                    // 假设权重布局为 [bias, w0, w1, ...], 每个卷积核权重部分大小为 (d2*d3), 偏置占1个short
                    // 因此整个核（带偏置）的大小为 (d2*d3 + 1)
                    if (ni == 0) {
                        *(out + output_offset + no*conv_size.d2*conv_size.d3 + y*conv_size.d3 + x) = 
                            *(weight + no*weight_size.d1*(weight_size.d2*weight_size.d3+1)); /* 加载偏置 */
                    }

                    // 遍历卷积核
                    for (ky = 0; ky < weight_size.d2; ky++) {
                        // 计算当前卷积核位置对应在输入特征图上的行号 (ih)
                        int ih = ky + mul(y, stride) - pad;
                        int offset = mul(ky, weight_size.d3); // 预计算卷积核内的行偏移
                        for (kx = 0; kx < weight_size.d3; kx++) {
                            // 计算当前卷积核位置对应在输入特征图上的列号 (iw)
                            int iw = kx + mul(x, stride) - pad;
                            
                            // 边界检查：确保计算区域在输入特征图的有效范围内
                            if (iw >= 0 && iw < input_fm_w &&
                                ih >= 0 && ih < input_fm_h)
                            {
                                // 乘加累算 (MAC)
                                // 权重地址计算：跳过偏置项，所以 +1
                                unfixed_res += mul(
                                    *(in + input_offset + ni*rd_size.d2*rd_size.d3 + ih*rd_size.d3 + iw),
                                    *(weight + no*weight_size.d1*(weight_size.d2*weight_size.d3+1) + ni*(weight_size.d2*weight_size.d3+1) + offset + kx + 1)
                                );
                            }
                        }
                    }
                    // 将累加结果进行定点数转换（右移FRAC_BIT位），并加到输出像素上
                    *(out + output_offset + no*conv_size.d2*conv_size.d3 + y*conv_size.d3 + x) += unfixed_res >> FRAC_BIT; /* 定点数转换 */
                }
            }
        }
    }
}

// 软件实现最大池化层
void pooling()
{
    // 输入和输出指针指向同一块内存区域，实现原地(in-place)池化操作
    short *in  = (short *)addr.wr_addr;
    short *out = (short *)addr.wr_addr;

    unsigned output_offset = 0;
    unsigned input_offset = 0;

    // 池化层的输入尺寸即为卷积层的输出尺寸
    unsigned input_fm_w = conv_size.d3;
    unsigned input_fm_h = conv_size.d2;

    // 获取池化属性
    unsigned pad = KERN_ATTR_POOL_PAD;
    unsigned pad_len = pad << 1;

    // 计算池化输出的理论尺寸
    // 公式: output_size = (input_size - kernel_size + 2 * padding) / stride + 1
    unsigned pad_w_test = conv_size.d3 - KERN_ATTR_POOL_KERN_SIZE;
    unsigned pad_h_test = conv_size.d2 - KERN_ATTR_POOL_KERN_SIZE;

    unsigned pool_out_w = pad_w_test + pad_len;
    unsigned pool_out_h = pad_h_test + pad_len;

    unsigned stride = KERN_ATTR_POOL_STRIDE;

    // 检查在无填充的情况下，输入尺寸是否能被步长整除。这部分逻辑用于处理非标准情况。
    unsigned pad_w_test_remain = pad_w_test - mul(div(pad_w_test, stride), stride);
    unsigned pad_h_test_remain = pad_h_test - mul(div(pad_h_test, stride), stride);

    // 计算输出尺寸
    pool_out_w = div(pool_out_w, stride);
    pool_out_h = div(pool_out_h, stride);
    pool_out_w++;
    pool_out_h++;

    // 特殊情况处理：当无填充且输入尺寸不能被步长完美覆盖时，输出尺寸需要+1
    if ((!pad) && (pad_w_test_remain || pad_h_test_remain))
    {
        pool_out_w++;
        pool_out_h++;
    }

    // === 池化计算核心逻辑 ===
    unsigned no;      // 循环变量：通道索引
    unsigned y, x;    // 循环变量：输出特征图坐标
    unsigned ky, kx;  // 循环变量：池化核内坐标
    unsigned iw;      // 临时变量：输入特征图列坐标

    // 遍历每一个通道
    for (no = 0; no < conv_size.d1; no++) {
        // 遍历池化输出的每一个位置 (y, x)
        for (y = 0; y < pool_out_h; y++) {
            for (x = 0; x < pool_out_w; x++) {
                // 初始化最大值为 short 类型的最小值
                short max = SHORT_MIN;
                // 遍历池化核
                for(ky = 0; ky < KERN_ATTR_POOL_KERN_SIZE; ky++) {
                    // 计算当前池化核位置对应在输入特征图上的行号 (ih)
                    int ih = ky + mul(y, stride) - pad;
                    for (kx = 0; kx < KERN_ATTR_POOL_KERN_SIZE; kx++) {
                        // 计算当前池化核位置对应在输入特征图上的列号 (iw)
                        iw = kx + mul(x, stride) - pad;
                        
                        // 边界检查：确保计算区域在输入特征图的有效范围内
                        if (iw >= 0 && iw < input_fm_w &&
                            ih >= 0 && ih < input_fm_h)
                        {
                            // 获取池化窗口内的像素值
                            short tmp = *(in + input_offset + no*conv_size.d2*conv_size.d3 + ih*conv_size.d3 + iw);
                            // 如果当前值大于记录的最大值，则更新最大值
                            if (max < tmp) {
                                max = tmp;
                            }
                        }
                    }
                }
                // 将池化窗口内的最大值写入到输出位置
                // 注意输出是紧凑存储的，地址计算使用 pool_out_h 和 pool_out_w
                *(out + output_offset + no*pool_out_h*pool_out_w + y*pool_out_w + x) = max;
            }
        }
    }
}

// 如果定义了 USE_HW_ACCEL 宏，则编译此函数
#ifdef USE_HW_ACCEL
// 启动硬件加速器并等待其完成
void launch_hw_accel()
{
    // 定义指向MMIO寄存器的 volatile 指针
    // volatile 关键字防止编译器优化对这些地址的读写操作
    volatile int* gpio_start = (void*)(GPIO_START_ADDR);
    volatile int* gpio_done = (void*)(GPIO_DONE_ADDR);

    // 启动硬件加速器
    // 通过位操作将启动寄存器的最低位置为1，同时保持其他位不变
    *gpio_start = (*gpio_start & 0xfffffffe) | 0x1; 
    
    // 轮询等待硬件完成
    // 不断读取完成状态寄存器，直到其最低位变为1
    while ((*gpio_done & 0x1) == 0)
        ; 
        
    // 停止硬件加速器（或清除启动信号）
    // 将启动寄存器的最低位清零
    *gpio_start = *gpio_start & 0xfffffffe; 
}
#endif

// 比较计算结果与黄金参考结果
int comparing()
{
    char *out = (char *)addr.wr_addr;          // 指向计算结果的指针
    char *result = (char *)_binary_data_result_bin_start; // 指向黄金参考结果的指针

    int count; // 需要比较的总字节数

#ifdef USE_HW_ACCEL
    // 如果使用硬件加速器，其输出可能有额外的填充(padding)以满足对齐要求
    // 此处计算的 count 包括了这些填充字节
    count = (int)_binary_data_result_bin_size +
            (16 - WR_SIZE_D3) * 2 * WR_SIZE_D2 * WR_SIZE_D1;
#else
    // 如果使用软件计算，则比较大小就是黄金参考结果的大小
    count = (int)_binary_data_result_bin_size;
#endif

    // 逐字节比较
    // i 遍历计算结果的内存区域，j 遍历黄金参考结果
    for (int i = 0, j = 0; i < count; i++)
    {
#ifdef USE_HW_ACCEL
        // 如果使用硬件，需要跳过每行末尾的填充字节
        int alignment = i & 0x0000001f; // 计算当前字节在32字节对齐块中的位置
        // WR_SIZE_D3 是有效数据宽度，乘以2是因为每个数据是short（2字节）
        // 如果当前位置超出了有效数据的范围，则跳过此字节
        if (alignment >= (WR_SIZE_D3 << 1))
            continue;
#endif
        // 比较计算结果和黄金参考结果
        if (*(out + i) != *(result + j))
        {
            // 如果不匹配，打印错误信息并返回失败
            printf("Failed! at address %x and %x with data %x and %x\n", out + i, result + j, *(out + i), *(result + j));
            return 1; // 返回1表示失败
        }
        j++; // 只有在有效数据被比较后，黄金参考的索引j才增加
    }

    printf("Passed!\n");
    return 0; // 返回0表示成功
}

// 主函数，程序入口
int main()
{
    // 初始化性能计数器结构体
    Result res;
    res.msec         = 0;
    res.ins          = 0;

    // 准备性能计数（例如，清零计数器）
    bench_prepare(&res);

// 根据 USE_HW_ACCEL 宏决定执行路径
#ifdef USE_HW_ACCEL
    printf("Launching task...\n");
    launch_hw_accel(); // 调用硬件加速器
#else
    printf("starting convolution\n");
    convolution(); // 调用软件卷积
    printf("starting pooling\n");
    pooling();     // 调用软件池化
#endif

    // 结束性能计数（例如，读取计数器值）
    bench_done(&res);
    printf("===================================================\n");
    printf("PERFORMANCE COUNTERS                               \n");
    printf("===================================================\n");

    // 打印性能结果
    printf("TOTAL CYCLES            : %u\n", res.msec        ); // 总时钟周期
    printf("TOTAL INSTRUCTIONS      : %u\n", res.ins         ); // 总执行指令数

    printf("===================================================\n");

    // 调用比较函数验证结果
    int result = comparing();
    printf("benchmark finished\n");

    if (result == 0) {
        // 如果结果正确，调用 hit_good_trap() 通知模拟器成功退出
        hit_good_trap();
    } else {
        // 如果结果错误，调用 nemu_assert(0) 触发断言，使模拟器停止
        nemu_assert(0);
    }

    return 0;
}