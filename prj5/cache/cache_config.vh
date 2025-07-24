// 防止重复包含 
`ifndef __DCACHE_CONFIG_VH__
`define __DCACHE_CONFIG_VH__

//所有Cache相关的共享宏
`define         CACHE_SET  8     // 缓存组数
`define         CACHE_WAY  4     // 缓存的路数
`define           TAG_LEN  24    // 标签的位宽
`define          LINE_LEN  256   // 缓存行的位宽 (32字节)
`define        DATA_WIDTH  32    // 数据总线的位宽 (4字节)
`define        TIME_WIDTH  32    // 用于LRU的时间戳位宽

`define       INDEX_WIDTH  3     /* 8个组 */
`define      OFFSET_WIDTH  5     /* 32字节 (缓存块大小) 的偏移量 */

`define        MAX_32_BIT  32'hffff_ffff // 32位无符号数的最大值

`define     NO_CACHE_MASK  32'hffff_ffe0  /* 不可缓存区域掩码 (例如 0x00 ~ 0x1f) */
`define     IO_SPACE_MASK  32'hc000_0000  /* IO空间掩码 (例如 0x4000_0000 及以上) */

`define     HIT_DATA_WIDTH 32    /* 命中时间戳数据宽度 */
`define     HIT_ADDR_WIDTH 3     /* 命中时间戳数组的地址宽度 (8个组) */

`define     ONE_BIT_ADDR_WIDTH 3     /* 单比特数组的地址宽度 (8个组) */

//data_array使用的宏
`define DARRAY_DATA_WIDTH  256   /* 32 字节的缓存字块大小 */
`define DARRAY_ADDR_WIDTH  3     /* 8个组 */

`endif 