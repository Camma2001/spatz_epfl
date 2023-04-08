// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// Register Package auto-generated by `reggen` containing data structure

package spatz_cluster_peripheral_reg_pkg;

  // Param list
  parameter int NumPerfCounters = 16;

  // Address widths within the block
  parameter int BlockAw = 9;

  ////////////////////////////
  // Typedefs for registers //
  ////////////////////////////

  typedef struct packed {
    struct packed {
      logic        q;
    } cycle;
    struct packed {
      logic        q;
    } tcdm_accessed;
    struct packed {
      logic        q;
    } tcdm_congested;
    struct packed {
      logic        q;
    } issue_fpu;
    struct packed {
      logic        q;
    } issue_fpu_seq;
    struct packed {
      logic        q;
    } issue_core_to_fpu;
    struct packed {
      logic        q;
    } retired_instr;
    struct packed {
      logic        q;
    } retired_load;
    struct packed {
      logic        q;
    } retired_i;
    struct packed {
      logic        q;
    } retired_acc;
    struct packed {
      logic        q;
    } dma_aw_stall;
    struct packed {
      logic        q;
    } dma_ar_stall;
    struct packed {
      logic        q;
    } dma_r_stall;
    struct packed {
      logic        q;
    } dma_w_stall;
    struct packed {
      logic        q;
    } dma_buf_w_stall;
    struct packed {
      logic        q;
    } dma_buf_r_stall;
    struct packed {
      logic        q;
    } dma_aw_done;
    struct packed {
      logic        q;
    } dma_aw_bw;
    struct packed {
      logic        q;
    } dma_ar_done;
    struct packed {
      logic        q;
    } dma_ar_bw;
    struct packed {
      logic        q;
    } dma_r_done;
    struct packed {
      logic        q;
    } dma_r_bw;
    struct packed {
      logic        q;
    } dma_w_done;
    struct packed {
      logic        q;
    } dma_w_bw;
    struct packed {
      logic        q;
    } dma_b_done;
    struct packed {
      logic        q;
    } dma_busy;
    struct packed {
      logic        q;
    } icache_miss;
    struct packed {
      logic        q;
    } icache_hit;
    struct packed {
      logic        q;
    } icache_prefetch;
    struct packed {
      logic        q;
    } icache_double_hit;
    struct packed {
      logic        q;
    } icache_stall;
  } spatz_cluster_peripheral_reg2hw_perf_counter_enable_mreg_t;

  typedef struct packed {
    logic [9:0] q;
  } spatz_cluster_peripheral_reg2hw_hart_select_mreg_t;

  typedef struct packed {
    logic [47:0] q;
    logic        qe;
  } spatz_cluster_peripheral_reg2hw_perf_counter_mreg_t;

  typedef struct packed {
    logic [31:0] q;
    logic        qe;
  } spatz_cluster_peripheral_reg2hw_cl_clint_set_reg_t;

  typedef struct packed {
    logic [31:0] q;
    logic        qe;
  } spatz_cluster_peripheral_reg2hw_cl_clint_clear_reg_t;

  typedef struct packed {
    logic [31:0] q;
  } spatz_cluster_peripheral_reg2hw_hw_barrier_reg_t;

  typedef struct packed {
    logic        q;
  } spatz_cluster_peripheral_reg2hw_icache_prefetch_enable_reg_t;

  typedef struct packed {
    logic [47:0] d;
  } spatz_cluster_peripheral_hw2reg_perf_counter_mreg_t;

  typedef struct packed {
    logic [31:0] d;
  } spatz_cluster_peripheral_hw2reg_hw_barrier_reg_t;

  // Register -> HW type
  typedef struct packed {
    spatz_cluster_peripheral_reg2hw_perf_counter_enable_mreg_t [15:0] perf_counter_enable; // [1538:1043]
    spatz_cluster_peripheral_reg2hw_hart_select_mreg_t [15:0] hart_select; // [1042:883]
    spatz_cluster_peripheral_reg2hw_perf_counter_mreg_t [15:0] perf_counter; // [882:99]
    spatz_cluster_peripheral_reg2hw_cl_clint_set_reg_t cl_clint_set; // [98:66]
    spatz_cluster_peripheral_reg2hw_cl_clint_clear_reg_t cl_clint_clear; // [65:33]
    spatz_cluster_peripheral_reg2hw_hw_barrier_reg_t hw_barrier; // [32:1]
    spatz_cluster_peripheral_reg2hw_icache_prefetch_enable_reg_t icache_prefetch_enable; // [0:0]
  } spatz_cluster_peripheral_reg2hw_t;

  // HW -> register type
  typedef struct packed {
    spatz_cluster_peripheral_hw2reg_perf_counter_mreg_t [15:0] perf_counter; // [799:32]
    spatz_cluster_peripheral_hw2reg_hw_barrier_reg_t hw_barrier; // [31:0]
  } spatz_cluster_peripheral_hw2reg_t;

  // Register offsets
  parameter logic [BlockAw-1:0] SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_ENABLE_0_OFFSET = 9'h 0;
  parameter logic [BlockAw-1:0] SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_ENABLE_1_OFFSET = 9'h 8;
  parameter logic [BlockAw-1:0] SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_ENABLE_2_OFFSET = 9'h 10;
  parameter logic [BlockAw-1:0] SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_ENABLE_3_OFFSET = 9'h 18;
  parameter logic [BlockAw-1:0] SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_ENABLE_4_OFFSET = 9'h 20;
  parameter logic [BlockAw-1:0] SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_ENABLE_5_OFFSET = 9'h 28;
  parameter logic [BlockAw-1:0] SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_ENABLE_6_OFFSET = 9'h 30;
  parameter logic [BlockAw-1:0] SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_ENABLE_7_OFFSET = 9'h 38;
  parameter logic [BlockAw-1:0] SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_ENABLE_8_OFFSET = 9'h 40;
  parameter logic [BlockAw-1:0] SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_ENABLE_9_OFFSET = 9'h 48;
  parameter logic [BlockAw-1:0] SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_ENABLE_10_OFFSET = 9'h 50;
  parameter logic [BlockAw-1:0] SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_ENABLE_11_OFFSET = 9'h 58;
  parameter logic [BlockAw-1:0] SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_ENABLE_12_OFFSET = 9'h 60;
  parameter logic [BlockAw-1:0] SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_ENABLE_13_OFFSET = 9'h 68;
  parameter logic [BlockAw-1:0] SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_ENABLE_14_OFFSET = 9'h 70;
  parameter logic [BlockAw-1:0] SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_ENABLE_15_OFFSET = 9'h 78;
  parameter logic [BlockAw-1:0] SPATZ_CLUSTER_PERIPHERAL_HART_SELECT_0_OFFSET = 9'h 80;
  parameter logic [BlockAw-1:0] SPATZ_CLUSTER_PERIPHERAL_HART_SELECT_1_OFFSET = 9'h 88;
  parameter logic [BlockAw-1:0] SPATZ_CLUSTER_PERIPHERAL_HART_SELECT_2_OFFSET = 9'h 90;
  parameter logic [BlockAw-1:0] SPATZ_CLUSTER_PERIPHERAL_HART_SELECT_3_OFFSET = 9'h 98;
  parameter logic [BlockAw-1:0] SPATZ_CLUSTER_PERIPHERAL_HART_SELECT_4_OFFSET = 9'h a0;
  parameter logic [BlockAw-1:0] SPATZ_CLUSTER_PERIPHERAL_HART_SELECT_5_OFFSET = 9'h a8;
  parameter logic [BlockAw-1:0] SPATZ_CLUSTER_PERIPHERAL_HART_SELECT_6_OFFSET = 9'h b0;
  parameter logic [BlockAw-1:0] SPATZ_CLUSTER_PERIPHERAL_HART_SELECT_7_OFFSET = 9'h b8;
  parameter logic [BlockAw-1:0] SPATZ_CLUSTER_PERIPHERAL_HART_SELECT_8_OFFSET = 9'h c0;
  parameter logic [BlockAw-1:0] SPATZ_CLUSTER_PERIPHERAL_HART_SELECT_9_OFFSET = 9'h c8;
  parameter logic [BlockAw-1:0] SPATZ_CLUSTER_PERIPHERAL_HART_SELECT_10_OFFSET = 9'h d0;
  parameter logic [BlockAw-1:0] SPATZ_CLUSTER_PERIPHERAL_HART_SELECT_11_OFFSET = 9'h d8;
  parameter logic [BlockAw-1:0] SPATZ_CLUSTER_PERIPHERAL_HART_SELECT_12_OFFSET = 9'h e0;
  parameter logic [BlockAw-1:0] SPATZ_CLUSTER_PERIPHERAL_HART_SELECT_13_OFFSET = 9'h e8;
  parameter logic [BlockAw-1:0] SPATZ_CLUSTER_PERIPHERAL_HART_SELECT_14_OFFSET = 9'h f0;
  parameter logic [BlockAw-1:0] SPATZ_CLUSTER_PERIPHERAL_HART_SELECT_15_OFFSET = 9'h f8;
  parameter logic [BlockAw-1:0] SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_0_OFFSET = 9'h 100;
  parameter logic [BlockAw-1:0] SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_1_OFFSET = 9'h 108;
  parameter logic [BlockAw-1:0] SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_2_OFFSET = 9'h 110;
  parameter logic [BlockAw-1:0] SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_3_OFFSET = 9'h 118;
  parameter logic [BlockAw-1:0] SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_4_OFFSET = 9'h 120;
  parameter logic [BlockAw-1:0] SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_5_OFFSET = 9'h 128;
  parameter logic [BlockAw-1:0] SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_6_OFFSET = 9'h 130;
  parameter logic [BlockAw-1:0] SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_7_OFFSET = 9'h 138;
  parameter logic [BlockAw-1:0] SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_8_OFFSET = 9'h 140;
  parameter logic [BlockAw-1:0] SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_9_OFFSET = 9'h 148;
  parameter logic [BlockAw-1:0] SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_10_OFFSET = 9'h 150;
  parameter logic [BlockAw-1:0] SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_11_OFFSET = 9'h 158;
  parameter logic [BlockAw-1:0] SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_12_OFFSET = 9'h 160;
  parameter logic [BlockAw-1:0] SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_13_OFFSET = 9'h 168;
  parameter logic [BlockAw-1:0] SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_14_OFFSET = 9'h 170;
  parameter logic [BlockAw-1:0] SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_15_OFFSET = 9'h 178;
  parameter logic [BlockAw-1:0] SPATZ_CLUSTER_PERIPHERAL_CL_CLINT_SET_OFFSET = 9'h 180;
  parameter logic [BlockAw-1:0] SPATZ_CLUSTER_PERIPHERAL_CL_CLINT_CLEAR_OFFSET = 9'h 188;
  parameter logic [BlockAw-1:0] SPATZ_CLUSTER_PERIPHERAL_HW_BARRIER_OFFSET = 9'h 190;
  parameter logic [BlockAw-1:0] SPATZ_CLUSTER_PERIPHERAL_ICACHE_PREFETCH_ENABLE_OFFSET = 9'h 198;

  // Reset values for hwext registers and their fields
  parameter logic [47:0] SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_0_RESVAL = 48'h 0;
  parameter logic [47:0] SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_1_RESVAL = 48'h 0;
  parameter logic [47:0] SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_2_RESVAL = 48'h 0;
  parameter logic [47:0] SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_3_RESVAL = 48'h 0;
  parameter logic [47:0] SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_4_RESVAL = 48'h 0;
  parameter logic [47:0] SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_5_RESVAL = 48'h 0;
  parameter logic [47:0] SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_6_RESVAL = 48'h 0;
  parameter logic [47:0] SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_7_RESVAL = 48'h 0;
  parameter logic [47:0] SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_8_RESVAL = 48'h 0;
  parameter logic [47:0] SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_9_RESVAL = 48'h 0;
  parameter logic [47:0] SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_10_RESVAL = 48'h 0;
  parameter logic [47:0] SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_11_RESVAL = 48'h 0;
  parameter logic [47:0] SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_12_RESVAL = 48'h 0;
  parameter logic [47:0] SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_13_RESVAL = 48'h 0;
  parameter logic [47:0] SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_14_RESVAL = 48'h 0;
  parameter logic [47:0] SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_15_RESVAL = 48'h 0;
  parameter logic [31:0] SPATZ_CLUSTER_PERIPHERAL_CL_CLINT_SET_RESVAL = 32'h 0;
  parameter logic [31:0] SPATZ_CLUSTER_PERIPHERAL_CL_CLINT_CLEAR_RESVAL = 32'h 0;
  parameter logic [31:0] SPATZ_CLUSTER_PERIPHERAL_HW_BARRIER_RESVAL = 32'h 0;

  // Register index
  typedef enum int {
    SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_ENABLE_0,
    SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_ENABLE_1,
    SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_ENABLE_2,
    SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_ENABLE_3,
    SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_ENABLE_4,
    SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_ENABLE_5,
    SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_ENABLE_6,
    SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_ENABLE_7,
    SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_ENABLE_8,
    SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_ENABLE_9,
    SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_ENABLE_10,
    SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_ENABLE_11,
    SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_ENABLE_12,
    SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_ENABLE_13,
    SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_ENABLE_14,
    SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_ENABLE_15,
    SPATZ_CLUSTER_PERIPHERAL_HART_SELECT_0,
    SPATZ_CLUSTER_PERIPHERAL_HART_SELECT_1,
    SPATZ_CLUSTER_PERIPHERAL_HART_SELECT_2,
    SPATZ_CLUSTER_PERIPHERAL_HART_SELECT_3,
    SPATZ_CLUSTER_PERIPHERAL_HART_SELECT_4,
    SPATZ_CLUSTER_PERIPHERAL_HART_SELECT_5,
    SPATZ_CLUSTER_PERIPHERAL_HART_SELECT_6,
    SPATZ_CLUSTER_PERIPHERAL_HART_SELECT_7,
    SPATZ_CLUSTER_PERIPHERAL_HART_SELECT_8,
    SPATZ_CLUSTER_PERIPHERAL_HART_SELECT_9,
    SPATZ_CLUSTER_PERIPHERAL_HART_SELECT_10,
    SPATZ_CLUSTER_PERIPHERAL_HART_SELECT_11,
    SPATZ_CLUSTER_PERIPHERAL_HART_SELECT_12,
    SPATZ_CLUSTER_PERIPHERAL_HART_SELECT_13,
    SPATZ_CLUSTER_PERIPHERAL_HART_SELECT_14,
    SPATZ_CLUSTER_PERIPHERAL_HART_SELECT_15,
    SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_0,
    SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_1,
    SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_2,
    SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_3,
    SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_4,
    SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_5,
    SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_6,
    SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_7,
    SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_8,
    SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_9,
    SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_10,
    SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_11,
    SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_12,
    SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_13,
    SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_14,
    SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_15,
    SPATZ_CLUSTER_PERIPHERAL_CL_CLINT_SET,
    SPATZ_CLUSTER_PERIPHERAL_CL_CLINT_CLEAR,
    SPATZ_CLUSTER_PERIPHERAL_HW_BARRIER,
    SPATZ_CLUSTER_PERIPHERAL_ICACHE_PREFETCH_ENABLE
  } spatz_cluster_peripheral_id_e;

  // Register width information to check illegal writes
  parameter logic [3:0] SPATZ_CLUSTER_PERIPHERAL_PERMIT [52] = '{
    4'b 1111, // index[ 0] SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_ENABLE_0
    4'b 1111, // index[ 1] SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_ENABLE_1
    4'b 1111, // index[ 2] SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_ENABLE_2
    4'b 1111, // index[ 3] SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_ENABLE_3
    4'b 1111, // index[ 4] SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_ENABLE_4
    4'b 1111, // index[ 5] SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_ENABLE_5
    4'b 1111, // index[ 6] SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_ENABLE_6
    4'b 1111, // index[ 7] SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_ENABLE_7
    4'b 1111, // index[ 8] SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_ENABLE_8
    4'b 1111, // index[ 9] SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_ENABLE_9
    4'b 1111, // index[10] SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_ENABLE_10
    4'b 1111, // index[11] SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_ENABLE_11
    4'b 1111, // index[12] SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_ENABLE_12
    4'b 1111, // index[13] SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_ENABLE_13
    4'b 1111, // index[14] SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_ENABLE_14
    4'b 1111, // index[15] SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_ENABLE_15
    4'b 0011, // index[16] SPATZ_CLUSTER_PERIPHERAL_HART_SELECT_0
    4'b 0011, // index[17] SPATZ_CLUSTER_PERIPHERAL_HART_SELECT_1
    4'b 0011, // index[18] SPATZ_CLUSTER_PERIPHERAL_HART_SELECT_2
    4'b 0011, // index[19] SPATZ_CLUSTER_PERIPHERAL_HART_SELECT_3
    4'b 0011, // index[20] SPATZ_CLUSTER_PERIPHERAL_HART_SELECT_4
    4'b 0011, // index[21] SPATZ_CLUSTER_PERIPHERAL_HART_SELECT_5
    4'b 0011, // index[22] SPATZ_CLUSTER_PERIPHERAL_HART_SELECT_6
    4'b 0011, // index[23] SPATZ_CLUSTER_PERIPHERAL_HART_SELECT_7
    4'b 0011, // index[24] SPATZ_CLUSTER_PERIPHERAL_HART_SELECT_8
    4'b 0011, // index[25] SPATZ_CLUSTER_PERIPHERAL_HART_SELECT_9
    4'b 0011, // index[26] SPATZ_CLUSTER_PERIPHERAL_HART_SELECT_10
    4'b 0011, // index[27] SPATZ_CLUSTER_PERIPHERAL_HART_SELECT_11
    4'b 0011, // index[28] SPATZ_CLUSTER_PERIPHERAL_HART_SELECT_12
    4'b 0011, // index[29] SPATZ_CLUSTER_PERIPHERAL_HART_SELECT_13
    4'b 0011, // index[30] SPATZ_CLUSTER_PERIPHERAL_HART_SELECT_14
    4'b 0011, // index[31] SPATZ_CLUSTER_PERIPHERAL_HART_SELECT_15
    4'b 1111, // index[32] SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_0
    4'b 1111, // index[33] SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_1
    4'b 1111, // index[34] SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_2
    4'b 1111, // index[35] SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_3
    4'b 1111, // index[36] SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_4
    4'b 1111, // index[37] SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_5
    4'b 1111, // index[38] SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_6
    4'b 1111, // index[39] SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_7
    4'b 1111, // index[40] SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_8
    4'b 1111, // index[41] SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_9
    4'b 1111, // index[42] SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_10
    4'b 1111, // index[43] SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_11
    4'b 1111, // index[44] SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_12
    4'b 1111, // index[45] SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_13
    4'b 1111, // index[46] SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_14
    4'b 1111, // index[47] SPATZ_CLUSTER_PERIPHERAL_PERF_COUNTER_15
    4'b 1111, // index[48] SPATZ_CLUSTER_PERIPHERAL_CL_CLINT_SET
    4'b 1111, // index[49] SPATZ_CLUSTER_PERIPHERAL_CL_CLINT_CLEAR
    4'b 1111, // index[50] SPATZ_CLUSTER_PERIPHERAL_HW_BARRIER
    4'b 0001  // index[51] SPATZ_CLUSTER_PERIPHERAL_ICACHE_PREFETCH_ENABLE
  };

endpackage

