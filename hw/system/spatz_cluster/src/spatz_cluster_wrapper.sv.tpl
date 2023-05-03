// Copyright 2021 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

${disclaimer}

<%def name="core_cfg(prop)">\
  % for c in cfg['cores']:
${c[prop]}${', ' if not loop.last else ''}\
  % endfor
</%def>\

<%def name="core_cfg_flat(prop)">\
${cfg['nr_cores']}'b\
  % for c in cfg['cores'][::-1]:
${int(c[prop])}\
  % endfor
</%def>\

<%def name="core_isa(isa)">\
${cfg['nr_cores']}'b\
  % for c in cfg['cores'][::-1]:
${int(getattr(c['isa_parsed'], isa))}\
  % endfor
</%def>\

`include "axi/typedef.svh"

// verilog_lint: waive-start package-filename
package ${cfg['pkg_name']};

  ///////////
  //  AXI  //
  ///////////

  // AXI Data Width
  localparam int unsigned SpatzAxiDataWidth = ${cfg['dma_data_width']};
  localparam int unsigned SpatzAxiStrbWidth = SpatzAxiDataWidth / 8;
  // AXI Address Width
  localparam int unsigned SpatzAxiAddrWidth = ${cfg['addr_width']};
  // AXI ID Width
  localparam int unsigned SpatzAxiIdInWidth = ${cfg['id_width_in']};
  localparam int unsigned SpatzAxiIdOutWidth = ${cfg['id_width_out']};
  // AXI User Width
  localparam int unsigned SpatzAxiUserWidth = ${cfg['user_width']};

  typedef logic [SpatzAxiDataWidth-1:0] axi_data_t;
  typedef logic [SpatzAxiStrbWidth-1:0] axi_strb_t;
  typedef logic [SpatzAxiAddrWidth-1:0] axi_addr_t;
  typedef logic [SpatzAxiIdInWidth-1:0] axi_id_in_t;
  typedef logic [SpatzAxiIdOutWidth-1:0] axi_id_out_t;
  typedef logic [SpatzAxiUserWidth-1:0] axi_user_t;

% if cfg['cdc_enable']:
  localparam int unsigned SpatzLogDepth = 3;
% endif

  `AXI_TYPEDEF_ALL(spatz_axi_in, axi_addr_t, axi_id_in_t, logic [63:0], logic [7:0], axi_user_t)
  `AXI_TYPEDEF_ALL(spatz_axi_out, axi_addr_t, axi_id_out_t, axi_data_t, axi_strb_t, axi_user_t)

  ////////////////////
  //  Spatz Cluster //
  ////////////////////

  localparam int unsigned NumCores = ${cfg['nr_cores']};

  localparam int unsigned DataWidth  = 64;
  localparam int unsigned BeWidth    = DataWidth / 8;
  localparam int unsigned ByteOffset = $clog2(BeWidth);

  localparam int unsigned ICacheLineWidth = ${cfg['icache']['cacheline']};
  localparam int unsigned ICacheLineCount = ${cfg['icache']['depth']};
  localparam int unsigned ICacheSets = ${cfg['icache']['sets']};

  localparam int unsigned TCDMStartAddr = ${to_sv_hex(cfg['cluster_base_addr'], cfg['addr_width'])};
  localparam int unsigned TCDMSize      = ${to_sv_hex(cfg['tcdm']['size'] * 1024, cfg['addr_width'])};

  localparam int unsigned PeriStartAddr = TCDMStartAddr + TCDMSize;

  localparam int unsigned BootAddr      = ${to_sv_hex(cfg['boot_addr'], cfg['addr_width'])};

  function automatic snitch_pma_pkg::rule_t [snitch_pma_pkg::NrMaxRules-1:0] get_cached_regions();
    automatic snitch_pma_pkg::rule_t [snitch_pma_pkg::NrMaxRules-1:0] cached_regions;
    cached_regions = '{default: '0};
% for i, cp in enumerate(cfg['pmas']['cached']):
    cached_regions[${i}] = '{base: ${to_sv_hex(cp[0], cfg['addr_width'])}, mask: ${to_sv_hex(cp[1], cfg['addr_width'])}};
% endfor
    return cached_regions;
  endfunction

  localparam snitch_pma_pkg::snitch_pma_t SnitchPMACfg = '{
      NrCachedRegionRules: ${len(cfg['pmas']['cached'])},
      CachedRegion: get_cached_regions(),
      default: 0
  };

  localparam fpnew_pkg::fpu_implementation_t FPUImplementation [NumCores] = '{
  % for c in cfg['cores']:
    '{
        PipeRegs: // FMA Block
                  '{
                    '{  ${cfg['timing']['lat_comp_fp32']}, // FP32
                        ${cfg['timing']['lat_comp_fp64']}, // FP64
                        ${cfg['timing']['lat_comp_fp16']}, // FP16
                        ${cfg['timing']['lat_comp_fp8']}, // FP8
                        ${cfg['timing']['lat_comp_fp16_alt']}, // FP16alt
                        ${cfg['timing']['lat_comp_fp8_alt']}  // FP8alt
                      },
                    '{1, 1, 1, 1, 1, 1},   // DIVSQRT
                    '{${cfg['timing']['lat_noncomp']},
                      ${cfg['timing']['lat_noncomp']},
                      ${cfg['timing']['lat_noncomp']},
                      ${cfg['timing']['lat_noncomp']},
                      ${cfg['timing']['lat_noncomp']},
                      ${cfg['timing']['lat_noncomp']}},   // NONCOMP
                    '{${cfg['timing']['lat_conv']},
                      ${cfg['timing']['lat_conv']},
                      ${cfg['timing']['lat_conv']},
                      ${cfg['timing']['lat_conv']},
                      ${cfg['timing']['lat_conv']},
                      ${cfg['timing']['lat_conv']}},   // CONV
                    '{${cfg['timing']['lat_sdotp']},
                      ${cfg['timing']['lat_sdotp']},
                      ${cfg['timing']['lat_sdotp']},
                      ${cfg['timing']['lat_sdotp']},
                      ${cfg['timing']['lat_sdotp']},
                      ${cfg['timing']['lat_sdotp']}}    // DOTP
                    },
        UnitTypes: '{'{fpnew_pkg::MERGED,
                       fpnew_pkg::MERGED,
                       fpnew_pkg::MERGED,
                       fpnew_pkg::MERGED,
                       fpnew_pkg::MERGED,
                       fpnew_pkg::MERGED},  // FMA
                    '{fpnew_pkg::DISABLED,
                        fpnew_pkg::DISABLED,
                        fpnew_pkg::DISABLED,
                        fpnew_pkg::DISABLED,
                        fpnew_pkg::DISABLED,
                        fpnew_pkg::DISABLED}, // DIVSQRT
                    '{fpnew_pkg::PARALLEL,
                        fpnew_pkg::PARALLEL,
                        fpnew_pkg::PARALLEL,
                        fpnew_pkg::PARALLEL,
                        fpnew_pkg::PARALLEL,
                        fpnew_pkg::PARALLEL}, // NONCOMP
                    '{fpnew_pkg::MERGED,
                        fpnew_pkg::MERGED,
                        fpnew_pkg::MERGED,
                        fpnew_pkg::MERGED,
                        fpnew_pkg::MERGED,
                        fpnew_pkg::MERGED},   // CONV
% if c["xfdotp"]:
                    '{fpnew_pkg::MERGED,
                        fpnew_pkg::MERGED,
                        fpnew_pkg::MERGED,
                        fpnew_pkg::MERGED,
                        fpnew_pkg::MERGED,
                        fpnew_pkg::MERGED}},  // DOTP
% else:
                    '{fpnew_pkg::DISABLED,
                        fpnew_pkg::DISABLED,
                        fpnew_pkg::DISABLED,
                        fpnew_pkg::DISABLED,
                        fpnew_pkg::DISABLED,
                        fpnew_pkg::DISABLED}}, // DOTP
% endif
        PipeConfig: fpnew_pkg::${cfg['timing']['fpu_pipe_config']}
    }${',\n' if not loop.last else '\n'}\
  % endfor
  };

endpackage
// verilog_lint: waive-stop package-filename

module ${cfg['name']}_wrapper
 import ${cfg['pkg_name']}::*;
 import fpnew_pkg::fpu_implementation_t;
 import snitch_pma_pkg::snitch_pma_t;
 #(
  parameter int unsigned AxiAddrWidth  = ${cfg['pkg_name']}::SpatzAxiAddrWidth,
  parameter int unsigned AxiDataWidth  = ${cfg['pkg_name']}::SpatzAxiDataWidth,
  parameter int unsigned AxiUserWidth  = ${cfg['pkg_name']}::SpatzAxiUserWidth,
  parameter int unsigned AxiInIdWidth  = ${cfg['pkg_name']}::SpatzAxiIdInWidth,
  parameter int unsigned AxiOutIdWidth = ${cfg['pkg_name']}::SpatzAxiIdOutWidth,
% if cfg['cdc_enable']:
  parameter int unsigned LogDepth      = ${cfg['pkg_name']}::SpatzLogDepth,
% endif

  parameter type axi_in_resp_t = spatz_axi_in_resp_t,
  parameter type axi_in_req_t  = spatz_axi_in_req_t,
% if cfg['cdc_enable']:
  parameter type axi_in_aw_chan_t = spatz_axi_in_aw_chan_t,
  parameter type axi_in_w_chan_t  = spatz_axi_in_w_chan_t,
  parameter type axi_in_b_chan_t  = spatz_axi_in_b_chan_t,
  parameter type axi_in_ar_chan_t = spatz_axi_in_ar_chan_t,
  parameter type axi_in_r_chan_t  = spatz_axi_in_r_chan_t,
% endif

  parameter type axi_out_resp_t = spatz_axi_out_resp_t,
% if cfg['cdc_enable']:
  parameter type axi_out_req_t  = spatz_axi_out_req_t,

  parameter type axi_out_aw_chan_t = spatz_axi_out_aw_chan_t,
  parameter type axi_out_w_chan_t  = spatz_axi_out_w_chan_t,
  parameter type axi_out_b_chan_t  = spatz_axi_out_b_chan_t,
  parameter type axi_out_ar_chan_t = spatz_axi_out_ar_chan_t,
  parameter type axi_out_r_chan_t  = spatz_axi_out_r_chan_t,

  // AXI Master
  parameter int unsigned AsyncAxiOutAwWidth = (2**LogDepth) * axi_pkg::aw_width(SpatzAxiAddrWidth,  SpatzAxiIdOutWidth, SpatzAxiUserWidth),
  parameter int unsigned AsyncAxiOutWWidth  = (2**LogDepth) * axi_pkg::w_width (SpatzAxiDataWidth,  SpatzAxiUserWidth),
  parameter int unsigned AsyncAxiOutBWidth  = (2**LogDepth) * axi_pkg::b_width (SpatzAxiIdOutWidth, SpatzAxiUserWidth),
  parameter int unsigned AsyncAxiOutArWidth = (2**LogDepth) * axi_pkg::ar_width(SpatzAxiAddrWidth,  SpatzAxiIdOutWidth, SpatzAxiUserWidth),
  parameter int unsigned AsyncAxiOutRWidth  = (2**LogDepth) * axi_pkg::r_width (SpatzAxiDataWidth,  SpatzAxiIdOutWidth, SpatzAxiUserWidth),

  // AXI Slave
  parameter int unsigned AsyncAxiInAwWidth = (2**LogDepth) * axi_pkg::aw_width(SpatzAxiAddrWidth, SpatzAxiIdInWidth, SpatzAxiUserWidth),
  parameter int unsigned AsyncAxiInWWidth  = (2**LogDepth) * axi_pkg::w_width (SpatzAxiDataWidth, SpatzAxiUserWidth),
  parameter int unsigned AsyncAxiInBWidth  = (2**LogDepth) * axi_pkg::b_width (SpatzAxiIdInWidth, SpatzAxiUserWidth),
  parameter int unsigned AsyncAxiInArWidth = (2**LogDepth) * axi_pkg::ar_width(SpatzAxiAddrWidth, SpatzAxiIdInWidth, SpatzAxiUserWidth),
  parameter int unsigned AsyncAxiInRWidth  = (2**LogDepth) * axi_pkg::r_width (SpatzAxiDataWidth, SpatzAxiIdInWidth, SpatzAxiUserWidth)
% else:
  parameter type axi_out_req_t  = spatz_axi_out_req_t
% endif
)(
  input  logic                clk_i,
  input  logic                rst_ni,
  input  logic                testmode_i,
  input  logic                scan_enable_i,
  input  logic                scan_data_i,
  output logic                scan_data_o,
% if cfg['enable_debug']:
  input  logic [NumCores-1:0] debug_req_i,
% endif

  input  logic [NumCores-1:0] meip_i,
  input  logic [NumCores-1:0] mtip_i,
  input  logic [NumCores-1:0] msip_i,
% if not cfg['tie_ports']:
  input  logic [9:0]                           hart_base_id_i,
  input  logic [AxiAddrWidth-1:0]              cluster_base_addr_i,
% endif
  output logic                                 cluster_probe_o,
% if cfg['cdc_enable']:
  // AXI Master port
  output logic [AsyncAxiOutAwWidth-1:0] async_axi_out_aw_data_o,
  output logic [LogDepth:0]             async_axi_out_aw_wptr_o,
  input  logic [LogDepth:0]             async_axi_out_aw_rptr_i,
  output logic [AsyncAxiOutWWidth-1:0]  async_axi_out_w_data_o,
  output logic [LogDepth:0]             async_axi_out_w_wptr_o,
  input  logic [LogDepth:0]             async_axi_out_w_rptr_i,
  input  logic [AsyncAxiOutBWidth-1:0]  async_axi_out_b_data_i,
  input  logic [LogDepth:0]             async_axi_out_b_wptr_i,
  output logic [LogDepth:0]             async_axi_out_b_rptr_o,
  output logic [AsyncAxiOutArWidth-1:0] async_axi_out_ar_data_o,
  output logic [LogDepth:0]             async_axi_out_ar_wptr_o,
  input  logic [LogDepth:0]             async_axi_out_ar_rptr_i,
  input  logic [AsyncAxiOutRWidth-1:0]  async_axi_out_r_data_i,
  input  logic [LogDepth:0]             async_axi_out_r_wptr_i,
  output logic [LogDepth:0]             async_axi_out_r_rptr_o,

  // AXI Slave port
  input  logic [AsyncAxiInArWidth-1:0] async_axi_in_ar_data_i,
  input  logic [LogDepth:0]            async_axi_in_ar_wptr_i,
  output logic [LogDepth:0]            async_axi_in_ar_rptr_o,
  input  logic [AsyncAxiInAwWidth-1:0] async_axi_in_aw_data_i,
  input  logic [LogDepth:0]            async_axi_in_aw_wptr_i,
  output logic [LogDepth:0]            async_axi_in_aw_rptr_o,
  output logic [AsyncAxiInBWidth-1:0]  async_axi_in_b_data_o,
  output logic [LogDepth:0]            async_axi_in_b_wptr_o,
  input  logic [LogDepth:0]            async_axi_in_b_rptr_i,
  output logic [AsyncAxiInRWidth-1:0]  async_axi_in_r_data_o,
  output logic [LogDepth:0]            async_axi_in_r_wptr_o,
  input  logic [LogDepth:0]            async_axi_in_r_rptr_i,
  input  logic [AsyncAxiInWWidth-1:0]  async_axi_in_w_data_i,
  input  logic [LogDepth:0]            async_axi_in_w_wptr_i,
  output logic [LogDepth:0]            async_axi_in_w_rptr_o
%else:
  input  axi_in_req_t   axi_in_req_i,
  output axi_in_resp_t  axi_in_resp_o,
  output axi_out_req_t  axi_out_req_o,
  input  axi_out_resp_t axi_out_resp_i
% endif
);

  localparam int unsigned NumIntOutstandingLoads   [NumCores] = '{${core_cfg('num_int_outstanding_loads')}};
  localparam int unsigned NumIntOutstandingMem     [NumCores] = '{${core_cfg('num_int_outstanding_mem')}};
  localparam int unsigned NumSpatzOutstandingLoads [NumCores] = '{${core_cfg('num_spatz_outstanding_loads')}};


% if cfg['cdc_enable']:
  // From CDC to Cluster
  axi_in_req_t   axi_to_cluster_req;
  axi_in_resp_t  axi_to_cluster_resp;

  // From Cluster to CDC
  axi_out_req_t  axi_from_cluster_req;
  axi_out_resp_t axi_from_cluster_resp;

  axi_cdc_dst #(
    .LogDepth   ( LogDepth          ),
    .aw_chan_t  ( axi_in_aw_chan_t  ),
    .w_chan_t   ( axi_in_w_chan_t   ),
    .b_chan_t   ( axi_in_b_chan_t   ),
    .ar_chan_t  ( axi_in_ar_chan_t  ),
    .r_chan_t   ( axi_in_r_chan_t   ),
    .axi_req_t  ( axi_in_req_t      ),
    .axi_resp_t ( axi_in_resp_t     )
  ) i_spatz_cluster_cdc_dst (
    // Asynchronous slave port
    .async_data_slave_aw_data_i ( async_axi_in_aw_data_i ),
    .async_data_slave_aw_wptr_i ( async_axi_in_aw_wptr_i ),
    .async_data_slave_aw_rptr_o ( async_axi_in_aw_rptr_o ),
    .async_data_slave_w_data_i  ( async_axi_in_w_data_i  ),
    .async_data_slave_w_wptr_i  ( async_axi_in_w_wptr_i  ),
    .async_data_slave_w_rptr_o  ( async_axi_in_w_rptr_o  ),
    .async_data_slave_b_data_o  ( async_axi_in_b_data_o  ),
    .async_data_slave_b_wptr_o  ( async_axi_in_b_wptr_o  ),
    .async_data_slave_b_rptr_i  ( async_axi_in_b_rptr_i  ),
    .async_data_slave_ar_data_i ( async_axi_in_ar_data_i ),
    .async_data_slave_ar_wptr_i ( async_axi_in_ar_wptr_i ),
    .async_data_slave_ar_rptr_o ( async_axi_in_ar_rptr_o ),
    .async_data_slave_r_data_o  ( async_axi_in_r_data_o  ),
    .async_data_slave_r_wptr_o  ( async_axi_in_r_wptr_o  ),
    .async_data_slave_r_rptr_i  ( async_axi_in_r_rptr_i  ),
    // Synchronous master port
    .dst_clk_i                  ( clk_i                  ),
    .dst_rst_ni                 ( rst_ni                 ),
    .dst_req_o                  ( axi_to_cluster_req     ),
    .dst_resp_i                 ( axi_to_cluster_resp    )
  );

  axi_cdc_src #(
   .LogDepth   ( LogDepth          ),
   .aw_chan_t  ( axi_out_aw_chan_t ),
   .w_chan_t   ( axi_out_w_chan_t  ),
   .b_chan_t   ( axi_out_b_chan_t  ),
   .ar_chan_t  ( axi_out_ar_chan_t ),
   .r_chan_t   ( axi_out_r_chan_t  ),
   .axi_req_t  ( axi_out_req_t     ),
   .axi_resp_t ( axi_out_resp_t    )
  ) i_spatz_cluster_cdc_src (
    // Asynchronous Master port
    .async_data_master_aw_data_o( async_axi_out_aw_data_o ),
    .async_data_master_aw_wptr_o( async_axi_out_aw_wptr_o ),
    .async_data_master_aw_rptr_i( async_axi_out_aw_rptr_i ),
    .async_data_master_w_data_o ( async_axi_out_w_data_o  ),
    .async_data_master_w_wptr_o ( async_axi_out_w_wptr_o  ),
    .async_data_master_w_rptr_i ( async_axi_out_w_rptr_i  ),
    .async_data_master_b_data_i ( async_axi_out_b_data_i  ),
    .async_data_master_b_wptr_i ( async_axi_out_b_wptr_i  ),
    .async_data_master_b_rptr_o ( async_axi_out_b_rptr_o  ),
    .async_data_master_ar_data_o( async_axi_out_ar_data_o ),
    .async_data_master_ar_wptr_o( async_axi_out_ar_wptr_o ),
    .async_data_master_ar_rptr_i( async_axi_out_ar_rptr_i ),
    .async_data_master_r_data_i ( async_axi_out_r_data_i  ),
    .async_data_master_r_wptr_i ( async_axi_out_r_wptr_i  ),
    .async_data_master_r_rptr_o ( async_axi_out_r_rptr_o  ),
    // Synchronous slave port
    .src_clk_i                  ( clk_i                   ),
    .src_rst_ni                 ( rst_ni                  ),
    .src_req_i                  ( axi_from_cluster_req    ),
    .src_resp_o                 ( axi_from_cluster_resp   )
  );
% endif

  // Spatz cluster under test.
  spatz_cluster #(
    .AxiAddrWidth (AxiAddrWidth),
    .AxiDataWidth (AxiDataWidth),
    .AxiIdWidthIn (AxiInIdWidth),
    .AxiIdWidthOut (AxiOutIdWidth),
    .AxiUserWidth (AxiUserWidth),
    .BootAddr (${to_sv_hex(cfg['boot_addr'], 32)}),
    .ClusterPeriphSize (${cfg['cluster_periph_size']}),
    .NrCores (${cfg['nr_cores']}),
    .TCDMDepth (${cfg['tcdm']['depth']}),
    .NrBanks (${cfg['tcdm']['banks']}),
    .ICacheLineWidth (${cfg['pkg_name']}::ICacheLineWidth),
    .ICacheLineCount (${cfg['pkg_name']}::ICacheLineCount),
    .ICacheSets (${cfg['pkg_name']}::ICacheSets),
    .FPUImplementation (${cfg['pkg_name']}::FPUImplementation),
    .SnitchPMACfg (${cfg['pkg_name']}::SnitchPMACfg),
    .NumIntOutstandingLoads (NumIntOutstandingLoads),
    .NumIntOutstandingMem (NumIntOutstandingMem),
    .NumSpatzOutstandingLoads (NumSpatzOutstandingLoads),
    .axi_in_req_t (axi_in_req_t),
    .axi_in_resp_t (axi_in_resp_t),
    .axi_out_req_t (axi_out_req_t),
    .axi_out_resp_t (axi_out_resp_t),
    .Xdma (${core_cfg_flat('xdma')}),
    .DMAAxiReqFifoDepth (${cfg['dma_axi_req_fifo_depth']}),
    .DMAReqFifoDepth (${cfg['dma_req_fifo_depth']}),
    .RegisterOffloadReq (${int(cfg['timing']['register_offload_req'])}),
    .RegisterOffloadRsp (${int(cfg['timing']['register_offload_rsp'])}),
    .RegisterCoreReq (${int(cfg['timing']['register_core_req'])}),
    .RegisterCoreRsp (${int(cfg['timing']['register_core_rsp'])}),
    .RegisterTCDMCuts (${int(cfg['timing']['register_tcdm_cuts'])}),
    .RegisterExt (${int(cfg['timing']['register_ext'])}),
    .XbarLatency (axi_pkg::${cfg['timing']['xbar_latency']}),
    .MaxMstTrans (${cfg['trans']}),
    .MaxSlvTrans (${cfg['trans']})
  ) i_cluster (
    .clk_i,
    .rst_ni,
% if cfg['enable_debug']:
    .debug_req_i,
% else:
    .debug_req_i ('0),
% endif
    .meip_i,
    .mtip_i,
    .msip_i,
% if cfg['tie_ports']:
    .hart_base_id_i (${to_sv_hex(cfg['hart_base_id'], 10)}),
    .cluster_base_addr_i (${to_sv_hex(cfg['cluster_base_addr'], cfg['addr_width'])}),
% else:
    .hart_base_id_i,
    .cluster_base_addr_i,
% endif
    .cluster_probe_o,
% if cfg['cdc_enable']:
    //AXI Slave Port
    .axi_in_req_i (axi_to_cluster_req),
    .axi_in_resp_o (axi_to_cluster_resp),
    //AXI Master Port
    .axi_out_req_o (axi_from_cluster_req),
    .axi_out_resp_i (axi_from_cluster_resp)
% else:
    .axi_in_req_i,
    .axi_in_resp_o,
    .axi_out_req_o,
    .axi_out_resp_i
% endif
  );

  // Assertions

  if (AxiAddrWidth != ${cfg['pkg_name']}::SpatzAxiAddrWidth)
    $error("[spatz_cluster_wrapper] AXI Address Width does not match the configuration.");

  if (AxiDataWidth != ${cfg['pkg_name']}::SpatzAxiDataWidth)
    $error("[spatz_cluster_wrapper] AXI Data Width does not match the configuration.");

  if (AxiUserWidth != ${cfg['pkg_name']}::SpatzAxiUserWidth)
    $error("[spatz_cluster_wrapper] AXI User Width does not match the configuration.");

  if (AxiIdInWidth != ${cfg['pkg_name']}::SpatzAxiIdInWidth)
    $error("[spatz_cluster_wrapper] AXI Id Width (In) does not match the configuration.");

  if (AxiIdOutWidth != ${cfg['pkg_name']}::SpatzAxiIdOutWidth)
    $error("[spatz_cluster_wrapper] AXI Id Width (Out) does not match the configuration.");

  if (LogDepth != ${cfg['pkg_name']}::SpatzLogDepth)
    $error("[spatz_cluster_wrapper] AXI Log Depth does not match the configuration.");
endmodule
