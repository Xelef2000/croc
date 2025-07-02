// Copyright 2024 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Authors:
// - Philippe Sauter <phsauter@iis.ee.ethz.ch>

module croc_soc import croc_pkg::*; #(
  parameter int unsigned GpioCount = 16
) (
  input  logic clk_i,
  input  logic rst_ni,
  input  logic ref_clk_i,
  input  logic testmode_i,
  input  logic fetch_en_i,
  output logic status_o,

  // JTAG
  input  logic jtag_tck_i,
  input  logic jtag_tdi_i,
  output logic jtag_tdo_o,
  input  logic jtag_tms_i,
  input  logic jtag_trst_ni,

  // UART
  input  logic uart_rx_i,
  output logic uart_tx_o,

  // SPI RAM
  input  logic spi_ram_miso_i,
  output logic spi_ram_mosi_o,
  output logic spi_ram_sck_o,
  output logic spi_ram_cs_n_o,

  // GPIOs
  input  logic [GpioCount-1:0] gpio_i,
  output logic [GpioCount-1:0] gpio_o,
  output logic [GpioCount-1:0] gpio_out_en_o
);

  ///////////////
  // Reset Sync //
  ///////////////
  logic synced_rst_n, synced_fetch_en;

  rstgen i_rstgen (
    .clk_i,
    .rst_ni,
    .test_mode_i ( testmode_i ),
    .rst_no      ( synced_rst_n ),
    .init_no     ( )
  );

  sync #(
    .STAGES     (2),
    .ResetValue (1'b0)
  ) i_ext_intr_sync (
    .clk_i,
    .rst_ni   ( synced_rst_n ),
    .serial_i ( fetch_en_i ),
    .serial_o ( synced_fetch_en )
  );

  ///////////////////////
  // OBI Interconnects //
  ///////////////////////

  // Croc <-> User (shared bus resource)
  sbr_obi_req_t user_sbr_obi_req;
  sbr_obi_rsp_t user_sbr_obi_rsp;

  // User <-> Croc (manager role)
  mgr_obi_req_t user_mgr_obi_req;
  mgr_obi_rsp_t user_mgr_obi_rsp;

  ///////////////////////
  // GPIO & Interrupts //
  ///////////////////////
  logic [GpioCount-1:0] gpio_in_sync;
  logic [NumExternalIrqs-1:0] interrupts;

  ////////////////////
  // Croc Domain SoC //
  ////////////////////
  croc_domain #(
    .GpioCount(GpioCount)
  ) i_croc (
    .clk_i,
    .rst_ni           ( synced_rst_n ),
    .ref_clk_i,
    .testmode_i,
    .fetch_en_i       ( synced_fetch_en ),

    // JTAG
    .jtag_tck_i,
    .jtag_tdi_i,
    .jtag_tdo_o,
    .jtag_tms_i,
    .jtag_trst_ni,

    // UART
    .uart_rx_i,
    .uart_tx_o,

    // SPI RAM
    .spi_ram_miso_i   ( spi_ram_miso_i ),
    .spi_ram_mosi_o   ( spi_ram_mosi_o ),
    .spi_ram_sck_o    ( spi_ram_sck_o  ),
    .spi_ram_cs_n_o   ( spi_ram_cs_n_o  ),

    // GPIOs
    .gpio_i,
    .gpio_o,
    .gpio_out_en_o,
    .gpio_in_sync_o   ( gpio_in_sync ),

    // Bus connections
    .user_sbr_obi_req_o ( user_sbr_obi_req ),
    .user_sbr_obi_rsp_i ( user_sbr_obi_rsp ),

    .user_mgr_obi_req_i ( user_mgr_obi_req ),
    .user_mgr_obi_rsp_o ( user_mgr_obi_rsp ),

    // Interrupts
    .interrupts_i     ( interrupts ),
    .core_busy_o      ( status_o )
  );

  /////////////////////
  // User Domain SoC //
  /////////////////////
  user_domain #(
    .GpioCount(GpioCount)
  ) i_user (
    .clk_i,
    .rst_ni        ( synced_rst_n ),
    .ref_clk_i,
    .testmode_i,

    .user_sbr_obi_req_i ( user_sbr_obi_req ),
    .user_sbr_obi_rsp_o ( user_sbr_obi_rsp ),

    .user_mgr_obi_req_o ( user_mgr_obi_req ),
    .user_mgr_obi_rsp_i ( user_mgr_obi_rsp ),

    .gpio_in_sync_i ( gpio_in_sync ),
    .interrupts_o   ( interrupts )
  );

endmodule
