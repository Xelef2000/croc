// Copyright 2024 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Nicole Narr <narrn@student.ethz.ch>
// Christopher Reinwardt <creinwar@student.ethz.ch>
// Cyril Koenig <cykoenig@iis.ee.ethz.ch>
// Yann Picod <ypicod@ethz.ch>
// Paul Scheffler <paulsc@iis.ee.ethz.ch>
// Philippe Sauter <phsauter@iis.ee.ethz.ch>

`define ila(__name, __signal)  \
  (* dont_touch = "yes" *) (* mark_debug = "true" *) logic [$bits(__signal)-1:0] __name; \
  assign __name = __signal;


module croc_lattice import croc_pkg::*; #(
  localparam int unsigned GpioCount = 4
)  (
  input  logic  sys_clk_p,


  // input  logic  sys_reset,



  // input  logic fetch_en_i,             // switch 7
  input  logic [GpioCount-1:0] gpio_i, // switch 0-3


  output logic [GpioCount-1:0] gpio_o,


  output logic   status_o,


  input  logic  jtag_tck_i,
  input  logic  jtag_tms_i,
  input  logic  jtag_tdi_i,
  output logic  jtag_tdo_o,


  output logic  uart_tx_o,
  input  logic  uart_rx_i
);

  ////////////////////////
  //  Clock Generation  //
  ////////////////////////

  wire sys_clk;
  wire soc_clk;

  wire sys_reset;
  assign sys_reset = 1'b0; // TODO: connect to reset button

  wire fetch_en_i;
  assign fetch_en_i = 1'b0; // TODO: connect to switch 7


  assign sys_clk = sys_clk_p;
  // clkwiz i_clkwiz (
  //   .clk_in1  ( sys_clk ),
  //   .reset    ( '0 ),
  //   .locked   ( ),
  //   .clk_20   ( soc_clk )
  // );

  assign soc_clk = sys_clk;

  /////////////////////
  //  System Inputs  //
  /////////////////////

  // Select SoC reset
  logic sys_resetn;
  assign sys_resetn = ~sys_reset;


  // Tie off inputs of no switches
  // logic                 fetch_en_i;
  // logic [GpioCount-2:0] gpio_i;
  // assign test_mode_i = '0;
  // assign fetch_en_i  = '0;
  // assign gpio_i      = '0;

  // logic status_o;

  // logic [GpioCount-1:0] gpio_o;

  ////////////
  //  VIOs  //
  ////////////
  logic       vio_reset, vio_fetch_en, vio_gpio;

// `ifdef USE_VIO
//   vio i_vio (
//     .clk        ( soc_clk      ),
//     .probe_out0 ( vio_reset    ),
//     .probe_out1 ( vio_fetch_en ),
//     .probe_out2 ( vio_gpio     )
//   );
// `else
//   assign vio_reset    = '0;
//   assign vio_fetch_en = '0;
//   assign vio_gpio     = '0;
// `endif


  //////////////
  //  SOC IO  //
  //////////////

  logic  soc_fetch_en;
  logic  soc_rst_n;

  assign soc_fetch_en = fetch_en_i | vio_fetch_en;
  assign soc_rst      = ~sys_resetn | vio_reset;

  logic [GpioCount-1:0] soc_gpio_i;             
  logic [GpioCount-1:0] soc_gpio_o;            
  logic [GpioCount-1:0] soc_gpio_out_en_o;

  for(genvar idx=0; idx<GpioCount; idx++) begin
    assign gpio_o[idx] = soc_gpio_out_en_o[idx] ? soc_gpio_o[idx] : '0;

    if(idx == 0) begin
      assign soc_gpio_i[idx] = ~soc_gpio_out_en_o[idx] ? vio_gpio | gpio_i[0] : '0;
    end else begin
      assign soc_gpio_i[idx] = ~soc_gpio_out_en_o[idx] ? gpio_i[idx] : '0;
    end
  end


  //////////////////
  //  Reset Sync  //
  //////////////////

  wire rst_n;

  rstgen i_rstgen (
    .clk_i        ( soc_clk     ),
    .rst_ni       ( ~soc_rst    ),
    .test_mode_i  ( '0          ),
    .rst_no       ( rst_n       ),
    .init_no      ( )
  );

  ////////////
  //  JTAG  //
  ////////////

  logic jtag_trst_ni;
  assign jtag_trst_ni = 1'b1;


  /////////////////////////
  // "RTC" Clock Divider //
  /////////////////////////

  logic rtc_clk_d, rtc_clk_q;
  logic [15:0] counter_d, counter_q;

  // Divide soc_clk (20 MHz) by 610 => ~32.768kHz RTC Clock
  // TODO: does genesys 2 have a 32.768kHz reference clock?
  always_comb begin
    counter_d = counter_q + 1;
    rtc_clk_d = rtc_clk_q;

    if(counter_q == 610) begin
      counter_d = '0;
      rtc_clk_d = ~rtc_clk_q;
    end
  end

  always_ff @(posedge soc_clk, negedge rst_n) begin
    if(~rst_n) begin
      counter_q <= '0;
      rtc_clk_q <= 0;
    end else begin
      counter_q <= counter_d;
      rtc_clk_q <= rtc_clk_d;
    end
  end



  //////////////////
  // Cheshire SoC //
  //////////////////
  logic  soc_testmode_i;
  assign soc_testmode_i = '0;

  croc_soc #(
    .GpioCount( GpioCount )
  )
  i_croc_soc (
    .clk_i           ( soc_clk        ),
    .rst_ni          ( rst_n          ),
    .ref_clk_i       ( rtc_clk_q      ),
    .testmode_i      ( soc_testmode_i ),
    .fetch_en_i      ( soc_fetch_en   ),
    .status_o        ( status_o       ),

    .jtag_tck_i      ( jtag_tck_i   ),
    .jtag_tdi_i      ( jtag_tdi_i   ),
    .jtag_tdo_o      ( jtag_tdo_o   ),
    .jtag_tms_i      ( jtag_tms_i   ),
    .jtag_trst_ni    ( jtag_trst_ni ),

    .uart_rx_i       ( uart_rx_i ),
    .uart_tx_o       ( uart_tx_o ),

    .gpio_i          ( soc_gpio_i        ),             
    .gpio_o          ( soc_gpio_o        ),            
    .gpio_out_en_o   ( soc_gpio_out_en_o ) 
  );

endmodule
