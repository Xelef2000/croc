// Copyright 2023 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
// gives us the `FF(...) macro making it easy to have properly defined flip-flops
`include "common_cells/registers.svh"
// simple ROM
module user_rom #(
/// The OBI configuration for all ports.
parameter obi_pkg::obi_cfg_t ObiCfg = obi_pkg::ObiDefaultConfig,
/// The request struct.
parameter type obi_req_t = logic,
/// The response struct.
parameter type obi_rsp_t = logic
) (
/// Clock
input logic clk_i,
/// Active-low reset
input logic rst_ni,
/// OBI request interface
input obi_req_t obi_req_i,
/// OBI response interface
output obi_rsp_t obi_rsp_o
);

// Define some registers to hold the requests fields
logic req_d, req_q, req_q2; // Request valid
logic we_d, we_q, we_q2; // Write enable
logic [ObiCfg.AddrWidth-1:0] addr_d, addr_q, addr_q2; // Internal address of the word to read
logic [ObiCfg.IdWidth-1:0] id_d, id_q, id_q2; // Id of the request, must be same for the response

// Signals used to create the response
logic [ObiCfg.DataWidth-1:0] rsp_data; // Data field of the obi response
logic rsp_err; // Error field of the obi response

// Wire the registers holding the request
// TODO 1 : Modify the code such that the ROM will respond after 2 cycles instead of 1
assign req_d = obi_req_i.req;
assign id_d = obi_req_i.a.aid;
assign we_d = obi_req_i.a.we;
assign addr_d = obi_req_i.a.addr;

always_ff @(posedge (clk_i) or negedge (rst_ni)) begin
  if (!rst_ni) begin
    req_q <= '0;
    id_q <= '0;
    we_q <= '0;
    addr_q <= '0;
    req_q2 <= '0;
    id_q2 <= '0;
    we_q2 <= '0;
    addr_q2 <= '0;
  end else begin
    req_q <= req_d;
    id_q <= id_d;
    we_q <= we_d;
    addr_q <= addr_d;
    req_q2 <= req_q;
    id_q2 <= id_q;
    we_q2 <= we_q;
    addr_q2 <= addr_q;
  end
end

// Assign the response data
// TODO 2 : Modify the code such that the ROM will contain (up to) 32 ASCII chars
// hold in your initials in the form: "JD&JD's ASIC\0"
logic [2:0] word_addr;
always_comb begin
  rsp_data = '0;
  rsp_err = '0;
  word_addr = addr_q2[4:2]; // Changed to support 8 words (0-7)
  
  if(req_q2) begin
    if(~we_q2) begin
      case(word_addr)
        // Example with initials "AB" -> "AB&AB's ASIC\0"
        // Replace with your actual initials
        3'h0: rsp_data = 32'h41422641; // "AB&A"
        3'h1: rsp_data = 32'h42277320; // "B's "
        3'h2: rsp_data = 32'h41534943; // "ASIC"
        3'h3: rsp_data = 32'h00000000; // "\0\0\0\0"
        3'h4: rsp_data = 32'h00000000; // padding
        3'h5: rsp_data = 32'h00000000; // padding
        3'h6: rsp_data = 32'h00000000; // padding
        3'h7: rsp_data = 32'h00000000; // padding
        default: rsp_data = 32'h0;
      endcase
    end else begin
      rsp_err = '1;
    end
  end
end

// Wire the response
// A channel
assign obi_rsp_o.gnt = obi_req_i.req;
// R channel:
assign obi_rsp_o.rvalid = req_q2;
assign obi_rsp_o.r.rdata = rsp_data;
assign obi_rsp_o.r.rid = id_q2;
assign obi_rsp_o.r.err = rsp_err;
assign obi_rsp_o.r.r_optional = '0;

endmodule