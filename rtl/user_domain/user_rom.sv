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
    logic req_d, req_q; // Request valid
    logic we_d, we_q; // Write enable
    logic [ObiCfg.AddrWidth-1:0] addr_d, addr_q; // Internal address of the word to read
    logic [ObiCfg.IdWidth-1:0] id_d, id_q; // Id of the request, must be same for the response
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
        end else begin
            req_q <= req_d;
            id_q <= id_d;
            we_q <= we_d;
            addr_q <= addr_d;
        end
    end
    // Assign the response data
    // TODO 2 : Modify the code such that the ROM will contain (up to) 32 ASCII chars
    // hold in your initials in the form: "JD&JD's ASIC\0"
    logic [3:0] word_addr;
    always_comb begin
        rsp_data = '0;
        rsp_err = '0;
        word_addr = addr_q[5:2];
        if(req_q) begin
            if(~we_q) begin
                case(word_addr)
                    // "Felix Niederer, Raphael Salzmann" + null terminator
                    // String: "Felix Niederer, Raphael Salzmann\0"
                    4'h0: rsp_data = 32'h696c6546; // "Feli"
                    4'h1: rsp_data = 32'h694e2078; // "x Ni"
                    4'h2: rsp_data = 32'h72656465; // "eder"
                    4'h3: rsp_data = 32'h202c7265; // "er, "
                    4'h4: rsp_data = 32'h68706152; // "Raph"
                    4'h5: rsp_data = 32'h206c6561; // "ael "
                    4'h6: rsp_data = 32'h7a6c6153; // "Salz"
                    4'h7: rsp_data = 32'h6e6e616d; // "mann"
                    4'h8: rsp_data = 32'h00000000; // "\0\0\0\0" - null terminator and padding
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
    assign obi_rsp_o.rvalid = req_q;
    assign obi_rsp_o.r.rdata = rsp_data;
    assign obi_rsp_o.r.rid = id_q;
    assign obi_rsp_o.r.err = rsp_err;
    assign obi_rsp_o.r.r_optional = '0;
endmodule