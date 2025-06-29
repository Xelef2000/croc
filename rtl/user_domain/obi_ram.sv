`include "common_cells/registers.svh"

module obi_ram #(
    /// The OBI configuration for all ports.
    parameter obi_pkg::obi_cfg_t ObiCfg = obi_pkg::ObiDefaultConfig,
    /// The request struct.
    parameter type obi_req_t = logic,
    /// The response struct.
    parameter type obi_rsp_t = logic,
    /// Base address of the ROM
    parameter logic [31:0] BaseAddr = 32'h1000_0000,
    /// Size of ROM address range in bytes
    parameter logic [31:0] Size = 32'h800
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
    logic req_d, req_q; // Request valid (added req_qq for two-cycle delay for SPI response)
    logic we_d, we_q;    // Write enable
    logic [ObiCfg.AddrWidth-1:0] addr_d, addr_q; // Internal address of the word to read
    logic [ObiCfg.IdWidth-1:0] id_d, id_q; // Id of the request, must be same for the response
    logic [ObiCfg.DataWidth-1:0] data_d, data_q; // Data to be written (for write requests)
    logic [ObiCfg.DataWidth-1:0] random_number, resp_data_d, resp_data_q; // Data to be returned in response

    // Check if address is in range
    logic addr_in_range;
    assign addr_in_range = (obi_req_i.a.addr >= BaseAddr) && 
                           (obi_req_i.a.addr < (BaseAddr + Size));

    

    // Wire the registers holding the request - only when address is in range
    assign req_d = obi_req_i.req && addr_in_range;
    assign id_d = obi_req_i.a.aid;
    assign we_d = obi_req_i.a.we;
    assign addr_d = obi_req_i.a.addr;
    assign data_d = obi_req_i.a.wdata;
    assign resp_data_d = random_number; 

    prand #(
        .Seed(32'h1A2B3C4D) 
    ) i_prand (
        .clk_i(clk_i),
        .rst_ni(rst_ni),
        .random_number_o(random_number)
    );



    always_comb begin
        rsp_data = '0;
        rsp_err = '0;
        
        if(req_q) begin
            if(we_q) begin
                // Write request
                rsp_data = '0; // No data to return on write
                rsp_err = '0; // No error
            end else begin
                // Read request
                rsp_data = resp_data_q;
                rsp_err = '0; // No error
            end
        end
    end



    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            req_q <= '0;
            id_q <= '0;
            we_q <= '0;
            addr_q <= '0;
            data_q <= '0;
            resp_data_q <= '0;
        end else begin
            req_q <= req_d;
            id_q <= id_d;
            we_q <= we_d;
            addr_q <= addr_d;
            data_q <= data_d;
            resp_data_q <= resp_data_d;
        end
    end

    // Signals used to create the response
    logic [ObiCfg.DataWidth-1:0] rsp_data; // Data field of the obi response
    logic rsp_err; // Error field of the obi response


    
    // Wire the response
    // A channel
    assign obi_rsp_o.gnt = obi_req_i.req && addr_in_range;
    // R channel:
    assign obi_rsp_o.rvalid = req_q;
    assign obi_rsp_o.r.rdata = rsp_data;
    assign obi_rsp_o.r.rid = id_q;
    assign obi_rsp_o.r.err = rsp_err;
    assign obi_rsp_o.r.r_optional = '0;
endmodule