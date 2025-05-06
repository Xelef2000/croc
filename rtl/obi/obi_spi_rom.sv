`include "common_cells/registers.svh"

module obi_spi_rom #(
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
    output obi_rsp_t obi_rsp_o,
    
    // /// SPI Interface signals (would be connected to actual SPI flash)
    // output logic spi_cs_n_o,    // Chip select (active low)
    // output logic spi_sck_o,     // Serial clock
    // output logic spi_mosi_o,    // Master out slave in data
    // input  logic spi_miso_i     // Master in slave out data
);
    // Define some registers to hold the requests fields
    logic req_d, req_q, req_qq; // Request valid (added req_qq for two-cycle delay for SPI response)
    logic we_d, we_q, we_qq;    // Write enable
    logic [ObiCfg.AddrWidth-1:0] addr_d, addr_q, addr_qq, spi_address; // Internal address of the word to read
    logic [ObiCfg.IdWidth-1:0] id_d, id_q, id_qq; // Id of the request, must be same for the response
    
    // Check if address is in range
    logic addr_in_range;
    assign addr_in_range = (obi_req_i.a.addr >= BaseAddr) && 
                           (obi_req_i.a.addr < (BaseAddr + Size));

    // Wire the registers holding the request - only when address is in range
    assign req_d = obi_req_i.req && addr_in_range;
    assign id_d = obi_req_i.a.aid;
    assign we_d = obi_req_i.a.we;
    assign addr_d = obi_req_i.a.addr;

    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            req_q <= '0;
            req_qq <= '0;
            id_q <= '0;
            id_qq <= '0;
            we_q <= '0;
            we_qq <= '0;
            addr_q <= '0;
            addr_qq <= '0;
        end else begin
            req_q <= req_d;
            req_qq <= req_q;
            id_q <= id_d;
            id_qq <= id_q;
            we_q <= we_d;
            we_qq <= we_q;
            addr_q <= addr_d;
            addr_qq <= addr_q;
        end
    end

    // Signals used to create the response
    logic [ObiCfg.DataWidth-1:0] rsp_data; // Data field of the obi response
    logic rsp_err; // Error field of the obi response


    // For development - just return all 1s for reads and error for writes
    always_comb begin
        rsp_data = '0;
        rsp_err = '0;
        spi_address = addr_q - BaseAddr;
        
        if(req_q) begin
            if(~we_q) begin
                case(spi_address)
                    32'h0: rsp_data = 32'h4647_4E20;
                    32'h4: rsp_data = 32'h5241_5320;
                    32'h8: rsp_data = 32'h4153_4943;
                    32'hC: rsp_data = 32'h4647_4E20;
                    default: rsp_data = spi_address;
                endcase
            end else begin
                rsp_err = '1;
            end
        end
    end

    // SPI signals - for now just stub these out
    // In a real implementation, these would drive the SPI flash memory
    assign spi_cs_n_o = 1'b1;  // Chip select inactive by default
    assign spi_sck_o = 1'b0;   // Clock low by default
    assign spi_mosi_o = 1'b0;  // Data line low by default
    
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