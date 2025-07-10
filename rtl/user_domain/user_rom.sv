`include "common_cells/registers.svh"
module user_rom #(
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
    logic req_d, req_q;
    logic we_d, we_q;
    logic [ObiCfg.AddrWidth-1:0] addr_d, addr_q;
    logic [ObiCfg.IdWidth-1:0] id_d, id_q;

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

    // Static ROM content: "Felix Niederer, Raphael Salzmann\0"
    // String is 33 characters + null terminator = 34 bytes
    // We'll store it as 32-bit words (little-endian)
    logic [31:0] rom_data [0:8]; // 9 words to hold 34 bytes (rounded up)

    // Initialize ROM with the string "Felix Niederer, Raphael Salzmann"
    initial begin
        // "Feli" (0x696c6546)
        rom_data[0] = 32'h696c6546;
        // "x Ni" (0x694e2078)
        rom_data[1] = 32'h694e2078;
        // "eder" (0x72656465)
        rom_data[2] = 32'h72656465;
        // "er, " (0x202c7265)
        rom_data[3] = 32'h202c7265;
        // "Raph" (0x68706152)
        rom_data[4] = 32'h68706152;
        // "ael " (0x206c6561)
        rom_data[5] = 32'h206c6561;
        // "Salz" (0x7a6c6153)
        rom_data[6] = 32'h7a6c6153;
        // "mann" (0x6e6e616d)
        rom_data[7] = 32'h6e6e616d;
        // "\0\0\0\0" (0x00000000) - null terminator and padding
        rom_data[8] = 32'h00000000;
    end

    // Signals used to create the response
    logic [ObiCfg.DataWidth-1:0] rsp_data;
    logic rsp_err;
    logic [31:0] word_addr;

    always_comb begin
        rsp_data = '0;
        rsp_err = '0;
        word_addr = (addr_q - BaseAddr) >> 2; // Convert byte address to word address

        if (req_q) begin
            if (we_q) begin
                // Write request - do nothing (ignore writes)
                rsp_data = '0;
                rsp_err = '0;
            end else begin
                // Read request
                if (word_addr < 9) begin
                    rsp_data = rom_data[word_addr];
                    rsp_err = '0;
                end else begin
                    // Address out of ROM range
                    rsp_data = '0;
                    rsp_err = '1;
                end
            end
        end
    end

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