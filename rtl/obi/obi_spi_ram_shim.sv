`include "common_cells/registers.svh"

module obi_spi_ram_shim #(
    /// The OBI configuration for all ports.
    parameter obi_pkg::obi_cfg_t ObiCfg = obi_pkg::ObiDefaultConfig,
    /// The request struct.
    parameter type obi_req_t = logic,
    /// The response struct.
    parameter type obi_rsp_t = logic,
    /// Base address of the RAM
    parameter logic [31:0] BaseAddr = 32'h1000_0000,
    /// Size of RAM address range in bytes
    parameter logic [31:0] SpiRamMaxSize = 32'h800
) (
    /// Clock
    input logic clk_i,
    /// Active-low reset
    input logic rst_ni,
    /// OBI request interface
    input obi_req_t obi_req_i,
    /// OBI response interface
    output obi_rsp_t obi_rsp_o,
    
    output [31:0] spi_address_o, // Address to SPI flash
    output [31:0] spi_data_o, // Data to SPI flash
    output logic spi_cs_n_o, // Chip select
    output [2:0] spi_md_o, // spi mode
    output logic spi_we_o,  // Write enable to SPI module
    output logic spi_cfg_o // SPI chip configuration mode

    
    input logic spi_rsp_i, // SPI response
    input [31:0] spi_data_i, // Data from SPI flash

    output logic clk_cfg_o, // Clock configuration output
    output [4:0] clk_div_hi_o, // Clock divider high cycles
    output [4:0] clk_div_lo_o, // Clock divider low cycles
);

    // FSM states
    typedef enum logic [2:0] {
        BASE      = 3'b000,
        CLK_CFG   = 3'b001,
        SPI_REQ   = 3'b010,
        SPI_RESP  = 3'b011,
        SPI_ERROR = 3'b100,
        SPI_CFG   = 3'b101
    } fsm_state_e;

    fsm_state_e state_d, state_q;

    // Configuration registers
    logic [31:0] spi_ram_size_q, spi_ram_size_d;
    logic [4:0] clk_div_hi_q, clk_div_hi_d;
    logic [4:0] clk_div_lo_q, clk_div_lo_d;
    logic [2:0] spi_mode_q, spi_mode_d;
    logic [9:0] timeout_cycles_q, timeout_cycles_d;

    // Request registers
    logic req_d, req_q;
    logic we_d, we_q;
    logic [ObiCfg.AddrWidth-1:0] addr_d, addr_q;
    logic [ObiCfg.IdWidth-1:0] id_d, id_q;
    logic [ObiCfg.DataWidth-1:0] data_d, data_q;

    // Response registers
    logic [ObiCfg.DataWidth-1:0] rsp_data_d, rsp_data_q;
    logic rsp_err_d, rsp_err_q;
    logic rsp_valid_d, rsp_valid_q;

    // Pipelined SPI response signals
    logic spi_rsp_d, spi_rsp_q;
    logic [31:0] spi_data_d, spi_data_q;

    // Timeout counter
    logic [15:0] timeout_counter_d, timeout_counter_q;
    logic [15:0] timeout_limit;

    // Address decoding
    logic memory_request, config_request, spi_cfg_request, addr_in_range;
    logic config_mem_size, config_params;

    assign memory_request = (obi_req_i.a.addr >= BaseAddr) && 
                           (obi_req_i.a.addr < BaseAddr + SpiRamMaxSize);
    
    assign config_request = (obi_req_i.a.addr >= BaseAddr + SpiRamMaxSize) && 
                           (obi_req_i.a.addr < BaseAddr + SpiRamMaxSize + 8);
    
    assign spi_cfg_request = (obi_req_i.a.addr >= BaseAddr + SpiRamMaxSize + 8) && 
                            (obi_req_i.a.addr < BaseAddr + SpiRamMaxSize + 12);
    
    assign addr_in_range = memory_request || config_request || spi_cfg_request;
    
    assign config_mem_size = config_request && (obi_req_i.a.addr == BaseAddr + SpiRamMaxSize);
    assign config_params = config_request && (obi_req_i.a.addr == BaseAddr + SpiRamMaxSize + 4);

    // SPI response signals
    assign spi_rsp_d = spi_rsp_i;
    assign spi_data_d = spi_data_i;

    // Timeout calculation
    assign timeout_limit = (clk_div_hi_q + clk_div_lo_q) * timeout_cycles_q;

    // Sample request when granted
    assign req_d = obi_req_i.req && addr_in_range && (state_q == BASE);
    assign id_d = obi_req_i.a.aid;
    assign we_d = obi_req_i.a.we;
    assign addr_d = obi_req_i.a.addr;
    assign data_d = obi_req_i.a.wdata;

    // FSM logic
    always_comb begin
        // Default assignments
        state_d = state_q;
        spi_ram_size_d = spi_ram_size_q;
        clk_div_hi_d = clk_div_hi_q;
        clk_div_lo_d = clk_div_lo_q;
        spi_mode_d = spi_mode_q;
        timeout_cycles_d = timeout_cycles_q;
        timeout_counter_d = timeout_counter_q;
        rsp_data_d = rsp_data_q;
        rsp_err_d = rsp_err_q;
        rsp_valid_d = rsp_valid_q;

        case (state_q)
            BASE: begin
                rsp_valid_d = 1'b0;
                if (req_q) begin
                    if (config_request) begin
                        if (we_q) begin
                            // Handle configuration writes
                            if (config_mem_size) begin
                                spi_ram_size_d = data_q;
                            end else if (config_params) begin
                                clk_div_hi_d = data_q[4:0];
                                clk_div_lo_d = data_q[9:5];
                                spi_mode_d = data_q[12:10];
                                timeout_cycles_d = data_q[22:13];
                                
                                // If clock config changed, go to CLK_CFG
                                if (clk_div_hi_d != clk_div_hi_q || clk_div_lo_d != clk_div_lo_q) begin
                                    state_d = CLK_CFG;
                                end
                            end
                        end else begin
                            // Handle configuration reads
                            if (config_mem_size) begin
                                rsp_data_d = spi_ram_size_q;
                            end else if (config_params) begin
                                rsp_data_d = {9'b0, timeout_cycles_q, spi_mode_q, clk_div_lo_q, clk_div_hi_q};
                            end
                            rsp_valid_d = 1'b1;
                            rsp_err_d = 1'b0;
                        end
                    end else if (spi_cfg_request) begin
                        if (we_q) begin
                            // Start SPI chip configuration
                            state_d = SPI_CFG;
                            timeout_counter_d = '0;
                        end else begin
                            // SPI chip config reads not supported
                            rsp_valid_d = 1'b1;
                            rsp_err_d = 1'b1;
                            rsp_data_d = '0;
                        end
                    end else if (memory_request) begin
                        // Start SPI transaction
                        state_d = SPI_REQ;
                        timeout_counter_d = '0;
                    end
                end
            end

            CLK_CFG: begin
                // Return to base after one cycle
                state_d = BASE;
                rsp_valid_d = 1'b1;
                rsp_err_d = 1'b0;
                rsp_data_d = '0;
            end

            SPI_REQ: begin
                timeout_counter_d = timeout_counter_q + 1;
                
                if (spi_rsp_q) begin
                    state_d = SPI_RESP;
                    // Capture read data immediately when spi_rsp_q goes high
                    if (!we_q) begin
                        rsp_data_d = spi_data_q;
                    end else begin
                        rsp_data_d = '0;
                    end
                    rsp_err_d = 1'b0;
                end else if (timeout_counter_q >= timeout_limit && timeout_limit != 0) begin
                    state_d = SPI_ERROR;
                end
            end

            SPI_RESP: begin
                rsp_valid_d = 1'b1;
                if (!spi_rsp_q) begin
                    state_d = BASE;
                end
            end

            SPI_CFG: begin
                timeout_counter_d = timeout_counter_q + 1;
                
                if (spi_rsp_q) begin
                    state_d = SPI_RESP;
                    rsp_data_d = '0; // No data return for config operations
                    rsp_err_d = 1'b0;
                end else if (timeout_counter_q >= timeout_limit && timeout_limit != 0) begin
                    state_d = SPI_ERROR;
                end
            end

            SPI_ERROR: begin
                rsp_valid_d = 1'b1;
                rsp_err_d = 1'b1;
                rsp_data_d = '0;
                state_d = BASE;
            end

            default: begin
                state_d = BASE;
            end
        endcase
    end

    // Sequential logic
    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            state_q <= BASE;
            req_q <= '0;
            id_q <= '0;
            we_q <= '0;
            addr_q <= '0;
            data_q <= '0;
            rsp_data_q <= '0;
            rsp_err_q <= '0;
            rsp_valid_q <= '0;
            timeout_counter_q <= '0;
            spi_rsp_q <= '0;
            spi_data_q <= '0;
            // Initialize configuration registers with safe defaults
            spi_ram_size_q <= SpiRamMaxSize;
            clk_div_hi_q <= 5'd1;
            clk_div_lo_q <= 5'd1;
            spi_mode_q <= 3'd0; // Standard SPI mode
            timeout_cycles_q <= 10'd100;
        end else begin
            state_q <= state_d;
            req_q <= req_d;
            id_q <= id_d;
            we_q <= we_d;
            addr_q <= addr_d;
            data_q <= data_d;
            rsp_data_q <= rsp_data_d;
            rsp_err_q <= rsp_err_d;
            rsp_valid_q <= rsp_valid_d;
            timeout_counter_q <= timeout_counter_d;
            spi_ram_size_q <= spi_ram_size_d;
            clk_div_hi_q <= clk_div_hi_d;
            clk_div_lo_q <= clk_div_lo_d;
            spi_mode_q <= spi_mode_d;
            timeout_cycles_q <= timeout_cycles_d;
            spi_rsp_q <= spi_rsp_d;
            spi_data_q <= spi_data_d;
        end
    end

    // Output assignments
    // OBI response
    assign obi_rsp_o.gnt = obi_req_i.req && addr_in_range && (state_q == BASE);
    assign obi_rsp_o.rvalid = rsp_valid_q;
    assign obi_rsp_o.r.rdata = rsp_data_q;
    assign obi_rsp_o.r.rid = id_q;
    assign obi_rsp_o.r.err = rsp_err_q;
    assign obi_rsp_o.r.r_optional = '0;

    // SPI interface
    assign spi_address_o = (state_q == SPI_REQ) ? (addr_q - BaseAddr) : 
                          (state_q == SPI_CFG) ? {24'b0, data_q[7:0]} : '0; // Opcode in address for config
    assign spi_data_o = (state_q == SPI_REQ) ? data_q : 
                       (state_q == SPI_CFG) ? {8'b0, data_q[31:8]} : '0; // Config data in upper 24 bits
    assign spi_cs_n_o = (state_q == SPI_REQ || state_q == SPI_CFG) ? 1'b0 : 1'b1; // Active low chip select
    assign spi_md_o = (state_q == SPI_REQ || state_q == SPI_CFG) ? spi_mode_q : '0;
    assign spi_we_o = (state_q == SPI_REQ) ? we_q : (state_q == SPI_CFG) ? 1'b1 : 1'b0; // Config is always write

    // Clock configuration interface
    assign clk_cfg_o = (state_q == CLK_CFG);
    assign clk_div_hi_o = clk_div_hi_q;
    assign clk_div_lo_o = clk_div_lo_q;
    
    // SPI chip configuration interface
    assign spi_cfg_o = (state_q == SPI_CFG);

endmodule