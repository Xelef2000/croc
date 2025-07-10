`include "common_cells/registers.svh"

module obi_rand #(
    /// The OBI configuration for all ports.
    parameter obi_pkg::obi_cfg_t ObiCfg = obi_pkg::ObiDefaultConfig,
    /// The request struct.
    parameter type obi_req_t = logic,
    /// The response struct.
    parameter type obi_rsp_t = logic,
    /// Base address of the Module
    parameter logic [31:0] BaseAddr = 32'h2000_0000,
    /// Size of address range in bytes
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
    logic req_d, req_q; // Request valid
    logic we_d, we_q;    // Write enable
    logic [ObiCfg.AddrWidth-1:0] addr_d, addr_q; // Internal address of the word to read
    logic [ObiCfg.IdWidth-1:0] id_d, id_q; // Id of the request, must be same for the response
    logic [ObiCfg.DataWidth-1:0] data_d, data_q; // Data to be written (for write requests)
    logic [ObiCfg.DataWidth-1:0] random_number_0, random_number_1, random_number_2, resp_data_d, resp_data_q; // Data to be returned in response


     // Signals used to create the response
    logic [ObiCfg.DataWidth-1:0] rsp_data; // Data field of the obi response
    logic rsp_err; // Error field of the obi response

    // Seed control signals
    logic set_seed_d, set_seed_q;
    logic [31:0] seed_value_d, seed_value_q;
    
    // Reset RNG control signals
    logic reset_rng_d, reset_rng_q;
    
    // Address decode signals (combinatorial and pipelined)
    logic addr_is_prng0, addr_is_prng1, addr_is_trand;
    logic addr_is_prng0_d, addr_is_prng0_q;
    logic addr_is_prng1_d, addr_is_prng1_q;
    logic addr_is_trand_d, addr_is_trand_q;

    // Individual seed control signals for each PRNG
    logic set_seed_prng0, set_seed_prng1;

    // Check if address is in range
    logic addr_in_range;
    assign addr_in_range = (obi_req_i.a.addr >= BaseAddr) && 
                           (obi_req_i.a.addr < (BaseAddr + Size));

    // Address decode for the three RNG instances
    assign addr_is_prng0 = (obi_req_i.a.addr == BaseAddr);           // First 32 bits (offset 0x0)
    assign addr_is_prng1 = (obi_req_i.a.addr == (BaseAddr + 32'h4)); // Next 32 bits (offset 0x4)
    assign addr_is_trand = (obi_req_i.a.addr == (BaseAddr + 32'h8)); // Third 32 bits (offset 0x8)

    assign addr_is_prng0_d = addr_is_prng0;
    assign addr_is_prng1_d = addr_is_prng1;
    assign addr_is_trand_d = addr_is_trand;

    // Wire the registers holding the request - only when address is in range
    assign req_d = obi_req_i.req && addr_in_range;
    assign id_d = obi_req_i.a.aid;
    assign we_d = obi_req_i.a.we;
    assign addr_d = obi_req_i.a.addr;
    assign data_d = obi_req_i.a.wdata;

    // Response data selection based on address
    always_comb begin
        if (addr_is_prng0) begin
            resp_data_d = random_number_0;
        end else if (addr_is_prng1) begin
            resp_data_d = random_number_1;
        end else if (addr_is_trand) begin
            resp_data_d = random_number_2;
        end else begin
            resp_data_d = '0;  // Return 0 for other addresses
        end
    end

    // Seed control logic - set seed when write is detected to PRNG addresses (not trand)
    assign set_seed_d = req_d && we_d && (addr_is_prng0 || addr_is_prng1);
    assign seed_value_d = data_d;       // Use write data as seed value

    // Reset RNG control logic - reset when write is detected to trand address
    assign reset_rng_d = req_d && we_d && addr_is_trand;

    // Individual seed control for each PRNG
    assign set_seed_prng0 = set_seed_q && addr_is_prng0_q;
    assign set_seed_prng1 = set_seed_q && addr_is_prng1_q;

    // First PRNG instance
    prand #(
        .Seed(32'h1A2B3C4D) 
    ) i_prand_0 (
        .clk_i(clk_i),
        .rst_ni(rst_ni),
        .set_seed_i(set_seed_prng0),
        .seed_i(seed_value_q),
        .random_number_o(random_number_0)
    );

    // Second PRNG instance with different seed
    prand #(
        .Seed(32'h4D3C2B1A) 
    ) i_prand_1 (
        .clk_i(clk_i),
        .rst_ni(rst_ni),
        .set_seed_i(set_seed_prng1),
        .seed_i(seed_value_q),
        .random_number_o(random_number_1)
    );

    // Third RNG instance (trand)
    trand i_trand (
        .clk_i(clk_i),
        .rst_ni(rst_ni),
        .reset_rng_i(reset_rng_q),
        .random_number_o(random_number_2)
    );

    always_comb begin
        rsp_data = '0;
        rsp_err = '0;
        
        if(req_q) begin
            if(we_q) begin
                // Write request - seed was set or RNG was reset, return success
                rsp_data = '0; // No data to return on write
                rsp_err = '0; // No error
            end else begin
                // Read request - return random number
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
            set_seed_q <= '0;
            seed_value_q <= '0;
            reset_rng_q <= '0;
            addr_is_prng0_q <= '0;
            addr_is_prng1_q <= '0;
            addr_is_trand_q <= '0;
        end else begin
            req_q <= req_d;
            id_q <= id_d;
            we_q <= we_d;
            addr_q <= addr_d;
            data_q <= data_d;
            resp_data_q <= resp_data_d;
            set_seed_q <= set_seed_d;
            seed_value_q <= seed_value_d;
            reset_rng_q <= reset_rng_d;
            addr_is_prng0_q <= addr_is_prng0_d;
            addr_is_prng1_q <= addr_is_prng1_d;
            addr_is_trand_q <= addr_is_trand_d;
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