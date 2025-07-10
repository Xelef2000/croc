`include "common_cells/registers.svh"

module user_rom #(
    /// OBI configuration
    parameter obi_pkg::obi_cfg_t ObiCfg = obi_pkg::ObiDefaultConfig,
    parameter type obi_req_t = logic,
    parameter type obi_rsp_t = logic,
    /// Base address of this static register
    parameter logic [31:0] BaseAddr = 32'h2000_0000,
    /// Fixed 32-bit value to return on read
    parameter logic [31:0] FixedValue = 32'hDEADBEEF
) (
    input  logic clk_i,
    input  logic rst_ni,
    input  obi_req_t obi_req_i,
    output obi_rsp_t obi_rsp_o
);

    // Request latching
    logic req_d, req_q;
    logic we_d, we_q;
    logic [ObiCfg.IdWidth-1:0] id_d, id_q;

    // Response fields
    logic [ObiCfg.DataWidth-1:0] rsp_data;
    logic rsp_err;

    // Address in range decode
    logic addr_match;
    assign addr_match = (obi_req_i.a.addr == BaseAddr);

    // Request capture
    assign req_d = obi_req_i.req && addr_match;
    assign we_d  = obi_req_i.a.we;
    assign id_d  = obi_req_i.a.aid;

    // Response data logic
    always_comb begin
        rsp_data = '0;
        rsp_err  = '0;
        if (req_q) begin
            if (we_q) begin
                rsp_data = '0;
                rsp_err  = 1'b0; // Optional: set to 1 if writes are illegal
            end else begin
                rsp_data = FixedValue;
                rsp_err  = 1'b0;
            end
        end
    end

    // Sequential state update
    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            req_q <= '0;
            we_q  <= '0;
            id_q  <= '0;
        end else begin
            req_q <= req_d;
            we_q  <= we_d;
            id_q  <= id_d;
        end
    end

    // OBI response wiring
    assign obi_rsp_o.gnt        = obi_req_i.req && addr_match;
    assign obi_rsp_o.rvalid     = req_q;
    assign obi_rsp_o.r.rdata    = rsp_data;
    assign obi_rsp_o.r.rid      = id_q;
    assign obi_rsp_o.r.err      = rsp_err;
    assign obi_rsp_o.r.r_optional = '0;

endmodule
