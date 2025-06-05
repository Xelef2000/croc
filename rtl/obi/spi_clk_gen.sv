module spi_clk_gen #(
    /// Initial divider value for high cycles
    parameter logic [4:0] InitialDivHigh = 5'd1,
    /// Initial divider value for low cycles  
    parameter logic [4:0] InitialDivLow = 5'd1
) (
    /// Clock
    input logic soc_clk_i,
    /// Active-low reset
    input logic rst_ni,
    /// Clock configuration pulse (held for exactly one soc_clk cycle)
    input logic clk_cfg_i,
    /// Clock divider high cycles configuration
    input logic [4:0] clk_div_hi_i,
    /// Clock divider low cycles configuration
    input logic [4:0] clk_div_lo_i,
    /// Generated SPI clock output
    output logic spi_clk_o
);

    // Internal registers to hold current divider configuration
    logic [4:0] div_high_q, div_high_d;
    logic [4:0] div_low_q, div_low_d; 
    
    // Counter for high/low periods
    logic [4:0] counter_q, counter_d;
    
    // SPI clock state register
    logic spi_clk_q, spi_clk_d;
    
    logic clk_state_q, clk_state_d; // 1 = high period, 0 = low period
    
    always_comb begin
        div_high_d = div_high_q;
        div_low_d = div_low_q;
        
        if (clk_cfg_i) begin
            div_high_d = clk_div_hi_i;
            div_low_d = clk_div_lo_i;
        end
    end
    
    // Clock generation logic
    always_comb begin
        counter_d = counter_q;
        clk_state_d = clk_state_q;
        spi_clk_d = spi_clk_q;
        
        // If either divider is 0, SPI clock equals SOC clock
        if (div_high_q == 5'd0 || div_low_q == 5'd0) begin
            spi_clk_d = soc_clk_i;
            counter_d = 5'd0;
            clk_state_d = 1'b0;
        end else begin
            if (clk_state_q) begin
                // Currently in high period
                if (counter_q >= (div_high_q - 1)) begin
                    spi_clk_d = 1'b0;
                    clk_state_d = 1'b0;
                    counter_d = 5'd0;
                end else begin
                    spi_clk_d = 1'b1;
                    counter_d = counter_q + 1;
                end
            end else begin
                // Currently in low period
                if (counter_q >= (div_low_q - 1)) begin
                    spi_clk_d = 1'b1;
                    clk_state_d = 1'b1;
                    counter_d = 5'd0;
                end else begin
                    spi_clk_d = 1'b0;
                    counter_d = counter_q + 1;
                end
            end
        end
        
        if (clk_cfg_i) begin
            counter_d = 5'd0;
            clk_state_d = 1'b1; 
            if (clk_div_hi_i == 5'd0 || clk_div_lo_i == 5'd0) begin
                spi_clk_d = soc_clk_i;
            end else begin
                spi_clk_d = 1'b1;
            end
        end
    end
    
    always_ff @(posedge soc_clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            div_high_q <= InitialDivHigh;
            div_low_q <= InitialDivLow;
            counter_q <= 5'd0;
            spi_clk_q <= 1'b0;
            clk_state_q <= 1'b1; // Start with high period
        end else begin
            div_high_q <= div_high_d;
            div_low_q <= div_low_d;
            counter_q <= counter_d;
            spi_clk_q <= spi_clk_d;
            clk_state_q <= clk_state_d;
        end
    end
    
    assign spi_clk_o = spi_clk_q;

endmodule