module metastable_trng (
    input  logic clk,
    input  logic reset,
    output logic random_bit,
    output logic valid
);

    // A free-running toggle flip-flop (asynchronous, non-coherent input)
    logic async_noise;
    always_ff @(posedge clk) begin
        async_noise <= ~async_noise; // simple oscillator
    end

    // Two flip-flops for metastability sampling
    logic meta_ff1, meta_ff2;
    always_ff @(posedge clk) begin
        meta_ff1 <= async_noise;
        meta_ff2 <= meta_ff1;
    end

    // Von Neumann debiasing
    logic prev_sample;
    logic [1:0] pair;
    
    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            prev_sample <= '0;
            random_bit  <= '0;
            valid       <= '0;
            pair        <= '0;
        end else begin
            // Shift in sampled bits
            pair <= {pair[0], meta_ff2};
            
            // Apply von Neumann: output if pair == 01 or 10
            valid <= '0;
            if (pair == 2'b01) begin
                random_bit <= '1;
                valid      <= '1;
            end else if (pair == 2'b10) begin
                random_bit <= '0;
                valid      <= '1;
            end
        end
    end

endmodule