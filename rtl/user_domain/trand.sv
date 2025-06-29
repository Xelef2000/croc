module trand (
    /// Clock
    input logic clk_i,
    /// Active-low reset
    input logic rst_ni,
    input logic reset_rng_i,

    /// 32-bit random number output
    output logic [31:0] random_number_o
);

    // Internal signals for metastable TRNG
    logic trng_random_bit;
    logic trng_valid;
    
    // Bit accumulation registers and control signals
    logic [31:0] accumulator_d, accumulator_q;  // Accumulate random bits
    logic [4:0] bit_counter_d, bit_counter_q;   // Counter for 32 bits
    logic output_ready_d, output_ready_q;      // Output ready flag
    logic [31:0] random_output_d, random_output_q; // Final random output

    // Instantiate the metastable TRNG
    metastable_trng u_trng (
        .clk        (clk_i),
        .reset      (~rst_ni),  // Convert active-low to active-high reset
        .random_bit (trng_random_bit),
        .valid      (trng_valid)
    );

    // Control logic for bit accumulation and output generation
    always_comb begin
        // Default assignments
        accumulator_d = accumulator_q;
        bit_counter_d = bit_counter_q;
        output_ready_d = output_ready_q;
        random_output_d = random_output_q;

        // Check if seed should be loaded (reset accumulator for true RNG)
        if (reset_rng_i) begin
            // Reset accumulator and counter when seed control is asserted
            accumulator_d = '0;
            bit_counter_d = 5'd0;
            output_ready_d = 1'b0;
        end else begin
            // Normal operation - accumulate valid random bits
            if (trng_valid) begin
                if (bit_counter_q < 5'd31) begin
                    // Shift in new random bit
                    accumulator_d = {accumulator_q[30:0], trng_random_bit};
                    bit_counter_d = bit_counter_q + 1'b1;
                    output_ready_d = 1'b0;
                end else begin
                    // 32 bits accumulated, output is ready
                    accumulator_d = {accumulator_q[30:0], trng_random_bit};
                    random_output_d = {accumulator_q[30:0], trng_random_bit};
                    bit_counter_d = 5'd0;  // Reset counter for next 32-bit word
                    output_ready_d = 1'b1;
                end
            end
        end
    end

    // Sequential logic
    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            accumulator_q <= '0;
            bit_counter_q <= '0;
            output_ready_q <= '0;
            random_output_q <= '0;
        end else begin
            accumulator_q <= accumulator_d;
            bit_counter_q <= bit_counter_d;
            output_ready_q <= output_ready_d;
            random_output_q <= random_output_d;
        end
    end

    // Output assignment
    assign random_number_o = random_output_q;

endmodule