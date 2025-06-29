module prand #(
    /// Seed value for the pseudo-random number generator
    parameter logic [31:0] Seed = 32'h800
) (
    /// Clock
    input logic clk_i,
    /// Active-low reset
    input logic rst_ni,
    /// Set seed control signal
    input logic set_seed_i,
    /// Seed input value
    input logic [31:0] seed_i,
    // random number output
    output logic [31:0] random_number_o
);

    // LFSR register and control signals
    logic [30:0] lfsr_d, lfsr_q;  // 31-bit LFSR register
    logic lfsr_feedback;          // Feedback bit for LFSR
    logic [7:0] filter_bits_d, filter_bits_q;  // Bits for non-linear filter (pipelined)
    logic [31:0] accumulator_d, accumulator_q;  // Accumulate filtered bits
    logic [4:0] bit_counter_d, bit_counter_q;   // Counter for 32 bits
    logic output_ready_d, output_ready_q;      // Output ready flag
    logic [31:0] random_output_d, random_output_q; // Final random output

    // LFSR tap positions for 31-bit maximum length sequence
    // Primitive polynomial: x^31 + x^28 + 1
    // Taps at positions 31 and 28 (1-indexed: 31 and 28, 0-indexed: 30 and 27)
    assign lfsr_feedback = lfsr_q[30] ^ lfsr_q[27];

    // Non-linear filter function
    // Takes 8 bits from different positions in LFSR and applies non-linear function
    logic [7:0] selected_bits;
    logic filtered_bit;
    
    always_comb begin
        // Select 8 bits from different positions for good distribution
        // Use more spread out positions to get better mixing
        selected_bits[0] = lfsr_q[1];
        selected_bits[1] = lfsr_q[5];
        selected_bits[2] = lfsr_q[9];
        selected_bits[3] = lfsr_q[13];
        selected_bits[4] = lfsr_q[17];
        selected_bits[5] = lfsr_q[21];
        selected_bits[6] = lfsr_q[25];
        selected_bits[7] = lfsr_q[29];
        
        // Store selected bits for next stage
        filter_bits_d = selected_bits;
    end

    // Non-linear filter function (pipelined in next stage)
    always_comb begin
        // Enhanced non-linear boolean function for better randomness
        logic term1, term2, term3, term4, term5, term6;
        logic linear_term1, linear_term2;
        
        // Non-linear terms (products of bits)
        term1 = filter_bits_q[0] & filter_bits_q[1];
        term2 = filter_bits_q[2] & filter_bits_q[3];
        term3 = filter_bits_q[4] & filter_bits_q[5];
        term4 = filter_bits_q[6] & filter_bits_q[7];
        term5 = filter_bits_q[0] & filter_bits_q[3] & filter_bits_q[6];
        term6 = filter_bits_q[1] & filter_bits_q[4] & filter_bits_q[7];
        
        // Linear mixing terms
        linear_term1 = filter_bits_q[0] ^ filter_bits_q[2] ^ filter_bits_q[4] ^ filter_bits_q[6];
        linear_term2 = filter_bits_q[1] ^ filter_bits_q[3] ^ filter_bits_q[5] ^ filter_bits_q[7];
        
        // Combine all terms for better non-linearity
        filtered_bit = term1 ^ term2 ^ term3 ^ term4 ^ term5 ^ term6 ^ linear_term1 ^ linear_term2;
    end

    // Control logic for bit accumulation and output generation
    always_comb begin
        // Default assignments
        lfsr_d = lfsr_q;
        accumulator_d = accumulator_q;
        bit_counter_d = bit_counter_q;
        output_ready_d = output_ready_q;
        random_output_d = random_output_q;

        // Check if seed should be loaded
        if (set_seed_i) begin
            // Load new seed (ensure non-zero)
            lfsr_d = (seed_i[30:0] == 31'b0) ? 31'h1 : seed_i[30:0];
            // Reset accumulator and counter when new seed is set
            accumulator_d = '0;
            bit_counter_d = 5'd0;
            output_ready_d = 1'b0;
        end else begin
            // Normal operation - always shift LFSR (shift left, insert feedback at LSB)
            lfsr_d = {lfsr_q[29:0], lfsr_feedback};

            // Accumulate filtered bits
            if (bit_counter_q < 5'd31) begin
                // Shift in new filtered bit
                accumulator_d = {accumulator_q[30:0], filtered_bit};
                bit_counter_d = bit_counter_q + 1'b1;
                output_ready_d = 1'b0;
            end else begin
                // 32 bits accumulated, output is ready
                accumulator_d = {accumulator_q[30:0], filtered_bit};
                random_output_d = {accumulator_q[30:0], filtered_bit};
                bit_counter_d = 5'd0;  // Reset counter
                output_ready_d = 1'b1;
            end
        end
    end

    // Sequential logic
    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            // Initialize with seed (ensure non-zero)
            lfsr_q <= (Seed[30:0] == 31'b0) ? 31'h1 : Seed[30:0];
            filter_bits_q <= '0;
            accumulator_q <= '0;
            bit_counter_q <= '0;
            output_ready_q <= '0;
            random_output_q <= '0;
        end else begin
            lfsr_q <= lfsr_d;
            filter_bits_q <= filter_bits_d;
            accumulator_q <= accumulator_d;
            bit_counter_q <= bit_counter_d;
            output_ready_q <= output_ready_d;
            random_output_q <= random_output_d;
        end
    end

    // Output assignment
    assign random_number_o = random_output_q;

endmodule