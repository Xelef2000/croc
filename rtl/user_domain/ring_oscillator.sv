module ring_oscillator (
    output wire osc_out
);

`ifndef TARGET_ASIC
    // Simulatable version: use a toggle register driven by a clock
    reg dummy_osc = 0;
    reg dummy_clk = 0;

    reg [7:0] counter = 0;
    always @(posedge dummy_clk) begin
        counter <= counter + 1;
        if (counter == 8'd0)
            dummy_osc <= ~dummy_osc;
    end

   
    always #500 dummy_clk = ~dummy_clk; // 1MHz simulated clock

    assign osc_out = dummy_osc;

`else
    // Real ring oscillator (for synthesis/silicon)
    wire n1, n2, n3, n4, n5;

    assign n1 = ~n5;
    assign n2 = ~n1;
    assign n3 = ~n2;
    assign n4 = ~n3;
    assign n5 = ~n4;

    assign osc_out = n3;
`endif

endmodule
