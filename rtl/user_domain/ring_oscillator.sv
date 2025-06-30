module ring_oscillator (
    output wire osc_out
);

    wire n1, n2, n3, n4, n5;

    // Inverter chain with odd number of stages (5)
    assign n1 = ~n5;
    assign n2 = ~n1;
    assign n3 = ~n2;
    assign n4 = ~n3;
    assign n5 = ~n4;

    // Output from one of the nodes
    assign osc_out = n3;

endmodule
