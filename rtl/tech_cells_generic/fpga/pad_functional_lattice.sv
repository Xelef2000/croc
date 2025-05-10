// Copyright 2018 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License. You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.

// Converted for Lattice ECP5 FPGA

module pad_functional_pd (
    input  logic OEN,
    input  logic I,
    output logic O,
    input  logic PEN,
    inout  logic PAD
);
    // For ECP5, we use BB primitive for bidirectional buffers
    // and specify pull-down using the parameters
    wire T_b; // Inverted tri-state control (ECP5 convention)
    assign T_b = ~OEN;
    
    // ECP5 BB (bidirectional buffer) with pull down
    BB #(
        .PULLMODE("DOWN"),    // Enable pull-down resistor
        .IOSTANDARD("LVCMOS33"),
        .DRIVE("8")          // 8mA drive strength
    ) bb_pd_i (
        .B(PAD),             // Bidirectional pad
        .I(I),               // Input to pad
        .T(T_b),             // Tri-state control (active low in ECP5)
        .O(O)                // Output from pad
    );
    
    // Note: PEN (pull enable) is ignored as PULLMODE parameter is used instead

endmodule

module pad_functional_pu (
    input  logic OEN,
    input  logic I,
    output logic O,
    input  logic PEN,
    inout  logic PAD
);
    // For ECP5, we use BB primitive for bidirectional buffers
    // and specify pull-up using the parameters
    wire T_b; // Inverted tri-state control (ECP5 convention)
    assign T_b = ~OEN;
    
    // ECP5 BB (bidirectional buffer) with pull up
    BB #(
        .PULLMODE("UP"),      // Enable pull-up resistor
        .IOSTANDARD("LVCMOS33"),
        .DRIVE("8")          // 8mA drive strength
    ) bb_pu_i (
        .B(PAD),             // Bidirectional pad
        .I(I),               // Input to pad
        .T(T_b),             // Tri-state control (active low in ECP5)
        .O(O)                // Output from pad
    );
    
    // Note: PEN (pull enable) is ignored as PULLMODE parameter is used instead

endmodule

module pad_functional_configurable (
    input  logic OEN,
    input  logic I,
    output logic O,
    input  logic PEN,      // 0: pullup, 1: pulldown
    inout  logic PAD
);
    // For conditional instantiation based on PEN
    // This uses generate blocks to select between pull-up and pull-down
    
    wire T_b; // Inverted tri-state control (ECP5 convention)
    assign T_b = ~OEN;
    
    // Use generate to conditionally instantiate based on PEN
    // Note: In actual hardware, this would be determined at synthesis time
    // You would need to set PEN as a parameter rather than an input
    
    // Use parameter to select pull type
    parameter bit PULL_TYPE = 0; // 0: pullup, 1: pulldown
    
    generate
        if (PULL_TYPE == 0) begin : g_pullup
            BB #(
                .PULLMODE("UP"),
                .IOSTANDARD("LVCMOS33"),
                .DRIVE("8")
            ) bb_i (
                .B(PAD),
                .I(I),
                .T(T_b),
                .O(O)
            );
        end else begin : g_pulldown
            BB #(
                .PULLMODE("DOWN"),
                .IOSTANDARD("LVCMOS33"),
                .DRIVE("8")
            ) bb_i (
                .B(PAD),
                .I(I),
                .T(T_b),
                .O(O)
            );
        end
    endgenerate
    
endmodule