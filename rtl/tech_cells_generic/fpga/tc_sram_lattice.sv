// Copyright 2020 ETH Zurich and University of Bologna (Original Xilinx Version)
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License. You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.
//
// Original Author: Wolfgang Roenninger <wroennin@ethz.ch>, ETH Zurich
//
// Description: Lattice ECP5 implementation for `tc_sram` using EBR (Embedded Block RAM)
//              The behaviour, parameters and ports are described in the header of the original `rtl/tc_sram.sv`.

module tc_sram #(
  parameter int unsigned NumWords     = 32'd1024, // Number of Words in data array
  parameter int unsigned DataWidth    = 32'd128,  // Data signal width (in bits)
  parameter int unsigned ByteWidth    = 32'd8,    // Width of a data byte (in bits)
  parameter int unsigned NumPorts     = 32'd2,    // Number of read and write ports
  parameter int unsigned Latency      = 32'd1,    // Latency when the read data is available
  parameter              SimInit      = "zeros",  // Simulation initialization
  parameter bit          PrintSimCfg  = 1'b0,     // Print configuration
  parameter              ImplKey      = "none",   // Reference to specific implementation
  // DEPENDENT PARAMETERS, DO NOT OVERWRITE!
  parameter int unsigned AddrWidth = (NumWords > 32'd1) ? $clog2(NumWords) : 32'd1,
  parameter int unsigned BeWidth   = (DataWidth + ByteWidth - 32'd1) / ByteWidth, // ceil_div
  parameter type         addr_t    = logic [AddrWidth-1:0],
  parameter type         data_t    = logic [DataWidth-1:0],
  parameter type         be_t      = logic [BeWidth-1:0]
) (
  input  logic                clk_i,      // Clock
  input  logic                rst_ni,     // Asynchronous reset active low
  // input ports
  input  logic  [NumPorts-1:0] req_i,      // request
  input  logic  [NumPorts-1:0] we_i,       // write enable
  input  addr_t [NumPorts-1:0] addr_i,     // request address
  input  data_t [NumPorts-1:0] wdata_i,    // write data
  input  be_t   [NumPorts-1:0] be_i,       // write byte enable
  // output ports
  output data_t [NumPorts-1:0] rdata_o     // read data
);

  // For ECP5, we need to adapt the design to use the EBR primitives
  // EBR supports up to 18Kb per block in various configurations
  // Common configurations: 18Kb (2K x 9, 4K x 4, 8K x 2, 16K x 1)
  //                         9Kb (1K x 9, 2K x 4, 4K x 2, 8K x 1)
  
  // Calculate number of EBRs needed
  // ECP5 EBRs are 18Kb (18432 bits) each
  localparam int unsigned EBR_SIZE = 18432;  // bits per EBR block
  localparam int unsigned TOTAL_BITS = NumWords * DataWidth;
  localparam int unsigned NUM_EBRS = (TOTAL_BITS + EBR_SIZE - 1) / EBR_SIZE;  // ceiling division
  
  // EBR primitives in ECP5 handle byte enables differently from Xilinx XPM
  // We'll need to handle byte enables in our wrapper logic
  
  // Configure for EBR constraints:
  // - Each EBR can be configured as single port, simple dual port, or true dual port
  // - Data width can be up to 36 bits in simple/true dual port mode (18 bits per port)
  // - Actual array depth depends on data width (trade-off)
  
  // For byte enables, we'll need to read-modify-write when only some bytes are written
  // This is because EBR primitives don't natively support byte enables in the same way

  // Define EBR module width configurations
  // For ECP5, common data widths are 1, 2, 4, 9, 18, or 36 bits
  // Choose the smallest configuration that can fit our DataWidth
  
  // DP16KD is the ECP5 dual port RAM primitive
  // It supports various data width configurations:
  // 1-bit: 16K x 1
  // 2-bit: 8K x 2
  // 4-bit: 4K x 4
  // 9-bit: 2K x 9 (8 data + 1 parity)
  // 18-bit: 1K x 18 (16 data + 2 parity)
  // 36-bit: 512 x 36 (32 data + 4 parity) - in True Dual Port mode, this becomes 18-bit per port
  
  // Define configuration based on DataWidth
  localparam int unsigned EBR_DATA_WIDTH = 
    (DataWidth <= 9) ? 9 :
    (DataWidth <= 18) ? 18 : 36;
    
  localparam int unsigned EBR_ADDR_WIDTH = 
    (EBR_DATA_WIDTH == 9) ? 13 :  // 8K words
    (EBR_DATA_WIDTH == 18) ? 12 : // 4K words
    11;                           // 2K words for 36-bit mode
  
  localparam int unsigned EBR_DEPTH = 
    (EBR_DATA_WIDTH == 9) ? 8192 :
    (EBR_DATA_WIDTH == 18) ? 4096 : 
    2048;
  
  localparam int unsigned NUM_PARALLEL_EBRS = (DataWidth + EBR_DATA_WIDTH - 1) / EBR_DATA_WIDTH;
  localparam int unsigned NUM_CASCADED_EBRS = (NumWords + EBR_DEPTH - 1) / EBR_DEPTH;
  
  localparam int unsigned TOTAL_EBRS = NUM_PARALLEL_EBRS * NUM_CASCADED_EBRS;
  
  // Generate RAM blocks based on port configuration
  if (NumPorts == 32'd1) begin : gen_1_port
    // Single port RAM implementation
    // For 1 port, we use the DP16KD in single port mode

    // Registers for output data
    data_t rdata_q;
    logic read_en_q;
    addr_t addr_q;
    
    // Registered read enable and address
    always_ff @(posedge clk_i or negedge rst_ni) begin
      if (!rst_ni) begin
        read_en_q <= 1'b0;
        addr_q <= '0;
      end else begin
        read_en_q <= req_i[0] & ~we_i[0];
        addr_q <= addr_i[0];
      end
    end
    
    // Generate parallel EBRs for data width
    for (genvar p = 0; p < NUM_PARALLEL_EBRS; p++) begin : gen_parallel
      // Determine the actual data width for this slice
      localparam int unsigned SLICE_WIDTH = 
        (p == NUM_PARALLEL_EBRS-1) ? 
          ((DataWidth - p*EBR_DATA_WIDTH) <= EBR_DATA_WIDTH ? 
            (DataWidth - p*EBR_DATA_WIDTH) : EBR_DATA_WIDTH) : 
          EBR_DATA_WIDTH;
      
      // Generate cascaded EBRs for depth
      for (genvar c = 0; c < NUM_CASCADED_EBRS; c++) begin : gen_cascade
        // Calculate address range for this cascade
        localparam int unsigned ADDR_BASE = c * EBR_DEPTH;
        localparam int unsigned ADDR_BOUND = ((c+1) * EBR_DEPTH) - 1;
        
        // Determine if address in this range
        wire addr_in_range = (addr_i[0] >= ADDR_BASE) && (addr_i[0] <= ADDR_BOUND);
        wire addr_match = addr_in_range && req_i[0];
        
        // Slice input data for this EBR
        wire [SLICE_WIDTH-1:0] data_slice_in = wdata_i[0][p*EBR_DATA_WIDTH +: SLICE_WIDTH];
        wire [SLICE_WIDTH-1:0] data_slice_out;
        
        // Determine byte enables for this slice
        wire [(SLICE_WIDTH+7)/8-1:0] slice_be = be_i[0][p*((EBR_DATA_WIDTH+7)/8) +: ((SLICE_WIDTH+7)/8)];
        
        // Adjust write address for this cascade
        wire [EBR_ADDR_WIDTH-1:0] ebr_addr = addr_i[0] - ADDR_BASE;
        
        // Instantiate DP16KD for this segment
        if (SLICE_WIDTH <= 9) begin : gen_9bit
          // 9-bit data width configuration
          DP16KD #(
            .DATA_WIDTH_A("9"),
            .DATA_WIDTH_B("9"),
            .REGMODE_A("NOREG"),
            .REGMODE_B("NOREG"),
            .RESETMODE("SYNC"),
            .CSDECODE_A("0b000"),
            .CSDECODE_B("0b000"),
            .WRITEMODE_A("READBEFOREWRITE"),
            .WRITEMODE_B("READBEFOREWRITE"),
            .GSR("ENABLED"),
            .INITVAL_00(SimInit == "zeros" ? 320'h0 : 320'hX),  // Init to zeros or X
            .INITVAL_01(SimInit == "zeros" ? 320'h0 : 320'hX)
            // Add more INITVAL_XX parameters as needed
          ) i_ram_9bit (
            .DIA(data_slice_in),
            .ADA(ebr_addr),
            .CLKA(clk_i),
            .WEA(we_i[0] && addr_in_range),
            .CEA(addr_match),
            .OCEA(1'b1),
            .RSTA(~rst_ni),
            .CSA(3'b000),  // Chip select always enabled
            .DOA(data_slice_out),
            
            // Port B not used in single port mode
            .DIB(9'b0),
            .ADB(13'b0),
            .CLKB(1'b0),
            .WEB(1'b0),
            .CEB(1'b0),
            .OCEB(1'b0),
            .RSTB(1'b0),
            .CSB(3'b111)  // Chip select disabled
          );
        end else if (SLICE_WIDTH <= 18) begin : gen_18bit
          // 18-bit data width configuration
          DP16KD #(
            .DATA_WIDTH_A("18"),
            .DATA_WIDTH_B("18"),
            .REGMODE_A("NOREG"),
            .REGMODE_B("NOREG"),
            .RESETMODE("SYNC"),
            .CSDECODE_A("0b000"),
            .CSDECODE_B("0b000"),
            .WRITEMODE_A("READBEFOREWRITE"),
            .WRITEMODE_B("READBEFOREWRITE"),
            .GSR("ENABLED"),
            .INITVAL_00(SimInit == "zeros" ? 320'h0 : 320'hX),
            .INITVAL_01(SimInit == "zeros" ? 320'h0 : 320'hX)
            // Add more INITVAL_XX parameters as needed
          ) i_ram_18bit (
            .DIA(data_slice_in),
            .ADA({ebr_addr, 1'b0}),  // Adjust for 18-bit mode
            .CLKA(clk_i),
            .WEA(we_i[0] && addr_in_range),
            .CEA(addr_match),
            .OCEA(1'b1),
            .RSTA(~rst_ni),
            .CSA(3'b000),
            .DOA(data_slice_out),
            
            // Port B not used in single port mode
            .DIB(18'b0),
            .ADB(14'b0),
            .CLKB(1'b0),
            .WEB(1'b0),
            .CEB(1'b0),
            .OCEB(1'b0),
            .RSTB(1'b0),
            .CSB(3'b111)
          );
        end else begin : gen_36bit
          // 36-bit data width configuration (using both ports in parallel)
          // We split the 36-bit into two 18-bit segments
          wire [17:0] data_slice_out_lower, data_slice_out_upper;
          
          DP16KD #(
            .DATA_WIDTH_A("18"),
            .DATA_WIDTH_B("18"),
            .REGMODE_A("NOREG"),
            .REGMODE_B("NOREG"),
            .RESETMODE("SYNC"),
            .CSDECODE_A("0b000"),
            .CSDECODE_B("0b000"),
            .WRITEMODE_A("READBEFOREWRITE"),
            .WRITEMODE_B("READBEFOREWRITE"),
            .GSR("ENABLED"),
            .INITVAL_00(SimInit == "zeros" ? 320'h0 : 320'hX),
            .INITVAL_01(SimInit == "zeros" ? 320'h0 : 320'hX)
          ) i_ram_36bit (
            // Lower 18 bits on Port A
            .DIA(data_slice_in[17:0]),
            .ADA({ebr_addr, 1'b0}),
            .CLKA(clk_i),
            .WEA(we_i[0] && addr_in_range),
            .CEA(addr_match),
            .OCEA(1'b1),
            .RSTA(~rst_ni),
            .CSA(3'b000),
            .DOA(data_slice_out_lower),
            
            // Upper 18 bits on Port B
            .DIB(data_slice_in[35:18]),
            .ADB({ebr_addr, 1'b0}),
            .CLKB(clk_i),
            .WEB(we_i[0] && addr_in_range),
            .CEB(addr_match),
            .OCEB(1'b1),
            .RSTB(~rst_ni),
            .CSB(3'b000),
            .DOB(data_slice_out_upper)
          );
          
          // Combine outputs
          assign data_slice_out = {data_slice_out_upper, data_slice_out_lower};
        end
        
        // Connect to output
        always_ff @(posedge clk_i or negedge rst_ni) begin
          if (!rst_ni) begin
            rdata_q[p*EBR_DATA_WIDTH +: SLICE_WIDTH] <= '0;
          end else if (read_en_q && addr_q >= ADDR_BASE && addr_q <= ADDR_BOUND) begin
            rdata_q[p*EBR_DATA_WIDTH +: SLICE_WIDTH] <= data_slice_out;
          end
        end
      end
    end
    
    // Connect output
    assign rdata_o[0] = rdata_q;
    
  end else if (NumPorts == 32'd2) begin : gen_2_ports
    // True dual port RAM implementation
    
    // Registers for output data
    data_t [1:0] rdata_q;
    logic [1:0] read_en_q;
    addr_t [1:0] addr_q;
    
    // Registered read enable and address
    always_ff @(posedge clk_i or negedge rst_ni) begin
      if (!rst_ni) begin
        read_en_q <= '0;
        addr_q <= '{default:'0};
      end else begin
        for (int i = 0; i < 2; i++) begin
          read_en_q[i] <= req_i[i] & ~we_i[i];
          addr_q[i] <= addr_i[i];
        end
      end
    end
    
    // Generate parallel EBRs for data width
    for (genvar p = 0; p < NUM_PARALLEL_EBRS; p++) begin : gen_parallel
      // Determine the actual data width for this slice
      localparam int unsigned SLICE_WIDTH = 
        (p == NUM_PARALLEL_EBRS-1) ? 
          ((DataWidth - p*EBR_DATA_WIDTH) <= EBR_DATA_WIDTH ? 
            (DataWidth - p*EBR_DATA_WIDTH) : EBR_DATA_WIDTH) : 
          EBR_DATA_WIDTH;
      
      // True dual port mode has a maximum of 18 bits per port
      localparam int unsigned EBR_PORT_WIDTH = (SLICE_WIDTH <= 18) ? SLICE_WIDTH : 18;
      
      // Generate cascaded EBRs for depth
      for (genvar c = 0; c < NUM_CASCADED_EBRS; c++) begin : gen_cascade
        // Calculate address range for this cascade
        localparam int unsigned ADDR_BASE = c * EBR_DEPTH;
        localparam int unsigned ADDR_BOUND = ((c+1) * EBR_DEPTH) - 1;
        
        // Determine if address in this range for each port
        wire [1:0] addr_in_range;
        wire [1:0] addr_match;
        wire [EBR_ADDR_WIDTH-1:0] ebr_addr [1:0];
        
        for (genvar i = 0; i < 2; i++) begin : gen_port_addr
          assign addr_in_range[i] = (addr_i[i] >= ADDR_BASE) && (addr_i[i] <= ADDR_BOUND);
          assign addr_match[i] = addr_in_range[i] && req_i[i];
          assign ebr_addr[i] = addr_i[i] - ADDR_BASE;
        end
        
        // Slice input data for this EBR
        wire [EBR_PORT_WIDTH-1:0] data_slice_in [1:0];
        wire [EBR_PORT_WIDTH-1:0] data_slice_out [1:0];
        
        for (genvar i = 0; i < 2; i++) begin : gen_port_data
          assign data_slice_in[i] = wdata_i[i][p*EBR_DATA_WIDTH +: EBR_PORT_WIDTH];
        end
        
        // Instantiate EBR for this segment - up to 18 bits per port
        if (EBR_PORT_WIDTH <= 9) begin : gen_9bit_tdp
          // 9-bit data width configuration per port
          DP16KD #(
            .DATA_WIDTH_A("9"),
            .DATA_WIDTH_B("9"),
            .REGMODE_A("NOREG"),
            .REGMODE_B("NOREG"),
            .RESETMODE("SYNC"),
            .CSDECODE_A("0b000"),
            .CSDECODE_B("0b000"),
            .WRITEMODE_A("READBEFOREWRITE"),
            .WRITEMODE_B("READBEFOREWRITE"),
            .GSR("ENABLED"),
            .INITVAL_00(SimInit == "zeros" ? 320'h0 : 320'hX)
            // Add more INITVAL_XX parameters as needed
          ) i_ram_9bit_tdp (
            .DIA(data_slice_in[0]),
            .ADA(ebr_addr[0]),
            .CLKA(clk_i),
            .WEA(we_i[0] && addr_in_range[0]),
            .CEA(addr_match[0]),
            .OCEA(1'b1),
            .RSTA(~rst_ni),
            .CSA(3'b000),
            .DOA(data_slice_out[0]),
            
            .DIB(data_slice_in[1]),
            .ADB(ebr_addr[1]),
            .CLKB(clk_i),
            .WEB(we_i[1] && addr_in_range[1]),
            .CEB(addr_match[1]),
            .OCEB(1'b1),
            .RSTB(~rst_ni),
            .CSB(3'b000),
            .DOB(data_slice_out[1])
          );
        end else begin : gen_18bit_tdp
          // 18-bit data width configuration per port
          DP16KD #(
            .DATA_WIDTH_A("18"),
            .DATA_WIDTH_B("18"),
            .REGMODE_A("NOREG"),
            .REGMODE_B("NOREG"),
            .RESETMODE("SYNC"),
            .CSDECODE_A("0b000"),
            .CSDECODE_B("0b000"),
            .WRITEMODE_A("READBEFOREWRITE"),
            .WRITEMODE_B("READBEFOREWRITE"),
            .GSR("ENABLED"),
            .INITVAL_00(SimInit == "zeros" ? 320'h0 : 320'hX)
            // Add more INITVAL_XX parameters as needed
          ) i_ram_18bit_tdp (
            .DIA(data_slice_in[0]),
            .ADA({ebr_addr[0], 1'b0}),  // Adjust for 18-bit mode
            .CLKA(clk_i),
            .WEA(we_i[0] && addr_in_range[0]),
            .CEA(addr_match[0]),
            .OCEA(1'b1),
            .RSTA(~rst_ni),
            .CSA(3'b000),
            .DOA(data_slice_out[0]),
            
            .DIB(data_slice_in[1]),
            .ADB({ebr_addr[1], 1'b0}),  // Adjust for 18-bit mode
            .CLKB(clk_i),
            .WEB(we_i[1] && addr_in_range[1]),
            .CEB(addr_match[1]),
            .OCEB(1'b1),
            .RSTB(~rst_ni),
            .CSB(3'b000),
            .DOB(data_slice_out[1])
          );
        end
        
        // For wider than 18-bit, we need multiple EBRs in parallel
        // This is handled by the outer generate loop
        
        // Connect to output registers
        for (genvar i = 0; i < 2; i++) begin : gen_out_regs
          always_ff @(posedge clk_i or negedge rst_ni) begin
            if (!rst_ni) begin
              rdata_q[i][p*EBR_DATA_WIDTH +: EBR_PORT_WIDTH] <= '0;
            end else if (read_en_q[i] && addr_q[i] >= ADDR_BASE && addr_q[i] <= ADDR_BOUND) begin
              rdata_q[i][p*EBR_DATA_WIDTH +: EBR_PORT_WIDTH] <= data_slice_out[i];
            end
          end
        end
      end
    end
    
    // Connect output
    assign rdata_o = rdata_q;
    
  end else begin : gen_err_ports
    $fatal(1, "Not supported port parametrization for NumPorts: %0d", NumPorts);
  end

// Validate parameters.
// pragma translate_off
`ifndef VERILATOR
`ifndef TARGET_SYNTHESIS
  initial begin: p_assertions
    assert ($bits(addr_i)  == NumPorts * AddrWidth) else $fatal(1, "AddrWidth problem on `addr_i`");
    assert ($bits(wdata_i) == NumPorts * DataWidth) else $fatal(1, "DataWidth problem on `wdata_i`");
    assert ($bits(be_i)    == NumPorts * BeWidth)   else $fatal(1, "BeWidth   problem on `be_i`"   );
    assert ($bits(rdata_o) == NumPorts * DataWidth) else $fatal(1, "DataWidth problem on `rdata_o`");
    assert (NumWords  >= 32'd1) else $fatal(1, "NumWords has to be > 0");
    assert (DataWidth >= 32'd1) else $fatal(1, "DataWidth has to be > 0");
    assert (ByteWidth >= 32'd1) else $fatal(1, "ByteWidth has to be > 0");
    assert (NumPorts  >= 32'd1) else $fatal(1, "The number of ports must be at least 1!");
    // ECP5 specific assertions
    assert (NumPorts <= 32'd2) else $fatal(1, "ECP5 implementation supports maximum 2 ports!");
  end
  
  initial begin: p_sim_hello
    if (PrintSimCfg) begin
      $display("#################################################################################");
      $display("tc_sram ECP5 implementation instantiated with the configuration:"                 );
      $display("Instance: %m"                                                                     );
      $display("Number of ports   (dec): %0d", NumPorts                                           );
      $display("Number of words   (dec): %0d", NumWords                                           );
      $display("Address width     (dec): %0d", AddrWidth                                          );
      $display("Data width        (dec): %0d", DataWidth                                          );
      $display("Byte width        (dec): %0d", ByteWidth                                          );
      $display("Byte enable width (dec): %0d", BeWidth                                            );
      $display("Latency Cycles    (dec): %0d", Latency                                            );
      $display("Simulation init   (str): %0s", SimInit                                            );
      $display("Target implementation: Lattice ECP5"                                              );
      $display("Number of EBRs used: %0d parallel x %0d cascaded = %0d total", 
               NUM_PARALLEL_EBRS, NUM_CASCADED_EBRS, TOTAL_EBRS                                   );
      $display("#################################################################################");
    end
  end
  
  for (genvar i = 0; i < NumPorts; i++) begin : gen_assertions
    assert property ( @(posedge clk_i) disable iff (!rst_ni)
        (req_i[i] |-> (addr_i[i] < NumWords))) else
      $warning("Request address %0h not mapped, port %0d, expect random write or read behavior!",
          addr_i[i], i);
  end

`endif
`endif
// pragma translate_on

endmodule