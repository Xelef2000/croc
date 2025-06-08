/// SPI Manager for 23LCV1024 SRAM
/// Always operates in mode 0 (CPOL=0, CPHA=0)
/// Always operates in 32-bit sequential mode (4-byte bursts, opcode 0x02/0x03)
/// Handles SPI command, 24-bit address, and 4-byte read/write transfers

module spi_manager (
    /// System clock domain
    input  logic        clk_i,
    /// Active-low reset
    input  logic        rst_ni,
    /// Transfer start pulse (asserted for 1 clk_i cycle)
    input  logic        start_i,
    /// Write enable: 1 = write (0x02), 0 = read (0x03)
    input  logic        we_i,
    /// 24-bit address
    input  logic [23:0] addr_i,
    /// Write data (used if we_i == 1)
    input  logic [31:0] wdata_i,

    /// Externally generated SPI clock (mode 0, synchronized to clk_i)
    input  logic        spi_clk_i,
    /// SPI MISO line
    input  logic        miso_i,
    /// SPI MOSI line
    output logic        mosi_o,
    /// SPI chip select (active low)
    output logic        cs_n_o,

    /// Response valid (1 clk_i pulse when transaction completes)
    output logic        rsp_valid_o,
    /// Read data (valid only when rsp_valid_o is high)
    output logic [31:0] rdata_o
);

    /// SPI FSM states
    typedef enum logic [2:0] {
        IDLE,   // Waiting for start pulse
        CMD,    // Sending command byte
        ADDR,   // Sending 24-bit address
        RW,     // Transferring 4 data bytes
        DONE    // End of SPI transaction
    } state_e;

    state_e state_d, state_q;

    /// Shift register used for sending command, address, and data
    logic [7:0]  shift_reg_d, shift_reg_q;
    /// Register for accumulating received read data
    logic [31:0] rdata_d, rdata_q;
    /// Bit counter for serial transfers (0–31)
    logic [5:0]  bit_cnt_d, bit_cnt_q;

    /// SPI chip select register (active-low)
    logic cs_n_d, cs_n_q;
    /// SPI MOSI register
    logic mosi_d, mosi_q;
    /// Previous SPI clock state for edge detection
    logic spi_clk_q;

    /// Rising edge detection on spi_clk_i (must be aligned with clk_i domain)
    logic spi_clk_rising;

    // Detect rising edge of external SPI clock
    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni)
            spi_clk_q <= 1'b0;
        else
            spi_clk_q <= spi_clk_i;
    end

    assign spi_clk_rising = (spi_clk_i && !spi_clk_q);

    // FSM + control logic
    always_comb begin
        // Default: hold current state/values
        state_d      = state_q;
        bit_cnt_d    = bit_cnt_q;
        shift_reg_d  = shift_reg_q;
        rdata_d      = rdata_q;
        cs_n_d       = cs_n_q;
        mosi_d       = mosi_q;
        rsp_valid_o  = 1'b0;

        case (state_q)

            /// Wait for start pulse
            IDLE: begin
                cs_n_d = 1'b1; // keep CS deasserted
                if (start_i) begin
                    cs_n_d = 1'b0; // assert CS
                    state_d = CMD;
                    bit_cnt_d = 6'd7;
                    shift_reg_d = we_i ? 8'h02 : 8'h03; // opcode
                end
            end

            /// Send opcode (0x02 for write, 0x03 for read)
            CMD: begin
                if (spi_clk_rising) begin
                    mosi_d = shift_reg_q[7];
                    shift_reg_d = {shift_reg_q[6:0], 1'b0}; // shift left

                    if (bit_cnt_q == 0) begin
                        state_d = ADDR;
                        bit_cnt_d = 6'd23;
                        shift_reg_d = addr_i[23:16]; // first address byte
                    end else begin
                        bit_cnt_d -= 1;
                    end
                end
            end

            /// Send 24-bit address MSB → LSB
            ADDR: begin
                if (spi_clk_rising) begin
                    mosi_d = shift_reg_q[7];
                    shift_reg_d = {shift_reg_q[6:0], 1'b0};

                    // Load next address byte after each 8-bit chunk
                    if (bit_cnt_q == 16) shift_reg_d = addr_i[15:8];
                    else if (bit_cnt_q == 8) shift_reg_d = addr_i[7:0];

                    if (bit_cnt_q == 0) begin
                        state_d = RW;
                        bit_cnt_d = 6'd31;
                        shift_reg_d = wdata_i[31:24]; // first byte of data
                    end else begin
                        bit_cnt_d -= 1;
                    end
                end
            end

            /// Data phase: either send (write) or receive (read) 32 bits
            RW: begin
                if (spi_clk_rising) begin
                    if (we_i) begin
                        // Write mode: send 32-bit wdata_i MSB → LSB
                        mosi_d = shift_reg_q[7];
                        shift_reg_d = {shift_reg_q[6:0], 1'b0};

                        if (bit_cnt_q == 24) shift_reg_d = wdata_i[23:16];
                        else if (bit_cnt_q == 16) shift_reg_d = wdata_i[15:8];
                        else if (bit_cnt_q == 8)  shift_reg_d = wdata_i[7:0];
                    end else begin
                        // Read mode: shift in 32-bit response
                        mosi_d = 1'b0; // don't care
                        rdata_d = {rdata_q[30:0], miso_i};
                    end

                    if (bit_cnt_q == 0) begin
                        state_d = DONE;
                    end else begin
                        bit_cnt_d -= 1;
                    end
                end
            end

            /// Transfer complete, deassert CS and pulse rsp_valid_o
            DONE: begin
                cs_n_d = 1'b1;
                rsp_valid_o = 1'b1;
                state_d = IDLE;
            end

            default: state_d = IDLE;
        endcase
    end

    // Register updates
    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            state_q      <= IDLE;
            shift_reg_q  <= '0;
            bit_cnt_q    <= '0;
            cs_n_q       <= 1'b1;
            mosi_q       <= 1'b0;
            rdata_q      <= '0;
        end else begin
            state_q      <= state_d;
            shift_reg_q  <= shift_reg_d;
            bit_cnt_q    <= bit_cnt_d;
            cs_n_q       <= cs_n_d;
            mosi_q       <= mosi_d;
            rdata_q      <= rdata_d;
        end
    end

    /// Outputs
    assign cs_n_o   = cs_n_q;
    assign mosi_o   = mosi_q;
    assign rdata_o  = rdata_q;

endmodule