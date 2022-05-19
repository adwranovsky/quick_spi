`default_nettype none

/*
 * quick_spi - Quickly adds a SPI interface to a project
 *
 * Parameters:
 *  CLK_FREQ_HZ - The input clock frequency
 *  SCLK_FREQ_HZ - The SPI clock frequency. The actual frequency will only match if `CLK_FREQ_HZ` / `SCLK_FREQ_HZ` is an
 *                 integer. Additionally, the frequency must be no greater than 1/4 the system clock.
 *  MAX_DATA_LENGTH - The maximum number of data bits that can be read and written per transaction
 *  NUM_DEVICES - The number of devices parallel to each other on the bus. Defaults to a single device.
 *  CS_TO_SCLK_TIME - The minimum allowed time in seconds from CS assertion to SCLK first going low
 *  SDATA_HOLD_TIME - The minimum amount of time to hold `sdata_o` steady after the rising edge of `sclk_o`
 *  QUIET_TIME - The minimum amount of time in seconds from the last rising edge of SCLK to the next CS assertion
 *  COVER - For testing use only. Set to 1 to include cover properties during formal verification
 *
 * Ports:
 *  clk_i - The system clock
 *  rst_i - An active high reset synchronous with `clk_i`
 *
 *  wrdata_valid_i - Indicates that `wrdata_i` and `wrdata_len_i` are valid. A SPI transaction is only started when both
 *                   `wrdata_valid_i` and `wrdata_ready_o` are high on the same clock cycle.
 *  wrdata_ready_o - Indicates that the module is ready to accept write data for a new transaction. A SPI transaction is
 *                   only started when both `wrdata_valid_i` and `wrdata_ready_o` are high on the same clock cycle.
 *  wrdata_len_i - The number of bits from `wrdata_i` to send on `sdata_o` and to receive on `sdata_i`.
 *  wrdata_i - The data to write out serially on `sclk_o`. Only the first `wrdata_len_i` bits are used.
 *
 *  rddata_valid_o - Indicates that `rddata_o` and `rddata_mask_o` are valid. The SPI transaction is only completed once
 *                   `rddata_valid_o` and `rddata_read_i` are high on the same clock cycle.
 *  rddata_ready_i - Indicates that the upstream device is ready to accept data. The SPI transaction is only completed
 *                   once `rddata_ready_i` and `rddata_valid_o` are high on the same clock cycle.
 *  rddata_mask_o - Indicates which bits of `rddata_o` are valid. The number of high bits
 *                  matches the data length requested on `wrdata_len_i`.
 *  rddata_o - The data read serially in on `sdata_i`. Valid when `rddata_valid_o` is high, though only the first
 *             `wrdata_len_i` bits requested are actually valid.
 *  sclk_o - The output clock for the SPI interface, with a frequency no greater than `SCLK_FREQ_HZ`.
 *  cs_n_o - The chip select signal of the SPI interface
 *  sdata_i - The SPI data input from the SPI device
 *  sdata_o - The SPI data output to the SPI device
 *
 * Description:
 *  The quickest way to integrate an external SPI device into a design. It has a simple request interface supporting
 *  variable payload lengths, and handles some common requirements on the device side of the interface. Additionally it
 *  can handle multiple devices connected to the same SPI clock and chip select in parallel. To start a transaction, put
 *  the data you want to write on `wrdata_i` along with the length on `wrdata_len_i`, and strobe `wrdata_valid_i`. Then just wait
 *  for `rddata_valid_o` to go high and read the data back on `wrdata_i`. 
 */
module quick_spi #(
    parameter CLK_FREQ_HZ = 100000000,
    parameter SCLK_FREQ_HZ = 20000000,
    parameter MAX_DATA_LENGTH = 16,
    parameter NUM_DEVICES = 1,
    parameter CS_TO_SCLK_TIME = 1.0 / SCLK_FREQ_HZ / 4.0,
    parameter SDATA_HOLD_TIME = 1.0 / SCLK_FREQ_HZ / 4.0,
    parameter QUIET_TIME = 1.0 / SCLK_FREQ_HZ * 2.0,
    parameter COVER = 0,

    localparam NUM_DATA_WIDTH = $clog2(MAX_DATA_LENGTH)
) (
    input wire clk_i,
    input wire rst_i,

    // FPGA interface
    input  wire wrdata_valid_i,
    output reg  wrdata_ready_o,
    input  wire [NUM_DATA_WIDTH-1:0] wrdata_len_i,
    input  wire [MAX_DATA_LENGTH*NUM_DEVICES-1:0] wrdata_i,
    output reg  rddata_valid_o,
    input  wire rddata_ready_i,
    output reg  [MAX_DATA_LENGTH-1:0] rddata_mask_o,
    output wire [MAX_DATA_LENGTH*NUM_DEVICES-1:0] rddata_o,

    // SPI interface
    output wire sclk_o,
    output reg  cs_n_o,
    input  wire [NUM_DEVICES-1:0] sdata_i,
    output wire [NUM_DEVICES-1:0] sdata_o
);
    // A function for indexing into the device data when there are multiple devices
    function [MAX_DATA_LENGTH-1:0] device_data (input [MAX_DATA_LENGTH*NUM_DEVICES-1:0] data, input integer dev);
        device_data = data[dev*MAX_DATA_LENGTH +: MAX_DATA_LENGTH];
    endfunction

    genvar i;

    // Try to force an elaboration failure if invalid parameters were specified
    generate if (SCLK_FREQ_HZ*4 > CLK_FREQ_HZ)
        invalid_verilog_parameter SCLK_FREQ_HZ_times_4_must_be_less_than_or_equal_to_CLK_FREQ_HZ ();
    endgenerate
    generate if (NUM_DEVICES <= 0)
        invalid_verilog_parameter NUM_DEVICES_must_be_a_positive_integer ();
    endgenerate


    // The number of system clocks per SPI clock
    localparam CLK_DIV = CLK_FREQ_HZ / SCLK_FREQ_HZ;
    // The minimum number of system clocks from chip select assertion to the first falling sclk edge
    localparam CS_TO_SCLK_CLOCKS = $rtoi($ceil(CLK_FREQ_HZ * CS_TO_SCLK_TIME));
    // The minimum number of system clocks to hold sdata_o steady after a rising sclk edge. Subtract one since edge
    // detection is delayed by one clock cycle
    localparam SDATA_HOLD_CLOCKS = $rtoi($ceil(CLK_FREQ_HZ * SDATA_HOLD_TIME)) - 1;
    // The minimum number of system clocks to wait from the last rising sclk edge until a transaction can be started
    // again
    localparam QUIET_CLOCKS = $rtoi($ceil(CLK_FREQ_HZ * QUIET_TIME));


    /*
     * Create a timer for handling SPI timings
     */
    localparam TIMER_WIDTH = $clog2(QUIET_CLOCKS);
    reg start_timer;
    reg timer_done;
    reg [TIMER_WIDTH-1:0] timer_count;
    timer #(
        .WIDTH(TIMER_WIDTH)
    ) spi_timer (
        .clk_i(clk_i),
        .rst_i(rst_i),
        .start_i(start_timer),
        .count_i(timer_count),
        .done_o(timer_done)
    );

    /*
     * Create a clock divider module for generating sclk_o
     */
    reg enable_sclk, sclk_idle;
    clkdiv #(
        .DIV(CLK_DIV),
        .IDLE_HIGH(1)
    ) sclk_generator (
        .clk_i(clk_i),
        .enable_i(enable_sclk),
        .idle_o(sclk_idle),
        .clk_o(sclk_o)
    );

    /*
     * Detect rising and falling edges of sclk_o
     */
    reg last_sclk = 1;
    always @(posedge clk_i)
        last_sclk <= sclk_o;
    wire sclk_rising_edge  = sclk_o && !last_sclk;

    /*
     * Handle sdata hold time using a shift register that delays sclk_rising_edge
     */
    wire shift_data_out;
    generate
        if (SDATA_HOLD_CLOCKS == 0) begin
            assign shift_data_out = sclk_rising_edge;
        end else begin
            reg [SDATA_HOLD_CLOCKS-1:0] shift_delay = 0;
            always @(posedge clk_i) begin
                if (rst_i) begin
                    shift_delay <= 0;
                end else begin
                    shift_delay    <= shift_delay << 1;
                    shift_delay[0] <= sclk_rising_edge;
                end
            end
            assign shift_data_out = shift_delay[SDATA_HOLD_CLOCKS-1];
        end
    endgenerate

    /*
     * Count the amount of data transferred
     */
    reg set_num_data = 0;
    reg [NUM_DATA_WIDTH-1:0] data_remaining = 0;
    wire completed_transfer = data_remaining == 0;
    always @(posedge clk_i)
        if (set_num_data)
            data_remaining <= wrdata_len_i;
        else if (sclk_rising_edge)
            data_remaining <= data_remaining - 1;
        else
            data_remaining <= data_remaining;

    /*
     * Create a serial-in parallel-out shift register for each device for reading in SPI data and converting it to
     * parallel
     */
    generate for (i = 0; i < NUM_DEVICES; i = i+1) begin
        wire [MAX_DATA_LENGTH-1:0] spi_parallel_out;
        shift_register_sipo #(
            .WIDTH(MAX_DATA_LENGTH)
        ) spi_serial_in_parallel_out (
            .clk_i(clk_i),
            .rst_i(rst_i),
            .advance_i(sclk_rising_edge && !cs_n_o), // only shift when chip select is asserted
            .bit_i(sdata_i[i]),
            .value_o(spi_parallel_out)
        );
        assign rddata_o[MAX_DATA_LENGTH*i +: MAX_DATA_LENGTH] = spi_parallel_out;
    end endgenerate

    /*
     * Create a parallel-in serial-out shift register from wrdata_i to sdata_o for each device
     */
    reg set_data_out;
    generate for (i = 0; i < NUM_DEVICES; i = i+1) begin
        shift_register_piso #(
            .WIDTH(MAX_DATA_LENGTH)
        ) spi_parallel_in_serial_out (
            .clk_i(clk_i),
            .rst_i(rst_i),
            .set_i(set_data_out),
            .value_i(wrdata_i[MAX_DATA_LENGTH*i +: MAX_DATA_LENGTH]),
            .advance_i(shift_data_out),
            .bit_o(sdata_o[i])
        );
    end endgenerate

    /*
     * Create the read data mask on write handshakes
     */
    initial rddata_mask_o = 0;
    always @(posedge clk_i)
        if (wrdata_valid_i && wrdata_ready_o)
            rddata_mask_o <= {MAX_DATA_LENGTH{1'b1}} >> (MAX_DATA_LENGTH - wrdata_len_i);
        else
            rddata_mask_o <= rddata_mask_o;

    /*
     * SPI state machine
     */
    localparam
        WRDATA_READY  = 3'h0,
        CHIP_SELECT   = 3'h1,
        TRANSFER_DATA = 3'h2,
        RDDATA_VALID  = 3'h3,
        BE_QUIET      = 3'h4,
        RESET         = 3'h5;
    reg [2:0] state, next_state;

    // State register
    initial state = RESET;
    always @(posedge clk_i)
        if (rst_i)
            state <= RESET;
        else
            state <= next_state;

    // State transition logic
    always @(*) begin
        next_state = state;
        /* verilator lint_off CASEINCOMPLETE */
        case (state)
            RESET:
                if (sclk_idle)
                    next_state = WRDATA_READY;
            WRDATA_READY:
                if (wrdata_valid_i)
                    next_state = CHIP_SELECT;
            CHIP_SELECT:
                if (timer_done)
                    next_state = TRANSFER_DATA;
            TRANSFER_DATA:
                if (completed_transfer)
                    next_state = RDDATA_VALID;
            RDDATA_VALID:
                if (rddata_ready_i)
                    next_state = BE_QUIET;
            BE_QUIET:
                if (timer_done)
                    next_state = WRDATA_READY;
        endcase
        /* verilator lint_on CASEINCOMPLETE */
    end

    // State outputs
    always @(*) begin
        cs_n_o = 1;
        rddata_valid_o = 0;
        wrdata_ready_o = 0;
        enable_sclk = 0;
        start_timer = 0;
        timer_count = CS_TO_SCLK_CLOCKS[TIMER_WIDTH-1:0];
        set_num_data = 0;
        set_data_out = 0;
        /* verilator lint_off CASEINCOMPLETE */
        case (state)
            RESET: begin
                // all outputs are the default
            end
            WRDATA_READY: begin
                wrdata_ready_o = 1;
                start_timer = wrdata_valid_i;
                set_num_data = wrdata_valid_i;
                set_data_out = wrdata_valid_i;
            end
            CHIP_SELECT: begin
                cs_n_o = 0;
                enable_sclk = timer_done;
            end
            TRANSFER_DATA: begin
                timer_count = QUIET_CLOCKS[TIMER_WIDTH-1:0];
                start_timer = completed_transfer;
                cs_n_o = 0;
                enable_sclk = 1;
            end
            RDDATA_VALID: begin
                rddata_valid_o = 1;
            end
            BE_QUIET: begin
                // all outputs are the default
            end
        endcase
        /* verilator lint_on CASEINCOMPLETE */
    end


`ifdef FORMAL
    genvar f;

    // Keep track of whether or not $past() is valid
    reg f_past_valid = 0;
    always @(posedge clk_i)
        f_past_valid <= 1;

    // Assume that the device starts in reset
    always @(*)
        if (!f_past_valid)
            assume(rst_i);

    // Assume that the downstream device is only going to change sdata_i on a falling clock edge
    always @(posedge clk_i)
        if (f_past_valid && $changed(sdata_i))
            assume($fell(sclk_o));

    // Make sure the state register is always valid
    always @(*)
        assert(
            state == WRDATA_READY ||
            state == CHIP_SELECT ||
            state == TRANSFER_DATA ||
            state == RDDATA_VALID ||
            state == BE_QUIET ||
            state == RESET
        );

    // Identify when handshakes happen
    wire f_write_handshake = wrdata_valid_i && wrdata_ready_o && !rst_i;
    wire f_read_handshake  = rddata_valid_o && rddata_ready_i && !rst_i;

    // Count the number of outstanding read transactions
    integer f_outstanding_read_handshakes = 0;
    always @(posedge clk_i)
        if (rst_i)
            f_outstanding_read_handshakes <= 0;
        else if (f_write_handshake)
            f_outstanding_read_handshakes <= f_outstanding_read_handshakes + 1;
        else if (f_read_handshake)
            f_outstanding_read_handshakes <= f_outstanding_read_handshakes - 1;

    // Make sure there's never more than one outstanding read handshake
    always @(*)
        assert( (f_outstanding_read_handshakes == 0) || (f_outstanding_read_handshakes == 1) );

    // Make sure the data presented on the read handshakes matches what came in on the SPI interface
    reg [MAX_DATA_LENGTH*NUM_DEVICES-1:0] f_spi_read_data = 0;
    generate for (f = 0; f < NUM_DEVICES; f = f + 1) begin
        // Shift in read data
        always @(posedge clk_i) begin
            if ($rose(sclk_o) && !cs_n_o) begin
                f_spi_read_data[f*MAX_DATA_LENGTH +: MAX_DATA_LENGTH] <= f_spi_read_data[f*MAX_DATA_LENGTH +: MAX_DATA_LENGTH] << 1;
                f_spi_read_data[f*MAX_DATA_LENGTH] <= sdata_i;
            end
        end

        // Check read handshakes
        always @(posedge clk_i)
            if (f_read_handshake)
                assert(
                    (rddata_mask_o & device_data(rddata_o, f))
                    ==
                    (rddata_mask_o & device_data(f_spi_read_data, f))
                );
    end endgenerate

    // Check each data bit clocked out on sdata_o against the last write handshake
    // TODO

    // Cover properties to demonstrate how the device is used
    generate if (COVER==1 && NUM_DEVICES==1 && MAX_DATA_LENGTH==5) begin
        reg [MAX_DATA_LENGTH-1:0] f_last_data_requested = 0;
        always @(posedge clk_i)
            if (state==WRDATA_READY && wrdata_valid_i)
                f_last_data_requested <= wrdata_i;

        always @(*)
            cover(
                !rst_i && rddata_valid_o && f_num_data_requested_mask == 5'b11111
                && rddata_o == 5'b10101 && f_last_data_requested == 5'b01010
            );
    end endgenerate
`endif

endmodule

`default_nettype wire
