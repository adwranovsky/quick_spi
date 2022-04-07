`default_nettype none

/*
 * quick_spi - Quickly adds a SPI interface to a project
 *
 * Parameters:
 *  CLK_FREQ_HZ - The input clock frequency
 *  SCLK_FREQ_HZ - The SPI clock frequency
 *  DATA_WIDTH - The number of data bits to read and write per transaction
 *  CS_TO_SCLK_TIME - The minimum allowed time in seconds from CS assertion to SCLK first going low
 *  HOLDOFF_TIME - The minimum amount of time from the last falling edge of SCLK to the next CS assertion
 *  COVER - For testing use only. Set to 1 to include cover properties during formal verification
 *
 * Ports:
 *  clk_i - The system clock
 *  enable_i - When high, enables clk_o
 *  clk_o - The output clock, with a frequency that is the frequency of clk_i divided by DIV
 *
 * Description:
 */
module quick_spi #(
    parameter CLK_FREQ_HZ = 100000000,
    parameter SCLK_FREQ_HZ = 20000000,
    parameter MAX_DATA_LENGTH = 16,
    parameter NUM_DEVICES = 1,
    parameter CS_TO_SCLK_TIME = 10e-9,
    parameter HOLDOFF_TIME = 90e-9,
    parameter COVER = 0,

    localparam NUM_DATA_WIDTH = $clog2(MAX_DATA_LENGTH)
) (
    input wire clk_i,
    input wire rst_i,

    // FPGA interface
    input  wire request_i,
    input  wire [NUM_DATA_WIDTH-1:0] num_data_i,
    input  wire [MAX_DATA_LENGTH*NUM_DEVICES-1:0] data_i,
    output wire [MAX_DATA_LENGTH*NUM_DEVICES-1:0] data_o,
    output reg  data_valid_o,

    // SPI interface
    output wire sclk_o,
    output reg  cs_n_o,
    input  wire [NUM_DEVICES-1:0] sdata_i,
    output wire [NUM_DEVICES-1:0] sdata_o
);
    genvar i;

    // TODO: Is this check real? Is it not ... > CLK_FREQ_HZ*2?
    // Try to force an elaboration failure if invalid parameters were specified
    generate if (SCLK_FREQ_HZ > CLK_FREQ_HZ)
        invalid_verilog_parameter CLK_FREQ_HZ_must_be_larger_than_SCLK_FREQ_HZ ();
    endgenerate
    generate if (NUM_DEVICES <= 0)
        invalid_verilog_parameter NUM_DEVICES_must_be_a_positive_integer ();
    endgenerate


    // The number of system clocks per SPI clock
    localparam CLK_DIV = CLK_FREQ_HZ / SCLK_FREQ_HZ;
    // The minimum number of system clocks it takes to satisfy t2 from the datasheet
    localparam CS_TO_SCLK_CLOCKS = $rtoi($ceil(CLK_FREQ_HZ * CS_TO_SCLK_TIME));
    // The minimum number of system clocks it takes to satisfy t8+tquiet from the datasheet
    localparam HOLDOFF_CLOCKS = $rtoi($ceil(CLK_FREQ_HZ * HOLDOFF_TIME));


    /*
     * Create a timer for handling SPI timings
     */
    localparam TIMER_WIDTH = $clog2(HOLDOFF_CLOCKS);
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
    wire sclk_falling_edge = !sclk_o && last_sclk;

    /*
     * Count the amount of data transferred
     */
    reg set_num_data = 0;
    reg [NUM_DATA_WIDTH-1:0] data_remaining = 0;
    wire completed_transfer = data_remaining == 0;
    always @(posedge clk_i)
        if (set_num_data)
            data_remaining <= num_data_i;
        else if (sclk_rising_edge)
            data_remaining <= data_remaining - 1;
        else
            data_remaining <= data_remaining;

    /*
     * Create a serial-in parallel-out shift register for each device for reading in SPI data and converting it to
     * parallel
     */
    localparam SIPO_SHIFT_REGISTER_WIDTH = MAX_DATA_LENGTH + 1; // add one to account for sclk_o's return to idle
    generate for (i = 0; i < NUM_DEVICES; i = i+1) begin
        wire [SIPO_SHIFT_REGISTER_WIDTH-1:0] spi_parallel_out;
        shift_register_sipo #(
            .WIDTH(SIPO_SHIFT_REGISTER_WIDTH)
        ) spi_serial_in_parallel_out (
            .clk_i(clk_i),
            .rst_i(rst_i),
            .advance_i(sclk_rising_edge),
            .bit_i(sdata_i[i]),
            .value_o(spi_parallel_out)
        );
        assign data_o[MAX_DATA_LENGTH*i +: MAX_DATA_LENGTH] = spi_parallel_out[1 +: MAX_DATA_LENGTH];
    end endgenerate

    /*
     * Create a parallel-in serial-out shift register from data_i to sdata_o for each device
     */
    reg set_data_out;
    generate for (i = 0; i < NUM_DEVICES; i = i+1) begin
        shift_register_piso #(
            .WIDTH(MAX_DATA_LENGTH)
        ) spi_parallel_in_serial_out (
            .clk_i(clk_i),
            .rst_i(rst_i),
            .set_i(set_data_out),
            .value_i(data_i[MAX_DATA_LENGTH*i +: MAX_DATA_LENGTH]),
            .advance_i(sclk_falling_edge),
            .bit_o(sdata_o[i])
        );
    end endgenerate

    /*
     * SPI master state machine
     */
    localparam
        WAIT          = 3'h0,
        CHIP_SELECT   = 3'h1,
        TRANSFER_DATA = 3'h2,
        BE_QUIET      = 3'h3,
        SAMPLE_STROBE = 3'h4,
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
                    next_state = WAIT;
            WAIT:
                if (request_i)
                    next_state = CHIP_SELECT;
            CHIP_SELECT:
                if (timer_done)
                    next_state = TRANSFER_DATA;
            TRANSFER_DATA:
                if (completed_transfer)
                    next_state = BE_QUIET;
            BE_QUIET:
                if (timer_done)
                    next_state = SAMPLE_STROBE;
            SAMPLE_STROBE:
                if (request_i)
                    next_state = CHIP_SELECT;
                else
                    next_state = WAIT;
        endcase
        /* verilator lint_on CASEINCOMPLETE */
    end

    // State outputs
    always @(*) begin
        cs_n_o = 1;
        data_valid_o = 0;
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
            WAIT: begin
                start_timer = request_i; // TODO: finish this
                set_num_data = request_i;
                set_data_out = request_i;
            end
            CHIP_SELECT: begin
                cs_n_o = 0;
                enable_sclk = timer_done;
            end
            TRANSFER_DATA: begin
                timer_count = HOLDOFF_CLOCKS[TIMER_WIDTH-1:0];
                start_timer = completed_transfer;
                cs_n_o = 0;
                enable_sclk = 1;
            end
            BE_QUIET: begin
                // all outputs are the default
            end
            SAMPLE_STROBE: begin
                data_valid_o = 1;
                start_timer = request_i;
                set_num_data = request_i;
                set_data_out = request_i;
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

    // Keep track of whether or not a transaction is currently outstanding
    reg f_transaction_outstanding = 0;
    always @(posedge clk_i)
        if (rst_i || (data_valid_o && !request_i) || state==RESET) // I'm cheating a bit here by checking `state`, but it makes this so much easier
            f_transaction_outstanding <= 0;
        else if (request_i)
            f_transaction_outstanding <= 1;
        else
            f_transaction_outstanding <= f_transaction_outstanding;

    // Remember how much data is requested and create a mask for those bits
    reg [MAX_DATA_LENGTH-1:0] f_num_data_requested_mask = 0;
    always @(posedge clk_i)
        if (!f_transaction_outstanding && request_i)
            f_num_data_requested_mask <= {MAX_DATA_LENGTH{1'b1}} >> (MAX_DATA_LENGTH - num_data_i);

    // Keep track of the last MAX_DATA_WIDTH+1 data bits clocked in for each device on sdata_i
    // The extra +1 is added because sclk_o idles high and will have one "fake" rising edge when it returns to idle
    reg [MAX_DATA_LENGTH*NUM_DEVICES:0] f_last_data_word;
    generate for (f = 0; f < NUM_DEVICES; f = f+1) begin
        always @(posedge clk_i)
            if (f_past_valid && $rose(sclk_o))
                // Shift in the value on sdata_i on the LSB of f_last_data_word
                f_last_data_word[f*(MAX_DATA_LENGTH+1) +: MAX_DATA_LENGTH+1] <= {
                    f_last_data_word[f*MAX_DATA_LENGTH +: MAX_DATA_LENGTH],
                    sdata_i[f]
                };
    end endgenerate

    // Make sure the state register is always valid
    always @(*)
        assert(
            state == WAIT ||
            state == CHIP_SELECT ||
            state == TRANSFER_DATA ||
            state == BE_QUIET ||
            state == SAMPLE_STROBE ||
            state == RESET
        );

    // Verify that the data put on sdata_i matches what's strobed on data_o
    generate for (f = 0; f < NUM_DEVICES; f = f+1) begin
        always @(posedge clk_i)
            if (data_valid_o) begin
                assert(
                    (data_o[f*MAX_DATA_LENGTH +: MAX_DATA_LENGTH] & f_num_data_requested_mask)
                    ==
                    (f_last_data_word[(f*(MAX_DATA_LENGTH+1))+1 +: MAX_DATA_LENGTH] & f_num_data_requested_mask)
                );
            end
    end endgenerate

    // Assert that data_valid_o is only asserted if there is an outstanding transaction
    always @(*)
        if (data_valid_o)
            assert(f_transaction_outstanding);


    // Cover properties
    generate if (COVER==1) begin
        always @(*)
            cover();
    end endgenerate
`endif

endmodule

`default_nettype wire
