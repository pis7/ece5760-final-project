//========================================================================
// DotProduct.v
//========================================================================
// Dot product computed on two input vectors. To synchronize timing, a val/rdy interface is employed.

module DotProduct #(
    parameter DATA_LEN = 32 // Size of data in vectors
)(
    input  wire        clk,
    input  wire        rst,
    input  wire [31:0] n,   // Vector length

    // Data from input vectors
    input  wire [DATA_LEN-1:0] data_in1,
    input  wire [DATA_LEN-1:0] data_in2,

    // Dot product computation outputs
    output reg  [DATA_LEN-1:0] result,
    output reg  [31:0]         compute_counter, // Current index in vector for which computations are being run

    // val/rdy interface at input
    input  wire dotp_req_val,
    output wire dotp_req_rdy,

    // val/rdy interface at output
    input  wire dotp_resp_rdy,
    output wire dotp_resp_val
);

    // ------------------------------------------------------------
    // STATE MACHINE SETUP
    // ------------------------------------------------------------
    localparam INIT    = 2'd0; // This state can probably be removed - only included to ensure correct start-up behavior
    localparam COMPUTE = 2'd1;
    localparam DONE    = 2'd2;

    reg [1:0] state, next_state; // State registers

    // ------------------------------------------------------------
    // VAL/RDY HANDLING
    // ------------------------------------------------------------
    wire   input_handshake;
    assign input_handshake = dotp_req_val && dotp_req_rdy;

    wire   output_handshake;
    assign output_handshake = dotp_resp_val && dotp_resp_rdy;

    assign dotp_req_rdy  = (state == COMPUTE); // Only take new data when actively computing
    assign dotp_resp_val = (state == DONE);    // Indicate that module is finished computing in DONE

    // ------------------------------------------------------------
    // STATE MACHINE LOGIC
    // ------------------------------------------------------------

    // State update logic
    always @(posedge clk) begin
        if (rst)
            state <= INIT;
        else
            state <= next_state;
    end

    // Next state logic
    always @(*) begin
        next_state = state;

        case (state)
            INIT:    next_state = COMPUTE;
            COMPUTE: if (input_handshake && (compute_counter == (n - 1))) next_state = DONE;
            DONE:    if (output_handshake) next_state = COMPUTE; // Only switch states when top-level module processes output

            default: next_state = INIT;
        endcase
    end

    // ------------------------------------------------------------
    // DOT PRODUCT COMPUTATION
    // ------------------------------------------------------------
    wire [DATA_LEN-1:0] current_dotp_result;
    assign current_dotp_result = data_in1 * data_in2;

    always @(posedge clk) begin
        if (rst) begin
            compute_counter <= 0;
            result          <= 0;
        end else begin
            case (state)

                INIT: begin
                    compute_counter <= 0;
                    result          <= 0;
                end

                COMPUTE: begin
                    if (input_handshake) begin
                        compute_counter <= compute_counter + 1;
                        result          <= result + current_dotp_result;
                    end
                end

                DONE: begin
                    // Hold values unless moving back to COMPUTE
                    if (output_handshake) begin
                        compute_counter <= 0;
                        result          <= 0;
                    end
                end

                default: begin
                    compute_counter <= 0;
                    result          <= 0;
                end

            endcase
        end
    end

endmodule