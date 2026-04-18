//========================================================================
// AXPY.v
//========================================================================
// Alpha X Plus Y (AXPY) algorithm computed on two vectors: computes z = y + αx, where α is a scalar quantity and 
// x, y, and z are vectors. To synchronize timing, a val/rdy interface is employed.

module AXPY #(
    parameter DATA_LEN = 32 // Size of data in vectors
)(
    input  wire        clk,
    input  wire        rst,
    input  wire [31:0] n,    // Vector length

    // AXPY computation inputs
    input  wire [DATA_LEN-1:0] alpha, // Scalar quantity by which we are multiplying
    input  wire [DATA_LEN-1:0] x_i,   // Current entry in x by which we are multiplying
    input  wire [DATA_LEN-1:0] y_i,   // Current entry in y being added

    // AXPY computation outputs
    output reg  [DATA_LEN-1:0] z_i,             // Current entry in z being output
    output reg  [31:0]         compute_counter, // Current index in vector for which computations are being run
    output wire                done,            // Raised when full AXPY operation is complete (not just a single entry)

    // val/rdy interface at input
    input  wire axpy_req_val,
    output wire axpy_req_rdy,

    // val/rdy interface at output
    input  wire axpy_resp_rdy,
    output wire axpy_resp_val
);

    // ------------------------------------------------------------
    // STATE MACHINE SETUP
    // ------------------------------------------------------------
    localparam INIT         = 2'd0; // This state can probably be removed - only included to ensure correct start-up behavior
    localparam COMPUTE      = 2'd1;
    localparam UPDATE_ENTRY = 2'd2;
    localparam DONE         = 2'd3;

    reg [1:0] state, next_state;

    // ------------------------------------------------------------
    // VAL/RDY HANDLING
    // ------------------------------------------------------------
    wire   input_handshake;
    assign input_handshake = axpy_req_val && axpy_req_rdy;

    wire   output_handshake;
    assign output_handshake = axpy_resp_val && axpy_resp_rdy;

    assign axpy_req_rdy  = (state == COMPUTE);
    assign axpy_resp_val = (state == UPDATE_ENTRY);
    
    // Indicate when full AXPY operation is complete
    assign done = (state == DONE);

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
            INIT: next_state = COMPUTE;
            COMPUTE: if (input_handshake) next_state = UPDATE_ENTRY;

            UPDATE_ENTRY: begin
                if (output_handshake) begin
                    // If last entry has been iterated through, move to DONE state; otherwise, continue iterating
                    if (compute_counter == n)
                        next_state = DONE;
                    else
                        next_state = COMPUTE;
                end
            end

            DONE:    next_state = COMPUTE;
            default: next_state = INIT;
        endcase
    end

    // ------------------------------------------------------------
    // AXPY COMPUTATION
    // ------------------------------------------------------------
    wire [DATA_LEN-1:0] current_axpy_result;
    assign current_axpy_result = y_i + (alpha * x_i);

    always @(posedge clk) begin
        if (rst) begin
            compute_counter <= 0;
            z_i             <= 0;
        end else begin
            case (state)

                INIT: begin
                    compute_counter <= 0;
                    z_i             <= 0;
                end

                COMPUTE: begin
                    if (input_handshake) begin
                        compute_counter <= compute_counter + 1;
                        z_i             <= current_axpy_result;
                    end
                end

                UPDATE_ENTRY: begin
                    // Hold values for top-level module to process
                end

                DONE: begin
                    // Reset everything for AXPY computation on next set of vectors
                    compute_counter <= 0;
                    z_i             <= 0;
                end

                default: begin
                    compute_counter <= 0;
                    z_i             <= 0;
                end

            endcase
        end
    end

endmodule